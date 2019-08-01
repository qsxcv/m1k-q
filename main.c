#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "srom_3360_0x05.h"
#include "usb_mouse.h"

#define delay_us(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000000))
#define delay_ms(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000))


#define PORT_SPI PORTB
#define DDR_SPI	DDRB

#define DD_SS	6 // aka NCS
#define DD_SCK	1
#define DD_MOSI	2
#define DD_MISO	3

#define SS_LOW	(PORT_SPI &= ~(1<<DD_SS))
#define SS_HIGH	(PORT_SPI |= (1<<DD_SS))

// use this instead of bitshifts or LSB/MSB macros.
union motion_data {
	int16_t all;
	struct { uint8_t lo, hi; };
};


static void pins_init(void)
{
	// L pullup input on D0 (NO) and D2 (NC)
	// R pullup input on D1 (NO) and D3 (NC)
	PORTD |= 0b00001111; // L, R pullup inputs on D0, D1. (active low)
	DDRC |= (1<<2); PORTC |= (1<<2); // C2, 3360 NRESET high output

	EICRA = 0b01010101; // generate interrupt request on any edge of D0/D1/D2/D3
	EIMSK = 0; // but don't enable any actual interrupts
	EIFR = 0b00001111; // clear EIFR
}


// spi functions
static void spi_init(void)
{
	DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS); // outputs
	DDRB |= (1<<0); PORTB |= (1<<0); // set the hardware SS pin to low to enable SPI
	// MISO pullup input is already done in hardware
	// enable spi, master mode, mode 3, clock rate = fck/4 = 2MHz
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
}

static inline void spi_send(const uint8_t b)
{
	SPDR = b;
	while (!(SPSR & (1<<SPIF)));
}

static inline uint8_t spi_recv(void)
{
	spi_send(0x00);
	return SPDR;
}

static inline void spi_write(const uint8_t addr, const uint8_t data)
{
	spi_send(addr | 0x80);
	spi_send(data);
	delay_us(180); // maximum of t_SWW, t_SWR
}

static inline uint8_t spi_read(const uint8_t addr)
{
	spi_send(addr);
	delay_us(160); // t_SRAD
	uint8_t data = spi_recv();
	delay_us(20);
	return data;
}


static void pmw3360_init(const uint8_t dpi)
{
	// drop and raise ncs to reset spi port (power-up step 2)
	SS_HIGH;
	delay_us(40);
	SS_LOW;
	delay_us(40);
	SS_HIGH;
	delay_us(40);

	// power up reset  (power-up step 3, 4)
	SS_LOW;
	spi_write(0x3a, 0x5a);
	SS_HIGH;
	delay_ms(50);

	// read from 0x02 to 0x06 (power-up step 5)
	SS_LOW;
	spi_read(0x02);
	spi_read(0x03);
	spi_read(0x04);
	spi_read(0x05);
	spi_read(0x06);

	// srom download (power-up step 6)
	spi_write(0x10, 0x00); // (srom step 2)
	spi_write(0x13, 0x1d); // (srom step 3)
	SS_HIGH;
	delay_ms(10); // (srom step 4)
	SS_LOW;
	spi_write(0x13, 0x18); // (srom step 5)

	spi_send(0x62 | 0x80); // (srom step 6)
	const uint8_t *psrom = srom;
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		delay_us(16);
		spi_send(pgm_read_byte(psrom++));
	}
	delay_us(18);
	SS_HIGH;
	delay_us(200);

	// configuration/settings
	SS_LOW;
	spi_write(0x10, 0x00); // no rest mode
	spi_write(0x42, 0x00); // no angle snapping
	spi_write(0x0f, dpi);
	//spi_write(0x63, 0x03); // 3mm lod
	//spi_write(0x0d, 0x00); // 0 degree
	SS_HIGH;
}

int main(void)
{
	// set clock prescaler for 8MHz
	CLKPR = 0x80;
	CLKPR = 0x01;

	pins_init();

	// previous debounced button state (1 for pressed, 0 for released)
	uint8_t btn_prev = (~PIND) & 0x03; // read L+R

	if (btn_prev == 3) { // jump to bootloader if both L+R held for >=2 seconds
		const uint8_t max = 100;
		uint8_t i;
		for (i = 0; i < max; i++) {
			delay_ms(20);
			if (((~PIND) & 0x03) != 3) break;
		}
		if (i == max) __asm__ volatile ("jmp 0x7000");
	}

	union motion_data x_sum, y_sum; // total motion after last usb transmission
	spi_init();
	// set dpi based on initial state of buttons
	//	none	left	right	left+right
	//	800	3500	3600	12000
	const uint8_t dpi = (uint8_t []){7, 34, 35, 119}[btn_prev];
	pmw3360_init(dpi); // yay compound literals
	// begin burst mode
	SS_LOW;
	spi_write(0x50, 0x00);
	SS_HIGH;

	usb_init();
	while (!usb_configured());
	delay_ms(456); // arbitrary

	// set up timer0 to set OCF0A in TIFR0 every 125us
	TCCR0A = 0x02; // CTC
	TCCR0B = 0x02; // prescaler 1/8 = 1us period
	OCR0A = 124; // = 125 - 1

	cli();
	for (uint8_t i = 0; ; i = (i + 1) % 8) {
	// synchronization to usb frames and 125us intervals
		// polling interrupt flags gives 5 clock cycles or so of
		// jitter. possible to eliminate by going into sleep
		// mode and waking up using interrupts, but whatever.
		if (i == 0) {
			// sync to usb frames (1ms)
			UDINT &= ~(1<<SOFI);
			while(!(UDINT & (1<<SOFI)));
			// reset prescaler phase, not really necessary
			GTCCR |= (1<<PSRSYNC);
			TCNT0 = 0;
		} else {
			// sync to 125us intervals using timer0
			while (!(TIFR0 & (1<<OCF0A)));
		}
		TIFR0 |= (1<<OCF0A); // 0CF0A is cleared by writing 1

	// sensor stuff
		union motion_data x, y;
		SS_LOW;
		spi_send(0x50);
		delay_us(35);
		spi_send(0x00); // motion, not used
		spi_send(0x00); // observation, not used
		x.lo = spi_recv();
		x.hi = spi_recv();
		y.lo = spi_recv();
		y.hi = spi_recv();
		SS_HIGH;

	// button stuff
		//high = not pressed, low = pressed
		//PIND 0 EIFR 0: low, no edges -> is low
		//PIND 0 EIFR 1: low, edge -> is low
		//PIND 1 EIFR 0: high, no edges -> always high during last 125us
		//PIND 1 EIFR 1: high, edge -> low at some point in the last 125us
		const uint8_t btn_raw = ((~PIND) | EIFR) & 0x0f; // 1 means low
		EIFR = 0b00001111; // clear EIFR
		const uint8_t btn_bot = btn_raw & 0x03;
		const uint8_t btn_top = btn_raw >> 2;
		// 0 means not in contact, 1 means in contact
		// bottom 0 top 0: floating, keep previous state
		// bottom 0 top 1: released
		// bottom 1 top 0: pressed
		// bottom 1 top 1: not possible
		const uint8_t btn_dbncd = btn_bot | (~btn_top & btn_prev);

	// usb
		// first make sure it's configured
		sei();
		if (!usb_configured()) {
			// if not, shut off sensor and restart everything
			PORTC &= ~(1<<2);
			while (!usb_configured());
			PORTC |= (1<<2);
			pmw3360_init(dpi);
			// begin burst mode
			SS_LOW;
			spi_write(0x50, 0x00);
			SS_HIGH;
		}
		cli();

		// this stuff is very intricate and confusing
		// i'm fairly certain all of it is correct.
		// there's nothing to do if nothing's changed in this 125us cycle
		if ((btn_dbncd != btn_prev) || x.all || y.all) {
			UENUM = MOUSE_ENDPOINT;
			if (UESTA0X & (1<<NBUSYBK0)) { // untransmitted data still in bank
				UEINTX |= (1<<RXOUTI); // kill bank; RXOUTI == KILLBK
				while (UEINTX & (1<<RXOUTI));
			} else {
				// transmission's finished, or the data that should be in the
				// bank is exactly the same as what was previously transmitted
				// so that there was nothing worth transmitting before.
				x_sum.all = 0;
				y_sum.all = 0;
			}
			x_sum.all -= x.all; // invert here for m1k
			y_sum.all -= y.all;
			// only load bank with data if there's something worth transmitting
			if ((btn_dbncd != btn_prev) || x_sum.all || y_sum.all) {
				UEDATX = btn_dbncd;
				UEDATX = x_sum.lo;
				UEDATX = x_sum.hi;
				UEDATX = y_sum.lo;
				UEDATX = y_sum.hi;
				UEDATX = 0; // wheel scrolls
				UEINTX = 0x3a;
				btn_prev = btn_dbncd;
			}
		}
	}
}
