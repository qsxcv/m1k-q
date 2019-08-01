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
	const uint8_t *psrom = srom;

	SS_HIGH;
	delay_ms(3);

	// shutdown first
	SS_LOW;
	spi_write(0x3b, 0xb6);
	SS_HIGH;
	delay_ms(300);

	// drop and raise ncs to reset spi port
	SS_LOW;
	delay_us(40);
	SS_HIGH;
	delay_us(40);

	// power up reset
	SS_LOW;
	spi_write(0x3a, 0x5a);
	SS_HIGH;
	delay_ms(50);

	// read from 0x02 to 0x06
	SS_LOW;
	spi_read(0x02);
	spi_read(0x03);
	spi_read(0x04);
	spi_read(0x05);
	spi_read(0x06);

	spi_write(0x10, 0x00); // well i know this disables rest mode. not sure purpose
	spi_write(0x22, 0x00); // ???

	// srom download
	spi_write(0x13, 0x1d);
	SS_HIGH;
	delay_ms(10);
	SS_LOW;
	spi_write(0x13, 0x18);

	spi_send(0x62 | 0x80);
	for (uint16_t i = 0; i < SROM_LENGTH; i++) {
		delay_us(16);
		spi_send(pgm_read_byte(psrom++));
	}
	delay_us(18);
	SS_HIGH;
	delay_us(200);

	// configuration/settings
	SS_LOW;
	spi_write(0x10, 0x00); // 0x20 (g502 default) enables rest mode after ~10s of inactivity
	spi_write(0x14, 0xff); // how long to wait before going to rest mode. 0xff is max (~10 seconds)
	spi_write(0x17, 0xff); // ???
	spi_write(0x18, 0x00); // ???
	spi_write(0x19, 0x00); // ???
	spi_write(0x1b, 0x00); // ???
	spi_write(0x1c, 0x00); // ???

	// surface tuning settings (default: 0x0a, 0x10)
	// probably not necessary to read them first.
//	spi_read(0x2c);
//	spi_read(0x2b);
//	delay_us(18);
//	spi_write(0x2c, 0x0a);
//	spi_write(0x2b, 0x10);

	// configuration/settings
	spi_write(0x0f, dpi);
	spi_write(0x42, 0x00); // no angle snapping
	//spi_write(0x63, 0x03); // 3mm lod
	//spi_write(0x0d, 0x00);
	SS_HIGH;
}

int main(void)
{
	union motion_data x, y;

	// set clock prescaler for 8MHz
	CLKPR = 0x80;
	CLKPR = 0x01;

	pins_init();

	// previous physical state to compare against for debouncing
	uint8_t btn_prev = (~PIND) & 0x03; // read L+R
	// previous state of btn_usb
	uint8_t btn_usb_prev = btn_prev;
	// previously transmitted button state
	uint8_t btn_usb_prev_trans = btn_prev;

	if (btn_prev == 3) { // jump to bootloader if both L+R held for >=2 seconds
		uint8_t i;
		for (i = 0; i < 100; i++) {
			delay_ms(20);
			if (((~PIND) & 0x03) != 3) break;
		}
		if (i == 100) __asm__ volatile ("jmp 0x7000");
	}

	const int8_t whl_step = 15;
	int16_t whl_loc = 0;
	int8_t dwhl = 0;

	int8_t r_click_mode = 0;
	uint8_t r_click_timer = 0;

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
	while (1) {
		for (uint8_t i = 0; i < 8; i++) {
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
			union motion_data _x, _y; // inverted
			SS_LOW;
			spi_send(0x50);
			delay_us(35);
			spi_send(0x00); // motion, not used
			spi_send(0x00); // observation, not used
			_x.lo = spi_recv();
			_x.hi = spi_recv();
			_y.lo = spi_recv();
			_y.hi = spi_recv();
			SS_HIGH;

		// button stuff
			//high = not pressed, low = pressed
			//PIND 0 EIFR 0: low, no edges -> is low
			//PIND 0 EIFR 1: low, edge -> is low
			//PIND 1 EIFR 0: high, no edges -> always high during last 125us
			//PIND 1 EIFR 1: high, edge -> low at some point in the last 125us
			const uint8_t btn_raw = ((~PIND) | EIFR) & 0b00001111; // 1 means low
			EIFR = 0b00001111; // clear EIFR

			const uint8_t btn_bot = btn_raw & 0b00000011;
			const uint8_t btn_top = btn_raw >> 2;
			const uint8_t btn_dbncd = btn_bot | (~btn_top & btn_prev);

			// wheel emulation
			int8_t _dwhl = 0;
			if (btn_dbncd & (1<<1)) { // R pressed
				if (!(btn_prev & (1<<1))) // R just pressed
					r_click_mode = 1;
				whl_loc += _y.all; // move down to scroll down
				int8_t sgn = 2*(whl_loc > 0) - 1;
				while (sgn*whl_loc > whl_step/2) {
					r_click_mode = 0;
					whl_loc -= sgn*whl_step;
					_dwhl += sgn;
				}
			} else if (r_click_mode && btn_prev & (1<<1)) { // R just released
				r_click_timer = 80;
				whl_loc = 0;
			}

			uint8_t btn_usb = btn_dbncd & (1<<0);
			if (r_click_timer > 0) {
				r_click_timer--;
				btn_usb |= (1<<1);
			}

		// usb
			// first make sure it's configured
			sei();
			if (!usb_configured()) {
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
			if ((btn_usb != btn_usb_prev) || _x.all || _y.all || _dwhl) {
				UENUM = MOUSE_ENDPOINT;
				if (UESTA0X & (1<<NBUSYBK0)) { // untransmitted data still in bank
					UEINTX |= (1<<RXOUTI); // kill bank; RXOUTI == KILLBK
					while (UEINTX & (1<<RXOUTI));
				} else {
					// transmission's finished, or the data that should be in the
					// bank is exactly the same as what was previously transmitted
					// so that there was nothing worth transmitting before.
					x.all = 0;
					y.all = 0;
					dwhl = 0;
				}
				x.all -= _x.all; // invert here
				y.all -= _y.all;
				dwhl += _dwhl;
				// only load bank with data if there's something worth transmitting
				if ((btn_usb != btn_usb_prev_trans) || x.all || y.all || dwhl) {
					UEDATX = btn_usb;
					UEDATX = x.lo;
					UEDATX = x.hi;
					UEDATX = y.lo;
					UEDATX = y.hi;
					UEDATX = dwhl; // wheel scrolls
					UEINTX = 0x3a;
					btn_usb_prev_trans = btn_usb;
				}
			}
			btn_prev = btn_dbncd;
			btn_usb_prev = btn_usb;
		}
	}
}
