/*
 * License: public domain (http://www.unlicense.org)
 */


/* 16 MHz */
#define F_CPU 16000000UL


//#define DEBUG

#define LOG_VOID_MEM_ACCESS


#include <avr/pgmspace.h>
#include <util/delay.h>

#include <stdio.h>
#include <string.h>


/* 4K RAM */
uint8_t ram[0x1000] __attribute__((section(".ram")));


/* pins used on Arduino Mega 2560 */

/* address high */
#define PORT_AH PORTC
#define PIN_AH PINC
#define DDR_AH DDRC

/* address low */
#define PORT_AL PORTF
#define PIN_AL PINF
#define DDR_AL DDRF

/* data */
#define PORT_D PORTA
#define PIN_D PINA
#define DDR_D DDRA

/* PHI0 */
#define PHI0 PB7
#define PORT_PHI0 PORTB
#define PIN_PHI0 PINB
#define DDR_PHI0 DDRB

/* RDY */
#define RDY PB6
#define PORT_RDY PORTB
#define DDR_RDY DDRB

/* SO */
#define SO PB5
#define PORT_SO PORTB
#define DDR_SO DDRB

/* RW */
#define RW PD7
#define PORT_RW PORTD
#define PIN_RW PIND
#define DDR_RW DDRD

/* RES */
#define RES PG2
#define PORT_RES PORTG
#define DDR_RES DDRG

/* IRQ */
#define IRQ PG1
#define PORT_IRQ PORTG
#define DDR_IRQ DDRG

/* NMI */
#define NMI PG0
#define PORT_NMI PORTG
#define DDR_NMI DDRG


/* convenience macros */

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define PORT_MODE_OUTPUT(x) { DDR_##x = 0xFF; }
#define PORT_MODE_INPUT(x)  { DDR_##x = 0x00; }

#define PIN_MODE_OUTPUT(x) { sbi(DDR_##x, x); }
#define PIN_MODE_INPUT(x)  { cbi(DDR_##x, x); }

#define SET_PIN_HIGH(x) { sbi(PORT_##x, x); }
#define SET_PIN_LOW(x)  { cbi(PORT_##x, x); }
#define TOGGLE_PIN(x)   { PIN_##x = _BV(x); }
#define PIN_IS_HIGH(x)  ( (bit_is_set(PIN_##x, x)) != 0)

#define LOOP_UNTIL_PIN_IS_HIGH(x) { loop_until_bit_is_set(PIN_##x, x); }
#define LOOP_UNTIL_PIN_IS_LOW(x)  { loop_until_bit_is_clear(PIN_##x, x); }

/* the last (8th) AVR cycle of a 6502 clock phase is used to toggle phi0 */
#define SKIP_CLOCK_PHASE { \
	asm volatile("nop"); \
	asm volatile("nop"); \
	asm volatile("nop"); \
	asm volatile("nop"); \
	asm volatile("nop"); \
	asm volatile("nop"); \
	asm volatile("nop"); \
}


/* PIA  */

static uint8_t crb = 0;


/* terminal */

static FILE uart0 = { 0 };


void uart0_putchar(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}


int uart0_putchar_libc(char c, FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}


void setup_uart0()
{
	uint32_t baud = 600;
	UBRR0 = (((F_CPU + (baud*8)) / (baud*16)) - 1);

#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);

	fdev_setup_stream(&uart0, uart0_putchar_libc, NULL, _FDEV_SETUP_WRITE);

#ifdef DEBUG
	fprintf(&uart0, "\nUART0 ready\n");
#endif
}


/* debug log */

static FILE uart1 = { 0 };


int uart1_putchar_libc(char c, FILE *stream)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);
	UDR1 = c;
}


void setup_uart1()
{
	uint32_t baud = 4800;
	UBRR1 = (((F_CPU + (baud*8)) / (baud*16)) - 1);

#if USE_2X
	UCSR1A |= _BV(U2X1);
#else
	UCSR1A &= ~(_BV(U2X1));
#endif
  
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
	UCSR1B = _BV(TXEN0);

	fdev_setup_stream(&uart1, uart1_putchar_libc, NULL, _FDEV_SETUP_WRITE);

#ifdef DEBUG
	fprintf(&uart1, "\nUART1 ready\n");
#endif
}


void setup_pins()
{
	PORT_MODE_INPUT(AH);
	PORT_MODE_INPUT(AL);

	PORT_MODE_INPUT(D);

	PIN_MODE_INPUT(RW);

	/* start in phase 2 */
	SET_PIN_HIGH(PHI0);
	PIN_MODE_OUTPUT(PHI0);

	/* ready */
	SET_PIN_HIGH(RDY);
	PIN_MODE_OUTPUT(RDY);

	/* no reset */
	SET_PIN_HIGH(RES);
	PIN_MODE_OUTPUT(RES);

	/* no irq */
	SET_PIN_HIGH(IRQ);
	PIN_MODE_OUTPUT(IRQ);

	/* no nmi */
	SET_PIN_HIGH(NMI);
	PIN_MODE_OUTPUT(NMI);

	/* no overflow */
	SET_PIN_HIGH(SO);
	PIN_MODE_OUTPUT(SO);
}


void print_a_rw()
{
	fprintf(&uart1, "A: %02X%02X, RW: %u\n", PIN_AH, PIN_AL, PIN_IS_HIGH(RW));
}


void print_d()
{
	fprintf(&uart1, "D: %02X\n", PIN_D);
}


void reset()
{
	/* print state before reset */

	TOGGLE_PIN(PHI0); /* L */
	SKIP_CLOCK_PHASE;
	TOGGLE_PIN(PHI0); /* H */
	SKIP_CLOCK_PHASE;

	fprintf(&uart1, "\ninitial state:\n\n");

	print_a_rw();

	fprintf(&uart1, "\nreset cycles:\n\n");

	SET_PIN_LOW(RES);

	/* RES must be held low for at least 2 clock cycles */

	TOGGLE_PIN(PHI0); /* L */
	SKIP_CLOCK_PHASE;
	TOGGLE_PIN(PHI0); /* H */
	SKIP_CLOCK_PHASE;;

	print_a_rw();

	TOGGLE_PIN(PHI0); /* L */
	SKIP_CLOCK_PHASE;
	TOGGLE_PIN(PHI0); /* H */
	SKIP_CLOCK_PHASE;;

	print_a_rw();

	fprintf(&uart1, "\nreset cycles finished, starting up...\n\n");

	SET_PIN_HIGH(RES);
}


void setup()
{
	/* zero-out RAM (not done automatically for data in separate sections) */
	memset(ram, 0, sizeof(ram));

	setup_pins();

	setup_uart0();
	setup_uart1();

	fprintf(&uart0, "\nresetting the 6502 in 5 seconds...\n");
	fprintf(&uart1, "\nresetting the 6502 in 5 seconds...\n");

	_delay_ms(5000);


	/* send XON to indicate that we are ready to receive */
	uart0_putchar(0x11);

	/* clear terminal screen */
	fprintf(&uart0, "\033[2J");
	fprintf(&uart0, "\033[1;1H");

	reset();
}


void handle_io_or_void_write()
{

#ifdef DEBUG
	fprintf(&uart1, "handle_io_or_void_write()\n");
#endif

	uint8_t ah = PIN_AH;
	uint8_t al = PIN_AL;

	uint8_t d = PIN_D;

#ifdef DEBUG
	fprintf(&uart1, "6502 writes: %02X\n", d);
#endif

	if (ah == 0xD0) {

		switch (al & 0x13) {

			case 0x10:

				/* DDRA or PDRA (KBD) */

				/* writing DDRA or PDRA is currently not supported */

#ifndef DEBUG
				print_a_rw();
#endif
				fprintf(&uart1, "*\n");

				break;

			case 0x11:

				/* CRA (KBD CR) */

				if (d == 0xA7) {

					/* that's what the Woz Monitor writes to select PDRA */

				} else {

#ifndef DEBUG
					print_a_rw();
#endif
					fprintf(&uart1, "*\n");
				}

				break;

			case 0x12:

				/* DDRB or PDRB (DSP) */

				if (bit_is_set(crb, 2)) {

					/* PDRB */

					/* assuming that DDRB is 0x7F */

					d &= 0x7F;

					if ((d >= 0x60) && (d <= 0x7F))
						d &= 0x5F;

					if ((d >= 0x20) && (d <= 0x5F))
						uart0_putchar(d);
					 else
						uart0_putchar('\n');

				} else {

					/* DDRB */

					if (d == 0x7F) {

						/* that's what the Woz Monitor writes */

					} else {
#ifndef DEBUG
						print_a_rw();
#endif
						fprintf(&uart1, "*\n");
					}
				}

				break;

			case 0x13:

				/* CRB (DSP CR) */

				crb = d;

				break;

			default:

#ifdef LOG_VOID_MEM_ACCESS

#ifndef DEBUG
				print_a_rw();
#endif
				fprintf(&uart1, "*\n");
#endif

				break;
		}

	} else {

#ifdef LOG_VOID_MEM_ACCESS

#ifndef DEBUG
		print_a_rw();
#endif
		fprintf(&uart1, "*\n");
#endif

	}
}


void handle_io_or_void_read()
{

#ifdef DEBUG
	fprintf(&uart1, "handle_io_or_void_read()\n");
#endif

	uint8_t ah = PIN_AH;
	uint8_t al = PIN_AL;

	uint8_t d;

	if (ah == 0xD0) {

		switch (al & 0x13) {

			case 0x10:

				/* DDRA or PDRA (KBD) */

				/* assuming that PDRA is selected and DDRA is 0x00 */

				d = UDR0;

				/* send XON to indicate that we are ready to receive */
				uart0_putchar(0x11);

				if ((d >= 'a') && (d <= 'z'))
					d -= 32;

				d |= 0x80;

				break;

			case 0x11:

				/* CRA (KBD CR) */

				/* IRQA1 */
				d = bit_is_set(UCSR0A, RXC0) ? 0x80 : 0x00;

				break;

			case 0x12:

				/* DDRB or PDRB (DSP) */

				/*
				 * assuming that PDRB is selected and DDRB is 0x7F
				 * and only bit 7 is evaluated by the reader.
				 */

				d = bit_is_set(UCSR0A, UDRE0) ? 0x00 : 0x80;

				break;

			case 0x13:

				/* CRB (DSP CR) */

				/* reading CRB is currently not supported */

				d = 0;

#ifndef DEBUG
				print_a_rw();
#endif
				fprintf(&uart1, "*\n");

				break;

			default:

				d = 0;

#ifdef LOG_VOID_MEM_ACCESS

#ifndef DEBUG
				print_a_rw();
#endif
				fprintf(&uart1, "*\n");
#endif

				break;
		}

	} else {

		d = 0;

#ifdef LOG_VOID_MEM_ACCESS

#ifndef DEBUG
		print_a_rw();
#endif
		fprintf(&uart1, "*\n");
#endif
	}

	PORT_D = d;      
	PORT_MODE_OUTPUT(D);

#ifdef DEBUG
	fprintf(&uart1, "6502 reads: %02X\n", d);
#endif
}


int main()
{
	setup();

	register uint8_t bit_phi0 asm ("r2") = _BV(PHI0);              /* callee-saved register */
	register uint8_t port_mode_output asm ("r29") = 0xFF;          /* callee-saved register */

	register uint8_t ram_size_high asm ("r28") = sizeof(ram) >> 8; /* callee-saved register */

	/* 6502 RAM starts at AVR SRAM address 0x1000 */
	register uint8_t ram_base_high asm ("r17") = 0x10;             /* callee-saved register */

	register uint8_t d asm ("r16");                                /* callee-saved register */

	/* loading from program memory uses Z register */
	register uint8_t ah asm ("r31");
	register uint8_t al asm ("r30");


	asm volatile (

		"	; toggle phi0 (L)                                                   \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

		"	nop                                                                 \n\t"
		"	nop                                                                 \n\t"


		"loop:                                                                  \n\t"

		"	; set data port to input                      (1)                   \n\t"

		"	out %[ddr_d], r1                                                    \n\t"

		"	; toggle phi0 (H)                             (1)                   \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

#ifdef DEBUG
		"	call print_a_rw                                                     \n\t"
#endif

		"	; read request?                               (true: 2, false: 3)   \n\t"

		"	sbis %[pin_rw], %[bit_rw]                                           \n\t"
		"	rjmp write_request                                                  \n\t"


		"read_request:                                                          \n\t"

		"	; read AH                                      (1)                  \n\t"

		"	in %[ah], %[pin_ah]                                                 \n\t"

		"	; RAM address? (AH < 0x10)                     (true: 2, false: 3)  \n\t"

		"	cp %[ah], %[ram_size_high]                                          \n\t"
		"	brcc read_address_not_in_ram                                        \n\t"


		"read_address_in_ram:                                                   \n\t"

		"	; read AL                                      (1)                  \n\t"

		"	in %[al], %[pin_al]                                                 \n\t"

		"	; convert 6502 RAM address to AVR SRAM address (1)                  \n\t"

		"	add %[ah], %[ram_base_high]                                         \n\t"

		"	; load data byte from SRAM                     (2)                  \n\t"

		"	ld %[d], Z                                                          \n\t"

		"	; write data byte to port                      (1)                  \n\t"

		"	out %[port_d], %[d]                                                 \n\t"

		"	; set data port to output                      (1)                  \n\t"

		"	out %[ddr_d], %[port_mode_output]                                   \n\t"

#ifdef DEBUG
		"	call print_d                                                        \n\t"
#endif

		"	; toggle phi0 (L)                              (1)                  \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

		"	; loop                                         (2)                  \n\t"

		"	rjmp loop                                                           \n\t"


		"read_address_not_in_ram:                                               \n\t"

		"	; ROM address? (AH >= 0xE0)                    (true: 2, false: 3)  \n\t"

		"	cpi %[ah], 0xE0                                                     \n\t"
		"	brcs read_address_not_in_ram_or_rom                                 \n\t"


		"read_address_in_rom:                                                   \n\t"

		"	; read AL                                      (1)                  \n\t"

		"	in %[al], %[pin_al]                                                 \n\t"

		"	; load data byte from Program Memory           (3)                  \n\t"

		"	lpm %[d], Z                                                         \n\t"

		"	; write data byte to port                      (1)                  \n\t"

		"	out %[port_d], %[d]                                                 \n\t"

		"	; set data port to output                      (1)                  \n\t"

		"	out %[ddr_d], %[port_mode_output]                                   \n\t"

#ifdef DEBUG
		"	call print_d                                                        \n\t"
#endif

		"	; toggle phi0 (L)                              (1)                  \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

		"	; loop                                         (2)                  \n\t"

		"	rjmp loop                                                           \n\t"


		"read_address_not_in_ram_or_rom:                                        \n\t"

		"	call handle_io_or_void_read                                         \n\t"

		"	; toggle phi0 (L)                              (1)                  \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

		"	; loop                                         (2)                  \n\t"

		"	rjmp loop                                                           \n\t"


		"write_request:                                                         \n\t"

		"	; read AH                                      (1)                  \n\t"

		"	in %[ah], %[pin_ah]                                                 \n\t"

		"	; RAM address? (AH < 0x10)                     (true: 2, false: 3)  \n\t"

		"	cp %[ah], %[ram_size_high]                                          \n\t"
		"	brcc write_address_not_in_ram                                       \n\t"


		"write_address_in_ram:                                                  \n\t"

		"	; read AL                                      (1)                  \n\t"

		"	in %[al], %[pin_al]                                                 \n\t"

		"	; convert 6502 RAM address to AVR SRAM address (1)                  \n\t"

		"	add %[ah], %[ram_base_high]                                         \n\t"

		"	; read data byte from port                     (1)                  \n\t"

		"	in %[d], %[pin_d]                                                   \n\t"

		"	; write data byte into SRAM                    (2)                  \n\t"

		"	st Z, %[d]                                                          \n\t"

#ifdef DEBUG
		"	call print_d                                                        \n\t"
#endif

		"	; toggle phi0 (L)                              (1)                  \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

		"	; loop                                         (2)                  \n\t"

		"	rjmp loop                                                           \n\t"


		"write_address_not_in_ram:                                              \n\t"

		"	call handle_io_or_void_write                                        \n\t"

		"	; toggle phi0 (L)                              (1)                  \n\t"

		"	out %[pin_phi0], %[bit_phi0]                                        \n\t"

		"	; loop                                         (2)                  \n\t"

		"	rjmp loop                                                           \n\t"

	:
	:
	  [pin_phi0] "I" (_SFR_IO_ADDR(PIN_PHI0)), [bit_phi0] "r" (bit_phi0),
	  [ddr_d] "I" (_SFR_IO_ADDR(DDR_D)),
	  [pin_rw] "I" (_SFR_IO_ADDR(PIN_RW)), [bit_rw] "I" (RW),
	  [ah] "r" (ah), [pin_ah] "I" (_SFR_IO_ADDR(PIN_AH)),
	  [ram_size_high] "r" (ram_size_high),
	  [al] "r" (al), [pin_al] "I" (_SFR_IO_ADDR(PIN_AL)),
	  [ram_base_high] "r" (ram_base_high),
	  [d] "r" (d),
	  [port_d] "I" (_SFR_IO_ADDR(PORT_D)),
	  [port_mode_output] "r" (port_mode_output),
	  [pin_d] "I" (_SFR_IO_ADDR(PIN_D))
	: "memory"
	);

	return 0;
}
