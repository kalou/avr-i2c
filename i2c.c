#include <avr/io.h>
#include <avr/interrupt.h>

#include "i2c.h"

static volatile i2caddr_t last_addr_seen = 0;

static uint8_t registers[] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

int valid_register(int r)
{
	return r < 8;
}

unsigned char default_handle_read()
{
	return registers[selected_reg & 7];
}
unsigned char (*handle_read)(void) = &default_handle_read;

void default_handle_write(char v)
{
	registers[selected_reg & 7] = v;
}
void (*handler_write)(char v) = &default_handle_write;

// Useful doc to follow that implementation:
// - Attiny85 datasheet
// - https://www.ti.com/lit/an/slva704/slva704.pdf?ts=1701529945068
// - https://ww1.microchip.com/downloads/en/AppNotes/Atmel-2560-Using-the-USI-Module-as-a-I2C-Slave_ApplicationNote_AVR312.pdf
//

// This interrupt handler is entered when a start condition
// has been detected on the SDA/SCL lines.
ISR(USI_START_vect) {
	switch (i2c) {
		case READING_WRITE:
			/* If we receive a start after we acked
			 * register write, this was a read. Enter
			 * the read prelude instead. */
			i2c = SEQ_RESTARTED;
			break;
		default:
			/* I guess in any other case we start 
			 * a transaction. */
			debug_note = 0;
			i2c = SEQ_STARTED;
			break;
	}

	// Start detector sometimes triggers *before* SCL is fully low.
	// If we reset the counter before that happens, the counter will
	// incorrectly get the SCL edge transition, overflowing before
	// the last bit is fully transmitted.
	// Ensure SCL goes fully low before we reset it:
	while (PINB & (1<<PINSCL));

	// Clear all flags, data register, ensure we don't hold SDA, and set counter to 0:
	DDRB &= ~(1<<DDSDA);
	USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF);
}

/* To send an ACK bit, the receiver shall pull down the SDA line
 * during the low phase of the ACK/NACK-related clock period
 * (period 9), so that the SDA line is stable low during the high phase
 * of the ACK/NACK-related clock period. Setup and hold times
 * must be taken into account.  When the SDA line remains high
 * during the ACK/NACK-related clock period, this is
 * interpreted as a NACK.
 */
void i2c_ack() {
	USIDR = 0;          // SDA can be forced low either by writing a zero to bit 7 of
			    // the USI Data Register, or by setting the corresponding
			    // bit in the PORTB register to zero. Note that the Data Direction
	DDRB |= (1<<DDSDA); // Register bit must be set to one for the output to be enabled.

	// Our state machine is driven by counter overflows.
	USISR = 14;
}

void i2c_nack() {
	// Just let the next clock cycle happen with SDA high by default (by pullup & config).
	USISR = 14;
}

ISR(USI_OVF_vect) {
	volatile i2caddr_t dr = USIDR;
	switch(i2c) {
		case IDLE:
			/* Ignore activity if we're not working */
			break;
		case SEQ_STARTED:
			last_addr_seen = (dr >> 1);
			if((dr >> 1) == myaddr) {
				// Normally should only enter acking_address
				// if write bit is set.
				// if !(dr & 1) {
				// }
				i2c = ACKING_ADDRESS;
				i2c_ack();
			} else {
				i2c = IDLE;
			}
			break;
		case ACKING_ADDRESS:
			// We expect a register next
			DDRB &= ~(1<<DDSDA);
			i2c = REGISTER_SELECT;
			break;
		case REGISTER_SELECT:
			if (valid_register(dr)) {
				selected_reg = dr;
				i2c = ACKING_REGISTER_SELECT;
				i2c_ack();
			} else {
				i2c = NACKING;
				i2c_nack();
			}
			break;
		case ACKING_REGISTER_SELECT:
			DDRB &= ~(1<<DDSDA);
			i2c = READING_WRITE;
			break;
		case READING_WRITE:
			handle_write(dr);
			i2c = ACKING_WRITE;
			i2c_ack();
			break;
		case ACKING_WRITE:
			/* Now that we ack'ed the write, we should either
			 * expect more data or a STOP condition. Do this by
			 * releasing SDA & SCL, and checking for either STOP
			 * is detected or another condition happens.
			 */
			PORTB |= (1<<PB4);
			DDRB &= ~(1<<DDSDA);
			USISR |= (1 << USIOIF);
			while (!PINB & (1<<PINSCL)); // Wait for clock to go high
			while (PINB & (1<<PINSCL) && !PINB & (1<<PINSDA)) // Wait for either
								    // clock to go low,
								    // or SDA high
			if (PINB & (1<<PINSCL) && PINB & (1<<PINSDA)) {
				// This is a stop condition, exit.
				i2c = IDLE;
				PORTB &= ~(1<<PB4);
			} else {
				// Clock went low: this was the 
				// first bit of the next packet.
				i2c = READING_WRITE;
			}
			// Do not clear flags twice.
			return;
		case SEQ_RESTARTED:
			if ((dr >> 1) == myaddr) {
				// Normally should only enter reading
				// if the write bit is unset.
				// if (dr & 1) {
				// }
				i2c = ACKING_READ_REQUEST;
				i2c_ack();
			} else {
				/* Just bailout */
				i2c = IDLE;
			}
			break;
		case ACKING_READ_REQUEST:
			USIDR = handle_read();
			USISR = 0;
			i2c = WRITING;
			break;
		case WRITING:
			/*
			 * We were writing, now read the ACK.
			 * 1/ Release the SDA line by removing DDRB
			 * 2/ Clear data register to read next clock cycle into it.
			 * 2/ Release the SCL line by clearing the USIOIF flag (happens at
			 *    end of this function), sets next overflow.
			 *
			 * At next overflow if data register has bit set, it was nack.
			 *
			 * Before the receiver can send an ACK, the transmitter 
			 * must release the SDA line. 
			 */
			i2c = READING_ACK;
			DDRB &= ~(1<<DDSDA);
			USIDR = 0;
			USISR = 14;
			break;
		case READING_ACK:
			if (dr) {
				/* We just got a nack: no more reads needed */
				i2c = IDLE;
			} else {
				/* We just got an ack, reset counter and continue */
				USIDR = handle_read();
				USISR = 0;
				DDRB |= (1<<DDSDA);
				i2c = WRITING;
			}
			break;
		case NACKING:
			/* When we're unhappy and nacked something */
			DDRB &= ~(1<<DDSDA);
			i2c = IDLE;
			break;

	}

	// Ack OVF bit, stop holding SCL
	USISR |= (1 << USIOIF);
}

void i2c_init() {
	DDRB |= 1<<DDSCL ; // Let detectors drive SCL.
	DDRB &= ~(1<<DDSDA); // Do not output DR[0] unless needed.
	PORTB |= 1<<PSCL | 1 << PSDA; // High by default up on both.
	i2c = IDLE;
	USICR = (1 << USISIE) | // Start condition interrupt enable
		(1<<USIOIE) | // Counter overflow interrupt enable
		(1<<USIWM1) | // TWI, with SCL held low
		(1<<USIWM0) | // during start and overflows until flag is cleared.
		(1<<USICS1); // External clock, shift at positive edges.
}
