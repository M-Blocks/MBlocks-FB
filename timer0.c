/*
 * timer0.c
 *
 * Created: 11/7/2014 8:01:51 AM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include "avr/io.h"
#include "util/atomic.h"

#include "timer0.h"

#ifndef F_CPU
#	define F_CPU 8000000UL
#endif

static bool m_configured = false;

bool timer0_init(uint8_t freq_khz) {
	uint32_t halfPeriod_counts;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Stop and reset Timer/counter */
		TCCR0A = 0x00;
		TCCR0B = 0x00;
	
		/* Make OC0A (PC0) an output. */
		DDRC |= (1<<DDRC0);
	
		/* Reset the counter to 0 */
		TCNT0 = 0x00;
	
		/* Toggle OC0A (PC0) on match.  Select fast PWM mode with TOP equal to 
		 * OCRA, update OCRx at BOTTOM, and set TOV flag at TOP. */
		TCCR0A = (1 << COM0A0) | (3 << WGM00);
	
		/* Keep clock source off, finish fast PWM mode configuration. */
		TCCR0B = (1<<WGM02);
	
		/* Disable all of the Timer0 interrupts.  Note that the TIMSK register is
		 * also used to control Timer1 interrupts.  */
		TIMSK &= ~((1<<OCIE0B) | (1<<TOIE0) | (1<<OCIE0A));
	
		/* Reset all Timer0 interrupt flags. Note that the TIFR register is also
		 * used to signal Timer1 interrupts.  */
		TIFR &= ~((1<<OCF0B) | (1<<TOV0) | (1<<OCF0A));
	
		/* With the timer configured to use OCR0A as its TOP value and OC0A 
		 * configured to toggle on each match, set set OCR0A to half of the desired
		 * timer period.  When the timer first starts counting up towards OCR0A,
		 * the OC0A pin will be low.  Then, the pin will toggle high, the counter
		 * will be reset to 0, and as the counter increments toward OCR0A a second 
		 * time, the OC0A pin will be high.  This pattern will repeat, generating
		 * a 50% duty cycle square wave at the desired frequency. */
		halfPeriod_counts = (F_CPU / ((uint32_t)freq_khz * 1000UL)) / 2UL;
	
		if ((halfPeriod_counts == 0) || (halfPeriod_counts > 255)) {
			return false;
		}
	
		OCR0A = (uint8_t)halfPeriod_counts;
	
		/* Indicate that the timer has been configured. */
		m_configured = true;
	}
	
	/* Note that we leave the timer off for now. */
	return true;
}

void timer0_deinit() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Stop and reset Timer/counter */
		TCCR0A = 0x00;
		TCCR0B = 0x00;
		
		/* Disable all of the Timer0 interrupts.  Note that the TIMSK register is
		 * also used to control Timer1 interrupts.  */
		TIMSK &= ~((1<<OCIE0B) | (1<<TOIE0) | (1<<OCIE0A));
	
		/* Reset all Timer0 interrupt flags. Note that the TIFR register is also
		 * used to signal Timer1 interrupts.  */
		TIFR &= ~((1<<OCF0B) | (1<<TOV0) | (1<<OCF0A));		
		
		/* Make OC0A (PC0) an output driving low */
		DDRC |= (1<<DDRC0);
		PORTC &= ~(1<<PORTC0);
		
		m_configured = false;
	}
}

bool timer0_enableCarrierOutput() {
	if (!m_configured) {
		return false;
	}
	
	/* Set OC0A (PC0) to toggle on match. */
	TCCR0A |= (1 << COM0A0);
	
	/* Enable counting.  */
	TCCR0B |= (1<<CS00);
	
	return true;
}

bool timer0_disableCarrierOutput() {
	if (!m_configured) {
		return false;
	}
	
	/* Disable counting */
	TCCR0B &= ~(1<<CS00);
	
	/* Reset the counter to 0 */
	TCNT0 = 0x00;
	
	/* Return the output compare pin to normal functionality so that we can
	 * manually drive it high. */
	TCCR0A &= ~((1<<COM0A1) | (1<<COM0A0));	
	
	/* Make OC0A (PC0) an output driving low */
	DDRC |= (1<<DDRC0);
	PORTC &= ~(1<<PORTC0);
	
	return true;
}

bool timer0_getCarrierStatus() {
	if (!m_configured) {
		return false;
	}
	
	if ((TCCR0B & (7<<CS00)) == 0x01) {
		return true;
	}
	
	return false;
}