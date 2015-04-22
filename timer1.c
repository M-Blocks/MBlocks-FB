/*
 * timer1.c
 *
 * Created: 4/10/2015 10:50:41 AM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/atomic.h>

static bool m_initialized = false;
static void (*m_overflowCallback)(void);

void timer1_init(void (*overflowCallback)(void)) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Stop and reset Timer/counter */
		TCCR1A = 0x00;
		TCCR1B = 0x00;
		TCCR1C = 0x00;

		/* Reset the counter to 0 */
		TCNT1 = 0x00;
	
		/* Select 16-bit fast PWM mode with the TOP equal to ICR1 so 
		 * that the counter will be reset when the counter reaches the value
		 * in ICR1.  Leave the counter disabled for now. */
		TCCR1A = (2<<WGM10);
		TCCR1B = (3<<WGM12);
		TCCR1C = 0x00;
		
		/* With an 8MHz clock divided by a prescalar of 64, each counter of the
		 * timer represents 8us.  For a overflow interrupt every 10ms, we set 
		 * the TOP value, held in the ICR1 register, to 10ms / 8us - 1 = 1249. 
		 */
		ICR1 = (1250 - 1);
	
		/* Enable just the overflow interrupt.  Note that the TIMSK register is
		 * also used to control Timer0 interrupts so we first clear all of the 
		 * Timer1 related bits and then set the overflow bit. */
		TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B));
		TIMSK |= (1<<TOIE1);
	
		/* Reset all Timer1 interrupt flags. Note that the TIFR register is also
		 * used to signal Timer0 interrupts.  */
		TIFR |= (1<<TOV1) | (1<<OCF1B) | (1<<OCF1A);
	
		/* Save the callback provided by the caller */
		m_overflowCallback = overflowCallback;
	
		/* Indicate that the timer has been initialized. */
		m_initialized = true;
	}
	
	/* Note that we leave the timer off for now */
}

bool timer1_start() {
	if (!m_initialized) {
		return false;
	}
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Enable counting with a prescalar of 64 */
		TCCR1B |= (3<<CS10);
	}
	
	return true;
}


bool timer1_stop() {
	if (!m_initialized) {
		return false;
	}
		
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Disable the clock */
		TCCR1B &= ~(7<<CS10);
	}
		
	return true;
}

ISR(TIMER1_OVF_vect) {
	if (m_overflowCallback != NULL) {
		m_overflowCallback();
	}
}