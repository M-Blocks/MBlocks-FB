/*
 * leds.c
 *
 * Created: 4/10/2015 11:16:44 AM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>

#include <util/atomic.h>

#include "defines.h"
#include "timer1.h"
#include "leds.h"

#if defined(I2C_SLAVE_ADDR) 
#	if (I2C_SLAVE_ADDR == 1)
#		define PORT_LED_ANODE_TOP			PORT_LED_ANODE1
#		define PIN_NUMBER_LED_ANODE_TOP		PIN_NUMBER_LED_ANODE1
#		define PORT_LED_ANODE_BOTTOM		PORT_LED_ANODE2
#		define PIN_NUMBER_LED_ANODE_BOTTOM	PIN_NUMBER_LED_ANODE2
#	elif (I2C_SLAVE_ADDR == 2)
#		define PORT_LED_ANODE_TOP			PORT_LED_ANODE1
#		define PIN_NUMBER_LED_ANODE_TOP		PIN_NUMBER_LED_ANODE1
#		define PORT_LED_ANODE_BOTTOM		PORT_LED_ANODE2
#		define PIN_NUMBER_LED_ANODE_BOTTOM	PIN_NUMBER_LED_ANODE2
#	elif (I2C_SLAVE_ADDR == 3)
#		define PORT_LED_ANODE_TOP			PORT_LED_ANODE1
#		define PIN_NUMBER_LED_ANODE_TOP		PIN_NUMBER_LED_ANODE1
#		define PORT_LED_ANODE_BOTTOM		PORT_LED_ANODE2
#		define PIN_NUMBER_LED_ANODE_BOTTOM	PIN_NUMBER_LED_ANODE2
#	elif (I2C_SLAVE_ADDR == 4)
#		define PORT_LED_ANODE_TOP			PORT_LED_ANODE1
#		define PIN_NUMBER_LED_ANODE_TOP		PIN_NUMBER_LED_ANODE1
#		define PORT_LED_ANODE_BOTTOM		PORT_LED_ANODE2
#		define PIN_NUMBER_LED_ANODE_BOTTOM	PIN_NUMBER_LED_ANODE2
#	elif (I2C_SLAVE_ADDR== 5)
#		define PORT_LED_ANODE_TOP			PORT_LED_ANODE1
#		define PIN_NUMBER_LED_ANODE_TOP		PIN_NUMBER_LED_ANODE1
#		define PORT_LED_ANODE_BOTTOM		PORT_LED_ANODE2
#		define PIN_NUMBER_LED_ANODE_BOTTOM	PIN_NUMBER_LED_ANODE2
#	elif (I2C_SLAVE_ADDR == 6)
#		define PORT_LED_ANODE_TOP			PORT_LED_ANODE1
#		define PIN_NUMBER_LED_ANODE_TOP		PIN_NUMBER_LED_ANODE1
#		define PORT_LED_ANODE_BOTTOM		PORT_LED_ANODE2
#		define PIN_NUMBER_LED_ANODE_BOTTOM	PIN_NUMBER_LED_ANODE2
#	else
#		error "Unrecognized I2C_SLAVE_ADDR"
#	endif
#else
#	error "I2C_SLAVE_ADDR not defined"
#endif


static bool m_initialized = false;
static uint8_t m_ledStateTop = 0x00;
static uint8_t m_ledStateBottom = 0x00;

void leds_init() {
	/* Make the pins driving the RGB LED's cathodes outputs driving high (to 
	 * keep the LEDs off).  */
	DDR_LED_RED |= (1<<PIN_NUMBER_LED_RED);
	PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
	
	DDR_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
	PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
	
	DDR_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
	PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
	
	/* Make the pins driving the RGB LED's anodes outputs driving low */
	DDR_LED_ANODE1 |= (1<<PIN_NUMBER_LED_ANODE1);
	PORT_LED_ANODE1 &= (1<<PIN_NUMBER_LED_ANODE1);
	
	DDR_LED_ANODE2 |= (1<<PIN_NUMBER_LED_ANODE2);
	PORT_LED_ANODE2 &= (1<<PIN_NUMBER_LED_ANODE2);
	
	m_initialized = true;
}

void leds_setStateTop(uint8_t ledStateTop) {	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* These variables are read by an ISR, so we want to make sure that
		 * writes are atomic. */
		m_ledStateTop = ledStateTop & 0x07;
	}
	
	/* If some LED is supposed to be on, start the timer which controls the
	 * multiplexing/strobing of the two LEDs.  Note that there is no harm in
	 * calling the *_start() function even if the timer is already running. */
	if ((m_ledStateTop != 0x00) || (m_ledStateBottom != 0x00)) {
		timer1_start();
	}
}

uint8_t leds_getStateTop() {
	return m_ledStateTop;
}

void leds_setStateBottom(uint8_t ledStateBottom) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* These variables are read by an ISR, so we want to make sure that
		 * writes are atomic. */
		m_ledStateBottom = ledStateBottom & 0x07;
	}
	
	/* If some LED is supposed to be on, start the timer which controls the
	 * multiplexing/strobing of the two LEDs.  Note that there is no harm in
	 * calling the *_start() function even if the timer is already running. */
	if ((m_ledStateTop != 0x00) || (m_ledStateBottom != 0x00)) {
		timer1_start();
	}
}

uint8_t leds_getStateBottom() {
	return m_ledStateBottom;
}

void leds_updateCallback() {
	static bool topActive = true;
	
	if (!m_initialized) {
		return;
	}
	
	if (topActive) {
		/* Drive the anode for the top LED (nearer to face 6) to ground. */
		PORT_LED_ANODE_TOP &= ~(1<<PIN_NUMBER_LED_ANODE_TOP);
		
		/* Drive the cathodes for the channels (RGB) of the bottom LED that are
		 * supposed to be on to ground, (and the other channels to VCC) */
		if (m_ledStateBottom & 0x01) {
			PORT_LED_RED &= ~(1<<PIN_NUMBER_LED_RED);
		} else {
			PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
		}
		
		if (m_ledStateBottom & 0x02) {
			PORT_LED_GREEN &= ~(1<<PIN_NUMBER_LED_GREEN);
		} else {
			PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
		}
		
		if (m_ledStateBottom & 0x04) {
			PORT_LED_BLUE &= ~(1<<PIN_NUMBER_LED_BLUE);
		} else {
			PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
		}
		
		/* Drive the anode for the bottom LED (nearer to face 5) to VCC */
		PORT_LED_ANODE_BOTTOM |= (1<<PIN_NUMBER_LED_ANODE_BOTTOM);
		
		topActive = false;
	} else {
		/* Drive the anode for the bottom LED (nearer to face 5) to ground. */
		PORT_LED_ANODE_BOTTOM &= ~(1<<PIN_NUMBER_LED_ANODE_BOTTOM);
		
		/* Drive the cathodes for the channels (RGB) of the top LED that are
		 * supposed to be on to ground, (and the other channels to VCC) */
		if (m_ledStateTop & 0x01) {
			PORT_LED_RED &= ~(1<<PIN_NUMBER_LED_RED);
		} else {
			PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
		}
		
		if (m_ledStateTop & 0x02) {
			PORT_LED_GREEN &= ~(1<<PIN_NUMBER_LED_GREEN);
		} else {
			PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
		}
		
		if (m_ledStateTop & 0x04) {
			PORT_LED_BLUE &= ~(1<<PIN_NUMBER_LED_BLUE);
		} else {
			PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
		}
		
		/* Drive the anode for the top LED (nearer to face 6) to VCC */
		PORT_LED_ANODE_TOP |= (1<<PIN_NUMBER_LED_ANODE_TOP);
		
		topActive = true;		
	}
	
	if ((m_ledStateTop == 0x00) && (m_ledStateBottom == 0x00)) {
		/* No need to keep the timer running if all LEDs are off */
		timer1_stop();
	}
}