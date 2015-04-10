/*
 * ircomm.c
 *
 * Created: 11/24/2014 10:54:29 AM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>

#include "defines.h"
#include "ircomm.h"

static uint8_t m_activeIRLEDs = 0x00;

void ircomm_setEnabledLEDs(uint8_t activeLEDs) {
	m_activeIRLEDs = activeLEDs & 0x0F;
}

uint8_t ircomm_getEnabledLEDs() {
	return m_activeIRLEDs;
}

void ircomm_activateEnabledLEDs() {
	if (m_activeIRLEDs & 0x01) {
		PORT_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
	} else {
		PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	}
	
	if (m_activeIRLEDs & 0x02) {
		PORT_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
		} else {
		PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	}
	
	if (m_activeIRLEDs & 0x04) {
		PORT_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
		} else {
		PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	}
	
	if (m_activeIRLEDs & 0x08) {
		PORT_IRLED4_ANODE |= (1<<PIN_NUMBER_IRLED4_ANODE);
	} else {
		PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
	}
}

void ircomm_deactivateAllLEDs() {
	PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
}