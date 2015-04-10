/*
 * sensors.c
 *
 * Created: 4/10/2015 12:59:56 PM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>

#include "defines.h"
#include "sensors.h"

void sensors_enable() {
	PORT_SENSOREN |= (1<<PIN_NUMBER_SENSOREN);
}

void sensors_disable() {
	PORT_SENSOREN &= ~(1<<PIN_NUMBER_SENSOREN);
}

bool sensors_getEnabled() {
	return ((PIN_SENSOREN & (1<<PIN_NUMBER_SENSOREN)) ? true : false);
}