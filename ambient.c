/*
 * ambient.c
 *
 * Created: 11/24/2014 12:14:21 PM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include "adc.h"
#include "ambient.h"

uint8_t ambient_get8Bit() {
	adc_init(ADC_REF1V10, ADC_PRESCALE_8);
	
	/* The result of the conversion is a 10-bit integer, which we right-shift
	 * by two bits so that we only return the 8 most significant bits. */
	return (uint8_t)(adc_convert(ADC_CH_ADC8, 8, 16) >> 2);
}