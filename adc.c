/*
 * adc.c
 *
 * Created: 11/24/2014 11:28:08 AM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

static void (*conversionCompleteCallback)(uint16_t) = NULL;

void adc_init(uint8_t vref, uint8_t prescalar) {
	/* Clear the reference bits without modifying the REFEN, ADC0EN, or MUX 
	 * bits. */
	ADMUX &= ~(3<<REFS0);
	
	/* Select the reference as specified  */
	ADMUX |= ((0x03 & vref) << REFS0);
	
	/* Clear the enable, start, auto trigger, interrupt flag, and prescalar 
	 * bits by writing 0's to them.  Clear the interrupt flag by writing a 1 to
	 * it. */
	ADCSRA = (1<<ADIF);
	
	/* Set the prescalar bits as specified */
	ADCSRA |= ((0x07 & prescalar) << ADPS0);
	
	/* Clear the VDEN and VDPD bits (which are reserved from QTouch), right-
	 * adjust the conversion result, and select free running mode. */
	ADCSRB = 0x00;
}

uint16_t adc_convert(uint8_t channel, uint16_t nThrowaway, uint16_t nAvg) {
	uint32_t accum;
	
	/* Accumulate the results of many conversions */
	accum = adc_convertAccum(channel, nThrowaway, nAvg);
	
	/* Return the average of the samples */
	return accum / nAvg;
}


uint32_t adc_convertAccum(uint8_t channel, uint16_t nThrowaway, uint16_t nAvg) {
	uint16_t i;
	uint32_t accum = 0;
	
	/* Clear the MUX bits without touching the reference bits */
	ADMUX &= ~(31 << MUX0);
	/* Set the MUX bits as specified */
	ADMUX |= ((0x0F & channel) << MUX0);
	
	/* Enable the ADC */
	ADCSRA |= (1<<ADEN);
	
	/* Throw-away the specified number of samples */
	for (i=0; i<nThrowaway; i++) {
		adc_oneSample();
	}
	
	/* Accumulate the specified number of samples */
	for (i=0; i<nAvg; i++) {
		accum += adc_oneSample();
	}
	
	/* Disable the ADC */
	ADCSRA &= ~(1<<ADEN);
	
	/* Return the accumulated value. */
	return accum;
}


uint16_t adc_oneSample() {
	/* Ensure that the ADC is enabled so that it does not hang after we start 
	 * the conversion. */
	ADCSRA |= (1<<ADEN);

	/* Wait for any ongoing conversion to finish */
	while (ADCSRA & (1<<ADSC));
	
	/* Start the new conversion */
	ADCSRA |= (1<<ADSC);
	
	/* Wait for it to complete */
	while (ADCSRA & (1<<ADSC));
	
	/* Return the result of the conversion */
	return ADC;
}

bool adc_oneSampleInterrupt(void (*callback)(uint16_t)) {
	/* Ensure that the ADC is enabled so that it does not hang after we start 
	 * the conversion. */
	ADCSRA |= (1<<ADEN);

	/* Wait for any ongoing conversion to finish */
	while (ADCSRA & (1<<ADSC));
	
	/* Save the callback address. */
	conversionCompleteCallback = callback;
	
	/* Clear the interrupt flag */
	ADCSRA |= (1<<ADIF);
	
	/* Enable the ADC interrupt */
	ADCSRA |= (1<<ADIE);
	
	/* Start the new conversion */
	ADCSRA |= (1<<ADSC);
	
	return true;
}

ISR(ADC_vect) {
	/* Disable the interrupt */
	ADCSRA &= ~(1<<ADIE);
	
	/* If it exists, execute the ADC conversion complete callback. */
	if (conversionCompleteCallback != NULL) {
		conversionCompleteCallback(ADC);
	}
}