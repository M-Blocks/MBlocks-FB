/*
 * adc.h
 *
 * Created: 11/24/2014 11:28:21 AM
 *  Author: kwgilpin
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>

#define ADC_REFAREF			0
#define ADC_REFAVCC			1
#define ADC_REF1V10			2


#define ADC_PRESCALE_2		1
#define ADC_PRESCALE_4		2
#define ADC_PRESCALE_8		3
#define ADC_PRESCALE_16		4
#define ADC_PRESCALE_32		5
#define ADC_PRESCALE_64		6
#define ADC_PRESCALE_128	7

#define ADC_CH_ADC0			0x00
#define ADC_CH_ADC1			0x01
#define ADC_CH_ADC2			0x02
#define ADC_CH_ADC3			0x03
#define ADC_CH_ADC4			0x04
#define ADC_CH_ADC5			0x05
#define ADC_CH_ADC6			0x06
#define ADC_CH_ADC7			0x07
#define ADC_CH_ADC8			0x08
#define ADC_CH_ADC9			0x09
#define ADC_CH_ADC10		0x0A
#define ADC_CH_ADC11		0x0B
#define ADC_CH_AGND			0x0C
#define ADC_CH_1V1BG		0x1D
#define ADC_CH_TEMP			0x1E

void adc_init(uint8_t vref, uint8_t prescalar);
uint16_t adc_convert(uint8_t channel, uint16_t nThrowaway, uint16_t nAvg);
uint32_t adc_convertAccum(uint8_t channel, uint16_t nThrowaway, uint16_t nAvg);

#endif /* ADC_H_ */