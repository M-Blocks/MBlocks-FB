/*
 * timer0.h
 *
 * Created: 11/7/2014 8:02:00 AM
 *  Author: kwgilpin
 */ 


#ifndef TIMER0_H_
#define TIMER0_H_

#include <stdint.h>
#include <stdbool.h>

bool timer0_init(uint8_t freq_khz);
void timer0_deinit(void);

bool timer0_enableCarrierOutput(void);
bool timer0_disableCarrierOutput(void);
bool timer0_getCarrierStatus(void);

#endif /* TIMER0_H_ */