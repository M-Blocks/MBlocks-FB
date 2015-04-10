/*
 * leds.h
 *
 * Created: 4/10/2015 11:16:37 AM
 *  Author: kwgilpin
 */ 


#ifndef LEDS_H_
#define LEDS_H_

#include <stdint.h>
#include <stdbool.h>

void leds_init(void);

void leds_setStateTop(uint8_t ledStateTop);
uint8_t leds_getStateTop(void);

void leds_setStateBottom(uint8_t ledStateBottom);
uint8_t leds_getStateBottom(void);

void leds_updateCallback(void);

#endif /* LEDS_H_ */