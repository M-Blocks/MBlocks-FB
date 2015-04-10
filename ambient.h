/*
 * ambient.h
 *
 * Created: 11/24/2014 12:20:16 PM
 *  Author: kwgilpin
 */ 


#ifndef AMBIENT_H_
#define AMBIENT_H_

#include <stdint.h>
#include <stdbool.h>

uint8_t ambient_get8Bit(void);
uint16_t ambient_get10Bit(void);

#endif /* AMBIENT_H_ */