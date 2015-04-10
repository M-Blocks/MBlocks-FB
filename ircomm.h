/*
 * ircomm.h
 *
 * Created: 4/10/2015 1:53:52 PM
 *  Author: kwgilpin
 */ 


#ifndef IRCOMM_H_
#define IRCOMM_H_

#include <stdint.h>
#include <stdbool.h>

void ircomm_setEnabledLEDs(uint8_t activeLEDs);
uint8_t ircomm_getEnabledLEDs(void);

void ircomm_activateEnabledLEDs(void);
void ircomm_deactivateAllLEDs(void);

#endif /* IRCOMM_H_ */