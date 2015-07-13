/*
 * global.h
 *
 * Created: 6/15/2015 2:39:18 PM
 *  Author: kwgilpin
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_


#include <stdint.h>
#include <stdbool.h>

extern uint8_t g_irLEDManualState;

extern bool g_postTxIRLEDsFlashingStart;
extern bool g_postTxIRLEDsFlashingActive;

extern bool g_postRxAmbientLightSamplingStart;
extern bool g_postRxAmbientLightSamplingActive;

#endif /* GLOBAL_H_ */