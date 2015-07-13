/*
 * global.c
 *
 * Created: 6/15/2015 2:39:10 PM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include "global.h"

uint8_t g_irLEDManualState = 0x00;

bool g_postTxIRLEDsFlashingStart = false;
bool g_postTxIRLEDsFlashingActive = false;

bool g_postRxAmbientLightSamplingStart = false;
bool g_postRxAmbientLightSamplingActive = false;