/*
 * irmsg.h
 *
 * Created: 6/15/2015 12:28:18 PM
 *  Author: kwgilpin
 */ 


#ifndef IRMSG_H_
#define IRMSG_H_

uint8_t irmsg_getTxBufferAvailableCount(void);
bool irmsg_getFlashIRLEDsPostTx(void);
int8_t irmsg_queueTxChar(uint8_t c);
bool irmsg_txQueuedMsg(bool flashIRLEDsPostTx);
bool irmsg_getTxActive(void);

bool irmsg_getAmbientLightSamplingActive(void);
uint8_t irmsg_getAmbientLightSampleCount(void);
int16_t irmsg_getAmbientLightSample(void);

void irmsg_uartTxCompleteCallback(void);
void irmsg_postTxFlashIRLEDsCallback(void);

void irmsg_uartRxCallback(uint8_t c);
void irmsg_postRxSampleAmbientLightCallback(void);
void irmsg_adcConversionCompleteCallback(uint16_t result);

#endif /* IRMSG_H_ */
