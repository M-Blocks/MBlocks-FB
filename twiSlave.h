/*
 * twiSlave.h
 *
 * Created: 4/9/2015 9:29:02 PM
 *  Author: kwgilpin
 */ 


#ifndef TWISLAVE_H_
#define TWISLAVE_H_

#include <stdint.h>
#include <stdbool.h>

void twiSlave_init(uint8_t slaveAddr, bool generalCallEnable,
		uint8_t (*getByteForMasterReadFcnPtr)(uint8_t registerAddr, uint8_t dataByteCount, bool prevNACK),
		bool (*processByteFromMasterWriteFcnPtr)(uint8_t registerAddr, uint8_t dataByteCount, uint8_t dataByte));
void twiSlave_busErrorCheck(void);


#endif /* TWISLAVE_H_ */