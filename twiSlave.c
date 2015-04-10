/*
 * twiSlave.c
 *
 * Created: 4/9/2015 2:27:29 PM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/atomic.h>

static bool m_initialized = false;

static volatile bool m_waitingForAddr;
static volatile uint8_t m_registerAddr;
static volatile uint8_t m_dataByteCount;

static uint8_t (*m_getByteForMasterRead)(uint8_t registerAddr, uint8_t dataByteCount, bool prevNACK) = NULL;
static bool (*m_processByteFromMasterWrite)(uint8_t registerAddr, uint8_t dataByteCount, uint8_t dataByte) = NULL;

void twiSlave_init(uint8_t slaveAddr, bool generalCallEnable,
		uint8_t (*getByteForMasterReadFcnPtr)(uint8_t registerAddr, uint8_t dataByteCount, bool prevNACK),
		bool (*processByteFromMasterWriteFcnPtr)(uint8_t registerAddr, uint8_t dataByteCount, uint8_t dataByte)) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Disable the TWI interface */
		TWSCRA = 0x00;
	
		m_registerAddr = 0x00;
		m_waitingForAddr = false;
		m_dataByteCount = 0;
	
		m_getByteForMasterRead = getByteForMasterReadFcnPtr;
		m_processByteFromMasterWrite = processByteFromMasterWriteFcnPtr;
	
		/* Set the address.  The least significant bit, if set, enable the Atmel to
		 * respond to the general call address. */
		TWSA = (slaveAddr << 1) | (generalCallEnable ? 0x1: 0x0);
   
		/* Make PB1 and PC1, SDA and SCL respectively, inputs */
		DDRB &= ~(1<<DDRB1);
		DDRC &= ~(1<<DDRC1);
   
		/* Enable 50ns hold on changing SDA after falling edge of SCL, enable the 
		 * data, address, and stop interrupts, and enable the TWI slave interface. 
		 */
		TWSCRA = (1<<TWSHE) | (1<<TWDIE) | (1<<TWASIE) | (1<<TWEN) | (1<<TWSIE);
	   
		m_initialized = true;
	}
} 


ISR(TWI_vect) {
	bool nack;
	
	if (TWSSRA & (1<<TWDIF)) {
		/* If the data interrupt flag is set... */
		if (TWSSRA & (1<<TWDIR)) {
			/* If the master is reading data from the slave... */
			
			/* If the TWRA bit is set, the master (receiver) NACK'd the
			 * previous byte that we sent it. */
			nack = ((TWSSRA & (1<<TWRA)) != 0x00);
			
			/* Load the data register with the next byte, which we get from a
			 * callback.  We provide the callback with the internal register
			 * address from which the master is reading, the current byte 
			 * count, and a flag indicating whether the master ACK'd or NACK'd
			 * the last byte it read. */
			if (m_getByteForMasterRead != NULL) {
				TWSD = m_getByteForMasterRead(m_registerAddr, m_dataByteCount++, nack);
			} else {
				TWSD = 0xFF;
			}
			
			/* Since the bus is in master receiver mode, the master controls
			 * the ACK/NACK bit, so we just leave the TWAA bit alone.  Writing
			 * 2'b11 to the CMD field releases the SCL hold and allows the master
			 * to read the byte that we just loaded. */
			TWSCRB = (1<<TWCMD1) | (1<<TWCMD0);
		} else {
			/* If the master is writing data to the slave... */
			
			if (m_waitingForAddr) {
				m_registerAddr = TWSD;
				/* Now that we have the register address to/from which the 
				 * master wishes to write/read, we clear the flag which we
				 * used to capture the address. */
				m_waitingForAddr = false;
				/* We always ACK the address byte */
				nack = false;
			} else {
				/* If the master continues writing bytes after the address 
				 * byte, we execute a processing callback for each.  Note
				 * that we provide the callback with the current byte count,
				 * which we increment with each additional byte.  The callback
				 * must return true if it wishes the incoming byte to be ACK'd.
				 */
				if (m_processByteFromMasterWrite != NULL) {
					nack = !m_processByteFromMasterWrite(m_registerAddr, m_dataByteCount++, TWSD);
				} else {
					/* If the data processing function does not exist, simply
					 * continue to ACK all incoming data. */
					nack = false;
				}
			}
			
			/* Based on the nack variable, set or clear the acknowledge action
			 * bit and then issues a 2'b11 command in order to execute the
			 * acknowledge action and wait for the next byte. */
			TWSCRB = ((nack?1:0)<<TWAA) | (1<<TWCMD1) | (1<<TWCMD0);
		}
	} else if (TWSSRA & (1<<TWASIF)) {
		/* If the address match/stop detected interrupt flag is set...*/

		/* A bus collision masquerades as an address match except that the
		 * collision flag will be set. If it is see, clear all flags and resume
		 * waiting for a start or restart condition. */
		if (TWSSRA & (1<<TWC)) {
			/* Clear all flags by writing them to 1 */
			TWSSRA = (1<<TWDIF) | (1<<TWASIF) | (1<<TWC) | (1<<TWBE);
			/* Wait for any Start Condition by writing 2'b10 to the command
			 * field in control register B. */
			TWSCRB = (1<<TWCMD1) | (0<<TWCMD0);
		} else {
			/* Otherwise, if the collision flag is not set, we have detected
			 * a legitimate address match or a stop condition. */
			if (TWSSRA & (1<<TWAS)) {
				/* If the TWAS bit is set, an address match has occurred. */
				
				/* After every start condition, reset the byte counter */
				m_dataByteCount = 0;
				
				if ((TWSSRA & (1<<TWDIR)) == 0x00) {
					/* When we detect a start condition and the TWDIR bit 
					 * indicates that it is the beginning of a master write 
					 * (the bit is cleared), we set the flag which indicates 
					 * that we are waiting for an address byte.  That is, we
					 * assume that the first byte of every master write 
					 * transaction is an internal address within the Atmel's
					 * I2C slave interface to/from which the master wishes to
					 * write/read data.  (In order to read data, the master 
					 * write a single byte (the desired register address) and
					 * then issues a restart or a stop followed by a start 
					 * condition followed by a master read transaction. */
					m_waitingForAddr = true;
				}
				
				/* We write 2'b11 to the command field in control register B to
				 * generate an ACK (because we leave the TWAA bit is not set). 
				 * If the master is writing data to us, the hardware will 
				 * automatically receive the next byte after generating the 
				 * ACK.  If the master is reading data from us, the TWDIF flag
				 * will be set after the ACK is generated so that we can load
				 * the data register with data to be read by the master. */
				TWSCRB = (1<<TWCMD1) | (1<<TWCMD0);				
			}  else {
				/* If we have detected a stop condition, we write 2'b10 to the
				 * command field in control register B in order to resume 
				 * waiting for the next start or restart condition. */
				TWSCRB = (1<<TWCMD1) | (0<<TWCMD0);
			}
		}
	}
}

void twiSlave_busErrorCheck() {
	if (!m_initialized) {
		return;
	}
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* If an error, i.e. illegal bus condition (repeated START or STOP and the
		 * number of bits since the last START is not a multiple of 9) the bus 
		 * error flags will be set.  We clear all flags and then re-initialize the
		 * TWI interface. */
		if (TWSSRA & (1<<TWBE)) {
			/* Clear all flags by writing them to 1 */
			TWSSRA = (1<<TWDIF) | (1<<TWASIF) | (1<<TWC) | (1<<TWBE);

			/* Disable the TWI interface */
			TWSCRA = 0x00;

			/* Re-initialize the interface */

			/* Default data */
			TWSD = 0xFF;

			/* Enable 50ns hold on changing SDA after falling edge of SCL, enable the 
			 * data, address, and stop interrupts, and enable the TWI slave interface. 
			 */
			TWSCRA = (1<<TWSHE) | (1<<TWDIE) | (1<<TWASIE) | (1<<TWEN) | (1<<TWSIE);
		}
	}
}