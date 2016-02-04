/*
 * FaceBoard.c
 *
 * Created: 11/5/2014 2:47:03 PM
 *  Author: kwgilpin
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/fuse.h>
#include <avr/lock.h>
#include <avr/signature.h>

#include <util/delay.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "defines.h"
#include "global.h"
#include "gitversion.h"
#include "timer0.h"
#include "timer1.h"
#include "leds.h"
#include "uart0.h"
#include "ircomm.h"
#include "irmsg.h"
#include "sensors.h"
#include "ambient.h"
#include "twiSlave.h"

static bool m_sleepRequested = false;

uint8_t getByteForMasterRead(uint8_t registerAddr, uint8_t dataByteCount, bool prevNACK);
bool getMasterWriteAllowed(uint8_t registerAddr);
bool processByteFromMasterWrite(uint8_t registerAddr, uint8_t dataByteCount, uint8_t dataByte);

static void sleep(void);
static void timer1_callback(void);

#ifndef DEBUG
FUSES = {
    .low = (FUSE_SUT_CKSEL4 & FUSE_SUT_CKSEL3 & FUSE_SUT_CKSEL2 & FUSE_SUT_CKSEL0), // internal 8MHz oscillator (6CK/16CK+16ms), no CKDIV8, no CKOUT
    .high = (FUSE_SPIEN & FUSE_BODLEVEL0), // 1.8V brownout voltage, no watchdog, no EE save, no SPI enabled, no debugWire, reset enabled
    .extended = (FUSE_BODPD1 & FUSE_BODACT0) // BOD enabled in active mode, sampled in power-down, no self-programming 
};
#endif

int main(void) {
    /* Make the pins driving the IR LED anodes outputs driving low.  We will
     * drive these pins high to turn on particular LEDs. */
    DDR_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
    PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	
    DDR_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
    PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	
    DDR_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
    PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	
    DDR_IRLED4_ANODE |= (1<<PIN_NUMBER_IRLED4_ANODE);
    PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
	
    /* Make the pin controlling power to the IR receiver and the ambient light
     * sensor an output driving low (to keep the sensors off for now). */
    DDR_SENSOREN |= (1<<PIN_NUMBER_SENSOREN);
    PORT_SENSOREN &= ~(1<<PIN_NUMBER_SENSOREN);
	
    /* Disable the digital input on the pin used for the ambient light sensor.
     */
    DIDR0 |= (1<<ADC3D);
	
    /* Disable power to unused peripherals (USI, USART1)*/
    PRR = (1<<PRUSI) | (1<<PRUSART1);
	
    /* Configure Timer0 to generate a 36kHz carrier wave for the IR 
     * communication system. */
    timer0_init(36);	

    /* Initialize the USART which we use to modulate the IR LEDs */
    uart0_init();
    /* Every time the UART receives a character, the irmsg_uartRxCallback
     * function will be called.  This function will start the sampling of the
     * ambient light sensor. */
    uart0_setRxCompleteCallback(irmsg_uartRxCallback);
	
    /* Initialize the RGB LEDs */
    leds_init();
		
    /* Initialize the timer which we use to strobe the two RGB LEDs*/
    timer1_init(timer1_callback);
	
    twiSlave_init(I2C_SLAVE_ADDR, true, getByteForMasterRead, getMasterWriteAllowed, processByteFromMasterWrite);
	
    /* Enable global interrupts */
    sei();
	
    /* Turn all LEDs on */
    leds_setStateTop(0x07);
    leds_setStateBottom(0x07);
	
    /* Delay 1 second */
    uint8_t i = 10;
    while (i-- >= 1) {
	_delay_ms(100);
    }
	
    /* Turn all LEDs off */
    leds_setStateTop(0x00);
    leds_setStateBottom(0x00);
	
    while (1) {
	if (m_sleepRequested) {
	    sleep();
	    m_sleepRequested = false;
	}
		
	twiSlave_busErrorCheck();
    }
}


uint8_t getByteForMasterRead(uint8_t registerAddr, uint8_t dataByteCount, bool prevNACK) {
    uint8_t response;
    int16_t getCharResp;
    static uint16_t ambientLightValue = 0x00;
	
    switch (registerAddr) {
    case I2C_REGISTER_ADDR_LEDS_TOP:
	response = leds_getStateTop();
	break;
			
    case I2C_REGISTER_ADDR_LEDS_BOTTOM:
	response = leds_getStateBottom();
	break;
			
    case I2C_REGISTER_ADDR_AMBIENT_LIGHT:
	if (!sensors_getEnabled()) {
	    response = 0x00;
	} else if (dataByteCount == 0) {
	    /* Left align the ADC result so that the master can get the
	     * 8 MSb's by just reading the first byte. */
	    ambientLightValue = ambient_get10Bit() << 6;
	    /* Reply with the most-significant byte first */
	    response = (uint8_t)(ambientLightValue >> 8);
	} else if (dataByteCount == 1) {
	    /* Reply with the least-significant byte second */
	    response = (uint8_t)(ambientLightValue & 0xFF);
	} else {
	    response = 0x00;
	}
	break;	
		
    case I2C_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL:
	response = g_irLEDManualState;
	break;
			
    case I2C_REGISTER_ADDR_TX_BUF:
	/* The transmit buffer is write-only, and we read always returns
	 * 0xFF. */
	response = 0xFF;
	break;
			
    case I2C_REGISTER_ADDR_TX_AVAILABLE_COUNT:
	response = uart0_getTxBufferAvailableCount();
	break;

    case I2C_REGISTER_ADDR_TX_LED_SELECT:
	response = ircomm_getEnabledLEDs();
	break;
			
    case I2C_REGISTER_ADDR_TX_MSG_CONTROL:
	if (irmsg_getTxActive()) {
	    response = (irmsg_getFlashIRLEDsPostTx() ? 0x02 : 0x00) | 0x01;	
	} else {
	    response = 0x00;	
	}
	break;
			
    case I2C_REGISTER_ADDR_TX_MSG_BUF:
	/* The transmit message buffer is write-only, and we read always 
	 * returns 0xFF. */
	response = 0xFF;
	break;
		
    case I2C_REGISTER_ADDR_RX_BUF:
	if ((getCharResp = uart0_getchar()) >= 0) {
	    response = (uint8_t)getCharResp;
	} else {
	    /* Return 0xFF characters once the receive buffer has been 
	     * emptied. */
	    response = 0xFF;
	}
	break;
			
    case I2C_REGISTER_ADDR_RX_CONSUMED_COUNT:
	response = uart0_getRxBufferConsumedCount();
	break;
			
    case I2C_REGISTER_ADDR_RX_FLUSH:
	/* The receive buffer flush register is write-only and always reads
	 * 0x00. */
	response = 0x00;
	break;

    case I2C_REGISTER_ADDR_RX_AMBIENT_BUF:
	if ((getCharResp = irmsg_getAmbientLightSample()) >= 0) {
	    response = (uint8_t)getCharResp;
	} else {
	    response = 0xFF;
	}
	break;

    case I2C_REGISTER_ADDR_RX_AMBIENT_CONSUMED_COUNT:
	response = irmsg_getAmbientLightSampleCount();
	break;
	
    case I2C_REGISTER_ADDR_SENSOR_ENABLE:
    case I2C_REGISTER_ADDR_RX_ENABLE:
	response = (sensors_getEnabled() ? 0x01 : 0x00);
	break;
		
    case I2C_REGISTER_ADDR_VERSION_STRING:
	if (dataByteCount < strlen(gitVersionStr)) {
	    response = gitVersionStr[dataByteCount];
	} else {
	    response = '\0';
	}
	break;
		
    case I2C_REGISTER_ADDR_VERSION:
	if (dataByteCount == 0) {
	    response = MAJOR_VERSION;
	} else if (dataByteCount == 1) {
	    response = MAJOR_VERSION;
	} else {
	    response = 0xFF;
	}
	break;
		
    default:
	response = 0xFF;
	break;
    }
	
    return response;
}

bool getMasterWriteAllowed(uint8_t registerAddr) {
    switch(registerAddr) {
    case I2C_REGISTER_ADDR_LEDS_TOP:
    case I2C_REGISTER_ADDR_LEDS_BOTTOM:
	return true;
			
    case I2C_REGISTER_ADDR_AMBIENT_LIGHT:
	/* The ambient light register is read-only */
	return false;
			
    case I2C_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL:
	/* We only allow manual control over the IR LEDs if the
	 * transmit buffer is empty because we will need to disable
	 * the transmitter to control the LEDs manually. */
	if (uart0_getTxBufferEmpty()) {
	    return true;
	} else {
	    return false;
	}

    case I2C_REGISTER_ADDR_TX_BUF:
	if ((g_irLEDManualState == 0x00) && (ircomm_getEnabledLEDs() != 0x00) && (uart0_getTxBufferAvailableCount() >= 1)) {
	    /* If there are no IR LEDs turned on in manual mode (i.e.
	     * without modulation), there are LED(s) selected for IR
	     * communication, and there is space in the transmit
	     * buffer, we allow the master to write to the transmit 
	     * register. */
	    return true;
	} else {
	    return false;
	}
			
    case I2C_REGISTER_ADDR_TX_AVAILABLE_COUNT:
	return false;

    case I2C_REGISTER_ADDR_TX_LED_SELECT:
	/* We only allow the IR LEDs used for transmission to be changed
	 * when the transmit buffer is empty. */
	if (uart0_getTxBufferEmpty()) {
	    return true;
	} else {
	    return false;
	}
			
    case I2C_REGISTER_ADDR_TX_MSG_CONTROL:
	/* If we are not busy transmitting a message already, we return
	 * true, indicating that the master is allowed to write to the 
	 * transmit message control register.  */
	if (!irmsg_getTxActive()) {
	    return true;
	}
	return false;
			
    case I2C_REGISTER_ADDR_TX_MSG_BUF:
	/* If we are not busy transmitting a message already and there is
	 * space available in the message transmit buffer, we return true,
	 * indicating that the I2C master is allowed to write to the 
	 * buffer. Otherwise, we return false. */
	if (!irmsg_getTxActive() && (irmsg_getTxBufferAvailableCount() > 0)) {
	    return true;
	}
	return false;
			
    case I2C_REGISTER_ADDR_RX_BUF:
    case I2C_REGISTER_ADDR_RX_CONSUMED_COUNT:
	/* The received data buffer and the received data counter registers
	 * are both read-only. */
	return false;

    case I2C_REGISTER_ADDR_RX_AMBIENT_BUF:
    case I2C_REGISTER_ADDR_RX_AMBIENT_CONSUMED_COUNT:
	/* The ambient light values post reception are read-only. */
	return false;

    case I2C_REGISTER_ADDR_SENSOR_ENABLE:
    case I2C_REGISTER_ADDR_RX_FLUSH:
    case I2C_REGISTER_ADDR_RX_ENABLE:
    case I2C_REGISTER_ADDR_SLEEP:
	return true;
		
    case I2C_REGISTER_ADDR_VERSION_STRING:
    case I2C_REGISTER_ADDR_VERSION:
	return false;
		
    default:
	return false;
    }
	
    return false;	
}


bool processByteFromMasterWrite(uint8_t registerAddr, uint8_t dataByteCount, uint8_t dataByte) {
    bool ack;
	
    switch(registerAddr) {
    case I2C_REGISTER_ADDR_LEDS_TOP:
	leds_setStateTop(dataByte);
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */			
	ack = false;
	break;
			
    case I2C_REGISTER_ADDR_LEDS_BOTTOM:
	leds_setStateBottom(dataByte);
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */
	ack = false;
	break;
			
    case I2C_REGISTER_ADDR_AMBIENT_LIGHT:
	/* The ambient light sensor value register is read-only */
	ack = false;
	break;
			
    case I2C_REGISTER_ADDR_IR_LEDS_MANUAL_CONTROL:
	if (uart0_getTxBufferEmpty()) {
	    /* We only allow manual control over the IR LEDs if the
	     * transmit buffer is empty because we will need to disable
	     * the transmitter to control the LEDs manually. */
				
	    /* Only the lower-order nibble is used to turn the IR LEDs on
	     * and off. */
	    dataByte &= 0x0F;
				
	    /* Save the state so that we if try to read the state while the
	     * UART is transmitting, we do not inadvertently read which IR
	     * LED is being used for transmission. */
	    g_irLEDManualState = dataByte;
				
	    if (g_irLEDManualState != 0x00) {
		/* If the master is attempting to turn at least one of the
		 * IR LEDs on manually, we disable the 36kHz IR carrier and
		 * the UART transmitter circuit, so that the OR gated 
		 * connected to all of the IR LEDs' cathodes will pull low.
		 */					 
		timer0_disableCarrierOutput();
		uart0_disableTx();
					
		/* Turn the individual LEDs on or off by pulling their
		 * anodes to VCC. */
		if (g_irLEDManualState & 0x01) {
		    PORT_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
		} else {
		    PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
		}
					
		if (g_irLEDManualState & 0x02) {
		    PORT_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
		} else {
		    PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
		}
					
		if (g_irLEDManualState & 0x04) {
		    PORT_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
		} else {
		    PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
		}
					
		if (g_irLEDManualState & 0x08) {
		    PORT_IRLED4_ANODE |= (1<<PIN_NUMBER_IRLED4_ANODE);
		} else {
		    PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
		}				
	    } else {
		/* Drive all IR LED anodes to ground */
		PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
		PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
		PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
		PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
		/* If the master is turning off all IR LEDs, we re-enable
		 * the UART transmitter, which will pull the TXD0 line high
		 * by default. With TXD0 high, the output of the OR gate 
		 * which drives all of the IR LEDs' cathodes will pull 
		 * high, thereby turning off all of the IR LEDs. */
		uart0_enableTx();
	    }
	}
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */			
	ack = false;
	break;		
			
    case I2C_REGISTER_ADDR_TX_BUF:
	if ((g_irLEDManualState == 0x00) && (ircomm_getEnabledLEDs() != 0x00)) {
	    /* If there are no IR LEDs turned on in manual mode (i.e.
	     * without modulation) and there are LED(s) select for IR
	     * communication, we allow the master to write bytes to
	     * the transmit buffer. Note that the transmitter should 
	     * already be enabled as it is always re-enabled when none of
	     * the IR LEDs are in manual mode. */
	    uart0_putchar(dataByte);
	    /* So long as there is still space for at least one additional
	     * byte in the transmit buffer, continue to ACK the master's
	     * transmissions. */
	    if (uart0_getTxBufferAvailableCount() >= 1) {
		ack = true;
	    } else {
		ack = false;
	    }
	} else {
	    /* If one or more IR LEDs are configured for manual, always-on
	     * mode, prevent writing to the transmit buffer. */
	    ack = false;
	}
	break;
			
    case I2C_REGISTER_ADDR_TX_AVAILABLE_COUNT:
	/* The transmit buffer counter register is read-only. */
	ack = false;
	break;

    case I2C_REGISTER_ADDR_TX_LED_SELECT:
	/* We only allow the IR LEDs used for transmission to be changed
	 * when the transmit buffer is empty. */
	if (uart0_getTxBufferEmpty()) {
	    ircomm_setEnabledLEDs(dataByte);
	}
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */				
	ack = false;
	break;	
			
    case I2C_REGISTER_ADDR_TX_MSG_CONTROL:
	if ((dataByte & ~0x02) == 0x01) {
	    /* Iff the LSb is set and bits 7:2 are all clear, we attempt to
	     * start the transmission.  Bit 1 is used to indicate whether 
	     * the IR LEDs should be flashed after the transmission.  */ 
	    irmsg_txQueuedMsg(dataByte & 0x02 ? true : false);
	}
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */
	ack = false;
	break;	
		
    case I2C_REGISTER_ADDR_TX_MSG_BUF:
	if (irmsg_getTxActive()) {
	    /* If we are already busy transmitting a message, NACK the
	     * byte.  We should have already NACK'd the write request, but
	     * in case the master insists on attempting to write, we 
	     * continue to NACK. */
	    ack = false;
	} else if (irmsg_queueTxChar(dataByte) >= 1) {
	    /* If the number of bytes remaining in the message transmit 
	     * buffer is at least 1, ack the received byte to indicate that
	     * we can accept another byte. */
	    ack = true;
	} else {
	    ack = false;
	}
	break;		
			
    case I2C_REGISTER_ADDR_RX_BUF:
    case I2C_REGISTER_ADDR_RX_CONSUMED_COUNT:
	/* The receive buffer and receive buffer count registers are both
	 * read-only. */
	ack = false;
	break;
			
    case I2C_REGISTER_ADDR_RX_FLUSH:
	uart0_flushRxBuffer();
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */				
	ack = false;
	break;

    case I2C_REGISTER_ADDR_RX_AMBIENT_BUF:
    case I2C_REGISTER_ADDR_RX_AMBIENT_CONSUMED_COUNT:
	/* The ambient light values post reception are read-only. */
	ack = false;
	return false;
		
    case I2C_REGISTER_ADDR_SENSOR_ENABLE:
    case I2C_REGISTER_ADDR_RX_ENABLE:	
	if (dataByte & 0x01) {
	    sensors_enable();
	} else {
	    sensors_disable();
	}
	ack = false;	
	break;
		
    case I2C_REGISTER_ADDR_SLEEP:
	/* Set a flag that is trapped in the main loop. */
	m_sleepRequested = true;
	/* NACK to indicate that this is the first, last, and only byte 
	 * that can be written. */				
	ack = false;
	break;	

    case I2C_REGISTER_ADDR_VERSION_STRING:			
    case I2C_REGISTER_ADDR_VERSION:
	/* The version registers are read-only */
	ack = false;
	break;
					
    default:
	ack = false;
	break;
    }
	
    return ack;
}


void sleep() {
    /* Remove power from the ADC */
    PRR |= (1<<PRADC);
	
    /* Disable the 36kHz output used for IR transmissions and power-down Timer0 */
    timer0_deinit();
    PRR |= (1<<PRTIM0);
	
    /* Cut power to the IR demodulator and light sensor */
    PORT_SENSOREN &= ~(1<<PIN_NUMBER_SENSOREN);
	
    /* Drive the anodes of all the IR LEDs low */
    PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
    PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
    PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
    PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);	
	
    /* Disable and power-down the UART0 */
    uart0_deinit();
    PRR |= (1<<PRUSART0);
	
    /* Turn off all RGB LEDs, but note that this make take up to 20ms to take
     * effect, so we manually set the RGB LED control lines below. */
    leds_setStateTop(0x00);
    leds_setStateBottom(0x00);
	
    /* Stop the timer which strobes the RGB LEDs and then power it down. */
    timer1_stop();
    PRR |= (1<<PRTIM1);
	
    /* Turn off the RGB LEDs: drive both the anodes and cathodes to ground. */
    PORT_LED_ANODE1 &= ~(1<<PIN_NUMBER_LED_ANODE1);
    PORT_LED_ANODE2 &= ~(1<<PIN_NUMBER_LED_ANODE2);
	
    PORT_LED_RED &= ~(1<<PIN_NUMBER_LED_RED);
    PORT_LED_GREEN &= ~(1<<PIN_NUMBER_LED_GREEN);
    PORT_LED_BLUE &= ~(1<<PIN_NUMBER_LED_BLUE);
	
    /* Enter sleep */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
	
    /* When we wake-up, we'll resume executing from here */
	
    /* Bring the RGB anodes and cathods high again */
    PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
    PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
    PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
	
    PORT_LED_ANODE1 |= (1<<PIN_NUMBER_LED_ANODE1);
    PORT_LED_ANODE2 |= (1<<PIN_NUMBER_LED_ANODE2);
	
    /* Restore power to the timer which strobes the RGB LEDs and then re-
     * initialize it, just to be safe.  It will automatically be restarted when
     * the RGB LEDs are next instructed to turn on. */
    PRR &= ~(1<<PRTIM1);
    timer1_init(timer1_callback);
	
    /* Restore power to and re-initialize the UART */
    PRR &= ~(1<<PRUSART0);
    uart0_init();
	
    /* Restore power to and re-initialize Timer0 */
    PRR &= ~(1<<PRTIM0);
    timer0_init(36);
	
    /* Restore power to the ADC */
    PRR &= ~(1<<PRADC);
}


void timer1_callback() {
    leds_updateCallback();
	
    if (g_postTxIRLEDsFlashingActive || g_postTxIRLEDsFlashingStart) {
	irmsg_postTxFlashIRLEDsCallback();
    }
	
    if (g_postRxAmbientLightSamplingActive || g_postRxAmbientLightSamplingStart) {
	irmsg_postRxSampleAmbientLightCallback();
    }
	
    if ((leds_getStateTop() == 0x00) && (leds_getStateBottom() == 0x00) && 
	(!g_postTxIRLEDsFlashingActive) && (!g_postRxAmbientLightSamplingActive)) {
	timer1_stop();
    }
}
