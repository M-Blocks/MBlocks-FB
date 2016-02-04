/*
 * irmsg.c
 *
 * Created: 6/15/2015 12:28:04 PM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <util/atomic.h>

#include "defines.h"
#include "global.h"
#include "adc.h"
#include "uart0.h"
#include "timer0.h"
#include "timer1.h"
#include "ircomm.h"
#include "irmsg.h"

#define TX_BUF_SIZE	(64)
#define RX_BUF_SIZE	(TX_BUF_SIZE)

#define TIMER_PERIODS_IN_SHORTEST_IRLED_FLASH	(4)
#define IRLED_FLASH_CYCLES			(3)

#define AMBIENT_LIGHT_SAMPLE_COUNT	(200)

static uint8_t m_txBufLength = 0;
static uint8_t m_txBuf[TX_BUF_SIZE];

static uint8_t m_rxBufLength = 0;
static uint8_t m_rxBuf[RX_BUF_SIZE];
static bool m_rxBufFull = false;

static bool m_txActive = false;
static bool m_flashIRLEDsPostTx = false;

static int m_sampleIndex = 0;
static uint8_t m_postRxAmbientLightSampleCounter = 0;
static uint8_t m_postRxAmbientLightSamples[AMBIENT_LIGHT_SAMPLE_COUNT];

static uint8_t m_ambientLightSamplePeriodsWithoutNewCharacter = 0;


uint8_t irmsg_getTxBufferAvailableCount() {
    if (m_txActive) {
	return 0;
    }
	
    return (TX_BUF_SIZE - m_txBufLength);
}

bool irmsg_getFlashIRLEDsPostTx() {
    return m_flashIRLEDsPostTx;
}

int8_t irmsg_queueTxChar(uint8_t c) {
    if (m_txActive) {
	/* Return -1 if we are currently transmitting a message */
	return -1;
    }
	
    if (m_txBufLength < TX_BUF_SIZE) {
	m_txBuf[m_txBufLength++] = c;
    } else {
	/* Return -1 if the character could not be added to the transmit
	 * buffer. */
	return -1;
    }
	
    /* Return the number of bytes still available in the buffer. */
    return (TX_BUF_SIZE - m_txBufLength);
}

bool irmsg_txQueuedMsg(bool flashIRLEDsPostTx) {
    uint8_t i;

    if (m_txActive) {
	/* If a transmission is already in progress, return true */
	return true;
    }
	
    if (uart0_getTxBufferConsumedCount() != 0) {
	/* Return false if the UART's transmit buffer is not empty */
	return false;
    }	
	
    if ((g_irLEDManualState != 0x00) || (ircomm_getEnabledLEDs() == 0x00)) {
	/* If some IR LEDs are currently in manual control mode or if no IR 
	 * LEDs have been selected to be used for the message transmission, 
	 * return false. */
	return false;
    }
	
    /* At this point, we know that the transmission can proceed, so before 
     * doing anything else, (e.g. moving any character to the UART), indicate
     * that we are now actively transmitting. */
    m_txActive = true;
	
    /* Remember whether we should flash the IR LEDS after transmitting the
     * message.  The flash pattern, if requested, is as follows: 
     * 
     * IRLED1: 4 periods of Timer1
     * IRLED2: 8 periods of Timer1
     * IRLED3: 12 periods of Timer1
     * IRLED4: 16 periods of Timer1
     * 
     * This pattern is repeated serveral times and allows the receiver to 
     * determine how it is oriented with respect to the transmitter by sampling
     * its ambient light sensor with period equal to that of Timer1 for 4x 
     * over-sampling. */ 
    m_flashIRLEDsPostTx = flashIRLEDsPostTx;
	
    /* Set the function to call when all characters have been transmitted */
    uart0_setTxCompleteCallback(irmsg_uartTxCompleteCallback);
	
    /* Copy all characters from the irmsg transmit buffer to the UART's
     * transmit buffer.  Interrupts will be used to automatically start
     * sending the characters. */
    i = 0;
    while (i < m_txBufLength) {
	uart0_putchar(m_txBuf[i]);
    }
	
    /* Resetting the transmit buffer length to 0 effectively erases the 
     * transmit buffer. */
    m_txBufLength = 0;
	
    return true;
}

bool irmsg_getRxActive() {
    return g_postRxAmbientLightSamplingActive;
}

uint8_t irmsg_getRxBufLength() {
    return m_rxBufLength;
}

uint8_t irmsg_getRxChar(uint8_t charIndex) {
    if (charIndex < m_rxBufLength) {
	return m_rxBuf[charIndex];
    } else {
	return 0xFF;
    }
}

void irmsg_resetReceiver() {
    m_rxBufFull = false;
    m_rxBufLength = 0;
    g_postRxAmbientLightSamplingStart = false;
    g_postRxAmbientLightSamplingActive = false;
}

bool irmsg_getTxActive() {
    return m_txActive;
}

uint8_t irmsg_getAmbientLightSampleCount() {
    return m_postRxAmbientLightSampleCounter;
}

int16_t irmsg_getAmbientLightSample() {
    int16_t c;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	if (g_postRxAmbientLightSamplingActive) {
	    c = -1;
	} else {
	    c = m_postRxAmbientLightSamples[m_sampleIndex];
	    m_sampleIndex = (m_sampleIndex + 1) % AMBIENT_LIGHT_SAMPLE_COUNT;
	}
    }

    return c;
}

void irmsg_uartTxCompleteCallback() {
    /* This callback is executed after the last character has been sent from
     * the UART's transmit FIFO. */
		
    if (!m_flashIRLEDsPostTx) {
	/* If we have been instructed not to flash the IR LED in a pattern 
	 * which allows the receiver to determine its relative orientation, we
	 * indicate that we are no longer busy transmitting a message */
	m_txActive = false;
	return;
    }
	
    /* Otherwise, if we are going to flash the IR LEDs in a distinctive pattern
     * which the receiver can use to determine its relative orientation, we 
     * disable the 36kHz IR carrier and the UART transmitter circuit, so that 
     * the OR gated connected to all of the IR LEDs' cathodes will pull low. */
    timer0_disableCarrierOutput();
    uart0_disableTx();
	
    /* Set the flag which causes the irmsg_flashIRLEDsPostTxCallback to be 
     * executed each time Timer1 overflows and then start the timer (if it is
     * not already running). */
    g_postTxIRLEDsFlashingStart = true;
    timer1_start();
}

void irmsg_postTxFlashIRLEDsCallback() {
    static uint8_t period = 0;
    static uint8_t cycle = 0;
	
    if (g_postTxIRLEDsFlashingStart && !g_postTxIRLEDsFlashingActive) {
	/* If the start flag is set and the active flag is not, we reset the
	 * counters to 0 to begin a new flash pattern. */
	period = 0;
	cycle = 0;
	/* Set the active flag and clear the start flag */
	g_postTxIRLEDsFlashingActive = true;
	/* Clear the start flag */
	g_postTxIRLEDsFlashingStart = false;
    } else if (!g_postTxIRLEDsFlashingActive) {
	/* Return if neither the start or active flags is set, but before doing
	 * so ensure that the transmit active flag is cleared so that we do
	 * not hang. */
	m_txActive = false;
	return;
    }
	
    if (cycle < IRLED_FLASH_CYCLES) {
	if (period < ((1) * TIMER_PERIODS_IN_SHORTEST_IRLED_FLASH)) {
	    PORT_IRLED1_ANODE |=  (1<<PIN_NUMBER_IRLED1_ANODE);
	    PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	    PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	    PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
	    period++;
	} else if (period < ((1 + 2) * TIMER_PERIODS_IN_SHORTEST_IRLED_FLASH)) {
	    PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	    PORT_IRLED2_ANODE |=  (1<<PIN_NUMBER_IRLED2_ANODE);
	    PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	    PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
	    period++;
	} else if (period < ((1 + 2 + 3) * TIMER_PERIODS_IN_SHORTEST_IRLED_FLASH)) {
	    PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	    PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	    PORT_IRLED3_ANODE |=  (1<<PIN_NUMBER_IRLED3_ANODE);
	    PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
	    period++;
	} else if (period < ((1 + 2 + 3 + 4) * TIMER_PERIODS_IN_SHORTEST_IRLED_FLASH)) {
	    PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	    PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	    PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	    PORT_IRLED4_ANODE |=  (1<<PIN_NUMBER_IRLED4_ANODE);
	    period++;
	} else {
	    /* Reset the period counter to 0 and increment the cycle counter */
	    period = 0;
	    cycle++;
	}
    } else {
	/* Now that we have finished flashing the IR LEDs, drive all of their
	 * anodes to ground to turn them off. */
	PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
	PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
	PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
	PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
		
	/* Additionally, re-enable the UART transmitter, which will pull the 
	 * TXD0 line high by default. With TXD0 high, the output of the OR gate
	 * which drives all of the IR LEDs' cathodes will pull high, thereby 
	 * turning off all of the IR LEDs. */
	uart0_enableTx();		
		
	/* Indicate that we have finished flashing the IR LEDs */
	g_postTxIRLEDsFlashingActive = false;
		
	/* Indicate that the transmission is complete */
	m_txActive = false;
    }
}

void irmsg_uartRxCallback(uint8_t c) {
    /* This callback is executed every time the UART receives a character.   */
	
    if (m_rxBufFull) {
	/* If the message receive buffer is marked full, we return immediately.
	 * It must be emptied with an I2C command before we can accept another
	 * message. */ 
	return;
    }
	
    /* Since we just received a character, reset the counter which we use to 
     * determine what constitutes the end of a message.  Since the transmitter
     * should have the complete messages queued before it starts sending, we
     * interpret even relatively small breaks between characters as the end of
     * one message and the start of another. */
    m_ambientLightSamplePeriodsWithoutNewCharacter = 0;
	
    /* If we are not already sampling the ambient light sensor, we set the 
     * start flag so that we will begin to do so the next time Timer1 
     * overflows. */
    if (!g_postRxAmbientLightSamplingActive) {
	g_postRxAmbientLightSamplingStart = true;
	/* Ensure that the timer is actually running */
	timer1_start();
		
	/* Reset the received message buffer pointer.  */
	m_rxBufLength = 0;
    }
	
    /* Save the received character */
    if (m_rxBufLength < sizeof(m_rxBuf)) {
	m_rxBuf[m_rxBufLength++] = c;
    }
	
    /* Mark the buffer full if we're out of space. */
    if (m_rxBufLength >= sizeof(m_rxBuf)) {
	m_rxBufFull = true;
    }
}

void irmsg_postRxSampleAmbientLightCallback() {
    if (g_postRxAmbientLightSamplingStart && !g_postRxAmbientLightSamplingActive) {
	/* If the start flag is set and the active flag is not, clear the start
	 * flag and set the active flag.  Then, reset the sample counter before
	 * starting an ADC conversion below... */
	g_postRxAmbientLightSamplingStart = false;
	g_postRxAmbientLightSamplingActive = true;
	m_postRxAmbientLightSampleCounter = 0;
    } else if (!g_postRxAmbientLightSamplingActive) {
	/* Otherwise if the active flag is not set, return now. */
	return;
    }
	
    if (m_ambientLightSamplePeriodsWithoutNewCharacter < UINT8_MAX) {
	m_ambientLightSamplePeriodsWithoutNewCharacter++;
    }
	
    /* If too many timer periods have expired since we received the previous
     * character, we mark the receive buffer full. */
    if (m_ambientLightSamplePeriodsWithoutNewCharacter > 4) {
	m_rxBufFull = true;
    }
	
    /* Start an ADC conversion */
    adc_oneSampleInterrupt(irmsg_adcConversionCompleteCallback);
}

void irmsg_adcConversionCompleteCallback(uint16_t result) {
    /* This code is called when the ADC conversion of the ambient light 
     * sensor's output is complete. */
	
    if (!g_postRxAmbientLightSamplingActive) {
	/* If the active flag indicates that we are not supposed to be sampling
	 * right now, return immediately. */
	return;
    }
	
    /* If we have not taken the specified number of samples, save the newest
     * sample. */ 
    if (m_postRxAmbientLightSampleCounter++ < AMBIENT_LIGHT_SAMPLE_COUNT) {
	/* Convert 10-bit result to 8 bits */
	m_postRxAmbientLightSamples[m_postRxAmbientLightSampleCounter++] = (result >> 2);
    }
	
    /* As soon as we have taken all samples, clear the active flag in order to
     * stop sampling.  This flag is checked by the timer1_callback() function 
     * in main.  If it is cleared, a new conversion will not be started. */
    if (m_postRxAmbientLightSampleCounter >= AMBIENT_LIGHT_SAMPLE_COUNT) {
	g_postRxAmbientLightSamplingActive = false;
	/* Once we have taken all of the ambient light samples, we go ahead an
	 * mark the receive buffer full.  Given that repeatedly sampling the 
	 * ambient light sensor likely takes much longer than it does to
	 * receive the message, it is likely that the buffer full flag is 
	 * already set.  Just in case it is not, we set it anyway. */
	m_rxBufFull = true;
    }
}
