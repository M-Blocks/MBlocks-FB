/*
 * uart0.c
 *
 * Created: 11/7/2014 9:03:51 AM
 *  Author: kwgilpin
 */ 

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "defines.h"
#include "timer0.h"
#include "ircomm.h"
#include "uart0.h"

#if !defined(UART_RX_BUFFER_SIZE)
#define UART_RX_BUFFER_SIZE 128
#endif

#if !defined(UART_TX_BUFFER_SIZE)
#define UART_TX_BUFFER_SIZE 128
#endif


static volatile uint8_t m_rxActualChar;
static volatile bool m_rxActualCharValid;
static volatile uint8_t m_rxBuf[UART_RX_BUFFER_SIZE];
static volatile uint16_t m_rxBufHead = 0;
static volatile uint16_t m_rxBufTail;
static volatile bool m_rxBufEmpty = true;
static volatile bool m_rxBufFull = false;

static volatile uint8_t m_txActual0OrInverse1;
static volatile uint8_t m_txBuf[UART_RX_BUFFER_SIZE];
static volatile uint16_t m_txBufHead = 0;
static volatile uint16_t m_txBufTail;
static volatile bool m_txBufEmpty = true;
static volatile bool m_txBufFull = false;

static bool m_initialized = false;

static void (*txCompleteCallback)(void) = NULL;
static void (*rxCompleteCallback)(uint8_t) = NULL;

void uart0_init() {
	/* Disable the transmitter, receiver, and all interrupts.  Set the UCSZ02
	 * bit to enable 9-bit data mode.  We'll use the 9th bit to indicate 
	 * whether a character is the actual/first (0) or inverse/second (1) in
	 * each pair or bytes we transmit. */
	UCSR0B = (0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (1<<UCSZ02);
	
	/* Clear the transmit complete interrupt flag, disable double-speed mode, 
	 * disable multi-processor communication mode. */
	UCSR0A = (1<<TXC0) | (0<<U2X0) | (0<<MPCM0);

	/* Asynchronous USART mode, odd parity, 2 stop bit, 8 data bits */
	UCSR0C = (0<<UMSEL00) | (3<<UPM00) | (1<<USBS0) | (3<<UCSZ00);
	
	/* Disable the RX start frame interrupt, clear the interrupt flag, and 
	 * disable the start frame detector. */
	UCSR0D = (0<<RXSIE0) | (1<<RXS0) | (0<<SFDE0);

	/* Make TXD0 (PB0) an output driving low.  As soon as the transmitter is
	 * enabled, it will pull the pin high automatically.  When the transmitter
	 * is disabled (as it is when using the IR LEDs in manual mode) we want
	 * the TXD0 pin to pull low so as to not preclude the OR gate which drives
	 * the IR LEDs' cathodes from going low. */
	DDRB |= (1<<DDRB0);
	PORTB &= ~(1<<PORTB0);
	
	/* Make RXD0 (PA7) an input */
	DDRA &= ~(1<<DDRA7);
	
	/* Assuming we're using a 8MHz system clock, set baud rate to 2400 bps. */
	UBRR0 = 207;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* Reset the buffer pointers */
		m_rxBufHead = m_rxBufTail = 0;
		m_rxBufEmpty = true;
		m_rxBufFull = false;

		m_txBufHead = m_txBufTail = 0;
		m_txBufEmpty = true;
		m_txBufFull = false;
		
		m_txActual0OrInverse1 = 0;
		m_rxActualCharValid = false;
	}

	/* Enable receiver, transmitter, and receive interrupt.  We do not enable
	 * data register empty interrupt (UDRIE0) because it will trigger 
	 * continuously until we load data into the register. */
	UCSR0B |= (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	
	m_initialized = true;
}

void uart0_deinit() {
	/* Disable the transmitter, receiver, and all interrupts.  */
	UCSR0B = (0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0);
	
	/* Drive TXD0 (PB0) to ground */
	PORTB &= ~(1<<PORTB0);
	
	m_initialized = false;
}

bool uart0_isInitialized() {
	bool init = false;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		init = m_initialized;
	}
	
	return init;
}

void uart0_enableTx() {
	if (!m_initialized) {
		return;
	}
	
	/* Enable the transmitter by setting the TXEN0 bit */
	UCSR0B |= (1<<TXEN0);
}

void uart0_disableTx() {
	/* Disable the transmitter by clearing just the TXEN0 bit */
	UCSR0B &= ~(1<<TXEN0);
	
	/* Drive TXD0 (PB0) to ground */
	PORTB &= ~(1<<PORTB0);	
}

bool uart0_isTxEnabled() {
	/* Check the TXEN0 bit to determine whether the transmitter is enabled. */
	return ((UCSR0B & (1<<TXEN0)) ? true : false);
}

bool uart0_setTxCompleteCallback(void (*callback)(void)) {
	txCompleteCallback = callback;
	return true;
}

bool uart0_setRxCompleteCallback(void (*callback)(uint8_t)) {
	rxCompleteCallback = callback;
	return true;
}

bool uart0_putchar(uint8_t c) {
	bool success;
		
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (!m_initialized) {
			return false;
		} else if (m_txBufFull) {
			/* If the transmit buffer is full, return failure. */
			success = false;
		} else {
			/* If the transmit buffer is empty, we need to enable the 36kHz 
			 * carrier and drive the anodes of the IR LEDs we'll be using for
			 * transmission to VCC.  We only do this if the transmit buffer
			 * is empty to avoid wasting time every time a character is 
			 * added to the transmit buffer. */
			if (m_txBufEmpty) {
				/* Enable the 36kHz carrier */
				timer0_enableCarrierOutput();
			
				/* Drive the anodes for the IR LEDs that have been selected for
				 * transmission to VCC. */
				ircomm_activateEnabledLEDs();
			}
			
			/* Place the new character in the transmit buffer */
			m_txBuf[m_txBufHead] = c;
			
			/* Increment the head index, wrapping around the end of the buffer
			 * back to the beginning as necessary. */
			m_txBufHead = (m_txBufHead + 1) % UART_TX_BUFFER_SIZE;
			
			/* We just placed a character in the transmit buffer, so it cannot
			 * be empty. */
			m_txBufEmpty = false;
			
			/* If the head index has overtaken the tail, the transmit FIFO is
			 * full. */
			if (m_txBufHead == m_txBufFull) {
				m_txBufFull = true;
			}
						
			/* Enable the data register empty interrupt, (if it is not already 
			 * enabled).  When the USART has nothing to transmit, this
			 * interrupt will trigger, and the ISR will transfer a character 
			 * from the transmit FIFO to the data register. */
			UCSR0B |= (1<<UDRIE0);
			
			/* Indicate success to the caller */
			success = true;
		}
	}
	
	return success;
}

int16_t uart0_getchar() {
	int16_t c;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (!m_initialized) {
			c = _FDEV_ERR;
		} else if (m_rxBufEmpty) {
			/* Return an error (a negative number) if there are no characters 
			 * waiting in the receive FIFO. */
			c = _FDEV_EOF;
		} else {
			/* The tail index points to the oldest character in the FIFO */
			c = m_rxBuf[m_rxBufTail];
			
			/* Increment the tail, being sure to wrap back to 0 at the end of
			 * the buffer. */
			m_rxBufTail = (m_rxBufTail + 1) % UART_RX_BUFFER_SIZE;
			
			/* We just removed a character, so the buffer cannot be full. */
			m_rxBufFull = false;
			
			/* If the tail has caught up to the head, the buffer is now 
			 * empty. */
			if (m_rxBufTail == m_rxBufHead) {
				m_rxBufEmpty = true;
			}
		}
	}
	
	return c;
}

uint8_t uart0_getchar_min() {
	int16_t c;
	
	c = uart0_getchar();
	
	if (c < 0) {
		c = '\0';
	}
	
	return (uint8_t)c;
}

bool uart0_getTxBufferEmpty() {
	bool empty;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		empty = m_txBufEmpty;
	}
	
	return empty;
}

uint8_t uart0_getTxBufferAvailableCount() {
	uint8_t size = 0;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (!m_initialized) {
			size = 0;
		} else if (m_txBufEmpty) {
			size = UART_TX_BUFFER_SIZE;
		} else if (m_txBufFull) {
			size = 0;
		} else if (m_txBufHead > m_txBufTail) {
			size = UART_TX_BUFFER_SIZE - (m_txBufHead - m_txBufTail);
		} else {
			size = m_txBufTail - m_txBufHead;
		}
	}
	
	return size;
}

uint8_t uart0_getTxBufferConsumedCount() {
	uint8_t size = 0;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (!m_initialized) {
			size = 0;
		} else if (m_txBufEmpty) {
			size = 0;
		} else if (m_txBufFull) {
			size = UART_TX_BUFFER_SIZE;
		} else if (m_txBufHead > m_txBufTail) {
			size = m_txBufHead - m_txBufTail;
		} else {
			size = UART_TX_BUFFER_SIZE - (m_txBufTail - m_txBufHead);
		}
	}
	
	return size;
}

uint8_t uart0_getRxBufferConsumedCount() {
	uint8_t size = 0;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (!m_initialized) {
			size = 0;
		} else if (m_rxBufEmpty) {
			size = 0;
		} else if (m_rxBufFull) {
			size = UART_RX_BUFFER_SIZE;
		} else if (m_rxBufHead > m_rxBufTail) {
			size = m_rxBufHead - m_rxBufTail;
		} else {
			size = UART_RX_BUFFER_SIZE - (m_rxBufTail - m_rxBufHead);
		}
	}
	
	return size;
}

void uart0_flushRxBuffer() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m_rxBufEmpty = true;
		m_rxBufFull = false;
		m_rxBufHead = m_rxBufTail = 0;
	}
}

ISR(USART0_UDRE_vect) {
	/* This interrupt is called when the USART needs to be supplied with more
	 * data to transmit. */
	
	if (m_txActual0OrInverse1 == 0) {
		/* If we are supposed to be transmitting the actual (not inverted) 
		 * character, we first check whether the transmit FIFO is empty.  If it
		 * is, we disable this interrupt's enable flag and then return. */
		if (m_txBufEmpty) {
			UCSR0B &= ~(1<<UDRIE0);
			/* Enable the transmit complete interrupt in which we will disable
			 * the carrier wave (which we still happen to need while the last
			 * byte is shifted out. */
			UCSR0B |= (1<<TXCIE0);
			return;
		}
	
		/* Otherwise, we clear the data 9th bit (to signify that this is non-
		 * inverted data) and then copy the data from the FIFO to the transmit
		 * data register.  Note that we do not modify the contents of the FIFO
		 * here because we still need to transmit the inverse of the same 
		 * character that we just loaded into the transmit buffer. */
		UCSR0B &= ~(1<<TXB80);
		UDR0 = m_txBuf[m_txBufTail];
		
		/* Indicate that the next time this ISR is executed that we should 
		 * transmit the inverse character. */
		m_txActual0OrInverse1 = 1;
	} else {
		/* If we are supposed to be transmitting the inverse of the character
		 * at the tail of the transmit FIFO, we set the 9th data bit and then
		 * load the inverse of the character into the transmit data buffer. */
		UCSR0B |= (1<<TXB80);
		UDR0 = ~m_txBuf[m_txBufTail];
	
		/* Indicate that the next time this ISR is executed that we should 
		 * transmit an actual (non-inverted) character. */
		m_txActual0OrInverse1 = 0;
	
		/* Now that we have transmitted the inverse character, we can remove 
		 * the character from the transmit FIFO. */
	
		/* Increment the tail index */
		m_txBufTail = (m_txBufTail + 1) % UART_TX_BUFFER_SIZE;
	
		/* We just pulled a byte out of the transmit FIFO, so it cannot be full. */
		m_txBufFull = false;
	
		/* If the tail has caught up to the head index, the buffer is now empty. 
		 * Set the empty flag so that after the byte we just loaded into the USART
		 * data buffer is sent, and this ISR is executed again, the ISR will be
		 * disabled. */
		if (m_txBufTail == m_txBufHead) {
			m_txBufEmpty = true;
		}
	}
}

ISR(USART0_TXC_vect) {
	/* Deactivate the anodes of all the IR LEDs */
	ircomm_deactivateAllLEDs();	
	
	/* Disable the carrier wave */
	timer0_disableCarrierOutput();
	
	/* Disable this interrupt */
	UCSR0B &= ~(1<<TXCIE0);
	
	/* If it exists, execute the transmit complete callback */
	if (txCompleteCallback != NULL) {
		txCompleteCallback();
	}
}

ISR(USART0_RX_vect) {
	volatile uint8_t c;
	volatile uint8_t rxActual0OrInverse1;
	bool err;
	
	/* Check for framing, over-run, and parity errors.  These bytes are only 
	 * valid until the data register is read */
	err = ((UCSR0A & ((1<<FE0) | (1<<DOR0) | (1<<UPE0))) != 0x00);
	
	/* Read the 9th data bit to determine whether the character is inverted. */
	rxActual0OrInverse1 = (UCSR0B & (1<<RXB80)) ? 1 : 0;
	
	/* Read the data register */
	c = UDR0;
	
	/* If there was an error with the received byte, return. */
	if (err) {
		/* Since we are discarding a character, we mark the any previously 
		 * received character invalid because we do not want to accidently
		 * match it against any inverse byte which we know cannot be its
		 * intended inverse. */
		m_rxActualCharValid = false;
		return;
	}
	
	/* To help filter noisy output from the IR demodulator, we expect incoming
	 * bytes to arrive in pairs which are the binary inverses of each other,
	 * with the non-inverted version arriving first.  When it does, we store it
	 * in the m_rxActualChar variable and set the m_rxActualCharValid flag. */
	if (rxActual0OrInverse1 == 0) {
		/* If we just received what we believe is a non-inverted character, we
		 * overwrite whatever we have already stored (if anything) and set the
		 * flag indicating that we are now waiting for the inverse to arrive. */
		m_rxActualChar = c;
		m_rxActualCharValid = true;
		/* Until we receive the inverse, we do not move the character to our
		 * receive buffer. */
		return;
	} else if ((rxActual0OrInverse1 == 1) && (m_rxActualCharValid) && ((m_rxActualChar ^ c) == 0xFF)) {
		/* If we received what we believe is an inverted character, and we have
		 * previously received and non-inverted character, and one is the
		 * binary inverse of the other, we have successfully received one byte
		 * of data. */
		
#if (1)
			/* Toggle the blue LED with each character received. */
			PORT_LED_BLUE ^= (1<<PIN_NUMBER_LED_BLUE);
#endif
		
		/* Return if the receive buffer is full. */
		if (m_rxBufFull) {
			return;
		}
		
		/* Place the non-inverted character into the FIFO */
		m_rxBuf[m_rxBufHead] = m_rxActualChar;
	
		/* Increment the head index, wrapping around back to 0 as necessary. */
		m_rxBufHead = (m_rxBufHead + 1) % UART_RX_BUFFER_SIZE;
	
		/* We just added a byte to the FIFO, so it cannot be empty. */
		m_rxBufEmpty = false;
	
		/* If the head pointer has caught up with the tail, the FIFO is full. */
		if (m_rxBufHead == m_rxBufTail) {
			m_rxBufFull = true;
		}
		
		/* If it exists, execute the receive callback */
		if (rxCompleteCallback != NULL) {
			rxCompleteCallback(m_rxActualChar);
		}
	} else {
		m_rxActualCharValid = false;
	}
}