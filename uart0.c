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
#include "uart0.h"

#if !defined(UART_RX_BUFFER_SIZE)
#define UART_RX_BUFFER_SIZE 32
#endif

#if !defined(UART_TX_BUFFER_SIZE)
#define UART_TX_BUFFER_SIZE 32
#endif


static volatile uint8_t m_rxBuf[UART_RX_BUFFER_SIZE];
static volatile uint16_t m_rxBufHead = 0;
static volatile uint16_t m_rxBufTail;
static volatile bool m_rxBufEmpty = true;
static volatile bool m_rxBufFull = false;


static volatile uint8_t m_txBuf[UART_RX_BUFFER_SIZE];
static volatile uint16_t m_txBufHead = 0;
static volatile uint16_t m_txBufTail;
static volatile bool m_txBufEmpty = true;
static volatile bool m_txBufFull = false;

void uart0_init() {
	/* Disable the transmitter, receiver, and all interrupts.  Clear the UCSZ02
	 * bit to disable 9-bit data mode.  */
	UCSR0B = (0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02);
	
	/* Clear the transmit complete interrupt flag, disable double-speed mode, 
	 * disable multi-processor communication mode. */
	UCSR0A = (1<<TXC0) | (0<<U2X0) | (0<<MPCM0);

	/* Asynchronous USART mode, odd parity, 2 stop bit, 8 data bits */
	UCSR0C = (0<<UMSEL00) | (3<<UPM00) | (1<<USBS0) | (3<<UCSZ00);
	
	/* Disable the RX start frame interrupt, clear the interrupt flag, and 
	 * disable the start frame detector. */
	UCSR0D = (0<<RXSIE0) | (1<<RXS0) | (0<<SFDE0);

	/* Make TXD0 (PB0) an output */
	DDRB |= (1<<DDRB0);
	
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
	}

	/* Enable receiver, transmitter, and receive interrupt.  We do not enable
	 * data register empty interrupt (UDRIE0) because it will trigger 
	 * continuously until we load data into the register. */
	UCSR0B |= (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
}

bool uart0_putchar(uint8_t c) {
	bool success;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (m_txBufFull) {
			/* If the transmit buffer is full, return failure. */
			success = false;
		} else {
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
			
			/* Enable the data register empty interrupt, (if it is not alraedy 
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
		if (m_rxBufEmpty) {
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

uint8_t uart0_getTxBufferCount() {
	uint8_t size;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (m_txBufEmpty) {
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

uint8_t uart0_getRxBufferCount() {
	uint8_t size;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (m_rxBufEmpty) {
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

ISR(USART0_UDRE_vect) {
	/* This interrupt is called when the USART needs to be supplied with more
	 * data to transmit. */
	
	/* If the transmit buffer is empty, disable this interrupt and return.  If
	 * we leave the interrupt enabled, it will continue to trigger indefinitely. 
	 */
	if (m_txBufEmpty) {
		UCSR0B &= ~(1<<UDRIE0);
		return;
	}
	
	/* More oldest byte from the transmit FIFO into the USART data register */
	UDR0 = m_txBuf[m_txBufTail];
	
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

ISR(USART0_RX_vect) {
	uint8_t c;
	bool err;
	
	/* Check for framing, over-run, and parity errors.  These bytes are only 
	 * valid until the data register is read */
	err = ((UCSR0A & ((1<<FE0) | (1<<DOR0) | (1<<UPE0))) != 0x00);
	
	/* Read the data register */
	c = UDR0;
	
	/* If the receive buffer is full or there was an error with the received 
	 * byte, return without placing the byte into the receive FIFO. */
	if (m_rxBufFull || err) {
		return;
	}
	
	/* Place the new byte into the FIFO */
	m_rxBuf[m_rxBufHead] = c;
	
	/* Increment the head index, wrapping around back to 0 as necessary. */
	m_rxBufHead = (m_rxBufHead + 1) % UART_RX_BUFFER_SIZE;
	
	/* We just added a byte to the FIFO, so it cannot be empty. */
	m_rxBufEmpty = false;
	
	/* If the head pointer has caught up with the tail, the FIFO is full. */
	if (m_rxBufHead == m_rxBufTail) {
		m_rxBufFull = true;
	}
}