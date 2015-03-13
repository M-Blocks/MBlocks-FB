/*
 * uart0.h
 *
 * Created: 11/7/2014 9:04:04 AM
 *  Author: kwgilpin
 */ 


#ifndef UART0_H_
#define UART0_H_

#include <stdint.h>
#include <stdbool.h>

void uart0_init(void);
void uart0_deinit(void);
bool uart0_isInitialized(void);

bool uart0_putchar(uint8_t c);
int16_t uart0_getchar(void);
uint8_t uart0_getchar_min(void);
uint8_t uart0_getTxBufferCount(void);
uint8_t uart0_getRxBufferCount(void);
void uart0_flushRxBuffer(void);


#endif /* UART0_H_ */