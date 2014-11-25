/*
 * FaceBoard.c
 *
 * Created: 11/5/2014 2:47:03 PM
 *  Author: kwgilpin
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdbool.h>

#include "defines.h"
#include "timer0.h"
#include "uart0.h"
#include "ambient.h"
#include "usiTwiSlave.h"

static uint8_t	onI2CReadFromRegister(uint8_t reg);
static void onI2CWriteToRegister(uint8_t reg, uint8_t value);

int main(void) {
	/* Make the pins driving the RGB LEDs output */
	DDR_LED_RED |= (1<<PIN_NUMBER_LED_RED);
	DDR_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
	DDR_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
	
	/* Make the pins driving the IR LED anodes outputs.  We drive these pins
	 * high to turn on particular LEDs. */
	DDR_IRLED0_ANODE |= (1<<PIN_NUMBER_IRLED0_ANODE);
	DDR_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
	DDR_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
	DDR_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
	
	/* Disable the digital input on the pin used for the ambient light sensor.
	 */
	DIDR1 |= (1<<ADC8D);
	
	/* Configure Timer0 to generate a 36kHz carrier wave for the IR 
	 * communication system. */
	timer0_init(36);
	
	/* Initialize the USART which we use to modulate the IR LEDs */
	uart0_init();
	
	usiTwiSlaveInit(I2C_SLAVE_ADDR, onI2CReadFromRegister, onI2CWriteToRegister);
	
	/* Enable global interrupts */
	sei();
	
    while(1) {
		;
    }
}


uint8_t	onI2CReadFromRegister(uint8_t reg) {
	uint8_t value = 0x00;
	
	switch (reg) {
		case I2C_REGISTER_ADDR_LEDS:
			if (PIN_LED_RED & (1<<PIN_NUMBER_LED_RED)) {
				value |= 0x01;
			}
			
			if (PIN_LED_GREEN & (1<<PIN_NUMBER_LED_GREEN)) {
				value |= 0x02;
			}
			
			if (PIN_LED_BLUE & (1<<PIN_NUMBER_LED_BLUE)) {
				value |= 0x04;
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_CARRIER:
			if (timer0_getCarrierStatus()) {
				value = 0x01;
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_LEDS:
			if (PIN_IRLED0_ANODE & (1<<PIN_NUMBER_IRLED0_ANODE)) {
				value |= 0x01;
			}
			
			if (PIN_IRLED1_ANODE & (1<<PIN_NUMBER_IRLED1_ANODE)) {
				value |= 0x02;
			}
						
			if (PIN_IRLED2_ANODE & (1<<PIN_NUMBER_IRLED2_ANODE)) {
				value |= 0x04;
			}

			if (PIN_IRLED3_ANODE & (1<<PIN_NUMBER_IRLED3_ANODE)) {
				value |= 0x08;
			}									
			break;
			
		case I2C_REGISTER_ADDR_AMBIENT_LIGHT:
			value = ambient_get8Bit();
			break;
			
		case I2C_REGISTER_ADDR_RX_BUF:
			value = uart0_getchar_min();
			break;
			
		case I2C_REGISTER_ADDR_TX_COUNT:
			value = uart0_getTxBufferCount();
			break;
		
		case I2C_REGISTER_ADDR_RX_COUNT:
			value = uart0_getRxBufferCount();
			break;
			
		default:
			return 0x00;
	}
	
	return value;
}

void onI2CWriteToRegister(uint8_t reg, uint8_t value) {
	switch (reg) {
		case I2C_REGISTER_ADDR_LEDS:
			if (value & 0x01) {
				PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
			} else {
				PORT_LED_RED &= ~(1<<PIN_NUMBER_LED_RED);
			}
			
			if (value & 0x02) {
				PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
			} else {
				PORT_LED_GREEN &= ~(1<<PIN_NUMBER_LED_GREEN);
			}
			
			if (value & 0x04) {
				PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
			} else {
				PORT_LED_BLUE &= ~(1<<PIN_NUMBER_LED_BLUE);
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_CARRIER:
			if (value & 0x01) {
				timer0_enableCarrierOutput();
			} else {
				timer0_disableCarrierOutput();
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_LEDS:
			if (value & 0x01) {
				PORT_IRLED0_ANODE |= (1<<PIN_NUMBER_IRLED0_ANODE);
			} else {
				PORT_IRLED0_ANODE &= ~(1<<PIN_NUMBER_IRLED0_ANODE);
			}

			if (value & 0x02) {
				PORT_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
			} else {
				PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
			}
			
			if (value & 0x04) {
				PORT_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
			} else {
				PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
			}
			
			if (value & 0x08) {
				PORT_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
			} else {
				PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
			}
			break;
			
		case I2C_REGISTER_ADDR_TX_BUF:
			uart0_putchar(value);
			break;
		
		default:
			break;
	}
}