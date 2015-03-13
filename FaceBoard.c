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

#include "defines.h"
#include "timer0.h"
#include "uart0.h"
#include "ambient.h"
#include "usiTwiSlave.h"

static bool m_sleepRequested = false;
static uint8_t m_txTestFace = 0x00;

static uint8_t	onI2CReadFromRegister(uint8_t reg);
static void onI2CWriteToRegister(uint8_t reg, uint8_t value);
static void sleep(void);

#if(0)
#ifndef DEBUG
FUSES = {
	.low = (FUSE_SUT_CKSEL4 & FUSE_SUT_CKSEL1), // internal 8MHz oscillator (6CK/16CK+16ms), no CKDIV8, no CKOUT
	.high = (FUSE_SPIEN & FUSE_BODLEVEL0), // 1.8V brownout voltage, no watchdog, no EE save, no SPI enabled, no debugWire, reset enabled
	.extended = (FUSE_BODPD1 & FUSE_BODACT0) // BOD enabled in active mode, sampled in power-down, no self-programming 
};

LOCKBITS = (LOCKBITS_DEFAULT);
#endif
#endif

int main(void) {
	/* Make the pins driving the RGB LED's cathodes outputs driving high (to 
	 * keep the LEDs off).  */
	DDR_LED_RED |= (1<<PIN_NUMBER_LED_RED);
	PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
	
	DDR_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
	PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
	
	DDR_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
	PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
	
	/* Make the pins driving the RGB LED's anodes outputs driving high */
	DDR_LED_ANODE1 |= (1<<PIN_NUMBER_LED_ANODE1);
	PORT_LED_ANODE1 |= (1<<PIN_NUMBER_LED_ANODE1);
	
	DDR_LED_ANODE2 |= (1<<PIN_NUMBER_LED_ANODE2);
	PORT_LED_ANODE2 |= (1<<PIN_NUMBER_LED_ANODE2);
	
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
	
	/* Disable power to unused peripherals */
	PRR = (1<<PRTWI) | (1<<PRTIM1) | (1<<PRUSART1);
	
	/* Configure Timer0 to generate a 36kHz carrier wave for the IR 
	 * communication system. */
	timer0_init(36);
	
	/* Initialize the USART which we use to modulate the IR LEDs */
	uart0_init();
	
	usiTwiSlaveInit(I2C_SLAVE_ADDR, onI2CReadFromRegister, onI2CWriteToRegister);
	
	/* Enable global interrupts */
	sei();
	
	
#if (1)	
	char c = 'A';

    while(1) {
		_delay_ms(400);
		
		if (((m_txTestFace & 0x0F) != 0x00) && (uart0_isInitialized())) {
			if (m_txTestFace & 0x01) {
				PORT_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
				} else {
				PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
			}
			
			if (m_txTestFace & 0x02) {
				PORT_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
				} else {
				PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
			}
			
			if (m_txTestFace & 0x04) {
				PORT_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
				} else {
				PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
			}
			
			if (m_txTestFace & 0x08) {
				PORT_IRLED4_ANODE |= (1<<PIN_NUMBER_IRLED4_ANODE);
				} else {
				PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
			}
			
			uart0_putchar(c);
		
			/* Flash the green LED */
			PORT_LED_GREEN &= ~(1<<PIN_NUMBER_LED_GREEN);
			_delay_ms(100);
			PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
		
			if (c < 'Z') {
				c++;
			} else {
				c = 'A';
			}
		}
		
		if (m_sleepRequested) {
			sleep();
			m_sleepRequested = false;
		}
    }
#elif (0)
	/* Enable the IR receiver */
	PORT_SENSOREN |= (1<<PIN_NUMBER_SENSOREN);
#endif

	while(1) {}
}


uint8_t	onI2CReadFromRegister(uint8_t reg) {
	uint8_t value = 0x00;
	
	switch (reg) {
		case I2C_REGISTER_ADDR_LEDS:
			if (!(PIN_LED_RED & (1<<PIN_NUMBER_LED_RED))) {
				value |= 0x01;
			}
			
			if (!(PIN_LED_GREEN & (1<<PIN_NUMBER_LED_GREEN))) {
				value |= 0x02;
			}
			
			if (!(PIN_LED_BLUE & (1<<PIN_NUMBER_LED_BLUE))) {
				value |= 0x04;
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_CARRIER:
			value = timer0_getCarrierStatus();
			break;
			
		case I2C_REGISTER_ADDR_IR_LEDS:
			if (PIN_IRLED1_ANODE & (1<<PIN_NUMBER_IRLED1_ANODE)) {
				value |= 0x01;
			}
			
			if (PIN_IRLED2_ANODE & (1<<PIN_NUMBER_IRLED2_ANODE)) {
				value |= 0x02;
			}
						
			if (PIN_IRLED3_ANODE & (1<<PIN_NUMBER_IRLED3_ANODE)) {
				value |= 0x04;
			}

			if (PIN_IRLED4_ANODE & (1<<PIN_NUMBER_IRLED4_ANODE)) {
				value |= 0x08;
			}									
			break;
			
		case I2C_REGISTER_ADDR_AMBIENT_LIGHT:
			value = ambient_get8Bit();
			break;
			
		case I2C_REGISTER_ADDR_SENSOREN:
			value = (PIN_SENSOREN & (1<<PIN_NUMBER_SENSOREN)) ? 1 : 0;
			break;
			
		case I2C_REGISTER_ADDR_RX_BUF:
			value = uart0_getchar_min();
			break;
			
		case I2C_REGISTER_ADDR_TX_COUNT:
			value = uart0_getTxBufferCount();
			break;
			
		case I2C_REGISTER_ADDR_TX_ENABLE:
			value = uart0_isInitialized();
			break;
		
		case I2C_REGISTER_ADDR_RX_COUNT:
			value = uart0_getRxBufferCount();
			break;
			
		case I2C_REGISTER_ADDR_TX_TEST:
			value = m_txTestFace;
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
				PORT_LED_RED &= ~(1<<PIN_NUMBER_LED_RED);
			} else {
				PORT_LED_RED |= (1<<PIN_NUMBER_LED_RED);
			}
			
			if (value & 0x02) {
				PORT_LED_GREEN &= ~(1<<PIN_NUMBER_LED_GREEN);
			} else {
				PORT_LED_GREEN |= (1<<PIN_NUMBER_LED_GREEN);
			}
			
			if (value & 0x04) {
				PORT_LED_BLUE &= ~(1<<PIN_NUMBER_LED_BLUE);
			} else {
				PORT_LED_BLUE |= (1<<PIN_NUMBER_LED_BLUE);
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_CARRIER:
			if (value == 0x00) {
				timer0_disableCarrierOutput();
			} else if (value == 0x01) {
				timer0_enableCarrierOutput();
			}
			break;
			
		case I2C_REGISTER_ADDR_IR_LEDS:
			if (value & 0x01) {
				PORT_IRLED1_ANODE |= (1<<PIN_NUMBER_IRLED1_ANODE);
			} else {
				PORT_IRLED1_ANODE &= ~(1<<PIN_NUMBER_IRLED1_ANODE);
			}
		
			if (value & 0x02) {
				PORT_IRLED2_ANODE |= (1<<PIN_NUMBER_IRLED2_ANODE);
			} else {
				PORT_IRLED2_ANODE &= ~(1<<PIN_NUMBER_IRLED2_ANODE);
			}
			
			if (value & 0x04) {
				PORT_IRLED3_ANODE |= (1<<PIN_NUMBER_IRLED3_ANODE);
			} else {
				PORT_IRLED3_ANODE &= ~(1<<PIN_NUMBER_IRLED3_ANODE);
			}
			
			if (value & 0x08) {
				PORT_IRLED4_ANODE |= (1<<PIN_NUMBER_IRLED4_ANODE);
			} else {
				PORT_IRLED4_ANODE &= ~(1<<PIN_NUMBER_IRLED4_ANODE);
			}			
			break;
			
		
		case I2C_REGISTER_ADDR_SENSOREN:
			if (value & 0x01) {
				PORT_SENSOREN |= (1<<PIN_NUMBER_SENSOREN);
			} else {
				PORT_SENSOREN &= ~(1<<PIN_NUMBER_SENSOREN);
			}
			break;	
		
		case I2C_REGISTER_ADDR_TX_BUF:
			uart0_putchar(value);
			break;
			
		case I2C_REGISTER_ADDR_TX_ENABLE:
			if ((value == 0x01) && (!uart0_isInitialized())){
				uart0_init();
			} else if (value == 0x00) {
				uart0_deinit();
			}
			break;
			
		case I2C_REGISTER_ADDR_RX_FLUSH:
			uart0_flushRxBuffer();
			break;
		
		case I2C_REGISTER_ADDR_SLEEP:
			m_sleepRequested = true;
			break;
		
		case I2C_REGISTER_ADDR_TX_TEST:
			m_txTestFace = value & 0x0F;
			break;
		
		default:
			break;
	}
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
	
	/* Restore power to and re-initialize the UART */
	PRR &= ~(1<<PRUSART0);
	uart0_init();
	
	/* Restore power to and re-initialize Timer0 */
	PRR &= ~(1<<PRTIM0);
	timer0_init(36);
	
	/* Restore power to the ADC */
	PRR &= ~(1<<PRADC);
}