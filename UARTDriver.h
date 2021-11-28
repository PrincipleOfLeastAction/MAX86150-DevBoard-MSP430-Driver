/*
 * UARTDriver.h
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */

#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include <msp430.h>
#include <stdint.h>

// UART Defines
#define RX_PIN_BIT                      BIT4
#define TX_PIN_BIT                      BIT3
#define UART_PIN_PORT_ALT_FUNCTION_REG  P3SEL
#define UART_PIN_PORT_DIR_REG           P3DIR

int uartInit(uint32_t baud);

#endif /* UARTDRIVER_H_ */
