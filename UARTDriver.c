/*
 * UARTDriver.c
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */

#include <msp430.h>
#include <stdint.h>
#include "BCUart.h"
#include "usci_a_uart.h"
#include "UARTDriver.h"

int uartInit(uint32_t baud) {
    // Configure the A0TX and A0RX pin in UART mode.
    UART_PIN_PORT_ALT_FUNCTION_REG |= TX_PIN_BIT;
//    UART_PIN_PORT_DIR_REG &= ~TX_PIN_BIT;


    // Use this tool to calculate baud rate numbers for your own board.
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
//    param.clockPrescalar = 26;
    if (baud == 9600) {
        param.clockPrescalar = 416;
        param.firstModReg  = 0;
        param.secondModReg = 6;
    } else {
        // See family guide pg 952
        // Values for oversampling = 0
    //    param.clockPrescalar = 34;
    //    param.firstModReg  = 0;
    //    param.secondModReg = 6;
        // Values for oversampling = 1
        param.clockPrescalar = 2;
        param.firstModReg  = 2;
        param.secondModReg = 3;
    }

//    param.overSampling = 0;
    param.overSampling = 1;
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;

    if (STATUS_FAIL == USCI_A_UART_init(USCI_A0_BASE, &param)) {
        return STATUS_FAIL;
    }

    USCI_A_UART_enable(USCI_A0_BASE);
    return 1;
}
