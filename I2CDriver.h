/*
 * I2CDriver.h
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */

#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_

#include <msp430.h>
#include <stdint.h>

// Better register defines to improve readability.
#define SDA_PIN_BIT                     BIT1
#define SCL_PIN_BIT                     BIT2
#define I2C_PIN_PORT_ALT_FUNCTION_REG   P4SEL
#define CONTROL_REG_0                   UCB1CTL0
#define CONTROL_REG_1                   UCB1CTL1
#define RESET_BIT                       UCSWRST
#define MASTER_BIT                      UCMST
#define I2C_MODE_BIT                    UCMODE_3
#define SYNCHRONOUS_MODE_BIT            UCSYNC
#define SMCLK_CLOCK_SOURCE_BIT          UCSSEL__SMCLK
#define MASTER_TRANSMIT_MODE_BIT        UCTR
#define START_CONDITION_BIT             UCTXSTT
#define STOP_CONDITION_BIT              UCTXSTP
#define BIT_RATE_CONTROL_REG_0          UCB1BR0
#define BIT_RATE_CONTROL_REG_1          UCB1BR1
#define USCI_INTERRUPT_ENABLE_REG       UCB1IE
#define NOT_ACK_INTERRUPT_ENABLE        UCNACKIE
#define TX_INTERRUPT_ENABLE             UCTXIE
#define RX_INTERRUPT_ENABLE             UCRXIE
#define SLAVE_ADDR_REG                  UCB1I2CSA
#define TRANSMIT_BUFFER_REG             UCB1TXBUF
#define RECEIVE_BUFFER_REG              UCB1RXBUF

void I2C_Init(void);
void masterWriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t writeData, uint8_t dataCount);
void masterReadReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t numReads);
__interrupt void USCI_B1_ISR(void);

#endif /* I2CDRIVER_H_ */
