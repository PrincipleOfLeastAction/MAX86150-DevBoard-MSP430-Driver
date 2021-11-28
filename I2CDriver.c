/*
 * I2CDriver.c
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */

#include <msp430.h>
#include <stdint.h>
#include "I2CDriver.h"
#include "settings.h"

typedef enum I2C_State_enum {
    IDLE = 0,
    TRANSMITTING = 2,
    RECEIVE_SEND_REG = 4,
    RESTART = 6,
    READING_DATA = 8,
    STOPPING = 10
} I2C_State;

// Set initial state of our program.
I2C_State state = IDLE;

/////////////////////// General I2C stuff below //////////////////////////
void I2C_Init() {
    // Set GPIO function
    I2C_PIN_PORT_ALT_FUNCTION_REG |= SDA_PIN_BIT + SCL_PIN_BIT;

    // USCI reset.
    CONTROL_REG_1 |= RESET_BIT;
    // Initialise Universal Serial Communication Interface with master mode (UCMST),
    // I2C mode (UCMODE_3) and synchronous mode (UCSYNC). See p.1016 MSPx5xx family
    // user guide and p.1004.
    CONTROL_REG_0 = MASTER_BIT + I2C_MODE_BIT + SYNCHRONOUS_MODE_BIT;

    // Setting clock speeds
    CONTROL_REG_1 = SMCLK_CLOCK_SOURCE_BIT + RESET_BIT;
    BIT_RATE_CONTROL_REG_0 = 40;
    BIT_RATE_CONTROL_REG_1 = 0;

    // Clear the reset.
    CONTROL_REG_1 &= ~RESET_BIT;

    // Turn NACK interrupt on for when we send slave address to catch
    // any NACK.
    USCI_INTERRUPT_ENABLE_REG |= NOT_ACK_INTERRUPT_ENABLE;

}

// This function writes a certain message to a register on
// the slave device. Used to configure certain settings on
// said I2C sensor. ONLY WORKS WITH 7 BIT SLAVE ADDR.
// IT ONLY SUPPORTS WRITING TO 1 BYTE TO A REGISTER.
void masterWriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t writeData, uint8_t dataCount) {
    // Set current program state to TRANSMITTING;
    // If 0 data is requested, then we are in writing to addr
    // and then receiving input.
    state = dataCount ? TRANSMITTING : RECEIVE_SEND_REG;

    // Update the global data transmit buffer.
    // 0 Must be the register address
    dataToTransmitBuffer[0] = regAddr;
    if (dataCount) {
        dataToTransmitBuffer[1] = writeData;
    }
    transmitIndex = 0;
//    numTransmitted = 0;

    // Write Slave address to register.
    SLAVE_ADDR_REG = slaveAddr;
    // Disable RX interrupt
    USCI_INTERRUPT_ENABLE_REG &= ~RX_INTERRUPT_ENABLE;
    // Enable TX interrupt
    USCI_INTERRUPT_ENABLE_REG |= TX_INTERRUPT_ENABLE;
    // Set master into transmit mode and generate start condition.
    // This generates an interrupt
    CONTROL_REG_1 |= MASTER_TRANSMIT_MODE_BIT + START_CONDITION_BIT;
    // Wait here patiently until the interrupt fires.
    __bis_SR_register(LPM0_bits + GIE);
}

// Reading the register and then stores the result into receive buffer.
void masterReadReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t numReads) {
    state = RECEIVE_SEND_REG;
    numReceiveRemaining = numReads;
    receivedBufferIndex = 0;
    // Write the reg_addr to the sensor to start reading there.
    masterWriteReg(slaveAddr, regAddr, 0, 0);

    // First send start bit.
    // Send slave addr + write
    // send regAddr
    // Send repeated start bit + read
    // While numReads > 0
    //      read the data.
    // Send NACK + stop bit.
}

uint8_t i2cRead() {
    static uint8_t i = 0;
    if (i >= receivedBufferIndex) {
        i = 0;
    }
    return receivedBuffer[i++];
}

/* This function is where the interrupt magic happens.
 * There is only 1 interrupt vector for transmit, receive and state
 * changes.
 */
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void) {

    // __even_in_range returns UCB1IV where UCB1IV is even number between
    // 0 and 0x0C. __even_in_range is used to tell the compiler to optimise
    // the code further as the register cannot be an odd number.
    switch(__even_in_range(UCB1IV, 0x0C)) {
    /* A few cases were not included as they cannot be generated under
     * master mode. These include
     *      - No interrupt (USCI_NONE)
     *      - Arbitration lost (USCI_I2C_UCALIFG)
     *          - Triggered when another master try to speak at the same
     *              time.
     *      - Received Start signal (USCI_I2C_UCSTTIFG) (We're master mode)
     *      - Received Stop signal (USCI_I2C_UCSTPIFG) (We're master.)
     * For more information, see P.1013 of MSP430x5xx Family User Guide
     */
        // Not acknowledgement
        case USCI_I2C_UCNACKIFG:
            // Best practice is to send the message again and retry.
            // Or master should generate a stop condition.
            break;
        // Data received
        case USCI_I2C_UCRXIFG:
            // Just got a data bit in buffer. Put it into another bigger
            // buffer
            if (numReceiveRemaining-- > 0) {
//                receivedBuffer[receivedBufferIndex++] = RECEIVE_BUFFER_REG;

//                if (bitIndex == 2) {
//                    // PPG mode requires the data to be padded with 0.
//                    // 7 is 0111 and doing an AND operation will turn
//                    // all leading numbers into 0s since we are 0&(0/1) == 0
//                    currentData = RECEIVE_BUFFER_REG;
//                    currentData = currentData << 16;
//                    currentData &= 0x7FFFF;
//                    bitIndex--;
//                } else if (bitIndex == 1) {
//                    currentData = currentData | RECEIVE_BUFFER_REG << 8;
//                    bitIndex--;
//                } else {
//                    receivedBuffer[receivedBufferIndex++] = currentData | RECEIVE_BUFFER_REG;
//                    bitIndex = 2;
//                }
                receivedBuffer[receivedBufferIndex++] = RECEIVE_BUFFER_REG;
            }

            // When there is only 1 item left to read, set the stop register.
            if (numReceiveRemaining == 1) {
                CONTROL_REG_1 |= STOP_CONDITION_BIT;
            // No more items remaining to read.
            // Signal that our last item has be done. Reset the variables and
            // we can finish.
            } else if (!numReceiveRemaining) {
                // change state to IDLE.
                state = IDLE;
                // disable RX interrupt
                USCI_INTERRUPT_ENABLE_REG &= ~RX_INTERRUPT_ENABLE;
                // Exit low power mode.
                __bic_SR_register_on_exit(LPM0_bits);
            }
            break;

        // Transmit buffer empty interrupt.
        case USCI_I2C_UCTXIFG:
            switch(__even_in_range(state, 12)) {
            // start condition is generated and we can then write data to
            // the UCB1TXBUF register
            case TRANSMITTING:
                // If we haven't transmitted the 2 data send the data
                // If we have transmitted the 2 data points, end the transaction.
                TRANSMIT_BUFFER_REG = dataToTransmitBuffer[transmitIndex++];
                // If there are more data to transmit, keep transmitting else
                // send stop signal.
                state = transmitIndex < 2 ? TRANSMITTING : STOPPING;
                break;
            // In this case, we just send the register address.
            case RECEIVE_SEND_REG:
                // Transmit the address to start reading at.
                TRANSMIT_BUFFER_REG = dataToTransmitBuffer[transmitIndex];
                state = RESTART;
                break;
            // Send restart signal on read mode. mode.
            case RESTART:
                // Turn on RX interrupt and turn off TX interrupt
                USCI_INTERRUPT_ENABLE_REG |= RX_INTERRUPT_ENABLE;
                USCI_INTERRUPT_ENABLE_REG &= ~TX_INTERRUPT_ENABLE;
                // Turn transmit mode off. i.e Turn read mode on.
                CONTROL_REG_1 &= ~MASTER_TRANSMIT_MODE_BIT;
                // Generate repeated start condition in read mode.
                CONTROL_REG_1 |= START_CONDITION_BIT;
                // If we're only reading 1 register, sent stop bit now.
                if (numReceiveRemaining == 1) {
                    while ((CONTROL_REG_1 & START_CONDITION_BIT));
                    CONTROL_REG_1 |= STOP_CONDITION_BIT;
                }
                state = READING_DATA;
                break;
            case STOPPING:
                // Sending STOP bit.
                CONTROL_REG_1 |= STOP_CONDITION_BIT;
                // Disable TX interrupt;
                USCI_INTERRUPT_ENABLE_REG &= ~TX_INTERRUPT_ENABLE;
                state = IDLE;
                // Exit low power mode.
                __bic_SR_register_on_exit(LPM0_bits);
                break;
            } // State case

            break; // TXIGF case

        default:
            break;
    }
}
