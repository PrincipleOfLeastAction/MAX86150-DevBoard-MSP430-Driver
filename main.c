/*
 * This program initialises the MAX86150 and requests the data for
 * PPG operation. It stores the data requested in an array and then
 * goes into sleep mode.
 *
 * Wiring Diagram
 * MSP430   --------- MAX86150      Logic Analyser
 * P4.1     --------- SDA
 * P4.2     --------- SCL
 * P2.7     --------- INT
 * 3.3V     --------- VCC
 * GND      --------- GND
 * P3.2 ----------------------------- Normal Pin (Interrupt Debug GPIO toggled)
 *
 * MSP430   --------- UART
 * P3.3 (TX)--------- RX
 *
 * Author: Steven Poon
 ************************* TO DO ************************
 * - Implement continuous logging into flash space.
 * - Implement SD card logging interface via SPI.
 * -
 * ************ PROBLEMS ENCOUNTERED AND *FIXED* ************
 * - Tbuf (bus free time between a stop condition and the following
 * start) not meeting I2C standards.
 * - Potentially voltage regulator dropping voltage below certain threshold
 * when turning on both R and IR LEDs to 50ma range causing an power error flag
 * to be raised inside MAX86150. Reading the flag should clear it and give it a bit
 * of time to stabilise itself.
 * - Interrupt not triggering on MAX86150's interrupts.
 * It appeared that inside sensorInterrupt() function, we needed to check
 * P2IV to see if it equals 0x10 which is equivalent of P2.7 triggering since
 * the interrupt was being triggered on P2IE being enabled.
 */

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include "usci_a_uart.h"
#include "BCUart.h"
#include "MAX86150.h"
#include "I2CDriver.h"
#include "ClockDriver.h"
#include "UARTDriver.h"
//#ifndef "settings.h"
//#include "settings.h"
//#endif

// UART SETTINGS
// Either 9600 or 115200 is supported
#define BAUD 115200

#define NUM_ACTIVE_ELEMENT              2
//#define NUM_ACTIVE_ELEMENT              1
#define PPG_DATA_SIZE                   3

#define TRANSMIT_BUFFER                 2
#define RECEIVED_BUFFER                 12
#define PPG_BUFFER_SIZE                 30


// OVERFLOW LED REGISTERS
#define OVERFLOW_LED_PORT               P1SEL
#define OVERFLOW_LED_PIN                BIT0
#define OVERFLOW_LED_DIR                P1DIR
#define OVERFLOW_LED_OUT                P1OUT

// Prototypes
uint8_t intToAscii(int32_t num, char* str);
void intToAsciiRecursive(int32_t num, char* str, uint8_t* bytesWritten);
void programInit(void);
void overflowLedOn();
__interrupt void sensorInterrupt(void);



// Some global variable to assist
// Holds the data that is to be transmitted.
uint8_t dataToTransmitBuffer[TRANSMIT_BUFFER];
uint8_t receivedBuffer[RECEIVED_BUFFER];
uint8_t receivedBufferIndex = 0;

int32_t currentData;
uint8_t bitIndex = 2;
uint8_t transmitIndex = 0;
uint8_t numReceiveRemaining;
//uint8_t numTransmitted = 0;
volatile uint8_t readingAvailable = 0;
volatile uint8_t interruptStatus = 0;

uint8_t ledState = 1;

void main(void) {

    WDTCTL = WDTPW | WDTHOLD; // Stop the watchdog timer.

    __disable_interrupt();
    // Initialise Global interrupt
    programInit();
    // Initialise I2C bus.
    I2C_Init();
    // Initialise UART
    if (uartInit(BAUD) == STATUS_FAIL) {
        return;
    }
    // Initialise Backchannel UART to send data via the usb.
    bcUartInit();

    // Turn MAX86150 on
    MAX86150Init();

    __enable_interrupt();

    // After the registers initalisations are done, enable interrupt on P2.7
    // Set P2.7 as interrupt input pin for the MAX86150 interrupts
    P2IE |= BIT7;
    // Edge select as going HIGH to LOW (aka FALLING)
    P2IES |= BIT7;

    // Set interruptStatus to output to P3.2
    // Declare 3.3 as output.
    P3DIR |= BIT2;
    // Set the output to output 0.
    P3OUT &= ~BIT2;
    // Set drive strength to 1.
    P3DS |= BIT2;

    // A place to store ppgData
//    int32_t ppgData[PPG_BUFFER_SIZE];
//    uint8_t ppgDataIndex = 0;
    // Back channel uart test
//    uint8_t bcUartByteArray[] = "Error: snprintf";
//    bcUartByteArray[0] = 'A';
//    bcUartByteArray[1] = 'B';
//    bcUartByteArray[2] = 'C';

    // Array to hold hex to char
    // Now, size. Maximum hex byte is 19 bits = 524287.
    // 524287 is 6 characters, 1 sign bit, 1 led char identifier, 1 new line, 1 nullbyte = 10 bytes
    // 255 is 3 characters. There are 3 bytes
    char tmpCharArray[10];
    int32_t redNum;
    int32_t irNum;

    uint8_t numBytes = 0;

    while (1) {
        if (readingAvailable) {
            // 3 read for every active device. (For the 3 bytes)
            masterReadReg(SLAVE_ADDR, FIFO_DATA_REG, NUM_ACTIVE_ELEMENT * PPG_DATA_SIZE);
//            ppgData[ppgDataIndex++] = receivedBufferIndex;
//            USCI_A_UART_transmitData(USCI_A0_BASE, (int8_t) (receivedBuffer[receivedBufferIndex - 3]));
//            USCI_A_UART_transmitData(USCI_A0_BASE, (int8_t) (receivedBuffer[receivedBufferIndex - 2]));
//            USCI_A_UART_transmitData(USCI_A0_BASE, (int8_t) (receivedBuffer[receivedBufferIndex - 1]));

            // Send 1 2 3 via back channel UART.
            // Convert the received buffer into a number.

            redNum = (0b00000111 & ((int32_t) receivedBuffer[0])) << 16 | ((int32_t) receivedBuffer[1]) << 8 | ((int32_t) receivedBuffer[2]);
            irNum = (0b00000111 & ((int32_t) receivedBuffer[3])) << 16 |  ((int32_t) receivedBuffer[4]) << 8 | ((int32_t) receivedBuffer[5]);

            // Convert the hex symbols into decimal characters.
//            numBytes = snprintf(tmpCharArray, n, "R%ld\n", -redNum);
            tmpCharArray[0] = 'R';
            numBytes = intToAscii(-redNum, tmpCharArray+1);
            tmpCharArray[numBytes+1] = '\n';
            tmpCharArray[numBytes+2] = '\0';
            numBytes += 2;
//            if (numBytes < 0) {
//                bcUartSend(bcUartByteArray, 16);
//            }
            bcUartSend(tmpCharArray, numBytes);

            tmpCharArray[0] = 'I';
            numBytes = intToAscii(-irNum, tmpCharArray+1);
            tmpCharArray[numBytes+1] = '\n';
            tmpCharArray[numBytes+2] = '\0';
            numBytes += 2;
//            numBytes = snprintf(tmpCharArray, n, "I%ld\n", -irNum);
            // If numBytes returned is negative, something wrong has happened.
//            if (numBytes < 0) {
//                bcUartSend(bcUartByteArray, 16);
//            }
            bcUartSend(tmpCharArray, numBytes);

            masterReadReg(SLAVE_ADDR, OVERFLOW_COUNTER, 1);
            // If there has been overflowed in buffer, turn led on.
            if (ledState & receivedBuffer[receivedBufferIndex - 1]) {
                ledState = 0;
                overflowLedOn();
            }
            // Set reading available back to 0.
            readingAvailable = 0;

            // Turn interrupt back on.
            P2IE = BIT7;
        }
    }
//    __bis_SR_register(LPM4_bits);
}

// This function converts int to ASCII characters.
// sprintf doesn't work because of some compiler/ architecture
// bug not supporting %ld format identifier.
// Possible architecture bug due to MSP430 being 16 bits and
// our data is stored in 32 bits hence the lack of support.
// Returns the length of str written with a nullbyte added.
uint8_t intToAscii(int32_t num, char* str) {
//    printf("%d\n", num);

    uint8_t bytesWritten = 0;

    intToAsciiRecursive(num, str, &bytesWritten);
    str[bytesWritten] = '\0';
    return bytesWritten;
}

// Recursive case for int to ASCII.
void intToAsciiRecursive(int32_t num, char* str, uint8_t* bytesWritten) {
    if (num < 0) {
        str[(*bytesWritten)++] = '-';
        // Make it positive
        num *= -1;
    }

    if (num < 10) {
        str[(*bytesWritten)++] = '0' + num;
        return;
    }
    intToAsciiRecursive(num / 10, str, bytesWritten);
    intToAsciiRecursive(num % 10, str, bytesWritten);
}

void programInit() {
    // Low power initialisation.
    PADIR = 0x0;
    PAOUT = 0x0;
    PBDIR = 0x0;
    PBOUT = 0x0;
    PCDIR = 0x0;
    PCOUT = 0x0;
    PDDIR = 0x0;
    PDOUT = 0x0;

    // LED P1.0 init
    // Configure led pin as output.
    OVERFLOW_LED_DIR |= OVERFLOW_LED_PIN;
    OVERFLOW_LED_OUT &= ~OVERFLOW_LED_PIN;

    clockInit();
}

void overflowLedOn() {
    OVERFLOW_LED_OUT |= OVERFLOW_LED_PIN;
}

/* This function is set when an interrupt is detected in P2.7
 * It is configured to trigger when a new PPG data is ready in the FIFO
 */
#pragma vector=PORT2_VECTOR
__interrupt void sensorInterrupt(void) {
    // Ensure that the interrupt is from P2.7.
    if (P2IV == 0x10) {
        // Output to GPIO to say we're inside ISR.
        interruptStatus = 1;
        P3OUT |= BIT2;

        readingAvailable = 1;
        P2IE &= ~BIT7;

        // Clear the ISR bit.
        interruptStatus = 0;
        P3OUT &= ~BIT2;
    }
}



