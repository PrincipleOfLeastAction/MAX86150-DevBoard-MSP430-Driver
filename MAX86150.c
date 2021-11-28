/*
 * MAX86150.c
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */
#include "MAX86150.h"
#include "I2CDriver.h"

// Need delay between all the writes since there's only a 0.875us delay between
// STOP and START conditions.
// This increases it to around 7.375us.
// I2C specification requires 4.3us between STOP and START CONDITIONS.
// This should bring it into specs.
#define FORLOOPDELAY { int i = 0; \
        for(i = 0; i < 50 ; i++){}\
    }

// Code specific to MAX86150
void MAX86150Init() {
    // Reset the sensor
    masterWriteReg(SLAVE_ADDR, SYS_CTRL, SYS_RESET, 1);
    FORLOOPDELAY

//    while (state != IDLE) {
//        __no_operation();
//    }
    // Configure the FIFO to clear interrupt on read, almost full interrupt
    // once until another Afull condition is met and FIFO roll over when full
    // Also configure FIFO Almost full to set when 4 samples are in register.
    masterWriteReg(SLAVE_ADDR, FIFO_CONFIG, RD_DATA_CLR + AFULL_ONCE + \
                   FIFO_ROLLS_ON_FULL + 0x0F, 1);
    FORLOOPDELAY
    // PPG Configuration 1
    // Set PPG ADC range to 32768nA (62.5 per LSB)
    // Set sample rate to 100
    // Sample rates
    //  0xD9 ----- 400
    //  0xD1 ----- 100
    //  0xD5 ----- 200
    // PPG LED pulse width 100us
    masterWriteReg(SLAVE_ADDR, PPG_CONFIG_1, 0xD9, 1);
    FORLOOPDELAY
    // PPG Configuration 1
    // Set PPG ADC range to 4096nA (7.8125 per LSB)
    // Set sample rate to 100
    // PPG LED pulse width 100us
//    masterWriteReg(SLAVE_ADDR, PPG_CONFIG_1, 0x11, 1);

    // PPG Configuration 2
    // 4 Samples averaging
    masterWriteReg(SLAVE_ADDR, PPG_CONFIG_2, 0x02, 1);
    FORLOOPDELAY

    // FIFO Control
    // Putting LED 2 (Red) into data slot 1 and LED 1 (IR) into data slot 2
    masterWriteReg(SLAVE_ADDR, FIFO_CTRL_1, PPG_LED2 + (PPG_LED1 << 4), 1);
    FORLOOPDELAY
    // FIFO Control
    // LED 1 (IR) into data slot 1
//    masterWriteReg(SLAVE_ADDR, FIFO_CTRL_1, PPG_LED1, 1);

    // Set led pulse range
    // Set both led to 50mA range.
    masterWriteReg(SLAVE_ADDR, LED_RANGE, LED_RANGE_50mA + (LED_RANGE_50mA << 2), 1);
    FORLOOPDELAY

    //////////////////////// TEST ////////////////////////
    // READ from INTERRUPT status register.
    // For some reason power ready flag within the MAX86150 is getting
    // triggered.
    //
    // "Indicates that VBATT went below the UVLO threshold. This bit is not triggered
    // by a soft reset. This bit is cleared when interrupt status 1 register is read."
    // (AKA 0x00 the one we're reading)
    // May have to do with the sudden load created after LED_RANGE_50mA is set on both
    // LEDs causing a voltage drop below the threshold.
    // Hopefully reading this bit clears the error and the voltage regulator can stabilise
    // itself.
    masterReadReg(SLAVE_ADDR, 0x00, 1);
    FORLOOPDELAY
    ////////////////////// END TEST ///////////////////////


    // Set pulse amplitude Red to 0.2 mA * <Num from 0 to 255> -> to hex
    // 0xCD = 205 (dec) => 41mA
    masterWriteReg(SLAVE_ADDR, RED_LED_PULSE_AMP, 0xCD, 1);
    FORLOOPDELAY

    // Set pulse amplitude IR
    // 0xCD = 205 (dec) => 205/0.2 = 41mA
    masterWriteReg(SLAVE_ADDR, IR_LED_PULSE_AMP, 0xCD, 1);
    FORLOOPDELAY

    // Set pulse amplitude Red
//    masterWriteReg(SLAVE_ADDR, RED_LED_PULSE_AMP, 0xFF, 1);

    // Enable almost full interrupt
    masterWriteReg(SLAVE_ADDR, INTERRUPT_ENABLE_1, PPG_RDY_EN, 1);
    FORLOOPDELAY
    // Now enable FIFO
    masterWriteReg(SLAVE_ADDR, SYS_CTRL, 0x04, 1);
    FORLOOPDELAY
}
