/*
 * ClockDriver.c
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */

#include <msp430.h>
#include "ucs.h"
#include "ClockDriver.h"

void clockInit() {
    // XT2 Clock init
    // Set port 5.3 and 5.2 to alternate mode
    XT2_PIN_PORT_ALT_SEL_REG |= XT2IN + XT2OUT;
    // Set XT2IN to Input and XT2OUT to output
    XT2_DIR_REG |= XT2OUT;

//    UNIFIED_CLOCK_SYSTEM_REG_4 |= SMCLK_CLOCK_SOURCE_XT2CLK;
    // Turns XT2 drive to low and XT2 on.
//    UNIFIED_CLOCK_SYSTEM_REG_6 &= ~(XT2DRIVE_HIGH + XT2OFF);
    UCS_setExternalClockSource(32768, 4000000);
    // Turn XT2 On
    UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ);
    // Initialise the Clocks
    UCS_initClockSignal(UCS_MCLK, UCS_XT2CLK_SELECT, UCS_CLOCK_DIVIDER_1);
    UCS_initClockSignal(UCS_SMCLK, UCS_XT2CLK_SELECT, UCS_CLOCK_DIVIDER_1);

}

