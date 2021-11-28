/*
 * ClockDriver.h
 *
 *  Created on: 26 Nov. 2021
 *      Author: Steven Poon
 */

#ifndef CLOCKDRIVER_H_
#define CLOCKDRIVER_H_

#include <msp430.h>
#include <stdint.h>

// Unified Clock Registers
#define UNIFIED_CLOCK_SYSTEM_REG_4      UCSCTL4
#define SMCLK_CLOCK_SOURCE_XT2CLK       SELS__XT2CLK
#define UNIFIED_CLOCK_SYSTEM_REG_6      UCSCTL6
#define XT2_PIN_PORT_ALT_SEL_REG        P5SEL
#define XT2_DIR_REG                     P5DIR
#define XT2IN                           BIT2
#define XT2OUT                          BIT3
#define XT2DRIVE_HIGH                   XT2DRIVE0 + XT2DRIVE1

void clockInit(void);

#endif /* CLOCKDRIVER_H_ */
