/*
 * MAX86150.H
 *
 *  Created on: 26 Nov. 2021
 *      Author: Mkafahul
 */

#ifndef MAX86150_H_
#define MAX86150_H_

// MAX86150 Specific
#define SLAVE_ADDR                      0x5E
#define SYS_CTRL                        0x0D
#define SYS_RESET                       0x01

// MAX86150 FIFO Configuration
#define FIFO_CONFIG                     0x08
#define RD_DATA_CLR                     0x40
#define AFULL_ONCE                      0x20
#define FIFO_ROLLS_ON_FULL              0x10

// MAX86150 FIFO Control
#define FIFO_CTRL_1                     0x09
#define FIFO_CTRL_2                     0x0A
#define PPG_LED1                        0x01
#define PPG_LED2                        0x02

// MAX86150 PPG Configuration
#define PPG_CONFIG_1                    0x0E
#define PPG_CONFIG_2                    0x0F

// MAX86150 LED Range
#define LED_RANGE                       0x14
#define LED_RANGE_50mA                  0x00
#define IR_LED_PULSE_AMP                0x11
#define RED_LED_PULSE_AMP               0x12

// MAX86150 FIFO Operation
#define FIFO_DATA_REG                   0x07
#define OVERFLOW_COUNTER                0x05

// Interrupt Enable 1
#define INTERRUPT_ENABLE_1              0x02
#define PPG_RDY_EN                      0x40

void MAX86150Init(void);

#endif /* MAX86150_H_ */
