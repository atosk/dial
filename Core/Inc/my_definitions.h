/*
 * my_definitions.h
 *
 *  Created on: Apr 2, 2021
 *      Author: jason
 */

#ifndef INC_MY_DEFINITIONS_H_
#define INC_MY_DEFINITIONS_H_

// Handy definitions
#define TRUE  1
#define FALSE 0

// Stepper constants
#define MICROSTEP 16 // Stepper driver's microstep mode
#define DELAY_MS 500 // Pause this long after each stepper move
#define DIAL_DEGREES_PER_TICK 3.6

// Timer PWM constants
#define PWM_PERIOD_US   600 // Slow for my human eyes.
//#define PWM_PERIOD_US 156 // Fast! Two full rotations per second with 16 stepping
//#define PWM_PERIOD_US 140 // Faster!!! Try to break it! 140 seems to about the shortest reliable period.
#define PWM_PULSE_WIDTH_US 10 // Pulse width for PWM

// AS5600 Hall Effect sensor constants
#define AS5600_ADDR 0x36 << 1
#define AS5600_REG_ANGLE_H 0x0E
#define AS5600_REG_ANGLE_L 0x0F
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_CONF_H 0x07
#define AS5600_REG_CONF_L 0x08
#define ENCODER_DEGREES_PER_BIT 0.087890625 // Hysteresis off. Full resolution.
//#define ENCODER_DEGREES_PER_BIT 0.17578125 // Hysteresis 1 LSB. 11-bit resolution.

// Enumerations
enum Direction {CW=0, CCW=1};
enum StepperStatus {Stopped=0, Running=1};

#endif /* INC_MY_DEFINITIONS_H_ */
