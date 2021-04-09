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
//#define PWM_PERIOD_US 156 // Fast! Two full rotations per second with 16 stepping
#define DIAL_DEGREES_PER_TICK 3.6

// Timer PWM constants
#define PWM_PERIOD_US   600 // Slow for my human eyes.
#define PWM_PULSE_WIDTH_US 10 // Pulse width for PWM

// AS5600 Hall Effect sensor constants
#define I2C_ENCODER_ADDRESS 0x36
#define ENCODER_DEGREES_PER_BIT 0.087890625

// Enumerations
enum Direction {CW=0, CCW=1};
enum StepperStatus {Stopped=0, Running=1};

#endif /* INC_MY_DEFINITIONS_H_ */
