/*
 * my_definitions.h
 *
 *  Created on: Apr 2, 2021
 *      Author: jason
 */

#ifndef INC_MY_DEFINITIONS_H_
#define INC_MY_DEFINITIONS_H_


#define TRUE  1
#define FALSE 0
#define MICROSTEP 16 // Stepper driver's microstep mode
#define DELAY_MS 200    // Step pulse train is disabled for this long before reversing the stepper's direction.
//#define PWM_PERIOD_US 156 // Fast! Two full rotations per second with 16 stepping
#define PWM_PERIOD_US   600 // Slow for my human eyes.
#define PWM_PULSE_WIDTH_US 10 // Pulse width for PWM

enum Direction {CW=0, CCW=1};
enum StepperStatus {Stopped=0, Running=1};

#endif /* INC_MY_DEFINITIONS_H_ */
