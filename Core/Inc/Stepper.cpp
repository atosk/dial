/*
 * Stepper.cpp
 *
 *  Created on: Apr 2, 2021
 *      Author: jason
 */

#include "Stepper.h"
#include "my_definitions.h"

namespace std {
// Constructor
StepperMotor::StepperMotor(TIM_TypeDef *TIMx) {
   m_steps_remaining = 0;
   m_TIMx = TIMx;
   m_flag_move_complete = TRUE;
   m_status = Stopped;

}

void StepperMotor::UpdateStep(void) {
   // Called by timer IRQ. It tracks each pulse that has been sent
   m_steps_remaining--;

   if (m_steps_remaining == 0) {
      m_flag_move_complete = TRUE;
   }
}

void StepperMotor::Stop(void) {
   m_TIMx->CR1 &= ~(TIM_CR1_CEN);
   m_TIMx->CNT = 0;
   m_flag_move_complete = FALSE;
   m_status = Stopped;

}

void StepperMotor::Move(int steps, enum Direction dir) {
   // Writes the stepper direction bit and starts the timer for specified number of steps
   m_steps_remaining = steps;
   m_flag_move_complete = FALSE;
   m_status = Running;

   // Set Stepper motor direction pin
   GPIO_PinState direction;
   if (dir == CW) {
      direction = GPIO_PIN_SET;
   } else
      direction = GPIO_PIN_RESET;
   HAL_GPIO_WritePin(DIR_Pin_GPIO_Port, DIR_Pin_Pin, direction);

   // Reset the counter and enable counting.
   m_TIMx->CNT = 0;
   m_TIMx->CR1 |= TIM_CR1_CEN;
}

StepperStatus StepperMotor::Status(void){
   return m_status;
}


void StepperMotor::ChangePeriod(uint16_t period_ms) {
   /* This function changes the overflow period of the
    * running timer. Used for acceleration/deceleration.
    *
    * Called by ISR so the counter will be zero
    */
   m_TIMx->CR1 &= ~(TIM_CR1_CEN); // Disable timer
   m_TIMx->ARR = period_ms;       // Change the timer period
   m_TIMx->CR1 |= TIM_CR1_CEN;    // Enable the timer

}

uint8_t StepperMotor::MoveComplete() {
   return m_flag_move_complete;
}

// Deconstructor
StepperMotor::~StepperMotor() {

}
} /* namespace std */
