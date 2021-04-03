/*
 * Stepper.h
 *
 *  Created on: Apr 2, 2021
 *      Author: jason
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_
#include "main.h"
#include "my_definitions.h"

namespace std {

class StepperMotor {
private:
   int m_steps_remaining;
   enum StepperStatus m_status;
   uint8_t m_flag_move_complete;
   TIM_TypeDef *m_TIMx;


public:
   StepperMotor(TIM_TypeDef *TIMx);
   void UpdateStep(void);
   void Stop(void);
   void Move(int steps, enum Direction dir);
   StepperStatus Status(void);
   void ChangePeriod(uint16_t period_ms);
   uint8_t MoveComplete(void);
   ~StepperMotor();
};

} /* namespace std */

#endif /* INC_STEPPER_H_ */
