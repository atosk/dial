/*
 * Dial.h
 *
 *  Created on: Apr 5, 2021
 *      Author: jason
 */

#ifndef INC_DIAL_H_
#define INC_DIAL_H_
#include "main.h"
#include "my_definitions.h"


namespace std {

class Dial {
private:
   float m_dial_location = 0;
   uint8_t m_next_number = 0;

public:
   Dial(); // Constructor
   int CalculateSteps(enum Direction, uint8_t, float);
   float GetPosition();
   void UpdatePosition(float);
   ~Dial(); // Destructor
};

} /* namespace std */

#endif /* INC_DIAL_H_ */
