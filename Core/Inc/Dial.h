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
   uint8_t m_dial_location = 0;
   uint8_t m_next_number = 0;

public:
   Dial(); // Constructor
   int CalculateSteps(enum Direction, uint8_t, uint8_t);
   uint8_t GetPosition();
   void UpdatePosition(uint8_t);
   ~Dial(); // Destructor
};

} /* namespace std */

#endif /* INC_DIAL_H_ */
