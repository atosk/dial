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
   uint8_t m_next_number = 0;
   int m_dial_location = 0;

public:
   Dial(); // Constructor
   int CalculateSteps(enum Direction direction, int full_turns, int next_number);
   int GetPosition();
   void UpdatePosition(int new_position);
   ~Dial(); // Destructor
};

} /* namespace std */

#endif /* INC_DIAL_H_ */
