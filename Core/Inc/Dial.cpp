/*
 * Dial.cpp
 *
 *  Created on: Apr 5, 2021
 *      Author: jason
 */

#include <Dial.h>
#include "my_definitions.h"

namespace std {


Dial::Dial() : m_dial_location (0), m_next_number(0) {} // Constructor


int Dial::CalculateSteps(enum Direction direction, uint8_t full_turns, uint8_t next_number){
   m_next_number = next_number;
   int m_dial_temp = 0; // Temporary location to collect dial displacement

   /* Dial moving CCW means the numbers under the dial indicator are increasing.
    * CW means the numbers are decreasing.
    *
    * CCW 95->05 should return 10*2*MICROSTEP
    * CW  95->05 should return 90*2*MICROSTEP
    *
    * CCW 05->95 should return 90*2*MICROSTEP
    * CW  05->95 should return 10*2*MICROSTEP
    *
    */

   // Measure displacement.
   if (direction == CCW) { m_dial_temp = next_number - m_dial_location; }  // CCW
   else { m_dial_temp = m_dial_location - next_number; } // CW

   // Handle zero crossings
   if (m_dial_temp < 0){ m_dial_temp += 100;}

   // Calculate the number of pulses to send with the next stepper move command.
   return ((full_turns + m_dial_temp) * 2 * MICROSTEP);

}

void Dial::UpdatePosition(uint8_t new_position){
   m_dial_location = new_position;
}


int Dial::GetPosition(){
   return m_dial_location;
}

Dial::~Dial() { // Destructor

}

} /* namespace std */
