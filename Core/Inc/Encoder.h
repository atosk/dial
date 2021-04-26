/*
 * Encoder.h
 *
 *  Created on: Apr 13, 2021
 *      Author: jason
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

// AS5600 Hall Effect sensor constants
#define AS5600_ADDR            0x36 << 1
#define AS5600_REG_ANGLE_H     0x0E
#define AS5600_REG_ANGLE_L     0x0F
#define AS5600_REG_ANGLE_RAW_H 0x0C
#define AS5600_REG_ANGLE_RAW_L 0x0D
#define AS5600_REG_STATUS      0x0B
#define AS5600_REG_CONF_H      0x07
#define AS5600_REG_CONF_L      0x08
#define AS5600_REG_ZPOS        0x01
#define AS5600_REG_MPOS        0x03

#define AS5600_HYSTERESIS_1LSB 4

enum I2C_Status {
   Ready,
   Read_Error,
   Write_Error,
   Other_Error,
   Magnet_Missing,
   Magnet_Too_Strong,
   Magnet_Too_Weak,
   Magnet_OK
};

namespace std {

class Encoder {
private:
   I2C_HandleTypeDef *m_hi2cx;
   uint8_t m_i2c_receive_buf[16] = {0};
   char m_msg[100] = {0};
   double m_angle = 0;
   int m_starting_angle = 0;
   enum I2C_Status m_status = Ready;
   enum I2C_Status m_mag_status = Magnet_Missing;


public:
   Encoder(I2C_HandleTypeDef*, int);
   enum I2C_Status GetStatus();
   float GetLocation();
   ~Encoder();
};

} /* namespace std */

#endif /* INC_ENCODER_H_ */
