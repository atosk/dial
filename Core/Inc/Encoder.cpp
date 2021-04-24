/*
 * Encoder.cpp
 *
 *  Created on: Apr 13, 2021
 *      Author: jason
 */

#include <Encoder.h>
#include "my_definitions.h"
#include "string.h"

namespace std {

Encoder::Encoder(I2C_HandleTypeDef HI2Cx, int hyst_mode) {
   m_hi2cx = HI2Cx;

   // Read magnet status
   if (HAL_I2C_Mem_Read(&m_hi2cx, AS5600_ADDR, AS5600_REG_STATUS,
         I2C_MEMADD_SIZE_8BIT, m_i2c_receive_buf, 1, 200) != HAL_OK) {
      m_status = Read_Error;
   } else m_status = Ready;

   // Parse magnet status
   if (m_status == Ready){
      // Mask off unnecessary bits
      char reg_status = (m_i2c_receive_buf[0] & (7UL << 3)); // Only bits [5:3] matter for now
      switch (reg_status) {
         case 0:
            m_mag_status = Magnet_Missing;
            break;
         case 40:
            m_mag_status = Magnet_Too_Strong;
            break;
         case 48:
            m_mag_status = Magnet_Too_Weak;
            break;
         case 32:
            m_mag_status = Magnet_OK;
            break;
         default:
            m_mag_status = Other_Error;
            break;

            // Catch errors
            if (m_mag_status != Magnet_OK) {
               m_status = m_mag_status;
            }
      }
   } // End status parse

   // Write to configuration Register
   if (m_status == Ready) {

      uint8_t i2c_write_buf[2] = { 0 };
      i2c_write_buf[1] = hyst_mode;

      if (HAL_I2C_Mem_Write(&m_hi2cx, AS5600_ADDR, AS5600_REG_CONF_H,
      I2C_MEMADD_SIZE_16BIT, i2c_write_buf, 2, 200) != HAL_OK) {
         m_status = Write_Error;
      }
   } // End configuration write

   // tare starting angle.
   if (m_status == Ready) {
      int starting_angle = Encoder::GetAngle();
   } // End tare angle

} // End constructor

enum I2C_Status Encoder::GetStatus(){
   return m_status;
}

int Encoder::GetAngle(){
   if (HAL_I2C_Mem_Read(&m_hi2cx, AS5600_ADDR, AS5600_REG_ANGLE_H, 1,m_i2c_receive_buf, 2, 200) != HAL_OK){
      m_angle = -1;
      m_status = Read_Error;
   }
   m_angle = (m_i2c_receive_buf[0] << 8) | m_i2c_receive_buf[1]; // Concatenate the two bytes
   return m_angle;
}

Encoder::~Encoder() {
   // TODO Auto-generated destructor stub
}

} /* namespace std */
