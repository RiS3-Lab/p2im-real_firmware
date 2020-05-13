/*
 * i2c.h
 *
 *  Created on: Jun 22, 2017
 *      Author: Heethesh
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f1xx_hal.h"

void I2C_WriteByte(uint16_t device_add, uint16_t register_add, uint8_t register_val, uint8_t delay_mode);
uint8_t I2C_ReadByte(uint16_t device_add, uint16_t register_add, char* file, int line);
void I2C_ReadBytes(uint16_t device_add, uint16_t register_add, uint8_t* bytes, uint16_t size, char* file, int line);
void I2C_ReadByteArray(uint16_t device_add, uint16_t register_add, uint8_t* byte_array, uint16_t size, char* file, int line);

#endif /* I2C_H_ */
