/*
 * i2c.c
 *
 *  Created on: Jun 22, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "timing.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

/**********************************
 Function name	:	I2C_WriteByte
 Functionality	:	To write a byte to a register on the I2C device
 Arguments		:	I2C Device Address, Register Address, Register Value, delay mode
 Return Value	:	None
 Example Call	:	I2C_WriteByte(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + MS5611_OSR, 1, 0)
 ***********************************/
void I2C_WriteByte(uint16_t device_add, uint16_t register_add, uint8_t register_val, uint8_t delay_mode)
{
	uint8_t byte[] = {register_val}, ret;
	ret = HAL_I2C_Mem_Write(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) byte, 1, 200);
	if (ret != HAL_OK) _Error_Handler(__FILE__, __LINE__);

	// Delay for device setup
	if (delay_mode) delay_ms(50);
}

/**********************************
 Function name	:	I2C_ReadByte
 Functionality	:	To read a byte from a register on the I2C device
 Arguments		:	I2C Device Address, Register Address, File name, Line number
 Return Value	:	Register value
 Example Call	:	I2C_ReadByte(MAG_ADDRESS, MAG_HXL, raw_data, __FILE__, __LINE__)
 ***********************************/
uint8_t I2C_ReadByte(uint16_t device_add, uint16_t register_add, char* file, int line)
{
	uint8_t byte[] = {0x00}, ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, byte, 1, 200);
	if (ret != HAL_OK) _Error_Handler(file, line);
	return byte[0];
}

/**********************************
 Function name	:	I2C_ReadBytes
 Functionality	:	To read multiple bytes from a register on the I2C device
 Arguments		:	I2C Device Address, Register Address, Buffer, Size, File name, Line number
 Return Value	:	Register value
 Example Call	:	I2C_ReadBytes(MS5611_ADDRESS, MS5611_CMD_ADC_READ, rxbuf, 3, __FILE__, __LINE__)
 ***********************************/
void I2C_ReadBytes(uint16_t device_add, uint16_t register_add, uint8_t* bytes, uint16_t size, char* file, int line)
{
	uint8_t ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, (uint16_t) device_add, (uint16_t) register_add,
				I2C_MEMADD_SIZE_8BIT, bytes, size, 200);
	if (ret != HAL_OK) _Error_Handler(file, line);
}

/**********************************
 Function name	:	I2C_ReadByteArray
 Functionality	:	To read multiple bytes from series of registers on the I2C device
 Arguments		:	I2C Device Address, Start Register Address, Buffer, Size, File name, Line number
 Return Value	:	None
 Example Call	:	I2C_ReadByteArray(MAG_ADDRESS, MAG_HXL, raw_data, 7, __FILE__, __LINE__)
 ***********************************/
void I2C_ReadByteArray(uint16_t device_add, uint16_t register_add, uint8_t* byte_array, uint16_t size, char* file, int line)
{
	int i = 0;
	while (i<size)
	{
		byte_array[i++] = I2C_ReadByte(device_add, register_add++, __FILE__, __LINE__);
	}
}
