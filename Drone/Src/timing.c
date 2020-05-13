/*
 * timing.c
 *
 *  Created on: Jun 19, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "timing.h"

static __IO uint32_t Micros;

/**********************************
 Function name	:	millis
 Functionality	:	Returns the program time in milliseconds
 Arguments		:	None
 Return Value	:	Program time in milliseconds
 Example Call	:	millis()
 ***********************************/
uint32_t millis(void)
{
	return HAL_GetTick();
}

/**********************************
 Function name	:	micros
 Functionality	:	Returns the program time in microseconds
 Arguments		:	None
 Return Value	:	Program time in microseconds
 Example Call	:	micros()
 ***********************************/
uint32_t micros(void)
{
	//Micros = millis() * 1000 + 1000 - SysTick->VAL/64;
	//return Micros;

	// Doesn't work properly with some libraries, currently returns millis()
	return HAL_GetTick();
}

/**********************************
 Function name	:	delay_ms
 Functionality	:	To provide a delay in milliseconds
 Arguments		:	Delay in milliseconds
 Return Value	:	None
 Example Call	:	delay_ms()
 ***********************************/
void delay_ms(uint32_t Delay)
{
	HAL_Delay(Delay);
}
