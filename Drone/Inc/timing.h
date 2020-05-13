/*
 * timing.h
 *
 *  Created on: Jun 19, 2017
 *      Author: Heethesh
 */

#ifndef TIMING_H_
#define TIMING_H_

#include "stm32f1xx_hal.h"

uint32_t millis();
uint32_t micros();
void delay_ms(__IO uint32_t Delay);

#endif /* TIMING_H_ */
