/*
 * joystick.h
 *
 *  Created on: Jun 26, 2017
 *      Author: Heethesh
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include "stdint.h"

typedef struct
{
	uint8_t MOTOR_ARM;
	uint8_t ALT_HOLD;
	volatile float throttle;
}Joystick_TypeDef;

extern Joystick_TypeDef joystick;

#endif /* JOYSTICK_H_ */
