/*
 * devices.h
 *
 *  Created on: Jun 7, 2017
 *      Author: Heethesh
 */

#ifndef DEVICES_H_
#define DEVICES_H_

#define RED		0
#define BLUE	1
#define WHITE	2

void Devices_Init(void);
void toggleLED(int led1, int led2, int led3);
void Motor1_SetPWM(int pwm);
void Motor2_SetPWM(int pwm);
void Motor3_SetPWM(int pwm);
void Motor4_SetPWM(int pwm);
void Motor5_SetPWM(int pwm);
void Motor6_SetPWM(int pwm);

#endif /* DEVICES_H_ */
