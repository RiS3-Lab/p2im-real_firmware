/*
 * motor.h
 *
 *  Created on: Jun 17, 2017
 *      Author: Heethesh
 */

#ifndef MOTOR_H_
#define MOTOR_H_

void Motor_StopAll();
void Motor_SetSpeed(int m1, int m2, int m3, int m4);
void Motor_DistributeSpeed(float throttle, float pitch, float roll, float yaw);

#endif /* MOTOR_H_ */
