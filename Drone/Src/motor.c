/*
 * motor.c
 *
 *  Created on: Jun 17, 2017
 *      Author: Heethesh
 */

/* * * * * * * * * * * * * * * * * * * *
 *									   *
 * 			 ---> +YAW (CW)			   *
 * 			|						   *
 * 			|	 -PITCH				   *
 *									   *
 * 	  M4 (CW)	X		X	M2 (CCW)   *
 * 				 \	_  /			   *
 * 				  \|^|/				   *
 * 	 	-ROLL  	   |_|	   +ROLL	   *
 * 		  		  /   \				   *
 * 				 /     \			   *
 * 	  M3 (CCW)	X		X	M1 (CW)	   *
 *									   *
 * 				 +PITCH				   *
 * 			|						   *
 * 			|---> -YAW (CCW)		   *
 *									   *
 *									   *
 * 	 QUADCOPTER MOTOR CONFIGURATION	   *
 *									   *
 * * * * * * * * * * * * * * * * * * * */

#include "common.h"
#include "devices.h"
#include "serial.h"
#include "msp.h"
#include "motor.h"

//#define MOTOR_DEBUG

extern msp_motor msp_txf_motor;
extern msp_set_motor msp_rxf_motor;

int motor_pwm [4] = {0, 0, 0, 0};

/**********************************
 Function name	:	Motor_UpdateMSP
 Functionality	:	To update the TX motor MSP frame
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Motor_UpdateMSP()
 ***********************************/
static void Motor_UpdateMSP(void)
{
	// Add 1000 as per MSP
	msp_txf_motor.motor[0] = motor_pwm[0] + 1000;
	msp_txf_motor.motor[1] = motor_pwm[1] + 1000;
	msp_txf_motor.motor[2] = motor_pwm[2] + 1000;
	msp_txf_motor.motor[3] = motor_pwm[3] + 1000;

	// Serial debug
#ifdef MOTOR_DEBUG
	serialInt(motor_pwm[0]+1000);
	serialWrite('\t');
	serialInt(motor_pwm[1]+1000);
	serialWrite('\t');
	serialInt(motor_pwm[2]+1000);
	serialWrite('\t');
	serialInt(motor_pwm[3]+1000);
	serialWrite('\n');
#endif
}

/**********************************
 Function name	:	Motor_UpdatePWM
 Functionality	:	Updates the PWM motor values directly. Range: [0, 1000]
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Motor_UpdatePWM()
 ***********************************/
static void Motor_UpdatePWM(void)
{
	Motor1_SetPWM(motor_pwm[0]); // Back Right
	Motor2_SetPWM(motor_pwm[1]); // Front Right
	Motor3_SetPWM(motor_pwm[2]); // Back Left
	Motor4_SetPWM(motor_pwm[3]); // Front Left

	// Update MSP frame
	Motor_UpdateMSP();
}

/**********************************
 Function name	:	Motor_StopAll
 Functionality	:	Directly set all PWM values to 0
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Motor_StopAll()
 ***********************************/
void Motor_StopAll(void)
{
	Motor1_SetPWM(0); // Back Right
	Motor2_SetPWM(0); // Front Right
	Motor3_SetPWM(0); // Back Left
	Motor4_SetPWM(0); // Front Left
}

/**********************************
 Function name	:	Motor_SetSpeed
 Functionality	:	Update the PWM motor values into the array. Range: [0, 1000]
 Arguments		:	PWM motor values
 Return Value	:	None
 Example Call	:	Motor_SetSpeed(0, 200, 500, 300)
 ***********************************/
void Motor_SetSpeed(int m1, int m2, int m3, int m4)
{
	motor_pwm[0] = constrain(m1, 0, 1000);
	motor_pwm[1] = constrain(m2, 0, 1000);
	motor_pwm[2] = constrain(m3, 0, 1000);
	motor_pwm[3] = constrain(m4, 0, 1000);

	// Write PWM values to the motors
	Motor_UpdatePWM();
}

/**********************************
 Function name	:	Motor_DistributeSpeed
 Functionality	:	To distribute the PID output values from the 4 axes to the different motors
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Motor_DistributeSpeed(1000, 1200, 1300, 1400)
 ***********************************/
void Motor_DistributeSpeed(float throttle, float pitch, float roll, float yaw)
{
	float M1, M2, M3, M4;

	M4 = throttle - pitch - roll + yaw - 1000; // Front Left
	M2 = throttle - pitch + roll - yaw - 1000; // Front Right
	M3 = throttle + pitch - roll - yaw - 1000; // Back Left
	M1 = throttle + pitch + roll + yaw - 1000; // Back Right

	// Update the PWM value array, but don't write the values to motors
	Motor_SetSpeed((int) M1, (int) M2, (int) M3, (int) M4);
}

/**********************************
 Function name	:	MSP_SetMotor_Callback
 Functionality	:	Callback function to handle received motor data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetMotor_Callback()
 ***********************************/
void MSP_SetMotor_Callback(void)
{
	float m1, m2, m3, m4;

	m1 = constrain(msp_rxf_motor.motor[0] - 1000, 0, 1000);
	m2 = constrain(msp_rxf_motor.motor[1] - 1000, 0, 1000);
	m3 = constrain(msp_rxf_motor.motor[2] - 1000, 0, 1000);
	m4 = constrain(msp_rxf_motor.motor[3] - 1000, 0, 1000);

	// Update the PWM value array, but don't write the values to motors
	Motor_SetSpeed(m1, m2, m3, m4);
}
