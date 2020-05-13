/*
 * pid.c
 *
 *  Created on: June 15, 2017
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

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "devices.h"
#include "timing.h"
#include "common.h"
#include "MPU9250.h"
#include "MS5611.h"
#include "motor.h"
#include "msp.h"
#include "serial.h"
#include "joystick.h"
#include "pid.h"

//#define PID_DEBUG
//#define PID_GYRO
#define PID_LIMIT 500


PID_TypeDef pid_pitch = {PITCH, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_roll = {ROLL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_yaw = {YAW, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_altitude = {ALT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

extern Joystick_TypeDef joystick;
extern msp_pid msp_txf_pid;
uint8_t ALT_FLAG = 0;
uint32_t pid_time = 0;

/**********************************
 Function name	:	PID_UpdateMSP
 Functionality	:	To update the PID MSP frame
 Arguments		:	None
 Return Value	:	None
 Example Call	:	PID_UpdateMSP()
 ***********************************/
static void PID_UpdateMSP(void)
{
	msp_txf_pid.pitch.p = pid_pitch.con_KP * 255;
	msp_txf_pid.pitch.i = pid_pitch.con_KI * 255;
	msp_txf_pid.roll.d = pid_pitch.con_KD * 255;

	msp_txf_pid.roll.p = pid_roll.con_KP * 255;
	msp_txf_pid.roll.i = pid_roll.con_KI * 255;
	msp_txf_pid.roll.d = pid_roll.con_KD * 255;

	msp_txf_pid.yaw.p = pid_yaw.con_KP * 255;
	msp_txf_pid.yaw.i = pid_yaw.con_KI * 255;
	msp_txf_pid.yaw.d = pid_yaw.con_KD * 255;

	msp_txf_pid.alt.p = pid_altitude.con_KP * 255;
	msp_txf_pid.alt.i = pid_altitude.con_KI * 255;
	msp_txf_pid.alt.d = pid_altitude.con_KD * 255;
}

/**********************************
 Function name	:	PID_Compute
 Functionality	:	PID control algorithm based on Brett Beauregard's PID library
 Arguments		:	PID struct instance
 Return Value	:	None
 Example Call	:	PID_Compute(&pid_pitch)
 ***********************************/
static void PID_Compute(PID_TypeDef *pid)
{
	float KP = 0, KI = 0, KD = 0;

	// Compute error
	pid->error = (pid->set_point + pid->offset) - pid->input;

	/* --------------------------- YAW WRAP ERROR ------------------------------
     * An example in degrees is the best way to explain this.  If the target is
     * -179 degrees and the input is +179 degrees, the standard PID output would
     * be -358 degrees leading to a very high yaw rotation rate to correct the
     * -358 degrees error.  However, +2 degrees achieves the same result, with
     * a  much lower rotation rate to fix the error.
     * -----------------------------------------------------------------------*/
	if (pid->instance == YAW)
	{
		if (abs(pid->error) > 180)
			pid->error = pid->error - (360 * pid->error / abs(pid->error));
	}

	// Conservative PID gains
	//if (pid_altitude.output < pid->breakpoint)
	//{
		KP = pid->con_KP;
		KI = pid->con_KI;
		KD = pid->con_KD;
	/*}

	// Aggressive PID gains - Not used currently
	else
	{
		KP = pid->agr_KP;
		KI = pid->agr_KI;
		KD = pid->agr_KD;
	}*/

	// Compute proportional term
	pid->proportional = KP * pid->error;

	// Compute integral sum
	pid->integral += pid->error;

	// Constrain integral term to prevent wind-up
	pid->integral = constrain(pid->integral, -PID_LIMIT, PID_LIMIT);

	// Compute derivative term
#ifndef PID_GYRO
	pid->derivative = pid->input - pid->last_input;
#endif

	// Experimental - Use gyro rate instead of computing derivative
#ifdef PID_GYRO
	pid->derivative = pid->gyro;
#endif

	// Compute angle PID output
	pid->output = (pid->proportional) + (KI * pid->integral) - (KD * pid->derivative);

	// Constrain angle PID output to PWM range
	pid->output = pid->direction * constrain(pid->output, -PID_LIMIT, PID_LIMIT);

	// Store variable for next iteration
	pid->last_input = pid->input;
	pid->last_error = pid->error;
	pid->last_time  = pid->time;
}

/**********************************
 Function name	:	PID_SetGains
 Functionality	:	To set the PID gains
 Arguments		:	PID instance macro, conservative and aggressive PID gains
 Return Value	:	None
 Example Call	:	PID_SetGains(ROLL, 3.8f, 0.02f, 260.0f, 3.8f, 0.02f, 320.0f)
 ***********************************/
void PID_SetGains(int instance, float ckp, float cki, float ckd, float akp, float aki, float akd)
{
	switch (instance)
	{
		case PITCH:
			pid_pitch.con_KP = ckp;
			pid_pitch.con_KI = cki;
			pid_pitch.con_KD = ckd;
			pid_pitch.agr_KP = akp;
			pid_pitch.agr_KI = aki;
			pid_pitch.agr_KD = akd;
			break;

		case ROLL:
			pid_roll.con_KP = ckp;
			pid_roll.con_KI = cki;
			pid_roll.con_KD = ckd;
			pid_roll.agr_KP = akp;
			pid_roll.agr_KI = aki;
			pid_roll.agr_KD = akd;
			break;

		case YAW:
			pid_yaw.con_KP = ckp;
			pid_yaw.con_KI = cki;
			pid_yaw.con_KD = ckd;
			pid_yaw.agr_KP = akp;
			pid_yaw.agr_KI = aki;
			pid_yaw.agr_KD = akd;
			break;

		case ALT:
			pid_altitude.con_KP = ckp;
			pid_altitude.con_KI = cki;
			pid_altitude.con_KD = ckd;
			pid_altitude.agr_KP = akp;
			pid_altitude.agr_KI = aki;
			pid_altitude.agr_KD = akd;
			break;
	}
}

/**********************************
 Function name	:	PID_Init
 Functionality	:	To setup the PID algorithm parameters
 Arguments		:	None
 Return Value	:	None
 Example Call	:	PID_Init()
 ***********************************/
void PID_Init(void)
{
	/* Set Controller Direction */
	pid_pitch.direction = DIRECT;
	pid_roll.direction = DIRECT;
	pid_yaw.direction = REVERSE;
	pid_altitude.direction = DIRECT;

	/* Set PID Loop Time - Not used currently */
	/*pid_pitch.delta = 5;
	pid_roll.delta = 5;
	pid_yaw.delta = 5;
	pid_altitude.delta = 5;*/

	/* Set Breakpoint Value - Border value between conservative and aggressive PID gains */
	pid_pitch.breakpoint = 1500;
	pid_roll.breakpoint = 1450;
	pid_yaw.breakpoint = 2000;
	pid_altitude.breakpoint = 0;

	/* Set PID Gains */
	PID_SetGains(PITCH, 4.4f, 0.02f, 280.0f, 4.5f, 0.02f, 360.0f);
	PID_SetGains(ROLL, 3.8f, 0.02f, 260.0f, 3.8f, 0.02f, 320.0f);
	PID_SetGains(YAW, 3.0f, 0.01f, 280.0f, 0, 0, 0);
	PID_SetGains(ALT, 10.0f, 0, 0, 0, 0, 0);
}


/**********************************
 Function name	:	PID_UpdateGyro
 Functionality	:	Experimental - To get gyro rate instead of computing derivative
 Arguments		:	None
 Return Value	:	None
 Example Call	:	PID_UpdateGyro()
 ***********************************/
#ifdef PID_GYRO
void PID_UpdateGyro(void)
{
	// Buffer
	float gyroData[3];

	// Get gyro rates
	MPU9250_GetGyroData(gyroData);

	// Update gyro rate in PID instances
	pid_pitch.gyro = gyroData[0];
	pid_roll.gyro = gyroData[1];
	pid_yaw.gyro = gyroData[2];
}
#endif

/**********************************
 Function name	:	PID_UpdateAltitude
 Functionality	:	To toggle PID between manual throttle control and auto altitude hold modes
 Arguments		:	None
 Return Value	:	None
 Example Call	:	PID_UpdateAltitude()
 ***********************************/
void PID_UpdateAltitude(void)
{
	// Enable altitude hold mode
	if (joystick.ALT_HOLD)
	{
		if (ALT_FLAG == 0)
		{
			// Start level holding at the current altitude
			pid_altitude.set_point = MS5611_GetFilteredAltitude()*100;
			ALT_FLAG = 1;
		}

		// Feedback input to hold level at the set altitude
		else pid_altitude.input = MS5611_GetFilteredAltitude()*100;
	}

	// Manual throttle control mode
	if (!joystick.ALT_HOLD)
	{
		if (ALT_FLAG) ALT_FLAG = 0;
		pid_altitude.output = (float) joystick.throttle;
	}
}

/**********************************
 Function name	:	PID_Update
 Functionality	:	To update the PID input values, compute outputs and update motors
 Arguments		:	None
 Return Value	:	None
 Example Call	:	PID_Update()
 ***********************************/
void PID_Update(void)
{
	// Update PID inputs
	pid_pitch.input = AHRS_GetPitch();
	pid_roll.input = AHRS_GetRoll();
	pid_yaw.input = AHRS_GetYaw();

	// Update altitude PID input
	PID_UpdateAltitude();

	// Serial PID debug to check input angles
#ifdef PID_DEBUG
	serialInt(pid_pitch.input);
	serialWrite('\t');
	serialInt(pid_roll.input);
	serialWrite('\t');
	serialInt(pid_yaw.input);
	serialWrite('\t');
#endif

	/* Emergency Power Down  for angles greater than 80 degrees */
	if ((abs(pid_pitch.input) > 80) || (abs(pid_roll.input) > 80))
	{
		// Stop motors for safety
		Motor_StopAll();
		toggleLED(0, 1, 0);
		return;
	}
	else toggleLED(1, 1, 1);

	/* Compute PID */
	PID_Compute(&pid_pitch);
	PID_Compute(&pid_roll);
	PID_Compute(&pid_yaw);

	if (joystick.ALT_HOLD)
	{
		PID_Compute(&pid_altitude);
		pid_altitude.output += 1500;
	}

	//pid_pitch.output = 0;
	//pid_roll.output = 0;
	//pid_yaw.output = 0;
	pid_altitude.output = joystick.throttle; // Currently in manual mode

	// If drone is armed, update motor PWM values
	if (joystick.MOTOR_ARM) Motor_DistributeSpeed(pid_altitude.output, pid_pitch.output, pid_roll.output, pid_yaw.output);
	else Motor_StopAll();

	// Update PID MSP frame
	PID_UpdateMSP();
}
