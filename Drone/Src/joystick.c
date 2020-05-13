/*
 * joystick.c
 *
 *  Created on: Jun 26, 2017
 *      Author: Heethesh
 */

#include "serial.h"
#include "msp.h"
#include "pid.h"
#include "common.h"
#include "joystick.h"
#include <string.h>

//#define PID_GUI_DEBUG
#define THROTTLE_LIMIT 10	// Limit pitch and roll angles

extern msp_set_pid msp_rxf_pid;
extern msp_set_raw_rc msp_rxf_raw_rc;
Joystick_TypeDef joystick = {0, 0};

extern PID_TypeDef pid_pitch;
extern PID_TypeDef pid_roll;
extern PID_TypeDef pid_yaw;
extern PID_TypeDef pid_altitude;

#ifdef PID_GUI_DEBUG
typedef struct __attribute__((__packed__))
{
	float set_point;
	float kp;
	float ki;
	float kd;
}Debug_PID;

typedef struct __attribute__((__packed__))
{
	Debug_PID pitch;
	Debug_PID roll;
	Debug_PID yaw;
	Debug_PID altitude;
}Debug_Buffer;

Debug_Buffer dbuff;
uint8_t debug_buffer[64];
#endif PID_GUI_DEBUG

/**********************************
 Function name	:	MSP_SetPID_Callback
 Functionality	:	Callback function to handle received PID data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetPID_Callback()
 ***********************************/
void MSP_SetPID_Callback(void)
{
	/* Scale and update PID gains*/
	pid_pitch.con_KP = msp_rxf_pid.pitch.p/10.0;
	pid_pitch.con_KI = msp_rxf_pid.pitch.i/200.0;
	pid_pitch.con_KD = msp_rxf_pid.pitch.d*4.0;

	pid_roll.con_KP = msp_rxf_pid.roll.p/10.0;
	pid_roll.con_KI = msp_rxf_pid.roll.i/200.0;
	pid_roll.con_KD = msp_rxf_pid.roll.d*4.0;

	pid_yaw.con_KP = msp_rxf_pid.yaw.p/10.0;
	pid_yaw.con_KI = msp_rxf_pid.yaw.i/200.0;
	pid_yaw.con_KD = msp_rxf_pid.yaw.d*4.0;

	pid_altitude.con_KP = msp_rxf_pid.alt.p/10.0;
	pid_altitude.con_KI = msp_rxf_pid.alt.i/200.0;
	pid_altitude.con_KD = msp_rxf_pid.alt.d*4.0;

	/* Echo PID gains back --> Used with PID-GUI.py */
#ifdef PID_GUI_DEBUG
	dbuff.pitch.set_point = pid_pitch.set_point + pid_pitch.offset;
	dbuff.pitch.kp = pid_pitch.con_KP;
	dbuff.pitch.ki = pid_pitch.con_KI;
	dbuff.pitch.kd = pid_pitch.con_KD;

	dbuff.roll.set_point = pid_roll.set_point + pid_roll.offset;
	dbuff.roll.kp = pid_roll.con_KP;
	dbuff.roll.ki = pid_roll.con_KI;
	dbuff.roll.kd = pid_roll.con_KD;

	dbuff.yaw.set_point = pid_yaw.set_point + pid_yaw.offset;
	dbuff.yaw.kp = pid_yaw.con_KP;
	dbuff.yaw.ki = pid_yaw.con_KI;
	dbuff.yaw.kd = pid_yaw.con_KD;

	dbuff.altitude.set_point = pid_altitude.set_point + pid_altitude.offset;
	dbuff.altitude.kp = pid_altitude.con_KP;
	dbuff.altitude.ki = pid_altitude.con_KI;
	dbuff.altitude.kd = pid_altitude.con_KD;

	memcpy(debug_buffer, &dbuff, 64);
	for (int i=0; i<64; i++)
		serialWrite(debug_buffer[i]);
#endif PID_GUI_DEBUG
}

/**********************************
 Function name	:	MSP_SetRawRC_Callback
 Functionality	:	Callback function to handle received raw RC data from joystick
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetRawRC_Callback()
 ***********************************/
void MSP_SetRawRC_Callback(void)
{
	/* Refer wireless joystick control in project documentation (report) */

	// AUX1 - Channel 5 - ARM Drone
	if (msp_rxf_raw_rc.aux1 > 1600) joystick.MOTOR_ARM = 1;
	else joystick.MOTOR_ARM = 0;

	// AUX2 - Channel 6 - Altitude Hold
	if (msp_rxf_raw_rc.aux2 > 1600) joystick.ALT_HOLD = 1;
	else joystick.ALT_HOLD = 0;

	// P-R-Y-T - Channel 1-4 - Set Points
	pid_pitch.set_point = constrain((float) msp_rxf_raw_rc.pitch - THROTTLE_LIMIT, -THROTTLE_LIMIT, THROTTLE_LIMIT);
	pid_roll.set_point = constrain((float) msp_rxf_raw_rc.roll - THROTTLE_LIMIT, -THROTTLE_LIMIT, THROTTLE_LIMIT);
	pid_yaw.set_point = (float) msp_rxf_raw_rc.yaw - 180.0f;
	if (!joystick.ALT_HOLD) joystick.throttle = (float) msp_rxf_raw_rc.throttle;

	// AUX 3-4 - Channel 7-8 - Pitch/Roll Trim
	pid_pitch.offset = ((float) msp_rxf_raw_rc.aux3 / 10.0f) - 10.0f;
	pid_roll.offset  = ((float) msp_rxf_raw_rc.aux4 / 10.0f) - 10.0f;
}
