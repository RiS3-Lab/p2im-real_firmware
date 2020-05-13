/*
 * pid.h
 *
 *  Created on: Jun 15, 2017
 *      Author: Heethesh
 */

#ifndef PID_H_
#define PID_H_

#define DIRECT	 1
#define REVERSE	-1

#define PITCH 	1
#define ROLL	2
#define YAW		3
#define ALT		4

typedef struct
{
	int instance; 					// PID instance ID

	volatile float con_KP;			// Conservative proportional gain
	volatile float con_KI;			// Conservative integral gain
	volatile float con_KD;			// Conservative derivative gain

	volatile float agr_KP;			// Aggressive proportional gain
	volatile float agr_KI;			// Aggressive integral gain
	volatile float agr_KD; 			// Aggressive derivative gain

	volatile float set_point;		// Set point value
	volatile float breakpoint;		// Breakpoint value (Con/Agr border)
	volatile float offset; 			// CG offset value

	volatile float error;			// Error value
	volatile float last_error;		// Last error value
	volatile float input;			// Current position
	volatile float last_input;		// Previous position

	volatile float proportional;	// Proportional term
	volatile float integral;		// Integral sum
	volatile float derivative;		// Derivative term
	volatile float gyro;			// Gyroscope rate
	volatile float output; 			// PID output

	volatile int direction;			// Controller direction
	volatile uint32_t time;			// Loop time
	volatile uint32_t last_time;	// Last loop time
	volatile uint32_t delta;		// Loop time
}PID_TypeDef;

extern PID_TypeDef pid_pitch;
extern PID_TypeDef pid_roll;
extern PID_TypeDef pid_yaw;
extern PID_TypeDef pid_altitude;

void Motor_ARM(uint8_t enable);
void PID_Init();
void PID_Update();
void PID_SetGains(int instance, float ckp, float cki, float ckd, float akp, float aki, float akd);

#endif /* PID_H_ */

