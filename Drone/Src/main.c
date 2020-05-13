/*
 * Project Name: Firmware V101-103C8
 * File Name: main.c
 * Created: June 7, 2017
 *
 * Mentors: Sanam Shakya, Pushkar Raj
 * Author : Heethesh Vhavle
 *
 * e-Yantra Summer Internship Program 2017
 * Project: Control and Algorithms Development for Quadcopter
 *
 * Firmware for Pluto Drone
 * Version: 1.0.1A
 * Target: STM32F103C8
 *
 * Hardware Connections:
 *
 * I2C1 Bus --> 0x68 - MPU9250
 * I2C1 Bus --> 0x0C - AK8963
 * I2C1 Bus --> 0x-- - MX5611
 *
 * Motor 1 --> PB9  - TIM4 PWM CH4
 * Motor 2 --> PB8  - TIM4 PWM CH3
 * Motor 3 --> PA1  - TIM2 PWM CH2
 * Motor 4 --> PB0  - TIM3 PWM CH3
 * Motor 5 --> PB1  - TIM3 PWM CH4
 * Motor 6 --> PA11 - TIM1 PWM CH4
 *
 * White LED D1 --> PC14
 * Red   LED D2 --> PC15
 * Blue  LED D3 --> PC13
 *
 * ESP8266 --> USART1 TX/RX/BOOT0/RESET
 *
 * * * * * * * * * * * * * * * * * * * *
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
 * * * * * * * * * * * * * * * * * * * *
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "devices.h"
#include "serial.h"
#include "telemetry.h"
#include "timing.h"
#include "MPU9250.h"
#include "MS5611.h"
#include "msp.h"
#include "pid.h"
#include "circular_buffer.h"
#include "main.h"

#include "afl_call.h"

#define MULTIWII_CONF

volatile uint32_t last_tick1 = 0, last_tick2 = 0, last_tick3 = 0;
extern struct txFrame txf;
extern CircularBuffer rxc, txc;

/**********************************
 Function name	:	taskScheduler
 Functionality	:	To scheduler various tasks in a non-blocking method
 Arguments		:	None
 Return Value	:	None
 Example Call	:	taskScheduler()
 ***********************************/
void taskScheduler(void)
{
	// Compute angles
	if ((millis() - last_tick1) > 1)
	{
		/** Save Time */
		last_tick1 = millis();

		/** Tasks */

		/* Sensor Update */
		//AK8963_ReadData();	// Un-comment for mag calibration
		AHRS_ComputeAngles();
	}

	// Compute PID
	if ((millis() - last_tick3) >= 3)
	{
		/** Save Time */
		last_tick3 = millis();

		/* Control */
		PID_Update();
	}

	// Telemetry
	if ((millis() - last_tick2) > 3)
	{
		/** Save Time */
		last_tick2 = millis();

		/** Tasks */

		/* MSP Telemetry (NOTE: READ BUG LOG IN DOCUMENTATION REPORT) */
#ifdef MULTIWII_CONF
		MSP_SendIdent();
		MSP_SendStatus();
		MSP_SendMotor();
		MSP_SendAttitude();
		MSP_SendAltitude();
		MSP_SendRawIMU();
		MSP_SendPID();
#endif
	}

	/** Non-blocking tasks */

	// Compute altitude
	MS5611_Update();

	// Process MSP RX request/command frames (NOTE: READ BUG LOG IN DOCUMENTATION REPORT)
#ifndef MULTIWII_CONF
	MSP_Update();
#endif
}

/**********************************
 Function name	:	setup
 Functionality	:	To initiate all the peripherals and devices
 Arguments		:	None
 Return Value	:	None
 Example Call	:	setup()
 ***********************************/
void setup(void)
{
	Devices_Init();
	serialBegin();
	IMU_Init();
	MS5611_Init();
	PID_Init();
}

/**********************************
 Function name	:	main
 Functionality	:	Main function
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Called automatically by the processor
 ***********************************/
int main(void)
{
    startForkserver(0);

	setup();
	while (1)
	{
		HAL_IncTick();
		taskScheduler();
	}
}
