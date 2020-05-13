/*
 * devices.c
 *
 *  Created on: June 7, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "devices.h"
#include "timing.h"

// PWM <--> Motor thrust calibration
#define MOTOR1_SCALE 1
#define MOTOR2_SCALE 1
#define MOTOR3_SCALE 1
#define MOTOR4_SCALE 1

// Configuration structure declarations
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/**********************************
 Function name	:	toggleLED
 Functionality	:	To toggle the different LEDs
 Arguments		:	LED logic values
 Return Value	:	None
 Example Call	:	toggleLED(1, 0 1)
 ***********************************/
void toggleLED(int led_white, int led_red, int led_blue)
{
	HAL_GPIO_WritePin(White_LED_GPIO_Port, White_LED_Pin, !led_white);
	HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, !led_red);
	HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, !led_blue);
}

/**********************************
 Function name	:	LED_StartupSequence
 Functionality	:	LED sequence on reset
 Arguments		:	None
 Return Value	:	None
 Example Call	:	LED_StartupSequence()
 ***********************************/
void LED_StartupSequence(void)
{
	for (int i=0; i<3; i++)
	{
		toggleLED(1, 0, 0);
		delay_ms(100);

		toggleLED(0, 1, 0);
		delay_ms(100);

		toggleLED(0, 0, 1);
		delay_ms(100);
	}

	toggleLED(1, 1, 1);
}

/**********************************
 Function name	:	Motor1_SetPWM
 Functionality	:	Set PWM value for the motor
 Arguments		:	PWM Value (0-1000)
 Return Value	:	None
 Example Call	:	Motor1_SetPWM(500)
 ***********************************/
void Motor1_SetPWM(int pwm)
{
	pwm = (int)(pwm * MOTOR1_SCALE);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwm); // Motor 1
}

/**********************************
 Function name	:	Motor2_SetPWM
 Functionality	:	Set PWM value for the motor
 Arguments		:	PWM Value (0-1000)
 Return Value	:	None
 Example Call	:	Motor2_SetPWM(500)
 ***********************************/
void Motor2_SetPWM(int pwm)
{
	pwm = (int)(pwm * MOTOR2_SCALE);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm); // Motor 2
}

/**********************************
 Function name	:	Motor3_SetPWM
 Functionality	:	Set PWM value for the motor
 Arguments		:	PWM Value (0-1000)
 Return Value	:	None
 Example Call	:	Motor3_SetPWM(500)
 ***********************************/
void Motor3_SetPWM(int pwm)
{
	pwm = (int)(pwm * MOTOR3_SCALE);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm); // Motor 3
}

/**********************************
 Function name	:	Motor4_SetPWM
 Functionality	:	Set PWM value for the motor
 Arguments		:	PWM Value (0-1000)
 Return Value	:	None
 Example Call	:	Motor4_SetPWM(500)
 ***********************************/
void Motor4_SetPWM(int pwm)
{
	pwm = (int)(pwm * MOTOR4_SCALE);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm); // Motor 4
}

/**********************************
 Function name	:	Motor5_SetPWM
 Functionality	:	Set PWM value for the motor
 Arguments		:	PWM Value (0-1000)
 Return Value	:	None
 Example Call	:	Motor5_SetPWM(500)
 ***********************************/
void Motor5_SetPWM(int pwm)
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm); // Motor 5
}

/**********************************
 Function name	:	Motor6_SetPWM
 Functionality	:	Set PWM value for the motor
 Arguments		:	PWM Value (0-1000)
 Return Value	:	None
 Example Call	:	Motor6_SetPWM(500)
 ***********************************/
void Motor6_SetPWM(int pwm)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm); // Motor 6
}

/**********************************
 Function name	:	PWM_Init
 Functionality	:	Start PWM Timers
					Motor 1 --> PB9  - TIM4 PWM CH4
					Motor 2 --> PB8  - TIM4 PWM CH3
					Motor 3 --> PA1  - TIM2 PWM CH2
					Motor 4 --> PB0  - TIM3 PWM CH3
					Motor 5 --> PB1  - TIM3 PWM CH4
					Motor 6 --> PA11 - TIM1 PWM CH4
 Arguments		:	None
 Return Value	:	None
 Example Call	:	PWM_Init()
 ***********************************/
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

#ifdef HEXCOPTER
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif
}

/**********************************
 Function name	:	Devices_Init
 Functionality	:	To initialize all devices and peripherals
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Devices_Init()
 ***********************************/
void Devices_Init(void)
{
	Peripherals_Init();
	PWM_Init();
	LED_StartupSequence();
}
