/**
 ******************************************************************************
 * File Name          : stm32f1xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"

extern void _Error_Handler(char *, int);

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (hi2c->Instance == I2C1)
	{
		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
	if (hi2c->Instance == I2C1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
	}

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
	if (htim_pwm->Instance == TIM1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();
	}
	else if (htim_pwm->Instance == TIM2)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();
	}
	else if (htim_pwm->Instance == TIM3)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
	else if (htim_pwm->Instance == TIM4)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (htim->Instance == TIM1)
	{
		/**TIM1 GPIO Configuration
		 PA11     ------> TIM1_CH4
		 */
		GPIO_InitStruct.Pin = Motor_6_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(Motor_6_GPIO_Port, &GPIO_InitStruct);
	}
	else if (htim->Instance == TIM2)
	{
		/**TIM2 GPIO Configuration
		 PA1     ------> TIM2_CH2
		 */
		GPIO_InitStruct.Pin = Motor_3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(Motor_3_GPIO_Port, &GPIO_InitStruct);
	}
	else if (htim->Instance == TIM3)
	{
		/**TIM3 GPIO Configuration
		 PB0     ------> TIM3_CH3
		 PB1     ------> TIM3_CH4
		 */
		GPIO_InitStruct.Pin = Motor_4_Pin | Motor_5_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	else if (htim->Instance == TIM4)
	{
		/**TIM4 GPIO Configuration
		 PB8     ------> TIM4_CH3
		 PB9     ------> TIM4_CH4
		 */
		GPIO_InitStruct.Pin = Motor_2_Pin | Motor_1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
	if (htim_pwm->Instance == TIM1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_TIM1_CLK_DISABLE();
	}
	else if (htim_pwm->Instance == TIM2)
	{
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();
	}
	else if (htim_pwm->Instance == TIM3)
	{
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
	else if (htim_pwm->Instance == TIM4)
	{
		/* Peripheral clock disable */
		__HAL_RCC_TIM4_CLK_DISABLE();
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (huart->Instance == USART1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10    ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10    ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

		/* USART1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
}
