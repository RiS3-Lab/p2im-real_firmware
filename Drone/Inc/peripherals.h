#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "stm32f1xx_hal.h"

/* Definitions */
#define UART1_BAUD 115200

// LED Pins
#define Blue_LED_Pin GPIO_PIN_13
#define Blue_LED_GPIO_Port GPIOC
#define White_LED_Pin GPIO_PIN_15
#define White_LED_GPIO_Port GPIOC
#define Red_LED_Pin GPIO_PIN_14
#define Red_LED_GPIO_Port GPIOC

// Motor Pins
#define Motor_1_Pin GPIO_PIN_9
#define Motor_1_GPIO_Port GPIOB
#define Motor_2_Pin GPIO_PIN_8
#define Motor_2_GPIO_Port GPIOB
#define Motor_3_Pin GPIO_PIN_1
#define Motor_3_GPIO_Port GPIOA
#define Motor_4_Pin GPIO_PIN_0
#define Motor_4_GPIO_Port GPIOB
#define Motor_5_Pin GPIO_PIN_1
#define Motor_5_GPIO_Port GPIOB
#define Motor_6_Pin GPIO_PIN_11
#define Motor_6_GPIO_Port GPIOA

/* Functions */
void Peripherals_Init(void);
void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif /* PERIPHERALS_H_ */
