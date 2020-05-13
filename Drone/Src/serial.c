/*
 * serial_new.c
 *
 *  Created on: Jun 12, 2017
 *      Author: Heethesh
 *
 * Transmit --> Blocking
 * Receive 	--> Non-Blocking (Interrupts) + FIFO
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "msp.h"
#include "circular_buffer.h"
#include "serial.h"
#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern CircularBuffer rxc;

unsigned char rx_buffer[2];
char tx_buffer[100];

/**********************************
 Function name	:	serialAvailable
 Functionality	:	To get receive buffer size
 Arguments		:	None
 Return Value	:	Error state
 Example Call	:	serialAvailable()
 ***********************************/
int serialAvailable(void)
{
	return (CB_Size(&rxc));
}

/**********************************
 Function name	:	serialWrite
 Functionality	:	To send a byte of data
 Arguments		:	Byte to send
 Return Value	:	None
 Example Call	:	serialWrite('H')
 ***********************************/
void serialWrite(unsigned char ch)
{
	tx_buffer[0] = ch;
	HAL_UART_Transmit(&huart1, (uint8_t*) &tx_buffer, 1, 5);

	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/**********************************
 Function name	:	serialRead
 Functionality	:	To read a byte of data from the FIFO buffer
 Arguments		:	None
 Return Value	:	Read data
 Example Call	:	serialRead()
 ***********************************/
unsigned char serialRead()
{
	unsigned char data;
	CB_Read(&rxc, &data);
	return data;
}

/**********************************
 Function name	:	HAL_UART_RxCpltCallback
 Functionality	:	This is the RX UART Interrupt callback function implementation
 Arguments		:	UART struct instance
 Return Value	:	None
 Example Call	:	HAL_UART_RxCpltCallback(&huart1)
 ***********************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// Put the received data into the FIFO buffer
		CB_Write(&rxc, rx_buffer[0]);

		// Enable RX interrupt flags to receive the next data
		HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

/**********************************
 Function name	:	serialFlush
 Functionality	:	To clear the FIFO buffers
 Arguments		:	None
 Return Value	:	None
 Example Call	:	serialFlush()
 ***********************************/
void serialFlush(void)
{
	CB_Init(&rxc);
}

/**********************************
 Function name	:	serialBegin
 Functionality	:	To reset FIFO buffer and enable UART interrupt flags
 Arguments		:	None
 Return Value	:	None
 Example Call	:	serialBegin()
 ***********************************/
void serialBegin(void)
{
	serialFlush();
	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
}

/**********************************
 Function name	:	serialPrint
 Functionality	:	To send a stream of bytes
 Arguments		:	Data stream
 Return Value	:	None
 Example Call	:	serialPrint("Hola Monde!")
 ***********************************/
void serialPrint(char* data)
{
	for (int i = 0; i < strlen(data); i++)
		serialWrite(data[i]);
}

/**********************************
 Function name	:	serialInt
 Functionality	:	To print a integer value on a serial terminal (digit-wise)
 Arguments		:	Integer value
 Return Value	:	None
 Example Call	:	serialInt()
 ***********************************/
void serialInt(int val)
{
	sprintf(tx_buffer, "%i", val);
	serialPrint(tx_buffer);
}

/**********************************
 Function name	:	serialFloat
 Functionality	:	To print a float value on a serial terminal (digit-wise)
 Arguments		:	Float value
 Return Value	:	None
 Example Call	:	serialFloat()
 ***********************************/
void serialFloat(float val)
{
	// Get sign
	char *tmpSign = (val < 0) ? "-" : "";
	float tmpVal = (val < 0) ? -val : val;

	int tmpInt1 = tmpVal;                  // Get the integer (678)
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123)
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123)

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf(tx_buffer, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
	serialPrint(tx_buffer);
}
