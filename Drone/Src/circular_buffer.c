/*
 * circular_buffer.c
 *
 *  Created on: June 13, 2017
 *      Author: Heethesh
 */

#include <stdint.h>
#include "circular_buffer.h"

CircularBuffer rxc, txc;

/**********************************
 Function name	:	CB_Write
 Functionality	:	To put a data into the FIFO
 Arguments		:	CircularBuffer struct, Data to be written
 Return Value	:	Error status
 Example Call	:	CB_Write(&rxc, data)
 ***********************************/
int CB_Write(CircularBuffer *cb, uint8_t data)
{
	// Buffer overrun handling and return failure
	if ((cb->size >= CBUF_SIZE) && !CBUF_OVERWRITE) return 0;

	// Increment current FIFO size
	cb->size++;

	// Add data to FIFO
	cb->buffer[cb->tail] = data;

	// Update tail
	cb->tail = (cb->tail+1) % CBUF_SIZE;

	// Return success
	return 1;
}

/**********************************
 Function name	:	CB_Read
 Functionality	:	To pop a data from the FIFO
 Arguments		:	CircularBuffer struct, Popped data pointer
 Return Value	:	Error status
 Example Call	:	CB_Read(&rxc, &data)
 ***********************************/
int CB_Read(CircularBuffer *cb, volatile uint8_t *data)
{
	// Empty buffer
	if (cb->size == 0) return 0;

	// Decrement current FIFO size
	cb->size--;

	// Pop to data pointer
	*data = cb->buffer[cb->head];

	// Update head
	cb->head = (cb->head+1) % CBUF_SIZE;

	// Return success
	return 1;
}

/**********************************
 Function name	:	CB_Size
 Functionality	:	Return current buffer size
 Arguments		:	CircularBuffer struct
 Return Value	:	Current buffer size
 Example Call	:	CB_Size(&rxc)
 ***********************************/
uint16_t CB_Size(CircularBuffer *cb)
{
	return (cb->size);
}

/**********************************
 Function name	:	CB_Init
 Functionality	:	Reset FIFO parameters and clear buffer
 Arguments		:	CircularBuffer struct
 Return Value	:	None
 Example Call	:	CB_Init(&rxc)
 ***********************************/
void CB_Init(CircularBuffer *cb)
{
	// Reset FIFO parameters
	cb->head = 0;
	cb->tail = 0;
	cb->size = 0;

	// Clear buffer
	for (int i=0; i<CBUF_SIZE; i++)
		cb->buffer[i] = 0;
}
