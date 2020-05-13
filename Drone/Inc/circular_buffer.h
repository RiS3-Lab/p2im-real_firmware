/*
 * circular_buffer.h
 *
 *  Created on: Jun 13, 2017
 *      Author: Heethesh
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include "stdint.h"

#define CBUF_SIZE 2048
#define CBUF_OVERWRITE 1

typedef struct
{
	volatile uint8_t  buffer[CBUF_SIZE]; 			// Block of memory
	volatile uint16_t head; 						// Holds current read position: 0 to (size-1)
	volatile uint16_t tail; 						// Holds current write position: 0 to (size-1)
	volatile uint16_t size;
}CircularBuffer;

extern CircularBuffer rxc, txc;

void CB_Init(CircularBuffer *cb);					// To initialize buffer
int CB_Write(CircularBuffer *cb, uint8_t data);		// Writes in the buffer and increment head
int CB_Read(CircularBuffer *cb, volatile uint8_t* data);		// Read in the buffer and increment tail
uint16_t CB_Size(CircularBuffer *cb); 				// Returns size of data present in buffer

#endif /* CIRCULAR_BUFFER_H_ */
