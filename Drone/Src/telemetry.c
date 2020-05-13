/*
 * telemetry.c
 *
 *  Created on: Jun 4, 2017
 *      Author: Heethesh
 */

/* Library deprecated, use MSP Debug Frame*/

#include <serial.h>
#include "telemetry.h"
#include <string.h>

struct txFrame txf;

/**********************************
 Function name	:	sendFrame
 Functionality	:	To send a frame of data
 	 	 	 	 	Format '<' <DATA> <CRC> '>'
 	 	 	 	 	Use MultiWii Serial Protocol debug frames, this method is deprecated
 Arguments		:	None
 Return Value	:	None
 Example Call	:	sendFrame()
 ***********************************/
void sendFrame(void)
{
	unsigned char checksum = 0;
	unsigned char txBuffer[12];

	// Start frame limiter
	serialWrite('<');

	// Convert struct elements to byte array
	memcpy(txBuffer, &txf, 12);

	// Transmit data payload and update checksum
	for (int i=0; i<12; i++)
	{
		checksum ^= txBuffer[i];
		serialWrite(txBuffer[i]);
	}

	// Calculate checksum
	serialWrite(checksum);

	// End frame limiter
	serialWrite('>');
}
