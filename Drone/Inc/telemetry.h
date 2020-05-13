/*
 * telemetry.h
 *
 *  Created on: Jun 4, 2017
 *      Author: Heethesh
 */

/* Library deprecated, use MSP */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

struct txFrame
{
	volatile float x;
	volatile float y;
	volatile float z;
};

void sendFrame();

#endif /* TELEMETRY_H_ */
