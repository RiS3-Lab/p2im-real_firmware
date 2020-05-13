/*
 * common.c
 *
 *  Created on: Jun 15, 2017
 *      Author: Heethesh
 */

#include "common.h"

/**********************************
 Function name	:	map
 Functionality	:	Re-maps a number from one range to another. That is, a value
 	 	 	 	 	of in_min would get mapped to out_min, a value of in_max to
 	 	 	 	 	out_max, values in-between to values in-between
 Arguments		:	Value, input range, output range
 Return Value	:	The mapped value
 Example Call	:	map(10, 0, 1000, 1000, 2000)
 ***********************************/
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**********************************
 Function name	:	mapfloat
 Functionality	:	Re-maps a number from one range to another. That is, a value
 	 	 	 	 	of in_min would get mapped to out_min, a value of in_max to
 	 	 	 	 	out_max, values in-between to values in-between
 Arguments		:	Value, input range, output range
 Return Value	:	The mapped value
 Example Call	:	mapfloat(10.23, 0, 20, 25, 50.5)
 ***********************************/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**********************************
 Function name	:	lowPassFilter
 Functionality	:	To get smooth sensor readings
 Arguments		:	LPF struct, new sensor reading
 Return Value	:	Filtered value
 Example Call	:	lowPassFilter(&angle, 22)
 ***********************************/
float lowPassFilter(struct LPF *var, float current)
{
	// Filter the new reading
	current = ((1 - var->beta) * current) + (var->beta * var->last);

	// Store current reading for next iteration
	var->last = current;

	// Return filtered value
	return current;
}
