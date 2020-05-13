/*
 * common.h
 *
 *  Created on: Jun 15, 2017
 *      Author: Heethesh
 */

#ifndef COMMON_H_
#define COMMON_H_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(x) ((x)>0?(x):-(x))

struct LPF
{
	float last;
	float beta;
};

long map(long x, long in_min, long in_max, long out_min, long out_max);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
float lowPassFilter(struct LPF *var, float current);

#endif /* COMMON_H_ */
