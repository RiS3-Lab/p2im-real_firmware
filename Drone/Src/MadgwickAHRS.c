#include "MadgwickAHRS.h"
#include <math.h>

#define _USE_MATH_DEFINES

volatile float beta, deltat;				// Algorithm gain and gyro integration time
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };	// Quaternion of sensor frame relative to auxiliary frame

/**********************************
 Function name	:	MadgwickSetBeta
 Functionality	:	Set beta gain
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MadgwickSetBeta(0.6)
 ***********************************/
void MadgwickSetBeta(float _beta)
{
	beta = _beta;
}

/**********************************
 Function name	:	MadgwickSetDelta
 Functionality	:	Set integration time delta in seconds
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MadgwickSetDelta(0.1)
 ***********************************/
void MadgwickSetDelta(float _deltat)
{
	deltat = _deltat;
}

/**********************************
 Function name	:	MadgwickQuaternionUpdate
 Functionality	:	Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
 	 	 	 	 	which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
 	 	 	 	 	device orientation -- which can be converted to yaw, pitch, and roll. The performance of the orientation filter
 	 	 	 	 	is at least as good as conventional Kalman-based filtering algorithms but is much less computationally intensive.

 	 	 	 	 	NOTE:
 	 	 	 	 	ACCEL IN G-UNITS
 	 	 	 	 	GYRO RATE IN DPS
 	 	 	 	 	MAG IN MILLIGAUSS

 Arguments		:	Acceleration, gyroscope rates and magnetic moments of all 3 axes
 	 	 	 	 	Array to store computed angles
 Return Value	:	None
 Example Call	:	MadgwickQuaternionUpdate(-accelData.y, -accelData.x, accelData.z, gyroData.y,
					gyroData.x, -gyroData.z, magData.x,	magData.y, magData.z, AHRS_Angle)
 ***********************************/
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz, float *angle)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	/* Convert DPS to RPS */
	gx *= M_PI / 180;
	gy *= M_PI / 180;
	gz *= M_PI / 180;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalize accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalize magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
			+ _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
			+ my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
			+ _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q2 * (2.0f * q1q2 + _2q3q4 - ay)
			- _2bz * q3
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q4 + _2bz * q2)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q3
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ _2bz * q4
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q3 + _2bz * q1)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q4 - _4bz * q2)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q4 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ (-_4bx * q3 - _2bz * q1)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q2 + _2bz * q4)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q1 - _4bz * q3)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay)
			+ (-_4bx * q4 + _2bz * q2)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q1 + _2bz * q3)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q2
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // Normalize step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);  // Normalize quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

	// Calculate pitch, roll and yaw
	angle[0] = asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180 / M_PI;
	angle[1] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180 / M_PI;
	angle[2] = (atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180 / M_PI);
}
