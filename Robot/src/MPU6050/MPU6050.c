/*
 * MPU6050.c
 *
 *  Created on: Mar 29, 2018
 *      Author: poseidon
 */
#include "MPU6050.h"

/*-------------------------------------------------------------------
 *	function that initializes the mpu6050 upon first use. Writes a
 *	series of 8 bit values into the registers PWR_MGMT_1, ACCEL_CONFIG,
 *	and GYRO_CONFIG which configure the IMU sampling rate, accelerometer
 *	scale, and gyro scale, respectively. Finally, it initializes an
 *	mpu6050 typedef to zero.
 *
 *	params:
 *
 *	*mpu6050 -- pointer to an Mpu6050 struct that holds sensor data.
 *	Members of this typedef are initialized to zero in this function.
 *------------------------------------------------------------------*/
void mpu6050_init(Mpu6050 *mpu6050) {
	I2C_Write_Reg(MPU6050, PWR_MGMT_1, BYTE(0x00), 1);
	I2C_Write_Reg(MPU6050, ACCEL_CONFIG, BYTE(0x10), 1);
	I2C_Write_Reg(MPU6050, GYRO_CONFIG, BYTE(0x08), 1);

	mpu6050->accX = 0;
	mpu6050->accY = 0;
	mpu6050->accZ = 0;
	mpu6050->temp = 0;
	mpu6050->gyroX = 0;
	mpu6050->gyroY = 0;
	mpu6050->gyroZ = 0;

	mpu6050->offset_gyroX = 0;
	mpu6050->offset_gyroY = 0;
	mpu6050->offset_acc_pitch = 0;
	mpu6050->offset_acc_roll = 0;

	mpu6050_calibrate(mpu6050);
}

/*-------------------------------------------------------------------
 *	function that calibrates the IMU upon boot. The mpu6050 is to be
 *	placed upon a level surface, and then turned on. This function
 *	will then calculate offsets for the gyroscope and accelerometer
 *	by reading and taking the average of a number of readings (given
 *	by the variable num_samples) at the set sampling rate. Further
 *	calculations are then compensated for using these offset values
 *
 *	params:
 *
 *	*mpu6050 -- pointer to an Mpu6050 struct that holds sensor data.
 *	The offset members of this typedef are set in this function.
 *------------------------------------------------------------------*/
void mpu6050_calibrate(Mpu6050 *mpu6050) {
	//number of samples to take
	int num_samples = 1;

	//local variables used for calculating offsets.
	float offset_gyroX = 0;
	float offset_gyroY = 0;
	float offset_accPitch = 0;
	float offset_accRoll = 0;

	uint32_t now = Get_ms_tick();
	for (int i = 0; i < num_samples; i++) {
		//variables that will hold accelerometer pitch/roll values
		//for each iteration.
		float acc_pitch = 0;
		float acc_roll = 0;

		//update sensor readings
		mpu6050_update(mpu6050);
		//calculate pitch/roll using only accelerometer
		mpu6050_calc_acc_pitch_roll(mpu6050, &acc_pitch, &acc_roll);

		//increment offset values using calculated values
		offset_accPitch += acc_pitch;
		offset_accRoll += acc_roll;
		offset_gyroX += mpu6050->gyroX;
		offset_gyroY += mpu6050->gyroY;

		//wait till sampling period has elapsed, and repeat
		//while (Get_ms_tick() - now < Ts) {
		//}
		now = Get_ms_tick();
	}

	//take the average of the total readings. Store that value
	//in the mpu6050 typedef
	mpu6050->offset_gyroX = offset_gyroX / (float) num_samples;
	mpu6050->offset_gyroY = offset_gyroY / (float) num_samples;
	mpu6050->offset_acc_pitch = offset_accPitch / (float) num_samples;
	mpu6050->offset_acc_roll = offset_accRoll / (float) num_samples;
}

/*-------------------------------------------------------------------
 *	function that updates IMU readings into an Mpu6050 struct
 *
 *	params:
 *
 *	*mpu6050 -- pointer to an Mpu6050 struct that holds sensor data.
 *------------------------------------------------------------------*/
void mpu6050_update(Mpu6050 *mpu6050) {

	//Signed 16 bit values that will hold IMU readings.
	int16_t ax, ay, az, t, gx, gy, gz;

	//8 bit wide buffer that will be used to read raw IMU readings
	uint8_t raw[14];

	I2C_Read_Reg(MPU6050, ACCEL_XOUT_H, raw, 14);

	ax = (raw[0] << 8) | (raw[1]);
	ay = (raw[2] << 8) | (raw[3]);
	az = (raw[4] << 8) | (raw[5]);
	t = (raw[6] << 8) | (raw[7]);
	gx = (raw[8] << 8) | (raw[9]);
	gy = (raw[10] << 8) | (raw[11]);
	gz = (raw[12] << 8) | (raw[13]);

	mpu6050->accX = (float) ax;
	mpu6050->accY = (float) ay;
	mpu6050->accZ = (float) az;
	mpu6050->temp = (float) t;
	mpu6050->gyroX = (float) gx;
	mpu6050->gyroY = (float) gy;
	mpu6050->gyroZ = (float) gz;
}

/*-------------------------------------------------------------------
 *	calculate only pitch angle using a complimentary filter
 *	that fuses both accelerometer and gyroscope readings.
 *
 *	params:
 *
 *	*mpu6050 -- pointer to an Mpu6050 struct that holds sensor data
 *
 *	*pitch -- pointer to a float that will hold the calculated pitch angle
 *
 *-----------------------------------------------------------------*/
void mpu6050_calc_pitch(Mpu6050 *mpu6050, float *pitch) {
	//bool that keeps track of whether or not this is the first iteration
	static bool first_run = true;

	//total pitch/ calculations
	static float total_pitch = 0;

	//get gyro's current x value, and subtract an offset
	float gyro_x = mpu6050->gyroX - mpu6050->offset_gyroX;

	//integrate gyro rates into the total pitch calculation. G = 1/G_R/Fs
	total_pitch += gyro_x * (G);

	//calculate pitch using accelerometer readings alone, minus an offset plus
	//desired setpoint(in this case, 90 degrees)
	float acc_pitch = (180 * atan2(mpu6050->accY / A_R, mpu6050->accZ / A_R)
			/ M_PI) - (mpu6050->offset_acc_pitch) + PITCH_SETPOINT;

	//if this is the first iteration, simply set pitch
	//to accelerometer's pitch
	if (first_run) {
		total_pitch = acc_pitch;

		first_run = false;
	}
	//else apply complimentary filter using gyro/accelerometer pitch
	//calculations
	else {
		total_pitch = total_pitch * .8 + acc_pitch * .2;
	}

	//dereference pitch pointer, set literal value to calculated
	//pitch angle
	*pitch = total_pitch;
}

/*-------------------------------------------------------------------
 *	calculate a pitch and a roll angle using a complimentary filter
 *	that fuses both accelerometer and gyroscope readings.
 *
 *	params:
 *
 *	*mpu6050 -- pointer to an Mpu6050 struct that holds sensor data
 *
 *	*pitch -- pointer to a float that will hold the calculated pitch angle
 *
 *	*roll -- pointer to a float that will hold the calculated roll angle
 *-----------------------------------------------------------------*/
void mpu6050_calc_pitch_roll(Mpu6050 *mpu6050, float *pitch, float *roll) {
	//bool that keeps track of whether or not this is the first iteration
	static bool first_run = true;

	//total pitch/roll calculations
	static float total_pitch = 0;
	static float total_roll = 0;

	//get gyro's current x/y values, and subtract an offset
	float gyro_x = mpu6050->gyroX - mpu6050->offset_gyroX;
	float gyro_y = mpu6050->gyroY - mpu6050->offset_gyroY;

	//integrate gyro rates into the total pitch/roll calculation. G = 1/G_R/Fs
	total_pitch += gyro_x * (G);
	total_roll += gyro_y * (G);

	//calculate pitch/roll using accelerometer readings alone, minus an offset
	float acc_pitch = (180 * atan2(mpu6050->accY / A_R, mpu6050->accZ / A_R)
			/ M_PI) - (mpu6050->offset_acc_pitch) + 90;
	float acc_roll = (180 * atan2(mpu6050->accX / A_R, mpu6050->accZ / A_R)
			/ M_PI) - (mpu6050->offset_acc_roll);

	//if this is the first iteration, simply set pitch/roll
	//to accelerometer's pitch/roll
	if (first_run) {
		total_pitch = acc_pitch;
		total_roll = acc_roll;

		first_run = false;
	}
	//else apply complimentary filter using gyro/accelerometer pitch/roll
	//calculations
	else {
		total_pitch = total_pitch * .8 + acc_pitch * .2;
		total_roll = total_roll * .8 + acc_roll * .2;
	}

	//dereference pitch and roll pointers, set literal value to calculated
	//pitch and roll angles
	*pitch = total_pitch;
	*roll = total_roll;
}

/*-------------------------------------------------------------------
 *	Calculate pitch and roll angles(in degrees) using accelerometer
 *	readings alone. Used during gyro calculation, but can be used
 *	on its own if one does not wish to use the gyro.
 *
 *	params:
 *
 *	*mpu6050 -- pointer to an Mpu6050 struct that holds sensor data
 *
 *	*pitch -- pointer to a float that will hold the calculated pitch
 *	angle
 *
 *	*roll -- pointer to a float that will hold the calculated roll
 *	angle
 *
 *-----------------------------------------------------------------*/
void mpu6050_calc_acc_pitch_roll(Mpu6050 *mpu6050, float *pitch, float *roll) {
	float acc_x, acc_y, acc_z;
	float acc_pitch, acc_roll;

	//normalized accelerometer readings. Constructed by taking
	//raw accelerometer readings and diving by accelerometer scaling
	//factor.
	acc_x = (mpu6050->accX) / A_R;
	acc_y = (mpu6050->accY) / A_R;
	acc_z = (mpu6050->accZ) / A_R;

	//calculated pitch and roll angles
	acc_pitch = 180 * atan2(acc_y, acc_z) / M_PI;
	acc_roll = 180 * atan2(acc_x, acc_z) / M_PI;

	//dereference pitch and roll pointers, set literal value to calculated
	//pitch and roll angles
	*pitch = acc_pitch;
	*roll = acc_roll;
}

