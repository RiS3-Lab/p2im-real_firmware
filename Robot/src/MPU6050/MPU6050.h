/*
 * MPU6050.h
 *
 *  Created on: Mar 29, 2018
 *      Author: poseidon
 */

#ifndef MPU6050_MPU6050_H_
#define MPU6050_MPU6050_H_

#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "MPU6050_defs.h"

#define A_R 4096.0 //Accelerometer scale factor. Set during initialization. Consult datasheet section 4.4 for more info.
#define G_R 65.5 //Gyroscope scale factor. Set during initialization. Consult datasheet section 4.5 for more info.

#define Fs 50 //Sampling Frequency(Hz) : 50Hz
#define Ts 20 //Sampling Period(ms) -> Ts = 1/(Fs*1000) = 20ms
#define G 0.0003053 //Constant to be multiplied with gyro readings -> G = 1/Fs/G_R


#define PITCH_SETPOINT 90.00
#define ROLL_SETPOINT 0.00

//struct used for holding IMU readings, as well as accelerometer/gyro offsets
typedef struct Mpu6050_t {
	float accX;
	float accY;
	float accZ;
	float temp;
	float gyroX;
	float gyroY;
	float gyroZ;

	float offset_gyroX;
	float offset_gyroY;

	float offset_acc_pitch;
	float offset_acc_roll;
} Mpu6050;

//platform independent function pointers that point to platform dependent
//i2c and timing functions. Used to achieve platform independence.
void (*I2C_Write_Reg)(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
void (*I2C_Read_Reg)(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
uint32_t (*Get_ms_tick)(void);

//core mpu6050 usage functions.
void mpu6050_init(Mpu6050 *mpu6050);
void mpu6050_calibrate(Mpu6050 *mpu6050);

void mpu6050_update(Mpu6050 *mpu6050);
void mpu6050_calc_pitch(Mpu6050 *mpu6050, float *pitch);
void mpu6050_calc_pitch_roll(Mpu6050 *mpu6050, float *pitch, float *roll);
void mpu6050_calc_acc_pitch_roll(Mpu6050 *mpu6050, float *pitch, float *roll);

#endif /* MPU6050_MPU6050_H_ */
