/*
 * MPU9250.c
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "msp.h"
#include "MadgwickAHRS.h"
#include "MPU9250.h"
#include "common.h"
#include <math.h>
#include "serial.h"
#include "timing.h"
#include "i2c.h"

//#define IMU_DEBUG
#define sq(x) ((x)*(x))
#define _USE_MATH_DEFINES

extern msp_status msp_txf_status;
extern msp_raw_imu msp_txf_raw_imu;
extern msp_attitude msp_txf_attitude;

uint32_t AHRS_lastUpdate = 0, AHRS_timeNow = 0;

float AHRS_Angle[3] = {0, 0, 0};
float mRes = 10.0*4912.0/32760.0;

struct IMUData
{
	float x;
	float y;
	float z;
}accelData, gyroData, magData, magCalib, magBias = {251.335, 437.810, -456.283},
gyroBias = {0.084, 0.132, 0.019}, magScale = {1.021, 0.983, 0.997};

struct IMURaw
{
	int16_t x;
	int16_t y;
	int16_t z;
}accelRaw, gyroRaw, magRaw;

struct LPF lpf_pitch = {0, 0.7}, lpf_roll = {0, 0.7}, lpf_yaw = {0, 0};

/**********************************
 Function name	:	MPU9250_Init
 Functionality	:	To setup the Accelerometer and Gyroscope
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MPU9250_Init()
 ***********************************/
void MPU9250_Init(void)
{
	// Verify device
	uint8_t data = I2C_ReadByte(MPU9250_ADDRESS, WHO_AM_I, __FILE__, __LINE__);
	//if (data != WHO_AM_I_VALUE) _Error_Handler(__FILE__, __LINE__);

	// Device configuration
	I2C_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x80, 1);	// Reset
	I2C_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x01, 1);	// Set clock source to be PLL with x-axis gyroscope reference
	I2C_WriteByte(MPU9250_ADDRESS, MPU_PWR_MGMT_2, 0x00, 1);	// Enable Accel and Gyro
	I2C_WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 	   0x00, 1);	// Sample Rate Divider (Not set)
	I2C_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2,  0x03, 1);	// DLPF 184Hz
	I2C_WriteByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03, 1);	// DLPF 184Hz

	// Full scale settings
	I2C_WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_1000_DPS, 1);
	I2C_WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACC_FS_4_G, 1);
}

/**********************************
 Function name	:	MPU9250_ReadAccelData
 Functionality	:	To read the raw data from the accelerometer and convert it G-units
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MPU9250_ReadAccelData()
 ***********************************/
void MPU9250_ReadAccelData(void)
{
	// Data buffer
	uint8_t raw_data[] = {0, 0, 0, 0, 0, 0};

	// Read raw data
	I2C_ReadByteArray(MPU9250_ADDRESS, ACCEL_XOUT_H, raw_data, 6, __FILE__, __LINE__);

	// Pack to integer
	accelRaw.x = (int16_t) ((raw_data[0]<<8) | raw_data[1]);
	accelRaw.y = (int16_t) ((raw_data[2]<<8) | raw_data[3]);
	accelRaw.z = (int16_t) ((raw_data[4]<<8) | raw_data[5]);

	// Scale to G-units
	accelData.x = (float) accelRaw.x * 4.0f/32768.0f;
	accelData.y = (float) accelRaw.y * 4.0f/32768.0f;
	accelData.z = (float) accelRaw.z * 4.0f/32768.0f;

#ifdef IMU_DEBUG
	serialFloat(accelData.x);
	serialWrite('\t');
	serialFloat(accelData.y);
	serialWrite('\t');
	serialFloat(accelData.z);
	serialWrite('\n');
#endif
}

/**********************************
 Function name	:	MPU9250_ReadGyroData
 Functionality	:	To read the raw data from the gyroscope and convert it DPS
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MPU9250_ReadGyroData()
 ***********************************/
void MPU9250_ReadGyroData(void)
{
	// Data buffer
	uint8_t raw_data[] = {0, 0, 0, 0, 0, 0};

	// Read raw data
	I2C_ReadByteArray(MPU9250_ADDRESS, GYRO_XOUT_H, raw_data, 6, __FILE__, __LINE__);

	// Pack to integer
	gyroRaw.x = (int16_t) ((raw_data[0]<<8) | raw_data[1]);
	gyroRaw.y = (int16_t) ((raw_data[2]<<8) | raw_data[3]);
	gyroRaw.z = (int16_t) ((raw_data[4]<<8) | raw_data[5]);

	// Scale to DPS
	gyroData.x = ((float) gyroRaw.x * 1000.0f/32768.0f) - gyroBias.x;
	gyroData.y = ((float) gyroRaw.y * 1000.0f/32768.0f) - gyroBias.y;
	gyroData.z = ((float) gyroRaw.z * 1000.0f/32768.0f) - gyroBias.z;

#ifdef IMU_DEBUG
	serialFloat(gyroData.x);
	serialWrite('\t');
	serialFloat(gyroData.y);
	serialWrite('\t');
	serialFloat(gyroData.z);
	serialWrite('\n');
#endif
}

/**********************************
 Function name	:	MPU9250_GetGyroData
 Functionality	:	Returns the converted gyroscope readings
 Arguments		:	Data array pointer
 Return Value	:	None
 Example Call	:	MPU9250_GetGyroData()
 ***********************************/
void MPU9250_GetGyroData(float* data)
{
	data[0] = gyroData.y;
	data[1] = gyroData.x;
	data[2] = -gyroData.z;
}

/**********************************
 Function name	:	AK8963_Init
 Functionality	:	To setup the magnetometer
 Arguments		:	None
 Return Value	:	None
 Example Call	:	AK8963_Init()
 ***********************************/
void AK8963_Init(void)
{
	// Enable access to Magnetometer via MPU
	I2C_WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22, 1);	// Set bypass mode for external I2C master connection
	I2C_WriteByte(MPU9250_ADDRESS, USER_CTRL,   0x01, 1); 	// Disable master mode and clear all signal paths

	// Verify magnetometer
	uint8_t data = I2C_ReadByte(MAG_ADDRESS, MAG_WIA, __FILE__, __LINE__);
	//if (data != MAG_WIA_VALUE) _Error_Handler(__FILE__, __LINE__);

	I2C_WriteByte(MAG_ADDRESS, MAG_CNTL2, 0x01, 1);		// Reset magnetometer
	I2C_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00, 1);		// Power down magnetometer
	I2C_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x0F, 1); 	// Enter Fuse ROM access mode

	// Read factory calibration registers
	uint8_t rawData[3];
	I2C_ReadByteArray(MAG_ADDRESS, MAG_ASAX, rawData, 3, __FILE__, __LINE__);

	// Calibrate magnetometer factory offset
	magCalib.x =  (float)(rawData[0] - 128)/256.0f + 1.0f;	// Return x-axis sensitivity adjustment values
	magCalib.y =  (float)(rawData[1] - 128)/256.0f + 1.0f;	// Return y-axis sensitivity adjustment values
	magCalib.z =  (float)(rawData[2] - 128)/256.0f + 1.0f;	// Return z-axis sensitivity adjustment values

#ifdef IMU_DEBUG
	serialFloat(magCalib.x);
	serialWrite('\t');
	serialFloat(magCalib.y);
	serialWrite('\t');
	serialFloat(magCalib.z);
	serialWrite('\n');
#endif

	// Magnetometer settings
	I2C_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x00, 1);		// Power down magnetometer
	I2C_WriteByte(MAG_ADDRESS, MAG_CNTL1, 0x16, 1); 	// Res: 16 Bit, Mode: Continuous Mode 2 (100Hz)
}

/**********************************
 Function name	:	AK8963_ReadData
 Functionality	:	To read the raw data from the magnetometer and convert it milliGauss
 Arguments		:	None
 Return Value	:	None
 Example Call	:	AK8963_ReadData()
 ***********************************/
void AK8963_ReadData(void)
{
	uint8_t raw_data[] = {0, 0, 0, 0, 0, 0, 0};

	// Check if data is ready
	if (I2C_ReadByte(MAG_ADDRESS, MAG_ST1, __FILE__, __LINE__) & 0x01)
	{
		// Read data registers and ST2 register to check overflow
		I2C_ReadByteArray(MAG_ADDRESS, MAG_HXL, raw_data, 7, __FILE__, __LINE__);
		uint8_t OVF = raw_data[6];

		// Store data if no overflow occurred
		if (!(OVF & 0x08))
		{
			// Pack into 16-bit integer
			magRaw.x = (int16_t) ((raw_data[1]<<8) | raw_data[0]);
			magRaw.y = (int16_t) ((raw_data[3]<<8) | raw_data[2]);
			magRaw.z = (int16_t) ((raw_data[5]<<8) | raw_data[4]);

			// Apply the calibration and conversion factors
			magData.x = (((float) magRaw.x * mRes * magCalib.x) - magBias.x) * magScale.x;
			magData.y = (((float) magRaw.y * mRes * magCalib.y) - magBias.y) * magScale.y;
			magData.z = (((float) magRaw.z * mRes * magCalib.z) - magBias.z) * magScale.z;

#ifdef IMU_DEBUG
			serialFloat(magData.x);
			serialWrite(',');
			serialFloat(magData.y);
			serialWrite(',');
			serialFloat(magData.z);
			serialWrite('\n');
#endif
		}

		else
		{
			msp_txf_status.i2c_errors_count++;
			//serialPrint("\n* Magnetometer Overflow *\n;");
		}
	}
}

/**********************************
 Function name	:	IMU_Init
 Functionality	:	Initialize the IMU and AHRS filter  parameters
 Arguments		:	None
 Return Value	:	None
 Example Call	:	IMU_Init()
 ***********************************/
void IMU_Init(void)
{
	MPU9250_Init();
	AK8963_Init();

	//float GyroMeasError = M_PI * (60.0f / 180.0f);	// Gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;	// Compute beta

	// Higher the beta value, depends more on noisy readings from accelerometer, faster response
	// Lower the value, smooth readings, but slower response
	MadgwickSetBeta(0.6f);
	MadgwickSetDelta(0.0f);
}

/**********************************
 Function name	:	AHRS_GetPitch
 Functionality	:	Returns the pitch angle
 Arguments		:	None
 Return Value	:	Pitch angle
 Example Call	:	AHRS_GetPitch()
 ***********************************/
float AHRS_GetPitch(void)
{
	return lowPassFilter(&lpf_pitch, AHRS_Angle[1]);
}

/**********************************
 Function name	:	AHRS_GetRoll
 Functionality	:	Returns the roll angle
 Arguments		:	None
 Return Value	:	Roll angle
 Example Call	:	AHRS_GetRoll()
 ***********************************/
float AHRS_GetRoll(void)
{
	return lowPassFilter(&lpf_roll, AHRS_Angle[0]);
}

/**********************************
 Function name	:	AHRS_GetYaw
 Functionality	:	Returns the yaw angle
 Arguments		:	None
 Return Value	:	Yaw angle
 Example Call	:	AHRS_GetYaw()
 ***********************************/
float AHRS_GetYaw(void)
{
	// Shift range to North heading
	float angle = AHRS_Angle[2];
	return ((angle >= -180) && (angle < 90)) ? (angle + 90) : (angle - 270);
}

/**********************************
 Function name	:	AHRS_ComputeAngles
 Functionality	:	Update the 9DOF data from IMU and compute pitch, roll and yaw
  	  	  	  	  	using Madgwick's AHRS filter
 Arguments		:	None
 Return Value	:	None
 Example Call	:	AHRS_ComputeAngles()
 ***********************************/
void AHRS_ComputeAngles(void)
{
	// Read converted IMU data
	MPU9250_ReadAccelData();
	MPU9250_ReadGyroData();
	AK8963_ReadData();

	// Set integration time by time elapsed since last filter update in seconds
	AHRS_timeNow = micros();
	float delta = (float)((AHRS_timeNow - AHRS_lastUpdate)/1000.0f) ;
	MadgwickSetDelta(delta);
	AHRS_lastUpdate = AHRS_timeNow;

	// Filter data and obtain the angles
	MadgwickQuaternionUpdate(-accelData.y, -accelData.x, accelData.z, gyroData.y,
			gyroData.x, -gyroData.z, magData.x,	magData.y, magData.z, AHRS_Angle);

	/* Update MSP frame */

	// Update raw IMU MSP frame
	msp_txf_raw_imu.accx = accelRaw.x / 100;
	msp_txf_raw_imu.accy = accelRaw.y / 100;
	msp_txf_raw_imu.accz = accelRaw.z / 100;

	msp_txf_raw_imu.gyrx = gyroRaw.x;
	msp_txf_raw_imu.gyry = gyroRaw.y;
	msp_txf_raw_imu.gyrz = gyroRaw.z;

	msp_txf_raw_imu.magx = magRaw.x;
	msp_txf_raw_imu.magy = magRaw.y;
	msp_txf_raw_imu.magz = magRaw.z;

	// Update RC MSP frame
	msp_txf_attitude.angx = AHRS_GetRoll() * 10;	// Multiply by 10 -> Required by MSP
	msp_txf_attitude.angy = AHRS_GetPitch() * 10;	// Multiply by 10 -> Required by MSP
	msp_txf_attitude.heading = AHRS_GetYaw();
}
