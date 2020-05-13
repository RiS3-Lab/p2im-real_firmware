/*
 * MPU9250.h
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

#ifndef MPU9250_H_
#define MPU9250_H_

/* Definitions */

// Device Addresses
#define MPU9250_ADDRESS		0x68 << 1
#define MAG_ADDRESS			0x0C << 1

// MPU Settings Registers
#define WHO_AM_I			0x75
#define WHO_AM_I_VALUE		0x71

#define SMPLRT_DIV			0x19
#define MPU9250_CONFIG		0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define ACCEL_CONFIG2		0x1D

#define USER_CTRL			0x6A
#define MPU_PWR_MGMT_1		0x6B
#define MPU_PWR_MGMT_2		0x6C

#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV0_DO			0x63
#define INT_PIN_CFG			0x37

// MPU Output Registers
#define ACCEL_XOUT_H 		0x3B
#define ACCEL_XOUT_L 		0x3C
#define ACCEL_YOUT_H 		0x3D
#define ACCEL_YOUT_L 		0x3E
#define ACCEL_ZOUT_H 		0x3F
#define ACCEL_ZOUT_L 		0x40

#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48

// MPU Scale Settings
#define GYRO_FS_250_DPS		0x00
#define GYRO_FS_500_DPS		0x08
#define GYRO_FS_1000_DPS	0x10
#define GYRO_FS_2000_DPS	0x18

#define ACC_FS_2_G			0x00
#define ACC_FS_4_G			0x08
#define ACC_FS_8_G			0x10
#define ACC_FS_16_G			0x18

// Magnetometer Registers
#define MAG_WIA				0x00
#define MAG_WIA_VALUE		0x48

#define MAG_CNTL1			0x0A
#define MAG_CNTL2			0x0B

#define MAG_ST1				0x02
#define MAG_ST2				0x09

#define MAG_HXL				0x03
#define MAG_HXH				0x04
#define MAG_HYL				0x05
#define MAG_HYH				0x06
#define MAG_HZL				0x07
#define MAG_HZH				0x08

#define MAG_ASAX			0x10
#define MAG_ASAY			0x11
#define MAG_ASAZ			0x12

/* Functions */
void IMU_Init();
void MPU9250_ReadAccelData();
void MPU9250_ReadGyroData();
void MPU9250_GetGyroData(float* data);
void AK8963_ReadData();
void AHRS_ComputeAngles();
float AHRS_GetPitch();
float AHRS_GetRoll();
float AHRS_GetYaw();

#endif /* MPU9250_H_ */
