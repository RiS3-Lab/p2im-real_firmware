/*
 * multiwii.h
 *
 *  Created on: June 8, 2017
 *      Author: Heethesh
 */

#ifndef MSP_H_
#define MSP_H_

#include "stm32f1xx_hal.h"
#define MSP_RX_BUFFER_LENGTH 100

/* MSP Protocol Message Codes*/
enum msp_message_id
{
	/* TX */
	MSP_IDENT = 100,				/* 0x64 */
	MSP_STATUS = 101,				/* 0x65 */
	MSP_RAW_IMU = 102, 				/* 0x66 */
	MSP_SERVO = 103, 				/* 0x67 */
	MSP_MOTOR = 104,				/* 0x68 */
	MSP_RC = 105, 					/* 0x69 */
	MSP_RAW_GPS = 106, 				/* 0x6a */
	MSP_COMP_GPS = 107, 			/* 0x6b */
	MSP_ATTITUDE = 108, 			/* 0x6c */
	MSP_ALTITUDE = 109, 			/* 0x6d */
	MSP_ANALOG = 110, 				/* 0x6e */
	MSP_RC_TUNING = 111, 			/* 0x6f */
	MSP_PID = 112, 					/* 0x70 */
	MSP_BOX = 113,		 			/* 0x71 */
	MSP_MISC = 114, 				/* 0x72 */
	MSP_MOTOR_PINS = 115, 			/* 0x73 */
	MSP_BOXNAMES = 116, 			/* 0x74 */
	MSP_PIDNAMES = 117, 			/* 0x75 */
	MSP_WP = 118, 					/* 0x76 */
	MSP_BOXIDS = 119, 				/* 0x77 */
	MSP_SERVO_CONF = 120, 			/* 0x78 */

	/* RX */
	MSP_SET_RAW_RC = 200, 			/* 0xc8 */
	MSP_SET_RAW_GPS = 201, 			/* 0xc9 */
	MSP_SET_PID = 202, 				/* 0xca */
	MSP_SET_BOX = 203, 				/* 0xcb */
	MSP_SET_RC_TUNING = 204, 		/* 0xcc */
	MSP_ACC_CALIBRATION = 205, 		/* 0xcd */
	MSP_MAG_CALIBRATION = 206, 		/* 0xce */
	MSP_SET_MISC = 207, 			/* 0xcf */
	MSP_RESET_CONF = 208, 			/* 0xd0 */
	MSP_SET_WP = 209, 				/* 0xd1 */
	MSP_SELECT_SETTING = 210, 		/* 0xd2 */
	MSP_SET_HEAD = 211, 			/* 0xd3 */
	MSP_SET_SERVO_CONF = 212, 		/* 0xd4 */
	MSP_SET_MOTOR = 214, 			/* 0xd6 */
	MSP_BIND = 240, 				/* 0xf0 */
	MSP_EEPROM_WRITE = 250, 		/* 0xfa */

	/* Unofficial protocol extensions */
	MSP_EXT_NAV_STATUS = 121, 		/* 0x79 */
	MSP_EXT_NAV_CONFIG = 122, 		/* 0x7a */
	MSP_EXT_FW_CONFIG = 123, 		/* 0x7b */
	MSP_EXT_UID = 160, 				/* 0xa0 */
	MSP_EXT_GPSSVINFO = 164, 		/* 0xa4 */
	MSP_EXT_GPSDEBUGINFO = 166, 	/* 0xa6 */
	MSP_EXT_ACC_TRIM = 240, 		/* 0xf0  WARNING: shadows MSP_MSG_BIND */
	MSP_EXT_SERVOMIX_CONF = 241, 	/* 0xf1 */

	MSP_EXT_SET_NAV_CONFIG = 215, 	/* 0xd7 */
	MSP_EXT_SET_FW_CONFIG = 216, 	/* 0xd8 */
	MSP_EXT_SET_ACC_TRIM = 239, 	/* 0xef */
	MSP_EXT_SET_SERVOMIX_CONF = 242,/* 0xf2 */

	MSP_EXT_DEBUGMSG = 253, 		/* 0xfd */
	MSP_EXT_DEBUG = 254, 			/* 0xfe */

	/* Custom */
	MSP_SET_LED = 244				/* 0xf4 */
};

/* Structure Definitions */
struct MSP_RX_FRAME
{
	uint8_t count;
	uint16_t length;
	uint8_t code;
	uint8_t checksum;
	uint8_t RTS;
	uint8_t buffer[MSP_RX_BUFFER_LENGTH];
};

typedef struct __attribute__((__packed__))
{
	uint8_t led1;
	uint8_t led2;
	uint8_t led3;
}msp_set_led;

typedef struct __attribute__((__packed__))
{
	uint8_t version;
	uint8_t multitype;
	uint8_t msp_version;
	uint32_t capability;
}msp_ident;

typedef struct __attribute__((__packed__))
{
	uint16_t cycle_time; /* us */
	uint16_t i2c_errors_count;
	uint16_t sensor;
	uint32_t flag;
	uint8_t current_set;
}msp_status;

typedef struct __attribute__((__packed__))
{
	int16_t accx;
	int16_t accy;
	int16_t accz;
	int16_t gyrx;
	int16_t gyry;
	int16_t gyrz;
	int16_t magx;
	int16_t magy;
	int16_t magz;
}msp_raw_imu;

typedef struct __attribute__((__packed__))
{
	uint16_t motor[4];
}msp_motor;

/* [1000:2000] us */
typedef struct __attribute__((__packed__))
{
	uint16_t motor[4];
}msp_set_motor;

/* [1000:2000] us */
typedef struct __attribute__((__packed__))
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t throttle;
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;
	uint16_t aux4;
}msp_rc;

/* [1000:2000] us */
typedef struct __attribute__((__packed__))
{
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	uint16_t throttle;
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;
	uint16_t aux4;
}msp_set_raw_rc;

typedef struct __attribute__((__packed__))
{
	int16_t angx; 			/* [-1800:1800] 1/10 deg */
	int16_t angy; 			/* [-900:900] 1/10 deg */
	int16_t heading; 		/* [-180:180] deg */
}msp_attitude;

typedef struct __attribute__((__packed__))
{
	int32_t est_alt; 		/* cm */
	int16_t vario; 			/* cm/s */
}msp_altitude;

typedef struct __attribute__((__packed__))
{
	uint8_t vbat; 			/* 1/10 V */
	uint16_t power_meter_sum;
	uint16_t rssi; 			/* [0:1023] */
	uint16_t amperage;
}msp_analog;

typedef struct __attribute__((__packed__))
{
	uint8_t rc_rate; 		/* [0:100] */
	uint8_t rc_expo;		/* [0:100] */
	uint8_t roll_pitch_rate;/* [0:100] */
	uint8_t yaw_rate; 		/* [0:100] */
	uint8_t dyn_thr_pid; 	/* [0:100] */
	uint8_t throttle_mid; 	/* [0:100] */
	uint8_t throttle_expo; 	/* [0:100] */
}msp_rc_tuning;

typedef struct __attribute__((__packed__))
{
	uint8_t rc_rate;
	uint8_t rc_expo;
	uint8_t roll_pitch_rate;
	uint8_t yaw_rate;
	uint8_t dyn_thr_pid;
	uint8_t throttle_mid;
	uint8_t throttle_expo;
}msp_set_rc_tuning;

typedef struct __attribute__((__packed__))
{
	uint8_t p;
	uint8_t i;
	uint8_t d;
}_msp_pid_item;

typedef struct __attribute__((__packed__))
{
	_msp_pid_item roll;
	_msp_pid_item pitch;
	_msp_pid_item yaw;
	_msp_pid_item alt;
	_msp_pid_item pos;
	_msp_pid_item posr;
	_msp_pid_item navr;
	_msp_pid_item level;
	_msp_pid_item mag;
	_msp_pid_item vel;
}msp_pid;

typedef struct
{
	_msp_pid_item roll;
	_msp_pid_item pitch;
	_msp_pid_item yaw;
	_msp_pid_item alt;
	_msp_pid_item pos;
	_msp_pid_item posr;
	_msp_pid_item navr;
	_msp_pid_item level;
	_msp_pid_item mag;
	_msp_pid_item vel;
}msp_set_pid;

/* TODO: this is extremely poorly specified */
typedef struct __attribute__((__packed__))
{
	uint16_t boxitems[1];
}msp_box;

typedef struct __attribute__((__packed__))
{
	uint16_t boxitems[1];
}msp_set_box;

typedef struct __attribute__((__packed__))
{
	uint16_t power_trigger;
	uint16_t min_throttle; 		/* [1000:2000] us, neutral value */
	uint16_t max_throttle; 		/* [1000:2000] us, max value */
	uint16_t min_command; 		/* [1000:2000] us, min value */
	uint16_t failsafe_throttle; /* [1000:2000] us */
	uint16_t arm_count;
	uint32_t lifetime;
	uint16_t mag_declination; 	/* 1/10 deg */
	uint8_t vbat_scale;
	uint8_t vbat_warn1; 		/* 1/10 V */
	uint8_t vbat_warn2; 		/* 1/10 V */
	uint8_t vbat_crit; 			/* 1/10 V */
}msp_misc;

typedef struct __attribute__((__packed__))
{
	uint16_t power_trigger;
	uint16_t min_throttle; 		/* [1000:2000] us, neutral value */
	uint16_t max_throttle; 		/* [1000:2000] us, max value */
	uint16_t min_command; 		/* [1000:2000] us, min value */
	uint16_t failsafe_throttle; /* [1000:2000] us */
	uint16_t arm_count;
	uint32_t lifetime;
	uint16_t mag_declination; 	/* 1/10 deg */
	uint8_t vbat_scale;
	uint8_t vbat_warn1; 		/* 1/10 V */
	uint8_t vbat_warn2; 		/* 1/10 V */
	uint8_t vbat_crit; 			/* 1/10 V */
}msp_set_misc;

typedef struct __attribute__((__packed__))
{
	uint8_t pwm_pin[8];
}msp_motor_pins;

/* Cannot know length in advance */
typedef struct __attribute__((__packed__))
{
	char items[0]; /* ; separated */
}msp_boxnames;

/* cannot know length in advance */
typedef struct __attribute__((__packed__))
{
	char items[0]; /* ; separated */
}msp_pidnames;

typedef struct __attribute__((__packed__))
{
	uint8_t checkbox_items[0]; /* TODO: poor specification */
}msp_boxids;

typedef struct __attribute__((__packed__))
{
	uint16_t mag_hold;
}msp_set_head;

/* Transfer packets struct declarations */
/*extern msp_status msp_txf_status;
extern msp_raw_imu msp_txf_raw_imu;
extern msp_motor msp_txf_motor;
extern msp_rc msp_txf_rc;
extern msp_attitude msp_txf_attitude;
extern msp_altitude msp_txf_altitude;
extern msp_analog msp_txf_analog;
extern msp_rc_tuning msp_txf_rc_tuning;
extern msp_pid msp_txf_pid;
extern msp_misc msp_txf_misc;
extern msp_motor_pins msp_txf_motor_pins;
extern msp_boxnames msp_txf_boxnames;
extern msp_pidnames msp_txf_pidnames;
extern msp_boxids msp_txf_boxids;*/

/* Received packets struct declarations */
/*extern msp_set_raw_rc msp_rxf_raw_rc;
extern msp_set_pid msp_rxf_pid;
extern msp_set_box msp_rxf_box;
extern msp_set_rc_tuning msp_rxf_rc_tuning;
extern msp_set_misc msp_rxf_misc;
extern msp_set_head msp_rxf_head;
extern msp_set_motor msp_rxf_motor;*/

/* Function Definitions */
void MSP_Init();
void MSP_Read();
void MSP_Update();

void MSP_SendIdent();
void MSP_SendStatus();
void MSP_SendRawIMU();
void MSP_SendMotor();
void MSP_SendRC();
void MSP_SendAttitude();
void MSP_SendAltitude();
void MSP_SendAnalog();
void MSP_SendRCTuning();
void MSP_SendPID();
void MSP_SendMisc();
void MSP_SendMotorPins();
void MSP_SendBoxNames();
void MSP_SendPIDNames();
void MSP_SendBoxIDs();

void MSP_SetRawRC_Callback();
void MSP_SetPID_Callback();
void MSP_SetBox_Callback();
void MSP_SetRCTuning_Callback();
void MSP_SetMisc_Callback();
void MSP_SetHead_Callback();
void MSP_SetMotor_Callback();
void MSP_SetLED_Callback();

#endif /* MSP_H_ */
