/*
 * multiwii.c
 *
 *  Created on: June 8, 2017
 *      Author: Heethesh
 */

#include "serial.h"
#include "msp.h"
#include "stm32f1xx_hal.h"
#include <string.h>

struct MSP_RX_FRAME rxf;

/* Transfer packets struct declarations */
msp_ident msp_txf_ident;
msp_status msp_txf_status;
msp_raw_imu msp_txf_raw_imu;
msp_motor msp_txf_motor;
msp_rc msp_txf_rc;
msp_attitude msp_txf_attitude;
msp_altitude msp_txf_altitude;
msp_analog msp_txf_analog;
msp_rc_tuning msp_txf_rc_tuning;
msp_pid msp_txf_pid;
msp_misc msp_txf_misc;
msp_motor_pins msp_txf_motor_pins;
msp_boxnames msp_txf_boxnames;
msp_pidnames msp_txf_pidnames;
msp_boxids msp_txf_boxids;

/* Received packets struct declarations */
msp_set_raw_rc msp_rxf_raw_rc;
msp_set_pid msp_rxf_pid;
msp_set_box msp_rxf_box;
msp_set_rc_tuning msp_rxf_rc_tuning;
msp_set_misc msp_rxf_misc;
msp_set_head msp_rxf_head;
msp_set_motor msp_rxf_motor;
msp_set_led msp_rxf_led;

/**********************************
 Function name	:	MSP_SendFrame
 Functionality	:	To create and send MSP frames
 Arguments		:	MSP message ID, byte array, array length
 Return Value	:	None
 Example Call	:	MSP_SendFrame(MSP_PIDNAMES, buff, data_length)
 ***********************************/
static void MSP_SendFrame(uint8_t code, uint8_t *data, uint16_t data_length)
{
	uint8_t checksum = 0;

	// Send Header
	serialPrint("$M>");

	// Send data length
	serialWrite(data_length);

	// Send MSP message ID
	serialWrite(code);

	// Update checksum
	checksum = code ^ data_length;

	// Send byte array and update checksum
	for (int i=0; i<data_length; i++)
	{
		serialWrite((char) data[i]);
		checksum ^= data[i];
	}

	// Send checksum
	serialWrite(checksum);
}

/**********************************
 Function name	:	MSP_ResetFrameBuffer
 Functionality	:	To reset all the RX frame parsing parameters
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_ResetFrameBuffer()
 ***********************************/
static void MSP_ResetFrameBuffer(void)
{
	rxf.count = 0;
	rxf.length = 0;
	rxf.code = 0;
	rxf.checksum = 0;
	rxf.RTS = 0;

	/*for (int i=0; i<MSP_RX_BUFFER_LENGTH; i++)
		rxf.buffer[i] = 0;*/
}

/**********************************
 Function name	:	MSP_ParseFrame
 Functionality	:	To parse request and command frames
 	 	 	 	 	If request frame --> Send the required MSP frame (TX frame request)
 	 	 	 	 	If command frame --> Invoke callback function (RX frame)
 Arguments		:	MSP message ID, byte array, array length, request to send status
 Return Value	:	None
 Example Call	:	MSP_ParseFrame(code, fbuffer, data_length, RTS)
 ***********************************/
static void MSP_ParseFrame(uint8_t code, uint8_t *data, uint16_t data_length, uint8_t RTS)
{
	switch (code)
	{
		case MSP_IDENT:
			if (RTS) MSP_SendIdent();											// Send Identifier data on RTS
			break;

		case MSP_STATUS:
			if (RTS) MSP_SendStatus();											// Send Status data on RTS
			break;

		case MSP_RAW_IMU:
			if (RTS) MSP_SendRawIMU();											// Send Raw IMU data on RTS
			break;

		case MSP_MOTOR:
			if (RTS) MSP_SendMotor();											// Send Motor data on RTS
			break;

		case MSP_RC:
			if (RTS) MSP_SendRC();												// Send RC data on RTS
			break;

		case MSP_ATTITUDE:
			if (RTS) MSP_SendAttitude();										// Send Attitude data on RTS
			break;

		case MSP_ALTITUDE:
			if (RTS) MSP_SendAltitude();										// Send Altitude data on RTS
			break;

		/*case MSP_ANALOG:
			if (RTS) MSP_SendAnalog();											// Send Analog data on RTS
			break;

		case MSP_RC_TUNING:
			if (RTS) MSP_SendRCTuning();										// Send RC Tuning data on RTS
			break;

		case MSP_PID:
			if (RTS) MSP_SendPID();												// Send PID data on RTS
			break;

		case MSP_MISC:
			if (RTS) MSP_SendMisc();											// Send Miscellaneous data on RTS
			break;

		case MSP_MOTOR_PINS:
			if (RTS) MSP_SendMotorPins();										// Send Motor Pins data on RTS
			break;

		case MSP_BOXNAMES:
			if (RTS) MSP_SendBoxNames();										// Send Box Names data on RTS
			break;

		case MSP_PIDNAMES:
			if (RTS) MSP_SendPIDNames();										// Send PID Names data on RTS
			break;

		case MSP_BOXIDS:
			if (RTS) MSP_SendBoxIDs();											// Send Box IDs data on RTS
			break;*/

		case MSP_SET_RAW_RC:
			memcpy(&msp_rxf_raw_rc, data, sizeof(msp_set_raw_rc));				// Convert byte array to struct
			MSP_SetRawRC_Callback();											// Callback function
			break;

		case MSP_SET_PID:
			memcpy(&msp_rxf_pid, data, sizeof(msp_set_pid));					// Convert byte array to struct
			MSP_SetPID_Callback();												// Callback function
			break;

		/*case MSP_SET_BOX:
			memcpy(&msp_rxf_box, data, sizeof(msp_set_box));					// Convert byte array to struct
			MSP_SetBox_Callback();												// Callback function
			break;

		case MSP_SET_RC_TUNING:
			memcpy(&msp_rxf_rc_tuning, data, sizeof(msp_set_rc_tuning));		// Convert byte array to struct
			MSP_SetRCTuning_Callback();											// Callback function
			break;

		case MSP_SET_MISC:
			memcpy(&msp_rxf_misc, data, sizeof(msp_set_misc));					// Convert byte array to struct
			MSP_SetMisc_Callback();												// Callback function
			break;

		case MSP_SET_HEAD:
			memcpy(&msp_rxf_head, data, sizeof(msp_set_head));					// Convert byte array to struct
			MSP_SetHead_Callback();												// Callback function
			break;

		case MSP_SET_MOTOR:
			memcpy(&msp_rxf_motor, data, sizeof(msp_set_motor));				// Convert byte array to struct
			MSP_SetMotor_Callback();											// Callback function
			break;

		case MSP_SET_LED:
			memcpy(&msp_rxf_led, data, sizeof(msp_set_led));					// Convert byte array to struct
			MSP_SetLED_Callback();												// Callback function
			break;

		case MSP_EEPROM_WRITE:													// Not implemented
			break;

		case MSP_EXT_DEBUG:														// Not implemented
			break;*/
	}
}


/**********************************
 Function name	:	MSP_Update
 Functionality	:	Non-blocking function to read and parse RX MSP frames
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_Update()
 ***********************************/
void MSP_Update(void)
{
	if (serialAvailable() == 0) return;

	/* Check '$' header element */
	if (rxf.count == 0)
	{
		// Reset frame counter and frame buffer
		MSP_ResetFrameBuffer();
		rxf.count = (serialRead() == '$') ? 1:0;
	}

	/* Check 'M' header element */
	else if (rxf.count == 1)
	{
		rxf.count = (serialRead() == 'M') ? 2:0;
	}

	/* Check '<' RX frame */
	else if (rxf.count == 2)
	{
		rxf.count = (serialRead() == '<') ? 3:0;
	}

	/* Get payload length and RTS condition */
	else if (rxf.count == 3)
	{
		rxf.length = serialRead();
		if (rxf.length == 0x00) rxf.RTS = 1;
		rxf.count++;
	}

	/* Get MSP code */
	else if (rxf.count == 4)
	{
		rxf.code = serialRead();
		rxf.checksum = rxf.code ^ rxf.length;
		rxf.count++;
	}

	/* Get payload */
	else if (rxf.count == 5)
	{
		if (rxf.RTS)
		{
			rxf.count++;
			return;
		}

		else if (serialAvailable() >= rxf.length)
		{
			for (int i=0; i<rxf.length; i++)
			{
				rxf.buffer[i] = serialRead();
				rxf.checksum ^= rxf.buffer[i];
			}
			rxf.count++;
		}
	}

	/* Verify checksum and parse the payload */
	else if (rxf.count == 6)
	{
		if (rxf.checksum != serialRead()) rxf.count = 0;
		else
		{
			MSP_ParseFrame(rxf.code, rxf.buffer, rxf.length, rxf.RTS);
			rxf.count = 0;
		}
	}

	/* Reset frame counter */
	else rxf.count = 0;
}

/**********************************
 Function name	:	MSP_Read
 Functionality	:	Blocking function to read and parse RX MSP frames
 	 	 	 	 	This requires all of the frame data to be received before hand
 	 	 	 	 	Deprecated, use MSP_Update()
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_Read()
 ***********************************/
void MSP_Read(void)
{
	// Buffer to store payload
	uint8_t fbuffer[50];

	// Request/command frame status
	uint8_t RTS = 0;

	// Check header
	if (serialRead() != '$') return;
	if (serialRead() != 'M') return;
	if (serialRead() != '<') return;

	// Read data length
	uint8_t data_length = serialRead();

	// Read MSP message ID
	uint8_t code = serialRead();

	// Update checksum
	uint8_t checksum = code ^ data_length;

	// Length = 0 --> Request frame
	// Length > 0 --> Command frame
	if (data_length != 0)
	{
		// Command frame
		for (int i=0; i<data_length; i++)
		{
			// Store data and update checksum
			fbuffer[i] = serialRead();
			checksum ^= fbuffer[i];
		}
	}

	// Request frame received
	else RTS = 1;

	// Verify checksum
	if (checksum != serialRead()) return;

	// Parse the payload
	MSP_ParseFrame(code, fbuffer, data_length, RTS);
}

/**********************************
 Function name	:	MSP_Init
 Functionality	:	To send MSP status and identifier frame to MultiWii Conf
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_Init()
 ***********************************/
void MSP_Init(void)
{
	for (int i=0; i<2; i++)
	{
		MSP_SendIdent();
		MSP_SendStatus();
	}
}

/**********************************
 Function name	:	MSP_SendIdent
 Functionality	:	To send identifier data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendIdent()
 ***********************************/
void MSP_SendIdent(void)
{
	msp_txf_ident.version = 233;
	msp_txf_ident.multitype = 3;
	msp_txf_ident.msp_version = 0;
	msp_txf_ident.capability = 4;

	uint16_t data_length = sizeof(msp_ident);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_ident, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_IDENT, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendStatus
 Functionality	:	To send status data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendStatus()
 ***********************************/
void MSP_SendStatus(void)
{
	msp_txf_status.sensor = 7;
	msp_txf_status.flag = 42;
	msp_txf_status.current_set = 0;

	uint16_t data_length = sizeof(msp_status);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_status, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_STATUS, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendRawIMU
 Functionality	:	To send raw IMU data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendRawIMU()
 ***********************************/
void MSP_SendRawIMU(void)
{
	uint16_t data_length = sizeof(msp_raw_imu);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_raw_imu, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_RAW_IMU, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendMotor
 Functionality	:	To send motor data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendMotor()
 ***********************************/
void MSP_SendMotor(void)
{
	uint16_t data_length = sizeof(msp_motor);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_motor, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_MOTOR, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendRC
 Functionality	:	To send RC data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendRC()
 ***********************************/
void MSP_SendRC(void)
{
	uint16_t data_length = sizeof(msp_rc);					// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_rc, data_length);					// Convert struct elements to bytes
	MSP_SendFrame(MSP_RC, buff, data_length);				// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendAttitude
 Functionality	:	To send attitude data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendAttitude()
 ***********************************/
void MSP_SendAttitude(void)
{
	uint16_t data_length = sizeof(msp_attitude);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_attitude, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_ATTITUDE, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendAltitude
 Functionality	:	To send altitude data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendAltitude()
 ***********************************/
void MSP_SendAltitude(void)
{
	uint16_t data_length = sizeof(msp_altitude);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_altitude, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_ALTITUDE, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendAnalog
 Functionality	:	To send analog data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendAnalog()
 ***********************************/
void MSP_SendAnalog(void)
{
	uint16_t data_length = sizeof(msp_analog);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_analog, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_ANALOG, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendRCTuning
 Functionality	:	To send RC tuning data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendRCTuning()
 ***********************************/
void MSP_SendRCTuning(void)
{
	uint16_t data_length = sizeof(msp_rc_tuning);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_rc_tuning, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_RC_TUNING, buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendPID
 Functionality	:	To send PID data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendPID()
 ***********************************/
void MSP_SendPID(void)
{
	uint16_t data_length = sizeof(msp_pid);					// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_pid, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_PID, buff, data_length);				// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendMisc
 Functionality	:	To send miscellaneous data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendMisc()
 ***********************************/
void MSP_SendMisc(void)
{
	uint16_t data_length = sizeof(msp_misc);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_misc, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_MISC, buff, data_length);				// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendMotorPins
 Functionality	:	To send motor pins data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendMotorPins()
 ***********************************/
void MSP_SendMotorPins(void)
{
	uint16_t data_length = sizeof(msp_motor_pins);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_motor_pins, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_MOTOR_PINS, buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendBoxNames
 Functionality	:	To send box names data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendBoxNames()
 ***********************************/
void MSP_SendBoxNames(void)
{
	uint16_t data_length = sizeof(msp_boxnames);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_boxnames, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_BOXNAMES, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendPIDNames
 Functionality	:	To send box IDs data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendPIDNames()
 ***********************************/
void MSP_SendPIDNames(void)
{
	uint16_t data_length = sizeof(msp_pidnames);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_pidnames, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_PIDNAMES, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendBoxIDs
 Functionality	:	To send SendBoxids using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendBoxIDs()
 ***********************************/
void MSP_SendBoxIDs(void)
{
	uint16_t data_length = sizeof(msp_boxids);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_boxids, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_BOXIDS, buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SetRawRC_Callback
 Functionality	:	Callback function to handle received raw RC data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetRawRC_Callback()
 ***********************************/
__weak void MSP_SetRawRC_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_RAW_RC);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetRawRC_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetPID_Callback
 Functionality	:	Callback function to handle received PID data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetPID_Callback()
 ***********************************/
__weak void MSP_SetPID_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_PID);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetPID_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetBox_Callback
 Functionality	:	Callback function to handle received box data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetBox_Callback()
 ***********************************/
__weak void MSP_SetBox_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_BOX);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetBox_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetRCTuning_Callback
 Functionality	:	Callback function to handle received RC tuning data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetRCTuning_Callback()
 ***********************************/
__weak void MSP_SetRCTuning_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_RC_TUNING);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetRCTuning_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetMisc_Callback
 Functionality	:	Callback function to handle received miscellaneous data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetMisc_Callback()
 ***********************************/
__weak void MSP_SetMisc_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_MISC);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetMisc_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetHead_Callback
 Functionality	:	Callback function to handle received head data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetHead_Callback()
 ***********************************/
__weak void MSP_SetHead_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_HEAD);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetHead_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetMotor_Callback
 Functionality	:	Callback function to handle received motor data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetMotor_Callback()
 ***********************************/
__weak void MSP_SetMotor_Callback(void)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(MSP_SET_MOTOR);
	/* NOTE: This function can be modified, when the callback is needed,
	or MSP_SetMotor_Callback() can be implemented in the user file*/
}

/**********************************
 Function name	:	MSP_SetLED_Callback
 Functionality	:	Callback function to handle received LED data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetLED_Callback()
 ***********************************/
void MSP_SetLED_Callback(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, msp_rxf_led.led1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, msp_rxf_led.led2);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, msp_rxf_led.led3);
}
