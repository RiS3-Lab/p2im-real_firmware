/*
 * MS5611.h
 *
 *  Created on: Jun 22, 2017
 *      Author: Heethesh
 */

#ifndef MS5611_H_
#define MS5611_H_

#define MS5611_ADDRESS			0x77 << 1
#define MS5611_CMD_ADC_READ		0x00
#define MS5611_CMD_RESET		0x1E
#define MS5611_CMD_CONV_D1		0x40
#define MS5611_CMD_CONV_D2		0x50
#define MS5611_CMD_READ_PROM	0xA0
#define MS5611_PROM_NB			8

#define SEA_LEVEL_PRESSURE		1013.25f // milliBars (hPa)

typedef enum
{
    MS5611_OSR_4096	= 0x08,
    MS5611_OSR_2048	= 0x06,
    MS5611_OSR_1024	= 0x04,
    MS5611_OSR_512	= 0x02,
    MS5611_OSR_256	= 0x00
} ms5611_osr_t;

void MS5611_Init(void);
float MS5611_GetTemperature(void);
float MS5611_GetPressure(void);
float MS5611_GetAltitude(void);
float MS5611_GetFilteredAltitude(void);
void MS5611_Update(void);

#endif /* MS5611_H_ */
