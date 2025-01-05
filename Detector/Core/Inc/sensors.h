/*
 * sensors.h
 *
 *  Created on: Dec 19, 2024
 *      Author: Pawel
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdbool.h>
#include <stdint.h>

#define CRC8_POLYNOMIAL 	0x31
#define CRC8_INIT 			0xFF

#define SEN55_ADDRESS		0x69
#define SCD41_ADDRESS		0x62

/* Endianess changed to little endian */
#define SEN55_CMD_READSTATUS	0x06D2
#define SEN55_CMD_STARTMEAS		0x2100
#define SEN55_CMD_STOPMEAS		0x0401
#define SEN55_CMD_READMEAS		0xC403
#define SEN55_CMD_FANCLEAN		0x0756
#define SEN55_CMD_SETTEMPOFST   0xB260
#define SEN55_CMD_SETACCEL      0xF760
#define SEN55_CMD_SETWARM       0xC660

#define SEN55_TIME_READSTATUS	20
#define SEN55_TIME_STARTMEAS	50
#define SEN55_TIME_STOPMEAS		200
#define SEN55_TIME_READMEAS		20
#define SEN55_TIME_FANCLEAN		20
#define SEN55_TIME_SETTEMPOFST  20
#define SEN55_TIME_SETACCEL     20
#define SEN55_TIME_SETWARM     20

/* Endianess changed to little endian */
#define SCD41_CMD_PERFORMTEST	0x3936
#define SCD41_CMD_STARTMEAS		0xB121
#define SCD41_CMD_STOPMEAS		0x863F
#define SCD41_CMD_READMEAS		0x05EC
#define SCD41_CMD_DATAREADY     0xB8E4
#define	SCD41_CMD_SETASC		0x1624
#define SCD41_CMD_PERFORMCAL	0x2F36
#define SCD41_CMD_SETTEMPOFST   0x1D24

#define SCD41_TIME_PERFORMTEST	10000
#define SCD41_TIME_STARTMEAS	5
#define SCD41_TIME_STOPMEAS		500
#define SCD41_TIME_READMEAS		5
#define SCD41_TIME_DATAREADY    5
#define	SCD41_TIME_SETASC		5
#define SCD41_TIME_PERFORMCAL	400
#define SCD41_TIME_SETTEMPOFST  5

#define SENSORS_LEVEL_ACTUAL    SENSORS_LEVEL_GOOD
#define SENSORS_LEVEL_COUNT     6
#define SENSORS_SEN55_COUNT     8
#define SENSORS_SCD41_COUNT     3

typedef enum {
  SENSORS_LEVEL_GOOD,
  SENSORS_LEVEL_MAX,
  SENSORS_LEVEL_MIN,
  SENSORS_LEVEL_STEP,
  SENSORS_LEVEL_WARNING,
  SENSORS_LEVEL_CRITICAL
} Sensors_Level_t;

typedef enum {
  SENSORS_TYPE_PM1_0,
  SENSORS_TYPE_PM2_5,
  SENSORS_TYPE_PM4_0,
  SENSORS_TYPE_PM10,
  SENSORS_TYPE_CO2,
  SENSORS_TYPE_VOC,
  SENSORS_TYPE_NOC,
  SENSORS_TYPE_TEMP1,
  SENSORS_TYPE_TEMP2,
  SENSORS_TYPE_RH1,
  SENSORS_TYPE_RH2
} Sensors_Type_t;

typedef struct {
  double value;
  Sensors_Level_t level;
} Sensors_Value_t;

uint8_t Sensors_CalcCrc( uint8_t *data, uint16_t count );
bool Sensors_CompareCrc( uint8_t *data, uint16_t count, uint8_t crc );

void Sensors_SetValue( Sensors_Type_t type, Sensors_Level_t level, double value );
void Sensors_RecallValues();

bool Sensors_SEN55_Init();
bool Sensors_SEN55_Start();
bool Sensors_SEN55_Stop();
bool Sensors_SEN55_Read();
bool Sensors_SEN55_GetStatus();
bool Sensors_SEN55_CleanFan();
bool Sensors_SEN55_SetTempOffset( double offset, double slope, uint16_t time, uint8_t acceleration );
bool Sensors_SEN55_SetWarm( uint16_t warm );
double Sensors_SEN55_GetTempOffset( Sensors_Level_t level );
double Sensors_SEN55_GetTempSlope( Sensors_Level_t level );
uint16_t Sensors_SEN55_GetTempTime( Sensors_Level_t level );
uint16_t Sensors_SEN55_GetTempAccel( Sensors_Level_t level );

bool Sensors_SCD41_Init();
bool Sensors_SCD41_Start();
bool Sensors_SCD41_Stop();
bool Sensors_SCD41_Read();
bool Sensors_SCD41_PerformCalibration();
bool Sensors_SCD41_SetTempOffset( double offset );
double Sensors_SCD41_GetTempOffset( Sensors_Level_t level );

Sensors_Level_t Sensors_GetLevel( Sensors_Type_t type );
double Sensors_GetValue( Sensors_Type_t type, Sensors_Level_t level );

#endif /* INC_SENSORS_H_ */
