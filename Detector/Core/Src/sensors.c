/*
 * sensors.c
 *
 *  Created on: Dec 19, 2024
 *      Author: Pawel
 */

#include <stdio.h>
#include <string.h>

#include "sensors.h"
#include "main.h"
#include "messages.h"
#include "gui.h"

/* 11 sensors: PM1.0, PM2.5, PM4.0, CO2, VOC, NOC, TEMP1, TEMP2, RH1, RH2 */
/* 6 values per sensor:  ACTUAL, WARNING, CRITICAL, MAX, MIN, STEP */
static double values[11][6];

extern I2C_HandleTypeDef hi2c2;

double TEMP1_Offset[4], TEMP1_Slope[4], TEMP2_Offset[4];
uint16_t TEMP1_Time[4], TEMP1_Accel[4];

uint8_t Sensors_CalcCrc( uint8_t *data, uint16_t count )
{
  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;

  /* calculates 8-Bit checksum with given polynomial */
  for ( current_byte = 0; current_byte < count; ++current_byte )
  {
    crc ^= (data[current_byte]);
    for ( crc_bit = 8; crc_bit > 0; --crc_bit )
    {
      if ( crc & 0x80 )
      {
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

bool Sensors_CompareCrc( uint8_t *data, uint16_t count, uint8_t crc )
{
  uint8_t crc_calc;
  crc_calc = Sensors_CalcCrc( data, count );

  if( crc_calc != crc )
  {
    return false;
  } else {
    return true;
  }
}

void Sensors_SetValue( Sensors_Type_t type, Sensors_Level_t level, double value )
{
  uint8_t warning_critical;
  if( (level == SENSORS_LEVEL_WARNING) || (level == SENSORS_LEVEL_CRITICAL) )
  {
    if( level == SENSORS_LEVEL_WARNING )
    {
      warning_critical = 0;
    } else {
      warning_critical = 1;
    }

    RTC_BKUPWrite( BKP_REG_SENSOR_ST + (type * 2) + warning_critical, value );
    values[type][level] = value;
  }
}

void Sensors_RecallValues()
{
  values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM1_0 * 2) );
  values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM1_0 * 2) + 1 );
  values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_MAX] = 100;
  values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_MIN] = 10;
  values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_STEP] = 5;

  values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM2_5 * 2) );
  values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM2_5 * 2) + 1 );
  values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_MAX] = 100;
  values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_MIN] = 10;
  values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_STEP] = 5;

  values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM4_0 * 2) );
  values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM4_0 * 2) + 1 );
  values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_MAX] = 100;
  values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_MIN] = 10;
  values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_STEP] = 5;

  values[SENSORS_TYPE_PM10][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM10 * 2) );
  values[SENSORS_TYPE_PM10][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM10 * 2) + 1 );
  values[SENSORS_TYPE_PM10][SENSORS_LEVEL_MAX] = 100;
  values[SENSORS_TYPE_PM10][SENSORS_LEVEL_MIN] = 10;
  values[SENSORS_TYPE_PM10][SENSORS_LEVEL_STEP] = 5;

  values[SENSORS_TYPE_CO2][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_CO2 * 2) );
  values[SENSORS_TYPE_CO2][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_CO2 * 2) + 1 );
  values[SENSORS_TYPE_CO2][SENSORS_LEVEL_MAX] = 3000;
  values[SENSORS_TYPE_CO2][SENSORS_LEVEL_MIN] = 400;
  values[SENSORS_TYPE_CO2][SENSORS_LEVEL_STEP] = 100;

  values[SENSORS_TYPE_VOC][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_VOC * 2) );
  values[SENSORS_TYPE_VOC][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_VOC * 2) + 1 );
  values[SENSORS_TYPE_VOC][SENSORS_LEVEL_MAX] = 300;
  values[SENSORS_TYPE_VOC][SENSORS_LEVEL_MIN] = 100;
  values[SENSORS_TYPE_VOC][SENSORS_LEVEL_STEP] = 10;

  values[SENSORS_TYPE_NOC][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_NOC * 2) );
  values[SENSORS_TYPE_NOC][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_NOC * 2) + 1 );
  values[SENSORS_TYPE_NOC][SENSORS_LEVEL_MAX] = 300;
  values[SENSORS_TYPE_NOC][SENSORS_LEVEL_MIN] = 5;
  values[SENSORS_TYPE_NOC][SENSORS_LEVEL_STEP] = 5;

  /* For temperature and humidity TEMP1/RH1 is for lower than, TEMP2/RH2 is for higher than */
  values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP1 * 2) + 1 );
  values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP1 * 2) );
  values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP2 * 2) );
  values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP2 * 2) + 1 );
  values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_MAX] = 20;
  values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_MIN] = 0;
  values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_STEP] = 1;
  values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_MAX] = 40;
  values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_MIN] = 20;
  values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_STEP] = 1;

  values[SENSORS_TYPE_RH1][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH1 * 2) + 1 );
  values[SENSORS_TYPE_RH1][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH1 * 2) );
  values[SENSORS_TYPE_RH2][SENSORS_LEVEL_WARNING] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH2 * 2) );
  values[SENSORS_TYPE_RH2][SENSORS_LEVEL_CRITICAL] = (double) RTC_BKUPRead( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH2 * 2) + 1 );
  values[SENSORS_TYPE_RH1][SENSORS_LEVEL_MAX] = 50;
  values[SENSORS_TYPE_RH1][SENSORS_LEVEL_MIN] = 10;
  values[SENSORS_TYPE_RH1][SENSORS_LEVEL_STEP] = 5;
  values[SENSORS_TYPE_RH2][SENSORS_LEVEL_MAX] = 90;
  values[SENSORS_TYPE_RH2][SENSORS_LEVEL_MIN] = 50;
  values[SENSORS_TYPE_RH2][SENSORS_LEVEL_STEP] = 5;

  TEMP1_Offset[SENSORS_LEVEL_ACTUAL] = ((double) ((int16_t) RTC_BKUPRead( BKP_REG_TEMP1OFST ))) / 10;
  TEMP1_Offset[SENSORS_LEVEL_MAX] = 10;
  TEMP1_Offset[SENSORS_LEVEL_MIN] = -10;
  TEMP1_Offset[SENSORS_LEVEL_STEP] = 0.1;

  TEMP1_Slope[SENSORS_LEVEL_ACTUAL] = ((double) RTC_BKUPRead( BKP_REG_TEMP1SLOPE)) / 10;
  TEMP1_Slope[SENSORS_LEVEL_MAX] = 5;
  TEMP1_Slope[SENSORS_LEVEL_MIN] = -5;
  TEMP1_Slope[SENSORS_LEVEL_STEP] = 0.1;

  TEMP1_Time[SENSORS_LEVEL_ACTUAL] = (uint16_t) RTC_BKUPRead( BKP_REG_TEMP1TIME );
  TEMP1_Time[SENSORS_LEVEL_MAX] = 100;
  TEMP1_Time[SENSORS_LEVEL_MIN] = 0;
  TEMP1_Time[SENSORS_LEVEL_STEP] = 1;

  TEMP1_Accel[SENSORS_LEVEL_ACTUAL] = (uint16_t) RTC_BKUPRead( BKP_REG_TEMP1ACCEL );
  TEMP1_Accel[SENSORS_LEVEL_MAX] = 2;
  TEMP1_Accel[SENSORS_LEVEL_MIN] = 0;
  TEMP1_Accel[SENSORS_LEVEL_STEP] = 1;

  TEMP2_Offset[SENSORS_LEVEL_ACTUAL] = ((double) RTC_BKUPRead( BKP_REG_TEMP2OFST )) / 10;
  TEMP2_Offset[SENSORS_LEVEL_MAX] = 10;
  TEMP2_Offset[SENSORS_LEVEL_MIN] = -10;
  TEMP2_Offset[SENSORS_LEVEL_STEP] = 0.1;

  Sensors_SEN55_SetTempOffset(TEMP1_Offset[SENSORS_LEVEL_ACTUAL], TEMP1_Slope[SENSORS_LEVEL_ACTUAL], TEMP1_Time[SENSORS_LEVEL_ACTUAL], TEMP1_Accel[SENSORS_LEVEL_ACTUAL]);
  Sensors_SCD41_SetTempOffset(TEMP2_Offset[SENSORS_LEVEL_ACTUAL]);

}

bool Sensors_SEN55_Init()
{
  for( uint8_t i = 0; i < SENSORS_SEN55_COUNT; i++ )
  {
    for( uint8_t k = 0; k < SENSORS_LEVEL_COUNT; k++ )
    {
      values[i][k] = 0;
    }
  }

  Message( Message_SEN55_Init, Message_Debug, "Issued" );

  if( !Sensors_SEN55_GetStatus() )
  {
    return false;
  }

  return true;
}

bool Sensors_SEN55_Start()
{
  uint16_t command;
  HAL_StatusTypeDef status;

  Message( Message_SEN55_Start, Message_Debug, "Issued" );

  /* Read status from (SEN55) */
  command = SEN55_CMD_STARTMEAS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_Start, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_STARTMEAS );

  Message( Message_SEN55_Start, Message_Information, "OK" );

  return true;
}

bool Sensors_SEN55_Stop()
{
  uint16_t command;
  HAL_StatusTypeDef status;

  Message( Message_SEN55_Stop, Message_Debug, "Issued" );

  /* Read status from (SEN55) */
  command = SEN55_CMD_STOPMEAS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_Stop, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_STOPMEAS );

  Message( Message_SEN55_Stop, Message_Information, "OK" );
  return true;
}

bool Sensors_SEN55_Read()
{
  uint16_t command;
  uint8_t data[24];
  uint16_t word;
  int16_t word_s;

  HAL_StatusTypeDef status;

  Message( Message_SEN55_Read, Message_Debug, "Issued" );

  /* Read status from (SEN55) */
  command = SEN55_CMD_READMEAS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if(status != HAL_OK)
  {
    Message( Message_SEN55_Read, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_READMEAS );

  status = HAL_I2C_Master_Receive( &hi2c2, (SEN55_ADDRESS << 1), data, 24, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_Read, Message_Error, "FAILED (no response)" );
    return false;
  }

  /* Check CRC */
  if(  !Sensors_CompareCrc(&data[0], 2, data[2]) ||
    !Sensors_CompareCrc(&data[3], 2, data[5]) ||
    !Sensors_CompareCrc(&data[6], 2, data[8]) ||
    !Sensors_CompareCrc(&data[9], 2, data[11]) ||
    !Sensors_CompareCrc(&data[12], 2, data[14]) ||
    !Sensors_CompareCrc(&data[15], 2, data[17]) ||
    !Sensors_CompareCrc(&data[18], 2, data[20]) ||
    !Sensors_CompareCrc(&data[21], 2, data[23]) )
  {
    Message( Message_SEN55_Read, Message_Error, "FAILED (CRC error)" );
    return false;
  }

  word = (((uint16_t) data[0]) << 8) | ((uint16_t) data[1]);
  values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_ACTUAL] = ((double) word) / 10;

  word = (((uint16_t) data[3]) << 8) | ((uint16_t) data[4]);
  values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_ACTUAL] = ((double) word) / 10;

  word = (((uint16_t) data[6]) << 8) | ((uint16_t) data[7]);
  values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_ACTUAL] = ((double) word) / 10;

  word = (((uint16_t) data[9]) << 8) | ((uint16_t) data[10]);
  values[SENSORS_TYPE_PM10][SENSORS_LEVEL_ACTUAL] = ((double) word) / 10;

  word_s = (((int16_t) data[12]) << 8) | ((int16_t) data[13]);
  values[SENSORS_TYPE_RH1][SENSORS_LEVEL_ACTUAL] = ((double) word_s) / 100;

  word_s = (((int16_t) data[15]) << 8) | ((int16_t) data[16]);
  values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_ACTUAL] = ((double) word_s) / 200;

  word_s = (((int16_t) data[18]) << 8) | ((int16_t) data[19]);
  values[SENSORS_TYPE_VOC][SENSORS_LEVEL_ACTUAL] = ((double) word_s) / 10;

  word_s = (((int16_t) data[21]) << 8) | ((int16_t) data[12]);
  values[SENSORS_TYPE_NOC][SENSORS_LEVEL_ACTUAL] = ((double) word_s) / 10;

  Message( Message_SEN55_Read, Message_Information, "OK" );

  Message( Message_SEN55_Read, Message_Debug, "(PM1.0: %.1f) "
                                              "(PM2.5: %.1f) "
                                              "(PM4.0: %.1f) "
                                              "(PM10: %.1f) "
                                              "(RH1: %.1f) "
                                              "(TEMP1: %.1f) "
                                              "(VOC: %.1f) "
                                              "(NOC: %.1f)",
                                              values[SENSORS_TYPE_PM1_0][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_PM2_5][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_PM4_0][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_PM10][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_RH1][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_VOC][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_NOC][SENSORS_LEVEL_ACTUAL] );
  return true;
}

bool Sensors_SEN55_GetStatus()
{
  bool result;
  uint16_t command;
  uint8_t data[10];
  uint32_t status_reg;

  HAL_StatusTypeDef status;

  Message( Message_SEN55_GetStatus, Message_Debug, "Issued" );

  result = true;

  /* Read status from SEN55 */
  command = SEN55_CMD_READSTATUS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_READSTATUS );
  status = HAL_I2C_Master_Receive( &hi2c2, (SEN55_ADDRESS << 1), data, 6, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "FAILED (no response)" );
    return false;
  }

  /* Check CRC */
  if(  !Sensors_CompareCrc(&data[0], 2, data[2]) ||
    !Sensors_CompareCrc(&data[3], 2, data[5]) )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "FAILED (CRC error)" );
    return false;
  }

  /* Move data to status */
  status_reg =   (((uint32_t) data[0]) << 24) |
          (((uint32_t) data[1]) << 16) |
          (((uint32_t) data[3]) << 8) |
          ((uint32_t) data[4]);

  Message( Message_SEN55_GetStatus, Message_Debug, "Status_reg: 0x%08lX", status_reg );

  /* FAN Error */
  if( status_reg & (1 << 4) )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "(FAN error)" );
    result = false;
  }

  /* LASER Error */
  if( status_reg & (1 << 5) )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "(LASER error)" );
    result = false;
  }

  /* RHT Error */
  if( status_reg & (1 << 6) )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "(RHT error)" );
    result = false;
  }

  /* GAS SENSOR Error */
  if( status_reg & (1 << 7) )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "(GAS SENSOR error)" );
    result = false;
  }

  /* FAN Warning */
  if( status_reg & (1 << 21) )
  {
    Message( Message_SEN55_GetStatus, Message_Error, "(FAN warning)" );
    result = false;
  }

  if( result )
  {
    Message( Message_SEN55_GetStatus, Message_Information, "OK" );
  } else {
    Message( Message_SEN55_GetStatus, Message_Error, "FAILED" );
  }

  return result;
}

bool Sensors_SEN55_CleanFan()
{
  uint16_t command;
  HAL_StatusTypeDef status;

  Message( Message_SEN55_CleanFan, Message_Debug, "Issued" );

  /* Read status from SEN55 */
  command = SEN55_CMD_FANCLEAN;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_CleanFan, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_FANCLEAN );

  Message( Message_SEN55_CleanFan, Message_Information, "OK" );
  return true;
}

bool Sensors_SEN55_SetTempOffset(double offset, double slope, uint16_t time, uint8_t acceleration)
{
  uint16_t command;
  HAL_StatusTypeDef status;
  uint8_t data[11];
  int16_t word;
  double calc;

  RTC_BKUPWrite( BKP_REG_TEMP1OFST, (int16_t) ((double) offset * 10) );
  RTC_BKUPWrite( BKP_REG_TEMP1SLOPE, (uint16_t) ((double) slope * 10) );
  RTC_BKUPWrite( BKP_REG_TEMP1TIME, time );
  RTC_BKUPWrite( BKP_REG_TEMP1ACCEL, acceleration );

  TEMP1_Offset[ SENSORS_LEVEL_ACTUAL ] = offset;
  TEMP1_Slope[ SENSORS_LEVEL_ACTUAL ] = slope;
  TEMP1_Time[ SENSORS_LEVEL_ACTUAL ] = time;
  TEMP1_Accel[ SENSORS_LEVEL_ACTUAL ] = acceleration;

  if( !Sensors_SEN55_Stop() )
  {
    return false;
  }

  command = SEN55_CMD_SETTEMPOFST;

  data[0] = (uint8_t) (((uint16_t) command & 0x00FF));
  data[1] = (uint8_t) (((uint16_t) command & 0xFF00) >> 8);

  calc = offset * 200;
  word = (int16_t) calc;
  data[2] = (uint8_t) (((int16_t) word & 0xFF00) >> 8);
  data[3] = (uint8_t) (((int16_t) word & 0x00FF));
  data[4] = Sensors_CalcCrc( &data[2], 2 );

  calc = slope * 10000;
  word = (int16_t) calc;
  data[5] = (uint8_t) (((int16_t) word & 0xFF00) >> 8);
  data[6] = (uint8_t) (((int16_t) word & 0x00FF));
  data[7] = Sensors_CalcCrc( &data[5], 2 );

  data[8] = (uint8_t) (((uint16_t) time & 0xFF00) >> 8);
  data[9] = (uint8_t) (((uint16_t) time & 0x00FF));
  data[10] = Sensors_CalcCrc( &data[8], 2 );

  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) data, 11, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_SetTempOffset, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_SETTEMPOFST );

  Message( Message_SEN55_SetTempOffset, Message_Debug, "Offset set to: %.1f, slope to: %.1f, time to: %u", offset, slope, time );

  memset( data, 0x00, sizeof(uint8_t) * 11);

  command = SEN55_CMD_SETACCEL;

  data[0] = (uint8_t) (((uint16_t) command & 0x00FF));
  data[1] = (uint8_t) (((uint16_t) command & 0xFF00) >> 8);

  data[2] = (uint8_t) (((int16_t) acceleration & 0xFF00) >> 8);
  data[3] = (uint8_t) (((int16_t) acceleration & 0x00FF));
  data[4] = Sensors_CalcCrc( &data[2], 2 );

  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) data, 5, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_SetTempOffset, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_SETTEMPOFST );

  if( !Sensors_SEN55_SetWarm( 65535 ) )
  {
    return false;
  }

  if( !Sensors_SEN55_Start() )
  {
    return false;
  }

  return true;
}

bool Sensors_SEN55_SetWarm( uint16_t warm )
{
  uint16_t command;
  HAL_StatusTypeDef status;
  uint8_t data[11];
  uint16_t word;

  if( !Sensors_SEN55_Stop() )
  {
    return false;
  }

  command = SEN55_CMD_SETWARM;

  data[0] = (uint8_t) (((uint16_t) command & 0x00FF));
  data[1] = (uint8_t) (((uint16_t) command & 0xFF00) >> 8);

  word = (uint16_t) warm;
  data[2] = (uint8_t) (((int16_t) word & 0xFF00) >> 8);
  data[3] = (uint8_t) (((int16_t) word & 0x00FF));
  data[4] = Sensors_CalcCrc( &data[2], 2 );

  status = HAL_I2C_Master_Transmit( &hi2c2, (SEN55_ADDRESS << 1), (uint8_t *) data, 5, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SEN55_SetTempOffset, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SEN55_TIME_SETWARM );

  Message( Message_SEN55_SetTempOffset, Message_Debug, "Warm set to: %u", warm );

  return true;
}

inline double Sensors_SEN55_GetTempOffset( Sensors_Level_t level )
{
  if( level <= SENSORS_LEVEL_STEP )
  {
    return TEMP1_Offset[level];
  } else {
    return 0;
  }
}

inline double Sensors_SEN55_GetTempSlope( Sensors_Level_t level )
{
  if( level <= SENSORS_LEVEL_STEP )
  {
    return TEMP1_Slope[level];
  } else {
    return 0;
  }
}

inline uint16_t Sensors_SEN55_GetTempTime( Sensors_Level_t level )
{
  if( level <= SENSORS_LEVEL_STEP )
  {
    return TEMP1_Time[level];
  } else {
    return 0;
  }
}

inline uint16_t Sensors_SEN55_GetTempAccel( Sensors_Level_t level )
{
  if( level <= SENSORS_LEVEL_STEP )
  {
    return TEMP1_Accel[level];
  } else {
    return 0;
  }
}

bool Sensors_SCD41_Init()
{
  uint16_t command;
  uint8_t data[3];
  uint16_t status_reg;

  HAL_StatusTypeDef status;

  for( uint8_t i = SENSORS_SEN55_COUNT; i < (SENSORS_SEN55_COUNT + SENSORS_SCD41_COUNT); i++ )
  {
    for( uint8_t k = 0; k < SENSORS_LEVEL_COUNT; k++ )
    {
      values[i][k] = 0;
    }
  }

  Message( Message_SCD41_Init, Message_Debug, "Issued" );

  /* Perform self test in SCD41 */
  command = SCD41_CMD_PERFORMTEST;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) &command, 2, 100 );
  if( status != HAL_OK )
  {
    Message( Message_SCD41_Init, Message_Error, "FAILED (no communication)" );
    return false;
  }

  for( uint16_t i = 0; i < SCD41_TIME_PERFORMTEST; i+=1000 )
  {
    uint16_t temp;

    temp = (((uint16_t) SCD41_TIME_PERFORMTEST) - i) / 1000;
    GUI_Init( (uint8_t) temp );
    HAL_Delay( 1000 );
  }

  status = HAL_I2C_Master_Receive( &hi2c2, (SCD41_ADDRESS << 1), data, 3, 100 );
  if( status != HAL_OK )
  {
    Message( Message_SCD41_Init, Message_Error, "FAILED (no response)" );
    return false;
  }

  /* Check CRC */
  if( !Sensors_CompareCrc(&data[0], 2, data[2]) )
  {
    Message( Message_SCD41_Init, Message_Error, "FAILED (CRC error)" );
    return false;
  }

  /* Move data to status */
  status_reg = (((uint16_t) data[0]) << 8) |
               ((uint16_t) data[1]);

  if( status_reg != 0x00 )
  {
    Message( Message_SCD41_Init, Message_Error, "FAILED (STATUS = 0x%04X)", status_reg );
    return false;
  }

  Message( Message_SCD41_Init, Message_Information, "OK" );

  return true;
}

bool Sensors_SCD41_Start()
{
  uint16_t command;
  HAL_StatusTypeDef status;

  Message( Message_SCD41_Start, Message_Debug, "Issued" );

  command = SCD41_CMD_STARTMEAS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SCD41_Start, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SCD41_TIME_STARTMEAS );

  Message( Message_SCD41_Start, Message_Information, "OK" );

  return true;
}

bool Sensors_SCD41_Stop()
{
  uint16_t command;
  HAL_StatusTypeDef status;

  Message( Message_SCD41_Stop, Message_Debug, "Issued" );

  command = SCD41_CMD_STOPMEAS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SCD41_Stop, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SCD41_TIME_STOPMEAS );

  Message( Message_SCD41_Stop, Message_Information, "OK" );

  return true;
}

bool Sensors_SCD41_Read()
{
  uint16_t command;
  HAL_StatusTypeDef status;
  uint8_t data[9];
  uint16_t word;

  command = SCD41_CMD_DATAREADY;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SCD41_DataReady, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SCD41_TIME_DATAREADY );

  status = HAL_I2C_Master_Receive( &hi2c2, (SCD41_ADDRESS << 1), data, 9, 100 );
  if( status != HAL_OK )
  {
    Message( Message_SCD41_DataReady, Message_Error, "FAILED (no response)" );
    return false;
  }

  /* Check CRC */
  if( !Sensors_CompareCrc(&data[0], 2, data[2]) )
  {
    Message( Message_SCD41_DataReady, Message_Error, "FAILED (CRC error)" );
    return false;
  }

  word = (uint16_t) ((data[0] << 8) | data[1]);

  /* Check if data is ready */
  if( (word & 0x07FF) == 0x0000 )
  {
    Message( Message_SCD41_DataReady, Message_Debug, "Data not ready" );
    return true;
  } else {
    Message( Message_SCD41_DataReady, Message_Debug, "Data ready" );
  }

  Message( Message_SCD41_Read, Message_Debug, "Issued" );

  command = SCD41_CMD_READMEAS;
  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) &command, 2, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SCD41_Read, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SCD41_TIME_READMEAS );

  status = HAL_I2C_Master_Receive( &hi2c2, (SCD41_ADDRESS << 1), data, 9, 100 );
  if( status != HAL_OK )
  {
    Message( Message_SCD41_Read, Message_Error, "FAILED (no response)" );
    return false;
  }

  /* Check CRC */
  if( !Sensors_CompareCrc(&data[0], 2, data[2]) ||
      !Sensors_CompareCrc(&data[3], 2, data[5]) ||
      !Sensors_CompareCrc(&data[6], 2, data[8]) )
  {
    Message( Message_SCD41_Read, Message_Error, "FAILED (CRC error)" );
    return false;
  }

  word = (uint16_t) ((data[0] << 8) | data[1]);
  values[SENSORS_TYPE_CO2][SENSORS_LEVEL_ACTUAL] = (double) word;

  word = (uint16_t) ((data[3] << 8) | data[4]);
  values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_ACTUAL] = -45 + (175 * ((double) word)) / (65536 - 1);

  word = (uint16_t) ((data[6] << 8) | data[7]);
  values[SENSORS_TYPE_RH2][SENSORS_LEVEL_ACTUAL] = (100 * ((double) word)) / (65536 - 1);

  Message( Message_SCD41_Read, Message_Information, "OK" );
  Message( Message_SCD41_Read, Message_Debug, "(CO2: %.0f) "
                                              "(TEMP2: %.1f) "
                                              "(RH2: %.1f) ",
                                              values[SENSORS_TYPE_CO2][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_ACTUAL],
                                              values[SENSORS_TYPE_RH2][SENSORS_LEVEL_ACTUAL] );

  return true;
}

bool Sensors_SCD41_PerformCalibration()
{
  uint16_t command;
  HAL_StatusTypeDef status;
  uint8_t data[5];
  uint16_t word;

  if( !Sensors_SCD41_Stop() )
  {
    return false;
  }

  command = SCD41_CMD_PERFORMCAL;

  data[0] = (uint8_t) (((uint16_t) command & 0x00FF));
  data[1] = (uint8_t) (((uint16_t) command & 0xFF00) >> 8);
  data[2] = 0x01; /* 400 ppm */
  data[3] = 0x90;
  data[4] = Sensors_CalcCrc( &data[2], 2 );

  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) data, 5, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SCD41_PerformCal, Message_Error, "FAILED (no communication)" );
    return false;
  }

  memset( data, 0x00, sizeof(uint8_t) * 5 );

  HAL_Delay( SCD41_TIME_PERFORMCAL );

  status = HAL_I2C_Master_Receive( &hi2c2, (SCD41_ADDRESS << 1), data, 3, 100 );
  if( status != HAL_OK )
  {
    Message( Message_SCD41_PerformCal, Message_Error, "FAILED (no response)" );
    return false;
  }

  /* Check CRC */
  if( !Sensors_CompareCrc(&data[0], 2, data[2]) )
  {
    Message( Message_SCD41_PerformCal, Message_Error, "FAILED (CRC error)" );
    return false;
  }

  word = (uint16_t) ((data[0] << 8) | data[1]);

  /* Check if success */
  if( word == 0xFFFF )
  {
    Message( Message_SCD41_PerformCal, Message_Debug, "Calibration failed" );
    return false;
  }

  Message( Message_SCD41_PerformCal, Message_Error, "Calibration OK" );

  if( !Sensors_SCD41_Start() )
  {
    return false;
  }

  return true;
}

bool Sensors_SCD41_SetTempOffset( double offset )
{
  uint16_t command;
  HAL_StatusTypeDef status;
  uint8_t data[5];
  uint16_t word;
  double calc;

  RTC_BKUPWrite( BKP_REG_TEMP2OFST, (uint16_t) ((double) offset * 10) );
  TEMP2_Offset[ SENSORS_LEVEL_ACTUAL ] = offset;

  if( !Sensors_SCD41_Stop() )
  {
    return false;
  }

  command = SCD41_CMD_SETTEMPOFST;

  calc = 65535 / 175 * offset;
  word = (uint16_t) calc;

  data[0] = (uint8_t) (((uint16_t) command & 0x00FF));
  data[1] = (uint8_t) (((uint16_t) command & 0xFF00) >> 8);
  data[2] = (uint8_t) (((uint16_t) word & 0xFF00) >> 8);
  data[3] = (uint8_t) (((uint16_t) word & 0x00FF));
  data[4] = Sensors_CalcCrc( &data[2], 2 );

  status = HAL_I2C_Master_Transmit( &hi2c2, (SCD41_ADDRESS << 1), (uint8_t *) data, 5, 100 );

  if( status != HAL_OK )
  {
    Message( Message_SCD41_SetTempOffset, Message_Error, "FAILED (no communication)" );
    return false;
  }

  HAL_Delay( SCD41_TIME_SETTEMPOFST );

  Message( Message_SCD41_SetTempOffset, Message_Debug, "Offset set to: %.1f", offset );

  if( !Sensors_SCD41_Start() )
  {
    return false;
  }

  return true;
}

inline double Sensors_SCD41_GetTempOffset( Sensors_Level_t level )
{
  if( level <= SENSORS_LEVEL_STEP )
  {
    return TEMP2_Offset[level];
  } else {
    return 0;
  }
}

inline double Sensors_GetValue( Sensors_Type_t type, Sensors_Level_t level )
{
  return values[type][level];
}

Sensors_Level_t Sensors_GetLevel( Sensors_Type_t type )
{
  Sensors_Level_t level;

  if( (type == SENSORS_TYPE_PM1_0) ||
      (type == SENSORS_TYPE_PM2_5) ||
      (type == SENSORS_TYPE_PM4_0) ||
      (type == SENSORS_TYPE_PM10) ||
      (type == SENSORS_TYPE_VOC) ||
      (type == SENSORS_TYPE_NOC) ||
      (type == SENSORS_TYPE_CO2))
  { /* PM1.0, PM2.5, PM4.0, PM10, VOC, NOC, CO2 */
    if( values[type][SENSORS_LEVEL_ACTUAL] >= values[type][SENSORS_LEVEL_CRITICAL] )
    {
      level = SENSORS_LEVEL_CRITICAL;
    } else if( values[type][SENSORS_LEVEL_ACTUAL] >= values[type][SENSORS_LEVEL_WARNING] )
    {
      level = SENSORS_LEVEL_WARNING;
    } else {
      level = SENSORS_LEVEL_GOOD;
    }

  } else if(  (type == SENSORS_TYPE_TEMP1) ||
              (type == SENSORS_TYPE_TEMP2))
  { /* Temperatures - two side warning & critical */
    if( (values[type][SENSORS_LEVEL_ACTUAL] <= values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_CRITICAL] ) ||
        (values[type][SENSORS_LEVEL_ACTUAL] >= values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_CRITICAL] ))
    {
      level = SENSORS_LEVEL_CRITICAL;
    } else if( (values[type][SENSORS_LEVEL_ACTUAL] <= values[SENSORS_TYPE_TEMP1][SENSORS_LEVEL_WARNING] ) ||
               (values[type][SENSORS_LEVEL_ACTUAL] >= values[SENSORS_TYPE_TEMP2][SENSORS_LEVEL_WARNING] ))
    {
      level = SENSORS_LEVEL_WARNING;
    } else {
      level = SENSORS_LEVEL_GOOD;
    }
  } else { /* Humidity - two side warning & critical */
    if( (values[type][SENSORS_LEVEL_ACTUAL] <= values[SENSORS_TYPE_RH1][SENSORS_LEVEL_CRITICAL] ) ||
        (values[type][SENSORS_LEVEL_ACTUAL] >= values[SENSORS_TYPE_RH2][SENSORS_LEVEL_CRITICAL] ))
    {
      level = SENSORS_LEVEL_CRITICAL;
    } else if( (values[type][SENSORS_LEVEL_ACTUAL] <= values[SENSORS_TYPE_RH1][SENSORS_LEVEL_WARNING] ) ||
               (values[type][SENSORS_LEVEL_ACTUAL] >= values[SENSORS_TYPE_RH2][SENSORS_LEVEL_WARNING] ))
    {
      level = SENSORS_LEVEL_WARNING;
    } else {
      level = SENSORS_LEVEL_GOOD;
    }

  }

  return level;
}

