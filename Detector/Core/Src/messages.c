/*
 * messages.c
 *
 *  Created on: Dec 21, 2024
 *      Author: Pawel
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "main.h"
#include "messages.h"

const bool MessageLevelRecord[Message_LevelLast] = {
    true,   /* Message_Error*/
    true,   /* Message_Warning */
    true,   /* Message_Information */
    true    /* Message_Debug */
};

const bool MessageSourceRecord[Message_SourceLast] = {
    true,   /* Message_Main */
    false,  /* Message_SEN55_Init */
    false,  /* Message_SEN55_Start */
    false,  /* Message_SEN55_Stop */
    false,  /* Message_SEN55_Read */
    false,  /* Message_SEN55_GetStatus */
    false,  /* Message_SEN55_CleanFan */
    true,   /* Message_SEN55_SetTempOffset */
    false,  /* Message_SCD41_Init */
    false,  /* Message_SCD41_Start */
    true,   /* Message_SCD41_Stop */
    false,  /* Message_SCD41_Read */
    false,  /* Message_SCD41_DataReady */
    true,   /* Message_SCD41_PerformCal */
    true,   /* Message_SCD41_SetTempOffset */
    true,   /* Message_TouchPanel */
    true,   /* Message_LCD */
    true,   /* Message_GUI */
    true    /* Message_FATFS */
};

const char *MessageNames[] = {
  "Main",
  "SEN55 (Init)",
  "SEN55 (Start)",
  "SEN55 (Stop)",
  "SEN55 (Read)",
  "SEN55 (GetStatus)",
  "SEN55 (CleanFan)",
  "SEN55 (SetTempOffset)",
  "SCD41 (Init)",
  "SCD41 (Start)",
  "SCD41 (Stop)",
  "SCD41 (Read)",
  "SCD41 (DataReady)",
  "SCD41 (PerformCal)",
  "SCD41 (SetTempOffset)",
  "TouchPanel",
  "LCD",
  "GUI",
  "FATFS"
};

const char *MessageTypeNames[] = {
  "E",
  "W",
  "I",
  "D"
};

void Message( MessageSource_t MessageSource, MessageType_t MessageLevel, char *format, ... )
{
  va_list args;

  if( MessageLevelRecord[MessageLevel] )
  {
    if( MessageSourceRecord[MessageSource] )
    {
      printf( "%s[%s]: ", MessageTypeNames[MessageLevel], MessageNames[MessageSource] );

      va_start( args, format );
      vprintf( format, args );
      va_end( args );

      printf( "\n" );
    }
  }
}
