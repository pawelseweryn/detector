/*
 * messages.h
 *
 *  Created on: Dec 21, 2024
 *      Author: Pawel
 */

#ifndef INC_MESSAGES_H_
#define INC_MESSAGES_H_

#include <stdio.h>
#include <stdarg.h>

enum {
  Message_Error,
  Message_Warning,
  Message_Information,
  Message_Debug,
  Message_LevelLast
} typedef MessageType_t;

enum {
  Message_Main,
  Message_SEN55_Init,
  Message_SEN55_Start,
  Message_SEN55_Stop,
  Message_SEN55_Read,
  Message_SEN55_GetStatus,
  Message_SEN55_CleanFan,
  Message_SCD41_Init,
  Message_SCD41_Start,
  Message_SCD41_Stop,
  Message_SCD41_Read,
  Message_SCD41_DataReady,
  Message_TouchPanel,
  Message_LCD,
  Message_GUI,
  Message_SourceLast
} typedef MessageSource_t;


void Message( MessageSource_t MessageSource, MessageType_t MessageLevel, char *format, ... );

#endif /* INC_MESSAGES_H_ */
