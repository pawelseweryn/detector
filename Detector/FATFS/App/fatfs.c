/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "fatfs.h"
#include "messages.h"
/* USER CODE END Header */

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */
char Filename[FILE_COUNT][FILENAME_LENGTH];
extern RTC_HandleTypeDef hrtc;
extern SD_HandleTypeDef hsd;

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */

  memset(&SDFatFS, 0x00, sizeof(FATFS));
  memset(Filename, 0x00, FILENAME_LENGTH);

  if( retSD == 0 )
  {
    if( f_mount( &SDFatFS, (TCHAR const*)SDPath, 1 ) != FR_OK )
    {
      Error_Handler();
    }
  }
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
  HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );

  return  ((DWORD)(sDate.Year + 20) << 25)  // Year 2014
          | ((DWORD)sDate.Month << 21)            // Month 7
          | ((DWORD)sDate.Date << 16)           // Mday 10
          | ((DWORD)sTime.Hours << 11)           // Hour 16
          | ((DWORD)sTime.Minutes << 5)             // Min 0
          | ((DWORD)sTime.Seconds >> 1);            // Sec 0
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
void FatFS_NewFile( FileType_t fileType )
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  char ext[4];
  uint32_t FreeSize;
  uint32_t SinKB;

  HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BCD );
  HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BCD );

  SinKB = 1024 / SDFatFS.ssize;
  FreeSize = SDFatFS.free_clust * SDFatFS.csize / SinKB;

  Message( Message_FATFS, Message_Debug, "Free size: %lu kB, %lu clusters", (unsigned long) FreeSize, (unsigned long) SDFatFS.free_clust );

  switch( fileType )
  {
    case FILE_RESULT:
      strcpy( ext, "csv" );
      break;

    case FILE_LOG:
      strcpy( ext, "log" );
      break;

    default:
      break;
  }

  sprintf( Filename[fileType], "%02X-%02X-%02X_%02X-%02X-%02X.%s", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds, ext );

  if( f_open( &SDFile, Filename[fileType], (FA_CREATE_ALWAYS | FA_WRITE) ) == FR_OK )
  {
    f_close( &SDFile );
  } else {
    Error_Handler();
  }

}

void FatFS_WriteToFile( FileType_t fileType, bool ReplaceDotToComa, char *format, ... )
{
  va_list args;
  char buffer[WRITE_BUFFER];
  uint32_t wbytes; /* File write counts */

  if( SDFatFS.free_clust > 0 )
  {
    if( f_open( &SDFile, Filename[fileType], (FA_OPEN_ALWAYS | FA_WRITE) ) == FR_OK )
    {
      f_lseek( &SDFile, SDFile.fsize );

      memset( buffer, 0x00, sizeof(char) * WRITE_BUFFER );

      va_start( args, format );
      vsprintf( buffer, format, args );
      va_end( args );

      if( ReplaceDotToComa )
      {
        for( uint16_t i = 0; i < WRITE_BUFFER; i++ )
        {
          if( buffer[i] == '.' )
          {
            buffer[i] = ',';
          } else if( buffer[i] == 0 )
          {
            break;
          }
        }
      }

      f_write( &SDFile, buffer, strlen(buffer), (void *)&wbytes );
      f_close( &SDFile );
    } else {
      Error_Handler();
    }
  } else {
    Error_Handler();
  }

}

uint8_t BSP_SD_Init( void )
{
  uint8_t sd_state = MSD_OK;
  /* Check if the SD card is plugged in the slot */
  if ( BSP_SD_IsDetected() != SD_PRESENT )
  {
    return MSD_ERROR;
  }
  /* HAL SD initialization */
  sd_state = HAL_SD_Init( &hsd );

  if( HAL_SD_ConfigWideBusOperation( &hsd, SDIO_BUS_WIDE_4B ) != HAL_OK )
  {
    return MSD_ERROR;
  }

  return sd_state;
}

uint32_t FatFS_GetFreeSpace()
{
  uint32_t FreeSize;
  uint32_t SinKB;

  SinKB = 1024 / SDFatFS.ssize;
  FreeSize = SDFatFS.free_clust * SDFatFS.csize / SinKB;

  return FreeSize;
}

uint32_t FatFS_GetCardSize()
{
  uint32_t CardSize;
  uint32_t SinKB;

  SinKB = 1024 / SDFatFS.ssize;
  CardSize = (SDFatFS.n_fatent - 2) * SDFatFS.csize / SinKB;

  return CardSize;
}

/* USER CODE END Application */
