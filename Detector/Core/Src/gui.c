/*
 * gui.c
 *
 *  Created on: Dec 23, 2024
 *      Author: Pawel
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "gui.h"
#include "GLCD.h"
#include "sensors.h"
#include "AsciiLib.h"
#include "TouchPanel.h"
#include "messages.h"
#include "fatfs.h"

#define LCD_WIDTH       240
#define LCD_HEIGHT      320

#define MARGIN_LEFT     2
#define MARGIN_RIGHT    2
#define MARGIN_TOP     2
#define MARGIN_BOTTOM   2

#define FONT_HEIGHT     16
#define FONT_WIDTH      8

extern RTC_HandleTypeDef hrtc;
static GUI_StateMachine_t StateActual = GUI_State_Reset;
static GUI_MenuButton_t ButtonsMenu[BUTTONS_MENU_MAX];
static GUI_LocalButton_t ButtonsLocal[BUTTONS_LOCAL_MAX];
static uint8_t ButtonsMenuAmount, ButtonsLocalAmount;
static uint8_t argument = 0;
static bool TouchPanelLocked = false;
static bool StateHasChanged = false;

inline uint16_t GUI_LevelColor (Sensors_Level_t level)
{
  switch(level)
  {
    case SENSORS_LEVEL_GOOD:
      return Green;
    case SENSORS_LEVEL_WARNING:
      return Yellow;
    case SENSORS_LEVEL_CRITICAL:
      return Red;
    default:
      return Blue;
  }
}

void GUI_Text( uint16_t x, uint16_t y, uint16_t color, uint16_t bkColor, char *format, ... )
{
  char buffer[BUFFER_SIZE];
  va_list args;

  memset( buffer, 0, BUFFER_SIZE * sizeof(char) );
  va_start( args, format );
  vsprintf( buffer, format, args );
  va_end( args );

  LCD_Text( x, y, buffer, color, bkColor );

  return;
}

void GUI_DrawFrame( uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize, uint16_t frameColor )
{
  LCD_DrawLine( xStart, yStart, (xStart + xSize), yStart, frameColor );   /* TOP FRAME */
  LCD_DrawLine( xStart, (yStart + ySize), (xStart + xSize), (yStart + ySize), frameColor );   /* BOTTOM FRAME */
  LCD_DrawLine( xStart, yStart, xStart, (yStart + ySize), frameColor );   /* LEFT FRAME */
  LCD_DrawLine( (xStart + xSize), yStart, (xStart + xSize), (yStart + ySize), frameColor );   /* RIGHT FRAME */

  return;
}

void GUI_MenuButton( uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize, uint16_t frameColor, uint16_t textColor, uint16_t bkColor, GUI_StateMachine_t nextState, char *format, ... )
{
  char buffer[BUFFER_SIZE];
  double lengthBuffer, posX, posY;
  va_list args;

  if(ButtonsMenuAmount < BUTTONS_MENU_MAX)
  {
    /* Frame */
    GUI_DrawFrame( xStart, yStart, xSize, ySize, frameColor );

    /* Text */
    memset( buffer, 0, BUFFER_SIZE * sizeof(char) );
    va_start( args, format );
    vsprintf( buffer, format, args );
    va_end( args );

    lengthBuffer = strlen(buffer) * FONT_WIDTH;
    posX = xStart + (xSize - lengthBuffer) / 2;
    posY = yStart + (ySize - FONT_HEIGHT) / 2;

    LCD_Text( (uint16_t) posX, (uint16_t) posY, buffer, textColor, bkColor );

    ButtonsMenu[ButtonsMenuAmount].xStart = xStart;
    ButtonsMenu[ButtonsMenuAmount].yStart = yStart;
    ButtonsMenu[ButtonsMenuAmount].xSize = xSize;
    ButtonsMenu[ButtonsMenuAmount].ySize = ySize;
    ButtonsMenu[ButtonsMenuAmount].nextState = nextState;

    Message(Message_TouchPanel, Message_Debug, "Added menu button (%u): %u, %u, %u, %u, %u", ButtonsMenuAmount, xStart, yStart, xSize, ySize, nextState);

    ButtonsMenuAmount++;
  }
}

void GUI_LocalButton( uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize, uint16_t frameColor, uint16_t textColor, uint16_t bkColor, uint8_t arg, char *format, ... )
{
  char buffer[BUFFER_SIZE];
  double lengthBuffer, posX, posY;
  va_list args;

  if(ButtonsLocalAmount < BUTTONS_LOCAL_MAX)
  {
    /* Frame */
    GUI_DrawFrame( xStart, yStart, xSize, ySize, frameColor );

    /* Text */
    memset( buffer, 0, BUFFER_SIZE * sizeof(char) );
    va_start( args, format );
    vsprintf( buffer, format, args );
    va_end( args );

    lengthBuffer = strlen(buffer) * FONT_WIDTH;
    posX = xStart + (xSize - lengthBuffer) / 2;
    posY = yStart + (ySize - FONT_HEIGHT) / 2;

    LCD_Text( (uint16_t) posX, (uint16_t) posY, buffer, textColor, bkColor );

    ButtonsLocal[ButtonsLocalAmount].xStart = xStart;
    ButtonsLocal[ButtonsLocalAmount].yStart = yStart;
    ButtonsLocal[ButtonsLocalAmount].xSize = xSize;
    ButtonsLocal[ButtonsLocalAmount].ySize = ySize;
    ButtonsLocal[ButtonsLocalAmount].arg = arg;

    Message(Message_TouchPanel, Message_Debug, "Added local button (%u): %u, %u, %u, %u, %u", ButtonsLocalAmount, xStart, yStart, xSize, ySize, arg);

    ButtonsLocalAmount++;
  }
}

void GUI_ClearButtons()
{
  ButtonsMenuAmount = 0;
  ButtonsLocalAmount = 0;
  for( uint8_t i = 0; i < BUTTONS_MENU_MAX; i++ )
  {
    ButtonsMenu[i].xStart = ButtonsMenu[i].xSize = ButtonsMenu[i].yStart = ButtonsMenu[i].ySize = 0;
    ButtonsMenu[i].nextState = GUI_State_Init;
  }

  for( uint8_t i = 0; i < BUTTONS_LOCAL_MAX; i++ )
  {
    ButtonsLocal[i].xStart = ButtonsLocal[i].xSize = ButtonsLocal[i].yStart = ButtonsLocal[i].ySize = 0;
    ButtonsLocal[i].arg = 0;
  }

  Message(Message_TouchPanel, Message_Debug, "Deleted all buttons");

}

void GUI_Init( uint8_t time )
{
  if( StateActual == GUI_State_Reset )
  {
    LCD_Clear( Yellow );
    StateHasChanged = true;
    StateActual = GUI_State_Init;
  }

  GUI_Text( 68, 134, Black, Yellow, "Initialization" );
  GUI_Text( 110, 150, Black, Yellow, "%2us", time );

  return;
}

void GUI_Clear()
{
  LCD_Clear( Black );
  GUI_ClearButtons();

  return;
}

void GUI_Main()
{
  /* Sensors values */
  GUI_Text( MARGIN_LEFT, (2 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_PM1_0 )), Black,  "PM1.0: %6.1f %cg/m%c", Sensors_GetValue( SENSORS_TYPE_PM1_0, SENSORS_LEVEL_ACTUAL ), SYM_MICRO, SYM_CUBEROOT );

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_PM2_5 )), Black,  "PM2.5: %6.1f %cg/m%c", Sensors_GetValue( SENSORS_TYPE_PM2_5, SENSORS_LEVEL_ACTUAL ), SYM_MICRO, SYM_CUBEROOT );

  GUI_Text( MARGIN_LEFT, (4 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_PM4_0 )), Black,  "PM4.0: %6.1f %cg/m%c", Sensors_GetValue( SENSORS_TYPE_PM4_0, SENSORS_LEVEL_ACTUAL ), SYM_MICRO, SYM_CUBEROOT );

  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_PM10 )), Black,  "PM10:  %6.1f %cg/m%c", Sensors_GetValue( SENSORS_TYPE_PM10, SENSORS_LEVEL_ACTUAL ), SYM_MICRO, SYM_CUBEROOT );

  GUI_Text( MARGIN_LEFT, (6 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_TEMP1 )), Black,  "TEMP1: %6.1f %cC", Sensors_GetValue( SENSORS_TYPE_TEMP1, SENSORS_LEVEL_ACTUAL ), SYM_DEGREE );

  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_RH1 )), Black,  "RH1:   %6.1f %%", Sensors_GetValue( SENSORS_TYPE_RH1, SENSORS_LEVEL_ACTUAL ) );

  GUI_Text( MARGIN_LEFT, (8 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_VOC )), Black,  "VOC:   %6.1f (INDEX)", Sensors_GetValue( SENSORS_TYPE_VOC, SENSORS_LEVEL_ACTUAL ) );

  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_NOC )), Black,  "NOC:   %6.1f (INDEX)", Sensors_GetValue( SENSORS_TYPE_NOC, SENSORS_LEVEL_ACTUAL ) );

  GUI_Text( MARGIN_LEFT, (10 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_CO2 )), Black, "CO%c:   %6.0f ppm", SYM_SUBTWO, Sensors_GetValue( SENSORS_TYPE_CO2, SENSORS_LEVEL_ACTUAL ) );

  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_TEMP2 )), Black, "TEMP2: %6.1f %cC", Sensors_GetValue( SENSORS_TYPE_TEMP2, SENSORS_LEVEL_ACTUAL ), SYM_DEGREE );

  GUI_Text( MARGIN_LEFT, (12 * FONT_HEIGHT + MARGIN_TOP), GUI_LevelColor(Sensors_GetLevel( SENSORS_TYPE_RH2 )), Black, "RH2:   %6.1f %%", Sensors_GetValue( SENSORS_TYPE_RH2, SENSORS_LEVEL_ACTUAL ) );


  if( StateHasChanged )
  {
    /* Menu */
    GUI_MenuButton( 0, 220, 119, 50, White, White, Black, GUI_State_SetSensors_1, "Sensors" );
    GUI_MenuButton( 119, 220, 120, 50, White, White, Black, GUI_State_SetTime, "Screen" );
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_TurnOff, "Turn off" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_SetSDCard, "SD card" );
  }

  StateHasChanged = false;

  return;
}

uint8_t GUI_IsLeapYear( uint16_t nYear )
{
  if ((nYear % 4U) != 0U)
  {
    return 0U;
  }

  if ((nYear % 100U) != 0U)
  {
    return 1U;
  }

  if ((nYear % 400U) == 0U)
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}

void GUI_SetTime()
{
  RTC_DateTypeDef sDate;
  RTC_TimeTypeDef sTime;
  uint8_t maxDate;
  uint8_t isLeap;

  if( argument > 0 )
  {
    HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
    HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BIN );

    isLeap = GUI_IsLeapYear( sDate.Year );

    if( (sDate.Month == 4) || (sDate.Month == 6) || (sDate.Month == 9) || (sDate.Month == 11) )
    {
      maxDate = 30;
    } else if( !isLeap && (sDate.Month == 2) )
    {
      maxDate = 28;
    } else if( isLeap && (sDate.Month == 2) )
    {
      maxDate = 29;
    } else {
      maxDate = 31;
    }


    switch( argument )
    {
      case 1: /* Hours ++ */
        if( sTime.Hours < 23 )
        {
          sTime.Hours++;
        } else {
          sTime.Hours = 0;
        }
        break;

      case 2: /* Hours -- */
        if( sTime.Hours > 0 )
        {
          sTime.Hours--;
        } else {
          sTime.Hours = 23;
        }
        break;

      case 3: /* Minutes ++ */
        if( sTime.Minutes < 59 )
        {
          sTime.Minutes++;
        } else {
          sTime.Minutes = 0;
        }
        break;

      case 4: /* Minutes ++ */
        if( sTime.Minutes > 0 )
        {
          sTime.Minutes--;
        } else {
          sTime.Minutes = 59;
        }
        break;

      case 5: /* Seconds ++ */
        if( sTime.Seconds < 59 )
        {
          sTime.Seconds++;
        } else {
          sTime.Seconds = 0;
        }
        break;

      case 6: /* Seconds -- */
        if( sTime.Seconds > 0 )
        {
          sTime.Seconds--;
        } else {
          sTime.Seconds = 59;
        }
        break;

      case 7:  /* Years ++ */
        if( sDate.Year < 99 )
        {
          sDate.Year++;
        } else {
          sDate.Year = 0;
        }
        break;

      case 8:  /* Years -- */
        if( sDate.Year > 0 )
        {
          sDate.Year--;
        } else {
          sDate.Year = 99;
        }
        break;

      case 9: /* Months ++ */
        if( sDate.Month < 12 )
        {
          sDate.Month++;
        } else {
          sDate.Month = 1;
        }
        break;

      case 10: /* Months -- */
        if( sDate.Month > 1 )
        {
          sDate.Month--;
        } else {
          sDate.Month = 12;
        }

        break;

      case 11: /* Date ++ */

        if( sDate.Date < maxDate )
        {
          sDate.Date++;
        } else {
          sDate.Date = 1;
        }
        break;

      case 12: /* Date -- */
        if( (sDate.Month == 4) || (sDate.Month == 6) || (sDate.Month == 9) || (sDate.Month == 11) )
        {
          maxDate = 30;
        } else if( !GUI_IsLeapYear( sDate.Year ) && (sDate.Month == 2) )
        {
          maxDate = 28;
        } else if( GUI_IsLeapYear( sDate.Year ) && (sDate.Month == 2) )
        {
          maxDate = 29;
        } else {
          maxDate = 31;
        }

        if( sDate.Date > 1 )
        {
          sDate.Date--;
        } else {
          sDate.Date = maxDate;
        }
        break;

      default:
        break;
    }

    if( argument <= 6 )
    {
      HAL_RTC_WaitForSynchro( &hrtc );
      HAL_RTC_SetTime( &hrtc, &sTime, RTC_FORMAT_BIN );

      Message( Message_TouchPanel, Message_Debug, "Argument detected %u", argument );
      argument = 0;

    } else {
      isLeap = GUI_IsLeapYear( sDate.Year );

      if( ((sDate.Month == 4) || (sDate.Month == 6) || (sDate.Month == 9) || (sDate.Month == 11)) && (sDate.Date == 31) )
      {
        sDate.Date = 30;
      } else if( !isLeap && (sDate.Month == 2) && (sDate.Date > 28) )
      {
        sDate.Date = 28;
      } else if( isLeap && (sDate.Month == 2) && (sDate.Date > 29) )
      {
        sDate.Date = 29;
      }

      HAL_RTC_WaitForSynchro( &hrtc );
      HAL_RTC_SetDate( &hrtc, &sDate, RTC_FORMAT_BIN );

      Message( Message_GUI, Message_Debug, "Argument detected %u", argument );
      argument = 0;
    }
  }

  HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BCD );
  HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BCD );

  /* Date & time  */
  GUI_Text( MARGIN_LEFT, MARGIN_TOP, White, Black, "%02X-%02X-%02X", sDate.Date, sDate.Month, sDate.Year );
  GUI_Text( (LCD_WIDTH - (8 * FONT_WIDTH + MARGIN_RIGHT)), MARGIN_TOP, White, Black, "%02X:%02X:%02X", sTime.Hours, sTime.Minutes, sTime.Seconds );

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black,       "Hour:    %02X", sTime.Hours );
  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP) + 5, White, Black,   "Minutes: %02X", sTime.Minutes );
  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP) + 10, White, Black,  "Seconds: %02X", sTime.Seconds );
  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP) + 15, White, Black,  "Year:    %02X", sDate.Year );
  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP) + 20, White, Black, "Month:   %02X", sDate.Month );
  GUI_Text( MARGIN_LEFT, (13 * FONT_HEIGHT + MARGIN_TOP) + 25, White, Black, "Day:     %02X", sDate.Date );

  if( StateHasChanged )
  {
    /* Set time & date */
    GUI_LocalButton( 120, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, 1, "+" );
    GUI_LocalButton( 170, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, 2, "-" );
    GUI_LocalButton( 120, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, 3, "+" );
    GUI_LocalButton( 170, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, 4, "-" );
    GUI_LocalButton( 120, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, 5, "+" );
    GUI_LocalButton( 170, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, 6, "-" );
    GUI_LocalButton( 120, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, 7, "+" );
    GUI_LocalButton( 170, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, 8, "-" );
    GUI_LocalButton( 120, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, 9, "+" );
    GUI_LocalButton( 170, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, 10, "-" );
    GUI_LocalButton( 120, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, 11, "+" );
    GUI_LocalButton( 170, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, 12, "-" );

    /* Menu */
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_CalibrateTP, "Touchpanel" );
  }

  StateHasChanged = false;
}

void GUI_CalibrateTP()
{
  TouchPanel_Calibrate();

  StateHasChanged = false;
}

void GUI_SetSensors_1()
{
  uint8_t operation, sensor;

  if( argument > 0 )
  {
    operation = (argument & 0xF0);
    sensor = (argument & 0x0F);

    switch( operation )
    {
      case 0x10: /* Add for warning */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_WARNING,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      case 0x20: /* Sub for warning */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_WARNING,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      case 0x30: /* Add for critical */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_CRITICAL,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;

        break;
      case 0x40: /* Sub for critical */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_CRITICAL,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      default:
        break;
    }
  }

  argument = 0;

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black,       "PM1.0 W: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM1_0, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP) + 5, White, Black,   "PM1.0 C: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM1_0, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP) + 10, White, Black,  "PM2.5 W: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM2_5, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP) + 15, White, Black,  "PM2.5 C: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM2_5, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP) + 20, White, Black, "PM4.0 W: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM4_0, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (13 * FONT_HEIGHT + MARGIN_TOP) + 25, White, Black, "PM4.0 C: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM4_0, SENSORS_LEVEL_CRITICAL ) );

  if( StateHasChanged )
  {
    /* Argument - 0xYZ, where Y is:
     * 1 - add warning
     * 2 - sub warning
     * 3 - add critical
     * 4 - sub critical
     * Z is number of sensor */

    GUI_LocalButton( 140, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_PM1_0), "+" );
    GUI_LocalButton( 190, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_PM1_0), "-" );
    GUI_LocalButton( 140, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_PM1_0), "+" );
    GUI_LocalButton( 190, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_PM1_0), "-" );
    GUI_LocalButton( 140, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_PM2_5), "+" );
    GUI_LocalButton( 190, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_PM2_5), "-" );
    GUI_LocalButton( 140, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_PM2_5), "+" );
    GUI_LocalButton( 190, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_PM2_5), "-" );
    GUI_LocalButton( 140, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_PM4_0), "+" );
    GUI_LocalButton( 190, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_PM4_0), "-" );
    GUI_LocalButton( 140, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_PM4_0), "+" );
    GUI_LocalButton( 190, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_PM4_0), "-" );

    /* Menu */
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_SetSensors_2, "Next" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );
  }

  StateHasChanged = false;
}

void GUI_SetSensors_2()
{
  uint8_t operation, sensor;

  if( argument > 0 )
  {
    operation = (argument & 0xF0);
    sensor = (argument & 0x0F);

    switch( operation )
    {
      case 0x10: /* Add for warning */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_WARNING,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      case 0x20: /* Sub for warning */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_WARNING,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      case 0x30: /* Add for critical */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_CRITICAL,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;

        break;
      case 0x40: /* Sub for critical */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_CRITICAL,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      default:
        break;
    }
  }

  argument = 0;

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black,       "PM10 W: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM10, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP) + 5, White, Black,   "PM10 C: %4.0f", Sensors_GetValue( SENSORS_TYPE_PM10, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP) + 10, White, Black,  "CO2 W:  %4.0f", Sensors_GetValue( SENSORS_TYPE_CO2, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP) + 15, White, Black,  "CO2 C:  %4.0f", Sensors_GetValue( SENSORS_TYPE_CO2, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP) + 20, White, Black, "VOC W:  %4.0f", Sensors_GetValue( SENSORS_TYPE_VOC, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (13 * FONT_HEIGHT + MARGIN_TOP) + 25, White, Black, "VOC C:  %4.0f", Sensors_GetValue( SENSORS_TYPE_VOC, SENSORS_LEVEL_CRITICAL ) );

  if( StateHasChanged )
  {
    /* Argument - 0xYZ, where Y is:
     * 1 - add warning
     * 2 - sub warning
     * 3 - add critical
     * 4 - sub critical
     * Z is number of sensor */

    GUI_LocalButton( 140, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_PM10), "+" );
    GUI_LocalButton( 190, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_PM10), "-" );
    GUI_LocalButton( 140, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_PM10), "+" );
    GUI_LocalButton( 190, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_PM10), "-" );
    GUI_LocalButton( 140, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_CO2), "+" );
    GUI_LocalButton( 190, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_CO2), "-" );
    GUI_LocalButton( 140, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_CO2), "+" );
    GUI_LocalButton( 190, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_CO2), "-" );
    GUI_LocalButton( 140, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_VOC), "+" );
    GUI_LocalButton( 190, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_VOC), "-" );
    GUI_LocalButton( 140, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_VOC), "+" );
    GUI_LocalButton( 190, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_VOC), "-" );

    /* Menu */
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_SetSensors_3, "Next" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );
  }

  StateHasChanged = false;
}

void GUI_SetSensors_3()
{
  uint8_t operation, sensor;

  if( argument > 0 )
  {
    operation = (argument & 0xF0);
    sensor = (argument & 0x0F);

    switch( operation )
    {
      case 0x10: /* Add for warning */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_WARNING,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      case 0x20: /* Sub for warning */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_WARNING,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      case 0x30: /* Add for critical */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_CRITICAL,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;

        break;
      case 0x40: /* Sub for critical */
        if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
        {
          Sensors_SetValue( sensor,
                            SENSORS_LEVEL_CRITICAL,
                            (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
        }
        break;
      default:
        break;
    }
  }

  argument = 0;

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black,       "NOC W:   %4.0f", Sensors_GetValue( SENSORS_TYPE_NOC, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP) + 5, White, Black,   "NOC C:   %4.0f", Sensors_GetValue( SENSORS_TYPE_NOC, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP) + 10, White, Black,  "TEMPL C: %4.0f", Sensors_GetValue( SENSORS_TYPE_TEMP1, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP) + 15, White, Black,  "TEMPL W: %4.0f", Sensors_GetValue( SENSORS_TYPE_TEMP1, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP) + 20, White, Black, "TEMPH W: %4.0f", Sensors_GetValue( SENSORS_TYPE_TEMP2, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (13 * FONT_HEIGHT + MARGIN_TOP) + 25, White, Black, "TEMPH C: %4.0f", Sensors_GetValue( SENSORS_TYPE_TEMP2, SENSORS_LEVEL_CRITICAL ) );

  if( StateHasChanged )
  {
    /* Argument - 0xYZ, where Y is:
     * 1 - add warning
     * 2 - sub warning
     * 3 - add critical
     * 4 - sub critical
     * Z is number of sensor */

    GUI_LocalButton( 140, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_NOC), "+" );
    GUI_LocalButton( 190, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_NOC), "-" );
    GUI_LocalButton( 140, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_NOC), "+" );
    GUI_LocalButton( 190, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_NOC), "-" );
    GUI_LocalButton( 140, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_TEMP1), "+" );
    GUI_LocalButton( 190, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_TEMP1), "-" );
    GUI_LocalButton( 140, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_TEMP1), "+" );
    GUI_LocalButton( 190, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_TEMP1), "-" );
    GUI_LocalButton( 140, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_TEMP2), "+" );
    GUI_LocalButton( 190, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_TEMP2), "-" );
    GUI_LocalButton( 140, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_TEMP2), "+" );
    GUI_LocalButton( 190, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_TEMP2), "-" );


    /* Menu */
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_SetSensors_4, "Next" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );
  }

  StateHasChanged = false;
}

void GUI_SetSensors_4()
{
  uint8_t operation, sensor;

  if( argument > 0 )
  {
    if( argument < 0xAA )
    {
      operation = (argument & 0xF0);
      sensor = (argument & 0x0F);

      switch( operation )
      {
        case 0x10: /* Add for warning */
          if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
          {
            Sensors_SetValue( sensor,
                              SENSORS_LEVEL_WARNING,
                              (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
          }
          break;
        case 0x20: /* Sub for warning */
          if( Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
          {
            Sensors_SetValue( sensor,
                              SENSORS_LEVEL_WARNING,
                              (Sensors_GetValue( sensor, SENSORS_LEVEL_WARNING ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
          }
          break;
        case 0x30: /* Add for critical */
          if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) < Sensors_GetValue( sensor, SENSORS_LEVEL_MAX ) )
          {
            Sensors_SetValue( sensor,
                              SENSORS_LEVEL_CRITICAL,
                              (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) + Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
          }
          break;

          break;
        case 0x40: /* Sub for critical */
          if( Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) > Sensors_GetValue( sensor, SENSORS_LEVEL_MIN ) )
          {
            Sensors_SetValue( sensor,
                              SENSORS_LEVEL_CRITICAL,
                              (Sensors_GetValue( sensor, SENSORS_LEVEL_CRITICAL ) - Sensors_GetValue( sensor, SENSORS_LEVEL_STEP )) );
          }
          break;
        default:
          break;
      }
    } else {
      switch( argument )
      {
        case 0xAA: /* Scroff++ */
          if( GetScrOff() < SCROFF_MAX )
          {
            SetScrOff( GetScrOff() + SCROFF_STEP );
          }
          break;
        case 0xAB: /* Scroff-- */
          if( GetScrOff() > SCROFF_MIN )
          {
            SetScrOff( GetScrOff() - SCROFF_STEP );
          }
          break;
        case 0xAC: /* Period++ */
          if( GetPeriod() < PERIOD_MAX )
          {
            SetPeriod( GetPeriod() + PERIOD_STEP );
          }
          break;
        case 0xAD: /* Period-- */
          if( GetPeriod() > PERIOD_MIN )
          {
            SetPeriod( GetPeriod() - PERIOD_STEP );
          }
          break;
        default:
          break;
      }
    }
  }

  argument = 0;

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black,       "RHL C:   %4.0f", Sensors_GetValue( SENSORS_TYPE_RH1, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP) + 5, White, Black,   "RHL W:   %4.0f", Sensors_GetValue( SENSORS_TYPE_RH1, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP) + 10, White, Black,  "RHH W:   %4.0f", Sensors_GetValue( SENSORS_TYPE_RH2, SENSORS_LEVEL_WARNING ) );
  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP) + 15, White, Black,  "RHH C:   %4.0f", Sensors_GetValue( SENSORS_TYPE_RH2, SENSORS_LEVEL_CRITICAL ) );
  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP) + 20, White, Black, "SCR OFF: %4.0f", ((double) GetScrOff()) );
  GUI_Text( MARGIN_LEFT, (13 * FONT_HEIGHT + MARGIN_TOP) + 25, White, Black, "Period:  %4.1f", ((double) GetPeriod()) / 1000 );

  if( StateHasChanged )
  {
    /* Argument - 0xYZ, where Y is:
     * 1 - add warning
     * 2 - sub warning
     * 3 - add critical
     * 4 - sub critical
     * Z is number of sensor */

    GUI_LocalButton( 140, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_RH1), "+" );
    GUI_LocalButton( 190, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_RH1), "-" );
    GUI_LocalButton( 140, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_RH1), "+" );
    GUI_LocalButton( 190, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_RH1), "-" );
    GUI_LocalButton( 140, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x10 | SENSORS_TYPE_RH2), "+" );
    GUI_LocalButton( 190, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0x20 | SENSORS_TYPE_RH2), "-" );
    GUI_LocalButton( 140, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x30 | SENSORS_TYPE_RH2), "+" );
    GUI_LocalButton( 190, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0x40 | SENSORS_TYPE_RH2), "-" );
    GUI_LocalButton( 140, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0xAA), "+" );
    GUI_LocalButton( 190, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0xAB), "-" );
    GUI_LocalButton( 140, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0xAC), "+" );
    GUI_LocalButton( 190, (12.5 * FONT_HEIGHT + MARGIN_TOP) + 25, 40, 2 * FONT_HEIGHT, White, White, Black, (0xAD), "-" );

    /* Menu */
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_SetSensors_5, "Next" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );

  }

  StateHasChanged = false;
}

void GUI_SetSensors_5()
{
  if( argument > 0 )
  {
    switch( argument )
    {
      case 0xAA: /* SEN55 temperature offset ++ */
        if( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ) < Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_MAX ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ) + Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_STEP ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ));
        }
        break;
      case 0xAB: /* SEN55 temperature offset -- */
        if( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ) > Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_MIN ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ) - Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_STEP ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ));
        }
        break;
      case 0xBA: /* SEN55 temperature slope ++ */
        if( Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ) < Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_MAX ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ) + Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_STEP ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ));
        }
        break;
      case 0xBB: /* SEN55 temperature slope -- */
        if( Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ) > Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_MIN ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ) - Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_STEP ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ));
        }
        break;
      case 0xCA: /* SEN55 temperature time ++ */
        if( Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ) < Sensors_SEN55_GetTempTime( SENSORS_LEVEL_MAX ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ) + Sensors_SEN55_GetTempTime( SENSORS_LEVEL_STEP ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ));
        }
        break;
      case 0xCB: /* SEN55 temperature time -- */
        if( Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ) > Sensors_SEN55_GetTempTime( SENSORS_LEVEL_MIN ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ) - Sensors_SEN55_GetTempTime( SENSORS_LEVEL_STEP ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ));
        }
        break;
      case 0xDA: /* SEN55 temperature acceleration ++ */
        if( Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ) < Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_MAX ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ) + Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_STEP ));
        }
        break;
      case 0xDB: /* SEN55 temperature acceleration -- */
        if( Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ) > Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_MIN ) )
        {
          Sensors_SEN55_SetTempOffset( Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ),
                                       Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ) - Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_STEP ));
        }
        break;
      case 0xEA: /* SCD41 temperature slope ++ */
        if( Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_ACTUAL ) < Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_MAX ) )
        {
          Sensors_SCD41_SetTempOffset( Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_ACTUAL ) + Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_STEP ));
        }
        break;
      case 0xEB: /* SCD41 temperature slope -- */
        if( Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_ACTUAL ) > Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_MIN ) )
        {
          Sensors_SCD41_SetTempOffset( Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_ACTUAL ) - Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_STEP ));
        }
        break;
      default:
        break;
    }
  }

  argument = 0;

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black,       "T1_OFST: %4.1f", Sensors_SEN55_GetTempOffset( SENSORS_LEVEL_ACTUAL ) );
  GUI_Text( MARGIN_LEFT, (5 * FONT_HEIGHT + MARGIN_TOP) + 5, White, Black,   "T1_SLOP: %4.1f", Sensors_SEN55_GetTempSlope( SENSORS_LEVEL_ACTUAL ) );
  GUI_Text( MARGIN_LEFT, (7 * FONT_HEIGHT + MARGIN_TOP) + 10, White, Black,  "T1_TIME: %4u", Sensors_SEN55_GetTempTime( SENSORS_LEVEL_ACTUAL ) );
  GUI_Text( MARGIN_LEFT, (9 * FONT_HEIGHT + MARGIN_TOP) + 15, White, Black,  "T1_ACC:  %4u", Sensors_SEN55_GetTempAccel( SENSORS_LEVEL_ACTUAL ) );
  GUI_Text( MARGIN_LEFT, (11 * FONT_HEIGHT + MARGIN_TOP) + 20, White, Black, "T2_OFST: %4.1f", Sensors_SCD41_GetTempOffset( SENSORS_LEVEL_ACTUAL ) );

  if( StateHasChanged )
  {
    /* Argument - 0xYZ, where Y is:
     * 1 - add warning
     * 2 - sub warning
     * 3 - add critical
     * 4 - sub critical
     * Z is number of sensor */

    GUI_LocalButton( 140, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0xAA), "+" );
    GUI_LocalButton( 190, (2.5 * FONT_HEIGHT + MARGIN_TOP), 40, 2 * FONT_HEIGHT, White, White, Black, (0xAB), "-" );
    GUI_LocalButton( 140, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0xBA), "+" );
    GUI_LocalButton( 190, (4.5 * FONT_HEIGHT + MARGIN_TOP) + 5, 40, 2 * FONT_HEIGHT, White, White, Black, (0xBB), "-" );
    GUI_LocalButton( 140, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0xCA), "+" );
    GUI_LocalButton( 190, (6.5 * FONT_HEIGHT + MARGIN_TOP) + 10, 40, 2 * FONT_HEIGHT, White, White, Black, (0xCB), "-" );
    GUI_LocalButton( 140, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0xDA), "+" );
    GUI_LocalButton( 190, (8.5 * FONT_HEIGHT + MARGIN_TOP) + 15, 40, 2 * FONT_HEIGHT, White, White, Black, (0xDB), "-" );
    GUI_LocalButton( 140, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0xEA), "+" );
    GUI_LocalButton( 190, (10.5 * FONT_HEIGHT + MARGIN_TOP) + 20, 40, 2 * FONT_HEIGHT, White, White, Black, (0xEB), "-" );

    /* Menu */
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_SetSensors_6, "Next" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );

  }

  StateHasChanged = false;
}

void GUI_SetSensors_6()
{
  if( argument > 0 )
  {
    switch( argument )
    {
      case 1:
        GUI_DrawFrame( 0, (MARGIN_TOP + 20), 119, 50, Yellow );
        if( Sensors_SEN55_CleanFan() )
        {
          GUI_DrawFrame( 0, (MARGIN_TOP + 20), 119, 50, Green );
        } else {
          GUI_DrawFrame( 0, (MARGIN_TOP + 20), 119, 50, Red );
        }
        break;
      case 2:
        GUI_DrawFrame( 119, (MARGIN_TOP + 20), 120, 50, Yellow );
        if( Sensors_SCD41_PerformCalibration() )
        {
          GUI_DrawFrame( 119, (MARGIN_TOP + 20), 120, 50, Green );
        } else {
          GUI_DrawFrame( 119, (MARGIN_TOP + 20), 120, 50, Red );
        }
        break;
      case 3:
        GUI_DrawFrame( 0, (MARGIN_TOP + 70), 119, 50, Yellow );
        ApplyDefaults();
        GUI_DrawFrame( 0, (MARGIN_TOP + 70), 119, 50, Green );
        break;
    }
  }

  argument = 0;

  if( StateHasChanged )
  {
    GUI_LocalButton( 0, (MARGIN_TOP + 20), 119, 50, White, White, Black, 1, "Clean fan" );
    GUI_LocalButton( 119, (MARGIN_TOP + 20), 120, 50, White, White, Black, 2, "Calibrate CO2" );
    GUI_LocalButton( 0, (MARGIN_TOP + 70), 119, 50, White, White, Black, 3, "Defaults" );

    /* Menu */
    GUI_MenuButton( 0, 270, 119, 49, White, White, Black, GUI_State_SetSensors_1, "Sensors" );
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );
  }

  StateHasChanged = false;
}

void GUI_SetSDCard()
{
  double FreeSize, CardSize;
  char FreeSizeUnit, CardSizeUnit;

  if( argument > 0 )
  {
    switch( argument )
    {
      case 1:
        GUI_DrawFrame( 0, (MARGIN_TOP + 20), 119, 50, Yellow );
        Main_NewMeasurementFile();
        GUI_DrawFrame( 0, (MARGIN_TOP + 20), 119, 50, Green );
        break;
      default:
        break;
    }
  }

  argument = 0;

  CardSize = FatFS_GetCardSize();
  CardSizeUnit = 'k';
  if( CardSize > 1024 )
  {
    CardSize /= 1024;
    CardSizeUnit = 'M';
  }
  if( CardSize > 1024 )
  {
    CardSize /= 1024;
    CardSizeUnit = 'G';
  }

  FreeSize = FatFS_GetFreeSpace();
  FreeSizeUnit = 'k';
  if( FreeSize > 1024 )
  {
    FreeSize /= 1024;
    FreeSizeUnit = 'M';
  }
  if( FreeSize > 1024 )
  {
    FreeSize /= 1024;
    FreeSizeUnit = 'G';
  }

  GUI_Text( MARGIN_LEFT, (3 * FONT_HEIGHT + MARGIN_TOP), White, Black, "Card size:  %7.3f %cB", CardSize, CardSizeUnit );
  GUI_Text( MARGIN_LEFT, (4 * FONT_HEIGHT + MARGIN_TOP), White, Black, "Free space: %7.3f %cB", FreeSize, FreeSizeUnit );

  if( StateHasChanged )
  {
    GUI_LocalButton( 0, 200, 119, 50, White, White, Black, 1, "New file" );

    /* Menu */
    GUI_MenuButton( 119, 270, 120, 49, White, White, Black, GUI_State_Main, "Main" );
  }

  StateHasChanged = false;
}

void GUI_HandleButton()
{
  static uint8_t released = 0;
  Coordinate point;

  TouchPanel_GetPoint( &point );

  if( (point.x < MAX_X) && (point.y < MAX_Y) )
  {
    released = 0;
    Main_TurnOnScreen();
    if(!TouchPanelLocked)
    {
      for( uint16_t i = 0; i < ButtonsMenuAmount; i++ )
      {
        if( (point.x >= ButtonsMenu[i].xStart) && (point.x <= (ButtonsMenu[i].xStart + ButtonsMenu[i].xSize)) &&
            (point.y >= ButtonsMenu[i].yStart) && (point.y <= (ButtonsMenu[i].yStart + ButtonsMenu[i].ySize)) )
        {
          TouchPanelLocked = true;
          StateActual = ButtonsMenu[i].nextState;
          StateHasChanged = true;
          Message( Message_TouchPanel, Message_Debug, "Button menu %u pressed. State changed to: %u", i, StateActual );
          GUI_Handle();
          return;
        }
      }

      for( uint16_t i = 0; i < ButtonsLocalAmount; i++ )
      {
        if( (point.x >= ButtonsLocal[i].xStart) && (point.x <= (ButtonsLocal[i].xStart + ButtonsLocal[i].xSize)) &&
            (point.y >= ButtonsLocal[i].yStart) && (point.y <= (ButtonsLocal[i].yStart + ButtonsLocal[i].ySize)) )
        {
          TouchPanelLocked = true;
          argument = ButtonsLocal[i].arg;
          Message( Message_TouchPanel, Message_Debug, "Button local %u pressed. Argument: %u", i, argument );
          GUI_Handle();
          return;
        }
      }
    }
  } else {
    if( TouchPanelLocked )
    {
      if( released < 10 )
      {
        released++;
      } else {
        TouchPanelLocked = false;
        released = 0;
        Message( Message_GUI, Message_Debug, "Button released" );
      }
    }
  }

  return;
}

void GUI_CheckDate( RTC_DateTypeDef *sDate )
{
  if( (sDate->Date != HAL_RTCEx_BKUPRead( &hrtc, BKP_REG_DAY )) ||
      (sDate->Month != HAL_RTCEx_BKUPRead( &hrtc, BKP_REG_MONTH )) ||
      (sDate->Year != HAL_RTCEx_BKUPRead( &hrtc, BKP_REG_YEAR )))
  {
    HAL_RTCEx_BKUPWrite( &hrtc, BKP_REG_DAY, sDate->Date );
    HAL_RTCEx_BKUPWrite( &hrtc, BKP_REG_MONTH, sDate->Month );
    HAL_RTCEx_BKUPWrite( &hrtc, BKP_REG_YEAR, sDate->Year );

    Message(Message_GUI, Message_Debug, "Date updated: %02X-%02X-%02X", sDate->Date, sDate->Month, sDate->Year);
  }
}

void GUI_Handle()
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  if( StateHasChanged )
  {
    GUI_Clear();
  }

  HAL_RTC_GetTime( &hrtc, &sTime, RTC_FORMAT_BCD );
  HAL_RTC_GetDate( &hrtc, &sDate, RTC_FORMAT_BCD );

  GUI_CheckDate( &sDate );

  /* Date & time  */
  GUI_Text( MARGIN_LEFT, MARGIN_TOP, White, Black, "%02X-%02X-%02X", sDate.Date, sDate.Month, sDate.Year );
  GUI_Text( (LCD_WIDTH - (8 * FONT_WIDTH + MARGIN_RIGHT)), MARGIN_TOP, White, Black, "%02X:%02X:%02X", sTime.Hours, sTime.Minutes, sTime.Seconds );

  switch( StateActual )
  {
    case GUI_State_Init:
      StateActual = GUI_State_Main;
      StateHasChanged = true;
      break;
    case GUI_State_SetTime:
      GUI_SetTime();
      break;
    case GUI_State_SetSensors_1:
      GUI_SetSensors_1();
      break;
    case GUI_State_SetSensors_2:
      GUI_SetSensors_2();
      break;
    case GUI_State_SetSensors_3:
      GUI_SetSensors_3();
      break;
    case GUI_State_SetSensors_4:
      GUI_SetSensors_4();
      break;
    case GUI_State_SetSensors_5:
      GUI_SetSensors_5();
      break;
    case GUI_State_SetSensors_6:
      GUI_SetSensors_6();
      break;
    case GUI_State_TurnOff:
      Main_TurnOff();
      break;
    case GUI_State_CalibrateTP:
      GUI_CalibrateTP();
      StateActual = GUI_State_Main;
      StateHasChanged = true;
      break;
    case GUI_State_SetSDCard:
      GUI_SetSDCard();
      break;
    case GUI_State_Main:
    default:
      GUI_Main();
      break;
  }
}
