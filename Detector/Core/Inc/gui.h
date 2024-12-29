/*
 * gui.h
 *
 *  Created on: Dec 23, 2024
 *      Author: Pawel
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include "sensors.h"
#include "TouchPanel.h"

#define    BUFFER_SIZE         40
#define    BUTTONS_MENU_MAX    20
#define    BUTTONS_LOCAL_MAX   20

typedef enum {
  GUI_State_Reset,
  GUI_State_Init,
  GUI_State_Main,
  GUI_State_SetSensors_1,
  GUI_State_SetSensors_2,
  GUI_State_SetSensors_3,
  GUI_State_SetSensors_4,
  GUI_State_SetSensors_5,
  GUI_State_SetTime,
  GUI_State_CalibrateTP,
  GUI_State_SetSDCard
} GUI_StateMachine_t;

typedef struct {
  uint16_t xStart, yStart, xSize, ySize;
  GUI_StateMachine_t nextState;
} GUI_MenuButton_t;

typedef struct {
  uint16_t xStart, yStart, xSize, ySize;
  uint8_t arg;
} GUI_LocalButton_t;

void GUI_Text( uint16_t x, uint16_t y, uint16_t color, uint16_t bkColor, char *format, ... );
void GUI_DrawFrame( uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize, uint16_t frameColor );
void GUI_MenuButton( uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize, uint16_t frameColor, uint16_t textColor, uint16_t bkColor, GUI_StateMachine_t nextState, char *format, ... );
void GUI_LocalButton( uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize, uint16_t frameColor, uint16_t textColor, uint16_t bkColor, uint8_t arg, char *format, ... );
void GUI_ClearButtons();

uint16_t GUI_LevelColor (Sensors_Level_t level);

void GUI_Init( uint8_t time );
void GUI_Clear();

void GUI_Main();
void GUI_SetTime();
void GUI_CalibrateTP();
void GUI_SetSensors_1();
void GUI_SetSensors_2();
void GUI_SetSensors_3();
void GUI_SetSensors_4();
void GUI_SetSensors_5();
void GUI_SetSDCard();

void GUI_Handle();
void GUI_HandleButton();
void GUI_CheckDate( RTC_DateTypeDef *sDate );
uint8_t GUI_IsLeapYear( uint16_t nYear );

#endif /* INC_GUI_H_ */
