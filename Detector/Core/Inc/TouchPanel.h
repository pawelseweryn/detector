/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.h
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef _TOUCHPANEL_H_
#define _TOUCHPANEL_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
typedef	struct POINT {
   uint16_t x;
   uint16_t y;
} Coordinate;


typedef struct Matrix {
long double An,  
            Bn,     
            Cn,   
            Dn,    
            En,    
            Fn,     
            Divider ;
} Matrix ;

/* Private define ------------------------------------------------------------*/
#define	CHX 	0x90
#define	CHY 	0xd0

/* Private function prototypes -----------------------------------------------*/				
void TP_Init( void );
Coordinate *Read_Ads7846( void );
void TouchPanel_Calibrate();
void TouchPanel_RestoreCalibration( );
void TouchPanel_DrawCross( uint16_t Xpos, uint16_t Ypos );
void TouchPanel_DrawPoint( uint16_t Xpos, uint16_t Ypos );
FunctionalState setCalibrationMatrix( Coordinate * displayPtr, Coordinate * screenPtr, Matrix * matrixPtr );
FunctionalState getDisplayPoint( Coordinate * displayPtr, Coordinate * screenPtr, Matrix * matrixPtr );
void TouchPanel_GetPoint( Coordinate *display );

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


