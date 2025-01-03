/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			GLCD.h
** Descriptions:		Has been tested SSD1289��ILI9320��R61505U��SSD1298��ST7781��SPFD5408B��ILI9325��ILI9328��
**						HX8346A��HX8347A
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2012-2-28
** Version:				2.1
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/

#ifndef __GLCD_H 
#define __GLCD_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

/* Private define ------------------------------------------------------------*/
#define DISP_ORIENTATION  180  /* angle 0 90 */

#if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )

#define  MAX_X  320
#define  MAX_Y  240   

#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )

#define  MAX_X  240
#define  MAX_Y  320   

#endif

/* LCD Registers */
#define R34            0x22

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

/******************************************************************************
* Function Name  : RGB565CONVERT
* Description    : 24λת��16λ
* Input          : - red: R
*                  - green: G 
*				   - blue: B
* Output         : None
* Return         : RGB ��ɫֵ
* Attention		 : None
*******************************************************************************/
#define RGB565CONVERT( red, green, blue ) \
            (uint16_t)( (( red   >> 3 ) << 11 ) | \
                        (( green >> 2 ) << 5  ) | \
                         ( blue  >> 3 ))

/* Private function prototypes -----------------------------------------------*/
void LCD_Initializtion( void );
void LCD_Clear( uint16_t Color );
uint16_t LCD_GetPoint( uint16_t Xpos, uint16_t Ypos );
void LCD_SetPoint( uint16_t Xpos, uint16_t Ypos, uint16_t point );
void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color );
void LCD_PutChar( uint16_t Xpos, uint16_t Ypos, char ASCII, uint16_t charColor, uint16_t bkColor );
void LCD_Text( uint16_t Xpos, uint16_t Ypos, char *str, uint16_t Color, uint16_t bkColor );
void LCD_Backlight( bool state );

#endif 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
