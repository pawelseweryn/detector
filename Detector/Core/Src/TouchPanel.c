/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.c
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

/* Includes ------------------------------------------------------------------*/
#include "TouchPanel.h"
#include "main.h"
#include "GLCD.h"
#include "messages.h"

/* Private variables ---------------------------------------------------------*/
static Matrix matrix ;

extern SPI_HandleTypeDef hspi1;

/* DisplaySample LCD�����϶�Ӧ��ads7843����ADֵ �磺LCD ����45,45 Ӧ�õ�X Y����ADC�ֱ�Ϊ3388,920 */	
Coordinate ScreenSample[3];
/* LCD�ϵ����� */
Coordinate DisplaySample[3] =   {
                                  { 45, 45 },
                                  { 45, 270},
                                  { 190,190}
	                            } ;

/* Private define ------------------------------------------------------------*/
#define THRESHOLD 2   /* ��ֵ���� */

/*******************************************************************************
* Function Name  : RD_AD
* Description    : ��ȡADCֵ
* Input          : None
* Output         : None
* Return         : ADS7843���ض��ֽ�����
* Attention		 : None
*******************************************************************************/
static int RD_AD( uint8_t cmd )
{ 
  uint8_t dataRx[3];
  uint8_t dataTx[3];
  uint16_t word;
  volatile HAL_StatusTypeDef status;

  dataTx[0] = cmd;
  dataTx[1] = dataTx[2] = 0x00;

  while( HAL_SPI_GetState( &hspi1 ) != HAL_SPI_STATE_READY );
  status = HAL_SPI_TransmitReceive( &hspi1, dataTx, dataRx, 3, 100 );

  if( status != HAL_OK )
  {
    Message( Message_LCD, Message_Error, "Transmission error" );
    return 0;
  }

  word = ((((uint16_t) dataRx[1]) << 8) | ((uint16_t) dataRx[2]));

  word >>= 3;
  word &= 0x0FFF;

  return word;
} 

/*******************************************************************************
* Function Name  : Read_X
* Description    : ��ȡADS7843ͨ��X+��ADCֵ 
* Input          : None
* Output         : None
* Return         : ADS7843����ͨ��X+��ADCֵ
* Attention		 : None
*******************************************************************************/
int Read_X( void )
{  
  int i; 

  HAL_GPIO_WritePin( TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET );
  i = RD_AD( CHX );
  HAL_GPIO_WritePin( TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET );

  return i;    
} 

/*******************************************************************************
* Function Name  : Read_Y
* Description    : ��ȡADS7843ͨ��Y+��ADCֵ
* Input          : None
* Output         : None
* Return         : ADS7843����ͨ��Y+��ADCֵ
* Attention		 : None
*******************************************************************************/
int Read_Y( void )
{  
  int i; 

  HAL_GPIO_WritePin( TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET );
  i = RD_AD( CHY );
  HAL_GPIO_WritePin( TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET );

  return i;     
} 

/*******************************************************************************
* Function Name  : TP_GetAdXY
* Description    : ��ȡADS7843 ͨ��X+ ͨ��Y+��ADCֵ
* Input          : None
* Output         : None
* Return         : ADS7843���� ͨ��X+ ͨ��Y+��ADCֵ 
* Attention		 : None
*******************************************************************************/
void TP_GetAdXY( int *x, int *y )
{ 
  int adx,ady; 

  adx = Read_X();
  ady = Read_Y();

  *x = adx;
  *y = ady;
} 

/*******************************************************************************
* Function Name  : TP_DrawPoint
* Description    : ��ָ�����껭��
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TouchPanel_DrawPoint( uint16_t Xpos, uint16_t Ypos )
{
  LCD_SetPoint( Xpos, Ypos, 0xf800 );     /* ���ĵ� */
  LCD_SetPoint( Xpos+1, Ypos, 0xf800 );
  LCD_SetPoint( Xpos, Ypos+1, 0xf800 );
  LCD_SetPoint( Xpos+1, Ypos+1, 0xf800 );
}	

/*******************************************************************************
* Function Name  : DrawCross
* Description    : ��ָ�����껭ʮ��׼��
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TouchPanel_DrawCross( uint16_t Xpos, uint16_t Ypos )
{
  LCD_DrawLine( Xpos-15, Ypos, Xpos-2, Ypos, 0xffff );
  LCD_DrawLine( Xpos+2, Ypos, Xpos+15, Ypos, 0xffff );
  LCD_DrawLine( Xpos, Ypos-15, Xpos, Ypos-2, 0xffff );
  LCD_DrawLine( Xpos, Ypos+2, Xpos, Ypos+15, 0xffff );
  
  LCD_DrawLine( Xpos-15, Ypos+15, Xpos-7, Ypos+15, RGB565CONVERT(184, 158, 131) );
  LCD_DrawLine( Xpos-15, Ypos+7, Xpos-15, Ypos+15, RGB565CONVERT(184, 158, 131) );

  LCD_DrawLine( Xpos-15, Ypos-15, Xpos-7, Ypos-15, RGB565CONVERT(184, 158, 131) );
  LCD_DrawLine( Xpos-15, Ypos-7, Xpos-15, Ypos-15, RGB565CONVERT(184, 158, 131) );

  LCD_DrawLine( Xpos+7, Ypos+15, Xpos+15, Ypos+15, RGB565CONVERT(184, 158, 131) );
  LCD_DrawLine( Xpos+15, Ypos+7, Xpos+15, Ypos+15, RGB565CONVERT(184, 158, 131) );

  LCD_DrawLine( Xpos+7, Ypos-15, Xpos+15, Ypos-15, RGB565CONVERT(184, 158, 131) );
  LCD_DrawLine( Xpos+15, Ypos-15, Xpos+15, Ypos-7, RGB565CONVERT(184, 158, 131) );
}	
	
/*******************************************************************************
* Function Name  : Read_Ads7846
* Description    : �õ��˲�֮���X Y
* Input          : None
* Output         : None
* Return         : Coordinate�ṹ���ַ
* Attention		 : None
*******************************************************************************/
Coordinate *Read_Ads7846( void )
{
  static Coordinate screen;
  int m0, m1, m2, TP_X[1], TP_Y[1], temp[3];
  uint8_t count = 0;
  int buffer[2][9] = {{0}, {0}};

  do {
    TP_GetAdXY( TP_X, TP_Y );
	buffer[0][count] = TP_X[0];
	buffer[1][count] = TP_Y[0];
	count++;  
  } while( (HAL_GPIO_ReadPin( TP_IRQ_GPIO_Port, TP_IRQ_Pin ) == GPIO_PIN_RESET) && (count < 9) );

  if(count == 9)
  {  
    temp[0] = (buffer[0][0] + buffer[0][1] + buffer[0][2]) / 3;
	temp[1] = (buffer[0][3] + buffer[0][4] + buffer[0][5]) / 3;
	temp[2] = (buffer[0][6] + buffer[0][7] + buffer[0][8]) / 3;

	m0 = temp[0] - temp[1];
	m1 = temp[1] - temp[2];
	m2 = temp[2] - temp[0];

	m0 = m0 > 0 ? m0 : (-m0);
    m1 = m1 > 0 ? m1 : (-m1);
	m2 = m2 > 0 ? m2 : (-m2);

	if((m0 > THRESHOLD) && (m1 > THRESHOLD) && (m2 > THRESHOLD))
	{
	  return 0;
	}

	if(m0 < m1)
	{
	  if(m2 < m0)
	  {
	    screen.x = (temp[0] + temp[2]) / 2;
	  } else {
	    screen.x = (temp[0] + temp[1]) / 2;
	  }
	} else if(m2 < m1) {
	  screen.x = (temp[0] + temp[2]) / 2;
	} else {
	  screen.x = (temp[1] + temp[2]) / 2;
	}

    temp[0] = (buffer[1][0] + buffer[1][1] + buffer[1][2]) / 3;
	temp[1] = (buffer[1][3] + buffer[1][4] + buffer[1][5]) / 3;
	temp[2] = (buffer[1][6] + buffer[1][7] + buffer[1][8]) / 3;
	m0 = temp[0] - temp[1];
	m1 = temp[1] - temp[2];
	m2 = temp[2] - temp[0];
	m0 = m0 > 0 ? m0 : (-m0);
	m1 = m1 > 0 ? m1 : (-m1);
	m2 = m2 > 0 ? m2 : (-m2);

	if((m0 > THRESHOLD) && (m1 > THRESHOLD) && (m2 > THRESHOLD))
    {
	  return 0;
    }

	if(m0 < m1)
	{
	  if(m2 < m0)
	  {
	    screen.y = (temp[0] + temp[2]) / 2;
	  } else {
	    screen.y = (temp[0] + temp[1]) / 2;
	  }
    } else if(m2 < m1) {
	   screen.y = (temp[0] + temp[2]) / 2;
    } else{
	   screen.y = (temp[1] + temp[2]) / 2;
    }

	return &screen;
  }  
  return 0; 
}
	 
/* �����Ǵ�������Һ��������任��ת������ */
/* ֻ����LCD�ʹ�����������Ƕȷǳ�Сʱ,�����������湫ʽ */


/*******************************************************************************
* Function Name  : setCalibrationMatrix
* Description    : ����� K A B C D E F
* Input          : None
* Output         : None
* Return         : ����1��ʾ�ɹ� 0ʧ��
* Attention		 : None
*******************************************************************************/
FunctionalState setCalibrationMatrix( Coordinate * displayPtr, Coordinate * screenPtr, Matrix * matrixPtr)
{

  FunctionalState retTHRESHOLD = ENABLE ;
  /* K��(X0��X2) (Y1��Y2)��(X1��X2) (Y0��Y2) */
  matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                       ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
  if( matrixPtr->Divider == 0 )
  {
    retTHRESHOLD = DISABLE;
  } else {
    /* A��((XD0��XD2) (Y1��Y2)��(XD1��XD2) (Y0��Y2))��K	*/
    matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
	/* B��((X0��X2) (XD1��XD2)��(XD0��XD2) (X1��X2))��K	*/
    matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                    ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;
    /* C��(Y0(X2XD1��X1XD2)+Y1(X0XD2��X2XD0)+Y2(X1XD0��X0XD1))��K */
    matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;
    /* D��((YD0��YD2) (Y1��Y2)��(YD1��YD2) (Y0��Y2))��K	*/
    matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
    /* E��((X0��X2) (YD1��YD2)��(YD0��YD2) (X1��X2))��K	*/
    matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                    ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;
    /* F��(Y0(X2YD1��X1YD2)+Y1(X0YD2��X2YD0)+Y2(X1YD0��X0YD1))��K */
    matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
  }
  return( retTHRESHOLD ) ;
}

/*******************************************************************************
* Function Name  : getDisplayPoint
* Description    : ͨ�� K A B C D E F ��ͨ��X Y��ֵת��ΪҺ��������
* Input          : None
* Output         : None
* Return         : ����1��ʾ�ɹ� 0ʧ��
* Attention		 : None
*******************************************************************************/
FunctionalState getDisplayPoint(Coordinate * displayPtr, Coordinate * screenPtr, Matrix * matrixPtr)
{
  FunctionalState retTHRESHOLD =ENABLE ;

  if(matrixPtr->Divider != 0)
  {
    /* XD = AX+BY+C */        
    displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                      (matrixPtr->Bn * screenPtr->y) + 
                       matrixPtr->Cn 
                    ) / matrixPtr->Divider ;
	/* YD = DX+EY+F */        
    displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                      (matrixPtr->En * screenPtr->y) + 
                       matrixPtr->Fn 
                    ) / matrixPtr->Divider ;
  } else {
    retTHRESHOLD = DISABLE;
  }

  return (retTHRESHOLD);
} 

/*******************************************************************************
* Function Name  : TouchPanel_Calibrate
* Description    : У׼������
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TouchPanel_Calibrate()
{
  uint8_t i;
  Coordinate *Ptr;

  for(i = 0; i < 3; i++)
  {
    LCD_Clear( Black );
    LCD_Text( 10, 10, "Touch crosshair to calibrate", 0xffff, Black );
    HAL_Delay( 500 );
    TouchPanel_DrawCross( DisplaySample[i].x, DisplaySample[i].y );

    do {
      Ptr = Read_Ads7846();
    } while( Ptr == (void*)0 );

    Message( Message_TouchPanel, Message_Debug, "%u, %u", Ptr->x, Ptr->y );

    ScreenSample[i].x = Ptr->x;
    ScreenSample[i].y = Ptr->y;
  }

  RTC_BKUPWrite( BKP_REG_SAMPLE_X1, ScreenSample[0].x );
  RTC_BKUPWrite( BKP_REG_SAMPLE_Y1, ScreenSample[0].y );
  RTC_BKUPWrite( BKP_REG_SAMPLE_X2, ScreenSample[1].x );
  RTC_BKUPWrite( BKP_REG_SAMPLE_Y2, ScreenSample[1].y );
  RTC_BKUPWrite( BKP_REG_SAMPLE_X3, ScreenSample[2].x );
  RTC_BKUPWrite( BKP_REG_SAMPLE_Y3, ScreenSample[2].y );

  Message( Message_TouchPanel, Message_Debug, "Touchpanel calibration updated: %u, %u, %u, %u, %u, %u",
                                              ScreenSample[0].x, ScreenSample[0].y,
                                              ScreenSample[1].x, ScreenSample[1].y,
                                              ScreenSample[2].x, ScreenSample[2].y );

  setCalibrationMatrix( &DisplaySample[0], &ScreenSample[0], &matrix );
  LCD_Clear( Black );
} 

void TouchPanel_RestoreCalibration()
{
  ScreenSample[0].x = RTC_BKUPRead( BKP_REG_SAMPLE_X1 );
  ScreenSample[0].y = RTC_BKUPRead( BKP_REG_SAMPLE_Y1 );
  ScreenSample[1].x = RTC_BKUPRead( BKP_REG_SAMPLE_X2 );
  ScreenSample[1].y = RTC_BKUPRead( BKP_REG_SAMPLE_Y2 );
  ScreenSample[2].x = RTC_BKUPRead( BKP_REG_SAMPLE_X3 );
  ScreenSample[2].y = RTC_BKUPRead( BKP_REG_SAMPLE_Y3 );

  Message( Message_TouchPanel, Message_Debug, "Touchpanel calibration restored: %u, %u, %u, %u, %u, %u",
                                              ScreenSample[0].x, ScreenSample[0].y,
                                              ScreenSample[1].x, ScreenSample[1].y,
                                              ScreenSample[2].x, ScreenSample[2].y );

  setCalibrationMatrix( &DisplaySample[0], &ScreenSample[0], &matrix );
}

void TouchPanel_GetPoint( Coordinate *display )
{
  getDisplayPoint( display, Read_Ads7846(), &matrix ) ;

  return;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
