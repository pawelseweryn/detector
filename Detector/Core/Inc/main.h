/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BKP_REG_INITVALUE   0xBB

#define BKP_REG_INIT        RTC_BKP_DR1
#define BKP_REG_DAY         RTC_BKP_DR2
#define BKP_REG_MONTH       RTC_BKP_DR3
#define BKP_REG_YEAR        RTC_BKP_DR4
#define BKP_REG_SAMPLE_X1   RTC_BKP_DR6
#define BKP_REG_SAMPLE_Y1   RTC_BKP_DR7
#define BKP_REG_SAMPLE_X2   RTC_BKP_DR8
#define BKP_REG_SAMPLE_Y2   RTC_BKP_DR9
#define BKP_REG_SAMPLE_X3   RTC_BKP_DR10
#define BKP_REG_SAMPLE_Y3   RTC_BKP_DR11
/* RTC_BKP_DR12 - RTC_BKP_DR33 reserved for sensors */
#define BKP_REG_SENSOR_ST   RTC_BKP_DR12
//#define BKP_REG_PM1_0_W     RTC_BKP_DR12
//#define BKP_REG_PM1_0_C     RTC_BKP_DR13
//#define BKP_REG_PM2_5_W     RTC_BKP_DR14
//#define BKP_REG_PM2_5_C     RTC_BKP_DR15
//#define BKP_REG_PM4_0_W     RTC_BKP_DR16
//#define BKP_REG_PM4_0_C     RTC_BKP_DR17
//#define BKP_REG_PM10_W      RTC_BKP_DR18
//#define BKP_REG_PM10_C      RTC_BKP_DR19
//#define BKP_REG_CO2_W       RTC_BKP_DR20
//#define BKP_REG_CO2_C       RTC_BKP_DR21
//#define BKP_REG_VOC_W       RTC_BKP_DR22
//#define BKP_REG_VOC_C       RTC_BKP_DR23
//#define BKP_REG_NOC_W       RTC_BKP_DR24
//#define BKP_REG_NOC_C       RTC_BKP_DR25
//#define BKP_REG_TEMP1_C     RTC_BKP_DR26
//#define BKP_REG_TEMP1_W     RTC_BKP_DR27
//#define BKP_REG_TEMP2_W     RTC_BKP_DR28
//#define BKP_REG_TEMP2_C     RTC_BKP_DR29
//#define BKP_REG_RH1_C       RTC_BKP_DR30
//#define BKP_REG_RH1_W       RTC_BKP_DR31
//#define BKP_REG_RH2_W       RTC_BKP_DR32
//#define BKP_REG_RH2_C       RTC_BKP_DR33
#define BKP_REG_SCROFF      RTC_BKP_DR34
#define BKP_REG_PERIOD      RTC_BKP_DR35
#define BKP_REG_TEMP1OFST   RTC_BKP_DR36
#define BKP_REG_TEMP1SLOPE  RTC_BKP_DR37
#define BKP_REG_TEMP1TIME   RTC_BKP_DR38
#define BKP_REG_TEMP1ACCEL  RTC_BKP_DR39
#define BKP_REG_TEMP2OFST   RTC_BKP_DR40

#define PERIOD_MIN          200
#define PERIOD_MAX          5000
#define PERIOD_STEP         100

#define SCROFF_MIN          5000
#define SCROFF_MAX          65000
#define SCROFF_STEP         1000

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_KEYA_Pin GPIO_PIN_13
#define USER_KEYA_GPIO_Port GPIOC
#define TP_CS_Pin GPIO_PIN_4
#define TP_CS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define USER_KEYB_Pin GPIO_PIN_2
#define USER_KEYB_GPIO_Port GPIOB
#define SD_CD_Pin GPIO_PIN_3
#define SD_CD_GPIO_Port GPIOD
#define BL_CNT_Pin GPIO_PIN_5
#define BL_CNT_GPIO_Port GPIOB
#define TP_IRQ_Pin GPIO_PIN_6
#define TP_IRQ_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_7
#define USB_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

void CheckDefaults();
void ApplyDefaults();
void RestoreSettings();

uint16_t GetPeriod();
void SetPeriod( uint16_t period_local );
uint16_t GetScrOff();
void SetScrOff( uint16_t scroff_local );

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
