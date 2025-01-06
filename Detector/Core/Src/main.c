/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "sensors.h"
#include "GLCD.h"
#include "TouchPanel.h"
#include "gui.h"
#include "messages.h"
#include "rtc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ITM register addresses */
#define ITM_STIMULUS_PORT0    *((volatile uint32_t*) 0xE0000000u)
#define ITM_TRACE_EN          *((volatile uint32_t*) 0xE0000E00u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
extern Matrix matrix ;
uint16_t period, scroff;
bool expired, turnoff;
uint32_t meas_count;
uint8_t idleTime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Override low-level _write system call */
int _write(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
      ITM_SendChar(*ptr++);
  }
  return len;
}

void CheckDefaults()
{
  if( RTC_BKUPRead( BKP_REG_INIT ) != BKP_REG_INITVALUE )
  {
    Message(Message_Main, Message_Information, "No defaults found in backup registers. Restoring defaults");

    ApplyDefaults();
  }

  return;
}

void ApplyDefaults()
{
  RTC_BKUPWrite( BKP_REG_DAY, 0x04 );
  RTC_BKUPWrite( BKP_REG_MONTH, 0x01 );
  RTC_BKUPWrite( BKP_REG_YEAR, 0x25 );

  RTC_BKUPWrite( BKP_REG_SAMPLE_X1, 635 );
  RTC_BKUPWrite( BKP_REG_SAMPLE_Y1, 933 );
  RTC_BKUPWrite( BKP_REG_SAMPLE_X2, 3132 );
  RTC_BKUPWrite( BKP_REG_SAMPLE_Y2, 852 );
  RTC_BKUPWrite( BKP_REG_SAMPLE_X3, 2356 );
  RTC_BKUPWrite( BKP_REG_SAMPLE_Y3, 3261 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM1_0 * 2), 10 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM1_0 * 2) + 1, 35 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM2_5 * 2), 15 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM2_5 * 2) + 1, 55 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM4_0 * 2), 15 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM4_0 * 2) + 1, 55 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM10 * 2), 45 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_PM10 * 2) + 1, 100 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_CO2 * 2), 1200 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_CO2 * 2) + 1, 2000 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_VOC * 2), 150 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_VOC * 2) + 1, 250 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_NOC * 2), 20 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_NOC * 2) + 1, 150 );

  /* For temperature and humidity TEMP1/RH1 is for lower than, TEMP2/RH2 is for higher than */
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP1 * 2) + 1, 10 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP1 * 2), 15 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP2 * 2), 27 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_TEMP2 * 2) + 1, 30 );

  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH1 * 2) + 1, 30 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH1 * 2), 40 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH2 * 2), 60 );
  RTC_BKUPWrite( BKP_REG_SENSOR_ST + (SENSORS_TYPE_RH2 * 2) + 1, 70 );

  RTC_BKUPWrite( BKP_REG_SCROFF, 30 );
  RTC_BKUPWrite( BKP_REG_PERIOD, 500 );

  RTC_BKUPWrite( BKP_REG_TEMP1OFST, (int16_t) ((double) -7.2 * 10) );
  RTC_BKUPWrite( BKP_REG_TEMP1SLOPE, (uint16_t) ((double) 0) );
  RTC_BKUPWrite( BKP_REG_TEMP1TIME, 0 );
  RTC_BKUPWrite( BKP_REG_TEMP1ACCEL, 1 );
  RTC_BKUPWrite( BKP_REG_TEMP2OFST, (uint16_t) ((double) 7.2 * 10) );

  RTC_BKUPWrite( BKP_REG_INIT, BKP_REG_INITVALUE );

  return;
}

void RestoreSettings()
{
  period = RTC_BKUPRead( BKP_REG_PERIOD );
  scroff = RTC_BKUPRead( BKP_REG_SCROFF );
}

inline uint16_t GetPeriod()
{
  return period;
}

void SetPeriod( uint16_t period_local )
{
  RTC_BKUPWrite( BKP_REG_PERIOD, period_local );
  period = period_local;
}

inline uint16_t GetScrOff()
{
  return scroff;
}

void SetScrOff( uint16_t scroff_local )
{
  RTC_BKUPWrite( BKP_REG_SCROFF, scroff_local );
  scroff = scroff_local;
}

void WriteMeasurements()
{
  if( meas_count < FILE_MAX_MEAS )
  {
    meas_count++;
  } else {
    Main_NewMeasurementFile();
  }

  FatFS_WriteToFile( FILE_RESULT, true, "%.1f;%.1f;%.1f;%.1f;%.0f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f\n",
                                        Sensors_GetValue( SENSORS_TYPE_PM1_0, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_PM2_5, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_PM4_0, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_PM10, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_CO2, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_VOC, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_NOC, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_TEMP1, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_RH1, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_TEMP2, SENSORS_LEVEL_ACTUAL ),
                                        Sensors_GetValue( SENSORS_TYPE_RH2, SENSORS_LEVEL_ACTUAL ));
}

void Main_ReplaceDotToComma( char *buffer )
{
  while( *buffer )
  {
    if((*buffer) == '.')
    {
      (*buffer) = ',';
    }
    buffer++;
  }
}

void SetExpired()
{
  expired = true;
}

void Main_TurnOff()
{
  turnoff = true;
}

void Main_NewMeasurementFile()
{
  meas_count = 0;
  FatFS_NewFile( FILE_RESULT );
  FatFS_WriteToFile( FILE_RESULT, false, "PM1.0;PM2.5;PM4.0;PM10;CO2;VOC;NOC;TEMP1;RH1;TEMP2;RH2\n" );
}

void Main_TurnOnScreen()
{
  idleTime = 0;
  LCD_Backlight( true );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t discard = 6;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  RTC_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_FSMC_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */


  CheckDefaults();
  RestoreSettings();

  /* Enable SWV trace */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM_TRACE_EN |= (1 << 0);
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* LCD Init */
  LCD_Initializtion();
  LCD_Backlight( true );

  uint16_t temp;
  temp = SCD41_TIME_PERFORMTEST / 1000;
  GUI_Init( (uint8_t) temp );

  expired = false;
  turnoff = false;
  meas_count = 0;
  idleTime = 0;

  /* Init Sensors */
  Sensors_SEN55_Stop();
  Sensors_SCD41_Stop();

  Sensors_SEN55_Init();
  Sensors_SCD41_Init();

  Sensors_SEN55_Start();
  Sensors_SCD41_Start();

  Sensors_RecallValues();

  /* Restore Touchpanel calibration */
  TouchPanel_RestoreCalibration();
  Main_NewMeasurementFile();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    GUI_HandleButton();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( turnoff )
    {
      __disable_irq();
      LCD_Clear( Green );
      while(1);
    } else if( expired )
    {
      Sensors_SEN55_Read();
      Sensors_SCD41_Read();

      if(discard)
      {
        discard--;
      } else {
        WriteMeasurements();
      }

      if(idleTime < scroff)
      {
        idleTime++;
        GUI_Handle();
      } else {
        LCD_Backlight( false );
      }

      expired = false;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 22;
  /* USER CODE BEGIN SDIO_Init 2 */

  // Comment //  (void)SDIO_Init(hsd->Instance, hsd->Init); in stm32f1xx_hal_sd.c

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|BL_CNT_Pin|USB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_KEYA_Pin */
  GPIO_InitStruct.Pin = USER_KEYA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_KEYA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_CS_Pin */
  GPIO_InitStruct.Pin = TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin BL_CNT_Pin USB_EN_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|BL_CNT_Pin|USB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_KEYB_Pin */
  GPIO_InitStruct.Pin = USER_KEYB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_KEYB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 10;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 10;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  LCD_Clear( Red );
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
