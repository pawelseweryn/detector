/*
 * rtc.c
 *
 *  Created on: Jan 5, 2025
 *      Author: Pawel
 */

#include <time.h>
#include <stdbool.h>

#include "rtc.h"
#include "main.h"
#include "stm32f103xe.h"
#include "stm32f1xx.h"

struct tm *timeDate;

#define __HAL_RTC_OVERFLOW_GET_FLAGA(__HANDLE__, __FLAG__)        (((((__HANDLE__)->CRL) & (__FLAG__)) != RESET)? SET : RESET)

/* Copied from STM HAL */
void RTC_Init()
{
  uint32_t tickstart = 0U;
  uint32_t prescaler = 0U;

  HAL_PWR_EnableBkUpAccess();
  /* Enable BKP CLK enable for backup registers */
  __HAL_RCC_BKP_CLK_ENABLE();
  /* Peripheral clock enable */
  __HAL_RCC_RTC_ENABLE();
  /* RTC interrupt Init */
  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);

  /* Clear RSF flag */
  CLEAR_BIT(RTC->CRL, RTC_FLAG_RSF);

  tickstart = HAL_GetTick();

  /* Wait the registers to be synchronised */
  while ( (RTC->CRL & RTC_FLAG_RSF) == (uint32_t)RESET )
  {
    if ( (HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE )
    {
      Error_Handler();
    }
  }

  tickstart = HAL_GetTick();

  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((RTC->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      Error_Handler();
    }
  }

  /* Disable the write protection for RTC registers */
  RTC_WRITEPROTECTION_DISABLE(RTC);

  prescaler = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_RTC);

  /* Check that RTC clock is enabled*/
  if (prescaler == 0U)
  {
    /* Should not happen. Frequency is not available*/
    Error_Handler();
  }
  else
  {
    /* RTC period = RTCCLK/(RTC_PR + 1) */
    prescaler = prescaler - 1U;
  }

  /* Configure the RTC_PRLH / RTC_PRLL */
  WRITE_REG(RTC->PRLH, ((prescaler >> 16U) & RTC_PRLH_PRL));
  WRITE_REG(RTC->PRLL, (prescaler & RTC_PRLL_PRL));

  /* Enable the write protection for RTC registers */
  RTC_WRITEPROTECTION_ENABLE(RTC);

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((RTC->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      Error_Handler();
    }
  }

  /* Enable Second interruption */
  RTC_SECOND_ENABLE_IT( RTC, RTC_IT_SEC );

  RTC_Read();
}

/* Copied from STM HAL */
void RTC_BKUPWrite(uint32_t BackupRegister, uint32_t Data)
{
  uint32_t tmp = 0U;

  tmp = (uint32_t)BKP_BASE;
  tmp += (BackupRegister * 4U);

  *(__IO uint32_t *) tmp = (Data & BKP_DR1_D);
}

/* Copied from STM HAL */
uint32_t RTC_BKUPRead(uint32_t BackupRegister)
{
  uint32_t backupregister = 0U;
  uint32_t pvalue = 0U;

  backupregister = (uint32_t)BKP_BASE;
  backupregister += (BackupRegister * 4U);

  /* Read the specified register */
  pvalue = (*(__IO uint32_t *)(backupregister)) & BKP_DR1_D;

  return pvalue;
}

uint32_t RTC_GetCounter()
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  /* Check if counter overflow occurred */
  if (__HAL_RTC_OVERFLOW_GET_FLAGA( RTC, RTC_CRL_OWF) )
  {
    /* TODO: Error handle */
    return 0;
  }

  high1 = READ_REG( RTC->CNTH & RTC_CNTH_RTC_CNT );
  low   = READ_REG( RTC->CNTL & RTC_CNTL_RTC_CNT );
  high2 = READ_REG( RTC->CNTH & RTC_CNTH_RTC_CNT );

  if (high1 != high2)
  {
    /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(RTC->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  {
    /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}

void RTC_Read()
{
  time_t timecounter = 0;

  timecounter = (time_t) RTC_GetCounter();

  timeDate = localtime( &timecounter );
}

void RTC_Write()
{
  uint32_t tickstart = 0U;
  uint8_t maxDays;
  bool isLeap;
  uint32_t counter;

  isLeap = RTC_IsLeapYear( timeDate->tm_year );

  if( (timeDate->tm_mon == 3) || (timeDate->tm_mon == 5) || (timeDate->tm_mon == 8) || (timeDate->tm_mon == 10) )
  {
    maxDays = 30;
  } else if( !isLeap && (timeDate->tm_mon == 1) )
  {
    maxDays = 28;
  } else if( isLeap && (timeDate->tm_mon == 1) )
  {
    maxDays = 29;
  } else {
    maxDays = 31;
  }

  if( timeDate->tm_mday > maxDays )
  {
    timeDate->tm_mday = maxDays;
  }

  counter = (uint32_t) mktime( timeDate );

  /* Set Initialization mode */
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((RTC->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      Error_Handler();
    }
  }

  /* Disable the write protection for RTC registers */
  RTC_WRITEPROTECTION_DISABLE(RTC);

  /* Set RTC COUNTER MSB word */
  WRITE_REG(RTC->CNTH, (counter >> 16U));
  /* Set RTC COUNTER LSB word */
  WRITE_REG(RTC->CNTL, (counter & RTC_CNTL_RTC_CNT));

  /* Wait for synchro */
  /* Enable the write protection for RTC registers */
  RTC_WRITEPROTECTION_ENABLE(RTC);

  tickstart = HAL_GetTick();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((RTC->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      Error_Handler();
    }
  }
}

bool RTC_IsLeapYear( uint16_t year )
{
  if( (year % 4U) != 0U )
  {
    return false;
  }
  else if( (year % 100U) != 0U )
  {
    return true;
  }
  else if( (year % 400U) == 0U )
  {
    return true;
  }
  else
  {
    return false;
  }
}

__attribute__((always_inline)) uint16_t RTC_GetYear()
{
  return (timeDate->tm_year + 1900);
}

__attribute__((always_inline)) uint8_t RTC_GetMonth()
{
  return (timeDate->tm_mon + 1);
}

__attribute__((always_inline)) uint8_t RTC_GetDay()
{
  return timeDate->tm_mday;
}

__attribute__((always_inline)) uint8_t RTC_GetHour()
{
  return timeDate->tm_hour;
}

__attribute__((always_inline)) uint8_t RTC_GetMinute()
{
  return timeDate->tm_min;
}

__attribute__((always_inline)) uint8_t RTC_GetSecond()
{
  return timeDate->tm_sec;
}

void RTC_IncYear()
{
  HAL_NVIC_DisableIRQ( RTC_IRQn );

  if( timeDate->tm_year < 200 )
  {
    timeDate->tm_year++;
  } else {
    timeDate->tm_year = 0;
  }

  RTC_Write();
  HAL_NVIC_EnableIRQ( RTC_IRQn );
}

void RTC_DecYear()
{
  HAL_NVIC_DisableIRQ( RTC_IRQn );

  if( timeDate->tm_year > 0 )
  {
    timeDate->tm_year--;
  } else {
    timeDate->tm_year = 200;
  }

  RTC_Write();
  HAL_NVIC_EnableIRQ( RTC_IRQn );
}

void RTC_SetYear(uint8_t year)
{
  HAL_NVIC_DisableIRQ( RTC_IRQn );

  if( (year >= 0 ) && (year <= 200) )
  {
    timeDate->tm_year = year;
  }

  RTC_Write();
  HAL_NVIC_EnableIRQ( RTC_IRQn );
}

void RTC_IncMonth()
{

}
void RTC_DecMonth()
{

}

void RTC_SetMonth(uint8_t month)
{

}

void RTC_IncDay()
{

}

void RTC_DecDay()
{

}

void RTC_SetDay(uint8_t day)
{

}


void RTC_IncHour()
{

}

void RTC_DecHour()
{

}

void RTC_SetHour(uint8_t hour)
{

}


void RTC_IncMinute()
{

}

void RTC_DecMinute()
{

}

void RTC_SetMinute(uint8_t minute)
{

}


void RTC_IncSecond()
{

}

void RTC_DecSecond()
{

}

void RTC_SetSecond(uint8_t second)
{

}


