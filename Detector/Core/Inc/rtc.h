/*
 * rtc.h
 *
 *  Created on: Jan 05, 2024
 *      Author: Pawel
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include <stdbool.h>
#include <stdint.h>

/* Copied from STM HAL */

#if RTC_BKP_NUMBER > 0U
#define RTC_BKP_DR1                         0x00000001U
#define RTC_BKP_DR2                         0x00000002U
#define RTC_BKP_DR3                         0x00000003U
#define RTC_BKP_DR4                         0x00000004U
#define RTC_BKP_DR5                         0x00000005U
#define RTC_BKP_DR6                         0x00000006U
#define RTC_BKP_DR7                         0x00000007U
#define RTC_BKP_DR8                         0x00000008U
#define RTC_BKP_DR9                         0x00000009U
#define RTC_BKP_DR10                        0x0000000AU
#endif /* RTC_BKP_NUMBER > 0 */

#if RTC_BKP_NUMBER > 10U
#define RTC_BKP_DR11                        0x00000010U
#define RTC_BKP_DR12                        0x00000011U
#define RTC_BKP_DR13                        0x00000012U
#define RTC_BKP_DR14                        0x00000013U
#define RTC_BKP_DR15                        0x00000014U
#define RTC_BKP_DR16                        0x00000015U
#define RTC_BKP_DR17                        0x00000016U
#define RTC_BKP_DR18                        0x00000017U
#define RTC_BKP_DR19                        0x00000018U
#define RTC_BKP_DR20                        0x00000019U
#define RTC_BKP_DR21                        0x0000001AU
#define RTC_BKP_DR22                        0x0000001BU
#define RTC_BKP_DR23                        0x0000001CU
#define RTC_BKP_DR24                        0x0000001DU
#define RTC_BKP_DR25                        0x0000001EU
#define RTC_BKP_DR26                        0x0000001FU
#define RTC_BKP_DR27                        0x00000020U
#define RTC_BKP_DR28                        0x00000021U
#define RTC_BKP_DR29                        0x00000022U
#define RTC_BKP_DR30                        0x00000023U
#define RTC_BKP_DR31                        0x00000024U
#define RTC_BKP_DR32                        0x00000025U
#define RTC_BKP_DR33                        0x00000026U
#define RTC_BKP_DR34                        0x00000027U
#define RTC_BKP_DR35                        0x00000028U
#define RTC_BKP_DR36                        0x00000029U
#define RTC_BKP_DR37                        0x0000002AU
#define RTC_BKP_DR38                        0x0000002BU
#define RTC_BKP_DR39                        0x0000002CU
#define RTC_BKP_DR40                        0x0000002DU
#define RTC_BKP_DR41                        0x0000002EU
#define RTC_BKP_DR42                        0x0000002FU
#endif /* RTC_BKP_NUMBER > 10 */

#define RTC_TIMEOUT_VALUE                   1000U

#define RTC_WRITEPROTECTION_DISABLE(__HANDLE__)         SET_BIT((__HANDLE__)->CRL, RTC_CRL_CNF)
#define RTC_WRITEPROTECTION_ENABLE(__HANDLE__)          CLEAR_BIT((__HANDLE__)->CRL, RTC_CRL_CNF)

#define RTC_SECOND_ENABLE_IT(__HANDLE__, __INTERRUPT__)  SET_BIT((__HANDLE__)->CRH, (__INTERRUPT__))
#define RTC_SECOND_DISABLE_IT(__HANDLE__, __INTERRUPT__) CLEAR_BIT((__HANDLE__)->CRH, (__INTERRUPT__))

#define RTC_SECOND_CLEAR_FLAG(__HANDLE__, __FLAG__)      ((__HANDLE__)->CRL) &= ~(__FLAG__)


#define RTC_IT_OW            RTC_CRH_OWIE       /*!< Overflow interrupt */
#define RTC_IT_ALRA          RTC_CRH_ALRIE      /*!< Alarm interrupt */
#define RTC_IT_SEC           RTC_CRH_SECIE      /*!< Second interrupt */
#define RTC_IT_TAMP1         BKP_CSR_TPIE       /*!< TAMPER Pin interrupt enable */

#define RTC_FLAG_RTOFF       RTC_CRL_RTOFF      /*!< RTC Operation OFF flag */
#define RTC_FLAG_RSF         RTC_CRL_RSF        /*!< Registers Synchronized flag */
#define RTC_FLAG_OW          RTC_CRL_OWF        /*!< Overflow flag */
#define RTC_FLAG_ALRAF       RTC_CRL_ALRF       /*!< Alarm flag */
#define RTC_FLAG_SEC         RTC_CRL_SECF       /*!< Second flag */
#define RTC_FLAG_TAMP1F      BKP_CSR_TEF        /*!< Tamper Interrupt Flag */


void RTC_Init();
uint32_t RTC_GetCounter();
void RTC_Read();
void RTC_Write();
void RTC_BKUPWrite(uint32_t BackupRegister, uint32_t Data);
uint32_t RTC_BKUPRead(uint32_t BackupRegister);
bool RTC_IsLeapYear( uint16_t year );

uint16_t RTC_GetYear();
uint8_t RTC_GetMonth();
uint8_t RTC_GetDay();
uint8_t RTC_GetHour();
uint8_t RTC_GetMinute();
uint8_t RTC_GetSecond();

void RTC_IncYear();
void RTC_DecYear();
void RTC_SetYear(uint8_t year);

void RTC_IncMonth();
void RTC_DecMonth();
void RTC_SetMonth(uint8_t month);

void RTC_IncDay();
void RTC_DecDay();
void RTC_SetDay(uint8_t day);

void RTC_IncHour();
void RTC_DecHour();
void RTC_SetHour(uint8_t hour);

void RTC_IncMinute();
void RTC_DecMinute();
void RTC_SetMinute(uint8_t minute);

void RTC_IncSecond();
void RTC_DecSecond();
void RTC_SetSecond(uint8_t second);

#endif

//    isLeap = GUI_IsLeapYear( RTC_GetYear() );
//
//    if( (RTC_GetMonth() == 4) || (RTC_GetMonth() == 6) || (RTC_GetMonth() == 9) || (RTC_GetMonth() == 11) )
//    {
//      maxDate = 30;
//    } else if( !isLeap && (RTC_GetMonth() == 2) )
//    {
//      maxDate = 28;
//    } else if( isLeap && (RTC_GetMonth() == 2) )
//    {
//      maxDate = 29;
//    } else {
//      maxDate = 31;
//    }
//
//
//    switch( argument )
//    {
//      case 1: /* Hours ++ */
//        if( RTC_GetHour() < 23 )
//        {
//          RTC_GetHour()++;
//        } else {
//          RTC_GetHour() = 0;
//        }
//        break;
//
//      case 2: /* Hours -- */
//        if( RTC_GetHour() > 0 )
//        {
//          RTC_GetHour()--;
//        } else {
//          RTC_GetHour() = 23;
//        }
//        break;
//
//      case 3: /* Minutes ++ */
//        if( RTC_GetMinute() < 59 )
//        {
//          RTC_GetMinute()++;
//        } else {
//          RTC_GetMinute() = 0;
//        }
//        break;
//
//      case 4: /* Minutes ++ */
//        if( RTC_GetMinute() > 0 )
//        {
//          RTC_GetMinute()--;
//        } else {
//          RTC_GetMinute() = 59;
//        }
//        break;
//
//      case 5: /* Seconds ++ */
//        if( RTC_GetSecond() < 59 )
//        {
//          RTC_GetSecond()++;
//        } else {
//          RTC_GetSecond() = 0;
//        }
//        break;
//
//      case 6: /* Seconds -- */
//        if( RTC_GetSecond() > 0 )
//        {
//          RTC_GetSecond()--;
//        } else {
//          RTC_GetSecond() = 59;
//        }
//        break;
//
//      case 7:  /* Years ++ */
//        if( RTC_GetYear() < 99 )
//        {
//          RTC_GetYear()++;
//        } else {
//          RTC_GetYear() = 0;
//        }
//        break;
//
//      case 8:  /* Years -- */
//        if( RTC_GetYear() > 0 )
//        {
//          RTC_GetYear()--;
//        } else {
//          RTC_GetYear() = 99;
//        }
//        break;
//
//      case 9: /* Months ++ */
//        if( RTC_GetMonth() < 12 )
//        {
//          RTC_GetMonth()++;
//        } else {
//          RTC_GetMonth() = 1;
//        }
//        break;
//
//      case 10: /* Months -- */
//        if( RTC_GetMonth() > 1 )
//        {
//          RTC_GetMonth()--;
//        } else {
//          RTC_GetMonth() = 12;
//        }
//
//        break;
//
//      case 11: /* Date ++ */
//
//        if( RTC_GetDay() < maxDate )
//        {
//          RTC_GetDay()++;
//        } else {
//          RTC_GetDay() = 1;
//        }
//        break;
//
//      case 12: /* Date -- */
//        if( (RTC_GetMonth() == 4) || (RTC_GetMonth() == 6) || (RTC_GetMonth() == 9) || (RTC_GetMonth() == 11) )
//        {
//          maxDate = 30;
//        } else if( !GUI_IsLeapYear( RTC_GetYear() ) && (RTC_GetMonth() == 2) )
//        {
//          maxDate = 28;
//        } else if( GUI_IsLeapYear( RTC_GetYear() ) && (RTC_GetMonth() == 2) )
//        {
//          maxDate = 29;
//        } else {
//          maxDate = 31;
//        }
//
//        if( RTC_GetDay() > 1 )
//        {
//          RTC_GetDay()--;
//        } else {
//          RTC_GetDay() = maxDate;
//        }
//        break;
//
//      default:
//        break;
//    }
//
//    if( argument <= 6 )
//    {
////      HAL_RTC_WaitForSynchro( &hrtc );
////      HAL_RTC_SetTime( &hrtc, &sTime, RTC_FORMAT_BIN );
//
//      Message( Message_TouchPanel, Message_Debug, "Argument detected %u", argument );
//      argument = 0;
//
//    } else {
//      isLeap = GUI_IsLeapYear( RTC_GetYear() );
//
//      if( ((RTC_GetMonth() == 4) || (RTC_GetMonth() == 6) || (RTC_GetMonth() == 9) || (RTC_GetMonth() == 11)) && (RTC_GetDay() == 31) )
//      {
//        RTC_GetDay() = 30;
//      } else if( !isLeap && (RTC_GetMonth() == 2) && (RTC_GetDay() > 28) )
//      {
//        RTC_GetDay() = 28;
//      } else if( isLeap && (RTC_GetMonth() == 2) && (RTC_GetDay() > 29) )
//      {
//        RTC_GetDay() = 29;
//      }
//
////      HAL_RTC_WaitForSynchro( &hrtc );
////      HAL_RTC_SetDate( &hrtc, &sDate, RTC_FORMAT_BIN );
//
//      Message( Message_GUI, Message_Debug, "Argument detected %u", argument );
//      argument = 0;
//    }
//  }
