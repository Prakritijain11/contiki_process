/*                            =======================
================================ C/C++ HEADER FILE =================================
                              =======================                          *//**
\file  rtcSensor.h
description Handles the functionality of the RTC sensor
\n\n
Copyright (c) Endress+Hauser AG \n
All rights reserved.
*//*==================================================================================*/
/*
CHANGE HISTORY:
---------------
Date       Author Description
14.11.2017 LRR    v0.0 Created
*/

/*---------------------------------------------------------------------------*/
#ifndef RTC_SENSOR_H_
#define RTC_SENSOR_H_

/*------------------------------------------------------------------------------------*/
/* INCLUDES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/
#define RTC_STANDARD_RAM_BASE_ADDR      0x40

#define RTC_RAM_BLOCK1      0
#define RTC_RAM_BLOCK2      1
#define RTC_RAM_BLOCK3      2
#define RTC_RAM_BLOCK4      3

#define RTC_STDRAM_BEGIN_ADDR_BLOCK1     0
#define RTC_STDRAM_END_ADDR_BLOCK1       63

#define RTC_STDRAM_BEGIN_ADDR_BLOCK2     64
#define RTC_STDRAM_END_ADDR_BLOCK2       127

#define RTC_STDRAM_BEGIN_ADDR_BLOCK3     128
#define RTC_STDRAM_END_ADDR_BLOCK3       191

#define RTC_STDRAM_BEGIN_ADDR_BLOCK4     192
#define RTC_STDRAM_END_ADDR_BLOCK4       255

#define RTCREG_EXTENSIONRAMADDR   0x3f

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS, CLASSES AND STRUCTURES */
/*------------------------------------------------------------------------------------*/
typedef enum
{
	RTCREG_Hundredths = 0,
	RTCREG_Seconds,
	RTCREG_Minutes,
	RTCREG_Hours,
	RTCREG_Date,
	RTCREG_Months,
	RTCREG_Years,
	RTCREG_Weekdays,
	RTCREG_HundredthsAlarm,
	RTCREG_SecondsAlarm,
	RTCREG_MinutesAlarm,
	RTCREG_HoursAlarm,
	RTCREG_DateAlarm,
	RTCREG_MonthsAlarm,
	RTCREG_WeekdaysAlarm,
	RTCREG_Status,
	RTCREG_Control1,
	RTCREG_Control2,
	RTCREG_IntMask,
	RTCREG_SqWave,
	RTCREG_CalXT,
	RTCREG_CalRCUpper,
	RTCREG_CalRCLower,
	RTCREG_SleepCntl,
	RTCREG_CntDownTimerCntl,
	RTCREG_CntDownTimer,
	RTCREG_TimerValue,
	RTCREG_WatchdogTimer,
	RTCREG_OscillatorCntl,
	RTCREG_OscillatorStatus,
	RTCREG_Reserved1,
	RTCREG_ConfigKey,
	RTCREG_TrickleCharge,
	RTCREG_BREFCntl,
	RTCREG_Reserved2,
	RTCREG_Reserved3,
	RTCREG_Reserved4,
	RTCREG_Reserved5,
	RTCREG_CapCntl,
	RTCREG_IOBatmode,
	RTCREG_ID0,
	RTCREG_ID1,
	RTCREG_ID2,
	RTCREG_ID3,
	RTCREG_ID4,
	RTCREG_ID5,
	RTCREG_ID6,
	RTCREG_AnalogStatus,
	RTCREG_OutputCntl,
	RTCREG_ExtensionRAMAddr,
	RTCREG_MaxRegs
}EN_RTC_REGISTERS;

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/
void RTC_StartI2C( void );
void RTC_ReadTime( void );
void RTC_Config( void );
void RTC_ReadCountdownTimer( void );
uint8_t RTC_ReadRAM( uint8_t u8_address );
void RTC_WriteRAM( uint8_t u8_address, uint8_t u8_value );

/*---------------------------------------------------------------------------*/
#endif /* RTC_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
