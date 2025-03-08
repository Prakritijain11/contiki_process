/*                            =======================
================================ C/C++ SOURCE FILE =================================
                              =======================                          *//**
\file  rtcSensor.c
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

/*------------------------------------------------------------------------------------*/
/* INCLUDES */
/*------------------------------------------------------------------------------------*/
#include "board-i2c.h"
#include "rtcSensor.h"
#include "board-i2c.h"


/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/
#define RTC_I2C_ADDRESS                 0x69
#define RTC_MAXARRAY                    5 

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS AND STRUCTURES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* PROTOTYPES */
/*------------------------------------------------------------------------------------*/
void RTC_ReadReg( EN_RTC_REGISTERS en_reg );
void RTC_WriteReg( EN_RTC_REGISTERS en_reg, uint8_t *au8_data, uint8_t u8_len );
bool RTC_SetStandartExtRAMAddress( uint8_t u8_address );

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* LOCAL VARIABLES */
/*------------------------------------------------------------------------------------*/
uint8_t mau8_wData[RTC_MAXARRAY];
uint8_t mau8_rData[RTC_MAXARRAY];

/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

void RTC_StartI2C( void )
{
	board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
}

/*---------------------------------------------------------------------------*/
void RTC_ReadTime( void )
{
	RTC_ReadReg( RTCREG_Hundredths );

}

/*---------------------------------------------------------------------------*/
void RTC_ReadCountdownTimer( void )
{

	RTC_ReadReg( RTCREG_CntDownTimer );

}

/*---------------------------------------------------------------------------*/
uint8_t RTC_ReadRAM( uint8_t u8_address )
{

	uint8_t u8_reg;

	mau8_rData[0] = 0;


	u8_reg = u8_address + RTC_STANDARD_RAM_BASE_ADDR;

	RTC_ReadReg( u8_reg );

	return ( mau8_rData[0] );

}

void RTC_WriteRAM( uint8_t u8_address, uint8_t u8_value )
{

	uint8_t u8_reg;
	uint8_t au8_buf[RTC_MAXARRAY];

	u8_reg = u8_address + RTC_STANDARD_RAM_BASE_ADDR;
	au8_buf[0] = u8_value;
	RTC_WriteReg( u8_reg, au8_buf, 1 );

}

/*---------------------------------------------------------------------------*/
void RTC_Config( void )
{
	uint8_t au8_buf[RTC_MAXARRAY];

	// Stop the CountdownTimer(18x) = 0x00
	au8_buf[0] = 0x00;
	RTC_WriteReg( RTCREG_CntDownTimerCntl, au8_buf, 1 );

	// Clear Interrupt mask register(12h) = 0x00
	au8_buf[0] = 0x00;
	RTC_WriteReg( RTCREG_IntMask, au8_buf, 1 );

	// Set ARST bit in Control1 register(10h)  - 0x04
	// Read of Status will clear interrupt flags
	au8_buf[0] = 0x04;
	RTC_WriteReg( RTCREG_Control1, au8_buf, 1 );

	// Read Status register to clear flags
	RTC_ReadReg( RTCREG_Status );

	// Setup Countdown timer

	// Timer Initial value register(1Ah) = 0xff
	au8_buf[0] = 0xff;
	RTC_WriteReg( RTCREG_TimerValue, au8_buf, 1 );

	// Load timer register (19h) = 0xff for4 3 secondsRTCREG_CntDownTimerCntl
	au8_buf[0] = 0xff;
	RTC_WriteReg( RTCREG_CntDownTimer, au8_buf, 1 );

	// CountdownTimer control register(18x) = 0xc2
//	au8_buf[0] = 0xa2; // Pulse witdh 15.63mS
	au8_buf[0] = 0xe1; // Pulse witdh 0.2446mS count rate 64Hz
	RTC_WriteReg( RTCREG_CntDownTimerCntl, au8_buf, 1 );

	// Setup Interrupt mask register(12h) = 0x08
	au8_buf[0] = 0x08;
	RTC_WriteReg( RTCREG_IntMask, au8_buf, 1 );

}

/*------------------------------------------------------------------------------------*/
/* LOCAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
bool RTC_SetStandartExtRAMAddress( uint8_t u8_address )
{
	bool b_res;
	uint8_t au8_buf[RTC_MAXARRAY];

	// Verify address is valid
	if( u8_address <= RTC_STDRAM_END_ADDR_BLOCK4 )
	{
		b_res = true;
		au8_buf[0] = 0;

		// Setup the Extension RAM Address register with the correct upper bits for the address
		if( RTC_STDRAM_BEGIN_ADDR_BLOCK2 > u8_address )
		{
			// Address is in block 1
			au8_buf[0] |= RTC_RAM_BLOCK1;
		}
		else if( RTC_STDRAM_BEGIN_ADDR_BLOCK3 > u8_address )
		{
			// Address is in block 2
			au8_buf[0] |= RTC_RAM_BLOCK2;
		} 
		else if( RTC_STDRAM_BEGIN_ADDR_BLOCK4 > u8_address )
		{
			// Address is in block 3
			au8_buf[0] |= RTC_RAM_BLOCK3;
		} 
		else
		{
			// Address is in block 4
			au8_buf[0] |= RTC_RAM_BLOCK4;
		}
	}
	else
	{
		// Invalid address
		b_res = false;
	}

	if( b_res )
	{
		RTC_WriteReg( RTCREG_EXTENSIONRAMADDR, au8_buf, 1 );
	}

	return ( b_res );

}


/*---------------------------------------------------------------------------*/
void RTC_ReadReg( EN_RTC_REGISTERS en_reg )
{
	uint8_t u8_rDataLen = 1;

	mau8_wData[0] = en_reg;

	(void)board_i2c_write_read( mau8_wData, 1, mau8_rData, u8_rDataLen );

}

/*---------------------------------------------------------------------------*/
void RTC_WriteReg( EN_RTC_REGISTERS en_reg, uint8_t *au8_data, uint8_t u8_len )
{
	uint8_t u8_loop;

	mau8_wData[0] = en_reg;

	if( u8_len < RTC_MAXARRAY )
	{
		// Copy the specified data to the wData array
		for( u8_loop = 0; u8_loop < u8_len; u8_loop++ )
		{
			mau8_wData[u8_loop+1] = au8_data[u8_loop];
		}

		// Start the write
		(void)board_i2c_write(mau8_wData, u8_len+1);
	}



}


/*---------------------------------------------------------------------------*/
/**
 * @}
 */