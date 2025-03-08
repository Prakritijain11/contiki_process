/*                            =======================
================================ C/C++ SOURCE FILE =================================
                              =======================                          *//**
\file  temp108Sensor.c
description Handles the functionality of the Temp sensor
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
#include "temp108Sensor.h"
#include "board-i2c.h"
#include "cbram.h"

/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/
#define TEMP108_I2C_ADDRESS                 0x49
#define TEMP108_MAXARRAY                    5 
#define TEMP108_NEG_TEMP_MASK               0x800
#define TEMP108_DEG_PER_COUNT               (float)0.0625

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS AND STRUCTURES */
/*------------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------------*/
/* PROTOTYPES */
/*------------------------------------------------------------------------------------*/
void TEMP108_ReadReg( EN_TEMP108_REGISTERS en_reg );
void TEMP108_WriteReg( EN_TEMP108_REGISTERS en_reg, uint8_t *au8_data, uint8_t u8_len );
float TEMP108_ConvertTemp( void );

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* LOCAL VARIABLES */
/*------------------------------------------------------------------------------------*/
uint8_t mau8_wData[TEMP108_MAXARRAY];
uint8_t mau8_rData[TEMP108_MAXARRAY];

/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

void TEMP108_ReadTemp( void )
{
	float f32_temp;
	uint32_t u32_val;

	/* Set up I2C */
	board_i2c_select(BOARD_I2C_INTERFACE_0, TEMP108_I2C_ADDRESS);

	// Read the temperature
	TEMP108_ReadReg( TEMP108_Temp );

	// Shutdown the I2C
	board_i2c_shutdown();

	// Convert temperature
	f32_temp = TEMP108_ConvertTemp(  );

	u32_val = (int)(f32_temp * 100);
	CBRAM_WriteParamValue( CBRAM_DeviceTemperature, u32_val );

}

/*------------------------------------------------------------------------------------*/
/* LOCAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void TEMP108_ReadReg( EN_TEMP108_REGISTERS en_reg )
{
	uint8_t u8_rDataLen = 2;


	// Make sure this is a valid register number 
	if( TEMP108_MaxRegs > en_reg ) 
	{

		mau8_wData[0] = en_reg;

		board_i2c_write_read( mau8_wData, 1, mau8_rData, u8_rDataLen );

	}

}

/*---------------------------------------------------------------------------*/
void TEMP108_WriteReg( EN_TEMP108_REGISTERS en_reg, uint8_t *au8_data, uint8_t u8_len )
{
	uint8_t u8_loop;

	// Make sure this is a valid register number 
	if( TEMP108_MaxRegs > en_reg ) 
	{

		mau8_wData[0] = en_reg;

		if( u8_len < TEMP108_MAXARRAY )
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

}

float TEMP108_ConvertTemp( void )
{
	uint16_t u16_temperature;
	float   f32_temperature;

	// The temperature is 12-bit register
	u16_temperature = ((uint16_t)mau8_rData[0] << 8 ) + ( (uint16_t)mau8_rData[1] & 0xf0  );

	// Check for negative temperature
	if( u16_temperature < TEMP108_NEG_TEMP_MASK )
	{
		// Positive temperature
		f32_temperature = (float)u16_temperature * TEMP108_DEG_PER_COUNT;
	}
	else
	{
		// Negative temperature
		f32_temperature = (float)(0x1000 - u16_temperature) * TEMP108_DEG_PER_COUNT;
	}

	return ( f32_temperature );
}


/*---------------------------------------------------------------------------*/
