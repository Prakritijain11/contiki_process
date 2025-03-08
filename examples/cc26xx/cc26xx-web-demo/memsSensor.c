/*                            =======================
================================ C/C++ SOURCE FILE =================================
=======================                          *//**
\file  memsSensor.c
description Handles the functionality of the MEMS sensor
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

#include "ti-lib.h"
#include "board-i2c.h"
#include "memsSensor.h"
#include "board-i2c.h"
#include "cbram.h"

/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/
#define MEMS_I2C_ADDRESS                 0x68
#define MEMS_MAXWRITEARRAY               3 
#define MEMS_MAXREADARRAY                10 
#define MEMS_STARTUP_DELAY				 0xffff  // ~14mS

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS AND STRUCTURES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* PROTOTYPES */
/*------------------------------------------------------------------------------------*/
void MEMS_ReadTemperature( );
void MEMS_ReadReg( uint8_t u8_reg, uint8_t u8_len );
void MEMS_ReadAccelOut( void );
void MEMS_ReadGyroOut( void );

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* LOCAL VARIABLES */
/*------------------------------------------------------------------------------------*/
uint8_t mau8_wData[MEMS_MAXWRITEARRAY];
uint8_t mau8_rData[MEMS_MAXREADARRAY];

/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

void MEMS_PowerDown(void)
{

	// Shutdown the I2C
	board_i2c_shutdown();

	/* Set Power pin low */
	ti_lib_gpio_clear_dio(BOARD_IOID_MEMS_PWR);
}

void MEMS_PowerUp(void)
{
	uint32_t u32_ticks;

	// Power up the CARMEN - Set up the Power pin 
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MEMS_PWR);
	ti_lib_gpio_set_dio(BOARD_IOID_MEMS_PWR);

	/* Set up I2C */
	board_i2c_select(BOARD_I2C_INTERFACE_1, MEMS_I2C_ADDRESS);

	u32_ticks = MEMS_STARTUP_DELAY;
	// Delay for power on
	while ( u32_ticks > 0 )
	{
		u32_ticks--;
	}
}

void MEMS_RequestMeasure( void )
{
	// Power up the MEMS
	MEMS_PowerUp();

	// Read the temperature
	MEMS_ReadTemperature( );

	// Read the accelerometer measurements
	MEMS_ReadAccelOut( );

	// Read Gyroscope measurements
	MEMS_ReadGyroOut( );

	MEMS_PowerDown();


}

/*------------------------------------------------------------------------------------*/
/* LOCAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

void MEMS_ReadTemperature( void )
{
	uint32_t u32_val;

	MEMS_ReadReg( MEMS_REG_TEMP, 2 );

	// Temperature was read, now convert the data
    u32_val = (uint32_t)(((uint32_t)mau8_rData[0] << 8) + ((uint32_t)mau8_rData[1]));

	CBRAM_WriteParamValue( CBRAM_MemsTemperature, u32_val );


}

/*---------------------------------------------------------------------------*/
void MEMS_ReadReg( uint8_t u8_reg, uint8_t u8_len )
{

	mau8_wData[0] = u8_reg;

	board_i2c_write_read( mau8_wData, 1, mau8_rData, u8_len );


}

/*---------------------------------------------------------------------------*/

void MEMS_ReadAccelOut( void )
{
	uint16_t u16_val;

	MEMS_ReadReg( MEMS_REG_ACCEL, 6 );

	// Coordinates were read, now convert the data
    u16_val = (uint16_t)(((uint16_t)mau8_rData[0] << 8) + ((uint16_t)mau8_rData[1]));
	CBRAM_WriteParamValue( CBRAM_MemsAccelAxisX, (uint32_t)u16_val );

    u16_val = (uint16_t)(((uint16_t)mau8_rData[2] << 8) + ((uint16_t)mau8_rData[3]));
	CBRAM_WriteParamValue( CBRAM_MemsAccelAxisY, (uint32_t)u16_val );

    u16_val = (uint16_t)(((uint16_t)mau8_rData[4] << 8) + ((uint16_t)mau8_rData[5]));
	CBRAM_WriteParamValue( CBRAM_MemsAccelAxisZ, (uint32_t)u16_val );
}

/*---------------------------------------------------------------------------*/

void MEMS_ReadGyroOut( void )
{
	uint16_t u16_val;

	MEMS_ReadReg( MEMS_REG_GYRO, 6 );

	// Coordinates were read, now convert the data
    u16_val = (uint16_t)(((uint16_t)mau8_rData[0] << 8) + ((uint16_t)mau8_rData[1]));
	CBRAM_WriteParamValue( CBRAM_MemsGyroAxisX, (uint32_t)u16_val );

    u16_val = (uint16_t)(((uint16_t)mau8_rData[2] << 8) + ((uint16_t)mau8_rData[3]));
	CBRAM_WriteParamValue( CBRAM_MemsGyroAxisY, (uint32_t)u16_val );

    u16_val = (uint16_t)(((uint16_t)mau8_rData[4] << 8) + ((uint16_t)mau8_rData[5]));
	CBRAM_WriteParamValue( CBRAM_MemsGyroAxisZ, (uint32_t)u16_val );
}