/*                            =======================
================================ C/C++ SOURCE FILE =================================
=======================                          *//**
\file  cbram.c
description Handles the functionality of the CBRAM sensor
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
#include "contiki.h"
#include "ti-lib.h"
#include <stdio.h> /* For printf() */
#include "board-spi.h"
#include "cbram.h"
#include <string.h>
#include "project-conf.h"


/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/

// Instruction registers
#define CBRAM_WRITE_STATUS_REGISTER       0x01
#define CBRAM_WRITE_BYTES                 0x02
#define CBRAM_READ_DATA_ARRAY             0x03
#define CBRAM_WRITE_DISABLE               0x04
#define CBRAM_READ_STATUS_REGISTER        0x05
#define CBRAM_WRITE_ENABLE                0x06
#define CBRAM_UDEEP_POWER_DOWN            0x79

#define CBRAM_FIRST_DATA_BYTE       (uint8_t)3

#define CBRAM_BUFFER_MAX			40

// Length defines
#define CBRAM_UINT8                 (uint8_t)1 
#define CBRAM_UINT16                (uint8_t)2
#define CBRAM_UINT32                (uint8_t)4

#define ASCII_ZERO					(uint8_t)48

#define CBRAM_STARTUP_DELAY				 0xffff  // ~14mS

//#define DBG(...) printf(__VA_ARGS__)

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS AND STRUCTURES */
/*------------------------------------------------------------------------------------*/


typedef struct
{
	uint8_t           en_param;
	uint16_t          u16_addr;
	uint8_t           u8_len;
	char              au8_unit[CBRAM_MAX_UNIT_SIZE];
	char              au8_descr[CBRAM_MAX_DESCR_SIZE];

} ST_PARAM_INFO;

typedef struct 
{

    uint8_t ReceiveBuffer[ CBRAM_BUFFER_MAX ];

    uint8_t ReceiveIndex;

    uint8_t TransmitBuffer[ CBRAM_BUFFER_MAX ];

    uint8_t TransmitLength;

}ST_CBRAM_SESSION;

static ST_CBRAM_SESSION mst_cbram;

static const ST_PARAM_INFO mast_paramInfo[CBRAM_MaxDataParameters] =
{
	//     Parameter                Address        Length               Unit        Desc

	{  CBRAM_DeviceTemperature,   	0x0000,		CBRAM_UINT32,      RND_UNIT_TEMP,   "1" },
	{  CBRAM_ProcessPressure,	 	0x0004,		CBRAM_UINT32,      RND_UNIT_MBAR,   "2" },
	{  CBRAM_ProcessTemperature,	0x0008,		CBRAM_UINT32,      RND_UNIT_TEMP,   "3" },
	{  CBRAM_ProcessPressStatus, 	0x000C,		CBRAM_UINT8,       RND_UNIT_NONE,   "4" },
	{  CBRAM_ProcessTempStatus,		0x000D,		CBRAM_UINT8,       RND_UNIT_NONE,   "5" },
	{  CBRAM_MemsTemperature,       0x000E,		CBRAM_UINT16,      RND_UNIT_TEMP,   "6" },
	{  CBRAM_MemsAccelAxisX,     	0x0010,		CBRAM_UINT16,	   RND_UNIT_ACC,    "7" },
	{  CBRAM_MemsAccelAxisY,     	0x0012,		CBRAM_UINT16,      RND_UNIT_ACC,    "8" },	
	{  CBRAM_MemsAccelAxisZ,     	0x0014,		CBRAM_UINT16,      RND_UNIT_ACC,    "9" },	
	{  CBRAM_MemsGyroAxisX,	   		0x0016,		CBRAM_UINT16,      RND_UNIT_GYRO,   "10" },	
	{  CBRAM_MemsGyroAxisY,	   		0x0018,		CBRAM_UINT16,      RND_UNIT_GYRO,   "11" },	
	{  CBRAM_MemsGyroAxisZ,	   		0x001A,		CBRAM_UINT16,      RND_UNIT_GYRO,   "12" },	
};

static const ST_PARAM_INFO mast_paramConfig[CBRAM_MaxConfigParameters] =
{
	//     Parameter                Address        Length               Unit        Desc
	{  CBRAM_UpdateRateSec,	   		0x0200,		CBRAM_UINT16,      RND_UNIT_NONE,   "0" },	
	{  CBRAM_TestParam1,	   		0x0202,		CBRAM_UINT16,      RND_UNIT_NONE,   "1" },	

};

/*------------------------------------------------------------------------------------*/
/* PROTOTYPES */
/*------------------------------------------------------------------------------------*/
bool CBRAM_WriteEnable( void );
bool CBRAM_SPI_Read_Write( );
bool CBRAM_SPI_Write( );
bool CBRAM_Send( void );
bool CBRAM_ReadRegister( uint8_t u8_reg );
bool CBRAM_ReadData( uint16_t u16_addr, uint8_t u8_len );
bool CBRAM_WriteData( uint16_t u16_addr, uint8_t *au8_buf, uint8_t u8_len );
void CBRAM_EnableCS( void );
void CBRAM_DisableCS( void );
void CBRAM_EnableSPI( void );
void CBRAM_DecodeMemParams( void );
uint32_t CBRAM_GetReceivedBytes( uint8_t u8_numBytes );
void CBRAM_PutBytesInBuf( uint32_t u32_val, uint8_t *pu8_buf, uint8_t *pu8_bufIndex, uint8_t u8_paramLen );
void ProcessToken( char * pc_dataToken );

void CBRAM_StoreConfiguration( uint16_t u16_param, uint32_t u32_val );

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* LOCAL VARIABLES */
/*------------------------------------------------------------------------------------*/
uint16_t mu16_startAddress;
uint16_t m16_memMapSize;
uint32_t mau32_paramValues[CBRAM_MaxDataParameters];
uint32_t mau32_writeParamValues[CBRAM_MaxDataParameters];
uint32_t mau32_writeConfigValues[CBRAM_MaxConfigParameters];
ST_RAM_VALUES mast_paramValues[CBRAM_MaxDataParameters];

uint8_t mu8_paramInfoIndex;

/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

void CBRAM_PowerDown(void)
{
	// Pull the Chip Select low
	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_CBRAM_CS);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_CBRAM_CS, IOC_IOPULL_DOWN);

	// Shut down the SPI
	board_spi_close(BOARD_SPI0);

	/* Set Power pin low */
	ti_lib_gpio_clear_dio(BOARD_IOID_CBRAM_PWR);
}

void CBRAM_PowerUp(void)
{
	uint32_t u32_ticks;

	// Power up the CBRAM - Set up the Power pin 
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_CBRAM_PWR);
	ti_lib_gpio_set_dio(BOARD_IOID_CBRAM_PWR);

	// Set up the Chip Select pin for - Output high to disable
//	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_CBRAM_CS);
	ti_lib_gpio_set_dio(BOARD_IOID_CBRAM_CS);

	// Init/Open SPI for RAM chip
	CBRAM_EnableSPI( );

	// Delay - give the peripheral time to power up
	u32_ticks = CBRAM_STARTUP_DELAY;
	while ( u32_ticks > 0 )
	{
		u32_ticks--;
	}

}

/*---------------------------------------------------------------------------*/
void CBRAM_Init( void )
{
	uint8_t u8_loop;

	// Calculate the size in bytes that are used in the memory map
	m16_memMapSize = 0;

	for( u8_loop = 0; u8_loop < CBRAM_MaxDataParameters; u8_loop++ )
	{
		m16_memMapSize += mast_paramInfo[u8_loop].u8_len;
		mau32_paramValues[u8_loop] = (uint32_t)0;
	}

	mu16_startAddress  = mast_paramInfo[0].u16_addr;
}

void CBRAM_WriteParamValue( uint8_t u8_param, uint32_t u32_val )
{

	if ( CBRAM_MaxDataParameters >  (EN_CBRAM_MAP_DATA_PARAMETERS)u8_param )
	{
		mau32_writeParamValues[(EN_CBRAM_MAP_DATA_PARAMETERS)u8_param] = u32_val;
	}

}

void CBRAM_WriteConfigValue( char *pu8_data, uint16_t u16_dataLen )
{
	const char comma[2]=",";
	char *token;
	char *pc_dataToken[CBRAM_MaxConfigParameters];
	uint8_t u8_index;
	uint8_t u8_loop;


	u8_index = 0;

	/* get the first token */
	token = strtok(pu8_data, comma);

	/* walk through other tokens */
	while( token != NULL ) {
		if ( CBRAM_MaxConfigParameters > u8_index )
		{
			pc_dataToken[u8_index] = token;
			u8_index++;
		}
		token = strtok(NULL, comma);
	}

	// Init/Open SPI for RAM chip
	CBRAM_PowerUp();

	// Process all of the tokens
	for( u8_loop = 0; u8_loop < u8_index; u8_loop++ ) {
		ProcessToken( pc_dataToken[u8_loop] );
	}

	// Close SPI for RAM chip
//	board_spi_close();
	CBRAM_PowerDown();

}

/*---------------------------------------------------------------------------*/
void CBRAM_ReadMemParams( void )
{

#if RND_SENSORS_CONNECTED
	// Init/Open SPI for RAM chip
	CBRAM_PowerUp();

	CBRAM_ReadData(mu16_startAddress, m16_memMapSize);

	// Shut down the SPI
	CBRAM_PowerDown();

	// Decode the parameters
	CBRAM_DecodeMemParams();

	// reset parameter index to start from beginning
	//	mu8_paramInfoIndex = (uint8_t)0;
#else

	mau32_paramValues[0] = (uint32_t)2500;    // 1 0x000009c4 TEMP
	mau32_paramValues[1] = (uint32_t)19000;   // 2 0x00004A38 PRESS
	mau32_paramValues[2] = (uint32_t)2610;    // 3 0x00000A32 PRESS
	mau32_paramValues[3] = (uint32_t)1530;    // 4 0x000005FA TEMP
	mau32_paramValues[4] = (uint32_t)0;       // 5 0x00       PRESS
	mau32_paramValues[5] = (uint32_t)0;       // 6 0x00       TEMP
	mau32_paramValues[6] = (uint32_t)9000;    // 7 0x00002328 TEMP
	mau32_paramValues[7] = (uint32_t)100;     // 8 0x0064     MOTION
	mau32_paramValues[8] = (uint32_t)77;      // 9 0x004D     MOTION
	mau32_paramValues[9] = (uint32_t)80;      // A 0x0050     MOTION
	mau32_paramValues[10] = (uint32_t)200;    // B 0x00C8     MOTION
	mau32_paramValues[11] = (uint32_t)300;    // C 0x012C     MOTION
	mau32_paramValues[12] = (uint32_t)10;     // D 0x000A     MOTION
#endif


}

/*---------------------------------------------------------------------------*/
void CBRAM_StoreRAMValues( void )
{
	uint8_t au8_buf[CBRAM_BUFFER_MAX];
	uint8_t u8_bufIndex = 0;
	uint8_t u8_loop;
	uint32_t u32_val;
	//	uint8_t u8_numParams;
	uint8_t u8_paramLen = 0; 


	for(u8_loop = 0; u8_loop < CBRAM_MaxDataParameters; u8_loop++)
	{
		u32_val = mau32_writeParamValues[u8_loop];
		u8_paramLen = mast_paramInfo[u8_loop].u8_len;
		CBRAM_PutBytesInBuf( u32_val, au8_buf, &u8_bufIndex, u8_paramLen  );
	}

	// make sure the number of bytes to be sent is not greater than the transmit buffer
	if( CBRAM_BUFFER_MAX >= u8_bufIndex + 3)
	{
		// Init/Open SPI for RAM chip
		CBRAM_PowerUp();

		CBRAM_WriteData( mast_paramInfo[0].u16_addr, au8_buf, u8_bufIndex );

		// Shut down the SPI
		CBRAM_PowerDown();

	}


}

bool CBRAM_GetParameterValues( ST_RAM_VALUES * st_paramInfo )
{
	bool   b_res = false;

	if( CBRAM_MaxDataParameters > mu8_paramInfoIndex )
	{
		snprintf(st_paramInfo->au8_descr , CBRAM_MAX_DESCR_SIZE, "%s", mast_paramInfo[mu8_paramInfoIndex].au8_descr);
		snprintf(st_paramInfo->au8_unit , CBRAM_MAX_UNIT_SIZE, "%s", mast_paramInfo[mu8_paramInfoIndex].au8_unit);

		switch( (EN_CBRAM_MAP_DATA_PARAMETERS)mu8_paramInfoIndex )
		{
		case CBRAM_DeviceTemperature:	
		case CBRAM_ProcessPressure:	
		case CBRAM_ProcessTemperature:
		case CBRAM_MemsTemperature:
			{
				snprintf(st_paramInfo->au8_converted , CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%02d", 
					(int)(mau32_paramValues[mu8_paramInfoIndex] / 100), (int)(mau32_paramValues[mu8_paramInfoIndex] % 100));
			}
			break;

		default:
			{
				snprintf(st_paramInfo->au8_converted , CC26XX_WEB_DEMO_CONVERTED_LEN, "%lu", mau32_paramValues[mu8_paramInfoIndex]);
			}
			break;
		}

		mu8_paramInfoIndex++;
		b_res = true;
	}
	else
	{
		mu8_paramInfoIndex = 0;
		b_res = false;
	}

	return ( b_res );

}

void CBRAM_TestReadWrite( void )
{
	uint8_t au8_buf[10];

	// Power up the RAM chip
	CBRAM_PowerUp();

	CBRAM_ReadRegister( CBRAM_READ_STATUS_REGISTER );

	CBRAM_ReadData( 0, 1 );

	// Send the Read Status instruction
	au8_buf[0] = 0xB6;

	CBRAM_WriteData( 0, au8_buf, 1 );

	CBRAM_ReadData( 0, 1 );

//	board_spi_close();
	CBRAM_PowerDown();
}

/*------------------------------------------------------------------------------------*/
/* LOCAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
*	Function: DecodeToken
*		The format of the token is: "paramId"-"unitCode"-"value"
*----------------------------------------------------------------------------*/
void ProcessToken( char * pc_dataToken )
{
	const char hypen[2]="-";
	char *token;
	uint16_t u16_param;
	uint16_t u16_unit;
	uint32_t u32_val;


//	for ( u8_loop = 0; u8_loop < u8_index; u8_loop++ )
//	{
		/* get the first token - paramId */
		token = strtok(pc_dataToken, hypen);
		u16_param = atoi(token);

		/* get the second token - unit */
		if( token != NULL ) {
			token = strtok(NULL, hypen);
			u16_unit = atoi(token);
		}

		/* get the third token - value */
		if( token != NULL ) {
			token = strtok(NULL, hypen);
			u32_val = atol(token);
		}

		// Write config values to ram
		if ( CBRAM_MaxConfigParameters > u16_param )
		{
			CBRAM_StoreConfiguration( u16_param, u32_val);
		}
//	}



}



/*---------------------------------------------------------------------------*/
void CBRAM_DecodeMemParams( void )
{
	uint8_t  u8_loop;
	uint8_t  u8_numBytes;

	// Set Receive index to the start of the data
	mst_cbram.ReceiveIndex = CBRAM_FIRST_DATA_BYTE; // start of data

	for( u8_loop = 0; u8_loop < CBRAM_MaxDataParameters; u8_loop++ )
	{
		// Get the size we need out of the map array
		u8_numBytes = mast_paramInfo[u8_loop].u8_len;

		// Get that number of bytes out of the receive buffer
		mau32_paramValues[u8_loop] = CBRAM_GetReceivedBytes( u8_numBytes );

	}

}

/*---------------------------------------------------------------------------*/
uint32_t CBRAM_GetReceivedBytes( uint8_t u8_numBytes )
{
	uint32_t u32_val = (uint32_t)0;

	// Check that we don't try to read beyound the transmit buffer
	if ( CBRAM_BUFFER_MAX > (mst_cbram.ReceiveIndex + u8_numBytes) )
	{
		switch( u8_numBytes )
		{
			case CBRAM_UINT8:
			{
				u32_val =  ( uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex];
				mst_cbram.ReceiveIndex++;
				
			}
			break;
			case CBRAM_UINT16:
			{
				u32_val =  ( ( uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex] << 8 ) & 0x0000ff00;
				mst_cbram.ReceiveIndex++;
				u32_val += (   uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex] & 0x000000ff;
				mst_cbram.ReceiveIndex++;

			}
			break;
			case CBRAM_UINT32:
			{
				u32_val =  ( ( uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex] << 24 ) & 0xff000000;
				mst_cbram.ReceiveIndex++;
				u32_val += ( ( uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex] << 16 ) & 0x00ff0000; 
				mst_cbram.ReceiveIndex++;
				u32_val += ( ( uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex] << 8 ) & 0x0000ff00;
				mst_cbram.ReceiveIndex++;
				u32_val += (   uint32_t )mst_cbram.ReceiveBuffer[mst_cbram.ReceiveIndex] & 0x000000ff;
				mst_cbram.ReceiveIndex++;
			}
			break;

			default:
			break;
		}

	}

	return ( u32_val );
}

/*---------------------------------------------------------------------------*/
void CBRAM_PutBytesInBuf( uint32_t u32_val, uint8_t *pu8_buf, uint8_t *pu8_bufIndex, uint8_t u8_paramLen )
{
	uint8_t u8_bufIndex = *pu8_bufIndex;

	// Check that we don't try to read beyound the transmit buffer
	if ( CBRAM_BUFFER_MAX > u8_bufIndex )
	{
		switch( u8_paramLen )
		{
			case CBRAM_UINT8:
			{
				pu8_buf[u8_bufIndex] = ( uint8_t )(u32_val & 0x000000ff);
				u8_bufIndex++;
			}
			break;
			case CBRAM_UINT16:
			{
				pu8_buf[u8_bufIndex] = ( uint8_t )((u32_val & 0x0000ff00) >> 8);
				u8_bufIndex++;
				pu8_buf[u8_bufIndex] = ( uint8_t )(u32_val & 0x000000ff);
				u8_bufIndex++;

			}
			break;
			case CBRAM_UINT32:
			{
				pu8_buf[u8_bufIndex] = ( uint8_t )((u32_val & 0xff000000) >> 24);
				u8_bufIndex++;
				pu8_buf[u8_bufIndex] = ( uint8_t )((u32_val & 0x00ff0000) >> 16);
				u8_bufIndex++;
				pu8_buf[u8_bufIndex] = ( uint8_t )((u32_val & 0x0000ff00) >> 8);
				u8_bufIndex++;
				pu8_buf[u8_bufIndex] = ( uint8_t )(u32_val & 0x000000ff);
				u8_bufIndex++;
			}
			break;

			default:
			{
			}
			break;
		}

	}

	*pu8_bufIndex = u8_bufIndex;

}

/*---------------------------------------------------------------------------*/
void CBRAM_EnableSPI( void )
{
		/* Enable Chip Select pin for SPI */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);
    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_CBRAM_CS);
    ti_lib_gpio_set_dio(BOARD_IOID_CBRAM_CS);

	// Configure the SPI
    board_spi_open(125000, BOARD_SPI0, SSI_FRF_MOTO_MODE_0);
}

/*---------------------------------------------------------------------------*/
void CBRAM_StoreConfiguration( uint16_t u16_param, uint32_t u32_val )
{
	uint8_t au8_buf[CBRAM_BUFFER_MAX];
	uint8_t u8_bufIndex = 0;
	uint8_t u8_paramLen = 0; 

	u8_paramLen = mast_paramConfig[u16_param].u8_len;
	CBRAM_PutBytesInBuf( u32_val, au8_buf, &u8_bufIndex, u8_paramLen  );

	// make sure the number of bytes to be sent is not greater than the transmit buffer
	if( CBRAM_BUFFER_MAX >= u8_bufIndex + 3)
	{

//		CBRAM_WriteData( mast_paramConfig[u16_param].u16_addr, au8_buf, u8_bufIndex );
	}
}


#if 0
/*---------------------------------------------------------------------------*/
void CBRAM_StoreRAMValues( uint8_t u8_processParams )
{
	uint8_t au8_buf[CBRAM_BUFFER_MAX];
	uint8_t u8_bufIndex = 0;
	uint8_t u8_loop;
	uint32_t u32_val;
	uint8_t u8_numParams;
	uint8_t u8_paramLen = 0; 

	switch( u8_processParams )
	{
	case CBRAM_PROCESS_PARAMS:
		{
			u8_numParams = CBRAM_NUM_PROCESS_PARAMS;
		}
		break;
	case CBRAM_DEVICE_PARAMS:
		{
			u8_numParams = CBRAM_NUM_DEVICE_PARAMS;
		}
		break;
	case CBRAM_MEMS_PARAMS:
		{
			u8_numParams = CBRAM_NUM_MEMS_PARAMS;
		}
		break;
	default:
		{
			u8_numParams = 0;
		}
		break;
	}

	// Make sure the number of parameters we are writing does not overflow the map
	if ( (u8_numParams > 0) && (u8_numParams < CBRAM_MaxDataParameters) )
	{
		for(u8_loop = 0; u8_loop < u8_numParams; u8_loop++)
		{
			u32_val = mau32_writeParamValues[u8_processParams + u8_loop];
			u8_paramLen = mast_paramInfo[u8_processParams + u8_loop].u8_len;
			CBRAM_PutBytesInBuf( u32_val, au8_buf, &u8_bufIndex, u8_paramLen  );
		}

		// make sure the number of bytes to be sent is not greater than the transmit buffer
		if( CBRAM_BUFFER_MAX >= u8_bufIndex + 3)
		{
			// Init/Open SPI for RAM chip
			CBRAM_EnableSPI( );
	
			CBRAM_WriteData( mast_paramInfo[u8_processParams].u16_addr, au8_buf, u8_bufIndex );

			// Shut down the SPI
			board_spi_close(BOARD_SPI0);

		}

	}

}
#endif

void CBRAM_EnableCS( void )
{
	// Enable CS
	ti_lib_gpio_clear_dio(BOARD_IOID_CBRAM_CS);
}

void CBRAM_DisableCS( void )
{

	while( SSIBusy( SSI0_BASE ) );

	// Disable CS
	ti_lib_gpio_set_dio(BOARD_IOID_CBRAM_CS);
}


bool CBRAM_WriteEnable( void )
{
	bool b_res;
	uint8_t au8_buf[1];

	// Enable CS
	CBRAM_EnableCS();

	// Send the WREN instruction 
	au8_buf[0] = CBRAM_WRITE_ENABLE;
	b_res = board_spi_write( au8_buf, 1, BOARD_SPI0 );

	// Disable CS
	CBRAM_DisableCS();

	return ( b_res );

}

bool CBRAM_ReadData( uint16_t u16_addr, uint8_t u8_len )
{
	bool b_res = false;
	uint8_t u8_loop;

	if( CBRAM_BUFFER_MAX  > u8_len )
	{

		// Send the Read Status instruction
		mst_cbram.TransmitBuffer[0] = CBRAM_READ_DATA_ARRAY;
		mst_cbram.TransmitBuffer[1] = (uint8_t)u16_addr >> 8;
		mst_cbram.TransmitBuffer[2] = (uint8_t)u16_addr & 0x00FF;
		mst_cbram.TransmitLength = 3;

		for( u8_loop = 0; u8_loop < u8_len; u8_loop++)
		{
//			printf( " CBRAM_ReadData RAM Values = %i\n", au8_buf[u8_loop] );
			mst_cbram.TransmitBuffer[mst_cbram.TransmitLength] = 0xff;
			mst_cbram.TransmitLength++;
		}
//		printf( "CBRAM_ReadData %i/n ",(int)b_res );
		b_res = CBRAM_SPI_Read_Write(  );
	}
	return ( b_res );

}

bool CBRAM_WriteData( uint16_t u16_addr, uint8_t *au8_buf, uint8_t u8_len )
{
	bool b_res = false;
	uint8_t u8_loop;

	// Set write enable bit
	CBRAM_WriteEnable();
	
	if( CBRAM_BUFFER_MAX  > u8_len )
	{
		// Send the Read Status instruction
		mst_cbram.TransmitBuffer[0] = CBRAM_WRITE_BYTES;
		mst_cbram.TransmitBuffer[1] = (uint8_t)u16_addr >> 8;
		mst_cbram.TransmitBuffer[2] = (uint8_t)u16_addr & 0x00FF;
		mst_cbram.TransmitLength = 3;

//		printf( " CBRAM_WriteData RAM Values =\n" );

		for( u8_loop = 0; u8_loop < u8_len; u8_loop++)
		{
//			DBG( " %x", au8_buf[u8_loop] );
			mst_cbram.TransmitBuffer[mst_cbram.TransmitLength] = au8_buf[u8_loop];
			mst_cbram.TransmitLength++;
		}

//		printf( " CBRAM_WriteData TRANSMIT BUF = %x %x %x %x %x %x %x\n", mst_cbram.TransmitBuffer[0], mst_cbram.TransmitBuffer[1], mst_cbram.TransmitBuffer[2], mst_cbram.TransmitBuffer[3], mst_cbram.TransmitBuffer[4],mst_cbram.TransmitBuffer[5],mst_cbram.TransmitBuffer[6] );
//		printf("TRANSMIT LENGTH %i\n",mst_cbram.TransmitLength );
		b_res = CBRAM_SPI_Write( );
	}

	return ( b_res );
}

bool CBRAM_ReadRegister( uint8_t u8_reg )
{
	bool b_res;

	switch ( u8_reg )
	{

	case CBRAM_READ_STATUS_REGISTER:
		{
			// Send the Read Status instruction
			mst_cbram.TransmitBuffer[0] = u8_reg;
			mst_cbram.TransmitBuffer[1] = 0xff;
			mst_cbram.TransmitBuffer[2] = 0xff;
			mst_cbram.TransmitLength = 3;
		}
	break;


#if 0
	case CBRAM_READ_MANF_ID:
		{
			// Send the Read Status instruction
			mst_cbram.TransmitBuffer[0] = u8_reg;
			mst_cbram.TransmitBuffer[1] = 0xff;
			mst_cbram.TransmitBuffer[2] = 0xff;
			mst_cbram.TransmitBuffer[3] = 0xff;
			mst_cbram.TransmitBuffer[4] = 0xff;
			mst_cbram.TransmitLength = 5;
		}
	break;
#endif
	

	default:
		{
			mst_cbram.TransmitLength = 0;
		}
		break;

	}

	b_res = CBRAM_SPI_Read_Write(  );

	return ( b_res );
}

bool CBRAM_SPI_Read_Write( void )
{
	bool b_res = false;

	// Enable CS
	CBRAM_EnableCS();

	if ( 0 != mst_cbram.TransmitLength )
	{
		b_res = board_spi_read_write(mst_cbram.TransmitBuffer, mst_cbram.ReceiveBuffer, mst_cbram.TransmitLength, BOARD_SPI0);
	}

	// Disable CS
	CBRAM_DisableCS();

	return ( b_res );

}

bool CBRAM_SPI_Write( void )
{
	bool b_res = false;

	// Enable write
//	CBRAM_WriteEnable( );

	// Enable CS
	CBRAM_EnableCS();

	if ( 0 != mst_cbram.TransmitLength )
	{
		printf("CBRAM_SPI_Write %d %d %d",mst_cbram.TransmitBuffer[0], mst_cbram.TransmitBuffer[1], mst_cbram.TransmitLength );
		// Enable CS
		ti_lib_gpio_clear_dio(BOARD_IOID_CBRAM_CS);
		b_res = board_spi_write(mst_cbram.TransmitBuffer, mst_cbram.TransmitLength, BOARD_SPI0);
	}

	// Disable CS
	CBRAM_DisableCS();

	return ( b_res );

}