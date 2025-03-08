/*                            =======================
================================ C/C++ HEADER FILE =================================
                              =======================                          *//**
\file  cbram.h
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



/*---------------------------------------------------------------------------*/
#ifndef CBRAM_H_
#define CBRAM_H_
/*---------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* INCLUDES */
/*------------------------------------------------------------------------------------*/
#include "cc26xx-web-demo.h"

/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/
// RAM Mapping
#define CBRAM_ADDR_TEMP                   (uint16_t)0x0004  // - 2 bytes
#define CBRAM_MAX_UNIT_SIZE               (uint8_t)6
#define CBRAM_MAX_DESCR_SIZE              (uint8_t)15

#define CBRAM_PROCESS_PARAMS  CBRAM_ProcessPressure
#define CBRAM_DEVICE_PARAMS   CBRAM_DeviceTemperature
#define CBRAM_MEMS_PARAMS     CBRAM_MemsTemperature

#define CBRAM_NUM_PROCESS_PARAMS         4
#define CBRAM_NUM_DEVICE_PARAMS          1
#define CBRAM_NUM_MEMS_PARAMS            7 

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS, CLASSES AND STRUCTURES */
/*------------------------------------------------------------------------------------*/
typedef struct
{
	char   au8_unit[CBRAM_MAX_UNIT_SIZE];
	char   au8_descr[CBRAM_MAX_DESCR_SIZE];
	char   au8_converted[CC26XX_WEB_DEMO_CONVERTED_LEN];
}ST_RAM_VALUES;

typedef enum
{
	CBRAM_ParamTypeConfig,
	CBRAM_ParamTypeData
}EN_CBRAM_PARAMETER_TYPE;

typedef enum
{
	CBRAM_DeviceTemperature,	
	CBRAM_ProcessPressure,	
	CBRAM_ProcessTemperature,
	CBRAM_ProcessPressStatus,
	CBRAM_ProcessTempStatus,		
	CBRAM_MemsTemperature,   
	CBRAM_MemsAccelAxisX,		
	CBRAM_MemsAccelAxisY,		
	CBRAM_MemsAccelAxisZ,		
	CBRAM_MemsGyroAxisX,		
	CBRAM_MemsGyroAxisY,		
	CBRAM_MemsGyroAxisZ,		
	CBRAM_MaxDataParameters
}EN_CBRAM_MAP_DATA_PARAMETERS;

typedef enum
{
	CBRAM_UpdateRateSec,	
	CBRAM_TestParam1,	
	CBRAM_MaxConfigParameters

}EN_CBRAM_MAP_CONFIG_PARAMETERS;


/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

void CBRAM_Init( void );
bool CBRAM_GetParameterValues( ST_RAM_VALUES * st_paramInfo );
void CBRAM_WriteConfigValue( char *pu8_data, uint16_t u16_dataLen );
void CBRAM_ReadMemParams( void );
void CBRAM_StoreRAMValues( void );
void CBRAM_WriteParamValue( uint8_t u8_param, uint32_t u32_val );
void CBRAM_TestReadWrite( void );
void CBRAM_PowerDown(void);
void CBRAM_PowerUp(void);

/*---------------------------------------------------------------------------*/
#endif /* CBRAM_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
