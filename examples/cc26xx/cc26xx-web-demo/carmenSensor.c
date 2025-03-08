/*                            =======================
================================ C/C++ SOURCE FILE =================================
=======================                          *//**
\file  carmenSensor.c
description Handles the functionality of the CARMEN sensor
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
#include "gpio-interrupt.h"
#include "spi.h"
#include "target.h"
#include "crc.h"
#include "FixedPointConversion.h"
#include "cbram.h"
#include "board-spi.h"

/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/
#define CARMEN_BUFFER_MAX       (20)

#define RDY_IO_CFG              (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO |       \
                                 IOC_IOPULL_DOWN  | IOC_SLEW_DISABLE  |       \
                                 IOC_HYST_DISABLE | IOC_RISING_EDGE   |       \
                                 IOC_INT_DISABLE  | IOC_IOMODE_NORMAL |       \
                                 IOC_WAKE_ON_HIGH | IOC_INPUT_ENABLE)

#define  CARMEN_DUMMYBYTE        0xff

/* CARMEN COMMANDS
 *
 */
#define CARMEN_WRITE_EEPROM         (0x02)
#define CARMEN_READ_EEPROM          (0x03)
#define CARMEN_READ_STATUS          (0x05)
#define CARMEN_WRITEEXT_EEPROM      (0x12)
#define CARMEN_WRITE_REG            (0x32)
#define CARMEN_READ_REG             (0x33)
#define CARMEN_READ_MEASFRAME1      (0x35)
#define CARMEN_READ_MEASFRAME2      (0x36)

// Read Register bit masks
#define CARMEN_REQ_CMD_MASK        (0x00)
#define CARMEN_REQ_BL_MASK         (0x03)
#define CARMEN_RPLY_BL_MASK        (0x02)

#define CARMEN_HEADERCRC_SIZE      (0x05)
#define CARMEN_REPLYMSG_SIZE       (0x04)

/* 
* CARMEN STATUS BYTE MASKS
*/
#define CARMEN_STATUS_MEAS_VALID_MASK    (0x01)
#define CARMEN_STATUS_DSP_RUN_MASK       (0x80)

#define MAX_AMOUNT_OF_BLOCKS_FOR_READ_ACCESS    (uint8_t)(4)

#define CARMEN_STARTUP_DELAY				 0xffff  // ~14mS

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS AND STRUCTURES */
/*------------------------------------------------------------------------------------*/
typedef enum _CARMEN_STATUS
{
    CARMEN_STATUS_BAD = 0,
	CARMEN_STATUS_GOOD
}
EN_CARMEN_STATUS;

typedef struct _CARMEN_SESSION
{
    uint8_t ReceiveBuffer[ CARMEN_BUFFER_MAX ];

    uint8_t ReceiveLength;

    uint8_t TransmitBuffer[ CARMEN_BUFFER_MAX ];

    uint8_t TransmitLength;
}
CARMEN_SESSION;

typedef enum _EN_CARMEN_STATUS_BYTES
{
    ///< Conditionner
    eCSTATUS_STAT1 = 0,     // Byte 0
    eCSTATUS_STAT2 = 1,     // Byte 1
    eCSTATUS_STAT3 = 2,     // Byte 2
    eCSTATUS_AMOUNT= 3      // Amount of bytes required

} EN_CARMEN_STATUS_BYTES;

struct ST_SENSOR_VALUES {
	float            f32_staticPressure;      
	float            f32_temp; 
	EN_CARMEN_STATUS en_pressStatus;
	EN_CARMEN_STATUS en_tempStatus;
};

/*------------------------------------------------------------------------------------*/
/* PROTOTYPES */
/*------------------------------------------------------------------------------------*/
static void carmen_rdy_interrupt_handler(uint8_t ioid);
void carmen_GetResponse( void );
void carmen_BuildDummyMsg( uint8_t u8_length );
bool carmen_Transmit( void );
void carmen_ResetTxBuf( void );
void carmen_IncrementTxPtr( uint8_t u8_size );
uint16_t target_getCARMENB16( unsigned char * Data );
uint32_t target_getCARMENB24( unsigned char * Data );
void carmen_ReadMeasurement( void );
void carmen_DecodeReply( void );
void DecodeReadRegister( void );
void DecodeReadMeasureFrame1( void );
void carmen_getValues();
void carmen_configRDY(void);

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* LOCAL VARIABLES */
/*------------------------------------------------------------------------------------*/

const uint8_t  CARMEN_STATUS_BYTES        = 3;

static volatile uint32_t value;
static struct ST_SENSOR_VALUES st_sensorValues;
static CARMEN_SESSION  mst_carmenSession;
uint8_t  mau8_status[]   = {0,0,0};
static volatile bool mb_isMeasureReady = false;


/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/
void carmen_PowerDown(void)
{
	// Pull the Chip Select low
	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SENSOR_CS);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_SENSOR_CS, IOC_IOPULL_DOWN);

	// Pull the RDY low
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_SENSOR_RDY, IOC_IOPULL_DOWN);

	// Shut down the SPI
	board_spi_close(BOARD_SPI1);

	/* Set Power pin low */
	ti_lib_gpio_clear_dio(BOARD_IOID_SENSOR_PWR);
}

void carmen_PowerUp(void)
{

	static uint32_t u32_ticks = CARMEN_STARTUP_DELAY;

	// Power up the CARMEN - Set up the Power pin 
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SENSOR_PWR);
	ti_lib_gpio_set_dio(BOARD_IOID_SENSOR_PWR);

	// Set up the Chip Select pin for - Output high to disable
	ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SENSOR_CS);
	ti_lib_gpio_set_dio(BOARD_IOID_SENSOR_CS);

	// Configure the RDY pin
	carmen_configRDY();

	// Configure the SPI
	board_spi_open(125000, BOARD_SPI1, SSI_FRF_MOTO_MODE_1);

	// Delay for power on
	while ( u32_ticks > 0 )
	{
		u32_ticks--;
	}
}

void carmen_RequestMeasure( void )
{

	carmen_PowerUp();

	mb_isMeasureReady = false;

	carmen_ReadMeasurement( );
	(void)carmen_Transmit();
	ti_lib_ioc_int_enable(BOARD_IOID_SENSOR_RDY);

	while( false == mb_isMeasureReady );

	carmen_getValues();

	carmen_PowerDown();

}


/*------------------------------------------------------------------------------------*/
/* LOCAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the button sensor for all buttons.
 *
 * \param type SENSORS_HW_INIT: Initialise. SENSORS_ACTIVE: Enables/Disables
 *        depending on 'value'
 * \param value 0: disable, non-zero: enable
 * \return Always returns 1
 */
void carmen_configRDY(void)
{

    ti_lib_ioc_int_disable(BOARD_IOID_SENSOR_RDY);
    ti_lib_gpio_clear_event_dio(BOARD_IOID_SENSOR_RDY);



    /* Enable the GPIO clock when the CM3 is running */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);


    /* S/W control, input, pull-down */
    ti_lib_ioc_port_configure_set(BOARD_IOID_SENSOR_RDY, IOC_PORT_GPIO,
                                  RDY_IO_CFG);

    gpio_interrupt_register_handler(BOARD_IOID_SENSOR_RDY,
                                    carmen_rdy_interrupt_handler);


}

void carmen_ReadMeasurement( void  )
{
    uint16_t u16_crc;

    carmen_ResetTxBuf(  );

    // CMD
    target_setB8( &mst_carmenSession.TransmitBuffer[ mst_carmenSession.TransmitLength ], CARMEN_READ_MEASFRAME1 );
    carmen_IncrementTxPtr( sizeof(uint8_t) );

    u16_crc = CalculateCrc( mst_carmenSession.TransmitBuffer, 1 );

    // CRC0
    target_setB8( &mst_carmenSession.TransmitBuffer[ mst_carmenSession.TransmitLength ], (uint8_t)( u16_crc & 0xFF ) );
    carmen_IncrementTxPtr( sizeof(uint8_t) );

    // CRC1
    target_setB8( &mst_carmenSession.TransmitBuffer[ mst_carmenSession.TransmitLength ], (uint8_t)( ( u16_crc >> 8 ) & 0xFF ) );
    carmen_IncrementTxPtr( sizeof(uint8_t) );

}

void carmen_GetResponse( void )
{
    uint32_t u32_length;
    uint8_t  u8_length;
    uint8_t  u8_cmd;

    u8_cmd = mst_carmenSession.TransmitBuffer[CARMEN_REQ_CMD_MASK];

    switch( u8_cmd )
    {
        case CARMEN_READ_REG:
        {
            // Retrieve the length of the block so we can determine how many data bytes to receive
            u32_length = (uint32_t)(mst_carmenSession.TransmitBuffer[CARMEN_REQ_BL_MASK] * sizeof(uint32_t));
            u32_length += CARMEN_HEADERCRC_SIZE;

            u32_length &= 0xff;
            if( CARMEN_BUFFER_MAX > u32_length )
            {
                // Put dummy bytes into the txBuffer
                carmen_BuildDummyMsg( (uint8_t)u32_length );
            }

        }
        break;

        case CARMEN_READ_MEASFRAME1:
            {
                // Retrieve the length of the block so we can determine how many data bytes to receive
                u8_length = (uint8_t)13;

                if( CARMEN_BUFFER_MAX > u8_length )
                {
                    // Put dummy bytes into the txBuffer
                    carmen_BuildDummyMsg( u8_length );
                }

            }
            break;

        default:
        {

        }
        break;
    } // end of switch



}

void carmen_BuildDummyMsg( uint8_t u8_length )
{
    uint8_t u8_loop;

    mst_carmenSession.TransmitLength = 0;

    if ( u8_length < CARMEN_BUFFER_MAX )
    {
        for( u8_loop = 1; u8_loop < u8_length; u8_loop++ )
        {
            mst_carmenSession.TransmitBuffer[u8_loop] = CARMEN_DUMMYBYTE;
        }

        mst_carmenSession.TransmitLength = u8_length;
     }

}

/*---------------------------------------------------------------------------*/

bool carmen_Transmit(void)
{
	bool b_res;

	ti_lib_gpio_clear_dio(BOARD_IOID_SENSOR_CS);
	//b_res = carmen_spi_read_write( mst_carmenSession.TransmitBuffer, mst_carmenSession.ReceiveBuffer, mst_carmenSession.TransmitLength );
	b_res = board_spi_read_write( mst_carmenSession.TransmitBuffer, mst_carmenSession.ReceiveBuffer, mst_carmenSession.TransmitLength, BOARD_SPI1 );

	return b_res;
}



void carmen_getValues()
{
	// The Sensor is ready to send the response, clock out the bytes
    // to receive the response.
    carmen_GetResponse();

	carmen_Transmit();

    ti_lib_gpio_set_dio(BOARD_IOID_SENSOR_CS);
	// We have a response from the sensor, decode the reply
    carmen_DecodeReply( );
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Handler for Carmen RDY interrupts
 */
static void
carmen_rdy_interrupt_handler(uint8_t ioid)
{
		
	// Disable the RDY interrupt so we can transmit again
	ti_lib_ioc_int_disable(BOARD_IOID_SENSOR_RDY);
	ti_lib_gpio_clear_event_dio(BOARD_IOID_SENSOR_RDY);

	mb_isMeasureReady = true;
}

void carmen_ResetTxBuf( void )
{

    mst_carmenSession.TransmitLength = 0;

}

void carmen_IncrementTxPtr( uint8_t u8_size )
{

    if( mst_carmenSession.TransmitLength < CARMEN_BUFFER_MAX + u8_size )
    {
        mst_carmenSession.TransmitLength += (uint16_t)u8_size;
    }
    else
    {
        mst_carmenSession.TransmitLength = 0;
    }



}
uint16_t
target_getCARMENB16( unsigned char * Data )
{

    /* assert( Data != NULL ); */
    return ( uint16_t ) ( ( ( Data[1] << 8 ) & 0xff00 ) |
            ( Data[0] & 0x00ff ) );
}

uint32_t
target_getCARMENB24( unsigned char * Data )
{

    /* assert( Data != NULL ); */
    return ( uint32_t ) ( ( ( Data[2] << 16 ) & 0x00ff0000 ) |
            ( ( Data[1] << 8 ) & 0x0000ff00 ) |
            ( Data[0] & 0x000000ff ) );
}

void carmen_DecodeReply( void )
{
	st_sensorValues.f32_temp = 22;
    // Make sure length is not to big for the buffer
    if( CARMEN_BUFFER_MAX > mst_carmenSession.TransmitLength )
    {
        // Validate the CRC
        if( IsCRCIdentical( mst_carmenSession.ReceiveBuffer, mst_carmenSession.TransmitLength ) )
        {
            // Validate the CMD byte we received matches the one we sent
            if( mst_carmenSession.TransmitBuffer[CARMEN_REQ_CMD_MASK] == mst_carmenSession.ReceiveBuffer[CARMEN_REQ_CMD_MASK] )
            {
                // The response looks ok, decode the reply
                switch( mst_carmenSession.ReceiveBuffer[CARMEN_REQ_CMD_MASK] )
                {
                    case CARMEN_READ_REG:
                    {
                        DecodeReadRegister( );
                    }
                    break;

                    case CARMEN_READ_MEASFRAME1:
                        {
                            DecodeReadMeasureFrame1( );
                        }
                        break;

                    default:
                    {

                    }
                    break;
                } // end of switch

            }
        }
    }
    else
    {
        // Invalid CRC don't dump the response
    }

 
}

void DecodeReadRegister( void )
{

}

void DecodeReadMeasureFrame1( void )
{
	uint32_t u32_ADC1;
    uint16_t u16_ADC2;
	float f32_val;
	uint32_t u32_val;

    // Parse the measurement frame, then get the values for A/D 1 & 2.
    u32_ADC1 = target_getCARMENB24( &mst_carmenSession.ReceiveBuffer[ 1 ] );
    u16_ADC2 = target_getCARMENB16( &mst_carmenSession.ReceiveBuffer[ 4 ] );

	/* ADC1 Conversion of the fixed point 24bits to 32 bits*/
	u32_ADC1 = u32_ADC1 << 8;
    f32_val = ConvertPSPFixedPointToPressure(u32_ADC1);
	u32_val = (int)(f32_val * 100);
	CBRAM_WriteParamValue( CBRAM_ProcessPressure, u32_val );
	
	f32_val = ConvertPSPFixedPointToTemperature(u16_ADC2);
	u32_val = (int)(f32_val * 100);
	CBRAM_WriteParamValue( CBRAM_ProcessTemperature, u32_val );
	
	mau8_status[eCSTATUS_STAT1] = target_getB8( &mst_carmenSession.ReceiveBuffer[ 8 ]  );
    mau8_status[eCSTATUS_STAT2] = target_getB8( &mst_carmenSession.ReceiveBuffer[ 9 ]  );
    mau8_status[eCSTATUS_STAT3] = target_getB8( &mst_carmenSession.ReceiveBuffer[ 10 ] );

	u32_val = (uint32_t)CARMEN_STATUS_BAD;
	if( (0 == (mau8_status[eCSTATUS_STAT1] & 0x10) ) && (0 == mau8_status[eCSTATUS_STAT2]) )
    {
        // None of the fault bits are set, set status to good
        u32_val = (uint32_t)CARMEN_STATUS_GOOD;
    }

	CBRAM_WriteParamValue( CBRAM_ProcessPressStatus, u32_val );

	u32_val = (uint32_t)CARMEN_STATUS_BAD;
	if( ( 0 == (mau8_status[eCSTATUS_STAT1] & 0x60) ) && (0 == mau8_status[eCSTATUS_STAT2]) )
    {
        // None of the fault bits are set, set status to good
        u32_val = (uint32_t)CARMEN_STATUS_GOOD;
    }
	CBRAM_WriteParamValue( CBRAM_ProcessTempStatus, u32_val );


	if( (mau8_status[eCSTATUS_STAT3] & CARMEN_STATUS_MEAS_VALID_MASK) != CARMEN_STATUS_MEAS_VALID_MASK )
	{
//			printf("CC26XX Data Not Valid %d\n", (int)mau8_status[eCSTATUS_STAT3] );
	}

}




/**
 * @}
 */
