/*                            =======================
================================ C/C++ HEADER FILE =================================
                              =======================                          *//**
\file  i2c_target.h
description Handles the functionality of the I2C
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

#ifndef I2C_TARGET_H_
#define I2C_TARGET_H_

/*------------------------------------------------------------------------------------*/
/* INCLUDES */
/*------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------------*/
/* DEFINITIONS AND MACROS */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* TYPEDEFS, CLASSES AND STRUCTURES */
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
/* GLOBAL VARIABLES */
/*------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------*/
/* GLOBAL FUNCTIONS */
/*------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * \brief Select an I2C slave
 * \param interface The I2C interface to be used (BOARD_I2C_INTERFACE_0 or _1)
 * \param slave_addr The slave's address
 *
 * The various sensors on the sensortag are connected either on interface 0 or
 * 1. All sensors are connected to interface 0, with the exception of the MPU
 * that is connected to 1.
 */
void I2CTarget_Open(uint8_t u8_new_interface, uint8_t u8_address);

/**
 * \brief Burst read from an I2C device
 * \param buf Pointer to a buffer where the read data will be stored
 * \param len Number of bytes to read
 * \return True on success
 */
bool I2CTarget_Read(uint8_t *pu8_data, uint8_t u8_len);

/**
 * \brief Burst write to an I2C device
 * \param buf Pointer to the buffer to be written
 * \param len Number of bytes to write
 * \return True on success
 */
bool I2CTarget_Write(uint8_t *pu8_data, uint8_t u8_len);

/**
 * \brief Single write to an I2C device
 * \param data The byte to write
 * \return True on success
 */
bool I2CTarget_WriteSingle(uint8_t u8_data);

/**
 * \brief Write and read in one operation
 * \param wdata Pointer to the buffer to be written
 * \param wlen Number of bytes to write
 * \param rdata Pointer to a buffer where the read data will be stored
 * \param rlen Number of bytes to read
 * \return True on success
 */
bool I2CTarget_WriteRead(uint8_t *pu8_wData, uint8_t u8_wlen, uint8_t *pu8_rdata, uint8_t u8_rlen);

/**
 * \brief Enables the I2C peripheral with defaults
 *
 * This function is called to wakeup and initialise the I2C.
 *
 * This function can be called explicitly, but it will also be called
 * automatically by board_i2c_select() when required. One of those two
 * functions MUST be called before any other I2C operation after a chip
 * sleep / wakeup cycle or after a call to board_i2c_shutdown(). Failing to do
 * so will lead to a bus fault.
 */
void I2CTarget_Wakeup(void);

/**
 * \brief Stops the I2C peripheral and restores pins to s/w control
 *
 * This function is called automatically by the board's LPM logic, but it
 * can also be called explicitly.
 */
void I2CTarget_Close();
/*---------------------------------------------------------------------------*/
#endif /* I2C_TARGET_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
