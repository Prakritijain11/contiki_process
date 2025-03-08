/*
 * crc.h
 *
 *  Created on: Jan 13, 2017
 *      Author: i01505402
 */

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>
#include "stdbool.h"

//******************************************************************************
//
// PROTOTYPES
//
//******************************************************************************

uint16_t CalculateCrc(  const uint8_t * const pu8_Buffer, uint32_t u32_Length );
bool IsCRCIdentical( const uint8_t * const pu8_Buffer, uint8_t u8_FrameLength );

#endif /* CRC_H_ */
