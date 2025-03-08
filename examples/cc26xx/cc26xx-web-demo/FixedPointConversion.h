/*
 * FixedPointConversion.h
 *
 *  Created on: Jan 17, 2017
 *      Author: i01505402
 */

#ifndef FIXEDPOINTCONVERSION_H_
#define FIXEDPOINTCONVERSION_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus
// ============================================================================
//     INCLUDES
// ============================================================================
#include "stdint.h"
#include "float.h"
// ===========================================================================
//    GLOBAL SYMBOL DEFINITIONS
// ===========================================================================

// ===========================================================================
//    GLOBAL FUNCTION PROTOTYPES
// ===========================================================================

float ConvertFixedPointToFloatingPoint ( uint32_t dwValue );
float ConvertPSPFixedPointToPressure(const uint32_t dwPressureValue);
float ConvertPSPFixedPointToTemperature(const uint32_t dwTemperatureValue);

#ifdef __cplusplus
}
#endif //__cplusplus


#endif /* FIXEDPOINTCONVERSION_H_ */
