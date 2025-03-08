/*
 * FixedPointConversion.c
 *
 *  Created on: Jan 17, 2017
 *      Author: i01505402
 */

// ===========================================================================
//     INCLUDES
// ===========================================================================
//lint_comment:
//esc: Code from M-Platform
//lint -save -e740 -e793 -e912 -e530 -e953

/** \name Generic definition of boolean values, used in FF and PA Stack */
/*\{*/
//#define B_FALSE   FALSE /**< boolean FALSE */
//#define B_TRUE    TRUE /**< boolean TRUE */
/*\}*/
// Common structured datatypes



#include "FixedPointConversion.h"

#define P_RANGE             ((float)1000.0f)

typedef union   //lint !e960 esc: code from M-Platform
{
   uint8_t    abValue[4];
   uint16_t   awValue[2];
   uint32_t   dwValue;
   int32_t    sdwValue;
   float      fValue;

} U_DWORD_FLOAT;


// ConvertFixedPointToFloatingPoint :: FixedPointConversion ==================
//
// Original Author:   Dirk Rapp / PT
// --------------------------------------------------------------------
// Revision History:
//
// Version:     Device Type Vx.y
// Date:        04.09.2001
// Author:      Dirk Rapp / PT
// Change(s):   Start initial release
// ----------
// Date:        20.03.02
// Author:      B.Lindner
// Change(s):   Complete new function with very low memory consumption
// --------------------------------------------------------------------
// Description: function to convert fixed-point values to floating-point values
//
// Calling Sequence:
// ---------------------------------------------------------------------------

float ConvertFixedPointToFloatingPoint(uint32_t dwValue)
{
   // How to calculate a fix's value:
   // x = -bit31/2 + bit30/4 + bit29/8 + ...

   U_DWORD_FLOAT uResult;
   int8_t sbShiftDistance = 0;

   if (dwValue==0) return (0.0); // If fixed-point number is 0, we can't clculate an exponent so we return 0.0

   if ((dwValue & 0x80000000) == 0x80000000) // Negative number?
   {
      uResult.dwValue = 0x80000000; // Add sign bit to result
      dwValue = ~(dwValue-1); // Convert negative fixed-point number to positive (complement-on-two)
   }
   else
   {
      uResult.dwValue = 0x00000000;
   }

   while ((dwValue & 0x80000000) != 0x80000000) // Normalize mantissa to get a 32-bit mantissa with MSB = 1
   {
      dwValue <<= 1;
      sbShiftDistance++; // count shifts for later exponent calculation
   }
   dwValue <<= 1;

   uResult.dwValue |= ((uint32_t)(126 - sbShiftDistance) << 23); //lint !e571 esc: M-platform code // Calculate exponent and add it to result
   uResult.dwValue |= (dwValue >> 9) & 0x007FFFFF; // Add mantissa to result

   return (uResult.fValue);
}

/*---------------------------------------------------------------------------------*//**
\description    ConvertPSPFixedPointToPressure():
                Function to convert a PSP pressure value (fix point) to an useable
                floating point pressure value in [Pa] unit.

\param[in]      fInternalPressure: normalized (-0.5 .. 0.5) input value

\param[out]     none

\return         output value in floating point format and in Pa unit
*//*----------------------------------------------------------------------------------*/
float ConvertPSPFixedPointToPressure(const uint32_t dwPressureValue)
{
    // How to calculate a fix's value:
    // x = -bit31/2 + bit30/4 + bit29/8 + ...
    float fPressureValue=0.0f;
    float fPressureRange= P_RANGE;

    fPressureValue = ConvertFixedPointToFloatingPoint(dwPressureValue) * 4.0f /* to fit the PSP normalized value (-0.25..0.25) to standard factor value (-1..1) */;

    return (fPressureValue * fPressureRange);
}

/*---------------------------------------------------------------------------------*//**
\description    ConvertPSPFixedPointToTemperature():
                Function to convert a PSP temperature value (fix point) to an usable
                floating point temperature value in Kelvin.
                edP: For Sensor HP

\param[in]      sdwTemperatureValue:  normalized (-0.5 .. 0.5) input value

\param[out]     none

\return         output value in floating point format and in Kelvin
*//*----------------------------------------------------------------------------------*/
#define ZERO_CELSIUS                            ((float)273.15)    // Zero degrees celsius [K]
#define MEASURE_LAROUSSE_TEMPERATURE_OFFSET     (ZERO_CELSIUS + (float)25.0) // Temperature offset [K]
#define PSP_INTERNAL_TEMPERATURE_MAX             ((float)+0.25)  // Maximum internal temperature
#define PSP_INTERNAL_TEMPERATURE_MINMAX_ABSRANGE (PSP_INTERNAL_TEMPERATURE_MAX)  // Absolutely largest internal temperature
#define TEMPERATURE_ABSOLUTE_RANGE              ((float)358.15)   // max range in [K] 85°C

float ConvertPSPFixedPointToTemperature(const uint32_t dwTemperatureValue)
{
    // How to calculate a fix's value:
    // x = -bit31/2 + bit30/4 + bit29/8 + ...
    float fTemperatureValue;
    float fTemperatureAbsRange = TEMPERATURE_ABSOLUTE_RANGE; // [K]


    fTemperatureValue = ConvertFixedPointToFloatingPoint(dwTemperatureValue);

    fTemperatureValue = (fTemperatureValue * ( (fTemperatureAbsRange - ZERO_CELSIUS) / PSP_INTERNAL_TEMPERATURE_MINMAX_ABSRANGE)) + MEASURE_LAROUSSE_TEMPERATURE_OFFSET;

    // Convert to celsius
    fTemperatureValue = (float)fTemperatureValue - ZERO_CELSIUS;

    return (fTemperatureValue);
}


//lint -restore

// ---- End of FixedPointConversion.C ----------------------------------------