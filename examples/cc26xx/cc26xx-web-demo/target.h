#ifndef __TARGET_H
#define __TARGET_H


#include <stdint.h>

/*!
    \fn uint8_t target_getB8( void * Target, unsigned char * Data )
    \brief Get 8-bit value from data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \returns Returns unsigned 8-bit value retrieved from data buffer
*/

uint8_t target_getB8( unsigned char * Data );


/*!
    \fn uint16_t target_getB16( void * Target, unsigned char * Data )
    \brief Get 16-bit value from data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \returns Returns unsigned 16-bit value retrieved from data buffer
*/

uint16_t target_getB16( unsigned char * Data );


/*!
    \fn uint32_t target_getB24( void * Target, unsigned char * Data )
    \brief Get 24-bit value from data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \returns Returns unsigned 32-bit value retrieved from data buffer
*/

uint32_t target_getB24( unsigned char * Data );

/*!
    \fn uint32_t target_getB32( void * Target, unsigned char * Data )
    \brief Get 32-bit value from data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \returns Returns unsigned 32-bit value retrieved from data buffer
*/

uint32_t target_getB32( unsigned char * Data );

/*!
    \fn uint64_t target_getB64( void * Target, unsigned char * Data )
    \brief Get 64-bit value from data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \returns Returns unsigned 64-bit value retrieved from data buffer
*/

uint64_t target_getB64( unsigned char * Data );


/*!
    \fn void target_setB8( void * Target, unsigned char * Data, uint8_t Value )
    \brief Set 8-bit value in data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \param Value Unsigned 8-bit value
    \returns No return value
*/

void target_setB8( unsigned char * Data, uint8_t Value );


/*!
    \fn void target_setB16( void * Target, unsigned char * Data, uint8_t Value )
    \brief Set 16-bit value in data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \param Value Unsigned 16-bit value
    \returns No return value
*/

void target_setB16( unsigned char * Data, uint16_t Value );


/*!
    \fn void target_setB32( void * Target, unsigned char * Data, uint8_t Value )
    \brief Set 32-bit value in data buffer
    \param Target Pointer to target API contextual data structure
    \param Data Pointer to data buffer
    \param Value Unsigned 32-bit value
    \returns No return value
*/

void target_setB32( unsigned char * Data, uint32_t Value );




#endif
