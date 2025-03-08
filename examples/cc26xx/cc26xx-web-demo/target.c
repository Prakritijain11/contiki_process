#include <stdio.h>

uint8_t
target_getB8( unsigned char * Data )
{
    /* assert( Data != NULL ); */
    return ( uint8_t ) *Data;
}


uint16_t
target_getB16( unsigned char * Data )
{

    /* assert( Data != NULL ); */
    return ( uint16_t ) ( ( ( Data[0] << 8 ) & 0xff00 ) |
            ( Data[1] & 0x00ff ) );
}




uint32_t
target_getB32( unsigned char * Data )
{
    /* assert( Data != NULL ); */
    return ( uint32_t ) ( ( ( Data[0] << 24 ) & 0xff000000 ) |
            ( ( Data[1] << 16 ) & 0x00ff0000 ) |
            ( ( Data[2] << 8 ) & 0x0000ff00 ) |
            ( Data[3] & 0x000000ff ) );
}

uint32_t
target_getB24( unsigned char * Data )
{
    /* assert( Data != NULL ); */
    return ( uint32_t ) ( ( ( Data[0] << 16 ) & 0x00ff0000 ) |
            ( ( Data[1] << 8 ) & 0x0000ff00 ) |
            ( Data[2] & 0x000000ff ) );
}



uint64_t
target_getB64( unsigned char * Data )
{
    /* assert( Data != NULL ); */
    return ( uint64_t ) 0;
}


void
target_setB8( unsigned char * Data, uint8_t Value )
{
    Data[0] = Value;
}


void
target_setB16( unsigned char * Data, uint16_t Value )
{
    /* assert( Data != NULL ); */
    Data[0] = ( ( Value >> 8 ) & 0xff );
    Data[1] = ( Value & 0xff );
}


void
target_setB32( unsigned char * Data, uint32_t Value )
{

    /* assert( Data != NULL ); */
    Data[0] = ( ( Value >> 24 ) & 0xff );
    Data[1] = ( ( Value >> 16 ) & 0xff );
    Data[2] = ( ( Value >> 8 ) & 0xff );
    Data[3] = ( Value & 0xff );
}



