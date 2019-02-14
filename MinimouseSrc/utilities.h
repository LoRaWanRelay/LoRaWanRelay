/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Helper functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __UTILITIES_H__
#define __UTILITIES_H__
#include "stdint.h"
#include "LoraWanProcess.h"
/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns 2 raised to the power of n
 *
 * \param [IN] n power value
 * \retval result of raising 2 to the power n
 */
#define POW2( n ) ( 1 << n )

/*!
 * \brief Initializes the pseudo random generator initial value
 *
 * \param [IN] seed Pseudo random generator initial value
 */
void srand1( uint32_t seed );

/*!
 * \brief Computes a random number between min and max
 *
 * \param [IN] min range minimum value
 * \param [IN] max range maximum value
 * \retval random random value in range min..max
 */
int32_t randr( int32_t min, int32_t max );

/*!
 * \brief Copies size elements of src array to dst array
 *
 * \remark STM32 Standard memcpy function only works on pointers that are aligned
 *
 * \param [OUT] dst  Destination array
 * \param [IN]  src  Source array
 * \param [IN]  size Number of bytes to be copied
 */
void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size );

/*!
 * \brief Copies size elements of src array to dst array reversing the byte order
 *
 * \param [OUT] dst  Destination array
 * \param [IN]  src  Source array
 * \param [IN]  size Number of bytes to be copied
 */
void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size );

/*!
 * \brief Set size elements of dst array with value
 *
 * \remark STM32 Standard memset function only works on pointers that are aligned
 *
 * \param [OUT] dst   Destination array
 * \param [IN]  value Default value
 * \param [IN]  size  Number of bytes to be copied
 */
void memset1( uint8_t *dst, uint8_t value, uint16_t size );

/*!
 * \brief Converts a nibble to an hexadecimal character
 *
 * \param [IN] a   Nibble to be converted
 * \retval hexChar Converted hexadecimal character
 */
int8_t Nibble2HexChar( uint8_t a );

/*!
 * \brief Certification apllication layer
 *
 */
#ifdef SX126x_BOARD 
    int  Certification ( bool NewCommand , uint8_t * UserFport , uint8_t * UserPayloadSize, uint8_t * UserRxPayloadSize, uint8_t * MsgType, uint8_t * UserRxPayload, uint8_t * UserPayload, LoraWanObject< LoraRegionsEU, SX126x > *Lp);
#endif
#ifdef SX1272_BOARD  
    int  Certification ( bool NewCommand , uint8_t * UserFport , uint8_t * UserPayloadSize, uint8_t * UserRxPayloadSize, uint8_t * MsgType, uint8_t * UserRxPayload, uint8_t * UserPayload, LoraWanObject< LoraRegionsEU, SX1272 > *Lp);
#endif
#ifdef SX1276_BOARD  
    int  Certification ( bool NewCommand , uint8_t * UserFport , uint8_t * UserPayloadSize, uint8_t * UserRxPayloadSize, uint8_t * MsgType, uint8_t * UserRxPayload, uint8_t * UserPayload, LoraWanObject< LoraRegionsEU, SX1276 > *Lp);
#endif

/*!
 * \brief Crc64 implementation for flash corruption
 *
 */

#define POLY64REV     0x95AC9329AC4BC9B5
#define INITIALCRC    0xFFFFFFFFFFFFFFFF
void Crc64(uint8_t *dataIn, int size, uint32_t * crcLow, uint32_t * crcHigh );



/*!
 * \brief Trace Debug 
 *
 */
#ifdef DEBUG_TRACE_ENABLE
#define  TRACE_SIZE 256 // should be 64 bits aligned
extern uint32_t ExtDebugTrace[TRACE_SIZE+4] __attribute__((section(".NoInit")));  // Have to declare a section in the scatter/link file

#endif

void StoreTraceInFlash( uint32_t TraceFlashAdress );
void ReadTraceInFlash ( uint32_t TraceFlashAdress );
extern void InsertTrace (uint8_t id, uint8_t FileId);
void ReadTrace (uint32_t * DebugTrace);


#endif // __UTILITIES_H__
