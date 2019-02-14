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
#include <stdlib.h>
#include <stdio.h>

#include "utilities.h"
#include "LoraWanProcess.h"
#include "ApiMcu.h"
#include "Define.h"
/*Trace Debug*/

static uint8_t TracePointer ;
#ifdef DEBUG_TRACE_ENABLE
    uint32_t ExtDebugTrace[TRACE_SIZE+4] __attribute__((section(".NoInit")));
#endif
void InsertTrace (uint8_t id, uint8_t FileId) {
#ifdef DEBUG_TRACE_ENABLE	
    ExtDebugTrace[ TRACE_SIZE - 1 ] ++;
    TracePointer = ExtDebugTrace[ TRACE_SIZE - 1 ] & 0xff;
    if ( id > 31 ) {
        DEBUG_MSG("ERROR TRACE COUNTER > 31\n");
        return;
    }
    if ( FileId > 7 ) {
        DEBUG_MSG("ERROR TRACE FILE_ID > 7\n");
        return;
    }
    ExtDebugTrace[ TracePointer ] = ( ( mcu.RtcGetTimeMs( )  & 0x00FFFFFF )<< 8 ) + (( FileId & 0x7) << 5) + (id & 0x1F) ;
#endif
}
void ReadTrace (uint32_t * DebugTrace) {
#ifdef DEBUG_TRACE_ENABLE
    int i;
    uint8_t TPointer ;
    uint8_t Fid;
    uint8_t id;
    TPointer = DebugTrace[ TRACE_SIZE - 1]&0xFF;
    DEBUG_MSG("\n Debug Trace = [ \n");
    for ( i = 0; i <  TRACE_SIZE - 1; i++ ) {
        Fid = (DebugTrace[(uint8_t)(TPointer - i)]&0xE0) >> 5;
        id  =  (DebugTrace[(uint8_t)(TPointer - i)]&0x1F);
        switch ( Fid ) {
            case 0 : 
                DEBUG_PRINTF(" @%.8d  id =  %.2x LoraWanProcess  file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                break;
            case 1 : 
                DEBUG_PRINTF(" @%.8d  id =  %.2x RegionsEU868    file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                break;
            case 2 : 
                switch ( id ) {
                    case 8 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file    RECEIVE CMD LINK_CHECK_ANS \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 9 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file   RECEIVE CMD LINK_ADR_ANS\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 10 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file   RECEIVE CMD LINK_ADR_ANS MULTIPLE\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 11 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file    RECEIVE CMD DUTY_CYCLE_ANS \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 12 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file   RECEIVE CMD RXPARRAM_SETUP_ANS\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 13 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file   RECEIVE CMD NEW_CHANNEL_ANS\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 14 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file    RECEIVE CMD RXTIMING_SETUP_ANS \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 15 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file   RECEIVE CMD TXPARAM_SETUP_ANS\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 16 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file   RECEIVE CMD DIC_CHANNEL_ANS\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    default:
                        DEBUG_PRINTF(" @%.8d  id =  %.2x MacLayer        file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                }
                break;
            case 3 :
                switch ( id ) {
                    case 6 :
                            DEBUG_PRINTF(" @%.8d  id =  %.2x PhyLayer        file    Wait For It \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                            break;
                    default:
                        DEBUG_PRINTF(" @%.8d  id =  %.2x PhyLayer        file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                 }
                 break;
            case 4 :
                switch ( id ) {
                    case 0 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Main            file    JUST RESET \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 2 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Main            file    Send \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 6 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Main            file    Sleep \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    default:
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Main            file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                }
                break;
            case 5 : 
                DEBUG_PRINTF(" @%.8d  id =  %.2x LoRaMacCrypto   file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                break;
            case 6 :
                switch ( id ) {
                    case 2 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Radio Isr       file    RECEIVE IT TX DONE \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    case 3 :
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Radio Isr       file    RECEIVE IT RX or TIMEOUT\n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                    default:
                        DEBUG_PRINTF(" @%.8d  id =  %.2x Radio Isr       file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                        break;
                }
                break;
            case 7 : 
                DEBUG_PRINTF(" @%.8d  id =  %.2x Timer Isr       file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                break;
            default :
                DEBUG_PRINTF(" @%.8d  id =  %.2x Unknown file \n", (DebugTrace[(uint8_t)(TPointer - i)]>>8), id);
                break;
        }
    }
DEBUG_MSG("] \n\n");
#endif
}

void StoreTraceInFlash( uint32_t TraceFlashAdress ) {
#ifdef DEBUG_TRACE_ENABLE	
    mcu.StoreContext(&ExtDebugTrace[0], TraceFlashAdress , ( TRACE_SIZE >> 1 )); 
#endif
}
void ReadTraceInFlash ( uint32_t TraceFlashAdress ) {
#ifdef DEBUG_TRACE_ENABLE
    uint32_t TraceTemp[TRACE_SIZE];
    mcu.RestoreContext((uint8_t * )(&TraceTemp[0]), TraceFlashAdress, TRACE_SIZE * 4);
    ReadTrace ( TraceTemp );
#endif
}
/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

int32_t rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

void srand1( uint32_t seed )
{
    next = seed;
}
// Standard random functions redefinition end

int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
}

void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

void memset1( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}

int8_t Nibble2HexChar( uint8_t a )
{
    if( a < 10 )
    {
        return '0' + a;
    }
    else if( a < 16 )
    {
        return 'A' + ( a - 10 );
    }
    else
    {
        return '?';
    }
}



#define POLY64REV     0x95AC9329AC4BC9B5
#define INITIALCRC    0xFFFFFFFFFFFFFFFF

void Crc64(uint8_t *dataIn, int size,uint32_t * crcLow, uint32_t * crcHigh )
{
    int i, j;
    uint64_t crc = INITIALCRC, part;
    static int init = 0;
    static  uint64_t  CRCTable[256];
    
    if (!init)
    {
        init = 1;
        for (i = 0; i < 256; i++)
        {
            part = i;
            for (j = 0; j < 8; j++)
            {
               if (part & 1)
                   part = (part >> 1) ^ POLY64REV;
               else
                   part >>= 1;
            }
            CRCTable[i] = part;
        }
    }
    
    for (i = 0 ; i < size ; i++)
    {
        crc = CRCTable[(crc ^ *dataIn++) & 0xff] ^ (crc >> 8);
    }
    /* 
    The output is done in two parts to avoid problems with 
    architecture-dependent word order
    */
    *crcLow = crc & 0xffffffff;
    *crcHigh = (crc >> 32) & 0xffffffff ; 
}

#ifdef SX126x_BOARD 
    int  Certification ( bool NewCommand , uint8_t * UserFport , uint8_t * UserPayloadSize, uint8_t * UserRxPayloadSize, uint8_t * MsgType, uint8_t * UserRxPayload, uint8_t * UserPayload, LoraWanObject< LoraRegionsEU, SX126x > *Lp){
#endif
#ifdef SX1272_BOARD  
    int  Certification ( bool NewCommand , uint8_t * UserFport , uint8_t * UserPayloadSize, uint8_t * UserRxPayloadSize, uint8_t * MsgType, uint8_t * UserRxPayload, uint8_t * UserPayload, LoraWanObject< LoraRegionsEU, SX1272 > *Lp){
#endif
#ifdef SX1276_BOARD  
    int  Certification ( bool NewCommand , uint8_t * UserFport , uint8_t * UserPayloadSize, uint8_t * UserRxPayloadSize, uint8_t * MsgType, uint8_t * UserRxPayload, uint8_t * UserPayload, LoraWanObject< LoraRegionsEU, SX1276 > *Lp){
#endif
    uint32_t temp ;
    static uint16_t FcntDwnCertif = 0;
    static uint32_t MsgTypePrevious = UNCONF_DATA_UP ;
    int i ;
    *UserFport       = 224;
    *UserPayloadSize = 2;
    *MsgType = UNCONF_DATA_UP ;
    if ( NewCommand == true) {
        switch ( UserRxPayload[0] ) {
            case 0 :  // end of test
                *UserFport       = 3;
                *UserPayloadSize = 14;
                for ( i = 0; i < 14 ; i ++) {
                    UserPayload[i]  = i;
                }
                break;
            case 1 :
                temp =  ( UserRxPayload[0] << 24 ) + ( UserRxPayload[1] << 16 ) + ( UserRxPayload[2] << 8 ) + ( UserRxPayload[3] );
                if ( temp == 0x01010101) {
                     Lp->SetDataRateStrategy( STATIC_ADR_MODE );
                     FcntDwnCertif   = 0;
                     UserPayload[0]  = FcntDwnCertif >> 8;
                     UserPayload[1]  = FcntDwnCertif & 0xFF;
                }
                break;            
            case 2 :  // Confirmed Uplink
                *MsgType = CONF_DATA_UP ; 
                MsgTypePrevious = *MsgType;
                UserPayload[0]  = FcntDwnCertif >> 8;
                UserPayload[1]  = FcntDwnCertif & 0xFF;
                break;
            case 3 :  // UnConfirmed Uplink
                *MsgType = UNCONF_DATA_UP ;
                MsgTypePrevious = *MsgType;   
                UserPayload[0]  = FcntDwnCertif >> 8;
                UserPayload[1]  = FcntDwnCertif & 0xFF;            
                break;
            case 4 :  //echo payload
                *UserPayloadSize = *UserRxPayloadSize;
                UserPayload[0] = 4;
                for ( i = 1 ; i < (*UserPayloadSize); i++ ) {
                    UserPayload[i]  = UserRxPayload [i] + 1;
                }
                break;
            case 5 :  // link check request 
              *UserPayloadSize = 1;
              UserPayload[0]  = 2;
              UserFport       = 0;
            case 6 :  // rejoin 
               Lp->SetOtaDevice (OTA_DEVICE);
               Lp->NewJoin( );
               *UserFport       = 3;
               *UserPayloadSize = 14;
               for ( i = 0; i < 14 ; i ++) {
                    UserPayload[i]  = i;
                }
                return(0);
            default :
                break;
        }
        FcntDwnCertif++;
    } else { // for the case of echo cmd
         *MsgType = MsgTypePrevious;
    }
    return ( UserRxPayload[0] );
}


sLoRaWanKeys DeriveKeyForTest ( uint8_t * uid, uint32_t devaddr ){
    uint8_t LoRaMacNwkSKeyInit[] = { 0x22, 0x33, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    uint8_t LoRaMacAppSKeyInit[] = { 0x11, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
    uint8_t LoRaMacAppKeyInit[]  = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xBB};
    uint8_t AppEuiInit[]         = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xff, 0x50 };
    uint8_t DevEuiInit[]         = { 0x11, 0x22, 0x33, 0x44, 0x44, 0x33, 0xcc, 0xbb };    
    sLoRaWanKeys  LoRaWanKeys ={LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, devaddr,OTA_DEVICE};
    memcpy( &(LoRaWanKeys.DevEui[0]), uid, 8 );
    return (LoRaWanKeys);
}

