/*

  __  __ _       _                                 
 |  \/  ( _)     ( _)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | ( _) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : LoraWan Regions Specific objets.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin ( SEMTECH)
*/
#ifndef REGIONS_H
#define REGIONS_H
#include "stdint.h"
#include "Define.h"
#include "MacLayer.h"

/*****************************************************************/
/*         Define specific for EU 863-870                        */
/*****************************************************************/

//template class LoraWanContainer<16,SX1276>;
template < class R >
class LoraRegionsEU : public LoraWanContainer<16,R> { 

public: 
    LoraRegionsEU ( sLoRaWanKeys LoRaWanKeys, RadioPLaner<R> * RadioUser, uint32_t FlashAdress ); 
    ~LoraRegionsEU ( void ) {};
/*********************************************************************/
/*            Define Regional parameter                              */
/*********************************************************************/
    static const int      JOIN_ACCEPT_DELAY1 = 5 ; // define in seconds
    static const int      JOIN_ACCEPT_DELAY2 = 6 ; // define in seconds
    static const int      RECEIVE_DELAY1     = 1 ; // define in seconds
    static const int      TX_POWER           = 14 ; // define in db
    static const int      ADR_ACK_LIMIT      = 64 ;
    static const int      ADR_ACK_DELAY      = 32 ;
    static const int      ACK_TIMEOUT        = 2 ;// +/- 1 s (random delay between 1 and 3 seconds)
    static const uint32_t FREQMIN            = 8630000 ;// MHz/100 coded over 24 bits
    static const uint32_t FREQMAX            = 8700000 ;// MHz/100 coded over 24 bits
    static const int      RX2DR_INIT         = 0;
    uint8_t DistriDataRate[8];
    eStatusLoRaWan RegionMaxPayloadSize         ( uint8_t sizeIn ) ;
    void RegionSetDataRateDistribution          ( uint8_t adrMode );
    void RegionLoadFromFlash                    ( void );
    void RegionSetBadCrcInFlash                 ( void );
    virtual void RegionGiveNextDataRate         ( void );
    virtual void RegionSaveInFlash              ( void );
protected : 
    virtual void RegionGetCFList                        ( void );
    virtual void RegionGiveNextChannel                  ( void ); 
    virtual void RegionSetRxConfig                      ( eRxWinType type );
    virtual void RegionSetPower                         ( uint8_t PowerCmd );
    virtual void RegionSetMask                          ( void ) ;
    virtual void RegionInitChannelMask                  ( void );
    virtual void RegionDecreaseDataRate                 ( void );
    virtual eStatusChannel RegionBuildChannelMask       ( uint8_t ChMaskCntl, uint16_t ChMask );
    virtual eStatusLoRaWan RegionIsValidRx1DrOffset     ( uint8_t Rx1DataRateOffset );
    virtual eStatusLoRaWan RegionIsValidDataRate        ( uint8_t temp );
    virtual eStatusLoRaWan RegionIsAcceptableDataRate   ( uint8_t DataRate );
    virtual eStatusLoRaWan RegionIsValidMacFrequency    ( uint32_t Frequency);
    virtual eStatusLoRaWan RegionIsValidMacRxFrequency  ( uint32_t Frequency);
    virtual eStatusLoRaWan RegionIsValidTxPower         ( uint8_t Power );
    virtual eStatusLoRaWan RegionIsValidChannelIndex    ( uint8_t ChannelIndex);
    virtual uint8_t RegionGetAdrAckLimit                ( void );
    virtual uint8_t RegionGetAdrAckDelay                ( void );
private :
    uint8_t DistriDataRateInit[8];
    void TxDataRateToSfBw                  ( uint8_t dataRate );
    void Rx2DataRateToSfBw                 ( uint8_t dataRate );
    uint16_t UnwrappedChannelMask ;                          // this variable is used for multiple linkadr cmds is region dependant at contruction use template
};
#endif

