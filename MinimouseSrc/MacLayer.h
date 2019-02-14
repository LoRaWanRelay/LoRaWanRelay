/*

  __  __ _       _                                 
 |  \/  ( _)     ( _)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | ( _) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : LoraWan Mac Layer objets.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin ( SEMTECH)
*/
#include "stdint.h"
#include  "Define.h"
#include "PhyLayer.h"
#include "sx1276.h"
#include "LoraMacDataStoreInFlash.h"
#ifndef MAC_LAYER_H
#define MAC_LAYER_H

#define LORA_MAC_SYNCWORD                           0x34

template <int NBCHANNEL, class R>
class LoraWanContainer { 
public: 

    LoraWanContainer( sLoRaWanKeys LoRaWanKeys, RadioPLaner<R> * RadioUser,uint32_t FlashAdress ); 
    ~LoraWanContainer( );
    static const uint8_t  NUMBER_OF_CHANNEL = NBCHANNEL; 
    void BuildTxLoraFrame                    ( void );
    void BuildJoinLoraFrame                  ( void );
    void EncryptTxFrame                      ( void );
    void ConfigureRadioAndSend               ( void );
    void ConfigureRadioForRx1                ( uint32_t TimetoRadioPlaner );
    void ConfigureRadioForRx2                ( uint32_t TimetoRadioPlaner );
    void ConfigureRadioForRx3                ( uint32_t TimetoRadioPlaner );
    void ConfigureRadioForRxClassC           ( void );
    void ConfigureTimerForRx                 ( eRxWinType type );
    void UpdateMacLayer                      ( void );
    void UpdateJoinProcedure                 ( void );
    uint8_t IsFrameToSend;
    eRxPacketType   DecodeRxFrame            ( void );
    eStatusLoRaWan  ParseManagementPacket    ( void );
    uint8_t NbOfReset;
/* LoraWan Context */ 
/* Only 16 ch mask => ChMaskCntl not used */
/* Duty cycle is not managed */

    /*******************************************/
    /*      Update by Link ADR command         */
    /*******************************************/
    uint8_t      MacTxDataRate;
    uint8_t      MacTxDataRateAdr;
    uint8_t      MacTxPower;
    uint16_t     MacChMask;
    uint8_t      MacNbTrans; 
    uint8_t      MacNbTransCpt;
    /********************************************/
    /*     Update by RxParamaSetupRequest       */
    /********************************************/
    uint8_t      MacRx2DataRate ;
    uint32_t     MacRx2Frequency ; 
    uint8_t      MacRx1DataRateOffset;
    /********************************************/
    /*     Update by NewChannelReq command      */
    /********************************************/
    uint32_t     MacTxFrequency        [NBCHANNEL];
    uint32_t     MacRx1Frequency       [NBCHANNEL];
    uint8_t      MacMinDataRateChannel [NBCHANNEL];
    uint8_t      MacMaxDataRateChannel [NBCHANNEL];
    uint8_t      MacChannelIndexEnabled[NBCHANNEL]; // Contain the index of the activated channel only NbOfActiveChannel value are valid

    
    /********************************************/
    /*   Update by RXTimingSetupReq command     */
    /********************************************/
    int          MacRx1Delay;
    /********************************************/
    /*   Other Data To store                    */
    /********************************************/
    uint32_t     FcntUp;
    uint32_t     FcntDwn;  // Wrapping 16 to 32 bits is managed in AcceptFcntDwn Method
    uint32_t     DevAddr;
    uint8_t      nwkSKey [16];
    uint8_t      appSKey [16];
    uint8_t      appKey  [16];
    uint8_t      devEui  [8];
    uint8_t      appEui  [8];
    bool         otaDevice ;
    /*******************************************/
    /* what about keys: AppEUI:Nwskey:AppSkey  */
    /*******************************************/

/* LoRaWan Mac Data for uplink*/
    uint8_t      fPort;
    uint8_t      MType;
    uint8_t      MajorBits;
    uint8_t      Fctrl;
    uint8_t      AckBitForTx;
    uint8_t      UserPayloadSize;
    uint8_t      MacPayloadSize;
    uint8_t      FoptsTxLength;
    uint8_t      FoptsTxData[15];
    uint8_t      FoptsTxLengthSticky;
    uint8_t      FoptsTxDataSticky[15];
    uint8_t      FoptsTxLengthCurrent;
    uint8_t      FoptsTxDataCurrent[15];
/* LoRaWan Mac Data for downlink*/
    uint8_t      fRxPort;
    uint8_t      MtypeRx;
    uint8_t      MajorRx;
    uint8_t      FctrlRx;
    uint8_t      FoptsLength;
    uint8_t      Fopts[16];
    uint8_t      FportRx;
    uint8_t      MacRxPayloadSize;  //@note Have to by replace by a fifo objet to manage class c
    uint8_t      MacRxPayload[255];  //@note Have to by replace by a fifo objet to manage class c
    uint8_t      AvailableRxPacketForUser;

/* LoRaWan Mac Data for join */
    uint16_t     DevNonce;
    uint8_t      CFList[16];
    
/* LoRaWan Mac Data for nwk Ans */
    uint8_t      MacNwkPayload[255];  //@note resize this buffer 
    uint8_t      MacNwkPayloadSize;

    uint8_t      MacNwkAns[255];  //@note reuse user payload data or at least reduce size or use opt byte
    uint8_t      MacNwkAnsSize;

/* LoraWan Config */
    uint8_t      AdrModeSelect;
    int          AdrAckCnt;
    int          AdrAckLimit;
    int          AdrAckDelay;
    uint8_t      AdrAckReq;
    uint8_t      AdrEnable;
   
/* Objet RadioContainer*/
    RadioContainer<R>  Phy;
    

/* Join Duty cycle management */
    uint32_t     RtcNextTimeJoinSecond ;
    uint32_t     RetryJoinCpt;

/*******************************************/
/*    Multicast and Class C Variables      */
/*******************************************/
    eClassCEnable    ClassCG0Enable; 
    eClassCEnable    ClassCG1Enable; 
    uint32_t         DevAddrClassCG0;
    uint32_t         DevAddrClassCG1;
    uint32_t         FcntDwnClassCG0;  // Wrapping 16 to 32 bits is managed in AcceptFcntDwn Method
    uint32_t         FcntDwnClassCG1;  // Wrapping 16 to 32 bits is managed in AcceptFcntDwn Method
    uint8_t          nwkSKeyClassCG0[16];
    uint8_t          appSKeyClassCG0[16];  
    uint8_t          nwkSKeyClassCG1[16];
    uint8_t          appSKeyClassCG1[16];
/***************************************************************/
/*  Virtual Method overwritten by the Class  of the region     */
/***************************************************************/
    virtual void              RegionGiveNextChannel            ( void )                                 = 0; 
    virtual void              RegionSetRxConfig                ( eRxWinType type )                      = 0;
    virtual void              RegionSetPower                   ( uint8_t PowerCmd )                     = 0;
    virtual void              RegionSetMask                    ( void )                                 = 0;
    virtual void              RegionInitChannelMask            ( void )                                 = 0;
    virtual void              RegionGetCFList                  ( void )                                 = 0;
    virtual void              RegionDecreaseDataRate           ( void )                                 = 0;
    virtual void              RegionGiveNextDataRate           ( void )                                 = 0;
    virtual eStatusChannel    RegionBuildChannelMask           ( uint8_t ChMaskCntl, uint16_t ChMaskIn) = 0;
    virtual eStatusLoRaWan    RegionIsValidRx1DrOffset         ( uint8_t Rx1DataRateOffset)             = 0;
    virtual eStatusLoRaWan    RegionIsValidDataRate            ( uint8_t temp )                         = 0;
    virtual eStatusLoRaWan    RegionIsAcceptableDataRate       ( uint8_t DataRate)                      = 0;
    virtual eStatusLoRaWan    RegionIsValidMacFrequency        ( uint32_t Frequency)                    = 0;
    virtual eStatusLoRaWan    RegionIsValidMacRxFrequency      ( uint32_t Frequency)                    = 0;
    virtual eStatusLoRaWan    RegionIsValidTxPower             ( uint8_t Power )                        = 0;
    virtual eStatusLoRaWan    RegionIsValidChannelIndex        ( uint8_t ChannelIndex)                  = 0;
    virtual uint8_t           RegionGetAdrAckLimit             ( void )                                 = 0;
    virtual uint8_t           RegionGetAdrAckDelay             ( void )                                 = 0;
    virtual void              RegionSaveInFlash                ( void )                                 = 0;

    uint32_t    UserFlashAdress;
    void             SetDevAddr                                ( uint32_t address );
/**************************************************************/
/*      Protected Methods and variables                       */
/**************************************************************/
    uint32_t         MacTxFrequencyCurrent;
protected :
    uint8_t          MacTxSfCurrent;
    eModulationType  MacTxModulationCurrent;
    eModulationType  MacRx2ModulationTypeCurrent;
    eBandWidth       MacTxBwCurrent;
    uint32_t         MacRx1FrequencyCurrent;
    uint8_t          MacRx1SfCurrent;
    eBandWidth       MacRx1BwCurrent;
    uint8_t          MacRx2SfCurrent;
    eBandWidth       MacRx2BwCurrent;
    int              FindEnabledChannel ( uint8_t Index );
    void             PrintMacContext ( void ) ;
    eModulationType  MacRx3ModulationTypeCurrent ;
    uint32_t         MacRx3Frequency; 
    uint8_t          MacRx3SfCurrent;
    eBandWidth       MacRx3BwCurrent;
    uint8_t          MacRx3Delay    ;
private :
    static const uint16_t MAX_FCNT_GAP       = 16384 ;
    void SetMacHeader              ( void );
    void SetFrameHeader            ( void );// no opts
    uint8_t GiveNextChannel        ( void );
    int ExtractRxMhdr              ( void );
    int CheckRxPayloadLength       ( void );
    int ExtractRxFhdr              ( uint16_t *FcntDwnTemp, uint32_t devaddr ); 
    int AcceptFcntDwn              ( uint16_t FcntDwnTmp,uint32_t *FcntLoraWan );
    void SetAlarm                  ( uint32_t alarmInMs ,eRxWinType type );
    void LinkCheckParser           ( void );
    void LinkADRParser             ( uint8_t NbMultiLinkAdrReq );
    void DutyCycleParser           ( void );
    void RXParamSetupParser        ( void );
    void DevStatusParser           ( void );
    void NewChannelParser          ( void );
    void RXTimingSetupParser       ( void );
    void DicChannelParser          ( void );
    void UpdateDataRateForAdr      ( void );
    void ComputeRxWindowParameters ( uint8_t SF, eBandWidth BW, uint32_t ClockAccuracy, uint32_t RxDelayMs, uint8_t BoardDelayRxMs );
    sBackUpFlash BackUpFlash;
    uint8_t      NwkPayloadIndex;
    uint8_t      RxEmptyPayload;
    bool         FirstDwn;
    uint16_t     MacRxWindowSymb;
    int32_t      RxOffsetMs;
    uint32_t     MacRxWindowMs;
    uint32_t     RxLateWindowMs;
    eStatusLoRaWan CheckValidMulticastPayload ( void );
}; 


#endif
