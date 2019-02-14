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
#include "MacLayer.h"
#include "LoRaMacCrypto.h"
#include "LoraWanProcess.h"
#include "Define.h"
#include "utilities.h"
#include "PhyLayer.h"
#include "ApiMcu.h"
#include "UserDefine.h"
#include "stdio.h"
#include "math.h"
#define FileId 2
/*************************************************/
/*                     Constructors              */
/*@note have to check init values                */
/*************************************************/
template class LoraWanContainer <16,SX1276>;
template class LoraWanContainer <16,SX126x>;
template class LoraWanContainer <16,SX1272>;
template class LoraWanContainer <72,SX1272>;
template class LoraWanContainer <72,SX1276>;
template class LoraWanContainer <72,SX126x>;
template <int NBCHANNEL, class R> LoraWanContainer<NBCHANNEL, R>::LoraWanContainer(sLoRaWanKeys LoRaWanKeys, RadioPLaner<R>* RadioUser,uint32_t FlashAdress):Phy( RadioUser ) { 
    AvailableRxPacketForUser = NO_LORA_RXPACKET_AVAILABLE;
    memcpy( appSKey, LoRaWanKeys.LoRaMacAppSKey, 16 );
    memcpy( nwkSKey, LoRaWanKeys.LoRaMacNwkSKey, 16 );
    memcpy( appKey, LoRaWanKeys.LoRaMacAppKey, 16 );
    memcpy( devEui, LoRaWanKeys.DevEui, 8 );
    memcpy( appEui, LoRaWanKeys.AppEui, 8 );
    otaDevice                   = LoRaWanKeys.OtaDevice;
    FcntUp                      = 0;
    FcntDwn                     = 0xFFFFFFFF;
    SetDevAddr                  ( LoRaWanKeys.LoRaDevAddr );
    AdrAckCnt                   = 0;
    AdrAckReq                   = 0;
    MacNbTrans                  = 1;
    IsFrameToSend               = NOFRAME_TOSEND;
    RtcNextTimeJoinSecond       = 0;
    RetryJoinCpt                = 0;
    FoptsTxLength               = 0;
    FoptsTxLengthCurrent        = 0;
    FoptsTxLengthSticky         = 0;
    FirstDwn                    = true;
    Phy.JoinedStatus            = ( otaDevice == APB_DEVICE ) ? JOINED : NOT_JOINED;
    UserFlashAdress             = FlashAdress;
    MacNwkPayloadSize           = 0;
    ClassCG0Enable              = CLASS_CG0_ENABLE; 
    ClassCG1Enable              = CLASS_CG1_DISABLE; 
    Phy.ClassCG0EnableIsr       = ClassCG0Enable;
    Phy.ClassCG1EnableIsr       = ClassCG1Enable;
    DevAddrClassCG0             = 0x11223344;
    Phy.DevAddrClassCG0Isr      = DevAddrClassCG0 ;
    DevAddrClassCG1             = 0x0 ;
    Phy.DevAddrClassCG1Isr      = DevAddrClassCG0 ;
    FcntDwnClassCG0             = 0xFFFFFFFF;  
    FcntDwnClassCG1             = 0xFFFFFFFF;  
    MacTxModulationCurrent      = LORA;
    MacRx2ModulationTypeCurrent = LORA;
    MacRx3Delay                 = MAC_RX3_DELAY;
    memcpy( appSKeyClassCG0, LoRaWanKeys.LoRaMacAppSKey, 16 );
    memcpy( nwkSKeyClassCG0, LoRaWanKeys.LoRaMacNwkSKey, 16 );
    memset( nwkSKeyClassCG1, 0 , 16 );
    memset( appSKeyClassCG1, 0 , 16 );
}; 

template <int NBCHANNEL, class R> LoraWanContainer<NBCHANNEL, R>::~LoraWanContainer( ) {
};


/***********************************************************************************************/
/*                      Public  Methods                                                        */
/***********************************************************************************************/

/**********************************************************************/
/*                    Called During LP.Send ()                        */
/**********************************************************************/

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::BuildTxLoraFrame( void ) {

    InsertTrace ( __COUNTER__, FileId );
    if ( FoptsTxLengthCurrent > 15 ) {
        DEBUG_PRINTF ( " ERROR FOPTS TOO LONG =  %d \n", FoptsTxLengthCurrent );
        FoptsTxLengthCurrent = 0;
    }
    Fctrl = 0;  
    Fctrl = ( AdrEnable << 7 ) + ( AdrAckReq << 6 ) + ( AckBitForTx << 5 ) + FoptsTxLengthCurrent;
    AckBitForTx = 0;
    SetMacHeader( );
    SetFrameHeader( );
    MacPayloadSize = UserPayloadSize + FHDROFFSET + FoptsTxLengthCurrent; 
    DEBUG_PRINTF("  Devaddr = %x\n",DevAddr);
};
template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::EncryptTxFrame( void ) {
    LoRaMacPayloadEncrypt( &Phy.TxPhyPayload[FHDROFFSET + FoptsTxLengthCurrent], UserPayloadSize, (fPort == PORTNWK)? nwkSKey :appSKey, DevAddr, UP_LINK, FcntUp, &Phy.TxPhyPayload[FHDROFFSET + FoptsTxLengthCurrent] );
    LoRaMacComputeAndAddMic( &Phy.TxPhyPayload[0], MacPayloadSize, nwkSKey, DevAddr, UP_LINK, FcntUp );
    MacPayloadSize = MacPayloadSize + 4;
};

/**********************************************************************/
/*                      Called During  LP.Process                     */
/**********************************************************************/

/************************************************************************************************************************************/
/*                                              ConfigureRadioAndSend                                                               */
/* Call in case of state = LWPSTATE_SEND                                                                                            */
/************************************************************************************************************************************/
template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::ConfigureRadioAndSend( void ) {
    InsertTrace ( __COUNTER__, FileId );
    //Phy.DevAddrIsr    = DevAddr ;  //@note copy of the mac devaddr in order to filter it in the radio isr routine.
    Phy.Send(MacTxModulationCurrent, MacTxFrequencyCurrent, MacTxPower, MacTxSfCurrent, MacTxBwCurrent, MacPayloadSize);
    AdrAckCnt ++ ; // increment adr counter each uplink frame;
};
/************************************************************************************************************************************/
/*                                              ConfigureRadioForRx1                                                                */
/* Call in case of state = LWPSTATE_SEND & Isr TX end                                                                               */
/************************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::ConfigureRadioForRx1 (  uint32_t TimetoRadioPlaner  ) {
    MacRx3Frequency = MacRx1FrequencyCurrent;

    Phy.SetRxConfig(TimetoRadioPlaner ,MacTxModulationCurrent,MacRx1FrequencyCurrent, MacRx1SfCurrent, MacRx1BwCurrent, MacRxWindowMs);
};
/************************************************************************************************************************************/
/*                                              ConfigureRadioForRx2 +  ConfigureTimerForRx                                         */
/* Call in case of state = LWPSTATE_RX1 & No Receive RX1 Packet                                                                     */
/************************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::ConfigureRadioForRx2 ( uint32_t TimetoRadioPlaner ) {
    InsertTrace ( __COUNTER__, FileId );
    Phy.SetRxConfig(TimetoRadioPlaner, MacRx2ModulationTypeCurrent, MacRx2Frequency, MacRx2SfCurrent, MacRx2BwCurrent, MacRxWindowMs );
};

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::ConfigureRadioForRx3 ( uint32_t TimetoRadioPlaner ) {
    InsertTrace ( __COUNTER__, FileId );
    MacRx3SfCurrent = 7;
    MacRx3BwCurrent = BW125;
    MacRx3ModulationTypeCurrent = LORA;
    Phy.SetRxConfig(TimetoRadioPlaner, MacRx3ModulationTypeCurrent, MacRx3Frequency, MacRx3SfCurrent, MacRx3BwCurrent, MacRxWindowMs );
};



template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::ConfigureRadioForRxClassC ( void ) {
    Phy.SetRxConfig(0,LORA, MacRx2Frequency, MacRx2SfCurrent-5, MacRx2BwCurrent, 10000 ); //@tbd RadioPlaner "0 ""
};

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::ConfigureTimerForRx ( eRxWinType type ) {
    InsertTrace ( __COUNTER__, FileId );
    uint32_t tCurrentMillisec;
    uint32_t tAlarmMillisec;
    tCurrentMillisec =  mcu.RtcGetTimeMs( );
    if (type == RX1) {
        RegionSetRxConfig ( RX1 );
        if (MacTxModulationCurrent == LORA) {
            ComputeRxWindowParameters ( MacRx1SfCurrent, MacRx1BwCurrent, CRYSTAL_ERROR, MacRx1Delay * 1000 , BOARD_DELAY_RX_SETTING_MS );
        } else {
            RxOffsetMs = 50; // tb reduce
        }
        tAlarmMillisec = ( ( MacRx1Delay * 1000 )+ Phy.TimestampRtcIsr )  - tCurrentMillisec  ;
        if ( (int)(tAlarmMillisec - RxOffsetMs) < 0 ) {// too late to launch a timer
            Phy.StateRadioProcess = RADIOSTATE_RX1FINISHED ;
        } else { 
           // SetAlarm( tAlarmMillisec - RxOffsetMs , type );
            RegionSetRxConfig ( RX1 );
            Phy.LastTimeRxWindowsMs =  ( ( MacRx1Delay * 1000 )+ Phy.TimestampRtcIsr ) - RxOffsetMs + MacRxWindowMs ; // timestamp of the end of rx1 windows
            ConfigureRadioForRx1 ( mcu.RtcGetTimeMs () + tAlarmMillisec - RxOffsetMs );
            DEBUG_PRINTF( "  Timer will expire in %d ms\n", ( tAlarmMillisec - RxOffsetMs ) );
        }
    } else if ( type == RX2) {
        RegionSetRxConfig ( RX2 );
        
        if (MacRx2ModulationTypeCurrent == LORA) {
           ComputeRxWindowParameters ( MacRx2SfCurrent, MacRx2BwCurrent, CRYSTAL_ERROR, MacRx1Delay * 1000 + 1000 , BOARD_DELAY_RX_SETTING_MS );
        } else {
            RxOffsetMs = 50; // tb reduce
        }
       
        tAlarmMillisec = ( MacRx1Delay * 1000 ) + 1000 + Phy.TimestampRtcIsr - tCurrentMillisec  ;// @note Rx2 Dalay is alway RX1DELAY + 1 second
        if ( (int)(tAlarmMillisec - RxOffsetMs) < 0 ) {// too late to launch a timer
            Phy.StateRadioProcess = RADIOSTATE_IDLE ;
            DEBUG_PRINTF( " error case negative Timer %d ms\n", tAlarmMillisec );
        } else { 
            //SetAlarm( tAlarmMillisec - RxOffsetMs, type );
            RegionSetRxConfig ( RX2 );
            Phy.LastTimeRxWindowsMs = ( MacRx1Delay * 1000 ) + 1000 + Phy.TimestampRtcIsr - RxOffsetMs + MacRxWindowMs ; // timestamp of the end of rx2 windows
            ConfigureRadioForRx2 ( mcu.RtcGetTimeMs () + tAlarmMillisec - RxOffsetMs );
            DEBUG_PRINTF( "  Timer will expire in %d ms\n", ( tAlarmMillisec - RxOffsetMs ) );
        }
    } else { //Rx3 
        tAlarmMillisec = ( MacRx3Delay * 1000 )  + Phy.TimestampRtcIsr - tCurrentMillisec  ;// @note Rx2 Dalay is alway RX1DELAY + 1 second
        ComputeRxWindowParameters ( MacRx3SfCurrent, MacRx3BwCurrent, CRYSTAL_ERROR, MacRx3Delay * 1000 , BOARD_DELAY_RX_SETTING_MS );
        ConfigureRadioForRx3 ( mcu.RtcGetTimeMs () + tAlarmMillisec - RxOffsetMs );
        DEBUG_PRINTF( "  Timer will expire in %d ms\n", ( tAlarmMillisec - RxOffsetMs ) );
    }
   
}
/************************************************************************************************************************************/
/*                                              DecodeRxFRame                                                                       */
/* Call in case of state = LWPSTATE_PROCESSDOWNLINK                                                                                 */
/************************************************************************************************************************************/

template <int NBCHANNEL, class R> eRxPacketType LoraWanContainer<NBCHANNEL, R>::DecodeRxFrame( void ) {

    int status = OKLORAWAN ;
    eRxPacketType RxPacketType = NO_MORE_VALID_RX_PACKET ; 
    uint32_t micIn ;
    status += CheckRxPayloadLength ( );
    status += ExtractRxMhdr ( ) ;

        /************************************************************************/
        /*                 Case : the receive packet is a Multicast Packet      */
        /************************************************************************/
    if (  Phy.CurrentDevaddrType == VALID_DEV_ADDR_MULTI_CAST_G0) {

        int status = OKLORAWAN ;
        status += CheckValidMulticastPayload( ) ;
        uint16_t FcntDownTmp = 0;
        status += ExtractRxFhdr ( &FcntDownTmp,DevAddrClassCG0) ;
        if ( status == OKLORAWAN) {
            status = AcceptFcntDwn ( FcntDownTmp, &FcntDwnClassCG0 ) ;
        }
        if ( status == OKLORAWAN) {
            MacRxPayloadSize = Phy.RxPhyPayloadSize - MICSIZE ;
            memcpy((uint8_t *)&micIn, &Phy.RxPhyPayload[MacRxPayloadSize], MICSIZE);
            status += LoRaMacCheckMic(&Phy.RxPhyPayload[0], MacRxPayloadSize, nwkSKeyClassCG0, DevAddrClassCG0, FcntDownTmp, micIn ); // @note api discussion see at the end of this file
        }
        if ( status == OKLORAWAN) {
            status += CheckValidMulticastPayload ( );
        }
        if ( status == OKLORAWAN) {     
            AdrAckCnt           = 0 ;  // reset adr counter , receive a valid frame.
            MacRxPayloadSize = MacRxPayloadSize - FHDROFFSET ;
            LoRaMacPayloadDecrypt( &Phy.RxPhyPayload[FHDROFFSET], MacRxPayloadSize, appSKeyClassCG0, DevAddrClassCG0, 1, FcntDwnClassCG0, &MacRxPayload[0] );
            AvailableRxPacketForUser = MULTI_CAST_G0_RX_PACKET_AVAILABLE ; 
            RxPacketType = USER_RX_PACKET; 
        }
        DEBUG_PRINTF(" RxPacketType = %d \n", RxPacketType );
        return ( RxPacketType );
    }
    if (  Phy.CurrentDevaddrType == VALID_DEV_ADDR_MULTI_CAST_G1) {
        int status = OKLORAWAN ;
        status += CheckValidMulticastPayload( ) ;
        uint16_t FcntDownTmp = 0;
        status += ExtractRxFhdr ( &FcntDownTmp,DevAddrClassCG1) ;
        if ( status == OKLORAWAN) {
            status = AcceptFcntDwn ( FcntDownTmp, &FcntDwnClassCG1 ) ;
        }
        if ( status == OKLORAWAN) {
            MacRxPayloadSize = Phy.RxPhyPayloadSize - MICSIZE ;
            memcpy((uint8_t *)&micIn, &Phy.RxPhyPayload[MacRxPayloadSize], MICSIZE);
            status += LoRaMacCheckMic(&Phy.RxPhyPayload[0], MacRxPayloadSize, nwkSKeyClassCG1, DevAddrClassCG1, FcntDownTmp, micIn ); // @note api discussion see at the end of this file
        }
        if ( status == OKLORAWAN) {
            status += CheckValidMulticastPayload ( );
        }
        if ( status == OKLORAWAN) {     
            AdrAckCnt           = 0 ;  // reset adr counter , receive a valid frame.
            MacRxPayloadSize = MacRxPayloadSize - FHDROFFSET ;
            LoRaMacPayloadDecrypt( &Phy.RxPhyPayload[FHDROFFSET], MacRxPayloadSize, appSKeyClassCG1, DevAddrClassCG1, 1, FcntDwnClassCG1, &MacRxPayload[0] );
            AvailableRxPacketForUser = MULTI_CAST_G1_RX_PACKET_AVAILABLE ; 
            RxPacketType = USER_RX_PACKET; 
        }
        DEBUG_PRINTF(" RxPacketType = %d \n", RxPacketType );
        return ( RxPacketType );
    }

        /************************************************************************/
        /*                 Case : the receive packet is a JoinResponse          */
        /************************************************************************/
    if ( MtypeRx == JOINACCEPT ) {
        InsertTrace ( __COUNTER__, FileId );
        LoRaMacJoinDecrypt( &Phy.RxPhyPayload[1], Phy.RxPhyPayloadSize-1, appKey, &MacRxPayload[1] );
        MacRxPayload[0] =  Phy.RxPhyPayload[0];
        MacRxPayloadSize = Phy.RxPhyPayloadSize - MICSIZE ;
        memcpy((uint8_t *)&micIn, &MacRxPayload[MacRxPayloadSize], MICSIZE);
        status += LoRaMacCheckJoinMic( MacRxPayload, MacRxPayloadSize, appKey, micIn);
        if ( status == OKLORAWAN) {
            return JOIN_ACCEPT_PACKET;
        }
    } else {
        InsertTrace ( __COUNTER__, FileId );
        /************************************************************************/
        /*               Case : the receive packet is not a JoinResponse        */
        /************************************************************************/
        uint16_t FcntDownTmp = 0;
        status += ExtractRxFhdr ( &FcntDownTmp, DevAddr) ;
        if ( status == OKLORAWAN) {
            status = AcceptFcntDwn ( FcntDownTmp, &FcntDwn ) ;
        }
        if ( status == OKLORAWAN) {
            MacRxPayloadSize = Phy.RxPhyPayloadSize - MICSIZE ;
            memcpy((uint8_t *)&micIn, &Phy.RxPhyPayload[MacRxPayloadSize], MICSIZE);
            status += LoRaMacCheckMic(&Phy.RxPhyPayload[0], MacRxPayloadSize, nwkSKey, DevAddr, FcntDownTmp, micIn ); // @note api discussion see at the end of this file
        }
        if ( status == OKLORAWAN) {     
            AdrAckCnt           = 0 ; // reset adr counter , receive a valid frame.
            if ( Phy.IsReceiveOnRXC == NOT_RECEIVE_ON_RXC ) {
                MacNbTransCpt       = 1 ; // reset retransmission counter if received on RX1 or RX2
                FoptsTxLengthSticky = 0 ; // reset the fopts of the sticky cmd receive a valide frame if received on RX1 or RX2
            }
            MacRxPayloadSize = ( RxEmptyPayload == 0 )? MacRxPayloadSize - FHDROFFSET - FoptsLength : 0;
            if ( RxEmptyPayload == 0 ) {
                if ( FportRx == 0 ) {
                    if (FoptsLength == 0) {
                        LoRaMacPayloadDecrypt( &Phy.RxPhyPayload[FHDROFFSET + FoptsLength], MacRxPayloadSize, nwkSKey, DevAddr, 1, FcntDwn, &MacNwkPayload[0] );
                        MacNwkPayloadSize = MacRxPayloadSize;
                        RxPacketType = NWKRXPACKET ;
                    }
                } else {
                    LoRaMacPayloadDecrypt( &Phy.RxPhyPayload[FHDROFFSET + FoptsLength], MacRxPayloadSize, appSKey, DevAddr, 1, FcntDwn, &MacRxPayload[0] );
                    if ( FoptsLength != 0 ) {
                        memcpy ( MacNwkPayload, Fopts, FoptsLength);
                        MacNwkPayloadSize = FoptsLength;
                        RxPacketType = USERRX_FOPTSPACKET ;
                    } 
                    if ( MacRxPayloadSize > 0 ) {
                        AvailableRxPacketForUser = LORA_RX_PACKET_AVAILABLE; 
                    }
                }
            } else {
                if ( FoptsLength != 0 ) {
                        memcpy ( MacNwkPayload, Fopts, FoptsLength);
                        MacNwkPayloadSize = FoptsLength;
                        RxPacketType = USERRX_FOPTSPACKET ;
                    } else {
                        RxPacketType = USER_RX_PACKET;
                    }
                AvailableRxPacketForUser = LORA_RX_PACKET_AVAILABLE; 
            }
        }
    }
    DEBUG_PRINTF(" RxPacketType = %d \n", RxPacketType );
    return ( RxPacketType );
}


/************************************************************************************************************************************/
/*                                              UPdate Mac Layer                                                                    */
/* Call in case of state = LWPSTATE_UPDATEMAC                                                                                       */
/************************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::UpdateMacLayer ( void ) {
    AdrAckLimit = RegionGetAdrAckLimit ( );
    AdrAckDelay = RegionGetAdrAckDelay ( );
    if  ( Phy.JoinedStatus == NOT_JOINED ) {
        RetryJoinCpt ++ ; // reset when join ok
        if ( RetryJoinCpt < MAX_RETRY_JOIN_DUTY_CYCLE_1000 ) {
            RtcNextTimeJoinSecond = mcu.RtcGetTimeSecond( ) + ( ( TIMEONAIR_JOIN_SF7_MS << ( MacTxSfCurrent - 7 ) ) )/10 ; //@note 1/100 duty cycle fix
        } else {
            RtcNextTimeJoinSecond = mcu.RtcGetTimeSecond( ) + ( ( TIMEONAIR_JOIN_SF7_MS << ( MacTxSfCurrent - 7 ) ) )/1 ; //@note 1/1000 duty cycle fix
        }
    }
    if ( ( AdrAckCnt >= AdrAckLimit) &&  ( AdrAckCnt < ( AdrAckLimit + AdrAckDelay ) ) ) {
        AdrAckReq = 1 ;
    }
    if ( AdrAckCnt >= ( AdrAckLimit + AdrAckDelay ) ) {
        RegionDecreaseDataRate ( ) ;
        AdrAckCnt = AdrAckLimit ;
        AdrAckReq = 1 ;
    }
    if ( AdrAckCnt < AdrAckLimit ) {
        AdrAckReq = 0 ;
    }
    if ( MacNbTransCpt <= 1 ) { // could also be set to 1 if receive valid ans
        FcntUp++; 
        MacNbTransCpt = 1 ; // error case shouldn't exist
    } else {
        IsFrameToSend = USRFRAME_TORETRANSMIT ;
        MacNbTransCpt -- ;
    }

    /*Store Context In EEPROM */
    if (( FcntUp % FLASH_UPDATE_PERIOD ) == 0 ){
        RegionSaveInFlash ( );
    }
    if ( FoptsTxLength > FoptsTxLengthSticky ) {
        FoptsTxLengthCurrent = FoptsTxLength;
        memcpy(FoptsTxDataCurrent, FoptsTxData , 15);
        FoptsTxLength = 0;
    } else {
        FoptsTxLengthCurrent = FoptsTxLengthSticky;
        memcpy(FoptsTxDataCurrent, FoptsTxDataSticky , 15);
    }
    switch ( IsFrameToSend ) {
        case NOFRAME_TOSEND :

            break;
        case NWKFRAME_TOSEND :
            RegionGiveNextDataRate ( );
            memcpy( &Phy.TxPhyPayload[FHDROFFSET], MacNwkAns, MacNwkAnsSize );
            UserPayloadSize = MacNwkAnsSize;
            fPort = PORTNWK;
            MType = UNCONF_DATA_UP; //@note Mtype have to be confirm 
            BuildTxLoraFrame( );
            EncryptTxFrame( );
            
            break;
        case USERACK_TOSEND :

            break;
    }
    RegionGiveNextDataRate ( );
}


/************************************************************************************************************************************/
/*                                              NWK MANAGEMENTS Methods                                                             */
/*  Call in case of state = LWPSTATE_UPDATEMAC & Receive either NWK Payload or Fopts                                                */
/************************************************************************************************************************************/


template <int NBCHANNEL, class R> eStatusLoRaWan LoraWanContainer<NBCHANNEL, R>::ParseManagementPacket( void ) {
    uint8_t CmdIdentifier;
    eStatusLoRaWan status = OKLORAWAN ;
    NwkPayloadIndex = 0;
    MacNwkAnsSize = 0;
    FoptsTxLength = 0;
    FoptsTxLengthSticky = 0;
    uint8_t NbMultiLinkAdrReq = 0;
    uint8_t MaxCmdNum = 16 ; //@note security to avoid an infinite While erro 
    while ( ( MacNwkPayloadSize > NwkPayloadIndex ) && (  MaxCmdNum > 0 ) ) { //@note MacNwkPayloadSize and MacNwkPayload[0] are updated in Parser's method
        MaxCmdNum --; 
        if ( MaxCmdNum == 0 ) {
            InsertTrace ( __COUNTER__, FileId );
            return ( ERRORLORAWAN );
        }
        CmdIdentifier = MacNwkPayload[NwkPayloadIndex];
        switch ( CmdIdentifier ) {
            case LINK_CHECK_ANS :  
                InsertTrace ( __COUNTER__, FileId );
                LinkCheckParser( );
                break;
            case LINK_ADR_REQ :
                InsertTrace ( __COUNTER__, FileId );
                NbMultiLinkAdrReq = 0;
            /* extract the number of multiple link adr req specification in LoRAWan1.0.2 */
                while (( MacNwkPayload[NwkPayloadIndex + ( NbMultiLinkAdrReq + 1 ) * LINK_ADR_REQ_SIZE ] == LINK_ADR_REQ ) && ( NwkPayloadIndex + LINK_ADR_REQ_SIZE < MacNwkPayloadSize ) ){
                    NbMultiLinkAdrReq ++;
                    InsertTrace ( __COUNTER__, FileId );
                }
                LinkADRParser( NbMultiLinkAdrReq );
                break;
            case DUTY_CYCLE_REQ :
                InsertTrace ( __COUNTER__, FileId );
                DutyCycleParser( ); //@note send answer but do nothing
                break;
            case RXPARRAM_SETUP_REQ :
                InsertTrace ( __COUNTER__, FileId );
                RXParamSetupParser( );
                RegionSaveInFlash ( );            
                break;
            case DEV_STATUS_REQ :
                InsertTrace ( __COUNTER__, FileId );
                DevStatusParser( ); //@note  Done but margin have no sense tb implemented
                break;
            case NEW_CHANNEL_REQ :
                InsertTrace ( __COUNTER__, FileId );
                NewChannelParser( );
                RegionSaveInFlash ( );
                break;
            case RXTIMING_SETUP_REQ :
                InsertTrace ( __COUNTER__, FileId );
                RXTimingSetupParser( ); 
                RegionSaveInFlash ( );
                break;
            case DIC_CHANNEL_REQ :
                InsertTrace ( __COUNTER__, FileId );
                DicChannelParser ( ); 
                RegionSaveInFlash ( );
                break;
            default: 
                InsertTrace ( __COUNTER__, FileId );
                DEBUG_MSG( " Illegal state in mac layer\n " );
                break;
        }
    }
    return ( status ); 
}

/************************************************************************************************/
/*                    Private NWK MANAGEMENTS Methods                                           */
/************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::LinkCheckParser( void ) {
    DEBUG_PRINTF(" Margin = %d , GwCnt = %d \n", MacNwkPayload[ NwkPayloadIndex + 1], MacNwkPayload[NwkPayloadIndex + 2]);
    NwkPayloadIndex += LINK_CHECK_ANS_SIZE;
    //@NOTE NOT YET IMPLEMENTED
}
/********************************************************************************************************************************/
/*                                               Private NWK MANAGEMENTS : LinkADR                                              */ 
/*  Note : describe multiple adr specification                                                                                  */
/*                                                                                                                              */
/*  Step 1 : Create a "unwrapped channel mask" in case of multiple adr cmd with both Channem Mask and ChannnelMaskCntl          */
/*       2 : Extract from the last adr cmd datarate candidate                                                                   */
/*       3 : Extract from the last adr cmd TxPower candidate                                                                    */       
/*       4 : Extract from the last adr cmd NBRetry candidate                                                                    */   
/*       5 : Check errors cases (described below)                                                                               */
/*       6 : If No error Set new channel mask, txpower,datarate and nbretry                                                     */ 
/*       7 : Compute duplicated LinkAdrAns                                                                                      */
/*                                                                                                                              */
/*  Error cases    1 : Channel Cntl mask RFU for each adr cmd (in case of multiple cmd)                                         */
/*                 2 : Undefined channel ( freq = 0 ) for active bit in the unwrapped channel mask                              */
/*                 3 : Unwrapped channel mask = 0 (none active channel)                                                         */
/*                 4 : For the last adr cmd not valid tx power                                                                  */
/*                 5 : For the last adr cmd not valid datarate ( datarate > dRMax or datarate < dRMin for all active channel )  */
/********************************************************************************************************************************/

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::LinkADRParser( uint8_t NbMultiLinkAdrReq  ) {

    DEBUG_PRINTF ("Cmd LinkADRParser =  %x %x %x %x \n", MacNwkPayload[ NwkPayloadIndex + 1], MacNwkPayload[NwkPayloadIndex + 2], MacNwkPayload[NwkPayloadIndex + 3], MacNwkPayload[NwkPayloadIndex + 4] );
    eStatusLoRaWan status = OKLORAWAN;
    eStatusChannel statusChannel = OKCHANNEL ;
    uint8_t StatusAns = 0x7 ; // initilised for ans answer ok 
    uint8_t ChMAstCntlTemp ; 
    uint16_t ChMaskTemp = 0; 
    uint8_t DataRateTemp;
    uint8_t TxPowerTemp;
    uint8_t NbTransTemp;
    int i ;
    /*Create "Unwrapped" chanel mask */
    RegionInitChannelMask ( );
    for ( i = 0 ; i <= NbMultiLinkAdrReq ; i++ ) {
        DEBUG_PRINTF("MULTIPLE LINK ADR REQ , channel mask = 0x%x\n",ChMaskTemp);
        ChMaskTemp = MacNwkPayload[ NwkPayloadIndex + ( i * LINK_ADR_REQ_SIZE ) + 2 ] + ( MacNwkPayload[ NwkPayloadIndex + ( i * LINK_ADR_REQ_SIZE ) +3 ] << 8 )  ;
        ChMAstCntlTemp = (MacNwkPayload[ NwkPayloadIndex + ( i * LINK_ADR_REQ_SIZE ) + 4] & 0x70 ) >> 4 ;
        statusChannel = RegionBuildChannelMask ( ChMAstCntlTemp, ChMaskTemp ) ; 
        DEBUG_PRINTF("MULTIPLE LINK ADR REQ , channel mask = 0x%x\n",ChMaskTemp);
        if ( statusChannel == ERROR_CHANNEL_CNTL ) { // Test ChannelCNTL not defined
            StatusAns &= 0x6 ;
            DEBUG_MSG("INVALID CHANNEL CNTL \n");
        }                       
    }
    /* Valid global channel mask  */
    if ( statusChannel == ERROR_CHANNEL_MASK ) {   // Test Channelmask enables a not defined channel or Channelmask = 0
        StatusAns &= 0x6 ;
        DEBUG_MSG("INVALID CHANNEL MASK \n");
    }             
    /* At This point global temporary channel mask is built and validated */
    /* Valid the last DataRate */
    DataRateTemp = ( ( MacNwkPayload[ NwkPayloadIndex + ( NbMultiLinkAdrReq * LINK_ADR_REQ_SIZE ) + 1 ] & 0xF0 ) >> 4 );
    status = RegionIsAcceptableDataRate( DataRateTemp );
    if ( status == ERRORLORAWAN ) {   // Test Channelmask enables a not defined channel
        StatusAns &= 0x5 ;
        DEBUG_MSG("INVALID DATARATE \n");
    }    
    
    /* Valid the last TxPower  And Prepare Ans */
    TxPowerTemp = ( MacNwkPayload[ NwkPayloadIndex +  ( NbMultiLinkAdrReq * LINK_ADR_REQ_SIZE ) + 1 ] & 0x0F );
    status = RegionIsValidTxPower( TxPowerTemp );
    if ( status == ERRORLORAWAN ) {   // Test tx power
        StatusAns &= 0x3 ;
        DEBUG_MSG("INVALID TXPOWER \n");
    }    

    NbTransTemp = (MacNwkPayload[ NwkPayloadIndex + ( NbMultiLinkAdrReq * LINK_ADR_REQ_SIZE ) + 4] & 0x0F );
    
    /* Update the mac parameters if case of no error */
    
    if ( StatusAns == 0x7 ) {
        RegionSetMask ( ) ;
        RegionSetPower ( TxPowerTemp );
        MacNbTrans = NbTransTemp ;
        MacTxDataRateAdr = DataRateTemp ;
        DEBUG_PRINTF("MacNbTrans = %d\n",MacNbTrans);
        DEBUG_PRINTF("MacTxDataRateAdr = %d\n",MacTxDataRateAdr);
        DEBUG_PRINTF("MacRx2Frequency = %d\n",MacRx2Frequency);
    }

    
    /* Prepare repeteated Ans*/
    for (i = 0 ; i <= NbMultiLinkAdrReq ; i++){
        FoptsTxData [ FoptsTxLength + ( i * LINK_ADR_ANS_SIZE )] = LINK_ADR_ANS ; // copy Cid
        FoptsTxData [ FoptsTxLength + ( i * LINK_ADR_ANS_SIZE ) + 1 ] = StatusAns ;
    }
    NwkPayloadIndex += ( NbMultiLinkAdrReq + 1 ) * LINK_ADR_REQ_SIZE ;
    FoptsTxLength   +=  ( NbMultiLinkAdrReq + 1 ) * LINK_ADR_ANS_SIZE ;
}

/********************************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS : RXParamSetupParser                                 */
/********************************************************************************************************************************/

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::RXParamSetupParser( void ) {
    DEBUG_PRINTF (" Cmd RXParamSetupParser = %x %x %x %x \n", MacNwkPayload[ NwkPayloadIndex + 1], MacNwkPayload[NwkPayloadIndex + 2], MacNwkPayload[NwkPayloadIndex + 3], MacNwkPayload[NwkPayloadIndex + 4] );
    int status = OKLORAWAN;
    uint8_t StatusAns = 0x7 ; // initilised for ans answer ok 
    uint8_t MacRx1DataRateOffsetTemp;
    uint8_t MacRx2DataRateTemp;
    uint32_t MacRx2FrequencyTemp; 
    /* Valid Rx1DrOffset And Prepare Ans */
    MacRx1DataRateOffsetTemp = ( MacNwkPayload[ NwkPayloadIndex + 1 ] & 0x70 ) >> 4 ;
    status = RegionIsValidRx1DrOffset( MacRx1DataRateOffsetTemp );
        
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x6 ; 
        DEBUG_MSG ("INVALID RX1DROFFSET \n");
    }
    
    /* Valid MacRx2Dr And Prepare Ans */
    status = OKLORAWAN;
    MacRx2DataRateTemp = ( MacNwkPayload[ NwkPayloadIndex + 1 ] & 0x0F );
    status = RegionIsValidDataRate( MacRx2DataRateTemp );
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x5 ; 
        DEBUG_MSG ("INVALID RX2DR \n");
    }
    
    /* Valid MacRx2Frequency And Prepare Ans */
    status = OKLORAWAN;
    MacRx2FrequencyTemp = ( MacNwkPayload[ NwkPayloadIndex + 2 ] ) + ( MacNwkPayload[ NwkPayloadIndex + 3 ] << 8 ) + ( MacNwkPayload[ NwkPayloadIndex + 4 ] << 16 );
    status = RegionIsValidMacRxFrequency ( MacRx2FrequencyTemp ) ;
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x3 ; 
        DEBUG_MSG ("INVALID RX2 FREQUENCY \n");
    }
    
    /* Update the mac parameters if case of no error */
    
    if ( StatusAns == 0x7 ) {
        MacRx1DataRateOffset = MacRx1DataRateOffsetTemp;
        MacRx2DataRate       = MacRx2DataRateTemp;
        MacRx2Frequency      = MacRx2FrequencyTemp * 100;
        DEBUG_PRINTF("MacRx1DataRateOffset = %d\n",MacRx1DataRateOffset);
        DEBUG_PRINTF("MacRx2DataRate = %d\n",MacRx2DataRate);
        DEBUG_PRINTF("MacRx2Frequency = %d\n",MacRx2Frequency);

    }

    /* Prepare Ans*/

    FoptsTxData [ FoptsTxLength ] = RXPARRAM_SETUP_ANS ;
    FoptsTxData [ FoptsTxLength + 1 ] = StatusAns ;
    FoptsTxLength = FoptsTxLength + RXPARRAM_SETUP_ANS_SIZE;
    FoptsTxDataSticky [ FoptsTxLengthSticky ] = RXPARRAM_SETUP_ANS ;
    FoptsTxDataSticky [ FoptsTxLengthSticky + 1 ] = StatusAns ;
    FoptsTxLengthSticky += RXPARRAM_SETUP_ANS_SIZE;
    NwkPayloadIndex += RXPARRAM_SETUP_REQ_SIZE;

}

/********************************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS : DutyCycleParser                                    */
/********************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::DutyCycleParser( void ) {
    DEBUG_PRINTF ("Cmd DutyCycleParser %x \n", MacNwkPayload[ NwkPayloadIndex + 1]);
    //uint8_t DutyCycleTemp = ( MacNwkPayload[ NwkPayloadIndex + 1] & 0xF );//@ note Duty cycle isn't manage
       /* Prepare Ans*/
    FoptsTxData [ FoptsTxLength ] = DUTY_CYCLE_ANS ; // copy Cid
    FoptsTxLength   += DUTY_CYCLE_ANS_SIZE ;
    NwkPayloadIndex += DUTY_CYCLE_REQ_SIZE;
}
/********************************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS : DevStatusParser                                    */
/********************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::DevStatusParser( void ) {
    DEBUG_MSG ( "Receive a dev status req\n");

    FoptsTxData [ FoptsTxLength ] = DEV_STATUS_ANS ; // copy Cid
    FoptsTxData [ FoptsTxLength + 1 ] = 0 ;
    FoptsTxData [ FoptsTxLength + 2 ] = 0 ;
    FoptsTxLength += DEV_STATUS_ANS_SIZE ;
    NwkPayloadIndex += DEV_STATUS_REQ_SIZE;
}
/********************************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS : NewChannelParser                                    */
/********************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::NewChannelParser( void ) {
    DEBUG_PRINTF (" Cmd NewChannelParser = %x %x %x %x %x \n", MacNwkPayload[ NwkPayloadIndex + 1], MacNwkPayload[NwkPayloadIndex + 2], MacNwkPayload[NwkPayloadIndex + 3], MacNwkPayload[NwkPayloadIndex + 4], MacNwkPayload[NwkPayloadIndex + 5]);
    int status = OKLORAWAN;
    uint8_t StatusAns = 0x3 ; // initilised for ans answer ok 
    uint8_t ChannelIndexTemp;
    uint8_t DataRateRangeMaxTemp;
    uint8_t DataRateRangeMinTemp;
    uint32_t FrequencyTemp; 
    /* Valid Channel Index */
    ChannelIndexTemp =  MacNwkPayload[ NwkPayloadIndex + 1 ]  ;
    status = RegionIsValidChannelIndex( ChannelIndexTemp );
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x0 ; 
        DEBUG_MSG ("INVALID CHANNEL INDEX \n");
    }
    /* Valid Frequency  */
    FrequencyTemp = ( MacNwkPayload[ NwkPayloadIndex + 2 ] ) + ( MacNwkPayload[ NwkPayloadIndex + 3 ] << 8 ) + ( MacNwkPayload[ NwkPayloadIndex + 4 ] << 16 );
    status = RegionIsValidMacFrequency ( FrequencyTemp ) ;
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x2 ; 
        DEBUG_MSG ("INVALID FREQUENCY\n");
    }
        /* Valid DRMIN/MAX */
    DataRateRangeMinTemp = MacNwkPayload[ NwkPayloadIndex + 5 ] & 0xF;
    status = RegionIsValidDataRate ( DataRateRangeMinTemp ) ;
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x1 ; 
        DEBUG_MSG ("INVALID DR MIN \n");
    }
    DataRateRangeMaxTemp = ( MacNwkPayload[ NwkPayloadIndex + 5 ] & 0xF0 ) >> 4;
    status = RegionIsValidDataRate ( DataRateRangeMaxTemp ) ;
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x1 ; 
        DEBUG_MSG ("INVALID DR MAX \n");
    }
    if ( DataRateRangeMaxTemp < DataRateRangeMinTemp ) {
        StatusAns &= 0x1 ; 
        DEBUG_MSG ("INVALID DR MAX < DR MIN \n");
    }

    /* Update the mac parameters if case of no error */
    
    if ( StatusAns == 0x3 ) {
        MacTxFrequency  [ ChannelIndexTemp ] = 100 * FrequencyTemp;
        MacRx1Frequency [ ChannelIndexTemp ] = 100 * FrequencyTemp;
        MacMinDataRateChannel [ ChannelIndexTemp ] = DataRateRangeMinTemp;
        MacMaxDataRateChannel [ ChannelIndexTemp ] = DataRateRangeMaxTemp;
        if ( FrequencyTemp == 0 ) {
             MacChannelIndexEnabled[ ChannelIndexTemp ] = CHANNEL_DISABLED;
        } else {
             MacChannelIndexEnabled[ ChannelIndexTemp ] = CHANNEL_ENABLED;
        }
        DEBUG_PRINTF("MacTxFrequency [ %d ] = %d\n", ChannelIndexTemp, MacTxFrequency [ ChannelIndexTemp ]);
        DEBUG_PRINTF("MacMinDataRateChannel [ %d ] = %d\n", ChannelIndexTemp, MacMinDataRateChannel [ ChannelIndexTemp ]);
        DEBUG_PRINTF("MacMaxDataRateChannel [ %d ] = %d\n", ChannelIndexTemp, MacMaxDataRateChannel [ ChannelIndexTemp ]);
    }

    /* Prepare Ans*/
    FoptsTxData [ FoptsTxLength ] = NEW_CHANNEL_ANS ; // copy Cid
    FoptsTxData [ FoptsTxLength + 1 ] = StatusAns ;
    FoptsTxLength += NEW_CHANNEL_ANS_SIZE ;
    NwkPayloadIndex += NEW_CHANNEL_REQ_SIZE;
}
/********************************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS : RXTimingSetupParser                                */
/********************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::RXTimingSetupParser( void ) {
    DEBUG_PRINTF ("Cmd RXTimingSetupParser = %x \n", MacNwkPayload[ NwkPayloadIndex + 1]);
    MacRx1Delay = ( MacNwkPayload[ NwkPayloadIndex + 1] & 0xF );
       /* Prepare Ans*/
    FoptsTxData [ FoptsTxLength ] =  RXTIMING_SETUP_ANS ;
    FoptsTxLength += RXTIMING_SETUP_ANS_SIZE;
    FoptsTxDataSticky [ FoptsTxLengthSticky ] = RXTIMING_SETUP_ANS ;
    FoptsTxLengthSticky += RXTIMING_SETUP_ANS_SIZE;
    NwkPayloadIndex += RXTIMING_SETUP_REQ_SIZE;
}

/********************************************************************************************************************************/
/*                                                 Private NWK MANAGEMENTS : DicChannelParser                                */
/********************************************************************************************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::DicChannelParser( void ) {
    DEBUG_PRINTF ("Cmd DicChannelParser = %x %x %x %x  \n", MacNwkPayload[ NwkPayloadIndex + 1], MacNwkPayload[NwkPayloadIndex + 2], MacNwkPayload[NwkPayloadIndex + 3], MacNwkPayload[NwkPayloadIndex + 4]);
    int status = OKLORAWAN;
    uint8_t StatusAns = 0x3 ; // initilised for ans answer ok 
    uint8_t ChannelIndexTemp;
    uint32_t FrequencyTemp; 
    /* Valid Channel Index */
    ChannelIndexTemp =  MacNwkPayload[ NwkPayloadIndex + 1 ]  ;
    if ( MacTxFrequency[ChannelIndexTemp] == 0 ) {
        StatusAns &= 0x1 ; 
        DEBUG_MSG ("INVALID CHANNEL INDEX \n");
    }
    /* Valid Frequency  */
    FrequencyTemp = ( MacNwkPayload[ NwkPayloadIndex + 2 ] ) + ( MacNwkPayload[ NwkPayloadIndex + 3 ] << 8 ) + ( MacNwkPayload[ NwkPayloadIndex + 4 ] << 16 );
    status = RegionIsValidMacRxFrequency ( FrequencyTemp ) ;
    if (status == ERRORLORAWAN ) {
        StatusAns &= 0x2 ; 
        DEBUG_MSG ("INVALID FREQUENCY\n");
    }
    /* Update the mac parameters if case of no error */
     if ( StatusAns == 0x3 ) {
        MacRx1Frequency [ ChannelIndexTemp ] = 100 * FrequencyTemp;
        DEBUG_PRINTF("MacRxAFrequency [ %d ] = %d\n", ChannelIndexTemp, MacRx1Frequency [ ChannelIndexTemp ]);
     }
    /* Prepare Ans*/
    FoptsTxData [ FoptsTxLength ]    =  DIC_CHANNEL_ANS ;
    FoptsTxData [ FoptsTxLength + 1] =  StatusAns ;
    FoptsTxLength += DIC_CHANNEL_ANS_SIZE;
    FoptsTxDataSticky [ FoptsTxLengthSticky ] = DIC_CHANNEL_ANS ;
    FoptsTxDataSticky [ FoptsTxLengthSticky + 1] =  StatusAns ;
    FoptsTxLengthSticky += DIC_CHANNEL_ANS_SIZE;
    NwkPayloadIndex += DIC_CHANNEL_REQ_SIZE;
}

/********************************************************************************************************************************/
/*                                          Special Case Join OTA                                                               */
/*  Call in case of state = LWPSTATE_UPDATEMAC & Receivea Join Ans                                                              */
/********************************************************************************************************************************/

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::UpdateJoinProcedure ( void ) { //@note tbd add valid test 
   // uint32_t c1 = mcu.RtcGetTimeMs();
  //  DEBUG_PRINTF ("GET Time %d\n",c1 );
    InsertTrace ( __COUNTER__, FileId );
    uint8_t AppNonce[6];
    int i;
    memcpy( AppNonce, &MacRxPayload[1], 6 );
    LoRaMacJoinComputeSKeys(appKey, AppNonce, DevNonce,  nwkSKey, appSKey );
    if ( MacRxPayloadSize > 13 ) { // cflist are presents
        for( i = 0 ; i < 16 ; i++) {
            CFList[i] = MacRxPayload[13 + i];
        }
    RegionGetCFList ( ) ;  
    }
    SetDevAddr( MacRxPayload[7] + ( MacRxPayload[8] << 8 ) + ( MacRxPayload[9] << 16 )+ ( MacRxPayload[10] << 24 ) );
    MacRx1DataRateOffset = ( MacRxPayload[11] & 0x70 ) >> 4;
    MacRx2DataRate       = ( MacRxPayload[11] & 0x0F );
    MacRx1Delay          = MacRxPayload[12];

    Phy.JoinedStatus = JOINED;

    FirstDwn = true;
    FcntDwn = 0xffffffff; 
    FcntUp = 0; 
    RetryJoinCpt = 0;
  
    RegionSaveInFlash ( ); //@Note have to save twice in case of lowpower , have to understand why and remove this workaround
    DEBUG_PRINTF(" DevAddr= %x\n",DevAddr);
    DEBUG_PRINTF(" MacRx1DataRateOffset= %d\n",MacRx1DataRateOffset);
    DEBUG_PRINTF(" MacRx2DataRate= %d\n",MacRx2DataRate);
    DEBUG_PRINTF(" MacRx1Delay= %d\n",MacRx1Delay);
    DEBUG_MSG(" Save In Flash After Join suceed \n");
    // uint32_t c2 = mcu.RtcGetTimeMs();
   // DEBUG_PRINTF ("GET Time %d Join duration =%d\n",c2,c2-c1 );
}

/********************************************************/
/*               Called During LP.Join()                */
/********************************************************/


template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::BuildJoinLoraFrame( void ) {
    InsertTrace ( __COUNTER__, FileId );
    DevNonce += 1;
    MType = JOINREQUEST ;
    SetMacHeader ( );
    for (int i = 0; i <8; i++){ 
        Phy.TxPhyPayload[1+i] = appEui[7-i];
        Phy.TxPhyPayload[9+i] = devEui[7-i];
    }
    Phy.TxPhyPayload[17] = ( uint8_t )( ( DevNonce & 0x00FF ) );
    Phy.TxPhyPayload[18] = ( uint8_t )( ( DevNonce & 0xFF00 ) >> 8 );
    MacPayloadSize = 19 ;
    uint32_t mic ; 
//    FcntUp = 1; 
    LoRaMacJoinComputeMic( &Phy.TxPhyPayload[0], MacPayloadSize, appKey, &mic );
    memcpy(&Phy.TxPhyPayload[MacPayloadSize], (uint8_t *)&mic, 4);
    MacPayloadSize = MacPayloadSize + 4;
}

/************************************************************************************************/
/*                      Private  Methods                                                        */
/************************************************************************************************/

template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::SetMacHeader( void ) {
    Phy.TxPhyPayload[0] = ( ( MType & 0x7 ) <<5 ) + ( MajorBits & 0x3 );
};
template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::SetFrameHeader( ) {

    Phy.TxPhyPayload[1] = ( uint8_t )( ( DevAddr & 0x000000FF ) );
    Phy.TxPhyPayload[2] = ( uint8_t )( ( DevAddr & 0x0000FF00 ) >> 8 );
    Phy.TxPhyPayload[3] = ( uint8_t )( ( DevAddr & 0x00FF0000 ) >> 16 );
    Phy.TxPhyPayload[4] = ( uint8_t )( ( DevAddr & 0xFF000000 ) >> 24 );
    Phy.TxPhyPayload[5] = Fctrl;
    Phy.TxPhyPayload[6] = ( uint8_t )( ( FcntUp & 0x000000FF ) );
    Phy.TxPhyPayload[7] = ( uint8_t )( ( FcntUp & 0x0000FF00 ) >> 8 );
    for (int i = 0 ; i < FoptsTxLengthCurrent ; i ++ ) {
        Phy.TxPhyPayload[ 8 + i ] = FoptsTxDataCurrent [ i ];
    }
    Phy.TxPhyPayload[ 8 + FoptsTxLengthCurrent ] = fPort;
}

template <int NBCHANNEL, class R> int LoraWanContainer<NBCHANNEL, R>::CheckRxPayloadLength ( void ) {
    InsertTrace ( __COUNTER__, FileId );
    int status = OKLORAWAN;
    if ( Phy.RxPhyPayloadSize < MIN_LORAWAN_PAYLOAD_SIZE ) {
        status = ERRORLORAWAN;
        DEBUG_PRINTF ( " ERROR CheckRxPayloadLength = %d \n",Phy.RxPhyPayloadSize);
        return (status);
    }
    return (status);
}

template <int NBCHANNEL, class R> int LoraWanContainer<NBCHANNEL, R>::ExtractRxMhdr ( void ) {
    int status = OKLORAWAN; 
    MtypeRx = Phy.RxPhyPayload[0] >> 5 ;
    MajorRx =  Phy.RxPhyPayload[0] & 0x3 ;
    if (( MtypeRx == JOINREQUEST) || ( MtypeRx == UNCONF_DATA_UP ) || ( MtypeRx == CONF_DATA_UP) || ( MtypeRx == REJOIN_REQUEST )) {
        status = ERRORLORAWAN;
        DEBUG_MSG( " BAD RX MHDR\n " );
    }
    AckBitForTx = ( MtypeRx == CONF_DATA_DOWN ) ? 1 : 0 ;
        
    return (status);
}

template <int NBCHANNEL, class R> int LoraWanContainer<NBCHANNEL, R>::ExtractRxFhdr ( uint16_t *FcntDwnTmp, uint32_t devaddr  ) { //@note Not yet at all finalized have to initiate action on each field
    int status = OKLORAWAN; 
    uint32_t DevAddrtmp = 0 ;
    DevAddrtmp = Phy.RxPhyPayload[1] + ( Phy.RxPhyPayload[2] << 8 ) + ( Phy.RxPhyPayload[3] << 16 )+ ( Phy.RxPhyPayload[4] << 24 );
    status = (DevAddrtmp == devaddr) ? OKLORAWAN : ERRORLORAWAN; 
    FctrlRx = Phy.RxPhyPayload[5] ;
    *FcntDwnTmp = Phy.RxPhyPayload[6] + ( Phy.RxPhyPayload[7] << 8 );
    FoptsLength = FctrlRx & 0x0F;
    memcpy(&Fopts[0], &Phy.RxPhyPayload[8], FoptsLength);
    // case empty payload without fport :
    if ( Phy.RxPhyPayloadSize > 8 + MICSIZE + FoptsLength){
        FportRx = Phy.RxPhyPayload[8+FoptsLength];
        RxEmptyPayload = 0;
    } else {
        RxEmptyPayload = 1;
        DEBUG_MSG( " EMPTY MSG \n" ); 
    }
    /**************************/
    /* manage Fctrl Byte      */
    /**************************/
    if (status == ERRORLORAWAN ) {
        DEBUG_PRINTF(" ERROR Bad DevAddr %x\n ", DevAddrtmp );
    }
    return (status);
}
template <int NBCHANNEL, class R> int LoraWanContainer<NBCHANNEL, R>::AcceptFcntDwn ( uint16_t FcntDwnTmp, uint32_t *FcntLoraWan ) {
    InsertTrace ( __COUNTER__, FileId );
    int status = OKLORAWAN; 
    uint16_t FcntDwnLsb = ( *FcntLoraWan & 0x0000FFFF );
    uint16_t FcntDwnMsb = ( *FcntLoraWan & 0xFFFF0000 ) >> 16;
    if  ( ( FcntDwnTmp > FcntDwnLsb ) || ( *FcntLoraWan == 0xFFFFFFFF) ) {
        *FcntLoraWan = FcntDwnTmp ;
      //  FirstDwn = false ;
    } else if ( ( FcntDwnLsb - FcntDwnTmp ) > MAX_FCNT_GAP )  {
        *FcntLoraWan = ( ( FcntDwnMsb + 1 ) << 16 ) + FcntDwnTmp ;
    } else {
        status = ERRORLORAWAN ;
        DEBUG_PRINTF (" ERROR FcntDwn is not acceptable fcntDwnReceive = %d fcntLoraStack = %d\n",FcntDwnTmp,(*FcntLoraWan));
    }
    return ( status ) ;
}




template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::PrintMacContext ( ) {
    DEBUG_PRINTF ("\n MacTxDataRate = %d ", MacTxDataRate ) ;
    DEBUG_PRINTF ("\n MacTxPower = %d ", MacTxPower ) ;
    DEBUG_PRINTF ("\n MacChMask = 0x%x ", MacChMask ) ;
    DEBUG_PRINTF ("\n MacRx2Frequency = %d ", MacRx2Frequency ) ;
    DEBUG_PRINTF ("\n MacRx2DataRate = %d ", MacRx2DataRate ) ;
    DEBUG_PRINTF ("\n MacRx1DataRateOffset = %d ", MacRx1DataRateOffset ) ;
    DEBUG_PRINTF ("\n MacRx1Delay = %d ", MacRx1Delay ) ;
    DEBUG_PRINTF ("\n FcntUp = %d ", FcntUp ) ;
    DEBUG_PRINTF ("\n FcntDwn = %d ", FcntDwn ) ;
    DEBUG_PRINTF ("\n DevAddr = 0x%x ", DevAddr ) ;
    DEBUG_PRINTF ("\n JoinedStatus = %d ",Phy.JoinedStatus  ) ;
    for (int i = 0 ; i < NUMBER_OF_CHANNEL ; i ++ ) {
        DEBUG_PRINTF ("\n MacTxFrequency[%d]= %d ", i, MacTxFrequency[i] ) ;
        DEBUG_PRINTF ("\n MacRx1Frequency[%d]= %d ", i, MacRx1Frequency[i] ) ;
        DEBUG_PRINTF ("\n MacMaxDataRateChannel[%d]   = %d ", i, MacMaxDataRateChannel[i] ) ;
        DEBUG_PRINTF ("\n MacMinDataRateChannel[%d]   = %d ", i, MacMinDataRateChannel[i] ) ;
        DEBUG_PRINTF ("\n MacChannelIndexEnabled[%d]  = %d \n", i, MacChannelIndexEnabled[i] );
    }
}



/*********************************************************************************/
/*                           Protected Methods                                   */
/*********************************************************************************/
template <int NBCHANNEL, class R> int  LoraWanContainer<NBCHANNEL, R>::FindEnabledChannel( uint8_t Index) {
    InsertTrace ( __COUNTER__, FileId );
    int i = 0;
    int cpt = 0;
    for ( i = 0 ; i < NUMBER_OF_CHANNEL; i ++ ) {
        if ( MacChannelIndexEnabled [ i ] == CHANNEL_ENABLED ) {
            cpt ++;
        }        
        if (cpt == ( Index + 1 ) ) {
            return ( i );
        }
    }
    return (-1) ; // for error case 
};
template <int NBCHANNEL, class R> void  LoraWanContainer<NBCHANNEL, R>::ComputeRxWindowParameters( uint8_t SF, eBandWidth BW, uint32_t ClockAccuracy, uint32_t RxDelayMs, uint8_t BoardDelayRxMs) {
    // ClockAccuracy is set in Define.h, it is board dependent. It must be equal to error in per thousand
    InsertTrace ( __COUNTER__, FileId );  
    uint32_t RxErrorMs= ( ClockAccuracy * RxDelayMs ) / 1000; // for example with an clockaccuracy = 30 (3%)  and a rx windows set to 5s => rxerror = 150 ms 
    if ( MacTxModulationCurrent == LORA) {
        int bwTemp = 125* ( BW + 1 );
        double tSymbol = (double) (1<<SF) / (double) bwTemp;        
        Phy.SymbolDuration = (uint32_t) tSymbol ;
        uint8_t minRxSymbols = 6;
        MacRxWindowSymb = (uint16_t) (MAX( ( 2 * minRxSymbols - 8 ) + (2 * RxErrorMs * bwTemp >> SF) + 1 , minRxSymbols ));
        RxOffsetMs = ( int32_t )((ceil( ( 4.0 * tSymbol ) - ( ( MacRxWindowSymb * tSymbol ) / 2.0 ) - BoardDelayRxMs ))*(-1));
        MacRxWindowMs = MacRxWindowSymb * tSymbol ;
      //  DEBUG_PRINTF ( "Rx win = %d delay = %d nb symbol %d\n",MacRxWindowMs,RxOffsetMs,MacRxWindowSymb);
    } else { //FSK
        RxOffsetMs = 40;//BoardDelayRxMs + 25 + ( ( BoardDelayRxMs * ClockAccuracy ) / 1000 );
    }
};

template <int NBCHANNEL, class R> eStatusLoRaWan  LoraWanContainer<NBCHANNEL, R>::CheckValidMulticastPayload( void ){
    eStatusLoRaWan status = OKLORAWAN; 
    uint8_t MtypeRxTmp = Phy.RxPhyPayload[0] >> 5 ;
    if ( MtypeRxTmp != UNCONF_DATA_DOWN)  {
        DEBUG_MSG( " ERROR : BAD Mtype for Multicast downlink\n " );
        status = ERRORLORAWAN;
        return ( status );
    }
    uint8_t FctrlRxTmp = Phy.RxPhyPayload[5] ;
    uint8_t FoptsLengthTmp = FctrlRxTmp & 0x0F;
    if ( FoptsLengthTmp != 0)  {
        DEBUG_MSG( " ERROR : Fopts field not equal to 0 for a Multicast downlink\n " );
        status = ERRORLORAWAN;
        return ( status );
    }
    uint8_t AckBitDwnTmp = (FctrlRxTmp & 0x20);
    if ( AckBitDwnTmp == 0x20)  {
        DEBUG_MSG( " ERROR : Ack Bit equal to 1 for a Multicast downlink\n " );
        status = ERRORLORAWAN;
        return ( status );
    }
    uint8_t FportRxTmp = Phy.RxPhyPayload[8];
    if ( FportRxTmp == 0)  {
        DEBUG_MSG( " ERROR : Fport equal to 0 for a Multicast downlink\n " );
        status = ERRORLORAWAN;
        return ( status );
    }
    return (status);
};
template <int NBCHANNEL, class R> void LoraWanContainer<NBCHANNEL, R>::SetDevAddr( uint32_t address ){
    DevAddr        = address;
    Phy.DevAddrIsr = address ; 
}
