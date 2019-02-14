/*

  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : LoraWan Phy Layer objets.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#include "PhyLayer.h"
#include "sx1276.h"
#include "sx1276Regs-LoRa.h"
#include "MacLayer.h"
#include "LoraWanProcess.h"
#include "Define.h"
#include "ApiMcu.h"
#include "UserDefine.h"
#include "utilities.h"
#include "DefineRadioPlaner.h"
#define FileId 3
template class RadioContainer<SX1276>;
template class RadioContainer<SX1272>;
template class RadioContainer<SX126x>;

template <class R> RadioContainer <R>::RadioContainer( RadioPLaner<R> * RadioUser ){
    StateRadioProcess = RADIOSTATE_IDLE;
    TimestampRtcIsr =0;
    TxFrequency = 868100000;
    TxPower = 14;
    TxSf = 7;
    Radio = RadioUser;
    LastItTimeFailsafe = mcu.RtcGetTimeSecond( );
    Rx3Activated        = RX3_NOT_ACTIVATED;
}; 
template <class R> RadioContainer<R>::~RadioContainer( ) {
};


/************************************************************************************************/
/*                      Public  Methods                                                         */
/************************************************************************************************/
 //@note Partionning Public/private not yet finalized
 
template <class R> void RadioContainer <R>::Send(eModulationType TxModulation , uint32_t TxFrequencyMac, uint8_t TxPowerMac, uint8_t TxSfMac, eBandWidth TxBwMac, uint16_t TxPayloadSizeMac ) { //@note could/should be merge with tx config
    Radio->GetMyHookId  ( this, MyHookId );
    TxPayloadSize               = (uint8_t)TxPayloadSizeMac;
    sRadioParam.Frequency       = TxFrequencyMac;
    sRadioParam.Power           = TxPowerMac;
    sRadioParam.Sf              = TxSfMac;
    sRadioParam.Bw              = TxBwMac;
    sRadioParam.Modulation      = TxModulation;
    sRadioParam.CrcMode         = CRC_YES;
    sRadioParam.IqMode          = IQ_NORMAL;
    sRadioParam.HeaderMode      = EXPLICIT_HEADER;
    sRadioParam.PreambuleLength = 8;
    sRadioParam.TimeOutMs       = 0;
    sRadioParam.SyncWord        = 0x34;
    sRadioParam.CodingRate      = CR_4_5;
    STask stask ;
    stask.HookId         = MyHookId;
    if ( (int) (SendTargetTime - mcu.RtcGetTimeMs() ) > 0 ) {
        stask.StartTime      = SendTargetTime;
    } else { 
        stask.StartTime      = mcu.RtcGetTimeMs()+200;
    }
    stask.TaskDuration   = 2000;//@tbd RadioPlaner  timeonair
    stask.State          = TASK_SCHEDULE;
    stask.TaskType       = ( TxModulation == LORA ) ? TX_LORA : TX_FSK;
    if (stask.TaskType ==TX_LORA){
        DEBUG_MSG("TX_LORA");
    } else {
          DEBUG_MSG("TX_FSK");
    }
    Radio->EnqueueTask (stask, TxPhyPayload, &TxPayloadSize, sRadioParam );
    if ( TxModulation == LORA ) {
        InsertTrace    ( __COUNTER__, FileId );
        DEBUG_PRINTF    ( "  TxFrequency = %d, RxSf = %d , RxBw = %d PayloadSize = %d\n", TxFrequencyMac, TxSfMac,TxBwMac, TxPayloadSizeMac) ; 
    } else {
        InsertTrace    ( __COUNTER__, FileId );
        DEBUG_MSG      ("FSK TRANSMISSION \n");
    }
    StateRadioProcess = RADIOSTATE_TXON;
};

template <class R> void RadioContainer <R>::SetRxConfig(uint32_t TimetoRadioPlaner , eModulationType RxModulation ,uint32_t RxFrequencyMac, uint8_t RxSfMac, eBandWidth RxBwMac ,uint32_t RxWindowMs) {
    RxFrequency                 = RxFrequencyMac;
    RxBw                        = RxBwMac;
    RxSf                        = RxSfMac;
    RxMod                       = RxModulation;
    CurrentMod                  = RxModulation;
    sRadioParam.Frequency       = RxFrequencyMac;
    sRadioParam.Sf              = RxSfMac;
    sRadioParam.Bw              = RxBwMac;
    sRadioParam.CrcMode         = CRC_NO;
    sRadioParam.IqMode          = IQ_INVERTED;
    sRadioParam.HeaderMode      = EXPLICIT_HEADER;
    sRadioParam.PreambuleLength = 8;
    sRadioParam.Modulation      = RxModulation;
    sRadioParam.TimeOutMs       = RxWindowMs;
    sRadioParam.Snr             = &RxPhyPayloadSnr;
    sRadioParam.Rssi            = &RxPhyPayloadRssi;
    sRadioParam.SyncWord        = 0x34;
    sRadioParam.CodingRate      = CR_4_5;
    STask stask ;
    stask.HookId         = MyHookId;
    stask.StartTime      = TimetoRadioPlaner;
    stask.TaskDuration   = RxWindowMs;
    stask.State    = TASK_SCHEDULE;
    stask.TaskType = (RxModulation == LORA ) ? RX_LORA : RX_FSK;
    Radio->EnqueueTask (stask, RxPhyPayload, &RxPhyPayloadSize, sRadioParam ); //@tbd RadioPlaner  timeonair
    if ( RxModulation == LORA ) {
        InsertTrace   ( __COUNTER__, FileId );
        DEBUG_PRINTF  ( "  RxFrequency = %d, RxSf = %d , RxBw = %d \n", RxFrequency, RxSf,RxBw );
    } else {
        InsertTrace  ( __COUNTER__, FileId );
        DEBUG_PRINTF ( "  RxFrequency = %d, FSK \n", RxFrequency );
    }
}

template <class R>int RadioContainer<R>::GetRadioState( void ) {
    InsertTrace ( __COUNTER__, FileId );
    return StateRadioProcess;
};


template <class R> uint32_t RadioContainer<R>::GetTxFrequency ( void ) {
    InsertTrace ( __COUNTER__, FileId );
    return( TxFrequency );
};


/************************************************************************************************/
/*                      Private  Methods                                                         */
/************************************************************************************************/
/********************************************************/
/*               Check is valid devaddr                 */
/********************************************************/
template <class R> eValidDevAddr RadioContainer<R>::CheckDevAddr (uint32_t devAddrToTest){

    if ( devAddrToTest == DevAddrIsr ) {
        return VALID_DEV_ADDR_UNICAST;
    }
    if (( devAddrToTest == 	DevAddrClassCG0Isr ) && ( ClassCG0EnableIsr ==CLASS_CG0_ENABLE )){
        return VALID_DEV_ADDR_MULTI_CAST_G0;
    }
    if (( devAddrToTest == 	DevAddrClassCG1Isr ) && ( ClassCG0EnableIsr ==CLASS_CG1_ENABLE )){
        return VALID_DEV_ADDR_MULTI_CAST_G1;
    }
    return(UNVALID_DEV_ADDR);
}


template <class R> int RadioContainer<R>::DumpRxPayloadAndMetadata ( void ) {

   /* check Mtype */
    int status = OKLORAWAN;
    InsertTrace ( __COUNTER__, FileId );	
    DEBUG_PRINTF ("payload size receive = %d, snr = %d , rssi = %d\n", RxPhyPayloadSize,RxPhyPayloadSnr,RxPhyPayloadRssi);
    uint8_t MtypeRxtmp = RxPhyPayload[0] >> 5 ;
    if (( MtypeRxtmp == JOINREQUEST) || ( MtypeRxtmp == UNCONF_DATA_UP ) || ( MtypeRxtmp == CONF_DATA_UP) || ( MtypeRxtmp == REJOIN_REQUEST )) {
        status += ERRORLORAWAN;
        InsertTrace ( __COUNTER__, FileId );
        DEBUG_PRINTF(" BAD Mtype = %d for RX Frame \n", MtypeRxtmp );
    }
    /* check devaddr */
    if ( JoinedStatus == JOINED ){
        uint32_t DevAddrtmp = RxPhyPayload[1] + ( RxPhyPayload[2] << 8 ) + ( RxPhyPayload[3] << 16 )+ ( RxPhyPayload[4] << 24 );
        CurrentDevaddrType = CheckDevAddr ( DevAddrtmp );
        if ( CurrentDevaddrType == UNVALID_DEV_ADDR ) {
            status += ERRORLORAWAN;
            InsertTrace ( __COUNTER__, FileId );
            DEBUG_PRINTF( " BAD DevAddr = %x for RX Frame \n", DevAddrtmp );
        }
        if ( status != OKLORAWAN ) {
            RxPhyPayloadSize = 0;
            InsertTrace ( __COUNTER__, FileId );
        }
    }
    if (status == OKLORAWAN) {
        IsReceiveOnRXC = (StateRadioProcess == RADIOSTATE_RXC) ? RECEIVE_ON_RXC : NOT_RECEIVE_ON_RXC ;
    }
    return (status);
}
