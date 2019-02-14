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
#include "sx1276.h"
#include "sx1272.h"
#include "SX126x.h"
#include "Define.h"
#include "RadioPlaner.h"
#ifndef PHY_LAYER_H
#define PHY_LAYER_H



template < class R >
class RadioContainer { 
public: 
    RadioContainer( RadioPLaner<R> * RadioUser  ); 
    ~RadioContainer( );
    RadioPLaner<R>* Radio;
    void Send              (eModulationType MacTxModulationCurrent, uint32_t TxFrequencyMac, uint8_t TxPowerMac, uint8_t TxSfMac, eBandWidth TxBwMac, uint16_t TxPayloadSizeMac );
    void Receive           ( void );
    void IsrRadio          ( void ); // Isr routine implemented in IsrRoutine.cpp file
    static void CallbackIsrRadio (void * obj){(reinterpret_cast<RadioContainer< R >*>(obj))->IsrRadio();} ;
    int GetRadioState      ( void );
    void SetRxConfig       ( uint32_t TimetoRadioPlaner , eModulationType RxModulation, uint32_t RxFrequencyMac, uint8_t RxSfMac, eBandWidth RxBwMac , uint32_t RxWindowMs);
    uint32_t               GetTxFrequency ( void );
    uint8_t                TxPhyPayload[MAX_TX_PAYLOAD_SIZE]; 
    uint8_t                RxPhyPayload[MAX_TX_PAYLOAD_SIZE]; 
    uint8_t                RxPhyPayloadSize;
    int16_t                RxPhyPayloadSnr;
    int16_t                RxPhyPayloadRssi;
    uint8_t                TxPayloadSize;
    uint32_t               DevAddrIsr ; // a copy of the devaddr to be tested in the isr routine + devaddrclassCG0 & devaddrclasccG1
    eClassCEnable          ClassCG0EnableIsr; 
    eClassCEnable          ClassCG1EnableIsr; 
    uint32_t               DevAddrClassCG0Isr;
    uint32_t               DevAddrClassCG1Isr;
    ePlanerStatus          PlanerStatus;
    eJoinStatus            JoinedStatus; // used in isr routine to not filter on devaddr
    int                    StateRadioProcess;
    uint32_t               TimestampRtcIsr;
    uint32_t               LastTimeRxWindowsMs;
    uint32_t               SymbolDuration;
    uint32_t               LastItTimeFailsafe;
    eValidDevAddr          CurrentDevaddrType;
    uint32_t               SendTargetTime; // Not equal to Zero in case of send At Time  
    /**********for class c************/
    eIsReceiveOnRXC        IsReceiveOnRXC;
    eDeviceTypeRx3         Rx3Activated;
private :
    uint8_t              MyHookId;
    SRadioParam          sRadioParam; 
    uint32_t             RxFrequency;
    eBandWidth           RxBw;
    uint8_t              RxSf;
    eModulationType      RxMod;
    eModulationType      CurrentMod;
    uint32_t             TxFrequency;
    uint8_t              TxPower;
    uint8_t              TxSf;
    eBandWidth           TxBw;


    
    int                  DumpRxPayloadAndMetadata ( void );
    eValidDevAddr        CheckDevAddr (uint32_t devAddrToTest); 
};
#endif
