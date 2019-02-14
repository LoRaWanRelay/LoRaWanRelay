
/*
  _____      _             
|  __ \    | |            
| |__) |___| | __ _ _   _ 
|  _  // _ \ |/ _` | | | |
| | \ \  __/ | (_| | |_| |
|_|  \_\___|_|\__,_|\__, |
                     __/ |
                    |___/ 

                                                   
Description       : Relay objets.  

License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin  (SEMTECH)

*/
#include "stdint.h"
#include "Define.h"
#include "DefineRadioPlaner.h"
#ifndef RELAY__H
#define RELAY__H

#define NB_NODE_IN_RELAY  16

typedef enum BoolYesNo {
    YES,
    NO,
} BoolYesNo;

typedef enum BoolOkKo {
    OK,
    KO,
} BoolOkKo;

typedef enum Rx3Activation {
    RX3_ENABLE,
    RX3_DISABLE,
} Rx3Activation;

class Relay  { 
public:
    Relay( ){
        for (int i = 0 ; i <NB_NODE_IN_RELAY; i ++ ) {
            WhiteList[i].Devaddr= 0xFFFFFFFF;    
            BlackList[i].Devaddr= 0xFFFFFFFF;
            WhiteList[i].CptWakeUpSequence = 0;
            WhiteList[i].LastFragNumbnerWakeUpSequence = 0;
            BlackList[i].CptWakeUpSequence = 0;
            BlackList[i].LastFragNumbnerWakeUpSequence = 0;
            JoinBlackList[i].CptWakeUpSequence = 0;
            JoinBlackList[i].ActiveNode = false;
            JoinWhiteList[i].CptWakeUpSequence = 0;
            JoinWhiteList[i].ActiveNode = false;
            memset (JoinWhiteList[i].DevEui,0xFF,8);
            memset (JoinBlackList[i].DevEui,0xFF,8);    
        }   
    };
    ~Relay ( ) {}; 
    uint32_t RelayExtractDevaddr ( uint8_t *data) {
        return ( data[3] + ( data[4] << 8) + ( data[5] << 16) +  ( data[6] << 24) );
    };
    uint32_t ExtractDevaddrDownLink ( uint8_t *data) {
        return ( data[1] + ( data[2] << 8) + ( data[3] << 16) +  ( data[4] << 24) );
    };
    BoolYesNo IsWhiteListedDevaddr (uint32_t devaddr) {
        BoolYesNo status = NO;
        for (int i = 0 ; i < NB_NODE_IN_RELAY ; i ++ ) {
            if ( WhiteList[i].Devaddr == devaddr ) {
                return (YES);
            }
        }
        return (status);
    };
    BoolYesNo IsJoinWhiteListedDevEui (uint8_t devEuiIn[8] ) {
        BoolYesNo status = NO;
        for (int i = 0 ; i < NB_NODE_IN_RELAY ; i ++ ) {
            if ( memcmp ( JoinWhiteList[i].DevEui ,devEuiIn , 8 ) == 0 ) {
                return (YES);
            }
        }
        return (status);
    };


    BoolOkKo RemoveDevaddrInBlackList (uint32_t devaddr){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( BlackList[i].Devaddr == devaddr) {
                BlackList[i].Devaddr = 0xFFFFFFFF;
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo RemoveDevEuiInJoinBlackList (uint8_t devEuiIn[8]){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp ( JoinBlackList[i].DevEui ,devEuiIn , 8 ) == 0 ) {
                memset(JoinBlackList[i].DevEui,0xFF,8);
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo AddDevaddrInBlackList (uint32_t devaddr){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( BlackList[i].Devaddr == devaddr) {
                return (KO);
            }
        }
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( BlackList[i].Devaddr == 0xFFFFFFFF) {
                BlackList[i].Devaddr = devaddr;
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo AddDevEuiInJoinBlackList (uint8_t devEuiIn[8] ){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp ( JoinBlackList[i].DevEui ,devEuiIn , 8 ) == 0 ) {
                return (KO);
            }
        }
        uint8_t VectTemp[8];
        memset ( VectTemp, 0xFF, 8);
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (JoinBlackList[i].DevEui , VectTemp, 8 ) == 0 ) {
                memcpy ( JoinBlackList[i].DevEui, devEuiIn , 8 ) ;
                return (OK);
            }
        }
        return (KO);
    };

    BoolOkKo RemoveDevaddrInWhiteList (uint32_t devaddr){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == devaddr) {
                WhiteList[i].Devaddr = 0xFFFFFFFF;
                AddDevaddrInBlackList (devaddr);
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo RemoveDevEuiInJoinWhiteList (uint8_t devEuiIn[8] ){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp ( JoinWhiteList[i].DevEui ,devEuiIn , 8 ) == 0 ) {
                memset(JoinWhiteList[i].DevEui,0xFF,8);
                AddDevEuiInJoinBlackList (devEuiIn);
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo AddDevaddrInWhiteList (uint32_t devaddr){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == devaddr) {
                return (KO);
            }
        }
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == 0xFFFFFFFF) {
                WhiteList[i].Devaddr = devaddr;
                RemoveDevaddrInBlackList (devaddr);
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo AddDevEuiInJoinWhiteList (uint8_t devEuiIn[8]){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (devEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ) {
                return (KO);
            }
        }
        uint8_t VectTemp[8];
        memset ( VectTemp, 0xFF, 8);
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (JoinWhiteList[i].DevEui , VectTemp, 8 ) == 0 ) {
                memcpy ( JoinWhiteList[i].DevEui, devEuiIn , 8 ) ;
                return (OK);
            }
        }
        return (KO);
    };
    BoolOkKo SetRssiStatus (uint32_t DevaddrIn , uint8_t Rssi ){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == DevaddrIn) {
                WhiteList[i].Rssi = Rssi;
                return (OK);
            }
            if ( BlackList[i].Devaddr == DevaddrIn) {
                BlackList[i].Rssi = Rssi;
                return (OK);
            }
        }
        return (KO);
    }
    BoolOkKo SetRssiStatus (uint8_t devEuiIn[8] , uint8_t Rssi ){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (devEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ) {
                JoinWhiteList[i].Rssi = Rssi;
                return (OK);
            }
            if ( memcmp (devEuiIn , JoinBlackList[i].DevEui, 8 ) == 0 ) {
                JoinBlackList[i].Rssi = Rssi;
                return (OK);
            }
        }
        return (KO);
    }
    void buildStatus( uint8_t * Payload, uint8_t *Size ) {
        int index = 1;
        uint8_t NbElementWhiteList = 0;
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr != 0xFFFFFFFF) {
                Payload[index]   = ( WhiteList[i].Devaddr >> 24 ) & 0xFF;
                Payload[index+1] = ( WhiteList[i].Devaddr >> 16 ) & 0xFF;
                Payload[index+2] = ( WhiteList[i].Devaddr >> 8 ) & 0xFF;
                Payload[index+3] = ( WhiteList[i].Devaddr >> 0 ) & 0xFF;
                Payload[index+4] =  WhiteList[i].Rssi;
                Payload[index+5] = ( WhiteList[i].CptWakeUpSequence >> 8 ) & 0xFF;
                Payload[index+6] = ( WhiteList[i].CptWakeUpSequence      ) & 0xFF;
                Payload[index+7] = WhiteList[i].LastFragNumbnerWakeUpSequence ;
                index = index + 8;
                NbElementWhiteList++;
            }
        }
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( BlackList[i].Devaddr != 0xFFFFFFFF) {
                Payload[index]   = ( BlackList[i].Devaddr >> 24 ) & 0xFF;
                Payload[index+1] = ( BlackList[i].Devaddr >> 16 ) & 0xFF;
                Payload[index+2] = ( BlackList[i].Devaddr >> 8 ) & 0xFF;
                Payload[index+3] = ( BlackList[i].Devaddr >> 0 ) & 0xFF;
                Payload[index+4] =  BlackList[i].Rssi;
                Payload[index+5] = ( BlackList[i].CptWakeUpSequence >> 8 ) & 0xFF;
                Payload[index+6] = ( BlackList[i].CptWakeUpSequence      ) & 0xFF;
                Payload[index+7] = BlackList[i].LastFragNumbnerWakeUpSequence ;
                index = index + 8;
            }
        }
        *Size = index;
        Payload[0] = NbElementWhiteList;
    };
     void buildJoinStatus( uint8_t * Payload, uint8_t *Size, SStatisticRP PowerStat ) {
        memcpy (&Payload[0],(uint8_t *)(&(PowerStat.TotalTxConsumptionMs)),4);
        memcpy (&Payload[4],(uint8_t *)(&(PowerStat.TotalRxConsumptionMs)),4);
        memcpy (&Payload[8],(uint8_t *)(&(mcu.PowerConsumptionTotal)),4);
        int index = 13;
        uint8_t NbElementJoinWhiteList = 0;
        uint8_t VectTemp[8];
        memset ( VectTemp, 0xFF, 8);
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp ( JoinWhiteList[i].DevEui , VectTemp, 8 ) != 0 ) {
                memcpy ( &Payload[index], JoinWhiteList[i].DevEui, 8 );    
                Payload[index+8] = JoinWhiteList[i].Rssi;;
                Payload[index+9]  = ( JoinWhiteList[i].CptWakeUpSequence >> 8 ) & 0xFF;
                Payload[index+10] = ( JoinWhiteList[i].CptWakeUpSequence      ) & 0xFF;
                if (JoinWhiteList[i].ActiveNode == true) {
                    Payload[index+11] = 0 ;
                    JoinWhiteList[i].LastTimeReceiveSecond = mcu.RtcGetTimeSecond () ;
                    JoinWhiteList[i].ActiveNode = false;
                } else {
                    uint32_t Temp = ( mcu.RtcGetTimeSecond () - JoinWhiteList[i].LastTimeReceiveSecond ) / 16 ;
                    Temp = ( Temp > 255 ) ? 255 : Temp ; 
                    Payload[index+11] =Temp;
                }
                index = index + 12;
                NbElementJoinWhiteList++;
            }
        }
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp ( JoinBlackList[i].DevEui , VectTemp, 8 ) != 0 ) {
                memcpy ( &Payload[index], JoinBlackList[i].DevEui, 8 );    
                Payload[index+8] = JoinBlackList[i].Rssi;
                Payload[index+9]  = ( JoinBlackList[i].CptWakeUpSequence >> 8 ) & 0xFF;
                Payload[index+10] = ( JoinBlackList[i].CptWakeUpSequence      ) & 0xFF;
                if (JoinBlackList[i].ActiveNode == true) {
                    Payload[index+11] = 0 ;
                    JoinBlackList[i].LastTimeReceiveSecond = mcu.RtcGetTimeSecond () ;
                    JoinBlackList[i].ActiveNode = false;
                } else {
                    uint32_t Temp = ( mcu.RtcGetTimeSecond () - JoinBlackList[i].LastTimeReceiveSecond ) / 16 ;
                    Temp = ( Temp > 255 ) ? 255 : Temp ; 
                    Payload[index+11] =Temp;
                }
                index = index + 12;
            }
        }
        *Size = index;
        Payload[12] = NbElementJoinWhiteList + 0x80;
        
    };
    BoolOkKo SetConfigForRx3 (  uint32_t  TxTimeForRx3In , uint32_t  DevaddrIn, uint32_t FreqIn ) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == DevaddrIn) {
                WhiteList[i].TxTimeForRx3 = TxTimeForRx3In;
                WhiteList[i].Rx3Activated = RX3_ENABLE;
                WhiteList[i].Rx3Frequency = FreqIn;
                return (OK);
            }
        }
        return (KO);
    }
    BoolOkKo SetConfigForRx3 (  uint32_t  TxTimeForRx3In , uint8_t  DevEuiIn[8] , uint32_t FreqIn) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (DevEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ){
                JoinWhiteList[i].TxTimeForRx3 = TxTimeForRx3In;
                JoinWhiteList[i].Rx3Activated = RX3_ENABLE;
                JoinWhiteList[i].Rx3Frequency = FreqIn;
                return (OK);
            }
        }
        return (KO);
    }
    BoolOkKo GetConfigForRx3 (  uint32_t * TxTimeForRx3Out , uint32_t  DevaddrIn, uint32_t * FreqOut  ) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == DevaddrIn) {
                *TxTimeForRx3Out = WhiteList[i].TxTimeForRx3 ;
                *FreqOut         = WhiteList[i].Rx3Frequency;;
                return (OK);
            }
        }
        return (KO);
    }
    BoolOkKo GetConfigForRx3 (  uint32_t * TxTimeForRx3Out ,  uint8_t  DevEuiIn[8] ,  uint32_t * FreqOut  ) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (DevEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ){
                *TxTimeForRx3Out = JoinWhiteList[i].TxTimeForRx3 ;
                *FreqOut         = JoinWhiteList[i].Rx3Frequency;;
                return (OK);
            }
        }
        return (KO);
    }
    BoolYesNo IsRx3Activated (uint32_t devaddrIn) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( ( WhiteList[i].Devaddr == devaddrIn) && ( WhiteList[i].Rx3Activated == RX3_ENABLE ) ) {
                return (YES);
            }
        }
        return (NO);
    }
    BoolYesNo IsRx3Activated (uint8_t  DevEuiIn[8]) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( ( memcmp (DevEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ) && ( JoinWhiteList[i].Rx3Activated == RX3_ENABLE ) ) {
                return (YES);
            }
        }
        return (NO);
    }
    BoolOkKo ClearRx3Activation ( Rx3Activation   Rx3ActivatedIn , uint32_t  DevaddrIn ) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == DevaddrIn) {
                WhiteList[i].Rx3Activated = Rx3ActivatedIn ;
                return (OK);
            }
        }
        return (KO);
    }
    BoolOkKo ClearRx3Activation ( Rx3Activation   Rx3ActivatedIn , uint8_t  DevEuiIn[8] ) {
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (DevEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ) {
                JoinWhiteList[i].Rx3Activated = Rx3ActivatedIn ;
                return (OK);
            }
        }
        return (KO);
    }
    BoolOkKo updateCounter ( uint32_t ReceiveDevAddr, uint8_t FragNum){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( WhiteList[i].Devaddr == ReceiveDevAddr) {
                WhiteList[i].CptWakeUpSequence++;
                WhiteList[i].LastFragNumbnerWakeUpSequence = FragNum;
                return(OK);
            }
        }
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( BlackList[i].Devaddr == ReceiveDevAddr) {
                BlackList[i].CptWakeUpSequence++;
                BlackList[i].LastFragNumbnerWakeUpSequence = FragNum;
                return(OK);
            }
        }
        return (KO);
    };
    BoolOkKo updateCounter ( uint8_t devEuiIn[8], uint8_t FragNum){
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (devEuiIn , JoinBlackList[i].DevEui, 8 ) == 0 ) {
                JoinBlackList[i].CptWakeUpSequence++;
                JoinBlackList[i].ActiveNode = true;
                return(OK);
            }
        }
        for (int i =0; i < NB_NODE_IN_RELAY; i++ ) {
            if ( memcmp (devEuiIn , JoinWhiteList[i].DevEui, 8 ) == 0 ) {
                JoinWhiteList[i].CptWakeUpSequence++;
                JoinWhiteList[i].ActiveNode = true;
                return(OK);
            }
        }
        return (KO);
    };
private :
typedef struct SDevice{
    uint32_t        Devaddr;
    uint32_t        TxTimeForRx3;
    uint32_t        Rx3Frequency;
    Rx3Activation   Rx3Activated;
    uint16_t        CptWakeUpSequence;
    uint8_t         LastFragNumbnerWakeUpSequence;
    uint8_t         Rssi;
}SDevice;

typedef struct SDeviceNotJoin{
    uint8_t         DevEui[8];
    uint32_t        TxTimeForRx3;
    uint32_t        Rx3Frequency;
    Rx3Activation   Rx3Activated;
    uint16_t        CptWakeUpSequence;
    bool            ActiveNode;
    uint32_t        LastTimeReceiveSecond;
    uint8_t         Rssi;
}SDeviceNotJoin;
DECLARE_ARRAY ( SDevice, NB_NODE_IN_RELAY, WhiteList );
DECLARE_ARRAY ( SDevice, NB_NODE_IN_RELAY, BlackList );
DECLARE_ARRAY ( SDeviceNotJoin, NB_NODE_IN_RELAY, JoinBlackList  );
DECLARE_ARRAY ( SDeviceNotJoin, NB_NODE_IN_RELAY, JoinWhiteList  );
};

extern Relay relay;
#endif