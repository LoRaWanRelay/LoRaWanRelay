/*

  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : LoraWanProcess Class definition.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#include "LoraWanProcess.h"
#include "utilities.h"
#include "Define.h"
#include "ApiMcu.h"
#define FileId 0
template class LoraWanObject< LoraRegionsEU, SX1276 >;
template class LoraWanObject< LoraRegionsEU, SX126x >;
template class LoraWanObject< LoraRegionsUS, SX1276 >;
template class LoraWanObject< LoraRegionsUS, SX126x >;
template class LoraWanObject< LoraRegionsEU, SX1272 >;
template class LoraWanObject< LoraRegionsUS, SX1272 >;
template <template <class R> class T, class RADIOTYPE>
LoraWanObject<T,RADIOTYPE>::LoraWanObject( sLoRaWanKeys LoRaWanKeys,RadioPLaner<RADIOTYPE> * RadioUser,uint32_t FlashAdress ):packet(  LoRaWanKeys, RadioUser,FlashAdress ) {
    StateLoraWanProcess = LWPSTATE_IDLE;
    packet.MajorBits    = LORAWANR1;
    FlashAdress         = FlashAdress;
    ClassCActivated     = CLASS_C_NOT_ACTIVATED;
}; 
template <template <class R> class T, class RADIOTYPE> 
LoraWanObject <T,RADIOTYPE> ::~LoraWanObject() {
};
template <template <class R> class T, class RADIOTYPE> 
void LoraWanObject <T,RADIOTYPE> ::Init() {
    packet.RegionGiveNextDataRate ();
};

/************************************************************************************************/
/*                      Public  Methods                                                         */
/************************************************************************************************/


/***********************************************************************************************/
/*    LoraWanProcess Method                                                                    */
/***********************************************************************************************/

template <template <class R> class T, class RADIOTYPE> 
eLoraWan_Process_States LoraWanObject <T,RADIOTYPE> ::LoraWanProcess( uint8_t* AvailableRxPacket ) {
    uint8_t MyHookId;
    packet.Phy.Radio->GetMyHookId  ( &(this->packet.Phy) , MyHookId );
    *AvailableRxPacket = NO_LORA_RXPACKET_AVAILABLE;
    #if LOW_POWER_MODE == 1
    if ( ( IsJoined ( ) == NOT_JOINED ) && ( mcu.RtcGetTimeSecond( ) < packet.RtcNextTimeJoinSecond ) ){
        //DEBUG_PRINTF("TOO SOON TO JOIN time is  %d time target is : %d \n",mcu.RtcGetTimeSecond( ), packet.RtcNextTimeJoinSecond);
       // StateLoraWanProcess = LWPSTATE_IDLE ;
    }        
    #endif

    if ( ( mcu.RtcGetTimeSecond( ) - GetFailSafeTimestamp () ) > 120 ) {
        StateLoraWanProcess = LWPSTATE_ERROR ;
        mcu.DisableIrq ( );
        DEBUG_MSG ( "ERROR : FAILSAFE EVENT OCCUR \n");
        while ( 1 ) {
        
        }
        // NVIC_SystemReset() move into the user main;
    }        
    switch ( StateLoraWanProcess ) {
    /************************************************************************************/
    /*                                    STATE IDLE                                    */
    /************************************************************************************/
        case LWPSTATE_IDLE :
            break;
    /************************************************************************************/
    /*                                    STATE TX                                      */
    /************************************************************************************/
        case LWPSTATE_SEND:
            switch ( GetRadioState( ) ) {
                case RADIOSTATE_IDLE :
                   // InsertTrace ( __COUNTER__, FileId );
                    //AttachRadioIsr ( );                    
                    packet.ConfigureRadioAndSend( );
                    DEBUG_MSG    ( "\n" );
                    DEBUG_MSG    ( "  *************************************\n " );
                    DEBUG_PRINTF ( " *       Send Payload  HOOK ID = %d   *\n ", MyHookId);
                    DEBUG_MSG    ( " **************************************\n " );
                    break ; 
            
                case RADIOSTATE_TXFINISHED :
                    InsertTrace ( __COUNTER__, FileId );
                    packet.ConfigureTimerForRx ( RX1 );
                    if ( ClassCActivated == CLASS_C_ACTIVATED ) {
                        packet.ConfigureRadioForRxClassC ();
                    } 
                    StateLoraWanProcess = LWPSTATE_RX1;
                    break ;
                default :
                    break;
            }
            break;
    /************************************************************************************/
    /*                                   STATE RX1                                      */
    /* RX1DELAY is defined in ms                                                        */
    /************************************************************************************/
        case LWPSTATE_RX1:
            if ( GetRadioState( ) == RADIOSTATE_RX1FINISHED ) {
                if ( GetPlanerStatus ( ) == PLANER_RX_PACKET) {
                    InsertTrace ( __COUNTER__, FileId );
                    DEBUG_MSG( "\n" );
                    DEBUG_MSG    ( "  ******************************************\n " );
                    DEBUG_PRINTF ( " *Receive a downlink RX1 for Hook Id= %d   *\n ", MyHookId);
                    DEBUG_MSG    ( " *******************************************\n " );
                    StateLoraWanProcess = LWPSTATE_PROCESSDOWNLINK;
                } else { 
                    InsertTrace ( __COUNTER__, FileId );
                    DEBUG_MSG( "\n" );
                    DEBUG_MSG     ( "  *************************************\n " );
                    DEBUG_PRINTF  ( " * RX1 Timeout for Hook Id = %d       *\n ", MyHookId);
                    DEBUG_MSG     ( " **************************************\n " );
                    packet.ConfigureTimerForRx ( RX2 );
                    if ( ClassCActivated == CLASS_C_ACTIVATED ) {
                         packet.ConfigureRadioForRxClassC ();
                    } 
                    StateLoraWanProcess = LWPSTATE_RX2;
                }
            }
            break;
    /************************************************************************************/
    /*                                   STATE RX2                                      */
    /************************************************************************************/
        case LWPSTATE_RX2:
                            
            if ( ( GetRadioState( ) == RADIOSTATE_IDLE ) || ( GetRadioState( ) == RADIOSTATE_RX2FINISHED ) )  {
                 if ( GetPlanerStatus ( ) == PLANER_RX_PACKET) {
                    InsertTrace ( __COUNTER__, FileId );
                    DEBUG_MSG( "\n" );
                    DEBUG_MSG    ( "  ********************************************\n " );
                    DEBUG_PRINTF ( " *  Receive a downlink RX2 for Hook Id = %d  *\n ", MyHookId);
                    DEBUG_MSG    ( " *********************************************\n " );
                    StateLoraWanProcess = LWPSTATE_PROCESSDOWNLINK; 
                } else {
                    InsertTrace ( __COUNTER__, FileId );
                    DEBUG_MSG( "\n" );
                     DEBUG_MSG    ( "  *************************************\n " );
                    DEBUG_PRINTF  ( " * RX2 Timeout for Hook Id = %d       *\n ", MyHookId);
                    DEBUG_MSG     ( " **************************************\n " );
                    if ( IsActivatedRX3 () ==  RX3_ACTIVATED ) {
                        packet.ConfigureTimerForRx ( RX3 );
                        StateLoraWanProcess = LWPSTATE_RX3;
                    } else {
                       StateLoraWanProcess = LWPSTATE_UPDATEMAC; 
                    }
                }
            }
            break;
    /************************************************************************************/
    /*                                   STATE RX3                                      */
    /************************************************************************************/
        case LWPSTATE_RX3:
                            
            if ( GetRadioState( ) == RADIOSTATE_IDLE ) {
                 if ( GetPlanerStatus ( ) == PLANER_RX_PACKET) {
                    InsertTrace ( __COUNTER__, FileId );
                    DEBUG_MSG( "\n" );
                    DEBUG_MSG    ( "  ********************************************\n " );
                    DEBUG_PRINTF ( " *  Receive a downlink RX3 for Hook Id = %d  *\n ", MyHookId);
                    DEBUG_MSG    ( " *********************************************\n " );
                    StateLoraWanProcess = LWPSTATE_PROCESSDOWNLINK; 
                } else {
                    InsertTrace ( __COUNTER__, FileId );
                    DEBUG_MSG( "\n" );
                     DEBUG_MSG    ( "  *************************************\n " );
                    DEBUG_PRINTF  ( " * RX3 Timeout for Hook Id = %d       *\n ", MyHookId);
                    DEBUG_MSG     ( " **************************************\n " );
                    StateLoraWanProcess = LWPSTATE_UPDATEMAC; 
                }
            }
            break;
    /************************************************************************************/
    /*                              STATE PROCESS DOWNLINK                              */
    /* At this step crc is valid                                                        */
    /*    Step 1 : CheckRxPayloadLength                                                 */
    /*    Step 2 : ExtractRxMhdr                                                        */
    /*    Step 3 : ExtractRxFhdr                                                        */
    /*    Step 4 : Check Mic                                                            */
    /*    Step 5 : Decrypt Payload                                                      */
    /*    Step 6 : Extract Fport to select Between App/nwm Payload                      */
    /************************************************************************************/
        case LWPSTATE_PROCESSDOWNLINK:
            InsertTrace ( __COUNTER__, FileId );
            DEBUG_MSG( "\n" );
            DEBUG_MSG     ( " *************************************\n " );
            DEBUG_PRINTF  ( " Process Downlink  for Hook Id  = %d *\n ", MyHookId);
            DEBUG_MSG     ( " *************************************\n " );
            ValidRxPacket = packet.DecodeRxFrame( ); // return NOVALIDRXPACKET or  USERRX_FOPTSPACKET or NWKRXPACKET or JOIN_ACCEPT_PACKET.
            StateLoraWanProcess = LWPSTATE_UPDATEMAC;
            break;
    /************************************************************************************/
    /*                              STATE UPDATE MAC                                    */
    /************************************************************************************/
        case LWPSTATE_UPDATEMAC:
            InsertTrace ( __COUNTER__, FileId );
         //   DetachRadioIsr ( );
            packet.Phy.StateRadioProcess = RADIOSTATE_IDLE;  
            DEBUG_MSG( "\n" );
            DEBUG_MSG( "  *********************************\n " );
            DEBUG_PRINTF  ( " Update Mac for Hook Id = %d *\n ", MyHookId);
            DEBUG_MSG( " **********************************\n " );
            if ( ValidRxPacket == JOIN_ACCEPT_PACKET){
                packet.UpdateJoinProcedure( );
                packet.RegionSetDataRateDistribution( packet.AdrModeSelect);//@note because datarate Distribution has been changed during join
            }
            if ( ( ValidRxPacket == NWKRXPACKET) || ( ValidRxPacket == USERRX_FOPTSPACKET) ) {
                packet.ParseManagementPacket( );
            }
            packet.UpdateMacLayer();
            *AvailableRxPacket = packet.AvailableRxPacketForUser;
            if ( ( packet.IsFrameToSend == NWKFRAME_TOSEND ) || ( packet.IsFrameToSend == USRFRAME_TORETRANSMIT) ) {// @note ack send during the next tx|| ( packet.IsFrameToSend == USERACK_TOSEND ) ) {
                packet.IsFrameToSend = NOFRAME_TOSEND;
                RtcTargetTimer = mcu.RtcGetTimeSecond( ) + randr( 1, 2 ); 
                StateLoraWanProcess = LWPSTATE_TXwait;
            } else {
                //RadioReset ( ) ; @tbd Radioplaner 
                if (( ClassCActivated == CLASS_C_NOT_ACTIVATED ) || ( *AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) ){
                    StateLoraWanProcess = LWPSTATE_IDLE;
                } else {
                    packet.Phy.StateRadioProcess = RADIOSTATE_RXC; 
                    packet.ConfigureRadioForRxClassC ();
                    StateLoraWanProcess = LWPSTATE_RX_CLASSC;
                }
            }
            ValidRxPacket = NO_MORE_VALID_RX_PACKET;
            break;
    /************************************************************************************/
    /*                              STATE TXwait MAC                                    */
    /************************************************************************************/
        case LWPSTATE_TXwait:
            InsertTrace ( __COUNTER__, FileId );
            DEBUG_MSG(".");
            if ( mcu.RtcGetTimeSecond( ) > RtcTargetTimer) {
                StateLoraWanProcess = LWPSTATE_SEND; //@note the frame have already been prepare in Upadate Mac Layer
            }
            break;

    /************************************************************************************/
    /*                              STATE RX CLASS C MAC                                */
    /************************************************************************************/
         case LWPSTATE_RX_CLASSC:
            if ( GetRadioState( ) == RADIOSTATE_IDLE ) {
                 if ( GetPlanerStatus ( ) == PLANER_RX_PACKET) {
                    DEBUG_MSG( "\n" );
                    DEBUG_MSG( "  **************************\n " );
                    DEBUG_MSG( " * Receive a downlink RXC  *\n " );
                    DEBUG_MSG( " **************************\n " );
                    StateLoraWanProcess = LWPSTATE_PROCESSDOWNLINK; 
                }
            }
            break;
        default: 
            InsertTrace ( __COUNTER__, FileId );
            DEBUG_MSG( " Illegal state in lorawan process\n " );
            break;
        }
    return ( StateLoraWanProcess );
}
 
/***********************************************************************************************/
/*    End Of LoraWanProcess Method                                                             */
/***********************************************************************************************/


/**************************************************/
/*            LoraWan  Join  Method               */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
eLoraWan_Process_States LoraWanObject <T,RADIOTYPE> ::Join ( uint32_t TargetTime ) {
    packet.Phy.LastItTimeFailsafe =  mcu.RtcGetTimeSecond ( );
    if ( StateLoraWanProcess != LWPSTATE_IDLE ) {
        DEBUG_MSG( " ERROR : LP STATE NOT EQUAL TO IDLE \n" );
        return ( LWPSTATE_ERROR );
    }
    if ( GetIsOtaDevice ( ) == APB_DEVICE ) {
        DEBUG_MSG( " ERROR : APB DEVICE CAN'T PROCCED A JOIN REQUEST \n" );
        return ( LWPSTATE_ERROR );
    }
    packet.Phy.JoinedStatus = NOT_JOINED;
    packet.MacNbTransCpt = packet.MacNbTrans = 1;
    packet.RegionSetDataRateDistribution( JOIN_DR_DISTRIBUTION ); 
    //packet.RegionGiveNextDataRate ( );
    packet.BuildJoinLoraFrame( );
    packet.MacRx2DataRate = packet.RX2DR_INIT;
    packet.MacRx1Delay = packet.JOIN_ACCEPT_DELAY1;
    StateLoraWanProcess = LWPSTATE_SEND;
    packet.Phy.SendTargetTime = TargetTime; // Use only for send at time
    return( StateLoraWanProcess );
};


/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
eJoinStatus LoraWanObject <T,RADIOTYPE> ::IsJoined( void ) {
    eJoinStatus status = NOT_JOINED;
    status = packet.Phy.JoinedStatus;
    return ( status );
}

/**************************************************/
/*          LoraWan  IsJoined  Method             */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
void LoraWanObject <T,RADIOTYPE> ::NewJoin ( void ) {
    packet.Phy.JoinedStatus = NOT_JOINED; 
}
/**************************************************/
/*         LoraWan  SendPayload  Method           */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
eLoraWan_Process_States LoraWanObject <T,RADIOTYPE> ::SendPayload ( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn, uint8_t PacketType,uint32_t TargetTime ) {
    packet.Phy.LastItTimeFailsafe =  mcu.RtcGetTimeSecond ( );
    eStatusLoRaWan status;
    //packet.RegionGiveNextDataRate ( ); // both choose  the next tx data rate but also compute the Sf and Bw (region )
    status = packet.RegionMaxPayloadSize ( sizeIn );
    if ( status == ERRORLORAWAN ) {
        DEBUG_MSG( " ERROR : PAYLOAD SIZE TOO HIGH \n" );
        return ( LWPSTATE_INVALID );
    }
    if ( GetIsOtaDevice ( ) == OTA_DEVICE ) {
        if ( packet.Phy.JoinedStatus ==  NOT_JOINED ) {
            DEBUG_MSG( " ERROR : OTA DEVICE NOT JOINED YET\n" );
            return ( LWPSTATE_INVALID );
        }
    }

    CopyUserPayload( dataIn,sizeIn );
    packet.UserPayloadSize = sizeIn;
    packet.fPort = fPort;
    packet.MType = PacketType;
    packet.BuildTxLoraFrame( );
    packet.EncryptTxFrame( );
    if (PacketType == CONF_DATA_UP){
        packet.MacNbTransCpt = MAX_CONFUP_MSG;
    } else {
        packet.MacNbTransCpt = packet.MacNbTrans;
    }
    StateLoraWanProcess = LWPSTATE_SEND;
    packet.Phy.SendTargetTime = TargetTime; // Use only for send at time
    return( StateLoraWanProcess );
};


/**************************************************/
/*        LoraWan  Receive  Method                */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
eStatusLoRaWan LoraWanObject <T,RADIOTYPE> ::ReceivePayload ( uint8_t* UserRxFport, uint8_t* UserRxPayload, uint8_t* UserRxPayloadSize ) {
    eStatusLoRaWan status = OKLORAWAN; 
    if (packet.AvailableRxPacketForUser == NO_LORA_RXPACKET_AVAILABLE) {
        status = ERRORLORAWAN ;
    } else {
        *UserRxPayloadSize = packet.MacRxPayloadSize;
        *UserRxFport = packet.FportRx;
        memcpy( UserRxPayload, &packet.MacRxPayload[0], packet.MacRxPayloadSize);
        packet.AvailableRxPacketForUser = NO_LORA_RXPACKET_AVAILABLE ;
    }
    return( status );
};

/**************************************************/
/*       LoraWan  AdrModeSelect  Method           */
/**************************************************/
template <template <class R> class T, class RADIOTYPE>
void LoraWanObject <T,RADIOTYPE> ::SetDataRateStrategy( eDataRateStrategy adrModeSelect ) {
    packet.AdrModeSelect = adrModeSelect;
    packet.RegionSetDataRateDistribution( adrModeSelect );
    packet.RegionGiveNextDataRate ();
};


/**************************************************/
/*         LoraWan  GetDevAddr  Method            */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
uint32_t LoraWanObject <T,RADIOTYPE> ::GetDevAddr ( void ) {
    return(packet.DevAddr);
}

/**************************************************/
/*         LoraWan  GetNextPower  Method          */
/**************************************************/
template <template <class R> class T, class RADIOTYPE>
uint8_t LoraWanObject <T,RADIOTYPE> ::GetNextPower ( void ) {
    return(packet.MacTxPower);
}

/**************************************************/
/*    LoraWan  GetLorawanProcessState  Method     */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
eLoraWan_Process_States LoraWanObject <T,RADIOTYPE> ::GetLorawanProcessState ( void ) {
    return(StateLoraWanProcess);
}
 
/**************************************************/
/*    LoraWan  RestoreContext  Method             */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
void LoraWanObject <T,RADIOTYPE> ::RestoreContext ( void ) {
    packet.RegionLoadFromFlash ( );
}; 

 
/**************************************************/
/*    LoraWan  storeContext  Method               */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
 void LoraWanObject <T,RADIOTYPE> ::SetProvisionning ( sLoRaWanKeys LoRaWanKeys ) {
    memcpy( packet.appSKey, LoRaWanKeys.LoRaMacAppSKey, 16 );
    memcpy( packet.nwkSKey, LoRaWanKeys.LoRaMacNwkSKey, 16 );
    memcpy( packet.appKey, LoRaWanKeys.LoRaMacAppKey, 16 );
    memcpy( packet.devEui, LoRaWanKeys.DevEui, 8 );
    memcpy( packet.appEui, LoRaWanKeys.AppEui, 8 );
    packet.otaDevice      = LoRaWanKeys.OtaDevice;
    packet.SetDevAddr( LoRaWanKeys.LoRaDevAddr ) ;
    packet.RegionSaveInFlash ( );
 }
/**************************************************/
/*   LoraWan  GetNextMaxPayloadLength  Method     */
/**************************************************/
template <template <class R> class T, class RADIOTYPE>
uint32_t LoraWanObject <T,RADIOTYPE> ::GetNextMaxPayloadLength ( void ) {// error return during tx send to be replace by get datarate?
    return(0);//@NOTE NOT YET IMPLEMENTED
}

/**************************************************/
/*   LoraWan  ActivateClassC  Method              */
/**************************************************/
template <template <class R> class T, class RADIOTYPE>
void LoraWanObject <T,RADIOTYPE> :: ActivateClassC ( void ){
    ClassCActivated = CLASS_C_ACTIVATED;
};
/**************************************************/
/*   LoraWan  DeActivateClassC  Method              */
/**************************************************/
template <template <class R> class T, class RADIOTYPE>
void LoraWanObject <T,RADIOTYPE> :: DeActivateClassC ( void ){
	  ClassCActivated = CLASS_C_NOT_ACTIVATED;
};
/**************************************************/
/*        LoraWan  GetNextDataRate  Method        */
/**************************************************/
template <template <class R> class T, class RADIOTYPE> 
uint8_t LoraWanObject <T,RADIOTYPE> ::GetNextDataRate ( void ) { // note return datareate in case of adr
    return( packet.MacTxDataRate ) ;
}

template <template <class R> class T, class RADIOTYPE> 
uint32_t LoraWanObject <T,RADIOTYPE> ::GetNextFrequency ( void ) { // note return datareate in case of adr
    return( packet.MacTxFrequencyCurrent ) ;
}

template <template <class R> class T, class RADIOTYPE> 
 void  LoraWanObject <T,RADIOTYPE> :: FactoryReset ( void ) {
     packet.RegionSetBadCrcInFlash ( ) ;
 }
 
template <template <class R> class T, class RADIOTYPE> 
eDeviceTypeOTA_APB LoraWanObject <T,RADIOTYPE> ::GetIsOtaDevice (void){
    return (eDeviceTypeOTA_APB)packet.otaDevice;
}
template <template <class R> class T, class RADIOTYPE> 
void LoraWanObject <T,RADIOTYPE> ::SetOtaDevice (eDeviceTypeOTA_APB  deviceType){
    packet.otaDevice = deviceType;
}

/************************************************************************************************/
/*                      Private  Methods                                                        */
/************************************************************************************************/
template <template <class R> class T, class RADIOTYPE> 
void LoraWanObject <T,RADIOTYPE> ::CopyUserPayload( const uint8_t* dataIn, const uint8_t sizeIn ) {
    memcpy( &packet.Phy.TxPhyPayload[ FHDROFFSET + packet.FoptsTxLengthCurrent ], dataIn, sizeIn );
};
 


template <template <class R> class T, class RADIOTYPE> 
uint8_t LoraWanObject <T,RADIOTYPE> ::GetRadioState ( void ) {
    return packet.Phy.GetRadioState( );
};
 
template <template <class R> class T, class RADIOTYPE> 
ePlanerStatus LoraWanObject <T,RADIOTYPE> ::GetPlanerStatus ( void ) {
    return packet.Phy.PlanerStatus;
};


template <template <class R> class T, class RADIOTYPE> 
uint8_t LoraWanObject <T,RADIOTYPE> ::GetNbOfReset (void){
    return packet.NbOfReset;
}
