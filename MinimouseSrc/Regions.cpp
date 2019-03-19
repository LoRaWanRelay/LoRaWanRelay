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

#include "Regions.h"
#include "Define.h"
#include "utilities.h"
#define FileId 1
/*************************************************/
/*                     Constructors              */
/*************************************************/
template class LoraRegionsEU<SX1276>;
template class LoraRegionsEU<SX1272>;
template class LoraRegionsEU<SX126x>;
template < class R > LoraRegionsEU<R>::LoraRegionsEU ( sLoRaWanKeys LoRaWanKeys, RadioPLaner<R>* RadioUser, uint32_t FlashAdress) : LoraWanContainer<16,R>  ( LoRaWanKeys, RadioUser, FlashAdress ){
    
    memset( this->MacChannelIndexEnabled, CHANNEL_DISABLED, this->NUMBER_OF_CHANNEL );
    memset( this->MacMinDataRateChannel, 0, this->NUMBER_OF_CHANNEL );
    for (int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
        this->MacTxFrequency  [i] = 0;
        this->MacRx1Frequency [i] = 0;
    }
    this->MacChannelIndexEnabled [0] = CHANNEL_ENABLED;
    this->MacChannelIndexEnabled [1] = CHANNEL_ENABLED;
    this->MacChannelIndexEnabled [2] = CHANNEL_ENABLED;
    this->MacMinDataRateChannel [0] = 0;
    this->MacMinDataRateChannel [1] = 0;
    this->MacMinDataRateChannel [2] = 0;
    this->MacMaxDataRateChannel [0] = 5;
    this->MacMaxDataRateChannel [1] = 5;
    this->MacMaxDataRateChannel [2] = 5;
    this->MacTxFrequency[0]    = 868100000;
    this->MacTxFrequency[1]    = 868300000;
    this->MacTxFrequency[2]    = 868500000;
    this->MacRx1Frequency[0]   = 868100000;
    this->MacRx1Frequency[1]   = 868300000;
    this->MacRx1Frequency[2]   = 868500000;
    this->MacRx2Frequency      = 869525000; 
    this->MacTxPower           = TX_POWER;
    this->MacRx1DataRateOffset = 0;
    this->MacRx2DataRate       = RX2DR_INIT;
    this->MacRx1Delay          = RECEIVE_DELAY1;
    this->MacTxDataRateAdr     = 0 ;
}

/***********************************************************************************************/
/*                      Protected  Methods                                                     */
/***********************************************************************************************/


/********************************************************************/
/*                  Region Tx Power  Configuration                  */
/* Chapter 7.1.3 LoRaWan 1.0.1 specification                        */
/*  TXPower    Configuration                                        */
/* Max TX POwer is suppose = to 14                                  */
/*                                                                  */
/********************************************************************/

template < class R >void LoraRegionsEU<R>::RegionSetPower ( uint8_t PowerCmd ) {
    InsertTrace ( __COUNTER__, FileId );
    uint8_t PowerTab [ 8 ] = { TX_POWER, TX_POWER-2, TX_POWER-4, TX_POWER-6, TX_POWER-8, TX_POWER-10, TX_POWER-12, TX_POWER-14 };
    if  ( PowerCmd > 7 ) {
            this->MacTxPower = 14 ;
            DEBUG_MSG ("INVALID POWER \n");
    } else {
        this->MacTxPower = PowerTab [ PowerCmd ] ;
    }
}

/********************************************************************/
/*                  Region Get Cf List                              */
/* Chapter 7.1.4 LoRaWan 1.0.1 specification                        */
/********************************************************************/
template < class R >void LoraRegionsEU<R>::RegionGetCFList ( void ) {
    InsertTrace ( __COUNTER__, FileId );
    for ( int i = 0 ; i < 5 ; i++ ) {
       this->MacTxFrequency [3 + i] = 100 * ( ( this->CFList[0 + ( 3 * i )] ) + ( this->CFList[1 + ( 3 * i )] << 8 )+ ( this->CFList[2 + ( 3 * i )] << 16 ) );
       this->MacRx1Frequency [3 + i] = 100 * ( ( this->CFList[0 + ( 3 * i )] ) + ( this->CFList[1 + ( 3 * i )] << 8 )+ ( this->CFList[2 + ( 3 * i )] << 16 ) );
        if ( ( this->MacTxFrequency [3 + i] >= ( FREQMIN * 100 ) ) && ( this->MacTxFrequency [3 + i] <=( FREQMAX * 100 ) ) ) {
            this->MacMinDataRateChannel [3 + i]  = 0;
            this->MacMaxDataRateChannel [3 + i]  = 5;
            this->MacChannelIndexEnabled [3 + i] = CHANNEL_ENABLED ;
            DEBUG_PRINTF( " MacTxFrequency [%d] = %d \n",i,this->MacTxFrequency [3 + i]);
            DEBUG_PRINTF( " MacMinDataRateChannel [%d] = %d \n",i,this->MacMinDataRateChannel [3 + i]);
            DEBUG_PRINTF( " MacMaxDataRateChannel [%d] = %d \n",i,this->MacMaxDataRateChannel [3 + i]);
            DEBUG_PRINTF( " MacChannelIndexEnabled [%d] = %d \n",i,this->MacChannelIndexEnabled [3 + i]);
        } else {
            this->MacTxFrequency  [3 + i] = 0;
            this->MacRx1Frequency [3 + i] = 0;
            DEBUG_MSG ("INVALID TX FREQUENCY IN CFLIST \n");
        }
    }
}

/********************************************************************/
/*                  Region Set Channel MAsk                         */
/* Chapter 7.1.5 LoRaWan 1.0.1 specification                        */
/********************************************************************/

template < class R >eStatusChannel LoraRegionsEU<R>::RegionBuildChannelMask ( uint8_t ChMaskCntl, uint16_t ChMask ) {
    InsertTrace ( __COUNTER__, FileId );
    eStatusChannel status = OKCHANNEL;
    switch ( ChMaskCntl ) {
        case 0 :
            UnwrappedChannelMask = UnwrappedChannelMask & ChMask; 
            DEBUG_PRINTF("UnwrappedChannelMask = 0x%x, ChMask = 0x%x\n",UnwrappedChannelMask,ChMask);
            for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i++) {
                if ( ( ( ( UnwrappedChannelMask >> i) & 0x1 ) == 1 ) && ( this->MacTxFrequency[i] == 0) ) {  
                    status = ERROR_CHANNEL_MASK ;   // this status is used only for the last multiple link adr req
                }
            }
            break;
        case 6 :
            UnwrappedChannelMask = 0;
            for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i++) {
                if ( this->MacTxFrequency[i] > 0 ) {
                    UnwrappedChannelMask = UnwrappedChannelMask ^ (1 << i ) ;
                }
            }
            break;
        default : 
            status = ERROR_CHANNEL_CNTL;
    }
    if ( UnwrappedChannelMask == 0 ) {
        status = ERROR_CHANNEL_MASK ; 
    }        
    return ( status );
};

template < class R >void LoraRegionsEU<R>::RegionInitChannelMask ( void ) {
    UnwrappedChannelMask = 0xFFFF;
};
template < class R >void LoraRegionsEU<R>::RegionSetMask ( void ) {
    InsertTrace ( __COUNTER__, FileId );
    DEBUG_MSG(" \n Mask = ");
    for (int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
        this->MacChannelIndexEnabled [i] = ( UnwrappedChannelMask >> i ) & 0x1; // @note trade off between size and code simplification
        DEBUG_PRINTF(" %d ",this->MacChannelIndexEnabled [i]);
    }
    DEBUG_MSG(" \n");
};
/********************************************************************/
/*                  Region MAx Payload SIze  Configuration          */
/* Chapter 7.1.6 LoRaWan 1.0.1 specification                        */
/********************************************************************/
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionMaxPayloadSize ( uint8_t sizeIn ) {
    eStatusLoRaWan  status ;
    uint8_t M [ 8 ] = { 59, 59, 59, 123, 230, 230, 230, 230 };
    status = ( sizeIn >= M [this->MacTxDataRate] ) ? ERRORLORAWAN : OKLORAWAN ;
    return ( status );
}

/********************************************************************/
/*                  Region Rx Window  Configuration                 */
/* Chapter 7.1.7 LoRaWan 1.0.1 specification                        */
/********************************************************************/
//@notereview return status
template < class R >void LoraRegionsEU<R>::RegionSetRxConfig ( eRxWinType type ) {
    InsertTrace ( __COUNTER__, FileId );
    if ( type == RX1 ) {
        this->MacRx1SfCurrent =  ( this->MacTxSfCurrent < 12 - this->MacRx1DataRateOffset) ? this->MacTxSfCurrent + this->MacRx1DataRateOffset : 12;
        this->MacRx1BwCurrent = this->MacTxBwCurrent;
    } else if ( type == RX2 ) {
        Rx2DataRateToSfBw ( this->MacRx2DataRate );
    } else {
        DEBUG_MSG ("INVALID RX TYPE \n");
    }
}


/********************************************************************************/
/*           Check parameter of received mac commands                           */
/********************************************************************************/
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionIsValidRx1DrOffset ( uint8_t Rx1DataRateOffset ) {
    eStatusLoRaWan status = OKLORAWAN;
    if (Rx1DataRateOffset > 5) {
        status = ERRORLORAWAN ;
        DEBUG_MSG ( "RECEIVE AN INVALID RX1 DR OFFSET \n");
    }
    return ( status );
}

template < class R >eStatusLoRaWan LoraRegionsEU<R>:: RegionIsValidDataRate ( uint8_t temp ) {
    eStatusLoRaWan status ;
    status = ( temp > 7) ? ERRORLORAWAN : OKLORAWAN;
    return ( status );
}
    
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionIsAcceptableDataRate ( uint8_t DataRate ) {
    InsertTrace ( __COUNTER__, FileId );
    eStatusLoRaWan status = ERRORLORAWAN;
    for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL; i++) {
        if ( ( ( UnwrappedChannelMask >> i) & 0x1) == 1 ) {
            if ( ( DataRate >= this->MacMinDataRateChannel [i] ) && ( DataRate <= this->MacMaxDataRateChannel [i] ) ) {
                return ( OKLORAWAN );
            }
        }
    }
    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionIsValidMacFrequency ( uint32_t Frequency) {
    InsertTrace ( __COUNTER__, FileId );
    eStatusLoRaWan status = OKLORAWAN;
    if ( Frequency == 0) {
        return ( status );
    }
    if ( ( Frequency > FREQMAX ) || ( Frequency < FREQMIN ) ) {
        status = ERRORLORAWAN ;
        DEBUG_PRINTF ( "RECEIVE AN INVALID FREQUENCY = %d\n", Frequency);
    }
    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionIsValidMacRxFrequency ( uint32_t Frequency) {
    InsertTrace ( __COUNTER__, FileId );
    eStatusLoRaWan status = OKLORAWAN;
    if ( ( Frequency > FREQMAX ) || ( Frequency < FREQMIN ) ) {
        status = ERRORLORAWAN ;
        DEBUG_PRINTF ( "RECEIVE AN INVALID Rx FREQUENCY = %d\n", Frequency);
    }
    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionIsValidTxPower ( uint8_t Power) {
    eStatusLoRaWan status = OKLORAWAN;
    if ( ( Power > 7 ) ) {
        status = ERRORLORAWAN ;
        DEBUG_PRINTF ( "RECEIVE AN INVALID Power Cmd = %d\n", Power);
    }
    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsEU<R>::RegionIsValidChannelIndex ( uint8_t ChannelIndex) {
    eStatusLoRaWan status = OKLORAWAN;
    if ( ( ChannelIndex  < 3 ) || ( ChannelIndex  > 15 ) ) {
        status = ERRORLORAWAN ;
    }

    return ( status );
};


/********************************************************************************/
/*           RegionGiveNextDataRate                                             */
/*    method to set the next data Rate in different mode                        */
/********************************************************************************/

template < class R >void LoraRegionsEU<R>::RegionSetDataRateDistribution( uint8_t adrMode) {
    memset(DistriDataRateInit,0 , 8);
    switch ( adrMode ) {
        case MOBILE_LONGRANGE_DR_DISTRIBUTION:  // in this example 4/7 dr0 2/7 dr1 and 1/7 dr2
            DistriDataRateInit[7]    = 0; 
            DistriDataRateInit[6]    = 0; 
            DistriDataRateInit[5]    = 0; 
            DistriDataRateInit[4]    = 0;
            DistriDataRateInit[3]    = 0;
            DistriDataRateInit[2]    = 1; 
            DistriDataRateInit[1]    = 2; 
            DistriDataRateInit[0]    = 4; 
            this->MacNbTrans         = 1;   
            break;
        case MOBILE_LOWPER_DR_DISTRIBUTION://in this example 5/10 dr5 4/10 dr4 and 1/10 dr0
            DistriDataRateInit[7]    = 0; 
            DistriDataRateInit[6]    = 0; 
            DistriDataRateInit[5]    = 0; 
            DistriDataRateInit[4]    = 0;
            DistriDataRateInit[3]    = 1;
            DistriDataRateInit[2]    = 0; 
            DistriDataRateInit[1]    = 0;         
            DistriDataRateInit[0]    = 0; 
            this->MacNbTrans         = 1;
            break;
        case JOIN_DR_DISTRIBUTION: //in this example 1/3 dr5 1/3 dr4 and 1/3 dr0
            DistriDataRateInit[7]    = 0; 
            DistriDataRateInit[6]    = 0;
            DistriDataRateInit[5]    = 0; 
            DistriDataRateInit[4]    = 0; 
            DistriDataRateInit[3]    = 0;
            DistriDataRateInit[2]    = 0; 
            DistriDataRateInit[1]    = 0; 
            DistriDataRateInit[0]    = 1; 
            this->MacNbTrans         = 1;
            break;
        case USER_DR_DISTRIBUTION: //in this example 1/3 dr5 1/3 dr4 and 1/3 dr0
            DistriDataRateInit[7]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x0000000F ) );
            DistriDataRateInit[6]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x000000F0 ) ) >> 4; //fsk 
            DistriDataRateInit[5]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x00000F00 ) ) >> 8; 
            DistriDataRateInit[4]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x0000F000 ) ) >> 12; 
            DistriDataRateInit[3]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x000F0000 ) ) >> 16; 
            DistriDataRateInit[2]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x00F00000 ) ) >> 20; 
            DistriDataRateInit[1]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0x0F000000 ) ) >> 24; 
            DistriDataRateInit[0]    = ( ( USER_DR_DISTRIBUTION_PARAMETERS )& ( 0xF0000000 ) ) >> 28; 
            this->MacNbTrans         = USER_NUMBER_OF_RETRANSMISSION;
        
            break;
        default: 
            DistriDataRateInit[0]    = 1; 
            DistriDataRateInit[1]    = 0; 
            DistriDataRateInit[2]    = 0; 
            DistriDataRateInit[3]    = 0; 
            DistriDataRateInit[4]    = 0; 
            DistriDataRateInit[5]    = 0; 
            DistriDataRateInit[6]    = 0; 
            DistriDataRateInit[7]    = 0; 
            this->MacNbTrans         = 1;
    }
    this->MacTxPower                 = 14;
    this->MacChannelIndexEnabled [0] = CHANNEL_ENABLED;
    this->MacChannelIndexEnabled [1] = CHANNEL_ENABLED;
    this->MacChannelIndexEnabled [2] = CHANNEL_ENABLED;
    memcpy(DistriDataRate, DistriDataRateInit, 8);
}

template < class R >void LoraRegionsEU<R>::RegionGiveNextDataRate( void ) {
    RegionGiveNextChannel ( );  
    InsertTrace ( __COUNTER__, FileId );
    if ( this->AdrModeSelect == STATIC_ADR_MODE ) {
        this->MacTxDataRate = this->MacTxDataRateAdr;
        this->AdrEnable = 1;
    } else {
        int i;
        uint8_t DistriSum = 0;
        for ( i= 0 ; i < 8; i++ ){
            DistriSum += DistriDataRate[i];
        }
        if ( DistriSum == 0) {
            memcpy(DistriDataRate,DistriDataRateInit,8);
        }
        uint8_t Newdr = randr(0,7);
        while (DistriDataRate[Newdr] == 0) {
            Newdr = randr(0,7);
        }
        this->MacTxDataRate = Newdr;
        DistriDataRate[Newdr] -- ;
        this->AdrEnable = 0;
    }
    this->MacTxDataRate = ( this->MacTxDataRate > 7 ) ? 7 : this->MacTxDataRate;
    TxDataRateToSfBw ( this->MacTxDataRate );
}

/********************************************************************************/
/*           RegionDecreaseDataRate                                             */
/*    method to update Datarate in ADR Mode                                     */
/********************************************************************************/
template < class R >void LoraRegionsEU<R>::RegionDecreaseDataRate ( void ) {
    InsertTrace ( __COUNTER__, FileId );
    uint8_t ValidTemp = 0;//@notereview boolfjerrek
    while ( ( this->MacTxDataRateAdr > 0 ) && ( ValidTemp == 0 ) ) {
        this->MacTxDataRateAdr --;
        for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
            if ( this->MacChannelIndexEnabled [i] == CHANNEL_ENABLED ) {
                if ( ( this->MacTxDataRateAdr <= this->MacMaxDataRateChannel [i] ) && ( this->MacTxDataRateAdr >= this->MacMinDataRateChannel [i] ) ) {
                    ValidTemp ++;
                }
            }
        }
    }
    /* if adr DR = 0 enable the default channel*/
    if ( ( this->MacTxDataRateAdr == 0 ) && ( ValidTemp == 0) ) {
        this->MacChannelIndexEnabled [0] = CHANNEL_ENABLED ;
        this->MacChannelIndexEnabled [1] = CHANNEL_ENABLED ;
        this->MacChannelIndexEnabled [2] = CHANNEL_ENABLED ;;
    }
//@notereview    join continuer flag 
}


/********************************************************************************/
/*           RegionGiveNextChannel                                              */
/*    method to set the next enable channel                                     */
/********************************************************************************/
template < class R >void  LoraRegionsEU<R>::RegionGiveNextChannel( void ) {
    InsertTrace ( __COUNTER__, FileId );
    uint8_t NbOfActiveChannel = 0 ;
    for (int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
        if ( this->MacChannelIndexEnabled [i] == CHANNEL_ENABLED ) { 
            NbOfActiveChannel++;
        }
    }
    uint8_t temp = randr ( 0, ( NbOfActiveChannel - 1 ) );
    int ChannelIndex = 0;
    ChannelIndex = this->FindEnabledChannel ( temp ); // @note datarate valid not yett tested
    if ( ChannelIndex == -1 ) {
        DEBUG_PRINTF ("INVALID CHANNEL  active channel = %d and random channel = %d \n",NbOfActiveChannel,temp);
    } else {
        this->MacTxFrequencyCurrent = this->MacTxFrequency[ChannelIndex];
        this->MacRx1FrequencyCurrent = this->MacRx1Frequency[ChannelIndex];
    }
};

template < class R >uint8_t  LoraRegionsEU<R>::RegionGetAdrAckLimit( void ) {
    return ( ADR_ACK_LIMIT );
}
template < class R >uint8_t  LoraRegionsEU<R>::RegionGetAdrAckDelay( void ) {
    return ( ADR_ACK_DELAY );
}


template < class R >void LoraRegionsEU<R>::RegionLoadFromFlash ( void ){
    InsertTrace ( __COUNTER__, FileId );
    uint32_t crcLow;
    uint32_t crcHigh;
    mcu.RestoreContext((uint8_t *)(&BackUpFlash), this->UserFlashAdress, sizeof(sBackUpFlash));
    Crc64((uint8_t * )(&BackUpFlash), sizeof(sBackUpFlash)-8 , &crcLow, &crcHigh );    
    if (( crcLow == BackUpFlash.CrcLow ) &&  ( crcHigh == BackUpFlash.CrcHigh ) ) { // explicit else = factory reset => the default value inside the constructor
        BackUpFlash.FcntUp                 +=  FLASH_UPDATE_PERIOD; //@note automatic increment
        BackUpFlash.NbOfReset ++;
        this->NbOfReset                     = BackUpFlash.NbOfReset;
        this->MacTxDataRate                 = BackUpFlash.MacTxDataRate;
        this->MacTxPower                    = BackUpFlash.MacTxPower;
        this->MacChMask                     = BackUpFlash.MacChMask;
        this->MacNbTrans                    = BackUpFlash.MacNbTrans; 
        this->MacRx2Frequency               = BackUpFlash.MacRx2Frequency; 
        this->MacRx2DataRate                = BackUpFlash.MacRx2DataRate;
        this->MacRx1DataRateOffset          = BackUpFlash.MacRx1DataRateOffset;
        this->MacRx1Delay                   = BackUpFlash.MacRx1Delay ;
        this->FcntUp                        = BackUpFlash.FcntUp ;
        this->FcntDwn                       = BackUpFlash.FcntDwn ;
        //this->DevAddr                       = BackUpFlash.DevAddr;
        this->SetDevAddr ( BackUpFlash.DevAddr );
        this->DevNonce                      = BackUpFlash.DevNonce;
        this->Phy.JoinedStatus              = ( eJoinStatus ) BackUpFlash.JoinedStatus;
        for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
            this->MacTxFrequency[i]         = BackUpFlash.MacTxFrequency[i] ;
            this->MacRx1Frequency[i]        = BackUpFlash.MacRx1Frequency[i] ;
            this->MacMaxDataRateChannel[i]  = BackUpFlash.MacMaxDataRateChannel[i] ;
            this->MacMinDataRateChannel[i]  = BackUpFlash.MacMinDataRateChannel[i];
            this->MacChannelIndexEnabled[i] = BackUpFlash.MacChannelIndexEnabled[i];
        }
        memcpy( &this->nwkSKey[0], &BackUpFlash.nwkSKey[0], 16);
        memcpy( &this->appSKey[0], &BackUpFlash.appSKey[0], 16);
        Crc64((uint8_t * )(&BackUpFlash), sizeof(sBackUpFlash) - 8, &crcLow, &crcHigh );
        BackUpFlash.CrcLow  = crcLow;
        BackUpFlash.CrcHigh = crcHigh;
        mcu.StoreContext( &BackUpFlash, this->UserFlashAdress, ( sizeof(sBackUpFlash) >> 3 ) );    
        DEBUG_PRINTF ("\n MacTxDataRate = %d ", this->MacTxDataRate ) ;
        DEBUG_PRINTF ("\n MacTxPower = %d ", this->MacTxPower ) ;
        DEBUG_PRINTF ("\n MacChMask = 0x%x ", this->MacChMask ) ;
        DEBUG_PRINTF ("\n MacRx2Frequency = %d ", this->MacRx2Frequency ) ;
        DEBUG_PRINTF ("\n MacRx2DataRate = %d ", this->MacRx2DataRate ) ;
        DEBUG_PRINTF ("\n MacRx1DataRateOffset = %d ", this->MacRx1DataRateOffset ) ;
        DEBUG_PRINTF ("\n MacRx1Delay = %d ", this->MacRx1Delay ) ;
        DEBUG_PRINTF ("\n FcntUp = %d ", this->FcntUp ) ;
        DEBUG_PRINTF ("\n FcntDwn = %d ", this->FcntDwn ) ;
        DEBUG_PRINTF ("\n DevAddr = 0x%x ", this->DevAddr ) ;
        DEBUG_PRINTF ("\n DevNonce = 0x%x ", this->DevNonce ) ;
        DEBUG_PRINTF ("\n JoinedStatus = %d ",this->Phy.JoinedStatus  ) ;
        DEBUG_PRINTF ("\n NbOfReset = %d ",BackUpFlash.NbOfReset  ) ;
        for (int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
            DEBUG_PRINTF ("\n MacTxFrequency[%d]= %d ", i, this->MacTxFrequency[i] ) ;
            DEBUG_PRINTF ("\n MacRx1Frequency[%d]= %d ", i, this->MacRx1Frequency[i] ) ;
            DEBUG_PRINTF ("\n MacMaxDataRateChannel[%d]   = %d ", i, this->MacMaxDataRateChannel[i] ) ;
            DEBUG_PRINTF ("\n MacMinDataRateChannel[%d]   = %d ", i, this->MacMinDataRateChannel[i] ) ;
            DEBUG_PRINTF ("\n MacChannelIndexEnabled[%d]  = %d \n", i, this->MacChannelIndexEnabled[i] );
        }

    } else {
        BackUpFlash.NbOfReset = 0;
        RegionSaveInFlash ( );
        DEBUG_MSG("WRONG CRC \n");
        NVIC_SystemReset();
    }
}

template < class R >void LoraRegionsEU<R>::RegionSaveInFlash ( void ){
    InsertTrace ( __COUNTER__, FileId );
    uint32_t crcLow;
    uint32_t crcHigh;
    BackUpFlash.MacTxDataRate           = this->MacTxDataRate;
    BackUpFlash.MacTxPower              = this->MacTxPower;
    BackUpFlash.MacChMask               = this->MacChMask;
    BackUpFlash.MacNbTrans              = this->MacNbTrans; 
    BackUpFlash.MacRx2Frequency         = this->MacRx2Frequency; 
    BackUpFlash.MacRx2DataRate          = this->MacRx2DataRate;
    BackUpFlash.MacRx1DataRateOffset    = this->MacRx1DataRateOffset;
    BackUpFlash.MacRx1Delay             = this->MacRx1Delay;
    BackUpFlash.FcntUp                  = this->FcntUp;
    BackUpFlash.FcntDwn                 = this->FcntDwn;
    BackUpFlash.DevAddr                 = this->DevAddr;
    BackUpFlash.DevNonce                = this->DevNonce;
    BackUpFlash.JoinedStatus            = this->Phy.JoinedStatus;
    for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
        BackUpFlash.MacTxFrequency[i]         = this->MacTxFrequency[i];
        BackUpFlash.MacRx1Frequency[i]        = this->MacRx1Frequency[i];
        BackUpFlash.MacMaxDataRateChannel[i]  = this->MacMaxDataRateChannel[i];
        BackUpFlash.MacMinDataRateChannel[i]  = this->MacMinDataRateChannel[i];
        BackUpFlash.MacChannelIndexEnabled[i] = this->MacChannelIndexEnabled[i];
    }
    memcpy( &BackUpFlash.nwkSKey[0], &this->nwkSKey[0], 16);
    memcpy( &BackUpFlash.appSKey[0], &this->appSKey[0], 16);
    Crc64((uint8_t * )(&BackUpFlash), sizeof(sBackUpFlash) - 8, &crcLow, &crcHigh );
    BackUpFlash.CrcLow  = crcLow ; 
    BackUpFlash.CrcHigh = crcHigh ;
    mcu.StoreContext( &BackUpFlash, this->UserFlashAdress, ( sizeof(sBackUpFlash) >> 3 ) );  
}

template < class R >void LoraRegionsEU<R>::RegionSetBadCrcInFlash ( void ){
    uint32_t crcLow;
    uint32_t crcHigh;
    BackUpFlash.MacTxDataRate           = this->MacTxDataRate;
    BackUpFlash.MacTxPower              = this->MacTxPower;
    BackUpFlash.MacChMask               = this->MacChMask;
    BackUpFlash.MacNbTrans              = this->MacNbTrans; 
    BackUpFlash.MacRx2Frequency         = this->MacRx2Frequency; 
    BackUpFlash.MacRx2DataRate          = this->MacRx2DataRate;
    BackUpFlash.MacRx1DataRateOffset    = this->MacRx1DataRateOffset;
    BackUpFlash.MacRx1Delay             = this->MacRx1Delay;
    BackUpFlash.FcntUp                  = this->FcntUp;
    BackUpFlash.FcntDwn                 = this->FcntDwn;
    BackUpFlash.DevAddr                 = this->DevAddr;
    BackUpFlash.DevNonce                = this->DevNonce;
    BackUpFlash.JoinedStatus            = this->Phy.JoinedStatus;
    for ( int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {
        BackUpFlash.MacTxFrequency[i]         = this->MacTxFrequency[i];
        BackUpFlash.MacRx1Frequency[i]        = this->MacRx1Frequency[i];
        BackUpFlash.MacMaxDataRateChannel[i]  = this->MacMaxDataRateChannel[i];
        BackUpFlash.MacMinDataRateChannel[i]  = this->MacMinDataRateChannel[i];
        BackUpFlash.MacChannelIndexEnabled[i] = this->MacChannelIndexEnabled[i];
    }
    memcpy( &BackUpFlash.nwkSKey[0], &this->nwkSKey[0], 16);
    memcpy( &BackUpFlash.appSKey[0], &this->appSKey[0], 16);
    Crc64((uint8_t * )(&BackUpFlash), sizeof(sBackUpFlash) - 8, &crcLow, &crcHigh );
    BackUpFlash.CrcLow  = crcLow +1 ; // bad crc
    BackUpFlash.CrcHigh = crcHigh + 1;
    mcu.StoreContext( &BackUpFlash, this->UserFlashAdress, ( sizeof(sBackUpFlash) >> 3 ) );
    mcu.mwait_ms( 100 );    
}
/***********************************************************************************************/
/*                      Private  Methods                                                        */
/***********************************************************************************************/
//@notereview function a commun
template < class R >void LoraRegionsEU<R>:: TxDataRateToSfBw ( uint8_t dataRate ) {
    InsertTrace ( __COUNTER__, FileId );
    this->MacTxModulationCurrent = LORA ;
    if ( dataRate < 6 ){ 
        this->MacTxSfCurrent = 12 - dataRate ;
        this->MacTxBwCurrent = BW125 ;
    } else if ( dataRate == 6 ){ 
        this->MacTxSfCurrent = 7;
        this->MacTxBwCurrent = BW250 ;}
    else if ( dataRate == 7 ) {
        this->MacTxModulationCurrent = FSK ;
    } else {
        this->MacTxSfCurrent = 12 ;
        this->MacTxBwCurrent = BW125 ;
        DEBUG_MSG( " Invalid Datarate \n" ) ; 
    }
}
template < class R >void LoraRegionsEU<R>:: Rx2DataRateToSfBw ( uint8_t dataRate ) {
    InsertTrace ( __COUNTER__, FileId );
    if ( dataRate < 6 ){ 
        this->MacRx2SfCurrent = 12 - dataRate ;
        this->MacRx2BwCurrent = BW125 ;
    } else if ( dataRate== 6 ){ 
        this->MacRx2SfCurrent = 7;
        this->MacRx2BwCurrent = BW250 ;}
    else if ( dataRate == 7 ) {
        this->MacRx2ModulationTypeCurrent = FSK;
        //@note tbd manage fsk case }
    }
    else {
        this->MacRx2SfCurrent = 12 ;
        this->MacRx2BwCurrent = BW125 ;
        DEBUG_MSG( " Invalid Datarate \n" ) ; 
    }
}
