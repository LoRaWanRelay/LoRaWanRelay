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

#include "RegionUS.h"
#include "Define.h"
#include "utilities.h"

/*************************************************/
/*                     Constructors              */
/*************************************************/
template class LoraRegionsUS<SX1276>;
template class LoraRegionsUS<SX126x>;
template class LoraRegionsUS<SX1272>;
template < class R > LoraRegionsUS<R>::LoraRegionsUS ( sLoRaWanKeys LoRaWanKeys, RadioPLaner<R> * RadioUser, uint32_t FlashAdress) : LoraWanContainer<72,R>  ( LoRaWanKeys, RadioUser, FlashAdress ){
    
    memset( this->MacChannelIndexEnabled, CHANNEL_DISABLED, this->NUMBER_OF_CHANNEL );
    memset( this->MacMinDataRateChannel, 0, this->NUMBER_OF_CHANNEL );
    for (int i = 0 ; i < 64 ; i ++ ) {
        this->MacTxFrequency         [i] = 902300000 + (i*200000);
        this->MacMinDataRateChannel  [i] = 0;
        this->MacMaxDataRateChannel  [i] = 3;
        this->MacChannelIndexEnabled [i] = CHANNEL_ENABLED;
    }
    for (int i = 64 ; i < 72 ; i ++ ) {
        this->MacTxFrequency         [i] = 903000000 + (( i - 64 ) *1600000);
        this->MacMinDataRateChannel  [i] = 4;
        this->MacMaxDataRateChannel  [i] = 4;
        this->MacChannelIndexEnabled [i] = CHANNEL_ENABLED;
    }
    for (int i = 0 ; i < 8 ; i ++ ) {
        this->MacRx1Frequency        [i] = this->MacTxFrequency        [i]; // note tbd
    }
    this->MacTxPower           = TX_POWER;
    this->MacRx1DataRateOffset = 0;
    this->MacRx2Frequency      = 923300000; 
    this->MacRx2DataRate       = 8;
    this->MacRx1Delay          = RECEIVE_DELAY1;
    this->MacTxDataRateAdr     = 0 ;
    memset(DistriDataRateInit,0,8);
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

template < class R >void LoraRegionsUS<R>::RegionSetPower ( uint8_t PowerCmd ) {
    uint8_t PowerTab [ 11 ] = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 }; //@note check the meaning of 30dbm -2*TXpower in the standart
    if  ( PowerCmd > 10 ) {
        this->MacTxPower = 20 ;
        DEBUG_MSG ("INVALID POWER \n");
    } else {
        this->MacTxPower = PowerTab [ PowerCmd ] ;
    }
}

/********************************************************************/
/*                  Region Get Cf List                              */
/* Chapter 7.1.4 LoRaWan 1.0.1 specification                        */
/********************************************************************/
template < class R >void LoraRegionsUS<R>::RegionGetCFList ( void ) {
    // do nothing in US region   
}

/********************************************************************/
/*                  Region Set Channel MAsk                         */
/* Chapter 7.1.5 LoRaWan 1.0.3 specification                        */
/********************************************************************/

template < class R >eStatusChannel LoraRegionsUS<R>::RegionBuildChannelMask ( uint8_t ChMaskCntl, uint16_t ChMask ) {
    eStatusChannel status = OKCHANNEL;     
    return ( status );
};

template < class R >void LoraRegionsUS<R>::RegionInitChannelMask ( void ) {

};
template < class R >void LoraRegionsUS<R>::RegionSetMask ( void ) {

};
/********************************************************************/
/*                  Region MAx Payload SIze  Configuration          */
/* Chapter 7.1.6 LoRaWan 1.0.1 specification                        */
/********************************************************************/
template < class R >eStatusLoRaWan LoraRegionsUS<R>::RegionMaxPayloadSize ( uint8_t sizeIn ) {
    eStatusLoRaWan  status = OKLORAWAN;
    return ( status );
}

/********************************************************************/
/*                  Region Rx Window  Configuration                 */
/* Chapter 7.1.7 LoRaWan 1.0.1 specification                        */
/********************************************************************/
//@notereview return status
template < class R >void LoraRegionsUS<R>::RegionSetRxConfig ( eRxWinType type ) {
    uint8_t RxDataRatetmp = this->MacTxDataRate + 10 - this->MacRx1DataRateOffset ;
    RxDataRatetmp = ( RxDataRatetmp > 13 ) ? 13 : RxDataRatetmp;
    if ( type == RX1 ) {
        this->MacRx1SfCurrent = 20 - RxDataRatetmp ;
        this->MacRx1BwCurrent = BW500;
    } else if ( type == RX2 ) {
        Rx2DataRateToSfBw ( this->MacRx2DataRate );
    } else {
        DEBUG_MSG ("INVALID RX TYPE \n");
    }
}


/********************************************************************************/
/*           Check parameter of received mac commands                           */
/********************************************************************************/
template < class R >eStatusLoRaWan LoraRegionsUS<R>::RegionIsValidRx1DrOffset ( uint8_t Rx1DataRateOffset ) {
    eStatusLoRaWan status = OKLORAWAN;
    return ( status );
}

template < class R >eStatusLoRaWan LoraRegionsUS<R>:: RegionIsValidDataRate ( uint8_t temp ) {
    eStatusLoRaWan status = OKLORAWAN;
    return ( status );
}
    
template < class R >eStatusLoRaWan LoraRegionsUS<R>::RegionIsAcceptableDataRate ( uint8_t DataRate ) {
    eStatusLoRaWan status = ERRORLORAWAN;

    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsUS<R>::RegionIsValidMacFrequency ( uint32_t Frequency) {
    eStatusLoRaWan status = OKLORAWAN;

    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsUS<R>::RegionIsValidTxPower ( uint8_t Power) {
    eStatusLoRaWan status = OKLORAWAN;

    return ( status );
}
template < class R >eStatusLoRaWan LoraRegionsUS<R>::RegionIsValidChannelIndex ( uint8_t ChannelIndex) {
    eStatusLoRaWan status = OKLORAWAN;

    return ( status );
};


/********************************************************************************/
/*           RegionGiveNextDataRate                                             */
/*    method to set the next data Rate in different mode                        */
/********************************************************************************/

template < class R >void LoraRegionsUS<R>::RegionSetDataRateDistribution( uint8_t adrMode) {
    
}

template < class R >void LoraRegionsUS<R>::RegionGiveNextDataRate( void ) {
    if ( this->AdrModeSelect == STATIC_ADR_MODE ) {
        this->MacTxDataRate = this->MacTxDataRateAdr;
        this->AdrEnable = 1;
    } else {
        int i;
        uint8_t DistriSum = 0;
        for ( i= 0 ; i < 4; i++ ){  //@Note what about DR8 to DR13 
            DistriSum += DistriDataRate[i];
        }
        if ( DistriSum == 0) {
            memcpy(DistriDataRate,DistriDataRateInit,8);
        }
        uint8_t Newdr = randr(0,3);
        while (DistriDataRate[Newdr] == 0) {
            Newdr = randr(0,3);
        }
        this->MacTxDataRate = Newdr;
        DistriDataRate[Newdr] -- ;
        this->AdrEnable = 0;
    }
    this->MacTxDataRate = ( this->MacTxDataRate > 4 ) ? 4 : this->MacTxDataRate;
    TxDataRateToSfBw ( this->MacTxDataRate );
  
}

/********************************************************************************/
/*           RegionDecreaseDataRate                                             */
/*    method to update Datarate in ADR Mode                                     */
/********************************************************************************/
template < class R >void LoraRegionsUS<R>::RegionDecreaseDataRate ( void ) {
  
}


/********************************************************************************/
/*           RegionGiveNextChannel                                              */
/*    method to set the next enable channel                                     */
/********************************************************************************/
template < class R >void  LoraRegionsUS<R>::RegionGiveNextChannel( void ) {
    uint8_t NbOfActiveChannel = 0 ;
    for (int i = 0 ; i < this->NUMBER_OF_CHANNEL ; i ++ ) {  //@NOte 64 or 72 channels ?
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
        this->MacRx1FrequencyCurrent = this->MacRx1Frequency[ChannelIndex%8];
    }
 
};

template < class R >uint8_t  LoraRegionsUS<R>::RegionGetAdrAckLimit( void ) {
    return ( ADR_ACK_LIMIT );
}
template < class R >uint8_t  LoraRegionsUS<R>::RegionGetAdrAckDelay( void ) {
    return ( ADR_ACK_DELAY );
}
/***********************************************************************************************/
/*                      Private  Methods                                                        */
/***********************************************************************************************/
//@notereview function a commun
template < class R >void LoraRegionsUS<R>:: TxDataRateToSfBw ( uint8_t dataRate ) {
    this->MacTxModulationCurrent = LORA ;
    if ( dataRate < 4 ){ 
        this->MacTxSfCurrent = 10 - dataRate ;
        this->MacTxBwCurrent = BW125 ;
    } else if ( dataRate == 4 ){ 
        this->MacTxSfCurrent = 8;
        this->MacTxBwCurrent = BW500 ;}
    else if (( dataRate > 7 ) && ( dataRate < 14 )){
        this->MacTxSfCurrent = 20 - dataRate ;
        this->MacTxBwCurrent = BW500 ;
    } else {
        this->MacTxSfCurrent = 12 ;
        this->MacTxBwCurrent = BW125 ;
        DEBUG_MSG( " Invalid Datarate \n" ) ; 
    }
}
template < class R >void LoraRegionsUS<R>:: Rx2DataRateToSfBw ( uint8_t dataRate ) {
    this->MacRx2SfCurrent = 12 ;
    this->MacRx2BwCurrent = BW500 ;
}

