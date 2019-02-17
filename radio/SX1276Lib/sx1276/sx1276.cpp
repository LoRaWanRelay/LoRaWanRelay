/*

  __  __ _       _
 |  \/  (_)     (_)
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___|


Description       : LoraWan Phy Layer objets.


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Olivier Gimenez (SEMTECH)
*/

#include "sx1276.h"
#include "Define.h"
#include "UserDefine.h"
#include "stdint.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"

#define FSK_DATARATE_LORAWAN_REG_VALUE     0x280  // XTAL_FREQ / 50000
#define FSK_FDEV_MSB_LORAWAN_REG_VALUE     RF_FDEVMSB_35000_HZ
#define FSK_FDEV_LSB_LORAWAN_REG_VALUE     RF_FDEVLSB_35000_HZ
#define FSK_PREAMBLE_MSB_LORAWAN_REG_VALUE 0x00
#define FSK_PREAMBLE_LSB_LORAWAN_REG_VALUE 0x05
#define FSK_SYNCWORD_LORAWAN_REG_VALUE     0xC194C1
#define FSK_MAX_MODEM_PAYLOAD              64
#define FSK_THRESHOLD_REFILL_LIMIT         32
#define LORAWAN_MIN_PACKET_SIZE            9
#define MAX_PAYLOAD_SIZE                   255
#define FSK_FAKE_IRQ_THRESHOLD             2


/************************************************************************************************
 *                                 Public  Methods                                              *
 ************************************************************************************************/

SX1276::SX1276( PinName nss, PinName reset, PinName TxRxIt, PinName RxTimeOutIt) :
isFakeIrq(false), fakeIrqFlag(RADIO_IRQ_NONE), pinCS( nss ), pinReset( reset ), lastPacketRssi(0){
    mcu.SetValueDigitalOutPin ( pinCS, 1);
    mcu.Init_Irq ( TxRxIt ) ;
    mcu.Init_Irq ( RxTimeOutIt ) ;
    rxPayloadSize = 0;
}

void SX1276::ClearIrqFlagsLora( void ) {
    Write ( REG_LR_IRQFLAGS, 0xFF);
}

void SX1276::ClearIrqFlagsFsk( void ) {
    Write ( REG_IRQFLAGS1, 0xFF);
    Write ( REG_IRQFLAGS2, 0xFF);
}

void SX1276::FetchPayloadLora( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi) {
    *payloadSize = Read( REG_LR_RXNBBYTES );
    ReadFifo( payload, *payloadSize );
    GetPacketStatusLora( NULL, snr, signalRssi );
}

void SX1276::FetchPayloadFsk( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi) {
    *payloadSize = this->rxPayloadSize;
    memcpy(payload, this->rxBuffer, this->rxPayloadSize);
    *snr = 0;
    *signalRssi = this->lastPacketRssi;
}




IrqFlags_t SX1276::GetIrqFlagsLora( eCrcMode crc_mode ) {
    uint8_t irqFlags              = 0x00;
    uint8_t headerCrcOnPayload   = 0x00;
    irqFlags                     = Read(REG_LR_IRQFLAGS);
    headerCrcOnPayload           = Read(REG_LR_HOPCHANNEL) & 0x40;
    // Parse it
    if ( ( irqFlags & IRQ_LR_RX_TX_TIMEOUT ) !=0 ) {
        return ( RXTIMEOUT_IRQ_FLAG );
    }
    if ( ( irqFlags & IRQ_LR_CAD_DONE ) != 0) {
        IrqFlags_t tmp = ( (irqFlags & IRQ_LR_CAD_DETECTED_MASK) != 0) ? CAD_SUCCESS_IRQ_FLAG : CAD_DONE_IRQ_FLAG ;   
        return ( tmp ) ;
    }
    if ( ( irqFlags & IRQ_LR_CRC_ERROR ) !=0 ) {
        return ( CRC_ERROR_IRQ_FLAG );
    } 

    if ( ( irqFlags & IRQ_LR_RX_DONE ) !=0 ) {
        IrqFlags_t tmp = ( (crc_mode == CRC_YES) && ( headerCrcOnPayload == 0 ) ) ? CRC_ERROR_IRQ_FLAG : RECEIVE_PACKET_IRQ_FLAG ;
        return ( tmp ) ;
    }
    if ( ( irqFlags & IRQ_LR_TX_DONE ) !=0 ) {
        return ( SENT_PACKET_IRQ_FLAG);
    } else {
        return ERROR_IN_IRQ_FLAG ;
    }
   
    return (IrqFlags_t) irqFlags;
}

IrqFlags_t SX1276::GetIrqFlagsFsk( void ) {
    uint8_t irq1 = 0x00;
    uint8_t irq2 = 0x00;
    uint16_t irqs = 0x0000;
    uint16_t flags = RADIO_IRQ_NONE;
    if(this->IsFakeIRQ()){
        return this->fakeIrqFlag;
    }

    Read(REG_IRQFLAGS1, &irq1, 1);
    Read(REG_IRQFLAGS2, &irq2, 1);
    irqs = irq1<<8 | irq2;
    if ( ( irqs & IRQ_FSK_TIMEOUT ) !=0 ) {
        flags |= RXTIMEOUT_IRQ_FLAG;
    }
    if ( ( irqs & IRQ_FSK_PAYLOAD_READY ) !=0 ) {
        flags |= RECEIVE_PACKET_IRQ_FLAG;
    }
    if ( ( irqs & IRQ_FSK_PACKET_SENT ) !=0 ) {
        flags |= SENT_PACKET_IRQ_FLAG;
    }
    /* This flag cannot be set in FSK based on IRQ registers parsing
        if ( ( irqFlags & IRQ_LR_CRC_ERROR ) != 0 ) {
        irqFlags |= ERROR_IN_IRQ_FLAG;
    }*/
    
    return (IrqFlags_t) flags;
}

void SX1276::Reset( void ) {
    mcu.SetValueDigitalOutPin ( pinReset, 0);
    mcu.waitUnderIt( 1000 );
    mcu.SetValueDigitalOutPin ( pinReset, 1);
    mcu.waitUnderIt( 10000 );
    this->ResetFakeIrq();
    lastPacketRssi = 0;
    Write(REG_OPMODE, RFLR_OPMODE_SLEEP);
    SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP) ) {
        SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    }
    #ifdef RADIO_WITH_TCX0
        mcu.SetValueDigitalOutPin ( RADIO_TCX0_POWER , 0 );
    #endif
}

void SX1276::TxLoRaGeneric( uint8_t *payload, uint8_t payloadSize, SRadioParam RadioParam ) {
    const uint8_t sf_value  = GetSfValue ( RadioParam.Sf );
    const uint8_t bw_value  = GetBwValue ( RadioParam.Bw );
    const uint8_t cr_value  = GetCrValue ( RadioParam.CodingRate );
    const uint8_t crc_value = GetCrcValue( RadioParam.CrcMode );
    const bool iq_inverted  = (RadioParam.IqMode == IQ_INVERTED) ? true : false;
    const uint8_t implicit_header_value = ( RadioParam.HeaderMode == IMPLICIT_HEADER ) ? RFLR_MODEMCONFIG1_IMPLICITHEADER_ON : RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF;
    uint8_t LowDatarateOptimize;
    int8_t power = ( RadioParam.Power > 20 ) ? 20 :  RadioParam.Power;
    if( ( ( RadioParam.Bw == 0 ) && ( ( RadioParam.Sf == 11 ) || ( RadioParam.Sf == 12 ) ) ) || ( ( RadioParam.Bw == 1 ) && ( RadioParam.Sf == 12 ) ) ) {
        LowDatarateOptimize = 0x08;
    } else {
        LowDatarateOptimize = 0x00;
    }
    #ifdef RADIO_WITH_TCX0
        mcu.SetValueDigitalOutPin ( RADIO_TCX0_POWER , 1 );
    #endif
    #ifdef RADIO_ANT_SWITCH_TX_RF0
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_TX_RF0,1);
    #endif
    #ifdef RADIO_ANT_SWITCH_RX
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_RX,0);
    #endif
    Write(REG_OPMODE, RFLR_OPMODE_SLEEP);
    SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP) ) {
        SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    }
    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY)) {
        Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    }  
    SetRfFrequency( RadioParam.Frequency );
    SetPowerParamsTx( power );
    Write( REG_LR_MODEMCONFIG1, bw_value + cr_value + implicit_header_value );
    Write( REG_LR_MODEMCONFIG2, sf_value + crc_value ) ;
    Write( REG_LR_MODEMCONFIG3, LowDatarateOptimize + 4 ); // + 4 for internal AGC loop
    Write( REG_LR_PREAMBLEMSB , (uint8_t)((RadioParam.PreambuleLength >> 8) & 0x00FF) );
    Write( REG_LR_PREAMBLELSB , (uint8_t)(RadioParam.PreambuleLength & 0x00FF));
     /* Set Normal IQ */
    if(iq_inverted){
        Write( REG_LR_INVERTIQ, RFLR_INVERTIQ_TX_ON + RFLR_INVERTIQ_RX_OFF) ;
        Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else{
        Write( REG_LR_INVERTIQ, 0x27) ;
        Write( REG_LR_INVERTIQ2, 0x1D );
    }
    
     /* Set Public sync word */
    Write( REG_LR_SYNCWORD, RadioParam.SyncWord);
    Write( REG_LR_DETECTOPTIMIZE, ( Read( REG_LR_DETECTOPTIMIZE ) & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
    Write( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    SetPayload( payload, payloadSize);
/* Configure IRQ */
    Write ( REG_LR_IRQFLAGSMASK, 0xF7 ); 
    Write ( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 );
    Write ( REG_DIOMAPPING2, 0x00 );
/* Send */
    mcu.SetValueDigitalOutPin ( DEBUG ,1 ) ;
    SetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX1276::RxLoRaGeneric(uint8_t payloadSize, SRadioParam RadioParam ) {
    const uint8_t sf_value  = GetSfValue ( RadioParam.Sf );
    const uint8_t bw_value  = GetBwValue ( RadioParam.Bw );
    const uint8_t cr_value  = GetCrValue ( RadioParam.CodingRate );

    const bool iq_inverted  = (RadioParam.IqMode == IQ_INVERTED) ? true : false;
    const uint8_t implicit_header_value = ( RadioParam.HeaderMode == IMPLICIT_HEADER ) ? RFLR_MODEMCONFIG1_IMPLICITHEADER_ON : RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF;
    uint8_t LowDatarateOptimize;
    if( ( (  RadioParam.Bw  == 0 ) && ( (  RadioParam.Sf == 11 ) || (  RadioParam.Sf == 12 ) ) ) || ( (  RadioParam.Bw  == 1 ) && (  RadioParam.Sf == 12 ) ) ) {
        LowDatarateOptimize = 0x08;
    } else {
        LowDatarateOptimize = 0x00;
    }
    uint16_t symbTimeout = ( ( RadioParam.TimeOutMs & 0xFFFF ) * ( (  RadioParam.Bw  + 1 ) * 125 ) ) >> RadioParam.Sf ;
    if ( symbTimeout > 0x3FF ) {
        symbTimeout = 0x3FF ;
    }
    #ifdef RADIO_WITH_TCX0
        mcu.SetValueDigitalOutPin ( RADIO_TCX0_POWER , 1 );
    #endif
    #ifdef RADIO_ANT_SWITCH_TX_RF0
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_TX_RF0,0);
    #endif
    #ifdef RADIO_ANT_SWITCH_RX
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_RX,1);
    #endif
    Write(REG_OPMODE, RFLR_OPMODE_SLEEP);
    SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP) ) {
        SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    }
    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY)) {
        Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    }  
    SetRfFrequency( RadioParam.Frequency );
    Write( REG_LR_MODEMCONFIG1, bw_value + cr_value + implicit_header_value );
    Write( REG_LR_MODEMCONFIG2, sf_value +  (uint8_t(symbTimeout>>8) ) ) ;
    Write(REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0x00FF ));
    Write( REG_LR_MODEMCONFIG3, LowDatarateOptimize + 4 ); // + 4 for internal AGC loop
    Write( REG_LR_PREAMBLEMSB , (uint8_t)((RadioParam.PreambuleLength >> 8) & 0x00FF) );
    Write( REG_LR_PREAMBLELSB , (uint8_t)(RadioParam.PreambuleLength & 0x00FF));
     /* Set Normal IQ */
    if(RadioParam.HeaderMode == IMPLICIT_HEADER){
        Write(REG_LR_PAYLOADLENGTH, payloadSize);
    }
    if(iq_inverted){
        Write( REG_LR_INVERTIQ, 0x67) ;
        Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else{
        Write( REG_LR_INVERTIQ, 0x27) ;
        Write( REG_LR_INVERTIQ2, 0x1D );
    }
    
     /* Set Public sync word */
    Write( REG_LR_SYNCWORD, RadioParam.SyncWord);
    Write( REG_LR_DETECTOPTIMIZE, ( Read( REG_LR_DETECTOPTIMIZE ) & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
    Write( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
    if( ( RadioParam.Bw  == 2 ) && ( Channel > RF_MID_BAND_THRESH ) ) {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        Write( REG_LR_TEST36, 0x02 );
        Write( REG_LR_TEST3A, 0x64 );
        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        Write( REG_LR_DETECTOPTIMIZE, Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
    }
    else if( RadioParam.Bw  == 2 ) {
        // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
        Write( REG_LR_TEST36, 0x02 );
        Write( REG_LR_TEST3A, 0x7F );
        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        Write( REG_LR_DETECTOPTIMIZE, Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
    }
    else {
        // ERRATA 2.1 - Sensitivity Optimization
        Write( REG_LR_TEST36, 0x03 );
        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        Write( REG_LR_DETECTOPTIMIZE, Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
        Write( REG_LR_TEST30, 0x00 );
        Write( REG_LR_TEST2F, 0x40 );
    }

     /* Configure IRQ Rx Done or Rx timeout */
    Write(REG_LR_IRQFLAGSMASK, 0x1F);
    Write(REG_DIOMAPPING1, 0);
    Write(REG_DIOMAPPING2, 0x00);
     /* Configure Fifo*/
    Write(REG_LR_FIFORXBASEADDR, 0);
    Write(REG_LR_FIFOADDRPTR, 0);
     /* Receive */
    mcu.SetValueDigitalOutPin ( DEBUGRX ,1 ) ;
    SetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
}




void SX1276::SendFsk( uint8_t *payload, uint8_t payloadSize,
                      uint32_t   channel,
                      int8_t     power
                    ) {
    uint8_t payloadChunkSize;
    uint8_t bytesAlreadyIFifo = 0;
    uint8_t remainingBytes = payloadSize;
    Channel = channel;
    Reset( );
    #ifdef RADIO_ANT_SWITCH_TX_RF0
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_TX_RF0,1);
    #endif
    #ifdef RADIO_ANT_SWITCH_RX
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_RX,0);
    #endif
    CalibrateImage( );
    /* Configure FSK Tx */
    this->Sleep( false );
    SetOpModeFsk( RF_OPMODE_MODULATIONTYPE_FSK, RFLR_OPMODE_FREQMODE_ACCESS_LF, RF_OPMODE_SLEEP );
    SetRfFrequency( channel );
    SetPowerParamsTx( power );
    SetModulationParamsTxFsk( );

    if( payloadSize < FSK_MAX_MODEM_PAYLOAD ) {
        WriteFifo( &payloadSize, 1);
        WriteFifo( payload, payloadSize);
        SetOpMode( RF_OPMODE_TRANSMITTER );

        return;
    }
    else {
        SetFifoThreshold(FSK_THRESHOLD_REFILL_LIMIT);
        payloadChunkSize = FSK_THRESHOLD_REFILL_LIMIT;
        WriteFifo( &payloadSize, 1);
        WriteFifo( payload + bytesAlreadyIFifo, FSK_MAX_MODEM_PAYLOAD - 1);
        bytesAlreadyIFifo += FSK_MAX_MODEM_PAYLOAD - 1;
        remainingBytes = payloadSize - bytesAlreadyIFifo;
        SetOpMode( RF_OPMODE_TRANSMITTER );
        while( remainingBytes > payloadChunkSize ) {
            while(IsFskFifoLevelReached()){}
            WriteFifo( payload + bytesAlreadyIFifo, payloadChunkSize);
            bytesAlreadyIFifo += payloadChunkSize;
            remainingBytes = payloadSize - bytesAlreadyIFifo;
        }
        while(IsFskFifoLevelReached()){}
        WriteFifo( payload + bytesAlreadyIFifo, remainingBytes);
        return;
    }
}

void SX1276::StartCad(uint32_t channel, uint8_t SF, eBandWidth BW)
{
    const uint8_t sf_value  = GetSfValue(SF);
    const uint8_t bw_value  = GetBwValue(BW);
    uint8_t LowDatarateOptimize;
    //mcu.waitUnderIt (1000);
    const uint8_t implicit_header_value = RFLR_MODEMCONFIG1_IMPLICITHEADER_ON ;
   
      /* Enable/disable Low datarate optimized */
    if (((BW == 0) && ((SF == 11) || (SF == 12))) || ((BW == 1) && (SF == 12))) {
        LowDatarateOptimize = 0x01;
    } else {
        LowDatarateOptimize = 0x00;
    }
    #ifdef RADIO_WITH_TCX0
        mcu.SetValueDigitalOutPin ( RADIO_TCX0_POWER , 1 );
    #endif
    #ifdef RADIO_ANT_SWITCH_TX_RF0
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_TX_RF0,0);
    #endif
    #ifdef RADIO_ANT_SWITCH_RX
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_RX,1);
    #endif
    Write(REG_OPMODE, RFLR_OPMODE_SLEEP);
    SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP) ) {
        SetOpModeLora( RFLR_OPMODE_ACCESSSHAREDREG_DISABLE, RFLR_OPMODE_LONGRANGEMODE_ON, RF_OPMODE_SLEEP );
    }
    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    while ( Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY)) {
        Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    }  
    SetRfFrequency(channel);
  

    Write( REG_LR_MODEMCONFIG1, bw_value + implicit_header_value);
    Write( REG_LR_MODEMCONFIG2, sf_value  ) ;
    Write( REG_LR_MODEMCONFIG3, LowDatarateOptimize + 4 ); // + 4 for internal AGC loop
    Write(REG_LR_PREAMBLEMSB, 0);
    Write(REG_LR_PREAMBLELSB, 32); // 8

  /* Set inverted IQ */
  Write(REG_LR_INVERTIQ, 0x27);                // 0x67
  Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF); // RFLR_INVERTIQ2_ON
  /* sensitivity optimization */

  // Write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);

  Write(0x34, 25);
  Write(0x35, 10);


  /* Configure IRQ CAD Done and CAD Detected */
  Write(REG_LR_IRQFLAGSMASK, 0xFA);
  Write(REG_LR_IRQFLAGS, 0xFF);
  Write(REG_DIOMAPPING1, 0x80);
  Write(REG_DIOMAPPING2, 0x00);
  /* CAD */

  mcu.SetValueDigitalOutPin ( DEBUGRX ,1 ) ;
  SetOpMode(RFLR_OPMODE_CAD);
}

void SX1276::RxFsk(uint32_t channel, uint16_t timeOutMs) {
   
    #ifdef RADIO_ANT_SWITCH_TX_RF0
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_TX_RF0,0);
    #endif
    #ifdef RADIO_ANT_SWITCH_RX
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_RX,1);
    #endif
    volatile int CptTimeOut = 0;
     mcu.SetValueDigitalOutPin ( pinReset, 0);
    for (int i = 0 ; i <10000; i++){
        CptTimeOut++;
    }
    mcu.SetValueDigitalOutPin ( pinReset, 1);
        for (int i = 0 ; i <10000; i++){
        CptTimeOut++;
    }
        CptTimeOut = 0;

    this->ResetFakeIrq();
    lastPacketRssi = 0;
    Channel = channel;
    rxPayloadSize = 0;
    uint8_t bytesReceived = 0;
    uint8_t remainingBytes = 0;
    uint8_t LORAWAN_MIN_PACKET_SIZEt = 2; 
    uint8_t firstBytesRx[LORAWAN_MIN_PACKET_SIZE] = {0x00};
    uint8_t payloadChunkSize = FSK_THRESHOLD_REFILL_LIMIT;
    SetOpModeFsk( RF_OPMODE_MODULATIONTYPE_FSK, RFLR_OPMODE_FREQMODE_ACCESS_LF, RF_OPMODE_SLEEP );
    SetRfFrequency( channel );
    uint8_t symbTimeout = 0;
    SetModulationParamsRxFsk( symbTimeout );
    SetFifoThreshold(LORAWAN_MIN_PACKET_SIZEt - 1);
    SetOpMode( RF_OPMODE_RECEIVER );

    while(!IsFskFifoLevelReached()) {
           mcu.waitUnderIt(32000);
           CptTimeOut++;
        //if(this->HasTimeouted()) {
            if ( CptTimeOut > 5000) {
            DEBUG_MSG ("rx timeout \n");
            this->Sleep(false);
            this->SetAndGenerateFakeIRQ(RXTIMEOUT_IRQ_FLAG); 
            return;
        }
    }
    ReadFifo( &firstBytesRx[0], LORAWAN_MIN_PACKET_SIZEt );
    rxPayloadSize = firstBytesRx[0];
    DEBUG_PRINTF("rx payload size = %d \n",rxPayloadSize);
    bytesReceived = LORAWAN_MIN_PACKET_SIZEt - 1;   // -1 because the first one is the payload size, which is not included into the payload
    memcpy(rxBuffer, firstBytesRx + 1, bytesReceived);
    remainingBytes = rxPayloadSize - bytesReceived;
    SetFifoThreshold(payloadChunkSize - 1);
    while(remainingBytes > payloadChunkSize) {
        while(!IsFskFifoLevelReached()) {
             
        }
        ReadFifo( rxBuffer + bytesReceived, payloadChunkSize );
        bytesReceived += payloadChunkSize;
        remainingBytes = rxPayloadSize - bytesReceived;
    }
    SetFifoThreshold(remainingBytes - 1);
   
    while(!IsPayloadReady()) {
      
    }
    ReadFifo( rxBuffer + bytesReceived, remainingBytes );
    lastPacketRssi = this->GetCurrentRssi();
    DEBUG_PRINTF("rssi = %d \n",lastPacketRssi);
     
    this->Sleep(false);
    this->SetAndGenerateFakeIRQ(RECEIVE_PACKET_IRQ_FLAG);
}

void SX1276::Sleep(  bool coldStart ) {
    SetOpMode( RF_OPMODE_SLEEP );
    #ifdef RADIO_WITH_TCX0
        mcu.SetValueDigitalOutPin ( RADIO_TCX0_POWER , 0 );
    #endif
}

/************************************************************************************************
 *                                Private  Methods                                              *
 ************************************************************************************************/
void SX1276::SetAndGenerateFakeIRQ(IrqFlags_t fakeIrqFlag ) {
    this->isFakeIrq = true;
    this->fakeIrqFlag = fakeIrqFlag;
    this->generateFakeIrq();
}

void SX1276::ResetFakeIrq(void) {
    this->isFakeIrq = false;
    this->fakeIrqFlag = RADIO_IRQ_NONE;
}

bool SX1276::IsFakeIRQ(void) {
    if(this->isFakeIrq == true ){
        this->isFakeIrq = false;
        return true;
    }
    return false;
}

void SX1276::generateFakeIrq(void) {
    Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_10 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_10 | RF_DIOMAPPING1_DIO3_01 );
    SetFifoThreshold(FSK_FAKE_IRQ_THRESHOLD);
    WriteFifo(this->rxBuffer, FSK_FAKE_IRQ_THRESHOLD + 1);
    Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_10 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_10 | RF_DIOMAPPING1_DIO3_01 );
}

void SX1276::CalibrateImage( void ) {
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = Read( REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )this->Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )this->Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )this->Read( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    this->Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    Write ( REG_IMAGECAL, ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SetRfFrequency( 868000000 );

    // Launch Rx chain calibration for HF band
    Write ( REG_IMAGECAL, ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    this->Write( REG_PACONFIG, regPaConfigInitVal );
    SetRfFrequency( initialFreq );
}

bool SX1276::IsFskFifoLevelReached() {
    uint8_t irqFlags = 0x00;
    Read(REG_IRQFLAGS2, &irqFlags, 1);
    return (irqFlags & 0x20);
}

bool SX1276::IsPayloadReady() {
    uint8_t irqFlags = 0x00;
    Read(REG_IRQFLAGS2, &irqFlags, 1);
    return (irqFlags & 0x04);
}

bool SX1276::IsFskFifoEmpty() {
    uint8_t irqFlags = 0x00;
    Read(REG_IRQFLAGS2, &irqFlags, 1);
    return (irqFlags & 0x40);
}

bool SX1276::HasTimeouted() {
    uint8_t irqFlags = 0x00;
    Read(REG_IRQFLAGS1, &irqFlags, 1);
    return (irqFlags & 0x04);
}

void SX1276::GetPacketStatusLora( int16_t *pktRssi, int16_t *snr, int16_t *signalRssi ) {
    *snr = ((int16_t) Read( REG_LR_PKTSNRVALUE ))/4;
    int16_t rssi = (int16_t) Read( REG_LR_PKTRSSIVALUE );
    rssi += rssi / 16;
    rssi = (Channel > RF_MID_BAND_THRESH ) ? RSSI_OFFSET_HF + rssi : RSSI_OFFSET_LF + rssi;
    *signalRssi = (*snr < 0 ) ? *snr + rssi : rssi;
}

void SX1276::SetPowerParamsTx( int8_t power ) {
    uint8_t ValueTemp;
    power = ( power > 20 ) ? 20 : power;
    /*  Power  Setting  registers*/
    if ( PA_BOOST_CONNECTED == 1 ) {
        power = ( power > 20 ) ? 20 : power;
        power = ( power < 2 )  ? 2  : power;
        ValueTemp = ( power > 17 ) ? 0x87 : 0x84; // Enable/disable +20dbM option ON PA_BOOSt pin
        Write( REG_PADAC, ValueTemp );
        ValueTemp = ( power > 17 ) ? 0xf0 + ( power - 5 ) : 0xf0 + ( power - 2 );
        Write( REG_PACONFIG, ValueTemp );
    } else {
        power = ( power > 14 ) ? 14 : power;
        power = ( power < -1 ) ? -1 : power;
        Write( REG_PADAC, 0x84 );
        Write( REG_PACONFIG, ( 0x70 +  power + 1 ));
    }
}

void SX1276::SetModulationParamsCommonFsk( ) {
    // Set Bitrate
    Write( REG_BITRATEMSB, ( uint8_t )( FSK_DATARATE_LORAWAN_REG_VALUE >> 8 ) );
    Write( REG_BITRATELSB, ( uint8_t )( FSK_DATARATE_LORAWAN_REG_VALUE & 0xFF ) );

    // Set Fdev
    Write( REG_FDEVMSB, FSK_FDEV_MSB_LORAWAN_REG_VALUE );
    Write( REG_FDEVLSB, FSK_FDEV_LSB_LORAWAN_REG_VALUE );

    // Set Preamble
    Write( REG_PREAMBLEMSB, FSK_PREAMBLE_MSB_LORAWAN_REG_VALUE );
    Write( REG_PREAMBLELSB, FSK_PREAMBLE_LSB_LORAWAN_REG_VALUE );

    // Set sync config
    Write( REG_SYNCCONFIG, RF_SYNCCONFIG_AUTORESTARTRXMODE_OFF | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA |
                           RF_SYNCCONFIG_SYNC_ON | RF_SYNCCONFIG_SYNCSIZE_3
    );

    // Set Packet Config 1
    Write( REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_WHITENING |
                              RF_PACKETCONFIG1_CRC_ON | RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF |
                              RF_PACKETCONFIG1_ADDRSFILTERING_OFF | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT
    );

      // Set Packet Config 2
    Write( REG_PACKETCONFIG2, RF_PACKETCONFIG2_DATAMODE_PACKET | RF_PACKETCONFIG2_IOHOME_OFF |
                              RF_PACKETCONFIG2_BEACON_OFF
    );

    // Set Sync Values
    Write( REG_SYNCVALUE1, (FSK_SYNCWORD_LORAWAN_REG_VALUE >> 16) & 0x0000FF );
    Write( REG_SYNCVALUE2, (FSK_SYNCWORD_LORAWAN_REG_VALUE >> 8) & 0x0000FF );
    Write( REG_SYNCVALUE3, FSK_SYNCWORD_LORAWAN_REG_VALUE & 0x0000FF );
}

void SX1276::SetFifoThreshold(uint8_t threshold){
    Write( REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY | (threshold & ~RF_FIFOTHRESH_FIFOTHRESHOLD_MASK));
}

void SX1276::SetModulationParamsTxFsk( ) {
    this->SetModulationParamsCommonFsk();
    Write( REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY | FSK_THRESHOLD_REFILL_LIMIT);
    Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_01 );
    Write( REG_DIOMAPPING2, 0x00 );
}

void SX1276::SetModulationParamsRxFsk( uint8_t symbTimeout ) {
    this->SetModulationParamsCommonFsk();
    this->ConfigureRssi();
    Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_10 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_10 | RF_DIOMAPPING1_DIO3_01 );
    Write( REG_DIOMAPPING2, 0x00 );
    Write( REG_RXCONFIG, RF_RXCONFIG_RESTARTRXONCOLLISION_OFF |
                         RF_RXCONFIG_AFCAUTO_OFF | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );
    Write( REG_RXBW, RF_RXBW_MANT_20 | RF_RXBW_EXP_2 );   // 100 kHz
    Write( REG_AFCBW, RF_RXBW_MANT_24 | RF_RXBW_EXP_2 );  // 83.3 kHz
    Write( REG_PREAMBLEDETECT, RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 | RF_PREAMBLEDETECT_DETECTORTOL_10 );
    Write( REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON );
    Write( REG_LNA, RF_LNA_GAIN_G1 | RF_LNA_BOOST_ON );
    Write( REG_PAYLOADLENGTH, 0xFF );
    Write( REG_RXTIMEOUT2, symbTimeout );
}



void SX1276::SetPayload (uint8_t *payload, uint8_t payloadSize) {
    // Initializes the payload size
    Write( REG_LR_PAYLOADLENGTH, payloadSize );

    // Full buffer used for Tx
    Write( REG_LR_FIFOTXBASEADDR, 0 );
    Write( REG_LR_FIFOADDRPTR, 0 );

    // FIFO operations can not take place in Sleep mode
    if( ( Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP ) {
        SetStandby( );
        mcu.mwait_ms( 1 );
    }
    // Write payload buffer
    WriteFifo( payload, payloadSize );
}

void SX1276::SetRfFrequency( uint32_t frequency ) {
    uint32_t initialFreqInt, initialFreqFrac;
    initialFreqInt = frequency / FREQ_STEP_8;
    initialFreqFrac = frequency - ( initialFreqInt * FREQ_STEP_8 );
    frequency = ( initialFreqInt << 8 ) + ( ( ( initialFreqFrac << 8 ) + ( FREQ_STEP_8 / 2 ) ) / FREQ_STEP_8 );
    Write( REG_FRFMSB, ( uint8_t )( ( frequency >> 16 ) & 0xFF ) );
    Write( REG_FRFMID, ( uint8_t )( ( frequency >> 8 ) & 0xFF ) );
    Write( REG_FRFLSB, ( uint8_t )( frequency & 0xFF ) );
}

void SX1276::SetStandby( void ) {
    Write( REG_OPMODE, ( Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_STANDBY );
}

void SX1276::Write( uint8_t addr, uint8_t *buffer, uint8_t size ) {
    uint8_t i;

    mcu.SetValueDigitalOutPin ( pinCS,0);
    mcu.SpiWrite( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        mcu.SpiWrite( buffer[i] );
    }
    mcu.SetValueDigitalOutPin ( pinCS, 1);
}

void SX1276::Read( uint8_t addr, uint8_t *buffer, uint8_t size ) {
    uint8_t i;

    mcu.SetValueDigitalOutPin ( pinCS, 0);
    mcu.SpiWrite( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = mcu.SpiWrite( 0 );
    }
    mcu.SetValueDigitalOutPin ( pinCS, 1);
}

void SX1276::Write( uint8_t addr, uint8_t data ) {
    Write( addr, &data, 1 );
}

void SX1276::WriteFifo( uint8_t *buffer, uint8_t size ) {
    Write( 0, buffer, size );
}

uint8_t SX1276::Read( uint8_t addr ) {
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1276::ReadFifo( uint8_t *buffer, uint8_t size ) {
    Read( 0, buffer, size );
}

void SX1276::SetOpModeLora( uint8_t accessSharedReg, uint8_t lowFrequencyModeOn, uint8_t opMode ) {
    Write( REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON | accessSharedReg | lowFrequencyModeOn | opMode );
}

void SX1276::SetOpModeFsk( uint8_t modulationType, uint8_t lowFrequencyModeOn, uint8_t opMode ) {
    Write( REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_OFF | modulationType | lowFrequencyModeOn | opMode );
}

void SX1276::SetOpMode( uint8_t opMode ) {
    Write( REG_OPMODE, ( Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1276::ConfigureRssi(void){
    Write(REG_RSSICONFIG, RF_RSSICONFIG_OFFSET_P_00_DB | RF_RSSICONFIG_SMOOTHING_256);
}

int8_t SX1276::GetCurrentRssi(void){
    uint8_t regVal = 0x00;
    Read(REG_RSSIVALUE, &regVal, 1);
    return -(regVal >> 1);
}

uint8_t SX1276::GetCrValue(const RadioCodingRate_t cr){
    uint8_t cr_value = 0x00;
    switch(cr){
        case CR_4_5:{
            cr_value = RFLR_MODEMCONFIG1_CODINGRATE_4_5;
            break;
        }
        case CR_4_6:{
            cr_value = RFLR_MODEMCONFIG1_CODINGRATE_4_6;
            break;
        }
        case CR_4_7:{
            cr_value = RFLR_MODEMCONFIG1_CODINGRATE_4_7;
            break;
        }
        case CR_4_8:{
            cr_value = RFLR_MODEMCONFIG1_CODINGRATE_4_8;
            break;
        }
        default:
            cr_value = 0x00;
    }
    return cr_value;
}

uint8_t SX1276::GetBwValue(const eBandWidth bw){
    uint8_t bw_value = 0x00;
    switch(bw){
        case BW125:{
            bw_value = RFLR_MODEMCONFIG1_BW_125_KHZ;
            break;
        }
        case BW250:{
            bw_value = RFLR_MODEMCONFIG1_BW_250_KHZ;
            break;
        }
        case BW500:{
            bw_value = RFLR_MODEMCONFIG1_BW_500_KHZ;
            break;
        }
        default:
            bw_value = 0x00;
    }
    return bw_value;
}

#define CASE_SF_TO_VAL(x, val) case x:{                             \
                                   val =  RFLR_MODEMCONFIG2_SF_##x; \
                                   break;                           \
                               }
uint8_t SX1276::GetSfValue( const uint8_t SF ) {
    uint8_t sf_value = 0x00;
    switch(SF){
        CASE_SF_TO_VAL(6,  sf_value)
        CASE_SF_TO_VAL(7,  sf_value)
        CASE_SF_TO_VAL(8,  sf_value)
        CASE_SF_TO_VAL(9,  sf_value)
        CASE_SF_TO_VAL(10, sf_value)
        CASE_SF_TO_VAL(11, sf_value)
        CASE_SF_TO_VAL(12, sf_value)
    }
    return sf_value;
}

uint8_t SX1276::GetCrcValue(const eCrcMode crc) {
    uint8_t crc_value = 0x00;
    switch( crc ) {
        case CRC_YES:{
            crc_value = RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON;
            break;
        }
        case CRC_NO:{
            crc_value = RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF;
            break;
        }
    }
    return crc_value;
}