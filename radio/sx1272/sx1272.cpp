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

#include "sx1272.h"
#include "Define.h"
#include "UserDefine.h"
#include "stdint.h"
#include "sx1272Regs-Fsk.h"
#include "sx1272Regs-LoRa.h"


/************************************************************************************************
 *                                 Public  Methods                                              *
 ************************************************************************************************/


SX1272::SX1272( PinName nss, PinName reset, PinName TxRxIt, PinName RxTimeOutIt) : pinCS( nss ), pinReset( reset ){
    mcu.SetValueDigitalOutPin ( pinCS, 1);
    mcu.Init_Irq ( TxRxIt ) ;
    mcu.Init_Irq ( RxTimeOutIt ) ;
}

void SX1272::ClearIrqFlagsLora( void ) {
    Write ( REG_LR_IRQFLAGS, 0xFF);
}

void SX1272::FetchPayloadLora( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi) {
    uint8_t payload_size_received = Read( REG_LR_RXNBBYTES );
    if(payload_size_received>*payloadSize){
        DEBUG_PRINTF("********** Epic fail payload size (received: %i, max: %i) **********\n", payload_size_received, *payloadSize);
    }
    *payloadSize = payload_size_received;
    ReadFifo( payload, *payloadSize );
    DEBUG_PRINTF("(%i)", payload[0] + payload[1] * 256);
    GetPacketStatusLora( NULL, snr, signalRssi );
}

IrqFlags_t
SX1272::GetIrqFlagsLora(eCrcMode crc_mode)
{
  uint8_t irqFlags = 0x00;
  uint8_t returnValue = 0;
  irqFlags = Read(REG_LR_IRQFLAGS);
  volatile uint8_t header = 0x00;
  header = Read(REG_LR_HOPCHANNEL);
  if ((irqFlags & IRQ_LR_RX_TX_TIMEOUT) != 0)
  {
    returnValue = RXTIMEOUT_IRQ_FLAG;
  }
  else if((irqFlags & IRQ_LR_CAD_DONE) != 0)
  {
      if((irqFlags & IRQ_LR_CAD_DETECTED_MASK) != 0){
        returnValue = CAD_SUCCESS_IRQ_FLAG;
      }else{
        returnValue = CAD_DONE_IRQ_FLAG;
      }
  } else
  {
    if ((irqFlags & IRQ_LR_RX_DONE) != 0)
    {
      if ((irqFlags & IRQ_LR_CRC_ERROR) != 0)
      {
        returnValue = CRC_ERROR_IRQ_FLAG;
      }
      else
      {
        if(crc_mode == CRC_YES){
            if( (header & 0x40) != 0){
                returnValue = RECEIVE_PACKET_IRQ_FLAG;
            }
            else{
                returnValue = CRC_ERROR_IRQ_FLAG;
            }
        }
        else{
            returnValue = RECEIVE_PACKET_IRQ_FLAG;
        }
      }
    }
    else
    {
      if ((irqFlags & IRQ_LR_TX_DONE) != 0)
      {
        returnValue = SENT_PACKET_IRQ_FLAG;
      }
      else
      {
        returnValue = ERROR_IN_IRQ_FLAG;
      }
    }
  }
  return (IrqFlags_t)returnValue;
}

void SX1272::Reset( void ) {
    mcu.SetValueDigitalOutPin ( pinReset, 1);
    mcu.waitUnderIt( 100 );
    mcu.SetValueDigitalOutPin ( pinReset, 0);
    mcu.waitUnderIt( 5000 );
    SetOpMode( RF_OPMODE_SLEEP );
    mcu.waitUnderIt( 5000 );
    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP); // todo
}







 void SX1272::TxLoRaGeneric( uint8_t *payload, uint8_t payloadSize, SRadioParam RadioParam) {
    const uint8_t sf_value  = GetSfValue ( RadioParam.Sf );
    const uint8_t bw_value  = GetBwValue ( RadioParam.Bw );
    const uint8_t cr_value  = GetCrValue ( RadioParam.CodingRate );
    const uint8_t crc_value = GetCrcValue( RadioParam.CrcMode );
    const bool iq_inverted = (RadioParam.IqMode == IQ_INVERTED)?true:false;
    const uint8_t implicit_header_value = ( RadioParam.HeaderMode == IMPLICIT_HEADER ) ? RFLR_MODEMCONFIG1_IMPLICITHEADER_ON : RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF;

    Write(REG_OPMODE, RFLR_OPMODE_SLEEP);

    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP);
    while(Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP)){}

    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    while(Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY)){}
    #ifdef RADIO_WITH_TCX0
        mcu.SetValueDigitalOutPin ( RADIO_TCX0_POWER , 1 );
    #endif
    #ifdef RADIO_ANT_SWITCH_TX_RF0
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_TX_RF0,1);
    #endif
    #ifdef RADIO_ANT_SWITCH_RX
        mcu.SetValueDigitalOutPin(RADIO_ANT_SWITCH_RX,0);
    #endif
    SetRfFrequency(  RadioParam.Frequency );
    
    uint8_t LowDatarateOptimize;
    uint8_t ValueTemp;
    int8_t power = ( RadioParam.Power  > 20 ) ? 20 : RadioParam.Power ;

    /*  Power  Setting  registers*/
    if ( PA_BOOST_CONNECTED == 1 ) {
        power = ( power < 2 )  ? 2  : power;
        ValueTemp = ( power > 17 ) ? 0x87 : 0x84; // Enable/disable +20dbM option ON PA_BOOSt pin
        Write( REG_PADAC, ValueTemp );
        ValueTemp = ( power > 17 ) ? 0xf0 + ( power - 5 ) : 0xf0 + ( power - 2 ); 
        Write( REG_PACONFIG, ValueTemp );
    } else {
        power = ( power > 14 ) ? 14 : power;
        power = ( power < -1 ) ? -1 : power;
        //Write( REG_PADAC, 0x84 );
        Write( REG_PACONFIG, (  power + 1 ));
    }
    
     /* Set SF */
    ValueTemp =  sf_value ;
    Write( REG_LR_MODEMCONFIG2, ValueTemp ) ;
    
     /* Enable/disable Low datarate optimized */
    if( ( ( RadioParam.Bw  == 0 ) && ( (  RadioParam.Sf == 11 ) || (  RadioParam.Sf == 12 ) ) ) || ( ( RadioParam.Bw  == 1 ) && (  RadioParam.Sf == 12 ) ) ) {
        LowDatarateOptimize = 0x01;
    } else {
        LowDatarateOptimize = 0x00;
    }
     /* Set Coding rate 4/5 , Explicite Header and BW */
    ValueTemp = bw_value + cr_value + crc_value + LowDatarateOptimize + implicit_header_value;
    Write( REG_LR_MODEMCONFIG1, ValueTemp );

     /* Set Preamble = 8 */
    Write(REG_LR_PREAMBLEMSB, (uint8_t)((RadioParam.PreambuleLength  >> 8) & 0x00FF) );
    Write(REG_LR_PREAMBLELSB, (uint8_t)(RadioParam.PreambuleLength & 0x00FF));
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

/* Configure IRQ Tx Done */
    Write ( REG_LR_IRQFLAGSMASK, 0xF7 ); 
    Write ( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 );
    Write ( REG_DIOMAPPING2, 0x00 );
/* Send */
    mcu.SetValueDigitalOutPin ( DEBUG ,1 ) ;
    SetOpMode( RF_OPMODE_TRANSMITTER );
}


void SX1272::RxLoRaGeneric( uint8_t payloadSize , SRadioParam RadioParam) {
    const uint8_t sf_value  = GetSfValue ( RadioParam.Sf );
    const uint8_t bw_value  = GetBwValue ( RadioParam.Bw );
    const uint8_t cr_value  = GetCrValue ( RadioParam.CodingRate );
    const uint8_t crc_value = GetCrcValue( RadioParam.CrcMode );
    const bool iq_inverted = (RadioParam.IqMode == IQ_INVERTED)?true:false;
    const uint8_t implicit_header_value = ( RadioParam.HeaderMode == IMPLICIT_HEADER ) ? RFLR_MODEMCONFIG1_IMPLICITHEADER_ON : RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF;


  uint16_t symbTimeout = ( (  RadioParam.TimeOutMs  & 0xFFFF ) * ( ( RadioParam.Bw  + 1 ) * 125 ) ) >> RadioParam.Sf  ;
    if ( symbTimeout > 0x3FF ) {
        symbTimeout = 0x3FF ;
    }
//   DEBUG_PRINTF("symbTimeout RX: %i\n", symbTimeout);

    uint8_t LowDatarateOptimize = 0x00;
    /* Enable/disable Low datarate optimized */
    if( ( ( RadioParam.Bw == 0 ) && ( ( RadioParam.Sf == 11 ) || ( RadioParam.Sf == 12 ) ) ) || ( ( RadioParam.Bw == 1 ) && ( RadioParam.Sf == 12 ) ) ) {
        LowDatarateOptimize = RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_ON;
    } else {
        LowDatarateOptimize = RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_OFF;
    }

    // The LoRa or FSK selection is possible only in sleep mode
    // and configuration during standby.
    // So first go to Sleep (the transition time is short when entering Sleep mode).
    // Then it activate the LoRa mode and wait for its activation by polling the status register.
    // The last step jumps to standby and waits for it by polling the status register
    Write(REG_OPMODE, RFLR_OPMODE_SLEEP);

    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP);
    while(Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP)){}

    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    while(Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY)){}
    Channel = RadioParam.Frequency;
    SetRfFrequency(RadioParam.Frequency);

    uint8_t ValueTemp = LowDatarateOptimize + bw_value + cr_value + implicit_header_value + crc_value;
    Write(REG_LR_MODEMCONFIG1, ValueTemp);

    ValueTemp = sf_value + ((uint8_t)(symbTimeout>>8));
    Write(REG_LR_MODEMCONFIG2, ValueTemp);
    Write(REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0x00FF ));


    /* Set Preamble = 8 */
    Write( REG_LR_PREAMBLEMSB, (uint8_t)(RadioParam.PreambuleLength >>8) );
    Write( REG_LR_PREAMBLELSB, (uint8_t)(RadioParam.PreambuleLength  & 0x00FF) );

    if(RadioParam.HeaderMode == IMPLICIT_HEADER){
        Write(REG_LR_PAYLOADLENGTH, payloadSize);
    }

    if(iq_inverted){
        /* Set inverted IQ */
        Write( REG_LR_INVERTIQ, 0x67) ;
        Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else{
        Write( REG_LR_INVERTIQ, 0x27) ;
        Write( REG_LR_INVERTIQ2, 0x1D );
    }
    /* sensitivity optimization */
    Write( REG_LR_DETECTOPTIMIZE,( Read( REG_LR_DETECTOPTIMIZE ) & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
    Write( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12 );

     /* Set Public sync word */
    Write( REG_LR_SYNCWORD, RadioParam.SyncWord);

//   Write(REG_LR_PAYLOADLENGTH, payload_size);

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

void SX1272::SetPayload (uint8_t *payload, uint8_t payloadSize) {
// Initializes the payload size
    Write( REG_LR_PAYLOADLENGTH, payloadSize );

    // Full buffer used for Tx            
    Write( REG_LR_FIFOTXBASEADDR, 0 );
    Write( REG_LR_FIFOADDRPTR, 0 );

    // FIFO operations can not take place in Sleep mode
    if( ( Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
    {
        SetStandby( );
        mcu.mwait_ms( 1 );
    }
    // Write payload buffer
    WriteFifo( payload, payloadSize );
}


void SX1272::StartCad(uint32_t channel, uint8_t SF, eBandWidth BW)
{
//   // CalibrateImage( );
    SetOpMode(RF_OPMODE_SLEEP);
//   /* Set Lora Mode and max payload to 0x40 */
//   Write(REG_OPMODE, (Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) |
//                         RFLR_OPMODE_LONGRANGEMODE_ON);
    // Write(REG_OPMODE, RFLR_OPMODE_SLEEP);

    // Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP);
    // while(Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_SLEEP)){}

    Write(REG_OPMODE, RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY);
    while(Read(REG_OPMODE) != (RFLR_OPMODE_LONGRANGEMODE_ON + RFLR_OPMODE_STANDBY)){
    }
    
  SetRfFrequency(channel);
  SetModulationParamsCad(SF, BW);

  /* Configure IRQ CAD Done and CAD Detected */
  Write(REG_LR_IRQFLAGSMASK, 0xFA);
  Write(REG_LR_IRQFLAGS, 0xFF);
  Write(REG_DIOMAPPING1, 0x80);
  Write(REG_DIOMAPPING2, 0x00);
  /* CAD */
  mcu.SetValueDigitalOutPin ( DEBUGRX ,1 ) ;
  SetOpMode(RFLR_OPMODE_CAD);
}

void SX1272::Sleep(  bool coldStart ) {
    // DEBUG_MSG("SX1272 Sleep\n");
    SetOpMode( RF_OPMODE_SLEEP );
    // DEBUG_MSG("SX1272 End Sleep\n");
}



/************************************************************************************************
 *                                Private  Methods                                              *
 ************************************************************************************************/
void SX1272::CalibrateImage( void )
{
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


void SX1272::GetPacketStatusLora( int16_t *pktRssi, int16_t *snr, int16_t *signalRssi ) {
    
    *snr = ((int16_t) Read( REG_LR_PKTSNRVALUE ))/4; 
    int16_t rssi = (int16_t) Read( REG_LR_PKTRSSIVALUE );
    rssi += rssi / 16; 
    rssi = (Channel > RF_MID_BAND_THRESH ) ? RSSI_OFFSET_HF + rssi : RSSI_OFFSET_LF + rssi;
    *signalRssi = (*snr < 0 ) ? *snr + rssi : rssi;
}




void SX1272::SetModulationParamsCad(uint8_t SF, eBandWidth BW)
{
  uint8_t LowDatarateOptimize;
  uint8_t ValueTemp;
  /* Set SF */
  ValueTemp = (SF << 4);
  Write(REG_LR_MODEMCONFIG2, ValueTemp);

  /* Enable/disable Low datarate optimized */
  if (((BW == 0) && ((SF == 11) || (SF == 12))) || ((BW == 1) && (SF == 12)))
  {
    LowDatarateOptimize = 0x01;
  }
  else
  {
    LowDatarateOptimize = 0x00;
  }
  /* Implicit Header and BW */
  ValueTemp = (0x01 << 3) + ((BW) << 6) + 0x2 + LowDatarateOptimize + 0x4;
  Write(REG_LR_MODEMCONFIG1, ValueTemp);
  /* Set Preamble = 8 */
  Write(REG_LR_PREAMBLEMSB, 0);
  Write(REG_LR_PREAMBLELSB, 32); // 8

  /* Set inverted IQ */
  Write(REG_LR_INVERTIQ, 0x27);                // 0x67
  Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF); // RFLR_INVERTIQ2_ON
  /* sensitivity optimization */

  // Write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);

  Write(0x34, 25);
  Write(0x35, 10);
}

void SX1272::SetRfFrequency( uint32_t frequency ) {
    uint32_t initialFreqInt, initialFreqFrac;
    initialFreqInt = frequency / FREQ_STEP_8;
    initialFreqFrac = frequency - ( initialFreqInt * FREQ_STEP_8 );
    frequency = ( initialFreqInt << 8 ) + ( ( ( initialFreqFrac << 8 ) + ( FREQ_STEP_8 / 2 ) ) / FREQ_STEP_8 ); 
    Write( REG_FRFMSB, ( uint8_t )( ( frequency >> 16 ) & 0xFF ) );
    Write( REG_FRFMID, ( uint8_t )( ( frequency >> 8 ) & 0xFF ) );
    Write( REG_FRFLSB, ( uint8_t )( frequency & 0xFF ) );
}

void SX1272::SetStandby( void ) {
    Write( REG_OPMODE, ( Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_STANDBY );
}


void SX1272::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    mcu.SetValueDigitalOutPin ( pinCS,0);
    mcu.SpiWrite( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        mcu.SpiWrite( buffer[i] );
    }
    mcu.SetValueDigitalOutPin ( pinCS, 1);
}

void SX1272::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    mcu.SetValueDigitalOutPin ( pinCS, 0);
    mcu.SpiWrite( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = mcu.SpiWrite( 0 );
    }
    mcu.SetValueDigitalOutPin ( pinCS, 1);
}
void SX1272::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}
void SX1272::WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

uint8_t SX1272::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}
void SX1272::ReadFifo( uint8_t *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}

void SX1272::SetOpMode( uint8_t opMode )
{
    Write( REG_OPMODE, ( Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}


uint8_t SX1272::GetCrValue(const RadioCodingRate_t cr){
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

uint8_t SX1272::GetBwValue(const eBandWidth bw){
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
uint8_t SX1272::GetSfValue(const uint8_t SF){
    uint8_t sf_value = 0x00;
    switch(SF){
        CASE_SF_TO_VAL(6, sf_value)
        CASE_SF_TO_VAL(7, sf_value)
        CASE_SF_TO_VAL(8, sf_value)
        CASE_SF_TO_VAL(9, sf_value)
        CASE_SF_TO_VAL(10, sf_value)
        CASE_SF_TO_VAL(11, sf_value)
        CASE_SF_TO_VAL(12, sf_value)
    }
    return sf_value;
}

uint8_t SX1272::GetCrcValue(const eCrcMode crc){
    uint8_t crc_value = 0x00;
    switch(crc){
        case CRC_YES:{
            crc_value = RFLR_MODEMCONFIG1_RXPAYLOADCRC_ON;
            break;
        }
        case CRC_NO:{
            crc_value = RFLR_MODEMCONFIG1_RXPAYLOADCRC_OFF;
            break;
        }
    }
    return crc_value;
}