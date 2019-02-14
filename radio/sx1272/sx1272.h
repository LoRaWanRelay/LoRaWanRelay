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
#ifndef SX1272_H
#define SX1272_H
#include <stdint.h>
#include "Define.h"
#include "ApiMcu.h"
#include "DefineRadioPlaner.h"
/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625
#define FREQ_STEP_8                                 15625 /* FREQ_STEP<<8 */
#define RX_BUFFER_SIZE                              256

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0
#define RF_MID_BAND_THRESH                          525000000


class SX1272  {
public:
    SX1272( PinName nss, PinName reset , PinName TxRxIt, PinName RxTimeOutIt);
    ~SX1272(){};
    void ClearIrqFlagsLora( void );
    void ClearIrqFlagsFsk( void ){};
    IrqFlags_t GetIrqFlagsLora(eCrcMode crc_mode);
    IrqFlags_t GetIrqFlagsFsk( void ){ return (IrqFlags_t)(0);};
    void FetchPayloadLora( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi);
    void FetchPayloadFsk( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi){};
    void Reset( void );
    void SendLora( uint8_t *payload, uint8_t payloadSize, uint8_t SF, eBandWidth BW, uint32_t channel, int8_t power);
    void SendFsk( uint8_t *payload, uint8_t payloadSize, uint32_t channel, int8_t power){};
    void RxLora( eBandWidth BW, uint8_t SF, uint32_t channel, uint16_t TimeOutMs );


    void RxLoraWakeUp(eBandWidth BW, uint8_t SF, uint32_t channel);
    void RxLoraData(eBandWidth BW, uint8_t SF, uint32_t channel);
    void StartCad(uint32_t channel, uint8_t SF, eBandWidth BW);


    void TxLoRaGeneric( uint8_t *payload, uint8_t payloadSize, SRadioParam RadioParam) ;
    void RxLoRaGeneric( uint8_t payloadSize , SRadioParam RadioParam) ;
    
    void RxFsk(uint32_t channel, uint16_t timeout){};
    void Sleep(  bool coldStart );
    uint8_t Read( uint8_t addr ) ;
    void Write( uint8_t addr, uint8_t data );
    uint32_t Channel;

//private:
    typedef enum {
            IRQ_LR_RADIO_ALL                           = 0xFF,
            IRQ_LR_RX_TX_TIMEOUT                       = 0x80,
            IRQ_LR_RX_DONE                             = 0x40,
            IRQ_LR_CRC_ERROR                           = 0x20,
            IRQ_LR_HEADER_VALID                        = 0x10,
            IRQ_LR_TX_DONE                             = 0x08,
            IRQ_LR_CAD_DONE                            = 0x04,
            IRQ_LR_FHSS_CHANGE_CHANNEL_MASK            = 0x02,
            IRQ_LR_CAD_DETECTED_MASK                   = 0x01,
            IRQ_LR_RADIO_NONE                          = 0x00,
        } IrqLoraValues_t;

    /*!
    * \brief Calibrates the Image rejection depending of the frequency
    */
    void CalibrateImage( void );

    /*!
    * \brief Gets the last received packet status of a LoRa packet (RSSI, SNR)
    * @param [Out] RSSI average over last received packet
    * @param [Out] SNR estimation over last received packet
    * @param [Out] RSSI estimation of the LoRa signal
    */
    void GetPacketStatusLora( int16_t *pktRssi, int16_t *snr, int16_t *signalRssi );

    /*!
    * \brief Set the modulation parameters for Tx
    * @param [IN] Speading factor
    * @param [IN] Bandwith
    * @param [IN] Power
    */
    void SetModulationParamsTx( uint8_t SF, eBandWidth BW, int8_t power );
    void SetModulationParamsTxGeneric( uint8_t SF, eBandWidth BW, int8_t power, eIqMode IqMode, eCrcMode CrcMode);
    /*!
    * \brief Set the modulation parameters for Rx
    * @param [IN] Speading factor
    * @param [IN] Bandwith
    * @param [IN] TimeOut : number of symbols
    */
    void SetModulationParamsRx( uint8_t SF, eBandWidth BW, uint16_t symbTimeout );
    void SetModulationParamsRxGenric( uint8_t SF, eBandWidth BW, uint16_t symbTimeout , eIqMode IqMode );
    /*!
    * \brief Set the modulation parameters for CAD operation
    * @param [IN] SF Spreading factor
    * @param [IN] BW Bandwith
    */
    void SetModulationParamsCad(uint8_t SF, eBandWidth BW);

    /*!
    * \brief Set the RF frequency
    * @param [IN] Frequency [Hz]
    */
    void SetRfFrequency( uint32_t frequency );

    /*!
    * \brief Sets the radio in configuration mode
    * @param [IN]  mode          Standby mode to put the radio into
    */
    void SetStandby( void );
    
    /*!
    * \brief Sets the radio opmode
    * @param [IN]  opMode        mode to put the radio into
    */
    void SetOpMode( uint8_t opMode );
    
    /*!
    * \brief Write Payload inside the sx1276 fifo 
    * @param [in] *payload      Buffer of the data
    * @param [in] payloadSize   Size of the data
    */
    void SetPayload(uint8_t *payload, uint8_t payloadSize);

    /*!
    * \brief Read data from the buffer holding the payload in the radio
    * \param [out] payload       A pointer to a buffer holding the data from the radio
    * \param [in]  payloadSize   The number of byte to be read
    */
    void ReadFifo( uint8_t *buffer, uint8_t size );
    
    /*!
    * \brief  Read 1 byte from  radio registers
    * \param  [In]  address      Address of the byte to read
    * \return Read byte
    */
  

    /*!
    * \brief  Read data from radio registers
    *
    * \param  [In]  address      Address of the first byte to read
    * \param  [Out] buffer       Buffer that holds read data
    * \param  [In]  size         The number of bytes to read
    */
    void Read( uint8_t addr, uint8_t *buffer, uint8_t size );
    
    /*!
    * \brief Write data to the buffer holding the payload in the radio
    *
    * \param [in]  buffer        The data to be written (the payload)
    * \param [in]  size          The number of bytes to be written
    */
    void WriteFifo( uint8_t *buffer, uint8_t size );


    /*!
    * \brief Write several  bytes  to the radio registers
    * @param [in] address      The address of the first byte to write in the radio
    * @param [in] value        The data to be written in radio's memory
    * @param [in] size         Size of the data
    */
    void Write( uint8_t addr, uint8_t *buffer, uint8_t size );

        static uint8_t GetCrValue(const RadioCodingRate_t cr);
        static uint8_t GetBwValue(const eBandWidth bw);
        static uint8_t GetSfValue(const uint8_t SF);
        static uint8_t GetCrcValue(const eCrcMode crc);

    /*!
    * \brief  Write 1 bytes  to the radio registers
    * \param  [In]  address      
    * \return Read byte
    */

    PinName pinCS;
    PinName pinReset;
};
#endif

