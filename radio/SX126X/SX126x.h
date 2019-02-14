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
#ifndef SX126X_H
#define SX126X_H
#include <stdint.h>
#include "Define.h"
#include "ApiMcu.h"
#include "stdio.h"
#include "math.h"
#include "DefineRadioPlaner.h"
/*!
 * \brief Provides the frequency of the chip running on the radio and the frequency step
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
// TODO: Move it ?
#define RADIO_XTAL_FREQ                                   ( double )32000000
#define RADIO_FREQ_DIV                                    ( double )pow( 2.0, 25.0 )
#define RADIO_FREQ_STEP                                   ( double )( RADIO_XTAL_FREQ / RADIO_FREQ_DIV )


//! \warning: FSK is under still test and not officialy supported on this driver
class SX126x {
    public:
        SX126x( PinName Busy, PinName nss, PinName reset,PinName Interrupt );
        ~SX126x(){}; 

        void ClearIrqFlagsLora( void );
        void ClearIrqFlagsFsk( void );

        IrqFlags_t GetIrqFlagsLora( eCrcMode crc_mode );
        IrqFlags_t GetIrqFlagsFsk(  eCrcMode crc_mode );

        void FetchPayloadLora(
            uint8_t *payloadSize,
            uint8_t payload[255],
            int16_t *snr,
            int16_t *signalRssi
        );

        void FetchPayloadFsk(
            uint8_t *payloadSize,
            uint8_t payload[255],
            int16_t *snr,
            int16_t *signalRssi
        );

        void Reset( void );



        void SendFsk(
            uint8_t *payload,
            uint8_t payloadSize,
            uint32_t channel,
            int8_t power
        );

        void RxLora(
            eBandWidth   BW,
            uint8_t      SF,
            uint32_t     channel,
            uint32_t     rxTimeoutMs
        );

        void RxFsk(
            uint32_t channel,
            uint32_t     rxTiemoutMs
        );

       void Sleep( bool coldStart );
       void SendCw(uint32_t frequency);


    void RxGen(eBandWidth BW, uint8_t SF, uint32_t channel, uint16_t TimeOutMs, eIqMode IqMode ){};
    void StartCad(uint32_t channel, uint8_t SF, eBandWidth BW) {};
    void TxLoRaGeneric( uint8_t *payload, uint8_t payloadSize, SRadioParam RadioParam);
    void RxLoRaGeneric( uint8_t payloadSize , SRadioParam RadioParam);

    //private:
        typedef enum {
            SLEEP,
            AWAKE
        } radioModes_t;
        
        typedef enum {
            CALIBRATE_IMAGE         = 0x98,
            CLR_IRQ_STATUS          = 0x02,
            STOP_TIMER_ON_PREAMBLE  = 0x9F,
            SET_RFSWITCHMODE        = 0x9D,
            GET_IRQ_STATUS          = 0x12,
            GET_RX_BUFFER_STATUS    = 0x13,
            GET_PACKET_STATUS       = 0x14,
            READ_BUFFER             = 0x1E,
            READ_REGISTER           = 0x1D,
            SET_DIO_IRQ_PARAMS      = 0x08,
            SET_MODULATION_PARAMS   = 0x8B,
            SET_PA_CONFIG           = 0x95,
            SET_PACKET_PARAMS       = 0x8C,
            SET_PACKET_TYPE         = 0x8A,
            SET_RF_FREQUENCY        = 0x86,
            SET_BUFFER_BASE_ADDRESS = 0x8F,
            SET_SLEEP               = 0x84,
            SET_STANDBY             = 0x80,
            SET_RX                  = 0x82,
            SET_TX                  = 0x83,
            SET_TX_PARAMS           = 0x8E,
            WRITE_BUFFER            = 0x0E,
            WRITE_REGISTER          = 0x0D,
            SET_TXCONTINUOUSWAVE    = 0xD1,
            SET_TXCONTINUOUSPREAMBLE= 0xD2,
            GET_STATUS              = 0xC0,
            SET_REGULATORMODE       = 0x96
        } OpCode_t;
        
        typedef enum {
            REG_LORA_PAYLOAD_LENGTH = 0x0702,
            REG_LORA_SYNC_WORD_MSB  = 0x0740,
            REG_OCP                 = 0x08E7,
        } Reg_t;
        
        typedef enum {
            STDBY_RC                                = 0x00,
            STDBY_XOSC                              = 0x01,
        } StandbyModes_t;
            
        typedef enum {
            USE_LDO                                 = 0x00, // default
            USE_DCDC                                = 0x01,
        }RadioRegulatorMode_t;
        
        typedef enum {
            IRQ_RADIO_NONE                          = 0x0000,
            IRQ_TX_DONE                             = 0x0001,
            IRQ_RX_DONE                             = 0x0002,
            IRQ_PREAMBLE_DETECTED                   = 0x0004,
            IRQ_SYNCWORD_VALID                      = 0x0008,
            IRQ_HEADER_VALID                        = 0x0010,
        IRQ_HEADER_ERROR                        = 0x0020,
            IRQ_CRC_ERROR                           = 0x0040,
            IRQ_CAD_DONE                            = 0x0080,
            IRQ_CAD_ACTIVITY_DETECTED               = 0x0100,
            IRQ_RX_TX_TIMEOUT                       = 0x0200,
            IRQ_RADIO_ALL                           = 0xFFFF,
        } IrqMasks_t;
        
        typedef enum {
            IQ_STANDARD                = 0x00,
            IQ_INVERTED                = 0x01
        } InvertIQ_t;
        
        
        // Constants
        static const uint8_t LoraSyncword[2];
        
        // Attributes
        radioModes_t radioMode;
        PinName pinBusy;
        PinName pinReset;
        PinName pinCS;
        
        /*!
         * \brief Calibrates the Image rejection depending of the frequency
         * @param [IN] Operating frequency
         */
        void CalibrateImage( uint32_t frequency );
        
        /*!
         * \brief Wakeup the radio if it is in Sleep mode and check that Busy is low
         */
        void CheckDeviceReady( void );
        
        /*!
         * \brief Clears the IRQs
         * @param [IN]  irq           IRQ(s) to be cleared
         */
        void ClearIrqStatus( uint16_t irq );
        
        /*!
         * \brief Gets the last received packet status of a LoRa packet (RSSI, SNR)
         * @param [Out] RSSI average over last received packet
         * @param [Out] SNR estimation over last received packet
         * @param [Out] RSSI estimation of the LoRa signal
         */
        void GetPacketStatusLora( int16_t *pktRssi, int16_t *snr, int16_t *signalRssi );
        
        /*!
         * \brief Gets the last received packet buffer status
         * \param [out] payloadSize Last received packet payload length
         * \param [out] rxStartBuffer Last received packet buffer address pointer
         */
        void GetRxBufferStatus( uint8_t *payloadSize, uint8_t *rxStartBuffer );
        
        /*!
         * \brief Read data from the buffer holding the payload in the radio
         *
         * \param [in]  offset        The offset to start reading the payload
         * \param [out] payload       A pointer to a buffer holding the data from the radio
         * \param [in]  payloadSize   The number of byte to be read
         */
        void ReadBuffer( uint8_t offset, uint8_t *payload, uint8_t payloadSize );
        
        /*!
         * \brief Read data from SX126x
         *
         * @param [In]  offset       Command to be sent
         * @param [Out] buffer       Buffer holding data from the radio
         * @param [In]  size         Number of bytes to be read
         * \return                   Radio status
         */
        uint8_t ReadCommand( OpCode_t command, uint8_t *buffer, uint16_t size );
        
        /*!
         * \brief  Read 1 byte from  radio registers
         *
         * \param  [In]  address      Address of the byte to read
         *
         * \return Read byte
         */
        uint8_t ReadRegister( uint16_t address );
        
        /*!
         * \brief  Read data from radio registers
         *
         * \param  [In]  address      Address of the first byte to read
         * \param  [Out] buffer       Buffer that holds read data
         * \param  [In]  size         The number of bytes to read
         */
        void ReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
        
        /*!
         * \brief Sets the data buffer base address for transmission and reception
         * @param [IN]  txBaseAddress Transmission base address
         * @param [IN]  rxBaseAddress Reception base address
         */
        void SetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress );

        
        /*!
         * \brief   Sets the IRQ mask and DIO masks
         *
         * \param [IN]  irqMask       General IRQ mask
         * \param [IN]  dio1Mask      DIO1 mask
         * \param [IN]  dio2Mask      DIO2 mask
         * \param [IN]  dio3Mask      DIO3 mask
         */
        void SetDioIrqParams(
            uint16_t irqMask,
            uint16_t dio1Mask,
            uint16_t dio2Mask,
            uint16_t dio3Mask
        );

        /*!
         * \brief Set the modulation parameters for LORA
         * @param [IN] Speading factor
         * @param [IN] Bandwith
         */
        void SetModulationParamsLora(
            uint8_t    SF,
            eBandWidth BW
        );

        /*!
         * \brief Set the modulation parameters for FSK
         */
        void SetModulationParamsFsk( );

        /*!
         * \brief Sets the packet parameters for LORA
         */
        void SetPacketParamsLora(
            uint8_t payloadSize,
            InvertIQ_t IqType
        );

        /*!
         * \brief Sets the packet parameters for FSK
         */
        void SetPacketParamsFsk(
            uint8_t payloadSize
        );

        /*!
         * \brief Sets the transmission parameters
         *
         * @param [in]  paDutyCycle     Duty Cycle for the PA
         * @param [in]  hpMax           0 for sx1261, 7 for sx1262
         * @param [in]  deviceSel       1 for sx1261, 0 for sx1262
         * @param [in]  paLut           0 for 14dBm LUT, 1 for 22dBm LUT
         */

        void SetPaConfig(
            uint8_t paDutyCycle,
            uint8_t hpMax,
            uint8_t deviceSel,
            uint8_t paLut
        );
        
        /*!
         * \brief Set packet type
         * @param [IN] Modulation (LoRa/FSK)
         */
        void SetPacketType( eModulationType modulation );
        
        /*!
         * \brief Set the RF frequency
         * @param [IN] Frequency [Hz]
         */
        void SetRfFrequency( uint32_t frequency );
        
        /*!
         * \brief Sets the radio in configuration mode
         * @param [IN]  mode          Standby mode to put the radio into
         */
        void SetStandby( StandbyModes_t mode );
                   /*!
         * \brief Sets the power regulators operating mode
         * \param [in]  mode          [0: LDO, 1:DC_DC]
         */
        void SetRegulatorMode( RadioRegulatorMode_t mode );
        
        /*!
         * \brief Indicates if DIO2 is used to control an RF Switch
         *
         * \param [in] enable     true of false
         */
        void SetDio2AsRfSwitchCtrl( uint8_t enable );
        
        /*!
         * \brief Sets the radio in continuous wave transmission mode
         */
        void SetTxContinuousWave( void );
        
        /*!
         * \brief Sets the radio in continuous preamble transmission mode
         */
        void SetTxInfinitePreamble( void );
        
         /*!
         * \brief Sets the radio in reception mode
         * @param [In]  timeout       Structure describing the transmission timeout value
         */
        void SetRx( uint32_t timeout );
        
        /*!
         * \brief Sets the radio in transmission mode
         * @param [IN]  timeout       Structure describing the transmission timeout value
         */
        void SetTx( uint32_t timeout );
        
        /*!
         * \brief Sets the transmission parameters
         * @param [IN]  RF output power [-3..15] dBm
         * @param [IN]  Transmission ramp up time
         */
        void SetTxParams( int8_t power );
        
        /*!
         * \brief Sets the transmission parameters
         * @param [IN]  True: Stop Rx timer if preamble is detected
         *              False: Stop timer if FSK sync word or LoRa header is detected
         */
         void StopTimerOnPreamble( bool state );
        
        /*!
         * \brief Blocking loop to wait while the Busy pin in high
         */
        void waitOnBusy( void );
        
        /*!
         * \brief Write data to the buffer holding the payload in the radio
         *
         * \param [in]  offset        The offset to start writing the payload
         * \param [in]  buffer        The data to be written (the payload)
         * \param [in]  size          The number of bytes to be written
         */
        void WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );
        
        /*!
         * \brief Write a command to the SX126x
         * @param [IN] Operation code
         * @param [IN] In/Out buffer
         * @param [IN] Size of the buffer
         */
        void WriteCommand( OpCode_t opCode, uint8_t *buffer, uint16_t size );
        
        /*!
         * \brief Write severak bytes of data to the radio memory
         *
         * @param [in] address      The address of the first byte to write in the radio
         * @param [in] value        The data to be written in radio's memory
         * @param [in] size         Size of the data
         */
        void WriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size );
        void WriteRegister( uint16_t address, uint8_t value );

        void ConfigureCrcCCITT(void);
        void SetCrcSeedFskCCITT(void);
        void SetCrcPolynomialFskCCITT(void);
        void SetSyncWordFskLorawan(void);
};

#endif
