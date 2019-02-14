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
#ifndef SX1276_H
#define SX1276_H
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


class SX1276 {
    public:
        SX1276( PinName nss, PinName reset , PinName TxRxIt, PinName RxTimeOutIt);
        ~SX1276(){};
        void ClearIrqFlagsLora( void );
        void ClearIrqFlagsFsk( void );
        IrqFlags_t GetIrqFlagsLora( eCrcMode crc_mode );
        IrqFlags_t GetIrqFlagsFsk( void );
        void FetchPayloadLora( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi);
        void FetchPayloadFsk( uint8_t *payloadSize, uint8_t payload[255], int16_t *snr, int16_t *signalRssi);
        void Reset( void );
        void SendFsk( uint8_t *payload, uint8_t payloadSize, uint32_t channel, int8_t power);
        void RxFsk(uint32_t channel, uint16_t timeout);
        void Sleep(  bool coldStart );
        uint32_t Channel;
        void StartCad(uint32_t channel, uint8_t SF, eBandWidth BW) ;
        void TxLoRaGeneric( uint8_t *payload, uint8_t payloadSize, SRadioParam RadioParam) ;
        void RxLoRaGeneric( uint8_t payloadSize , SRadioParam RadioParam) ;
    private:
        uint8_t rxBuffer[255];
        uint8_t rxPayloadSize;
        bool isFakeIrq;
        IrqFlags_t fakeIrqFlag;

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

        typedef enum {
            IRQ_FSK_MODE_READY                         = 0x8000,
            IRQ_FSK_RX_READY                           = 0x4000,
            IRQ_FSK_TX_READY                           = 0x2000,
            IRQ_FSK_PLL_LOCK                           = 0x1000,
            IRQ_FSK_RSSI                               = 0x0800,
            IRQ_FSK_TIMEOUT                            = 0x0400,
            IRQ_FSK_PREAMBLE_DETECTED                  = 0x0200,
            IRQ_FSK_SYNC_ADDRESS_MATCH                 = 0x0100,
            IRQ_FSK_FIFO_FULL                          = 0x0080,
            IRQ_FSK_FIFO_EMPTY                         = 0x0040,
            IRQ_FSK_FIFO_LEVEL                         = 0x0020,
            IRQ_FSK_FIFO_OVERRUN                       = 0x0010,
            IRQ_FSK_PACKET_SENT                        = 0x0008,
            IRQ_FSK_PAYLOAD_READY                      = 0x0004,
            IRQ_FSK_CRC_OK                             = 0x0002,
            IRQ_FSK_LOW_BATTERY                        = 0x0001,
        } IrqFskValues_t;

        /*!
        * \brief Indicate if the FIFO threshold level has been reached
        */
        bool IsFskFifoLevelReached( void );

        /*!
         * \brief Indicate if the FIFO is empty
         */
        bool IsFskFifoEmpty( void );

        /*!
         * \brief Indicate if the RX operation has timeouted by reading IRQ buffer, not
         * by reading an IRQ line
         */
        bool HasTimeouted( void );

        bool IsPayloadReady(void);

        int8_t GetCurrentRssi(void);
        void ConfigureRssi(void);

        /*!
         * \brief Make the radio generate an IRQ with reason fakeIrqFlag
         * \param [IN] fakeIrqFlag   The fake reason for the radio to generate the IRQ
         */
        void SetAndGenerateFakeIRQ(IrqFlags_t fakeIrqFlag );

        /*!
         * \brief Indicate if the last IRQ was a fake one or not
         */
        bool IsFakeIRQ(void);

        /*!
         * \brief Reset the Fake IRQ flag and Fake IRQ reason
         */
        void ResetFakeIrq(void);

        /*!
         * \brief Make the radio generate an IRQ
         */
        void generateFakeIrq(void);

        /*!
         * \brief Set the threshold value for the threshold level detection mechanism
         */
        void SetFifoThreshold(uint8_t threshold);

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
        * \brief Set the power parameters for Tx (modem independant)
        * @param [IN] Power
        */
            void SetPowerParamsTx( int8_t power );

        /*!
        * \brief Set the modulation parameters for Lora Tx
        * @param [IN] Speading factor
        * @param [IN] Bandwith
            * @see SX1276::SetPowerParamsTx, SX1276::SetRfFrequency
        */
        void SetModulationParamsTxLora( uint8_t SF, eBandWidth BW );

        /*!
        * \brief Set the modulation parameters for FSK Tx
            * @see SX1276::SetPowerParamsTx, SX1276::SetRfFrequency
        */
            void SetModulationParamsTxFsk( void );

            //void SetModulationParamsRxFsk( uint8_t symbTimeout );
            void SetModulationParamsCommonFsk( void );

        /*!
        * \brief Set the modulation parameters for Rx with Lora
        * @param [IN] Speading factor
        * @param [IN] Bandwith
        * @param [IN] TimeOut : number of symbols
        */
        void SetModulationParamsRxLora( uint8_t SF, eBandWidth BW, uint16_t symbTimeout );
        void SetModulationParamsRxGeneric( uint8_t SF, eBandWidth BW, uint16_t symbTimeout , eIqMode IqMode );
        /*!
        * \brief Set the modulation parameters for Rx with FSK
        * @param [IN] symbTimeout : number of symbols before raising the timeout interrupt
        */
            void SetModulationParamsRxFsk( uint8_t symbTimeout );

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
        * \brief Sets the radio opmode for Lora operations
            * \param [IN] accessSharedReg
            * \param [IN] lowFrequencyModeOn
            * \param [IN] opMode
        */
        void SetOpModeLora( uint8_t accessSharedReg, uint8_t lowFrequencyModeOn, uint8_t opMode );

        /*!
        * \brief Sets the radio opmode for FSK operations
            * \param [IN] modulationType
            * \param [IN] lowFrequencyModeOn
            * \param [IN] opMode
        */
            void SetOpModeFsk( uint8_t modulationType, uint8_t lowFrequencyModeOn, uint8_t opMode );

        /*!
        * \brief Sets the radio opmode for FSK operations
            * TODO
            * @param [IN]  modulationType      The modulation scheme to be used for FSK/OOK:
            * @param [IN]  lowFrequencyModeOn  Access Low Frequency mode registers. Powwible values
            *                                    - RFLR_OPMODE_FREQMODE_ACCESS_LF
            *                                    - RFLR_OPMODE_FREQMODE_ACCESS_HF
        * @param [IN]  opMode              mode to put the radio into
        */
        //void SetOpModeFsk( uint8_t modulationType, uint8_t lowFrequencyModeOn, uint8_t opMode );

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
        uint8_t Read( uint8_t addr ) ;

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

        /*!
        * \brief  Write 1 bytes  to the radio registers
        * \param  [In]  address
        * \return Read byte
        */
        void Write( uint8_t addr, uint8_t data );
        PinName pinCS;
        PinName pinReset;
        int8_t lastPacketRssi;
        uint8_t GetCrValue(const RadioCodingRate_t cr);
        uint8_t GetBwValue(const eBandWidth bw);
        uint8_t GetSfValue(const uint8_t SF);
        uint8_t GetCrcValue(const eCrcMode crc);
};
#endif

