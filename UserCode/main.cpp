/*!
 * \file      Main4certification.c
 *
 * \brief     Description : Lorawan Stack example
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
 * \endcode

Maintainer        : Fabien Holin (SEMTECH)
*/

#include "LoraMacDataStoreInFlash.h"
#include "LoraWanProcess.h"
#include "Define.h"
#include "utilities.h"
#include "UserDefine.h"
#include "appli.h"
#include "ApiMcu.h"
#include "utilities.h"
#include "main.h"
#include "UserDefine.h"
#include "RadioPlaner.h"

#define FileId 4
/*!
 * \brief   BackUpFlash The LoraWan structure parameters save into the flash memory for failsafe restauration.
 */
struct sBackUpFlash BackUpFlash;

/*!
 * \brief   Radio Interrupt Pin declarations
 */
#ifdef MURATA_BOARD
    McuXX<McuSTM32L072> mcu ( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
#endif
#ifdef BOARD_L4
    McuXX<McuSTM32L4> mcu ( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
#endif
/*!
 * \brief   Parameters of the LoraWanKeys structure. 
 * \remark  For APB Devices only NwkSkey, AppSKey and devaddr are mandatory
 * \remark  For OTA Devices only DevEUI, AppEUI and AppKey are mandatory
 */


/* User Radio ISR routine */

int main ( ) {

    /*!
    * \brief  RtcInit , WakeUpInit, LowPowerTimerLoRaInit() are Mcu dependant . 
    */
#ifdef DEVICE_UNDER_TEST
    mainRelay ( );
#else
    uint8_t LoRaMacNwkSKeyInit[]      = { 0x22, 0x33, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    uint8_t LoRaMacAppSKeyInit[]      = { 0x11, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
    uint8_t LoRaMacAppKeyInit[]       = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xBB};
    uint8_t AppEuiInit[]              = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xff, 0x50 };
    uint8_t DevEuiInit[]              = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x33, 0x22 };    
    uint32_t LoRaDevAddrInit          = 0x26011920;
    int i;
    uint8_t UserPayloadSize ;
    uint8_t UserPayload   [255];
    uint8_t UserRxPayload [125];
    uint8_t UserRxPayloadSize;
    uint8_t UserFport ;
    uint8_t UserRxFport ;
    uint8_t MsgType ;
    uint32_t AppTimeSleeping = 10;
    sLoRaWanKeys  LoraWanKeys  = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, LoRaDevAddrInit,OTA_DEVICE };
    mcu.InitMcu ( );
    #ifdef SX126x_BOARD
    #define FW_VERSION     0x18
        SX126x  RadioUser( LORA_BUSY, LORA_CS, LORA_RESET,TX_RX_IT );
        RadioPLaner < SX126x > RP( &RadioUser );
    #endif
    #ifdef SX1276_BOARD
    #define FW_VERSION     0x17
        SX1276  RadioUser( LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT);
        RadioPLaner < SX1276 > RP( &RadioUser );
    #endif
    #ifdef SX1272_BOARD
    #define FW_VERSION     0x13
        SX1272  RadioUser( LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT);
        RadioPLaner < SX1272 > RP( &RadioUser );
    #endif
        mcu.WatchDogStart ( );
        uint8_t uid[8];
        mcu.GetUniqueId (uid); 
        memcpy(&LoraWanKeys.DevEui[0], uid , 8);
        /*!
        * \brief   Lp<LoraRegionsEU>: A LoRaWan Object with Eu region's rules. 
        * \remark  The Current implementation  support radio SX1276 and sx1261
        */
    #ifdef SX126x_BOARD
        LoraWanObject<LoraRegionsEU,SX126x> Lp ( LoraWanKeys,&RP,USERFLASHADRESS); 
    #endif
    #ifdef SX1276_BOARD
        LoraWanObject<LoraRegionsEU,SX1276> Lp ( LoraWanKeys,&RP,USERFLASHADRESS); 
    #endif
    #ifdef SX1272_BOARD
        LoraWanObject<LoraRegionsEU,SX1272> Lp ( LoraWanKeys,&RP,USERFLASHADRESS); 
    #endif
        uint8_t AvailableRxPacket  = NO_LORA_RXPACKET_AVAILABLE ;
        eLoraWan_Process_States LpState = LWPSTATE_IDLE;  
      
        DEBUG_PRINTF("MM is starting ...{ %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x } \n",uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7]);
        
        RP.InitHook ( 0 , &(Lp.packet.Phy.CallbackIsrRadio), &(Lp.packet.Phy) );
        RadioUser.Reset();
        mcu.GotoSleepMSecond ( 300 );
        /*!
        * \brief Restore the LoraWan Context
        */
        //Lp.RestoreContext  ( );
        Lp.SetDataRateStrategy ( USER_DR_DISTRIBUTION );
        UserFport       = 3;
        UserPayloadSize = 14;
        for (int i = 0 ; i < UserPayloadSize ; i++ ) {
            UserPayload [i]= i;
        }
        UserPayload [ 0 ]  = FW_VERSION ;
        MsgType = UNCONF_DATA_UP;
        Lp.NewJoin();   
        while(1) {
            if ( ( Lp.IsJoined ( ) == NOT_JOINED ) && ( Lp.GetIsOtaDevice ( ) == OTA_DEVICE) ) {       
                LpState  = Lp.Join( mcu.RtcGetTimeMs () + 200 );
            } else {
                LpState  = Lp.SendPayload( UserFport, UserPayload, UserPayloadSize, MsgType , mcu.RtcGetTimeMs () + 200);
            }

    /*!
    * \brief 
    *        This function manages the state of the MAC and performs all the computation intensive (crypto) tasks of the MAC.
    *        This function is called periodically by the user�s application whenever the state of the MAC is not idle.
    *        The function is not timing critical, can be called at any time and can be interrupted by any IRQ (including the user�s IRQ).
    *        The only requirement is that this function must be called at least once between the end of the transmission and the beginning of the RX1 slot.
    *        Therefore when the stack is active a call periodicity of roughly 300mSec is recommended.
    */ 
            DEBUG_MSG (" new packet \n");
            while ( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) ) {
                LpState = Lp.LoraWanProcess( &AvailableRxPacket );
                mcu.GotoSleepMSecond ( 400 );
                mcu.WatchDogRelease  ( );
            }
            RP.GetStatistic ( );
            if ( LpState == LWPSTATE_ERROR )  {
                InsertTrace ( __COUNTER__, FileId );
                while(1){}
                // user application have to save all the need
                NVIC_SystemReset();
            }
            if ( AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) { 
                AvailableRxPacket  = NO_LORA_RXPACKET_AVAILABLE ;
                InsertTrace ( __COUNTER__, FileId );
                Lp.ReceivePayload( &UserRxFport, UserRxPayload, &UserRxPayloadSize );
                DEBUG_PRINTF("Receive on port %d  an Applicative Downlink \n DATA[%d] = [ ",UserRxFport,UserRxPayloadSize);
                for ( i = 0 ; i < UserRxPayloadSize ; i++){
                    DEBUG_PRINTF( "0x%.2x ",UserRxPayload[i]);
                }
                DEBUG_MSG("]\n\n\n");
            }

        
    /*
    * \brief Send a �Packet every 5 seconds in case of join 
    *        Send a packet every AppTimeSleeping seconds in normal mode
    */
            if ( ( Lp.IsJoined ( ) == NOT_JOINED ) && ( Lp.GetIsOtaDevice ( ) == OTA_DEVICE) && ( LpState != LWPSTATE_INVALID)){
                InsertTrace ( __COUNTER__, FileId ); 
                mcu.GotoSleepMSecond ( 5000 );
            } else {
                InsertTrace ( __COUNTER__, FileId );
                mcu.GotoSleepSecond ( AppTimeSleeping );
                InsertTrace ( __COUNTER__, FileId );
            }
        }
#endif // ELSE DEVICE_UNDER_TEST
}





/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*

  */
void _Error_Handler( int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
        DEBUG_MSG ("ERROR\n");
    }
  /* USER CODE END Error_Handler_Debug */
}

