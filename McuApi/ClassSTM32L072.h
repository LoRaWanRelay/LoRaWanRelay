/*

  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : STM32L4 MCU .   
                    Example of MCU class implementation based on stm32L4 + mbed library
License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#ifndef McuSTM32L072_H
#define McuSTM32L072_H
#include "stm32l0xx_hal.h"
#include "stm32l072xx.h"
#include "stdio.h"
#include "string.h"
#include "stm32l0xx_hal_iwdg.h"
#include "stm32l0xx_it.h"

typedef enum {
    LSI,
    LSE,
} eLOW_CLK_USE;

typedef enum {
    PA_0  = 0x00,
    PA_1  = 0x01,
    PA_2  = 0x02,
    PA_3  = 0x03,
    PA_4  = 0x04,
    PA_5  = 0x05,
    PA_6  = 0x06,
    PA_7  = 0x07,
    PA_8  = 0x08,
    PA_9  = 0x09,
    PA_10 = 0x0A,
    PA_11 = 0x0B,
    PA_12 = 0x0C,
    PA_13 = 0x0D,
    PA_14 = 0x0E,
    PA_15 = 0x0F,

    PB_0  = 0x10,
    PB_1  = 0x11,
    PB_2  = 0x12,
    PB_3  = 0x13,
    PB_4  = 0x14,
    PB_5  = 0x15,
    PB_6  = 0x16,
    PB_7  = 0x17,
    PB_8  = 0x18,
    PB_9  = 0x19,
    PB_10 = 0x1A,
    PB_11 = 0x1B,
    PB_12 = 0x1C,
    PB_13 = 0x1D,
    PB_14 = 0x1E,
    PB_15 = 0x1F,

    PC_0  = 0x20,
    PC_1  = 0x21,
    PC_2  = 0x22,
    PC_3  = 0x23,
    PC_4  = 0x24,
    PC_5  = 0x25,
    PC_6  = 0x26,
    PC_7  = 0x27,
    PC_8  = 0x28,
    PC_9  = 0x29,
    PC_10 = 0x2A,
    PC_11 = 0x2B,
    PC_12 = 0x2C,
    PC_13 = 0x2D,
    PC_14 = 0x2E,
    PC_15 = 0x2F,

    PD_2  = 0x32,

    PH_0  = 0x70,
    PH_1  = 0x71,

    // ADC internal channels
    ADC_TEMP = 0xF0,
    ADC_VREF = 0xF1,
    ADC_VBAT = 0xF2,

} PinName;


class McuSTM32L072 {
public :    
     McuSTM32L072 ( PinName mosi, PinName miso, PinName sclk );
    ~McuSTM32L072 ( );
    void InitMcu ( void );
    void Init_Irq ( PinName pin);
    void DisableIrq ( void ) { DisableIrqExtGpio ( ) ; LowPowerTimerDisableIrq ( ); };
    void EnableIrq  ( void ) { EnableIrqExtGpio  ( ) ; LowPowerTimerEnableIrq  ( ); };
/******************************************************************************/
/*                                Mcu Spi Api                                 */
/******************************************************************************/
    /** Create a SPI master connected to the specified pins
    *
    *  @param mosi SPI Master Out, Slave In pin
    *  @param miso SPI Master In, Slave Out pin
    *  @param sclk SPI Clock pin
    */
    void InitSpi ( );
    /** Write to the SPI Slave and return the response
    *
    *  @param value Data to be sent to the SPI slave
    *
    *  @returns
    *    Response from the SPI slave
    */
    uint8_t SpiWrite(int value);
    
    /** Configure the data transmission format
    *
    *  @param bits Number of bits per SPI frame (4 - 16)
    *  @param mode Clock polarity and phase mode (0 - 3)
    *
    * @code
    * mode | POL PHA
    * -----+--------
    *   0  |  0   0
    *   1  |  0   1
    *   2  |  1   0
    *   3  |  1   1
    * @endcode
    */
    void Spiformat(int bits, int mode = 0) { } ;
        
    /** Set the spi bus clock frequency
    *
    *  @param hz SCLK frequency in hz (default = 1MHz)
    */
    void SetSpiFrequency(int hz = 1000000) { } ;
    PinName McuMosi;
    PinName McuMiso;
    PinName McuSclk;

/******************************************************************************/
/*                                Mcu Flash Api                               */
/******************************************************************************/
     /** RestoreContext data from a flash device.  
     * 
     *  This method invokes memcpy - reads number of bytes from the address 
     * 
     *  @param buffer Buffer to write to 
     *  @param addr   Flash address to begin reading from 
     *  @param size   Size to read in bytes 
     *  @return       0 on success, negative error code on failure 
     */ 
    int RestoreContext(uint8_t *buffer, uint32_t addr, uint32_t size);

 
    /** StoreContext data to flash
     *  To be safer this function have to implement a read/check data sequence after programation 
     *  
     * 
     *  @param buffer Buffer of data to be written 
     *  @param addr   Flash Address to begin writing to,
     *  @param size   Size to write in 64 Bits,
     *  @return       0 on success, negative error code on failure 
     */ 
    int StoreContext(const void *buffer, uint32_t addr, uint32_t size); 
    

     /** RestoreContext data from a flash device.  
     * 
     *  This method invokes memcpy - reads number of bytes from the address 
     * 
     *  @param buffer Buffer to write to 
     *  @param addr   Flash address to begin reading from 
     *  @param size   Size to read in bytes 
     *  @return       0 on success, negative error code on failure 
     */ 
    int WriteFlashWithoutErase(uint8_t *buffer, uint32_t addr, uint32_t size);

 
/******************************************************************************/
/*                                Mcu RTC Api                                 */
/******************************************************************************/
    /*!
    * RtcInit Function
    * \remark must be called before any call to initiliaze the timers
    * \param [IN]   void
    * \param [OUT]  void       
    */
    void     RtcInit            ( void ) ;
    
    /*!
    * RtcGetTimeSecond : return the Current Rtc time in Second 
    * \remark is used for :
    * \remark scheduling autonomous retransmissions (for exemple NbTrans) , transmitting MAC answers , basically any delay without accurate time constraints
    * \remark also used to measure the time spent inside the LoRaWAN process for the integrated failsafe
    * \param [IN]   void
    * \param [OUT]  uint32_t RTC time in Second       
    */
    uint32_t RtcGetTimeSecond       ( void ) ;
   /*!
    * RtcGetTimeMs : return the Current Rtc time in Ms 
    * \remark is used to timestamp radio events (end of TX), will also be used for future classB
    * \remark this function may be used by the application.
    * \param [IN]   void
    * \param [OUT]  uint32_t Current RTC time in ms wraps every 49 days       
    */
    uint32_t RtcGetTimeMs  ( void ) ;
    
/******************************************************************************/
/*                                Mcu Sleep Api                               */
/******************************************************************************/
    /*!
    * A function to set the mcu in low power mode  for duration seconds
    * \remark inside this function watchdog has to be manage to not reset the mcu
    * \param [IN]   int delay 
    * \param [OUT]  void       
    */
    void GotoSleepSecond (int duration ) ;
    
        /*!
    * A function to set the mcu in low power mode  for duration in milliseconds
    * \remark 
    * \param [IN]   int delay 
    * \param [OUT]  void       
    */
    void     GotoSleepMSecond     ( int delay );
    void     WakeUpAfterDeepSleep ( void );
    void I2cInit(void);
    void I2cDeInit(void);
    HAL_StatusTypeDef I2cTransmit (uint16_t DevAddress, uint8_t *pData, uint16_t Size ) ;
    HAL_StatusTypeDef I2cReceive  (uint16_t DevAddress, uint8_t *pData, uint16_t Size ) ;
    HAL_StatusTypeDef I2cReadMem  (uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef I2cWriteMem (uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
 
/******************************************************************************/
/*                             Mcu WatchDog Api                               */
/******************************************************************************/
    /* A function to init and start the Watchdog 
    * \remark The expired period = WATCH_DOG_PERIOD_RELEASE seconds
    * \param [IN]   void  
    * \param [OUT]  void       
    */
    void WatchDogStart ( void ) ;

    /* A function to release the Watchdog 
    * \remark Application have to call this function periodically (with a period <WATCH_DOG_PERIOD_RELEASE)
    *         If not , the mcu will reset.
    * \param [IN]   void  
    * \param [OUT]  void       
    */
    void WatchDogRelease ( void ) ;
    
    
/******************************************************************************/
/*                             Mcu WatchDog Api                               */
/******************************************************************************/
/*!
    * LowPowerTimerLoRa Init
    *\remark initializes the dedicated LoRaWAN low power timer object. MCU specific.
    * \param [IN]  void
    * \param [OUT] void         
    */
    void LowPowerTimerLoRaInit ( void );
    /*!
    * LowPowerTimerLoRa AttachMsecond
    *
    * \param void (* _Func) (void *) a static method member of the current Obj
    * \param *_obj a pointer to the current objet
    * \param int delay in ms delay should be between 1ms and 16s.
    * \param [OUT] void         
    * \remark the code  Func =  _Func ; and obj  = _obj; isn't mcu dependent , and could be keep as already implemented
    * \remark starts the LoRaWAN dedicated timer and attaches the IRQ to the handling Interupt Service Routine in the LoRaWAN object.
    */
    void StartTimerMsecond       ( void (* _Func) (void *) , void * _obj, int delay) ;
    void StopTimerMsecond        ( void );
    void LowPowerTimerDisableIrq ( void );     
    void LowPowerTimerEnableIrq  ( void );     
    /*!
    *  timerISR
    * \remark    Do Not Modify 
    */
    void timerISR              ( void ) { Func(obj); };

/******************************************************************************/
/*                           Mcu Gpio Api                                     */
/******************************************************************************/
    void InitGpio              ( void );
    void DisableIrqExtGpio     ( void );
    void EnableIrqExtGpio      ( void );
    void InitGpioOut           ( PinName Pin );
    void InitGpioIn            ( PinName Pin );
    void SetValueDigitalOutPin ( PinName Pin, int Value );
    int  GetValueDigitalInPin  ( PinName Pin );
    void AttachInterruptIn     (  void (* _Funcext) (void *) , void * _objext) ;
    void AttachInterruptIn     (  void (* _Funcext) ( void ) ) { _UserFuncext = _Funcext; userIt = 1 ; };
    void DetachInterruptIn     (  void (* _Funcext) ( void ) ) { userIt = 0 ; };
    /*!
    *  ExtISR
    * \remark    Do Not Modify 
    */
    void ExtISR                ( void ) { 
        if (userIt == 0 ) { 
        Funcext(objext); 
        } else {
        _UserFuncext ();
        }; 
    };
    
/******************************************************************************/
/*                           Mcu wait                                         */
/******************************************************************************/   
    void mwait_ms    ( int delayms );
    void mwait       ( int delays ) ;
    volatile uint8_t counter;
    void waitUnderIt ( uint32_t delayus ) {
        for (uint32_t  i = 0 ; i < 3 * delayus ; i ++ ) {
            counter ++;
        }
    }; 

/******************************************************************************/
/*                           Mcu Uart Api                                     */
/******************************************************************************/
    void UartInit ( void ) ;
    void MMprint( const char *fmt, ...);
    void MMStoreBuffer ( const char *fmt, ...);
    void MMClearDebugBufferRadioPlaner ( void );
    void MMPrintBuffer ( void ) ;
/*****************************************************************************/
/*                                    Get Unique Id                          */
/*****************************************************************************/

    void GetUniqueId ( uint8_t  DevEui[8] ) {
        const char* UID = (char*)0x1FF80050;
        uint32_t uid;
        memcpy(&uid, UID, 4);
        DevEui[7] =(uint8_t)(uid&0xff);
        DevEui[6] =(uint8_t)((uid>>8)&0xFF);
        DevEui[5] =(uint8_t)((uid>>16)&0xFF);
        DevEui[4] =(uint8_t)((uid>>24)&0xFF);
        memcpy(&uid, UID + 4, 4);
        DevEui[3] =(uint8_t)(uid&0xFF);
        DevEui[2] =(uint8_t)((uid>>8)&0xFF);
        DevEui[1] =(uint8_t)((uid>>16)&0xFF);
        DevEui[0] =(uint8_t)((uid>>24)&0xFF);
    }

    uint32_t PowerConsumptionTotal;
    uint32_t PowerConsumption;
private :
    /*!
    *  Low power timer
    * \remark    Do Not Modify 
    */
    static void DoNothing (void *) { };
    void (* Func) (void *);
    void * obj;
    void (* Funcext) (void *);
    void * objext;
    void (* _UserFuncext) ( void );
    int userIt;
};

#endif
