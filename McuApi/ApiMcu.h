/*

  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : Mcu Api. 
                    To implement a new mcu , contributor have to create a new mcu class with the same prototype as McuXX class 


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#ifndef MCUXX_H
#define MCUXX_H
#include "UserDefine.h"
#ifdef BOARD_L4
    #include "ClassSTM32L4.h"
#else
    #include "ClassSTM32L072.h"
#endif
#include <string.h>
template < class R >
    class McuXX : public R{
public :    
    McuXX ( PinName mosi, PinName miso, PinName sclk ):R (mosi,miso,sclk){};
    ~McuXX ( ){};
//    void InitMcu ( void );
///******************************************************************************/
///*                                Mcu Spi Api                                 */
///******************************************************************************/
//    /** Create a SPI master connected to the specified pins
//    *
//    *  @param mosi SPI Master Out, Slave In pin
//    *  @param miso SPI Master In, Slave Out pin
//    *  @param sclk SPI Clock pin
//    */
//    void InitSpi ( );
//    /** Write to the SPI Slave and return the response
//    *
//    *  @param value Data to be sent to the SPI slave
//    *
//    *  @returns
//    *    Response from the SPI slave
//    */
//    uint8_t SpiWrite(int value);
//    
//    /** Configure the data transmission format
//    *
//    *  @param bits Number of bits per SPI frame (4 - 16)
//    *  @param mode Clock polarity and phase mode (0 - 3)
//    *
//    * @code
//    * mode | POL PHA
//    * -----+--------
//    *   0  |  0   0
//    *   1  |  0   1
//    *   2  |  1   0
//    *   3  |  1   1
//    * @endcode
//    */
//    void Spiformat(int bits, int mode = 0) { } ;
//        
//    /** Set the spi bus clock frequency
//    *
//    *  @param hz SCLK frequency in hz (default = 1MHz)
//    */
//    void SetSpiFrequency(int hz = 1000000) { } ;
//    PinName McuMosi;
//    PinName McuMiso;
//    PinName McuSclk;

///******************************************************************************/
///*                                Mcu Flash Api                               */
///******************************************************************************/
//     /** RestoreContext data from a flash device.  
//     * 
//     *  This method invokes memcpy - reads number of bytes from the address 
//     * 
//     *  @param buffer Buffer to write to 
//     *  @param addr   Flash address to begin reading from 
//     *  @param size   Size to read in bytes 
//     *  @return       0 on success, negative error code on failure 
//     */ 
//    int RestoreContext(uint8_t *buffer, uint32_t addr, uint32_t size);

// 
//    /** StoreContext data to flash
//     *  To be safer this function have to implement a read/check data sequence after programation 
//     *  
//     * 
//     *  @param buffer Buffer of data to be written 
//     *  @param addr   Flash Address to begin writing to,
//     *  @param size   Size to write in bytes,
//     *  @return       0 on success, negative error code on failure 
//     */ 
//    int StoreContext(const void *buffer, uint32_t addr, uint32_t size); 
//    
///******************************************************************************/
///*                                Mcu RTC Api                                 */
///******************************************************************************/
//    /*!
//    * RtcInit Function
//    * \remark must be called before any call to initiliaze the timers
//    * \param [IN]   void
//    * \param [OUT]  void       
//    */
//    void     RtcInit            ( void ) ;
//    
//    /*!
//    * RtcGetTimeSecond : return the Current Rtc time in Second 
//    * \remark is used for :
//    * \remark scheduling autonomous retransmissions (for exemple NbTrans) , transmitting MAC answers , basically any delay without accurate time constraints
//    * \remark also used to measure the time spent inside the LoRaWAN process for the integrated failsafe
//    * \param [IN]   void
//    * \param [OUT]  uint32_t RTC time in Second       
//    */
//    uint32_t RtcGetTimeSecond       ( void ) ;
//   /*!
//    * RtcGetTimeMs : return the Current Rtc time in Ms 
//    * \remark is used to timestamp radio events (end of TX), will also be used for future classB
//    * \remark this function may be used by the application.
//    * \param [IN]   void
//    * \param [OUT]  uint32_t Current RTC time in ms wraps every 49 days       
//    */
//    uint32_t RtcGetTimeMs  ( void ) ;
//    
///******************************************************************************/
///*                                Mcu Sleep Api                               */
///******************************************************************************/
//    /*!
//    * A function to set the mcu in low power mode  for duration seconds
//    * \remark inside this function watchdog has to be manage to not reset the mcu
//    * \param [IN]   int delay 
//    * \param [OUT]  void       
//    */
//    void GotoSleepSecond (int duration ) ;
//    
//        /*!
//    * A function to set the mcu in low power mode  for duration in milliseconds
//    * \remark 
//    * \param [IN]   int delay 
//    * \param [OUT]  void       
//    */
//    void     GotoSleepMSecond   ( int delay );
//    
///******************************************************************************/
///*                             Mcu WatchDog Api                               */
///******************************************************************************/
//    /* A function to init and start the Watchdog 
//    * \remark The expired period = WATCH_DOG_PERIOD_RELEASE seconds
//    * \param [IN]   void  
//    * \param [OUT]  void       
//    */
//    void WatchDogStart ( void ) ;

//    /* A function to release the Watchdog 
//    * \remark Application have to call this function periodically (with a period <WATCH_DOG_PERIOD_RELEASE)
//    *         If not , the mcu will reset.
//    * \param [IN]   void  
//    * \param [OUT]  void       
//    */
//    void WatchDogRelease ( void ) ;
//    
//    
///******************************************************************************/
///*                             Mcu WatchDog Api                               */
///******************************************************************************/
///*!
//    * LowPowerTimerLoRa Init
//    *\remark initializes the dedicated LoRaWAN low power timer object. MCU specific.
//    * \param [IN]  void
//    * \param [OUT] void         
//    */
//    void LowPowerTimerLoRaInit ( void );
//    /*!
//    * LowPowerTimerLoRa AttachMsecond
//    *
//    * \param void (* _Func) (void *) a static method member of the current Obj
//    * \param *_obj a pointer to the current objet
//    * \param int delay in ms delay should be between 1ms and 16s.
//    * \param [OUT] void         
//    * \remark the code  Func =  _Func ; and obj  = _obj; isn't mcu dependent , and could be keep as already implemented
//    * \remark starts the LoRaWAN dedicated timer and attaches the IRQ to the handling Interupt Service Routine in the LoRaWAN object.
//    */
//    void StartTimerMsecond     ( void (* _Func) (void *) , void * _obj, int delay) ;
//        
//    /*!
//    *  timerISR
//    * \remark    Do Not Modify 
//    */
//    void timerISR              ( void ) { Func(obj); };

///******************************************************************************/
///*                           Mcu Gpio Api                                     */
///******************************************************************************/
//    void SetValueDigitalOutPin ( PinName Pin, int Value );
//    int  GetValueDigitalInPin  ( PinName Pin );
//    void AttachInterruptIn     (  void (* _Funcext) (void *) , void * _objext) ;
//    void AttachInterruptIn     (  void (* _Funcext) ( void ) ) { _UserFuncext = _Funcext; userIt = 1 ; };
//    void DetachInterruptIn     (  void (* _Funcext) ( void ) ) { userIt = 0 ; };
//    /*!
//    *  ExtISR
//    * \remark    Do Not Modify 
//    */
//    void ExtISR                ( void ) { if (userIt == 0 ) { Funcext(objext); } else { _UserFuncext ();}; };
//    
///******************************************************************************/
///*                           Mcu wait                                         */
///******************************************************************************/   
////    void mwait   (uint16_t value) { wait ( value );};
////    void mwait_ms (uint32_t value){ wait_ms ( value );};
//private :
//    /*!
//    *  Low power timer
//    * \remark    Do Not Modify 
//    */
//    static void DoNothing (void *) { };
//    void (* Func) (void *);
//    void * obj;
//    void (* Funcext) (void *);
//    void * objext;
//    void (* _UserFuncext) ( void );
//    int userIt;
};
HAL_StatusTypeDef FLASH_If_BankSwitch(void);
    #ifdef BOARD_L4
        extern McuXX<McuSTM32L4> mcu;
    #else
        extern McuXX<McuSTM32L072> mcu;
    #endif
#endif
