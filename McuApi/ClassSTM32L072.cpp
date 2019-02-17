/*
 __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : Flash Api.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#include "ClassSTM32L072.h"
#include "stdint.h"
#include "ApiMcu.h"
#include "stm32l0xx_hal.h"
#include "time.h"
#include "UserDefine.h"
#include "Define.h"
#include "stdint.h"


#if DEBUG_TRACE == 1
    #include <stdarg.h>
    #include <string.h>
#endif

#define WATCH_DOG_PERIOD_RELEASE 20 // this period have to be lower than the Watch Dog period of 32 seconds


I2C_HandleTypeDef hi2c1;




/********************************************************************/
/*                    SystemClock_Config functions                  */
/********************************************************************/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  if ( UART_NUM == USART1 ) { 
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  } else {
       PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPTIM1;
       PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  }
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
   
  }
  

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/********************************************************************/
/*                           Flash local functions                  */
/********************************************************************/



/********************************************************************/
/*                         Wake Up local functions                  */
/********************************************************************/
static RTC_HandleTypeDef RtcHandle;


/*!
 * Irq Handler dedicated for wake up It
 * 
 * \param [IN]  void
 * \param [OUT] void         
 */




/*!
* WakeUpAlarmMSecond : Configures the application wake up timer with a delay duration in ms
 * When the timer expires , the rtc block generates an It to wake up the Mcu 
 * \remark this function is not used by the LoRaWAN object, only provided for application purposes.
 * \param [IN]  int delay in ms
 * \param [OUT] void         
 */

void WakeUpAlarmMSecond ( int delay) {
    int DelayMs2tick = delay * 2 + ( ( 6 * delay ) >> 7);
    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, DelayMs2tick, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}
/*!
* WakeUpAlarmMecond : Configure the wake up timer with a delay duration in second
 * \remark this function is not used by the LoRaWAN object, only provided for application purposes.
 * When the timer expires , the rtc block generates an It to wake up the Mcu 
 * 
 * \param [IN]  int delay in s
 * \param [OUT] void         
 */
void WakeUpAlarmSecond ( int delay) {
    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, delay, 4);
}


/*************************************************************/
/*           Mcu Object Definition Constructor               */
/*************************************************************/
McuSTM32L072::McuSTM32L072(PinName mosi, PinName miso, PinName sclk )  {
    Func = DoNothing; // don't modify
    obj = NULL;       // don't modify
    McuMosi = mosi;   // don't modify
    McuMiso = miso;   // don't modify
    McuSclk = sclk;   // don't modify
}     


McuSTM32L072::~McuSTM32L072(){
      // to be completed by mcu providers
} 

/*******************************************/
/*                  Mcu Init               */
/*******************************************/
void McuSTM32L072::InitMcu( void ) {
    // system clk Done with mbed to be completed by mcu providers if mbed is removed
      /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  InitGpio ( );
  UartInit();
  LowPowerTimerLoRaInit ( );
  InitSpi () ;
  RtcInit ();
  I2cInit ();
  PowerConsumptionTotal = 0;
  PowerConsumption = RtcGetTimeMs ();

}

/*******************************************/
/*            GPIO Init                    */
/*******************************************/
void McuSTM32L072::InitGpio ( ) {
      /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    InitGpioOut ( LORA_CS );
    InitGpioOut ( LORA_RESET );
    InitGpioOut ( RADIO_ANT_SWITCH_RX );
    InitGpioOut ( RADIO_ANT_SWITCH_TX_RF0 );
    InitGpioOut ( RADIO_ANT_SWITCH_TX_BOOST );
    InitGpioOut ( RADIO_TCX0_POWER          );
    InitGpioOut (DEBUG);
    SetValueDigitalOutPin ( DEBUG , 0 );
    InitGpioOut (DEBUGRX);
    SetValueDigitalOutPin ( DEBUGRX , 0 );
    SetValueDigitalOutPin ( RADIO_TCX0_POWER , 1 );
    SetValueDigitalOutPin ( RADIO_ANT_SWITCH_TX_RF0, 1 ) ;
    SetValueDigitalOutPin ( RADIO_ANT_SWITCH_TX_BOOST, 0 ) ;
    SetValueDigitalOutPin ( RADIO_ANT_SWITCH_RX, 0 ) ;   
}
void McuSTM32L072::InitGpioOut( PinName Pin ){
     /*Configure GPIO pin Output Level */

    int port = ( Pin & 0xF0 ) >> 4 ;
      /*Configure GPIO pin : PB6   */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin   = (1 << ( Pin & 0x0F ) );
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    switch (port) {
        case 0 : 
            HAL_GPIO_WritePin(GPIOA, (1 << ( Pin & 0x0F ) ), GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        case 1 : 
            HAL_GPIO_WritePin(GPIOB, (1 << ( Pin & 0x0F ) ), GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        case 2 : 
            HAL_GPIO_WritePin(GPIOC, (1 << ( Pin & 0x0F ) ), GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        default:
            break;
    }
}
void McuSTM32L072::InitGpioIn( PinName Pin ){
     /*Configure GPIO pin Output Level */

    int port = ( Pin & 0xF0 ) >> 4 ;
      /*Configure GPIO pin : PB6   */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin   = (1 << ( Pin & 0x0F ) );
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    switch (port) {
        case 0 : 
            HAL_GPIO_WritePin(GPIOA, (1 << ( Pin & 0x0F ) ), GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        case 1 : 
            HAL_GPIO_WritePin(GPIOB, (1 << ( Pin & 0x0F ) ), GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        case 2 : 
            HAL_GPIO_WritePin(GPIOC, (1 << ( Pin & 0x0F ) ), GPIO_PIN_RESET);
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        default:
            break;
    }
}


void McuSTM32L072::SetValueDigitalOutPin ( PinName Pin, int Value ){
    int port = ( Pin & 0xF0 ) >> 4 ;
      /*Configure GPIO pin : PB6   */
    switch (port) {
        case 0 : 
            HAL_GPIO_WritePin(GPIOA, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
            break;
        case 1 : 
             HAL_GPIO_WritePin(GPIOB, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
            break;
        case 2 : 
             HAL_GPIO_WritePin(GPIOC, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
             
            break;
        default:
            break;
    }
}
int McuSTM32L072::GetValueDigitalInPin ( PinName Pin ){
      int port = ( Pin & 0xF0 ) >> 4 ;
      /*Configure GPIO pin : PB6   */
    switch (port) {
        case 0 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOA, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        case 1 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOB, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        case 2 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOC, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        default:
            return ( (int) HAL_GPIO_ReadPin(GPIOA, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
    }
};
void  McuSTM32L072::Init_Irq ( PinName pin) {

    int port = ( pin & 0xF0 ) >> 4 ;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin =  (1 << ( pin & 0x0F ) );
    switch (port) {
        case 0 : 
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        case 1 : 
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        case 2 : 
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        default:
            break;
    }
    switch (pin & 0x0F) {
        case 0 :
        case 1 :
            HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
            break;
        case 2 :
        case 3 :
            HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
            break;
        default :
            HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
            break;
    }

} 

void  McuSTM32L072::EnableIrqExtGpio ( void ){
    HAL_NVIC_EnableIRQ ( EXTI0_1_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI2_3_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI4_15_IRQn );
}
void  McuSTM32L072::DisableIrqExtGpio ( void ){
    HAL_NVIC_DisableIRQ ( EXTI0_1_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI2_3_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI4_15_IRQn );
}
/******************************************************************************/
/*                                Mcu Spi Api                                 */
/******************************************************************************/
    /** Create a SPI master connected to the specified pins
    *
    *  @param mosi SPI Master Out, Slave In pin
    *  @param miso SPI Master In, Slave Out pin
    *  @param sclk SPI Clock pin
    */
/********************************************************************/
/*                   Utilities for SPI                              */
/********************************************************************/

SPI_HandleTypeDef hspi1;
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle) {
    if (spiHandle->Instance==SPI1) {
        __HAL_RCC_SPI1_CLK_DISABLE();
    }
    if (spiHandle->Instance==SPI2) {
        __HAL_RCC_SPI2_CLK_DISABLE();
    }

    int port = ( LORA_SPI_MOSI & 0xF0 ) >> 4 ;
    switch (port) {
        case 0 : 
            HAL_GPIO_DeInit(GPIOA, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        case 1 : 
            HAL_GPIO_DeInit(GPIOB, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        case 2 : 
            HAL_GPIO_DeInit(GPIOC, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        default:
            HAL_GPIO_DeInit(GPIOA, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
    }
}


__STATIC_INLINE uint32_t LL_SPI_IsActiveFlag_TXE(SPI_TypeDef *SPIx)
{
  return (READ_BIT(SPIx->SR, SPI_SR_TXE) == (SPI_SR_TXE));
}

__STATIC_INLINE void LL_SPI_TransmitData8(SPI_TypeDef *SPIx, uint8_t TxData)
{
  *((__IO uint8_t *)&SPIx->DR) = TxData;
}

__STATIC_INLINE uint32_t LL_SPI_IsActiveFlag_RXNE(SPI_TypeDef *SPIx)
{
  return (READ_BIT(SPIx->SR, SPI_SR_RXNE) == (SPI_SR_RXNE));
}

__STATIC_INLINE uint8_t LL_SPI_ReceiveData8(SPI_TypeDef *SPIx)
{
  return (uint8_t)(READ_REG(SPIx->DR));
}

/********************************************************************/
/*              END  Of  Utilities for SPI                          */
/********************************************************************/
void McuSTM32L072::InitSpi ( ){

    hspi1.Instance = LORA_SPIx;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;

    if ( LORA_SPIx == SPI1 ) {
        __HAL_RCC_SPI1_CLK_ENABLE();
    } else if ( LORA_SPIx == SPI2 ) {
        __HAL_RCC_SPI2_CLK_ENABLE();
    } 

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = (GPIO_AF0_SPI1 );
    GPIO_InitStruct.Pin       = ( 1 << ( LORA_SPI_MOSI & 0x0F ) ) ;
    int port = ( LORA_SPI_MOSI & 0xF0 ) >> 4 ;
    switch (port) {
        case 0 : 
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        case 1 : 
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        case 2 : 
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        default:
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
    }
    GPIO_InitStruct.Pin       = ( 1 << ( LORA_SPI_MISO & 0x0F ) ) ;
    port = ( LORA_SPI_MISO & 0xF0 ) >> 4 ;
    switch (port) {
        case 0 : 
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        case 1 : 
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        case 2 : 
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        default:
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
    }
    GPIO_InitStruct.Pin       = ( 1 << ( LORA_SPI_SCLK & 0x0F ) ); 
    port = ( LORA_SPI_SCLK & 0xF0 ) >> 4 ;
    switch (port) {
        case 0 : 
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
        case 1 : 
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        case 2 : 
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
            break;
        default:
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            break;
    }



    if (HAL_SPI_Init(&hspi1) != HAL_OK ) {
    }
    __HAL_SPI_ENABLE(&hspi1); 
}


    
    /** Write to the SPI Slave and return the response
    *
    *  @param value Data to be sent to the SPI slave
    *
    *  @returns
    *    Response from the SPI slave
    */
uint8_t McuSTM32L072::SpiWrite(int value){
        uint8_t rxData = 0;
  
        while( LL_SPI_IsActiveFlag_TXE ( SPI1 ) == 0  ){};
        LL_SPI_TransmitData8 (SPI1, uint8_t (value&0xFF));
        while( LL_SPI_IsActiveFlag_RXNE ( SPI1 ) == 0 ){};
        rxData =  LL_SPI_ReceiveData8( SPI1 );
        return (rxData);
}


/******************************************************************************/
/*                                Mcu Flash Api                               */
/******************************************************************************/
int McuSTM32L072::RestoreContext(uint8_t *buffer, uint32_t addr, uint32_t size){
    uint32_t i;
    for (i=0;i<size;i++){
        buffer[i]= *((uint8_t*)(addr)+i);
    }
    return 0;
}



int McuSTM32L072::StoreContext(const void *buffer, uint32_t addr, uint32_t size){
     HAL_StatusTypeDef res = HAL_OK; 
        HAL_FLASH_Unlock(); 
        for (uint32_t i = 0 ; i < (8 * size ) ; i++){
         *((uint8_t*)(addr)+i) = *((uint8_t *)(buffer) + i);
         __WFI();
        }
        return (res);
} 
   


/******************************************************************************/
/*                                Mcu RTC Api                                 */
/******************************************************************************/
/********************************************************************/
/*                   Utilities for RTC                              */
/********************************************************************/
RTC_HandleTypeDef hrtc;


void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_ENABLE();
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_DISABLE();
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  }
} 

/********************************************************************/
/*            End Of Utilities for RTC                              */
/********************************************************************/
void McuSTM32L072::RtcInit (void)
{
   RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 31;
  if (LOW_SPEED_CLK == LSI ) {
      hrtc.Init.SynchPrediv = ( LSI_VALUE / 128 ) -1;
  } else {
      hrtc.Init.SynchPrediv = 1023;
  }
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
  }
  time_t t = 0x418924;
   t = 0x18924;
  
  struct tm * timeinfo;
  timeinfo = localtime (&t);
  sTime.Minutes = timeinfo->tm_min;
  sTime.Seconds = timeinfo->tm_sec;
  sTime.Hours   = timeinfo->tm_hour;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {

  }
  
  sDate.WeekDay = timeinfo->tm_wday;
  sDate.Month   = timeinfo->tm_mon ;
  sDate.Date    = timeinfo->tm_mday;
  sDate.Year    = timeinfo->tm_year;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
  }
 
  //if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK) {
  //}

}

uint32_t McuSTM32L072::RtcGetTimeMs( void )
{
    RTC_DateTypeDef dateStruct;
    RTC_TimeTypeDef timeStruct;
    struct tm timeinfo;
    RtcHandle.Instance = RTC;
    HAL_RTC_GetTime(&RtcHandle, &timeStruct, FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &dateStruct, FORMAT_BIN);
    timeinfo.tm_wday = dateStruct.WeekDay;
    timeinfo.tm_mon  = dateStruct.Month ;
    timeinfo.tm_mday = dateStruct.Date;
    timeinfo.tm_year = dateStruct.Year ;
    timeinfo.tm_hour = timeStruct.Hours;
    timeinfo.tm_min  = timeStruct.Minutes;
    timeinfo.tm_sec  = timeStruct.Seconds;
    // Convert to timestamp   
    time_t t = mktime(&timeinfo);
    return ( ( t * 1000 ) + ( 999 - ( ( timeStruct.SubSeconds *999) / hrtc.Init.SynchPrediv ) ) );  // get time en ms
}

uint32_t McuSTM32L072::RtcGetTimeSecond( void )
{
    RTC_DateTypeDef dateStruct;
    RTC_TimeTypeDef timeStruct;
    struct tm timeinfo;
    RtcHandle.Instance = RTC;

    HAL_RTC_GetTime(&RtcHandle, &timeStruct, FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &dateStruct, FORMAT_BIN);
    timeinfo.tm_wday = dateStruct.WeekDay;
    timeinfo.tm_mon  = dateStruct.Month ;
    timeinfo.tm_mday = dateStruct.Date;
    timeinfo.tm_year = dateStruct.Year ;
    timeinfo.tm_hour = timeStruct.Hours;
    timeinfo.tm_min  = timeStruct.Minutes;
    timeinfo.tm_sec  = timeStruct.Seconds;
    time_t t = mktime(&timeinfo);
    return ( t );
}



/********************************************************************/
/*                   Utilities for Uart                              */
/********            //mcu.mwait_ms ( 100 );************************************************************/

UART_HandleTypeDef huart2;
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==UART_NUM) {
        if ( UART_NUM == USART1) {
           __HAL_RCC_USART1_CLK_ENABLE();
            GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
        } else {
            __HAL_RCC_USART2_CLK_ENABLE();
            GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
        }
      GPIO_InitStruct.Pin = (1 << ( UART_TX & 0x0F ) )| (1 << ( UART_RX & 0x0F ) );
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance==USART2) {
      __HAL_RCC_USART2_CLK_DISABLE();
    }
    if(huart->Instance==USART1) {
      __HAL_RCC_USART1_CLK_DISABLE();
    }
      HAL_GPIO_DeInit(GPIOA, (1 << ( UART_TX & 0x0F ) )| (1 << ( UART_RX & 0x0F ) ) );
  
}

/********************************************************************/
/*              END  Of  Utilities for UART                         */
/********************************************************************/

/******************************************************************************/
/*                           Mcu Uart Api                                     */
/******************************************************************************/

 
#if DEBUG_TRACE == 1
void vprint(const char *fmt, va_list argp) {
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}
#endif

void McuSTM32L072::UartInit ( void ) {

  huart2.Instance = UART_NUM;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
  }
};
//#endif

void McuSTM32L072::MMprint( const char *fmt, ...){
#if DEBUG_TRACE == 1
  va_list argp;
  va_start(argp, fmt);
  vprint(fmt, argp);
  va_end(argp);
#endif
};


#if DEBUG_TRACERP == 1
    static std::string BufferDebugRadioPlaner ("") ;
#endif


void McuSTM32L072::MMStoreBuffer( const char *fmt, ...){
#if DEBUG_TRACERP == 1
  va_list argp;
  va_start(argp, fmt);
  char string[200];
  if(0 < vsprintf(string,fmt,argp)) {
    BufferDebugRadioPlaner = BufferDebugRadioPlaner + string;
  }

  va_end(argp);
#endif 
};

void McuSTM32L072::MMClearDebugBufferRadioPlaner ( void ) {
#if DEBUG_TRACERP == 1
    BufferDebugRadioPlaner = "";
#endif 
};
void McuSTM32L072::MMPrintBuffer ( void ) {
#if DEBUG_TRACERP == 1
    HAL_UART_Transmit(&huart2, (uint8_t*)BufferDebugRadioPlaner.c_str(), strlen(BufferDebugRadioPlaner.c_str()), 0xffffff); // send message via UART
    MMClearDebugBufferRadioPlaner();
#endif 
};
void sleepAndWakeUp (void) {
    mcu.PowerConsumptionTotal +=  mcu.RtcGetTimeMs () - mcu.PowerConsumption ;
    HAL_SPI_DeInit (&hspi1);
    HAL_UART_DeInit (&huart2);
     mcu.I2cDeInit ();
    HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI );
}
void McuSTM32L072::WakeUpAfterDeepSleep (void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
 // __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;//RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;//RCC_PLLDIV_4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){}
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK){}
  if ( UART_NUM == USART1 ) { 
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  } else {
       PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
       PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  }
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){}
  CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
  // mcu.SetValueDigitalOutPin ( PB_14, 1 );
  //I2cInit ();
  UartInit();
  InitSpi();
  I2cInit ();
  PowerConsumption = RtcGetTimeMs ();

}

void sleepAndWakeUpCool (void) {
 
  HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI );
 
  SystemClock_Config();
  //DEBUG_MSG ("wu\n") ;

}
/******************************************************************************/
/*                                Mcu Sleep Api                               */
/******************************************************************************/
void McuSTM32L072::GotoSleepSecond (int duration ) {
#if LOW_POWER_MODE == 1
    int cpt = duration ;
    while ( cpt > ( WATCH_DOG_PERIOD_RELEASE ) ) {
        cpt -= WATCH_DOG_PERIOD_RELEASE ;
        WakeUpAlarmSecond( WATCH_DOG_PERIOD_RELEASE );
        sleepAndWakeUp();
        WatchDogRelease ( );
    }
    WakeUpAlarmSecond( cpt );
    sleepAndWakeUp();
# else
    int cpt = duration ;
    WatchDogRelease ( );
    while ( cpt > ( WATCH_DOG_PERIOD_RELEASE ) ) {
        cpt -= WATCH_DOG_PERIOD_RELEASE ;
        mwait( WATCH_DOG_PERIOD_RELEASE );
        WatchDogRelease ( );
    }
    mwait( cpt );
    WatchDogRelease ( );
#endif
}

void McuSTM32L072::GotoSleepMSecond (int duration ) {
#if LOW_POWER_MODE == 1
    WakeUpAlarmMSecond ( duration );
    sleepAndWakeUp();
    WatchDogRelease ( );
# else
    mwait_ms ( duration ) ;
    WatchDogRelease ( );
#endif
}


/******************************************************************************/
/*                             Mcu WatchDog Api                               */
/******************************************************************************/

IWDG_HandleTypeDef hiwdg;

/*!
 * Watch Dog Init And start with a period befor ereset set to 32 seconds
*/
void McuSTM32L072::WatchDogStart ( void ) {

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload = 0xFFF;

  /* Enable IWDG. LSI is turned on automaticaly */
  __HAL_IWDG_START(&hiwdg);

  /* Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers by writing
  0x5555 in KR */
  IWDG_ENABLE_WRITE_ACCESS(&hiwdg);
  /* Write to IWDG registers the Prescaler & Reload values to work with */
  (&hiwdg)->Instance->PR = hiwdg.Init.Prescaler;
  (&hiwdg)->Instance->RLR = hiwdg.Init.Reload;
  /* Check pending flag, if previous update not done, return timeout */

   /* Wait for register to be updated */
  while((&hiwdg)->Instance->SR != RESET)
  {

  }

  /* If window parameter is different than current value, modify window 
  register */
  if((&hiwdg)->Instance->WINR != hiwdg.Init.Window)
  {
    /* Write to IWDG WINR the IWDG_Window value to compare with. In any case,
    even if window feature is disabled, Watchdog will be reloaded by writing 
    windows register */
    (&hiwdg)->Instance->WINR = hiwdg.Init.Window;
  }
  else
  {
    /* Reload IWDG counter with value defined in the reload register */
    __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
  }

  /* Return function status */
}
/*!
 * Watch Dog Release
*/
void McuSTM32L072::WatchDogRelease ( void ) {
    __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
};
/******************************************************************************/
/*                             Mcu LOwPower timer Api                         */
/******************************************************************************/

/********************************************************************/
/*             Utilities for LowPower Timer                         */
/********************************************************************/
LPTIM_HandleTypeDef hlptim1;

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* lptimHandle)
{
  if(lptimHandle->Instance==LPTIM1) {
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
  }
}
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* lptimHandle)
{
  if(lptimHandle->Instance==LPTIM1) {
    __HAL_RCC_LPTIM1_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
  }
} 


/********************************************************************/
/*       END  Of  Utilities for LowPower Timer                      */
/********************************************************************/
void McuSTM32L072::LowPowerTimerLoRaInit ( ) {
    hlptim1.Instance = LPTIM1;
    hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV16;
    hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;

    if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
    }
    Func = DoNothing;
    obj = NULL;
};

void McuSTM32L072::LowPowerTimerDisableIrq ( ) {
     HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
}
void McuSTM32L072::LowPowerTimerEnableIrq ( ) {
     HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}
/*!
 * LowPowerTimerLoRa AttachMsecond
 *
 * \param void (* _Func) (void *) a static method member of the current Obj
 * \param *_obj a pointer to the current objet
 * \param int delay in ms delay should be between 1ms and 16s.
 * \param [OUT] void         
 * \remark the code  Func =  _Func ; and obj  = _obj; isn't mcu dependent
 * \remark starts the LoRaWAN dedicated timer and attaches the IRQ to the handling Interupt SErvice Routine in the LoRaWAN object.
 */
void McuSTM32L072::StartTimerMsecond ( void (* _Func) (void *) , void * _obj, int delay){

    uint32_t DelayMs2tick;
    if (LOW_SPEED_CLK == LSI ) {
        uint32_t mult = LSI_VALUE / 16 ;
        DelayMs2tick = (delay * mult)/1000; // LSI VALUE / LPTIM_PRESCALER_DIV16  
       
    } else {   
        uint32_t mult = LSE_VALUE / 16 ;
        DelayMs2tick = (delay * mult ) / 1000;
    }
    HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 65535, DelayMs2tick); // MCU specific
    Func =  _Func ;
    obj  = _obj;
};

void McuSTM32L072::StopTimerMsecond ( void ) {

    HAL_LPTIM_TimeOut_Stop_IT (&hlptim1 );
}



void  McuSTM32L072::AttachInterruptIn       (  void (* _Funcext) (void *) , void * _objext) {
    Funcext =  _Funcext ;
    objext  = _objext;
    userIt  = 0 ; 
};

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/
volatile static uint32_t tempo;
    void  McuSTM32L072::mwait_ms (int delayms) {
    
        HAL_Delay(delayms);
    };
    void  McuSTM32L072::mwait (int delays) {
        HAL_Delay(1000*delays);
    };
/******************************************************************************/
/*                             Mcu I2C Api                                    */
/******************************************************************************/



void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct;
    if(hi2c->Instance==I2C1) {
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        __HAL_RCC_I2C1_CLK_ENABLE();
      
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    if(hi2c->Instance==I2C1)  {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9|GPIO_PIN_8);
    }
}

void McuSTM32L072::I2cInit(void) {
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x20602938;//0x00506682;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init ( &hi2c1 ) ;
   // HAL_I2CEx_ConfigAnalogFilter( &hi2c1, I2C_ANALOGFILTER_ENABLE);
  //  HAL_I2CEx_ConfigDigitalFilter( &hi2c1, 0) ;
}
void McuSTM32L072::I2cDeInit(void) {
    HAL_I2C_DeInit( &hi2c1); 
}

HAL_StatusTypeDef McuSTM32L072::I2cTransmit (uint16_t DevAddress, uint8_t *pData, uint16_t Size ) {
    HAL_StatusTypeDef tmp ;
    tmp = HAL_I2C_Master_Transmit(&hi2c1,  DevAddress, pData,  Size,  200) ;
    return ( tmp ) ;
} 

HAL_StatusTypeDef McuSTM32L072::I2cReceive (uint16_t DevAddress, uint8_t *pData, uint16_t Size ) {
    HAL_StatusTypeDef tmp ;
    tmp =HAL_I2C_Master_Receive(&hi2c1,  DevAddress,pData,  Size, 200 ) ;
    return ( tmp ) ;
}

HAL_StatusTypeDef McuSTM32L072::I2cReadMem ( uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    return ( HAL_I2C_Mem_Read(&hi2c1,  DevAddress,  MemAddress,  MemAddSize, pData,  Size, 1000) );
}
HAL_StatusTypeDef McuSTM32L072::I2cWriteMem( uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout){

    return ( HAL_I2C_Mem_Write(&hi2c1,  DevAddress,  MemAddress,  MemAddSize, pData,  Size, 1000) );
}
 

    

 
