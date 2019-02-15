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
#include "ClassSTM32L4.h"
#include "stdint.h"
#include "ApiMcu.h"
#include "stm32l4xx_hal.h"
#include "time.h"
#include "UserDefine.h"
#include "Define.h"

#if DEBUG_TRACE == 1
    #include <stdarg.h>
    #include <string.h>
#endif

#define WATCH_DOG_PERIOD_RELEASE 30 // this period have to be lower than the Watch Dog period of 32 seconds







/********************************************************************/
/*                    SystemClock_Config functions                  */
/********************************************************************/
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
   
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK){
   
  }
 
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
  
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK){
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
#define DATA_EEPROM_BASE       ( ( uint32_t )0x8000000 )              /*!< DATA_EEPROM base address in the alias region */
#define DATA_EEPROM_END        ( ( uint32_t )DATA_EEPROM_BASE + 4096 ) /*!< DATA EEPROM end address in the alias region */

/**
  * @brief  This function does an erase of all user flash area
  * @param  bank_active: start of user flash area
  * @retval FLASHIF_OK : user flash area successfully erased
  *         FLASHIF_ERASEKO : error occurred
  */
enum
{
    FLASHIF_OK = 0,
    FLASHIF_ERASEKO,
    FLASHIF_WRITINGCTRL_ERROR,
    FLASHIF_WRITING_ERROR,
    FLASHIF_CRCKO,
    FLASHIF_RECORD_ERROR,
    FLASHIF_EMPTY,
    FLASHIF_PROTECTION_ERRROR
};

enum{
    FLASHIF_PROTECTION_NONE         = 0,
    FLASHIF_PROTECTION_PCROPENABLED = 0x1,
    FLASHIF_PROTECTION_WRPENABLED   = 0x2,
    FLASHIF_PROTECTION_RDPENABLED   = 0x4,
};

/* protection update */
enum {
    FLASHIF_WRP_ENABLE,
    FLASHIF_WRP_DISABLE
};
uint32_t BankActive = 0, BFSysMem = 0;
FLASH_OBProgramInitTypeDef OBConfig;
/* Notable Flash addresses */
#define FLASH_START_BANK1             ((uint32_t)0x08000000)
#define FLASH_START_BANK2             ((uint32_t)0x08080000)
#define USER_FLASH_END_ADDRESS        ((uint32_t)0x08100000)

#define NVIC_VT_FLASH_B2           FLASH_START_BANK1
#define NVIC_VT_FLASH_B1           FLASH_START_BANK2

uint32_t FLASH_If_Erase(uint32_t bank_active)
{
    uint32_t bank_to_erase, error = 0;
    FLASH_EraseInitTypeDef pEraseInit;
    HAL_StatusTypeDef status = HAL_OK;
    if (bank_active == 0) {
        bank_to_erase = FLASH_BANK_2;
    } else {
        bank_to_erase = FLASH_BANK_1;
    }
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    pEraseInit.Banks = bank_to_erase;
    pEraseInit.NbPages = 255;
    pEraseInit.Page = 0;
    pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;
    status = HAL_FLASHEx_Erase(&pEraseInit, &error);

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
    if (status != HAL_OK) {
    /* Error occurred while page erase */
        return FLASHIF_ERASEKO;
    }
    return FLASHIF_OK;
}

/**
  * @brief  This function does an CRC check of an application loaded in a memory bank.
  * @param  start: start of user flash area
  * @retval FLASHIF_OK: user flash area successfully erased
  *         other: error occurred
  */
uint32_t FLASH_If_Check(uint32_t start)
{
    /* checking if the data could be code (first word is stack location) */
    if ((*(uint32_t*)start >> 24) != 0x20 ) return FLASHIF_EMPTY;
    return FLASHIF_OK;
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  destination: start address for target location
  * @param  p_source: pointer on buffer with data to write
  * @param  length: length of data buffer (unit is 32-bit word)
  * @retval uint32_t 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
    uint32_t status = FLASHIF_OK;
    uint32_t i = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* DataLength must be a multiple of 64 bit */
    for (i = 0; (i < length / 2) && (destination <= (USER_FLASH_END_ADDRESS - 8)); i++) {
        /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
           be done by word */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination, *((uint64_t *)(p_source + 2*i))) == HAL_OK) {
          /* Check the written value */
            if (*(uint64_t*)destination != *(uint64_t *)(p_source + 2*i)) {
            /* Flash content doesn't match SRAM content */
                status = FLASHIF_WRITINGCTRL_ERROR;
                break;
            }
          /* Increment FLASH destination address */
            destination += 8;
        } else {
          /* Error occurred while writing data in Flash memory */
            status = FLASHIF_WRITING_ERROR;
            break;
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
    return status;
}

/**
  * @brief  Configure the write protection status of user flash area.
  * @retval uint32_t FLASHIF_OK if change is applied.
  */
uint32_t FLASH_If_WriteProtectionClear( void ) {
    FLASH_OBProgramInitTypeDef OptionsBytesStruct1;
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    /* Unlock the Options Bytes *************************************************/
    HAL_FLASH_OB_Unlock();
    OptionsBytesStruct1.RDPLevel = OB_RDP_LEVEL_0;
    OptionsBytesStruct1.OptionType = OPTIONBYTE_WRP;
    OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK2_AREAA;
    OptionsBytesStruct1.WRPEndOffset = 0x00;
    OptionsBytesStruct1.WRPStartOffset = 0xFF;
    HAL_FLASHEx_OBProgram(&OptionsBytesStruct1);
    OptionsBytesStruct1.WRPArea = OB_WRPAREA_BANK2_AREAB;
    HAL_FLASHEx_OBProgram(&OptionsBytesStruct1);
    return (0);
}

/**
  * @brief  Modify the BFB2 status of user flash area.
  * @param  none
  * @retval HAL_StatusTypeDef HAL_OK if change is applied.
  */
HAL_StatusTypeDef FLASH_If_BankSwitch(void)
{
    FLASH_OBProgramInitTypeDef ob_config;
    HAL_StatusTypeDef result;
    HAL_FLASH_Lock();
    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    /* Get the current configuration */
    HAL_FLASHEx_OBGetConfig( &ob_config );
    ob_config.OptionType = OPTIONBYTE_USER;
    ob_config.USERType = OB_USER_BFB2;
    if ((ob_config.USERConfig) & (OB_BFB2_ENABLE)) {
        ob_config.USERConfig = OB_BFB2_DISABLE;
    } else {
        ob_config.USERConfig = OB_BFB2_ENABLE;
    }

    /* Initiating the modifications */
    result = HAL_FLASH_Unlock();
    /* program if unlock is successful */
    if ( result == HAL_OK ) {
        result = HAL_FLASH_OB_Unlock();
    /* program if unlock is successful*/
        if ((READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) == RESET)) {
            result = HAL_FLASHEx_OBProgram(&ob_config);
        }
        if (result == HAL_OK) {
            HAL_FLASH_OB_Launch();
        }
    }
    return result;
}


void FlashPageErase( uint32_t page, uint32_t banks )
{
    // Check the parameters
    assert_param( IS_FLASH_PAGE( page ) );
    assert_param( IS_FLASH_BANK_EXCLUSIVE( banks ) );
    if( ( banks & FLASH_BANK_1 ) != RESET ) {
        CLEAR_BIT( FLASH->CR, FLASH_CR_BKER );
    } else {
        SET_BIT( FLASH->CR, FLASH_CR_BKER );
    }
    // Proceed to erase the page
    MODIFY_REG( FLASH->CR, FLASH_CR_PNB, ( page << 3 ) );
    SET_BIT( FLASH->CR, FLASH_CR_PER );
    SET_BIT( FLASH->CR, FLASH_CR_STRT );
}

uint8_t EepromMcuWriteBuffer( uint32_t addr, uint8_t *buffer, uint16_t size )
{   
    HAL_StatusTypeDef status = HAL_OK; 
    uint64_t *flash = ( uint64_t* )buffer;
    uint32_t Findpage = (addr - 0x8000000 )>>11;
    uint32_t NumberOfPage = (size >> 11)+1;	
    HAL_FLASH_Unlock( );
    for (uint32_t i = 0 ; i < NumberOfPage; i ++){
        FlashPageErase( Findpage + i, 1 );
    }
    WRITE_REG( FLASH->CR, 0x40000000 );
    for( uint32_t i = 0; i < size; i++ )
    {
        if (HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, addr + ( 8 * i ), flash[i] ) == HAL_OK)
        {
            /* Check the written value */
            if (*(uint64_t*)(addr + ( 8 * i )) != flash[i])
            {
            /* Flash content doesn't match SRAM content */
                status = HAL_ERROR;
                break;
            }
        } else {
           /* Error occurred while writing data in Flash memory */
            status = HAL_ERROR;
            break;
        }
    }
    HAL_FLASH_Lock( );
    return status;
}

uint8_t EepromMcuReadBuffer( uint32_t addr, uint8_t *buffer, uint16_t size )
{
    assert_param( buffer != NULL ); 
    //assert_param( addr >= DATA_EEPROM_BASE );
    assert_param( buffer != NULL );
    assert_param( size < ( DATA_EEPROM_END - DATA_EEPROM_BASE ) );
    for( uint32_t i = 0; i < size; i++ ) {
        buffer[i]= *((( uint8_t* )addr)+i);
    }
    return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    assert_param( FAIL );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    assert_param( FAIL );
    return 0;
}


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
McuSTM32L4::McuSTM32L4(PinName mosi, PinName miso, PinName sclk )  {
    Func = DoNothing; // don't modify
    obj = NULL;       // don't modify
    McuMosi = mosi;   // don't modify
    McuMiso = miso;   // don't modify
    McuSclk = sclk;   // don't modify
}     


McuSTM32L4::~McuSTM32L4(){
      // to be completed by mcu providers
} 

/*******************************************/
/*                  Mcu Init               */
/*******************************************/
void McuSTM32L4::InitMcu( void ) {
    // system clk Done with mbed to be completed by mcu providers if mbed is removed
      /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  InitGpio ( );
  UartInit();
  LowPowerTimerLoRaInit ( );
  InitSpi () ;
  RtcInit ();
  PowerConsumptionTotal = 0;
  PowerConsumption = RtcGetTimeMs ();
    /*For dual boot */
    FLASH_If_WriteProtectionClear();
    /* Test from which bank the program runs */
    /* Bit 8 FB_MODE: Flash Bank mode selection
    0: Flash Bank 1 mapped at 0x0800 0000 (and aliased @0x0000 0000(1)) and Flash Bank 2
    mapped at 0x0808 0000 (and aliased at 0x0008 0000)
    1: Flash Bank2 mapped at 0x0800 0000 (and aliased @0x0000 0000(1)) and Flash Bank 1
    mapped at 0x0808 0000 (and aliased at 0x0008 0000)
            */
    BankActive = READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE);
    if (BankActive == 0) { //bank 1
        DEBUG_MSG("Dual Boot is activated and code running on Bank 1 \n");
    } else {
        DEBUG_MSG("Dual Boot is activated and code running on Bank 2 \n");
        uint32_t result = FLASH_If_Erase( BankActive ); //Erase the 0x8080000
        if (result == FLASHIF_OK) {
        DEBUG_MSG("Copying BANK1 to BANK2\n");
        result = FLASH_If_Write( FLASH_START_BANK2, (uint32_t*)FLASH_START_BANK1, 20480);
        }
        if (result != FLASHIF_OK) {
            DEBUG_PRINTF("Failure! %d \n",result);
        } else {
            DEBUG_MSG("Sucess!\n");
            FLASH_If_BankSwitch();
            NVIC_SystemReset();
        }
    }
}

/*******************************************/
/*            GPIO Init                    */
/*******************************************/
void McuSTM32L4::InitGpio ( ) {
      /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
   /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
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
    InitGpioOut ( DEBUGRX);
    SetValueDigitalOutPin ( DEBUGRX , 0 );
    InitGpioOut ( DEBUG);
    SetValueDigitalOutPin ( DEBUG , 0 );
}
void McuSTM32L4::InitGpioOut( PinName Pin ){
    int port = ( Pin & 0xF0 ) >> 4 ;
      /*Configure GPIO pin : PB6   */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin   = (1 << ( Pin & 0x0F ) );
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
        case 4 : 
            HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
            break;
        case 8 : 
            HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
            break;
        case 16 : 
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
        case 32 : 
            HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
            break;
        case 64 : 
            HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
            break;
        default:
            break;
    }
}
void McuSTM32L4::InitGpioIn( PinName Pin ){
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
        case 4 : 
            HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
            break;
        case 8 : 
            HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
            break;
        case 16 : 
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
        case 32 : 
            HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
            break;
        case 64 : 
            HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
            break;
        default:
            break;
    }
}

void McuSTM32L4::SetValueDigitalOutPin ( PinName Pin, int Value ){
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
        case 4 : 
             HAL_GPIO_WritePin(GPIOD, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
            break;
        case 8 : 
             HAL_GPIO_WritePin(GPIOE, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
            break;
        case 16 : 
             HAL_GPIO_WritePin(GPIOF, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
        case 32 : 
             HAL_GPIO_WritePin(GPIOG, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
            break;
        case 64 : 
             HAL_GPIO_WritePin(GPIOH, (1 << ( Pin & 0x0F ) ), (GPIO_PinState) Value);
            break;
        default:
            break;
    }
}
int McuSTM32L4::GetValueDigitalInPin ( PinName Pin ){
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
        case 4 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOD, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        case 8 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOE, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        case 16 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOF, ( 1 << ( Pin & 0x0F ) ) ) );
        case 32 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOG, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        case 64 : 
            return ( (int) HAL_GPIO_ReadPin(GPIOH, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
        default:
            return ( (int) HAL_GPIO_ReadPin(GPIOA, ( 1 << ( Pin & 0x0F ) ) ) );
            break;
    }
};
void  McuSTM32L4::Init_Irq ( PinName pin) {

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
        case 4 : 
            HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
            break;
        case 8 : 
            HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
            break;
        case 16 : 
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
        case 32 : 
            HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
            break;
        case 64 : 
            HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
            break;
        default:
            break;
    }
    switch (pin & 0x0F) {
        case 0 :
            HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
            break;
        case 1 :
            HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
            break;
        case 2 :
            HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);
            break;
        case 3 :
            HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            break;
        case 4 :
            HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
            break;
        case 5 :
        case 6 :
        case 7 :
        case 8 :
        case 9 :

            HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            break;
        default :
            HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
            break;
    }

} 

void  McuSTM32L4::EnableIrqExtGpio ( void ){
    HAL_NVIC_EnableIRQ ( EXTI0_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI1_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI2_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI3_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI4_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI9_5_IRQn );
    HAL_NVIC_EnableIRQ ( EXTI15_10_IRQn );
}
void  McuSTM32L4::DisableIrqExtGpio ( void ){
    HAL_NVIC_DisableIRQ ( EXTI0_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI1_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI2_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI3_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI4_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI9_5_IRQn );
    HAL_NVIC_DisableIRQ ( EXTI15_10_IRQn );
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
    if (spiHandle->Instance==SPI3) {
        __HAL_RCC_SPI3_CLK_DISABLE();
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
        case 4 : 
            HAL_GPIO_DeInit(GPIOD, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        case 8 : 
           HAL_GPIO_DeInit(GPIOE, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        case 16 : 
            HAL_GPIO_DeInit(GPIOF, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        case 32 : 
            HAL_GPIO_DeInit(GPIOG, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
            break;
        case 64 : 
            HAL_GPIO_DeInit(GPIOH, ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) );
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
void McuSTM32L4::InitSpi ( ){

    hspi1.Instance = LORA_SPIx;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;

    if ( LORA_SPIx == SPI1 ) {
        __HAL_RCC_SPI1_CLK_ENABLE();
    } else if ( LORA_SPIx == SPI2 ) {
        __HAL_RCC_SPI2_CLK_ENABLE();
    } else {
        __HAL_RCC_SPI3_CLK_ENABLE();
    }

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin       = ( 1 << ( LORA_SPI_MOSI & 0x0F ) )  | ( 1 << ( LORA_SPI_MISO & 0x0F ) )  | ( 1 << ( LORA_SPI_SCLK & 0x0F ) ) ;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = (LORA_SPI_SCLK & 0xF );
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
        case 4 : 
            HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
            break;
        case 8 : 
            HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
            break;
        case 16 : 
            HAL_GPIO_Init(GPIOF , &GPIO_InitStruct);
            break;
        case 32 : 
            HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
            break;
        case 64 : 
            HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
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
uint8_t McuSTM32L4::SpiWrite(int value){
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
int McuSTM32L4::RestoreContext(uint8_t *buffer, uint32_t addr, uint32_t size){
     /* have to be implemented by mcu providers
    the following code propose a lite implementation without any error cases*/
    uint16_t sizet = size & 0xFFFF;
    EepromMcuReadBuffer( addr, buffer, sizet );
    return ( 0 );
}
static uint8_t copyPage [2048] ;
int McuSTM32L4::WriteFlashWithoutErase(uint8_t *buffer, uint32_t addr, uint32_t size){

    int findPage = 0 ;
    uint32_t findByteAdress = 0 ;
    uint32_t findLastAdress = 0 ;
    int status = 0;
    uint32_t i;
    uint32_t flashBaseAdress;    
    uint64_t *flash = ( uint64_t* )copyPage;
    assert_param( buffer != NULL );
    assert_param( size < ( 2048 ) );
    findPage = ((addr - 0x8080000 )) >> 11 ;  // 2048 page size;
    findByteAdress = ( (addr - 0x8080000 )  - ( findPage << 11 ));
    findLastAdress = findByteAdress + size ; //if >2048 across two pages !!
    flashBaseAdress = 0x8080000 ;
    HAL_FLASH_Unlock( );

    for( i = 0; i < 2048; i++ ) {
        copyPage[i]= *(( uint8_t* )(flashBaseAdress + (findPage * 2048))+i);
    }
    FlashPageErase(   findPage, 2 );
    WRITE_REG( FLASH->CR, 0x40000000 );
    if ( findLastAdress < 2048) { // all is done on the same page
        for( i = 0; i < size; i++ ) {
            copyPage[i + findByteAdress] = buffer [i];
        }
        for( i = 0; i < 256; i++ ) {
            status += HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, flashBaseAdress + (findPage * 2048) + ( 8 * i ), flash[i] );
        }
    } else { // require 2 pages
        for( i = 0; i < (2048-findByteAdress); i++ ) {
            copyPage[i + findByteAdress] = buffer [i];
        }
        for( i = 0; i < 256; i++ ) {
            status += HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, flashBaseAdress + (findPage * 2048) + ( 8 * i ),  flash[i] );
        }
        for( i = 0; i < 2048; i++ ) { //copy the next page
            copyPage[i]= *(( uint8_t* )(flashBaseAdress + ((findPage + 1) * 2048))+i);
        }
        FlashPageErase(   1 + findPage, 2 );
        WRITE_REG( FLASH->CR, 0x40000000 );
        for( i = 0; i < (findLastAdress - 2048); i++ ) {
            copyPage[i ] =  buffer [i + 2048 - findByteAdress ];
        }
        for( i = 0; i < 256; i++ ) {
            status += HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, flashBaseAdress + ((findPage + 1) * 2048) + ( 8 * i ),  flash[i] );
        }
    }
    if ( status > 0 ) {
        // DEBUG_MSG("ERROR HAL FLASH \n");
    }
    HAL_FLASH_Lock( );
    return ( 0 ); 
}



int McuSTM32L4::StoreContext(const void *buffer, uint32_t addr, uint32_t size){
    /* have to be implemented by mcu providers
    the following code propose a lite implementation without any error cases
    this section have to be very robust, have to support failure mode such as  power off during flash programmation    
    This basic implementation suppose that the addr is 4 Bytes aligned and suppose also that the size can be divide by 4.
    */
    uint16_t sizet = size & 0xFFFF;
    while ( EepromMcuWriteBuffer( addr,  (uint8_t*) buffer, sizet ) != HAL_OK) { // in case of infinite error watchdog will expire
        mwait_ms ( 300 );
    }
    mwait_ms ( 300 );
    return ( 0 ); 
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
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_DISABLE();
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
  }
} 

/********************************************************************/
/*            End Of Utilities for RTC                              */
/********************************************************************/
void McuSTM32L4::RtcInit (void)
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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {

  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
  }
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK) {
  }
}

uint32_t McuSTM32L4::RtcGetTimeMs( void )
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
    return ( ( t * 1000 ) + ( 999 - ( ( timeStruct.SubSeconds *999) /  hrtc.Init.SynchPrediv  ) ) );  // get time en ms
}

uint32_t McuSTM32L4::RtcGetTimeSecond( void )
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

UART_HandleTypeDef huart;
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, ( 1 << ( UART_TX & 0x0F ) )  | ( 1 << ( UART_RX & 0x0F ) ) );
  }
  if(uartHandle->Instance==USART1) {
    __HAL_RCC_USART1_CLK_DISABLE();
     HAL_GPIO_DeInit(GPIOA, ( 1 << ( UART_TX & 0x0F ) )  | ( 1 << ( UART_RX & 0x0F ) ) );
  }

} 

/********************************************************************/
/*              END  Of  Utilities for UART                         */
/********************************************************************/

/******************************************************************************/
/*                           Mcu Uart Api                                     */
/******************************************************************************/

 
#if DEBUG_TRACE == 1
void vprint(const char *fmt, va_list argp)
{

    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }

}
#endif


void McuSTM32L4::UartInit ( void ) {
//#if DEBUG_TRACE == 1

  GPIO_InitTypeDef GPIO_InitStruct;
  if( UART_NUM==USART2 ) {
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  huart.Instance = UART_NUM;
  huart.Init.BaudRate = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart) != HAL_OK){
  }

};
//#endif

void McuSTM32L4::MMprint( const char *fmt, ...){
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


void McuSTM32L4::MMStoreBuffer( const char *fmt, ...){
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

void McuSTM32L4::MMClearDebugBufferRadioPlaner ( void ) {
#if DEBUG_TRACERP == 1
    BufferDebugRadioPlaner = "";
#endif 
};
void McuSTM32L4::MMPrintBuffer ( void ) {
#if DEBUG_TRACERP == 1
    HAL_UART_Transmit(&huart, (uint8_t*)BufferDebugRadioPlaner.c_str(), strlen(BufferDebugRadioPlaner.c_str()), 0xffffff); // send message via UART
    MMClearDebugBufferRadioPlaner();
#endif 
};

void sleepAndWakeUp (void) {
  /* 
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();GPIO_PIN_ALL
  __HAL_RCC_GPIOH_CLK_DISABLE();*/
  HAL_SPI_DeInit (&hspi1);
  HAL_UART_DeInit (&huart);
  HAL_PWREx_EnterSTOP2Mode ( PWR_STOPENTRY_WFI );
  HAL_Init();
  SystemClock_Config();
  //MX_GPIO_Init();
  mcu.UartInit();
  mcu.InitSpi();
}


void sleepAndWakeUpCool (void) {
  
  HAL_PWREx_EnterSTOP2Mode ( PWR_STOPENTRY_WFI );
  SystemClock_Config();

}
/******************************************************************************/
/*                                Mcu Sleep Api                               */
/******************************************************************************/
void McuSTM32L4::GotoSleepSecond (int duration ) {
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

void McuSTM32L4::GotoSleepMSecond (int duration ) {
#if LOW_POWER_MODE == 1
    WakeUpAlarmMSecond ( duration );
    sleepAndWakeUpCool();
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
void McuSTM32L4::WatchDogStart ( void ) {

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
void McuSTM32L4::WatchDogRelease ( void ) {
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

void McuSTM32L4::LowPowerTimerDisableIrq ( ) {
     HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
}
void McuSTM32L4::LowPowerTimerEnableIrq ( ) {
     HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
}
/********************************************************************/
/*       END  Of  Utilities for LowPower Timer                      */
/********************************************************************/
void McuSTM32L4::LowPowerTimerLoRaInit ( ) {
    hlptim1.Instance = LPTIM1;
    hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV4;
    hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
    hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
    hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
    if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
    }
    Func = DoNothing;
    obj = NULL;
};
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
void McuSTM32L4::StartTimerMsecond ( void (* _Func) (void *) , void * _obj, int delay){

   // int DelayMs2tick = delay * 2 + ( ( 6 * delay ) >> 7);
    //HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 65535, DelayMs2tick); // MCU specific
    //Func =  _Func ;
    
    //obj  = _obj;

    uint32_t DelayMs2tick;
    if (LOW_SPEED_CLK == LSI ) {
        uint32_t mult = LSI_VALUE / 16 ;
        DelayMs2tick = (delay * mult)/1000; // LSI VALUE / LPTIM_PRESCALER_DIV16  
    } else {   
        uint32_t mult = LSE_VALUE / 4 ;
        DelayMs2tick = (delay * mult ) / 1000;
    }

    HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 65535, DelayMs2tick); // MCU specific
    Func =  _Func ;
    obj  = _obj;
};

void McuSTM32L4::StopTimerMsecond ( void ) {

    HAL_LPTIM_TimeOut_Stop_IT (&hlptim1 );
}



void  McuSTM32L4::AttachInterruptIn       (  void (* _Funcext) (void *) , void * _objext) {
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


