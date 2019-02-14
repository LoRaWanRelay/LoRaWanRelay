/*

______          _ _      ______ _                       
| ___ \        | (_)     | ___ \ |                      
| |_/ /__ _  __| |_  ___ | |_/ / | __ _ _ __   ___ _ __ 
|    // _` |/ _` | |/ _ \|  __/| |/ _` | '_ \ / _ \ '__|
| |\ \ (_| | (_| | | (_) | |   | | (_| | | | |  __/ |   
\_| \_\__,_|\__,_|_|\___/\_|   |_|\__,_|_| |_|\___|_|   
                                                        
                                                  
                                                   
Description       : RadioPlaner objets.  

License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Matthieu Verdy - Fabien Holin (SEMTECH)
*/
#ifndef DEFINE_RADIOPLANER_H
#define DEFINE_RADIOPLANER_H
#include "Define.h"
#define NB_HOOK 4
#define RadioPlanerTimeOut 20000 // A task couldn't be stay inside the Radioplaner more than 20 second except the background tasks . 
typedef struct SRadioParam {
    uint32_t             Frequency;
    eBandWidth           Bw;
    uint8_t              Sf;
    uint8_t              Power;
    eCrcMode             CrcMode;
    eIqMode              IqMode;
    eHeaderMode          HeaderMode;
    uint16_t             PreambuleLength;
    uint8_t              SyncWord;
    RadioCodingRate_t    CodingRate;
    eModulationType      Modulation;
    uint32_t             TimeOutMs;
    int16_t *            Snr;
    int16_t *            Rssi;
}SRadioParam;



typedef enum {
    RX_LORA,
    RX_FSK, 
    TX_LORA,
    TX_FSK,
    CAD,
    NONE,
}eRadioPlanerTask;

typedef enum {
    TASK_SCHEDULE,
    TASK_ASAP,
    TASK_RUNNING,
    TASK_ABORTED,
    TASK_FINISHED,
}PlanerState;

typedef struct STask {
    uint8_t           HookId ;
    uint32_t          StartTime ; // absolute Ms
    uint32_t          TaskDuration  ;  
    eRadioPlanerTask  TaskType  ;
    PlanerState       State;
    uint8_t           Priority; 
    uint8_t           TokenDebug;
}STask;

typedef enum { 
    PLANER_RX_CRC_ERROR,
    PLANER_CAD_POSITIVE,
    PLANER_CAD_NEGATIVE,
    PLANER_TX_DONE,
    PLANER_RX_PACKET,
    PLANER_RX_TIMEOUT, 
    PLANER_TASK_ABORTED
} ePlanerStatus;

typedef enum { 
    HOOK_OK,
    HOOK_ID_ERROR,
    TASK_ALREADY_RUNNING, 
}eHookStatus;

typedef enum { 
    TIMER_IDLE,
    TIMER_BUSY 
}ePlanerTimerState;

typedef enum{
    NO_MORE_TASK_SCHEDULE, 
    HAVE_TO_SET_TIMER
} eGetNextStateStatus ;

typedef struct SStatisticRP {
    DECLARE_ARRAY ( uint32_t , NB_HOOK, StatTxConsumptionMs );
    DECLARE_ARRAY ( uint32_t , NB_HOOK, StatRxConsumptionMs );
    uint32_t TotalTxConsumptionMs;
    uint32_t TotalRxConsumptionMs;
    uint32_t InternalCounterRx ;
    uint32_t InternalCounterTx ;
    void PrintStat ( void ) { 
        for (int i = 0 ; i < NB_HOOK ; i++) {
            DEBUG_PRINTF ( "Tx Comsumption Hook [%d] = %d \n ", i, StatTxConsumptionMs [ i ] );
        }
        for (int i = 0 ; i < NB_HOOK ; i++) {
            DEBUG_PRINTF ( "Rx Comsumption Hook [%d] = %d \n ", i, StatRxConsumptionMs [ i ] );
        }
            DEBUG_PRINTF ( "Tx Total Comsumption     = %d \n ",TotalTxConsumptionMs );
            DEBUG_PRINTF ( "Rx Total Comsumption     = %d \n ",TotalRxConsumptionMs );
    }
    void InitStat ( void ) {
        for (int i = 0 ; i < NB_HOOK ; i++) {
            StatTxConsumptionMs [ i ] = 0;
            StatRxConsumptionMs [ i ] = 0;
            TotalTxConsumptionMs = 0;
            TotalRxConsumptionMs = 0;
        }
        InternalCounterRx = 0;
        InternalCounterTx = 0;

    } 
    void StartTxCounter ( void ) {
        InternalCounterTx = mcu.RtcGetTimeMs ( );
    }
    void StartRxCounter ( void ) {
        InternalCounterRx = mcu.RtcGetTimeMs ( );
    }
    void UpdateState ( uint32_t TimeStamp, uint8_t HookIdIn ) {
        if ( InternalCounterTx != 0 ) {
            StatTxConsumptionMs [ HookIdIn ] += ( TimeStamp - InternalCounterTx ); // test wrapping ??
            TotalTxConsumptionMs             += ( TimeStamp - InternalCounterTx );
        } 
        if ( InternalCounterRx != 0 ) {
            StatRxConsumptionMs [ HookIdIn ] += ( TimeStamp - InternalCounterRx ); // test wrapping ??
            TotalRxConsumptionMs             += ( TimeStamp - InternalCounterRx );
        } 
        InternalCounterTx = 0;
        InternalCounterRx = 0;
    }
}SStatisticRP;

#define NO_MORE_TASK          0
#define SOMETHING_TO_DO       1


#define MARGIN_DELAY          5  // for 3 ms
#define MARGIN_DELAY_NEG     -500  // for 500 ms
#endif