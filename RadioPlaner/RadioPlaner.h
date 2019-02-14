/*

______          _ _      ______ _                       
| ___ \        | (_)     | ___ \ |                      
| |_/ /__ _  __| |_  ___ | |_/ / | __ _ _ __   ___ _ __ 
|    // _` |/ _` | |/ _ \|  __/| |/ _` | '_ \ / _ \ '__|
| |\ \ (_| | (_| | | (_) | |   | | (_| | | | |  __/ |   
\_| \_\__,_|\__,_|_|\___/\_|   |_|\__,_|_| |_|\___|_|   
                                                        
                                                  
                                                   
Description       : RadioPlaner objets.  

License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin  (SEMTECH)
 * \remark        : 
 * Each Tasks enqueued inside the radio planer should Provide :
 *              Task descriptor : 
                                    struct STask {
                                        uint8_t           HookId ;
                                        uint32_t          StartTime ; // absolute Ms
                                        uint32_t          TaskDuration  ;  
                                        eTimingTypeTask   TaskTimingType ;
                                        eRadioPlanerTask  TaskType  ; 
                                    };

                RadioParameter:     struct SRadioParam {
                                            uint32_t             Frequency;
                                            eBandWidth           Bw;
                                            uint8_t              Sf;
                                            uint8_t              Power;
                                            eCrcMode             CrcMode;
                                            eIqMode              IqMode;
                                            eHeaderMode          HeaderMode;
                                            uint8_t              PreambuleLength;
                                            eModulationType      Modulation;
                                };
                BufferParameter : *Buffer + BufferSize 
         
                                                                                                
*/
#ifndef RADIOPLANER_H
#define RADIOPLANER_H
#include "Define.h"
#include "DefineRadioPlaner.h"


template < class R > 
class RadioPLaner  { 
public:
    RadioPLaner( R* RadioUser );
    ~RadioPLaner ( ); 
    eHookStatus  InitHook         ( uint8_t HookIdIn,  void (* AttachCallBack) (void * ), void * objHookIn ) ;
    eHookStatus  GetMyHookId      ( void * objHookIn, uint8_t& HookIdIn );
    eHookStatus  EnqueueTask      ( STask& staskIn, uint8_t *payload, uint8_t* payloadSize, SRadioParam& sRadioParamIn );
    eHookStatus  AbortTask        ( STask& staskIn );
    void         GetStatusPlaner  ( uint8_t HookIdIn, uint32_t& IrqTimestampMs, ePlanerStatus& PlanerStatus );
    SStatisticRP GetStatistic     ( void ) { //sStatisticRP.PrintStat ( ) ;  
        return ( sStatisticRP ) ; 
    };
 
private :

  R*                Radio;     
  STask             sPriorityTask;
  DECLARE_ARRAY ( SRadioParam   , NB_HOOK, sRadioParam       );
  DECLARE_ARRAY ( STask         , NB_HOOK, sTask             );
  DECLARE_ARRAY ( uint8_t*      , NB_HOOK, Payload           );
  DECLARE_ARRAY ( uint8_t*      , NB_HOOK, PayloadSize       );
  DECLARE_ARRAY ( uint8_t       , NB_HOOK, Ranking           );
  DECLARE_ARRAY ( void*         , NB_HOOK, objHook           );
  DECLARE_ARRAY ( ePlanerStatus , NB_HOOK, RadioPlanerStatus );
  DECLARE_ARRAY ( uint32_t      , NB_HOOK, IrqTimeStampMs    );


  SStatisticRP         sStatisticRP;
  uint8_t              HookToExecute;
  uint32_t             TimeOfHookToExecute;
  ePlanerTimerState    PlanerTimerState;
  uint8_t              RadioTaskId;  
  uint8_t              TimerTaskId;
  uint8_t              SemaphoreRadio;
  uint8_t              SemaphoreAbortRadio;
  uint32_t             TimerValue;
  uint8_t              TimerHookId ;
  eGetNextStateStatus  GetNextStateStatus ; 
/************************************************************************************/
/*                                 Planer Utilities                                 */
/*                                                                                  */
/************************************************************************************/
  void                UpdateTimeTaskASAP              ( uint32_t CurrentTimeIn ); 
  void                CallPlanerArbitrer              ( std::string  WhoCallMe ); 
  void                GetIRQStatus                    ( uint8_t HookIdIn ); 
  void                ComputeRanking                  ( void ); 
  void                LaunchCurrentTask               ( void ); 
  uint8_t             SelectPriorityTask              (  uint32_t Now  ); 
  eGetNextStateStatus GetNextTask                     ( uint32_t& duration, uint8_t& TaskIdOut, uint32_t NowIn ) ;
  uint8_t             FindHighestPriority             ( uint8_t * vec, uint8_t length );
  eHookStatus         ReadRadioFifo                   ( STask TaskIn );
  void                SetAlarm                        ( uint32_t alarmInMs ); 
  void                IsrTimerRadioPlaner             ( void );
  void                IsrRadioPlaner                  ( void ); // Isr routine implemented in IsrRoutine.cpp file
  void                AbortTaskInRadio                ( void );
  void                CallAbortedTask                 ( void );
  void                (* AttachCallBackHook[NB_HOOK]) (void * ) ;
  static void         CallbackIsrTimerRadioPlaner     ( void * obj )   { ( reinterpret_cast<RadioPLaner<R>*>(obj) )->IsrTimerRadioPlaner(); };
  static void         CallbackIsrRadioPlaner          ( void * obj )   { ( reinterpret_cast<RadioPLaner< R >*>(obj))->IsrRadioPlaner();} ; 
  void                CallBackHook                    (uint8_t HookId) { AttachCallBackHook[HookId] ( objHook[HookId] ); };  
/************************************************************************************/
/*                                 DEBUG Utilities                                 */
/*                                                                                  */
/************************************************************************************/
void PrintTask ( STask TaskIn);

};

#endif
