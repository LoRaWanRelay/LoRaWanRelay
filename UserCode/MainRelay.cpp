#include "ApiMcu.h"
#include "Define.h"
#include "PointToPointReceiver.h"
#include "PointToPointTransmitter.h"
#include "RadioPlaner.h"
#include "UserDefine.h"
#include "appli.h"
#include "main.h"
#include "utilities.h"
#include "LoraMacDataStoreInFlash.h"
#include "LoraWanProcess.h"
#include "Relay.h"
#include "Sensor.h"

#define MAX_PYLOAD_TO_RELAY       255
#define TX_ON_RX3_ID              0
#define LP_HOOK_ID                1
#define POINT_TO_POINT_TX_HOOK_ID 2
#define POINT_TO_POINT_RX_HOOK_ID 3
#define FW_VERSION                18
#define PERIOD_STATUS             300
#define WAKEUPDURATION            600 
#define JOIN_PERIOD               6 //(seconds)

Relay         relay;
#ifndef RELAY
    LSM303H_ACC Accelero       ( PA_3 );
#endif

#define FileId 4
int16_t  RxcSnr ;
int16_t  RxcRssi ;
uint32_t PeriodicSchedule ; 
uint8_t  UserTxPeriodicPayload [125];
uint8_t  UserTxPeriodicPayloadSize;
uint8_t  UserRxPayload [125];
uint8_t  UserRxPayloadSize;

/* specific callback on thread Number 0 - Do nothing*/ 
void CallBackTxOnRx3 ( void * RadioPlanerIn) {
}

int mainRelay( void ) {

uint8_t LoRaMacNwkSKeyInit[]      = { 0x22, 0x33, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t LoRaMacAppSKeyInit[]      = { 0x11, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
uint8_t LoRaMacAppKeyInit[]       = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xBB};
uint8_t AppEuiInit[]              = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xff, 0x50 };
uint8_t DevEuiInit2[]             = { 0x38, 0x35, 0x31, 0x31, 0x18, 0x47, 0x37, 0x51 };
int i;
uint8_t UserRxFport ; 

#ifdef RELAY
    uint32_t LoRaDevAddrInit   = 0x260114FC;
    uint8_t DevEuiInit[]       = { 0x38, 0x35, 0x31, 0x31, 0x18, 0x47, 0x37, 0x57 };    
    sLoRaWanKeys  LoraWanKeys  = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, LoRaDevAddrInit,OTA_DEVICE };
#else
    uint8_t UserFport = 3 ;
    uint32_t LoRaDevAddrInit   = 0x26011695;
    sLoRaWanKeys  LoraWanKeys  = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit2, LoRaDevAddrInit,APB_DEVICE };
#endif

mcu.InitMcu();

#ifdef SX126x_BOARD
    #define FW_VERSION     0x18
    SX126x  RadioUser( LORA_BUSY, LORA_CS, LORA_RESET,TX_RX_IT );
    RadioPLaner < SX126x > RP( &RadioUser );
    LoraWanObject<LoraRegionsEU,SX126x> Lp ( LoraWanKeys,&RP,USERFLASHADRESS); 
#endif
#ifdef SX1276_BOARD
    SX1276  RadioUser( LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT);
    RadioPLaner < SX1276 > RP( &RadioUser );
    LoraWanObject<LoraRegionsEU,SX1276> Lp ( LoraWanKeys,&RP,USERFLASHADRESS); 
#endif
#ifdef SX1272_BOARD
    #define FW_VERSION     0x13
    SX1272  RadioUser( LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT);
    RadioPLaner < SX1272 > RP( &RadioUser );
    LoraWanObject<LoraRegionsEU,SX1272> Lp ( LoraWanKeys,&RP,USERFLASHADRESS); 
#endif

PointToPointReceiver    ptpRx(&RP, POINT_TO_POINT_RX_HOOK_ID);
PointToPointTransmitter ptpTx(&RP, POINT_TO_POINT_TX_HOOK_ID);

/* Create 4 Threads fom highest to lowest priority,
    Thread 0 : Tx (on RX3) from the relay to Sensor in case of downlink , use on Relay device
    Thread 1 : LoraWan Stack. Use both on relay device and sensor device, on sensor device the stack take in charge the RX3 windows
    Thread 2 : Transmit Wake up sequence , only use on Sensor device
    Thread 3 : Rx cad , only use by Relay Device 
*/
RP.InitHook ( TX_ON_RX3_ID             , &CallBackTxOnRx3                     , reinterpret_cast <void * > (&RP) );
RP.InitHook ( LP_HOOK_ID               , &(Lp.packet.Phy.CallbackIsrRadio)    , &(Lp.packet.Phy)                 );
RP.InitHook ( POINT_TO_POINT_TX_HOOK_ID, &(PointToPointTransmitter::Callback ), reinterpret_cast<void*>(&ptpTx)  );
RP.InitHook ( POINT_TO_POINT_RX_HOOK_ID, &(PointToPointReceiver::Callback )   , reinterpret_cast<void*>(&ptpRx)  );


RadioUser.Reset();
mcu.GotoSleepMSecond ( 300 );
DEBUG_MSG("Init  Done\n");
uint8_t AvailableRxPacket       = NO_LORA_RXPACKET_AVAILABLE ;
eLoraWan_Process_States LpState = LWPSTATE_IDLE;  
/*Just Useful for debug purpose of the RadioPlaner*/
mcu.MMClearDebugBufferRadioPlaner ( );


uint8_t UserPayloadSizeClassA ;
uint8_t UserPayloadClassA [ 250 ];
/* The following code is specific to the Relay Device*/

#ifdef RELAY
    uint8_t FportRelay          = 3; 
    SStatisticRP  PowerStat;
    STask         Tx4Rx3Task;
    Tx4Rx3Task.HookId                              = TX_ON_RX3_ID;
    Tx4Rx3Task.TaskDuration                        = 200; // tbupdated with Timeonair
    Tx4Rx3Task.State                               = TASK_SCHEDULE;
    Tx4Rx3Task.TaskType                            = TX_LORA;
    
    uint8_t PayloadToRelay[MAX_PYLOAD_TO_RELAY]    = { 0x00 };
    uint8_t PayloadToRelay_size                    = 0;
    bool SendDevaddrStatus                         = true;
    bool SendDevEuiStatus                          = false;
    
    uint8_t StatusFport                            = 4;
    bool JoinOnGoing                               = false;
    uint8_t toggle                                 = 1;
    int cpt                                        = 200;
    uint8_t MsgTypeClassA                          = UNCONF_DATA_UP;
    uint32_t RxAppTime                             = 0;
    uint8_t CurrentJoinDevEui[8];
    memset( CurrentJoinDevEui , 0 ,8);

    Lp.Init ();
    Lp.SetDataRateStrategy ( MOBILE_LOWPER_DR_DISTRIBUTION );
    Lp.NewJoin();
    //Lp.RestoreContext  ( ); // to ReStore lorawan context from flash  and so avoid rejoin
    /* white List 2 devices one in OTA one in APB */
    relay.AddDevaddrInWhiteList    ( 0x26011695 ); 
    relay.AddDevEuiInJoinWhiteList ( DevEuiInit2 );   
// Normal Join for the Relay Itself , unuseful in case of APB 
    while ( ( Lp.IsJoined ( ) == NOT_JOINED ) && ( Lp.GetIsOtaDevice ( ) == OTA_DEVICE) ) {   
        LpState  = Lp.Join( mcu.RtcGetTimeMs() + 200 );
        while ( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) ) {
            LpState = Lp.LoraWanProcess( &AvailableRxPacket );
            mcu.GotoSleepMSecond ( 400 );
            mcu.WatchDogRelease  ( );
        }
        mcu.GotoSleepSecond ( JOIN_PERIOD );
    } 
// At this step the relay is Joined , Start CAD process
    DEBUG_MSG("Join Done\n");
    ptpRx.Start( ); // Start Cad Reception thread 
    while(1) {
        uint8_t DevOrDevEui[8];
        uint8_t DevAddrOrDevEUILength;
        uint32_t Freq4RX3; 
        ptpRx.GetRxPayload ( PayloadToRelay, &PayloadToRelay_size, &RxAppTime, &DevOrDevEui[0], &DevAddrOrDevEUILength,&Freq4RX3 );
        if ( PayloadToRelay_size > 0 ) {
        //Case : Relay Catch a payload from a sensor ,  this sensor is already joined  + in the White List
            if (DevAddrOrDevEUILength == 4) { // CAse not a join 
                uint32_t ReceiveDevaddr = (DevOrDevEui [0] << 24) + ( DevOrDevEui [1] << 16 )+ ( DevOrDevEui [2] << 8 )+ DevOrDevEui [3] ;
                if ( relay.IsWhiteListedDevaddr(ReceiveDevaddr) == YES ) { // could be remove because Function GetRxPayload already filter the non White listed Device 
                    DEBUG_PRINTF ( "devaddr = %x\n", ReceiveDevaddr);
                    LpState  = Lp.SendPayload( FportRelay, PayloadToRelay, PayloadToRelay_size, MsgTypeClassA , mcu.RtcGetTimeMs () + 200 );
                    relay.SetConfigForRx3 ( RxAppTime , ReceiveDevaddr, Freq4RX3);
                    cpt               = PERIOD_STATUS;
                    SendDevaddrStatus = false ;
                    SendDevEuiStatus  = false;
                }
        //Case :  Relay Catch a payload from a sensor ,  this sensor is not yet Joined but in the White List
            } else { // case join
                LpState  = Lp.SendPayload( FportRelay, PayloadToRelay, PayloadToRelay_size, MsgTypeClassA , mcu.RtcGetTimeMs () + 200 );
                relay.SetConfigForRx3 ( RxAppTime , DevOrDevEui, Freq4RX3);
                cpt               = PERIOD_STATUS;
                SendDevaddrStatus = false ;
                SendDevEuiStatus  = false;
                memcpy (CurrentJoinDevEui, DevOrDevEui, 8);
                JoinOnGoing = true;
            }
        } else if ( SendDevaddrStatus == true ) {
        // Case No Payload received from the relay but it is time to send a Devaddr Status Payload
            relay.buildStatus( UserPayloadClassA, &UserPayloadSizeClassA );
            LpState  = Lp.SendPayload( StatusFport, UserPayloadClassA, UserPayloadSizeClassA, MsgTypeClassA , mcu.RtcGetTimeMs () + 2000 );
            cpt = 0;
            toggle = 1; 
            SendDevaddrStatus = false ;
        } else if  ( SendDevEuiStatus == true ) {
        // Case No Payload received from the relay but it is time to send a DevEui Status Payload
            cpt = 0 ;
            toggle = 0;
            relay.buildJoinStatus( UserPayloadClassA, &UserPayloadSizeClassA, RP.GetStatistic() );
            LpState  = Lp.SendPayload( StatusFport+1, UserPayloadClassA, UserPayloadSizeClassA, MsgTypeClassA , mcu.RtcGetTimeMs () + 2000 );
            SendDevEuiStatus = false;
        }
    // If a LoRawan frame is already sent, MiniMouse have to be call periodically   
        while ( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) ) {
            LpState = Lp.LoraWanProcess( &AvailableRxPacket );
            mcu.GotoSleepMSecond ( 200 );
            mcu.WatchDogRelease  ( );
        }
    // Relay Recaive a downlink 
        if ( AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) { 
            AvailableRxPacket  = NO_LORA_RXPACKET_AVAILABLE ;
            InsertTrace ( __COUNTER__, FileId );
            Lp.ReceivePayload( &UserRxFport, UserRxPayload, &UserRxPayloadSize );
            DEBUG_PRINTF("Receive on port %d  an Applicative Downlink \n DATA[%d] = [ ",UserRxFport,UserRxPayloadSize);
                    for ( i = 0 ; i < UserRxPayloadSize ; i++){
                        DEBUG_PRINTF( "0x%.2x ",UserRxPayload[i]);
                    }
            if ( UserRxPayloadSize > 0) {
        // Case : Cmd to add devAddr in whiteList
                if ( ( UserRxPayloadSize == 4 ) && (UserRxFport == 1)  ) {
                    relay.AddDevaddrInWhiteList ( UserRxPayload [3] + (UserRxPayload [2] << 8) + (UserRxPayload [1] << 16) + (UserRxPayload [0] << 24) );
                    relay.RemoveDevaddrInBlackList ( UserRxPayload [3] + (UserRxPayload [2] << 8) + (UserRxPayload [1] << 16) + (UserRxPayload [0] << 24) );
                    SendDevaddrStatus = true;
                }
        // Case : Cmd to add devEui in whiteList
                if ( ( UserRxPayloadSize == 8 ) && (UserRxFport == 1)  ) {
                    relay.AddDevEuiInJoinWhiteList ( UserRxPayload );
                    relay.RemoveDevEuiInJoinBlackList ( UserRxPayload );
                    SendDevEuiStatus = true;
                }
        // Case : Cmd to Remove devAddr in whiteList
                if ( ( UserRxPayloadSize == 4 ) && (UserRxFport == 2) ) {
                    uint32_t Removedevaddr =  UserRxPayload [3] + (UserRxPayload [2] << 8) + (UserRxPayload [1] << 16) + (UserRxPayload [0] << 24);
                    relay.RemoveDevaddrInWhiteList ( Removedevaddr );
                    SendDevaddrStatus = true;
                }
        // Case : Cmd to Remove devEui in whiteList      
                if ( ( UserRxPayloadSize == 8 ) && (UserRxFport == 2) ) {
                    relay.RemoveDevEuiInJoinWhiteList ( UserRxPayload );
                    SendDevEuiStatus = true;
                }
        // Case : DownLink for Sensor Join Response or Applicative/Mac Management Downlink 
                if ( UserRxPayloadSize > 9 )   {
                    uint32_t ReceiveDevaddr = relay.ExtractDevaddrDownLink(UserRxPayload);
                    if ( ( relay.IsRx3Activated ( ReceiveDevaddr ) == YES ) && ( relay.GetConfigForRx3 ( &(Tx4Rx3Task.StartTime) , ReceiveDevaddr, &(ptpRx.Tx4Rx3Param.Frequency) ) == OK ) ) {
                        RP.EnqueueTask (Tx4Rx3Task, UserRxPayload, &UserRxPayloadSize, ptpRx.Tx4Rx3Param ); //@tbd RadioPlaner  timeonair
                        relay.ClearRx3Activation (RX3_DISABLE, ReceiveDevaddr  ); 
                    } else if ( JoinOnGoing == true ){ // Have to be robustified with a token 
                        if ( ( relay.IsRx3Activated ( CurrentJoinDevEui ) == YES ) && ( relay.GetConfigForRx3 ( &(Tx4Rx3Task.StartTime), CurrentJoinDevEui,&(ptpRx.Tx4Rx3Param.Frequency) ) == OK ) ) {
                        RP.EnqueueTask (Tx4Rx3Task, UserRxPayload, &UserRxPayloadSize, ptpRx.Tx4Rx3Param ); //@tbd RadioPlaner  timeonair
                        relay.ClearRx3Activation (RX3_DISABLE, CurrentJoinDevEui  );  
                        JoinOnGoing = false;
                        } 
                    }         
                }
            } else {
        // Case  Other AcK Downlink for the relay itself
                DEBUG_MSG ("Receive Ack \n");
            }
        }
    // SEND periodically Status     
        cpt++;
        if ( ( cpt >= PERIOD_STATUS ) && (toggle == 1) ) {
            SendDevEuiStatus  =  true  ;
            SendDevaddrStatus =  false ;
        }
        if ( ( cpt >= PERIOD_STATUS ) && (toggle == 0) ) {
            SendDevaddrStatus = true  ;
            SendDevEuiStatus  = false  ;
        }
        mcu.GotoSleepMSecond ( 2000 );
    }
/* The following code is specific to the Sensor Device*/    
#else
#ifdef BOARD_WITH_SENSOR
    SHT21 Sht21 ( PB_14 );
    Sht21.PowerSht();
#endif    
    bool start_tx        = true; 
    uint8_t DummyPayloadSend[10];
    uint8_t DummyPayloadSize = 10;
    Lp.Init ();
    RadioUser.Sleep( true );
    mcu.GotoSleepMSecond ( 100 );
    Lp.SetDataRateStrategy ( USER_DR_DISTRIBUTION );
    Lp.ActivateRX3 ();
    Accelero.Running =0;
#ifdef BOARD_WITH_SENSOR     
    mcu.InitGpioOut (PA_5);
    mcu.SetValueDigitalOutPin (PA_5, 1); // switch in iddle mode the TimeOf flight sensor
    Accelero.Running =1;
    mcu.mwait_ms (200);
    Accelero.InitConfig (); 
    Accelero.ReadStatusAccelero();
    Accelero.ClearIrqAccelero();
#endif  
    uint32_t RunningTime = 0 ;
    ptpTx.SetDevAddr ( DevEuiInit2, 8 );
    uint8_t FirstJoin = 0;
    UserPayloadClassA[0]  = 0x17 ;
    UserPayloadSizeClassA = 6; 
    uint32_t next_start = mcu.RtcGetTimeMs();
    while (1) {
        #ifdef BOARD_WITH_SENSOR   
            if ( Accelero.Running == 1) {                                 // wake up in interrupt coming from accelero
                if ( Accelero.AcceleroWup () )  {
                    Accelero.ClearIrqAccelero();
                    DEBUG_MSG ("Wake up\n");
                    DEBUG_PRINTF("status accelero = %d\n",Accelero.ReadStatusAccelero());
                    UserPayloadClassA[5] = Accelero.Temperature ; 
                    DEBUG_PRINTF("status accelero Temp  = %d\n", UserPayloadClassA[5] );
                    Accelero.PowerOffLSM303H_ACC ();
                    Accelero.Running = 0;
                    RunningTime = mcu.RtcGetTimeSecond ();
                    start_tx = true;
                }
            } 
        #endif
        if   ( Accelero.Running == 0 ) {                                                        // Normal mode send payload in lora + relay 
            if ( start_tx ) {
                DEBUG_MSG ("lora\n");
                start_tx = false;
                ptpTx.SetChannelDr (  Lp.GetNextFrequency ( ), Lp.GetNextDataRate  ( ) );
                uint32_t NextSendSlot = ptpTx.Start(DummyPayloadSend, DummyPayloadSize);
                if ( ( Lp.IsJoined ( ) == NOT_JOINED ) && ( Lp.GetIsOtaDevice ( ) == OTA_DEVICE) ) {       
                    LpState  = Lp.Join( NextSendSlot );
                    FirstJoin = 0;
                } else {
                    if ( FirstJoin == 0 ) {
                        ptpTx.SetDevAddr( Lp.GetDevAddr() );
                        UserPayloadClassA[1] =  ( Lp.GetDevAddr() >> 24 );
                        UserPayloadClassA[2] =  ( Lp.GetDevAddr() >> 16 ) & 0xFF;   
                        UserPayloadClassA[3] =  ( Lp.GetDevAddr() >> 8 ) & 0xFF;
                        UserPayloadClassA[4] =  ( Lp.GetDevAddr() ) & 0xFF;
                        //ptpTx.SetDevAddr( LoRaDevAddrInit );
                        ptpTx.ClearDevEui ();
                        FirstJoin = 1;
                    }
                    if ( AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) { 
                        AvailableRxPacket  = NO_LORA_RXPACKET_AVAILABLE ;
                        InsertTrace ( __COUNTER__, FileId );
                        Lp.ReceivePayload( &UserRxFport, UserRxPayload, &UserRxPayloadSize );
                        if ( UserRxPayloadSize > 0) {
                            DEBUG_PRINTF("Receive on port %d  an Applicative Downlink \n DATA[%d] = [ ",UserRxFport,UserRxPayloadSize);
                            for ( i = 0 ; i < UserRxPayloadSize ; i++){
                                DEBUG_PRINTF( "0x%.2x ",UserRxPayload[i]);
                            }
                        } else {
                            DEBUG_MSG ("Receive Ack \n");
                        }
                        LpState  = Lp.SendPayload( UserFport, UserRxPayload, UserRxPayloadSize, UNCONF_DATA_UP,NextSendSlot );
                    } else {
                        LpState  = Lp.SendPayload( UserFport, UserPayloadClassA, UserPayloadSizeClassA, UNCONF_DATA_UP,NextSendSlot );
                    }
                }
            }
            while ( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID ) ) {
                LpState = Lp.LoraWanProcess( &AvailableRxPacket );
                mcu.GotoSleepMSecond ( 200 );
                mcu.WatchDogRelease  ( );
            }
            if ( ( (mcu.RtcGetTimeSecond () - RunningTime ) > WAKEUPDURATION ) &&  ( Accelero.Running == 0 )) { // Goto sleep and wait it from accelero
                #ifdef BOARD_WITH_SENSOR
                    Accelero.Running = 1;
                    Accelero.PowerLSM303H_ACC ();
                    Accelero.InitConfig (); 
                    Accelero.ReadStatusAccelero();
                #else
                    RunningTime = mcu.RtcGetTimeSecond ();
                #endif
            } else {
                if((int32_t)(next_start - mcu.RtcGetTimeMs() ) <= 0){
                    start_tx = true;
                    next_start = mcu.RtcGetTimeMs() + 20000;
                }
            }
        }
    mcu.GotoSleepMSecond ( 5000 );
    }
#endif
}