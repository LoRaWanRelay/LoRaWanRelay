
/*
 _____      _       _ _______    _____      _       _   
|  __ \    (_)     | |__   __|  |  __ \    (_)     | |  
| |__) |__  _ _ __ | |_ | | ___ | |__) |__  _ _ __ | |_ 
|  ___/ _ \| | '_ \| __|| |/ _ \|  ___/ _ \| | '_ \| __|
| |  | (_) | | | | | |_ | | (_) | |  | (_) | | | | | |_ 
|_|   \___/|_|_| |_|\__||_|\___/|_|   \___/|_|_| |_|\__|
                                                          
Description       : PointToPoint objets.  

License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Mathieu Verdi - Fabien Holin  (SEMTECH)                                                    
*/

#include "PointToPointReceiver.h"
#include "Relay.h"
#include "LoRaMacCrypto.h"
#define CAD_DURATION_MS 1
#define ACK_LENGTH 2



PointToPointReceiver::PointToPointReceiver(RadioPLaner<SX1276>* radio_planner,
                                          const uint8_t hook_id)
  : radio_planner(radio_planner)
  , hook_id(hook_id)
  , state(STATE_INIT)
  , last_cad_ms(0)
  , next_cad_ms(0)
  , fragment_length(WAKE_UP_FRAGMENT_LENGTH)
  , ack_length(ACK_LENGTH)
{

  memset(PtPKey, 1, 16) ;
  AddKey = 0x12345678; 
  this->FrequencyList[0] = 863600000;
  this->FrequencyList[1] = 864200000;
  this->channel = this->FrequencyList[0];

  cad_task_param.Bw              = BW125;
  cad_task_param.Sf              = 7;
  cad_task_param.CodingRate      = CR_4_5;
  cad_task_param.CrcMode         = CRC_YES;
  cad_task_param.HeaderMode      = IMPLICIT_HEADER;
  cad_task_param.IqMode          = IQ_NORMAL;
  cad_task_param.Modulation      =  LORA;
  cad_task_param.Power           = 14;
  cad_task_param.PreambuleLength = 32;
  cad_task_param.SyncWord        = 0x34;
  cad_task_param.TimeOutMs       = 0;

  cad_task.HookId                = this->hook_id;
  cad_task.TaskDuration          = CAD_DURATION_MS;
  cad_task.State                 = TASK_SCHEDULE;
  cad_task.TaskType              = CAD;


  rx_wakeup_fragment_task_param.Bw               = BW_PTP;
  rx_wakeup_fragment_task_param.Sf               = SF_PTP;
  rx_wakeup_fragment_task_param.CodingRate       = CR_4_5;
  rx_wakeup_fragment_task_param.CrcMode          = CRC_YES;
  rx_wakeup_fragment_task_param.HeaderMode       = IMPLICIT_HEADER;
  rx_wakeup_fragment_task_param.IqMode           = IQ_NORMAL;
  rx_wakeup_fragment_task_param.Modulation       = LORA;
  rx_wakeup_fragment_task_param.Power            = 14;
  rx_wakeup_fragment_task_param.PreambuleLength  = 976;
  rx_wakeup_fragment_task_param.SyncWord         = 0x34;
  rx_wakeup_fragment_task_param.TimeOutMs        = MAX_PREAMBLE_PTP + WAKE_UP_FRAGMENT_DURATION_MS + 100;
  rx_wakeup_fragment_task_param.Snr              = &SnrRxFragmentTask;;
  rx_wakeup_fragment_task_param.Rssi             = &RssiRxFragmentTask;

  rx_wakeup_fragment_task.HookId                 = this->hook_id;
  rx_wakeup_fragment_task.TaskDuration           = MAX_PREAMBLE_PTP + WAKE_UP_FRAGMENT_DURATION_MS + 100;
  rx_wakeup_fragment_task.State                  = TASK_SCHEDULE;
  rx_wakeup_fragment_task.TaskType               = RX_LORA;


  rx_data_task_param.Bw              = BW125;
  rx_data_task_param.Sf              = 7;
  rx_data_task_param.CodingRate      = CR_4_5;
  rx_data_task_param.CrcMode         = CRC_YES;
  rx_data_task_param.HeaderMode      = EXPLICIT_HEADER;
  rx_data_task_param.IqMode          = IQ_NORMAL;
  rx_data_task_param.Modulation      = LORA;
  rx_data_task_param.Power           = 14;
  rx_data_task_param.PreambuleLength = 8;
  rx_data_task_param.SyncWord        = 0x34;
  rx_data_task_param.TimeOutMs       = 40;
  rx_data_task_param.Snr             = &SnrRxDataTask;
  rx_data_task_param.Rssi            = &RssiRxDataTask;

  rx_data_task.HookId                = this->hook_id;
  rx_data_task.TaskDuration          = 40;
  rx_data_task.State                 = TASK_SCHEDULE;
  rx_data_task.TaskType              = RX_LORA;
 


  Tx4Rx3Param.Bw              = BW125;
  Tx4Rx3Param.Sf              = 7;
  Tx4Rx3Param.CodingRate      = CR_4_5;
  Tx4Rx3Param.CrcMode         = CRC_NO;
  Tx4Rx3Param.HeaderMode      = EXPLICIT_HEADER;
  Tx4Rx3Param.IqMode          = IQ_INVERTED;
  Tx4Rx3Param.Modulation      = LORA;
  Tx4Rx3Param.Power           = 14;
  Tx4Rx3Param.PreambuleLength = 8;
  Tx4Rx3Param.SyncWord        = 0x34;
  Tx4Rx3Param.TimeOutMs       = 40;
  Tx4Rx3Param.Snr             = 0;
  Tx4Rx3Param.Rssi            = 0;

  tx_ack_relay_task_param.Bw              = BW125;
  tx_ack_relay_task_param.Sf              = 7;
  tx_ack_relay_task_param.CodingRate      = CR_4_5;
  tx_ack_relay_task_param.CrcMode         = CRC_YES;
  tx_ack_relay_task_param.HeaderMode      = EXPLICIT_HEADER;
  tx_ack_relay_task_param.IqMode          = IQ_NORMAL;
  tx_ack_relay_task_param.Modulation      = LORA;
  tx_ack_relay_task_param.Power           = 14;
  tx_ack_relay_task_param.PreambuleLength = 8;
  tx_ack_relay_task_param.SyncWord        = 0x34;
  tx_ack_relay_task_param.TimeOutMs       = 10;
  tx_ack_relay_task_param.Rssi            = 0;
  tx_ack_relay_task_param.Snr             = 0;

  tx_ack_relay_task.HookId                = this->hook_id;
  tx_ack_relay_task.TaskDuration          = 58;
  tx_ack_relay_task.State                 = TASK_SCHEDULE;
  tx_ack_relay_task.TaskType              = TX_LORA;
  
  RxBufferAppLength                       = 0;
}

PointToPointReceiver::~PointToPointReceiver() {}

void PointToPointReceiver::Start( void ) {
  this->cad_success = false;
  this->state = STATE_WAIT_CAD_COMPLETION;
  this->last_cad_ms = mcu.RtcGetTimeMs();
  this->ConfigureAndEnqueueNextCad();
}

void PointToPointReceiver::ExecuteStateMachine() {
  switch (this->state) {
    case STATE_INIT: {
      break;
    }

    case STATE_ENQUEUE_CAD: {
      this->ConfigureAndEnqueueNextCad();
      break;
    }

    case STATE_WAIT_CAD_COMPLETION: {
      if (this->cad_success) {
       
        state = STATE_WAIT_RX_WUF_COMPLETION;
        rx_wakeup_fragment_task_param.Frequency = this->channel;
        rx_success = false;
        rx_wakeup_fragment_task.StartTime = mcu.RtcGetTimeMs() + 1;
        radio_planner->EnqueueTask( rx_wakeup_fragment_task, (uint8_t*)&this->fragment.buffer,&this->fragment_length, rx_wakeup_fragment_task_param);
        DEBUG_MSG("CAD SUCESS\n");
      } else {
        this->ConfigureAndEnqueueNextCad();
      }
      break;
    }

    case STATE_WAIT_RX_WUF_COMPLETION: {
      if (this->rx_success) {
        this->rx_success = false;
        eStatusPtP status = DecodeWakeUpSequence ( );
        if (status == ERROR_PTP) {
            this->ConfigureAndEnqueueNextCad();
            DEBUG_MSGRP("Receive a bad WU sequence\n");
        } else {
            this->ack.delay = (rx_done_timestamp - cadTime) & 0xFFFF;
            this->rx_data_task.StartTime = mcu.RtcGetTimeMs() + 1;
            int16_t Rssi = *(this->rx_wakeup_fragment_task_param.Rssi);
            if ( Rssi < (-127) ) {
                Rssi = -127;
            }
            radio_planner->EnqueueTask(rx_data_task, rx_buffer,&rx_buffer_length,rx_data_task_param);
            state = STATE_WAIT_RX_DATA_COMPLETION;
        }  
      } else {
          ConfigureAndEnqueueNextCad();
          DEBUG_MSG("Rx Wuf failed\n");
      }
      break;
    }

    case STATE_WAIT_RX_DATA_COMPLETION: {
      if (this->rx_success) {
          DEBUG_MSG ("rx sucess\n");
          RxBufferAppLength = this->rx_buffer_length ;
          int16_t Rssi = *(this->rx_data_task_param.Rssi);
          if ( Rssi < (-127) ) {
             Rssi = -127;
          }
          uint8_t RssiByte = (uint8_t) (Rssi * (-1));
          RxBufferApp[0] = RssiByte;
          // RxBufferApp[1] is already updated when received the wake up sequence
          memcpy( &RxBufferApp[2], this->rx_buffer, RxBufferAppLength);
          RxBufferAppLength = RxBufferAppLength + 2;
          RxBufferAppTime   = rx_done_timestamp;//mcu.RtcGetTimeMs ();        
          this->rx_success  = false;
          this->tx_ack_relay_task.StartTime = mcu.RtcGetTimeMs() + 1;
          this->tx_ack_relay_task_param.Frequency = this->channel;
          this->radio_planner->EnqueueTask(this->tx_ack_relay_task,this->ack.buffer, &this->ack_length,this->tx_ack_relay_task_param);
          this->state = STATE_WAIT_TX_ACK_COMPLETION;
      } else {
          DEBUG_PRINTFRP("Wakeup id: %i\n", this->wake_up_id);
          DEBUG_MSGRP("Missed data\n");
          this->ConfigureAndEnqueueNextCad();
      }
      break;
    }

    case STATE_WAIT_TX_ACK_COMPLETION: {
      // DEBUG_PRINTF("Wake Up id: %i\n", this->wake_up_id);
      DEBUG_PRINTFRP("Wakeup id: %i\n", this->wake_up_id);
      DEBUG_PRINTFRP("Ack Sent: %i\n", this->delay_indicator);
      this->ConfigureAndEnqueueNextCad();
      break;
    }

    default: {
      DEBUG_MSG("1-> Forgot break?\n");
      this->ConfigureAndEnqueueNextCad();
    }
  }
}

void
PointToPointReceiver::Callback(void* self)
{
  PointToPointReceiver* me = reinterpret_cast<PointToPointReceiver*>(self);
  // DEBUG_PRINTF("  --> State = %i\n", me->state);

  uint32_t irq_timestamp_ms;
  ePlanerStatus planner_status;

  me->radio_planner->GetStatusPlaner(me->hook_id, irq_timestamp_ms,
                                    planner_status);

  switch (planner_status) {
    case PLANER_CAD_POSITIVE: {
      me->cad_success = true;
      me->cadTime = irq_timestamp_ms;
      break;
    }

    case PLANER_CAD_NEGATIVE: {

      break;
    }

    case PLANER_RX_PACKET: {
      me->rx_done_timestamp = irq_timestamp_ms;
      me->rx_success = true;

      break;
    }

    case PLANER_RX_TIMEOUT:
    case PLANER_RX_CRC_ERROR: {
      me->rx_success = false;
      break;
    }

    case PLANER_TX_DONE: {
      break;
    }

    case PLANER_TASK_ABORTED: {
      DEBUG_MSGRP("CA\n");
      me->state = STATE_ENQUEUE_CAD;

      break;
    }

    default: {
      DEBUG_PRINTF("2-> Forgot break?: 0x%x\n", planner_status);
    }
  }
  me->ExecuteStateMachine();
}

uint32_t
PointToPointReceiver::GetNextCadStartMs(const uint32_t lastCadMs)
{
  uint32_t delay_ms = 10;
  uint32_t actual_ms = mcu.RtcGetTimeMs() + delay_ms;
  uint32_t next_cad_start_ms = ((uint32_t)(lastCadMs / CAD_BEAT_MS)) * CAD_BEAT_MS; // Warning: overflow
  while ( (int )( next_cad_start_ms - actual_ms ) <= 0) {
    next_cad_start_ms += CAD_BEAT_MS;
  }
  return next_cad_start_ms;
}

uint32_t
PointToPointReceiver::GetNextFreqency(const uint32_t nextCadMs)
{
  uint8_t frequency_index = ((nextCadMs) / CAD_BEAT_MS) % 2;
  return this->FrequencyList[frequency_index];
}

void
PointToPointReceiver::ConfigureAndEnqueueNextCad()
{
  cad_task_param.Bw = BW125;
  cad_task_param.Sf = 7;
  cad_task_param.CodingRate = CR_4_5;
  cad_task_param.CrcMode = CRC_YES;
  cad_task_param.HeaderMode = IMPLICIT_HEADER;
  cad_task_param.IqMode = IQ_NORMAL;
  cad_task_param.Modulation = LORA;
  cad_task_param.Power = 14;
  cad_task_param.PreambuleLength = 32;
  cad_task_param.SyncWord = 0x34;
  cad_task_param.TimeOutMs = 0;
  
  this->cad_success = false;
  this->state = STATE_WAIT_CAD_COMPLETION;
  this->next_cad_ms = this->GetNextCadStartMs(this->last_cad_ms);
  this->last_cad_ms = this->next_cad_ms;
  this->channel = this->GetNextFreqency(this->next_cad_ms);
  this->cad_task.StartTime = this->next_cad_ms;
  this->cad_task_param.Frequency = this->channel;
  this->radio_planner->EnqueueTask(this->cad_task, NULL, NULL,
                                   this->cad_task_param);
  //  DEBUG_PRINTF("next_cad_ms: %i\n"
  //               "channel: %d\n",
  //               this->next_cad_ms, this->channel);
  //DEBUG_MSG(".");
}

int32_t
PointToPointReceiver::GetDelayIndicatorMs(
  const uint32_t lastCadMs, const uint32_t expectedEndLastWakeUpFragment)
{
  static uint32_t correction_end_packet_middle_preamble_ms = 51;
  int32_t WakeUpSequenceDelay = expectedEndLastWakeUpFragment - lastCadMs -
                                correction_end_packet_middle_preamble_ms;
  while (WakeUpSequenceDelay > CAD_BEAT_PER_CHANNEL_MS) {
    WakeUpSequenceDelay -= CAD_BEAT_PER_CHANNEL_MS;
  }
  return WakeUpSequenceDelay;
}

eStatusPtP PointToPointReceiver::DecodeWakeUpSequence ( ) {
    // compute mic + insert check @tbd
    DEBUG_MSGRP ("Decode Wake up sequence\n");
    eStatusPtP status = OK_PTP ;
    uint32_t mic;
    if (fragment.buffer[0] == WUS_WITH_DEVEUI){
        DevEuiWakeUpSequence[0] =fragment.buffer[1]  ;
        DevEuiWakeUpSequence[1] =fragment.buffer[2]  ;
        DevEuiWakeUpSequence[2] =fragment.buffer[3]  ;
        DevEuiWakeUpSequence[3] =fragment.buffer[4]  ;
        DevEuiWakeUpSequence[4] =fragment.buffer[6]  ;
        DevEuiWakeUpSequence[5] =fragment.buffer[7]  ;
        DevEuiWakeUpSequence[6] = fragment.buffer[9] ;
        DevEuiWakeUpSequence[7] =fragment.buffer[10] ;
        DevLength = 8;
        relay.updateCounter (DevEuiWakeUpSequence,fragment.buffer[5]);
        relay.SetRssiStatus (DevEuiWakeUpSequence, (RssiRxFragmentTask < (-127) ) ? ( 127 ) : (uint8_t)( ( -1 ) * RssiRxFragmentTask ));
        if ( relay.IsJoinWhiteListedDevEui (DevEuiWakeUpSequence ) == NO) {
            relay.AddDevEuiInJoinBlackList (DevEuiWakeUpSequence);
            return (ERROR_PTP);
        }
    } else {
        LoRaMacComputeMic( &(fragment.buffer[0]), 9, PtPKey , AddKey , 0 , 0,&mic );
        uint16_t CheckMic = (fragment.buffer [9] << 8) + ( fragment.buffer [10] );
        if ( CheckMic != (uint16_t)(mic & 0x0000FFFF) ) {    
            DEBUG_PRINTF ( " Receive a bad Mic %x calculted mix %x \n", CheckMic,  (uint16_t)(mic & 0x0000FFFF));
            return (ERROR_PTP);
        }
        uint32_t ReceiveDevAddr = (fragment.buffer [1] << 24) + ( fragment.buffer [2] << 16 )+ ( fragment.buffer [3] << 8 )+  fragment.buffer [4] ;
        relay.SetRssiStatus (ReceiveDevAddr, (RssiRxFragmentTask < (-127) ) ? ( 127 ) : (uint8_t)( ( -1 ) * RssiRxFragmentTask ));
        relay.updateCounter (ReceiveDevAddr,fragment.buffer[5]);
        if (relay.IsWhiteListedDevaddr(ReceiveDevAddr) == NO ) {
            relay.AddDevaddrInBlackList ( ReceiveDevAddr ) ;
            DEBUG_PRINTF ( " Receive a bad WU dev_addr %x\n", ReceiveDevAddr);
      
            return (ERROR_PTP);
        } else {
          memcpy ( DevAddrWakeUpSequence , &fragment.buffer[1], 4) ;
          DevLength = 4;
        }

    }
    wake_up_id = fragment.buffer[5];
    if ( wake_up_id > WAKE_UP_SEQUENCE_LENGTH_MAX) {
        DEBUG_PRINTF ( " Receive a bad Wake up id  %d\n", wake_up_id);
        return (ERROR_PTP);
    }
    rx_data_task_param.Sf        = 12 - (fragment.buffer[8] & 0xF);
    RxBufferApp[1] = fragment.buffer[8];
    if ( ( rx_data_task_param.Sf < 7) ||  ( rx_data_task_param.Sf > 12 ) ) {
        DEBUG_PRINTF ( " Receive a bad SF  %d \n", rx_data_task_param.Sf);
        return (ERROR_PTP);
    }
    rx_data_task_param.TimeOutMs = 40 * ( 1 << (rx_data_task_param.Sf - 7));
    switch ( fragment.buffer[8] >> 4 ) {
      case 0 :
          rx_data_task_param.Frequency = 868100000;
          break;
      case 1 :
          rx_data_task_param.Frequency = 868300000;
          break;
      case 2 :
          rx_data_task_param.Frequency = 868500000;
          break;
      case 3 :
          rx_data_task_param.Frequency = 867100000;
          break;
      case 4 :
          rx_data_task_param.Frequency = 867300000;
          break;
      case 5 :
          rx_data_task_param.Frequency = 867500000;
          break;
      case 6 :
          rx_data_task_param.Frequency = 867700000;
          break;
      case 7 :
          rx_data_task_param.Frequency = 867900000;
          break;
      default :
          DEBUG_PRINTF ( " Receive a bad Frequency  %d", fragment.buffer[8]);
          rx_data_task_param.Frequency = 868100000;
          break;
          //return (ERROR_PTP);
          //break;
    }
DEBUG_PRINTFRP ( "sf = %d , freq = %d\n",rx_data_task_param.Sf,rx_data_task_param.Frequency );
//Tx4Rx3Param.Frequency = rx_data_task_param.Frequency ;
return (status);                         
};


void PointToPointReceiver::GetRxPayload ( uint8_t * RxPayload, uint8_t * PayloadLength, uint32_t *RxTime, uint8_t * DevaddrOut, uint8_t * DevLengthOut , uint32_t *FreqOut ) {
    *PayloadLength = RxBufferAppLength;
    *RxTime        = RxBufferAppTime + ( MAC_RX3_DELAY * 1000 );
    if ( RxBufferAppLength > 0 ) {
        memcpy( RxPayload , RxBufferApp, RxBufferAppLength);
    }
    RxBufferAppLength = 0; // clear Rx buffer; 
    ( DevLength == 4 ) ? memcpy ( DevaddrOut , DevAddrWakeUpSequence , 4)  : memcpy ( DevaddrOut , DevEuiWakeUpSequence , 8) ;
    * DevLengthOut  = DevLength ;
    *FreqOut        = rx_data_task_param.Frequency;
};