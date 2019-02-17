
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

#include "UserDefine.h"
#include "PointToPointTransmitter.h"
#include "LoRaMacCrypto.h"
#include "sx1272.h"
#include "sx1276.h"
#include "SX126x.h"
template class PointToPointTransmitter<SX1276>;
template class PointToPointTransmitter<SX1272>;
template class PointToPointTransmitter<SX126x>;
template <class R> 
PointToPointTransmitter<R>::PointToPointTransmitter(RadioPLaner<R> *radio_planner,  uint8_t hook_id_in) : radio_planner(radio_planner){
    state                 = STATE_INIT;
    count_ack_rx_attempt  = 0;
    count_ack_rx_success  = 0; 
    ack_length            = 2; 
    ChannelIndex          = 0;
    count_wus_tx_attempt  = 0; 
    count_wuf_tx_attempt  = 0;
    count_data_tx_attempt = 0;
    NbMissedAck           = MISSED_ACK_BEFORE_LONG_PREAMBLE;
  
    hook_id               = hook_id_in;
    WakeUpSequenceDelay   = 0;
    memset(PtPKey, 1 , 16) ;
    AddKey = 0x12345678; 
    fragment_length = WAKE_UP_FRAGMENT_LENGTH;

    FrequencyList[0] = 863600000;
    FrequencyList[1] = 864200000;

    wakeup_fragment_task_param.Bw              = BW_PTP;
    wakeup_fragment_task_param.Sf              = SF_PTP;
    wakeup_fragment_task_param.CodingRate      = CR_4_5;
    wakeup_fragment_task_param.CrcMode         = CRC_YES;
    wakeup_fragment_task_param.HeaderMode      = IMPLICIT_HEADER;
    wakeup_fragment_task_param.IqMode          = IQ_NORMAL;
    wakeup_fragment_task_param.Modulation      = LORA;
    wakeup_fragment_task_param.Power           = POWER_PTP;
    wakeup_fragment_task_param.PreambuleLength = ( MAX_PREAMBLE_PTP * (( BW_PTP + 1)*125) ) >> SF_PTP;
    WakeUpSequenceLength                       = MAX_PREAMBLE_PTP + WAKE_UP_FRAGMENT_DURATION_MS;
    wakeup_fragment_task_param.SyncWord        = 0x34;
    wakeup_fragment_task_param.TimeOutMs       = 1000;
    wakeup_fragment_task.HookId                = hook_id;
    wakeup_fragment_task.TaskDuration          = ((wakeup_fragment_task_param.PreambuleLength << SF_PTP)/(( BW_PTP + 1)*125))  + WAKE_UP_FRAGMENT_DURATION_MS;
    wakeup_fragment_task.State                 = TASK_SCHEDULE;
    wakeup_fragment_task.TaskType              = TX_LORA;

    ack_relay_rx_task_param.Bw                 = BW125;
    ack_relay_rx_task_param.Sf                 = 7;
    ack_relay_rx_task_param.CodingRate         = CR_4_5;
    ack_relay_rx_task_param.CrcMode            = CRC_YES;
    ack_relay_rx_task_param.HeaderMode         = EXPLICIT_HEADER;
    ack_relay_rx_task_param.IqMode             = IQ_NORMAL;
    ack_relay_rx_task_param.Modulation         = LORA;
    ack_relay_rx_task_param.Power              = 14;
    ack_relay_rx_task_param.PreambuleLength    = 32;
    ack_relay_rx_task_param.SyncWord           = 0x34;
    ack_relay_rx_task_param.TimeOutMs          = 20;
    ack_relay_rx_task_param.Rssi               = 0;
    ack_relay_rx_task_param.Snr                = 0;

    ack_relay_rx_task.HookId                   = hook_id;
    ack_relay_rx_task.TaskDuration             = 20;
    ack_relay_rx_task.State                    = TASK_SCHEDULE;
    ack_relay_rx_task.TaskType                 = RX_LORA;

    data_send_task_param.Bw                    = BW125;
    data_send_task_param.Sf                    = 7;
    data_send_task_param.CodingRate            = CR_4_5;
    data_send_task_param.CrcMode               = CRC_YES;
    data_send_task_param.HeaderMode            = EXPLICIT_HEADER;
    data_send_task_param.IqMode                = IQ_NORMAL;
    data_send_task_param.Modulation            = LORA;
    data_send_task_param.Power                 = 14;
    data_send_task_param.PreambuleLength       = 8;
    data_send_task_param.SyncWord              = 0x34;
    data_send_task_param.TimeOutMs             = 50;

    data_send_task.HookId                      = hook_id;
    data_send_task.TaskDuration                = 400;
    data_send_task.State                       = TASK_SCHEDULE;
    data_send_task.TaskType                    = TX_LORA;

    NextSendSlot                 = mcu.RtcGetTimeMs();
    last_ack_success_received_ms = NextSendSlot - 1000000 ;// init very far in the past 
    Ftype                        = 0;
    DevAddr                      = 0xFFFFFFFF;
   // DevAddr                     = 0x26011D16;
    CntDnw                       = WAKE_UP_FRAGMENT_LENGTH;
    Fcount                       = 0;
    Channel_Dr                   = 0;
    CadTime                      = 0;
    memset ( DevEUI, 0xFF , 8);
};

template <class R>  PointToPointTransmitter<R>::~PointToPointTransmitter() {}
template <class R>  void PointToPointTransmitter<R>::SetChannelDr ( uint32_t Channel, uint8_t DataRate ) { 
   // DEBUG_PRINTF ( "channel = %d dr = %d\n",Channel,DataRate);
    switch  (Channel ) {
        case 868100000 :
            Channel_Dr =  DataRate  & 0xf ; 
            break;
        case 868300000 :
            Channel_Dr = ( 1<<4 ) +  ((DataRate)  & 0xf) ; 
            break; 
        case 868500000 :
            Channel_Dr = ( 2<<4 ) +  ( (DataRate)  & 0xf) ; 
            break ;
        case 867100000 :
            Channel_Dr = ( 3<<4 ) +  ( (DataRate)  & 0xf) ; 
            break;
        case 867300000 :
            Channel_Dr = ( 4<<4 ) +  ((DataRate)  & 0xf) ; 
            break; 
        case 867500000 :
            Channel_Dr = ( 5<<4 ) +  ( (DataRate)  & 0xf) ; 
            break ;
        case 867700000 :
            Channel_Dr = ( 6<<4 ) +  ((DataRate)  & 0xf) ; 
            break; 
        case 867900000 :
            Channel_Dr = ( 7<<4 ) +  ( (DataRate)  & 0xf) ; 
            break ;
            
        default        :
            Channel_Dr =DataRate  & 0xf ; 
            break;
    }
} ; 
template <class R> 
uint32_t  PointToPointTransmitter<R>::Start(uint8_t *data_payload, const uint8_t data_payload_length)
{
    if (state != STATE_INIT)
    {
        DEBUG_PRINTF("Refuse to start: already running (in state 0x%x)\n", state);
        state = STATE_INIT ;
        return (0);
    }
    DEBUG_PRINTFRP ("start and wk up length = %d\n",WakeUpSequenceLength );
    this->data_payload         = data_payload;
    this->data_payload_length  = data_payload_length;
    Fcount++;
    GetNextSendSlotTimeAndChannel( mcu.RtcGetTimeMs(), WakeUpSequenceDelay, last_ack_success_received_ms,&WakeUpSequenceLength,&NextSendSlot, &ChannelIndex);
    fragment_index = 0;
    PrepareNextWakeUpFragment(&this->fragment, fragment_index);
    uint32_t mic;
    (this->fragment).buffer[5] = 0;
    LoRaMacComputeMic((this->fragment).buffer, 9, PtPKey , AddKey , 0 , 0,&mic );
    MicPtp[0] = mic;
    PrepareNextWakeUpFragment(&this->fragment, this->fragment_index); // to update mic for the first frag


    wakeup_fragment_task_param.Frequency = this->FrequencyList[ChannelIndex];
    data_send_task_param.Frequency       = this->FrequencyList[ChannelIndex];
    ack_relay_rx_task_param.Frequency    = this->FrequencyList[ChannelIndex];
    wakeup_fragment_task.StartTime       = NextSendSlot;
    NextSendSlot  = wakeup_fragment_task.StartTime ;
    wakeup_fragment_task_param.PreambuleLength = ( ( WakeUpSequenceLength- WAKE_UP_FRAGMENT_DURATION_MS) * (( BW_PTP + 1)*125) ) >> SF_PTP;
    wakeup_fragment_task.TaskDuration          = ((wakeup_fragment_task_param.PreambuleLength << SF_PTP)/(( BW_PTP + 1)*125))  + WAKE_UP_FRAGMENT_DURATION_MS;

    this->radio_planner->EnqueueTask(wakeup_fragment_task, (uint8_t *)&fragment, &this->fragment_length, wakeup_fragment_task_param);

    DEBUG_PRINTFRP(
        "Go!\n"
        "  --> WakeUp #Fragments: %i\n"
        "  --> NextSendSlot: %i\n"
        "  --> Actual Time : %i\n"
        "  --> Next channel index: %i\n",
        WakeUpSequenceLength,
        NextSendSlot,
        mcu.RtcGetTimeMs(),
        ChannelIndex);

    this->state = STATE_SEND_WAKEUP_SEQUENCE_FRAGMENT;
    count_wus_tx_attempt++;
    return (NextSendSlot + wakeup_fragment_task.TaskDuration + 3 );
}
template <class R> 
void PointToPointTransmitter<R>::ExecuteStateMachine (void ) {
    eHookStatus hookStatus ;
    DEBUG_PRINTFRP ( "state =  %d\n",this->state);
    switch (this->state) {
    case STATE_INIT:
        break;

    case STATE_SEND_WAKEUP_SEQUENCE_FRAGMENT:
        DEBUG_MSGRP("Send data \n");
        data_send_task.StartTime = mcu.RtcGetTimeMs() + 16 ;  // +20 to be sure that collide with TX from lorawan dedicated to relay implementation not point to point 
        this->state = STATE_WAIT_RELAY_ACK;
        hookStatus = radio_planner->EnqueueTask(data_send_task, data_payload, &data_payload_length, data_send_task_param);
        count_data_tx_attempt++;
        if (hookStatus == HOOK_ID_ERROR) {
            DEBUG_MSGRP("Cannot enqueue: abort\n");
            this->Abort();
            break;
        }
        break;
    case STATE_WAIT_RELAY_ACK:
        ack_relay_rx_task.StartTime = mcu.RtcGetTimeMs() + 3;
        this->rxSuccess = false;
        count_ack_rx_attempt++;
        state = ACK_RECEIVED;
        this->radio_planner->EnqueueTask(ack_relay_rx_task, this->rx_buffer, &this->ack_length, ack_relay_rx_task_param);
        
        break;

    case ACK_RECEIVED:
        if (this->rxSuccess) {
            this->last_ack_success_received_ms = mcu.RtcGetTimeMs();
            this->rxSuccess = false;
            count_ack_rx_success++;
            WakeUpSequenceDelay = this->rx_buffer[0] + (this->rx_buffer[1] << 8);
            CadTime = wakeup_fragment_task.StartTime +  WakeUpSequenceLength - WakeUpSequenceDelay;
            NbMissedAck = 0;
           
        } else {
            NbMissedAck ++;
            DEBUG_MSG("Missed ack!\n");
        }
        this->state = STATE_INIT;
        break;

    default:
        DEBUG_MSG("Forgot break?");
        break;
    }
}
template <class R> 
void PointToPointTransmitter<R>::Abort() {
    this->state                        = STATE_INIT;
    this->last_ack_success_received_ms = 0;
    this->WakeUpSequenceLength         = MAX_PREAMBLE_PTP;
    this->NextSendSlot                 = mcu.RtcGetTimeMs();
    this->ChannelIndex                 = 0;
    this->TxChannel                    = this->FrequencyList[this->ChannelIndex];
    this->RxDone                       = false;
    this->TxDone                       = false;
    this->rxSuccess                    = false;
}
template <class R> 
void PointToPointTransmitter<R>::GetStatistics(StatisticCounters_t *counters){
    counters->ack_rx_attempt = this->count_ack_rx_attempt;
    counters->ack_rx_success = this->count_ack_rx_success;
    counters->wuf_tx_attempt = this->count_wuf_tx_attempt;
    counters->wus_tx_attempt = this->count_wus_tx_attempt;
    counters->data_tx_attempt = this->count_data_tx_attempt;
}
template <class R> 
void PointToPointTransmitter<R>::Callback(void *self)
{
    PointToPointTransmitter *me = reinterpret_cast<PointToPointTransmitter *>(self);
    DEBUG_PRINTFRP("  --> State = %i\n", me->state);

    ePlanerStatus planner_status;

    uint32_t irq_ts_ms;
    me->radio_planner->GetStatusPlaner(me->hook_id, irq_ts_ms, planner_status);
    me->irq_timestamp_ms = irq_ts_ms;
    // DEBUG_PRINTF("  --> Planner status: %i\n  --> IRQ timestamp: %i\n", planner_status, irq_timestamp_ms);

    switch (planner_status)
    {
    case PLANER_RX_PACKET:
    {
        me->rxSuccess = true;
        break;
    }
    case PLANER_RX_CRC_ERROR:
    case PLANER_RX_TIMEOUT:
    {
        me->rxSuccess = false;
        break;
    }
    case PLANER_TASK_ABORTED:
    {   //if ( me->fragment_index > 1) { 
           // DEBUG_PRINTF("PtP aborted, stop all %d\n",me->fragment_index);
           //me->Abort();
        //}
        break;
    }
    case PLANER_TX_DONE:
    {
        break;
    }
    default:
        DEBUG_MSG("Forgot break?\n");
     
    }
    me->ExecuteStateMachine();
}

template <class R> 
void PointToPointTransmitter<R>::GetNextSendSlotTimeAndChannel(const uint32_t actual_time, const int16_t delay_rx, const uint32_t last_ack_success_time, uint16_t *wake_up_sequence_length, uint32_t *next_send_slot, uint8_t *channel_index)
{
    uint32_t t_cad_rx = this->CadTime ;
    //DEBUG_PRINTF("last time = %d \n tcad rx = %d \n Delay reported = : %d , preambule length = %d \n",*next_send_slot, t_cad_rx,delay_rx,*wake_up_sequence_length);
    // Search next send opportunity
    uint16_t next_wake_up_sequence_length;
    ComputeNextWakeUpLength(&next_wake_up_sequence_length, actual_time, last_ack_success_time);
    const uint32_t next_wake_up_sequence_window = (next_wake_up_sequence_length - (WAKE_UP_FRAGMENT_DURATION_MS + MIN_PREAMBULE_DURATION_MS) ) /2 ;
    uint32_t t_cad_next = t_cad_rx;
    uint8_t channel_index_temp = (*channel_index);
    while ( (int) ( (actual_time + next_wake_up_sequence_window  - t_cad_next ) ) > 0) 
    {
        channel_index_temp = !channel_index_temp;
        t_cad_next += CAD_BEAT_MS;
    }
    *next_send_slot = t_cad_next - next_wake_up_sequence_window  + (CAD_BEAT_MS<<1) ;
    *wake_up_sequence_length = next_wake_up_sequence_length;
    *channel_index = channel_index_temp;
}
template <class R> 
void PointToPointTransmitter<R>::PrepareNextWakeUpFragment(WakeUpFragments_t *fragment, const uint8_t fragment_index)
{
    if ( DevAddr != 0xFFFFFFFF) {
        Ftype = WUS_WITH_DEVADDR; 
        fragment->buffer[0]  = Ftype;
        fragment->buffer[1]  = ( DevAddr >> 24 ) & 0xFF;
        fragment->buffer[2]  = ( DevAddr >> 16 ) & 0xFF;
        fragment->buffer[3]  = ( DevAddr >> 8 ) & 0xFF;
        fragment->buffer[4]  =   DevAddr & 0xFF;
        fragment->buffer[5]  = fragment_index ;
        fragment->buffer[6]  = ( Fcount >> 8 ) & 0xFF ; 
        fragment->buffer[7]  = Fcount & 0xFF ;
        fragment->buffer[8]  = Channel_Dr ;
        fragment->buffer[9]  = (MicPtp[fragment_index] >> 8) & 0xFF ;
        fragment->buffer[10] = MicPtp[fragment_index] & 0xFF ;
    } else {
        Ftype = WUS_WITH_DEVEUI; 
        fragment->buffer[0]  = Ftype;
        fragment->buffer[1]  = DevEUI[0];
        fragment->buffer[2]  = DevEUI[1];
        fragment->buffer[3]  = DevEUI[2];
        fragment->buffer[4]  = DevEUI[3];
        fragment->buffer[5]  = fragment_index ;
        fragment->buffer[6]  = DevEUI[4];
        fragment->buffer[7]  = DevEUI[5];
        fragment->buffer[8]  = Channel_Dr ;
        fragment->buffer[9]  = DevEUI[6];
        fragment->buffer[10] = DevEUI[7];
    }
}
template <class R> 
void PointToPointTransmitter<R>::ComputeNextWakeUpLength(uint16_t *nextWakeUpLength, const uint32_t actual_time, const uint32_t last_ack_success_time)
{
    if ( NbMissedAck >= MISSED_ACK_BEFORE_LONG_PREAMBLE ) {
        *nextWakeUpLength = MAX_WUS_DURATION_MS;
    } else {
       uint32_t TimmingErrorMs = ( ( actual_time - last_ack_success_time ) * PPM_OFFSET ) / 1000000 ;
      *nextWakeUpLength = ( WAKE_UP_FRAGMENT_DURATION_MS + MIN_PREAMBULE_DURATION_MS ) + TimmingErrorMs + 16;
    }
}
template <class R> 
 void PointToPointTransmitter<R>::SetDevAddr ( uint8_t* addr, uint8_t Length) {
    if (Length == 8) {
        for (int i = 0 ; i < Length ; i++) {
            DevEUI[i] =  addr [i];
        }
    }
}
template <class R> 
 void PointToPointTransmitter<R>::SetDevAddr ( uint32_t addr){
    DevAddr = addr;
}
template <class R> 
void PointToPointTransmitter<R>::ClearDevEui(void) {
    for (int i = 0 ; i < 8 ; i++) {
            DevEUI[i] =  0xFF;
        }
}