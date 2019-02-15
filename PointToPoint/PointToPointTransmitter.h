
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
#ifndef __POINT_TO_POINT_TRANSMITTER_H__
#define __POINT_TO_POINT_TRANSMITTER_H__
#include "PointToPointBase.h"
#include "RadioPlaner.h"
#include "sx1276.h"
template < class R >  
class PointToPointTransmitter
{
public:
   PointToPointTransmitter(RadioPLaner<R> *radio_planner, const uint8_t hook_id_in);
  
  ~PointToPointTransmitter();

  uint32_t Start(uint8_t *data_payload, const uint8_t data_payload_length);
  void SetChannelDr ( uint32_t Channel, uint8_t DataRate ) ; 
  void SetDevAddr ( uint8_t* addr, uint8_t Length);
  void SetDevAddr ( uint32_t addr);
  void Abort();
  void GetStatistics(StatisticCounters_t *counters);
  static void Callback(void *);
  void ClearDevEui(void);
protected:
  void ExecuteStateMachine();
  void PrepareNextWakeUpFragment(WakeUpFragments_t *fragment, const uint8_t fragment_index);
  void ComputeNextWakeUpLength(uint16_t *nextWakeUpLength, const uint32_t actual_time, const uint32_t last_ack_success_time);
  void GetNextSendSlotTimeAndChannel(const uint32_t actual_time, const int16_t delay_rx, const uint32_t last_ack_success_time, uint16_t *wake_up_sequence_length, uint32_t *next_send_slot, uint8_t *channel_index);

private:
  RadioPLaner<R> *radio_planner;
  uint8_t hook_id;
  State_t state;
  uint32_t SendWakeUpCount;
  volatile int16_t WakeUpSequenceDelay;
  volatile uint32_t last_ack_success_received_ms;
  volatile uint32_t irq_timestamp_ms;
  uint16_t WakeUpSequenceLength;
  uint32_t NextSendSlot;
  uint8_t ChannelIndex;
  uint32_t TxChannel;
  uint32_t FrequencyList[NBR_FREQUENCIES];
  uint8_t DataSf;
  eBandWidth DataBw;
  volatile bool RxDone;
  volatile bool TxDone;
  WakeUpFragments_t fragment;
  uint8_t fragment_length;
  uint8_t fragment_index;
  Acknowledges_t ack;
  uint8_t rx_buffer[2];
  uint8_t ack_length;
  uint8_t *data_payload;
  uint8_t data_payload_length;
  SRadioParam wakeup_fragment_task_param;
  STask wakeup_fragment_task;
  SRadioParam ack_relay_rx_task_param;
  STask ack_relay_rx_task;
  SRadioParam data_send_task_param;
  STask data_send_task;
  volatile bool rxSuccess;

  uint32_t count_wus_tx_attempt;
  uint32_t count_wuf_tx_attempt;
  uint32_t count_data_tx_attempt;
  uint32_t count_ack_rx_attempt;
  uint32_t count_ack_rx_success;
  uint32_t CadTime;
  uint32_t NbMissedAck;
  /* Buffer parameters of the Wake up sequences*/
  uint8_t   Ftype;
  uint32_t  DevAddr;
  uint8_t   CntDnw;
  uint16_t  Fcount;
  uint8_t   Channel_Dr;
  uint32_t  MicPtp[WAKE_UP_SEQUENCE_LENGTH_MAX+5];
  uint8_t  PtPKey[16];
  uint32_t AddKey ;
  uint8_t  DevEUI [8];
};

#endif // __POINT_TO_POINT_TRANSMITTER_H__