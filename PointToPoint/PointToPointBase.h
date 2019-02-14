
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


#ifndef __POINT_TO_POINT_BASE_H__
#define __POINT_TO_POINT_BASE_H__

#include <stdint.h>

#define SF_PTP     7
#define BW_PTP     BW125
#define POWER_PTP  14
#define MAX_PREAMBLE_PTP 1000 //(ms)
#define PPM_OFFSET       CRYSTAL_ERROR*10//ppm

#define WAKE_UP_FRAGMENT_LENGTH 11
#define WAKE_UP_FRAGMENT_DURATION_MS 30*(SF_PTP-6)
#define MAX_WUS_DURATION_MS       MAX_PREAMBLE_PTP + WAKE_UP_FRAGMENT_DURATION_MS 
#define MIN_PREAMBULE_DURATION_MS ( 8 << SF_PTP ) / ((BW125+1)*125)
#define CAD_BEAT_MS 500
#define CAD_BEAT_PER_CHANNEL_MS (2 * CAD_BEAT_MS)
#define NBR_FREQUENCIES 2
#define WAKE_UP_SEQUENCE_LENGTH_MAX 40  //((CAD_BEAT_MS * 4 )  / WAKE_UP_FRAGMENT_DURATION_MS)
#define MISSED_ACK_BEFORE_LONG_PREAMBLE 16
 typedef enum
  {
    STATE_INIT,
    STATE_SEND_WAKEUP_SEQUENCE_FRAGMENT,
    STATE_WAIT_RELAY_ACK,
    ACK_RECEIVED
  } State_t;



typedef struct
{
  uint32_t wus_tx_attempt;
  uint32_t wuf_tx_attempt;
  uint32_t data_tx_attempt;
  uint32_t ack_rx_attempt;
  uint32_t ack_rx_success;
} StatisticCounters_t;

typedef union
{
  struct
  {
    int16_t delay;
  };
  uint8_t buffer[2];
} Acknowledges_t;

typedef union
{
  struct
  {
    uint8_t blob[10];
    uint8_t index;
  };
  uint8_t buffer[WAKE_UP_FRAGMENT_LENGTH];
} WakeUpFragments_t;

typedef enum { 
    ERROR_PTP = -1,
    OK_PTP    = 0,
}eStatusPtP;

enum {
    WUS_WITH_DEVADDR,
    WUS_WITH_DEVEUI,
};
#endif // __POINT_TO_POINT_BASE_H__