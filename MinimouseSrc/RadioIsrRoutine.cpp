
/*
  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : LoraWan Radio ISR Routine.  
Note              : the isr routine isn't a global function , it is a method of RadioContainer template class. 
                  : It could be inside the PhyLayer.cpp file but for more readibility , it is choose to create a .cpp file for this method
License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#include "PhyLayer.h"
#include "Define.h"
#include "ApiMcu.h"
#include "utilities.h"
#include "RadioPlaner.h"
#include "DefineRadioPlaner.h"
#define FileId 6


template class RadioContainer<SX1276>;
template class RadioContainer<SX1272>;
template class RadioContainer<SX126x>;
template <class R> void RadioContainer <R>::IsrRadio( void ) {
    int status = OKLORAWAN;
    uint32_t tCurrentMillisec;
    LastItTimeFailsafe = mcu.RtcGetTimeSecond ( );
    Radio->GetStatusPlaner ( MyHookId, tCurrentMillisec, PlanerStatus );
    switch ( PlanerStatus ) {
        case PLANER_TX_DONE :
            break;

        case PLANER_RX_PACKET :
            InsertTrace ( __COUNTER__, FileId );
            DEBUG_PRINTF( "Receive a packet %d ms after tx done\n",tCurrentMillisec-TimestampRtcIsr);
            status = DumpRxPayloadAndMetadata ( );
            if ( status != OKLORAWAN ) { // Case receive a packet but it isn't a valid packet 
                InsertTrace ( __COUNTER__, FileId );
                tCurrentMillisec =  mcu.RtcGetTimeMs( );
                DEBUG_MSG( "Receive a packet But rejected and too late to restart\n");
                PlanerStatus = PLANER_RX_TIMEOUT;
            } 
            break;

        case PLANER_RX_TIMEOUT :
            if ( StateRadioProcess == RADIOSTATE_RXC ) {
                // Radio->RxLora( RxBw, RxSf, RxFrequency, 10000);@tbdone RadioPlaner
                DEBUG_MSG( "  **************************\n " );
                DEBUG_MSG( " *      RXC  Timeout       *\n " );
                DEBUG_MSG( " ***************************\n " );
                return;
            }
            break;
        default :
            DEBUG_PRINTF ("receive It RADIO error %d\n",PlanerStatus);
            tCurrentMillisec =  mcu.RtcGetTimeMs( );
            break;
    }
    switch ( StateRadioProcess ) { 

        case RADIOSTATE_TXON :
        
            InsertTrace ( __COUNTER__, FileId );
            TimestampRtcIsr = tCurrentMillisec; //@info Timestamp only on txdone it
            StateRadioProcess = RADIOSTATE_TXFINISHED;
            break; 
        case RADIOSTATE_TXFINISHED :
            InsertTrace ( __COUNTER__, FileId );
            StateRadioProcess = RADIOSTATE_RX1FINISHED;
            break; 
        case RADIOSTATE_RX1FINISHED :
            InsertTrace ( __COUNTER__, FileId );
            if ( Rx3Activated == RX3_ACTIVATED ) {
                StateRadioProcess = RADIOSTATE_RX2FINISHED;
            } else { 
                StateRadioProcess = RADIOSTATE_IDLE;
            }
            break;
        case RADIOSTATE_RX2FINISHED : 
            StateRadioProcess = RADIOSTATE_IDLE;
            break;
        case RADIOSTATE_RXC :
            StateRadioProcess = RADIOSTATE_IDLE;
            break;
        
        default :
            InsertTrace ( __COUNTER__, FileId ); 
            DEBUG_PRINTF ("Unknown state in Radio Process %d \n",StateRadioProcess);
            break;
    }
    
};

