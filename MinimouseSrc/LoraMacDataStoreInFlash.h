/*

  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : Data store in flash.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#ifndef DATASTOREINFLASH_H
#define DATASTOREINFLASH_H
#include "Define.h"

struct sBackUpFlash
{    /*******************************************/
    /*      Update by Link ADR command         */
    /*******************************************/
    uint8_t      MacTxDataRate;
    uint8_t      MacTxPower;
    uint16_t     MacChMask; //@notereview remove
    uint8_t      MacNbTrans; 
    /********************************************/
    /*     Update by TxParamaSetupRequest       */
    /********************************************/
    uint32_t     MacRx2Frequency ; 
    uint8_t      MacRx2DataRate;
    uint8_t      MacRx1DataRateOffset;
    /********************************************/
    /*     Update by NewChannelReq command      */
    /********************************************/
    uint32_t     MacTxFrequency[16];//@note region dependant
    uint32_t     MacRx1Frequency[16];//@note region dependant
    uint8_t      MacMinDataRateChannel [16];
    uint8_t      MacMaxDataRateChannel [16];
    uint8_t      MacChannelIndexEnabled [16];
    uint16_t     MacChannelMask; //@notereview supprimer
    /********************************************/
    /*   Update by RXTimingSetupReq command     */
    /********************************************/
    int          MacRx1Delay;
    /********************************************/
    /*   Other Data To store                    */
    /********************************************/
    uint32_t     FcntUp;   
    uint32_t     FcntDwn;
    uint32_t     DevAddr;
    uint8_t      nwkSKey [16];
    uint8_t      appSKey [16];
    uint32_t     DevAddrClasCG0;
    uint8_t      nwkSKeyClasCG0[16];
    uint8_t      appSKeyClasCG0[16];
    uint32_t     DevAddrClasCG1;
    uint8_t      nwkSKeyClasCG1[16];
    uint8_t      appSKeyClasCG1[16];
    uint8_t      JoinedStatus;
    uint16_t     DevNonce;    
    uint8_t      NbOfReset;
    uint8_t      Reserved [7];
    uint32_t     CrcHigh;   
    uint32_t     CrcLow;    
} ;
extern struct sBackUpFlash BackUpFlash;
#endif
