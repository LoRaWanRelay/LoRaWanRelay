/*!
 * \file      appli.c
 *
 * \brief     Description : Example of usal application 
 *            This apllication uses an i2C sensor to get both Temperature and Hygro
 *            This application uses an analog input to get Vbat 
 *            This application transfers a float value for demo purpose
 *            This application send an APPLI_FW_VERSION as first byte of the payload
 *            This application used a part of the UID of the STM32 to set the DEVEUI 
 *            (It could be dangerous because deveui isn't necessary unique due to the fact that it is compute from a part of the uid and not the full uid)
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
 * \endcode

Maintainer        : Fabien Holin (SEMTECH)
*/


#include "appli.h"
#include "Define.h"
#include "LoraMacDataStoreInFlash.h"


payload ppayload;
int8_t TempMeas (void)
{

    return(20);
}

uint8_t HydroMeas (void)
{
    return (80);
}

uint8_t VbatMeas (void)
{

    return(95);
}
void PrepareFrame (uint8_t *Buffer) {
    ppayload.Temp       =  TempMeas  ();
    ppayload.Hygro      =  HydroMeas ();
    ppayload.Vbat       =  VbatMeas  ();
    ppayload.Reserved   =  81.2;
    uint8_t* bytes = (uint8_t*)&ppayload;
    uint8_t temp;
    Buffer[0]=APPLI_FW_VERSION;
    for ( int i = 1; i < 8; i++) {
        Buffer[i]= bytes[i-1];
    }
    temp = Buffer[1];
    Buffer[1] = Buffer[4];
    Buffer[4] = temp;
    temp = Buffer[2];
    Buffer[2] = Buffer[3];
    Buffer[3] = temp;
    Buffer[8] = BackUpFlash.NbOfReset;
}
