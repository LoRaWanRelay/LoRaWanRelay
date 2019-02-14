 #include "stdint.h"
 #include "Define.h"
 #include "DefineRadioPlaner.h"
#ifndef APPLI__H
#define APPLI__H



#define APPLI_FW_VERSION    14


int8_t  TempMeas  ( void );
uint8_t HydroMeas ( void );
uint8_t VbatMeas  ( void );
float   LoadMeas  ( void );
typedef struct {
    float    Reserved;
    uint8_t  Vbat;
    int8_t   Temp;
    uint8_t  Hygro;
} payload;
void PrepareFrame (uint8_t *Buffer);

#endif

