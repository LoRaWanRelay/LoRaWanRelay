/*
                                                   
Description       : User Define for Sensor Layer.  
License           : Revised BSD License, see LICENSE.TXT file include in the project
Maintainer        : Fabien Holin ()
*/

#ifndef SENSOR__H
#define SENSOR__H
#include "ApiMcu.h"
#include "Define.h"
//sensor I2C address
#define SHT_I2C_ADDR        0x80
//Commands...
//Trigger Temp with hold master
#define SHT_TRIG_TEMP_HOLD  0xE3
//Trigger RH with hold master
#define SHT_TRIG_RH_HOLD    0xE5
//Trigger Temp with no hold master
#define SHT_TRIG_TEMP       0xF3
//Trigger RH with no hold master
#define SHT_TRIG_RH         0xF5
//Write to user register
#define SHT_WRITE_REG       0xE6
//Read from user register
#define SHT_READ_REG        0xE7
//Soft reset the sensor
#define SHT_SOFT_RESET      0xFE
//Data precision settings
//RH 12 T 14 - default
#define SHT_PREC_1214       0x00
//RH 8  T 10 
#define SHT_PREC_0812       0x01
//RH 10 T 13
#define SHT_PREC_1013       0x80
//RH 11 T 11
#define SHT_PREC_1111       0x81
//Battery status
#define SHT_BATTERY_STAT    0x40
//Enable on chip heater
#define SHT_HEATER          0x04
//Disable OTP reload
#define SHT_DISABLE_OTP     0x02
//Fail conditions on the I2C bus
#define SHT_FAIL            1
#define SHT_SUCCESS         0
//Author fail conditions
//1, 2, 3 can be used because these are status bits
//in the received measurement value
#define SHT_GOOD            0xFFFC
#define SHT_TRIG_FAIL       1
#define SHT_READ_FAIL       2

class SHT21
{
public:

    SHT21 ( PinName PowerOffOnIn ) {
        PowerOffOn = PowerOffOnIn;
       // PowerSht ( );
    };
    int8_t readTemp ( void ) {
        //PowerSht ( );
       //mcu.SetValueDigitalOutPin ( PowerOffOn, 1 );
        int res = 1 ;
        while (res !=0) {
            StartI2c ();
            res= triggerTemp() ;
            mcu.mwait_ms (190);  
             requestTemp() ;  
        }
       // mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );
        float realtemp;
        realtemp = -46.85 + 175.72 * ( ((float) temperature) / 65536 );
        DEBUG_PRINTF ( "temp =%d\n",(int32_t)temperature);
        return ((int8_t)realtemp);
        //mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );
    } ;
    uint8_t readHumidity ( void ) {
      //  PowerSht ( );
        int res = 1 ;
        while (res !=0) {
            StartI2c ();
            res = triggerRH() ;
            mcu.mwait_ms (10);  
            res += requestRH() ;  
        }
        mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );
        float realhum;
        realhum = -6 + 125 * ( ((float) humidity) / 65536 );
        return ((uint8_t) realhum ) ;
    } ;
//private:
    int triggerTemp ( ) { 
       // uint8_t RegValue;
       // mcu.I2cWriteMem (SHT_I2C_ADDR, SHT_SOFT_RESET, I2C_MEMADD_SIZE_8BIT , &RegValue, 0 , 1000 ); // One Shot Mesaue 0x11
       // mcu.mwait_ms (30);
       // RegValue = 0xC1;
       // mcu.I2cWriteMem (SHT_I2C_ADDR, SHT_WRITE_REG, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
      //  RegValue = 0;
      //  mcu.I2cWriteMem (SHT_I2C_ADDR, SHT_TRIG_TEMP, I2C_MEMADD_SIZE_8BIT ,  &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
      //  return (RegValue);
     //   }; 
        return wr(SHT_TRIG_TEMP_HOLD);};  
    int requestTemp ( ) {
        int res;
        uint16_t rx_bytes = 3;
        uint8_t rx[rx_bytes];
        res = mcu.I2cReceive (SHT_I2C_ADDR, rx, rx_bytes );
        //res = mcu.I2cReadMem (SHT_I2C_ADDR, SHT_TRIG_TEMP_HOLD, I2C_MEMADD_SIZE_8BIT , rx, 3 , 1000 );
        unsigned short msb = (rx[0] << 8);
        unsigned short lsb = (( rx[1] & 0xFC) << 0);
        temperature = (msb + lsb);
       // res= CheckCrc(rx,2,rx[2]);
        return res;
    };
    unsigned short temperature;
    int triggerRH ( ){ return wr(SHT_TRIG_RH_HOLD);};
    int requestRH ( ){
        int res;
        uint8_t rx[3];
        res = mcu.I2cReceive(SHT_I2C_ADDR,rx,3);
        humidity = (rx[0]<<8) + rx[1];
        res+= CheckCrc(rx,2,rx[2]);
        return res;
    };
    unsigned short humidity;
    int wr(uint8_t cmd) {
        int res;
        uint8_t command[1];
        command[0] = cmd;
        res = mcu.I2cTransmit (SHT_I2C_ADDR, command, 1 );   
        DEBUG_PRINTF ("status wr cmd = %d\n",res);
        return res;
    };
    void PowerSht ( void ) {
        mcu.InitGpioOut ( PowerOffOn ) ;
        mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );
        mcu.mwait_ms (10);
        mcu.SetValueDigitalOutPin ( PowerOffOn, 1 );
        mcu.mwait_ms (10);
    }
    void StartI2c ( void ) {
        mcu.I2cDeInit();
        mcu.mwait_ms (100);
        mcu.I2cInit ();
        mcu.mwait_ms (200);
    }

    PinName PowerOffOn;
    int CheckCrc(uint8_t data[], int nbrOfBytes, uint8_t checksum) {
        int crc = 0;
        int byteCtr;
        const int POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001
        //calculates 8-Bit checksum with given polynomial
        for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) {
            crc ^= (data[byteCtr]);
            for (int bit = 8; bit > 0; --bit) {
                if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
                else crc = (crc << 1);
            }
        }
        if (crc != checksum) return 1;
        else return 0;
    };
};
//////////////////////////////////////////////////////////////////////////
/*                          Pressure Sensor                             */
//////////////////////////////////////////////////////////////////////////

#define LPS22HB_ADDR   0xBBU

/** Device Identification (Who am I) **/
#define LPS22HB_ID             0xB1U
#define LPS22HB_THS_P_L        0x0CU
#define LPS22HB_THS_P_H        0x0DU
#define LPS22HB_WHO_AM_I       0x0FU
#define LPS22HB_CTRL_REG1      0x10U
#define LPS22HB_CTRL_REG2      0x11U
#define LPS22HB_PRESS_OUT_XL   0x28U
#define LPS22HB_PRESS_OUT_L    0x29U
#define LPS22HB_PRESS_OUT_H    0x2AU
#define LPS22HB_TEMP_OUT_L     0x2BU
#define LPS22HB_TEMP_OUT_H     0x2CU
class LPS22HB
{
    public:

    LPS22HB ( PinName PowerOffOnIn ) {
        PowerOffOn = PowerOffOnIn;
    };
    uint8_t GetWhoAmI ( void ) {
        PowerLps22Hb ();
        int res = 1 ;
        uint8_t WhoAmI = 0; 
        while (res != 0) {
            StartI2c ();
            res = mcu.I2cReadMem (LPS22HB_ADDR, LPS22HB_WHO_AM_I, I2C_MEMADD_SIZE_8BIT , &WhoAmI, 1 , 1000 );
        }
        PowerOffLps22Hb ();
        return ( WhoAmI );
    };
    uint32_t GetPressure (void) {
        PowerLps22Hb ();
        int res = 1 ;
        uint8_t Pressure[3];
        while (res != 0) {
            StartI2c ();
            uint8_t RegValue = 0x11;
            res = mcu.I2cWriteMem (LPS22HB_ADDR, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
            mcu.mwait_ms (100);
            res += mcu.I2cReadMem (LPS22HB_ADDR, LPS22HB_PRESS_OUT_XL, I2C_MEMADD_SIZE_8BIT , Pressure, 3 , 1000 );
        }
        uint32_t Result = ( Pressure[2] << 16 )  + ( Pressure[1] << 8 ) +  Pressure[0];
        PowerOffLps22Hb ();
        return ( Result >> 12); 
    };
     uint32_t GetTemperature (void) {
        PowerLps22Hb ();
        int res = 1 ;
        uint8_t Temp[3];
        while (res != 0) {
            StartI2c ();
            uint8_t RegValue = 0x11;
            res = mcu.I2cWriteMem (LPS22HB_ADDR, LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
            mcu.mwait_ms (100);
            res += mcu.I2cReadMem (LPS22HB_ADDR, LPS22HB_TEMP_OUT_L, I2C_MEMADD_SIZE_8BIT , Temp, 2 , 1000 );
        }
        uint32_t Result = ( Temp[1] << 8 ) +  Temp[0];
        PowerOffLps22Hb ();
        return ( Result );
    };
private:
    int wr(uint8_t cmd) {
        int res;
        uint8_t command[1];
        command[0] = cmd;
        res = mcu.I2cTransmit (LPS22HB_ADDR, command, 1 );   
        return res;
    };
    void PowerLps22Hb ( void ) {
        mcu.InitGpioOut ( PowerOffOn ) ;
        mcu.InitGpioOut ( PB_14 ) ;
        mcu.SetValueDigitalOutPin ( PB_14, 1 );
        mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );
        mcu.mwait_ms (10);
        mcu.SetValueDigitalOutPin ( PowerOffOn, 1 );
        mcu.mwait_ms (10);
    }
    void PowerOffLps22Hb ( void ) {

        mcu.SetValueDigitalOutPin ( PB_14, 0 );
        mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );

    }
    void StartI2c ( void ) {
        mcu.I2cDeInit();
        mcu.mwait_ms (5);
        mcu.I2cInit ();
        mcu.mwait_ms (5);
    }
    PinName PowerOffOn;
};



//////////////////////////////////////////////////////////////////////////
/*                          Pressure Sensor                             */
//////////////////////////////////////////////////////////////////////////

#define LSM303H_ACC_ADDR       0x3BU  
#define LSM303H_MAGNETO_ADDR   0x3DU  
/** Device Identification (Who am I) **/

#define LSM303H_WHO_AM_I       0x0FU
#define LSM303H_CTRL1A         0x20U
#define LSM303H_OUT_TEMP_A     0x26U
#define LSM303H_STATUS_A       0x27U
#define LSM303H_OUT_X_L_A      0x28U
#define LSM303H_OUT_X_H_A      0x29U
#define LSM303H_OUT_Y_L_A      0x2AU
#define LSM303H_OUT_Y_H_A      0x2BU
#define LSM303H_OUT_Z_L_A      0x2CU
#define LSM303H_OUT_Z_H_A      0x2DU

#define LSM303H_OUT_X_L_M      0x68U
#define LSM303H_OUT_X_H_M      0x69U
#define LSM303H_OUT_Y_L_M      0x6AU
#define LSM303H_OUT_Y_H_M      0x6BU
#define LSM303H_OUT_Z_L_M      0x6CU
#define LSM303H_OUT_Z_H_M      0x6DU

#define LSM303H_WAKE_UP_THS_A  0x33U
#define LSM303H_WAKE_UP_DUR_A  0x34U
#define LSM303H_WAKE_UP_SRC_A  0x37U
#define LSM303H_CTRL4_A        0x23U
#define LSM303H_CTRL3_A        0x23U
#define LSM303H_MAGNETO_CTRLReg1A         0x60U

class LSM303H_ACC
{
    public:

    LSM303H_ACC ( PinName PowerOffOnIn ) {
        PowerOffOn = PowerOffOnIn;
      
    };
    uint8_t GetWhoAmI ( void ) {
        
        int res = 1 ;
        uint8_t WhoAmI = 0;
        while (res != 0) {
            res = mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_WHO_AM_I, I2C_MEMADD_SIZE_8BIT , &WhoAmI, 1 , 1000 );
        }
        return ( WhoAmI );
    };
    uint8_t GetWhoAmIMagneto ( void ) {
        int res = 1 ;
        uint8_t WhoAmI = 0;
        while (res != 0) {
            res = mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, 0x4F, I2C_MEMADD_SIZE_8BIT , &WhoAmI, 1 , 1000 );
        }
        return ( WhoAmI );
    };
    void InitConfig ( void ) {
          PowerLSM303H_ACC ();
        int res = 1 ;
        while (res != 0) {
            uint8_t RegValue = 0xA0;  // 25 HZ Low Power mode
            res = mcu.I2cWriteMem (LSM303H_ACC_ADDR, LSM303H_CTRL1A, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
            RegValue = 0x0;  //// No duration For Wake up
            res += mcu.I2cWriteMem (LSM303H_ACC_ADDR, LSM303H_WAKE_UP_DUR_A, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
            RegValue = 0x3;  ////MAx For Wake up Threshold
            res += mcu.I2cWriteMem (LSM303H_ACC_ADDR, LSM303H_WAKE_UP_THS_A, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
            RegValue = 0x04;  //// Latch the interrupt
            res += mcu.I2cWriteMem (LSM303H_ACC_ADDR, LSM303H_CTRL3_A, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
            RegValue = 0x20;  //// Wake up on interrput pin
            res += mcu.I2cWriteMem (LSM303H_ACC_ADDR, LSM303H_CTRL4_A, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
        }
    };
    uint8_t ReadStatusAccelero ( void ) {
        int res = 1 ;
        uint8_t Status = 0;
        uint8_t XaH = 0;
        uint8_t XaL = 0;
        uint8_t YaH = 0;
        uint8_t YaL = 0;
        uint8_t ZaH = 0;
        uint8_t ZaL = 0;
        uint8_t temp = 0;
     
        while (res != 0) {
            res = mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_CTRL1A, I2C_MEMADD_SIZE_8BIT , &Status, 1 , 1000 );
            DEBUG_PRINTF ("ctrlA reg = %x\n",Status);
            Status = 0;
            while ((Status&0x1) == 0) {
                res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_STATUS_A, I2C_MEMADD_SIZE_8BIT , &Status, 1 , 1000 );
            }
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_X_L_A, I2C_MEMADD_SIZE_8BIT , &XaL, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_X_H_A, I2C_MEMADD_SIZE_8BIT , &XaH, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_Y_L_A, I2C_MEMADD_SIZE_8BIT , &YaL, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_Y_H_A, I2C_MEMADD_SIZE_8BIT , &YaH, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_Z_L_A, I2C_MEMADD_SIZE_8BIT , &ZaL, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_Z_H_A, I2C_MEMADD_SIZE_8BIT , &ZaH, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_OUT_TEMP_A, I2C_MEMADD_SIZE_8BIT ,&temp, 1 , 1000 );
            Temperature = (uint8_t)(25 + (int8_t)temp) ;   
        }
        DEBUG_PRINTF ("Acceleration X  = %d\n",(int16_t)(( XaH<<8 ) + XaL));
        DEBUG_PRINTF ("Acceleration Y = %d\n",(int16_t)(( YaH<<8 ) + YaL));
        DEBUG_PRINTF ("Acceleration Z  = %d\n",(int16_t)(( ZaH<<8 ) + ZaL));
        return ( Status );
    };
  void InitConfigMagneto ( void ) {
       
        int res = 1 ;
        while (res != 0) {
            uint8_t RegValue = 0x81;  // 10 HZ Low Power mode
            res = mcu.I2cWriteMem (LSM303H_MAGNETO_ADDR, LSM303H_MAGNETO_CTRLReg1A, I2C_MEMADD_SIZE_8BIT , &RegValue, 1 , 1000 ); // One Shot Mesaue 0x11
        }
    };

  uint8_t ReadStatusMagneto ( void ) {
        int res = 1 ;
        uint8_t Status = 0;
        uint8_t XaH = 0;
        uint8_t XaL = 0;
        uint8_t YaH = 0;
        uint8_t YaL = 0;
        uint8_t ZaH = 0;
        uint8_t ZaL = 0;
       //   while (res != 0) {
            res = mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_MAGNETO_CTRLReg1A, I2C_MEMADD_SIZE_8BIT , &Status, 1 , 1000 );
            DEBUG_PRINTF ("LSM303H_MAGNETO_CTRLReg1A reg = %x\n",Status);
            Status = 0;
           // mcu.mwait_ms (200);
            while ((Status & 0x8) == 0) {
                res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, 0x67, I2C_MEMADD_SIZE_8BIT , &Status, 1 , 1000 );
            }
            res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_OUT_X_L_M, I2C_MEMADD_SIZE_8BIT , &XaL, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_OUT_X_H_M, I2C_MEMADD_SIZE_8BIT , &XaH, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_OUT_Y_L_M, I2C_MEMADD_SIZE_8BIT , &YaL, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_OUT_Y_H_M, I2C_MEMADD_SIZE_8BIT , &YaH, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_OUT_Z_L_M, I2C_MEMADD_SIZE_8BIT , &ZaL, 1 , 1000 );
            res += mcu.I2cReadMem (LSM303H_MAGNETO_ADDR, LSM303H_OUT_Z_H_M, I2C_MEMADD_SIZE_8BIT , &ZaH, 1 , 1000 );
      //  }
        DEBUG_PRINTF ("MAGNETO X  = %d\n",(int16_t)(( XaH<<8 ) + XaL));
        DEBUG_PRINTF ("MAGNETO Y = %d\n",(int16_t)(( YaH<<8 ) + YaL));
        DEBUG_PRINTF ("MAGNETO Z  = %d\n",(int16_t)(( ZaH<<8 ) + ZaL));
        return ( Status );
    };
uint8_t AcceleroWup (void) {
    DEBUG_PRINTF ("irq accelero = %d\n",mcu.GetValueDigitalInPin(PA_8));
    return (mcu.GetValueDigitalInPin(PA_8));
}  
void ClearIrqAccelero (void) {
    uint8_t Status = 0;
    mcu.I2cReadMem (LSM303H_ACC_ADDR, LSM303H_WAKE_UP_SRC_A, I2C_MEMADD_SIZE_8BIT , &Status, 1 , 1000 );
}
uint8_t Running;
uint8_t Temperature;
  void PowerLSM303H_ACC ( void ) {
        mcu.InitGpioOut ( PowerOffOn ) ;
        mcu.InitGpioOut ( PB_14 ) ;
        mcu.Init_Irq (PA_8);

        mcu.SetValueDigitalOutPin ( PB_14, 1 );
        mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );
        mcu.mwait_ms (10);
        mcu.SetValueDigitalOutPin ( PowerOffOn, 1 );
        mcu.mwait_ms (10);
        mcu.I2cDeInit();
        mcu.mwait_ms (5);
        mcu.I2cInit ();
        mcu.mwait_ms (5);
    }
    void PowerOffLSM303H_ACC ( void ) {

        mcu.SetValueDigitalOutPin ( PB_14, 0 );
        mcu.SetValueDigitalOutPin ( PowerOffOn, 0 );

    }
private:
    int wr(uint8_t cmd) {
        int res;
        uint8_t command[1];
        command[0] = cmd;
        res = mcu.I2cTransmit (LSM303H_ACC_ADDR, command, 1 );   
        return res;
    };
  
    PinName PowerOffOn;
};
extern  LSM303H_ACC Accelero;   

#endif