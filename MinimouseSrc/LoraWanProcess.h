/*!
 * \file      LoraWanProcess.h
 *
 * \brief     Description : Minimalistic LoRaWAN stack implementation
 *
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
#ifndef LORAWANPROCESS_H
#define LORAWANPROCESS_H
#include "stdint.h"
#include "Define.h"
#include "MacLayer.h"
#include "Regions.h"
#include "RegionUS.h"
/*!
 * \class LoraWanObject 
 * \brief An absolutely minimalistic LoRaWAN Class A stack implementation .
 * \remark  In future implementation the constructor will contain :
 *            APPSKey, NWSKey, DevAdrr mandatory for APB devices
 *            DevEui, AppEUI, APPKey mandatory for OTA devices
 *            OTA or APB Flag.
 *         In future implementation A Radio objet will be also a parameter of this class.
 */
template < template <class R> class T, class RADIOTYPE>
class LoraWanObject { 
public:
    /*!
     * \brief LoraWanObject class constructor.
     * \param DevEui LoraWan Key for OTA Devices
     */    
    LoraWanObject(  sLoRaWanKeys LoRaWanKeys,RadioPLaner<RADIOTYPE>* RadioUser, uint32_t FlashAdress ); 


     /*!
     * \brief LoraWanObject class destructor.
     * \param None
     */    
    ~LoraWanObject();
    void Init     (void);

     /*!
     * \brief Sends an uplink 
     * \param [IN] uint8_t           fPort          Uplink Fport
     * \param [IN] const uint8_t*    dataInFport    User Payload
     * \param [IN] const uint8_t     sizeIn         User Payload Size
     * \param [IN] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

     * \param [OUT] eLoraWan_Process_States         Current state of the LoraWan stack :
     * \param                                            => return LWPSATE_SEND if all is ok
     * \param                                            => return Error in case of payload too long
     * \param                                            => return Error In case of the Lorawan stack previous state is not equal to iddle
     */  
    eLoraWan_Process_States    SendPayload             ( uint8_t fPort, const uint8_t* dataIn, const uint8_t sizeIn, uint8_t PacketType, uint32_t TargetTime );

    
     /*!
     * \brief  Receive Applicative Downlink 
     * \param [IN] uint8_t*          UserRxFport          Downlinklink Fport
     * \param [IN] uint8_t*          UserRxPayload        Applicative Downlink Payload
     * \param [IN] uint8_t*          UserRxPayloadSize    Applicative Downlink Payload Size
     * \param [IN] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

     * \param [OUT] eStatusLoRaWan   Return an error if No Packet available.
     */  
    eStatusLoRaWan             ReceivePayload          ( uint8_t* UserRxFport, uint8_t* UserRxPayload, uint8_t* UserRxPayloadSize );
    
    
     /*!
     * \brief to Send a Join request
     * \param [] None
     * \param [OUT] eLoraWan_Process_States         Current state of the LoraWan stack :
     *                                                 => return LWPSATE_SEND if all is ok
     *                                                 => return Error In case of the Lorawan stack previous state is not equal to iddle
     */  
    eLoraWan_Process_States    Join                    ( uint32_t TargetTime );
    
    
     /*!
     * \brief Returns the join state
     * \param [] None
     * \param [OUT] Returns the join state         NOT_JOINED: the device is joined to a network
     *                                             JOINED: the device is not connected
     *                                             Always returns JOINED for ABP devices 
     */  
    eJoinStatus                IsJoined                ( void );
    
    
    /*!
     * \brief SetDataRateStrategy of the devices
     * \remark Refered to the dedicated chapter in Wiki page for detailed explanation about 
     *         implemented data rate choice (distribution data rate).
     * \remark The current implementation support 4 different dataRate Strategy :
     *            STATIC_ADR_MODE                  for static Devices with ADR managed by the Network 
     *            MOBILE_LONGRANGE_DR_DISTRIBUTION for Mobile Devices with strong Long range requirement
     *            MOBILE_LOWPER_DR_DISTRIBUTION    for Mobile Devices with strong Low power requirement
     *            JOIN_DR_DISTRIBUTION             Dedicated for Join requests
     *
     * \param [IN]  eDataRateStrategy              DataRate Mode (describe above)
     * \param [OUT] None
     */  
    void                       SetDataRateStrategy     ( eDataRateStrategy adrModeSelect );
    
    
     /*!
     * \brief   Runs the MAC layer state machine. 
     *          Must be called periodically by the application. Not timing critical. Can be interrupted.
     * \remark  Not timing critical. Can be interrupted.
     *
     * \param [IN]  AvailableRxPacket *            Return if an applicative packet is available
     * \param [OUT] eLoraWan_Process_States        return the lorawan state machine state 
     */  
    eLoraWan_Process_States    LoraWanProcess          ( uint8_t* AvailableRxPacket );
    
        
     /*!
     * \brief   Return the state of the Radio 
     * \param [IN]  none
     * \param [OUT] return the state of the radio (Not yet finalized will be replace by an enum)
     */  
    uint8_t                    GetRadioState           ( void );
    
     /*!
     * \brief   Reload the LoraWAN context saved in the flash 
     * \param [IN]  none
     * \param [OUT] none
     */  
    void                       RestoreContext          ( void ); 

     /*!
     * \brief   Store provisionning informations in flash
     * \param [IN]  none
     * \param [OUT] none
     */  
    void                       SetProvisionning            ( sLoRaWanKeys LoRaWanKeys ); 

     /*!
     * \brief   Reload the factory Config in the LoraWAN Stack 
     * \param [IN]  none
     * \param [OUT] none
     */  
     void                       FactoryReset         ( void );  // load factory MAC context from constructor
     
     /*!
     * \brief   Get Device type : OTA or APB
     * \param [IN]  none
     * \param [OUT] enum Ota Device Type     OTA_DEVICE or APB_DEVICE,
     */  
       eDeviceTypeOTA_APB     GetIsOtaDevice (void);
       
     /*!
     * \brief Set Device type : OTA or APB
     * \param [IN]  none
     * \param [OUT] enum Ota Device Type     OTA_DEVICE or APB_DEVICE useful for certification,
     */  
       void     SetOtaDevice (eDeviceTypeOTA_APB  deviceType);
   
    /*!
     * \brief   Return the Max payload length allowed for the next transmit 
     * \remark  DataRate + FOPTS + region  dependant ( NOT YET IMPLEMENTED )
     * \remark  In any case if user set a too long payload, the send method will answer by an error status
     * \param [IN]  none
     * \param [OUT] Return max payload length for next Transmision
     */  
    uint32_t                   GetNextMaxPayloadLength ( void );
    
       
    /*!
     * \brief   Call this function to set the loraWan join variable in NOT_JOINED state
     * \param [IN]  none
     * \param [OUT] none
     */  
    void                       NewJoin                 ( void );
    
    
    /*! 
     * \brief   Return the DevAddr of the device
     * \param [IN]  none
     * \param [OUT] return DevAddr
     */ 
    uint32_t                   GetDevAddr              ( void );
    
    /*! 
     * \brief   Return the next transmission power 
     * \remark  NOT YET IMPLEMENTED
     * \param [IN]  none
     * \param [OUT] return the next transmission power 
     */ 
    uint8_t                    GetNextPower            ( void );
    
    
    /*! 
     * \brief   Return the returns the next data rate
     * \remark  NOT YET IMPLEMENTED
     * \param [IN]  none
     * \param [OUT] return the next transmission power 
     */ 
    uint8_t                    GetNextDataRate         ( void );
    
        /*! 
     * \brief   Return the returns the next Tx Frequency
     * \remark  NOT YET IMPLEMENTED
     * \param [IN]  none
     * \param [OUT] return the next transmission power 
     */ 
    
    
    uint32_t                   GetNextFrequency       ( void ); 
    
    
    /*! 
     * \brief   returns the current state of the MAC layer.
     * \remark  If the MAC is not in the idle state, the user cannot call any methods except the LoraWanProcess() method and the GetLorawanProcessState() method
     * \param [IN]  none
     * \param [OUT] return the next transmission power 
     */ 
    eLoraWan_Process_States    GetLorawanProcessState  ( void );
    
    uint8_t  GetNbOfReset (void);
    void                       ActivateClassC          ( void );
    void                       DeActivateClassC        ( void );
    void                       ActivateRX3             ( void )  { packet.Phy.Rx3Activated = RX3_ACTIVATED; };
    void                       DeActivateRX3           ( void )  { packet.Phy.Rx3Activated = RX3_NOT_ACTIVATED; };
    eDeviceTypeRx3             IsActivatedRX3          ( void )  { return (packet.Phy.Rx3Activated) ; };
    void                       SetDefaultChannel       (uint32_t *Freq, uint8_t NbChannel ) { 
        for (int i = 0; i<NbChannel ; i++) {
            packet.MacTxFrequency[i]    = Freq [i];
            packet.MacRx1Frequency[i]    = Freq [i];
        }
        packet.RegionGiveNextDataRate ();
    }
    T<RADIOTYPE>               packet;    
private :


    eLoraWan_Process_States    StateLoraWanProcess; // for debug not private
    uint32_t                   FlashAdress ;
    eDeviceTypeClassC          ClassCActivated;
    uint8_t                    ValidRxPacket; 
    uint32_t                   RtcTargetTimer;
    void CopyUserPayload          ( const uint8_t* dataIn, const uint8_t sizeIn );
    uint32_t GetFailSafeTimestamp ( void ){return packet.Phy.LastItTimeFailsafe;};
    ePlanerStatus  GetPlanerStatus      ( void );


};

#endif
