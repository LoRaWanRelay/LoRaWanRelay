# LoRaWanRelay
LoRaWanRelay is an open source LoRaWAN battery powered repeater that integrate both the source code of the relay itself but also the source code of the end device.

With the proposal implementation the relay is able to support up to 16 nodes during 5 years with a battery of 3AH.

The sensor embeds a full loRaWAN Stack class A and a specific wake up sequences to communicate with the relay.

This sensor can be catch both by a LoRaWAN Gateway and also by the relay.

LoRaWanRelay is based on a mechanism of synchronization between the Relay and the End Device to reduce the power consumption on both side.

Frequency diversity is also implemented to be robust in case of interference.

Security features with Mic Identification are implemented between End Device and the relay.

# Relay software stack
The main file of the  relay fw project is available here  :
[https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp](https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp)                                                  
By default the Makefile is configured in Relay Mode for running over a STMicroelectronics Discovery Kit CMWX1ZZABZ.
![](http://lorae.ddns.net/Images/murataborad.jpg)

To compile the source code , you have to install the free gcc-arm-none-eabi package (working both on linux and windows : [https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)).
 
### Build source code :
` make clean all `
The Binary file is available in the repository /build/MiniMouse.bin 
# Sensor software stack
The main file of the Sensor code is also available at  :
[https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp](https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp) .
The Code has been devlopp for a HW platform who demonstrate some sensor named BezLoc but you can also run this code on the Discovery kit just by changing an option in the Makefile as described below.
![BeLoc Board](http://lorae.ddns.net/Images/bezlocV.jpg)
To switch from relay code to sensor code you have to select a compilation option in the Makefile :
set variable `RELAY to 0`.

### Build source code :
` make clean all `

The Binary file is available in the repository /build/MiniMouse.bin 

# MakeFile  Options

 * `BOARD_MURATA      = 1`To Select the Murata Board as Hw Target Platform
 * `RELAY             = 1`To Activate the Relay FW (set 0 for Sensor Fw )
* `BOARD_WITH_SENSOR = 0` To Select BezLoc Board
* `BOARD_L4          = 0` To Select Nucleo Board L4 (not fully tested up to now)
* `RADIO_SX1276      = 1` To select Radio Sx1276
* `RADIO_SX1272      = 0` To Select Radio Sx1272 ( Not Yet Fully Implemented Cad Not Tested)
* `RADIO_SX126x      = 0` To Select Radio Sx126x ( Not Yet Fully Implemented Cad Not Tested)
* `DEVICE_UNDER_TEST = 1` Set To One for Relay Demo
* `RADIO_WITH_TCX0   = 1` Set To One if radio Board embeds a TCXO

# NodeRed relay manager
The project will also implement a very simple Relay manager running on a Node red Platform.
![Relay Manager](http://lorae.ddns.net/Images/relaymanager.png)
For this purpose, the Relay manager behaves as a LoRa gateway (we call it the relay virtual gateway).
From the NS stand point, this new uplink comes directly from the sensor and has been received through a single LoRa gateway, which happens to be our virtual gateway. The data structure coming from that gateway has all the fields normally expected from one of the regular LoRA gateways of the network. The Relay is therefore transparent for the NS and does not require any modification of the Ns.
The Node Red Manages Only one Relay, it is just an example of implementation but it is not scalable.

The NodeRed source code is available on the directory /NodeRedSrc

Just Copy It and use Import Clipboard in NodeRed  to create your own node red relay manager.
![Relay Manager Node Red Flow](http://lorae.ddns.net/Images/noderedcrop.png)

### Node Red Dependencies : 

-	Install node-red-dashboard to enable the user interface integrated in the node red source code 
[https://www.npmjs.com/package/node-red-dashboard](https://www.npmjs.com/package/node-red-dashboard)
-	Install the TTN node red node :
[https://www.thethingsnetwork.org/docs/applications/nodered/](https://www.thethingsnetwork.org/docs/applications/nodered/)

### Configure the following boxes : 
-	In the Box VirtualGW set your virtual gateway Id, you have to provision a virtual gateway with the same Id in your  TTN Console : var gw_id = [0xaa,0x55,0x5a,0x00,0x11,0x11,0x11,0x11];
-	In the Box ParseDev_id set the Device Id  of your relay as declared in your TTN console (in the example Device Id = relay).
The Node Red source code embeds a dashboard , to access it open in your navigator the url :
 Your_Node-red_Url:1880/ui/ 
 
You should have this kind of dashboard 
![Relay Manager UI ](http://lorae.ddns.net/Images/nodered2.png)
This User Interface provides a Forwared List / Not Yet Forwarded list of devices seen by the Relay.

The user can choose to enable/disable the  forwarding for each devices already provisioned (devAddr List) or still in join mode (DevEui List) .

The User Interface provides also the Tx/RX/CPU power consumption updated when a  status frame  is received.

# Configuration of the Network Server Backend
In this project use the TTN bakcend as network server
### Relay Provisioning  :
Nothing special to do , by default in the source code the relay is in OTA mode, the LoRaWAN keys have to be modify  in the source code in the MainRelay File : 
```cpp int mainRelay( void ) {

uint8_t LoRaMacNwkSKeyInit[]      = { 0x22, 0x33, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t LoRaMacAppSKeyInit[]      = { 0x11, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
uint8_t LoRaMacAppKeyInit[]       = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xBB};
uint8_t AppEuiInit[]              = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xff, 0x50 };
uint8_t DevEuiInit2[]             = { 0x38, 0x35, 0x31, 0x31, 0x18, 0x47, 0x37, 0x51 };
int i;
uint8_t UserRxFport ; 

#ifdef RELAY
    uint32_t LoRaDevAddrInit   = 0x260114FC;
    uint8_t DevEuiInit[]       = { 0x38, 0x35, 0x31, 0x31, 0x18, 0x47, 0x37, 0x57 };    
    sLoRaWanKeys  LoraWanKeys  = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, LoRaDevAddrInit,OTA_DEVICE };
#else
    uint8_t UserFport = 3 ;
    uint32_t LoRaDevAddrInit   = 0x26011695;
    sLoRaWanKeys  LoraWanKeys  = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit2, LoRaDevAddrInit,APB_DEVICE };
#endif 
```

### Sensor Provisioning : 
Provision the sensor as usual on the TTN Backend , can be either in OTA mode or in ABP mode. 
For demo purpose, disable Frame counter checks   (as on the figure below), like this you will see in your console the packets receive twice : first time by the LoRaWAN GATEWAY as Usual and a second time by the "Virtual Gateway" 
![Sensor Provisionning](http://lorae.ddns.net/Images/ttnsensor.png)

### Gateway Provisionning : 
Configure a new gateway on ttn dashboard with the same DevEUI as one inside your NodeRed dashboard , enable the option :

` I'm using the legacy packet forwarder
Select this if you are using the legacy Semtech packet forwarder.`


# Running the system 
How can we verify that everything is running?
### On TTN Console
* You can verify that the sensor in receive both by the Normal Gateway but also by the virtual gateway.
To proceed in your TTN console, the uplink receive twice will appear with the mention retry :
![Uplink received by the virtual Gw](http://lorae.ddns.net/Images/packet.jpg)
* Downlink check : the sensor is configured to retransmit all the applicative downlink in it next uplink. So each time the sensor will received a downlink , the next uplink inside the ttn console should be a copy of the downlink.
### On Node Red UI 
* When a new node is starting for the first time, it will appear in the node red UI console as the mention Enable Forwarding
That men that the relay have seen the sensor but it not relay the packets of this sensor.
If you enable the forwarding , the sensor become white listed and the packet start to be relayed.
### Observing the power consumption
The power consumtion give you lots of informations :
The following picture describe the different state of a relay devices :
![Power State of the relay Gw](http://lorae.ddns.net/Images/power.jpg)



