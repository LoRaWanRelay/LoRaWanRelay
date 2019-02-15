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


