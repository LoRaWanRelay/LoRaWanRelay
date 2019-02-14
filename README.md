# LoRaWanRelay
LoRaWanRelay is an open source LoRaWan battery powered repeater that integrate both the source code of the relay itself but also the source code of the end device.

With the proposal implementation the relay is abble to support up to 16 nodes during 5 years with a battery of 3AH.

The sensor embeds a full loRaWan Stack class A and a specific wake up sequences to communicate with the Relay.

This sensor can be catch both by a LoRaWAN Gateway and also by the Relay.

LoRaWanRelay is based on a mechanism of synchronization between the Relay and the End Device to reduce the power consumption on both Side.

Frequency diversity is also implemented to be robust in case of interference.

Security features with Mic Identification are implemented between End Device and the Relay.

# Relay software stack
The main file of the  relay fw project is available here  :
[https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp](https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp)                                                  
By default the Makefile is configured in Relay Mode for running over a STMicroelectronics Discovery Kit CMWX1ZZABZ.
![](http://lorae.ddns.net/Images/R1345557-01.jpg)

To compile the source code , you have to install the free gcc-arm-none-eabi package (working both on linux and windows : [https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)).

To Compile source code : ` make clean all `
The Binary file is available in the repository /build/MiniMouse.bin 
# Sensor software stack
The main file of the Sensor code is also available at  :
[https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp](https://github.com/LoRaWanRelay/LoRaWanRelay/blob/master/UserCode/MainRelay.cpp)  
To switch from relay code to sensor code you have to select a compilation option in the Makefile (set variable `RELAY to 0`).

To Compile source code : ` make clean all `

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



