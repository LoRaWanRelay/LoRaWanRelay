# LoRaWanRelay
Implementation of a LoRaWan battery powered repeater .
# Relay software stack
The relay software project is available on this page :                                                  
By default the Makefile is configured in Relay Mode for running over a STMicroelectronics Discovery Kit CMWX1ZZABZ.
![](http://lorae.ddns.net/Images/R1345557-01.jpg)
To compile the source code , you have to install the free gcc-arm-none-eabi package (working both on linux and windows : [https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)).

Compile source code : ` make clean all `
The Binary file is available in the repository /build/MiniMouse.bin 
# Sensor software stack
The Sensor Code is also available at the same place.
To switch from relay code to sensor code you have to select a compilation option in the Makefile (set variable `RELAY to 0`).
Compile source code : ` make clean all `
The Binary file is available in the repository /build/MiniMouse.bin 
# MakeFile  Options

 * `BOARD_MURATA      = 1`  To Select the Murata Board as Hw Target Platform
 * `RELAY             = 1`  To Activate the Relay FW (set 0 for Sensor Fw )
* `BOARD_WITH_SENSOR = 0` To Select BezLoc Board
* `BOARD_L4          = 0` To Select Nucleo Board L4 (not fully tested up to now)
* `RADIO_SX1276      = 1` To select Radio Sx1276
* `RADIO_SX1272      = 0` To Select Radio Sx1272 ( Not Yet Fully Implemented Cad Not Tested)
* `RADIO_SX126x      = 0` To Select Radio Sx126x ( Not Yet Fully Implemented Cad Not Tested)
* `DEVICE_UNDER_TEST = 1` Set To One for Relay Demo
* `RADIO_WITH_TCX0   = 1` Set To One if radio Board embeds a TCXO
