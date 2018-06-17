# arduino-climate

Arduino based CAN bus climate control system.

Originally designed for use in the Australian Ford Falcon (AU), as I had performed a whole car electronics upgrade to the later FG MK1 model; the FG (and BA/BF which came beforehand) use what Ford call a "HIM" which controls all heater box related functions. The heater box from the later model would not fit into my car, therefore it was necessary to create my own HIM module. This program receives messages from the ICC from a BA-FG1 over HS-CAN (FG MK2 and FG-X HIM modules not tested, may work) and then changes solenoids etc as requested, and then sends back the correct packet from this emulated HIM module.

## Features (when complete)
- Send and receive CANBus packets and make decisions based upon these
- Stores items required on start up to Arduino EEPROM
- Control vacuum solenoids, fan speed controller and blend door motor (with potentiometer based positioning).
- Receive inputs from thermistors and send this information via CAN bus
- Receive voltage level from fan speed controller and send via CAN bus

## Required software and hardware
- Arduino Mega (a uno can work too - just in this situation needed the extra outputs)
- MCP2515 board, used to connect to vehicle CAN bus
- L298N board, used to control the blend door motor
- TIP12x, I used a TIP122 as it was easier to get a hold of locally to me. A TIP120 would have easily sufficed as this is only switching 12 volts for vacuum solenoids.
- MCP_CAN_LIB to interface with the MCP2515, available [here](https://github.com/coryjfowler/MCP_CAN_lib)