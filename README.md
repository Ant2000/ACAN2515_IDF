## MCP2515 CAN Controller Library for ESP-IDF

> This is a fork of the library ACAN2515 created by pierremolinaro with changes made to be compatible with ESP-IDF. It has been tested on the ESP32 microcontroller.

### ACAN2515 library description
ACAN2515 is a driver for the MCP2515 CAN Controller. It is to be used as a component in ESP-IDF

You can choose any frequency for your MCP2515, the actual frequency is a parameter of the library.

The driver supports many bit rates: for a 16 MHz quartz, the CAN bit timing calculator finds settings for standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, 1 Mbit/s, but also for an exotic bit rate as 727 kbit/s. If the desired bit rate cannot be achieved, the `begin` method does not configure the hardware and returns an error code.

> Driver API is fully described by the PDF file in the `extras` directory. Note - parts of the document only apply to the Arduino version of the library.

### Demo Code

> The demo of this library is in the `examples/MCP_2515_EVAL` directory.