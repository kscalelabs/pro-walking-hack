# CH340 CAN module

This is the C++ code with a demo for interfacing with the Robstride CAN module.

Notes:

- MacOS doesn't support a baudrate of 921600 required by the Robstride CAN module. However the code will still compile on MacOS
- Jetson default image has issues out of the box. Need to install [this driver](https://github.com/WCHSoftGroup/ch341ser_linux) and make sure to set the TTY device baudrate to 921600

