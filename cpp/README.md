# CH340 CAN module

This is the C++ code with a demo for interfacing with the Robstride CAN module.

Notes:

- MacOS doesn't support a baudrate of 921600 required by the Robstride CAN module out of the box. Need to install [this driver](https://github.com/WCHSoftGroup/ch34xser_macos).
- Jetson default image has issues out of the box. Need to install [this driver](https://github.com/WCHSoftGroup/ch341ser_linux).
- Make sure to set the TTY device baudrate to 921600:

To set the baudrate on Jetson:

```bash
sudo stty -F /dev/ttyCH341USB0 921600
```

To set the baudrate on MacOS:

```bash
stty -f /dev/tty.usbserial-110 921600
```
