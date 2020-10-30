
# IoTeX Pebble Firmware

The firmware implements the asset tracking application using Pebble hardware designed by IoTeX.

## IoTeX nRF Connect Programmer

nRF Connect Programmer is implemented as an app for nRF Connect. The customized nRF Connect Programmer by IoTeX (i.e., *pc-nrfconnect-programmer-1.4.2.tgz*) allows 
a user to flash firmware on Windows and Linx for both device and modem, respectively. 

### Install IoTeX nRF Connect Programmer on Windows

1. Copy the file *pc-nrfconnect-programmer-1.4.2.tgz* to %USERPROFILE%/.nrfconnect-apps/local;
2. Restart nRF Connect for Desktop and the IoTeX nRF Connect Programmer (i.e., Programmer-Iotex) should now appear in the apps list.

### Install IoTeX nRF Connect Programmer on Linux

1. Install JLINK_ARM for Linux (Please refere to https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack);
2. Install *pebble-udev_1.0.1-all.deb* to allow the Programmer app to read /dev/ttyUSB* CP2105 serial port device;
3. Install nRF Connect for Desktop v3.4.2 (The latest version v.3.6.0 has some issues);
4. Copy the file *pc-nrfconnect-programmer-1.4.2.tgz* to $HOME/.nrfconnect-apps/local;
5. Restart nRF Connect for Desktop and the IoTeX nRF Connect Programmer (i.e., Programmer-Iotex) should now appear in the apps list.
