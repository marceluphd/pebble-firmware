
nRF Connect Programmer is implemented as an app for nRF Connect. The customized nRF Connect Programmer by IoTeX (i.e., *pc-nrfconnect-programmer-1.4.2.tgz*) allows 
a user to flash firmware on Windows and Linx for both device and modem, respectively. 

# Install IoTeX nRF Connect Programmer on Windows

1. Copy the file *pc-nrfconnect-programmer-1.4.2.tgz* to %USERPROFILE%/.nrfconnect-apps/local;
2. Restart nRF Connect for Desktop and the IoTeX nRF Connect Programmer (i.e., Programmer-Iotex) should now appear in the apps list.

# Install IoTeX nRF Connect Programmer on Linux

1. Install JLINK_ARM for Linux (Please refere to https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack);
2. Install *pebble-udev_1.0.1-all.deb* to allow the Programmer app to read /dev/ttyUSB* CP2105 serial port device;
3. Unzip the file *pc-nrfconnect-programmer-1.4.2.tgz* and modify the source code of 'lib\actions\mcubootTargetActions.js' as follows:
   - Remove '$' in 'return serialports.find(s => (/-if01$/.test(s.pnpId)));', i.e.,
     ```javascript 
     case 'lin':
            return serialports.find(s => (/-if01/.test(s.pnpId)));
     ```
   - Remove '$' in 'return serialports.find(s => (/-if00/.test(s.pnpId)))', i.e.,
     ```javascript 
     case 'lin':
            return serialports.find(s => (/-if00/.test(s.pnpId)));
     ```
 4. Install nRF Connect for Desktop v3.4.2 (The latest version v.3.6.0 has some issues);
 5. Copy the updated file *pc-nrfconnect-programmer-1.4.2.tgz* to $HOME/.nrfconnect-apps/local;
 6. Restart nRF Connect for Desktop and the IoTeX nRF Connect Programmer (i.e., Programmer-Iotex) should now appear in the apps list.
 
