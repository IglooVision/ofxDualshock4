# ofxDualshock4 #
ofxDualShock4 is an addon which accesses the gyroscope and accelerometer data from a PS4 controller, and uses them to estimate the controller’s rotation. Currently the addon does not allow access to the state of controller buttons / joysticks, however extending it to provide this would not be particularly challenging. The controller can also only be connected via bluetooth as it stands - again it is possible to extend to suppot a controller connected via USB.

PS4 controllers present as HID devices on PCs and this addon uses the cross platform HIDAPI libary (http://www.signal11.us/oss/hidapi/) to access sensor data. Currently the addon only supports windows, but it’s just a matter of tweaking compiler settings to get it to compile on OSX / Linux. 

To build up an estimate estimate of controller orientation from raw accelerometer and gyroscope data, Seb Madgwick's IMU sensor fusion algorithm (http://www.x-io.co.uk/open-source-ahrs-with-x-imu/). 

Igloo Vision (http://www.igloovision.com/) are intrested in this technology because of its potential to enable the intuative exploration of VR environments. 

## Installation ##
Under windows and VS2015 the this addon can be included using the openframeworks project generator. Once you've generated your project do need to tweek project settings slightly:
-   Using the solution  explorer navigate to /addons/ofxDualshock4/libs/HIDAPI/hid.c
-  Right click on hid.c in the solution explorer and select 'properties'
-   Check all configurations is selected in the top right hand box
-  In the box on the left, click C/C++ -> all options
-  In the search bar type 'compile as'
-  Select to compile as select C code

For other systems you'll have to manually add the hidapi source files to the linker and flag hid.c to compile as C code but it should still work okay

## Pairing a PS4 controller to a PC via bluetooth ##
To pair the device:
-   Open up Bluetooth settings on the computer
-   Press and hold the share button and the button with the PS logo simultaneously, and unti the light bar starts to flash white
-   At this point a game controller just pop up in the list of devices on your computer - click pair and you should be good to go!  

## Limitations / TODO ##
-   Provide access to state of controller buttons / joysticks
-   Setup Linux / OSX support
-   Look into supporting multiple controllers (currently you can only use 1 at a time)
-   Implement a switch to select bluetooth / USB at setup

## Compatibility ##
Written for of openFrameworks 0.9.3, haven't tested with other versions

## Troubleshooting ##

### Slow update rate ###
If the addon works but there's a large delay between device updates, it's possible a Windows power setting is to blaim. To ensure this is not the case, go to cotrol panel -> power settings, and check that high performance mode is selected. If your on balanced mode you might have problems (even on a desktop PC). 