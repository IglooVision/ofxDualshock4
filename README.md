# ofxDualshock4 #
This addon phrases the IMU (Inertial Measurement Unit, https://en.wikipedia.org/wiki/Inertial_measurement_unitdata) which a PS4 controller provides over HID, and uses this info' to estimate the orientation of the controller. It could be extended to access the rest of the HID data provided by the controller fairly easily.
 
The orientation estimate is developed using Seb Madgwick's AHRS algorithm (http://www.x-io.co.uk/open-source-ahrs-with-x-imu/), and HID data is accessed via HIDAPI (https://github.com/signal11/hidapi).

Igloo Vision (http://www.igloovision.com/) use this addon to allow a users to interact with 360degree immersive environments. 


## Installation ##
Under windows and VS2015 the this addon can be included using the openframeworks project generator. Once you've generated your project do need to teek project settings slightly:
1) Using the solution  explorer navigate to /addons/ofxDualshock4/libs/HIDAPI/hid.c
2) Right click on hid.c in the solution explorer and select 'properties'
2.1) Check all configurations is selected in the top right hand box
3) In the box on the left, click C/C++ -> all options
4) In the search bar type 'compile as'
5) Select to compile as select C code

For other systems you'll have to manually add the hidapi source files to the linker and flag hid.c to compile as C code but it should still work okay

## Compatibility ##
Written for of 0.9.3, haven't tested with other versions

## Contact info' ##
Any problems, give me an email (jack.halliday12@imperial.ac.uk).
