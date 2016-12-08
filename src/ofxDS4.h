// App to phrase IMU data supplied via HID from a PS4 controller, and estimate the orientation of the controller
// Written by Jack Halliday (jack.halliday12@imperial.ac.uk) for Igloo Vision LTD (http://www.igloovision.com/).

#pragma once
#include "hidapi.h"
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h>
#include "ofMain.h"

//****************************  DEVICE ID  ****************************//
/*
// Address the correct HID using it's vendor and product IDs
*/
#define VENDOR_ID 1356		// The vendor & product ID of HID device to be used:
#define PRODUCT_ID 1476		//		(Currently setup for PS4 controller)

//****************************  SENSOR DETAILS  ****************************//
/*
// Values for the ranges of sensors, and factors to map sensor units into SI units
// These have been worked out from a combination of guess work and info contained 
// at:
//	1) https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI055-DS000-08.pdf (IMU datasheet)
//	2) http://gamedev.stackexchange.com/questions/87106/accessing-dualshock-4-motion-sensor-in-windows-ideally-unity
//  3) https://github.com/Flafla2/Unity-DS4
*/
#define ACC_FACTOR 9.8 / 8100.		// No' of m/s2 per arb unit of acceleration

#define GYRO_FACTOR 1. / 1024.		// No of rad/s per arb unit of angular velocity
#define GYRO_ERROR 0.001			// Estimated error on gyroscope measurement in rad/s

#define TIME_FACTOR 0.00125 / 188.	// No' of secs per arb unit of time 


//****************************  MEMORY ADDRESSES  ****************************//
/*
// The location of relevant bytes within the HID read buffer. All the motion sensors
// use 16bit memory space, and values here refer to the byte with smaller index. Note that
// byte order is reversed for all of the IMU outputs. Values here have come from:
// 1) https://gist.github.com/johndrinkwater/7708901
// 2) http://eleccelerator.com/wiki/index.php?title=DualShock_4
// 3) https://github.com/ehd/node-ds4
// The coordinate sys used by the IMU is right handed, and oriented as shown in 
// - \GyroProject\docs\DS4-axis.pdf
*/


// Indices for controller connected via USB
#define GX_INDEX_USB 14		
#define GY_INDEX_USB 16
#define GZ_INDEX_USB 18
#define DX_INDEX_USB 20
#define DY_INDEX_USB 22
#define DZ_INDEX_USB 24
#define TIME_INDEX_USB 11 //Index of the timestamp byte

// Indices for controller connected via Bluetooth
#define GX_INDEX_BLUETOOTH 16
#define GY_INDEX_BLUETOOTH 18
#define GZ_INDEX_BLUETOOTH 20
#define DX_INDEX_BLUETOOTH 22
#define DY_INDEX_BLUETOOTH 24
#define DZ_INDEX_BLUETOOTH 26
#define TIME_INDEX_BLUETOOTH 13


//****************************   ALGO' SETTING    ****************************//
/*
//  We use Seb Madgwick's IMU sensor fusion algo' to couple the gro' and accelerometer
// sensor readings from the controller. For more detail, refer to Seb's website:
// http://www.x-io.co.uk/open-source-ahrs-with-x-imu/.
// 
// My understanding of what Beta does is to dictate how much attention is paid to the 
// accelerometer data in the sensor fusion process. This define specifies it's default
// value - it can also be changed via a set method within the class.
*/
#define BETA 0.5

enum ConectionMode {USB, BLUETOOTH};

class ofxDS4{

	public:
		void setup(ConectionMode mode = BLUETOOTH);
		void update();
		void close();
		void setBeta(float beta) { _beta = beta; }
		bool getDS4Found() { return _hidDevice; }
		
		int getRawGyroX()		{ return _readInt16LE(_buffer, GX_Index); }
		int getRawGyroY()		{ return _readInt16LE(_buffer, GY_Index); }
		int getRawGyroZ()		{ return _readInt16LE(_buffer, GZ_Index); }

		int getRawAccelX()		{ return _readInt16LE(_buffer, DX_Index); }
		int getRawAccelY()		{ return _readInt16LE(_buffer, DY_Index); }
		int getRawAccelZ()		{ return _readInt16LE(_buffer, DZ_Index); }

		int getRawTimestamp() { return _readInt16LE(_buffer, TIME_Index); }
		float getTimestamp() { return float(getRawTimestamp() * TIME_FACTOR);  }
		float getTime() { return _time; }

		float getGyroX() { return float( getRawGyroX() ) * GYRO_FACTOR; }
		float getGyroY() { return float ( getRawGyroY() ) * GYRO_FACTOR; }
		float getGyroZ() { return float ( getRawGyroZ() ) * GYRO_FACTOR; }

		float getAccelX() { return getRawAccelX() * ACC_FACTOR; }
		float getAccelY() { return getRawAccelY() * ACC_FACTOR; }
		float getAccelZ() { return getRawAccelZ() * ACC_FACTOR; }
		
		ofVec3f getGyro() { return ofVec3f(getGyroX(), getGyroY(), getGyroZ()); }
		ofVec3f getAccel() { return ofVec3f(getAccelX(), getAccelY(), getAccelZ()); }
		ofQuaternion getQuaternions();
		

		int getArbPair(int index) { return _readInt16LE(_buffer, index); }
		int getArbSingle(int index) { return _readInt8(_buffer, index); }
	
	private:
		int _readInt16LE(unsigned char buffer[], int index);
		int _readInt16LEUnsigned(unsigned char buffer[], int index);
		int _readInt8(unsigned char buffer[], int index);

		void _updateHID();
		void _updateTimeStep();
		void _updateAHRS(float dt, float gx, float gy, float gz, float ax, float ay, float az);
		
		hid_device* _hidDevice;
		unsigned char _buffer[256];
		int _resultFlag;
		
		float _previousTimestamp;
		float _timeStep;
		float _time;

		float _quaternion[4];
		float _beta;

		int GX_Index;
		int GY_Index; 
		int GZ_Index;
		int DX_Index; 
		int DY_Index;
		int DZ_Index; 
		int TIME_Index; 
};
