#include "ofxDS4.h"

void ofxDS4::setup(ConectionMode mode) {

	if (mode == USB) {
		GX_Index = GX_INDEX_USB;
		GY_Index = GY_INDEX_USB;
		GZ_Index = GZ_INDEX_USB;
		DX_Index = DX_INDEX_USB;
		DY_Index = DY_INDEX_USB;
		DZ_Index = DZ_INDEX_USB;
		TIME_Index = TIME_INDEX_USB; 
	}
	else {
		GX_Index = GX_INDEX_BLUETOOTH;
		GY_Index = GY_INDEX_BLUETOOTH;
		GZ_Index = GZ_INDEX_BLUETOOTH;
		DX_Index = DX_INDEX_BLUETOOTH;
		DY_Index = DY_INDEX_BLUETOOTH;
		DZ_Index = DZ_INDEX_BLUETOOTH;
		TIME_Index = TIME_INDEX_BLUETOOTH;
	}

	_hidDevice = hid_open(VENDOR_ID, PRODUCT_ID, NULL);
	if (!_hidDevice) {
		ofLogError("Unable to find a Gyro");
	}
	
	else {
		_updateHID();
		_previousTimestamp = getTimestamp();
		_buffer[0] = 0x2;
		hid_get_feature_report(_hidDevice, _buffer, sizeof(_buffer)); // When you first pair a controller via bluetooth 
		if (_resultFlag < 0) { ofLogNotice("unable to sent report"); }
		_time = 0.;													  // starts in low energy mode where IMU data not sent
		_quaternion[0] = 1.0;										  // requesting feature report puts it into full mode. Source:   
		_quaternion[1] = 0.;										  // (https://github.com/torvalds/linux/blob/master/drivers/hid/hid-sony.c)
		_quaternion[2] = 0.;
		_quaternion[3] = 0.;
		_beta = BETA;
	}
}

void ofxDS4::update() {
	if (_hidDevice) {
		ofResetElapsedTimeCounter();
		_updateHID();
		_updateTimeStep();
		ofVec3f a = getAccel();
		ofVec3f g = getGyro();
		_updateAHRS(_timeStep, g.x, g.y, g.z, a.x, a.y, a.z);
	}
}

void ofxDS4::close()
{
	if (_hidDevice) {
		hid_close(_hidDevice);
		hid_exit();
	}
}

ofQuaternion ofxDS4::getQuaternions()
{
	ofQuaternion quaternion;
	quaternion.set(_quaternion[1], _quaternion[2], _quaternion[3], _quaternion[0]);
	return quaternion;
}

int ofxDS4::_readInt16LE(unsigned char buffer[], int index){

	unsigned short highByte = buffer[index];	//Byte order swapped in ofxDS4 io buffer
	unsigned short lowByte = buffer[index++];
	short combined = lowByte << 8 | highByte ;
	return int(combined);
}

int ofxDS4::_readInt16LEUnsigned(unsigned char buffer[], int index)
{
	unsigned short highByte = buffer[index];
	unsigned short lowByte = buffer[index++];
	unsigned short combined = lowByte << 8 | highByte;
	return int(combined);
}

int ofxDS4::_readInt8(unsigned char buffer[], int index)
{
	return int(buffer[index]);
}

void ofxDS4::_updateHID()
{
	_resultFlag = 0;
	while (_resultFlag == 0) {
		_resultFlag = hid_read(_hidDevice, _buffer, sizeof(_buffer));
		if (_resultFlag < 0) ofLogError("Unable to read data from the Gyro");
	}
}

void ofxDS4::_updateTimeStep()
{
	float timestamp = getTimestamp();
	float dif = timestamp - _previousTimestamp;
	if (dif > 0) { _timeStep = dif; }
	else { _timeStep = 0.00125; }
	_previousTimestamp = timestamp;
	_time += _timeStep;
}

void ofxDS4::_updateAHRS(float dt, float gx, float gy, float gz, float ax, float ay, float az)
{
	/*
	// This is an implementation of Seb Madgwick's AHRS filter.
	// (http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
	// TODO: This assumes that the initial orientation of the controller
	// is such that gravity points in the -z direction, but in reality the 
	// controller is usually initially oriented so that gravity points in the 
	// -y direction. This is why the sphere in the test app processes pi/2 radians
	// about the x axis in the ~5 secs after the test app is started. It's not ideal 
	// and could be fixed, either by re-writing the algo' with the correct 
	// initial condition, or (possibly) by altering the axis order of the controller.
	*/

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-_quaternion[1] * gx - _quaternion[2] * gy - _quaternion[3] * gz);
	qDot2 = 0.5f * (_quaternion[0] * gx + _quaternion[2] * gz - _quaternion[3] * gy);
	qDot3 = 0.5f * (_quaternion[0] * gy - _quaternion[1] * gz + _quaternion[3] * gx);
	qDot4 = 0.5f * (_quaternion[0] * gz + _quaternion[1] * gy - _quaternion[2] * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1. / sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * _quaternion[0];
		_2q1 = 2.0f * _quaternion[1];
		_2q2 = 2.0f * _quaternion[2];
		_2q3 = 2.0f * _quaternion[3];
		_4q0 = 4.0f * _quaternion[0];
		_4q1 = 4.0f * _quaternion[1];
		_4q2 = 4.0f * _quaternion[2];
		_8q1 = 8.0f * _quaternion[1];
		_8q2 = 8.0f * _quaternion[2];
		q0q0 = _quaternion[0] * _quaternion[0];
		q1q1 = _quaternion[1] * _quaternion[1];
		q2q2 = _quaternion[2] * _quaternion[2];
		q3q3 = _quaternion[3] * _quaternion[3];

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _quaternion[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * _quaternion[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * _quaternion[3] - _2q1 * ax + 4.0f * q2q2 * _quaternion[3] - _2q2 * ay;
		recipNorm = 1. / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= _beta * s0;
		qDot2 -= _beta * s1;
		qDot3 -= _beta * s2;
		qDot4 -= _beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	_quaternion[0] += qDot1 * dt;
	_quaternion[1] += qDot2 * dt;
	_quaternion[2] += qDot3 * dt;
	_quaternion[3] += qDot4 * dt;

	// Normalise quaternion
	recipNorm = 1. / sqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
	_quaternion[0] *= recipNorm;
	_quaternion[0] *= recipNorm;
	_quaternion[2] *= recipNorm;
	_quaternion[3] *= recipNorm;
}

