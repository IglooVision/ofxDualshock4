#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	_ds4.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
	_ds4.update();
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofQuaternion q = _ds4.getQuaternions();
	ofVec3f axis;
	float angle;
	q.getRotate(angle, axis);

	string gyroVals = "Gyro : " + ofToString(_ds4.getGyro(), 4);
	string accelVals = "Accel : " + ofToString(_ds4.getAccel(), 4);
	string instructions = "Rotation of the sphere approximates rotation of the controller.";
	
	ofPushStyle();
		ofPushView();
			ofTranslate(ofGetWindowWidth() / 2., ofGetWindowHeight() / 2.);
			ofClear(ofColor::black);
			ofRotate(angle, axis.x, axis.y, axis.z);
			ofSetLineWidth(5);
			ofDrawAxis(150);
			ofNoFill();
			ofSetLineWidth(0.5);
			ofSetColor(255, 255, 255, 127);
			ofDrawSphere(0, 0, 0, 200);
		ofPopView();
	ofPopStyle();


	ofPushStyle();
		ofPushView();
			ofTranslate(100, 100);
			ofSetColor(ofColor::white);
			ofDrawBitmapString(gyroVals, 0, 0);
			ofDrawBitmapString(accelVals, 0, 25);
			ofDrawBitmapString(instructions, 0, 50);
		ofPopView();
	ofPopStyle();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
