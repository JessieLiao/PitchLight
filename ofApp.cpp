#include "ofApp.h"

bool motor_on = true;
int speed = 180;

//--------------------------------------------------------------
//--------------------------------------------------------------
void ofApp::setup() {
	ofSetVerticalSync(true);

	plotHeight = 128;
	bufferSize = 2048;

	fft = ofxFft::create(bufferSize, OF_FFT_WINDOW_HAMMING);
	// To use FFTW, try:
	//fft = ofxFft::create(bufferSize, OF_FFT_WINDOW_HAMMING, OF_FFT_FFTW);

	drawBins.resize(fft->getBinSize());
	middleBins.resize(fft->getBinSize());
	audioBins.resize(fft->getBinSize());

	// 0 output channels,
	// 1 input channel
	// 44100 samples per second
	// [bins] samples per buffer
	// 4 num buffers (latency)

	ofSoundStreamSetup(0, 1, this, 44100, bufferSize, 4);

	ofSetFrameRate(60);

	ofBackground(0, 0, 0);

	buttonState = "digital pin:";
	potValue = "analog pin:";

	// replace the string below with the serial port for your Arduino board
	// you can get this from the Arduino application or via command line
	// for OSX, in your terminal type "ls /dev/tty.*" to get a list of serial devices
	ard.connect("COM6", 57600);

	// listen for EInitialized notification. this indicates that
	// the arduino is ready to receive commands and it is safe to
	// call setupArduino()
	ofAddListener(ard.EInitialized, this, &ofApp::setupArduino);
	bSetupArduino = false;	// flag so we setup arduino when its ready, you don't need to touch this :)
}

//--------------------------------------------------------------
void ofApp::update() {
	if (ard.isArduinoReady() && !bSetupArduino) {
		const int i = 0;
		setupArduino(i);
	}
	updateArduino();

}

//--------------------------------------------------------------
void ofApp::setupArduino(const int & version) {

	// remove listener because we don't need it anymore
	ofRemoveListener(ard.EInitialized, this, &ofApp::setupArduino);

	// it is now safe to send commands to the Arduino
	bSetupArduino = true;

	// print firmware name and version to the console
	ofLogNotice() << ard.getFirmwareName();
	ofLogNotice() << "firmata v" << ard.getMajorFirmwareVersion() << "." << ard.getMinorFirmwareVersion();

	// Note: pins A0 - A5 can be used as digital input and output.
	// Refer to them as pins 14 - 19 if using StandardFirmata from Arduino 1.0.
	// If using Arduino 0022 or older, then use 16 - 21.
	// Firmata pin numbering changed in version 2.3 (which is included in Arduino 1.0)
	ard.sendDigitalPinMode(2, ARD_OUTPUT);
	ard.sendDigitalPinMode(3, ARD_OUTPUT);
	ard.sendDigitalPinMode(4, ARD_OUTPUT);
	ard.sendDigitalPinMode(5, ARD_OUTPUT);
	ard.sendDigitalPinMode(6, ARD_OUTPUT);
	ard.sendDigitalPinMode(7, ARD_OUTPUT);
	ard.sendDigitalPinMode(8, ARD_OUTPUT);

}

//--------------------------------------------------------------
void ofApp::updateArduino() {

	// update the arduino, get any data or messages.
	// the call to ard.update() is required
	ard.update();

	// do not send anything until the arduino has been set up
	if (bSetupArduino) {
		// fade the led connected to pin D11
		ard.sendPwm(13, (int)(128 + 128 * sin(ofGetElapsedTimef())));   // pwm...
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ard.sendDigital(2, ARD_LOW);
	ard.sendDigital(3, ARD_LOW);
	ard.sendDigital(4, ARD_LOW);
	ard.sendDigital(5, ARD_LOW);
	ard.sendDigital(6, ARD_LOW);
	ard.sendDigital(7, ARD_LOW);
	ard.sendDigital(8, ARD_LOW);

	float cbin = fft->getBinFromFrequency(523.25, 44100.0);
	float campbin = fft->getAmplitudeAtBin(cbin);

	float dbin = fft->getBinFromFrequency(587.33, 44100.0);
	float dampbin = fft->getAmplitudeAtBin(dbin);

	float ebin = fft->getBinFromFrequency(659.25, 44100.0);
	float eampbin = fft->getAmplitudeAtBin(ebin);

	float fbin = fft->getBinFromFrequency(698.46, 44100.0);
	float fampbin = fft->getAmplitudeAtBin(fbin);

	float gbin = fft->getBinFromFrequency(783.99, 44100.0);
	float gampbin = fft->getAmplitudeAtBin(gbin);

	float abin = fft->getBinFromFrequency(880.00, 44100.0);
	float aampbin = fft->getAmplitudeAtBin(abin);

	float bbin = fft->getBinFromFrequency(987.77, 44100.0);
	float bampbin = fft->getAmplitudeAtBin(bbin);

	if (campbin > 0.1) {
		ofBackground(0, 0, 0);
		cout << "C" << endl;
		ard.sendDigital(2, ARD_HIGH);
	}
	else if (dampbin > 0.1) {
		ofBackground(36, 36, 36);
		cout << "D" << endl;
		ard.sendDigital(3, ARD_HIGH);
	}
	else if (eampbin > 0.1) {
		ofBackground(72, 72, 72);
		cout << "E" << endl;
		ard.sendDigital(4, ARD_HIGH);
	}
	else if (fampbin > 0.1) {
		ofBackground(108, 108, 108);
		cout << "F" << endl;
		ard.sendDigital(5, ARD_HIGH);
	}
	else if (gampbin > 0.1) {
		ofBackground(144, 144, 144);
		cout << "G" << endl;
		ard.sendDigital(6, ARD_HIGH);
	}
	else if (aampbin > 0.1) {
		ofBackground(180, 180, 180);
		cout << "A" << endl;
		ard.sendDigital(7, ARD_HIGH);
	}
	else if (bampbin > 0.1) {
		ofBackground(216, 216, 216);
		cout << "B" << endl;
		ard.sendDigital(8, ARD_HIGH);
	}
	else {
		ofBackground(ofColor::lavender);
	}

	ofSetColor(255);
	ofPushMatrix();
	ofTranslate(16, 16);

	soundMutex.lock();
	drawBins = middleBins;
	soundMutex.unlock();

	ofDrawBitmapString("Frequency Domain", 0, 0);
	plot(drawBins, -plotHeight, plotHeight / 2);
	ofPopMatrix();
	string msg = ofToString((int)ofGetFrameRate()) + " fps";
	ofDrawBitmapString(msg, ofGetWidth() - 80, ofGetHeight() - 20);
	string amplitude = ofToString((int)fft->getAmplitude());
	ofDrawBitmapString(amplitude, 5, ofGetHeight() - 20);
}

void ofApp::plot(vector<float>& buffer, float scale, float offset) {
	ofNoFill();
	int n = buffer.size();
	ofDrawRectangle(0, 0, n, plotHeight);
	glPushMatrix();
	glTranslatef(0, plotHeight / 2 + offset, 0);
	ofBeginShape();
	for (int i = 0; i < n; i++) {
		ofVertex(i, sqrt(buffer[i]) * scale);
	}
	ofEndShape();
	glPopMatrix();
}

void ofApp::audioReceived(float* input, int bufferSize, int nChannels) {
	float maxValue = 0;
	for (int i = 0; i < bufferSize; i++) {
		if (abs(input[i]) > maxValue) {
			maxValue = abs(input[i]);
		}
	}
	for (int i = 0; i < bufferSize; i++) {
		input[i] /= maxValue;
	}

	fft->setSignal(input);

	float* curFft = fft->getAmplitude();
	memcpy(&audioBins[0], curFft, sizeof(float) * fft->getBinSize());

	maxValue = 0;
	for (int i = 0; i < fft->getBinSize(); i++) {
		if (abs(audioBins[i]) > maxValue) {
			maxValue = abs(audioBins[i]);
		}
	}
	for (int i = 0; i < fft->getBinSize(); i++) {
		audioBins[i] /= maxValue;
	}

	soundMutex.lock();
	middleBins = audioBins;
	soundMutex.unlock();
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
