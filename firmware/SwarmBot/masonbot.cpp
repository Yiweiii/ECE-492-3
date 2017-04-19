// MasonBot Arduino library
// 3/3/2017

#include "Arduino.h"
#include "masonbot.h"

MasonBot::MasonBot() {
	// Initialize all ports
	pinMode(_motor1_pwm_A, OUTPUT);
	pinMode(_motor1_pwm_B, OUTPUT);

	pinMode(_motor2_pwm_A, OUTPUT);
	pinMode(_motor2_pwm_B, OUTPUT);

	pinMode(_motor3_pwm_A, OUTPUT);
	pinMode(_motor3_pwm_B, OUTPUT);

	// Set all low
	_stop_all_motors();
}

void MasonBot::_stop_all_motors() {
	// Stopping motors requires HALT low...
	// and PWM and DIR high.
	digitalWrite(_motor1_pwm_A, HIGH);
	digitalWrite(_motor1_pwm_B, HIGH);
  
	digitalWrite(_motor2_pwm_A, HIGH);
	digitalWrite(_motor2_pwm_B, HIGH);

	digitalWrite(_motor3_pwm_A, HIGH);
	digitalWrite(_motor3_pwm_B, HIGH);
}

void MasonBot::_M1Dirset(double dir) {
	_dir1 = dir;
}
	
void MasonBot::_M2Dirset(double dir) {
	_dir2 = dir;
}	 

void MasonBot::_M3Dirset(double dir) {
	_dir3 = dir;
}	 
	
void MasonBot::_motor1(int speed1) {
	if (_dir1 > 0) {
		analogWrite(_motor1_pwm_A, speed1);
		digitalWrite(_motor1_pwm_B, LOW);
	} else if (_dir1 < 0) {
		analogWrite(_motor1_pwm_B, speed1);
		digitalWrite(_motor1_pwm_A, LOW);
	} else {
		digitalWrite(_motor1_pwm_A, HIGH);
		digitalWrite(_motor1_pwm_B, HIGH);
	}
}

void MasonBot::_motor2(int speed2) {
	if (_dir2 > 0) {
		analogWrite(_motor2_pwm_A, speed2);
		digitalWrite(_motor2_pwm_B, LOW);
	} else if (_dir2 < 0) {
		analogWrite(_motor2_pwm_B, speed2);
		digitalWrite(_motor2_pwm_A, LOW);
	} else {
		digitalWrite(_motor2_pwm_A, HIGH);
		digitalWrite(_motor2_pwm_B, HIGH);
	}
	////Serial.println(speed);
}

void MasonBot::_motor3(int speed3) {
	if (_dir3 > 0) {
		analogWrite(_motor3_pwm_A, speed3);
		digitalWrite(_motor3_pwm_B, LOW);
	} else if (_dir3 < 0) {
		analogWrite(_motor3_pwm_B, speed3);
		digitalWrite(_motor3_pwm_A, LOW);
	} else {
		digitalWrite(_motor3_pwm_A, HIGH);
		digitalWrite(_motor3_pwm_B, HIGH);
	}
//Serial.println(speed);
}

int MasonBot::_read_battery() {
	digitalWrite(_battery_en_check, HIGH);
	int result = (5/1024.0)*analogRead(_battery_read);
	digitalWrite(_battery_en_check, LOW);
	return result;
}

int MasonBot::getBatteryPower() {
	// Not sure if this is right....
	return _read_battery();
}

void MasonBot::moveForward(int velocity) {
	_robo_move(1,0,0, velocity);  //move forward
}
void MasonBot::moveRotateCCW(int velocity) {
	_robo_move(0,0,1, velocity);  //rotate CCW
}
void MasonBot::moveRotateCW(int velocity) {
	_robo_move(0,0,-1, velocity);  //rotate CCW
}

void MasonBot::moveStop() {
	_stop_all_motors();  //stop
}

void MasonBot::runForward(int *count){
	*count = 0;
	Serial.println(*count);
	while (*count < 9){
		moveForward(120);
	}
	_stop_all_motors();  
}

void MasonBot::feedbackRun(float Xloc, float Yloc, int Thetaloc, float Xexpected, float Yexpected, int Thetaexpected){
	int velocity;
	int direction;
	float dirchoice;
	int rotmag2;
	float mag = sqrt(pow(Xloc - Xexpected, 2) + pow(Yloc - Yexpected,2));
	// mag is in meters
	int rotmag = Thetaexpected - Thetaloc;
	dirchoice = (rotmag + 360) % 360;
	rotmag = abs(rotmag);
	if (rotmag > 180)
	    rotmag = abs(360 - rotmag); 
	//rotmag is in degrees
	if (rotmag > 10 and mag > .10){
		velocity = 50 + (.72*rotmag);
		if (velocity > 150){
			velocity = 150;
		}
		Serial.println(rotmag2);
		if (dirchoice >= 180){
			Serial.println("CW"); 
			moveRotateCW(velocity);
		}else{
			////////Serial.println("CCW");
			moveRotateCCW(velocity); 
		}
	}else if( mag > .10){
		
		velocity = 50 + 400*(mag);
		if (velocity > 250){
			velocity = 250;
		}
		moveForward(velocity);
	} else {
		_stop_all_motors();		
	}

}


void MasonBot::controlRun(int *count, float Xloc, float Yloc, float Thetaloc, float Xexpected, float Yexpected, float Thetaexpected){
	int velocity;
	float mag = sqrt(pow(Xloc - Xexpected, 2) + pow(Yloc - Yexpected,2));
	
	// mag is in meters .23 meters for 8 counts
	float countmax = ((mag/.223) * 8);
	float rotmag = Thetaexpected - Thetaloc;
	rotmag = abs(rotmag);
	float countmaxrot = (rotmag/135) * 8;
	*count = 0;
	while (*count < countmaxrot){
		velocity = (int)( 75 + 5*(countmaxrot - *count));
		moveRotateCW(velocity);
		//Serial.println(velocity);
	}  
	*count = 0;
     //Serial.println(countmax);
	while (*count < countmax){
		velocity = (int)(75 + (countmax - *count));	
		if (velocity > 225) 	
			velocity = 225;
		moveForward(velocity);
		//Serial.println(*count);
	}
	_stop_all_motors();
}


void MasonBot::_robo_move(int x, int y, int w, int velocity) {
	//matrix equation to calc. forces for each of the motors of the holonomic robot
	double f1 = (0.58*x) - (0.33*y) + (0.33*w);
	double f2 = (-0.58*x) - (0.33*y) + (0.33*w);
	double f3 = (0*x) + (0.67*y) + (0.33*w);
	
	double f1t = abs(f1);
	double f2t = abs(f2);
	double f3t = abs(f3);
	double f_max = 0;
	double f_max_ard = 0;	


	if ((f1t>f2t) & (f1t>f3t))
		f_max = f1t;
	else if(f2t>f3t)
		f_max = f2t;
	else 
		f_max = f3t;

	if (f_max != 0)				//if max force is non-zero
		f_max_ard = (200/f_max) * (double(velocity)/200);
	else				        //if max force is 0 (i.e STOP)
		f_max_ard = 0;

	int dc1_ard = int(f1t*f_max_ard);	//normalized duty-cycle1
	int dc2_ard = int(f2t*f_max_ard);	//normalized duty-cycle2
	int dc3_ard = int(f3t*f_max_ard);	//normalized duty-cycle3


	_M1Dirset(f1);
	_M2Dirset(f2);
	_M3Dirset(f3);
			

	_motor1(dc1_ard);
	_motor2(dc2_ard);
	_motor3(dc3_ard);
}
