// MasonBot Arduino library
// 3/3/2017

#include "Arduino.h"
#include "masonbot.h"

MasonBot::MasonBot() {

	// Initialize all ports
	pinMode(_motor1_pwm, OUTPUT);
	pinMode(_motor1_dir, OUTPUT);
	pinMode(_motor1_halt, OUTPUT);

	pinMode(_motor2_pwm, OUTPUT);
	pinMode(_motor2_dir, OUTPUT);
	pinMode(_motor2_halt, OUTPUT);

	pinMode(_motor3_pwm, OUTPUT);
	pinMode(_motor3_dir, OUTPUT);
	pinMode(_motor3_halt, OUTPUT);

	// Set all low
	_stop_all_motors();
}

void MasonBot::_stop_all_motors() {
	// Stopping motors requires HALT low...
	// and PWM and DIR high.
	digitalWrite(_motor1_halt, LOW);
	digitalWrite(_motor1_pwm, HIGH);
	digitalWrite(_motor1_dir, HIGH);

	digitalWrite(_motor2_halt, LOW);
	digitalWrite(_motor2_pwm, HIGH);
	digitalWrite(_motor2_dir, HIGH);

	digitalWrite(_motor3_halt, LOW);
	digitalWrite(_motor3_pwm, HIGH);
	digitalWrite(_motor3_dir, HIGH);
}

void MasonBot::_M1Dirset(double dir){
	if (dir < 0)
		digitalWrite(_motor1_dir, HIGH);
	else
		digitalWrite(_motor1_dir, LOW);
}
	
void MasonBot::_M2Dirset(double dir){
	if (dir < 0)
		digitalWrite(_motor2_dir, HIGH);
	else
		digitalWrite(_motor2_dir, LOW);
}	 

void MasonBot::_M3Dirset(double dir){
	if (dir < 0)
		digitalWrite(_motor3_dir, HIGH);
	else
		digitalWrite(_motor3_dir, LOW);
}	 
 
	
void MasonBot::_motor1(int speed1) {
	digitalWrite(_motor1_halt, HIGH);
	analogWrite(_motor1_pwm, speed1);
	//Serial.println(speed);
}

void MasonBot::_motor2(int speed2) {
	digitalWrite(_motor2_halt, HIGH);
	analogWrite(_motor2_pwm, speed2);
	//Serial.println(speed);
}

void MasonBot::_motor3(int speed3) {
	digitalWrite(_motor3_halt, HIGH);
	analogWrite(_motor3_pwm, speed3);
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

void MasonBot::moveForward()  {
	_robo_move(1,0,0);  //move forward
}
void MasonBot::moveRotateCCW(){
	_robo_move(0,0,1);  //rotate CCW
}
void MasonBot::moveRotateCW(){
	_robo_move(0,0,-1);  //rotate CCW
}

void MasonBot::moveStop(){
	_stop_all_motors();  //stop
}

void MasonBot::_robo_move(int x, int y, int w){
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
		f_max_ard = 200/f_max;
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