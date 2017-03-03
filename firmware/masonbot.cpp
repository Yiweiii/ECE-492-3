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

MasonBot::_stop_all_motors() {
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

MasonBot::_motor1(int speed) {
	analogWrite(_motor1_pwm, speed);
}

MasonBot::_motor2(int speed) {
	analogWrite(_motor2_pwm, speed);
}

MasonBot::_motor3(int speed) {
	analogWrite(_motor3_pwm, speed);
}

int MasonBot::_read_battery() {
	digitalWrite(_battery_en_check, HIGH);
	int result = analogRead(_battery_read);
	digitalWrite(_battery_en_check, LOW);
	return result;
}

int MasonBot::getBatteryPower() {
	// Not sure if this is right....
	return _read_battery();
}
