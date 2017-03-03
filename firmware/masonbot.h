// MasonBot Arduino library
// 3/3/2017
//
//
// Arduino pins 2-13 and 44-46 for PWM on Arduino Mega
// 2, 3, 18, 19, 20, 21 interrupt pins

#ifndef MASONBOT_H
#define MASONBOT_H

/* Pinout definitions
 * The following correspond to pins on the Arduino Mega board.
 */

/* Motor pins */
#define _motor1_pwm 44
#define _motor1_dir 22
#define _motor1_halt 23

#define _motor2_pwm 45
#define _motor2_dir 24
#define _motor2_halt 25

#define _motor3_pwm 46
#define _motor3_dir 26
#define _motor3_halt 27

/* IR light gate pins
 *
 * IR receivers are put on interrupt lines.
 */
#define _ir_en_1 28
#define _ir_en_2 29
#define _ir_en_3 30

#define _ir_read_1 2
#define _ir_read_2 3
#define _ir_read_3 18

/* Battery check pins */
#define _battery_read 0 // Analog in header?
#define _battery_en_check 47

#include "Arduino.h"

class MasonBot {
	public:
		MasonBot();
		// Motor control
		//
	private:
		_stop_all_motors();
};
#endif


