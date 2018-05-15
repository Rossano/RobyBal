/*
 * Motor.h
 *
 *  Created on: 14 avr. 2018
 *      Author: Ross
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "mbed.h"

#define ENABLE	true
#define DSIABLE false

enum motor_direction
{
	STOP,
	FORWARD,
	BACKWARD
};

enum motor_rotation
{
	CLOCKWISE,
	ANTICLOCKWISE
};

class Motor {
public:
	Motor(PinName EN, PinName DIR, PinName PWM, uint16_t FREQ);
	void enableBridge();
	void disableBridge();
	void stop();
	void run();
	void HiZ();
	void setPwmFrequency(uint16_t FREQ);
	void setRotation(motor_rotation rot);
	motor_rotation getRotation();
	void setDirection();
	void resetDirection();
	bool getDirection();
	uint16_t getPwmFrequency();
	float getPwmValue();
	void setPwm(float PWM);
	Motor& operator=(Motor &m);
	Motor operator=(float PWM);
	void setDebug();
	void resetDebug();
private:
	DigitalOut *enable;
	DigitalOut *direction;
	PwmOut *pwm;
	uint16_t frequency;
	motor_rotation rotationDir;
	bool active;
	bool debug;
};

#endif /* MOTOR_H_ */
