/*
 * Stspin240.h
 *
 *  Created on: 15 avr. 2018
 *      Author: Ross
 */

#ifndef STSPIN240_H_
#define STSPIN240_H_

#define WAIT_MS		10

#include "mbed.h"
#include <MBED/motor.h>
#include "Robo_motor.h"

class Stspin240 {
public:

private:
	Motor *ma;
	Motor *mb;
	DigitalOut *enable;
	DigitalOut *reset;
	DigitalOut *ref;
	unsigned int pwmFrequency;
	float speedA;
	float speedB;
	bool debug;
public:
	Stspin240(PinName EN, PinName REF, PinName RST, PinName PHA, PinName PWMA, PinName PHB, PinName PWMB, unsigned int freq);
	virtual ~Stspin240();
	void HiZ();
	void setReset();
	void releaseReset();
	void sleep();
	void awake();
	void enableBridge();
	void disableBridge();
	void enableRef();
	void disableRef();
	void setDirectionA(motor_direction dir);
	void setDirectionB(motor_direction dir);
	//void setDirectionA(eMotorMove_t dir);
	//void setDirectionB(eMotorMove_t dir);
	void setRotationA(motor_rotation rot);
	void setRotationB(motor_rotation rot);
	motor_direction getDirectionA();
	motor_direction getDirectionB();
	//eMotorMove_t getDirectionA();
	//eMotorMove_t getDirectionB();
	motor_rotation getRotationA();
	motor_rotation getRotationB();
	void setPwmFrequency(unsigned int freq);
	unsigned int getPwmFrequency();
	void updateSpeed(float sA, float sB);
	void updateSpeedA(float speed);
	void updateSpeedB(float speed);
	void stop();
	void run();
	void setDebug();
	void resetDebug();
};

extern Stspin240 * motor;

#endif /* STSPIN240_H_ */
