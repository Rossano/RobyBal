/*
 * motor.cpp
 *
 *  Created on: 11 mai 2018
 *      Author: Ross
 */

#include <mbed.h>
#include <MBED/Robo_motor.h>
#include "board.h"
#include "Stspin240.h"


eMotorTurn_t eMotorTurn;
//eMotorMove_t eMotorMove;
float fMotorA_Offset = 0;
float fMotorB_Offset = 0;
float fMotorA_Move = 0;
float fMotorB_Move = 0;

//_motor motor;

extern void vUsage(char *str);

#if 0
_motor::_motor() {
	// Configure Arduino I/Os
	pinMode(MOTOR_E1, OUTPUT);
	pinMode(MOTOR_M1, OUTPUT);
	pinMode(MOTOR_E2, OUTPUT);
	pinMode(MOTOR_M2, OUTPUT);

	// Set outputs
	digitalWrite(MOTOR_E1, LOW);
	digitalWrite(MOTOR_M1, LOW);
	digitalWrite(MOTOR_E2, LOW);
	digitalWrite(MOTOR_M2, LOW);

	pwmA = pwmB = 0;
	moveStatus = STOP;
	turnStatus = NO_TURN;
	uiMotorA_Offset = 0;
	uiMotorB_Offset = 0;
}

_motor::~_motor() {
	// TODO Auto-generated destructor stub
}

void _motor::move_A(int pwm)
{
	// Make saturation
	if (pwm > 255)
	{
		pwmA = 255;
	}
	else if (pwm < -255)
	{
		pwmA = -255;
	}
	else
	{
		pwmA = pwm;
	}

	// Checks direction and make the motor A moving
	if (pwmA > 0)
	{
		digitalWrite(MOTOR_M1, LOW);
		analogWrite(MOTOR_E1, pwmA);
	}
	else if (pwmA < 0)
	{
		digitalWrite(MOTOR_M1, HIGH);
		analogWrite(MOTOR_E1, -pwmA);
	}
	else
	{
		this->stop_A();
	}
}

void _motor::move_B(int pwm)
{
	// Make saturation
	if (pwm > 255)
	{
		pwmB = 255;
	}
	else if (pwm < -255)
	{
		pwmB = -255;
	}
	else
	{
		pwmB = pwm;
	}

	// Checks direction and make the motor A moving
	if (pwmB > 0)
	{
		digitalWrite(MOTOR_M2, HIGH);
		analogWrite(MOTOR_E2, pwmB);
	}
	else if (pwmB < 0)
	{
		digitalWrite(MOTOR_M2, LOW);
		analogWrite(MOTOR_E2, -pwmA);
	}
	else
	{
		stop_B();
	}
}

void _motor::stop_A()
{
	digitalWrite(MOTOR_E1, LOW);
	digitalWrite(MOTOR_M1, LOW);
}

void _motor::stop_B()
{
	digitalWrite(MOTOR_E2, LOW);
	digitalWrite(MOTOR_M2, LOW);
}

void _motor::stop()
{
	stop_A();
	stop_B();
}

int _motor::getPWM_A()
{
	return pwmA;
}

int _motor::getPWM_B()
{
	return pwmB;
}
#endif


void vMotorTurn(int argc, char *argv[]) 		// Turn the Bot
{
	if(argc != 1) {
		vUsage((char *)"turn <0=NO_TURN, 1=LEFT, 2=RIGHT");
	}
	else {
		uint8_t val = atoi(argv[0]);

		if(!val) {
			eMotorTurn = NO_TURN;
			//motor->setRotationA(NO_TURN);
			//motor->setRotationB(NO_TURN);
			fMotorA_Offset = 0;
			fMotorB_Offset = 0;
		}
		else {
			/*if(val == 1 && motor.uiMotorA_Offset != LEFT) {
				motor.turnStatus = LEFT;
				motor.uiMotorA_Offset += MOTOR_OFFSET;
			}
			else if (motor.uiMotorB_Offset != RIGHT) {
				motor.turnStatus = RIGHT;
				motor.uiMotorB_Offset += MOTOR_OFFSET;
			}*/
			if(val == 1 && fMotorA_Offset != 0)
			{
				eMotorTurn = LEFT;
				fMotorA_Offset = TURN_OFFSET;
				fMotorB_Offset = 0;
			}
			else if(fMotorB_Offset != 0)
			{
				eMotorTurn = RIGHT;
				fMotorA_Offset = 0;
				fMotorB_Offset = MOTOR_OFFSET;
			}

		}
	}
}

void vMotorMove(int argc, char *argv[]) 		// Move the Bot
{
	if(argc != 2) {
		vUsage((char *)"move <0=STOP 1=FORWARD 2=BACKWARD>");
	}
	else {
		uint8_t val = atoi(argv[0]);
//		uint8_t pwm = atoi(argv[1]);

		if(!val) {
			/*motor.moveStatus = STOP;
			motor.uiMotorA_Offset = 0;
			motor.uiMotorB_Offset = 0;*/
			eMotorMove = STOP;
			fMotorA_Offset = 0;
			fMotorB_Offset = 0;
			fMotorA_Move = 0;
			fMotorB_Move = 0;
		}
		else {
			/*if(val == 1 && motor.moveStatus != FORWARD) {
				motor.moveStatus = FORWARD;
				motor.uiMotorA_Offset += MOTOR_OFFSET;
				motor.uiMotorB_Offset += MOTOR_OFFSET;
			}
			else if(val == 2 && motor.moveStatus != BACKWARD) {
				motor.moveStatus = BACKWARD;
				motor.uiMotorA_Offset -= MOTOR_OFFSET;
				motor.uiMotorB_Offset -= MOTOR_OFFSET;
			}*/
			if(val == 1 && eMotorMove != FORWARD)
			{
				eMotorMove = FORWARD;
				fMotorA_Move = MOVE_OFFSET;
				fMotorB_Move = MOVE_OFFSET;
			}
			else if(val == 2 && eMotorMove != BACKWARD)
			{
				eMotorMove = BACKWARD;
				fMotorA_Move = -MOVE_OFFSET;
				fMotorB_Move = -MOVE_OFFSET;
			}
		}
		//motor.move_A(pwm);
		//motor.move_B(pwm);
		motor->setDirectionA(eMotorMove);
		motor->setDirectionB(eMotorMove);
		motor->updateSpeed(fMotorA_Move, fMotorB_Move);
	}
}

//#endif // ARDUINO_AVR_YUN



