/*
 * motor.h
 *
 *  Created on: 11 mai 2018
 *      Author: Ross
 */

#ifndef MBED_ROBO_MOTOR_H_
#define MBED_ROBO_MOTOR_H_

#include "Stspin240.h"

#define MOTOR_OFFSET 	0.1 //30
#define TURN_OFFSET		0.1 //30
#define MOVE_OFFSET 	0.25 //50

/*typedef enum {
	STOP,
	FORWARD,
	BACKWARD
} eMotorMove_t;*/

typedef enum {
	NO_TURN,
	LEFT,
	RIGHT
} eMotorTurn_t;

//extern eMotorTurn_t eMotorTurn;
//extern eMotorMove_t eMotorMove;
eMotorTurn_t eMotorTurn;
//eMotorMove_t eMotorMove;
motor_direction eMotorMove;

/*class _motor {
public:
	eMotorMove_t moveStatus;
	eMotorTurn_t turnStatus;
	//uint8_t uiMotorA_Offset;
	//uint8_t uiMotorB_Offset;
	unsigned char uiMotorA_Offset;
	unsigned char uiMotorB_Offset;
	_motor();
	virtual ~_motor();
	void move_A(int);
	void move_B(int);
	void stop_A();
	void stop_B();
	void stop();
	int getPWM_A();
	int getPWM_B();
private:
	int pwmA;
	int pwmB;
};

extern _motor motor;
*/

//Stspin240 * motor;
float fMotorA_Offset = 0;
float fMotorB_Offset = 0;
float fMotorA_Move = 0;
float fMotorB_Move = 0;

//
//	Function Prototypes
//
void vMotorTurn(int argc, char *argv[]); 		// Turn the Bot
void vMotorMove(int argc, char *argv[]); 		// Move the Bot

#endif /* MBED_ROBO_MOTOR_H_ */
