/*
 * Stspin240.cpp
 *
 *  Created on: 15 avr. 2018
 *      Author: Ross
 */

#include "Motor.h"
#include "Stspin240.h"

Stspin240 * motor;

Stspin240::Stspin240(PinName EN, PinName REF, PinName RST, PinName PHA, PinName PWMA, PinName PHB, PinName PWMB, unsigned int freq):
	pwmFrequency(freq)
{

	reset = new DigitalOut(RST);
	enable = new DigitalOut(EN);
	ref = new DigitalOut(REF);
	ma = new Motor(EN, PHA, PWMA, pwmFrequency);
	mb = new Motor(EN, PHB, PWMB, pwmFrequency);
	speedA = 0.0;
	speedB = 0.0;
	*reset = 0;
	*enable = 0;
	*ref = 0;
	wait_ms(WAIT_MS);
	debug = false;
}

Stspin240::~Stspin240() {
	if (debug)
	{
		printf("STSPIN240: Entering destructor\n");
	}
	*reset = 0;
	*enable = 0;
	wait_ms(WAIT_MS);
	delete(ma);
	delete(mb);
}

void Stspin240::HiZ()
{
	*enable = 0;
	wait_ms(WAIT_MS);
	if (debug)
	{
		printf("STSPIN240: Disabling motor bridge\n");
	}
}

void Stspin240::setReset()
{
	*reset = 0;
	if(debug)
	{
		printf("STSPIN240: Set Reset\n");
	}
}

void Stspin240::releaseReset()
{
	*reset = 1;
	if(debug)
	{
		printf("STSPIN240: Release Reset\n");
	}
}

void Stspin240::sleep()
{
	*reset = 0;
	if (debug)
	{
		printf("STSPIN240: Entering Motor sleep mode\n");
	}
}

void Stspin240::awake()
{
	*reset = 1;
	wait_ms(WAIT_MS);
	if (debug)
	{
		printf("STSPIN240: Wakeup up motor bridge\n");
	}
}

void Stspin240::enableBridge()
{
	*enable = 1;
	wait_ms(WAIT_MS);
	if (debug)
	{
		printf("STSPIN240: Enable Motor Bridge\n");
	}
}

void Stspin240::disableBridge()
{
	*enable = 0;
	wait_ms(WAIT_MS);
	if (debug)
	{
		printf("STSPIN240: Disable Motor Bridge\n");
	}
}

void Stspin240::enableRef()
{
	*ref = 1;
	if(debug)
	{
		printf("STSPIN240: enable reference\n");
	}
}

void Stspin240::disableRef()
{
	*ref = 0;
	if(debug)
	{
		printf("STSPIN240: disable reference\n");
	}
}

void Stspin240::setDirectionA(motor_direction dir)
//void Stspin240::setDirectionA(eMotorMove_t dir)
{
	if (dir == FORWARD)
	{
		ma->setRotation(CLOCKWISE);
		ma->setDirection();
		//ma->direction = 1;
		if (debug)
		{
			printf("STSPIN240:Set Direction A Motor to CLOCKWISE\n");
		}
	}
	else
	{
		ma->setRotation(ANTICLOCKWISE);
		ma->resetDirection();
		//ma->direction = 0;
		if (debug)
		{
			printf("STSPIN240: Set Direction A Motor to COUNTERCLOCKWISE\n");
		}
	}
}

void Stspin240::setDirectionB(motor_direction dir)
//void Stspin240::setDirectionB(eMotorMove_t dir)
{
	if (dir == FORWARD)
	{
		mb->setRotation(CLOCKWISE);
		mb->setDirection();
		//mb->direction = 1;
		if (debug)
		{
			printf("STSPIN240: Set Direction B Motor to CLOCKWISE\n");
		}
	}
	else
	{
		mb->setRotation(ANTICLOCKWISE);
		mb->resetDirection();
		//mb->direction = 0;
		if (debug)
		{
			printf("STSPIN240: Set Direction B Motor to COUNTERCLOCKWISE\n");
		}
	}
}

void Stspin240::setRotationA(motor_rotation rot)
{
	if (rot == CLOCKWISE)
	{
		ma->setRotation(CLOCKWISE);
		ma->setDirection();
		//ma->direction = 1;
		if (debug)
		{
			printf("STSPIN240: Set Direction A Motor to CLOCKWISE\n");
		}
	}
	else
	{
		ma->setRotation(ANTICLOCKWISE);
		ma->resetDirection();
		//ma->direction = 0;
		if (debug)
		{
			printf("STSPIN240: Set Direction A Motor to COUNTERCLOCKWISE\n");
		}
	}
}

void Stspin240::setRotationB(motor_rotation rot)
{
	if (rot == CLOCKWISE)
	{
		mb->setRotation(CLOCKWISE);
		mb->setDirection();
		//mb->direction = 1;
		if (debug)
		{
			printf("STSPIN240: Set Direction B Motor to CLOCKWISE\n");
		}
	}
	else
	{
		mb->setRotation(ANTICLOCKWISE);
		mb->resetDirection();
		//mb->direction = 0;
		if (debug)
		{
			printf("STSPIN240: Set Direction B Motor to COUNTERCLOCKWISE\n");
		}
	}
}

motor_direction Stspin240::getDirectionA()
{
	if (ma->getDirection() == 1)
	{
		if (debug)
		{
			printf("STSPIN240: Return direction A Motor to FORWARD\n");
		}
		return FORWARD;
	}
	else
	{
		if (debug)
		{
			printf("STSPIN240: Return direction A Motor to BACKWARD\n");
		}
		return BACKWARD;
	}
}

motor_direction Stspin240::getDirectionB()
{
	if (mb->getDirection() == 1)
	{
		if (debug)
		{
			printf("STSPIN240: Return direction B Motor to FORWARD\n");
		}
		return FORWARD;
	}
	else
	{
		if (debug)
		{
			printf("STSPIN240: Return direction B Motor to BACKWARD\n");
		}
		return BACKWARD;
	}
}

motor_rotation Stspin240::getRotationA()
{
	if (ma->getDirection() == 1)
	{
		if (debug)
		{
			printf("STSPIN240: Return direction A Motor to CLOCKWISE\n");
		}
		return CLOCKWISE;
	}
	else
	{
		if (debug)
		{
			printf("STSPIN240: Return direction A Motor to COUNTERCLOCKWISE\n");
		}
		return ANTICLOCKWISE;
	}
}

motor_rotation Stspin240::getRotationB()
{
	if(mb->getDirection() == 1)
	{
		if (debug)
		{
			printf("STSPIN240: Return direction B Motor to CLOCKWISE\n");
		}
		return CLOCKWISE;
	}
	else
	{
		if (debug)
		{
			printf("STSPIN240: Return direction B Motor to COUNTERCLOCKWISE\n");
		}
		return ANTICLOCKWISE;
	}
}

void Stspin240::setPwmFrequency(unsigned int freq)
{
	ma->setPwmFrequency(freq);
	mb->setPwmFrequency(freq);
	if (debug)
	{
		printf("STSPIN240: Set PWM frequency to %d Hz\n", freq);
	}
}

unsigned int Stspin240::getPwmFrequency()
{
	if (ma->getPwmFrequency() == mb->getPwmFrequency())
	{
		uint16_t foo = ma->getPwmFrequency();
		if (debug)
		{
			printf("STSPIN240: PWM frequency = %d Hz\n", foo);
		}
		return foo;
	}
	else
	{
		return 0;
	}
}

void Stspin240::updateSpeed(float sA, float sB)
{
	ma->setPwm(sA);
	mb->setPwm(sB);
	speedA = sA;
	speedB = sB;
	if (debug)
	{
		printf("STSPIN240: Update Motors speed to A=%f B=%f \n", speedA, speedB);
	}
}

void Stspin240::updateSpeedA(float speed)
{
	ma->setPwm(speed);
	speedA = speed;
	if (debug)
	{
		printf("STSPIN240: Update A Motor speed to A=%f\n", speedA);
	}
}

void Stspin240::updateSpeedB(float speed)
{
	mb->setPwm(speed);
	speedB = speed;
	if (debug)
	{
		printf("STSPIN240: Update B Motor speed to B=%f \n", speedB);
	}
}

void Stspin240::stop()
{
	ma->stop();
	mb->stop();
	if (debug)
	{
		printf("STSPIN240: Stop Motors\n");
	}
}

void Stspin240::run()
{
	updateSpeed(speedA, speedB);
}

void Stspin240::setDebug()
{
	debug = true;
	printf("STSPIN240: Starting debug mode\n");
}

void Stspin240::resetDebug()
{
	printf("STSPIN240: Ending debug mode\n");
	debug = false;
}
