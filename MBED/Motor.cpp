/*
 * Motor.cpp
 *
 *  Created on: 14 avr. 2018
 *      Author: Ross
 */

#include "mbed.h"
#include "Motor.h"

Motor::Motor(PinName EN, PinName DIR, PinName PWM, uint16_t FREQ)
{
	enable = new DigitalOut(EN);
	direction = new DigitalOut(DIR);
	pwm = new PwmOut(PWM);
	active = false;
	rotationDir = CLOCKWISE;
	debug = false;
	if(FREQ != 0)
	{
		frequency = FREQ;
	}
}

void Motor::enableBridge()
{
	active = true;
	*enable = 1;
	wait_ms(1);
	if(debug)
	{
		printf("Enable bridge\n");
	}
}

void Motor::disableBridge()
{
	active = false;
	*enable = 0;
	wait_ms(1);
	if(debug)
	{
		printf("Disable bridge\n");
	}
}

void Motor::stop()
{
	*pwm = 0;
	if (debug)
	{
		printf("Stop motor\n");
	}
}

void Motor::run()
{
	enableBridge();
	if (debug)
	{
		printf("Run the motor\n");
	}
}

void Motor::HiZ()
{
	disableBridge();
	if (debug)
	{
		printf("Bridge HiZ\n");
	}
}

void Motor::setPwmFrequency(uint16_t FREQ)
{
	float period_ms = 1 / FREQ;
	pwm->period(period_ms);
	if (debug)
	{
		printf("Set PWM period to %f\n", period_ms);
	}
}

void Motor::setRotation(motor_rotation rot)
{
	rotationDir = rot;
	if(rot == CLOCKWISE)
	{
		*direction = 1;
		if (debug)
		{
			printf("Set rotation to counterclockwise\n");
		}
	}
	else
	{
		*direction = 0;
		if (debug)
		{
			printf("Set rotation to clockwise\n");
		}
	}
}

motor_rotation Motor::getRotation()
{
	if (debug)
	{
		if(rotationDir == CLOCKWISE) printf("Rotation = 0\n");
		else printf("Rotation = 1\n");
	}
	return rotationDir;
}

void Motor::setDirection()
{
	*direction = 1;
	if(debug)
	{
		printf("Set direction to 1\n");
	}
}

void Motor::resetDirection()
{
	*direction = 0;
	if (debug)
	{
		printf("Set direction to 0\n");
	}
}

bool Motor::getDirection()
{
	if(*direction == 1)
	{
		if (debug)
		{
			printf("Direction = 1\n");
		}
		return true;
	}
	else
	{
		if (debug)
		{
			printf("Direction = 0\n");
		}
		return false;
	}
}

uint16_t Motor::getPwmFrequency()
{
	if (debug)
	{
		printf("PWM frequency = %d\n", frequency);
	}
	return frequency;
}

float Motor::getPwmValue()
{
	if (debug)
	{
		printf("PWM value = %f\n", pwm->read());
	}
	return pwm->read();
}

Motor& Motor::operator=(Motor &m)
{
	return m;
}

Motor Motor::operator=(float PWM)
{
	if(PWM < 0)
	{
		rotationDir = ANTICLOCKWISE;
		if (*direction == 1) *direction = 0;
		else *direction = 1;
		pwm->pulsewidth(-PWM);
	}
	else {
		rotationDir = CLOCKWISE;
		pwm->pulsewidth(PWM);
	}
	return *this;
}

void Motor::setPwm(float PWM)
{
	if (debug)
	{
		printf("Set PWM value to %f\n", PWM);
	}
	*pwm = PWM;
}

void Motor::setDebug()
{
	printf("Set Debug ON\n");
	debug = true;
}

void Motor::resetDebug()
{
	printf("Set Debug OFF\n");
	debug = false;
}
