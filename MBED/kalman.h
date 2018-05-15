/*
 * kalman.h
 *
 *  Created on: 14 mai 2018
 *      Author: Ross
 */

#ifndef MBED_KALMAN_H_
#define MBED_KALMAN_H_

class _kalman {
public:
	_kalman();
	void update(double dt, double angle, double gyro);
};

_kalman *kalman;

#endif /* MBED_KALMAN_H_ */
