/*
 * kalman.h
 *
 *  Created on: 14 mai 2018
 *      Author: Ross
 */

#ifndef MBED_KALMAN_H_
#define MBED_KALMAN_H_

#include "mbed.h"

class _kalman {
private:
	uint8_t N;
	double Q_angle;
	double Q_gyro;
	double R_angle;
	double X[2];
	double K[2];
    double P[2][2];
    double Q[2][2];
    double rate;
    double angle_err;
    double C0;
public:
	double angle;
	double q_bias;
	_kalman();
	void init();
	void init(double p[], double q[]);
	double * kalmanUpdate(double dt, double angle, double gyro);
	double * getState();
	double getAngle();
	double getGyroBias();
	double * getKalmanState();
	void stateUpdate(double q_m, double dt);
};

extern _kalman *kalman;

#endif /* MBED_KALMAN_H_ */
