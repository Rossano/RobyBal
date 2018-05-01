/*
 * board.h
 *
 *  Created on: 29 avr. 2018
 *      Author: Ross
 */

#ifndef MBED_BOARD_H_
#define MBED_BOARD_H_

///
/// Motor Define
///
#define MOTOR_EN		(D2)
#define MOTOR_RST		(D9)
#define MOTOR_REF		(A0)
#define MOTOR_PHA		(D6)
#define MOTOR_PWMA		(D5)
#define MOTOR_PHB		(D7)
#define MOTOR_PWMB		(D4)
#define MOTOR_FREQ		(1000)

///
///	MEMS
///
#define MEMS_I2C_SCL	(D15)
#define MEMS_I2C_SDA	(D14)
#define	MEMS_INT1		(D0)
#define MEMS_INT2		(D1)


///
/// Misc
///
#define SOL_LED			(13)

#define BAUDRATE		(115200)

#endif /* MBED_BOARD_H_ */
