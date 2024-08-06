/*
 * algorithms.h
 *
 *  Created on: May 10, 2024
 *      Author: yahya
 */

#ifndef INC_ALGORITHMS_H_
#define INC_ALGORITHMS_H_
#include "bmi088.h"
#include "bme280.h"
#include <stdint.h>

extern uint8_t rocketStatus;

enum flightStates{
	STAT_ROCKET_READY	=	(uint8_t)0x01,
	STAT_FLIGHT_STARTED	=	(uint8_t)0x02,
	STAT_MOTOR_BURNOUT	=	(uint8_t)0x03,
	STAT_P1_OK_P2_NO	=	(uint8_t)0x04,
	STAT_P1_OK_P2_OK	=	(uint8_t)0x05,
	STAT_P1_NO_P2_OK	=	(uint8_t)0x06,
	STAT_TOUCH_DOWN		=	(uint8_t)0x07,
};



#define RISING_VELOCITY_TRESHOLD	(float)30.0			//ms/sn
#define FALLING_VELOCITY_TRESHOLD	(float)1.0			//m/sn

#define SECOND_DEPLOY_ALTITUDE 		(float)2.0		//meters				500.0
#define FIRST_DEPLOY_ARM_ALT 		(float)500.0		//meters

#define RISING_G_TRESHOLD 			(float)3000.0		//mG
#define ARMING_ALTITUDE				(float)-5.0		//m				500.0
#define BURNOUT_THRESHOLD			(float)0.0		//mG			3000.0
#define ANGLE_THRESHOLD				(float)80.0			//degree
typedef uint8_t algorithmStatus;

void algorithm_1_update(BME_280_t* BME, algorithmStatus* stat);
void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI, float angle);

#endif /* INC_ALGORITHMS_H_ */
