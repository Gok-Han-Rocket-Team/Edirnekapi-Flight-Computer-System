/*
 * algorithms.h
 *
 *  Created on: May 10, 2024
 *      Author: yahya
 */

#ifndef INC_ALGORITHMS_H_
#define INC_ALGORITHMS_H_

#include <stdint.h>
#include "bmi088.h"
#include "bme280.h"
#include "externalPins.h"
#include "configuration.h"


extern ext_pin_s led;
extern ext_pin_s buzzer;
extern uint8_t rocketStatus;

enum flightStates{
	STAT_ROCKET_READY	=	(uint8_t)0x01,
	STAT_FLIGHT_STARTED	=	(uint8_t)0x02,
	STAT_MOTOR_BURNOUT	=	(uint8_t)0x03,
	STAT_P1_OK_P2_NO	=	(uint8_t)0x04,
	STAT_P1_OK_P2_OK	=	(uint8_t)0x05,
	STAT_TOUCH_DOWN		=	(uint8_t)0x06,
	STAT_P1_NO_P2_OK	=	(uint8_t)0x07,
};

typedef uint8_t algorithmStatus;

void algorithm_1_update(BME_280_t* BME);
void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI);

#endif /* INC_ALGORITHMS_H_ */
