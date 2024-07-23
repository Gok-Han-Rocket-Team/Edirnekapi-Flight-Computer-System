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


#define SECOND_DEPLOY_ALTITUDE 		(float)500.0		//meters
#define FIRST_DEPLOY_ARM_ALT 		(float)500.0		//meters
#define RISING_G_TRESHOLD 			(float)3000.0		//mG
#define RISING_VELOCITY_TRESHOLD	(float)30.0			//ms/sn
#define FALLING_VELOCITY_TRESHOLD	(float)1.0			//m/sn

typedef uint8_t algorithmStatus;

void algorithm_1_update(BME_280_t* BME, algorithmStatus* stat);
void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI, algorithmStatus* stat);

#endif /* INC_ALGORITHMS_H_ */
