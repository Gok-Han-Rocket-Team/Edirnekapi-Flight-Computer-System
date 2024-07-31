/*
 * algorithms.c
 *
 *  Created on: May 10, 2024
 *      Author: yahya
 */
#include "algorithms.h"
#include <string.h>
#include <stdio.h>
#include <math.h>


extern UART_HandleTypeDef huart1;

float currentTime_1 = 0.0;
float currentTime_2 = 0.0;
float lastTime_1 = 0.0;
float lastTime_2 = 0.0;
float lastAltitude_1 = 0.0;
float lastAltitude_2 = 0.0;

int risingCounter = 0;
int fallingCounter = 0;

uint8_t isFalling = 0;
uint8_t isFalling_2 = 0;
uint8_t isRising = 0;
uint8_t isRising_2 = 0;
uint8_t isUpdated_1 = 0;
uint8_t isUpdated_2 = 0;

uint8_t buffer_alg[100];

static double sqr(double nmbr)
{
	return pow(nmbr, 2);
}


/*
 * it works only with BME280 pressure sensor. Measures the vertical velocity.
 * it detects the first deploy
 * it detecets the second deploy via altitude
 */
void algorithm_1_update(BME_280_t* BME, algorithmStatus* stat)
{

	//velocity measuiring
	currentTime_1 = (float)HAL_GetTick() / 1000.0;
  if(fabs(currentTime_1 - lastTime_1) > 0.05)
  {
	  float currentAltitude = BME->altitude;
	  BME->velocity = (currentAltitude - lastAltitude_1) / (currentTime_1 - lastTime_1);
	  lastAltitude_1 = currentAltitude;
	  lastTime_1 = currentTime_1;
	  isUpdated_1 = 1;
  }

  if(isUpdated_1)
  {

	  isUpdated_1 = 0;

	  //rising detection
	  if(BME->velocity > RISING_VELOCITY_TRESHOLD)
	  {
		  risingCounter++;
	  }
	  else
	  {
		  risingCounter = 0;
	  }

	  if(risingCounter == 3 && isRising == 0 && isFalling == 0 )
	  {
		  isRising = 1;
	  }

	  //falling detection
	  if(BME->velocity < FALLING_VELOCITY_TRESHOLD)
	  {
		  fallingCounter++;
	  }
	  else
	  {
		  fallingCounter = 0;
	  }

	  if(fallingCounter == 3 && isRising == 1 && isFalling == 0 && BME->altitude > FIRST_DEPLOY_ARM_ALT)
	  {
		  isFalling = 1;
		  stat[0] = 1;
	  }

	  //second parachute
	  if(isFalling == 1 && BME->altitude < SECOND_DEPLOY_ALTITUDE && stat[1] == 0)
	  {
		  stat[1] = 1;
	  }
  }


}


void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI, float angle)
{
	//Rising detection
	if((sqrtf(sqr(BMI->acc_x) + sqr(BMI->acc_y) + sqr(BMI->acc_z)) > RISING_G_TRESHOLD) && isRising_2 == 0)
	{
		if(BME->altitude < 200.0 && BME->altitude > -200.0){
			BME->baseAltitude = BME->altitude + BME->baseAltitude;
		}

		isRising_2 = 1;
		rocketStatus = rocketStatus < STAT_FLIGHT_STARTED ? STAT_FLIGHT_STARTED : rocketStatus;
	}

	//falling detection
	if((fabs(angle) > 85) && isRising_2 == 1 && isFalling_2 == 0 && BME->altitude > ARMING_ALTITUDE)
	{
		isFalling_2 = 1;
		rocketStatus = rocketStatus < STAT_P1_OK_P2_NO ? STAT_P1_OK_P2_NO : rocketStatus;
	}

	//Second Parachute
	static int secondP_counter = 0;
	if(BME->altitude < SECOND_DEPLOY_ALTITUDE && isFalling_2 == 1 && rocketStatus < STAT_P1_OK_P2_OK)
		secondP_counter++;

	if(secondP_counter > 10)
	{
		rocketStatus = rocketStatus < STAT_P1_OK_P2_OK ? STAT_P1_OK_P2_OK : rocketStatus;
	}


}


