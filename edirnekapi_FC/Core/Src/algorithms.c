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
#include "main.h"
#include <stdint.h>
#include "reset_detect.h"
#include "queternion.h"

extern UART_HandleTypeDef huart1;
extern backup_sram_datas_s *saved_datas;
extern int is_BME_ok;

float currentTime_1 = 0.0;
float currentTime_2 = 0.0;
float lastTime_1 = 0.0;
float lastTime_2 = 0.0;
float lastAltitude_1 = 0.0;
float lastAltitude_2 = 0.0;

int risingCounter = 0;
int fallingCounter = 0;
static int is_quaternion_zeroed = 0;
static int TD_counter = 0;
static int secondP_counter = 0;

uint32_t algorithm_1_start_time_u32 = 0;
uint32_t algorithm_2_start_time_u32 = 0;

uint8_t isFalling = 0;
uint8_t isFalling_2 = 0;
uint8_t isRising = 0;
uint8_t isRising_2 = 0;
uint8_t isUpdated_1 = 0;
uint8_t isUpdated_2 = 0;
uint8_t is_secondP_OK = 0;
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
void algorithm_1_update(BME_280_t* BME)
{

	//velocity measuiring
	currentTime_1 = (float)HAL_GetTick() / 1000.0;
  if(fabs(currentTime_1 - lastTime_1) > 0.1)
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
	if(BME->velocity > RISING_VELOCITY_TRESHOLD && isRising == 0)
	{
	  risingCounter++;
	}
	else
	{
	  risingCounter = 0;
	}

	if(risingCounter == 1 && isRising == 0 && isFalling == 0 )
	{
	  isRising = 1;
	  algorithm_1_start_time_u32 = HAL_GetTick();
	  saved_datas->r_status = saved_datas->r_status < STAT_FLIGHT_STARTED ? STAT_FLIGHT_STARTED : saved_datas->r_status;
	  ext_pin_open(&buzzer);
	}

	//Falling detection || First parachute
	if(BME->velocity < FALLING_VELOCITY_TRESHOLD && HAL_GetTick() - algorithm_1_start_time_u32 > ALGORITHM_1_LOCKOUT_TIME)
	{
	  fallingCounter++;
	}
	else
	{
	  fallingCounter = 0;
	}

	if(fallingCounter == 1 && isRising == 1 && isFalling == 0 && BME->altitude > ARMING_ALTITUDE_1)
	{
	  isFalling = 1;
	  saved_datas->r_status = saved_datas->r_status < STAT_P1_OK_P2_NO ? STAT_P1_OK_P2_NO : saved_datas->r_status;
	  deploy_p_1();
	}

	//Second Parachute
	static int second_p_counter_1 = 0;
	static uint8_t is_second_p_OK_1 = 0;

	if(BME->altitude < SECOND_DEPLOY_ALTITUDE && isFalling == 1 && is_second_p_OK_1 == 0)
	{
		second_p_counter_1++;
	}
	else{
		second_p_counter_1 = 0;
	}
	if(second_p_counter_1 == 10)
	{
		saved_datas->r_status = saved_datas->r_status < STAT_P1_OK_P2_OK ? STAT_P1_OK_P2_OK : saved_datas->r_status;
		is_second_p_OK_1 = 1;
		deploy_p_2();
	}
  }
}

void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI)
{
	//Rising detection
	if((sqrtf(sqr(BMI->acc_x) + sqr(BMI->acc_y) + sqr(BMI->acc_z)) > RISING_G_TRESHOLD) && isRising_2 == 0)
	{
		if(BME->altitude < 200.0 && BME->altitude > -200.0){
			saved_datas->base_altitude = BME->altitude + saved_datas->base_altitude;
		}

		isRising_2 = 1;
		algorithm_2_start_time_u32 = HAL_GetTick();
		saved_datas->r_status = saved_datas->r_status < STAT_FLIGHT_STARTED ? STAT_FLIGHT_STARTED : saved_datas->r_status;
		ext_pin_open(&buzzer);
	}

	//Burnout detection
	static int burnout_counter = 0;
	if(BMI->acc_y < BURNOUT_THRESHOLD && isRising_2 == 1 && burnout_counter < 12)
	{
		burnout_counter++;
	}
	if(burnout_counter == 10)
	{
		saved_datas->r_status = saved_datas->r_status < STAT_MOTOR_BURNOUT ? STAT_MOTOR_BURNOUT : saved_datas->r_status;
		ext_pin_open(&buzzer);
	}

	//quaternion setting to zero
	if((HAL_GetTick() - algorithm_2_start_time_u32) > QUATERNION_ZERO_TIME && is_quaternion_zeroed == 0 && isRising_2 == 1)
	{
	  quaternionSet_zero();
	  is_quaternion_zeroed = 1;
	  ext_pin_open(&buzzer);
	}

	//Falling detection || First parachute
	if(BMI->angle > ANGLE_THRESHOLD && isRising_2 == 1 && isFalling_2 == 0 && HAL_GetTick() - algorithm_2_start_time_u32 > ALGORITHM_2_LOCKOUT_TIME)
	{
		isFalling_2 = 1;
		saved_datas->r_status = saved_datas->r_status < STAT_P1_OK_P2_NO ? STAT_P1_OK_P2_NO : saved_datas->r_status;
		deploy_p_1();
	}

	if(is_BME_ok == 1)
	{
		//Second Parachute
		if(BME->altitude < SECOND_DEPLOY_ALTITUDE && isFalling_2 == 1 && is_secondP_OK == 0)
		{
			secondP_counter++;
		}
		else{
			secondP_counter = 0;
		}
		if(secondP_counter == 10)
		{
			saved_datas->r_status = saved_datas->r_status < STAT_P1_OK_P2_OK ? STAT_P1_OK_P2_OK : saved_datas->r_status;
			is_secondP_OK = 1;
			deploy_p_2();
		}
	}
	//Touchdown Detection
	static uint8_t is_TD = 0;
	if(sqrt(sqr(BMI->gyro_x) + sqr(BMI->gyro_y) + sqr(BMI->gyro_z)) < 10.0 && isFalling_2 == 1 && is_secondP_OK == 1 && is_TD == 0)
	{
			TD_counter++;
	}
	else{
		TD_counter = 0;
	}
	if(TD_counter > 1000)
	{
		is_TD = 1;
		saved_datas->r_status = saved_datas->r_status < STAT_TOUCH_DOWN ? STAT_TOUCH_DOWN : saved_datas->r_status;
		ext_pin_open(&buzzer);
	}
}
