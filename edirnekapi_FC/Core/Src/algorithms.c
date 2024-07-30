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


//it works only with BME280 pressure sensor. Measures the vertical velocity.
//it detects the first deplo
//it detecets the second deploy via altitude
void algorithm_1_update(BME_280_t* BME, algorithmStatus* stat)
{

	//velocity measuiring
	currentTime_1 = (float)HAL_GetTick() / 1000.0;
  if(fabs(currentTime_1 - lastTime_1) > 0.05)
  {
	  float currentAltitude = BME->altitude;
	  BME->velocity = (currentAltitude - lastAltitude_1) / (currentTime_1 - lastTime_1);
	  //sprintf((char*)buffer_alg, "dikey hiz: %.0f\tirtifa: %.0f\r\n", BME->velocity, BME->altitude);
	  //sprintf((char*)buffer_alg, "hello\r\n");
	  //HAL_UART_Transmit(&huart1, buffer_alg, strlen((char*)buffer_alg), 50);
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
		  /*
		  HAL_GPIO_WritePin(Ld3_GPIO_Port, Ld3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Ld10_GPIO_Port, Ld10_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Ld6_GPIO_Port, Ld6_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld7_GPIO_Port, Ld7_Pin, GPIO_PIN_SET);
		  */
		  sprintf((char*)buffer_alg, "\n\n\n\n***********   RISING   ***********\n\n\n\n\r\n");
		  HAL_UART_Transmit(&huart1, buffer_alg, strlen((char*)buffer_alg), 50);
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
		  /*
		  HAL_GPIO_WritePin(Ld3_GPIO_Port, Ld3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld10_GPIO_Port, Ld10_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld6_GPIO_Port, Ld6_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Ld7_GPIO_Port, Ld7_Pin, GPIO_PIN_RESET);
		  */
		  sprintf((char*)buffer_alg, "\n\n\n\n***********   FALLING, FIRST PARACHUTE   ***********\n\n\n\n\n\r\n");
		  HAL_UART_Transmit(&huart1, buffer_alg, strlen((char*)buffer_alg), 50);
	  }

	  //second parachute
	  if(isFalling == 1 && BME->altitude < SECOND_DEPLOY_ALTITUDE && stat[1] == 0)
	  {
		  stat[1] = 1;
		  /*
		  HAL_GPIO_WritePin(Ld3_GPIO_Port, Ld3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld4_GPIO_Port, Ld4_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld5_GPIO_Port, Ld5_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld6_GPIO_Port, Ld6_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld7_GPIO_Port, Ld7_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld8_GPIO_Port, Ld8_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld9_GPIO_Port, Ld9_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld10_GPIO_Port, Ld10_Pin, GPIO_PIN_SET);
		  */
		  sprintf((char*)buffer_alg, "\n\n\n\n***********   SECOND PARACHUTE   ***********\n\n\n\n\r\n");
		  HAL_UART_Transmit(&huart1, buffer_alg, strlen((char*)buffer_alg), 50);
	  }
  }


}
void algorithm_2_update(BME_280_t* BME, bmi088_struct_t* BMI, algorithmStatus* stat)
{

	uint8_t buf[100];
	currentTime_2 = (float)HAL_GetTick() / 1000.0;

	  float teta = atan(sqrt(BMI->acc_x * BMI->acc_x + BMI->acc_y * BMI->acc_y) / BMI->acc_z) * 180.0 / M_PI;
	  if( teta < 0)
	  {
		teta = teta + 180.0 ;
	  }

	if(fabs(currentTime_2 - lastTime_2) > 0.1)
	{
		sprintf((char*)buf, "irtifa: %0.f ivme: %.0f  aci: %.0f\r\n", BME->altitude, sqrtf(sqr(BMI->acc_x) + sqr(BMI->acc_y) + sqr(BMI->acc_z)), teta);
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 50);
		lastTime_2 = currentTime_2;
	}

	//Rising detection
	if((sqrtf(sqr(BMI->acc_x) + sqr(BMI->acc_y) + sqr(BMI->acc_z)) > RISING_G_TRESHOLD) && isRising_2 == 0)
	{
		stat[0] = 1;
		isRising_2 = 1;
		sprintf((char*)buf, "\n\n\n\n***********   RISING   ***********\n\n\n\n\r\n");
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 50);
		/*
		  HAL_GPIO_WritePin(Ld3_GPIO_Port, Ld3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Ld10_GPIO_Port, Ld10_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Ld6_GPIO_Port, Ld6_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Ld7_GPIO_Port, Ld7_Pin, GPIO_PIN_SET);
		  */
	}

	//falling detction
	if(isRising_2 == 1 && (teta > 70) && isFalling_2 == 0 && BME->altitude > 500)
	{
		stat[0] = 1;
		isFalling_2 = 1;
		sprintf((char*)buf, "\n\n\n\n***********   FALLING, FIRST PARACHUTE   ***********\n\n\n\n\n\r\n");
		HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 50);
		/*
		HAL_GPIO_WritePin(Ld3_GPIO_Port, Ld3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Ld10_GPIO_Port, Ld10_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Ld6_GPIO_Port, Ld6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ld7_GPIO_Port, Ld7_Pin, GPIO_PIN_RESET);
		*/
	}

	//second parachute
	if(isFalling_2 == 1 && BME->altitude < SECOND_DEPLOY_ALTITUDE && stat[1] == 0)
	{
	  stat[1] = 1;
	  /*
	  HAL_GPIO_WritePin(Ld3_GPIO_Port, Ld3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld4_GPIO_Port, Ld4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld5_GPIO_Port, Ld5_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld6_GPIO_Port, Ld6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld7_GPIO_Port, Ld7_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld8_GPIO_Port, Ld8_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld9_GPIO_Port, Ld9_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Ld10_GPIO_Port, Ld10_Pin, GPIO_PIN_SET);
	  */
	  sprintf((char*)buf, "\n\n\n\n***********   SECOND PARACHUTE   ***********\n\n\n\n\r\n");
	  HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 50);
	}
}


