/*
 * bme280.c
 *
 *  Created on: May 1, 2024
 *      Author: yahya
 */

#include "bme280.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define filterSize 100

extern UART_HandleTypeDef huart1;

static I2C_HandleTypeDef* 	I2C_;
static BME_280_t*			BME;

float array[filterSize];


int currentIndex = 0;

static void bme280_getVals()
{
	uint8_t params[8];
	HAL_StatusTypeDef retVal = HAL_I2C_Mem_Read(I2C_, BME280_ADD, BME280_STATUS, I2C_MEMADD_SIZE_8BIT, params, 1, 100);
	BME->isUpdated = 0;
	if((params[0] & (0x01 << 3)) == (0x01 << 3))
	{
		BME->isUpdated = 1;
		retVal = HAL_I2C_Mem_Read(I2C_, BME280_ADD, BME280_P_MSB_ADD, I2C_MEMADD_SIZE_8BIT, params, 8, 200);
		BME->adcVals.ut = 	((int32_t)params[3] << 12) | ((int32_t)params[4] << 4) | ((int32_t)params[5]  >> 4);
		BME->adcVals.up =	((int32_t)params[0] << 12) | ((int32_t)params[1] << 4) | ((int32_t)params[2]  >> 4);
		BME->adcVals.uh =	((int32_t)params[6] << 8) | ((int32_t)params[7]);
	}
	UNUSED(retVal);
}

static void bme280_get_altitude()
{
	float p_seaLevel = 1013.25;		//hPa
	float alt = 44330.0 * (1.0 - pow((BME->pressure / p_seaLevel), (1.0 / 5.255)));
	BME->altitude = alt - BME->baseAltitude;
	if(BME->altitude > BME->maxAltitude && BME->baseAltitude != 0.0)
	{
		BME->maxAltitude = BME->altitude;
	}
}

HAL_StatusTypeDef bme280_init(BME_280_t* BME_sensor,  I2C_HandleTypeDef* I2C_bme, uint8_t mode, uint8_t OS, uint8_t filter){
	BME = BME_sensor;
	I2C_ = I2C_bme;
	BME->baseAltitude = 0.0;

	uint8_t params[25];

	HAL_StatusTypeDef retVal;

	uint8_t resetData = BME280_SOFT_RESET;
	retVal = HAL_I2C_Mem_Write(I2C_bme, BME280_ADD, BME280_RESET, I2C_MEMADD_SIZE_8BIT, &resetData, 1, 50);		//Soft Reset.
	HAL_Delay(50);
	HAL_I2C_DeInit(I2C_bme);
	HAL_Delay(5);
	HAL_I2C_Init(I2C_bme);
	HAL_Delay(5);

	retVal = HAL_I2C_Mem_Read(I2C_bme, BME280_ADD, BME280_PARAM1_START, I2C_MEMADD_SIZE_8BIT, params, 25, 200);
	BME->parameters.dig_T1 = params[0] | (uint16_t)(params[1] << 8);
	BME->parameters.dig_T2 = params[2] | ((int16_t)params[3] << 8);
	BME->parameters.dig_T3 = params[4] | ((int16_t)params[5] << 8);
	BME->parameters.dig_P1 = params[6] | ((uint16_t)params[7] << 8);
	BME->parameters.dig_P2 = params[8] | ((int16_t)params[9] << 8);
	BME->parameters.dig_P3 = params[10] | ((int16_t)params[11] << 8);
	BME->parameters.dig_P4 = params[12] | ((int16_t)params[13] << 8);
	BME->parameters.dig_P5 = params[14] | ((int16_t)params[15] << 8);
	BME->parameters.dig_P6 = params[16] | ((int16_t)params[17] << 8);
	BME->parameters.dig_P7 = params[18] | ((int16_t)params[19] << 8);
	BME->parameters.dig_P8 = params[20] | ((int16_t)params[21] << 8);
	BME->parameters.dig_P9 = params[22] | ((int16_t)params[23] << 8);
	BME->parameters.dig_H1 = params[24];

	retVal = HAL_I2C_Mem_Read(I2C_bme, BME280_ADD, BME280_PARAM2_START, I2C_MEMADD_SIZE_8BIT, params, 7, 50);
	BME->parameters.dig_H2 = params[0] | ((int16_t)params[1] << 8);
	BME->parameters.dig_H3	= params[2];
	BME->parameters.dig_H4 = (params[4] & 0xF) | ((int16_t)params[3] << 4);
	BME->parameters.dig_H5 = ((params[4] & 0xF0) >> 4) | ((int16_t)params[5] << 4);
	BME->parameters.dig_H6 = params[6];

	uint8_t data_ctrl = 0;
	data_ctrl = OS;
	retVal = HAL_I2C_Mem_Write(I2C_bme, BME280_ADD, BME280_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &data_ctrl, 1, 50);		//Humidity sensor over sampling set to OS.
	data_ctrl = 0;
	data_ctrl = mode | (OS << 2) | (OS << 5);																		//Mode has been chosed.
	retVal = HAL_I2C_Mem_Write(I2C_bme, BME280_ADD, BME280_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &data_ctrl, 1, 50);		//Temp and pressure sensors' over sampling set to OS.
	data_ctrl = 0;
	data_ctrl = (filter << 2);
	retVal = HAL_I2C_Mem_Write(I2C_bme, BME280_ADD, BME280_CONFIG, I2C_MEMADD_SIZE_8BIT, &data_ctrl, 1, 50);

	//uint8_t buf[50];

	float base = 0.0;
	HAL_Delay(200);
	for(int i = 0; i < 50; i++)		//Taking base altitude
	{
	  bme280_update();
	  base +=  BME->altitude;
	  HAL_Delay(10);
	}
	BME->baseAltitude = (base / 50.0);
	return retVal;
}


void bme280_update(){
	int32_t var1_t, var2_t, T, adc_T;
	bme280_getVals();

	if(BME->isUpdated == 1)
	{
		//For tempereature
		adc_T =	BME->adcVals.ut;
		var1_t = ((((adc_T >> 3 ) - ((int32_t)BME->parameters.dig_T1 << 1))) * ((int32_t)BME->parameters.dig_T2)) >> 11;
		var2_t = (((((adc_T >> 4) - ((int32_t)BME->parameters.dig_T1)) * ((adc_T >> 4) - ((int32_t)BME->parameters.dig_T1))) >> 12) * ((int32_t)BME->parameters.dig_T3)) >> 14;
		int32_t t_fine = var1_t + var2_t;
		T = (t_fine * 5 + 128) >> 8;
		BME->temperature = (float)T / 100.0;

		//For pressure
		int64_t var1_p, var2_p, P, adc_P;
		adc_P = (int64_t)BME->adcVals.up;
		var1_p = ((int64_t)t_fine) - 128000;
		var2_p = var1_p * var1_p * (int64_t)BME->parameters.dig_P6;
		var2_p = var2_p + ((var1_p *(int64_t)BME->parameters.dig_P5) <<17);
		var2_p = var2_p + (((int64_t)BME->parameters.dig_P4) << 35);
		var1_p = ((var1_p * var1_p * (int64_t)BME->parameters.dig_P3) >> 8) + ((var1_p * (int64_t)BME->parameters.dig_P2) << 12);
		var1_p = (((((int64_t)1) <<47 ) + var1_p)) * ((int64_t) BME->parameters.dig_P1) >> 33;
		if(var1_p == 0)
		{
			P = 0;
		}else
		{
		P = 1048576 - adc_P;
		P = (((P << 31) - var2_p) * 3125) / var1_p;
		var1_p = (((int64_t) BME->parameters.dig_P9) * (P >> 13) * (P >> 13)) >> 25;
		var2_p = (((int64_t) BME->parameters.dig_P8) * P) >> 19;
		P = (( P + var1_p + var2_p) >> 8) + (((int64_t)BME->parameters.dig_P7) << 4);
		}

		BME->pressure = ((float)P / 256.0 / 100.0);

		//for humidity
		uint32_t var_h, adc_H;
		adc_H = BME->adcVals.uh;

		var_h = (t_fine - ((int32_t)76800));
		var_h = (((((adc_H << 14) - (((int32_t)BME->parameters.dig_H4) << 20) - (((int32_t)BME->parameters.dig_H5) * var_h)) + ((int32_t)16384)) >> 15) * (((((((var_h *((int32_t)BME->parameters.dig_H6)) >> 10) * (((var_h * ((int32_t)BME->parameters.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)BME->parameters.dig_H2) + 8192) >> 14));
		var_h = (var_h - (((((var_h >> 15) * (var_h >> 15)) >> 7) * ((int32_t)BME->parameters.dig_H1)) >> 4));
		var_h = (var_h < 0 ? 0 : var_h);
		var_h = (var_h > 419430400 ? 419430400 : var_h);
		BME->humidity = ((float)(var_h >> 12)) / 1024.0;

		//get altitude
		bme280_get_altitude();
		BME->isUpdated = 0;
	}

}

float moving_average(float newVal)
{
	float sum = 0.0;
	if (currentIndex == filterSize - 1)
		currentIndex = 0;
	array[currentIndex] = newVal;
	for(int i = 0; i < filterSize; i++)
	{
		sum += array[i];
	}
	sum /= (float)filterSize;
	currentIndex++;
	return sum;
}






