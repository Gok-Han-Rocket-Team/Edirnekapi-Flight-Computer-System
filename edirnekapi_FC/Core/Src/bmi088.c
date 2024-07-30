/*
 * bmi088.c
 *
 *  Created on: May 6, 2024
 *      Author: yahya
 */

#include "bmi088.h"
#include "math.h"
#include "queternion.h"

//#define SELFTEST_ENABLED

static I2C_HandleTypeDef* bmi_I2C;
static bmi088_struct_t* BMI;
static uint8_t isTimeUpdated = 0;
uint8_t isStarded = 0;

int errorLine = 0;
int offsetCounter = 0;
double g[2][3];		//Offset Values.


extern UART_HandleTypeDef huart1;

#ifdef SELFTEST_ENABLED
#include <string.h>
#include <stdio.h>
#endif

static void bmi088_selfTest()
{
	HAL_Delay(100);

	HAL_StatusTypeDef retVal = HAL_OK;

	uint8_t buf[1];

	buf[0] = ACC_RESET;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // Accel reset
	HAL_Delay(100);

	buf[0] = ACC_RANGE_24G;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc range set to 24G

	buf[0] = (0x01 << 7) | (0x02 << 4) | ACC_ODR_1600;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc some configs
	HAL_Delay(4); // wait for 3 ms

	//positive self test for accel
	buf[0] = ACC_SELF_TEST_POSITIVE;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_SELF_TEST, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc positive self test activated
	HAL_Delay(70); // wait for min 50ms

	retVal |= HAL_I2C_Mem_Read(bmi_I2C, ACC_I2C_ADD, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.accel, 6, 20);
	BMI->testVals.selfTest_acc_z = (BMI->rawDatas.accel[5] << 8) | BMI->rawDatas.accel[4];
	BMI->testVals.selfTest_acc_y = (BMI->rawDatas.accel[3] << 8) | BMI->rawDatas.accel[2];
	BMI->testVals.selfTest_acc_x = (BMI->rawDatas.accel[1] << 8) | BMI->rawDatas.accel[0];

	BMI->testVals.acc_z[0] = (double)BMI->testVals.selfTest_acc_z / 32768.0 * 1000.0 * 1.5 * pow(2.0, (double)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_y[0] = (double)BMI->testVals.selfTest_acc_y / 32768.0 * 1000.0 * 1.5 * pow(2.0, (double)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_x[0] = (double)BMI->testVals.selfTest_acc_x / 32768.0 * 1000.0 * 1.5 * pow(2.0, (double)(ACC_RANGE_24G + 1));

	//negative self test for accel
	buf[0] = ACC_SELF_TEST_NEGATIVE;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_SELF_TEST, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc negative self test activated
	HAL_Delay(70); // wait for min 50ms

	retVal |= HAL_I2C_Mem_Read(bmi_I2C, ACC_I2C_ADD, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.accel, 6, 20);
	BMI->testVals.selfTest_acc_z = (BMI->rawDatas.accel[5] << 8) | BMI->rawDatas.accel[4];
	BMI->testVals.selfTest_acc_y = (BMI->rawDatas.accel[3] << 8) | BMI->rawDatas.accel[2];
	BMI->testVals.selfTest_acc_x = (BMI->rawDatas.accel[1] << 8) | BMI->rawDatas.accel[0];

	BMI->testVals.acc_z[1] = (float)BMI->testVals.selfTest_acc_z / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_y[1] = (float)BMI->testVals.selfTest_acc_y / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_x[1] = (float)BMI->testVals.selfTest_acc_x / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(ACC_RANGE_24G + 1));

	buf[0] = ACC_SELF_TEST_OFF;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_SELF_TEST, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc self test off
	HAL_Delay(70);	//wait for steady state.
	if(retVal != HAL_OK)
		Error_Handler();
}

void bmi088_init(bmi088_struct_t* BMI_, I2C_HandleTypeDef* I2C_)
{
	quaternionSet_zero();
	HAL_StatusTypeDef retVal = HAL_OK;
	bmi_I2C = I2C_;
	BMI = BMI_;
	BMI->rawDatas.isGyroUpdated = 0;
	BMI->rawDatas.isAccelUpdated = 0;

#ifdef SELFTEST_ENABLED
	bmi088_selfTest();
	uint8_t buffer[100];
	sprintf((char*)buffer, "self test positive:  a_x: %f  a_y: %f  a_z: %f\r\n", BMI->testVals.acc_x[0], BMI->testVals.acc_y[0], BMI->testVals.acc_z[0]);
	HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer), 50);
	sprintf((char*)buffer, "self test negative:  a_x: %f  a_y: %f  a_z: %f\r\n", BMI->testVals.acc_x[1], BMI->testVals.acc_y[1], BMI->testVals.acc_z[1]);
	HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer), 50);
	while(1);
#endif
	uint8_t buf[1];

	HAL_Delay(10);

	buf[0] = 0x01;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_PWR_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // power save ultra

	buf[0] = ACC_DISABLE;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // accel disable
	HAL_Delay(20);

	buf[0] = ACC_RESET;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // Accel reset
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(40);

	buf[0] = FIFO_RESET;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // FIFO reset
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(40);

	buf[0] = GYRO_RESET;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_SOFT_RESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro reset
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(40);

	HAL_I2C_DeInit(bmi_I2C);  // I2C arayüzünü de-initialize edin
	HAL_I2C_Init(bmi_I2C);    // I2C arayüzünü yeniden initialize edin
/*
	//accel interrupt renew
	buf[0] = (0x00);
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_INT1_IO_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel interrupt config.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = (0x00);
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_INT_MAP_DATA, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel interrupt DRDY map to pin1.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(40);
*/

	//Gyroscope configuration.
	buf[0] = BMI->deviceConfig.gyro_range;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_RANGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro range config
	retVal != HAL_OK ? errorLine =__LINE__ : 0;


	buf[0] = BMI->deviceConfig.gyro_bandWidth;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_BANDWITH, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro bandwidth config
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = BMI->deviceConfig.gyro_powerMode;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_LPM1, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro power mode config.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(40);

	//gyro interrupt
	buf[0] = GYRO_INT_ENABLE;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_INT_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro interrupt enabled.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = (GYRO_INT_IO_PP << 1) | (GYRO_INT_ACT_HIGH << 0);
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_INT_3_4_IO_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro interrupt 4 config
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = GYRO_INT_MAP_3;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, GYRO_I2C_ADD, GYRO_INT_3_4_IO_MAP, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro interrupt pin 4 mapped.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	//Accelerometer configuration.
	buf[0] = ACC_ENABLE;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // Accel on
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(8);

	buf[0] = BMI->deviceConfig.acc_powerMode;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_PWR_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel mode active
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(8);

	buf[0] = (BMI->deviceConfig.acc_bandwith << 4) | BMI->deviceConfig.acc_outputDateRate;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel bandwith and odr selection
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = BMI->deviceConfig.acc_range;
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel range config.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	//accel interrupt
	buf[0] = (0x01 << 3) | (ACC_INT1_OD_PP << 2) | (ACC_INT1_LVL_ACT_HIGH << 1);
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_INT1_IO_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel interrupt config.
	//retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = (0x01 << 2);
	retVal |= HAL_I2C_Mem_Write(bmi_I2C, ACC_I2C_ADD, ACC_INT_MAP_DATA, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel interrupt DRDY map to pin1.
	//retVal != HAL_OK ? errorLine =__LINE__ : 0;

	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	//if(retVal != HAL_OK)

		//Error_Handler();

}
void bmi088_update()
{

	if(BMI->rawDatas.isAccelUpdated)
	{
		HAL_StatusTypeDef retVal = HAL_I2C_Mem_Read(bmi_I2C, ACC_I2C_ADD, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.accel, 9, 20);
		HAL_I2C_Mem_Read(bmi_I2C, ACC_I2C_ADD, ACC_TEMP_MSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.temp, 2, 20);

		uint16_t Temp_uint11 = (BMI->rawDatas.temp[0] << 3) | (BMI->rawDatas.temp[1] >> 5);
		int16_t Temp_int11 = 0;
		if (Temp_uint11 > 1023){
			Temp_int11 = Temp_uint11 - 2048;
		}
		else{
			Temp_int11 = Temp_uint11;
			BMI->temp = (float)Temp_int11 * 0.125 + 23.0;
		}
		uint32_t sensorTime = (BMI->rawDatas.accel[8] << 16) | (BMI->rawDatas.accel[7] << 8) | BMI->rawDatas.accel[6];

		BMI->currentTime= (float)sensorTime * 39.0625 / 1000000.0;

		int16_t acc_z_16 = (BMI->rawDatas.accel[5] << 8) | BMI->rawDatas.accel[4];
		int16_t acc_y_16 = (BMI->rawDatas.accel[3] << 8) | BMI->rawDatas.accel[2];
		int16_t acc_x_16 = (BMI->rawDatas.accel[1] << 8) | BMI->rawDatas.accel[0];

		BMI->acc_z = (float)acc_z_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->deviceConfig.acc_range + 1)) - ACCEL_Z_OFFSET;
		BMI->acc_y = (float)acc_y_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->deviceConfig.acc_range + 1)) - ACCEL_Y_OFFSET;
		BMI->acc_x = (float)acc_x_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->deviceConfig.acc_range + 1)) - ACCEL_X_OFFSET;

		//BMI->vel_z = ((float)acc_z_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->deviceConfig.acc_range + 1)) - ACCEL_Z_OFFSET - 1000.0) * BMI->deltaTime;
		//BMI->vel_y = ((float)acc_y_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->deviceConfig.acc_range + 1)) - ACCEL_Y_OFFSET) * BMI->deltaTime;
		//BMI->vel_x = ((float)acc_x_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->deviceConfig.acc_range + 1)) - ACCEL_X_OFFSET) * BMI->deltaTime;

		BMI->deltaTime = BMI->currentTime - BMI->lastTime < 0 ? 0.0 : BMI->currentTime - BMI->lastTime;
		BMI->lastTime = BMI->currentTime;

		BMI->rawDatas.isAccelUpdated = 0;
		isTimeUpdated = 1;
		UNUSED(retVal);
	}

	if(BMI->rawDatas.isGyroUpdated && isTimeUpdated)
	{
		if(isStarded){
			HAL_I2C_Mem_Read(bmi_I2C, GYRO_I2C_ADD, GYRO_RATE_X_LSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.gyro, 6, 20);
			int16_t gyro_z_16 = (BMI->rawDatas.gyro[5] << 8) | BMI->rawDatas.gyro[4];
			int16_t gyro_y_16 = (BMI->rawDatas.gyro[3] << 8) | BMI->rawDatas.gyro[2];
			int16_t gyro_x_16 = (BMI->rawDatas.gyro[1] << 8) | BMI->rawDatas.gyro[0];

			/*
			BMI->delta_angle_z = (((float)gyro_z_16 / 32767.0) * (float)(2000 >> BMI->deviceConfig.gyro_range) - GYRO_Z_OFFSET) * BMI->deltaTime;
			BMI->delta_angle_y = (((float)gyro_y_16 / 32767.0) * (float)(2000 >> BMI->deviceConfig.gyro_range) - GYRO_Y_OFFSET) * BMI->deltaTime;
			BMI->delta_angle_x = (((float)gyro_x_16 / 32767.0) * (float)(2000 >> BMI->deviceConfig.gyro_range) - GYRO_X_OFFSET) * BMI->deltaTime;
			 */
			BMI->gyro_z = (((double)gyro_z_16 / 32767.0) * (double)(2000 >> BMI->deviceConfig.gyro_range) - GYRO_Z_OFFSET);
			BMI->gyro_y = (((double)gyro_y_16 / 32767.0) * (double)(2000 >> BMI->deviceConfig.gyro_range) - GYRO_Y_OFFSET);
			BMI->gyro_x = (((double)gyro_x_16 / 32767.0) * (double)(2000 >> BMI->deviceConfig.gyro_range) - GYRO_X_OFFSET);

			BMI->gyro_z_angle += (BMI->gyro_z) * BMI->deltaTime;
			BMI->gyro_y_angle += (BMI->gyro_y) * BMI->deltaTime;
			BMI->gyro_x_angle += (BMI->gyro_x) * BMI->deltaTime;

			//update_quaternion(q, BMI->gyro_x, BMI->gyro_y, BMI->gyro_z, BMI->deltaTime);
			//calculateQuaternion(q, BMI->gyro_x, BMI->gyro_y, BMI->gyro_z, BMI->deltaTime, vector);

			updateQuaternion(BMI->gyro_x * M_PI / 180.0, BMI->gyro_y * M_PI / 180.0, BMI->gyro_z * M_PI / 180.0, BMI->deltaTime);
			quaternionToEuler();

			BMI->rawDatas.isGyroUpdated = 0;
			isTimeUpdated = 0;
		}else
		{
			BMI->lastTime = BMI->currentTime;
			isStarded = 1;
		}
	}
}


void bmi088_getAccelDatas_INT()
{
	BMI->rawDatas.isAccelUpdated = 1;
}

void bmi088_getGyroDatas_INT()
{
	BMI->rawDatas.isGyroUpdated = 1;
}

uint8_t bmi088_getGyroChipId()
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(bmi_I2C, GYRO_I2C_ADD, GYRO_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);
	return data;
}

void getOffset()
{

	uint8_t buffer[250];
	 if(offsetCounter < 1000){
		 g[0][0] += BMI->gyro_x;
		 g[0][1] += BMI->gyro_y;
		 g[0][2] += BMI->gyro_z;
		 g[1][0] += BMI->acc_x;
		 g[1][1] += BMI->acc_y;
		 g[1][2] += BMI->acc_z;
		 offsetCounter++;
	 }else{
		 g[0][0] /= 999.0;
		 g[0][1] /= 999.0;
		 g[0][2] /= 999.0;
		 g[1][0] /= 1000.0;
		 g[1][1] /= 1000.0;
		 g[1][2] /= 1000.0;
		 sprintf((char*)buffer, "\n\n\rg_x:%f \t g_y:%f \t g_z:%f \t a_x:%f \t a_y:%f \ta_z:%f\n\n\r", g[0][0], g[0][1], g[0][2], g[1][0], g[1][1], g[1][2]);
		 HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer), 250);
		 Error_Handler();
	 }
}



