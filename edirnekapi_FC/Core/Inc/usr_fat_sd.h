/*
 * usr_fat_sd.h
 *
 *  Created on: 28 Ağu 2024
 *      Author: numan
 */

#ifndef USR_FAT_SD_H_
#define USR_FAT_SD_H_

#define bool _Bool
#define true 1
#define false 0
#define _io static
#define PUBLIC
// #define GNSS_SD_INTEGRATION_MODE

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "bme280.h"
#include "bmi088.h"
#include "algorithms.h"
#include "queternion.h"
#include "lora.h"
#include "usr_gnss_l86_parser.h"
#include "dataPacking.h"
#include "externalPins.h"
#include "configuration.h"
#include "reset_detect.h"

// #include "fatfs_sd.h"

#include "fatfs.h"
typedef struct _TEST_PAYLOAD1_TAG
{
    float a;
    float b;
    float c;
} _TEST_PAYLOAD1;
typedef struct _TEST_PAYLOAD2_TAG
{
    float lat;
    float lon;
    float speedKN;
    float timeDateBuf;
} _TEST_PAYLOAD2;

/*logger structures*/
typedef struct BME280_TAG
{
    float presure;
    float humidity;
    volatile float altitude;
    float velocity;
} _BME_280_T;
typedef struct BMI088_TAG
{
    float acc_x, acc_y, acc_z;
    double gyro_x, gyro_y, gyro_z;
} _BMI_088_T;
typedef struct BACKUP_SRAM_DATAS_TAG
{
    uint8_t r_status;
    float q[4];
    float max_altitude;
} _BACKUP_SRAM_DATA_T;
typedef struct GPS_DATA_TAG
{
    float lat;
    float lon;
    float timeBuf;
    int satInUse;
    float altitudeInMeter;
} _GPS_DATA_T;
typedef struct POWER_DATA_TAG
{
    float voltage;
    float current;
    float mWatt;
    float mWatt_s;
} _POWER_DATA_T;

/*EOF logger structures*/

void usr_fatfsInitial(void);
void sdInitials(void);
int sd_transmit(const char *str);
void usrFatTest(void);
void sdDataLogger(uint32_t counter, BME_280_t *BME_Pack, bmi088_struct_t *BMI_Pack, backup_sram_datas_s *Backup_Pack, S_GPS_L86_DATA *GPS_Pack, power *Power_Pack);

#endif /* USR_FAT_SD_H_ */
