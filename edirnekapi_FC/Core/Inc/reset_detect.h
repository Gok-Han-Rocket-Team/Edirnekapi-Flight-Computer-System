/*
 * reset_detect.h
 *
 *  Created on: Aug 22, 2024
 *      Author: yahya
 */

#ifndef INC_RESET_DETECT_H_
#define INC_RESET_DETECT_H_
#include "main.h"
#include "stdlib.h"
#include "bme280.h"
#include "algorithms.h"

extern RTC_HandleTypeDef hrtc;

typedef struct backup_sram_datas
{
	enum flightStates r_status;
	float q[4];
	BME_parameters_t bme_params;
	float base_altitude;
	float max_altitude;
	double offset_vals[3];
}backup_sram_datas_s;

void save_status(const uint8_t status);
void save_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t);
uint32_t measure_abs_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t);

#endif /* INC_RESET_DETECT_H_ */
