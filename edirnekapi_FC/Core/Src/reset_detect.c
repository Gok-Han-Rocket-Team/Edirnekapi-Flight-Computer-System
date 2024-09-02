/*
 * reset_detect.c
 *
 *  Created on: Aug 22, 2024
 *      Author: yahya
 */

#include "reset_detect.h"


void save_status(const uint8_t status)
{
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, (uint32_t)status);
}

void save_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t)
{
	uint32_t seconds = 0;
	seconds += time_t.Hours * 3600 + time_t.Minutes * 60 + time_t.Seconds;
	seconds += date_t.Date * 86400;
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, seconds);
}

uint32_t measure_abs_time(const RTC_TimeTypeDef time_t, const RTC_DateTypeDef date_t)
{
	uint32_t seconds = 0;
	seconds += time_t.Hours * 3600 + time_t.Minutes * 60 + time_t.Seconds;
	seconds += date_t.Date * 86400;
	HAL_PWR_EnableBkUpAccess();
	uint32_t saved_seconds = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	return((uint32_t)abs((int32_t)seconds - (int32_t)saved_seconds));
}
