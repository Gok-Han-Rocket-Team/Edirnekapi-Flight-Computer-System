/*
 * externalPins.c
 *
 *  Created on: 19 Ağu 2024
 *      Author: yahya
 */
#include "externalPins.h"

void ext_pin_open(ext_pin_s *p_ext_pin)
{
	HAL_GPIO_WritePin(p_ext_pin->gpio_port, p_ext_pin->gpio_pin, SET);
	p_ext_pin->last_time = HAL_GetTick();
	p_ext_pin->duration = 100;		//ms
}

void ext_pin_open_duration(ext_pin_s *p_ext_pin, uint32_t duration)
{
	HAL_GPIO_WritePin(p_ext_pin->gpio_port, p_ext_pin->gpio_pin, SET);
	p_ext_pin->last_time = HAL_GetTick();
	p_ext_pin->duration = duration;		//ms
}

void ext_pin_close(ext_pin_s *p_ext_pin)
{
	HAL_GPIO_WritePin(p_ext_pin->gpio_port, p_ext_pin->gpio_pin, RESET);
}

void ext_pin_update(ext_pin_s *p_ext_pin)
{
	if((HAL_GetTick() - p_ext_pin->last_time) > p_ext_pin->duration)
	{
		HAL_GPIO_WritePin(p_ext_pin->gpio_port, p_ext_pin->gpio_pin, RESET);
	}
}
