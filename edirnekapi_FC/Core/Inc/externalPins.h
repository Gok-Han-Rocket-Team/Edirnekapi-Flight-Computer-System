/*
 * externalPins.h
 *
 *  Created on: 19 Ağu 2024
 *      Author: yahya
 */

#ifndef INC_EXTERNALPINS_H_
#define INC_EXTERNALPINS_H_
#include "main.h"

typedef struct extPin_s{
	GPIO_TypeDef 	*gpio_port;
	uint32_t 		last_time;
	uint32_t 		duration;
	uint16_t		gpio_pin;
}ext_pin_s;

void ext_pin_open(ext_pin_s *p_ext_pin);
void ext_pin_open_duration(ext_pin_s *p_ext_pin, uint32_t duration);
void ext_pin_close(ext_pin_s *p_ext_pin);
void ext_pin_update(ext_pin_s *p_ext_pin);
#endif /* INC_EXTERNALPINS_H_ */
