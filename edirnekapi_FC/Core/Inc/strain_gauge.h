/*
 * strain_gauge.h
 *
 *  Created on: Aug 31, 2024
 *      Author: yahya
 */

#ifndef INC_STRAIN_GAUGE_H_
#define INC_STRAIN_GAUGE_H_
#include "main.h"



#define __nop() __asm volatile ("nop")
#define   _HX711_DELAY_US_LOOP  4
#define hx711_delay(x)    HAL_Delay(x)
//####################################################################################################################

typedef struct
{
  GPIO_TypeDef  *clk_gpio;
  GPIO_TypeDef  *dat_gpio;
  uint16_t      clk_pin;
  uint16_t      dat_pin;
  int32_t       offset;
  float         coef;
  uint8_t       lock;

}hx711_t;

//####################################################################################################################
void straing_gage_gpio_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin);
int32_t strain_gage_get_vals(hx711_t *hx711);
#endif /* INC_STRAIN_GAUGE_H_ */
