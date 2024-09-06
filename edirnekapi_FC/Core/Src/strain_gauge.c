/*
 * strain_gauge.c
 *
 *  Created on: Aug 31, 2024
 *      Author: yahya
 */
#include "main.h"
#include "strain_gauge.h"

extern UART_HandleTypeDef huart1;

static void strain_gage_delay_us(void)
{
  uint32_t delay = 4;
  while (delay > 0)
  {
    delay--;
    __nop(); __nop(); __nop(); __nop();
  }
}

void straing_gage_gpio_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin)
{
	uint8_t buf[50];
  hx711->clk_gpio = clk_gpio;
  hx711->clk_pin = clk_pin;
  hx711->dat_gpio = dat_gpio;
  hx711->dat_pin = dat_pin;

  HAL_GPIO_DeInit(clk_gpio, clk_pin);
  HAL_GPIO_DeInit(dat_gpio, dat_pin);

  GPIO_InitTypeDef  gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = clk_pin;
  HAL_GPIO_Init(clk_gpio, &gpio);

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = dat_pin;
  HAL_GPIO_Init(dat_gpio, &gpio);

  hx711_delay(10);
  int counter = 0;
  int32_t offset = 0;
  for(int i = 0; i < 20; i++)
  {
	  int32_t value = strain_gage_get_vals(hx711);
	  if(value != 0)
	  {
		  offset += value;
		  counter++;

	  }
	  hx711_delay(10);
  }
  hx711->offset = offset / counter;
}

int32_t strain_gage_get_vals(hx711_t *hx711)
{
  uint32_t data = 0;
  uint32_t  startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_SET)
  {
    hx711_delay(1);
    if(HAL_GetTick() - startTime > 1)
      return 0;
  }
  for(int8_t i=0; i<24 ; i++)
  {
    HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
    strain_gage_delay_us();
    HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
    strain_gage_delay_us();
    data = data << 1;
    if(HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_SET)
      data ++;
  }
  data = data ^ 0x800000;
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
  strain_gage_delay_us();
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
  strain_gage_delay_us();
  return (int32_t)data - hx711->offset;
}

