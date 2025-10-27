/*
 * led.c
 *
 *  Created on: Oct 13, 2025
 *      Author: user13
 */

#include "led.h"

LED_CONTROL led[7]=
		{
				{GPIOB, GPIO_PIN_13, GPIO_PIN_SET, GPIO_PIN_RESET},
				{GPIOB, GPIO_PIN_14, GPIO_PIN_SET, GPIO_PIN_RESET},
				{GPIOB, GPIO_PIN_15, GPIO_PIN_SET, GPIO_PIN_RESET},
				{GPIOB, GPIO_PIN_1, GPIO_PIN_SET, GPIO_PIN_RESET},
				{GPIOB, GPIO_PIN_2, GPIO_PIN_SET, GPIO_PIN_RESET},
				{GPIOB, GPIO_PIN_12, GPIO_PIN_SET, GPIO_PIN_RESET},
				{GPIOA, GPIO_PIN_11, GPIO_PIN_SET, GPIO_PIN_RESET},
		};

uint8_t fndData[10]=
    {
        0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x67
    };

void segNumOn(uint8_t num)
{
    uint8_t data = fndData[num];

    for (int i = 0; i < 7; i++)
    {
        if (data & (1 << i))
            HAL_GPIO_WritePin(led[i].port, led[i].pinNumber, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(led[i].port, led[i].pinNumber, GPIO_PIN_RESET);
    }
}

void segNumOff(uint8_t num)
{
  for (int i = 0; i < 7; i++)
  {
          HAL_GPIO_WritePin(led[i].port, led[i].pinNumber, GPIO_PIN_RESET);
  }
}
