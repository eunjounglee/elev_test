/*
 * button.c
 *
 *  Created on: Oct 14, 2025
 *      Author: user13
 */

#include "button.h"

BUTTON_CONTROL button[4]=
		{
				{GPIOB, GPIO_PIN_1, 0},
				{GPIOB, GPIO_PIN_15, 0},
				{GPIOB, GPIO_PIN_14, 0},
				{GPIOB, GPIO_PIN_13, 0}
		};

bool buttonGetPressed(uint8_t num)
{
	static uint32_t prevTime[3] = {0xFFFFFFFF};

	if(prevTime[num] == 0xFFFFFFFF)
	{
		prevTime[num] = HAL_GetTick();
	}

	bool ret = false;

	if(HAL_GPIO_ReadPin(button[num].port, button[num].pinNumber) == button[num].onState)
	{
		uint32_t currTime = HAL_GetTick();

		if(currTime - prevTime[num] > 200)
		{
			if(HAL_GPIO_ReadPin(button[num].port, button[num].pinNumber) == button[num].onState)
			{
				ret = true;
			}
			prevTime[num] = currTime;
		}
	}
	return ret;
}
