/*
 * stepper.c
 *
 *  Created on: Oct 16, 2025
 *      Author: user13
 */

#include "stepper.h"
#include "swTimer.h"


void stepMotor(uint8_t step)
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, HALF_STEP_SEO[step][0]);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, HALF_STEP_SEO[step][1]);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, HALF_STEP_SEO[step][2]);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, HALF_STEP_SEO[step][3]);
}

void rotateSteps(uint16_t steps, uint8_t direction)
{
//	softTimer_Init(swTimerID1, 1); // 1ms

	for(uint16_t i = 0 ; i < steps ; i++ )
	{
		uint8_t step;
		// 회전방향
		if(direction == DIR_CW)
		{
			step = i % 8;				// CW
		}
		else
		{
			step = 7 - (i % 8); // CCW
		}
		stepMotor(step);

		HAL_Delay(1);
	}

}

void rotateDegrees(uint16_t degrees, uint8_t direction)
{
	// 각도계산

	uint16_t steps = (uint16_t)((uint32_t)(degrees * STEPS_PER_REVOLUTION) / 360);

	rotateSteps(steps, direction);

}
