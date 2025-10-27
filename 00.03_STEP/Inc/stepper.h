/*
 * stepper.h
 *
 *  Created on: Oct 16, 2025
 *      Author: user13
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"

#define STEPS_PER_REVOLUTION	4096
#define DIR_CW								0 // 시계방향
#define DIR_CCW								1 // 반시계방향

#define IN1_PIN  	GPIO_PIN_5
#define IN1_GPIO_PORT  	GPIOA
#define IN2_PIN  	GPIO_PIN_6
#define IN2_GPIO_PORT  	GPIOA
#define IN3_PIN  	GPIO_PIN_7
#define IN3_GPIO_PORT  	GPIOA
#define IN4_PIN  	GPIO_PIN_6
#define IN4_GPIO_PORT  	GPIOB

static const uint8_t HALF_STEP_SEO[8][4]=
		{
				{1, 0, 0, 0},
				{1, 1, 0, 0},
				{0, 1, 0, 0},
				{0, 1, 1, 0},
				{0, 0, 1, 0},
				{0, 0, 1, 1},
				{0, 0, 0, 1},
				{1, 0, 0, 1}
		};

void stepMotor(uint8_t step);
void rotateSteps(uint16_t steps, uint8_t direction);
void rotateDegrees(uint16_t degrees, uint8_t direction);



#endif /* INC_STEPPER_H_ */
