/*
 * led.h
 *
 *  Created on: Oct 13, 2025
 *      Author: user13
 */

#ifndef INC_LED_H_
#define INC_LED_H_


#include "main.h"
#include "stdint.h"


typedef struct
{
	GPIO_TypeDef	*port;
	uint16_t			pinNumber;
	GPIO_PinState onState;
	GPIO_PinState offState;
}LED_CONTROL;


void segNumOn(uint8_t num);
void segNumOff(uint8_t num);

#endif /* INC_LED_H_ */
