/*
 * mode.h
 *
 *  Created on: Oct 16, 2025
 *      Author: user13
 */

#ifndef INC_ELEVATOR_H_
#define INC_ELEVATOR_H_

typedef enum
  {
  	MODE_OFF,
		MODE_ON
  }MODE;

void elevator_init();
void elevator_loop(void);

#endif /* INC_ELEVATOR_H_ */
