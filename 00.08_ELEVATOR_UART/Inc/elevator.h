/*
 * mode.h
 *
 *  Created on: Oct 16, 2025
 *      Author: user13
 */

#ifndef INC_ELEVATOR_H_
#define INC_ELEVATOR_H_

#include "main.h"

typedef enum
  {
      MODE_OFF,
        MODE_ON
  }MODE;

  typedef enum {
      BUTTON_CALL_UP   = 1,   // 위로 가기 호출
      BUTTON_CALL_DOWN = 2,   // 아래로 가기 호출
  } buttonUpDownCall;

void elevator_init();
void elevator_loop(void);
void handleExternalButtonCall(uint8_t buttonCall, buttonUpDownCall call_dir);
void read_external_buttons(void);

#endif /* INC_ELEVATOR_H_ */
