/*
 * elevator.h
 *
 *  Created on: Oct 16, 2025
 *      Author: user13
 */

#ifndef INC_ELEVATOR_H_
#define INC_ELEVATOR_H_

#include "main.h"

/* ------------------------------------------------------------------------
 * 동작 모드 (현재는 사용 안 하지만 유지 가능)
 * ------------------------------------------------------------------------ */
typedef enum {
    MODE_OFF,
    MODE_ON
} MODE;

/* ------------------------------------------------------------------------
 * 엘리베이터 외부 호출 버튼 방향 정의
 * ------------------------------------------------------------------------ */
typedef enum {
    BUTTON_CALL_UP   = 1,   // 위로 호출
    BUTTON_CALL_DOWN = 2    // 아래로 호출
} buttonUpDownCall;

/* ------------------------------------------------------------------------
 * 함수 프로토타입
 * ------------------------------------------------------------------------ */
void elevator_init(void);
void elevator_loop(void);
void handleExternalButtonCall(uint8_t floor, buttonUpDownCall call_dir);
void read_external_buttons(void);
void read_inside_buttons(void);
void handleInsideButton(uint8_t floor);

#endif /* INC_ELEVATOR_H_ */
