/*
 * mode.c
 *
 *  Created on: Oct 16, 2025
 *      Author: user13
 */

#include "mode.h"
#include "led.h"
#include "button.h"

static inline uint8_t wrap_prev(uint8_t v) { return (v == 1) ? 10 : (uint8_t)(v - 1); }
static inline uint8_t wrap_next(uint8_t v) { return (v == 10) ? 1 : (uint8_t)(v + 1); }

void mode()
{
  typedef enum
  {
  	MODE_OFF,
		MODE_ON
  }MODE;

  static MODE    mode  = MODE_OFF;  // 시작은 OFF
  static uint8_t first = 0;         // 전원 후 '첫 동작'만 특별 처리
  static uint8_t num   = 1;         // 1~8 (led index = num-1)

  segNumOff(9);

    while (1)
    {
      // ---- 버튼 엣지 값은 '한 번만' 읽어서 캐시! ----
  		uint8_t btn0 = buttonGetPressed(0);  // OFF
  		uint8_t btn1 = buttonGetPressed(1);  // 이전
  		uint8_t btn2 = buttonGetPressed(2);  // 다음

  		// ---- OFF 버튼: 그냥 끄기만 (first는 유지) ----
  		if (btn0)
  		{
  				mode = MODE_OFF;
  				segNumOff(9);
  		}

  		// ---- 전원 직후 '첫 동작' 한 번만 특수 처리 ----
  		if (first && ~mode == MODE_OFF)
  		{
  				if (btn1)
  				{         // 요구사항: 첫 버튼이 btn1이면 ledOn(7)부터
  						num   = 10;         // index 10
  						mode  = MODE_ON;
  				}
  				else if (btn2)
  				{        // 첫 버튼이 btn2이면 ledOn(0)부터
  						num   = 1;         // index 0
  						mode  = MODE_ON;
  				}
  			first = 1;
  		}
  		else
  		{
  				// ---- 일반 동작 (이미 ON이거나, OFF였다가 재개) ----
  				if (btn1)
  				{               // 이전
  						num  = wrap_prev(num);
  						mode = MODE_ON;
  				}
  				if (btn2)
  				{               // 다음
  						num  = wrap_next(num);
  						mode = MODE_ON;
  				}
  		}

  		// ---- 출력 반영 ----
  		if (mode == MODE_ON)  segNumOn(num - 1);
  		else                  segNumOff(9);
    }
}
