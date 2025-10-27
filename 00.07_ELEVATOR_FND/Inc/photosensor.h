#ifndef INC_PHOTOSENSOR_H_
#define INC_PHOTOSENSOR_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/** 층(센서) 개수: 필요 시 변경 */
#define PS_FLOOR_COUNT  3

/** 층 인덱스 (가독성용, 0~N-1) */
typedef enum {
  PS_F0 = 0,
  PS_F1 = 1,
  PS_F2 = 2,
} ps_floor_t;

/** 초기화
 *  - GPIO/EXTI 설정은 CubeMX에서 완료돼 있다고 가정(핀/풀업/엣지)
 *  - 이 함수는 내부 상태(래치/디바운스)만 초기화
 */
void ps_init(void);

/** EXTI에서 호출: HAL 콜백에서 그대로 전달해주면 됨 */
void ps_on_exti(uint16_t gpio_pin);

/** 레벨 읽기(현재 빔 차단 여부)
 *  true  = 빔 끊김(차단, "막힘")
 *  false = 빔 통과(정상)
 */
bool ps_is_blocked(uint8_t floor);

/** 엣지 이벤트(빔이 방금 끊겼는지) — 1회성 래치, 읽으면 자동 클리어 */
bool ps_was_broken(uint8_t floor);

/** 아무 층이나 방금 끊김이 있었는지 (있으면 해당 floor 인덱스 반환, 없으면 0xFF) */
uint8_t ps_any_broken(void);

#endif /* INC_PHOTOSENSOR_H_ */
