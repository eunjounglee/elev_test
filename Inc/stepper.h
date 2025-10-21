/*
 * stepper.h
 * 논블로킹(ms 기반) 스텝모터 제어 API
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"
#include "swTimer.h"
#include <stdbool.h>

#define STEPS_PER_REVOLUTION  4096
#define DIR_CW   0
#define DIR_CCW  1

// 핀 매핑 (네 프로젝트에 맞춰 유지)
#define IN1_PIN         GPIO_PIN_5
#define IN1_GPIO_PORT   GPIOA
#define IN2_PIN         GPIO_PIN_6
#define IN2_GPIO_PORT   GPIOA
#define IN3_PIN         GPIO_PIN_7
#define IN3_GPIO_PORT   GPIOA
#define IN4_PIN         GPIO_PIN_6
#define IN4_GPIO_PORT   GPIOB

void stepper_init(void);
void stepper_set_dir(uint8_t dir);
void stepper_set_period_ms(uint32_t ms);   // 속도 설정 (한 스텝 간격, ms)
void stepper_stop(void);                   // 정지
void stepper_resume(void);                 // 재개
void stepper_task(void);                   // 매 루프마다 호출(논블로킹)
void rotateUntilSensor(uint8_t direction);

#endif /* INC_STEPPER_H_ */
