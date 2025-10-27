#ifndef INC_MOTOR_SPEED_H_
#define INC_MOTOR_SPEED_H_

#include <stdint.h>
#include <stdbool.h>

// 초기화: ADC 구간, 주기 구간(ms), 램프 스텝(ms), 역매핑 여부
void     motor_speed_init(uint16_t adc_min, uint16_t adc_max,
                          uint32_t period_min_ms, uint32_t period_max_ms,
                          uint32_t ramp_step_ms, bool invert);

// 주기 갱신: adc_raw 넣으면 내부 필터/매핑/램프 반영 후 현재 주기(ms) 반환
uint32_t motor_speed_update(uint16_t adc_raw);

// 조회용(디버깅)
uint32_t motor_speed_get_period_ms(void);
int32_t  motor_speed_get_filtered_adc(void);

#endif /* INC_MOTOR_SPEED_H_ */
