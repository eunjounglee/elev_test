#include "currentstate.h"
#include "button.h"
#include "stepper.h"
#include "swTimer.h"
#include "adc.h"
#include "motor_speed.h"   // ★ 추가
#include "stdio.h"
#include "stdint.h"

void currentstate(void)
{
    ADC1_StartIT();
    stepper_init();

    // 일시정지 타이머
    softTimer_Init(swTimerID1, 500);
    bool paused = false;

    // ★ 속도 모듈 초기화: (캘리브레이션 값/범위는 기존과 동일)
    motor_speed_init(/*adc_min*/   100,
                     /*adc_max*/   3300,
                     /*min_ms*/    1,     // 최저 1ms 허용 (원래 값 유지해도 됨)
                     /*max_ms*/    10,
                     /*ramp_ms*/   1,
                     /*invert*/    true); // ADC↑ → ms↓

    while (1)
    {
        // 1) 로그 (선택) — 모듈에서 값 가져와 출력
        static uint32_t prev = 0;
        if (HAL_GetTick() - prev >= 1000)
        {
            prev = HAL_GetTick();
            printf("ADC(raw)=%u, filt=%ld, period=%lums\r\n",
                   adc_value,
                   (long)motor_speed_get_filtered_adc(),
                   motor_speed_get_period_ms());
        }

        // 2) 버튼 일시정지
        if (buttonGetPressed(0))
        {
            paused = true;
            stepper_stop();
            softTimer_Reset(swTimerID1);
        }
        if (paused && softTimer_IsTimeOut(swTimerID1))
        {
            paused = false;
            stepper_resume();
        }

        // 3) 속도 갱신 + 스텝퍼 적용
        uint32_t period_ms = motor_speed_update(adc_value);
        if (!paused) {
            stepper_set_period_ms(period_ms);
            stepper_set_dir(DIR_CW);
        }

        // 4) 논블로킹 스텝
        stepper_task();
    }
}
