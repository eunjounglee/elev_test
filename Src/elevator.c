#include "elevator.h"      // 엘리베이터 상위 로직(상태머신, 목표층 판단 등) 헤더
#include "button.h"        // 버튼 입력 처리(디바운싱/엣지 검출 포함 추정)
#include "stepper.h"       // 스테퍼 모터 구동 모듈(논블로킹 stepper_task 포함)
#include "swTimer.h"       // 소프트웨어 타이머 유틸리티(비차단 타이밍 관리)
#include "adc.h"           // ADC 드라이버(ADC 시작/ISR 및 adc_value 전역 제공 가정)
#include "motor_speed.h"   // ★ 추가: ADC값 → 스텝 주기(ms)로 변환 + 필터/램핑
#include "stdio.h"         // printf 디버그 출력
#include "stdint.h"        // 고정폭 정수 타입(uint32_t 등)
#include "photosensor.h"   // 포토센서(층 감지) 모듈, 끊김(broken) 이벤트 제공

void currentstate(void)
{
    ADC1_StartIT();        // ADC1 변환을 인터럽트 모드로 시작.
                           // → 변환 완료 ISR에서 'adc_value'(volatile 전역 가정)가 갱신됨.

    stepper_init();        // 스테퍼 모듈 내부 상태 초기화(스텝 인덱스/주기/방향 등).
                           // GPIO 초기화는 CubeMX에서 이미 됐다고 가정.

    ps_init();             // 포토센서 모듈 초기화. 층 감지(빛 끊김) 이벤트 사용 준비.

    // 일시정지 타이머: 포토센서 끊김 시 '정지 유지 시간'을 비차단으로 잴 용도
    softTimer_Init(swTimerID1, 500);  // swTimerID1을 500ms 주기로 arm(지금부터 500ms 뒤 만료 의미가 일반적)
    bool paused = false;              // 현재 '일시정지 상태' 플래그(정지 중이면 true)

    // ★ 속도 모듈 초기화: ADC→주기(ms) 매핑 파라미터(보정/필터/램핑/반전)
    motor_speed_init(/*adc_min*/   100,   // ADC 하한(이하 컷 또는 스케일 기준)
                     /*adc_max*/   3300,  // ADC 상한(이상 컷 또는 스케일 기준)
                     /*min_ms*/    1,     // 스텝 간 최소 지연(가장 빠른 속도). 너무 작으면 토크 부족 가능
                     /*max_ms*/    10,    // 스텝 간 최대 지연(가장 느린 속도)
                     /*ramp_ms*/   1,     // 램핑 단위(ms). 속도 급변 억제(부드러운 가감속)
                     /*invert*/    true); // true면 ADC↑ → period_ms↓ (손잡이 올릴수록 더 빠르게)

    while (1)  // 메인 상태 루프(논블로킹. 내부에서 HAL_Delay 사용 안 함)
    {
        // 버튼 입력 폴링(예: 짧은 펄스 입력을 놓치지 않도록 루프마다 읽기)
        uint8_t btn0 = buttonGetPressed(0);  // 버튼0 눌림 감지(엣지/레벨은 모듈 정의에 따름)
        uint8_t btn1 = buttonGetPressed(1);  // 버튼1 눌림 감지
        uint8_t btn2 = buttonGetPressed(2);  // 버튼2 눌림 감지

        // 1) (선택) 속도/ADC 상태를 1초마다 로그로 찍기
        static uint32_t prev = 0;                 // 마지막 로그 시각(ms)
        if (HAL_GetTick() - prev >= 1000)         // 1초 경과 확인(비차단)
        {
            prev = HAL_GetTick();                 // 기준 시각 갱신
            // motor 속도 test(디버깅 필요 시 주석 해제)
            // printf는 느릴 수 있으므로 1Hz 정도로 제한하는 게 좋음.
            // printf("ADC(raw)=%u, filt=%ld, period=%lums\r\n",
            //        adc_value,                                // ISR이 갱신한 원시 ADC
            //        (long)motor_speed_get_filtered_adc(),    // 필터 적용된 ADC
            //        motor_speed_get_period_ms());            // 현재 적용 주기(ms)
        }

        // 2) 포토센서 끊김 감지 → 일시정지 진입
        uint8_t floor = ps_any_broken();   // "방금" 끊김 발생한 층(없으면 0xFF 반환이 일반적)
        if (floor != 0xFF)                 // 유효층이면
        {
            // 모터 정지 및 정지 유지 타이머 리셋
            paused = true;                 // 정지 상태 진입
            stepper_stop();                // 스테퍼 즉시 정지(코일 OFF 구현 시 발열↓). 주석의 '50ms'는 오해 소지 있음.
            softTimer_Reset(swTimerID1);   // 지금부터 500ms를 새로 카운트(정지 유지 시간 시작점)

            // (선택) 이벤트 로그: 어느 층에서 끊겼는지 기록
            printf("[PS] broken at floor %u\r\n", floor);
        }

        // 일시정지 상태이며, 정지 타이머(500ms)가 만료되었으면 재개 조건 검사
        if (paused && softTimer_IsTimeOut(swTimerID1))
        {
            if (btn0 || btn1 || btn2)
            {
            paused = false;            // 버튼이 눌리면 정지 해제(의도 추정: 수동 승인 후 재개)


            stepper_resume();
            }
        }

        // 3) 속도 갱신 + 스텝퍼에 적용(정지 상태가 아닐 때만)
        // motor_speed_update()는 adc_value(volatile 전역 가정)를 읽어
        // 보정/필터/램핑 후 주기(ms)를 산출해서 반환.
        uint32_t period_ms = motor_speed_update(adc_value);
        if (!paused)                       // 정지 중에는 속도 적용하지 않음
        {
            stepper_set_period_ms(period_ms); // 비차단 스텝 엔진의 주기 업데이트(속도 제어)
            stepper_set_dir(DIR_CW);          // 회전 방향 설정(여기서는 시계방향 고정)
        }

        // 4) 논블로킹 스텝 엔진 한 틱 처리
        // softTimer가 만료되었을 때만 1스텝 출력하고 즉시 리턴.
        // HAL_Delay가 없어 다른 작업과 공존(협조적 스케줄링).
        stepper_task();
    }
}
