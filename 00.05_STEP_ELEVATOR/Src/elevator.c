#include "elevator.h"      // 엘리베이터 상위 로직(상태머신, 목표층 판단 등) 헤더
#include "button.h"        // 버튼 입력 처리(디바운싱/엣지 검출 포함 추정)
#include "stepper.h"       // 스테퍼 모터 구동 모듈(논블로킹 stepper_task 포함)
#include "swTimer.h"       // 소프트웨어 타이머 유틸리티(비차단 타이밍 관리)
#include "adc.h"           // ADC 드라이버(ADC 시작/ISR 및 adc_value 전역 제공 가정)
#include "motor_speed.h"   // ★ 추가: ADC값 → 스텝 주기(ms)로 변환 + 필터/램핑
#include "stdio.h"         // printf 디버그 출력
#include "stdint.h"        // 고정폭 정수 타입(uint32_t 등)
#include "photosensor.h"   // 포토센서(층 감지) 모듈, 끊김(broken) 이벤트 제공



static int8_t current = -1;   // 현재 층 (0~2, 모르면 -1)
static int8_t target  = -1;   // 목표 층 (0~2, 없으면 -1)
static bool   moving  = false; // 이동 중인지 표시
static volatile uint8_t btn0 = 0, btn1 = 0, btn2 = 0; // 버튼 스냅샷(또는 edge 누적)
static void check_buttons(void);


void elevator_init(void)
{
    ADC1_StartIT();        // ADC1 변환을 인터럽트 모드로 시작.
                           // → 변환 완료 ISR에서 'adc_value'(volatile 전역 가정)가 갱신됨.

    stepper_init();        // 스테퍼 모듈 내부 상태 초기화(스텝 인덱스/주기/방향 등).
                           // GPIO 초기화는 CubeMX에서 이미 됐다고 가정.

    ps_init();             // 포토센서 모듈 초기화. 층 감지(빛 끊김) 이벤트 사용 준비.

    // 일시정지 타이머: 포토센서 끊김 시 '정지 유지 시간'을 비차단으로 잴 용도
    softTimer_Init(swTimerID1, 500);  // swTimerID1을 500ms 주기로 arm(지금부터 500ms 뒤 만료 의미가 일반적)

    // ★ 속도 모듈 초기화: ADC→주기(ms) 매핑 파라미터(보정/필터/램핑/반전)
    motor_speed_init(/*adc_min*/   100,   // ADC 하한(이하 컷 또는 스케일 기준)
                     /*adc_max*/   3300,  // ADC 상한(이상 컷 또는 스케일 기준)
                     /*min_ms*/    1,     // 스텝 간 최소 지연(가장 빠른 속도). 너무 작으면 토크 부족 가능
                     /*max_ms*/    10,    // 스텝 간 최대 지연(가장 느린 속도)
                     /*ramp_ms*/   1,     // 램핑 단위(ms). 속도 급변 억제(부드러운 가감속)
                     /*invert*/    true); // true면 ADC↑ → period_ms↓ (손잡이 올릴수록 더 빠르게)

    for (uint8_t floor = 0; floor < 3; floor++)
    {
			if (ps_is_blocked(floor))
			{
					current = floor;   // 해당 층 센서가 막혀 있으면 → 현재층으로 판단
					break;             // 한 층만 찾으면 충분하므로 반복문 종료
			}
    }
    target = -1;
    moving = false;

}

static void check_buttons(void)
{
        // 버튼 입력 폴링(예: 짧은 펄스 입력을 놓치지 않도록 루프마다 읽기)
  btn0 = buttonGetPressed(0);
  btn1 = buttonGetPressed(1);
  btn2 = buttonGetPressed(2);

  if (btn0) target = 0;  // 0층 버튼
  if (btn1) target = 1;  // 1층 버튼
  if (btn2) target = 2;  // 2층 버튼

  if (target != -1 && target != current)
  {
      // 방향 결정
      if (target > current)
          stepper_set_dir(DIR_CW);    // 위로 이동
      else
          stepper_set_dir(DIR_CCW);   // 아래로 이동

      stepper_resume();               // 스테퍼 모터 회전 시작
      moving = true;                  // 이동 중 상태로 표시

      printf("[MOVE] %d → %d\n", current, target);
  }
  stepper_task();
}

static void check_sensors(void)
{
	uint8_t floor = ps_any_broken();

    if (moving && target != -1)
    {
        // 목표층 센서가 "방금 끊김" 감지되면 도착
        if (floor != 0xFF)
        {
            stepper_stop();      // 스테퍼 즉시 정지
            softTimer_Reset(swTimerID1);
            current = target;    // 현재 층 갱신
            target = -1;         // 목표층 초기화
            moving = false;      // 이동 종료
            printf("[PS] broken at floor %u\r\n", current);
        }
    }
}

void elevator_loop(void)
{
    check_buttons();   // 버튼 입력 확인
    check_sensors();   // 센서 감지 확인

    // 1) (선택) 속도/ADC 상태를 1초마다 로그로 찍기
      static uint32_t prev = 0;                 // 마지막 로그 시각(ms)
      if (HAL_GetTick() - prev >= 1000)         // 1초 경과 확인(비차단)
      {
          prev = HAL_GetTick();                 // 기준 시각 갱신
      }

      // 3) 속도 갱신 + 스텝퍼에 적용(정지 상태가 아닐 때만)
      // motor_speed_update()는 adc_value(volatile 전역 가정)를 읽어
      // 보정/필터/램핑 후 주기(ms)를 산출해서 반환.
      uint32_t period_ms = motor_speed_update(adc_value);
      if (!moving)                       // 정지 중에는 속도 적용하지 않음
      {
          stepper_set_period_ms(period_ms); // 비차단 스텝 엔진의 주기 업데이트(속도 제어)
          stepper_set_dir(DIR_CW);          // 회전 방향 설정(여기서는 시계방향 고정)
      }
//    stepper_task();    // 논블로킹 스텝 처리
}
