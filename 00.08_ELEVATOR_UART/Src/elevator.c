/***** Elevator Controller (3층/외부 4버튼, 타깃 확장=동일방향만 허용, 경유 일시정지) *****
 * 요구조건:
 *  1) 2층 'UP', 3층 'DOWN' → 2층 먼저 정지, 그 뒤 3층
 *  2) 2층 'DOWN', 3층 'DOWN' → 3층 먼저 정지, 그 뒤 2층
 *
 * 핵심 아이디어:
 *  - target과 함께 그 호출의 '방향'도 저장(target_dir).
 *  - 이동 중 '타깃 확장'은 반드시 "현재 target의 호출 방향과 동일할 때만" 허용.
 *  - 경로 '사이' 경유는 기존처럼 동일방향일 때만 허용.
 *  - pending에도 층과 방향을 함께 저장(pending_dir).
 *  - 경유 도착 시 500ms 논블로킹 일시정지 후 자동 재출발.
 ************************************************************************************************/

#include "elevator.h"
#include "button.h"
#include "stepper.h"
#include "swTimer.h"
#include "adc.h"
#include "motor_speed.h"
#include "stdio.h"
#include "stdint.h"
#include "photosensor.h"
#include "led.h"            // (선택) FND 표시용


/* ===== 내부 상태 ===== */
static int8_t current      = -1;   // 현재 층(0..2, 미확정: -1)
static int8_t target       = -1;   // 목적지 층(없으면 -1)
static int8_t pending      = -1;   // 경유 후 재개 목적지(없으면 -1). 1칸만 사용
static int8_t fndfloor      = -1;   // 현재 층(0..2, 미확정: -1)
static bool   moving       = false;      // 모터 실제 회전 중?
static bool   pause_active = false;      // 경유 도착 후 잠시 정지 중?

/* 호출 방향까지 함께 보관 */
static buttonUpDownCall target_dir  = 0; // 현재 target의 호출 방향(없음: 0)
static buttonUpDownCall pending_dir = 0; // pending의 호출 방향(없음: 0)

/* ===== 진행 방향 헬퍼 ===== */
static inline bool is_moving_up(void)   { return (target > current); }
static inline bool is_moving_down(void) { return (target < current); }

/* 버튼 스냅샷(루프마다 갱신) */
static volatile uint8_t btn0 = 0, btn1 = 0, btn2 = 0, btn3 = 0;

/* =========================================================================
 * 초기화
 * ========================================================================= */
void elevator_init(void)
{
    ADC1_StartIT();        // ADC IT 시작(완료 ISR이 adc_value 갱신)
    stepper_init();        // 스텝 상태/핀 초기화
    ps_init();             // 포토센서 초기화

    // 500ms 일시정지 타이머 준비(이후 재가동은 Reset 사용)
    softTimer_Init(swTimerID1, 500);

    // 속도 모듈(ADC→주기(ms)) 셋업
    motor_speed_init(/*adc_min*/   100,
                     /*adc_max*/   3300,
                     /*min_ms*/    1,
                     /*max_ms*/    10,
                     /*ramp_ms*/   1,
                     /*invert*/    true);

    // 부팅 시 ‘막혀있는’ 층을 찾아 현재층 추정
    for (uint8_t floor = 0; floor < 3; floor++)
    {
        if (ps_is_blocked(floor)) {
            current = floor;
            printf("[SET current by ps_is_blocked] %d\n", (int)current);
            break;
        }
    }

    target  = -1;  target_dir  = 0;
    pending = -1;  pending_dir = 0;
    moving  = false;
    pause_active = false;
    stepper_stop();
}

/* =========================================================================
 * 필요 시 이동 개시 (pause_active이면 출발 금지)
 * ========================================================================= */
static void start_move_if_needed(void)
{
    if (pause_active) return;                 // 잠시 정지 중엔 어떤 출발도 금지
    if (target == -1 || target == current) return;

    if (target > current) stepper_set_dir(DIR_CW);    // 위로
    else                  stepper_set_dir(DIR_CCW);   // 아래로

    stepper_resume();
    moving = true;
    printf("[MOVE] %d → %d (dir=%s)\n",
           current, target, (target > current) ? "UP" : "DOWN");

    stepper_task();     // 즉시 1틱 진행으로 반응성↑
}

/* =========================================================================
 * 도착 판정/처리: floor==target이면 도착
 *   - 경유가 있으면: 500ms 일시정지 시작 + 다음 목적지(target=pending)
 *   - 없으면: 완전 정지(target=-1)
 * ========================================================================= */
static void check_arrival(void)
{
    uint8_t floor = ps_any_broken();     // 방금 끊긴 층(없으면 0xFF 가정)

    if (moving && target != -1)
    {
        if (floor != 0xFF && floor == (uint8_t)target)
        {
            stepper_stop();
            moving = false;
            current = target;
            printf("[ARRIVED] %d\n", current);

            if (pending != -1)
            {
                // 500ms 잠시 정지
                softTimer_Reset(swTimerID1);
                pause_active = true;

                // 다음 목적지와 방향을 '미리' 걸어놓기(출발은 타이머 만료 시점)
                target     = pending;
                target_dir = pending_dir;
                pending    = -1;
                pending_dir= 0;

                printf("[after-pause target] %d (dir=%s)\n",
                       target, (target_dir==BUTTON_CALL_UP?"UP":"DOWN"));
                return; // 일시정지 중이므로 즉시 출발 금지
            }
            else
            {
                // 최종 정지
                target = -1;
                target_dir = 0;
            }
        }
    }
}

/* =========================================================================
 * 외부 버튼 처리(타깃 확장=동일방향만 허용 + 경유 규칙)
 *  - 공통 무시: (buttonCall > 2) / (!moving && buttonCall==current)
 *  - 정지 상태: target=buttonCall; target_dir=call_dir; 즉시 출발 시도
 *  - 이동 상태:
 *     [B] 경로 사이 + "호출 방향 일치" → 경유(기존 규칙)
 *     [A] 타깃 '확장'은 "현재 target_dir과 동일 방향"일 때만 허용
 *     [C] 그 외 → pending 비었으면 적재(층+방향)
 * ========================================================================= */
 void handleExternalButtonCall(uint8_t buttonCall, buttonUpDownCall call_dir)
{
    if (buttonCall > 2) return;                     // 층 범위 보호(0..2)

    // 이미 그 층에 '정지' 중이면 무시
    if (!moving && buttonCall == (uint8_t)current) return;

    // 정지 상태면 바로 target 설정 + 방향 저장
    if (!moving || target == -1)
    {
        target     = (int8_t)buttonCall;
        target_dir = call_dir;                      // ★ 호출 방향 기억
        start_move_if_needed();
        return;
    }

    // ===== 이동 중 =====
    if (is_moving_up())
    {
        // [B] 경로 '사이' + UP 호출 → 경유
        if ((current < (int8_t)buttonCall) &&
            ((int8_t)buttonCall < target) &&
            (call_dir == BUTTON_CALL_UP))
        {
            // 기존 target을 pending으로(방향 포함)
            pending     = target;
            pending_dir = target_dir;

            // 새 경유를 target으로 승격(방향 포함)
            target     = (int8_t)buttonCall;
            target_dir = call_dir;

            start_move_if_needed();
            return;
        }

        // [A] 타깃 '확장'은 "현재 target_dir과 동일 방향"일 때만 허용
        if (buttonCall > (uint8_t)target)
        {
            if (call_dir == target_dir && pending == -1)
            {
                pending     = target;
                pending_dir = target_dir;

                target     = (int8_t)buttonCall;
                target_dir = call_dir;

                start_move_if_needed();
            }
            else
            {
                // 방향 다르면 확장 금지 → pending에 대기(비었을 때만)
                if (pending == -1 &&
                    buttonCall != (uint8_t)target &&
                    buttonCall != (uint8_t)current)
                {
                    pending     = (int8_t)buttonCall;
                    pending_dir = call_dir;
                }
            }
            return;
        }
    }
    else if (is_moving_down())
    {
        // [B] 경로 '사이' + DOWN 호출 → 경유
        if ((target < (int8_t)buttonCall) &&
            ((int8_t)buttonCall < current) &&
            (call_dir == BUTTON_CALL_DOWN))
        {
            pending     = target;
            pending_dir = target_dir;

            target     = (int8_t)buttonCall;
            target_dir = call_dir;

            start_move_if_needed();
            return;
        }

        // [A] 타깃 '확장'은 "현재 target_dir과 동일 방향"일 때만 허용
        if (buttonCall < (uint8_t)target)
        {
            if (call_dir == target_dir && pending == -1)
            {
                pending     = target;
                pending_dir = target_dir;

                target     = (int8_t)buttonCall;
                target_dir = call_dir;

                start_move_if_needed();
            }
            else
            {
                if (pending == -1 &&
                    buttonCall != (uint8_t)target &&
                    buttonCall != (uint8_t)current)
                {
                    pending     = (int8_t)buttonCall;
                    pending_dir = call_dir;
                }
            }
            return;
        }
    }

    // [C] 그 외: 도착 후 처리용 pending 1칸에 적재(중복/현재/타깃 동일은 무시)
    if (pending == -1 &&
        buttonCall != (uint8_t)target &&
        buttonCall != (uint8_t)current)
    {
        pending     = (int8_t)buttonCall;
        pending_dir = call_dir;
    }
}

/* (선택) 현재층 FND 표시 */
void read_fnd_floor(void)
{
    // 1) 현재 막혀있는 센서 층을 확인
    for (int i = 0; i < 3; i++)
    {
        if (ps_is_blocked(i))  // i층 센서가 막혀 있다면
        {
        	fndfloor = i;
          break;
        }
    }
    // 2) FND 표시
    if (fndfloor == -1)
    {
        // 감지되는 층이 없을 때 (층 사이 이동 중)
        segNumOn(1);  // 예: 0 또는 "-" 표시
    }
    else
    {
        segNumOn(fndfloor + 1);  // FND는 1층부터 표시
    }
}

/* =========================================================================
 * 버튼 폴링: (예) btn0:0F UP, btn1:1F DOWN, btn2:1F UP, btn3:2F DOWN
 * ========================================================================= */
 void read_external_buttons(void)
{
    btn0 = buttonGetPressed(0);
    btn1 = buttonGetPressed(1);
    btn2 = buttonGetPressed(2);
    btn3 = buttonGetPressed(3);

    if (btn0) handleExternalButtonCall(0, BUTTON_CALL_UP);
    if (btn1) handleExternalButtonCall(1, BUTTON_CALL_DOWN);
    if (btn2) handleExternalButtonCall(1, BUTTON_CALL_UP);
    if (btn3) handleExternalButtonCall(2, BUTTON_CALL_DOWN);
}

/* =========================================================================
 * 주 루프: 버튼→도착 판정→스텝 진행→일시정지 해제→(옵션)로그/속도
 * ========================================================================= */
void elevator_loop(void)
{
    read_external_buttons();
    check_arrival();
    stepper_task();
    read_fnd_floor();

    // 일시정지 해제(500ms 타이머 만료 시 ‘한 번’만 출발 시도)
    if (pause_active && softTimer_IsTimeOut(swTimerID1))
    {
        pause_active = false;

        if (target != current) {
            start_move_if_needed();  // 경유 재출발
        } else {
            target = -1;             // 최종 정지
            target_dir = 0;
            moving = false;
        }
    }

    // (옵션) 1초마다 상태 로그
    static uint32_t prev = 0;
    if (HAL_GetTick() - prev >= 1000) {
        prev = HAL_GetTick();
        // printf("[STAT] cur=%d tgt=%d pend=%d mov=%d pause=%d dir=%d adc=%u\n",
        //        current, target, pending, moving, pause_active, target_dir, (unsigned)adc_value);
    }

    // 정지 중 속도 갱신(다음 출발 속도)
    uint32_t period_ms = motor_speed_update(adc_value);
    if (!moving) {
        stepper_set_period_ms(period_ms);
        stepper_set_dir(DIR_CW); // 의미 없는 기본값(원하면 삭제 가능)
    }
}
