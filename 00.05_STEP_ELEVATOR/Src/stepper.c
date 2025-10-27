#include "stepper.h"

static const uint8_t HALF_STEP_SEQ[8][4] =
{
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

// 내부 상태
static bool     s_enabled   = true;
static uint8_t  s_dir       = DIR_CW;
static uint8_t  s_step_idx  = 0;
static uint32_t s_period_ms = 3;                 // 기본 3ms/half-step
static const swTimerID_tbl s_timer = swTimerID3; // 타이머 ID 충돌 시 바꿔주세요

static inline void stepMotor(uint8_t step)
{
    HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, HALF_STEP_SEQ[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, HALF_STEP_SEQ[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, HALF_STEP_SEQ[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, HALF_STEP_SEQ[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stepper_init(void)
{
    // GPIO는 Cube에서 설정했다고 가정
    softTimer_Init(s_timer, s_period_ms);  // ms 단위
}

void stepper_set_dir(uint8_t dir)
{
    s_dir = (dir == DIR_CCW) ? DIR_CCW : DIR_CW;	// 방향 세팅
}

void stepper_set_period_ms(uint32_t ms)
{
    if (ms < 1)  ms = 1;    // 너무 빠르면 토크 부족
    if (ms > 20) ms = 20;   // 과도한 지연 제한(필요시 조정)
    s_period_ms = ms;
    // 바로 반영하고 싶으면 다음 줄 주석 해제:
    // softTimer_Init(s_timer, s_period_ms);
}

void stepper_stop(void)
{
    s_enabled = false;
    // 필요시 코일 전류를 끄고 싶다면:
    // HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, GPIO_PIN_RESET);
}

void stepper_resume(void)
{
    s_enabled = true;
    softTimer_Init(s_timer, s_period_ms);
}

void stepper_task(void)
{
    if (!s_enabled) return;

    if (softTimer_IsTimeOut(s_timer))
    {
        // 한 스텝 수행
        stepMotor(s_step_idx);

        // 다음 인덱스
        if (s_dir == DIR_CW)
        {
            s_step_idx = (uint8_t)((s_step_idx + 1) & 0x7);
        } else
        {
            s_step_idx = (uint8_t)((s_step_idx + 7) & 0x7);
        }

        // 다음 주기 예약
        softTimer_Init(s_timer, s_period_ms);
    }
}

