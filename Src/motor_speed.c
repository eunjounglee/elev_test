#include "motor_speed.h"
#include "adc.h"      // adc_map_u32 사용
#include <stdint.h>

static int32_t  s_filt_adc = 0;
static uint16_t s_adc_min  = 100;
static uint16_t s_adc_max  = 3300;

static uint32_t s_min_ms   = 1;    // 최소 주기 (빠름)
static uint32_t s_max_ms   = 10;   // 최대 주기 (느림)
static uint32_t s_ramp_ms  = 1;    // 램프 1ms/틱
static bool     s_invert   = true; // ADC↑ -> 주기↓

static uint32_t s_curr_ms  = 6;    // 현재 적용 주기(ms), 시작값

void motor_speed_init(uint16_t adc_min, uint16_t adc_max,
                      uint32_t period_min_ms, uint32_t period_max_ms,
                      uint32_t ramp_step_ms, bool invert)
{
    s_adc_min = adc_min;
    s_adc_max = adc_max;
    if (s_adc_min >= s_adc_max) { s_adc_min = 0; s_adc_max = 4095; }

    s_min_ms  = (period_min_ms == 0) ? 1 : period_min_ms;
    s_max_ms  = (period_max_ms < s_min_ms) ? (s_min_ms + 1) : period_max_ms;

    s_ramp_ms = (ramp_step_ms == 0) ? 1 : ramp_step_ms;
    s_invert  = invert;

    s_filt_adc = 0;
    s_curr_ms  = (s_min_ms + s_max_ms) / 2;
}

uint32_t motor_speed_update(uint16_t adc_raw)
{
    // IIR 필터 (÷8)
    s_filt_adc += (((int32_t)adc_raw - s_filt_adc) >> 3);
    if (s_filt_adc < 0)     s_filt_adc = 0;
    if (s_filt_adc > 4095)  s_filt_adc = 4095;

    // 관측 구간으로 클램프
    uint16_t fa = (uint16_t)s_filt_adc;
    if (fa < s_adc_min) fa = s_adc_min;
    if (fa > s_adc_max) fa = s_adc_max;

    // 매핑: adc_min~adc_max -> s_min_ms~s_max_ms
    uint32_t target_ms = adc_map_u32(fa, s_adc_min, s_adc_max,
                                     s_min_ms, s_max_ms, s_invert);

    // 램프: s_curr_ms를 한 번에 s_ramp_ms만 변경
    if (s_curr_ms < target_ms) {
        uint32_t d = target_ms - s_curr_ms;
        s_curr_ms += (d > s_ramp_ms) ? s_ramp_ms : d;
    } else if (s_curr_ms > target_ms) {
        uint32_t d = s_curr_ms - target_ms;
        s_curr_ms -= (d > s_ramp_ms) ? s_ramp_ms : d;
    }
    return s_curr_ms;
}

uint32_t motor_speed_get_period_ms(void)
{
    return s_curr_ms;
}

int32_t motor_speed_get_filtered_adc(void)
{
    return s_filt_adc;
}
