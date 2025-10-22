#include "photosensor.h"   // 포토센서 모듈의 외부 인터페이스(PS_FLOOR_COUNT, 프로토타입 등)를 선언한 헤더

// 개별 포토센서(층별) 핀 구성을 묶어 표현하기 위한 구조체
typedef struct {
  GPIO_TypeDef *port;      // 해당 센서가 연결된 GPIO 포트 주소 (예: GPIOA, GPIOB, GPIOC ...)
  uint16_t      pin;       // 해당 센서가 연결된 GPIO 핀 번호 (예: GPIO_PIN_5)
} ps_cfg_t;

// 층(=센서)마다 GPIO 포트/핀 매핑 테이블
// PS_FLOOR_COUNT는 photosensor.h에서 정의되어 있다고 가정
// 인덱스: 0→0층, 1→1층, 2→2층 ... (주석으로 층 번호를 병기)
static const ps_cfg_t s_cfg[PS_FLOOR_COUNT] = {
  {GPIOC, GPIO_PIN_2},   // 0층 센서 입력 핀: PC8
  {GPIOC, GPIO_PIN_1},   // 1층 센서 입력 핀: PC6
  {GPIOC, GPIO_PIN_0},   // 2층 센서 입력 핀: PC5
};

// ─────────────────────────────────────────────────────────────────────────────
// 전기적 레벨 정의
// ─────────────────────────────────────────────────────────────────────────────

// "빛이 막혔을 때"(=빔 차단 상태) GPIO 입력 레벨을 지정
// 사용 중인 포토인터럽터/리시버의 출력 극성, 풀업/풀다운 구성에 따라
// 차단 시 HIGH(SET)일 수도 있고, 차단 시 LOW(RESET)일 수도 있음.

#define PS_BLOCKED_LEVEL  GPIO_PIN_SET

// ─────────────────────────────────────────────────────────────────────────────
// 디바운스 파라미터
// ─────────────────────────────────────────────────────────────────────────────

// 간단한 소프트 디바운싱(락아웃) 시간(ms)
// EXTI 인터럽트가 물리적/전기적 노이즈 등으로 연속 발생하는 것을
// 일정 시간 동안 무시하여 이벤트 폭주를 완화
#define PS_DEBOUNCE_MS    2              // 2ms 이내 재발생은 무시(프로젝트에 맞게 조정 가능)

// ─────────────────────────────────────────────────────────────────────────────
// 내부 상태 (ISR와 메인 루프에서 공유되므로 volatile로 선언)
// ─────────────────────────────────────────────────────────────────────────────

static volatile uint8_t  s_latched_broken[PS_FLOOR_COUNT];  // "방금 끊김" 1회성 이벤트 래치(읽으면 0으로 클리어)
static volatile uint8_t  s_level_blocked[PS_FLOOR_COUNT];   // 현재 센서 레벨: 1=차단, 0=통과 (실시간 상태)
static volatile uint32_t s_last_tick[PS_FLOOR_COUNT];       // 디바운스용 마지막 이벤트 시각(ms, HAL_GetTick 기준)

// 모듈 초기화: 내부 상태 배열을 모두 초기 상태로 리셋
void ps_init(void)
{
  for (uint8_t i = 0; i < PS_FLOOR_COUNT; ++i) {
    s_latched_broken[i] = 0;   // 아직 "끊김" 이벤트 없음
    s_level_blocked[i]  = 0;   // 초기 상태는 통과(차단 아님)로 가정
    s_last_tick[i]      = 0;   // 마지막 이벤트 시각 초기화
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 유틸: EXTI 콜백이 넘겨주는 건 '핀 번호'이므로, 그 핀을 가진 floor 인덱스를 역으로 찾는다.
// (센서마다 포트가 같지 않아도 '핀 번호'만 매칭하면 된다. 포트가 다르면 동일 핀번호가
//  겹칠 수 있으니 실제 하드웨어에서는 '포트+핀' 쌍 매칭이 더 안전하지만,
//  여기서는 핀번호만으로 충분하다는 전제. 필요시 포트도 함께 비교하도록 확장 가능.)
// ─────────────────────────────────────────────────────────────────────────────
static int8_t ps_find_by_pin(uint16_t gpio_pin)
{
  for (uint8_t i = 0; i < PS_FLOOR_COUNT; ++i) {
    if (s_cfg[i].pin == gpio_pin)        // 매핑 테이블의 핀과 동일하면
      return (int8_t)i;                  // 해당 floor 인덱스를 반환
  }
  return -1;                             // 찾지 못하면 -1 (에러)
}

// ─────────────────────────────────────────────────────────────────────────────
// EXTI 인터럽트 처리 진입점(콜백)
// 외부에서 HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 안에서
// 이 함수(ps_on_exti(GPIO_Pin))를 호출해주는 패턴을 가정.
// ─────────────────────────────────────────────────────────────────────────────
void ps_on_exti(uint16_t gpio_pin)
{
  int8_t idx = ps_find_by_pin(gpio_pin); // 핀 번호로 floor 인덱스 찾기
  if (idx < 0) return;                   // 해당 없음(우리가 관리하는 핀이 아님) → 무시하고 복귀

  uint32_t now = HAL_GetTick();          // 현재 시스템 틱(ms) 확보(32비트, 약 49.7일마다 오버플로)
  if (now - s_last_tick[idx] < PS_DEBOUNCE_MS) {
    // 마지막 이벤트로부터 PS_DEBOUNCE_MS 미만이면 너무 촘촘한 인터럽트 → 노이즈로 보고 무시
    // (uint32_t 뺄셈은 오버플로 시에도 modulo 2^32로 동작하므로 일반적으로 안전.
    //  단, 오버플로 경계 근처에서의 짧은 간격 비교는 그대로 의도대로 작동한다.)
    return;
  }
  s_last_tick[idx] = now;                // 디바운싱 통과 → 이번 이벤트를 "마지막"으로 기록

  // 해당 센서의 현재 입력 레벨을 읽음 (포트/핀은 매핑 테이블에서 가져옴)
  GPIO_PinState lvl = HAL_GPIO_ReadPin(s_cfg[idx].port, s_cfg[idx].pin);

  // 지금 상태가 "차단인지" 판정(우리 모듈의 기준 레벨과 비교)
  // PS_BLOCKED_LEVEL==GPIO_PIN_SET면, 입력이 HIGH일 때 차단으로 본다.
  uint8_t is_blocked = (lvl == PS_BLOCKED_LEVEL) ? 1 : 0;

  // 현재 센서 레벨(차단/통과)을 내부 상태에 반영 (폴링 API ps_is_blocked가 즉시 이 값을 반환)
  s_level_blocked[idx] = is_blocked;

  // "방금 끊김" 1회성 이벤트 래치:
  // 차단 레벨로 '변경되는 순간'만 1을 기록하여, 상위 로직에서 에지 방식으로 감지하게 함.
  // ※ 여기 구현은 "차단이면 1"로 간단히 처리하므로, 연속 차단 동안 인터럽트가 여러 번 들어오면
  //    여러 번 래치될 수 있음. '변화(엣지)만'을 원하면 이전 레벨과 비교하여 상승엣지에서만 세트하도록 확장 가능.
  if (is_blocked) {
    s_latched_broken[idx] = 1;           // 읽는 순간(ps_was_broken/ps_any_broken) 0으로 클리어됨
  }

  // (선택) 테스트용 LED 토글: 센서 핀별로 PA의 다른 핀을 토글하여 이벤트 확인
  // 실제 제품 코드에서는 비활성화(아래 #if 0 유지)
#if 0
  if (gpio_pin == GPIO_PIN_6)  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  if (gpio_pin == GPIO_PIN_8)  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
  if (gpio_pin == GPIO_PIN_9)  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// 현재 '차단(빔이 막힘)' 상태인지 즉시 상태를 반환하는 폴링 API
// floor: 0..PS_FLOOR_COUNT-1 유효 범위인지 확인 후 반환
// ─────────────────────────────────────────────────────────────────────────────
bool ps_is_blocked(uint8_t floor)
{
  if (floor >= PS_FLOOR_COUNT) return false;   // 범위 밖 요청은 안전하게 false 처리
  return s_level_blocked[floor] ? true : false;// 1이면 true, 0이면 false
}

// ─────────────────────────────────────────────────────────────────────────────
// "방금 끊김" 1회성 이벤트를 읽는 API (읽으면 해당 플래그를 클리어)
// - 사용처: 엘리베이터 제어에서 "층 통과 이벤트"로 처리하고 싶을 때 등
// ─────────────────────────────────────────────────────────────────────────────
bool ps_was_broken(uint8_t floor)
{
  if (floor >= PS_FLOOR_COUNT) return false;     // 범위 체크
  if (s_latched_broken[floor]) {                 // 래치가 세트되어 있으면
    s_latched_broken[floor] = 0;                 // 읽는 순간 클리어(1회성)
    return true;                                 // true로 '방금 끊김' 알림
  }
  return false;                                  // 없으면 false
}

// ─────────────────────────────────────────────────────────────────────────────
// 어느 층이든 "방금 끊김" 이벤트가 있는지 확인하고, 가장 낮은 인덱스를 반환
// - 있으면 해당 층 인덱스(0..), 없으면 0xFF
// - 읽는 순간 해당 래치들은 클리어하므로, 이벤트는 1회성으로 소비됨
// ─────────────────────────────────────────────────────────────────────────────

uint8_t ps_any_broken(void)
{
  for (uint8_t i = 0; i < PS_FLOOR_COUNT; ++i) {
    if (s_latched_broken[i]) {                 // 어떤 층이라도 래치가 세트되어 있으면
      s_latched_broken[i] = 0;                 // 읽는 즉시 클리어(중복 처리 방지)
      return i;                                // 그 층 인덱스 반환
    }
  }
  return 0xFF;                                 // 아무 이벤트도 없으면 0xFF
}
