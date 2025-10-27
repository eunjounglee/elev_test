/***** Elevator Controller (간결화 버전) ********************************************
 * 요구조건 유지:
 *  1) 2층 'UP', 3층 'DOWN' → 2층 먼저, 그 뒤 3층
 *  2) 2층 'DOWN', 3층 'DOWN' → 3층 먼저, 그 뒤 2층
 *
 * 변경점(간결화):
 *  - 상행/하행 분기 중복 제거 → '공통 헬퍼'로 판단 통일
 *  - target/pending 교체 시 '방향'도 함께 저장(동일)
 *  - pause/도착/버튼 폴링/FND 표시는 기존 동작 동일
 ***********************************************************************************/

#include "elevator.h"
#include "button.h"
#include "stepper.h"
#include "swTimer.h"
#include "adc.h"
#include "motor_speed.h"
#include "stdio.h"
#include "stdint.h"
#include "photosensor.h"
#include "led.h"    // FND 표시


/* ===== 내부 상태 ===== */
static int8_t current      = -1;   // 현재 층(0..2, 미확정: -1)
static int8_t target       = -1;   // 목적지 층(없으면 -1)
static int8_t pending      = -1;   // 경유 후 재개 목적지(없으면 -1). 1칸만
static int8_t fndfloor     = -1;   // FND 표시용 현재 감지 층(없으면 -1)
static bool   moving       = false;
static bool   pause_active = false;

/* 호출 방향 보관 */
static buttonUpDownCall target_dir  = 0; // 현재 target의 호출 방향
static buttonUpDownCall pending_dir = 0; // pending의 호출 방향

/* ===== 진행 방향 헬퍼 ===== */
static inline bool is_moving_up(void)   { return (target > current); }
static inline bool is_moving_down(void) { return (target < current); }

/* ===== 공통 헬퍼: 방향/위치 관계 계산 ===== */
// 진행부호: 상행(+1), 하행(-1), 정지(0)
static inline int move_sign(void)
{
    if (target > current) return +1;
    if (target < current) return -1;
    return 0;
}
// 같은 방향 호출인가?
static inline bool same_dir(buttonUpDownCall call_dir, buttonUpDownCall target_dir)
{
	bool is_same = (call_dir == target_dir);     // 방향이 같다
	bool call_valid = (call_dir != 0);           // 호출 방향이 유효하다
	bool target_valid = (target_dir != 0);       // 타깃 방향이 유효하다

	if (is_same && call_valid && target_valid)
	    return true;
	else
	    return false;
}
// 호출이 '경로 사이'에 있는가?
// (상행: current < call < target / 하행: target < call < current)
static inline bool is_between_on_route(int current, int target, int call)
{
    if (target > current)
    	return (current < call && call < target);
    if (target < current)
    	return (target < call && call < current);
    return false;
}
// 호출이 타깃을 '넘어선' 위치인가? (진행방향 쪽으로 더 멀리)
bool beyond_target_on_route(int current, int target, int call)
{
    if (target > current)       // 상행
      return (call > target);   // call > target이면 true, call ≤ target이면 false
    else if (target < current)  // 하행
      return (call < target);   // call < target이면 true, call ≥ target이면 false
    else                        // 정지 상태
        return false;
}

/* 버튼 (루프마다 갱신) */
static volatile uint8_t externalButtonOneFloorUp = 0,
												externalButtonTwoFloorDown = 0,
												externalButtonTwoFloorUp = 0,
												externalButtonThreeFloorDown = 0;

static volatile uint8_t insideButtonOneFloor = 0,
												insideButtonTwoFloor = 0,
												insideButtonThreeFloor = 0;

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
        if (ps_is_blocked(floor))
        {
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
// 엘리베이터를 출발시킬지 확인하고,
// 조건이 맞으면 모터를 회전시켜 이동을 시작하는 함수
static void start_move_if_needed(void)
{
    // ① 잠시 정지 중이면 출발 금지
    // 예: 경유층에 도착해서 500ms 동안 멈춰있는 중이라면
    if (pause_active)
    {
        // 아직 멈춰있어야 하므로 아무 것도 하지 않음
        return;
    }

    // ② 목적지(target)가 없거나, 이미 그 층이라면 출발하지 않음
    if (target == -1 || target == current)
    {
        // 갈 곳이 없거나 이미 도착했으니 종료
        return;
    }

    // ③ 이동 방향 결정
    if (target > current)
    {
        // 목표층이 현재보다 높으면 위로 이동
        stepper_set_dir(DIR_CW);   // CW = ClockWise (시계방향)
    }
    else
    {
        // 목표층이 현재보다 낮으면 아래로 이동
        stepper_set_dir(DIR_CCW);  // CCW = Counter ClockWise (반시계방향)
    }

    // ④ 모터 회전 시작
    stepper_resume();    // 정지 상태에서 다시 모터 동작 시작
    moving = true;       // "지금 이동 중" 상태로 표시

    // ⑤ 현재층 → 목표층 방향 출력 (디버그용)
    if (target > current)
        printf("[MOVE] %d → %d (UP)\n", current, target);
    else
        printf("[MOVE] %d → %d (DOWN)\n", current, target);

    // ⑥ 스텝 모터를 바로 한 번 구동 (즉시 반응성 향상)
    // stepper_task()는 실제 한 스텝 진행하는 함수로,
    // 바로 한 틱 실행시켜 "딜레이 없는 출발" 효과를 줌
    stepper_task();
}

/* =========================================================================
 * 도착 판정/처리: floor==target이면 도착
 *   - 경유가 있으면: 500ms 일시정지 시작 + 다음 목적지(target=pending)
 *   - 없으면: 완전 정지(target=-1)
 * ========================================================================= */
// 엘리베이터가 목표층에 도착했는지 확인하고,
// 도착 시 모터를 멈추거나, 다음 층으로 이동을 예약하는 함수
static void check_arrival(void)
{
    // ① 현재 어떤 층의 센서가 빛을 감지(차단)했는지 확인
    //    - 0, 1, 2 : 해당 층 센서 감지됨
    //    - 0xFF : 아무 층도 감지되지 않음
    uint8_t detected_floor = ps_any_broken();

    // ② 엘리베이터가 이동 중이 아닐 때는 도착을 확인할 필요가 없음
    //    (멈춰있거나 이동 목적이 없으면 그냥 종료)
    if (!moving || target == -1)
    {
        return;
    }

    // ③ 센서가 어떤 층을 감지했고, 그 층이 목표층이면 도착으로 판단
    if (detected_floor != 0xFF && detected_floor == (uint8_t)target)
    {
        // --- 도착 처리 시작 ---
        stepper_stop();      // 모터를 멈춤 (물리적 정지)
        moving = false;      // 이동 상태 해제
        current = target;    // 현재층을 목표층으로 갱신

        printf("[arrive] current floor: %d\n", current + 1);

        // ④ 다음 가야 할 층이 있는 경우 (예: 2층 → 3층 가는 중)
        if (pending != -1 && pending != target)
        {
            // ④-1) 500ms 동안 잠시 정지 (논블로킹 방식)
            softTimer_Reset(swTimerID1);  // 타이머 다시 시작
            pause_active = true;           // “정지 중” 상태 표시

            // ④-2) 다음 목적지 예약 (일시정지 끝나면 자동 출발)
            target     = pending;          // 다음 이동할 층 설정
            target_dir = pending_dir;      // 다음 방향 설정
            pending    = -1;               // 경유층 정보 초기화
            pending_dir= 0;

            printf("[next position] %d (dir: %s)\n",
                   target + 1,
                   (target_dir == BUTTON_CALL_UP ? "UP" : "DOWN"));

            // 일시정지 상태이므로 지금은 출발하지 않음
            return;
        }
        else
        {
            // ⑤ 경유층이 없을 때 → 완전 정지
            target = -1;       // 목표층 초기화
            target_dir = 0;    // 방향 초기화
            printf("[stop] NO ACTION.\n");
        }
    }
}

/* =========================================================================
 * 공통 처리: target/pending 교체를 한 곳에서
 * ========================================================================= */

// 현재 목표층(target)을 대기층(pending)으로 옮긴다
static inline void promote_to_pending(void)
{
    pending     = target;
    pending_dir = target_dir;
}

// 새로운 목표층과 방향을 설정한다
static inline void set_new_target(int floor, buttonUpDownCall direction)
{
    target     = (int8_t)floor;
    target_dir = direction;
}

/* =========================================================================
 * 외부 버튼 처리(간결화):
 * 규칙(둘 다 만족):
 *  - '경로 사이' & 동일 방향 호출 → 경유 (기존 규칙)
 *  - '타깃을 넘어서' & (새 호출 방향 == 현재 target 방향) → 타깃 확장
 *  - 그 외는 pending 1칸에 적재(비었을 때)
 * ========================================================================= */
// 외부 버튼 call 을 '경유 / 확장 / 대기' 중 하나로 분류해서 처리한다.
void handleExternalButtonCall(uint8_t floor, buttonUpDownCall call_dir)
{
    /* ------------------------------------------------------------------------
     * [CASE 0] 기본 입력 검증 및 불필요한 호출 무시
     * ------------------------------------------------------------------------ */

    // 유효 층(0..2)만 처리 (잘못된 인덱스 방지)
    if (floor > 2) return;

    // 엘리베이터가 완전히 정지 중이며 (이동 중 아님 + 일시정지도 아님)
    // 현재 층 버튼이 다시 눌렸을 경우 → 무시
    if ((moving == false) && (pause_active == false) && (floor == current))
    {
        // 이미 그 층에 있기 때문에 아무 동작도 하지 않음
        return;
    }

    /* ------------------------------------------------------------------------
     * [CASE 1] 정지 상태에서의 호출 (즉시 출발)
     * ------------------------------------------------------------------------
     * - 아직 이동 중이 아니거나, target이 없는 경우
     * - 새로 누른 층을 바로 목적지로 설정하고 이동 시작
     * ------------------------------------------------------------------------ */
    if (!moving || target == -1)
    {
        set_new_target(floor, call_dir);   // 새 target 층 + 방향 저장
        start_move_if_needed();            // 즉시 이동 시작
        return;
    }

    /* ------------------------------------------------------------------------
     * [CASE 2] 이동 중일 때: 보조 변수 설정
     * ------------------------------------------------------------------------
     * - 현재 이동 방향을 파악해두면 이후 판단이 간결해짐
     * ------------------------------------------------------------------------ */

    const int move_dir = move_sign();  // +1=상행, -1=하행, 0=정지(이론상 없음)
    const bool going_up   = (move_dir > 0);
    const bool going_down = (move_dir < 0);

    // 혹시 move_dir이 0이라면 (안전장치용)
    if (!going_up && !going_down)
    {
        set_new_target(floor, call_dir);
        start_move_if_needed();
        return;
    }

    // 이동 경로 관련 판단 변수들
    const bool on_the_way  = is_between_on_route(current, target, floor);   // '경로 사이'인가?
    const bool beyond_target = beyond_target_on_route(current, target, floor);// '타깃 넘어섰나?'
    const bool same_dir_ok = same_dir(call_dir, target_dir);                // 새 호출 방향 == 현재 타깃 방향?

    /* ------------------------------------------------------------------------
     * [CASE 3] 경유 호출 처리 (같은 방향 + 이동 경로 안쪽)
     * ------------------------------------------------------------------------
     * 조건:
     *   - on_the_way == true  → 현재층과 목표층 사이의 중간층
     *   - same_travel_dir == true → 진행 방향과 호출 방향이 일치
     * 동작:
     *   - 현재 target을 pending으로 옮기고, 새 호출층을 target으로 교체
     *   - 즉시 경유층으로 향함
     * ------------------------------------------------------------------------ */
    const bool same_travel_dir =
        (going_up   && call_dir == BUTTON_CALL_UP) ||
        (going_down && call_dir == BUTTON_CALL_DOWN);

    if (on_the_way && same_travel_dir)
    {
        // ① 지금 가던 목표층을 나중으로 미룸 (pending)
        promote_to_pending();

        // ② 새로 호출된 층을 새로운 목적지로 설정
        set_new_target(floor, call_dir);

        // ③ (정지 상태가 아니라면) 즉시 이동 재개
        start_move_if_needed();

        // ④ 다른 조건은 볼 필요 없음 → 종료
        return;
    }

    /* ------------------------------------------------------------------------
     * [CASE 4] 타깃 확장 (현재 타깃을 넘어서는 같은 방향 호출)
     * ------------------------------------------------------------------------
     * 조건:
     *   - beyond_target == true → 새 호출층이 기존 target보다 멀리 있음
     *   - same_dir_ok == true → 새 호출 방향이 기존 target 방향과 같음
     *   - pending == -1 → 대기 중 호출이 없음
     * 동작:
     *   - 기존 target을 pending으로 저장 후, 새 호출층으로 target 갱신
     *   - 방향이 다르거나 pending이 차 있으면 단순히 pending에 대기 저장
     * ------------------------------------------------------------------------ */
    if (beyond_target)
    {
        if (same_dir_ok && pending == -1)
        {
            // (A) 같은 방향이면 확장 허용 → 더 먼 층으로 목적지 확장
            promote_to_pending();          // 기존 target을 pending으로
            set_new_target(floor, call_dir); // 새 호출층을 새 target으로
            start_move_if_needed();          // 바로 이동 재개
        }
        else
        {
            // (B) 확장 불가: pending이 비어 있고 중복이 아닐 때 대기 저장
            const bool can_queue =
                (pending == -1) &&
                (floor != (uint8_t)target) &&
                (floor != (uint8_t)current);

            if (can_queue)
            {
                pending     = (int8_t)floor;
                pending_dir = call_dir;
            }
        }
        return;
    }
    /* ------------------------------------------------------------------------
     * [CASE 5] 기타 (경로 밖 + 다른 방향 호출)
     * ------------------------------------------------------------------------
     * 조건:
     *   - 위 모든 조건에 해당하지 않음
     *   - pending이 비어 있고, 현재층/타깃층과 중복되지 않음
     * 동작:
     *   - 대기용으로 pending에 1건 저장 (이후 도착 후 처리)
     * ------------------------------------------------------------------------ */
    const bool can_queue =
        (pending == -1) &&
        (floor != (uint8_t)target);

    if (can_queue)
    {
        pending     = (int8_t)floor;
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
 * 버튼 폴링
 * ========================================================================= */
void read_external_buttons(void)
{
	externalButtonOneFloorUp = buttonGetPressed(0);
	externalButtonTwoFloorDown = buttonGetPressed(1);
	externalButtonTwoFloorUp = buttonGetPressed(2);
	externalButtonThreeFloorDown = buttonGetPressed(3);

	if (externalButtonOneFloorUp) handleExternalButtonCall(0, BUTTON_CALL_UP);
	if (externalButtonTwoFloorDown) handleExternalButtonCall(1, BUTTON_CALL_DOWN);
	if (externalButtonTwoFloorUp) handleExternalButtonCall(1, BUTTON_CALL_UP);
	if (externalButtonThreeFloorDown) handleExternalButtonCall(2, BUTTON_CALL_DOWN);
}

/* 내부 버튼 폴링: 여기서는 buttonGetPressed(4/5/6)로 가정 */
void read_inside_buttons(void)
{
	insideButtonOneFloor = buttonGetPressed(4);
	insideButtonTwoFloor = buttonGetPressed(5);
  insideButtonThreeFloor = buttonGetPressed(6);

	if (insideButtonOneFloor) handleInsideButton(0); // 내부 0층(1F)
	if (insideButtonTwoFloor) handleInsideButton(1); // 내부 1층(2F)
	if (insideButtonThreeFloor) handleInsideButton(2); // 내부 2층(3F)
}

/* 내부 버튼 처리기:
 *  - 내부 버튼은 "현재 위치 기준 방향"을 계산해(UP/DOWN) 외부 호출과 동일 규칙을 적용.
 *  - 체류 중(pause_active=1) 내부 버튼은 최우선: 타이머 만료 후 곧장 그 층으로 출발.
 */
void handleInsideButton(uint8_t floor)
{
    /* --------------------------------------------------------------------
     * [CASE 0] 입력값 1차 검증: 유효한 층인가? (0~2만 허용)
     *  - floor > 2 이면 잘못된 입력 → 즉시 무시
     *  - 예) floor=3/4 등 범위 밖 값
     * -------------------------------------------------------------------- */
    if (floor > 2) return; // 유효층 범위 검사

    /* --------------------------------------------------------------------
     * [CASE 1] "완전 정지" 상태에서 같은 층 버튼이 눌린 경우 무시
     *
     * 조건:
     *   - isStopped   : 모터가 돌지 않는다(moving == false)
     *   - isNotPaused : 500ms 체류 중이 아니다(pause_active == false)
     *   - isSameFloor : 현재층과 요청층이 같다(floor == current)
     *
     * 이유:
     *   - 이미 그 층에 "완전히" 서 있으므로, 더 할 일이 없다.
     * 예)
     *   - current=1, moving=false, pause_active=false, floor=1(=같은 층)
     * -------------------------------------------------------------------- */
    bool isStopped   = (moving == false);       // 현재 모터가 돌고 있지 않음
    bool isNotPaused = (pause_active == false); // 일시정지(500ms) 상태가 아님
    bool isSameFloor = (floor == current);      // 이미 그 층에 서 있음

    if (isStopped && isNotPaused && isSameFloor)
    {
        // 이미 그 층에 완전히 멈춰 있으니 아무 것도 하지 않음
        return;
    }

    /* --------------------------------------------------------------------
     * [CASE 2] 호출 방향 계산: 내부 버튼은 '현재 위치 기준'으로 방향 결정
     *
     * 규칙:
     *   - 호출층이 현재층보다 위  → BUTTON_CALL_UP
     *   - 호출층이 현재층보다 아래→ BUTTON_CALL_DOWN
     *
     * 주의:
     *   - floor == current 인 경우는 [CASE 1]에서 걸러졌으므로 여기선 > 또는 < 만 남음
     * -------------------------------------------------------------------- */
    buttonUpDownCall call_dir;
    if (floor > current)
    {
        call_dir = BUTTON_CALL_UP;     // 호출층이 더 위쪽이면 → 위로 가는 호출
    }
    else
    {
        call_dir = BUTTON_CALL_DOWN;   // 호출층이 더 아래쪽이면 → 아래로 가는 호출
    }

    /* --------------------------------------------------------------------
     * [CASE 3] 체류(pause_active) 중 내부 호출은 '최우선 예약'
     *
     * 상황:
     *   - 방금 경유 도착하여 500ms 동안 정지(pause_active==true) 중임
     *
     * 동작:
     *   - 다음 출발 목적지(target)를 '즉시' 이 층으로 교체 (pending은 보존)
     *   - 실제 출발은 타이머 만료 후(=체류 종료 시점)에 이루어짐
     *
     * 예)
     *   - 1F→2F 가는 중 2F에서 잠시 정지, 이때 내부에서 3F 버튼 누르면
     *     → 500ms 뒤 3F로 출발하도록 예약(target=3)
     * -------------------------------------------------------------------- */
    if (pause_active)
    {
        target     = (int8_t)floor;   // 체류 종료 후 이 층으로 출발
        target_dir = call_dir;
        // pending은 보존(경유 후 이어갈 목적지를 유지)
        return;                       // 실제 출발은 타이머 만료 시점에 수행
    }

    /* --------------------------------------------------------------------
     * [CASE 4] 정지 상태(=모터가 서 있거나, 목표층 미설정)면 즉시 출발
     *
     * 조건:
     *   - !moving : 모터가 서 있음
     *   - 또는 target == -1 : 목표층이 아직 없음
     *
     * 동작:
     *   - target/target_dir를 설정하고 바로 start_move_if_needed() 호출
     *
     * 예)
     *   - 아무 데도 안 가는 상태에서 내부 2F 버튼 → 곧바로 2F로 출발
     * -------------------------------------------------------------------- */
    if (!moving || target == -1) {
        target     = (int8_t)floor;
        target_dir = call_dir;
        start_move_if_needed();
        return;
    }

    /* --------------------------------------------------------------------
     * [CASE 5] (이제부턴 "이동 중") 외부 호출 규칙과 동일 로직 적용
     *   - 상행/하행 각각에 대해 "경유(Between)"/ "확장(Beyond)"을 분리 처리
     * -------------------------------------------------------------------- */

    if (is_moving_up())
    {
        /* ================================================================
         * [CASE 5-1] 상행 중 "경유": current < floor < target && call_dir==UP
         *
         * 의미:
         *   - 지금 1F→3F로 올라가는 중인데,
         *     중간층(2F)에서 UP 호출이 들어오면 2F를 '먼저' 들른다.
         *
         * 동작:
         *   ① 지금 가던 target(예: 3F)을 pending으로 밀어둠
         *   ② 새로 호출된 floor(예: 2F)를 target으로 교체
         *   ③ 바로 경유층으로 향해 이동 재개
         * ================================================================ */
        if ((current < (int8_t)floor) &&
            ((int8_t)floor < target) &&
            (call_dir == BUTTON_CALL_UP))
        {
            pending     = target;
            pending_dir = target_dir;

            target     = (int8_t)floor;
            target_dir = call_dir;

            start_move_if_needed();
            return;
        }

        /* ================================================================
         * [CASE 5-2] 상행 중 "확장": floor > target
         *
         * 의미:
         *   - 지금 1F→2F로 올라가는데, 3F가 눌림(더 먼 위쪽)
         *
         * 분기:
         *   (A) call_dir == target_dir && pending == -1
         *       → '같은 방향'이고 '대기칸이 비어있으면' 확장 허용
         *       → 기존 target(2F)은 pending으로, 새 floor(3F)를 target으로
         *
         *   (B) 그 외(방향 다르거나 pending 차있음)
         *       → 당장은 확장 금지. pending이 비고 중복 아니면 pending에 대기
         * ================================================================ */
        if ((uint8_t)floor > (uint8_t)target)
        {
            if (call_dir == target_dir && pending == -1)
            {
                pending     = target;
                pending_dir = target_dir;

                target     = (int8_t)floor;
                target_dir = call_dir;

                start_move_if_needed();
            }
            else
            {
                // 확장 불가 시: 중복/현재/타깃과 다르고 pending이 비어있다면 대기 저장
                if (pending == -1 &&
                    floor != (uint8_t)target &&
                    floor != (uint8_t)current)
                {
                    pending     = (int8_t)floor;
                    pending_dir = call_dir;
                }
            }
            return;
        }
    }
    else if (is_moving_down())
    {
        /* ================================================================
         * [CASE 5-3] 하행 중 "경유": target < floor < current && call_dir==DOWN
         *
         * 의미:
         *   - 지금 3F→1F로 내려가는데,
         *     중간층(2F)에서 DOWN 호출이 들어오면 2F를 '먼저' 들른다.
         *
         * 동작:
         *   ① 지금 가던 target(예: 1F)을 pending으로 밀어둠
         *   ② 새로 호출된 floor(예: 2F)를 target으로 교체
         *   ③ 바로 경유층으로 향해 이동 재개
         * ================================================================ */
        if ((target < (int8_t)floor) &&
            ((int8_t)floor < current) &&
            (call_dir == BUTTON_CALL_DOWN))
        {
            pending     = target;
            pending_dir = target_dir;

            target     = (int8_t)floor;
            target_dir = call_dir;

            start_move_if_needed();
            return;
        }

        /* ================================================================
         * [CASE 5-4] 하행 중 "확장": floor < target
         *
         * 의미:
         *   - 지금 3F→2F로 내려가는 중인데, 1F가 눌림(더 먼 아래쪽)
         *
         * 분기:
         *   (A) call_dir == target_dir && pending == -1
         *       → '같은 방향'이고 '대기칸 비었으면' 확장 허용
         *       → 기존 target(2F)은 pending으로, 새 floor(1F)를 target으로
         *
         *   (B) 그 외
         *       → 당장 확장 금지. pending이 비고 중복 아니면 pending에 대기
         * ================================================================ */
        if ((uint8_t)floor < (uint8_t)target)
        {
            if (call_dir == target_dir && pending == -1)
            {
                pending     = target;
                pending_dir = target_dir;

                target     = (int8_t)floor;
                target_dir = call_dir;

                start_move_if_needed();
            }
            else
            {
                if (pending == -1 &&
                    floor != (uint8_t)target &&
                    floor != (uint8_t)current)
                {
                    pending     = (int8_t)floor;
                    pending_dir = call_dir;
                }
            }
            return;
        }
    }

    /* --------------------------------------------------------------------
     * [CASE 6] 그 외 일반 케이스: pending 한 칸에만 적재(중복/현재/타깃 동일은 무시)
     *
     * 의미:
     *   - 경유/확장 어느 조건에도 해당하지 않지만,
     *     나중에 처리할 가치가 있는 호출 → pending으로 '하나만' 보관
     *
     * 조건:
     *   - pending이 비어 있고
     *   - 현재층/target과 중복되지 않을 때
     *
     * 예)
     *   - 여러 호출이 동시에 들어오지만 설계 단순화를 위해 1건만 저장
     * -------------------------------------------------------------------- */
    if (pending == -1 &&
        floor != (uint8_t)target)
    {
        pending     = (int8_t)floor;
        pending_dir = call_dir;
    }
}
/* =========================================================================
 * 주 루프
 * ========================================================================= */
void elevator_loop(void)
{
    read_external_buttons();  // 외부(홀) 버튼을 읽어 호출 처리
    read_inside_buttons();    // 내부(카) 버튼을 읽어 호출 처리
    check_arrival();
    stepper_task();
    read_fnd_floor();

    // 일시정지 해제(500ms 타이머 만료 시 ‘한 번’만 출발 시도)
    if (pause_active && softTimer_IsTimeOut(swTimerID1))
    {
        pause_active = false;

        if (target != current)
        {
            start_move_if_needed();  // 경유 재출발
        }
        else
        {
            target = -1;             // 최종 정지
            target_dir = 0;
            moving = false;
        }
    }

    // (옵션) 1초마다 상태 로그
    static uint32_t prev = 0;
    if (HAL_GetTick() - prev >= 1000)
    {
        prev = HAL_GetTick();
        // printf("[STAT] cur=%d tgt=%d pend=%d mov=%d pause=%d dir=%d adc=%u\n",
        //        current, target, pending, moving, pause_active, target_dir, (unsigned)adc_value);
    }

    // 정지 중 속도 갱신(다음 출발 속도)
    uint32_t period_ms = motor_speed_update(adc_value);
    if (!moving)
    {
        stepper_set_period_ms(period_ms);
        stepper_set_dir(DIR_CW); // 의미 없는 기본값(원하면 삭제 가능)
    }
}
