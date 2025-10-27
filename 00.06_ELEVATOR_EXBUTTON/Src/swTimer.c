
#include "swTimer.h"

// 소프트웨어 타이머 배열 선언

swTimer_tbl softTimer[TIMER_COUNT] = {0};

/**
 * 주기적으로 호출할 인터럽트 서비스 루틴
 * systick 핸들러나 타이머 인터럽트 에서 호출하면 됨.
 */

void softTimer_ISR()
{
	for(uint8_t i = 0 ; i < TIMER_COUNT ; i ++)
	{
		if(softTimer[i].isRunning == true)
		{
			if(softTimer[i].time >= softTimer[i].setTime)
			{
				softTimer[i].timeOut = true;		// 타이머 완료
			}
			else
			{
				softTimer[i].time ++;						// 시간 카운트 증가
			}
		}
		else
		{
			// 타이머가 동작중이 아니면 초기화
			softTimer[i].timeOut = false;
			softTimer[i].time = 0;
		}
	}
}


/**
 * 소프트웨어 타이머 초기화
 * Timer ID : 사용할 타이머 ID
 * time 		: 설정할 시간 (단위 : Tick, ms)
 */

void softTimer_Init(swTimerID_tbl timerID, uint32_t time)	// 소프트타이머 초기화
{
	if(timerID < TIMER_COUNT)
	{
		softTimer[timerID].time = 0;
		softTimer[timerID].isRunning = true;
		softTimer[timerID].setTime = time;
		softTimer[timerID].timeOut = false;
	}
}

/**
 *  소프트타이머 동작상태 반환
 *  반환값 : true 면 동작상태, false 면 장지싱테
 */

bool softTimer_GetStatus(swTimerID_tbl timerID)						// 타이머 동작 상태 확인
{
	if(timerID < TIMER_COUNT)
	{
		return softTimer[timerID].isRunning;
	}
	return false;
}

/**
 *  소프트타이머 만료여부 반환
 *  반환값 : true 면 만료, false 면 만료아님
 */

bool softTimer_IsTimeOut(swTimerID_tbl timerID)						// 타이머 완료 여부 확인
{
	if(timerID < TIMER_COUNT)
	{
		return softTimer[timerID].timeOut;
	}
	return false;
}

/**
 *  소프트타이머 재시작
 *  현재카운트 초기화
 */

void softTimer_Reset(swTimerID_tbl timerID)								// 타이머 리셋
{
	if(timerID < TIMER_COUNT)
	{
		softTimer[timerID].time = 0;
		softTimer[timerID].isRunning = true;
		softTimer[timerID].timeOut = false;
	}
}
