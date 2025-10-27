/*
 * swTimer.h
 *
 *  Created on: Oct 15, 2025
 *      Author: user13
 */

#ifndef INC_SWTIMER_H_
#define INC_SWTIMER_H_

#include "stdint.h"   // int 사용위함
#include "stdbool.h"  // bool 사용 (참, 거짓) 사용

#define TIMER_COUNT		4  // 사용가능한 소프트웨어 타이머 갯수

typedef enum
{
	swTimerID1 = 0,// 첫번째 타이머, 배열의 인덱스 때문에 0으로 잡아줌.
	swTimerID2,
	swTimerID3,
	swTimerID4,
	swTimerMAX
}swTimerID_tbl;

typedef struct
{
	bool		isRunning;		 // 타이머 동작 여부
	volatile bool timeOut; // 타이머 만료
	uint32_t setTime; 		 // 설정시간 (단위 : Tick -> ms)
	uint32_t time;				 // 현재 카운트된 시간
}swTimer_tbl;

// 외부에서 사용하기 위한 extern 선언
extern swTimer_tbl softTimer[TIMER_COUNT];

// 사용 할 함수 선언
void softTimer_ISR();			// 주기적으로 호출할 인터럽트 서비스 루틴
void softTimer_Init(swTimerID_tbl timerID, uint32_t time);	// 소프트타이머 초기화
bool softTimer_GetStatus(swTimerID_tbl timerID);						// 타이머 동작 상태 확인
bool softTimer_IsTimeOut(swTimerID_tbl timerID);						// 타이머 완료 여부 확인
void softTimer_Reset(swTimerID_tbl timerID);								// 타이머 리셋


#endif /* INC_SWTIMER_H_ */
