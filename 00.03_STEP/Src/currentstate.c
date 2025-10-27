

#include "currentstate.h"
#include "button.h"
#include "stepper.h"
#include "swTimer.h"

void currentstate()
{

  softTimer_Init(swTimerID2, 500); // 500ms
  bool paused = false;

  while (1)
	{
			// 버튼 눌림 읽기
			uint8_t btn0 = buttonGetPressed(0);
			uint8_t btn1 = buttonGetPressed(1);
			uint8_t btn2 = buttonGetPressed(2);

			// ▶ 버튼이 눌리면 잠시 정지 시작
			if (btn0 || btn1 || btn2)
			{
					paused = true;
					softTimer_Reset(swTimerID2);   // 정지 시간(500ms) 리셋
			}

			// ▶ 정지 상태
			if (paused)
			{
					rotateDegrees(0, DIR_CW);      // 모터 정지 (또는 motor_stop())

					// 타이머 만료되면 정지 해제
					if (softTimer_IsTimeOut(swTimerID2))
					{
							paused = false;            // 다시 동작 모드로 전환
					}
			}
			// ▶ 동작 상태
			else
			{
					rotateDegrees(100, DIR_CW);    // 모터 회전
			}
	}

}
