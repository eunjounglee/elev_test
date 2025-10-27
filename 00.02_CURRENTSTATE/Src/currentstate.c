

#include "currentstate.h"
#include "led.h"
#include "button.h"

void currentstate()
{
	segNumOff(8);

    while (1)
    {
    	if(buttonGetPressed(0)) // 배열의 인덱스를 넘기는것
    	{
    		segNumOn(1);
    	}
    	if(buttonGetPressed(1))
    	{
    		segNumOn(2);
    	}
    	if(buttonGetPressed(2))
    	{
    		segNumOn(3);
    	}
    }
}
