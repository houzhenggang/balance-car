#include "iom128v.h"
#include "timer.h"

//T3定时器初始化，用于定时
//1分频,最大计时时间65535*(1/16000000)*1000 = 4.0959375ms
//则定时1ms计数值为 65535*((4.0959375-1)/4.0959375) = 49535
void timer3_init(void)
{
 	 TCCR3A=0X00;
	 TCCR3B=(1<<CS30);
	 TCNT3=49535;
	 ETIMSK|=(1<<TOIE3);
}