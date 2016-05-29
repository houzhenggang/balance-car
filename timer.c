#include "iom128v.h"
#include "timer.h"

//T3��ʱ����ʼ�������ڶ�ʱ
//1��Ƶ,����ʱʱ��65535*(1/16000000)*1000 = 4.0959375ms
//��ʱ1ms����ֵΪ 65535*((4.0959375-1)/4.0959375) = 49535
void timer3_init(void)
{
 	 TCCR3A=0X00;
	 TCCR3B=(1<<CS30);
	 TCNT3=49535;
	 ETIMSK|=(1<<TOIE3);
}