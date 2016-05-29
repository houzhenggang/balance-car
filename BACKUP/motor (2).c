#include "iom128v.h"
#include "motor.h"

/*
电机A：PWM引脚PB5，方向控制引脚PC0，PC1
电机B: PWM引脚PB6，方向控制引脚PC2，PC3
*/
void motor_io_init(void)
{
	//PE5,6设置为输出,控制PWM
	DDRE &= ~(0x03<<5);
	DDRE |= (0x03<<5);
	PORTE &= ~(0x03<<5)
	
	//PC0,1,2,3设置为输出控制方向
	DDRC &= ~(0x0F);
	DDRC |= (0x0F);
	PORTC &= ~(0x0F);
}

/*
8 位相位修正PWM

*/
void motor_pwm_init(void)
{
	OCR1A = 0;
	OCR1B = 0;
	
	//8 位相位修正PWM
	TCCR1A &= ~(0xF);
	TCCR1A |= (1<<WGM10);
	//OC1A(PB5) OC1B(PB6)输出PWM
	TCCR1A &= ~(0xF<<4);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);
	
	TCCR3B
}

