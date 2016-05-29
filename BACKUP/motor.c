#include "iom128v.h"
#include "motor.h"
#include "stdio.h"

//电机A：PWM引脚PB5，方向控制引脚PC0，PC1
//电机B: PWM引脚PB6，方向控制引脚PC2，PC3
void motor_io_init(void)
{
	//PB5,6设置为输出,控制PWM
	DDRB |= (0x03<<5);
	
	//PC0,1,2,3设置为输出控制方向
	DDRC |= (0x0F);
	PORTC &= ~(0x0F);
}

//电机pwm初始化
void motor_pwm_init(void)
{
	OCR1A = 0;
	OCR1B = 0;
	
	TCCR1A=0xA9;
	TCCR1B=0x09;
}

//电机初始化
void motor_init(void)
{
 	motor_io_init();
	motor_pwm_init();
}

//电机A转动
void motor_A_run(int pwm)
{
 	if(pwm>=0)
	{
	 	OCR1A = pwm; 
		PORTC |= 0x02;
	}
	else
	{
		OCR1A = -pwm; 
		PORTC |= 0x01;
	}
}

//电机B转动
void motor_B_run(int pwm)
{
 	if(pwm>=0)
	{
	 	OCR1B = pwm; 
		PORTC |= 0x08;
	}
	else
	{
	    OCR1B = -pwm; 
		PORTC |= 0x04;
	}
}