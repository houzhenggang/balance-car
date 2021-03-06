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
	
	//10位快速pwm
	TCCR1A |= (1<<WGM11)|(1<<WGM10);
	TCCR1B |= (1<<WGM12);
	
	//pwm输出口
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);
	
	TCCR1B |= ((1<<CS11)|(1<<CS10));
}

void motor_init(void)
{
    motor_io_init();
	motor_pwm_init();
}

void PWM_Output(int PWM_L,int PWM_R)
{
	if (PWM_L<0)
	{
		PORTC &= ~0x03;
		PORTC |= 0x01;
		PWM_L *= -1; 
	}
	else
	{
		PORTC &= ~0x03;
		PORTC |= 0x02;
	}
	
	if (PWM_L>1000)
	{
		PWM_L=1000;
	}
	
	if (PWM_R<0)
	{ 
		PORTC &= ~0x0C;
		PORTC |= 0x04;
		PWM_R *= -1;
	}
	else
	{
		PORTC &= ~0x0C;
		PORTC |= 0x08;
	}
	
	if (PWM_R>1000)
	{
		PWM_R=1000;
	}
	
	OCR1AH=(PWM_L>>8);
	OCR1AL=PWM_L;			
	
	OCR1BH=(PWM_R>>8);
	OCR1BL=PWM_R;					
}