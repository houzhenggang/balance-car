#include "iom128v.h"
#include "motor.h"
#include "stdio.h"

//���A��PWM����PB5�������������PC0��PC1
//���B: PWM����PB6�������������PC2��PC3
void motor_io_init(void)
{
	//PB5,6����Ϊ���,����PWM
	DDRB |= (0x03<<5);
	
	//PC0,1,2,3����Ϊ������Ʒ���
	DDRC |= (0x0F);
	PORTC &= ~(0x0F);
}

//���pwm��ʼ��
void motor_pwm_init(void)
{
	OCR1A = 0;
	OCR1B = 0;
	
	TCCR1A=0xA9;
	TCCR1B=0x09;
}

//�����ʼ��
void motor_init(void)
{
 	motor_io_init();
	motor_pwm_init();
}

//���Aת��
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

//���Bת��
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