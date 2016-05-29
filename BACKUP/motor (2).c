#include "iom128v.h"
#include "motor.h"

/*
���A��PWM����PB5�������������PC0��PC1
���B: PWM����PB6�������������PC2��PC3
*/
void motor_io_init(void)
{
	//PE5,6����Ϊ���,����PWM
	DDRE &= ~(0x03<<5);
	DDRE |= (0x03<<5);
	PORTE &= ~(0x03<<5)
	
	//PC0,1,2,3����Ϊ������Ʒ���
	DDRC &= ~(0x0F);
	DDRC |= (0x0F);
	PORTC &= ~(0x0F);
}

/*
8 λ��λ����PWM

*/
void motor_pwm_init(void)
{
	OCR1A = 0;
	OCR1B = 0;
	
	//8 λ��λ����PWM
	TCCR1A &= ~(0xF);
	TCCR1A |= (1<<WGM10);
	//OC1A(PB5) OC1B(PB6)���PWM
	TCCR1A &= ~(0xF<<4);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);
	
	TCCR3B
}

