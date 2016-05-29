#include "iom128v.h"
#include "stdio.h"
#include "math.h"
#include "macros.h"
#include "motor.h"
#include "usart.h"
#include "encoder.h"
#include "mpu6050.h"
#include "i2c.h"
#include "pid.h"

//------------------------------------------------
//���ڱ���
//------------------------------------------------
#define  FARWARD  '1'
#define  BACKWARD '2'
#define  LEFT     '3'
#define  RIGHT    '4'
#define  STOP     '5'
#define  FIX      '6'
#define  WAIT     '7'
unsigned char rchar = WAIT;

//------------------------------------------------
//ѭ������
//------------------------------------------------
unsigned int loops = 0;
unsigned char loopsBit = 0;

//------------------------------------------------
//ƽ�����
//------------------------------------------------
unsigned int Kp_A=4000, Ki_A=300, Kd_A=4500;
PID_ANGLGE PID_angle;	
float angle_Targer;
float fix_angle;
float angle, angle_dot;
int PWM_Balance;
int PWM_L=0,PWM_R=0;

//----------------------------------- 0-------------
//ת������
//------------------------------------------------
unsigned char turn_flag = 0;
int PWM_Lturn=0;
int PWM_Rturn=0;

//------------------------------------------------
//�˶�����
//------------------------------------------------
int speed, distance;
float Ksp=6, Ksn=0.2;

unsigned char rec_buf[3][10];
unsigned char i=0 , j=0;
unsigned int pid_num;

int main(void)
{	
	usart0_init(9600);
	printf("\tusart0 init OK!\n");
	
	motor_init();
	
	encoder_init();
	
	MPU6050_Config();
	
	Get_Angle(&angle, &angle_dot);
	fix_angle = angle;
	
	PID_angle_init(&PID_angle, Kp_A, Ki_A, Kd_A, angle_Targer);	
	
	timer3_init();
	
	while(1)
	{	
		//--------------------------------------------------------------------
		//3msѭ��һ��
		//���нǶ�ƽ�⣬ʹС������վ��
		//--------------------------------------------------------------------
		if(loopsBit & 0x01)
		{	   
		   loopsBit &= ~(0x01);
		   
		   //��ȡ��ǰ�Ƕȣ����ٶ�
		   Get_Angle(&angle, &angle_dot);	
		   
		   //ͨ��λ��ʽPID�õ�С������Ƕ�Ϊ������Ҫ��PWM
	 	   PWM_Balance = PID_Proc(&PID_angle, angle, angle_dot);
		   
		   //�������ҵ��PWM���
	 	   PWM_L = (PWM_Balance + PWM_Lturn); 
	 	   PWM_R = (PWM_Balance + PWM_Rturn);	
		   
		   //���PWM
		   PWM_Output(PWM_L, PWM_R);  	   
		}
		
		//--------------------------------------------------------------------
		//10msѭ��һ��
		//�����ٶ�ƽ�⣬ʹС����������
		//--------------------------------------------------------------------
		if(loopsBit & 0x02)
		{
		   loopsBit &= ~(0x02);
		   
		   //��ȡ��ǰ�ٶȣ�λ��
		   Get_Speed(&speed, &distance);
		   
		   //ͨ���ٶȺ�λ�Ƶ���С��ƽ����Ҫ��PWM
		   PWM_Balance += Ksp*speed + Ksn*distance;
		   
		   //�������ҵ��PWM���
		   PWM_L = PWM_Balance; 
	 	   PWM_R = PWM_Balance;	
		   
		   //�ٳ�PWM
		   PWM_Output(PWM_L, PWM_R);
		   
		   //��������������
		   Clean_Speed();
		}
		
		//--------------------------------------------------------------------
		//50msѭ��һ��
		//����ת����ƣ�ת��һ���Ƕȿ���ͣ��
		//--------------------------------------------------------------------
		if(loopsBit & 0x04)
		{
		   loopsBit &= ~(0x04);
		   
		   //ת��1.5s��ֹͣת��
		   if(turn_flag<15)
		   {
		      turn_flag++;
		   }
		   else
		   {
		   	  turn_flag = 15;
		      PWM_Lturn = 0;
			  PWM_Rturn = 0;
		   }
		}
		
		//--------------------------------------------------------------------
		//100msѭ��һ��
		//���ڽ��գ����ո��ֿ���С��������
		//--------------------------------------------------------------------
		
		if(loopsBit & 0x08)
		{
	 	   loopsBit &= ~(0x08);
		   loops = 0;	   	
		   
		   //ǰ��
		   if(rchar == FARWARD)
   		   {
   	  	   	   //Ŀ�����ÿ�μ���0.005���ȣ���ǰ��
			   angle_Targer -= 0.005;
			   PID_angle.target = angle_Targer;
	  		   printf("angle_Targer:%f\n", angle_Targer);
   		   }
			
		   //����
   		   if(rchar == BACKWARD)
   		   {
     	       //Ŀ�����ÿ������0.005���ȣ�������
			   angle_Targer += 0.005;
			   PID_angle.target = angle_Targer;
	           printf("angle_Targer��%f\n", angle_Targer);
   		   }
		   
		   if(rchar == LEFT)
   		   {
			   //����PWM����,���ֲ���
			   PWM_Lturn = 150;
			   PWM_Rturn = 0;
			   turn_flag = 1;
   		   }
		   
		   if(rchar == RIGHT)
   		   {
			   //����PWM���ӣ����ֲ���
			   PWM_Lturn = 0;
			   PWM_Rturn = 150;
			   turn_flag = 1;
   		   }
		   
		   if(rchar == STOP)
   		   {
			   //ʹĿ����ǻص�ƽ��Ƕ�
			   angle_Targer = fix_angle;
			   PID_angle.target = angle_Targer;
			   printf("angle_Targer��%f\n", angle_Targer);
   		   }
		   
		   if(rchar == FIX)
		   {
		   	   //�趨ƽ���
			   fix_angle = angle;
		   }
		   
		   rchar = WAIT;
		   loops = 0;
		}
	}
	return 0;
}

//------------------------------------------------
//��ʱ��3�ж����,1ms�ж�һ��
//------------------------------------------------
#pragma interrupt_handler timer3_interrupt: 30
void timer3_interrupt(void) 
{    
	 TCNT3=49535;
	 loops++; 
	 
	 if(!(loops % 3))     {loopsBit |= 0x01;}  
	 if(!(loops % 10))    {loopsBit |= 0x02;} 
	 if(!(loops % 100))   {loopsBit |= 0x04;}   
	 if(!(loops % 1000))  {loopsBit |= 0x08;}  
}

//------------------------------------------------
//����0�����ж�
//------------------------------------------------
#pragma interrupt_handler uart_receive:19
void uart_receive(void)
{   
   rchar = UDR0;
   
   if(rchar == 'P')
   {
   	  memset(&rec_buf[0],'\0',sizeof(rec_buf[0]));
	  pid_num = 0;
	  i=0;
   }
   
   if(rchar == 'I')
   {
   	  memset(&rec_buf[1],'\0',sizeof(rec_buf[1]));
	  pid_num = 1;
	  i=0;
   }
   
   if(rchar == 'D')
   {
   	  memset(&rec_buf[2],'\0',sizeof(rec_buf[2]));
	  pid_num = 2;
	  i=0;
   }
   
   if(isdigit(rchar))
       rec_buf[pid_num][i++] = rchar;
	   
   if(rchar == 'E')
   {
	   Kp_A = atoi(rec_buf[0]);
	   Ki_A = atoi(rec_buf[1]);
	   Kd_A = atoi(rec_buf[2]);
	   
	   PID_angle.Kp = Kp_A;
	   PID_angle.Ki = Ki_A;
	   PID_angle.Kd = Kd_A;
	   
	   printf("P=%d, I=%d, D=%d\n", Kp_A,Ki_A,Kd_A);
   }
}