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
//串口变量
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
//循环变量
//------------------------------------------------
unsigned int loops = 0;
unsigned char loopsBit = 0;

//------------------------------------------------
//平衡变量
//------------------------------------------------
unsigned int Kp_A=4000, Ki_A=300, Kd_A=4500;
PID_ANGLGE PID_angle;	
float angle_Targer;
float fix_angle;
float angle, angle_dot;
int PWM_Balance;
int PWM_L=0,PWM_R=0;

//----------------------------------- 0-------------
//转动变量
//------------------------------------------------
unsigned char turn_flag = 0;
int PWM_Lturn=0;
int PWM_Rturn=0;

//------------------------------------------------
//运动变量
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
		//3ms循环一次
		//进行角度平衡，使小车可以站稳
		//--------------------------------------------------------------------
		if(loopsBit & 0x01)
		{	   
		   loopsBit &= ~(0x01);
		   
		   //获取当前角度，角速度
		   Get_Angle(&angle, &angle_dot);	
		   
		   //通过位置式PID得到小车保存角度为零所需要的PWM
	 	   PWM_Balance = PID_Proc(&PID_angle, angle, angle_dot);
		   
		   //调整左右电机PWM输出
	 	   PWM_L = (PWM_Balance + PWM_Lturn); 
	 	   PWM_R = (PWM_Balance + PWM_Rturn);	
		   
		   //输出PWM
		   PWM_Output(PWM_L, PWM_R);  	   
		}
		
		//--------------------------------------------------------------------
		//10ms循环一次
		//进行速度平衡，使小车可以行走
		//--------------------------------------------------------------------
		if(loopsBit & 0x02)
		{
		   loopsBit &= ~(0x02);
		   
		   //获取当前速度，位移
		   Get_Speed(&speed, &distance);
		   
		   //通过速度和位移调整小车平衡需要的PWM
		   PWM_Balance += Ksp*speed + Ksn*distance;
		   
		   //调整左右电机PWM输出
		   PWM_L = PWM_Balance; 
	 	   PWM_R = PWM_Balance;	
		   
		   //速出PWM
		   PWM_Output(PWM_L, PWM_R);
		   
		   //编码器计数归零
		   Clean_Speed();
		}
		
		//--------------------------------------------------------------------
		//50ms循环一次
		//进行转向控制，转动一定角度可以停下
		//--------------------------------------------------------------------
		if(loopsBit & 0x04)
		{
		   loopsBit &= ~(0x04);
		   
		   //转动1.5s后停止转动
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
		//100ms循环一次
		//串口接收，接收各种控制小车的命令
		//--------------------------------------------------------------------
		
		if(loopsBit & 0x08)
		{
	 	   loopsBit &= ~(0x08);
		   loops = 0;	   	
		   
		   //前进
		   if(rchar == FARWARD)
   		   {
   	  	   	   //目标倾角每次减少0.005弧度，往前倾
			   angle_Targer -= 0.005;
			   PID_angle.target = angle_Targer;
	  		   printf("angle_Targer:%f\n", angle_Targer);
   		   }
			
		   //后退
   		   if(rchar == BACKWARD)
   		   {
     	       //目标倾角每次增加0.005弧度，往后仰
			   angle_Targer += 0.005;
			   PID_angle.target = angle_Targer;
	           printf("angle_Targer：%f\n", angle_Targer);
   		   }
		   
		   if(rchar == LEFT)
   		   {
			   //左轮PWM增加,右轮不动
			   PWM_Lturn = 150;
			   PWM_Rturn = 0;
			   turn_flag = 1;
   		   }
		   
		   if(rchar == RIGHT)
   		   {
			   //右轮PWM增加，左轮不动
			   PWM_Lturn = 0;
			   PWM_Rturn = 150;
			   turn_flag = 1;
   		   }
		   
		   if(rchar == STOP)
   		   {
			   //使目标倾角回到平衡角度
			   angle_Targer = fix_angle;
			   PID_angle.target = angle_Targer;
			   printf("angle_Targer：%f\n", angle_Targer);
   		   }
		   
		   if(rchar == FIX)
		   {
		   	   //设定平衡角
			   fix_angle = angle;
		   }
		   
		   rchar = WAIT;
		   loops = 0;
		}
	}
	return 0;
}

//------------------------------------------------
//定时器3中断溢出,1ms中断一次
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
//串口0接收中断
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
