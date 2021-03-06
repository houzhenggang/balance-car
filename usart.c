#include "iom128v.h"
#include "macros.h"
#include "usart.h"
#include "stdio.h"

void usart0_init(unsigned int band)//USART 初始化
{	
	//设置帧格式: 8 个数据位, 1个停止位
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01); 
	
	//U2X=0时的公式计算
    UBRR0L= (16000000/band/16-1)%256;
    UBRR0H= (16000000/band/16-1)/256;
	
	//使能接收中断，使能接收，使能发送
	UCSR0A = 0x00;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	
	//开总中断
	SEI();
}

// 数据发送
void send_char(unsigned char data )
{
     //等待发送缓冲器为空 
     while ( !( UCSR0A & (1<<UDRE0)) );
     // 将数据放入缓冲器，发送数据 
     UDR0 = data;
}

extern int putchar(char input)
{
  if(input== '\n')
  {  
   	while(!(UCSR0A & (1<<UDRE0)));
 	UDR0 = '\t';
  }   
   
  while(!(UCSR0A&(1<<UDRE0)));
  UDR0=input;
  return 1;
} 
