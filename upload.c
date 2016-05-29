#include"upload.h"
#include"usart.h"

//����ͨ������1�ϴ���λ����������

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportIMU(s16 yaw,s16 pitch,s16 roll
				,s16 alt,s16 tempr,s16 press)
*��������:		����λ�����;�����������̬����
���������
		s16 yaw ���������ĺ���Ƕȡ���λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
		s16 pitch ����õ��ĸ����Ƕȣ���λ 0.1�ȡ�-900 - 900 ��Ӧ -90.0 -> 90.0 ��
		s16 roll  �����õ��ĺ���Ƕȣ���λ0.1�ȡ� -1800 -> 1800 ��Ӧ -180.0  ->  180.0��
		s16 alt   ��ѹ�߶ȡ� ��λ0.1�ס�  ��Χһ�����ͱ���
		s16 tempr �¶� �� ��λ0.1���϶�   ��Χ��ֱ����ĵ�·�岻����������
		s16 press ��ѹѹ������λ10Pa  һ������ѹǿ��101300pa ����Ѿ�����һ�����͵ķ�Χ����Ҫ����10�ٷ�����λ��
		s16 IMUpersec  ��̬�������ʡ�����IMUpersecÿ�롣
���������û��	
*******************************************************************************/
void UART0_ReportIMU(short int yaw,short int pitch,short int roll,short int alt,short int tempr,short int press)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	send_char(0xa5);
	send_char(0x5a);
	send_char(14+2);
	send_char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	send_char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	send_char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	send_char(ctemp);
	temp+=ctemp;
   	 
	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	send_char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	send_char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=press;
	send_char(ctemp);
	temp+=ctemp;

	send_char(temp%256);
	send_char(0xaa);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
*��������:		����λ�����͵�ǰ�����������ֵ
���������
	int16_t ax  ���ٶ� X��ADC��� ��Χ ��һ���з�������
	int16_t ay  ���ٶ� Y��ADC��� ��Χ ��һ���з�������
	int16_t az  ���ٶ� Z��ADC��� ��Χ ��һ���з�������
	int16_t gx  ������ X��ADC��� ��Χ ��һ���з�������
	int16_t gy  ������ Y��ADC��� ��Χ ��һ���з�������
	int16_t gz  ������ Z��ADC��� ��Χ ��һ���з�������
	int16_t hx  ������ X��ADC��� ��Χ ��һ���з�������
	int16_t hy  ������ Y��ADC��� ��Χ ��һ���з�������
	int16_t hz  ������ Z��ADC��� ��Χ ��һ���з�������
	
���������û��	
*******************************************************************************/
void UART0_ReportMotion(int ax,int ay,int az,int gx,int gy,int gz,int hx,int hy,int hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	send_char(0xa5);
	send_char(0x5a);
	send_char(14+8);
	send_char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	send_char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	send_char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=az;
	send_char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	send_char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	send_char(ctemp);
	temp+=ctemp;

	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	send_char(ctemp);
	temp+=ctemp;
 
	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	send_char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	send_char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	send_char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	send_char(ctemp);
	temp+=ctemp;

	send_char(temp%256);
	send_char(0xaa);
}
