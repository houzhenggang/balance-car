#ifndef __I2C_H
#define __I2C_H
#include "iom128v.h"

#define true 1
#define false 0 
#define   bool  uint8_t

#define TRUE  0
#define FALSE -1

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef char int8_t;

//0��ʾд
#define	I2C_DirectionTransmitter   0
//����ʾ��
#define	I2C_DirectionReceiver      1	 
/*====================================================================================================*/
/*====================================================================================================*/
void I2C_Config(void);
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
uint16_t i2cGetErrorCounter(void);
static void i2cUnstick(void);

int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

uint8_t MPU6050_ReadI2C(uint8_t SlaveAddress,uint8_t REG_Address);
void MPU6050_WriteI2C(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
short GetData(uint8_t SlaveAddress,uint8_t REG_Address);
void delay_us(unsigned int a);
void delay_ms(unsigned int a);

/*====================================================================================================*/
/*====================================================================================================*/
#endif
