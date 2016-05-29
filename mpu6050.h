#ifndef __MPU6050_H
#define __MPU6050_H

#define DEFAULT_MPU_HZ  (200)


//定义不同测量范围的刻度因子
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f


unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
void run_self_test(void);
void MPU6050_Config(void);
void Get_Angle(float *angle, float *angle_dot);

#endif
