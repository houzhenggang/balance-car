#ifndef ___UPLOAD_H
#define ___UPLOAD_H

void UART0_ReportIMU(short int yaw,short int pitch,short int roll,short int alt,short int tempr,short int press);

void UART0_ReportMotion(int ax,int ay,int az,int gx,int gy,int gz,int hx,int hy,int hz);


#endif
