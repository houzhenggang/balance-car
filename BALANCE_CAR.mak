CC = iccavr
LIB = ilibw
CFLAGS =  -IE:\大学课程\平衡小车设计\平衡小车程序final -e -D__ICC_VERSION=722 -DATMega128  -l -g -MLongJump -MHasMul -MEnhanced -Wf-use_elpm 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -g -e:0x20000 -ucrtatmega.o -bfunc_lit:0x8c.0x20000 -dram_end:0x10ff -bdata:0x100.0x10ff -dhwstk_size:30 -beeprom:0.4096 -fihx_coff -S2
FILES = main.o motor.o usart.o i2c.o inv_mpu.o inv_mpu_dmp_motion_driver.o mpu6050.o encoder.o upload.o pid.o timer.o 

BALANCE_CAR:	$(FILES)
	$(CC) -o BALANCE_CAR $(LFLAGS) @BALANCE_CAR.lk   -lfpatm128 -lcatm128
main.o: D:\installation_software\iccavr\include\iom128v.h D:\installation_software\iccavr\include\stdio.h D:\installation_software\iccavr\include\stdarg.h D:\installation_software\iccavr\include\_const.h D:\installation_software\iccavr\include\math.h D:\installation_software\iccavr\include\macros.h D:\installation_software\iccavr\include\AVRdef.h .\motor.h .\usart.h .\encoder.h .\mpu6050.h .\i2c.h .\pid.h
main.o:	main.c
	$(CC) -c $(CFLAGS) main.c
motor.o: D:\installation_software\iccavr\include\iom128v.h .\motor.h D:\installation_software\iccavr\include\stdio.h D:\installation_software\iccavr\include\stdarg.h D:\installation_software\iccavr\include\_const.h
motor.o:	motor.c
	$(CC) -c $(CFLAGS) motor.c
usart.o: D:\installation_software\iccavr\include\iom128v.h D:\installation_software\iccavr\include\macros.h D:\installation_software\iccavr\include\AVRdef.h .\usart.h D:\installation_software\iccavr\include\stdio.h D:\installation_software\iccavr\include\stdarg.h D:\installation_software\iccavr\include\_const.h
usart.o:	usart.c
	$(CC) -c $(CFLAGS) usart.c
i2c.o: .\i2c.h D:\installation_software\iccavr\include\iom128v.h
i2c.o:	i2c.c
	$(CC) -c $(CFLAGS) i2c.c
inv_mpu.o: D:\installation_software\iccavr\include\stdio.h D:\installation_software\iccavr\include\stdarg.h D:\installation_software\iccavr\include\_const.h D:\installation_software\iccavr\include\stdlib.h D:\installation_software\iccavr\include\limits.h D:\installation_software\iccavr\include\string.h D:\installation_software\iccavr\include\math.h .\inv_mpu.h .\i2c.h D:\installation_software\iccavr\include\iom128v.h
inv_mpu.o:	inv_mpu.c
	$(CC) -c $(CFLAGS) inv_mpu.c
inv_mpu_dmp_motion_driver.o: D:\installation_software\iccavr\include\iom128v.h D:\installation_software\iccavr\include\stdio.h D:\installation_software\iccavr\include\stdarg.h D:\installation_software\iccavr\include\_const.h D:\installation_software\iccavr\include\stdlib.h D:\installation_software\iccavr\include\limits.h D:\installation_software\iccavr\include\string.h D:\installation_software\iccavr\include\math.h .\inv_mpu.h .\inv_mpu_dmp_motion_driver.h .\dmpKey.h .\dmpmap.h
inv_mpu_dmp_motion_driver.o:	inv_mpu_dmp_motion_driver.c
	$(CC) -c $(CFLAGS) inv_mpu_dmp_motion_driver.c
mpu6050.o: .\mpu6050.h .\inv_mpu_dmp_motion_driver.h .\inv_mpu.h D:\installation_software\iccavr\include\stdio.h D:\installation_software\iccavr\include\stdarg.h D:\installation_software\iccavr\include\_const.h .\i2c.h D:\installation_software\iccavr\include\iom128v.h D:\installation_software\iccavr\include\math.h .\upload.h
mpu6050.o:	mpu6050.c
	$(CC) -c $(CFLAGS) mpu6050.c
encoder.o: D:\installation_software\iccavr\include\iom128v.h .\encoder.h
encoder.o:	encoder.c
	$(CC) -c $(CFLAGS) encoder.c
upload.o: .\upload.h .\usart.h
upload.o:	upload.c
	$(CC) -c $(CFLAGS) upload.c
pid.o: D:\installation_software\iccavr\include\iom128v.h .\pid.h
pid.o:	pid.c
	$(CC) -c $(CFLAGS) pid.c
timer.o: D:\installation_software\iccavr\include\iom128v.h .\timer.h
timer.o:	timer.c
	$(CC) -c $(CFLAGS) timer.c
