NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        func_lit   008C   0002 =      2. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __func_lit_start
       008E  __func_lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008E   0284 =    644. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008E  __lit_start
       0138  __ctype
       0312  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   0312   0EE5 =   3813. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0312  __idata_start
       11F7  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   11F8   920A =  37386. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       08FC  __text_start
       08FC  __start
       091E  _exit
       091F  _main
       0B16  _timer3_interrupt
       0B73  _uart_receive
       0C6E  _motor_io_init
       0C78  _motor_pwm_init
       0C8B  _motor_init
       0C8D  _PWM_Output
       0CD5  _usart0_init
       0D43  _send_char
       0D47  _putchar$device_specific$
       0D53  _I2C_Config
       0D59  i2c.c:I2C_delay
       0D5B  _delay_us
       0D6E  _delay_ms
       0D81  i2c.c:I2C_Start
       0D92  i2c.c:I2C_Stop
       0D9A  i2c.c:I2C_Ack
       0DA2  i2c.c:I2C_NoAck
       0DAA  i2c.c:I2C_WaitAck
       0DB8  i2c.c:I2C_SendByte
       0DD1  i2c.c:I2C_ReceiveByte
       0DE9  _i2cWriteBuffer
       0E1B  _i2cwrite
       0E31  _i2cread
       0E47  _i2cWrite
       0E64  _i2cRead
       0E95  _i2cGetErrorCounter
       0E98  _MPU6050_ReadI2C
       0EAF  _MPU6050_WriteI2C
       0EC1  _GetData
       0ED2  inv_mpu.c:set_int_enable
       0F31  _mpu_reg_dump
       0F61  _mpu_read_reg
       0F8E  _mpu_init
       1088  _mpu_lp_accel_mode
       110B  _mpu_get_gyro_reg
       1147  _mpu_get_accel_reg
       1183  _mpu_get_temperature
       11FC  _mpu_set_accel_bias
       1336  _mpu_reset_fifo
       1487  _mpu_get_gyro_fsr
       14BE  _mpu_set_gyro_fsr
       1521  _mpu_get_accel_fsr
       1556  _mpu_set_accel_fsr
       15B6  _mpu_get_lpf
       160D  _mpu_set_lpf
       1665  _mpu_get_sample_rate
       1676  _mpu_set_sample_rate
       16DD  _mpu_get_gyro_sens
       1722  _mpu_get_accel_sens
       1762  _mpu_get_fifo_config
       1769  _mpu_configure_fifo
       17A7  _mpu_get_power_state
       17B5  _mpu_set_sensors
       1827  _mpu_get_int_status
       1854  _mpu_read_fifo
       19B8  _mpu_read_fifo_stream
       1A3C  _mpu_set_int_level
       1A41  _mpu_set_int_latched
       1A7E  inv_mpu.c:get_accel_prod_shift
       1B0D  inv_mpu.c:accel_self_test
       1BCE  inv_mpu.c:gyro_self_test
       1CCA  inv_mpu.c:get_st_biases
       20B0  _mpu_run_self_test
       215D  _mpu_write_mem
       21B9  _mpu_read_mem
       2215  _mpu_set_bypass
       22EF  _mpu_load_firmware
       2381  _mpu_set_dmp_state
       23D6  _mpu_get_dmp_state
       23DD  _mpu_lp_motion_interrupt
       252B  _get_ms
       252C  _dmp_load_motion_driver_firmware
       253F  _dmp_set_orientation
       263A  _dmp_set_gyro_bias
       2854  _dmp_set_accel_bias
       2A65  _dmp_set_fifo_rate
       2AB6  _dmp_get_fifo_rate
       2AC0  _dmp_set_tap_thresh
       2BF1  _dmp_set_tap_axes
       2C12  _dmp_set_tap_count
       2C2D  _dmp_set_tap_time
       2C4B  _dmp_set_tap_time_multi
       2C69  _dmp_set_shake_reject_thresh
       2CE3  _dmp_set_shake_reject_time
       2D02  _dmp_set_shake_reject_timeout
       2D21  _dmp_get_pedometer_step_count
       2D72  _dmp_set_pedometer_step_count
       2DC6  _dmp_get_pedometer_walk_time
       2E22  _dmp_set_pedometer_walk_time
       2E88  _dmp_enable_feature
       2F9A  _dmp_get_enabled_features
       2FA4  _dmp_enable_gyro_cal
       2FD9  _dmp_enable_lp_quat
       3001  _dmp_enable_6x_lp_quat
       3029  inv_mpu_dmp_motion_driver.c:decode_gesture
       3067  _dmp_set_interrupt_mode
       30A9  _dmp_read_fifo
       3327  _dmp_register_tap_cb
       332E  _dmp_register_android_orient_cb
       3335  _inv_row_2_scale
       336A  _inv_orientation_matrix_to_scalar
       338B  _run_self_test
       341C  _MPU6050_Config
       34B6  _Get_Angle
       35FE  _encoder_init
       3619  _Get_Speed
       366F  _Clean_Speed
       367A  _INT_T0
       369C  _INT_T1
       36BE  _UART0_ReportIMU
       37EC  _UART0_ReportMotion
       39A4  _PID_angle_init
       39D9  _PID_Proc
       3A33  _PID_speed_init
       3A85  _PID_Inc
       3B17  _timer3_init
       3B29  __print
       3F96  _memcmp
       3FA8  _memcpy
       3FB6  _memset
       3FC1  _strlen
       3FCD  push_arg4
       3FCF  push_arg2
       3FD2  asr32
       3FDE  mod16s
       3FE2  div16s
       3FFC  mod16u
       3FFE  div16u
       3FFF  xdiv16u
       4018  div32u
       401A  mod32u
       401E  div32s
       4020  mod32s
       404C  long_div_prolog
       405F  long_div_epilog
       406F  tst_R16_R19
       4074  tst_R24_R27
       4079  neg_R24_R27
       4082  mod8u
       4084  div8u
       4085  xdiv8u
       409A  elpm32
       40AE  empy16s
       40BE  empy32u
       40BE  empy32s
       40F2  pop_xgset003C
       40F7  pop_xgset00FC
       40FE  pop_xgset30FC
       4107  pop_xgsetF0FC
       4112  push_xgsetF0FC
       4114  push_xgset30FC
       4116  push_xgset00FC
       4118  push_xgset003C
       411D  push_xgset300C
       4122  pop_xgset300C
       4127  push_xgsetF000
       412C  pop_xgsetF000
       4131  push_xgsetF00C
       4138  pop_xgsetF00C
       413F  push_xgset303C
       4146  pop_xgset303C
       414D  push_xgsetF03C
       4156  pop_xgsetF03C
       415F  xicallx
       4168  neg32
       4171  long_prolog
       417F  long_epilog
       418A  tstzero1
       4190  tstzero2
       4196  lsl16
       419D  lsl32
       41A9  lsr16
       41B0  lsr32
       41BC  asgnblk
       41D4  fpsub
       41D6  fpadd
       41D7  fpadd_alt
       423E  fpadd1
       4248  fpadd1x
       4255  fpadd2
       4260  fpadd2x
       426E  fpsub1
       4278  fpsub2
       4283  fpsub1x
       4290  fpint
       42C2  uint2fp
       42C4  int2fp
       42CD  ulong2fp
       42CF  long2fp
       42E6  fpdiv1
       42EF  fpdiv2
       42F9  fpdiv1x
       4305  fpdiv2x
       4380  unpacks
       4388  tstR16_R19
       4390  unpacku
       43A9  normalize_and_pack
       43DE  pack
       43EE  saveFPRegs
       43FD  restoreFPRegs
       440C  fpmule
       445F  fpmule1
       4469  fpmule2
       4474  fpmule1x
       4481  fpmule2x
       448F  fpcmp
       449C  fpcmp1
       44A7  fpcmp2
       44B3  fpneg
       44BB  fpneg2
       44C2  _cstrcpy
       44CC  _cstrlen
       44D9  _atoi
       451F  _itoa
       458A  _ltoa
       4642  _ultoa
       46A9  _utoa
       46E9  _ftoa
       4878  _isdigit
       4882  _islower
       488C  _isspace
       4896  _toupper
       48A9  _satan
       491A  _xatan
       497B  _asinf
       4A25  _atan2f
       4AEA  _fabsf
       4B17  _frexpf
       4B83  _sqrtf
       4CC1  _printf
       4CF2  __FormatFP_1
       51DC  _labs
       5201  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0100   0EE5 =   3813. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  _rchar
       0100  __data_start
       0101  _loops
       0103  _loopsBit
       0104  _Kp_A
       0106  _Ki_A
       0108  _Kd_A
       010A  _PWM_L
       010C  _PWM_R
       010E  _turn_flag
       010F  _PWM_Lturn
       0111  _PWM_Rturn
       0113  _Ksp
       0117  _Ksn
       011B  _i
       011C  _j
       0164  _hw
       016E  _reg
       0189  _test
       01B0  inv_mpu.c:st
       01D3  inv_mpu_dmp_motion_driver.c:dmp_memory
       0DC9  inv_mpu_dmp_motion_driver.c:sStartAddress
       0DCB  inv_mpu_dmp_motion_driver.c:dmp
       0E16  _gyro_orientation
       0E1F  _hal
       0FE1  _Lcount
       0FE3  _Rcount
       0FE5  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0FE5   0085 =    133. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0FE5  _pid_num
       0FE5  __bss_start
       0FE7  _rec_buf
       1005  _distance
       1007  _speed
       1009  _PWM_Balance
       100B  _angle_dot
       100F  _angle
       1013  _fix_angle
       1017  _angle_Targer
       101B  _PID_angle
       102F  _rx_new
       1030  _sensors
       1032  _accel
       1038  _gyro
       103E  _Speed_rl
       1068  _errno
       106A  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   0078 =    120. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

D:\installation_software\iccavr\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
motor.o	[ motor.c ]
usart.o	[ usart.c ]
i2c.o	[ i2c.c ]
inv_mpu.o	[ inv_mpu.c ]
inv_mpu_dmp_motion_driver.o	[ inv_mpu_dmp_motion_driver.c ]
mpu6050.o	[ mpu6050.c ]
encoder.o	[ encoder.c ]
upload.o	[ upload.c ]
pid.o	[ pid.c ]
timer.o	[ timer.c ]
<library>	[ _printf.s, memcmp.s, memcpy.s, memset.s, strlen.s, asave.s, asr32.s, div16s.s, div16u.s, div32.s, div8u.s ]
[ elpm32.s, emul16s.s, emul32.s, gpop2.s, gpop3.s, gpop4.s, gpop5.s, gpush.s, gpushx.s, gpushy.s, gpushz.s ]
[ gpusha.s, gpushb.s, icallx.s, lneg.s, longutil.s, lsl16.s, lsl32.s, lsr16.s, lsr32.s, struct.s, fpadd.s ]
[ fpadd1.s, fpadd1x.s, fpadd2.s, fpadd2x.s, fpsub1.s, fpsub2.s, fpsub1x.s, fpint.s, int2fp.s, fpdiv_new.s ]
[ fputil.s, fpmule.s, fpmule1.s, fpmule2.s, fpmule1x.s, fpmule2x.s, fpcmp.s, fpcmp1.s, fpcmp2.s, fpneg.s ]
[ fpneg2.s, cstrcpy.s, cstrlen.s, atoi.s, itoa.s, ltoa.s, ultoa.s, utoa.s, ftoa.s, isdigit.s, islower.s ]
[ isspace.s, ctype.s, toupper.s, asinf.s, atan2f.s, fabsf.s, frexpf.s, sqrtf.s, printf.s, formatfp1.s ]
[ labs.s, errno.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x1e

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

