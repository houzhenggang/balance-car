#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"
#include "i2c.h"
 
#define MOTION_DRIVER_TARGET_MSP430

#if defined MOTION_DRIVER_TARGET_MSP430

#define i2c_write   i2cwrite
#define i2c_read    i2cread
#define delay_ms    delay_ms
#define get_ms      get_ms

#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)

#elif defined EMPL_TARGET_MSP430
#include "msp430.h"
#include "msp430_i2c.h"
#include "msp430_clock.h"
#include "msp430_interrupt.h"
#include "log.h"
#define i2c_write   msp430_i2c_write
#define i2c_read    msp430_i2c_read
#define delay_ms    msp430_delay_ms
#define get_ms      msp430_get_clock_ms
static inline int reg_int_cb(struct int_param_s *int_param)
{
	return msp430_reg_int_cb(int_param->cb, int_param->pin, int_param->lp_exit,
	int_param->active_low);
}
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE

#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)
#elif defined EMPL_TARGET_UC3L0

#define i2c_write(a, b, c, d)   twi_write(a, b, d, c)
#define i2c_read(a, b, c, d)    twi_read(a, b, d, c)

#define get_ms  uc3l0_get_clock_ms
static inline int reg_int_cb(struct int_param_s *int_param)
{
	sensor_board_irq_connect(int_param->pin, int_param->cb, int_param->arg);
	return 0;
}
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE

#define labs        abs
#define fabs(x)     (((x)>0)?(x):-(x))
#else
#error  Gyro driver is missing the system layer implementations.
#endif

static int set_int_enable(unsigned char enable);

struct gyro_reg_s 
{
	unsigned char who_am_i;
	unsigned char rate_div;
	unsigned char lpf;
	unsigned char prod_id;
	unsigned char user_ctrl;
	unsigned char fifo_en;
	unsigned char gyro_cfg;
	unsigned char accel_cfg;

	unsigned char motion_thr;
	unsigned char motion_dur;
	unsigned char fifo_count_h;
	unsigned char fifo_r_w;
	unsigned char raw_gyro;
	unsigned char raw_accel;
	unsigned char temp;
	unsigned char int_enable;
	unsigned char dmp_int_status;
	unsigned char int_status;

	unsigned char pwr_mgmt_1;
	unsigned char pwr_mgmt_2;
	unsigned char int_pin_cfg;
	unsigned char mem_r_w;
	unsigned char accel_offs;
	unsigned char i2c_mst;
	unsigned char bank_sel;
	unsigned char mem_start_addr;
	unsigned char prgm_start_h;
};

struct hw_s 
{
	unsigned char addr;
	unsigned short max_fifo;
	unsigned char num_reg;
	unsigned short temp_sens;
	short temp_offset;
	unsigned short bank_size;
};


struct motion_int_cache_s 
{
	unsigned short gyro_fsr;
	unsigned char accel_fsr;
	unsigned short lpf;
	unsigned short sample_rate;
	unsigned char sensors_on;
	unsigned char fifo_sensors;
	unsigned char dmp_on;
};

struct chip_cfg_s 
{
	unsigned char gyro_fsr;

	unsigned char accel_fsr;
 
	unsigned char sensors;
 
	unsigned char lpf;
	unsigned char clk_src;
	
	unsigned short sample_rate;

	unsigned char fifo_enable;
 
	unsigned char int_enable;

	unsigned char bypass_mode;

	unsigned char accel_half;

	unsigned char lp_accel_mode;
 
	unsigned char int_motion_only;
	struct motion_int_cache_s cache;
 
	unsigned char active_low_int;
	
	unsigned char latched_int;

	unsigned char dmp_on;
	
	unsigned char dmp_loaded;

	unsigned short dmp_sample_rate;
};

struct test_s 
{
	unsigned long gyro_sens;
	unsigned long accel_sens;
	unsigned char reg_rate_div;
	unsigned char reg_lpf;
	unsigned char reg_gyro_fsr;
	unsigned char reg_accel_fsr;
	unsigned short wait_ms;
	unsigned char packet_thresh;
	float min_dps;
	float max_dps;
	float max_gyro_var;
	float min_g;
	float max_g;
	float max_accel_var;
};

struct gyro_state_s 
{
	const struct gyro_reg_s *reg;
	const struct hw_s *hw;
	struct chip_cfg_s chip_cfg;
	const struct test_s *test;
};

enum lpf_e 
{
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};

enum gyro_fsr_e 
{
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_GYRO_FSR
};

enum accel_fsr_e 
{
	INV_FSR_2G = 0,
	INV_FSR_4G,
	INV_FSR_8G,
	INV_FSR_16G,
	NUM_ACCEL_FSR
};

enum clock_sel_e 
{
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

enum lp_accel_rate_e 
{
	INV_LPA_1_25HZ,
	INV_LPA_5HZ,
	INV_LPA_20HZ,
	INV_LPA_40HZ
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

const struct hw_s hw=
{
	0x68,	 //addr
	1024,	 //max_fifo
	118,	 //num_reg
	340,	 //temp_sens
	-521,	 //temp_offset
	256	 //bank_size
};
const struct gyro_reg_s reg = 
{
	0x75,  //who_am_i
	0x19,  //rate_div
	0x1A,  //lpf
	0x0C,  //prod_id
	0x6A,  //user_ctrl
	0x23,  //fifo_en
	0x1B,  //gyro_cfg
	0x1C,  //accel_cfg
	0x1F,  // motion_thr
	0x20,  // motion_dur
	0x72,  // fifo_count_h
	0x74,  // fifo_r_w
	0x43,  // raw_gyro
	0x3B,  // raw_accel
	0x41,  // temp
	0x38,  // int_enable
	0x39,  //  dmp_int_status
	0x3A,  //  int_status
	0x6B,  // pwr_mgmt_1
	0x6C,  // pwr_mgmt_2
	0x37,  // int_pin_cfg
	0x6F,  // mem_r_w
	0x06,  // accel_offs
	0x24,  // i2c_mst
	0x6D,  // bank_sel
	0x6E,  // mem_start_addr
	0x70   // prgm_start_h
};

const struct test_s test=
{
	32768/250,		 //gyro_sens
	32768/16,		 //	accel_sens
	0,				 //	reg_rate_div
	1,				//	reg_lpf
	0,				 //	reg_gyro_fsr
	0x18,			//	reg_accel_fsr
	50,				//	wait_ms
	5,				//	packet_thresh
	10.0f,			 //	min_dps
	105.0f,			 //	max_dps
	0.14f,			//	max_gyro_var
	0.3f,		   //	min_g
	0.95f,		   //	max_g
	0.14f		   //	max_accel_var
};

static struct gyro_state_s st=
{
	&reg,
	&hw,
	{0},
	&test
};

#define MAX_PACKET_LENGTH (12)

static int set_int_enable(unsigned char enable)
{
	unsigned char tmp;

	if (st.chip_cfg.dmp_on) 
	{
		if (enable)
			tmp = BIT_DMP_INT_EN;
		else
			tmp = 0x00;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
			return -1;
		st.chip_cfg.int_enable = tmp;
	} 
	else 
	{
		if (!st.chip_cfg.sensors)
			return -1;
		if (enable && st.chip_cfg.int_enable)
			return 0;
		if (enable)
			tmp = BIT_DATA_RDY_EN;
		else
			tmp = 0x00;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
			return -1;
		st.chip_cfg.int_enable = tmp;
	}
	return 0;
}

int mpu_reg_dump(void)
{
	unsigned char ii;
	unsigned char data;

	for (ii = 0; ii < st.hw->num_reg; ii++) 
  {
		if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)
			continue;
		if (i2c_read(st.hw->addr, ii, 1, &data))
			return -1;
	}
	return 0;
}


int mpu_read_reg(unsigned char reg, unsigned char *data)
{
	if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w)
		return -1;
	if (reg >= st.hw->num_reg)
		return -1;
	return i2c_read(st.hw->addr, reg, 1, data);
}

int mpu_init(void)
{
	unsigned char data[6], rev;

	data[0] = 0x80;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &(data[0])))
		return -1;
	delay_ms(100);

	data[0] = 0x00;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &(data[0])))
		return -1;

	if (i2c_read(st.hw->addr, st.reg->accel_offs, 6, data))
		return -1;
	rev = ((data[5] & 0x01) << 2) | ((data[3] & 0x01) << 1) |
		(data[1] & 0x01);

	if (rev) 
	{
		if (rev == 1)
				st.chip_cfg.accel_half = 1;
			else if (rev == 2)
				st.chip_cfg.accel_half = 0;
			else 
			{
				return -1;
			}
	} 
	
	else 
	{
		if (i2c_read(st.hw->addr, st.reg->prod_id, 1, &(data[0])))
			return -1;
		rev = data[0] & 0x0F;
		if (!rev) 
		{
			return -1;
		} 
		else if (rev == 4) 
		{
			st.chip_cfg.accel_half = 1;
		} 
		else
			st.chip_cfg.accel_half = 0;
	}

	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.bypass_mode = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	st.chip_cfg.active_low_int = 1;
	st.chip_cfg.latched_int = 0;
	st.chip_cfg.int_motion_only = 0;
	st.chip_cfg.lp_accel_mode = 0;
	memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
	st.chip_cfg.dmp_on = 0;
	st.chip_cfg.dmp_loaded = 0;
	st.chip_cfg.dmp_sample_rate = 0;

	if (mpu_set_gyro_fsr(2000))
		return -1;
	if (mpu_set_accel_fsr(2))
		return -1;
	if (mpu_set_lpf(42))
		return -1;
	if (mpu_set_sample_rate(50))
		return -1;
	if (mpu_configure_fifo(0))
		return -1;
	return 0;
}

int mpu_lp_accel_mode(unsigned char rate)
{
	unsigned char tmp[2];

	if (rate > 40)
		return -1;

	if (!rate) 
	{
		mpu_set_int_latched(0);
		tmp[0] = 0;
		tmp[1] = BIT_STBY_XYZG;
		if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
				return -1;
		st.chip_cfg.lp_accel_mode = 0;
		return 0;
	}
	
	mpu_set_int_latched(1);
	tmp[0] = BIT_LPA_CYCLE;
	
	if (rate == 1) 
	{
		tmp[1] = INV_LPA_1_25HZ;
		mpu_set_lpf(5);
	} 	
	else if (rate <= 5) 
	{
		tmp[1] = INV_LPA_5HZ;
		mpu_set_lpf(5);
	} 	
	else if (rate <= 20) 
	{
		tmp[1] = INV_LPA_20HZ;
		mpu_set_lpf(10);
	} 	
	else 
	{
		tmp[1] = INV_LPA_40HZ;
		mpu_set_lpf(20);
	}
	
	tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
		return -1;
	st.chip_cfg.sensors = INV_XYZ_ACCEL;
	st.chip_cfg.clk_src = 0;
	st.chip_cfg.lp_accel_mode = 1;
	mpu_configure_fifo(0);

	return 0;
}

int mpu_get_gyro_reg(short *data, unsigned long *timestamp)
{
	unsigned char tmp[6];

	if (!(st.chip_cfg.sensors & INV_XYZ_GYRO))
		return -1;

	if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp))
		return -1;
	data[0] = (tmp[0] << 8) | tmp[1];
	data[1] = (tmp[2] << 8) | tmp[3];
	data[2] = (tmp[4] << 8) | tmp[5];
	if (timestamp)
		get_ms(timestamp);
	return 0;
}

int mpu_get_accel_reg(short *data, unsigned long *timestamp)
{
	unsigned char tmp[6];

	if (!(st.chip_cfg.sensors & INV_XYZ_ACCEL))
		return -1;

	if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp))
		return -1;
	data[0] = (tmp[0] << 8) | tmp[1];
	data[1] = (tmp[2] << 8) | tmp[3];
	data[2] = (tmp[4] << 8) | tmp[5];
	if (timestamp)
		get_ms(timestamp);
	return 0;
}

int mpu_get_temperature(long *data, unsigned long *timestamp)
{
	unsigned char tmp[2];
	short raw;

	if (!(st.chip_cfg.sensors))
		return -1;

	if (i2c_read(st.hw->addr, st.reg->temp, 2, tmp))
		return -1;
	raw = (tmp[0] << 8) | tmp[1];
	if (timestamp)
		get_ms(timestamp);

	data[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L);
	return 0;
}

int mpu_set_accel_bias(const long *accel_bias)
{
	unsigned char data[6];
	short accel_hw[3];
	short got_accel[3];
	short fg[3];

	if (!accel_bias)
		return -1;
	if (!accel_bias[0] && !accel_bias[1] && !accel_bias[2])
		return 0;

	if (i2c_read(st.hw->addr, 3, 3, data))
		return -1;
	fg[0] = ((data[0] >> 4) + 8) & 0xf;
	fg[1] = ((data[1] >> 4) + 8) & 0xf;
	fg[2] = ((data[2] >> 4) + 8) & 0xf;

	accel_hw[0] = (short)(accel_bias[0] * 2 / (64 + fg[0]));
	accel_hw[1] = (short)(accel_bias[1] * 2 / (64 + fg[1]));
	accel_hw[2] = (short)(accel_bias[2] * 2 / (64 + fg[2]));

	if (i2c_read(st.hw->addr, 0x06, 6, data))
		return -1;

	got_accel[0] = ((short)data[0] << 8) | data[1];
	got_accel[1] = ((short)data[2] << 8) | data[3];
	got_accel[2] = ((short)data[4] << 8) | data[5];

	accel_hw[0] += got_accel[0];
	accel_hw[1] += got_accel[1];
	accel_hw[2] += got_accel[2];

	data[0] = (accel_hw[0] >> 8) & 0xff;
	data[1] = (accel_hw[0]) & 0xff;
	data[2] = (accel_hw[1] >> 8) & 0xff;
	data[3] = (accel_hw[1]) & 0xff;
	data[4] = (accel_hw[2] >> 8) & 0xff;
	data[5] = (accel_hw[2]) & 0xff;

	if (i2c_write(st.hw->addr, 0x06, 6, data))
		return -1;
	return 0;
}

int mpu_reset_fifo(void)
{
	unsigned char data;

	if (!(st.chip_cfg.sensors))
		return -1;

	data = 0;
	if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
		return -1;

	if (st.chip_cfg.dmp_on) 
	{
		data = BIT_FIFO_RST | BIT_DMP_RST;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return -1;
		delay_ms(50);
		data = BIT_DMP_EN | BIT_FIFO_EN;
		if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
			data |= BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return -1;
		if (st.chip_cfg.int_enable)
			data = BIT_DMP_INT_EN;
		else
			data = 0;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
			return -1;
		data = 0;
		if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
			return -1;
	} 
	else 
	{
		data = BIT_FIFO_RST;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return -1;
		if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors ))
			data = BIT_FIFO_EN;
		else
			data = BIT_FIFO_EN | BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return -1;
		delay_ms(50);
		if (st.chip_cfg.int_enable)
			data = BIT_DATA_RDY_EN;
		else
			data = 0;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
			return -1;
		if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable))
			return -1;
	}
	return 0;
}

int mpu_get_gyro_fsr(unsigned short *fsr)
{
	switch (st.chip_cfg.gyro_fsr) 
	{
	case INV_FSR_250DPS:
		fsr[0] = 250;
		break;
	case INV_FSR_500DPS:
		fsr[0] = 500;
		break;
	case INV_FSR_1000DPS:
		fsr[0] = 1000;
		break;
	case INV_FSR_2000DPS:
		fsr[0] = 2000;
		break;
	default:
		fsr[0] = 0;
		break;
	}
	return 0;
}

int mpu_set_gyro_fsr(unsigned short fsr)
{
	unsigned char data;

	if (!(st.chip_cfg.sensors))
		return -1;

	switch (fsr) 
	{
	case 250:
		data = INV_FSR_250DPS << 3;
		break;
	case 500:
		data = INV_FSR_500DPS << 3;
		break;
	case 1000:
		data = INV_FSR_1000DPS << 3;
		break;
	case 2000:
		data = INV_FSR_2000DPS << 3;
		break;
	default:
		return -1;
	}

	if (st.chip_cfg.gyro_fsr == (data >> 3))
		return 0;
	if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data))
		return -1;
	st.chip_cfg.gyro_fsr = data >> 3;
	return 0;
}

int mpu_get_accel_fsr(unsigned char *fsr)
{
	switch (st.chip_cfg.accel_fsr) 
	{
	case INV_FSR_2G:
		fsr[0] = 2;
		break;
	case INV_FSR_4G:
		fsr[0] = 4;
		break;
	case INV_FSR_8G:
		fsr[0] = 8;
		break;
	case INV_FSR_16G:
		fsr[0] = 16;
		break;
	default:
		return -1;
	}
	if (st.chip_cfg.accel_half)
		fsr[0] <<= 1;
	return 0;
}

int mpu_set_accel_fsr(unsigned char fsr)
{
	unsigned char data;

	if (!(st.chip_cfg.sensors))
		return -1;

	switch (fsr) 
	{
	case 2:
		data = INV_FSR_2G << 3;
		break;
	case 4:
		data = INV_FSR_4G << 3;
		break;
	case 8:
		data = INV_FSR_8G << 3;
		break;
	case 16:
		data = INV_FSR_16G << 3;
		break;
	default:
		return -1;
	}

	if (st.chip_cfg.accel_fsr == (data >> 3))
		return 0;
	if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data))
		return -1;
	st.chip_cfg.accel_fsr = data >> 3;
	return 0;
}

int mpu_get_lpf(unsigned short *lpf)
{
	switch (st.chip_cfg.lpf) 
	{
	case INV_FILTER_188HZ:
		lpf[0] = 188;
		break;
	case INV_FILTER_98HZ:
		lpf[0] = 98;
		break;
	case INV_FILTER_42HZ:
		lpf[0] = 42;
		break;
	case INV_FILTER_20HZ:
		lpf[0] = 20;
		break;
	case INV_FILTER_10HZ:
		lpf[0] = 10;
		break;
	case INV_FILTER_5HZ:
		lpf[0] = 5;
		break;
	case INV_FILTER_256HZ_NOLPF2:
	case INV_FILTER_2100HZ_NOLPF:
	default:
		lpf[0] = 0;
		break;
	}
	return 0;
}

int mpu_set_lpf(unsigned short lpf)
{
	unsigned char data;

	if (!(st.chip_cfg.sensors))
		return -1;

	if (lpf >= 188)
		data = INV_FILTER_188HZ;
	else if (lpf >= 98)
		data = INV_FILTER_98HZ;
	else if (lpf >= 42)
		data = INV_FILTER_42HZ;
	else if (lpf >= 20)
		data = INV_FILTER_20HZ;
	else if (lpf >= 10)
		data = INV_FILTER_10HZ;
	else
		data = INV_FILTER_5HZ;

	if (st.chip_cfg.lpf == data)
		return 0;
	if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data))
		return -1;
	st.chip_cfg.lpf = data;
	return 0;
}

int mpu_get_sample_rate(unsigned short *rate)
{
	if (st.chip_cfg.dmp_on)
		return -1;
	else
		rate[0] = st.chip_cfg.sample_rate;
	return 0;
}

int mpu_set_sample_rate(unsigned short rate)
{
	unsigned char data;

	if (!(st.chip_cfg.sensors))
		return -1;

	if (st.chip_cfg.dmp_on)
		return -1;
	else 
	{
		if (st.chip_cfg.lp_accel_mode) 
		{
			if (rate && (rate <= 40)) 
			{
				mpu_lp_accel_mode(rate);
				return 0;
			}

			mpu_lp_accel_mode(0);
		}
		if (rate < 4)
			rate = 4;
		else if (rate > 1000)
			rate = 1000;

		data = 1000 / rate - 1;
		if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data))
			return -1;

		st.chip_cfg.sample_rate = 1000 / (1 + data);

		mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
		return 0;
	}
}

int mpu_get_gyro_sens(float *sens)
{
	switch (st.chip_cfg.gyro_fsr) 
	{
	case INV_FSR_250DPS:
		sens[0] = 131.f;
		break;
	case INV_FSR_500DPS:
		sens[0] = 65.5f;
		break;
	case INV_FSR_1000DPS:
		sens[0] = 32.8f;
		break;
	case INV_FSR_2000DPS:
		sens[0] = 16.4f;
		break;
	default:
		return -1;
	}
	return 0;
}

int mpu_get_accel_sens(unsigned short *sens)
{
	switch (st.chip_cfg.accel_fsr) 
	{
	case INV_FSR_2G:
		sens[0] = 16384;
		break;
	case INV_FSR_4G:
		sens[0] = 8092;
		break;
	case INV_FSR_8G:
		sens[0] = 4096;
		break;
	case INV_FSR_16G:
		sens[0] = 2048;
		break;
	default:
		return -1;
	}
	if (st.chip_cfg.accel_half)
		sens[0] >>= 1;
	return 0;
}

int mpu_get_fifo_config(unsigned char *sensors)
{
	sensors[0] = st.chip_cfg.fifo_enable;
	return 0;
}

int mpu_configure_fifo(unsigned char sensors)
{
	unsigned char prev;
	int result = 0;

	sensors &= ~INV_XYZ_COMPASS;

	if (st.chip_cfg.dmp_on)
		return 0;
	else {
		if (!(st.chip_cfg.sensors))
			return -1;
		prev = st.chip_cfg.fifo_enable;
		st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
		if (st.chip_cfg.fifo_enable != sensors)

			result = -1;
		else
			result = 0;
		if (sensors || st.chip_cfg.lp_accel_mode)
			set_int_enable(1);
		else
			set_int_enable(0);
		if (sensors) 
		{
			if (mpu_reset_fifo()) 
			{
				st.chip_cfg.fifo_enable = prev;
				return -1;
			}
		}
	}

	return result;
}

int mpu_get_power_state(unsigned char *power_on)
{
	if (st.chip_cfg.sensors)
		power_on[0] = 1;
	else
		power_on[0] = 0;
	return 0;
}

int mpu_set_sensors(unsigned char sensors)
{
	unsigned char data;

	if (sensors & INV_XYZ_GYRO)
		data = INV_CLK_PLL;
	else if (sensors)
		data = 0;
	else
		data = BIT_SLEEP;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) 
	{
		st.chip_cfg.sensors = 0;
		return -1;
	}
	st.chip_cfg.clk_src = data & ~BIT_SLEEP;

	data = 0;
	if (!(sensors & INV_X_GYRO))
		data |= BIT_STBY_XG;
	if (!(sensors & INV_Y_GYRO))
		data |= BIT_STBY_YG;
	if (!(sensors & INV_Z_GYRO))
		data |= BIT_STBY_ZG;
	if (!(sensors & INV_XYZ_ACCEL))
		data |= BIT_STBY_XYZA;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) 
	{
		st.chip_cfg.sensors = 0;
		return -1;
	}

	if (sensors && (sensors != INV_XYZ_ACCEL))

		mpu_set_int_latched(0);

	st.chip_cfg.sensors = sensors;
	st.chip_cfg.lp_accel_mode = 0;
	delay_ms(50);
	return 0;
}

int mpu_get_int_status(short *status)
{
	unsigned char tmp[2];
	if (!st.chip_cfg.sensors)
		return -1;
	if (i2c_read(st.hw->addr, st.reg->dmp_int_status, 2, tmp))
		return -1;
	status[0] = (tmp[0] << 8) | tmp[1];
	return 0;
}

int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
		unsigned char *sensors, unsigned char *more)
{
	unsigned char data[MAX_PACKET_LENGTH];
	unsigned char packet_size = 0;
	unsigned short fifo_count, index = 0;

	if (st.chip_cfg.dmp_on)
		return -1;

	sensors[0] = 0;
	if (!st.chip_cfg.sensors)
		return -1;
	if (!st.chip_cfg.fifo_enable)
		return -1;

	if (st.chip_cfg.fifo_enable & INV_X_GYRO)
		packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_Y_GYRO)
		packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_Z_GYRO)
		packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)
		packet_size += 6;

	if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
		return -1;
	fifo_count = (data[0] << 8) | data[1];
	if (fifo_count < packet_size)
		return 0;
	if (fifo_count > (st.hw->max_fifo >> 1)) 
	{
		if (i2c_read(st.hw->addr, st.reg->int_status, 1, data))
				return -1;
		if (data[0] & BIT_FIFO_OVERFLOW) {
				mpu_reset_fifo();
				return -2;
		}
	}
	get_ms((unsigned long*)timestamp);

	if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data))
		return -1;
	more[0] = fifo_count / packet_size - 1;
	sensors[0] = 0;

	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) 
	{
		accel[0] = (data[index+0] << 8) | data[index+1];
		accel[1] = (data[index+2] << 8) | data[index+3];
		accel[2] = (data[index+4] << 8) | data[index+5];
		sensors[0] |= INV_XYZ_ACCEL;
		index += 6;
	}
	
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO) 
	{
		gyro[0] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_X_GYRO;
		index += 2;
	}
	
	if((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO) 
	{
		gyro[1] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_Y_GYRO;
		index += 2;
	}
	
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO) 
	{
		gyro[2] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_Z_GYRO;
		index += 2;
	}

	return 0;
}

int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more)
{
	unsigned char tmp[2];
	unsigned short fifo_count;
	if (!st.chip_cfg.dmp_on)
		return -1;
	if (!st.chip_cfg.sensors)
		return -1;

	if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp))
		return -1;
	fifo_count = (tmp[0] << 8) | tmp[1];
	if (fifo_count < length) {
		more[0] = 0;
		return -1;
	}
	if (fifo_count > (st.hw->max_fifo >> 1)) {

		if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp))
				return -1;
		if (tmp[0] & BIT_FIFO_OVERFLOW) {
				mpu_reset_fifo();
				return -2;
		}
	}

	if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data))
		return -1;
	more[0] = fifo_count / length - 1;
	return 0;
}

int mpu_set_int_level(unsigned char active_low)
{
    st.chip_cfg.active_low_int = active_low;
    return 0;
}

int mpu_set_int_latched(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else
        tmp = 0;
    if (st.chip_cfg.bypass_mode)
        tmp |= BIT_BYPASS_EN;
    if (st.chip_cfg.active_low_int)
        tmp |= BIT_ACTL;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
        return -1;
    st.chip_cfg.latched_int = enable;
    return 0;
}

static int get_accel_prod_shift(float *st_shift)
{
	unsigned char tmp[4], shift_code[3], ii;

	if (i2c_read(st.hw->addr, 0x0D, 4, tmp))
		return 0x07;

	shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
	shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
	shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
	for (ii = 0; ii < 3; ii++) 
	{
		if (!shift_code[ii]) 
		{
			st_shift[ii] = 0.f;
			continue;
		}

		st_shift[ii] = 0.34f;
		while (--shift_code[ii])
			st_shift[ii] *= 1.034f;
	}
	return 0;
}

static int accel_self_test(long *bias_regular, long *bias_st)
{
	int jj, result = 0;
	float st_shift[3], st_shift_cust, st_shift_var;

	get_accel_prod_shift(st_shift);
	for(jj = 0; jj < 3; jj++) 
	{
		st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
		if (st_shift[jj]) 
		{
			st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
		
			if (fabs(st_shift_var) > test.max_accel_var)
				result |= 1 << jj;
		} 
		else if ((st_shift_cust < test.min_g) ||
			(st_shift_cust > test.max_g))
			result |= 1 << jj;
	}

	return result;
}

static int gyro_self_test(long *bias_regular, long *bias_st)
{
	int jj, result = 0;
	unsigned char tmp[3];
	float st_shift, st_shift_cust, st_shift_var;

	if (i2c_read(st.hw->addr, 0x0D, 3, tmp))
		return 0x07;

	tmp[0] &= 0x1F;
	tmp[1] &= 0x1F;
	tmp[2] &= 0x1F;

	for (jj = 0; jj < 3; jj++) {
		st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
		if (tmp[jj]) 
		{
			st_shift = 3275.f / test.gyro_sens;
			while (--tmp[jj])
				st_shift *= 1.046f;
			st_shift_var = st_shift_cust / st_shift - 1.f;
			if (fabs(st_shift_var) > test.max_gyro_var)
				result |= 1 << jj;
		} 
		else if ((st_shift_cust < test.min_dps) ||
			(st_shift_cust > test.max_dps))
			result |= 1 << jj;
	}
	return result;
}

static int get_st_biases(long *gyro, long *accel, unsigned char hw_test)
{
	unsigned char data[MAX_PACKET_LENGTH];
	unsigned char packet_count, ii;
	unsigned short fifo_count;

	data[0] = 0x01;
	data[1] = 0;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
		return -1;
	delay_ms(200);
	data[0] = 0;
	if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
		return -1;
	data[0] = BIT_FIFO_RST | BIT_DMP_RST;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
		return -1;
	delay_ms(15);
	data[0] = st.test->reg_lpf;
	if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
		return -1;
	data[0] = st.test->reg_rate_div;
	if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
		return -1;
	if (hw_test)
		data[0] = st.test->reg_gyro_fsr | 0xE0;
	else
		data[0] = st.test->reg_gyro_fsr;
	if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
		return -1;

	if (hw_test)
		data[0] = st.test->reg_accel_fsr | 0xE0;
	else
		data[0] = test.reg_accel_fsr;
	if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
		return -1;
	if (hw_test)
		delay_ms(200);

	data[0] = BIT_FIFO_EN;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
		return -1;

	data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
		return -1;
	delay_ms(test.wait_ms);
	data[0] = 0;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
		return -1;

	if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
		return -1;

	fifo_count = (data[0] << 8) | data[1];
	packet_count = fifo_count / MAX_PACKET_LENGTH;
	gyro[0] = gyro[1] = gyro[2] = 0;
	accel[0] = accel[1] = accel[2] = 0;

	for (ii = 0; ii < packet_count; ii++) {
		short accel_cur[3], gyro_cur[3];
		if (i2c_read(st.hw->addr, st.reg->fifo_r_w, MAX_PACKET_LENGTH, data))
			return -1;
		accel_cur[0] = ((short)data[0] << 8) | data[1];
		accel_cur[1] = ((short)data[2] << 8) | data[3];
		accel_cur[2] = ((short)data[4] << 8) | data[5];
		accel[0] += (long)accel_cur[0];
		accel[1] += (long)accel_cur[1];
		accel[2] += (long)accel_cur[2];
		gyro_cur[0] = (((short)data[6] << 8) | data[7]);
		gyro_cur[1] = (((short)data[8] << 8) | data[9]);
		gyro_cur[2] = (((short)data[10] << 8) | data[11]);
		gyro[0] += (long)gyro_cur[0];
		gyro[1] += (long)gyro_cur[1];
		gyro[2] += (long)gyro_cur[2];
	}
	#ifdef EMPL_NO_64BIT
	gyro[0] = (long)(((float)gyro[0]*65536.f) / test.gyro_sens / packet_count);
	gyro[1] = (long)(((float)gyro[1]*65536.f) / test.gyro_sens / packet_count);
	gyro[2] = (long)(((float)gyro[2]*65536.f) / test.gyro_sens / packet_count);
	if (has_accel) 
	{
		accel[0] = (long)(((float)accel[0]*65536.f) / test.accel_sens /
			packet_count);
		accel[1] = (long)(((float)accel[1]*65536.f) / test.accel_sens /
			packet_count);
		accel[2] = (long)(((float)accel[2]*65536.f) / test.accel_sens /
			packet_count);
		accel[2] -= 65536L;
	}
	#else
	gyro[0] = (long)(((long long)gyro[0]<<16) / test.gyro_sens / packet_count);
	gyro[1] = (long)(((long long)gyro[1]<<16) / test.gyro_sens / packet_count);
	gyro[2] = (long)(((long long)gyro[2]<<16) / test.gyro_sens / packet_count);
	accel[0] = (long)(((long long)accel[0]<<16) / test.accel_sens /
		packet_count);
	accel[1] = (long)(((long long)accel[1]<<16) / test.accel_sens /
		packet_count);
	accel[2] = (long)(((long long)accel[2]<<16) / test.accel_sens /
		packet_count);
	if (accel[2] > 0L)
		accel[2] -= 65536L;
	else
		accel[2] += 65536L;
	#endif

	return 0;
}

int mpu_run_self_test(long *gyro, long *accel)
{
	const unsigned char tries = 2;
	long gyro_st[3], accel_st[3];
	unsigned char accel_result, gyro_result;
	int ii;
	int result;
	unsigned char accel_fsr, fifo_sensors, sensors_on;
	unsigned short gyro_fsr, sample_rate, lpf;
	unsigned char dmp_was_on;

	if (st.chip_cfg.dmp_on) 
	{
		mpu_set_dmp_state(0);
		dmp_was_on = 1;
	} else
		dmp_was_on = 0;

	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
	mpu_get_lpf(&lpf);
	mpu_get_sample_rate(&sample_rate);
	sensors_on = st.chip_cfg.sensors;
	mpu_get_fifo_config(&fifo_sensors);

	for (ii = 0; ii < tries; ii++)
		if (!get_st_biases(gyro, accel, 0))
			break;
	if (ii == tries) {
		result = 0;
	}
	for (ii = 0; ii < tries; ii++)
		if (!get_st_biases(gyro_st, accel_st, 1))
			break;
	if (ii == tries) 
	{
		result = 0;
	}
	accel_result = accel_self_test(accel, accel_st);
	gyro_result = gyro_self_test(gyro, gyro_st);

	result = 0;
	if (!gyro_result)
		result |= 0x01;
	if (!accel_result)
		result |= 0x02;
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);
	mpu_set_lpf(lpf);
	mpu_set_sample_rate(sample_rate);
	mpu_set_sensors(sensors_on);
	mpu_configure_fifo(fifo_sensors);

	if (dmp_was_on)
		mpu_set_dmp_state(1);

	return result;
}

int mpu_write_mem(unsigned short mem_addr, unsigned short length,
		unsigned char *data)
{
	unsigned char tmp[2];

	if (!data)
		return -1;
	if (!st.chip_cfg.sensors)
		return -1;

	tmp[0] = (unsigned char)(mem_addr >> 8);
	tmp[1] = (unsigned char)(mem_addr & 0xFF);

	if (tmp[1] + length > st.hw->bank_size)
		return -1;

	if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
		return -1;
	if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data))
		return -1;
	return 0;
}

int mpu_read_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data)
{
	unsigned char tmp[2];

	if (!data)
		return -1;
	if (!st.chip_cfg.sensors)
		return -1;

	tmp[0] = (unsigned char)(mem_addr >> 8);
	tmp[1] = (unsigned char)(mem_addr & 0xFF);

	if (tmp[1] + length > st.hw->bank_size)
		return -1;

	if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
		return -1;
	if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data))
		return -1;
	return 0;
}

int mpu_set_bypass(unsigned char bypass_on)
{
    unsigned char tmp;

    if (st.chip_cfg.bypass_mode == bypass_on)
        return 0;

    if (bypass_on) {
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        tmp = BIT_BYPASS_EN;
        if (st.chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    } else {
        /* Enable I2C master mode if compass is being used. */
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
            tmp |= BIT_AUX_IF_EN;
        else
            tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        if (st.chip_cfg.active_low_int)
            tmp = BIT_ACTL;
        else
            tmp = 0;
        if (st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    }
    st.chip_cfg.bypass_mode = bypass_on;
    return 0;
}


int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate)
{
	unsigned short ii;
	unsigned short this_write;
	#define LOAD_CHUNK  (16)
	unsigned char cur[LOAD_CHUNK], tmp[2];

	if (st.chip_cfg.dmp_loaded)
		return -1;

	if (!firmware)
		return -1;
	for (ii = 0; ii < length; ii += this_write) {
		this_write = min(LOAD_CHUNK, length - ii);
		if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii]))
				return -1;
		if (mpu_read_mem(ii, this_write, cur))
				return -1;
		if (memcmp((unsigned char*)firmware+ii, (unsigned char*)cur, this_write))
				return -2;
	}

	tmp[0] = start_addr >> 8;
	tmp[1] = start_addr & 0xFF;
	if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
		return -1;

	st.chip_cfg.dmp_loaded = 1;
	st.chip_cfg.dmp_sample_rate = sample_rate;
	return 0;
}

int mpu_set_dmp_state(unsigned char enable)
{
	unsigned char tmp;
	if (st.chip_cfg.dmp_on == enable)
		return 0;

	if (enable) {
		if (!st.chip_cfg.dmp_loaded)
				return -1;
		set_int_enable(0);
		mpu_set_bypass(0);

		mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);

		tmp = 0;
		i2c_write(st.hw->addr, 0x23, 1, &tmp);
		st.chip_cfg.dmp_on = 1;

		set_int_enable(1);
		mpu_reset_fifo();
	} 
	else 
	{
		set_int_enable(0);
		tmp = st.chip_cfg.fifo_enable;
		i2c_write(st.hw->addr, 0x23, 1, &tmp);
		st.chip_cfg.dmp_on = 0;
		mpu_reset_fifo();
	}
	return 0;
}

int mpu_get_dmp_state(unsigned char *enabled)
{
	enabled[0] = st.chip_cfg.dmp_on;
	return 0;
}

int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned char lpa_freq)
{
	unsigned char data[3];

	if (lpa_freq) 
	{
	unsigned char thresh_hw;
	if (thresh > 8160)
		thresh_hw = 255;
	else if (thresh < 32)
		thresh_hw = 1;
	else
		thresh_hw = thresh >> 5;
	if (!time)
		time = 1;

	if (lpa_freq > 40)
		return -1;

	if (!st.chip_cfg.int_motion_only) 
	{
		if (st.chip_cfg.dmp_on) 
		{
				mpu_set_dmp_state(0);
				st.chip_cfg.cache.dmp_on = 1;
		}
		else
			st.chip_cfg.cache.dmp_on = 0;
		mpu_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
		mpu_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
		mpu_get_lpf(&st.chip_cfg.cache.lpf);
		mpu_get_sample_rate(&st.chip_cfg.cache.sample_rate);
		st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
		mpu_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
	}
	set_int_enable(0);
	mpu_lp_accel_mode(0);
	data[0] = INV_FILTER_256HZ_NOLPF2;
	if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
			return -1;
	data[0] = BIT_MOT_INT_EN;
	if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
			goto lp_int_restore;

	data[0] = thresh_hw;
	data[1] = time;
	if (i2c_write(st.hw->addr, st.reg->motion_thr, 2, data))
		goto lp_int_restore;

	delay_ms(5);
	data[0] = (st.chip_cfg.accel_fsr << 3) | BITS_HPF;
	if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
		goto lp_int_restore;

	data[0] = BIT_LPA_CYCLE;
	if (lpa_freq == 1)
		data[1] = INV_LPA_1_25HZ;
	else if (lpa_freq <= 5)
		data[1] = INV_LPA_5HZ;
	else if (lpa_freq <= 20)
		data[1] = INV_LPA_20HZ;
	else
		data[1] = INV_LPA_40HZ;
	data[1] = (data[1] << 6) | BIT_STBY_XYZG;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
		goto lp_int_restore;

		st.chip_cfg.int_motion_only = 1;
		return 0;
	} 
	else 
		{
		int ii;
		char *cache_ptr = (char*)&st.chip_cfg.cache;
		for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++) 
		{
				if (cache_ptr[ii] != 0)
						goto lp_int_restore;
		}
		return -1;
	}
	lp_int_restore:
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_sensors(st.chip_cfg.cache.sensors_on);
	mpu_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
	mpu_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
	mpu_set_lpf(st.chip_cfg.cache.lpf);
	mpu_set_sample_rate(st.chip_cfg.cache.sample_rate);
	mpu_configure_fifo(st.chip_cfg.cache.fifo_sensors);

	if (st.chip_cfg.cache.dmp_on)
		mpu_set_dmp_state(1);

	st.chip_cfg.int_motion_only = 0;
	return 0;
}

void get_ms(unsigned long *time)
{

}
