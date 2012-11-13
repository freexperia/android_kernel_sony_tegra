/*
 * SX863X Cap Touch 
 * Currently Supports:
 *  SX8636
 * Future Supports:
 *  SX8634, SX8635
 *
 * Copyright 2011 Semtech Corp.
 *
 *
 * Licensed under the GPL-2 or later.
 */

//#define DEBUG
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include  <linux/delay.h> 
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#include <linux/fs.h>
#include <linux/file.h>
#include <asm/uaccess.h>

#include <linux/input/sx863x.h>

/*
 *  I2C Registers
 */
#include <linux/input/sx863x_i2c_reg.h>

/*
 * spm address map... (currently only showing SX8636 )
 */
#include <linux/input/sx863x_spm_cfg.h>

typedef int (*sx863x_read_t)(struct device *, unsigned char, unsigned char *);
typedef int (*sx863x_write_t)(struct device *, unsigned char, unsigned char);
typedef int (*sx863x_writeEx_t)(struct device *, unsigned char, unsigned char *, int);
typedef int (*sx863x_readEx_t)(struct device *, unsigned char, unsigned char *,int);

#define FOXCONN_SX8636_DEBUG		0



#if FOXCONN_SX8636_DEBUG
#define SX8636_LOG(x...)	printk(KERN_INFO "[SX8636] " x)
#else
#define SX8636_LOG(x...)	do {} while(0)
#endif

#define MAX_WRITE_ARRAY_SIZE 16 /* when performing writeEx. define max array size */
#define DEV_NAME		"sx863x"
#define P_SENSOR_INITIAL_OPERATION_MODE	SX863X_COMPOPMODE_OPERATINGMODE_SLEEP

// 0x80 ~ 0x99 raw
// 0x9A ~ 0xB3 avg
// 0xB4 ~ 0xC8 diff
#define RAW_CAP_VALUE_SPM_BASE		0x80
#define AVE_CAP_VALUE_SPM_BASE		0x9A
#define DIF_CAP_VALUE_SPM_BASE		0xB4

/* monitor period = n * 15ms */
#define MONITOR_PERIOD_195MS		13
#define MONITOR_PERIOD_150MS		10
#define MONITOR_PERIOD_60MS		4
#define CHANNEL_NUM		8

#define NEAR_ZERO(x)	(((x > -100) && (x < 175)) ? 1 : 0)
#define AVE_NEGATIVE_COMP_VALUE	-1000
#define PROX_OFFSET_HIGH_CHECK		550

/* P-Sensor driver mode setting */
enum sx863x_driver_mode {NORMAL_MODE, MONITOR_MODE};
enum sx863x_driver_mode P_SensorDriverMode = MONITOR_MODE;

unsigned char read_dif_in_normal = 0;

/*
 * keep track of the state for each button.
 */
enum sx863x_button_state { ACTIVE, IDLE };

struct sx863x_button_drv {
	enum sx863x_button_state state;

	struct input_dev *input;
};
/*
 *  pointer for each feature of the device.
 *  When more devices are implemented in the driver, this struct will contain
 *  those (e.g. slider, wheel).
 */
struct sx863x_driver_data {
	struct sx863x_button_drv *button;
};

/*
 * information to integrate all specific items in code
 */
struct sx863x_chip {

	unsigned char lastIrqSrc; /* updated when reading interrupt */
  
	/* Updated whenever their respective flags in irqsrc occur */
	unsigned char gpistat_Value;
	unsigned char capstatelsb_Value;
	unsigned char compopmode_Value;
	unsigned char spmstat_Value;

	struct sx863x_platform_data *hw;
	struct sx863x_driver_data *sw;
  
	int irq;            /* irq used */
	struct device *dev; /* pointer to device */

	/* pointers to functions for read/write */
	sx863x_write_t write;
	sx863x_writeEx_t writeEx;
	sx863x_read_t read;
	sx863x_readEx_t readEx;

	struct mutex mutex;
	struct mutex fn_mutex;
	struct mutex fs_mutex;

	unsigned product;
	unsigned version;
};

/* Notify the sensor change status to power reduction controller */
extern void sensor_status_changed();

struct sx863x_chip *my_sx863x_chip;

/* Proximity status */
enum sx863x_proximity_status pre_proximity_status = CAP_SENSE_PROX;
enum sx863x_proximity_status proximity_status = CAP_SENSE_PROX;
enum sx863x_proximity_status cap0_proximity_status = CAP_SENSE_PROX;	/* TOP CENTER */
enum sx863x_proximity_status cap4_proximity_status = CAP_SENSE_PROX;	/* TOP EDGE */

#define I2C_RETURN_CHECK(ret)		\
	if (ret < 0)	\
	{	\
		proximity_status = CAP_SENSE_PROX;	\
		cap0_proximity_status = CAP_SENSE_PROX; \
		cap4_proximity_status = CAP_SENSE_PROX;	\
		if (proximity_status != pre_proximity_status) {	\
			sensor_status_changed();	\
			pre_proximity_status = proximity_status;	\
		}	\
		if (my_sx863x_chip != NULL && my_sx863x_chip->irq > 0)		\
			disable_irq_nosync(my_sx863x_chip->irq);	\
	}		\
	return ret;

unsigned char sx8636_data_logging_enable = 0;
unsigned char sx8636_debug_log_enable = 0;
unsigned long logIndex = 0;

signed short CapAdcValueArray_raw[8];
signed short CapAdcValueArray_ave[8];
signed short CapAdcValueArray_dif[8];

unsigned char chan_mask = 0;
unsigned char prox_detect_debounce = 0;
unsigned char prox_release_debounce = 0;
unsigned char prox_status;
unsigned char touch_status;
signed short prox_detect_thresh[CHANNEL_NUM];
signed short touch_detect_thresh[CHANNEL_NUM];
signed short touch_release_thresh[CHANNEL_NUM];
unsigned char prox_detect_hyst[CHANNEL_NUM];
unsigned char use_filter_alpha[CHANNEL_NUM];
unsigned char ave_filter_alpha[CHANNEL_NUM];
unsigned long ave_data_over_touch_flag[CHANNEL_NUM];
unsigned char pre_touch_status[CHANNEL_NUM];
signed short ave_data_over_touch[CHANNEL_NUM];
signed short raw_data[CHANNEL_NUM];
signed short use_data[CHANNEL_NUM];
signed short ave_data[CHANNEL_NUM];
signed short dif_data[CHANNEL_NUM];
signed short prox_offset[CHANNEL_NUM];
unsigned char debounce_pos_count[CHANNEL_NUM];
unsigned char debounce_neg_count[CHANNEL_NUM];

#define SPM_SIZE	128
unsigned char p_sensor_bin[SPM_SIZE];
unsigned char p_sensor_bin_found = 0;

unsigned char scan_period = MONITOR_PERIOD_195MS;
unsigned char monitor_scan_period  = MONITOR_PERIOD_195MS;
unsigned char monitor_rate_changed = 0;

static struct task_struct *kmain_task = NULL;
static wait_queue_head_t kthread_wait;

/*
 * Update SPM memory 
 */
static int sx863x_update_spm_8block(struct sx863x_chip *sx863x,
                                    unsigned char baseAddress, unsigned char *pSpmBuffer)
{
	int ret = 0;
	dev_dbg(sx863x->dev, "inside sx863x_update_spm_block()\n");

	mutex_lock(&sx863x->fn_mutex);

	/* reset in case no interrupt */
	sx863x->lastIrqSrc &= ~SX863X_IRQSRC_SPM_WRITE_FLAG; 

	ret = sx863x->read(sx863x->dev, SX863X_COMPOPMODE_REG, &sx863x->compopmode_Value);
	if (unlikely(ret < 0)) goto function_end;

	ret = sx863x->write( sx863x->dev, SX863X_SPMCFG_REG, SX863X_SPMCFG_I2CSPM_ON );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	ret = sx863x->write( sx863x->dev, SX863X_SPMBASEADDR_REG, baseAddress );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	ret = sx863x->writeEx( sx863x->dev, 0x00, pSpmBuffer, 8 );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	ret = sx863x->write( sx863x->dev, SX863X_SPMCFG_REG, SX863X_SPMCFG_I2CSPM_OFF );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */

	/* instead of looking at interrupt, just wait 300ms. Since this is only done
	 * on startup, this shouldn't affect performance 
	 */

	if (0) /* (( sx863x->compopmode_Value & SX863X_COMPOPMODE_OPERATINGMODE_MSK ) != 
                                  SX863X_COMPOPMODE_OPERATINGMODE_SLEEP )*/
	{

		/* should we have msleep() ?? */
		/* relies on interrupt process to update lastIrqSrc */
		while (!(sx863x->lastIrqSrc & SX863X_IRQSRC_SPM_WRITE_FLAG ))
		{
			dev_dbg(sx863x->dev, "going to check if irqsrc shows spm wrote\n");
			ret = sx863x->read(sx863x->dev, SX863X_IRQSRC_REG, &sx863x->lastIrqSrc);
			if (unlikely(ret < 0)) goto function_end;
			dev_dbg(sx863x->dev, "irqsrc is showing 0x%x\n",sx863x->lastIrqSrc);
		}
	}
	else
	{
		dev_dbg(sx863x->dev, "going to wait 300ms for programing complete\n");
		msleep(300); /* Wait for 300ms */
	}
	dev_dbg(sx863x->dev, "leaving sx863x_update_spm_block()\n");

function_end:
	mutex_unlock(&sx863x->fn_mutex);

	return ret;
}

/*
 * Read from SPM memory
 */
static int sx863x_retrieve_spm_8block(struct sx863x_chip *sx863x,
                                    unsigned char baseAddress, unsigned char *pSpmBuffer)
{
	int ret = 0;
	dev_dbg(sx863x->dev, "inside sx863x_retrieve_spm_8block()\n");

	mutex_lock(&sx863x->fn_mutex);

	ret = sx863x->write( sx863x->dev, SX863X_SPMCFG_REG, 
                         SX863X_SPMCFG_I2CSPM_ON | SX863X_SPMCFG_RWSPM_READ );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	ret = sx863x->write( sx863x->dev, SX863X_SPMBASEADDR_REG, baseAddress );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	ret = sx863x->readEx( sx863x->dev, 0x00, pSpmBuffer, 8 );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	ret = sx863x->write( sx863x->dev, SX863X_SPMCFG_REG, SX863X_SPMCFG_I2CSPM_OFF );
	if (unlikely(ret<0)) goto function_end; /* ERROR! */
	dev_dbg(sx863x->dev, "leaving sx863x_retrieve_spm_8block()\n");

function_end:
	mutex_unlock(&sx863x->fn_mutex);
	
	return ret;
}

static int sx863x_update_spm_single(struct sx863x_chip *sx863x, unsigned char spmAddress, unsigned char *data)
{
	int ret = 0;
	unsigned char offset = 0;
	unsigned char spmDataBuf[8];
	unsigned char spmBaseAddress;

	spmBaseAddress = 0xF8 & spmAddress;
	offset = spmAddress - spmBaseAddress;

	/* Read 8 bytes data from spmBaseAddress */
	ret = sx863x_retrieve_spm_8block(sx863x, spmBaseAddress, spmDataBuf);
	if (unlikely(ret < 0)) return ret;

	spmDataBuf[offset] = data[0];
	/* Write 8 bytes data into spmBaseAddress */
	ret = sx863x_update_spm_8block(sx863x, spmBaseAddress, spmDataBuf);
	if (unlikely(ret < 0)) return ret;

	return ret;
}

static int sx863x_retrieve_spm_single(struct sx863x_chip *sx863x, unsigned char spmAddress, unsigned char *data)
{
	int ret = 0;
	unsigned char offset = 0;
	unsigned char spmDataBuf[8];
	unsigned char spmBaseAddress;

	spmBaseAddress = 0xF8 & spmAddress;
	offset = spmAddress - spmBaseAddress;

	/* Read 8 bytes data from spmBaseAddress */
	ret = sx863x_retrieve_spm_8block(sx863x, spmBaseAddress, spmDataBuf);
	if (unlikely(ret < 0)) return ret;

	*data = spmDataBuf[offset];

	return ret;
}

/*
 * Write Register(s), allows data array
 */
static int sx863x_i2c_writeEx(struct device *dev, unsigned char reg,
				unsigned char *data, int size)
{
	u8 tx[MAX_WRITE_ARRAY_SIZE+1];
	struct i2c_client *client = 0;
	int ret = 0;
	dev_dbg(dev, "inside sx863x_i2c_writeEx()\n");

	client = to_i2c_client(dev);

	if (data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		tx[0] = reg;
		dev_dbg(dev, "going to call i2c_master_send(0x%x, 0x%x ", (unsigned int)client, tx[0]);
		for (ret = 0; ret < size; ret++)
		{
			tx[ret+1] = data[ret];
			dev_dbg(dev, "0x%x, ",tx[ret+1]);
		}
		dev_dbg(dev, "\n");

		ret = i2c_master_send(client, tx, size+1 );
		if (ret < 0)
			dev_err(&client->dev, "I2C write error\n");
	}
	dev_dbg(dev, "leaving sx863x_i2c_writeEx()\n");

	I2C_RETURN_CHECK(ret);
}

/*
 * Read Register(s), allows data array
 */
static int sx863x_i2c_readEx(struct device *dev, unsigned char reg,
				unsigned char *data, int size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	u8 tx[] = {
		reg
	};
	if (data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_dbg(dev, "inside sx863x_i2c_readEx()\n");
		dev_dbg(dev, "going to call i2c_master_send(0x%x,0x%x,1) Reg: 0x%x\n", (unsigned int)client, (unsigned int)tx, (unsigned int)tx[0]);
		ret = i2c_master_send(client, tx, 1);
		if (ret >= 0)
		{
			dev_dbg(dev, "going to call i2c_master_recv(0x%x,0x%x,%x)\n", (unsigned int)client, (unsigned int)data, size);
			ret = i2c_master_recv(client, data, size);
		}
	}
	if (unlikely(ret < 0))
		dev_err(dev, "I2C read error\n");

	dev_dbg(dev, "leaving sx863x_i2c_readEx()\n");

	I2C_RETURN_CHECK(ret);
}

/*
 * Write Register
 */
static int sx863x_i2c_write(struct device *dev, unsigned char reg,
				unsigned char data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	u8 tx[2] = {
		reg,
		data
	};

	dev_dbg(dev, "inside sx863x_i2c_write()\n");
	dev_dbg(dev, "going to call i2c_master_send(0x%x,0x%x,2) Reg: 0x%x Val: 0x%x\n", (unsigned int)client, (unsigned int)tx, (unsigned int)tx[0], (unsigned int)tx[1]);
	ret = i2c_master_send(client, tx, 2);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	dev_dbg(dev, "leaving sx863x_i2c_write()\n");

	I2C_RETURN_CHECK(ret);
}

/*
 * Read Register
 */
static int sx863x_i2c_read(struct device *dev, unsigned char reg,
				unsigned char *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	u8 tx[] = {
		reg
	};
	u8 rx[1];

	dev_dbg(dev, "inside sx863x_i2c_read()\n");
	dev_dbg(dev, "going to call i2c_master_send(0x%x,0x%x,1) Reg: 0x%x\n", (unsigned int)client, (unsigned int)tx, (unsigned int)tx[0]);
	ret = i2c_master_send(client, tx, 1);
	if (ret >= 0)
	{
		dev_dbg(dev, "going to call i2c_master_recv(0x%x,0x%x,1)\n", (unsigned int)client, (unsigned int)rx);
		ret = i2c_master_recv(client, rx, 1);
	}

	if (unlikely(ret < 0)) {
		dev_err(dev, "I2C read error\n");
	} else {
		*data = rx[0];
	}
	dev_dbg(dev, "leaving sx863x_i2c_read(0x%x)\n",*data);

	I2C_RETURN_CHECK(ret);
}

#if 0
/*
 * Check the NVM is valid or not
 *
 */
static int sx863x_check_NVM(struct sx863x_chip *sx863x)
{
	int ret = 0;
	unsigned char data;
	unsigned char count;

	ret = sx863x->read(sx863x->dev, SX863X_SPMSTAT_REG, &data);
	if (unlikely(ret < 0)) return ret;

	printk(KERN_INFO "%s: the data = 0x%02x\n", __func__, data);

	if ((data & 0x08) == 0x08)		// NVM is valid
	{
		count = data & 0x07;
		printk(KERN_INFO "%s: the number of times NVM has been burned = %d\n", __func__, count);
		return count;
	} else {
		return 0;
	}
}
#endif

/*
 * Check the /configs/p_sensor.bin is exist or not
 * If p_sensor.bin is exist, copy the data to SPM registers
 *
 */
static int sx863x_check_eMMC(void)
{
	struct file *fp;
	int bytes_read = 0;
	mm_segment_t old_fs;
	loff_t offset = 0;

	printk(KERN_INFO "%s called\n", __func__);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	fp = filp_open("/configs/p_sensor.bin", O_RDONLY, 0);

	if (IS_ERR_OR_NULL(fp)) {
		p_sensor_bin_found = 0;
		printk(KERN_INFO "/configs/p_sensor.bin is not found\n");
		set_fs(old_fs);
	} else {
		printk(KERN_INFO "/configs/p_sensor.bin is found\n");
		if ((fp->f_op != NULL) && (fp->f_op->read != NULL)) {
			bytes_read = fp->f_op->read(fp, p_sensor_bin, SPM_SIZE, &offset);
			if (!bytes_read) {				
				filp_close(fp, NULL);
				return -EBADF;
			}

			p_sensor_bin_found = 1;
		}
		filp_close(fp, NULL);
		set_fs(old_fs);
	}

	return p_sensor_bin_found;
}

/*
 * Copy the necessary value to SPM address if the p_sensor.bin is found in /configs
 * 
 *
 */
static int sx863x_update_spm_from_binary(struct sx863x_chip *sx863x)
{
	unsigned char data;
	int ret = 0;
	int i = 0;

	printk(KERN_INFO "%s called\n", __func__);
	for (i = 0; i < SPM_SIZE; i++)
	{
		if (i == SX863X_ACTIVESCANPERIOD_SPM /* 0x05*/ ||
			i == SX863X_CAPEMODEMISC_SPM /* 0x09 */ ||
			i == SX863X_CAPMODE7_4_SPM /* 0x0B */ ||
			i == SX863X_CAPMODE3_0_SPM /* 0x0C */ ||
			i == SX863X_CAPSENSITIVITY0_1_SPM /* 0x0D */ ||
			i == SX863X_CAPSENSITIVITY2_3_SPM /* 0x0E */ ||
			i == SX863X_CAPSENSITIVITY4_5_SPM /* 0x0F */ ||
			i == SX863X_CAPSENSITIVITY6_7_SPM /* 0x10 */ ||
			i == SX863X_CAPTHRESH0_SPM /* 0x13 */ ||
			i == SX863X_CAPTHRESH1_SPM /* 0x14 */ ||
			i == SX863X_CAPTHRESH2_SPM /* 0x15 */ ||
			i == SX863X_CAPTHRESH3_SPM /* 0x16 */ ||
			i == SX863X_CAPTHRESH4_SPM /* 0x17 */ ||
			i == SX863X_CAPTHRESH5_SPM /* 0x18 */ ||
			i == SX863X_CAPTHRESH6_SPM /* 0x19 */ ||
			i == SX863X_CAPTHRESH7_SPM /* 0x1A */ ||
			i == SX863X_CAPPERCOMP_SPM /* 0x1F */ ||
			i == SX863X_BTNCFG_SPM /* 0x21 */ ||
			i == SX863X_BTNAVGTHRESH_SPM /* 0x22 */ ||
			i == SX863X_BTNCOMPNEGTHRESH_SPM /* 0x23 */ ||
			i == SX863X_BTNCOMPNEGCNTMAX_SPM /* 0x24 */ ||
			i == SX863X_BTNHYSTERESIS_SPM /* 0x25 */ ||
			i == SX863X_BTNSTUCKATTIMEOUT_SPM /* 0x26 */ ||
			i == SX863X_CAPPROXENABLE_SPM /* 0x70 */)
		{
			ret = sx863x_retrieve_spm_single(sx863x, i, &data);
			if (unlikely(ret < 0)) return ret;

			if (data != p_sensor_bin[i])
			{
				/* Only changes if current value is not equal to p_sensor.bin */
				printk(KERN_INFO "[0x%02X] orig: 0x%02X, now: 0x%02X\n", i, data, p_sensor_bin[i]);
				data = p_sensor_bin[i];
				ret = sx863x_update_spm_single(sx863x, i, &data);
				if (unlikely(ret < 0)) return ret;
			}
		}
	}

	return ret;
}

/*
 * Update the current proximity status (Cap0/Cap4)
 *
 */
enum sx863x_proximity_status sx863x_update_proximity_status(struct sx863x_chip *sx863x)
{
	int i;
	unsigned char proc_mask = 0x01;	/* Start from Cap0 */

	if (P_SensorDriverMode == NORMAL_MODE)
	{
		for (i = 0; i < sx863x->hw->button_num; i++)
		{
			if (sx863x->sw->button[i].state == ACTIVE)
			{
				if (i == 0)
					cap0_proximity_status = CAP_SENSE_PROX;		// Cap0
				else if (i == 1)
					cap4_proximity_status = CAP_SENSE_PROX;	// Cap4
			}
			if (sx863x->sw->button[i].state == IDLE)
			{
				if (i == 0)
					cap0_proximity_status = CAP_SENSE_NO_PROX;		// Cap0
				else if (i == 1)
					cap4_proximity_status = CAP_SENSE_NO_PROX;	// Cap4
			}
		}

		if (sx8636_debug_log_enable) {
			printk(KERN_INFO "[Cap0, Cap4] = %d, %d\n", cap0_proximity_status, cap4_proximity_status);
		}

		if (cap0_proximity_status == CAP_SENSE_PROX ||
			cap4_proximity_status == CAP_SENSE_PROX)
			return CAP_SENSE_PROX;
		else
			return CAP_SENSE_NO_PROX;
	} 
	else if (P_SensorDriverMode == MONITOR_MODE)
	{
		for (i = 0; i < CHANNEL_NUM; i++)
		{
			if ((prox_status & proc_mask) || (touch_status & proc_mask))
			{
				if (proc_mask == 0x01)	// channel 0
					cap0_proximity_status = CAP_SENSE_PROX;
				else if (proc_mask == 0x10)		// channel 4
					cap4_proximity_status = CAP_SENSE_PROX;
			} else if ((prox_status & proc_mask) == 0 && (touch_status & proc_mask) == 0)
			{
				if (proc_mask == 0x01)	// channel 0
					cap0_proximity_status = CAP_SENSE_NO_PROX;
				else if (proc_mask == 0x10)		// channel 4
					cap4_proximity_status = CAP_SENSE_NO_PROX;
			}

			proc_mask  = proc_mask << 1;
		}

		if (sx8636_debug_log_enable) {
			printk(KERN_INFO "[Cap0, Cap4] = %d, %d\n", cap0_proximity_status, cap4_proximity_status);
		}

		if ((cap0_proximity_status == CAP_SENSE_PROX) || 
			(cap4_proximity_status == CAP_SENSE_PROX))
		{
			return CAP_SENSE_PROX;
		}
		else
		{
			return CAP_SENSE_NO_PROX;
		}
	}
	else		/* Unknown P-Sensor Driver Mode */
	{
		printk(KERN_ERR "%s: Unknown P-Sensor driver mode\n", __func__);

		return CAP_SENSE_NO_PROX;
	}
}

/*
 * Read the CapX adc value
 *
 */
static int sx863x_read_cap_adc_value(struct sx863x_chip *chip, unsigned char spm_base, signed short *CapValue)
{
	int ret = 0;
	
	unsigned char HighCapValue, LowCapValue;
	
	ret = sx863x_retrieve_spm_single(chip, spm_base, &HighCapValue);
	if (unlikely(ret < 0)) return ret;
	ret = sx863x_retrieve_spm_single(chip, spm_base + 1, &LowCapValue);
	if (unlikely(ret < 0)) return ret;
	
	*CapValue = (signed short)((HighCapValue << 8) | LowCapValue);

	return ret;
}

/*
 * Should be called when an interrupt occurs with the button flag.
 * This relies on capstatelsb_Mask being updated before entering
 * the function.
 *
 */
static void sx863x_button_state(struct sx863x_chip *sx863x, int idx)
{
	struct sx863x_button_platform_data *hw = &sx863x->hw->button[idx];
	struct sx863x_button_drv *sw = &sx863x->sw->button[idx];

	dev_dbg(sx863x->dev, "inside sx863x_button_state() State: %d CapState: 0x%x Mask: 0x%x\n",
				sw->state,sx863x->capstatelsb_Value,hw->capstatelsb_Mask);
	SX8636_LOG("%s called\n", __func__);

	switch (sw->state) {
  	case IDLE: /* Button is not being touched! */
	  	if (((sx863x->capstatelsb_Value & hw->capstatelsb_Mask) 
                                          == hw->capstatelsb_Mask)) {
			/* User pressed button */
  			dev_info(sx863x->dev, "cap button %d touched\n", idx);
			SX8636_LOG("cap button %d touched\n", idx);
	  		//input_report_key(sw->input, hw->keycode, 1);
		  	//input_sync(sw->input);
			sw->state = ACTIVE;
  		}
		else
		{
			dev_dbg(sx863x->dev, "Button %d already released.\n",idx);
		}
	  	break;
  	case ACTIVE: /* Button is being touched! */
	  	if (((sx863x->capstatelsb_Value & hw->capstatelsb_Mask) 
                                          != hw->capstatelsb_Mask)) {
			/* User released button */
		  	dev_info(sx863x->dev, "cap button %d released\n", idx);
			SX8636_LOG("cap button %d released\n", idx);
			//input_report_key(sw->input, hw->keycode, 0);
  			//input_sync(sw->input);
	  		sw->state = IDLE;
		}
		else
		{
			dev_dbg(sx863x->dev, "Button %d still touched.\n",idx);
		}
  		break;
	default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
		break;
	};

	/* Update proximity status */
	proximity_status = sx863x_update_proximity_status(sx863x);
#if 1	/* Report to PRC if proximity status changed */
	if (proximity_status != pre_proximity_status) {
		sensor_status_changed();	// Notify to power reduction controller driver
		pre_proximity_status = proximity_status;
	}
#endif

	SX8636_LOG("proximity status = %d\n", proximity_status);

  	dev_dbg(sx863x->dev, "leaving sx863x_button_state()\n");
}

/* 
 * Reset sx8636 chip
 *
 */
static int sx863x_software_reset(struct sx863x_chip *sx863x)
{
	int ret = 0;

	printk(KERN_INFO "%s called\n", __func__);
	ret = sx863x->write(sx863x->dev, SX863X_SOFTRESET_REG, 0xDE);
	if (unlikely(ret < 0)) return ret;
	ret = sx863x->write(sx863x->dev, SX863X_SOFTRESET_REG, 0x00);
	if (unlikely(ret < 0)) return ret;

	/* Initialize the proximity_status after chip reset */
	cap0_proximity_status = CAP_SENSE_NO_PROX;
	cap4_proximity_status = CAP_SENSE_NO_PROX;
	proximity_status = CAP_SENSE_NO_PROX;

	msleep(300);

	return ret;
}

/* 
 * Currently, only SX8636 is valid, but can use this to make sure
 * other functions that may be specific to certain devices are used
 * when appropriate.
 *
 */
static int sx863x_hw_detect(struct sx863x_chip *sx863x)
{
	int ret = 0;
	unsigned char product_id;

	if (unlikely(!sx863x))
	{
		dev_err(sx863x->dev, "sx8653x_hw_detect() was passed in a null value!\n");
		return -ENODEV;
	}
#if defined(CONFIG_INPUT_SX8636)
	ret = sx863x_update_spm_single(sx863x, SX863X_PRODUCT_ID_SPM, &product_id);
	if (unlikely(ret < 0)) return ret;

	sx863x->product = 0x8636;
	dev_dbg(sx863x->dev, "found SX8636\n");
	sx863x->version = 1;
	return 0;
#endif

	dev_err(sx863x->dev,
		"could not determine device (SX863X)\n");
	return -ENODEV;
}

/*
 * This should normally only be called inside the interrupt
 * handler. 
 */
static int sx863x_process_interrupt(struct sx863x_chip *sx863x)
{
	int ret = 0;
	int i;
	/* IrqSrc Register contains information needed to know what triggered interrupt */
	dev_dbg(sx863x->dev, "inside sx863x_process_interrupt()\n");
	SX8636_LOG("%s called\n", __func__);

	ret = sx863x->read(sx863x->dev, SX863X_IRQSRC_REG, &sx863x->lastIrqSrc);
	if (unlikely(ret < 0)) return ret;

	if (sx8636_debug_log_enable)
		printk(KERN_INFO "IRQ source register = 0x%02X\n", sx863x->lastIrqSrc);

	if ( ((sx863x->lastIrqSrc) & SX863X_IRQSRC_SPM_WRITE_FLAG) ||
			((sx863x->lastIrqSrc) & SX863X_IRQSRC_NVM_BURN_FLAG) ) {
		ret = sx863x->read(sx863x->dev, SX863X_SPMSTAT_REG, &sx863x->spmstat_Value);
		if (unlikely(ret < 0)) return ret; 
	}
	if ((sx863x->lastIrqSrc) & SX863X_IRQSRC_GPI_FLAG) {
		ret = sx863x->read(sx863x->dev, SX863X_GPISTAT_REG, &sx863x->gpistat_Value);
		if (unlikely(ret < 0)) return ret;
	}
	if ((sx863x->lastIrqSrc) & SX863X_IRQSRC_BUTTON_FLAG) {
		if (P_SensorDriverMode != MONITOR_MODE)	// don't update button status if monitor mode is enabled
		{
			ret = sx863x->read(sx863x->dev, SX863X_CAPSTATLSB_REG, &sx863x->capstatelsb_Value);
			if (unlikely(ret < 0)) return ret;

			for (i = 0; i < sx863x->hw->button_num; i++) /* Update Button States */
			{
				dev_dbg(sx863x->dev, "going to call sx863x_button_state(0x%x,0x%x)\n", (unsigned int)sx863x,i);
				sx863x_button_state(sx863x, i);
				dev_dbg(sx863x->dev, "finished calling sx863x_button_state(0x%x,0x%x)\n", (unsigned int)sx863x,i);
			}
		}
	}
	if ( ((sx863x->lastIrqSrc) & SX863X_IRQSRC_OPERATING_MODE_FLAG) ||
			((sx863x->lastIrqSrc) & SX863X_IRQSRC_COMPENSATION_FLAG) ) {
		ret = sx863x->read(sx863x->dev, SX863X_COMPOPMODE_REG, &sx863x->compopmode_Value);
		if (ret < 0)
			return ret;
	}
	dev_dbg(sx863x->dev, "leaving sx863x_process_interrupt()\n");

	return ret;
}

/*
 * Intiialize SPM Config from board source code
 */
static int sx863x_hw_spm_cfg_init(struct sx863x_chip *sx863x)
{
	int ret = 0;
	unsigned char spmBlock[8];
	unsigned char spmBaseAddress = 0;
	int i = 0;
	int j = 0;
	unsigned char *pSpmBlock = 0;
	unsigned char data = 0;

	/* Program device to go into sleep mode */
	ret = sx863x->read(sx863x->dev, SX863X_COMPOPMODE_REG, &data);
	if (unlikely(ret < 0)) return ret;
	data = data & ~SX863X_COMPOPMODE_OPERATINGMODE_MSK;
	data = data | SX863X_COMPOPMODE_OPERATINGMODE_SLEEP;
	ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, data);
	if (unlikely(ret < 0)) return ret;

	msleep(300); /* Wait for 300ms */
  
	while ( i < sx863x->hw->spm_cfg_num)
	{
		spmBaseAddress = 0xF8 & sx863x->hw->spm_cfg[i].reg;
		pSpmBlock = &spmBlock[0];
		dev_dbg( sx863x->dev, "Going to Read Spm Block Start: 0x%x\n", spmBaseAddress);
		ret = sx863x_retrieve_spm_8block(sx863x,spmBaseAddress,pSpmBlock);
		if (unlikely(ret < 0)) return ret;
			
		dev_dbg( sx863x->dev, "Read Spm Block StartReg: 0x%x\n", spmBaseAddress);
		for( j = (sx863x->hw->spm_cfg[i].reg - spmBaseAddress); j < 8; j++ )
		{
			/* write out as many config parameters that belong to current block */
			if((spmBaseAddress+j) == sx863x->hw->spm_cfg[i].reg)
			{
				spmBlock[j] = sx863x->hw->spm_cfg[i].val;
				i++;
			}
			else
				break;
		}
    
		dev_dbg( sx863x->dev, "Going to Write Spm Block Start: 0x%x\n",spmBaseAddress);
		ret = sx863x_update_spm_8block(sx863x,spmBaseAddress,spmBlock);
		if (unlikely(ret < 0)) return ret;

		dev_dbg( sx863x->dev, "Wrote Spm Block\n");
	}

	/* Program device to go into active mode */
	ret = sx863x->read(sx863x->dev, SX863X_COMPOPMODE_REG, &data);
	if (unlikely(ret < 0)) return ret;

	data = data & ~SX863X_COMPOPMODE_OPERATINGMODE_MSK;
	//data = data | SX863X_COMPOPMODE_OPERATINGMODE_ACTIVE;
	data = data | P_SENSOR_INITIAL_OPERATION_MODE;
	printk(KERN_INFO "%s: sx863x initial mode = %d\n", __func__, P_SENSOR_INITIAL_OPERATION_MODE);
	ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, data);
	if (unlikely(ret < 0)) return ret;

	msleep(300); /* Wait for 300ms */

	return ret;
}

/*
 * initialize hardware
 */
static int sx863x_hw_init(struct sx863x_chip *sx863x)
{
	int ret = 0;
	int i = 0;
	/* configure device */

	/* Write all registers/values contained in i2c_reg */
	for (i = 0; i < sx863x->hw->i2c_reg_num; i++)
	{
		dev_dbg(sx863x->dev, "Going to Write Reg: 0x%x Value: 0x%x\n", sx863x->hw->i2c_reg[i].reg,sx863x->hw->i2c_reg[i].val);
		ret = sx863x->write(sx863x->dev, sx863x->hw->i2c_reg[i].reg,sx863x->hw->i2c_reg[i].val);
		if (unlikely(ret < 0)) return ret;
	}

	ret = sx863x_hw_spm_cfg_init(sx863x);
	if (unlikely(ret < 0)) return ret;

	return ret;
}

/*
 * Configure the sx8636 into monitor mode
 */
static int sx863x_monitor_mode_enable(struct sx863x_chip *sx863x, int time)
{
	int ret = 0;
	unsigned char data;

	SX8636_LOG("%s is called. en = %d\n", __func__, en);
	
	/* Enable monitor mode */
	ret = sx863x->read(sx863x->dev, SX863X_SPMCFG_REG, &data);
	if (unlikely(ret < 0)) return ret;
	data |= 0x04;	/* Set bit[2] = 1 */
	ret = sx863x->write(sx863x->dev, SX863X_SPMCFG_REG, data);
	if (unlikely(ret < 0)) return ret;

	/* Set the monitor rate (n * 15ms) */
	monitor_rate_changed = 1;
	monitor_scan_period = time;

	return ret;
}

/*
 * Initialize the related variables for touch/prox detect/release calculation on host side.
 * This is only used for P-Sensor driver monitor mode.
 *
 */
static int sx863x_initialize_prox_variables(struct sx863x_chip *sx863x)
{
	int ret;
	int i;
	unsigned char data = 0;

	for (i = 0; i < CHANNEL_NUM; i++)
	{
		prox_detect_thresh[i] = 0;
		touch_detect_thresh[i] = 0;
		touch_release_thresh[i] = 0;
		prox_detect_hyst[i] = 0;
		use_filter_alpha[i] = 0;
		ave_filter_alpha[i] = 0;
		use_data[i] = 0;
		ave_data[i] = 0;
		dif_data[i] = 0;
		prox_offset[i] = 0;
		ave_data_over_touch_flag[i] = 0;
		ave_data_over_touch[i] = 0;
		pre_touch_status[i] = 0;
	}

	chan_mask = 0x11;	// channel 0, 4

	// define prox detect/release debounce
	ret = sx863x_retrieve_spm_single(sx863x, SX863X_BTNCFG_SPM, &data);
	if (unlikely(ret < 0)) return ret;
	prox_detect_debounce = ((data & 0x0C) >> 2) + 1;
	prox_release_debounce = (data & 0x03) + 1;

	printk(KERN_INFO "%s: detect debounce = %d, release debounce = %d\n", __func__, prox_detect_debounce, prox_release_debounce);

	// define prox threshold
	ret = sx863x_retrieve_spm_single(sx863x, SX863X_CAPTHRESH0_SPM, &data);
	if (unlikely(ret < 0)) return ret;
	prox_detect_thresh[0] = data * 4;
	ret = sx863x_retrieve_spm_single(sx863x, SX863X_CAPTHRESH4_SPM, &data);
	if (unlikely(ret < 0)) return ret;
	prox_detect_thresh[4] = data * 4;

	printk(KERN_INFO "%s: Cap0 threshold = %d, Cap4 threshold = %d\n", __func__, prox_detect_thresh[0], prox_detect_thresh[4]);

	// Calculate prox hysteresis
	ret = sx863x_retrieve_spm_single(sx863x, SX863X_BTNHYSTERESIS_SPM, &data);
	if (unlikely(ret < 0)) return ret;
	if (data > 0)
	{
		prox_detect_hyst[0] = (prox_detect_thresh[0] / data);
		prox_detect_hyst[4] = (prox_detect_thresh[4] / data);
	} else {
		prox_detect_hyst[0] = 0;
		prox_detect_hyst[4] = 0;
	}

	printk(KERN_INFO "%s: Cap0 hyst = %d, Cap4 hyst = %d\n", __func__, prox_detect_hyst[0], prox_detect_hyst[4]);
	// define touch detect threshold
	touch_detect_thresh[0] = 1300;
	touch_detect_thresh[4] = 1300;
	printk(KERN_INFO "Touch detect threshold [Cap0, Cap4] = [%d, %d]\n", touch_detect_thresh[0], touch_detect_thresh[4]);

	// define touch release threshold
	touch_release_thresh[0] = 500;
	touch_release_thresh[4] = 500;
	printk(KERN_INFO "Touch release threshold [Cap0, Cap4] = [%d, %d]\n", touch_release_thresh[0], touch_release_thresh[4]);

	use_filter_alpha[0] = 1;
	use_filter_alpha[4] = 1;
	printk(KERN_INFO "Use filter alpha [Cap0, Cap4] = [%d, %d]\n", use_filter_alpha[0], use_filter_alpha[4]);
	
	ave_filter_alpha[0] = 5;
	ave_filter_alpha[4] = 5;
	printk(KERN_INFO "Ave filter alpha [Cap0, Cap4] = [%d, %d]\n", ave_filter_alpha[0], ave_filter_alpha[4]);

	prox_status = 0;
	touch_status = 0;

	return ret;
}

/*
 * Check the /configs/p_sensor.bin is exist or not
 * Initialize variables
 * Configure the P-Sensor driver operation mode (Normal mode or Monitor mode)
 *
 */
static int sx863x_delay_check(void *arg)
{
	struct sx863x_chip *my_chip = my_sx863x_chip;
	int ret;
	unsigned char status = 0;
	unsigned char data;

	printk(KERN_INFO "%s call\n", __func__);
	sleep_on_timeout(&kthread_wait, 5 * HZ);

	printk(KERN_INFO "Checking p_sensor.bin in /configs\n");
	status = sx863x_check_eMMC();

	if (status == 1) {	// /configs/p_sensor.bin is found
		//Copy data from bin file to SPM address
		ret = sx863x_update_spm_from_binary(my_chip);
		if (ret < 0) return ret;
	} else {
		printk(KERN_INFO "[Error] p_sensor.bin is not exist!!!\n");		
		return -1;
	}

	// Read the scaning period
	ret = sx863x_retrieve_spm_single(my_chip, SX863X_ACTIVESCANPERIOD_SPM, &data);
	if (ret < 0) {
		printk(KERN_INFO "Fail to retrieve SPM register 0x%02X\n", SX863X_ACTIVESCANPERIOD_SPM);
		return ret;
	}

	printk(KERN_INFO "%s: scan period = 0x%02X\n", __func__, data);
				
	scan_period = data;

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		// Initialize the prox variables for monitor usage
		ret = sx863x_initialize_prox_variables(my_chip);
		if (ret < 0) {
			printk(KERN_INFO "Fail to initialize the variables\n");
			return ret;
		}
				
		// Configure the sx8636 into monitor mode
		ret = sx863x_monitor_mode_enable(my_chip, scan_period);
		if (ret < 0) {
			printk(KERN_INFO "Fail to enable monitor mode\n");
			return ret;
		}
	}

	return ret;
}

/*
 *  Process the raw data to calculate use/ave/dif data
 *  Update touch detect/release & prox detect/release status
 *
 */
static int process_raw_data_norm(struct sx863x_chip *sx863x, signed short *raw)
{
	int ret = 0;
	int i = 0;
	int bytes_write = 0;
	unsigned char proc_mask = 1;
	//struct file *fp_raw, *fp_use, *fp_ave, *fp_dif;
	struct file *fp_dif;
	unsigned char temp_buf[100];

	SX8636_LOG("%s: chan_mask = 0x%02X\n", __func__, chan_mask);
	for (i = 0; i <  CHANNEL_NUM; i++)
	{
		if ((chan_mask & proc_mask) == proc_mask)	// check to see if current channel is active channel
		{
			if (sx8636_debug_log_enable)
				printk(KERN_INFO "raw_data[%d] = %d\n", i, raw[i]);

			// low pass filter the raw data to create the useful data
			use_data[i] = (use_data[i] - (use_data[i] >> use_filter_alpha[i])) +
							(raw[i] >> use_filter_alpha[i]);

			if (sx8636_debug_log_enable)
				printk(KERN_INFO "use_data[%d] = %d\n", i, use_data[i]);

			// Calculate ave_data if not in touch status
			if ((touch_status & proc_mask) != proc_mask)
			{
				// low pass filter the useful data to create the average data
				ave_data[i] = (ave_data[i] - (ave_data[i] >> ave_filter_alpha[i])) +
						(use_data[i] >> ave_filter_alpha[i]);
			}

		#if 1	/* Workaround: In some cases, the average data is a large negative value. */
				/* It'll cause the diff value become large even we don't touch the sensor pads. */
				/* So we compensate channels to avoid this. */
			if (ave_data[i] < AVE_NEGATIVE_COMP_VALUE)
			{
				dif_data[i] = 0;
				use_data[i] = 0;
				ave_data[i] = 0;
				// calibrate all channels
				if (sx8636_debug_log_enable)
					printk(KERN_INFO "%s: Negative compensation", __func__);

				ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, 0x04);
				if (ret < 0) return ret;
			}
		#endif
			
			if (sx8636_debug_log_enable)	
				printk(KERN_INFO "ave_data[%d] = %d\n", i, ave_data[i]);

			// create diff data as useful - average + offset (offset is 0 when no prox and is peak diff when in prox)
			dif_data[i] = use_data[i] - ave_data[i] + prox_offset[i];

			if (sx8636_debug_log_enable) {
				printk(KERN_INFO "prox_offset[%d] = %d\n", i, prox_offset[i]);
				printk(KERN_INFO "dif_data[%d] = %d\n", i, dif_data[i]);
			}

			if ((touch_status & proc_mask) == 0)		// currently touch not detected for this channel
			{
				if (dif_data[i] >= touch_detect_thresh[i])
				{
					SX8636_LOG("touch detect status\n");
					touch_status |= proc_mask;
					prox_status &= ~proc_mask;
					prox_offset[i] = 0;
				}
			}
			else		// currently touch is detected for this channel
			{
				if (dif_data[i] < touch_release_thresh[i])
				{
					SX8636_LOG("touch release status\n");
					touch_status &= ~proc_mask;

					dif_data[i] = 0;
					use_data[i] = 0;
					ave_data[i] = 0;
					// calibrate all channels
					if (sx8636_debug_log_enable)
						printk(KERN_INFO "%s: Touch release compensation", __func__);

					ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, 0x04);
					if (ret < 0) return ret;
				}
			}

			if(((prox_status & proc_mask) == 0) && ((touch_status & proc_mask) == 0)) // currently prox not detected for this channel
			{
				// if dif_data is above thresh + hyst then increment pos debounce count and zero neg debounce count
				if(dif_data[i] >= (prox_detect_thresh[i] + (signed short)prox_detect_hyst[i]))
				{
					debounce_pos_count[i]++;
					debounce_neg_count[i] = 0;
				}

				// if pos debounce count >= prox_detect_debounce setting then declare prox detect for this channel and 
				// set prox_offset to current diff and set ave_data to current diff so that averaging durring prox
				// does not suffer a lag
				if(debounce_pos_count[i] >= prox_detect_debounce)
				{
					prox_status |= proc_mask;
					prox_offset[i] = dif_data[i];
					ave_data[i] = dif_data[i];
				}
			}
			else	 if ((touch_status & proc_mask) == 0)		// currently prox is detected for this channel
			{
				#if 1	/* Workaround: The prox status is stuck on if use_data & ave_data are small but prox_offset is large */
				if (NEAR_ZERO(use_data[i]) && NEAR_ZERO(ave_data[i]) && prox_offset[i] > PROX_OFFSET_HIGH_CHECK)
				{
					printk(KERN_INFO "The prox_offset[%d] is set too high. Value = %d\n", i, prox_offset[i]);
					prox_offset[i] = 0;
				}
				#endif
				// if dif_data is below thresh - hyst then increment neg debounce count and zero pos debounce count
				if(dif_data[i] <= (prox_detect_thresh[i] - (signed short)prox_detect_hyst[i]))
				{
					debounce_pos_count[i] = 0;
					debounce_neg_count[i]++;
				}
				// if neg debounce count >= prox_release_debounce setting then declare prox release for this channel and 
				// set prox_offset to zero and set ave_data to current diff so that averaging
				// does not suffer a lag
				if(debounce_neg_count[i] >= prox_release_debounce)
				{
					prox_status &= ~proc_mask;
					prox_offset[i] = 0;
					if ((touch_status & proc_mask) != proc_mask)
						ave_data[i] = dif_data[i];
				}
				else if(dif_data[i] > prox_offset[i]) // have to think about this... intent is to have prox_offset represent true peak of original use - ave
				{
					// while in prox keep accumulating peak value of diff as prox offset to hold best value before loss
					// need to verify that this method does not get into fault condition where prox is stuck true
					// prox_offset[i] = dif_data[i];
				}
			}
		}
		proc_mask = proc_mask << 1;
	}

	// Save the data into files if logging is enabled
	if (sx8636_data_logging_enable)
	{
		//memset(temp_buf, 0, sizeof(temp_buf));
		//fp_raw = filp_open("/data/sx8636_raw.txt", O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		//sprintf(temp_buf, "[%d],%d,%d,%d,%d,%d,%d,%d,%d\n", logIndex, raw[0],
		//	raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7]);
		//fp_raw->f_op->write(fp_raw, temp_buf, strlen(temp_buf), &fp_raw->f_pos);
		//filp_close(fp_raw, NULL);

		//memset(temp_buf, 0, sizeof(temp_buf));
		//fp_use = filp_open("/data/sx8636_use.txt", O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		//sprintf(temp_buf, "[%d],%d,%d,%d,%d,%d,%d,%d,%d\n", logIndex, use_data[0],
		//	use_data[1], use_data[2], use_data[3], use_data[4], use_data[5], use_data[6], use_data[7]);
		//fp_use->f_op->write(fp_use, temp_buf, strlen(temp_buf), &fp_use->f_pos);
		//filp_close(fp_use, NULL);

		//memset(temp_buf, 0, sizeof(temp_buf));
		//fp_ave = filp_open("/data/sx8636_ave.txt", O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		//sprintf(temp_buf, "[%d],%d,%d,%d,%d,%d,%d,%d,%d\n", logIndex, ave_data[0],
		//	ave_data[1], ave_data[2], ave_data[3], ave_data[4], ave_data[5], ave_data[6], ave_data[7]);
		//fp_raw->f_op->write(fp_ave, temp_buf, strlen(temp_buf), &fp_ave->f_pos);
		//filp_close(fp_ave, NULL);
		
		memset(temp_buf, 0, sizeof(temp_buf));
		fp_dif = filp_open("/data/sx8636_dif.txt", O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		if (IS_ERR_OR_NULL(fp_dif)) {
			printk(KERN_INFO "Failed to create /data/sx8636_dif.txt\n");
			return -1;
		} else {
			sprintf(temp_buf, "[%ld],%d,%d,%d,%d,%d,%d,%d,%d\n", logIndex, dif_data[0], dif_data[1], dif_data[2], dif_data[3], dif_data[4], dif_data[5], dif_data[6], dif_data[7]);
			bytes_write = fp_dif->f_op->write(fp_dif, temp_buf, strlen(temp_buf), &fp_dif->f_pos);
			if (!bytes_write) {
				printk(KERN_INFO "Data write fail\n");
				return -1;
			}
			filp_close(fp_dif, NULL);
		}

		logIndex++;
	}

	// Update proximity status
	proximity_status = sx863x_update_proximity_status(sx863x);
#if 1
	if (proximity_status != pre_proximity_status) {
		sensor_status_changed();	// Notify to power reduction controller driver
		pre_proximity_status = proximity_status;
	}
#endif
	SX8636_LOG("proximity status = %d\n", proximity_status);

	SX8636_LOG("===============\n");
	SX8636_LOG("prox_status = 0x%02X, touch_status = 0x%02X\n", prox_status, touch_status);
	SX8636_LOG("===============\n");

	return 0;
}

/*
 * Called when interrupt has been triggered.
 * Calls another function to do the actual processing so that we can
 * reset INTB if an interrupt occured before this was actived.
 * (We only look at falling edge and not LOW level)
 *
 */
static irqreturn_t sx863x_interrupt_thread(int irq, void *data)
{
	struct sx863x_chip *sx863x = data;
	unsigned char reg_data;
	int ret = 0;
	int bytes_write = 0;
	mm_segment_t old_fs;

	SX8636_LOG("%s called\n", __func__);

	if (sx8636_debug_log_enable)
		printk(KERN_INFO "%s called. P_SensorDriverMode = %d\n", __func__, P_SensorDriverMode);

	mutex_lock(&sx863x->mutex);

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		/* Configure monitor scan period in ISR */
		if (monitor_rate_changed == 1)
		{
			printk(KERN_INFO "Monitor scan period is changed to %d\n", (monitor_scan_period * 15));
			ret = sx863x_update_spm_single(sx863x, 0xF9, &monitor_scan_period);
			if (ret < 0) goto function_end;
			monitor_rate_changed = 0;
		}

		/* Read the raw data from SPM address */
		// Only read raw data of Cap0 & Cap4
		memset(CapAdcValueArray_raw, 0, sizeof(CapAdcValueArray_raw));
		ret = sx863x_read_cap_adc_value(sx863x, RAW_CAP_VALUE_SPM_BASE, &CapAdcValueArray_raw[0]);
		if (ret < 0) goto function_end;
		ret = sx863x_read_cap_adc_value(sx863x, RAW_CAP_VALUE_SPM_BASE + 8, &CapAdcValueArray_raw[4]);
		if (ret < 0) goto function_end;
		
		if ((CapAdcValueArray_raw[0] < -5000) || (CapAdcValueArray_raw[4] < -5000))
		{
			printk(KERN_INFO "Too large negative raw data, so we discard it. raw_data[0] = %d, raw_data[4] = %d\n", CapAdcValueArray_raw[0], CapAdcValueArray_raw[4]);
		}
		else {
			ret = process_raw_data_norm(sx863x, CapAdcValueArray_raw);
			if (ret < 0) {
				printk(KERN_INFO "Failed to process raw data\n");
				goto function_end;
			}
		}
	}
	else if (P_SensorDriverMode == NORMAL_MODE)
	{
		if (read_dif_in_normal == 1)
		{
			struct file *fp_dif;
			unsigned char temp_buf[100];

			/* Configure monitor scan period in ISR */
			if (monitor_rate_changed == 1)
			{
				printk(KERN_INFO "Monitor scan period is changed to %d\n", (monitor_scan_period * 15));
				ret = sx863x_update_spm_single(sx863x, 0xF9, &monitor_scan_period);
				if (ret < 0) goto function_end;
				monitor_rate_changed = 0;
			}

			/* read the dif values from chip */
			memset(dif_data, 0, sizeof(dif_data));
			ret = sx863x_read_cap_adc_value(sx863x, DIF_CAP_VALUE_SPM_BASE, &dif_data[0]);
			if (unlikely(ret < 0)) goto function_end;
			ret = sx863x_read_cap_adc_value(sx863x, DIF_CAP_VALUE_SPM_BASE + 8, &dif_data[4]);
			if (unlikely(ret < 0)) goto function_end;

			memset(raw_data, 0, sizeof(raw_data));
			ret = sx863x_read_cap_adc_value(sx863x, RAW_CAP_VALUE_SPM_BASE, &raw_data[0]);
			if (unlikely(ret < 0)) goto function_end;
			ret = sx863x_read_cap_adc_value(sx863x, RAW_CAP_VALUE_SPM_BASE + 8, &raw_data[4]);
			if (unlikely(ret < 0)) goto function_end;

			memset(ave_data, 0, sizeof(ave_data));
			ret = sx863x_read_cap_adc_value(sx863x, AVE_CAP_VALUE_SPM_BASE, &ave_data[0]);
			if (unlikely(ret < 0)) goto function_end;
			ret = sx863x_read_cap_adc_value(sx863x, AVE_CAP_VALUE_SPM_BASE + 8, &ave_data[4]);
			if (unlikely(ret < 0)) goto function_end;

			if (sx8636_debug_log_enable)
			{
				printk(KERN_INFO "raw_data[0] = %d, raw_data[4] = %d\n", raw_data[0], raw_data[4]);
				printk(KERN_INFO "ave_data[0] = %d, ave_data[4] = %d\n", ave_data[0], ave_data[4]);
				printk(KERN_INFO "dif_data[0] = %d, dif_data[4] = %d\n", dif_data[0], dif_data[4]);
			}

			if (sx8636_data_logging_enable)
			{
				memset(temp_buf, 0, sizeof(temp_buf));
				
				old_fs = get_fs();
				set_fs(KERNEL_DS);
				fp_dif = filp_open("/data/sx8636_dif.txt", O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
				if (IS_ERR_OR_NULL(fp_dif)) {
					printk(KERN_INFO "Failed to create /data/sx8636_dif.txt\n");
				} else {
					sprintf(temp_buf, "[%ld],%d,%d,%d,%d,%d,%d,%d,%d\n", logIndex, dif_data[0], dif_data[1], dif_data[2], dif_data[3], dif_data[4], dif_data[5], dif_data[6], dif_data[7]);
					bytes_write = fp_dif->f_op->write(fp_dif, temp_buf, strlen(temp_buf), &fp_dif->f_pos);
					if (!bytes_write) {
						printk(KERN_INFO "Failed to write data into file\n");
					}
					filp_close(fp_dif, NULL);
					logIndex++;
				}
				set_fs(old_fs);
			}
		}
	}

function_end:
	if ((P_SensorDriverMode == MONITOR_MODE) || 
		((P_SensorDriverMode == NORMAL_MODE) && (read_dif_in_normal == 1)))
	{
		/* enable monitor mode for next interrupt */
		reg_data = 0x04;	
		ret = sx863x->write(sx863x->dev, SX863X_SPMCFG_REG, reg_data);
		if (ret < 0)
			printk(KERN_INFO "Failed to write register 0x%02x\n", SX863X_SPMCFG_REG);
	}
	ret = sx863x_process_interrupt(sx863x);
	if (ret < 0)
		printk(KERN_INFO "Failed to execute sx863x_process_interrupt: ret = %d\n", ret);

	mutex_unlock(&sx863x->mutex);

	return IRQ_HANDLED;
}

#if FOXCONN_SX8636_SYSFS_ENABLED
static ssize_t attr_get_proximity_status(struct device *dev, struct device_attribute *attr,
                                        char *buf)
{
	return snprintf(buf, 8, "%d\n", proximity_status);
}

static ssize_t attr_get_cap0_proximity_status(struct device *dev, struct device_attribute *attr,
                                        char *buf)
{
	return snprintf(buf, 8, "%d\n", cap0_proximity_status);
}

static ssize_t attr_get_cap4_proximity_status(struct device *dev, struct device_attribute *attr,
                                        char *buf)
{
	return snprintf(buf, 8, "%d\n", cap4_proximity_status);
}

static ssize_t attr_get_all_proximity_status(struct device *dev, struct device_attribute *attr,
                                        char *buf)
{
	return snprintf(buf, 8, "%d\n%d\n", cap0_proximity_status, cap4_proximity_status);
}

static ssize_t attr_set_sx863x_chip_reset(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval != 1)
		return -EINVAL;

	data = (unsigned char)(lval);
	
	if (data == 1)	/* Enable chip reset */
	{	
		printk(KERN_INFO "%s: Reset sx8636 chip...\n", __func__);
		ret = chip->write(chip->dev, SX863X_SOFTRESET_REG, 0xDE);
		if (unlikely(ret < 0)) return ret;
		ret = chip->write(chip->dev, SX863X_SOFTRESET_REG, 0x00);
		if (unlikely(ret < 0)) return ret;

		/* Initialize the proximity_status after chip reset */
		cap0_proximity_status = CAP_SENSE_NO_PROX;
		cap4_proximity_status = CAP_SENSE_NO_PROX;
		proximity_status = CAP_SENSE_NO_PROX;
	}

	/* Wait 150ms for sx8636 ready */
	msleep(1500);
#if 1
	/* Initialize the register setting of sx8636 */
	ret = sx863x_hw_init(chip);
	if (unlikely(ret < 0)) return ret;

	if (sx863x_check_eMMC()) {	// Update the SPM register from p_sensor.bin
		ret = sx863x_update_spm_from_binary(chip);
		if (unlikely(ret < 0)) return ret;
	}
	ret = sx863x_initialize_prox_variables(chip);
	if (unlikely(ret < 0)) return ret;
#endif

	return size;
}

static ssize_t attr_set_sx863x_chip_comp(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	int i;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval != 1)
		return -EINVAL;

	data = (unsigned char)(lval);

	if (data == 1)	/* Enable chip compensation */
	{
		printk(KERN_INFO "%s: Compensate sx8636 chip...\n", __func__);

		// calibrate all channels
		if (sx8636_debug_log_enable)
			printk(KERN_INFO "%s: Compensation", __func__);

		ret = chip->write(chip->dev, SX863X_COMPOPMODE_REG, 0x04);
		if (unlikely(ret < 0)) return ret;

		for (i = 0; i  < CHANNEL_NUM; i++)
		{
			dif_data[i] = 0;
			use_data[i] = 0;
			ave_data[i] = 0;
		}

		prox_status = 0;
		touch_status = 0;

		/* Initialize the proximity_status after chip reset */
		cap0_proximity_status = CAP_SENSE_NO_PROX;
		cap4_proximity_status = CAP_SENSE_NO_PROX;
		proximity_status = CAP_SENSE_NO_PROX;
	}

	return size;
}

// spm address 0x05
static ssize_t attr_get_ActiveScanPeriod(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_ACTIVESCANPERIOD_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x05
static ssize_t attr_set_ActiveScanPeriod(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);
	
	ret = sx863x_update_spm_single(chip, SX863X_ACTIVESCANPERIOD_SPM, &data);
	if (ret < 0)
		return ret;

	scan_period = data;

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		monitor_rate_changed = 1;
		monitor_scan_period = scan_period;
	}
	
	return size;
}

// spm address 0x06
static ssize_t attr_get_DozeScanPeriod(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_DOZESCANPERIOD_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x06
static ssize_t attr_set_DozeScanPeriod(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_DOZESCANPERIOD_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x07
static ssize_t attr_get_PassiveTimer(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_PASSIVETIMER_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x07
static ssize_t attr_set_PassiveTimer(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_PASSIVETIMER_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x09
static ssize_t attr_get_CapModeMisc(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPEMODEMISC_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x09
static ssize_t attr_set_CapModeMisc(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPEMODEMISC_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x0B
static ssize_t attr_get_CapMode7_4(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPMODE7_4_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x0B
static ssize_t attr_set_CapMode7_4(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPMODE7_4_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x0C
static ssize_t attr_get_CapMode3_0(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPMODE3_0_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x0C
static ssize_t attr_set_CapMode3_0(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPMODE3_0_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x0D
static ssize_t attr_get_CapSensitivity0_1(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPSENSITIVITY0_1_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x0D
static ssize_t attr_set_CapSensitivity0_1(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPSENSITIVITY0_1_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x0E
static ssize_t attr_get_CapSensitivity2_3(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPSENSITIVITY2_3_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x0E
static ssize_t attr_set_CapSensitivity2_3(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPSENSITIVITY2_3_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x0F
static ssize_t attr_get_CapSensitivity4_5(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPSENSITIVITY4_5_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x0F
static ssize_t attr_set_CapSensitivity4_5(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPSENSITIVITY4_5_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x10
static ssize_t attr_get_CapSensitivity6_7(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPSENSITIVITY6_7_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x10
static ssize_t attr_set_CapSensitivity6_7(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPSENSITIVITY6_7_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x13
static ssize_t attr_get_Cap0_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH0_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x13
static ssize_t attr_set_Cap0_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	mutex_lock(&chip->fs_mutex);

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH0_SPM, &data);
 	if (ret < 0) goto function_end;

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		prox_detect_thresh[0] = (signed short)(data * 4);

		// re-calculate hysteresis
		ret = sx863x_update_spm_single(chip, SX863X_BTNHYSTERESIS_SPM, &data);
		if (ret < 0) goto function_end;

		if (data > 0)
			prox_detect_hyst[0] = (prox_detect_thresh[0] / data);	// 10%
		else
			prox_detect_hyst[0] = 0;
	}

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return size;
}

// spm address 0x14
static ssize_t attr_get_Cap1_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH1_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x14
static ssize_t attr_set_Cap1_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH1_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x15
static ssize_t attr_get_Cap2_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH2_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x15
static ssize_t attr_set_Cap2_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);
	SX8636_LOG("%s: data = 0x%02X\n", __func__, data);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH2_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x16
static ssize_t attr_get_Cap3_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH3_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x16
static ssize_t attr_set_Cap3_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);
	SX8636_LOG("%s: data = 0x%02X\n", __func__, data);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH3_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x17
static ssize_t attr_get_Cap4_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH4_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x17
static ssize_t attr_set_Cap4_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	mutex_lock(&chip->fs_mutex);

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH4_SPM, &data);
	if (ret < 0) goto function_end;

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		prox_detect_thresh[4] = (signed short)(data * 4);

		// re-calculate hysteresis
		ret = sx863x_update_spm_single(chip, SX863X_BTNHYSTERESIS_SPM, &data);
		if (ret < 0) goto function_end;

		if (data > 0)
			prox_detect_hyst[4] = (prox_detect_thresh[4] / data);	// 10%
		else
			prox_detect_hyst[4] = 0;
	}

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return size;
}

// spm address 0x18
static ssize_t attr_get_Cap5_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH5_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x18
static ssize_t attr_set_Cap5_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);
	SX8636_LOG("%s: data = 0x%02X\n", __func__, data);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH5_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x19
static ssize_t attr_get_Cap6_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH6_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x19
static ssize_t attr_set_Cap6_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);
	SX8636_LOG("%s: data = 0x%02X\n", __func__, data);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH6_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x1A
static ssize_t attr_get_Cap7_Thresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPTHRESH7_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x1A
static ssize_t attr_set_Cap7_Thresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	mutex_lock(&chip->fs_mutex);
	
	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPTHRESH7_SPM, &data);
	if (ret < 0) goto function_end;

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		prox_detect_thresh[7] = (signed short)(data * 4);

		// re-calculate hysteresis
		ret = sx863x_update_spm_single(chip, SX863X_BTNHYSTERESIS_SPM, &data);
 		if (ret < 0) goto function_end;

		if (data > 0)
			prox_detect_hyst[7] = (prox_detect_thresh[7] / 10);	// 10%
		else
			prox_detect_hyst[7] = 0;
	}

function_end:
	mutex_unlock(&chip->fs_mutex);
	
	if (ret < 0)
		return ret;
	else
		return size;
}

// spm address 0x1F
static ssize_t attr_get_CapPerComp(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPPERCOMP_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x1F
static ssize_t attr_set_CapPerComp(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPPERCOMP_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x21
static ssize_t attr_get_BtnCfg(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_BTNCFG_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x21
static ssize_t attr_set_BtnCfg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	mutex_lock(&chip->fs_mutex);

	ret = sx863x_update_spm_single(chip, SX863X_BTNCFG_SPM, &data);
	if (ret < 0) goto function_end;

	prox_detect_debounce = ((data & 0x0C) >> 2) + 1;
	prox_release_debounce = (data & 0x03) + 1;

function_end:
	mutex_unlock(&chip->fs_mutex);
	
	if (ret < 0)
		return ret;
	else
		return size;
}

// spm address 0x22
static ssize_t attr_get_BtnAvgThresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_BTNAVGTHRESH_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x22
static ssize_t attr_set_BtnAvgThresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_BTNAVGTHRESH_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x23
static ssize_t attr_get_BtnCompNegThresh(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_BTNCOMPNEGTHRESH_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x23
static ssize_t attr_set_BtnCompNegThresh(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_BTNCOMPNEGTHRESH_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x24
static ssize_t attr_get_BtnCompNegCntMax(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_BTNCOMPNEGCNTMAX_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x24
static ssize_t attr_set_BtnCompNegCntMax(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_BTNCOMPNEGCNTMAX_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x25
static ssize_t attr_get_BtnHysteresis(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_BTNHYSTERESIS_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x25
static ssize_t attr_set_BtnHysteresis(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	mutex_lock(&chip->fs_mutex);
	
	ret = sx863x_update_spm_single(chip, SX863X_BTNHYSTERESIS_SPM, &data);
	if (ret < 0) goto function_end;

	if (data > 0)
	{
		prox_detect_hyst[0] = (prox_detect_thresh[0] / data);
		prox_detect_hyst[4] = (prox_detect_thresh[4] / data);
	}
	else
	{
		prox_detect_hyst[0] = 0;
		prox_detect_hyst[4] = 0;
	}

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return size;
}

// spm address 0x26
static ssize_t attr_get_BtnStuckAtTimeout(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_BTNSTUCKATTIMEOUT_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x26
static ssize_t attr_set_BtnStuckAtTimeout(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_BTNSTUCKATTIMEOUT_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

// spm address 0x70
static ssize_t attr_get_CapProxEnable(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	ret = sx863x_retrieve_spm_single(chip, SX863X_CAPPROXENABLE_SPM, &data);
	if (ret < 0)
		return ret;

	return snprintf(buf, 8, "%02X\n", data);
}

// spm address 0x70
static ssize_t attr_set_CapProxEnable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval > 0xff)
		return -EINVAL;

	data = (unsigned char)(lval);

	ret = sx863x_update_spm_single(chip, SX863X_CAPPROXENABLE_SPM, &data);
	if (ret < 0)
		return ret;

	return size;
}

static ssize_t attr_get_sx8636_monitor_mode(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 8, "%d\n", P_SensorDriverMode);
}

static ssize_t attr_set_sx8636_monitor_mode(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval != 0 && lval != 1)
		return -EINVAL;

	data = (unsigned char)(lval);	// 0: disable, 1: enable
	printk(KERN_INFO "%s called\n", __func__);

	mutex_lock(&chip->fs_mutex);
	
	if (P_SensorDriverMode == MONITOR_MODE)
	{
		printk(KERN_INFO "The P-Sensor driver has been configured as monitor mode\n");
	} else if (P_SensorDriverMode == NORMAL_MODE)
	{
		if (data == 1)	/* read diff values in normal mode for P-Sensor tool & factory calibration */
		{
			read_dif_in_normal = 1;

			/* Enable monitor mode */
			ret = chip->read(chip->dev, SX863X_SPMCFG_REG, &data);
			if (unlikely(ret < 0)) goto function_end;
			data |= 0x04;	/* Set bit[2] = 1 */
			ret = chip->write(chip->dev, SX863X_SPMCFG_REG, data);
			if (unlikely(ret < 0)) goto function_end;

			/* Set the monitor rate */
			monitor_scan_period = scan_period;
			monitor_rate_changed = 1;
		} else if (data == 0)
		{
			read_dif_in_normal = 0;

			/* Disable monitor mode */
			ret = chip->read(chip->dev, SX863X_SPMCFG_REG, &data);
			if (unlikely(ret < 0)) goto function_end;
			data &= ~0x04;	/* Set bit[2] = 0 */
			ret = chip->write(chip->dev, SX863X_SPMCFG_REG, data);
			if (unlikely(ret < 0)) goto function_end;
		}
	}

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return size;
}

static ssize_t attr_get_data_logging(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 8, "%d\n", sx8636_data_logging_enable);
}

static ssize_t attr_set_data_logging(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned long lval;
	int i;
	//struct file *fp_raw, *fp_use, *fp_ave;
	struct file *fp_dif;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;
	
	if (lval != 0 && lval != 1)
		return -EINVAL;

	mutex_lock(&chip->fs_mutex);

	logIndex = 0;
	sx8636_data_logging_enable = (unsigned char)(lval);	// 0: disable, 1: enable

	if (sx8636_data_logging_enable == 0)
	{
		// clear the file
	#if 0
		if (P_SensorDriverMode == MONITOR_MODE)
		{
			fp_raw = filp_open("/data/sx8636_raw.txt", O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
			if (IS_ERR_OR_NULL(fp_raw)) {
				printk(KERN_INFO "Failed to open /data/sx8636_raw.txt\n");
			} else {
				filp_close(fp_raw, NULL);
			}

			fp_use = filp_open("/data/sx8636_use.txt", O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
			if (IS_ERR_OR_NULL(fp_use)) {
				printk(KERN_INFO "Failed to open /data/sx8636_use.txt\n");
			} else {
				filp_close(fp_use, NULL);
			}
			
			fp_ave = filp_open("/data/sx8636_ave.txt", O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
			if (IS_ERR_OR_NULL(fp_ave)) {
				printk(KERN_INFO "Failed to open /data/sx8636_ave.txt\n");
			} else {
				filp_close(fp_ave, NULL);
			}
		}
	#endif
		
		fp_dif = filp_open("/data/sx8636_dif.txt", O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		if (IS_ERR_OR_NULL(fp_dif)) {
			printk(KERN_INFO "Failed to open /data/sx8636_dif.txt\n");
		} else {
			filp_close(fp_dif, NULL);
		}
		
		// calibrate all channels
		if (sx8636_debug_log_enable)
			printk(KERN_INFO "%s: Compensation", __func__);

		ret = chip->write(chip->dev, SX863X_COMPOPMODE_REG, 0x04);
		if (unlikely(ret < 0)) goto function_end;

		for (i = 0; i  < CHANNEL_NUM; i++)
		{
			dif_data[i] = 0;
			use_data[i] = 0;
			ave_data[i] = 0;
		}

		prox_status = 0;
		touch_status = 0;

		/* Initialize the proximity_status after chip reset */
		cap0_proximity_status = CAP_SENSE_NO_PROX;
		cap4_proximity_status = CAP_SENSE_NO_PROX;
		proximity_status = CAP_SENSE_NO_PROX;
	}

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return size;
}

static ssize_t attr_set_monitor_rate_60ms(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	unsigned long lval;
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval != 0 && lval != 1)
		return -EINVAL;

	data = (unsigned char)lval;

	printk(KERN_INFO "%s: enable = %d\n", __func__, data);

	mutex_lock(&chip->fs_mutex);

	if (P_SensorDriverMode == MONITOR_MODE)
	{
		monitor_rate_changed = 1;
		if (data == 1) {
			monitor_scan_period = MONITOR_PERIOD_60MS;
		} else if (data == 0) {
			/* Set to original scaning period */
			monitor_scan_period = scan_period;
		}
	}
	else if (P_SensorDriverMode == NORMAL_MODE)
	{
		if (read_dif_in_normal == 1)
		{
			monitor_rate_changed = 1;
			if (data == 1) {
				monitor_scan_period = MONITOR_PERIOD_60MS;
			} else if (data == 0) {
				/* Set to original scaning period */
				monitor_scan_period = scan_period;
			}
		}
	}

	mutex_unlock(&chip->fs_mutex);

	return size;
}

static ssize_t attr_get_sx8636_operation_mode(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	int ret = 0;
	unsigned char data;
	unsigned char curr_mode;
	unsigned char *modeText[3] = {"ACTIVE", "DOZE", "SLEEP"};
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->fs_mutex);
	
	// read the current operation mode
	ret = chip->read(chip->dev, SX863X_COMPOPMODE_REG, &data);
	if (unlikely(ret < 0)) goto function_end;

	curr_mode = data & SX863X_COMPOPMODE_OPERATINGMODE_MSK;
	printk(KERN_INFO "Current operation mode is %s mode\n", modeText[curr_mode]);

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return snprintf(buf, 8, "%02X\n", curr_mode);
}

static ssize_t attr_set_sx8636_operation_mode(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned char curr_mode;
	unsigned char set_mode;
	unsigned long lval;
	unsigned char *modeText[3] = {"ACTIVE", "DOZE", "SLEEP"};
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval != 0 && lval != 1 && lval != 2)
		return -EINVAL;

	mutex_lock(&chip->fs_mutex);

	if (0 == sx863x_check_eMMC()) {
		printk(KERN_INFO "This operation is not allowed because the p_sensor.bin is not exist!\n");
		proximity_status = CAP_SENSE_PROX;
		cap0_proximity_status = CAP_SENSE_PROX;
		cap4_proximity_status = CAP_SENSE_PROX;
		ret = -1;
		goto function_end;
	}

	set_mode = (unsigned char)(lval);

	// read the current operation mode
	ret = chip->read(chip->dev, SX863X_COMPOPMODE_REG, &data);
	if (unlikely(ret < 0)) goto function_end;

	curr_mode = data & SX863X_COMPOPMODE_OPERATINGMODE_MSK;
	printk(KERN_INFO "Current operation mode is %s mode\n", modeText[curr_mode]);
	// 0: ACTIVE
	// 1: DOZE
	// 2: SLEEP

	if (set_mode == curr_mode) {
		printk(KERN_INFO "We already in %s mode\n", modeText[curr_mode]);
		ret = -1;
		goto function_end;
	} else {
		/* Program device to go into sleep mode */
		ret = chip->read(chip->dev, SX863X_COMPOPMODE_REG, &data);
		if (unlikely(ret < 0)) goto function_end;
		data = data & ~SX863X_COMPOPMODE_OPERATINGMODE_MSK;
		data = data | set_mode;
		ret = chip->write(chip->dev, SX863X_COMPOPMODE_REG, data);
		if (unlikely(ret < 0)) goto function_end;

		printk(KERN_INFO "The operation mode is set to %s mode\n", modeText[set_mode]);

		msleep(300); /* Wait for 300ms */
	}

function_end:
	mutex_unlock(&chip->fs_mutex);
	
	if (ret < 0)
		return ret;
	else
		return size;
}

static ssize_t attr_get_p_sensor_bin(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 8, "%d\n", sx863x_check_eMMC());
}

static ssize_t attr_get_debug_log_status(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 8, "%d\n", sx8636_debug_log_enable);
}

static ssize_t attr_set_debug_log_status(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	unsigned long lval;
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval != 0 && lval != 1)
		return -EINVAL;

	sx8636_debug_log_enable = (unsigned char)(lval);	// 0: disable, 1: enable

	printk(KERN_INFO "%s: status = %d\n", __func__, sx8636_debug_log_enable);

	return size;
}

static ssize_t attr_get_driver_mode(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 8, "%d\n", P_SensorDriverMode);
}

static ssize_t attr_set_driver_mode(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	int ret = 0;
	unsigned char data;
	unsigned long lval;
	struct i2c_client *client = to_i2c_client(dev);
	struct sx863x_chip *chip = i2c_get_clientdata(client);
	
	if (strict_strtoul(buf, 16, &lval))
		return -EINVAL;

	if (lval != 0 && lval != 1)
		return -EINVAL;

	data = (unsigned char)(lval);	// 0: normal mode, 1: monitor mode

	mutex_lock(&chip->fs_mutex);

	if (P_SensorDriverMode == NORMAL_MODE)
	{
		if (data == 0)
		{
			printk(KERN_INFO "%s: P-Sensor driver mode is already in NORMAL mode\n", __func__);
			ret = -1;
			goto function_end;
		}
		else if (data == 1)
		{
			printk(KERN_INFO "%s: P-Sensor driver mode is changed to MONITOR mode\n", __func__);

			ret = chip->write(chip->dev, SX863X_SOFTRESET_REG, 0xDE);
			if (ret < 0) goto function_end;
			ret = chip->write(chip->dev, SX863X_SOFTRESET_REG, 0x00);
			if (ret < 0) goto function_end;

			/* Initialize the proximity_status after chip reset */
			cap0_proximity_status = CAP_SENSE_NO_PROX;
			cap4_proximity_status = CAP_SENSE_NO_PROX;
			proximity_status = CAP_SENSE_NO_PROX;

			/* Wait 150ms for sx8636 ready */
			msleep(1500);

			/* Initialize the register setting of sx8636 */
			ret = sx863x_hw_init(chip);
			if (ret < 0) goto function_end;

			if (sx863x_check_eMMC()) {
				ret = sx863x_update_spm_from_binary(chip);
				if (ret < 0) goto function_end;
			}
			
			P_SensorDriverMode = MONITOR_MODE;
			ret = sx863x_initialize_prox_variables(chip);
			if (ret < 0) goto function_end;
			ret = sx863x_monitor_mode_enable(chip, scan_period);
			if (ret < 0) goto function_end;
		}
	}
	else if (P_SensorDriverMode == MONITOR_MODE)
	{
		if (data == 0)
		{
			printk(KERN_INFO "%s: P-Sensor driver mode is changed to NORMAL mode\n", __func__);
			P_SensorDriverMode = NORMAL_MODE;
			
			/* Disable monitor mode */
			ret = chip->read(chip->dev, SX863X_SPMCFG_REG, &data);
			if (ret < 0) goto function_end;
			data &= ~0x04;	/* Set bit[2] = 0 */
			ret = chip->write(chip->dev, SX863X_SPMCFG_REG, data);
			if (ret < 0) goto function_end;
		}
		else if (data == 1)
		{
			printk(KERN_INFO "%s: P-Sensor driver mode is already in MONITOR mode\n", __func__);
			ret = -1;
			goto function_end;
		}
	}

function_end:
	mutex_unlock(&chip->fs_mutex);

	if (ret < 0)
		return ret;
	else
		return size;
}

static struct device_attribute attributes[] = {
	__ATTR(proximity_status, 0444, attr_get_proximity_status, NULL),
	__ATTR(cap0_proximity_status, 0444, attr_get_cap0_proximity_status, NULL),
	__ATTR(cap4_proximity_status, 0444, attr_get_cap4_proximity_status, NULL),
	__ATTR(all_proximity_status, 0444, attr_get_all_proximity_status, NULL),
	__ATTR(sx863x_chip_reset, 0400, NULL, attr_set_sx863x_chip_reset),
	__ATTR(sx863x_chip_comp, 0400, NULL, attr_set_sx863x_chip_comp),
	__ATTR(ActiveScanPeriod, 0400, attr_get_ActiveScanPeriod, attr_set_ActiveScanPeriod),	// spm address 0x05
	__ATTR(DozeScanPeriod, 0400, attr_get_DozeScanPeriod, attr_set_DozeScanPeriod),	// spm address 0x06
	__ATTR(PassiveTimer, 0400, attr_get_PassiveTimer, attr_set_PassiveTimer),	// spm address 0x07
	__ATTR(CapModeMisc, 0400, attr_get_CapModeMisc, attr_set_CapModeMisc),	// spm address 0x09
	__ATTR(CapMode7_4, 0400, attr_get_CapMode7_4, attr_set_CapMode7_4),	// spm address 0x0B
	__ATTR(CapMode3_0, 0400, attr_get_CapMode3_0, attr_set_CapMode3_0),	// spm address 0x0C
	__ATTR(CapSensitivity0_1, 0400, attr_get_CapSensitivity0_1, attr_set_CapSensitivity0_1),	// spm address 0x0D
	__ATTR(CapSensitivity2_3, 0400, attr_get_CapSensitivity2_3, attr_set_CapSensitivity2_3),	// spm address 0x0E
	__ATTR(CapSensitivity4_5, 0400, attr_get_CapSensitivity4_5, attr_set_CapSensitivity4_5),	// spm address 0x0F
	__ATTR(CapSensitivity6_7, 0400, attr_get_CapSensitivity6_7, attr_set_CapSensitivity6_7),	// spm address 0x10
	__ATTR(Cap0_Thresh, 0400, attr_get_Cap0_Thresh, attr_set_Cap0_Thresh),	// spm address 0x13
	__ATTR(Cap1_Thresh, 0400, attr_get_Cap1_Thresh, attr_set_Cap1_Thresh),	// spm address 0x14
	__ATTR(Cap2_Thresh, 0400, attr_get_Cap2_Thresh, attr_set_Cap2_Thresh),	// spm address 0x15
	__ATTR(Cap3_Thresh, 0400, attr_get_Cap3_Thresh, attr_set_Cap3_Thresh),	// spm address 0x16
	__ATTR(Cap4_Thresh, 0400, attr_get_Cap4_Thresh, attr_set_Cap4_Thresh),	// spm address 0x17
	__ATTR(Cap5_Thresh, 0400, attr_get_Cap5_Thresh, attr_set_Cap5_Thresh),	// spm address 0x18
	__ATTR(Cap6_Thresh, 0400, attr_get_Cap6_Thresh, attr_set_Cap6_Thresh),	// spm address 0x19
	__ATTR(Cap7_Thresh, 0400, attr_get_Cap7_Thresh, attr_set_Cap7_Thresh),	// spm address 0x1A
	__ATTR(CapPerComp, 0400, attr_get_CapPerComp, attr_set_CapPerComp),	// spm address 0x1F
	__ATTR(BtnCfg, 0400, attr_get_BtnCfg, attr_set_BtnCfg),	// spm address 0x21
	__ATTR(BtnAvgThresh, 0400, attr_get_BtnAvgThresh, attr_set_BtnAvgThresh),	// spm address 0x22
	__ATTR(BtnCompNegThresh, 0400, attr_get_BtnCompNegThresh, attr_set_BtnCompNegThresh),	// spm address 0x23
	__ATTR(BtnCompNegCntMax, 0400, attr_get_BtnCompNegCntMax, attr_set_BtnCompNegCntMax),	// spm address 0x24
	__ATTR(BtnHysteresis, 0400, attr_get_BtnHysteresis, attr_set_BtnHysteresis),	// spm address 0x25
	__ATTR(BtnStuckAtTimeout, 0400, attr_get_BtnStuckAtTimeout, attr_set_BtnStuckAtTimeout),	// spm address 0x26
	__ATTR(CapProxEnable, 0400, attr_get_CapProxEnable, attr_set_CapProxEnable),	// spm address 0x70
	__ATTR(sx8636_monitor_mode, 0400, attr_get_sx8636_monitor_mode, attr_set_sx8636_monitor_mode),
	__ATTR(data_logging, 0400, attr_get_data_logging, attr_set_data_logging),
	__ATTR(monitor_rate_60ms, 0400, NULL, attr_set_monitor_rate_60ms),
	__ATTR(operation_mode, 0400, attr_get_sx8636_operation_mode, attr_set_sx8636_operation_mode),
	__ATTR(check_p_sensor_bin, 0400, attr_get_p_sensor_bin, NULL),
	__ATTR(debug_log_status, 0400, attr_get_debug_log_status, attr_set_debug_log_status),
	__ATTR(driver_mode, 0400, attr_get_driver_mode, attr_set_driver_mode),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
	{
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}

	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);

	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}
#endif	// FOXCONN_SX8636_SYSFS_ENABLED

static int __devinit sx863x_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int i;
	int error = 0;
	struct input_dev *input = 0;

	struct sx863x_platform_data *plat_data = client->dev.platform_data;
	struct sx863x_chip *sx863x = 0;
	void *drv_mem = 0;

	struct sx863x_button_drv *bt_drv;

	printk(KERN_INFO "%s called\n", __func__);

	if (client->irq <= 0) {
		dev_err(&client->dev, "IRQ not configured!\n");
		error = -EINVAL;
		goto err_out;
	}

	if (plat_data == NULL) {
		dev_err(&client->dev, "platform data for sx863x doesn't exist\n");
		error = -EINVAL;
		goto err_out;
	}

	sx863x = kzalloc(sizeof(*sx863x) + sizeof(*sx863x->sw) +
			 sizeof(*bt_drv) * plat_data->button_num, GFP_KERNEL);
	if (!sx863x) {
		error = -ENOMEM;
		goto err_out;
	}

	sx863x->hw = plat_data;

	drv_mem = sx863x + 1;
	sx863x->sw = drv_mem;
	drv_mem += sizeof(*sx863x->sw);
	sx863x->sw->button = bt_drv = drv_mem;
	drv_mem += sizeof(*bt_drv) * sx863x->hw->button_num;

	/* keep these as function pointers */
	sx863x->read = sx863x_i2c_read;
	sx863x->write = sx863x_i2c_write;
	sx863x->writeEx = sx863x_i2c_writeEx;
	sx863x->readEx = sx863x_i2c_readEx;
  
	sx863x->irq = client->irq;
	sx863x->dev =  &client->dev;

	mutex_init(&sx863x->mutex);
	mutex_init(&sx863x->fn_mutex);
	mutex_init(&sx863x->fs_mutex);

	error = sx863x_hw_detect(sx863x);
	if (error)
		goto err_free_mem;

	dev_dbg(sx863x->dev, "Number_Buttons: %d Number_I2CRegs %d\n", sx863x->hw->button_num,sx863x->hw->i2c_reg_num);
	/* initialize buttons */
	if (sx863x->hw->button_num > 0) {
		struct sx863x_button_platform_data *bt_plat = sx863x->hw->button;

		input = input_allocate_device();
		if (!input) {
			error = -ENOMEM;
			goto err_free_dev;
		}

		__set_bit(EV_KEY, input->evbit);
		for (i = 0; i < sx863x->hw->button_num; i++) {
			bt_drv[i].input = input;
			__set_bit(bt_plat[i].keycode, input->keybit);
		}

		input->name = "SX863X Cap Touch";
		input->id.bustype = BUS_I2C;
		input->id.product = sx863x->product;
		input->id.version = sx863x->version;

		error = input_register_device(input);
		if (error)
			goto err_unreg_dev;
	}

#if FOXCONN_SX8636_SYSFS_ENABLED
	error = create_sysfs_interfaces(&client->dev);
	if (error < 0) {
		dev_err(&client->dev, "%s device register failed\n", DEV_NAME);
		goto err_unreg_dev;
	}
#endif // FOXCONN_SX8636_SYSFS_ENABLED

	error = request_threaded_irq(sx863x->irq, NULL, sx863x_interrupt_thread,
			IRQF_TRIGGER_FALLING, "sx863x_captouch", sx863x);
	if (error) {
		dev_err(sx863x->dev, "can't allocate irq %d\n", sx863x->irq);
		goto err_unreg_dev;
	}

	dev_info(sx863x->dev, "registered with irq (%d)\n", sx863x->irq);
  
	my_sx863x_chip = sx863x;

	//error = sx863x_process_interrupt(sx863x); 
	//if (error < 0) {
	//	goto err_unreg_dev;
	//}

	printk(KERN_INFO "[SX8636] before i2c_set_clientdata()\n");
	i2c_set_clientdata(client,sx863x);

	/* Reset chip before normal operation */
	error = sx863x_software_reset(sx863x);
	if (error < 0) {
		goto err_unreg_dev;
	}

	error = sx863x_hw_init(sx863x);
	if (error < 0) {
		goto err_unreg_dev;
	}

	init_waitqueue_head(&kthread_wait);

	kmain_task = kthread_create(sx863x_delay_check, NULL, "sx863x_delay_check");

	wake_up_process(kmain_task);

	return 0;

 err_unreg_dev:
	input_unregister_device(input);
 err_free_dev:
	dev_err(sx863x->dev, "failed to setup SX863x input device\n");
	input_free_device(input);
 err_free_mem:
	kfree(sx863x);
 err_out:
	return PTR_ERR(sx863x);
}

EXPORT_SYMBOL(sx863x_i2c_probe);

static int __devexit sx863x_i2c_remove(struct i2c_client *client)
{
	struct sx863x_chip *sx863x = i2c_get_clientdata(client);
	struct sx863x_platform_data *hw = sx863x->hw;
	struct sx863x_driver_data *sw = sx863x->sw;

	free_irq(sx863x->irq, sx863x);

	/* unregister and free all input devices */

	if (hw->button_num && sw->button)
		input_unregister_device(sw->button[0].input);

	kfree(sx863x);
	return 0;
}

#ifdef CONFIG_PM
static int sx863x_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret = 0;
	unsigned char data;
	struct sx863x_chip *sx863x = i2c_get_clientdata(client);

	dev_dbg(sx863x->dev, "%s enter\n", __func__);
	printk(KERN_INFO "%s enter\n", __func__);

	mutex_lock(&sx863x->mutex);

	/* Program device to go into sleep mode */
	ret = sx863x->read(sx863x->dev, SX863X_COMPOPMODE_REG, &data);
	printk(KERN_INFO "%s: ret = %d\n", __func__, ret);
	if (unlikely(ret < 0)) goto function_end;
	data = data & ~SX863X_COMPOPMODE_OPERATINGMODE_MSK;
	data = data | SX863X_COMPOPMODE_OPERATINGMODE_SLEEP;
	ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, data);
	printk(KERN_INFO "%s: ret = %d\n", __func__, ret);
	if (unlikely(ret < 0)) goto function_end;

	msleep(300); /* Wait for 300ms */

function_end:
	mutex_unlock(&sx863x->mutex);

	if (ret < 0)
		return ret;
	else
		return 0;
}

static int sx863x_i2c_resume(struct i2c_client *client)
{
	int ret = 0;
	unsigned char data;
	struct sx863x_chip *sx863x = i2c_get_clientdata(client);

	dev_dbg(sx863x->dev, "%s enter\n", __func__);
	printk(KERN_INFO "%s enter\n", __func__);

	mutex_lock(&sx863x->mutex);

	/* Program device to go into active mode */
	ret = sx863x->read(sx863x->dev, SX863X_COMPOPMODE_REG, &data);
	if (unlikely(ret < 0)) goto function_end;
	data = data & ~SX863X_COMPOPMODE_OPERATINGMODE_MSK;
	data = data | SX863X_COMPOPMODE_OPERATINGMODE_ACTIVE;
	ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, data);
	if (unlikely(ret < 0)) goto function_end;

	#if 1	/* Customer's requirement: Calibrate all channels in resume state */
	printk(KERN_INFO "%s: Compensate all channels\n", __func__);
	ret = sx863x->write(sx863x->dev, SX863X_COMPOPMODE_REG, 0x04);
	if (unlikely(ret < 0)) goto function_end;

	/* Initialize proximity status */
	cap0_proximity_status = CAP_SENSE_NO_PROX;
	cap4_proximity_status = CAP_SENSE_NO_PROX;
	proximity_status = CAP_SENSE_NO_PROX;
	#endif

	msleep(300); /* Wait for 300ms */

	/* time to take for INTB to go low should be caught in int handler 
	 * However, we may need to have a check in here as a safe guard since
	 * interrupt handler only triggers when nirq is low, and if interrupts
	 * are disabled and they do not become enabled before the end of this
	 * function, there could be a chance..
	 * */
	sx863x_process_interrupt(sx863x);  /* in case interrupt didn't catch it */ 

	/* NOTE: If the device gets reset when in suspend, need to re-initialize hardware */

function_end:
	mutex_unlock(&sx863x->mutex);

	if (ret < 0)
		return ret;
	else
		return 0;
}
#else
# define sx863x_i2c_suspend NULL
# define sx863x_i2c_resume  NULL
#endif

static const struct i2c_device_id sx863x_id[] = {
	{ "sx8636", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx863x_id);

static struct i2c_driver sx863x_i2c_driver = {
	.driver = {
		.name = DEV_NAME,
	},
	.probe    = sx863x_i2c_probe,
	.remove   = __devexit_p(sx863x_i2c_remove),
	.suspend  = sx863x_i2c_suspend,
	.resume	  = sx863x_i2c_resume,
	.id_table = sx863x_id,
};

static __init int sx863x_i2c_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return i2c_add_driver(&sx863x_i2c_driver);
}
module_init(sx863x_i2c_init);

static __exit void sx863x_i2c_exit(void)
{
	i2c_del_driver(&sx863x_i2c_driver);
}
module_exit(sx863x_i2c_exit);

MODULE_DESCRIPTION("Semtech SX863X Capacitive Touch Controller Driver");
MODULE_AUTHOR("Semtech (http://www.semtech.com/)");
MODULE_LICENSE("GPL");
