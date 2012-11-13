/* 2012-07-20: File changed by Sony Corporation */
/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011 Atmel Corporation
 * Copyright (C) 2011 NVIDIA Corporation
 * Copyright (C) 2012 Sony Corporation
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts_txs03.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

/* Family ID */
#define MXT224_ID		0x80
#define MXT1386_ID		0xA0

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x24
#define MXT_BOOT_HIGH		0x25

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC	37
#define MXT_GEN_MESSAGE		5
#define MXT_GEN_COMMAND		6
#define MXT_GEN_POWER		7
#define MXT_GEN_ACQUIRE		8
#define MXT_TOUCH_MULTI		9
#define MXT_TOUCH_KEYARRAY	15
#define MXT_TOUCH_PROXIMITY	23
#define MXT_PROCI_GRIPFACE	20
#define MXT_PROCG_NOISE		22
#define MXT_PROCI_ONETOUCH	24
#define MXT_PROCI_TWOTOUCH	27
#define MXT_PROCI_GRIP		40
#define MXT_PROCI_PALM		41
#define MXT_SPT_COMMSCONFIG	18
#define MXT_SPT_GPIOPWM		19
#define MXT_SPT_SELFTEST	25
#define MXT_SPT_CTECONFIG	28
#define MXT_SPT_USERDATA	38
#define MXT_SPT_DIGITIZER	43
#define MXT_SPT_MESSAGECOUNT	44

/* MXT_GEN_COMMAND field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* MXT_GEN_POWER field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

/* MXT_TOUCH_MULTI field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30

/* MXT_PROCI_GRIPFACE field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_CTECONFIG field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

/* Define for MXT_GEN_COMMAND */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		200	/* msec */
#define MXT224_RESET_TIME       65      /* msec */
#define MXT1386_RESET_TIME      200     /* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_NOCHGREAD     400     /* msec */

#define MXT_RESET_TIME		200	/* msec */
#define MXT_WAKEUP_TIME		25

#define MXT_FWRESET_TIME	175	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

/* Fixed Report ID values */
#define MXT_RPTID_NOMSG		0xFF	/* No messages available to read */

#define MXT_MAX_FINGER		10

#define RESUME_READS		100

#define MXT_MAX_RETRY_NUM   	3

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int pressure;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	u8(*read_chg) (void);
	u16 msg_address;
	u16 last_address;
	u8 actv_cycle_time;
	u8 idle_cycle_time;
	u8 is_stopped;
	struct mutex access_mutex;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es);
static void mxt_early_resume(struct early_suspend *es);
#endif

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_MESSAGE:
	case MXT_GEN_COMMAND:
	case MXT_GEN_POWER:
	case MXT_GEN_ACQUIRE:
	case MXT_TOUCH_MULTI:
	case MXT_TOUCH_KEYARRAY:
	case MXT_TOUCH_PROXIMITY:
	case MXT_PROCI_GRIPFACE:
	case MXT_PROCG_NOISE:
	case MXT_PROCI_ONETOUCH:
	case MXT_PROCI_TWOTOUCH:
	case MXT_PROCI_GRIP:
	case MXT_PROCI_PALM:
	case MXT_SPT_COMMSCONFIG:
	case MXT_SPT_GPIOPWM:
	case MXT_SPT_SELFTEST:
	case MXT_SPT_CTECONFIG:
	case MXT_SPT_USERDATA:
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND:
	case MXT_GEN_POWER:
	case MXT_GEN_ACQUIRE:
	case MXT_TOUCH_MULTI:
	case MXT_TOUCH_KEYARRAY:
	case MXT_TOUCH_PROXIMITY:
	case MXT_PROCI_GRIPFACE:
	case MXT_PROCG_NOISE:
	case MXT_PROCI_ONETOUCH:
	case MXT_PROCI_TWOTOUCH:
	case MXT_PROCI_GRIP:
	case MXT_PROCI_PALM:
	case MXT_SPT_GPIOPWM:
	case MXT_SPT_SELFTEST:
	case MXT_SPT_CTECONFIG:
	case MXT_SPT_COMMSCONFIG:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
}

static int mxt_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int retval = 0;
	struct mxt_data *data = i2c_get_clientdata(client);

	if ((data->last_address == reg) && (reg == data->msg_address)) {
		mutex_lock(&data->access_mutex);
		if (i2c_master_recv(client, val, len) != len) {
			dev_err(&client->dev,
				"%s: Failure reading maxTouch device\n",
				__func__);
			retval = -EIO;
		}
		goto mxt_read_exit;
	}

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	mutex_lock(&data->access_mutex);
	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		retval = -EIO;
	}

	data->last_address = reg;

mxt_read_exit:
	mutex_unlock(&data->access_mutex);
	return retval;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];
	int retval = 0;
	struct mxt_data *data = i2c_get_clientdata(client);

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	mutex_lock(&data->access_mutex);
	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		retval = -EIO;
		goto mxt_write_exit;
	}
	data->last_address = reg + 1;

mxt_write_exit:
	mutex_unlock(&data->access_mutex);
	return retval;
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

                if (finger[id].status == MXT_RELEASE)
                        finger[id].status = 0;
                else {
                        finger_num++;

			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
					finger[id].status != MXT_RELEASE ?
					finger[id].area : 0);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					finger[id].y);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR,
					finger[id].pressure);
			input_mt_sync(input_dev);
		}
	}

	if (finger_num > 0)
		input_report_key(input_dev, BTN_TOUCH, 1);
	else
		input_mt_sync(input_dev);

#if 0
	if (status != MXT_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		input_report_abs(input_dev,
				 ABS_PRESSURE, finger[single_id].pressure);
	}
#endif

	input_sync(input_dev);
}

static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{
	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;
	int pressure;

	if ( MXT_MAX_FINGER <= id ) {
		return;
	}

	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & MXT_SUPPRESS) {
			dev_dbg(dev, "[%d] suppressed\n", id);

			finger[id].status = MXT_RELEASE;
			mxt_input_report(data, id);
		} else if (status & MXT_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT_RELEASE;
			mxt_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->max_x < 1024)
		x = x >> 2;
	if (data->max_y < 1024)
		y = y >> 2;

	area = message->message[4];
	pressure = message->message[5];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT_MOVE ? "moved" : "pressed",
		x, y, area);

	finger[id].status = status & MXT_MOVE ?
				MXT_MOVE : MXT_PRESS;
    if ( x >= data->max_x ) x = data->max_x - 1;
	if (data->pdata->orient & MXT_X_INVERT) {
        finger[id].x = data->max_x - x;
    }
    else {
        finger[id].x = x;
    }
    if ( y >= data->max_y ) y = data->max_y - 1;
	if (data->pdata->orient & MXT_Y_INVERT) {
        finger[id].y = data->max_y - y;
    }
    else {
        finger[id].y = y;
    }
	finger[id].area = area;
	finger[id].pressure = pressure;

	mxt_input_report(data, id);
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

    // XXX: dev_err(dev, "*** INT %d\n", irq);
	do {
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI */
		object = mxt_get_object(data, MXT_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt_input_touchevent(data, &message, id);
		else
			mxt_dump_message(dev, &message);
	} while (reportid != 0xff);

end:
	return IRQ_HANDLED;
}

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
	int count = 30;
	int error;

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != 0xff && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

static int mxt_check_reg_init(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct mxt_message message;
	struct device *dev = &data->client->dev;
	int index = 0;
	int timeout_counter;
	int i, j, config_offset;
	int error;
	unsigned long current_crc;
	u8 command_register;

	if (!pdata->config) {
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	/* Try to read the config checksum of the existing cfg */
	mxt_write_object(data, MXT_GEN_COMMAND,
			 MXT_COMMAND_REPORTALL, 1);
	msleep(30);

	error = mxt_read_message(data, &message);
	if (error)
		return error;

	object = mxt_get_object(data, MXT_GEN_COMMAND);
	if (!object)
		return -EIO;

	/* Check if this message is from command processor (which has
	   only one reporting ID), if so, bytes 1-3 are the checksum. */
	if (message.reportid == object->max_reportid) {
		current_crc = message.message[1] | (message.message[2] << 8) |
			      (message.message[3] << 16);
	} else {
		dev_info(dev, "Couldn't retrieve the current cfg checksum, "
			 "forcing load\n");
		current_crc = 0xFFFFFFFF;
	}
	dev_info(dev,
		 "Config CRC read from the mXT: %X\n",
		 (unsigned int) current_crc);

	if (current_crc == pdata->config_crc) {
		dev_info(dev,
			 "Matching CRC's, skipping CFG load.\n");
		return 0;
	} else {
		dev_info(dev, "Doesn't match platform data config CRC (%X), "
			 "writing config from platform data...\n",
			 (unsigned int) pdata->config_crc);
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;
		dev_info(dev, "Writing object type %d, config offset %d", data->object_table[i].type, index);
		for (j = 0;
		     j < ((object->size + 1) * (object->instances + 1));
		     j++) {
			config_offset = index + j;
			if (config_offset > pdata->config_length) {
				dev_err(dev, "Not enough config data!\n");
				dev_err(dev, "config base is %d, offset is %d\n", index, config_offset);
				return -EINVAL;
			}
			mxt_write_object(data, object->type, j,
					 pdata->config[config_offset]);
		}
		index += (object->size + 1) * (object->instances + 1);
	}
	dev_info(dev, "Config written!");

	error = mxt_make_highchg(data);
	if (error)
		return error;

	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	do {
                error =  mxt_read_object(data, MXT_GEN_COMMAND,
					MXT_COMMAND_BACKUPNV,
					&command_register);
                if (error)
                        return error;
		msleep(2);
        } while ((command_register != 0) && (timeout_counter++ <= 100));
	if (timeout_counter >= 100) {
		dev_err(&client->dev, "No response after backup!\n");
		return -EIO;
	}

	/* Clear the interrupt line */
	error = mxt_make_highchg(data);
	if (error)
		return error;

	/* Soft reset */
	mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_RESET, 1);
	if (data->pdata->read_chg == NULL) {
		msleep(MXT_RESET_NOCHGREAD);
	} else {
		switch (data->info.family_id) {
		case MXT224_ID:
			msleep(MXT224_RESET_TIME);
			break;
		case MXT1386_ID:
			msleep(MXT1386_RESET_TIME);
			break;
		default:
			msleep(MXT_RESET_TIME);
		}
		timeout_counter = 0;
		while ((timeout_counter++ <= 100) && data->pdata->read_chg())
			msleep(2);
		if (timeout_counter >= 100) {
			dev_err(&client->dev, "No response after reset!\n");
			return -EIO;
		}
	}

	return 0;
}


static void mxt_handle_pdata(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;

	if (pdata->read_chg != NULL)
		data->read_chg = pdata->read_chg;

}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}

		/* Store message window address so we don't have to
		   search the object table every time we read message */
		if (object->type == MXT_GEN_MESSAGE)
			data->msg_address = object->start_address;

	}

	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 idle_cycle_time;
	u8 actv_cycle_time;

	static int reported = 0;

	error = mxt_get_info(data);
	if (error)
		return error;

	if (data->object_table) {
		kfree(data->object_table);
	}
	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		return error;

	/* Load initial touch chip configuration */
	error = mxt_check_reg_init(data);
	if (error)
		return error;

	mxt_handle_pdata(data);

	/* Read current cycle times */
	error = mxt_read_object(data, MXT_GEN_POWER, MXT_POWER_IDLEACQINT,
				&idle_cycle_time);
	if (error)
		return error;
	error = mxt_read_object(data, MXT_GEN_POWER, MXT_POWER_ACTVACQINT,
				&actv_cycle_time);
	if (error)
		return error;

	if ( !reported ) {
		reported = 1;
		dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);
		
		dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);
	}

	/* clear finger info */
	memset(data->finger, 0, sizeof(data->finger));

	return 0;
}

static void mxt_calc_resolution(struct mxt_data *data)
{
	unsigned int max_x = data->pdata->x_size - 1;
	unsigned int max_y = data->pdata->y_size - 1;

	if (data->pdata->orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	mxt_write_object(data, MXT_GEN_COMMAND,
			MXT_COMMAND_RESET, MXT_BOOT_VALUE);
	msleep(MXT_RESET_TIME);

	/* Change to slave address of bootloader */
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;

	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);

		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;

	return ret;
}

static void mxt_start(struct mxt_data *data)
{
	int error;
	struct device *dev = &data->client->dev;

	/* clear finger info */
	memset(data->finger, 0, sizeof(data->finger));

	//dev_info(dev, "in MXT_START(), idle time: %d %d", data->idle_cycle_time, data->actv_cycle_time);
	/* Restore the cycle time settings to wake from sleep */
	error = mxt_write_object(data, MXT_GEN_POWER, MXT_POWER_ACTVACQINT,
				 data->actv_cycle_time);
	if (error)
		dev_info(dev, "\n\nResume failed!");
	error = mxt_write_object(data, MXT_GEN_POWER, MXT_POWER_IDLEACQINT,
				 data->idle_cycle_time);
	if (error)
		dev_info(dev, "\n\nResume failed!");

	//dev_info(dev, "Restored ACTV %d, IDLE %d", data->actv_cycle_time,
	//       data->idle_cycle_time);
	data->is_stopped = 0;
}

static void mxt_stop(struct mxt_data *data)
{
	u8 actv_cycle_time;
	u8 idle_cycle_time;
	int error;
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;

	if (data->is_stopped)
		return;


	error = mxt_read_object(data, MXT_GEN_POWER, MXT_POWER_ACTVACQINT,
			&actv_cycle_time);
	if (error)
		goto i2c_error;

	data->actv_cycle_time = actv_cycle_time;

	error = mxt_read_object(data, MXT_GEN_POWER, MXT_POWER_IDLEACQINT,
			&idle_cycle_time);
	if (error)
		goto i2c_error;

	data->idle_cycle_time = idle_cycle_time;

	/* Set to deep sleep mode for maximum power savings */
	error = mxt_write_object(data, MXT_GEN_POWER, MXT_POWER_ACTVACQINT, 0);
	if (error)
		goto i2c_error;

	error = mxt_write_object(data, MXT_GEN_POWER, MXT_POWER_IDLEACQINT, 0);
	if (error)
		goto i2c_error;

	//dev_info(dev, "MXT Suspended, saved ACTV %d and IDLE %d",
	//       actv_cycle_time, idle_cycle_time);
	data->is_stopped = 1;

	/* reset all finger status */
	memset(data->finger, 0, sizeof(data->finger));
	if ( input_dev != NULL ) {
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_sync(input_dev);
	}

	return;

i2c_error:
	dev_info(dev, "MXT Suspend failed!");

}

static int mxt_input_open(struct input_dev *dev)
{
	/*
	struct mxt_data *data = input_get_drvdata(dev);
	*/
	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error;

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "atmel-maxtouch";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	//input_dev->open = mxt_input_open;
	//input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;
	data->irq = client->irq;
	data->is_stopped = 0;

	mxt_calc_resolution(data);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#if 0
	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);
#endif

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	mutex_init(&data->access_mutex);

	error = mxt_initialize(data);
	if (error)
	{   
        if (pdata->destruct)
            pdata->destruct();

        goto err_free_object;
	}
	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			pdata->irqflags, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	error = mxt_make_highchg(data);
	if (error)
		goto err_free_irq;

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	if (pdata->destruct != NULL)
		pdata->destruct();
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
// XXX: temporary implementation of power validation for EVT1 panel
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);

    // XXX: power off
    //	free_irq(client->irq, data);
	disable_irq(client->irq);
	if (data->pdata->power != NULL)
        data->pdata->power(0);
	return 0;
}

// XXX: temporary implementation of power validation for EVT1 panel
static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	int error;
	int max_retry = MXT_MAX_RETRY_NUM;

	do {
		// XXX: power on
		if (data->pdata->power != NULL) {
			data->pdata->power(1);
		}
		error = mxt_initialize(data);
		if (!max_retry--)
			break;
		if (error) {
			// XXX: power off
			if (data->pdata->power != NULL) {
				data->pdata->power(0);
			}
			msleep(10);
		}
	} while (error);

	if (error) {
		dev_err(&client->dev, "Failed to initialize \n");
	}
	enable_irq(client->irq);
#if 0
	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			data->pdata->irqflags, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err;
	}
#endif
	data->is_stopped = 0;

	return 0;
}

#if 0
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	struct mxt_message message;
	struct mxt_object *object;
	int id;
	int max_retry = RESUME_READS;	/* Set max retry default value */
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
	u8 n;

	/* A dummy read to wake up the chip */
	mxt_read_object(data, MXT_GEN_POWER, MXT_POWER_IDLEACQINT, &n);
	msleep(MXT_WAKEUP_TIME);

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data);

	mutex_unlock(&input_dev->mutex);

	do {
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}
		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI */
		object = mxt_get_object(data, MXT_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt_input_touchevent(data, &message, id);
		else
			mxt_dump_message(dev, &message);
	} while (reportid != MXT_RPTID_NOMSG && --max_retry);
	if (!max_retry)
		dev_info(dev, "Read %d messages at resume(), and there's still more!\n", RESUME_READS);

end:
	return 0;
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *mxt;
	struct device *dev;
	mxt = container_of(es, struct mxt_data, early_suspend);
	dev = &mxt->client->dev;
	//dev_info(dev, "MXT Early Suspend entered\n");

	if (mxt_suspend(&mxt->client->dev) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);
	//dev_info(dev, "MXT Early Suspended\n");
}

static void mxt_early_resume(struct early_suspend *es)
{
	struct mxt_data *mxt;
	struct device *dev;
	mxt = container_of(es, struct mxt_data, early_suspend);
	dev = &mxt->client->dev;
	//dev_info(dev, "MXT Early Resume entered\n");

	if (mxt_resume(&mxt->client->dev) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);
	//dev_info(dev, "MXT Early Resumed\n");
}
#else
static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
};
#endif
#endif

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
