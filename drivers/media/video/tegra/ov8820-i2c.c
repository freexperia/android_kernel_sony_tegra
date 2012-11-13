/* 2012-07-20: File changed by Sony Corporation */
/*
 * ov8820.c - ov8820 sensor driver
 *
 * Copyright (C) 2011 Google Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov8820-i2c.h>
#include <media/ov8820.h>

#define OV8820_MAX_RETRIES 3

struct ov8820_i2c_info {
	int mode;
	struct i2c_client *i2c_client;
	struct ov8820_platform_data *pdata;
};

static struct ov8820_i2c_info *info;

int ov8820_i2c_write_reg(struct i2c_client *badclient, u16 addr, u8 val)
{
    struct i2c_client * client = info->i2c_client;
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("ov8820: i2c transfer failed, retrying %x %x\n",
			addr, val);
		usleep_range(3000, 3250);
	} while (retry <= OV8820_MAX_RETRIES);

	return err;
}
EXPORT_SYMBOL(ov8820_i2c_write_reg);

int ov8820_i2c_read_reg(struct i2c_client *badclient, u16 addr, u8 *val)
{
    struct i2c_client * client = info->i2c_client;
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	do {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err == 2)
			break;
		retry++;
		pr_err("ov8820: i2c transfer failed, retrying %x %x\n",
			addr, val);
		usleep_range(3000, 3250);
	} while (retry <= OV8820_MAX_RETRIES);

	*val = data[2];

	if (err != 2)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(ov8820_i2c_read_reg);


static int ov8820_i2c_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
	int err;
	pr_debug(KERN_DEBUG "%s: probing sensor.\n", __func__);

	info = kzalloc(sizeof(struct ov8820_i2c_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov8820: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
/*    if (info->pdata && info->pdata->power_on)
        info->pdata->power_on();
*/
	i2c_set_clientdata(client, info);
	return 0;
}

static int ov8820_i2c_remove(struct i2c_client *client)
{
pr_debug(KERN_DEBUG "OV8820 %s\n", __func__);
	struct ov8820_i2c_info *info;
	info = i2c_get_clientdata(client);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ov8820_i2c_id[] = {
	{ "ov8820-i2c", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov8820_i2c_id);

static struct i2c_driver ov8820_i2c_driver = {
	.driver = {
		.name = "ov8820-i2c",
		.owner = THIS_MODULE,
	},
	.probe = ov8820_i2c_probe,
	.remove = ov8820_i2c_remove,
	.id_table = ov8820_i2c_id,
};

static struct miscdevice ov8820_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov8820-i2c",
	.fops = NULL,
};

static int __init ov8820_i2c_init(void)
{
pr_debug(KERN_DEBUG "OV8820 %s\n", __func__);
	misc_register(&ov8820_device);
	return i2c_add_driver(&ov8820_i2c_driver);
}

static void __exit ov8820_i2c_exit(void)
{
pr_debug(KERN_DEBUG "OV8820 %s\n", __func__);
	i2c_del_driver(&ov8820_i2c_driver);
}

module_init(ov8820_i2c_init);
module_exit(ov8820_i2c_exit);

