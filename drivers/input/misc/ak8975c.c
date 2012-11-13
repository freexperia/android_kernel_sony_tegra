/*
 * Copyright (C) 2011,2012 Sony Corporation
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/ak8975c.h>

MODULE_LICENSE("GPL");


/*
 * AK8975C Registers
 */
#define REG_WIA		0x00
#define REG_INFO	0x01
#define REG_ST1		0x02
#define REG_HXL		0x03
#define REG_HXH		0x04
#define REG_HYL		0x05
#define REG_HYH		0x06
#define REG_HZL		0x07
#define REG_HZH		0x08
#define REG_ST2		0x09

#define REG_CNTL	0x0a
#define REG_RSV		0x0b
#define REG_ASTC	0x0c
#define REG_TS1		0x0d
#define REG_TS2		0x0e
#define REG_I2CDIS	0x0f

#define REG_ASAX	0x10
#define REG_ASAY	0x11
#define REG_ASAZ	0x12


struct ak8975c_data {
	struct input_dev	*input_dev;
	struct work_struct	work;
	struct semaphore	sem;
	struct ak8975c_platform_data plat_data;
	u8			asa[3];
	s16			offset[3];
	s32			adj[3];
	s32			delay;
	s32			open_count;
	s8			is_active;
};


static void set_power(s8 on);
static void ak8975c_set_active(struct ak8975c_data *mag, s8 is_active);
static void ak8975c_start_measurement(struct ak8975c_data *magnetometer);
static void ak8975c_stop_measurement(struct ak8975c_data *magnetometer);
static void timer_func(unsigned long arg);

static DEFINE_TIMER(g_polling_timer, timer_func, 0, 0);
static struct i2c_client *g_client;

#define MIN_DELAY	(16)

static inline int32_t div256(int32_t a)
{
	int32_t t;
	if (a > 0) {
		t = a + 128;
		return t >> 8;
	} else {
		t = -a + 128;
		return -(t >> 8);
	}
}

static s32 _i2c_read_byte(struct i2c_client *client, u8 command)
{
	s32 err;

	err = i2c_smbus_read_byte_data(client, command);
	if (err < 0) {
		pr_err("i2c_smbus_read_byte_data: %d\n", err);
	}
	return err;
}

static s32 _i2c_write_byte(struct i2c_client *client, u8 command, u8 value)
{
	s32 err;

	err = i2c_smbus_write_byte_data(client, command, value);
	if (err < 0) {
		pr_err("i2c_smbus_write_byte_data: %d\n", err);
	}
	return err;
}

#define DEF_REG8_SHOW(reg) \
static ssize_t reg##_show(struct device *dev, struct device_attribute *attr, char* buf) \
{ \
	return snprintf(buf, 8, "%02x\n", (u8)_i2c_read_byte(g_client, REG_##reg)); \
}


#define DEF_REG8_STORE(reg) \
static ssize_t reg##_store(struct device *dev, struct device_attribute *attr, \
		const char* buf, size_t count) \
{ \
	s32 err; \
	unsigned long ul = simple_strtoul(buf, NULL, 0); \
	if (ul > 0xff) { \
		return -EINVAL; \
	}\
	err = _i2c_write_byte(g_client, REG_##reg, ul); \
	return err < 0 ? err : count; \
}

#define DEF_REG16_SHOW(reg) \
static ssize_t reg##_show(struct device *dev, struct device_attribute *attr, char* buf) \
{ \
	return snprintf(buf, 8, "%04x\n", (u8)_i2c_read_word(g_client, REG_##reg)); \
}

#define DEF_REG16_STORE(reg) \
static ssize_t reg##_store(struct device *dev, struct device_attribute *attr, \
		const char* buf, size_t count) \
{ \
	s32 err; \
	unsigned long ul = simple_strtoul(buf, NULL, 0); \
	if (ul > 0xffff) { \
		return -EINVAL; \
	}\
	err = _i2c_write_word(g_client, REG_##reg, ul); \
	return err < 0 ? err : count; \
}

#define DIAG_REG8_RO(reg) \
DEF_REG8_SHOW(reg) \
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, reg##_show, NULL)

#define DIAG_REG8_RW(reg) \
DEF_REG8_SHOW(reg) \
DEF_REG8_STORE(reg) \
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, reg##_show, reg##_store)

#define DIAG_REG16_RO(reg) \
DEF_REG16_SHOW(reg) \
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, reg##_show, NULL)

#define DIAG_REG16_RW(reg) \
DEF_REG16_SHOW(reg) \
DEF_REG16_STORE(reg) \
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, reg##_show, reg##_store)


DIAG_REG8_RO(WIA);
DIAG_REG8_RO(INFO);
DIAG_REG8_RO(ST1);
DIAG_REG8_RO(HXL);
DIAG_REG8_RO(HXH);
DIAG_REG8_RO(HYL);
DIAG_REG8_RO(HYH);
DIAG_REG8_RO(HZL);
DIAG_REG8_RO(HZH);
DIAG_REG8_RO(ST2);

DIAG_REG8_RW(CNTL);
DIAG_REG8_RW(RSV);
DIAG_REG8_RW(ASTC);
DIAG_REG8_RW(TS1);
DIAG_REG8_RW(TS2);
DIAG_REG8_RW(I2CDIS);

DIAG_REG8_RO(ASAX);
DIAG_REG8_RO(ASAY);
DIAG_REG8_RO(ASAZ);


static ssize_t is_active_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);
	return snprintf(buf, 8, "%x\n", mag->is_active);
}
static DEVICE_ATTR(is_active, S_IRUGO, is_active_show, NULL);


static int initialize_device_files(struct device* dev)
{
	int i;
	int err = 0;
	static struct device_attribute* attrs[] = {
		&dev_attr_WIA,
		&dev_attr_INFO,
		&dev_attr_ST1,
		&dev_attr_HXL,
		&dev_attr_HXH,
		&dev_attr_HYL,
		&dev_attr_HYH,
		&dev_attr_HZL,
		&dev_attr_HZH,
		&dev_attr_ST2,
		&dev_attr_CNTL,
		&dev_attr_RSV,
		&dev_attr_ASTC,
		&dev_attr_TS1,
		&dev_attr_TS2,
		&dev_attr_I2CDIS,
		&dev_attr_ASAX,
		&dev_attr_ASAY,
		&dev_attr_ASAZ,
		&dev_attr_is_active,
	};
	for (i = 0; i < ARRAY_SIZE(attrs); ++i) {
		err = device_create_file(dev, attrs[i]);
		if (err) {
			pr_err("device_create_file: %d\n", err);
			break;
		}
	}
	return err;
}


static int read_data(struct ak8975c_data *mag, s32* px, s32* py, s32* pz)
{
	s32 err;
	s32 x, y, z, t;
	s32 xx, yy, zz;
	s8 data[7];
	u8 st2;
	s8* R;

	/* Read HXL ... ST2*/
	err = i2c_smbus_read_i2c_block_data(g_client, REG_HXL, sizeof data, data);
	if (err < 0) {
		pr_err("i2c_smbus_read_i2c_block_data: %d\n", err);
		goto out;
	}
	x = (((s16)data[1]) << 8) | ((u8)data[0]);
	y = (((s16)data[3]) << 8) | ((u8)data[2]);
	z = (((s16)data[5]) << 8) | ((u8)data[4]);
	st2 = data[6];
	if (st2 != 0) {
		err = -1;
		/* overflow */
		goto out;
	}
	t = x * mag->adj[0];
	x = div256(t);

	t = y * mag->adj[1];
	y = div256(t);

	t = z * mag->adj[2];
	z = div256(t);
/*
 * Swap the axes if necessory
 */
	R = mag->plat_data.transformation_matrix;
	xx = R[0] * x + R[1] * y + R[2] * z;
	yy = R[3] * x + R[4] * y + R[5] * z;
	zz = R[6] * x + R[7] * y + R[8] * z;

	*px = xx;
	*py = yy;
	*pz = zz;
	err = 0;
out:
	return err;
}


static void work_func(struct work_struct *work)
{
	s32 err;
	s32 x,y,z;
	s32 data;
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);

	x = y = z = 0;
	err = data = _i2c_read_byte(g_client, REG_ST1);
	if (err < 0) {
		goto out;
	}
	if ((data & 0x01) == 0) {
		goto out;
	}

	err = read_data(mag, &x, &y, &z);
	_i2c_write_byte(g_client, REG_CNTL, 0x01);
	if (err < 0) {
		goto out;
	}
	input_report_abs(mag->input_dev, ABS_X, x);
	input_report_abs(mag->input_dev, ABS_Y, y);
	input_report_abs(mag->input_dev, ABS_Z, z);
	input_sync(mag->input_dev);
out:
	return;
}


static void set_power(s8 on)
{
	if (on) {
		;
	} else {
		_i2c_write_byte(g_client, REG_CNTL, 0x00);
	}
}


static void timer_func(unsigned long arg)
{
	struct ak8975c_data *mag = (struct ak8975c_data*)arg;

	if (!work_pending(&mag->work)) {
		schedule_work(&mag->work);
	}

	g_polling_timer.expires = jiffies + msecs_to_jiffies(mag->delay);
	g_polling_timer.data = (unsigned long)mag;
	add_timer(&g_polling_timer);
}


static void ak8975c_start_measurement(struct ak8975c_data *mag)
{
	pr_debug("called\n");

	_i2c_write_byte(g_client, REG_CNTL, 0x01);

	g_polling_timer.expires = jiffies + msecs_to_jiffies(1);
	g_polling_timer.data = (unsigned long)mag;
	add_timer(&g_polling_timer);
}


static void ak8975c_stop_measurement(struct ak8975c_data *mag)
{
	pr_debug("called\n");
	del_timer_sync(&g_polling_timer);

	cancel_work_sync(&mag->work);
}


static void ak8975c_set_active(struct ak8975c_data *mag, s8 is_active)
{
	is_active = is_active ? 1 : 0;
	if (mag->is_active == is_active) {
		return ;
	}
	if (is_active) {
		/* POWER ON */
		set_power(1);
		ak8975c_start_measurement(mag);
		mag->is_active = 1;
	} else {
		mag->is_active = 0;
		ak8975c_stop_measurement(mag);
		/* POWER OFF */
		set_power(0);
	}
	return ;
}


static int ak8975c_open(struct inode *inode, struct file* filp)
{
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);
	int retval;

	retval = 0;
	if (down_interruptible(&mag->sem)) {
		retval = -ERESTARTSYS;
		goto out;
	}
	pr_debug("called: %d\n", mag->open_count);
	{
		mag->open_count++;
		if (mag->open_count == 1) {
			mag->delay = MIN_DELAY;
		}
	}
	up(&mag->sem);
out:
	return retval;
}


static int ak8975c_release(struct inode *inode, struct file* filp)
{
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);
	int retval;

	retval = 0;
	if (down_interruptible(&mag->sem)) {
		retval = -ERESTARTSYS;
		goto out;
	}
	pr_debug("called: %d\n", mag->open_count);
	{
		mag->open_count--;
		if (mag->open_count < 1) {
			mag->open_count = 0;
			ak8975c_set_active(mag, 0);
		}
	}
	up(&mag->sem);
out:
	return retval;
}

static long ak8975c_ioctl(
		struct file* filp, unsigned int cmd, unsigned long arg)
{
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);
	int	 retval;

	retval = -ENOTTY;
	if (_IOC_TYPE(cmd) != AK8975C_IOC_MAGIC) {
		goto out;
	}
	if (_IOC_NR(cmd) >= AK8975C_IOC_MAXNR) {
		goto out;
	}
	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (!access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto out;
		}
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (!access_ok(VERIFY_WRITE,
					(void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto out;
		}
	}

	if (down_interruptible(&mag->sem)) {
		retval = -ERESTARTSYS;
		goto out;
	}

	{
		s8 prev;
		s32 n;
		s16 data[3];
		switch (cmd) {
			case AK8975C_IOC_GET_ACTIVE:
				n = mag->is_active;
				pr_debug("AK8975C_IOC_GET_ACTIVE: %d\n", n);
				retval = copy_to_user(
							(void __user*)arg, &n, sizeof(n));
				if (retval != 0) {
					goto out_locked;
				}
				break;
			case AK8975C_IOC_SET_ACTIVE:
				retval = copy_from_user(
							&n, (void __user*)arg, sizeof(n));
				if (retval != 0) {
					goto out_locked;
				}
				pr_debug("AK8975C_IOC_SET_ACTIVE: %d, %d\n",
							n, mag->is_active);
				ak8975c_set_active(mag, n);
				break;
			case AK8975C_IOC_DO_CALIBRATION:
				goto out_locked;
				break;
			case AK8975C_IOC_GET_OFS_DATA:
				memcpy(&data[0], mag->offset, sizeof(mag->offset));
				retval = copy_to_user(
							(void __user*)arg, data, sizeof(data));
				if (retval != 0) {
					goto out_locked;
				}
				pr_debug("AK8975C_IOC_GET_OFS_DATA: "
							"0x%04x, 0x%04x, 0x%04x\n",
							(u16)mag->offset[0],
							(u16)mag->offset[1],
							(u16)mag->offset[2]);
				break;
			case AK8975C_IOC_SET_OFS_DATA:
				retval = copy_from_user(
							data, (void __user*)arg, sizeof(data));
				if (retval != 0) {
					goto out_locked;
				}
				prev = mag->is_active;
				ak8975c_set_active(mag, 0);
				memcpy(mag->offset, &data[0], sizeof(mag->offset));
				pr_debug("AK8975C_IOC_SET_OFS_DATA: "
							"0x%04x, 0x%04x, 0x%04x\n",
							(u16)mag->offset[0],
							(u16)mag->offset[1],
							(u16)mag->offset[2]);
				ak8975c_set_active(mag, prev);
				break;
			case AK8975C_IOC_GET_B0_DATA:
				memcpy(&data[0], mag->asa, sizeof(mag->asa));
				retval = copy_to_user(
							(void __user*)arg, data, sizeof(data));
				if (retval != 0) {
					goto out_locked;
				}
				pr_debug("AK8975C_IOC_GET_B0_DATA: "
							"0x%04x, 0x%04x, 0x%04x\n",
							(u16)mag->asa[0],
							(u16)mag->asa[1],
							(u16)mag->asa[2]);
				break;
			case AK8975C_IOC_SET_B0_DATA:
				goto out_locked;
				break;
			case AK8975C_IOC_GET_DELAY:
				retval = copy_to_user(
							(void __user*)arg, &mag->delay, sizeof(mag->delay));
				if (retval != 0) {
					goto out_locked;
				}
				pr_debug("AK8975C_IOC_GET_DELAY: %d\n", mag->delay);
				break;
			case AK8975C_IOC_SET_DELAY:
				retval = copy_from_user(
							&n, (void __user*)arg, sizeof(n));
				if (retval != 0) {
					goto out_locked;
				}
				if (n < MIN_DELAY) {
					n = MIN_DELAY;
				}
				mag->delay = n;
				prev = mag->is_active;
				ak8975c_set_active(mag, 0);
				pr_debug("AK8975C_IOC_SET_DELAY: %d\n", mag->delay);
				ak8975c_set_active(mag, prev);
				break;
			default:
				goto out_locked;
		}
	}
	retval = 0;
out_locked:
	up(&mag->sem);

out:
	return retval;
}


static struct file_operations ak8975c_fops = {
	.owner = THIS_MODULE,
	.open = ak8975c_open,
	.release = ak8975c_release,
	.unlocked_ioctl = ak8975c_ioctl,
};

static struct miscdevice ak8975c_dev = {
	MISC_DYNAMIC_MINOR,
	"ak8975c",
	&ak8975c_fops,
};


static int ak8975c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	s32 data;
	int err;
	int retval;
	struct ak8975c_data *mag;

	pr_info("probe: %s\n", client->name);
	retval = 0;

	data = _i2c_read_byte(client, REG_WIA);
	if (data != 0x48) {
		pr_err("unknown device: %2x\n", data);
		retval = -ENODEV;
		goto err;
	}

	mag = kzalloc(sizeof *mag, GFP_KERNEL);
	if (mag == NULL) {
		pr_err("kzalloc()\n");
		retval = -ENOMEM;
		goto err_kzalloc;
	}

	if (client->dev.platform_data == NULL) {
		s8 E[9] = { 1, 0, 0, 
			    0, 1, 0, 
			    0, 0, 1 };
		memcpy(mag->plat_data.transformation_matrix, E, 
			sizeof mag->plat_data.transformation_matrix);
	} else {
		memcpy(&mag->plat_data, client->dev.platform_data,
			sizeof (struct ak8975c_platform_data));
	}
	sema_init(&mag->sem, 1);
	INIT_WORK(&mag->work, work_func);

	mag->input_dev = input_allocate_device();
	if (mag->input_dev == NULL) {
		pr_err("input_allocate_device()\n");
		retval = -ENOMEM;
		goto err_input_allocate_device;
	}
	mag->input_dev->name = "ak8975c";
	set_bit(EV_ABS, mag->input_dev->evbit);
	input_set_abs_params(mag->input_dev, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(mag->input_dev, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(mag->input_dev, ABS_Z, -32768, 32767, 0, 0);

	err = input_register_device(mag->input_dev);
	if (err) {
		pr_err("input_alloc_device(): %d\n", err);
		retval = err;
		goto err_input_register_device;
	}

	err = misc_register(&ak8975c_dev);
	if (err) {
		pr_err("misc_register(): %d\n", err);
		retval = err;
		goto err_misc_register;
	}

	i2c_set_clientdata(client, mag);
	g_client = client;

	/* Fuse ROM access mode */
	data = _i2c_write_byte(g_client, REG_CNTL, 0xff);
	if (data < 0) {
		retval = data;
		goto err_misc_register;
	}
	err = i2c_smbus_read_i2c_block_data(g_client, REG_ASAX, sizeof mag->asa, mag->asa);
	if (err < 0) {
		pr_err("i2c_smbus_read_i2c_block_data: %d\n", err);
		retval = err;
		goto err_misc_register;
	}

	mag->adj[0] = mag->asa[0] + 128;
	mag->adj[1] = mag->asa[1] + 128;
	mag->adj[2] = mag->asa[2] + 128;
	pr_debug("Initial ASAX: 0x%02x\n", mag->asa[0]);
	pr_debug("Initial ASAY: 0x%02x\n", mag->asa[1]);
	pr_debug("Initial ASAZ: 0x%02x\n", mag->asa[2]);
	pr_debug("Adjust X: 0x%02x\n", mag->adj[0]);
	pr_debug("Adjust Y: 0x%02x\n", mag->adj[1]);
	pr_debug("Adjust Z: 0x%02x\n", mag->adj[2]);

	/* Power down mode */
	data = _i2c_write_byte(g_client, REG_CNTL, 0x00);
	if (data < 0) {
		retval = data;
		goto err_misc_register;
	}

	err = initialize_device_files(&client->dev);
	if (err < 0) {
		pr_err("initialize_device_files (%d)\n", err);
	}

	return retval;

err_misc_register:
err_input_register_device:
	input_unregister_device(mag->input_dev);
err_input_allocate_device:
	kfree(mag);
err_kzalloc:
err:
	return retval;
}

static int ak8975c_remove(struct i2c_client *client)
{
	struct ak8975c_data *mag = i2c_get_clientdata(client);

	pr_debug("called\n");
	misc_deregister(&ak8975c_dev);
	input_unregister_device(mag->input_dev);
	i2c_set_clientdata(client, NULL);
	g_client = NULL;
	kfree(mag);

	return 0;
}

#ifdef CONFIG_PM

static int ak8975c_suspend(struct device* dev)
{
	int retval;
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);

	pr_debug("called\n");
	retval = 0;
	if (down_interruptible(&mag->sem)) {
		retval = -ERESTARTSYS;
		goto out;
	}
	{
		if (mag->open_count > 0) {
			ak8975c_stop_measurement(mag);
			set_power(0);
		}
	}
	up(&mag->sem);
out:
	return retval;
}

static int ak8975c_resume(struct device* dev)
{
	int retval;
	struct ak8975c_data *mag = i2c_get_clientdata(g_client);

	pr_debug("called\n");
	retval = 0;
	if (down_interruptible(&mag->sem)) {
		retval = -ERESTARTSYS;
		goto out;
	}
	{
		if (mag->open_count > 0) {
			ak8975c_set_active(mag, mag->is_active);
		}
	}
	up(&mag->sem);
out:
	return retval;
}

static struct dev_pm_ops ak8975c_pm_ops = {
	.suspend = ak8975c_suspend,
	.resume = ak8975c_resume,
};

#define AK8975C_PM_OPS (&ak8975c_pm_ops)

#else /* CONFIG_PM */
#define AK8975C_PM_OPS NULL

#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(i2c, ak8975c_idtable);

static struct i2c_device_id ak8975c_idtable[] = {
	{ "ak8975c", 0 },
	{ }
};

static struct i2c_driver ak8975c_driver = {
	.driver = {
		.name = "ak8975c",
		.owner = THIS_MODULE,
		.pm = AK8975C_PM_OPS,
	},
	.probe = ak8975c_probe,
	.remove = ak8975c_remove,
	.id_table = ak8975c_idtable,
};

static int __init ak8975c_driver_init(void)
{
	int res;

	res = i2c_add_driver(&ak8975c_driver);

	return res;
}

static void __exit ak8975c_driver_exit(void)
{
	i2c_del_driver(&ak8975c_driver);
}

module_init(ak8975c_driver_init);
module_exit(ak8975c_driver_exit);

