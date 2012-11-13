/* 2012-07-20: File changed by Sony Corporation */
/*
 * OV8820 focuser driver.
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
 *
 * Contributors:
 *      Sachin Nikam <snikam@nvidia.com>
 *
 * Based on ov5650.c.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/* Implementation
 * --------------
 * The board level details about the device need to be provided in the board
 * file with the ov8820_focuser_platform_data structure.
 * Standard among NVODM kernel drivers in this structure is:
 * .cfg = Use the NVODM_CFG_ defines that are in nvodm.h.
 *        Descriptions of the configuration options are with the defines.
 *        This value is typically 0.
 * .num = The number of the instance of the device.  This should start at 1 and
 *        and increment for each device on the board.  This number will be
 *        appended to the MISC driver name, Example: /dev/focuser.1
 *        If not used or 0, then nothing is appended to the name.
 * .sync = If there is a need to synchronize two devices, then this value is
 *         the number of the device instance (.num above) this device is to
 *         sync to.  For example:
 *         Device 1 platform entries =
 *         .num = 1,
 *         .sync = 2,
 *         Device 2 platfrom entries =
 *         .num = 2,
 *         .sync = 1,
 *         The above example sync's device 1 and 2.
 *         This is typically used for stereo applications.
 * .dev_name = The MISC driver name the device registers as.  If not used,
 *             then the part number of the device is used for the driver name.
 *             If using the ODM user driver then use the name found in this
 *             driver under _default_pdata.
 *
 * The following is specific to NVODM kernel focus drivers:
 * .odm = Pointer to the nvodm_focus_odm structure.  This structure needs to
 *        be defined and populated if overriding the driver defaults.
 * .cap = Pointer to the nvodm_focus_cap structure.  This structure needs to
 *        be defined and populated if overriding the driver defaults.
 *
 * The following is specific to only this NVODM kernel focus driver:
 * .info = Pointer to the ov8820_focuser_pdata_info structure.  This structure does
 *         not need to be defined and populated unless overriding ROM data.
 * .gpio_reset = The GPIO connected to the devices reset.  If not used then
 *               leave blank.
 * .gpio_en = Due to a Linux limitation, a GPIO is defined to "enable" the
 *            device.  This workaround is for when the device's power GPIO's
 *            are behind an I2C expander.  The Linux limitation doesn't allow
 *            the I2C GPIO expander to be ready for use when this device is
 *            probed.  When this problem is solved, this driver needs to
 *            hard-code the regulator names (vreg_vdd & vreg_i2c) and remove
 *            the gpio_en WAR.
 * .vreg_vdd = This is the name of the power regulator for the device's power.
 * .vreg_i2c = This is the name of the power regulator for I2C power.
.* .i2c_addr_rom = The I2C address of the onboard ROM.
 *
 * The above values should be all that is needed to use the device with this
 * driver.  Modifications of this driver should not be needed.
 */


#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov8820-i2c.h>
#include <media/ov8820_focuser.h>

#define POS_LOW (192)
#define POS_HIGH (1016)
#define SETTLETIME_MS 200

#define FOCAL_LENGTH (4.27f)
#define FNUMBER (2.65f)

/* #define FPOS_COUNT 1024 */

#define ov8820_vcm_mode 0x01

struct ov8820_focuser_info {
	struct miscdevice miscdev;
	struct platform_device *platform_device;
	struct ov8820_focuser_platform_data *pdata;
	struct ov8820_foc_config config;
	atomic_t in_use;
	u32 cur_pos;
};

static u8 OTPReadOnce = 0;

static int ov8820_focuser_set_position(struct ov8820_focuser_info *info, u32 position)
{
	int err = 0;

	if (position < info->config.pos_low ||
	    position > info->config.pos_high)
		return -EINVAL;

	pr_debug(KERN_DEBUG "%s %d\n", __func__, position);

	/* err = ov8820_i2c_write_reg(NULL, 0x361A, 0xB0);
	err |= ov8820_i2c_write_reg(NULL, 0x361B, 0x04); */

	/* Set Position */
	err |= ov8820_i2c_write_reg(NULL, 0x3619, ((position & 0x3F0) >> 4));
	/* chahuang */
	err |= ov8820_i2c_write_reg(NULL, 0x3618,
				(((position & 0x00F) << 4) | ov8820_vcm_mode));

	if (err) {
		pr_err("ov8820_focuser: %s: set position failed\n", __func__);
		return err;
	}
	info->cur_pos = position;
	return 0;
}

static int ov8820_focuser_read_otp(struct ov8820_focuser_info *info)
{

	int nerr = 0;
	u32 temp_low;
	u32 temp_high;
	u8 pos_low[3];
	u8 pos_high[3];

	if ( OTPReadOnce ) return 0;

	OTPReadOnce = 1;

	nerr = ov8820_i2c_write_reg(NULL, 0x0103, 0x01);
	msleep(5);
	nerr |= ov8820_i2c_write_reg(NULL, 0x0103, 0x00);
	msleep(5);
	nerr |= ov8820_i2c_write_reg(NULL, 0x0100, 0x01);
	msleep(5);
	/*
	 * Read Macro and Inf position from OTP
	 */

	/* Buffer reset */
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d13, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d14, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d15, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d16, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d17, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d18, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d19, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d1a, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d1b, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d1c, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d1d, 0x00);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d1e, 0x00);

	/* Read sequence */
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d84, 0x09);
	nerr |= ov8820_i2c_write_reg(NULL, 0x3d81, 0x01);

	msleep(1);

	nerr |= ov8820_i2c_read_reg(NULL, 0x3d14, &pos_low[0]);
	nerr |= ov8820_i2c_read_reg(NULL, 0x3d18, &pos_low[1]);
	nerr |= ov8820_i2c_read_reg(NULL, 0x3d1c, &pos_low[2]);
	nerr |= ov8820_i2c_read_reg(NULL, 0x3d16, &pos_high[0]);
	nerr |= ov8820_i2c_read_reg(NULL, 0x3d1a, &pos_high[1]);
	nerr |= ov8820_i2c_read_reg(NULL, 0x3d1e, &pos_high[2]);

	nerr |= ov8820_i2c_write_reg(NULL, 0x3d84, 0x00);

	if (nerr) {
		pr_err("%s : read otp failed\n.",__func__);
		return nerr;
	}

	if (pos_low[0] != 0xFF) {
		temp_low = (pos_low[0] << 2) & 0x3FC;
	} else if (pos_low[1] != 0xFF) {
		temp_low = (pos_low[1] << 2) & 0x3FC;
	} else if (pos_low[2] != 0xFF){
		temp_low = (pos_low[2] << 2) & 0x3FC;
	} else {
		pr_err("%s : pos_low should not be 0xFF\n.",__func__);
		return -EINVAL;
	}

	if (pos_high[0] != 0xFF) {
		temp_high = (pos_high[0] << 2) & 0x3FC;
	} else if (pos_high[1] != 0xFF) {
		temp_high = (pos_high[1] << 2) & 0x3FC;
	} else if (pos_high[2] != 0xFF){
		temp_high = (pos_high[2] << 2) & 0x3FC;
	} else {
		pr_err("%s : pos_high should not be 0xFF\n.",__func__);
		return -EINVAL;
	}

	if (temp_low >= temp_high) {
		pr_err("%s : pos_low(0x%x) should not be greater than pos_high(0x%x)\n",
				__func__, temp_low, temp_high);
		return -EINVAL;
	}

	info->config.pos_low = temp_low;
	info->config.pos_high = temp_high;

	pr_debug("pos_low=%#x,pos_high=%#x\n",
			info->config.pos_low,info->config.pos_high);

	return 0;
}

static long ov8820_focuser_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ov8820_focuser_info *info = file->private_data;

	switch (cmd) {
	case OV8820_FOC_IOCTL_READ_OTP:
	{
		int err;
		err = ov8820_focuser_read_otp(info);
		if (err) {
			pr_err("%s READ_OTP failed\n",__func__);
			return err;
		}

		if (copy_to_user((void __user *) arg,
					&info->config,
					sizeof(info->config))) {
			pr_err("ov8820_focuser: %s: %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	}
	case OV8820_FOC_IOCTL_SET_POSITION:
		return ov8820_focuser_set_position(info, (u32) arg);
	case OV8820_FOC_IOCTL_GET_CONFIG:
		if (copy_to_user((void __user *) arg,
				&info->config,
				sizeof(info->config))) {
			pr_err("ov8820_focuser: %s: %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ov8820_focuser_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ov8820_focuser_info *info;

	pr_debug(KERN_DEBUG "%s\n", __func__);
	if (!miscdev) {
		pr_err("miscdev == NULL\n");
		return -1;
	}
	info = container_of(miscdev, struct ov8820_focuser_info, miscdev);
	if (atomic_xchg(&info->in_use, 1)) {
		pr_err("Oops! device busy.\n");
		return -EBUSY;
	}

	file->private_data = info;

	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	return 0;
}

static int ov8820_focuser_release(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ov8820_focuser_info *info;

	if (!miscdev) {
		pr_err("miscdev == NULL\n");
		return -1;
	}

	info = container_of(miscdev, struct ov8820_focuser_info, miscdev);
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	pr_debug(KERN_DEBUG "%s\n", __func__);
	return 0;
}


static const struct file_operations ov8820_focuser_fileops = {
	.owner = THIS_MODULE,
	.open = ov8820_focuser_open,
	.unlocked_ioctl = ov8820_focuser_ioctl,
	.release = ov8820_focuser_release,
};

static struct miscdevice ov8820_focuser_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov8820_focuser",
	.fops = &ov8820_focuser_fileops,
};

static int ov8820_focuser_probe(struct platform_device *pdev,
			const struct platform_device_id *id)
{
	struct ov8820_focuser_info *info;
	int err;

	pr_debug(KERN_DEBUG "%s: probing sensor.\n", __func__);

	info = kzalloc(sizeof(struct ov8820_focuser_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov8820_focuser: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	memcpy(&info->miscdev, &ov8820_focuser_device, sizeof(struct miscdevice));
	err = misc_register(&info->miscdev);
	if (err) {
		pr_err("ov8820_focuser: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	if (pdev->dev.platform_data)
		info->pdata = pdev->dev.platform_data;
	info->platform_device = pdev;
	info->config.settle_time = SETTLETIME_MS;
	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.pos_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
	platform_set_drvdata(pdev, info);

	return 0;
}

static int ov8820_focuser_remove(struct platform_device *pdev)
{
	struct ov8820_focuser_info *info = platform_get_drvdata(pdev);
	misc_deregister(&ov8820_focuser_device);
	kfree(info);
	return 0;
}

static const struct platform_device_id ov8820_focuser_id[] = {
	{ "ov8820_focuser", 0 },
	{ },
};

MODULE_DEVICE_TABLE(platform, ov8820_focuser_id);

static struct platform_driver ov8820_focuser_driver = {
	.driver = {
		.name = "ov8820_focuser",
		.owner = THIS_MODULE,
	},
	.id_table = ov8820_focuser_id,
	.probe = ov8820_focuser_probe,
	.remove = ov8820_focuser_remove,
};

static int __init ov8820_focuser_init(void)
{
	pr_debug(KERN_DEBUG "[CAM] ov8820 focuser driver loading\n");
	return platform_driver_register(&ov8820_focuser_driver);
}

static void __exit ov8820_focuser_exit(void)
{
	OTPReadOnce = 0;
	platform_driver_unregister(&ov8820_focuser_driver);
}

module_init(ov8820_focuser_init);
module_exit(ov8820_focuser_exit);
