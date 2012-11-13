/*
 * Copyright (C) 2012 Sony Corporation
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

/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	int			flags;
	int			duty;
};

#define BL_FLAGS_PWM_ENABLE	(1 << 0)

/*
** conversion table
** from brightness(0-255) to backlight pwm(0-100)
*/
static const int bl_tbl[256] = {
	0, 70, 150, 230, 310, 390, 470, 540, 620, 700,
	780, 840, 900, 960, 1010, 1070, 1130, 1190, 1250, 1310,
	1370, 1370, 1390, 1400, 1420, 1430, 1440, 1450, 1450, 1450,
	1460, 1532, 1605, 1680, 1757, 1836, 1917, 1999, 2084, 2171,
	2259, 2349, 2442, 2536, 2632, 2730, 2830, 2932, 3036, 3142,
	3250, 3274, 3297, 3321, 3344, 3368, 3392, 3415, 3439, 3463,
	3487, 3512, 3536, 3560, 3585, 3609, 3634, 3659, 3684, 3709,
	3734, 3759, 3784, 3809, 3835, 3860, 3886, 3912, 3937, 3963,
	3989, 4015, 4041, 4068, 4094, 4120, 4147, 4173, 4200, 4227,
	4254, 4281, 4308, 4335, 4362, 4390, 4417, 4445, 4472, 4500,
	4528, 4556, 4584, 4612, 4640, 4668, 4697, 4725, 4754, 4782,
	4811, 4840, 4869, 4898, 4927, 4956, 4986, 5015, 5044, 5074,
	5104, 5134, 5163, 5193, 5223, 5254, 5284, 5314, 5345, 5375,
	5406, 5437, 5468, 5498, 5529, 5561, 5592, 5623, 5655, 5686,
	5718, 5749, 5781, 5813, 5845, 5877, 5909, 5941, 5974, 6006,
	6039, 6072, 6104, 6137, 6170, 6203, 6236, 6269, 6303, 6336,
	6370, 6403, 6437, 6471, 6505, 6539, 6573, 6607, 6641, 6676,
	6710, 6745, 6779, 6814, 6849, 6884, 6919, 6954, 6990, 7025,
	7060, 7096, 7132, 7167, 7203, 7239, 7275, 7311, 7347, 7384,
	7420, 7457, 7493, 7530, 7567, 7604, 7641, 7678, 7715, 7753,
	7790, 7827, 7865, 7903, 7941, 7978, 8016, 8055, 8093, 8131,
	8169, 8208, 8247, 8285, 8324, 8363, 8402, 8441, 8480, 8519,
	8559, 8598, 8638, 8678, 8717, 8757, 8797, 8837, 8877, 8918,
	8958, 8999, 9039, 9080, 9121, 9162, 9202, 9244, 9285, 9326,
	9367, 9409, 9450, 9492, 9534, 9576, 9618, 9660, 9702, 9744,
	9787, 9829, 9872, 9914, 9957, 10000,
};


static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		if (pb->flags & BL_FLAGS_PWM_ENABLE) {
			pwm_config(pb->pwm, 0, pb->period);
			pwm_disable(pb->pwm);
			pb->flags &= ~BL_FLAGS_PWM_ENABLE;
			pb->duty = 0;
		}
	} else {
		brightness = pb->period / 10000 * bl_tbl[brightness];
		if (!(pb->flags & BL_FLAGS_PWM_ENABLE) || 
					(pb->duty != brightness)) {
			pwm_config(pb->pwm, brightness, pb->period);
			pb->duty = brightness;
		}
		if (!(pb->flags & BL_FLAGS_PWM_ENABLE)) {
			if (!pwm_enable(pb->pwm)) {
				pb->flags |= BL_FLAGS_PWM_ENABLE;
			}
		}
	}
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	if (pb->flags & BL_FLAGS_PWM_ENABLE) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
		pb->flags &= ~BL_FLAGS_PWM_ENABLE;
		pb->duty = 0;
	}
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);

	if (pb->flags & BL_FLAGS_PWM_ENABLE) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
		pb->flags &= ~BL_FLAGS_PWM_ENABLE;
		pb->duty = 0;
	}
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

