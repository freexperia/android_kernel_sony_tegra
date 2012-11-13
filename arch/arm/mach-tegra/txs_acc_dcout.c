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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>

#include "txs_acc_det.h"

static int txs_acc_dcout_pin_num = -1;
static const int txs_acc_dcout_pin_active = 1;

static void txs_acc_dcout_pin_update(int state, int mask, int maskable)
{
	int pin_mask;
	static int dcout_pin_state = -1;
	static int dcout_pin_mask = -1;
	static int dcout_pin_maskable = -1;

	if(0 <= state) {
		dcout_pin_state = state;
	}
	if(0 <= mask) {
		dcout_pin_mask = mask;
	}
	if(0 <= maskable) {
		dcout_pin_maskable = maskable;
	}
	/* not initialize yet. */
	if( (dcout_pin_state < 0) ||
		(dcout_pin_mask < 0) ||
		(dcout_pin_maskable < 0) ) return;

	pin_mask = dcout_pin_mask && dcout_pin_maskable;

	if( !pin_mask && dcout_pin_state ){
		gpio_set_value(txs_acc_dcout_pin_num, txs_acc_dcout_pin_active);
	}
	else {
		gpio_set_value(txs_acc_dcout_pin_num, !txs_acc_dcout_pin_active);
	}
}
#define txs_acc_dcout_pin_set() txs_acc_dcout_pin_update(1, -1, -1)
#define txs_acc_dcout_pin_clear() txs_acc_dcout_pin_update(0, -1, -1)
#define txs_acc_dcout_pin_mask() txs_acc_dcout_pin_update(-1, 1, -1)
#define txs_acc_dcout_pin_unmask() txs_acc_dcout_pin_update(-1, 0, -1)
#define txs_acc_dcout_pin_maskable() txs_acc_dcout_pin_update(-1, -1, 1)
#define txs_acc_dcout_pin_unmaskable() txs_acc_dcout_pin_update(-1, -1, 0)

/*--- define DC out control pattern ---*/
struct {
	int num;
	int maskable;
} static txs_acc_dcout_control_table[] = {
	{ TXS_ACC_DCOUT_RESERVED, 0, }, /* reserved. unmaskable. */
	{ TXS_ACC_HDMI_DONGLE,    1, }, /* HDMI.     maskable. */
};
static int txs_acc_dcout_control_table_size = sizeof(txs_acc_dcout_control_table) / sizeof(txs_acc_dcout_control_table[0]);

static void txs_acc_dcout_switch_changed(void)
{
	int i;
	int det_num;
	det_num = txs_acc_det_get_num();

	for(i = 0; i < txs_acc_dcout_control_table_size; i++) {
		if(txs_acc_dcout_control_table[i].num == det_num) {
			txs_acc_dcout_pin_set();

			if(txs_acc_dcout_control_table[i].maskable) {
				txs_acc_dcout_pin_maskable();
			}
			else {
				txs_acc_dcout_pin_unmaskable();
			}
			break;
		}
	}
	if(txs_acc_dcout_control_table_size <= i) {
		/* not found */
		txs_acc_dcout_pin_clear();
		txs_acc_dcout_pin_maskable();
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void txs_acc_dcout_early_suspend(struct early_suspend *h)
{
}
static void txs_acc_dcout_late_resume(struct early_suspend *h)
{
}
struct early_suspend txs_acc_dcout_early_suspender = {
	.suspend = &txs_acc_dcout_early_suspend,
	.resume  = &txs_acc_dcout_late_resume,
	.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_SUSPEND

static int txs_acc_dcout_suspend(struct device *dev)
{
	txs_acc_dcout_pin_mask();

	return 0;
}
static int txs_acc_dcout_resume(struct device *dev)
{
	txs_acc_dcout_pin_unmask();

	return 0;
}
static const struct dev_pm_ops txs_acc_dcout_pm_ops = {
	.suspend = txs_acc_dcout_suspend,
	.resume  = txs_acc_dcout_resume,
};

#endif /* CONFIG_SUSPEND */

static int txs_acc_dcout_probe(struct platform_device* pdev)
{
	int ret = 0;
	struct txs_acc_dcout_platform_data* acc_dcout_pdata =
		(struct txs_acc_dcout_platform_data*)(pdev->dev.platform_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* register_early_suspend(&txs_acc_dcout_early_suspender); */
#endif

	/* init GPIO ACC DCOUT pin */
	txs_acc_dcout_pin_num = acc_dcout_pdata->dcout_pin_num;
	tegra_gpio_enable(txs_acc_dcout_pin_num);
	ret = gpio_request(txs_acc_dcout_pin_num, "TXS Accessory DC output");
	if(ret < 0) {
		pr_err("txs_acc_dcout:gpio_request() failed. %d\n", ret);
		goto error_exit;
	}
	gpio_direction_output(txs_acc_dcout_pin_num, 0);

	txs_acc_dcout_pin_clear();
	txs_acc_dcout_pin_unmask();
	txs_acc_dcout_pin_maskable();
	txs_acc_det_register_callback( &txs_acc_dcout_switch_changed );
	txs_acc_dcout_switch_changed(); /* 1st check */

	return 0;

error_exit:
	txs_acc_det_unregister_callback( &txs_acc_dcout_switch_changed );

	gpio_free(txs_acc_dcout_pin_num);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* unregister_early_suspend(&txs_acc_dcout_early_suspender); */
#endif
	return ret;
}

static int txs_acc_dcout_remove(struct platform_device* pdev)
{
	txs_acc_det_unregister_callback( &txs_acc_dcout_switch_changed );

	gpio_free(txs_acc_dcout_pin_num);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* unregister_early_suspend(&txs_acc_dcout_early_suspender); */
#endif
	return 0;
}

static struct platform_driver txs_acc_dcout_driver = {
	.probe   = txs_acc_dcout_probe,
	.remove  = txs_acc_dcout_remove,
	.driver  = {
		.name  = "txs_acc_dcout",
		.owner = THIS_MODULE,
#ifdef CONFIG_SUSPEND
		.pm    = &txs_acc_dcout_pm_ops,
#endif
	},
};

static int __init txs_acc_dcout_init(void)
{
	int ret;

	ret = platform_driver_register(&txs_acc_dcout_driver);
	if (ret < 0) {
		pr_err("%s:platform_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit txs_acc_dcout_exit(void)
{
	platform_driver_unregister(&txs_acc_dcout_driver);
}

module_init(txs_acc_dcout_init);
module_exit(txs_acc_dcout_exit);

MODULE_LICENSE("GPL");

