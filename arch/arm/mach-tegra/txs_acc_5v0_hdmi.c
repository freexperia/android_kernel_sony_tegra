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
#include <linux/regulator/consumer.h>

#include "txs_acc_det.h"

static struct regulator *txs_en_5v0_hdmi = NULL;
static int enabled = 0;

static void txs_acc_5v0_hdmi_switch_changed(void)
{
	int det_num;
	det_num = txs_acc_det_get_num();

	if (txs_en_5v0_hdmi == NULL) {
		txs_en_5v0_hdmi = regulator_get(NULL, "en_5v0_boost");
		if (WARN_ON(IS_ERR(txs_en_5v0_hdmi))) {
			pr_err("%s: couldn't get regulator en_5v0_boost: %ld\n",
			       __func__, PTR_ERR(txs_en_5v0_hdmi));
			return;
		}
	}

	/* Not connected */
	if( TXS_ACC_NOT_CONNECTED==det_num ){
		/* txs_en_5v0_hdmi is common GPIO with usb vbus.
		   Essentially, this "enabled" flag is unnecessary.
		   The virtual regulator (such as 5v0_hdmi)should be created and
		   the regulator should manage the reference counter. */
		if( enabled==1 ){
			regulator_disable(txs_en_5v0_hdmi);
			enabled = 0;
		}
	}else{
		if( enabled==0  ){
			regulator_enable(txs_en_5v0_hdmi);
			enabled = 1;
		}
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void txs_acc_5v0_hdmi_early_suspend(struct early_suspend *h)
{
	return;
}
static void txs_acc_5v0_hdmi_late_resume(struct early_suspend *h)
{
	return;
}
struct early_suspend txs_acc_5v0_hdmi_early_suspender = {
	.suspend = &txs_acc_5v0_hdmi_early_suspend,
	.resume  = &txs_acc_5v0_hdmi_late_resume,
	.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int txs_acc_5v0_hdmi_probe(struct platform_device* pdev)
{
	int ret = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&txs_acc_5v0_hdmi_early_suspender);
#endif

	/* regulator off */
	if (txs_en_5v0_hdmi == NULL) {
		txs_en_5v0_hdmi = regulator_get(NULL, "en_5v0_boost");
		if (WARN_ON(IS_ERR(txs_en_5v0_hdmi))) {
			pr_err("%s: couldn't get regulator en_5v0_boost: %ld\n",
			       __func__, PTR_ERR(txs_en_5v0_hdmi));
			goto error_exit;
		}
	}
	//regulator_disable(txs_en_5v0_hdmi);

	txs_acc_det_register_callback( &txs_acc_5v0_hdmi_switch_changed );
	txs_acc_5v0_hdmi_switch_changed(); /* 1st check */

	return 0;

error_exit:
	txs_acc_det_unregister_callback( &txs_acc_5v0_hdmi_switch_changed );

	regulator_put(txs_en_5v0_hdmi);
	txs_en_5v0_hdmi = NULL;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&txs_acc_5v0_hdmi_early_suspender);
#endif
	return ret;
}

static int txs_acc_5v0_hdmi_remove(struct platform_device* pdev)
{
	txs_acc_det_unregister_callback( &txs_acc_5v0_hdmi_switch_changed );

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&txs_acc_5v0_hdmi_early_suspender);
#endif
	return 0;
}

static struct platform_driver txs_acc_5v0_hdmi_driver = {
	.probe   = txs_acc_5v0_hdmi_probe,
	.remove  = txs_acc_5v0_hdmi_remove,
	.driver  = {
		.name = "txs_acc_5v0_hdmi",
		.owner = THIS_MODULE,
	},
};

static int __init txs_acc_5v0_hdmi_init(void)
{
	int ret;

	ret = platform_driver_register(&txs_acc_5v0_hdmi_driver);
	if (ret < 0) {
		pr_err("%s:platform_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit txs_acc_5v0_hdmi_exit(void)
{
	platform_driver_unregister(&txs_acc_5v0_hdmi_driver);
}

module_init(txs_acc_5v0_hdmi_init);
module_exit(txs_acc_5v0_hdmi_exit);

MODULE_LICENSE("GPL");

