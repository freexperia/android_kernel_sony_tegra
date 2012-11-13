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

#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>

#include "txs_acc_det.h"

#define DRIVER_NAME "txs_acc_dock"

static struct switch_dev txs_acc_dock_switch_dev;

static void txs_acc_dock_switch_changed(void)
{
	int det_num;
	det_num = txs_acc_det_get_num();

	/* Charging cradle or Docking stand */
	if( TXS_ACC_CHARGING_CRADLE==det_num || TXS_ACC_DOCKING_STAND==det_num ){
		switch_set_state(&txs_acc_dock_switch_dev, 1 );
	}else{
		switch_set_state(&txs_acc_dock_switch_dev, 0 );
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void txs_acc_dock_early_suspend(struct early_suspend *h)
{
	return;
}
static void txs_acc_dock_late_resume(struct early_suspend *h)
{
	return;
}
struct early_suspend txs_acc_dock_early_suspender = {
	.suspend = &txs_acc_dock_early_suspend,
	.resume  = &txs_acc_dock_late_resume,
	.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int txs_acc_dock_probe(struct platform_device* pdev)
{
	int ret = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&txs_acc_dock_early_suspender);
#endif

	/* init switch class */
	txs_acc_dock_switch_dev.name = "dock";
	txs_acc_dock_switch_dev.print_name = NULL;
	txs_acc_dock_switch_dev.print_state = NULL;
	ret = switch_dev_register( &txs_acc_dock_switch_dev );
	if(ret < 0) {
		pr_err("txs_acc_dock:switch_dev_register() failed. %d\n", ret);
		goto error_exit;
	}

	ret = txs_acc_det_register_callback( &txs_acc_dock_switch_changed );
	if(ret < 0) {
		pr_err("txs_acc_dock:txs_acc_det_register_callback() failed. %d\n", ret);
		goto err_unregister_switch;
	}
	txs_acc_dock_switch_changed(); /* 1st check */

	return 0;

err_unregister_switch:
	switch_dev_unregister(&txs_acc_dock_switch_dev);
error_exit:

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&txs_acc_dock_early_suspender);
#endif
	return ret;
}

static int txs_acc_dock_remove(struct platform_device* pdev)
{
	txs_acc_det_unregister_callback( &txs_acc_dock_switch_changed );

	switch_dev_unregister( &txs_acc_dock_switch_dev );

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&txs_acc_dock_early_suspender);
#endif
	return 0;
}

static struct platform_driver txs_acc_dock_driver = {
	.probe   = txs_acc_dock_probe,
	.remove  = txs_acc_dock_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init txs_acc_dock_init(void)
{
	int ret;

	ret = platform_driver_register(&txs_acc_dock_driver);
	if (ret < 0) {
		pr_err("%s:platform_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit txs_acc_dock_exit(void)
{
	platform_driver_unregister(&txs_acc_dock_driver);
}

module_init(txs_acc_dock_init);
module_exit(txs_acc_dock_exit);

MODULE_LICENSE("GPL");

