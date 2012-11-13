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
#include <linux/nbx_ec_ipc_acplug.h>
#include <linux/mutex.h>

#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
static struct wake_lock dockspeaker_lock;
#endif

#include "txs_acc_det.h"

#define DRIVER_NAME "txs_acc_dockspeaker"

static struct switch_dev txs_acc_dockspeaker_switch_dev;

static DEFINE_MUTEX(cb_lock);
static int wakelock_flag = 0;

static void txs_acc_dockspeaker_acplug_changed(void)
{
	int det_num = -1;
	int acplug_state = -1;

	mutex_lock(&cb_lock);

	det_num = txs_acc_det_get_num();
	acplug_state = nbx_ec_ipc_acplug_get_state();

#ifdef ACC_DET_DEBUG_PRINT
	if( det_num==TXS_ACC_DOCK_SPEAKER )
		pr_info("txs_acc_dockspeaker:state=DockSP, acplug=%d\n", acplug_state );
#endif

	if( det_num==TXS_ACC_DOCK_SPEAKER && acplug_state > 0 ){
		/* wake lock */
		if( wakelock_flag==0 ){
#if defined(CONFIG_HAS_WAKELOCK)
			wake_lock(&dockspeaker_lock);
#ifdef ACC_DET_DEBUG_PRINT
			pr_info("wake_lock:dockspeaker connect\n");
#endif
#endif
			wakelock_flag = 1;
		}
	}else{
		if( wakelock_flag==1 ){
#if defined(CONFIG_HAS_WAKELOCK)
			wake_unlock(&dockspeaker_lock);
#ifdef ACC_DET_DEBUG_PRINT
			pr_info("wake_unlock:dockpeaker disconnect\n");
#endif
#endif
			wakelock_flag = 0;
		}
	}

	mutex_unlock(&cb_lock);
}

static int txs_acc_dockspeaker_probe(struct platform_device* pdev)
{
	int ret = 0;

	/* init switch class */
	txs_acc_dockspeaker_switch_dev.name = "dockspeaker";
	txs_acc_dockspeaker_switch_dev.print_name = NULL;
	txs_acc_dockspeaker_switch_dev.print_state = NULL;
	ret = switch_dev_register( &txs_acc_dockspeaker_switch_dev );
	if(ret < 0) {
		pr_err("txs_acc_dockspeaker:switch_dev_register() failed. %d\n", ret);
		goto error_exit;
	}

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_init(&dockspeaker_lock, WAKE_LOCK_SUSPEND, "dockspeakerlock");
#endif

	ret = txs_acc_det_register_callback( &txs_acc_dockspeaker_acplug_changed );
	if(ret < 0) {
		pr_err("txs_acc_dockspeaker:txs_acc_det_register_callback() failed. %d\n", ret);
		goto err_unregister_switch;
	}

	ret = nbx_ec_ipc_acplug_register_callback( &txs_acc_dockspeaker_acplug_changed );
	if(ret < 0){
		pr_err("txs_acc_dockspeaker:nbx_ec_ipc_acplug_register_callback() failed. %d\n", ret);
		goto err_unregister_acc_det_callback;
	}
	txs_acc_dockspeaker_acplug_changed(); /* 1st check */

	return 0;

err_unregister_acc_det_callback:
	txs_acc_det_unregister_callback( &txs_acc_dockspeaker_acplug_changed );
err_unregister_switch:
	switch_dev_unregister(&txs_acc_dockspeaker_switch_dev);

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_destroy(&dockspeaker_lock);
#endif
error_exit:
	return ret;
}

static int txs_acc_dockspeaker_remove(struct platform_device* pdev)
{
	nbx_ec_ipc_acplug_unregister_callback( &txs_acc_dockspeaker_acplug_changed );

	txs_acc_det_unregister_callback( &txs_acc_dockspeaker_acplug_changed );

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_destroy(&dockspeaker_lock);
#endif
	switch_dev_unregister( &txs_acc_dockspeaker_switch_dev );

	return 0;
}

static struct platform_driver txs_acc_dockspeaker_driver = {
	.probe   = txs_acc_dockspeaker_probe,
	.remove  = txs_acc_dockspeaker_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init txs_acc_dockspeaker_init(void)
{
	int ret;

	ret = platform_driver_register(&txs_acc_dockspeaker_driver);
	if (ret < 0) {
		pr_err("%s:platform_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit txs_acc_dockspeaker_exit(void)
{
	platform_driver_unregister(&txs_acc_dockspeaker_driver);
}

module_init(txs_acc_dockspeaker_init);
module_exit(txs_acc_dockspeaker_exit);

MODULE_LICENSE("GPL");

