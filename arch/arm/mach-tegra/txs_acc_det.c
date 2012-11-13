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
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/semaphore.h>
#include <linux/wakelock.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/nbx_ec_ipc.h>
#include "txs_acc_det.h"

#define EC_IPC_CID_ACCAD_REQUEST 0x53

#define LOCKED_STATEMENT(mutex, statement)	\
	down(mutex);				\
	do {					\
		statement;			\
	} while(0);				\
	up(mutex);

static LIST_HEAD(callback_list);
static DEFINE_SEMAPHORE(callback_list_mutex);

struct callback_t {
	struct list_head list;
	void (*func)(void);
};

struct txs_acc_det_range_t {
	uint16_t bottom;
	uint16_t top;
};
static struct txs_acc_det_range_t txs_acc_det_table[] = {
	{ 0x00, 0x0E, },
	{ 0x0F, 0x21, }, /* charging cradle */
	{ 0x22, 0x34, },
	{ 0x35, 0x46, }, /* docking stand */
	{ 0x47, 0x5C, },
	{ 0x5D, 0x73, }, /* docking speaker */
	{ 0x74, 0x89, },
	{ 0x8A, 0xA0, },
	{ 0xA1, 0xBD, },
	{ 0xBE, 0xD9, }, /* reserved */
	{ 0xDA, 0xEE, }, /* HDMI dongle */
	{ 0xEF, 0xFF, }, /* not connected */
};
static int txs_acc_det_table_num = sizeof(txs_acc_det_table) / sizeof(txs_acc_det_table[0]);

static int txs_acc_det_state_last = -1; /* unknown */

static struct workqueue_struct* txs_acc_det_workqueue;
static struct work_struct txs_acc_det_work;

static struct wake_lock txs_acc_det_wake_lock;

static int txs_acc_det_pin_num = -1;
static const int txs_acc_det_pin_active = 0;

static atomic_t txs_acc_det_suspended;

static int txs_acc_ad_read(uint16_t* result)
{
	ssize_t ret;

	enum {
		ACCAD_RES_AD_LOW = 0,
		ACCAD_RES_AD_HIGH,
		NOOF_ACCAD_RES,
	};
	uint8_t res_buf[NOOF_ACCAD_RES];

	ret = ec_ipc_send_request(EC_IPC_PID_ACCAD, EC_IPC_CID_ACCAD_REQUEST,
				NULL, 0,
				res_buf, sizeof(res_buf) );
	if(ret < (int)sizeof(res_buf)){
		pr_err("txs_acc_det:ec_ipc_send_request failed. %d\n", ret);
		return -ENXIO;
	}

	*result = (((uint16_t)res_buf[ACCAD_RES_AD_HIGH]) << 8) + res_buf[ACCAD_RES_AD_LOW];

	return 0;
}

static void txs_acc_det_worker(struct work_struct* work)
{
	uint16_t acc_ad;
	uint16_t acc_ad_last;
	int read_count;
	int acc_det_state;
	int limit;
	int pin_state;

	do {
		acc_ad_last = acc_ad = 0xFFFF;
		read_count = 0;

		/* delay 100ms when activate */
		pin_state = gpio_get_value(txs_acc_det_pin_num);
		if(pin_state == txs_acc_det_pin_active) {
			msleep(100);
		}

		for(limit = 100; 0 < limit; limit--) {
			pin_state = gpio_get_value(txs_acc_det_pin_num);
			if(pin_state != txs_acc_det_pin_active) {
				acc_ad = 0xFFFF; /* emulate AD max when pin state inactive */
				break;
			}

			if( 0 != txs_acc_ad_read(&acc_ad) ){
				continue;
			}
			acc_ad >>= (10 - 8); /* use upper 8 bits */

			if(acc_ad != acc_ad_last) {
				read_count = 0;
			}
			acc_ad_last = acc_ad;

			read_count++;
			if(3 <= read_count) {
				break; /* same 3 cycles */
			}
		}
		if(0 < limit) {
			/* find number */
			for(acc_det_state = 0; acc_det_state < txs_acc_det_table_num; acc_det_state++) {
				if((txs_acc_det_table[acc_det_state].bottom <= acc_ad) &&
					(acc_ad <= txs_acc_det_table[acc_det_state].top) ){
					break;
				}
			}
		}
		else {
			/* not stable */
			acc_det_state = txs_acc_det_table_num;
		}

		/* out of range */
		if(txs_acc_det_table_num <= acc_det_state) {
			acc_det_state = txs_acc_det_table_num - 1;
		}

		if(txs_acc_det_state_last != acc_det_state) {
			struct callback_t* callback;

			txs_acc_det_state_last = acc_det_state;
#ifdef ACC_DET_DEBUG_PRINT
			if( (txs_acc_det_state_last < 0) || ((txs_acc_det_table_num -1) <= txs_acc_det_state_last) ){
				pr_info("txs_acc_det:state=NOT_CONNECTED\n");
			}else{
				pr_info("txs_acc_det:state=%d\n", txs_acc_det_state_last );
			}
#endif
			LOCKED_STATEMENT(&callback_list_mutex,
					list_for_each_entry(callback, &callback_list, list) {
						(callback->func)();
					}
				);
		}

	} while(pin_state != gpio_get_value(txs_acc_det_pin_num));

	/* wake_lock until timeout when DockSp connected */
	if(txs_acc_det_state_last != TXS_ACC_DOCK_SPEAKER) {
		wake_unlock(&txs_acc_det_wake_lock);
	}
}

int txs_acc_det_get_num(void)
{
	int ret = txs_acc_det_state_last;

	if( (ret < 0) || ((txs_acc_det_table_num - 1) <= ret) ){
		return TXS_ACC_NOT_CONNECTED;
	}
	else {
		return ret;
	}
}
EXPORT_SYMBOL(txs_acc_det_get_num);

int txs_acc_det_register_callback( void (*func)(void) )
{
	struct callback_t* callback;

	callback = kmalloc(sizeof(struct callback_t), GFP_KERNEL);
	if(callback == NULL) {
		return -ENOMEM;
	}
	memset(callback, 0, sizeof(struct callback_t));

	callback->func = func;

	LOCKED_STATEMENT(&callback_list_mutex,
			INIT_LIST_HEAD(&(callback->list));
			list_add_tail(&(callback->list), &callback_list);
		);

	return 0;
}
EXPORT_SYMBOL(txs_acc_det_register_callback);

void txs_acc_det_unregister_callback( void (*func)(void) )
{
	struct callback_t* del_callback;
	struct callback_t* n_callback;

	LOCKED_STATEMENT(&callback_list_mutex,
			list_for_each_entry_safe(del_callback, n_callback, &callback_list, list) {
				if(del_callback->func == func) {
					list_del(&(del_callback->list));
					kfree(del_callback);
				}
			}
		);
}
EXPORT_SYMBOL(txs_acc_det_unregister_callback);

/*
 *----------------------------------------------------------------
 * switch class driver
 *----------------------------------------------------------------
 */
static struct switch_dev txs_acc_det_switch_dev;

static ssize_t txs_acc_det_switch_print_state(struct switch_dev *sdev, char *buf)
{
	int det_num = txs_acc_det_get_num();
	if(TXS_ACC_NOT_CONNECTED != det_num) {
		return snprintf(buf, PAGE_SIZE, "%d", det_num+1); /* start No.1 */
	}
	else {
		return snprintf(buf, PAGE_SIZE, "NOT_CONNECTED");
	}
}

static void txs_acc_det_switch_changed(void)
{
	switch_set_state(&txs_acc_det_switch_dev, txs_acc_det_get_num());
}

static irqreturn_t txs_acc_det_interrupt(int irq, void *arg)
{
	if( (txs_acc_det_workqueue != NULL) && (0 == atomic_read(&txs_acc_det_suspended)) ){
		wake_lock_timeout(&txs_acc_det_wake_lock, 10 * HZ);
		queue_work(txs_acc_det_workqueue, &txs_acc_det_work);
	}

	return IRQ_HANDLED;
}


#ifdef CONFIG_SUSPEND

static int txs_acc_det_suspend(struct nbx_ec_ipc_device* edev)
{
	atomic_set(&txs_acc_det_suspended, 1);

	cancel_work_sync(&txs_acc_det_work);
	txs_acc_det_state_last = -1; /* unknown */

	return 0;
}
static int txs_acc_det_resume(struct nbx_ec_ipc_device* edev)
{
	if(txs_acc_det_workqueue != NULL) {
		wake_lock_timeout(&txs_acc_det_wake_lock, 10 * HZ);
		queue_work(txs_acc_det_workqueue, &txs_acc_det_work);
	}

	atomic_set(&txs_acc_det_suspended, 0);

	return 0;
}

#else /* !CONFIG_SUSPEND */
#define txs_acc_det_suspend NULL
#define txs_acc_det_resume NULL
#endif /* CONFIG_SUSPEND */

static int txs_acc_det_probe(struct nbx_ec_ipc_device* edev)
{
	int ret = 0;
	int irq_num;
	struct txs_acc_det_platform_data* acc_det_pdata =
		(struct txs_acc_det_platform_data*)(edev->dev.platform_data);

	/* init workqueue */
	txs_acc_det_workqueue = create_singlethread_workqueue("txs_acc_det_workqueue");
	if(txs_acc_det_workqueue == NULL) {
		pr_err("txs_acc_det:create_singlethread_workqueue() failed.\n");
		ret = -ENOMEM;
		goto error_exit;
	}

	INIT_WORK(&txs_acc_det_work, txs_acc_det_worker);
	atomic_set(&txs_acc_det_suspended, 0);

	/* init switch class */
	txs_acc_det_switch_dev.name = "txs_acc_det";
	txs_acc_det_switch_dev.print_name = NULL;
	txs_acc_det_switch_dev.print_state = &txs_acc_det_switch_print_state;
	ret = switch_dev_register( &txs_acc_det_switch_dev );
	if(ret < 0) {
		pr_err("txs_acc_det:switch_dev_register() failed. %d\n", ret);
		goto error_exit;
	}

	/* init GPIO ACC DET pin */
	txs_acc_det_pin_num = acc_det_pdata->det_pin_num;
	tegra_gpio_enable(txs_acc_det_pin_num);
	ret = gpio_request(txs_acc_det_pin_num, "TXS Accessory detect");
	if(ret < 0) {
		pr_err("txs_acc_det:gpio_request() failed. %d\n", ret);
		goto error_exit;
	}
	gpio_direction_input(txs_acc_det_pin_num);
	irq_num = gpio_to_irq(txs_acc_det_pin_num);
	if(irq_num < 0) {
		pr_err("txs_acc_det:gpio_to_irq() failed. %d\n", irq_num);
		goto error_exit;
	}
	ret = request_irq(irq_num, txs_acc_det_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"txs_acc_det", NULL);
	if(ret < 0) {
		pr_err("txs_acc_det:request_irq() failed. %d\n", ret);
		goto error_exit;
	}
	irq_set_irq_wake(irq_num, 1);

	wake_lock_init(&txs_acc_det_wake_lock, WAKE_LOCK_SUSPEND, "txs_acc_det");

	txs_acc_det_register_callback( &txs_acc_det_switch_changed );

	txs_acc_det_state_last = -1; /* unknown */
	queue_work(txs_acc_det_workqueue, &txs_acc_det_work); /* 1st check */

	return 0;

error_exit:
	txs_acc_det_unregister_callback( &txs_acc_det_switch_changed );

	if(txs_acc_det_workqueue != NULL) {
		destroy_workqueue(txs_acc_det_workqueue);
	}

	txs_acc_det_workqueue = NULL;

	gpio_free(txs_acc_det_pin_num);

	switch_dev_unregister( &txs_acc_det_switch_dev );

	return ret;
}

static int txs_acc_det_remove(struct nbx_ec_ipc_device* edev)
{
	struct callback_t* del_callback;
	struct callback_t* n_callback;

	txs_acc_det_unregister_callback( &txs_acc_det_switch_changed );

	LOCKED_STATEMENT(&callback_list_mutex,
			list_for_each_entry_safe(del_callback, n_callback, &callback_list, list) {
				list_del(&(del_callback->list));
				kfree(del_callback);
			}
		);

	if(txs_acc_det_workqueue != NULL) {
		cancel_work_sync(&txs_acc_det_work);
		flush_workqueue(txs_acc_det_workqueue);
		destroy_workqueue(txs_acc_det_workqueue);
	}

	txs_acc_det_workqueue = NULL;

	free_irq( gpio_to_irq(txs_acc_det_pin_num), NULL);
	gpio_free(txs_acc_det_pin_num);

	switch_dev_unregister( &txs_acc_det_switch_dev );

	wake_lock_destroy(&txs_acc_det_wake_lock);

	return 0;
}

static struct nbx_ec_ipc_driver txs_acc_det_driver = {
	.probe   = txs_acc_det_probe,
	.remove  = txs_acc_det_remove,
	.suspend = txs_acc_det_suspend,
	.resume  = txs_acc_det_resume,
	.drv = {
		.name = "txs_acc_det",
	},
};

static int __init txs_acc_det_init(void)
{
	int ret;

	txs_acc_det_workqueue = NULL;

	ret = nbx_ec_ipc_driver_register(&txs_acc_det_driver);
	if (ret < 0) {
		pr_err("%s:nbx_ec_ipc_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit txs_acc_det_exit(void)
{
	nbx_ec_ipc_driver_unregister(&txs_acc_det_driver);
}

module_init(txs_acc_det_init);
module_exit(txs_acc_det_exit);

MODULE_LICENSE("GPL");

