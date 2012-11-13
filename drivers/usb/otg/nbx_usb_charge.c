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
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/err.h>

#include <linux/nbx_ec_ipc.h>

static int nbx_usb_charge_mA = -1;
static int nbx_usb_charge_block = 1;
static DEFINE_SPINLOCK(nbx_usb_charge_block_lock);
static int nbx_usb_charge_deferred_mA = -1;

static struct workqueue_struct* nbx_usb_charge_workqueue = NULL;
static struct work_struct nbx_usb_charge_work;

#define EC_IPC_CID_USBSDP_CHANGE 0x22

static void nbx_usb_charge_worker(struct work_struct* work)
{
	ssize_t ret;
	unsigned long flags;

	enum {
		EC_USBSDP_CURRENT_L = 0,
		EC_USBSDP_CURRENT_H,
		NOOF_EC_USBSDP_CHANGE_REQ,
	};
	uint8_t req_buf[NOOF_EC_USBSDP_CHANGE_REQ];

	enum {
		EC_USBSDP_CHANGE_RESULT = 0,
		NOOF_EC_USBSDP_CHANGE_RES,
	};
	uint8_t res_buf[NOOF_EC_USBSDP_CHANGE_RES];
#define EC_USBSDP_CHANGE_RESULT_OK 0
#define EC_USBSDP_CHANGE_RESULT_NG 1

	spin_lock_irqsave(&nbx_usb_charge_block_lock, flags);
	{
		req_buf[EC_USBSDP_CURRENT_L] = nbx_usb_charge_mA & 0xFF;
		req_buf[EC_USBSDP_CURRENT_H] = (nbx_usb_charge_mA >> 8) & 0xFF;
	}
	spin_unlock_irqrestore(&nbx_usb_charge_block_lock, flags);

	ret = ec_ipc_send_request(EC_IPC_PID_USBSDP, EC_IPC_CID_USBSDP_CHANGE,
				req_buf, sizeof(req_buf),
				res_buf, sizeof(res_buf) );
	if(ret < (int)sizeof(res_buf)){
		pr_err("nbx_usb_charge: ec_ipc_send_request failed. %d\n", ret);
	}
	else if(res_buf[EC_USBSDP_CHANGE_RESULT] != EC_USBSDP_CHANGE_RESULT_OK) {
		pr_err("nbx_usb_charge: set USB SDP failed.\n");
	}
}

static int nbx_usb_charge_current_changed(int mA)
{
	unsigned long flags;

	if(nbx_usb_charge_workqueue == NULL) return -1;

	spin_lock_irqsave(&nbx_usb_charge_block_lock, flags);
	{
		if(!nbx_usb_charge_block) {
			nbx_usb_charge_mA = mA;
			queue_work(nbx_usb_charge_workqueue, &nbx_usb_charge_work);
		}
		else {
			nbx_usb_charge_deferred_mA = mA;
		}
	}
	spin_unlock_irqrestore(&nbx_usb_charge_block_lock, flags);

	return 0;
}

#ifdef CONFIG_SUSPEND

static int nbx_usb_charge_suspend(struct nbx_ec_ipc_device* edev)
{
	unsigned long flags;

	if(nbx_usb_charge_workqueue == NULL) return 0;

	spin_lock_irqsave(&nbx_usb_charge_block_lock, flags);
	{
		nbx_usb_charge_block = 1;
		nbx_usb_charge_mA = 0;
		queue_work(nbx_usb_charge_workqueue, &nbx_usb_charge_work);
	}
	spin_unlock_irqrestore(&nbx_usb_charge_block_lock, flags);

	flush_work_sync(&nbx_usb_charge_work);

	return 0;
}
static int nbx_usb_charge_resume_noirq(struct nbx_ec_ipc_device* edev)
{
	nbx_usb_charge_deferred_mA = -1; /* not request yet. */

	return 0;
}
static int nbx_usb_charge_resume(struct nbx_ec_ipc_device* edev)
{
	unsigned long flags;

	if(nbx_usb_charge_workqueue == NULL) return 0;

	spin_lock_irqsave(&nbx_usb_charge_block_lock, flags);
	{
		if(0 <= nbx_usb_charge_deferred_mA) {
			nbx_usb_charge_mA = nbx_usb_charge_deferred_mA;
			queue_work(nbx_usb_charge_workqueue, &nbx_usb_charge_work);
		}
		nbx_usb_charge_block = 0;
	}
	spin_unlock_irqrestore(&nbx_usb_charge_block_lock, flags);

	return 0;
}

#else /* !CONFIG_SUSPEND */
#define nbx_usb_charge_suspend NULL
#define nbx_usb_charge_resume NULL
#endif /* CONFIG_SUSPEND */

extern void register_usb_charge_current_changed(int (*changed)(int));
extern void unregister_usb_charge_current_changed(int (*changed)(int));

static int nbx_usb_charge_probe(struct nbx_ec_ipc_device* edev)
{
	nbx_usb_charge_workqueue = create_singlethread_workqueue("nbx_usb_charge_workqueue");
	if(nbx_usb_charge_workqueue == NULL) {
		pr_err("nbx_usb_charge:create_singlethread_workqueue() failed.\n");
		return -ENOMEM;
	}

	INIT_WORK(&nbx_usb_charge_work, nbx_usb_charge_worker);

	nbx_usb_charge_block = 0;
	register_usb_charge_current_changed(nbx_usb_charge_current_changed);

	return 0;
}

static int nbx_usb_charge_remove(struct nbx_ec_ipc_device* edev)
{
	unregister_usb_charge_current_changed(nbx_usb_charge_current_changed);
	nbx_usb_charge_block = 1;

	if(nbx_usb_charge_workqueue != NULL) {
		cancel_work_sync(&nbx_usb_charge_work);
		flush_workqueue(nbx_usb_charge_workqueue);
		destroy_workqueue(nbx_usb_charge_workqueue);
	}
	nbx_usb_charge_workqueue = NULL;

	return 0;
}

static void nbx_usb_charge_shutdown(struct nbx_ec_ipc_device* edev)
{
	unsigned long flags;

	if(nbx_usb_charge_workqueue == NULL) return;

	spin_lock_irqsave(&nbx_usb_charge_block_lock, flags);
	{
		nbx_usb_charge_block = 1;
		nbx_usb_charge_mA = 0;
		queue_work(nbx_usb_charge_workqueue, &nbx_usb_charge_work);
	}
	spin_unlock_irqrestore(&nbx_usb_charge_block_lock, flags);

	flush_work_sync(&nbx_usb_charge_work);

	return;
}

static struct nbx_ec_ipc_driver nbx_usb_charge_driver = {
	.probe        = nbx_usb_charge_probe,
	.remove       = nbx_usb_charge_remove,
	.shutdown     = nbx_usb_charge_shutdown,
	.suspend      = nbx_usb_charge_suspend,
	.resume_noirq = nbx_usb_charge_resume_noirq,
	.resume       = nbx_usb_charge_resume,
	.drv = {
		.name = "nbx_usb_charge",
	},
};

static int __init nbx_usb_charge_init(void)
{
	int ret;

	ret = nbx_ec_ipc_driver_register(&nbx_usb_charge_driver);
	if (ret < 0) {
		pr_err("%s:nbx_ec_ipc_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit nbx_usb_charge_exit(void)
{
	nbx_ec_ipc_driver_unregister(&nbx_usb_charge_driver);
}

module_init(nbx_usb_charge_init);
module_exit(nbx_usb_charge_exit);

MODULE_LICENSE("GPL");
