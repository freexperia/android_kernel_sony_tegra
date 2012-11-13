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
#include <linux/suspend.h>
#include <linux/wakelock.h>

#include <linux/nbx_ec_ipc.h>

#define EC_IPC_CID_CHECK_EVENT_REQUEST 0xB0
#define EC_IPC_CID_CHECK_EVENT_EVENT   0xB1

#define CHECK_EVENT_TIMEOUT_MS 3000

static struct workqueue_struct* ec_ipc_chk_event_workqueue;
static struct work_struct ec_ipc_chk_event_work;

static struct wake_lock ec_ipc_chk_event_wake_lock;

static DECLARE_WAIT_QUEUE_HEAD(ec_ipc_chk_event_wait);

static atomic_t ec_ipc_chk_event_received;

static void ec_ipc_chk_event(const uint8_t* buf, int size)
{
	if(buf == NULL) return;

	atomic_set(&ec_ipc_chk_event_received, 1);
	wake_up_interruptible(&ec_ipc_chk_event_wait);
}

static void ec_ipc_chk_event_worker(struct work_struct* work)
{
	ssize_t ret;
	long timeout;

	enum {
		CHK_EVENT_REQ_DELAY_MSEC_L = 0,
		CHK_EVENT_REQ_DELAY_MSEC_H,
		NOOF_CHK_EVENT_REQ,
	};
	uint8_t req_buf[NOOF_CHK_EVENT_REQ];

	enum {
		CHK_EVENT_RES_RESULT = 0,
		NOOF_CHK_EVENT_RES,
	};
	uint8_t res_buf[NOOF_CHK_EVENT_RES];

#define CHK_EVENT_RES_RESULT_OK 0
#define CHK_EVENT_RES_RESULT_NG 1

	req_buf[CHK_EVENT_REQ_DELAY_MSEC_L] = 0;
	req_buf[CHK_EVENT_REQ_DELAY_MSEC_H] = 0;

	atomic_set(&ec_ipc_chk_event_received, 0);

	ret = ec_ipc_send_request(0xFF, EC_IPC_CID_CHECK_EVENT_REQUEST,
				req_buf, sizeof(req_buf),
				res_buf, sizeof(res_buf) );
	if(ret < (int)sizeof(res_buf)) {
		pr_err("ec_ipc_chk_event:ec_ipc_send_request failed. %d\n", ret);
		goto err_exit;
	}
	if(res_buf[CHK_EVENT_RES_RESULT] != CHK_EVENT_RES_RESULT_OK ){
		pr_err("ec_ipc_chk_event:ec_ipc_send_request result NG.\n");
		goto err_exit;
	}

	timeout = wait_event_interruptible_timeout(ec_ipc_chk_event_wait,
						0 != atomic_read(&ec_ipc_chk_event_received),
						msecs_to_jiffies(CHECK_EVENT_TIMEOUT_MS));

	if( (0 == atomic_read(&ec_ipc_chk_event_received)) && (timeout <= 0) ){
		pr_err("ec_ipc_chk_event: wait event packet timeout.\n");
	}

err_exit:
	wake_unlock(&ec_ipc_chk_event_wake_lock);
}

#ifdef CONFIG_SUSPEND

static int ec_ipc_chk_event_suspend(struct nbx_ec_ipc_device* edev)
{
	atomic_set(&ec_ipc_chk_event_received, 1); /* suppress error message. */
	wake_up_interruptible(&ec_ipc_chk_event_wait);
	cancel_work_sync(&ec_ipc_chk_event_work);

	return 0;
}
static int ec_ipc_chk_event_resume(struct nbx_ec_ipc_device* edev)
{
	if(ec_ipc_chk_event_workqueue != NULL) {
		wake_lock_timeout(&ec_ipc_chk_event_wake_lock, 10 * HZ);
		queue_work(ec_ipc_chk_event_workqueue, &ec_ipc_chk_event_work);
	}

	return 0;
}

#else /* !CONFIG_SUSPEND */
#define ec_ipc_chk_event_suspend NULL
#define ec_ipc_chk_event_resume NULL
#endif /* CONFIG_SUSPEND */

static int ec_ipc_chk_event_probe(struct nbx_ec_ipc_device* edev)
{
	int ret = 0;

	ec_ipc_chk_event_workqueue = create_singlethread_workqueue("ec_ipc_chk_event_workqueue");
	if(ec_ipc_chk_event_workqueue == NULL) {
		pr_err("ec_ipc_chk_event:create_singlethread_workqueue() failed.\n");
		ret = -ENOMEM;
		goto error_exit;
	}

	INIT_WORK(&ec_ipc_chk_event_work, ec_ipc_chk_event_worker);

	wake_lock_init(&ec_ipc_chk_event_wake_lock, WAKE_LOCK_SUSPEND, "ec_ipc_chk_event");

	ec_ipc_register_recv_event(EC_IPC_CID_CHECK_EVENT_EVENT, ec_ipc_chk_event);
	queue_work(ec_ipc_chk_event_workqueue, &ec_ipc_chk_event_work); /* 1st check */

	return 0;

error_exit:
	if(ec_ipc_chk_event_workqueue != NULL) {
		destroy_workqueue(ec_ipc_chk_event_workqueue);
	}

	ec_ipc_chk_event_workqueue = NULL;

	return ret;
}

static int ec_ipc_chk_event_remove(struct nbx_ec_ipc_device* edev)
{
	if(ec_ipc_chk_event_workqueue != NULL) {
		cancel_work_sync(&ec_ipc_chk_event_work);
		flush_workqueue(ec_ipc_chk_event_workqueue);
		destroy_workqueue(ec_ipc_chk_event_workqueue);
	}

	ec_ipc_chk_event_workqueue = NULL;

	wake_lock_destroy(&ec_ipc_chk_event_wake_lock);

	return 0;
}

static struct nbx_ec_ipc_driver ec_ipc_chk_event_driver = {
	.probe   = ec_ipc_chk_event_probe,
	.remove  = ec_ipc_chk_event_remove,
	.suspend = ec_ipc_chk_event_suspend,
	.resume  = ec_ipc_chk_event_resume,
	.drv  = {
		.name = "ec_ipc_chk_event",
	},
};

static int __init ec_ipc_chk_event_init(void)
{
	int ret;

	ec_ipc_chk_event_workqueue = NULL;

	ret = nbx_ec_ipc_driver_register(&ec_ipc_chk_event_driver);
	if (ret < 0) {
		pr_err("%s:nbx_ec_ipc_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
static void  __exit ec_ipc_chk_event_exit(void)
{
	nbx_ec_ipc_driver_unregister(&ec_ipc_chk_event_driver);
}

module_init(ec_ipc_chk_event_init);
module_exit(ec_ipc_chk_event_exit);

MODULE_LICENSE("GPL");
