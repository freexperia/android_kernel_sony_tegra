/* 
 * SAR BACKOFF POWER REDUCTION CONTROLLER
 * Author: Damon Hsieh (damon.ch.hsieh@foxconn.com)
 * Company: Foxconn
 * Date: 2012-03-02
 * Description: 
 * 
 * Note: This file is created by referring to MOR's sample code
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/percpu.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/input/sx863x.h>
#include <linux/suspend.h>
#include <linux/input/sx863x.h>

#define PROX		0
#define NO_PROX	1

static wait_queue_head_t sar_state_wait;
static wait_queue_head_t sar_ack_wait;

int sar_continue_ok;
int suspended = 0;

struct kobject *sar_kobj;

const char* sys_sar_notifier;

static const char *STRING_SAR_CONTROL_ON   = "SAR_CONTROL_ON";  /* SAR Back-off control ON */
static const char *STRING_SAR_CONTROL_OFF  = "SAR_CONTROL_OFF"; /* SAR Back-off control OFF */
static const char *STRING_SAR_CONTINUE     = "SAR_CONTINUE";    /* Ack from ril */
static const char *STRING_SAR_CONTROL_SUSPEND = "SAR_CONTROL_SUSPEND";

static void sar_notify_ril(const char* notice);

static ssize_t
sar_state_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int nchar;

	/* Block if the value is not available yet. */
	if (! sys_sar_notifier)
	{
		pr_debug("%s: blocking\n", __func__);
		if( wait_event_interruptible(sar_state_wait, sys_sar_notifier))
		{
			return -ERESTARTSYS;
		}
	}
	pr_debug("sys_sar_notifier = %s\n", sys_sar_notifier);

	/* In case of false wakeup, return "". */
	if (! sys_sar_notifier)
	{
		pr_debug("%s: false wakeup, returning with '\\n'\n", __func__);
		nchar = sprintf(buf, "\n");
		return nchar;
	}

	/* Return the value, and clear. */
	pr_debug("%s: returning with '%s'\n", __func__, sys_sar_notifier);
	nchar = sprintf(buf, "%s\n", sys_sar_notifier);
	sys_sar_notifier = NULL;

	return nchar;
}

static ssize_t
sar_state_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	if (!strncmp(buf, STRING_SAR_CONTINUE, strlen(STRING_SAR_CONTINUE)))
	{
		/* Wake up notifier. */
		pr_debug("%s: SAR_CONTINUE\n", "sar_notifier_store");
		sar_continue_ok = 1;
		wake_up(&sar_ack_wait);
	}
	else
	{
		pr_err("%s: wrong value '%s'\n", __func__, buf);
	}

	return count;
}

static struct kobj_attribute sar_notifier_attribute =  
	__ATTR(state, 0400, sar_state_show, sar_state_store);

const struct attribute *sar_attribute[] = {
	&sar_notifier_attribute.attr,
	NULL,
};

static void sar_notify_ril(const char* notice)
{
	sar_continue_ok = 0;

	/* Notify ril */
	sys_sar_notifier = notice;
	pr_debug("%s: sys_sar_notifier = %s\n", __func__, sys_sar_notifier);
	wake_up(&sar_state_wait);
#if 0
	/* Wait for the reply from ril */
	pr_debug("%s: wait for ril\n", __func__);
	if (wait_event_timeout(sar_ack_wait, sar_continue_ok, timeout) == 0)
	{
		pr_err("%s: timed out. ril did not reply\n", __func__);
	}
#endif
	/* Go back to the initial state. */
	//sys_sar_notifier = NULL;
}

void sensor_status_changed()
{
	/* Notify the event to RIL */

	pr_debug("%s called. suspended = %d\n", __func__, suspended);

	if (proximity_status == NO_PROX) {
		pr_debug("%s: STRING_SAR_CONTROL_OFF\n", __func__);
		if (!suspended)
			sar_notify_ril(STRING_SAR_CONTROL_OFF);
	} else {
		pr_debug("%s: STRING_SAR_CONTROL_ON\n", __func__);
		if (!suspended)
			sar_notify_ril(STRING_SAR_CONTROL_ON);
	}

	return;
}

#if defined (CONFIG_PM)
int sar_pm_notifier(struct notifier_block *nb, unsigned long event, void *nouse)
{
	int timeout = 10 * HZ;
	switch (event) {
		case PM_SUSPEND_PREPARE:
			pr_debug("%s: PM_SUSPEND_PREPARE\n", __func__);
			sar_notify_ril(STRING_SAR_CONTROL_SUSPEND);

			// wait for the SAR_CONTINUE from vendor RIL
			pr_debug("%s: wait for ril\n", __func__);
			if (wait_event_timeout(sar_ack_wait, sar_continue_ok, timeout) == 0)
			{
				pr_err("%s: timed out. ril did not reply\n", __func__);
			}
			pr_debug("%s: receive SAR_CONTINUE\n", __func__);
			suspended = 1;
			break;
		case PM_POST_SUSPEND:
			suspended = 0;
			pr_debug("%s: PM_POST_SUSPEND\n", __func__);
			sensor_status_changed();
			break;
		default:
			pr_debug("unknown event %ld\n", event);
			return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}
#endif

static int __init ril_pm_init(void)
{
	int ret = 0;

	suspended = 0;

#if defined (CONFIG_PM)
	/* Register PM notifier (callback) */
	pm_notifier(sar_pm_notifier, 0);
#endif

	return ret;
}

static int __init sar_init(void)
{
	int ret = 0;
	
	printk(KERN_INFO "%s called\n", __func__);

	/* Create /sys/power/sar/ type */
	sar_kobj = kobject_create_and_add("sar", power_kobj);

	ret = sysfs_create_files(sar_kobj, sar_attribute);
	if (ret)
	{
		printk(KERN_ERR "%s: entry with the given name already exist\n", __func__);
		return ret;
	}

	ril_pm_init();

	init_waitqueue_head(&sar_state_wait);
	init_waitqueue_head(&sar_ack_wait);

	return ret;
}

static void __exit sar_deinit(void)
{
	printk(KERN_INFO "%s called\n", __func__);
}

module_init(sar_init);
module_exit(sar_deinit);

