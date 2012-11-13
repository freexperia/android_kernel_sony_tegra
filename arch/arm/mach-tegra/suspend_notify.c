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
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

struct suspend_notifier {
	struct notifier_block pm_notifier;
	struct platform_device* pdev;
	uint32_t suspended_count;
	struct wake_lock wake_lock;
	char wake_lock_name[20];
};


static int suspend_notifier_func(struct notifier_block *nb,
				unsigned long event, void *nouse)
{
	struct suspend_notifier* notif = container_of(nb, struct suspend_notifier, pm_notifier);
	struct platform_device* pdev = notif->pdev;

	switch (event) {
	case PM_POST_SUSPEND:
		(notif->suspended_count)++;
		kobject_uevent(&(pdev->dev.kobj), KOBJ_CHANGE);
		wake_lock_timeout(&(notif->wake_lock), 2 * HZ);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static ssize_t suspend_notify_suspended_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct suspend_notifier* notif = dev_get_platdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d", notif->suspended_count);
}

static struct device_attribute suspend_notify_suspended_count_attr = {
	.attr = { .name = "suspended_count", .mode = S_IRUGO },
	.show = suspend_notify_suspended_count_show,
	.store = NULL,
};

static int suspend_notify_probe(struct platform_device* pdev)
{
	int ret;
	struct suspend_notifier _notif;
	struct suspend_notifier* notif;
	memset(&_notif, 0, sizeof(struct suspend_notifier));

	ret = device_create_file(&(pdev->dev), &suspend_notify_suspended_count_attr);
	if(ret < 0) {
		pr_err("%s:device_create_file() failed, %d\n", __func__, ret);
		goto error_exit;
	}

	ret = platform_device_add_data(pdev, &_notif, sizeof(struct suspend_notifier));
	if(ret < 0) {
		pr_err("%s:platform_device_add_data() failed, %d\n", __func__, ret);
		goto error_exit;
	}

	notif = dev_get_platdata(&(pdev->dev));
	notif->pdev = pdev;
	notif->suspended_count = 0;
	notif->pm_notifier.notifier_call = suspend_notifier_func;

	ret = register_pm_notifier(&(notif->pm_notifier));
	if(ret < 0) {
		pr_err("%s:register_pm_notifier() failed, %d\n", __func__, ret);
		goto error_exit;
	}

	if(0 <= pdev->id) {
		snprintf(notif->wake_lock_name, sizeof(notif->wake_lock_name), "suspend_notify%d", pdev->id);
	}
	else {
		snprintf(notif->wake_lock_name, sizeof(notif->wake_lock_name), "suspend_notify");
	}
	notif->wake_lock_name[sizeof(notif->wake_lock_name)-1] = '\0';
	wake_lock_init(&(notif->wake_lock), WAKE_LOCK_SUSPEND, notif->wake_lock_name);

error_exit:

	return ret;
}

static int suspend_notify_remove(struct platform_device* pdev)
{
	struct suspend_notifier* notif = dev_get_platdata(&(pdev->dev));

	wake_lock_destroy(&(notif->wake_lock));
	unregister_pm_notifier(&(notif->pm_notifier));
	device_remove_file(&(pdev->dev), &suspend_notify_suspended_count_attr);

	return 0;
}

static struct platform_driver suspend_notify_driver = {
	.probe   = suspend_notify_probe,
	.remove  = suspend_notify_remove,
	.driver  = {
		.name  = "suspend_notify",
		.owner = THIS_MODULE,
	},
};

static int __init suspend_notify_init(void)
{
	int ret;

	ret = platform_driver_register(&suspend_notify_driver);
	if (ret < 0) {
		pr_err("%s:platform_driver_register() failed, %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void __exit suspend_notify_deinit(void)
{
	platform_driver_unregister(&suspend_notify_driver);
}

module_init(suspend_notify_init);
module_exit(suspend_notify_deinit);

MODULE_LICENSE("GPL");

