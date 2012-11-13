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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include "board-txs03.h"

//#define SYSFS_THROTTLING_DEBUG_PRINTS 1

#define THROTTLING_MODULE_NAME        "throttling"

#ifdef SYSFS_THROTTLING_DEBUG_PRINTS
#define PRINT_THROTTLING(x) pr_debug x
#else
#define PRINT_THROTTLING(x)
#endif

static ssize_t sysfsthrottling_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t sysfsthrottling_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count);


static struct kobj_attribute throttling_temp_attr =
	__ATTR(temp, 0400, sysfsthrottling_show, sysfsthrottling_store);

static ssize_t sysfsthrottling_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	long temp_throttle = 0;
	tegra_get_temp_throttle(&temp_throttle);
	PRINT_THROTTLING(("sysfsthrottling_show temp_throttle = %ld \n", temp_throttle));
	snprintf(buf,PAGE_SIZE,"%ld\n",temp_throttle);
	return (strlen(buf));
}

static ssize_t sysfsthrottling_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	long temp;

	if (buf==NULL || count == 0) {
		PRINT_THROTTLING(("Invalid Argument\n"));
		return -EINVAL;
	}
	if(strict_strtol(buf, 10, &temp)){
		return -EINVAL;
	}
	tegra_set_temp_throttle(temp);

	return count;
}

static struct attribute *throttling_attrs[] = {
	&throttling_temp_attr.attr,
	NULL
};

static const struct attribute_group throttling_sysfs_files = {
	.attrs	= throttling_attrs,
};

static int throttling_probe(struct platform_device *pdev)
{
	int err;
	PRINT_THROTTLING(("in throttling_probe\n"));
	err = sysfs_create_group(&pdev->dev.kobj, &throttling_sysfs_files);
	if (err)
		goto fail_no_sysfs;
	return 0;
fail_no_sysfs:
	return err;
}

static int throttling_remove(struct platform_device *pdev)
{
	PRINT_THROTTLING(("in throttling_remove\n"));
	sysfs_remove_group(&pdev->dev.kobj, &throttling_sysfs_files);
	return 0;
}

static struct platform_driver throttling_driver = {
	.probe  = throttling_probe,
	.remove = throttling_remove,
	.driver = {
		.name  = THROTTLING_MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device* throttling_dev;

static int __init throttling_init(void)
{
	int ret;
	PRINT_THROTTLING(("in throttling_init\n"));
	ret = platform_driver_register(&throttling_driver);
	if (ret < 0)
		return ret;
	
	throttling_dev = platform_device_register_simple(THROTTLING_MODULE_NAME, -1, NULL, 0);
	if (IS_ERR(throttling_dev)) {
		ret = PTR_ERR(throttling_dev);
		platform_driver_unregister(&throttling_driver);
	}
	return ret;
}

static void __exit throttling_exit(void)
{
	PRINT_THROTTLING(("in throttling_exit\n"));
	if ( throttling_dev ) {
		platform_device_unregister(throttling_dev);
		throttling_dev = NULL;
	}
	platform_driver_unregister(&throttling_driver);
}

module_init(throttling_init);
module_exit(throttling_exit);
MODULE_LICENSE("GPL");

