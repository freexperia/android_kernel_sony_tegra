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

//#define SYSFS_CABC_DEBUG_PRINTS 1

#define CABC_MODULE_NAME        "cabc"

#ifdef SYSFS_CABC_DEBUG_PRINTS
#define PRINT_CABC(x) pr_debug x
#else
#define PRINT_CABC(x)
#endif

static ssize_t sysfscabc_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t sysfscabc_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count);


static struct kobj_attribute cabc_enable_attr =
	__ATTR(enable, 0400, sysfscabc_show, sysfscabc_store);

static ssize_t sysfscabc_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int enable;
	enable = txs03_panel_cabc_get_status();
	PRINT_CABC(("sysfscabc_show enable = %d \n", enable));
	snprintf(buf,PAGE_SIZE,"%d\n",enable);
	return (strlen(buf));
}

static ssize_t sysfscabc_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (buf==NULL || count == 0) {
		PRINT_CABC(("Invalid Argument\n"));
		return -EINVAL;
	}
	
	if (strncmp(buf, "0", 1) == 0) {
		txs03_panel_cabc_disable();
	} else if (strncmp(buf, "1", 1) == 0) {
		txs03_panel_cabc_enable();
	} else {
		PRINT_CABC(("Invalid data in ssize_t sysfscabc_store_1\n"));
		return -EINVAL;
	} 
	return count;
}

static struct attribute *cabc_attrs[] = {
	&cabc_enable_attr.attr,
	NULL
};

static const struct attribute_group cabc_sysfs_files = {
	.attrs	= cabc_attrs,
};

static int cabc_probe(struct platform_device *pdev)
{
	int err;
	PRINT_CABC(("in cabc_probe\n"));
	err = sysfs_create_group(&pdev->dev.kobj, &cabc_sysfs_files);
	if (err)
		goto fail_no_sysfs;
	return 0;
fail_no_sysfs:
	return err;
}

static int cabc_remove(struct platform_device *pdev)
{
	PRINT_CABC(("in cabc_remove\n"));
	sysfs_remove_group(&pdev->dev.kobj, &cabc_sysfs_files);
	return 0;
}

static struct platform_driver cabc_driver = {
	.probe  = cabc_probe,
	.remove = cabc_remove,
	.driver = {
		.name  = CABC_MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device* cabc_dev;

static int __init cabc_init(void)
{
	int ret;
	PRINT_CABC(("in cabc_init\n"));
	ret = platform_driver_register(&cabc_driver);
	if (ret < 0)
		return ret;
	
	cabc_dev = platform_device_register_simple(CABC_MODULE_NAME, -1, NULL, 0);
	if (IS_ERR(cabc_dev)) {
		ret = PTR_ERR(cabc_dev);
		platform_driver_unregister(&cabc_driver);
	}
	return ret;
}

static void __exit cabc_exit(void)
{
	PRINT_CABC(("in cabc_exit\n"));
	if ( cabc_dev ) {
		platform_device_unregister(cabc_dev);
		cabc_dev = NULL;
	}
	platform_driver_unregister(&cabc_driver);
}

module_init(cabc_init);
module_exit(cabc_exit);
MODULE_LICENSE("GPL");

