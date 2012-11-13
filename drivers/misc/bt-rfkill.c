/* 2012-04-18: File changed by Sony Corporation */
/*
 * Bluetooth rfkill power control
 * Copyright (C) 2010 Sony Corporation
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include "wireless_power_control.h" 

static int bt_rfkill_set_power(void *data, bool blocked)
{
	int ret;

	if (!blocked) {
		ret = wireless_power_control(WPC_MODULE_BLUETOOTH, 1);
		if (ret < 0) 
			pr_err("%s: power on failed\n", __func__);

	} else {
		ret = wireless_power_control(WPC_MODULE_BLUETOOTH, 0);
		if (ret < 0) 
			pr_err("%s: power off failed\n", __func__);
	}
	return ret;
}

static struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_rfkill_set_power,
};

static int bt_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill;

	int rc;

	rfkill = rfkill_alloc("bt-rfkill", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &bt_rfkill_ops, pdev);
	if (!rfkill) {
		pr_err("%s: rfkill allocation failed\n", __func__);
		rc = -ENOMEM;
		return rc;
	}
	platform_set_drvdata(pdev, rfkill);
	
	rc = rfkill_register(rfkill);
	if (rc < 0) {
		pr_err("%s: rfkill registration failed\n", __func__);
		rfkill_destroy(rfkill);
		return rc;
	}

	return 0;
}

static int bt_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);

	return 0;
	
}

static struct platform_driver bt_rfkill_driver = {
	.probe = bt_rfkill_probe,
	.remove = bt_rfkill_remove,
	.driver = {
		.name = "bt-rfkill",
		.owner = THIS_MODULE,
	},
};

static struct platform_device bt_device = {
	.name = "bt-rfkill",
};

static int __init bt_rfkill_init(void)
{
	int ret;

	ret = platform_driver_register(&bt_rfkill_driver);
	if (ret < 0) {
		pr_err("%s: driver registration failed\n", __func__);
		return ret;
	}

	ret = platform_device_register(&bt_device);
	if (ret < 0) {	
		pr_err("%s: device registration failed\n", __func__);
		platform_driver_unregister(&bt_rfkill_driver);
	}

	return ret;
}

static void __exit bt_rfkill_exit(void)
{
	platform_driver_unregister(&bt_rfkill_driver);
}

module_init(bt_rfkill_init);
module_exit(bt_rfkill_exit);

MODULE_DESCRIPTION("BT rfkill");
MODULE_AUTHOR("Sony");
MODULE_LICENSE("GPL");
