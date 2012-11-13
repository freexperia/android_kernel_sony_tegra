/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-txs03-kbc.c
 * Keys configuration for Nvidia tegra3 txs03 platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/max77663-core.h>
#include <linux/interrupt_keys.h>
#include <linux/gpio_scrollwheel.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-txs03.h"

#include "gpio-names.h"
#include "devices.h"

int __init txs03_scroll_init(void)
{
	return 0;
}

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button txs03_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PQ1, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PQ2, 0),
};

static struct gpio_keys_platform_data txs03_keys_platform_data = {
	.buttons        = txs03_keys,
	.nbuttons       = ARRAY_SIZE(txs03_keys),
};

static struct platform_device txs03_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &txs03_keys_platform_data,
	},
};

int __init txs03_keys_init(void)
{
	int i;

	pr_info("Registering gpio keys\n");

	for (i = 0; i < ARRAY_SIZE(txs03_keys); i++)
		tegra_gpio_enable(txs03_keys[i].gpio);

	platform_device_register(&txs03_keys_device);

	return 0;
}
