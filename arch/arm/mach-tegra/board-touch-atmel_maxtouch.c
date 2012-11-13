/* 2012-07-20: File changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-touch-atmel_maxtouch.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)
#include <linux/i2c/atmel_mxt_ts.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_DIAG)
#include <linux/i2c/atmel_mxt_ts_diag.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_TXS03)
#include <linux/i2c/atmel_mxt_ts_txs03.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
#include <linux/i2c/atmel_maxtouch.h>
#endif

#if defined(CONFIG_MACH_CARDHU)
#include "board-spica.h"
#define MXT_I2C_ADDRESS	MXT1386_I2C_ADDR2
#endif

#if defined(CONFIG_MACH_VENTANA)
#include "board-ventana.h"
#define MXT_I2C_ADDRESS	MXT1386_I2C_ADDR3
#endif

#if defined(CONFIG_MACH_TXS03)
#include "board-txs03.h"
#define MXT_I2C_ADDRESS MXT1386_I2C_ADDR2
#endif

#include "gpio-names.h"
#include "touch.h"

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT) ||  defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_DIAG) || defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_TXS03)
/* Atmel MaxTouch touchscreen              Driver data */
/*-----------------------------------------------------*/
/*
 * Config converted from memory-mapped cfg-file with
 * following version information:
 *
 *
 *
 *	FAMILY_ID=160
 *	VARIANT=0
 *	VERSION=16
 *	BUILD=170
 *	VENDOR_ID=255
 *	PRODUCT_ID=TBD
 *	CHECKSUM=0x144A89
 *
 *
 */

#if defined(CONFIG_MACH_VENTANA) || defined(CONFIG_MACH_CARDHU) || defined(CONFIG_MACH_TXS03)

int atmel_gpio_power_init(void)
{
	int ret;

	ret = gpio_request( TOUCH_GPIO_RST_ATMEL_T9, "touch-reset" );
	if( ret < 0 ){
		pr_err("%s(): gpio_request() fails for gpio %d (touch-reset)\n",
		       __func__, TOUCH_GPIO_RST_ATMEL_T9);
		return ret;
	}

	ret = gpio_request( TOUCH_GPIO_POW_ATMEL_T9, "touch-power" );
	if( ret < 0 ) {
		pr_err("%s(): gpio_request() fails for gpio %d (touch-power)\n",
		       __func__, TOUCH_GPIO_POW_ATMEL_T9);
		gpio_free( TOUCH_GPIO_RST_ATMEL_T9 );
		return ret;
	}

	gpio_direction_output( TOUCH_GPIO_RST_ATMEL_T9, 0 );
	gpio_direction_output( TOUCH_GPIO_POW_ATMEL_T9, 0 );

	tegra_gpio_enable( TOUCH_GPIO_RST_ATMEL_T9 );
	tegra_gpio_enable( TOUCH_GPIO_POW_ATMEL_T9 );

	return ret;
}

void atmel_gpio_power_free(void)
{
    gpio_free( TOUCH_GPIO_RST_ATMEL_T9 );
    gpio_free( TOUCH_GPIO_POW_ATMEL_T9 );
}

int atmel_gpio_power_control( int on ){
	if( on!=1 && on!= 0 ) return -1;

	pr_debug("atmel_gpio_power_control %d\n", on );
	if( on==1 ){
		txs03_pinmux_i2c_gen2_resume();
		gpio_set_value(TOUCH_GPIO_POW_ATMEL_T9, on );
		msleep( 10 );
		gpio_set_value(TOUCH_GPIO_RST_ATMEL_T9, on );
	}else if( on==0 ){
		gpio_set_value(TOUCH_GPIO_RST_ATMEL_T9, on );
		msleep( 10 );
		gpio_set_value(TOUCH_GPIO_POW_ATMEL_T9, on );
		txs03_pinmux_i2c_gen2_suspend();
	}
	return 0;
}

static void destruct(void)
{
    gpio_free( TOUCH_GPIO_IRQ_ATMEL_T9 );
    atmel_gpio_power_free();
    return;
}

static void power(int on)
{   
    pr_debug("### mXT TP power(%d)\n", on);
    atmel_gpio_power_control(on);
    if(on)  msleep(165);
    return;
}

#define MXT_CONFIG_CRC  0xD62DE8
static const u8 config[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3C, 0x04, 0x05, 0x00,
	0x02, 0x01, 0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x64, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0xFF, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static struct mxt_platform_data atmel_mxt_info = {
	.x_line         = 27,
	.y_line         = 42,
	.x_size         = 1280,
	.y_size         = 800,
	.blen           = 0x20,
	.threshold      = 0x3C,
	.voltage        = 3300000,              /* 3.3V */
	.orient         = 0,
	// XXX:	.config         = config,
	// XXX:	.config_length  = 157,
	.config_crc     = MXT_CONFIG_CRC,
	.irqflags       = IRQF_TRIGGER_FALLING,
/*	.read_chg       = &read_chg, */
	.read_chg       = NULL,
    .destruct       = &destruct,
    .power          = &power,
};

#define MXT_CONFIG_CRC_SKU2000  0xA24D9A
static const u8 config_sku2000[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x19,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3A, 0x04, 0x05, 0x00,  //23=thr  2 di
	0x04, 0x04, 0x41, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,  //0A=limit
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0x00, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void init_sku2000(void)
{
	struct mxt_platform_data *mxt = &atmel_mxt_info;
	// XXX: mxt->config = config_sku2000;
    // XXX:	mxt->config_crc = MXT_CONFIG_CRC_SKU2000;
}
#endif	/* CONFIG_MACH_VENTANA, CONFIG_MACH_CARDHU */

/* Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 *
static u8 read_chg(void)
{
	return gpio_get_value(TOUCH_GPIO_IRQ_ATMEL_T9);
}
 */

static struct i2c_board_info __initdata atmxt_i2c_info[] = {
	{
	 I2C_BOARD_INFO("atmel_mxt_ts", MXT_I2C_ADDRESS),
	 .irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ_ATMEL_T9),
	 .platform_data = &atmel_mxt_info,
	 },
};
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
/* Atmel MaxTouch touchscreen              Driver data */
/*-----------------------------------------------------*/
/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */

static u8 read_chg(void)
{
	return gpio_get_value(TOUCH_GPIO_IRQ_ATMEL_T9);
}

static u8 valid_interrupt(void)
{
	return !read_chg();
}


static struct mxt_platform_data atmel_mxt_info = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch = 10,
	/* TODO: no need for any hw-specific things at init/exit? */
	.init_platform_hw = NULL,
	.exit_platform_hw = NULL,

	.max_x = 1366,
	.max_y = 768,

	.valid_interrupt = &valid_interrupt,
	.read_chg = &read_chg,
};

static struct i2c_board_info __initdata atmxt_i2c_info[] = {
	{
	 I2C_BOARD_INFO("maXTouch", MXT_I2C_ADDRESS),
	 .irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ_ATMEL_T9),
	 .platform_data = &atmel_mxt_info,
	 },
};
#endif

struct tegra_touchscreen_init __initdata atmel_mxt_init_data = {
	.irq_gpio = TOUCH_GPIO_IRQ_ATMEL_T9,			/* GPIO1 Value for IRQ */
	.rst_gpio = TOUCH_GPIO_RST_ATMEL_T9,			/* GPIO2 Value for RST */
	.sv_gpio2 = {1, TOUCH_GPIO_RST_ATMEL_T9, 1, 100},	/* Valid, GPIOx, Set value, Delay      */
	.ts_boardinfo = {TOUCH_BUS_ATMEL_T9, atmxt_i2c_info, 1}	/* BusNum, BoardInfo, Value     */
};

