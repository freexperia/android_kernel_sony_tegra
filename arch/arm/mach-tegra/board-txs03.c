/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-txs03.c
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/nbx_ec_ipc.h>
#include <linux/nbx_ec_battery.h>
#include <linux/spi/cs48l10.h>
#include <linux/irq.h>

#include <sound/wm8903.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_wm8903_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#include <mach/hci-event.h>

#include "board.h"
#include "clock.h"
#include "board-txs03.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "baseband-xmm-power.h"
#include "txs_acc_det.h"
#include "wireless_power_control.h"
#include "board-nbx-common.h"


/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.temp_throttle = 66000,
	.temp_shutdown = 90000,
	.temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#else
	.hysteresis_throttle = 1000,
#endif
};

/* !!!TODO: Change for cardhu (Taken from Ventana) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
#if 0
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
#endif
};

#ifdef CONFIG_BCM4329_RFKILL
static struct resource txs03_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device txs03_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(txs03_bcm4329_rfkill_resources),
	.resource       = txs03_bcm4329_rfkill_resources,
};

static noinline void __init txs03_bt_rfkill(void)
{
	platform_device_register(&txs03_bcm4329_rfkill_device);

	return;
}
#else
static inline void txs03_bt_rfkill(void) { }
#endif

#ifdef CONFIG_BT_BLUESLEEP
static noinline void __init tegra_setup_bluesleep(void)
{
	struct platform_device *pdev = NULL;
	struct resource *res;

	pdev = platform_device_alloc("bluesleep", 0);
	if (!pdev) {
		pr_err("unable to allocate platform device for bluesleep");
		return;
	}

	res = kzalloc(sizeof(struct resource) * 2, GFP_KERNEL);
	if (!res) {
		pr_err("unable to allocate resource for bluesleep\n");
		goto err_free_dev;
	}

	res[0].name   = "gpio_host_wake";
	res[0].start  = TEGRA_GPIO_PU6;
	res[0].end    = TEGRA_GPIO_PU6;
	res[0].flags  = IORESOURCE_IO;

	res[1].name   = "host_wake";
	res[1].start  = gpio_to_irq(TEGRA_GPIO_PU6);
	res[1].end    = gpio_to_irq(TEGRA_GPIO_PU6);
	res[1].flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE ;

	if (platform_device_add_resources(pdev, res, 2)) {
		pr_err("unable to add resources to bluesleep device\n");
		goto err_free_res;
	}

	if (platform_device_add(pdev)) {
		pr_err("unable to add bluesleep device\n");
		goto err_free_res;
	}
	tegra_gpio_enable(TEGRA_GPIO_PU6);

return;

err_free_res:
	kfree(res);
err_free_dev:
	platform_device_put(pdev);
	return;
}
#else
static inline void tegra_setup_bluesleep(void) { }
#endif

static __initdata struct tegra_clk_init_table txs03_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	0,		false},
	{ "dam0",	"pll_a_out0",	0,		false},
	{ "dam1",	"pll_a_out0",	0,		false},
	{ "dam2",	"pll_a_out0",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data txs03_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data txs03_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data txs03_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data txs03_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data txs03_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};


#if 0
struct tegra_wired_jack_conf audio_wr_jack_conf = {
	.hp_det_n = TEGRA_GPIO_PW2,
	.en_mic_ext = TEGRA_GPIO_PX1,
	.en_mic_int = TEGRA_GPIO_PX0,
};
#endif

static struct wm8903_platform_data txs03_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = TXS03_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_MICBIAS_SHORT_DETECT << WM8903_GP1_FN_SHIFT),
		WM8903_GPIO_NO_CONFIG,
		0,                     /* as output pin */
		(WM8903_GPn_FN_MICBIAS_CURRENT_DETECT << WM8903_GP4_FN_SHIFT),
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &txs03_wm8903_pdata,
};

static void txs03_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &txs03_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &txs03_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &txs03_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &txs03_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &txs03_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(4, &wm8903_board_info, 1);
}

static struct platform_device *txs03_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk_default[] = {
	[0] = {.name = "pll_p"},
#if 0
	[1] = {.name = "clk_m"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
#endif
};

static struct uart_clk_parent uart_parent_clk_bluetooth[] = {
	[0] = {.name = "pll_p"},
};

static struct uart_clk_parent uart_parent_clk_gps[] = {
	[0] = {.name = "pll_p"},
};


static struct tegra_uart_platform_data txs03_uart_pdata_default;
static struct tegra_uart_platform_data txs03_loopback_uart_pdata;

static struct tegra_uart_platform_data txs03_uart_pdata_bluetooth;
static struct tegra_uart_platform_data txs03_uart_pdata_gps;


static void __init uart_debug_init(void)
{
	/* UARTA is the debug port. */
	pr_info("Selecting UARTA as the debug console\n");
	txs03_uart_devices[0] = &debug_uarta_device;
	debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
	debug_uart_port_base = ((struct plat_serial8250_port *)(
		debug_uarta_device.dev.platform_data))->mapbase;
}

static void __init txs03_uart_init_fixclocklist(struct uart_clk_parent* clklist, int num)
{
	int i;
	struct clk *c;

	for (i = 0; i < num; ++i) {
		c = tegra_get_clock_by_name(clklist[i].name);
                if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
			       clklist[i].name);
			continue;
		}
		clklist[i].parent_clk = c;
		clklist[i].fixed_clk_rate = clk_get_rate(c);
	}
}

static void __init txs03_uart_init(void)
{
	struct clk *c;

	txs03_uart_init_fixclocklist(uart_parent_clk_default, ARRAY_SIZE(uart_parent_clk_default));
	txs03_uart_pdata_default.parent_clk_list = uart_parent_clk_default;
	txs03_uart_pdata_default.parent_clk_count = ARRAY_SIZE(uart_parent_clk_default);
	txs03_loopback_uart_pdata.parent_clk_list = uart_parent_clk_default;
	txs03_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk_default);
	txs03_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &txs03_uart_pdata_default;
	tegra_uartd_device.dev.platform_data = &txs03_uart_pdata_default;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &txs03_loopback_uart_pdata;

	/* gps(UARTB) */
	txs03_uart_init_fixclocklist(uart_parent_clk_gps, ARRAY_SIZE(uart_parent_clk_gps));
	txs03_uart_pdata_gps.parent_clk_list = uart_parent_clk_gps;
	txs03_uart_pdata_gps.parent_clk_count = ARRAY_SIZE(uart_parent_clk_gps);
	tegra_uartb_device.dev.platform_data = &txs03_uart_pdata_gps;

	/* bluetooth(UARTC) */
	txs03_uart_init_fixclocklist(uart_parent_clk_bluetooth, ARRAY_SIZE(uart_parent_clk_bluetooth));
	txs03_uart_pdata_bluetooth.parent_clk_list = uart_parent_clk_bluetooth;
	txs03_uart_pdata_bluetooth.parent_clk_count = ARRAY_SIZE(uart_parent_clk_bluetooth);
	tegra_uartc_device.dev.platform_data = &txs03_uart_pdata_bluetooth;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(txs03_uart_devices,
				ARRAY_SIZE(txs03_uart_devices));
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *txs03_spi_devices[] __initdata = {
  //	&tegra_spi_device4,
	&tegra_spi_device2,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data txs03_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static struct tegra_spi_platform_data cs48l10_spi_pdata = {
        .is_dma_based           = true,
        .max_dma_buffer         = (16 * 1024),
        .is_clkon_always        = false,
        .max_rate               = 25000000,
};

static void __init txs03_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	//	txs03_spi_pdata.parent_clk_list = spi_parent_clk;
	//	txs03_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	//	tegra_spi_device4.dev.platform_data = &txs03_spi_pdata;
	cs48l10_spi_pdata.parent_clk_list = spi_parent_clk;
	cs48l10_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device2.dev.platform_data = &cs48l10_spi_pdata;
	platform_add_devices(txs03_spi_devices,
				ARRAY_SIZE(txs03_spi_devices));

}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_wm8903_platform_data txs03_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_cdc_irq		= TEGRA_GPIO_CDC_IRQ,
	.gpio_cdc_shrt		= TEGRA_GPIO_CDC_SHRT,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1, //TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= -1, //TEGRA_GPIO_EXT_MIC_EN,
	.echo_canceler_sleep    = 0,
	.echo_canceler_wake     = 0,
};

static struct platform_device txs03_audio_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data  = &txs03_audio_pdata,
	},
};

static struct txs_acc_dcout_platform_data txs_acc_dcout_pdata = {
	.dcout_pin_num = TEGRA_GPIO_PU3,
};
static struct platform_device txs_acc_dcout_device = {
	.name = "txs_acc_dcout",
	.id   = -1,
	.dev  = {
		.platform_data = &txs_acc_dcout_pdata,
	},
};

static struct platform_device txs_acc_5v0_hdmi_device = {
	.name = "txs_acc_5v0_hdmi",
	.id   = -1,
};

static struct platform_device txs_acc_dock_device = {
	.name = "txs_acc_dock",
	.id   = -1,
};

static struct platform_device txs_acc_dockspeaker_device = {
	.name = "txs_acc_dockspeaker",
	.id   = -1,
};

static struct hci_event_platform_data hci_event_platform = {
	.ocdet_pin = TEGRA_GPIO_PH4, /* USB_OC */
	.pwren_pin = TEGRA_GPIO_PH5, /* PWR_EN */
};
static struct platform_device hci_event_device = {
	.name = "hci-event",
	.dev = {
		.platform_data = &hci_event_platform,
	},
};

static struct platform_device *txs03_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
	&txs03_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&txs_acc_dcout_device,
	&txs_acc_5v0_hdmi_device,
	&txs_acc_dock_device,
	&txs_acc_dockspeaker_device,
	&hci_event_device,
};

static struct tegra_uhsic_config uhsic_phy_config = {
	.enable_gpio = EN_HSIC_GPIO,
	.reset_gpio = -1,
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
};

static struct tegra_ehci_platform_data tegra_ehci_uhsic_pdata = {
	.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
	.phy_config = &uhsic_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.default_enable = true,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = true,
	},
#if 0
	[2] = {
			.phy_config = &utmi_phy_config[2],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.hotplug = 1,
	},
#endif
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

#ifdef CONFIG_USB_SUPPORT
static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_micro_usb",
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_typea_usb",
	},
};

static void txs03_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata,
			ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	platform_device_register(&tegra_ehci2_device);

#if 0
	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
#endif
}
#else
static void txs03_usb_init(void) { }
#endif

static void txs03_gps_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PU2);
	tegra_gpio_enable(TEGRA_GPIO_PU3);
}

static void txs03_modem_init(void)
{
	return;
}

#ifdef CONFIG_SATA_AHCI_TEGRA
static void txs03_sata_init(void)
{
	platform_device_register(&tegra_sata_device);
}
#else
static void txs03_sata_init(void) { }
#endif

static void __init txs03_wwan_init(void)
{
	int irq;

	wireless_power_control(WPC_MODULE_WWAN, 1);
	msleep(10);
	wireless_power_control(WPC_MODULE_WWAN_RF, 1);

	irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV1);
	irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	enable_irq_wake(irq);
}

static struct spi_board_info cs48l10_spi_board_info[] __initdata = {
    {
/*      .modalias               = "spidev", */ /* Documentation/spi/spidev */
        .modalias               = "cs48l10",
        .bus_num                = 1,            /* SPI2*/
        .chip_select            = 1,            /* CSa1 */
        .max_speed_hz           = 25000000,      /* Max clock per spec is 24Mhz */
        .mode                   = SPI_MODE_0,   /* normal clock, rising edge */
//        .controller_data        = &cs48l10_spi_cdata,
        .platform_data          = &cs48l10_spi_pdata,
    },
};

static void __init txs03_cs48l10_init(void)
{
    tegra_gpio_enable(TEGRA_CS48L10_RESET_GPIO);
    tegra_gpio_enable(TEGRA_CS48L10_INT_GPIO);
    tegra_gpio_enable(TEGRA_CS48L10_BUSY_GPIO);

        if (gpio_request(TEGRA_CS48L10_RESET_GPIO, "cs48l10_hw_reset") < 0){
                printk(KERN_ERR "CS48L10: Failed to request GPIO%d for RESET",
                    TEGRA_CS48L10_RESET_GPIO);
    }


   gpio_direction_output(TEGRA_CS48L10_RESET_GPIO, 1);
    udelay(100);
    gpio_direction_output(TEGRA_CS48L10_RESET_GPIO, 0);
    udelay(100);
    gpio_direction_output(TEGRA_CS48L10_RESET_GPIO, 1);

        gpio_free(TEGRA_CS48L10_RESET_GPIO);

    spi_register_board_info(cs48l10_spi_board_info,
              ARRAY_SIZE(cs48l10_spi_board_info));
}

static void __init tegra_txs03_init(void)
{
	tegra_thermal_init(&thermal_data);
	tegra_clk_init_from_table(txs03_clk_init_table);
	txs03_pinmux_init();
	txs03_i2c_init();
	txs03_spi_init();
	txs03_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	txs03_edp_init();
#endif
	txs03_uart_init();
	//txs03_tsensor_init();
	platform_add_devices(txs03_devices, ARRAY_SIZE(txs03_devices));
	tegra_ram_console_debug_init();
	txs03_sdhci_init();
	txs03_regulator_init();
	txs03_gpio_switch_regulator_init();
	txs03_suspend_init();
	txs03_power_off_init();
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT) || defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_DIAG) || defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_TXS03)
	touch_init();
#endif
	//txs03_gps_init();
	//txs03_modem_init();
	//txs03_kbc_init();
	txs03_scroll_init();
	txs03_keys_init();
	txs03_panel_init();
	//txs03_pmon_init();
	txs03_sensors_init();
	//txs03_bt_rfkill();
#ifdef CONFIG_BT_BLUESLEEP
        tegra_setup_bluesleep();
#endif
	//txs03_sata_init();
	//audio_wired_jack_init(); /* FIXME: fixed gpio pins for jack */
	txs03_pins_state_init();
	txs03_emc_init(); /* FIXME: customize emc table */
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	txs03_wwan_init();
	txs03_cs48l10_init();
}

#ifdef CONFIG_NBX_EC_IPC

static struct nbx_ec_ipc_device nbx_ec_ipc_core_device = {
	.name = "nbx_ec_ipc",
};
static struct nbx_ec_ipc_device nbx_ec_ipc_suspend_device = {
	.name = "ec_ipc_suspend",
};
static struct nbx_ec_ipc_device nbx_ec_ipc_acplug_device = {
	.name = "ec_ipc_acplug",
};
#ifdef CONFIG_NBX_EC_IPC_LID_SWITCH
static struct nbx_ec_ipc_device nbx_ec_ipc_lidsw_device = {
	.name = "ec_ipc_lidsw",
};
#endif
#ifdef CONFIG_INPUT_NBX_EC_LID_SWITCH
static struct nbx_ec_ipc_device nbx_lidsw_device = {
	.name = "nbx_lidsw",
};
#endif
static struct nbx_ec_ipc_device nbx_powerkey_device = {
	.name = "nbx_powerkey",
};
static struct nbx_ec_ipc_device nbx_led_device = {
	.name = "nbx_led",
};

static struct nbx_ec_battery_platform_data nbx_ec_battery_pdata = {
	.degradation = 1,
	.trickle_charge = 1,
};
static struct nbx_ec_ipc_device nbx_battery_device = {
	.name = "nbx_battery",
	.dev = {
		.platform_data = &nbx_ec_battery_pdata,
	},
};
static struct nbx_ec_ipc_device nbx03_ec_ipc_light_device = {
	.name = "bh1690",
};

static struct txs_acc_det_platform_data txs_acc_det_pdata = {
	.det_pin_num = TEGRA_GPIO_PU5,
};
static struct nbx_ec_ipc_device txs_acc_det_device = {
	.name = "txs_acc_det",
	.dev = {
		.platform_data = &txs_acc_det_pdata,
	},
};
static struct nbx_ec_ipc_device nbx_usb_charge_device = {
	.name = "nbx_usb_charge",
};
static struct nbx_ec_ipc_device nbx_ec_ipc_chk_event_device = {
	.name = "ec_ipc_chk_event",
};

int __init txs03_ec_devices_init(void)
{
	int ret;

	/* must later than register tegra_uartd_device/driver */
	nbx_ec_ipc_core_device.dev.parent = &(tegra_uartd_device.dev);
	ret = nbx_ec_ipc_core_device_register(&nbx_ec_ipc_core_device);
	if (ret < 0) {
		return ret;
	}
	ret = nbx_ec_ipc_device_register(&nbx_ec_ipc_suspend_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&nbx_ec_ipc_acplug_device);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_NBX_EC_IPC_LID_SWITCH
	ret = nbx_ec_ipc_device_register(&nbx_ec_ipc_lidsw_device);
	if (ret < 0) {
		return ret;
	}
#endif
#ifdef CONFIG_INPUT_NBX_EC_LID_SWITCH
	ret = nbx_ec_ipc_device_register(&nbx_lidsw_device);
	if (ret < 0) {
		return ret;
	}
#endif

	ret = nbx_ec_ipc_device_register(&nbx_powerkey_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&nbx_led_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&nbx_battery_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&txs_acc_det_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&nbx_usb_charge_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&nbx03_ec_ipc_light_device);
	if (ret < 0) {
		return ret;
	}

	ret = nbx_ec_ipc_device_register(&nbx_ec_ipc_chk_event_device);
	if (ret < 0) {
		return ret;
	}

	nbx_ecipc_irq_setup(TEGRA_GPIO_PV0, IRQF_TRIGGER_LOW);

	return 0;
}
late_initcall(txs03_ec_devices_init);

#endif /* CONFIG_NBX_EC_IPC */

static void __init txs03_ramconsole_reserve(unsigned long size)
{
	struct resource *res;
	long ret;

	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_txs03_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	txs03_ramconsole_reserve(SZ_1M);
}

MACHINE_START(TXS03, "txs03")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_txs03_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_txs03_init,
MACHINE_END
