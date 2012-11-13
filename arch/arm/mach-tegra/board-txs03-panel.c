/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-txs03-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "board-txs03.h"
#include "devices.h"
#include "gpio-names.h"

static const int txs03_LcdEn   = TEGRA_GPIO_PP2;
static const int txs03_LcdVddEn= TEGRA_GPIO_PW1;
static const int txs03_LcdCabc = TEGRA_GPIO_PB0;
static const int txs03_LcdLVDS = TEGRA_GPIO_PC6;

static int txs03_lcd_status = 0;
static int txs03_cabc_status = 0;

/* HW Spec */
#define LCD_VDD_OFF_MIN_TIME 410 /* msec */
static unsigned long lcd_vdd_off_jiffies = 0;
static int lcd_vdd_off_jiffies_flag = 0;

/* Select DSI panel to be used. */
#define DSI_PANEL_219 0
#define DSI_PANEL_218 0
#define AVDD_LCD PMU_TCA6416_GPIO_PORT17
#define DSI_PANEL_RESET 0

/* Select LVDS panel resolution. 13X7 is default */
#define PM313_LVDS_PANEL_19X12_18BPP		1

/* PM313 display board specific pins */
#define pm313_R_FDE			TEGRA_GPIO_PW0
#define pm313_R_FB			TEGRA_GPIO_PN4
#define pm313_MODE0			TEGRA_GPIO_PZ4
#define pm313_MODE1			TEGRA_GPIO_PW1
#define pm313_lvds_shutdown		TEGRA_GPIO_PH1

/* E1247 reworked for pm269 pins */
#define e1247_pm269_lvds_shutdown	TEGRA_GPIO_PN6

/* E1247 txs03 default display board pins */
#define txs03_lvds_shutdown		TEGRA_GPIO_PL2

/* common pins( backlight ) for all display boards */
#define txs03_bl_enb			TEGRA_GPIO_PH2
#define txs03_bl_pwm			TEGRA_GPIO_PH0
#define txs03_hdmi_hpd			TEGRA_GPIO_PN7
#define txs03_vsync_gpio		TEGRA_GPIO_PJ4
static int txs03_bl_pwm_handle = -1;
static int txs03_vsync_gpio_handle = -1;

#ifdef CONFIG_TEGRA_DC
static struct regulator *txs03_hdmi_reg = NULL;
static struct regulator *txs03_hdmi_pll = NULL;
static struct regulator *txs03_hdmi_vddio = NULL;
#endif

static atomic_t sd_brightness = ATOMIC_INIT(255);

static struct regulator *txs03_lvds_reg = NULL;
static struct regulator *txs03_lvds_vdd_bl = NULL;
static struct regulator *txs03_lvds_vdd_panel = NULL;

static struct workqueue_struct* txs03_panel_workqueue = NULL;
static struct delayed_work txs03_panel_enable_delayed_work;
static DEFINE_MUTEX(txs03_panel_bl_mutex);
static int txs03_panel_bl_enabled = 0;

/* CABC functions */
void txs03_panel_cabc_enable(void){
	if( txs03_cabc_status ) {
		return;
	}
	if( txs03_lcd_status ){
		gpio_set_value(txs03_LcdCabc, 0x1);
		msleep(5);
	}
	txs03_cabc_status = 1;
}

void txs03_panel_cabc_disable(void){
	if( !txs03_cabc_status ) {
		return;
	}
	if( txs03_lcd_status ){
		gpio_set_value(txs03_LcdCabc, 0);
		msleep(5);
	}
	txs03_cabc_status = 0;
}

int txs03_panel_cabc_get_status(void){
	return txs03_cabc_status;
}

static tegra_dc_bl_output txs03_bl_output_measured = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 49, 50, 51, 52, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62,
	63, 64, 65, 66, 67, 68, 69, 70,
	70, 72, 73, 74, 75, 76, 77, 78,
	79, 80, 81, 82, 83, 84, 85, 86,
	87, 88, 89, 90, 91, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 108, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 124, 125, 126,
	127, 128, 129, 130, 131, 132, 133, 133,
	134, 135, 136, 137, 138, 139, 140, 141,
	142, 143, 144, 145, 146, 147, 148, 148,
	149, 150, 151, 152, 153, 154, 155, 156,
	157, 158, 159, 160, 161, 162, 163, 164,
	165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 179, 180, 181,
	182, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 196, 197, 198,
	199, 200, 201, 202, 203, 204, 205, 206,
	207, 208, 209, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};

static p_tegra_dc_bl_output bl_output;

#define MODELID_EVT 0x1c
#define MODELID_DVT 0x18
#define MODELID_PVT 0x14
#define MODELID_MP  0x10
static int get_model_id(void)
{
#if 0
    int ret=0;
    u32 odm_rsvd[8];
    int model_id;

    ret = tegra_fuse_read(ODM_RSVD, odm_rsvd, sizeof(odm_rsvd));
    if(ret!=0)return 0;
    model_id = odm_rsvd[0]&0x7f;
/*{
    int i;
    for(i=0; i<8; i++){pr_info("%08x ", odm_rsvd[i]);}
    pr_info("\n[%s] 0x%x\n", __FUNCTION__, model_id);
}*/
    return model_id;
#endif

    return MODELID_PVT;
};



static int txs03_backlight_init(struct device *dev) {

	bl_output = txs03_bl_output_measured;

        if (WARN_ON(ARRAY_SIZE(txs03_bl_output_measured) != 256))
                pr_err("bl_output array does not have 256 elements\n");
	return 0;
};

static void txs03_backlight_exit(struct device *dev) {
	return;
}

static int txs03_backlight_notify(struct device *unused, int brightness)
{
        int cur_sd_brightness = atomic_read(&sd_brightness);

        /* SD brightness is a percentage, 8-bit value. */
        brightness = (brightness * cur_sd_brightness) / 255;

        /* Apply any backlight response curve */
        if (brightness > 255) {
                pr_info("Error: Brightness > 255!\n");
        } else {
                /* This value depends on the panel.
                   Current 19X12 panel with PM313 gets
                   full brightness when the output is 0. */
                brightness = bl_output[brightness];
        }

        return brightness;
}

static int txs03_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data txs03_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 88,
	.pwm_period_ns	= 4760000,
	.init		= txs03_backlight_init,
	.exit		= txs03_backlight_exit,
	.notify		= txs03_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= txs03_disp1_check_fb,
};

static struct platform_device txs03_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &txs03_backlight_data,
	},
};

static u64 panel_enable_jiffies = 0;

static int txs03_panel_enable(void)
{
	unsigned long cur_jiffies;
	unsigned long duration_jiffies = 0;

	/* If LCD is turned off, LCD must be off for 400msec */
	if( lcd_vdd_off_jiffies_flag ){
		cur_jiffies = jiffies;
		if( cur_jiffies >= lcd_vdd_off_jiffies ){
			duration_jiffies = cur_jiffies - lcd_vdd_off_jiffies;
		}

		if( 0 <= jiffies_to_msecs(duration_jiffies) &&
		    jiffies_to_msecs(duration_jiffies) < LCD_VDD_OFF_MIN_TIME ){
			msleep(LCD_VDD_OFF_MIN_TIME - jiffies_to_msecs(duration_jiffies));
		}

		lcd_vdd_off_jiffies = 0;
		lcd_vdd_off_jiffies_flag = 0;
	}

	if( txs03_vsync_gpio_handle >= 0 ){
		gpio_free(txs03_vsync_gpio);
		tegra_gpio_disable(txs03_vsync_gpio);
		txs03_vsync_gpio_handle = -1;
	}

        gpio_set_value(txs03_LcdVddEn, 0x1);
        msleep(5);

	if( txs03_cabc_status ){
		gpio_set_value(txs03_LcdCabc, 0x1);
	}
        msleep(5);

        gpio_set_value(txs03_LcdLVDS, 0x1);

	msleep(10);

	txs03_lcd_status = 1;

	panel_enable_jiffies = get_jiffies_64();
	queue_delayed_work(txs03_panel_workqueue, &txs03_panel_enable_delayed_work, msecs_to_jiffies(250)); /* spec 200ms + expected latency 50ms */

	return 0;
}

static int txs03_panel_backlight_poweron(void)
{
	u64 cur_jiffies;
	u64 expire_jiffies;

	/* Only the earlier one is performed among delayed_work and late_resume. */

	mutex_lock(&txs03_panel_bl_mutex);
	do {
		if(txs03_panel_bl_enabled) break;

		cur_jiffies = get_jiffies_64();
		expire_jiffies = panel_enable_jiffies + msecs_to_jiffies(250); /* spec 200ms + expected latency 50ms */

		if(cur_jiffies < expire_jiffies) {
			msleep(jiffies_to_msecs(expire_jiffies - cur_jiffies));
		}

		gpio_set_value(txs03_LcdEn, 0x1);
		msleep(10);

		if( txs03_bl_pwm_handle >= 0 ){
			gpio_free(txs03_bl_pwm);
			tegra_gpio_disable(txs03_bl_pwm);
			txs03_bl_pwm_handle = -1;
		}

		txs03_panel_bl_enabled = 1;
	} while(0);
	mutex_unlock(&txs03_panel_bl_mutex);

	return 0;
}

static void txs03_panel_enable_delayed_work_func(struct work_struct *work)
{
	txs03_panel_backlight_poweron();
}

static int txs03_panel_backlight_poweroff(void)
{
	/* Although it may be called twice, it performs only once. */

	mutex_lock(&txs03_panel_bl_mutex);
	do {
		if(!txs03_panel_bl_enabled) break;

		txs03_bl_pwm_handle = gpio_request(txs03_bl_pwm, "txs03_bl_pwm");
		if(txs03_bl_pwm_handle<0) pr_err("gpio_request err txs03_bl_pwm\n");
		tegra_gpio_enable(txs03_bl_pwm);
		gpio_direction_output(txs03_bl_pwm, 0);
		msleep(10);

		gpio_set_value(txs03_LcdEn, 0x0);
		msleep(205);

		txs03_panel_bl_enabled = 0;
	} while(0);
	mutex_unlock(&txs03_panel_bl_mutex);

	return 0;
}

static int txs03_panel_disable(void)
{
	txs03_panel_backlight_poweroff();

        gpio_set_value(txs03_LcdLVDS, 0x0);
        msleep(5);

	if( txs03_cabc_status ){
		gpio_set_value(txs03_LcdCabc, 0x0);
	}
        msleep(5);

        gpio_set_value(txs03_LcdVddEn, 0x0);

	msleep(10);
	txs03_vsync_gpio_handle = gpio_request(txs03_vsync_gpio, "txs03_vsync_gpio");
	if(txs03_vsync_gpio_handle<0) {
		pr_err("gpio_request err txs03_vsync_gpio\n");
	}else{
		tegra_gpio_enable(txs03_vsync_gpio);
		gpio_direction_output(txs03_vsync_gpio, 0);
	}

	lcd_vdd_off_jiffies = jiffies;
	lcd_vdd_off_jiffies_flag = 1;

	txs03_lcd_status = 0;
	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int txs03_hdmi_vddio_enable(void)
{
	int ret;
	if (!txs03_hdmi_vddio) {
		txs03_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(txs03_hdmi_vddio)) {
			ret = PTR_ERR(txs03_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			txs03_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(txs03_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(txs03_hdmi_vddio);
		txs03_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int txs03_hdmi_vddio_disable(void)
{
	if (txs03_hdmi_vddio) {
		regulator_disable(txs03_hdmi_vddio);
		regulator_put(txs03_hdmi_vddio);
		txs03_hdmi_vddio = NULL;
	}
	return 0;
}

static int txs03_hdmi_enable(void)
{
	int ret;
	if (!txs03_hdmi_reg) {
		txs03_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(txs03_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			txs03_hdmi_reg = NULL;
			return PTR_ERR(txs03_hdmi_reg);
		}
	}
	ret = regulator_enable(txs03_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!txs03_hdmi_pll) {
		txs03_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(txs03_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			txs03_hdmi_pll = NULL;
			regulator_put(txs03_hdmi_reg);
			txs03_hdmi_reg = NULL;
			return PTR_ERR(txs03_hdmi_pll);
		}
	}
	ret = regulator_enable(txs03_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int txs03_hdmi_disable(void)
{
	regulator_disable(txs03_hdmi_reg);
	regulator_put(txs03_hdmi_reg);
	txs03_hdmi_reg = NULL;

	regulator_disable(txs03_hdmi_pll);
	regulator_put(txs03_hdmi_pll);
	txs03_hdmi_pll = NULL;
	return 0;
}

static struct resource txs03_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by txs03_panel_init() */
		.end	= 0,	/* Filled in by txs03_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
#ifdef CONFIG_TEGRA_DSI_INSTANCE_1
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSIB_BASE,
		.end	= TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#else
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct resource txs03_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

static struct tegra_dc_mode txs03_panel_modes[] = {
	{
		.pclk = 69000000,
		.h_ref_to_sync = 0,
		.v_ref_to_sync = 1,
		.h_sync_width = 32,
		.v_sync_width = 6,
		.h_back_porch = 42,
		.v_back_porch = 15,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 44,
		.v_front_porch = 2,
	},
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_fb_data txs03_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 800,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};
#endif

static struct tegra_fb_data txs03_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 800,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out txs03_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 3,
	.hotplug_gpio	= txs03_hdmi_hpd,

	.parent_clk     = "pll_d2_out0",
	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= txs03_hdmi_enable,
	.disable	= txs03_hdmi_disable,

	.postsuspend	= txs03_hdmi_vddio_disable,
	.hotplug_init	= txs03_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data txs03_disp2_pdata = {
	.flags		= 0,
	.default_out	= &txs03_disp2_out,
	.fb		= &txs03_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct tegra_dc_out_pin txs03_panel_out_pins[] = {
	{
		.name = TEGRA_DC_OUT_PIN_H_SYNC,
		.pol  = TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name = TEGRA_DC_OUT_PIN_V_SYNC,
		.pol  = TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name = TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol  = TEGRA_DC_OUT_PIN_POL_LOW,
	},
};

static struct tegra_dc_out txs03_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.parent_clk	= "pll_d_out0",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 24,
	.dither		= TEGRA_DC_DISABLE_DITHER,

	.modes	 	= txs03_panel_modes,
	.n_modes 	= ARRAY_SIZE(txs03_panel_modes),

	.enable		= txs03_panel_enable,
	.disable	= txs03_panel_disable,

	.out_pins       = txs03_panel_out_pins,
	.n_out_pins     = ARRAY_SIZE(txs03_panel_out_pins),
};

#ifdef CONFIG_TEGRA_DC
static struct tegra_dc_platform_data txs03_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &txs03_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &txs03_fb_data,
};

static struct nvhost_device txs03_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= txs03_disp1_resources,
	.num_resources	= ARRAY_SIZE(txs03_disp1_resources),
	.dev = {
		.platform_data = &txs03_disp1_pdata,
	},
};

static int txs03_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &txs03_disp1_device.dev;
}

static struct nvhost_device txs03_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= txs03_disp2_resources,
	.num_resources	= ARRAY_SIZE(txs03_disp2_resources),
	.dev = {
		.platform_data = &txs03_disp2_pdata,
	},
};
#else
static int txs03_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

static struct nvmap_platform_carveout txs03_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by txs03_panel_init() */
		.size		= 0,	/* Filled in by txs03_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data txs03_nvmap_data = {
	.carveouts	= txs03_carveouts,
	.nr_carveouts	= ARRAY_SIZE(txs03_carveouts),
};

static struct platform_device txs03_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &txs03_nvmap_data,
	},
};


static struct platform_device *txs03_gfx_devices[] __initdata = {
	&txs03_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&tegra_pwfm0_device,
	&txs03_backlight_device,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend txs03_panel_early_suspender;
struct early_suspend txs03_panel_bl_early_suspender;

static void txs03_panel_bl_early_suspend(struct early_suspend *h)
{
	cancel_delayed_work_sync(&txs03_panel_enable_delayed_work);
	txs03_panel_backlight_poweroff();
}

static void txs03_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
}

static void txs03_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}

static void txs03_panel_bl_late_resume(struct early_suspend *h)
{
	cancel_delayed_work_sync(&txs03_panel_enable_delayed_work);
	txs03_panel_backlight_poweron();
}
#endif

static int txs03_gpio_request(int gpio_num, const char *gpio_name, int value)
{
	int ret;

	ret = gpio_request(gpio_num, gpio_name);
	if (ret < 0) {
		pr_err("[%s]gpio req err: %d,%s\n", __FUNCTION__, gpio_num, gpio_name);
		return ret;
	}
	ret = gpio_direction_output(gpio_num, value);
	if (ret < 0) {
		pr_err("[%s]gpio dir err: %d,%s\n", __FUNCTION__, gpio_num, gpio_name);
		gpio_free(gpio_num);
		return ret;
	}

	tegra_gpio_enable(gpio_num);
	pr_info("[%s]gpio req: %d as %s\n", __FUNCTION__, gpio_num, gpio_name);

	return 0;
}


static int txs03_lcd_init(void)
{
	int ret, model_id;

	/* set value of each pin in the init has taken over from bootloader */
	pr_info("[%s]\n", __FUNCTION__);
	ret = txs03_gpio_request(txs03_LcdVddEn, "VddEn", 1);
	if (ret < 0) return ret;
	msleep(5);

        ret = txs03_gpio_request(txs03_LcdCabc, "LcdCabc", 0);
        if (ret < 0) return ret;
        msleep(5);

        ret = txs03_gpio_request(txs03_LcdLVDS, "LcdLVDS", 1);
        if (ret < 0) return ret;
	msleep(50);

	ret = atmel_gpio_power_init();
	if(ret < 0) return ret;
	atmel_gpio_power_control(1);
        msleep(170);

        ret = txs03_gpio_request(txs03_LcdEn, "LcdEn", 1);
        if (ret < 0) return ret;

	txs03_lcd_status = 1;
	txs03_cabc_status = 0;
        return ret;
}

int __init txs03_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;

	txs03_carveouts[1].base = tegra_carveout_start;
	txs03_carveouts[1].size = tegra_carveout_size;

	tegra_gpio_disable(txs03_bl_pwm);
	txs03_lcd_init();

    tegra_gpio_enable(txs03_hdmi_hpd);
    gpio_request(txs03_hdmi_hpd, "hdmi_hpd");
    gpio_direction_input(txs03_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	txs03_panel_early_suspender.suspend = txs03_panel_early_suspend;
	txs03_panel_early_suspender.resume = txs03_panel_late_resume;
	txs03_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&txs03_panel_early_suspender);

	txs03_panel_bl_early_suspender.suspend = txs03_panel_bl_early_suspend;
	txs03_panel_bl_early_suspender.resume = txs03_panel_bl_late_resume;
	txs03_panel_bl_early_suspender.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&txs03_panel_bl_early_suspender);
#endif

	err = platform_add_devices(txs03_gfx_devices,
				ARRAY_SIZE(txs03_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&txs03_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&txs03_disp1_device);

	res = nvhost_get_resource_byname(&txs03_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&txs03_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif

	txs03_panel_bl_enabled = 0;
	txs03_panel_workqueue = create_singlethread_workqueue("ec_ipc_tx_thread");
	if(txs03_panel_workqueue == NULL) {
		pr_err("board-txs03-panel:create_singlethread_workqueue() failed.\n");
		err = -ENOMEM;
	}
	INIT_DELAYED_WORK(&txs03_panel_enable_delayed_work, &txs03_panel_enable_delayed_work_func);

	return err;
}
