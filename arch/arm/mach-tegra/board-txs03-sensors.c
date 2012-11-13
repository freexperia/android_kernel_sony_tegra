/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-txs03-sensors.c
 *
 * Copyright (c) 2010-2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*****************************************************************************
 *****************************************************************************
 *
 * Include Files
 *
 *****************************************************************************
 *****************************************************************************/


#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#include <media/ov9726.h>
#include <media/ov8820.h>
#include <media/ov8820_focuser.h>
#include <generated/mach-types.h>
#include "gpio-names.h"
#include "board.h"
#include <linux/mpu.h>
#include <linux/bq27x00.h>
#include <mach/gpio.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include <media/tegra_camera.h>

#include "board-txs03.h"
#include "cpu-tegra.h"

#include <linux/lis331dlh_1.h>

#ifdef SF_BUILD_AKMD_ENABLED
#if defined(CONFIG_INPUT_AK8975C)
#include <linux/akm8975.h>
#endif
#else /* not SF_BUILD_AKMD_ENABLED */
#if defined(CONFIG_INPUT_AK8975C)
#include <linux/ak8975c.h>
#endif
#endif

#include <linux/l3g4200dh.h>

#ifdef CONFIG_INPUT_SX8636
#include <linux/input/sx863x.h>
#include <linux/input/sx863x_i2c_reg.h>
#include <linux/input/sx863x_spm_cfg.h>
#include <linux/input.h>
#endif

/*****************************************************************************
 *****************************************************************************
 *
 * Structure and Type definitions
 *
 *****************************************************************************
 *****************************************************************************/

#ifdef CONFIG_INPUT_SX8636
static struct sx863x_button_platform_data sx8636_i2c_button_plat[] = {
	{
		.keycode = KEY_0,
		.capstatelsb_Mask = SX863X_CAPSTATLSB_BUTTON0,		// top
	},
	{
		.keycode = KEY_1,
		.capstatelsb_Mask = SX863X_CAPSTATLSB_BUTTON4,		// bottom
	},
};

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct sx863x_reg_platform_data sx863x_i2c_reg_setup[] = {
	{
		.reg = SX863X_COMPOPMODE_REG,
		.val = SX863X_COMPOPMODE_OPERATINGMODE_ACTIVE,
	},
};
/* Define SPM configuration map to be intialized to values different than
 * default. NOTE: Programming is done in 8 blocks. 
 * The driver is setup to perform needed reads and adjustments so that
 * it is not required to add in extra default values.
 */ 
static struct sx863x_reg_platform_data sx863x_spm_cfg_setup[] = {
	{
		.reg = SX863X_ACTIVESCANPERIOD_SPM,
		.val = 0x0D,
	},
	{
		.reg = SX863X_CAPEMODEMISC_SPM,
		.val = 0x04,
	},
	{
		.reg = SX863X_CAPMODE7_4_SPM,
		.val = SX863X_CAPMODE7_4_CAP4_BUTTON,
	},	
	{
		.reg = SX863X_CAPMODE3_0_SPM,
		.val = SX863X_CAPMODE3_0_CAP0_BUTTON,
	},
	{
		.reg = SX863X_CAPSENSITIVITY0_1_SPM,
		.val = 0x70,	// Cap0 sensitivity = 7
	},
	{
		.reg = SX863X_CAPSENSITIVITY2_3_SPM,
		.val = 0x00,
	},
	{
		.reg = SX863X_CAPSENSITIVITY4_5_SPM,
		.val = 0x70,	// Cap4 sensitivity = 7
	},
	{
		.reg = SX863X_CAPTHRESH0_SPM,
		.val = 0x39, //0x41,
	},
	{
		.reg = SX863X_CAPTHRESH4_SPM,
		.val = 0x39, //0x41,
	},
	{
		.reg = SX863X_CAPPERCOMP_SPM,
		.val = SX863X_CAPPERCOMP_OFF,
	},	
	{
		.reg = SX863X_CAPPROXENABLE_SPM,
		.val = 0xB4, //SX863X_CAPPROXENABLE_ON, // SX863X_CAPPROXENABLE_OFF
	},
	{
		.reg = SX863X_ENGBTNCFG1,
		.val = 0x22,		// default: 0x23
	},	
	{
		.reg = SX863X_BTNCFG_SPM,
		.val = SX863X_BTNCFG_INTERRUPT_TOUCHED | 
			SX863X_BTNCFG_INTERRUPT_RELEASE | 
			SX863X_BTNCFG_TOUCH_DEBOUNCE_4SAMPLE | 
			SX863X_BTNCFG_RELEASE_DEBOUNCE_4SAMPLE,
	},
	{
		.reg = SX863X_BTNHYSTERESIS_SPM,
		.val = 0x0A,		/* 10% */
	},
	{
		.reg = SX863X_BTNAVGTHRESH_SPM,
		.val = 0xFF,
	},
	{
		.reg = SX863X_BTNCOMPNEGTHRESH_SPM,
		.val = 0xFF, //0x32,
	},
	{
		.reg = SX863X_BTNCOMPNEGCNTMAX_SPM,
		.val = 0x01,
	},
	{
		.reg = SX863X_BTNSTUCKATTIMEOUT_SPM,
		.val = SX863X_BTNSTUCKATTIMEOUT_DEFAULT,
	},
};
/* Define the arrays and size of arrays for driver use */
static struct sx863x_platform_data sx8636_i2c_platform_data = {
	.button_num = ARRAY_SIZE(sx8636_i2c_button_plat),
	.i2c_reg_num = ARRAY_SIZE(sx863x_i2c_reg_setup),
	.spm_cfg_num = ARRAY_SIZE(sx863x_spm_cfg_setup),
	.button = sx8636_i2c_button_plat,
	.i2c_reg = sx863x_i2c_reg_setup,
	.spm_cfg = sx863x_spm_cfg_setup,
};
#endif

/*
 * opaque handle to power regulator.
 */
typedef struct regulator *HREGULATOR;


/*
 * structure for defining a GPIO pin assignment.
 */
typedef struct _gpio_definition
{
	unsigned    nPinId;             /* ID of the pin (use TEGRA_GPIO_... values) */
	char       *pszSymbol;          /* symbolic name for the pin */
	int         nState;             /* the initialisation state for the pin (0/1) */

} GPIODEFINITION, *PGPIODEFINITION;


/*****************************************************************************
 *****************************************************************************
 *
 * Local function prototypes
 *
 *****************************************************************************
 *****************************************************************************/


/*
 * sensor power on/off functions.
 */
static int        txs03_ov8820_power_on   (void);
static int        txs03_ov8820_power_off  (void);
static int        txs03_ov9726_power_on   (void);
static int        txs03_ov9726_power_off  (void);


/* 
 * local processing functions.
 */
static int        gpio_initialise_table     (GPIODEFINITION *paGpioTable, int nTableSize);


/*****************************************************************************
 *****************************************************************************
 *
 * Local read-only variables
 *
 *****************************************************************************
 *****************************************************************************/


/*
 * table of initialisation-time GPIO pin definitions for the camera sensors.
 *
 * NOTES:
 *
 * 1) sensors are always held in a state of reset if they're not being used.
 */
static GPIODEFINITION s_arrInitGpioDefs[] =
{
	{ CAMERA_VDDIO_ENABLE,          "CAMERA_VDDIO_ENABLE",          1 },
	{ CAMERA_OV8820_RESET,          "CAMERA_OV8820_RESET",          0 },
	{ CAMERA_OV8820_ENABLE_1V8,     "CAMERA_OV8820_ENABLE_1V8",     0 },
	{ CAMERA_OV8820_ENABLE_2V8,     "CAMERA_OV8820_ENABLE_2V8",     0 },
	{ CAMERA_OV8820_ENABLE_2V8_VCM, "CAMERA_OV8820_ENABLE_2V8_VCM", 0 },
	{ CAMERA_OV8820_POWERDWN,       "CAMERA_OV8820_POWERDWN",       0 },
	{ CAMERA_OV9726_RESET,          "CAMERA_OV9726_RESET",          0 },
	{ CAMERA_OV9726_ENABLE,         "CAMERA_OV9726_ENABLE",         0 },
	{ CAMERA_OV9726_POWERDWN,       "CAMERA_OV9726_POWERDWN",       0 },
};


/*
 * table of GPIO pin definitions required before camera startup and after
 * camera shutdown to guarantee GPIO starting and idle state.
 */
static GPIODEFINITION s_arrResetGpioDefs[] =
{
	{ CAMERA_VDDIO_ENABLE,          (char*) NULL, 1 },
	{ CAMERA_OV8820_RESET,          (char*) NULL, 0 },
	{ CAMERA_OV8820_ENABLE_1V8,     (char*) NULL, 0 },
	{ CAMERA_OV8820_ENABLE_2V8,     (char*) NULL, 0 },
	{ CAMERA_OV8820_ENABLE_2V8_VCM, (char*) NULL, 0 },
	{ CAMERA_OV8820_POWERDWN,       (char*) NULL, 0 },
	{ CAMERA_OV9726_RESET,          (char*) NULL, 0 },
	{ CAMERA_OV9726_ENABLE,         (char*) NULL, 0 },
	{ CAMERA_OV9726_POWERDWN,       (char*) NULL, 0 },
};

/*
 * platform (sensor-specific) attributes that the kernel driver will use to
 * operate and control the OV9726.
 */
static struct ov9726_platform_data txs03_ov9726_platform_data =
{
	.power_on  = txs03_ov9726_power_on,
	.power_off = txs03_ov9726_power_off
};


/*
 * i2c bus definitions for the cameras.
 */
static struct i2c_board_info txs03_camera_board_info[] =
{
	{
		I2C_BOARD_INFO("ov8820-i2c", 0x36),
	},

	{
		I2C_BOARD_INFO("ov9726", 0x10),
		.platform_data = &txs03_ov9726_platform_data
	}
};

static struct platform_device ov8820_device = {
        .name = "ov8820",
        .id = -1,
};

/*
 * platform (sensor-specific) attributes that the kernel driver will use to
 * operate and control the OV8820.
 */
struct ov8820_focuser_platform_data txs03_ov8820_focuser_pdata = {
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.power_on  = txs03_ov8820_power_on,
	.power_off = txs03_ov8820_power_off
};

static struct platform_device ov8820_focuser_device = {
        .name = "ov8820_focuser",
        .id = -1,
        .dev = {
		    .platform_data = &txs03_ov8820_focuser_pdata,
	    },
};

/*****************************************************************************
 *****************************************************************************
 *
 * Local Processing Functions
 *
 *****************************************************************************
 *****************************************************************************/


/*****************************************************************************
 *
 * Initialise a set of GPIOs defined using a GPIO definition table.
 *
 * "paGpioTable" specifies a pointer to a table of of GPIODEFINITION structures
 * containing the GPIO definitions to be processed.
 *
 * "nTableSize" specifies the number of structures in the table pointed to by
 * "paGpioTable".
 *
 * RETURN
 *
 * Returns zero if the table is initialised successfully or non-zero if an error
 * occurred.
 */
static int gpio_initialise_table(GPIODEFINITION *paGpioTable, int nTableSize)
{
	int             nRetCode, nTableIdx;
	PGPIODEFINITION pGpioDef;

	/*
	 * initialise scan of supplied definition table.
	 */
	nRetCode = 0;
	pGpioDef = paGpioTable;

	/*
	 * scan the supplied definition table...
	 */
	for (nTableIdx = 0; nTableIdx < nTableSize; nTableIdx++, pGpioDef++)
	{
		/*
		 * has a GPIO symbolic name been provided?
		 */
		if (pGpioDef->pszSymbol)
		{
			/*
			 * attempt to enable the GPIO, request ownership of it, then set it to
			 * its initial state. if any of these operations fail then abort.
			 */
			tegra_gpio_enable(pGpioDef->nPinId);

			pr_debug(KERN_DEBUG "%s: initialise GPIO '%s'.\n", __func__, pGpioDef->pszSymbol);

			/*
			 * attempt to request ownership of the pin.
			 */
			if ((nRetCode = gpio_request(pGpioDef->nPinId, pGpioDef->pszSymbol)) != 0)
				break;
		}

		/*
		 * set the state of the pin, treating it as an output.
		 */
		pr_debug(KERN_DEBUG "%s: set GPIO '%s' state to %i.\n", __func__, pGpioDef->pszSymbol, pGpioDef->nState);

		if ((nRetCode = gpio_direction_output(pGpioDef->nPinId, pGpioDef->nState)) != 0)
			break;
	}

	/*
	 * if we reached the end of the table then everything went well.
	 */
	if (nTableIdx >= nTableSize)
		return 0;

	pr_debug(KERN_DEBUG "%s: failure.\n", __func__);

	/*
	 * an error occurred somewhere. free all the GPIO pins we managed to process
	 * up to the failure point.
	 */
	for (; nTableIdx >= 0; nTableIdx--, pGpioDef--)
		gpio_free(pGpioDef->nPinId);

	/*
	 * failed.
	 */
	return nRetCode;
}

/*****************************************************************************
 *****************************************************************************
 *
 * Sensor related power on/power off functions.
 *
 *****************************************************************************
 *****************************************************************************/


/*****************************************************************************
 *
 * Perform OV8820 (rear camera) power on sequence.
 *
 * RETURN
 *
 * Returns zero if successful or non-zero if an error occurred.
 */
static int txs03_ov8820_power_on(void)
{
	int nRetCode;

	pr_debug(KERN_DEBUG "%s: enter\n", __func__);

	pr_debug(KERN_DEBUG "%s: setting GPIOs..\n", __func__);

	/*
	 * guarantee GPIO pins are in the default initialisation state before starting.
	 */
	if ((nRetCode = gpio_initialise_table(s_arrResetGpioDefs, ARRAY_SIZE(s_arrResetGpioDefs))) < 0)
	{
		pr_debug(KERN_DEBUG "%s: GPIO pin reset failed.\n", __func__);
		txs03_ov8820_power_off();
		return nRetCode;
	}

	/*
	 * perform power-on sequence. core GPIO setting function never returns an error
	 * so we do no error checking here.
	 */
	gpio_direction_output(CAMERA_VDDIO_ENABLE, 0);
	gpio_direction_output(CAMERA_OV9726_ENABLE, 1);
	gpio_direction_output(CAMERA_OV8820_ENABLE_1V8, 1);
	mdelay(2);

	gpio_direction_output(CAMERA_OV8820_ENABLE_2V8, 1);
	mdelay(2);

	gpio_direction_output(CAMERA_OV8820_ENABLE_2V8_VCM, 1);
	mdelay(7);
 
	tegra_camera_enable_mclk();

	gpio_direction_output(CAMERA_OV8820_POWERDWN, 1);
	mdelay(2);

	gpio_direction_output(CAMERA_OV8820_RESET, 1);
	mdelay(20);

	/*
	 * success.
	 */
	pr_debug(KERN_DEBUG "%s: exit\n", __func__);

	return 0;
}


/*****************************************************************************
 *
 * Perform OV8820 (rear camera) power off sequence.
 *
 * RETURN
 *
 * Returns zero if successful or non-zero if an error occurred.
 */
static int txs03_ov8820_power_off(void)
{
	pr_debug(KERN_DEBUG "%s: enter\n", __func__);

    /*
     * perform power-off sequence.
     */
    gpio_direction_output(CAMERA_OV8820_RESET, 0);

    mdelay(2);
    tegra_camera_disable_mclk();

    mdelay(2);
    gpio_direction_output(CAMERA_OV8820_ENABLE_2V8_VCM, 0);

    mdelay(2);
    gpio_direction_output(CAMERA_OV8820_ENABLE_2V8, 0);

    mdelay(2);
    gpio_direction_output(CAMERA_OV8820_POWERDWN, 0);

    mdelay(2);
    gpio_direction_output(CAMERA_OV8820_ENABLE_1V8, 0);
    gpio_direction_output(CAMERA_OV9726_ENABLE, 0);
    gpio_direction_output(CAMERA_VDDIO_ENABLE, 1);

	/*
	 * success.
	 */
	pr_debug(KERN_DEBUG "%s: exit\n", __func__);

	return 0;
}


/*****************************************************************************
 *
 * Perform OV9726 (front camera) power on sequence.
 *
 * RETURN
 *
 * Returns zero if successful or non-zero if an error occurred.
 */
static int txs03_ov9726_power_on(void)
{
	int nRetCode;

	pr_debug(KERN_DEBUG "%s: enter\n", __func__);

	/*
	 * guarantee GPIO pins are in the default initialisation state before starting.
	 */
	if ((nRetCode = gpio_initialise_table(s_arrResetGpioDefs, ARRAY_SIZE(s_arrResetGpioDefs))) < 0)
	{
		pr_debug(KERN_DEBUG "%s: GPIO pin reset failed.\n", __func__);
		txs03_ov9726_power_off();
		return nRetCode;
	}

	/*
	 * perform power-on sequence. core GPIO setting function never returns an error
	 * so we do no error checking here.
	 */
	gpio_direction_output(CAMERA_VDDIO_ENABLE, 0);
	gpio_direction_output(CAMERA_OV9726_ENABLE, 1);
	gpio_direction_output(CAMERA_OV8820_ENABLE_1V8, 1);
	mdelay(2);

	gpio_direction_output(CAMERA_OV9726_RESET, 1);
	mdelay(7);

	tegra_camera_enable_mclk();

	/*
	 * success.
	 */
	pr_debug(KERN_DEBUG "%s: exit\n", __func__);

	return 0;
}


/*****************************************************************************
 *
 * Perform OV9726 (front camera) power off sequence.
 *
 * RETURN
 *
 * Returns zero if successful or non-zero if an error occurred.
 */
static int txs03_ov9726_power_off(void)
{
	pr_debug(KERN_DEBUG "%s: enter\n", __func__);

    /*
     * perform power-off sequence.
     */
    tegra_camera_disable_mclk();

    mdelay(2);
    gpio_direction_output(CAMERA_OV9726_RESET, 0);

    mdelay(2);
    gpio_direction_output(CAMERA_OV8820_ENABLE_1V8, 0);
    gpio_direction_output(CAMERA_OV9726_ENABLE, 0);
    gpio_direction_output(CAMERA_VDDIO_ENABLE, 1);


	/*
	 * success.
	 */
	pr_debug(KERN_DEBUG "%s: exit\n", __func__);

	return 0;
}


/*
 * structure for holding board details (queried on the fly at boot).
 */
static struct board_info board_info;

#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
static struct temp_threshold_table tegra_temp_threshold[] = {
	{ 85000, 85000 },
};

#if 0 //sample data
static struct temp_threshold_table tegra_temp_threshold[] = {
	{ 60000, 65000 },
	{ 50000, 75000 },
};
#endif //sample data

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_local_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_local_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;
	struct tegra_extra_throttle *extra_throttle;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}
	extra_throttle = kzalloc(sizeof(struct tegra_extra_throttle),
					GFP_KERNEL);
	if (!extra_throttle) {
		pr_err("unable to allocate thermal device\n");
		return;
	}
	extra_throttle->get_local_temp = nct_get_local_temp;
	extra_throttle->threshold_table_tj = (struct temp_threshold_table *)NULL;
	extra_throttle->threshold_table_size = 0;

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->throttle_ex = extra_throttle;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_set_device(thermal_device);
}
#endif

static struct nct1008_platform_data txs03_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
	.probe_callback = nct1008_probe_callback,
#endif
};

static struct i2c_board_info txs03_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &txs03_nct1008_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
	}
};

static int txs03_nct1008_init(void)
{
	int ret;

	ret = gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	if (ret < 0)
		return ret;
	ret = gpio_direction_input(NCT1008_THERM2_GPIO);
	if (ret < 0)
		gpio_free(NCT1008_THERM2_GPIO);
	else
		tegra_gpio_enable(NCT1008_THERM2_GPIO);

	return ret;
}

#ifdef CONFIG_INPUT_SX8636
static int txs03_sx8636_init(void)
{
	int ret;
	int value;

	printk(KERN_INFO "%s called\n", __func__);
	ret = gpio_request(TEGRA_GPIO_PC7, "sx8636_int");
	if (ret < 0)
		return ret;

	value = gpio_get_value(TEGRA_GPIO_PC7);

	ret = gpio_direction_input(TEGRA_GPIO_PC7);
	if (ret < 0)
		gpio_free(TEGRA_GPIO_PC7);
	else
		tegra_gpio_enable(TEGRA_GPIO_PC7);

	printk(KERN_INFO "%s ret = %d\n", __func__, ret);

	return ret;
}
#endif

static struct lis331dlh_platform_data txs03_lis331dlh_pdata = {
	.transformation_matrix = {
		 1,  0,  0,
		 0,  1,  0,
		 0,  0,  1,
	},
};

#ifdef SF_BUILD_AKMD_ENABLED
#if defined(CONFIG_INPUT_AK8975C)
#define AKM_COMPASS_DRDY_GPIO TEGRA_GPIO_PI7
static int txs03_akm8975_init(void)
{
	int ret;

	ret = gpio_request(AKM_COMPASS_DRDY_GPIO, "compass_drdy");
	if (ret < 0)
		return ret;
	ret = gpio_direction_input(AKM_COMPASS_DRDY_GPIO);
	if (ret < 0)
		gpio_free(AKM_COMPASS_DRDY_GPIO);
	else
		tegra_gpio_enable(AKM_COMPASS_DRDY_GPIO);

	return ret;
}

static struct akm8975_platform_data akm_platform_data_8975 = {
	.layout = 1,
};
#endif
#else
#if defined(CONFIG_INPUT_AK8975C)
static struct ak8975c_platform_data txs03_ak8975c_pdata = {
	.transformation_matrix = {
		 1,  0,  0,
		 0,  1,  0,
		 0,  0,  1,
		},
	};
#endif
#endif

static struct l3g4200dh_platform_data txs03_l3g4200dh_pdata = {
	.transformation_matrix = {
		 1,  0,  0,
		 0,  1,  0,
		 0,  0,  1,
		},
#ifdef CONFIG_NBX_L3G4200DH_HIGHSPEED
	.odr = 100,
#endif
	};

static struct i2c_board_info txs03_i2c_sensors_info[] = {
#ifdef CONFIG_INPUT_LIS331DLH
	{
		I2C_BOARD_INFO("lis331dlh", 0x18), /* LIS331DLH */
		.platform_data = &txs03_lis331dlh_pdata,
	},
#endif
#ifdef SF_BUILD_AKMD_ENABLED
#if defined(CONFIG_INPUT_AK8975C)
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x0c), /* AK8975C */
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &akm_platform_data_8975,
		.irq = TEGRA_GPIO_TO_IRQ(AKM_COMPASS_DRDY_GPIO),
	},
#endif
#else /* not SF_BUILD_AKMD_ENABLED */
#if defined(CONFIG_INPUT_AK8975C)
	{
		I2C_BOARD_INFO("ak8975c", 0x0c), /* AK8975C */
		.platform_data = &txs03_ak8975c_pdata,
	},
#endif
#endif

#ifdef CONFIG_INPUT_L3G4200DH
	{
		I2C_BOARD_INFO("l3g4200dh", 0x68), /* L3G4200DH */
		.platform_data = &txs03_l3g4200dh_pdata,
	},
#endif
#ifdef CONFIG_INPUT_SX8636
	{
		I2C_BOARD_INFO("sx8636", 0x2B),
		.platform_data = (void *)&sx8636_i2c_platform_data,		
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7),
	},
#endif
};


int __init txs03_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	pr_debug(KERN_DEBUG "%s: about to initialise camera GPIOs...\n", __func__);

	/*
	 * attempt to enable the camera-related GPIO pins.
	 */
	if (gpio_initialise_table(s_arrInitGpioDefs, ARRAY_SIZE(s_arrInitGpioDefs)) < 0)
	{
		pr_err("%s: failed to initialise GPIOs\n", __func__);
		return -1;
	}

	pr_debug(KERN_DEBUG "%s: successfully initialised camera GPIOs...\n", __func__);

	/*
	 * attempt to enable the camera-related GPIO pins.
	 */
	err = txs03_nct1008_init();
	if (err)
		return err;

#ifdef CONFIG_INPUT_SX8636
	err = txs03_sx8636_init();
	if (err)
		return err;
#endif

	i2c_register_board_info(4, txs03_i2c4_nct1008_board_info,
		ARRAY_SIZE(txs03_i2c4_nct1008_board_info));

	/*
	 * register cameras.
	 */
	i2c_register_board_info(2, txs03_camera_board_info, ARRAY_SIZE(txs03_camera_board_info));
	platform_device_register(&ov8820_device);
	platform_device_register(&ov8820_focuser_device);

#if defined(SF_BUILD_AKMD_ENABLED) && defined(CONFIG_INPUT_AK8975C)
	err = txs03_akm8975_init();
	if (err)
		return err;
#endif
#if defined(CONFIG_INPUT_LIS331DLH) || defined(CONFIG_INPUT_L3G4200DH) || defined(CONFIG_INPUT_AK8975C)
	i2c_register_board_info(0, txs03_i2c_sensors_info, ARRAY_SIZE(txs03_i2c_sensors_info));
#endif

	return 0;
}

