/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-txs03.h
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

#ifndef _MACH_TEGRA_BOARD_TXS03_H
#define _MACH_TEGRA_BOARD_TXS03_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/ricoh583.h>

/* Processor Board  ID */
#define BOARD_E1187   0x0B57
#define BOARD_E1186   0x0B56
#define BOARD_E1198   0x0B62
#define BOARD_E1256   0x0C38
#define BOARD_E1257   0x0C39
#define BOARD_E1291   0x0C5B
#define BOARD_PM267   0x0243
#define BOARD_PM269   0x0245
#define BOARD_E1208   0x0C08
#define BOARD_PM305   0x0305
#define BOARD_PM311   0x030B

#define BOARD_PMU_PM299   0x0263

/* SKU Information */
#define SKU_DCDC_TPS62361_SUPPORT	0x1
#define SKU_SLT_ULPI_SUPPORT		0x2
#define SKU_T30S_SUPPORT		0x4
#define SKU_TOUCHSCREEN_MECH_FIX	0x10

#define SKU_TOUCH_MASK			0xFF00
#define SKU_TOUCH_2000			0x0B00

#define SKU_MEMORY_TYPE_BIT		0x4
#define SKU_MEMORY_TYPE_MASK		0x3
/* If BOARD_PM269 */
#define SKU_MEMORY_SAMSUNG_EC		0x0
#define SKU_MEMORY_ELPIDA		0x1
#define SKU_MEMORY_SAMSUNG_EB		0x2
/* If other BOARD_ variants */
#define SKU_MEMORY_TXS03_1GB_1R	0x0
#define SKU_MEMORY_TXS03_2GB_2R	0x1
#define SKU_MEMORY_TXS03_2GB_1R	0x2
#define MEMORY_TYPE(sku) (((sku) >> SKU_MEMORY_TYPE_BIT) & SKU_MEMORY_TYPE_MASK)

/* Board Fab version */
#define BOARD_FAB_A00			0x0
#define BOARD_FAB_A01			0x1
#define BOARD_FAB_A02			0x2
#define BOARD_FAB_A03			0x3
#define BOARD_FAB_A04			0x4
#define BOARD_FAB_A05			0x5

/* Display Board ID */
#define BOARD_DISPLAY_PM313		0x030D
#define BOARD_DISPLAY_E1247		0x0C2F

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)

/* THERMAL SENSOR NCT1008 GPIO */
#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PI3

/* RICOH583 GPIO */
#define RICOH583_GPIO_BASE	TEGRA_NR_GPIOS
#define RICOH583_GPIO_END	(RICOH583_GPIO_BASE + 8)

/* MAX77663 GPIO */
#define MAX77663_GPIO_BASE	TEGRA_NR_GPIOS
#define MAX77663_GPIO_END	(MAX77663_GPIO_BASE + MAX77663_GPIO_NR)

/* PMU_TCA6416 GPIOs */
#define PMU_TCA6416_GPIO_BASE	(TPS6591X_GPIO_END)
#define PMU_TCA6416_GPIO_PORT00	(PMU_TCA6416_GPIO_BASE + 0)
#define PMU_TCA6416_GPIO_PORT01	(PMU_TCA6416_GPIO_BASE + 1)
#define PMU_TCA6416_GPIO_PORT02	(PMU_TCA6416_GPIO_BASE + 2)
#define PMU_TCA6416_GPIO_PORT03	(PMU_TCA6416_GPIO_BASE + 3)
#define PMU_TCA6416_GPIO_PORT04	(PMU_TCA6416_GPIO_BASE + 4)
#define PMU_TCA6416_GPIO_PORT05	(PMU_TCA6416_GPIO_BASE + 5)
#define PMU_TCA6416_GPIO_PORT06	(PMU_TCA6416_GPIO_BASE + 6)
#define PMU_TCA6416_GPIO_PORT07	(PMU_TCA6416_GPIO_BASE + 7)
#define PMU_TCA6416_GPIO_PORT10	(PMU_TCA6416_GPIO_BASE + 8)
#define PMU_TCA6416_GPIO_PORT11	(PMU_TCA6416_GPIO_BASE + 9)
#define PMU_TCA6416_GPIO_PORT12	(PMU_TCA6416_GPIO_BASE + 10)
#define PMU_TCA6416_GPIO_PORT13	(PMU_TCA6416_GPIO_BASE + 11)
#define PMU_TCA6416_GPIO_PORT14	(PMU_TCA6416_GPIO_BASE + 12)
#define PMU_TCA6416_GPIO_PORT15	(PMU_TCA6416_GPIO_BASE + 13)
#define PMU_TCA6416_GPIO_PORT16	(PMU_TCA6416_GPIO_BASE + 14)
#define PMU_TCA6416_GPIO_PORT17	(PMU_TCA6416_GPIO_BASE + 15)
#define PMU_TCA6416_GPIO_END	(PMU_TCA6416_GPIO_BASE + 16)

/* PMU_TCA6416 GPIO assignment */
#define EN_HSIC_GPIO				PMU_TCA6416_GPIO_PORT11 /* PMU_GPIO25 */
#define PM267_SMSC4640_HSIC_HUB_RESET_GPIO	PMU_TCA6416_GPIO_PORT17 /* PMU_GPIO31 */

/* CAM_TCA6416 GPIOs */
#define CAM_TCA6416_GPIO_BASE		PMU_TCA6416_GPIO_END
#define CAM1_PWR_DN_GPIO			CAM_TCA6416_GPIO_BASE + 0
#define CAM1_RST_L_GPIO				CAM_TCA6416_GPIO_BASE + 1
#define CAM1_AF_PWR_DN_L_GPIO		CAM_TCA6416_GPIO_BASE + 2
#define CAM1_LDO_SHUTDN_L_GPIO		CAM_TCA6416_GPIO_BASE + 3
#define CAM2_PWR_DN_GPIO			CAM_TCA6416_GPIO_BASE + 4
#define CAM2_RST_L_GPIO				CAM_TCA6416_GPIO_BASE + 5
#define CAM2_AF_PWR_DN_L_GPIO		CAM_TCA6416_GPIO_BASE + 6
#define CAM2_LDO_SHUTDN_L_GPIO		CAM_TCA6416_GPIO_BASE + 7
#define CAM_FRONT_PWR_DN_GPIO		CAM_TCA6416_GPIO_BASE + 8
#define CAM_FRONT_RST_L_GPIO		CAM_TCA6416_GPIO_BASE + 9
#define CAM_FRONT_AF_PWR_DN_L_GPIO	CAM_TCA6416_GPIO_BASE + 10
#define CAM_FRONT_LDO_SHUTDN_L_GPIO	CAM_TCA6416_GPIO_BASE + 11
#define CAM_FRONT_LED_EXP			CAM_TCA6416_GPIO_BASE + 12
#define CAM_SNN_LED_REAR_EXP		CAM_TCA6416_GPIO_BASE + 13
/* PIN 19 NOT USED and is reserved */
#define CAM_NOT_USED				CAM_TCA6416_GPIO_BASE + 14
#define CAM_I2C_MUX_RST_EXP			CAM_TCA6416_GPIO_BASE + 15
#define CAM_TCA6416_GPIO_END		CAM_TCA6416_GPIO_BASE + 16

/* WM8903 GPIOs */
#define TXS03_GPIO_WM8903(_x_)		(CAM_TCA6416_GPIO_END + (_x_))
#define TXS03_GPIO_WM8903_END		TXS03_GPIO_WM8903(4)

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PW3
#define TEGRA_GPIO_CDC_SHRT		TEGRA_GPIO_PW2
#define TEGRA_GPIO_SPKR_EN		TXS03_GPIO_WM8903(2)
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PJ2

/* CAMERA RELATED GPIOs on TXS03 */
#define OV5650_RESETN_GPIO              TEGRA_GPIO_PBB0
#define CAM1_POWER_DWN_GPIO             TEGRA_GPIO_PBB5
#define CAM2_POWER_DWN_GPIO             TEGRA_GPIO_PBB6
#define CAM3_POWER_DWN_GPIO             TEGRA_GPIO_PBB7
#define CAMERA_CSI_CAM_SEL_GPIO         TEGRA_GPIO_PBB4
#define CAMERA_CSI_MUX_SEL_GPIO         TEGRA_GPIO_PCC1
#define CAM1_LDO_EN_GPIO                TEGRA_GPIO_PR6
#define CAM2_LDO_EN_GPIO                TEGRA_GPIO_PR7
#define CAM3_LDO_EN_GPIO                TEGRA_GPIO_PS0
#define OV14810_RESETN_GPIO             TEGRA_GPIO_PBB0

#define CAMERA_VDDIO_ENABLE			TEGRA_GPIO_PD0  /* active low */
#define CAMERA_MCLK			TEGRA_GPIO_PCC0  /* active high */
#define CAMERA_OV8820_RESET			TEGRA_GPIO_PBB0  /* active low */
#define CAMERA_OV8820_ENABLE_1V8		TEGRA_GPIO_PR5   /* active high */
#define CAMERA_OV8820_ENABLE_1V2		TEGRA_GPIO_PR4   /* active high - not used */
#define CAMERA_OV8820_ENABLE_2V8		TEGRA_GPIO_PR6   /* active high */
#define CAMERA_OV8820_ENABLE_2V8_VCM		TEGRA_GPIO_PR7   /* active high */
#define CAMERA_OV8820_POWERDWN			TEGRA_GPIO_PBB5  /* active low */
#define CAMERA_OV9726_RESET			TEGRA_GPIO_PBB7  /* active low */
#define CAMERA_OV9726_ENABLE			TEGRA_GPIO_PBB4  /* active high */
#define CAMERA_OV9726_POWERDWN                  TEGRA_GPIO_PCC1  /* active high */     

#define CAMERA_FLASH_SYNC_GPIO		TEGRA_GPIO_PBB3
#define CAMERA_FLASH_MAX_TORCH_AMP	7
#define CAMERA_FLASH_MAX_FLASH_AMP	7

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE	6
#define PCA954x_I2C_BUS0	(PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1	(PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2	(PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3	(PCA954x_I2C_BUS_BASE + 3)

#define AC_PRESENT_GPIO		TPS6591X_GPIO_4

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)

#define AC_PRESENT_INT          (TPS6591X_INT_GPIO4 + TPS6591X_IRQ_BASE)

/* RICOH583 IRQs */
#define RICOH583_IRQ_BASE	TEGRA_NR_IRQS
#define RICOH583_IRQ_END	(RICOH583X_IRQ_BASE + RICOH583_NR_IRQS)
#define DOCK_DETECT_GPIO        TEGRA_GPIO_PU4

/* MAX77663 IRQs */
#define MAX77663_IRQ_BASE	TEGRA_NR_IRQS
#define MAX77663_IRQ_END	(MAX77663_IRQ_BASE + MAX77663_IRQ_NR)

int txs03_charge_init(void);
int txs03_regulator_init(void);
int txs03_suspend_init(void);
int txs03_sdhci_init(void);
int txs03_pinmux_init(void);
int txs03_panel_init(void);
int txs03_sensors_init(void);
int txs03_kbc_init(void);
int txs03_scroll_init(void);
int txs03_keys_init(void);
int txs03_gpio_switch_regulator_init(void);
int txs03_pins_state_init(void);
int txs03_emc_init(void);
int txs03_power_off_init(void);
int txs03_edp_init(void);
int txs03_pmon_init(void);
int txs03_pm298_gpio_switch_regulator_init(void);
int txs03_pm298_regulator_init(void);
int txs03_pm299_gpio_switch_regulator_init(void);
int txs03_pm299_regulator_init(void);
int touch_init(void);

/* Tentative */
/* Atmel TP Power Gpio Function */
int atmel_gpio_power_init(void);
void atmel_gpio_power_free(void);
int atmel_gpio_power_control( int on );

/* CABC Functions */
void txs03_panel_cabc_enable(void);
void txs03_panel_cabc_disable(void);
int txs03_panel_cabc_get_status(void);

/* GEN2_I2C pinmux suspend/resume function */
int txs03_pinmux_i2c_gen2_suspend(void);
int txs03_pinmux_i2c_gen2_resume(void);

/* Throttling Sysfs Functions */
int tegra_get_temp_throttle(long *val);
int tegra_set_temp_throttle(long val);

#define TOUCH_GPIO_IRQ_ATMEL_T9 TEGRA_GPIO_PK2
#define TOUCH_GPIO_RST_ATMEL_T9 TEGRA_GPIO_PH6
#define TOUCH_GPIO_POW_ATMEL_T9 TEGRA_GPIO_PN0
#define TOUCH_BUS_ATMEL_T9      1

/* Baseband GPIO addresses */
#define BB_GPIO_BB_EN			TEGRA_GPIO_PR5
#define BB_GPIO_BB_RST			TEGRA_GPIO_PS4
#define BB_GPIO_SPI_INT			TEGRA_GPIO_PS6
#define BB_GPIO_SPI_SS			TEGRA_GPIO_PV0
#define BB_GPIO_AWR			TEGRA_GPIO_PS7
#define BB_GPIO_CWR			TEGRA_GPIO_PU5

#define XMM_GPIO_BB_ON			BB_GPIO_BB_EN
#define XMM_GPIO_BB_RST			BB_GPIO_BB_RST
#define XMM_GPIO_IPC_HSIC_ACTIVE	BB_GPIO_SPI_INT
#define XMM_GPIO_IPC_HSIC_SUS_REQ	BB_GPIO_SPI_SS
#define XMM_GPIO_IPC_BB_WAKE		BB_GPIO_AWR
#define XMM_GPIO_IPC_AP_WAKE		BB_GPIO_CWR

#define TDIODE_OFFSET    (10000)   /* in millicelsius */

#endif
