/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * arch/arm/mach-tegra/board-txs03-pinmux.c
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/pinmux.h>
#include "board.h"
#include "board-txs03.h"
#include "gpio-names.h"

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* !!!FIXME!!!! POPULATE THIS TABLE */
static __initdata struct tegra_drive_pingroup_config txs03_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SET_DRIVE(ATA, DISABLE, DISABLE, DIV_1, 31, 31, FAST, FAST) */
	SET_DRIVE(DAP2, 	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* All I2C pins are driven to maximum drive strength */
	/* GEN1 I2C */
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GEN2 I2C */
	SET_DRIVE(AT5,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* CAM I2C */
	SET_DRIVE(GME,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* DDC I2C */
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* PWR_I2C */
	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* UART3 */
	SET_DRIVE(UART3,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* SDMMC1 */
	SET_DRIVE(SDIO1,	DISABLE, DISABLE, DIV_1, 46, 42, FAST, FAST),

	/* SDMMC3 */
	SET_DRIVE(SDIO3,	DISABLE, DISABLE, DIV_1, 46, 42, FAST, FAST),

	/* SDMMC4 */
	SET_DRIVE(GMA,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),
	SET_DRIVE(GMB,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),
	SET_DRIVE(GMC,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),
	SET_DRIVE(GMD,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),

	/* Camera */
	SET_DRIVE(GMG,		DISABLE, DISABLE, DIV_2, 15, 15, SLOWEST, SLOWEST),
};

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

static __initdata struct tegra_pingroup_config txs03_pinmux_common[] = {
	/* SDMMC1 pinmux */
	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT3,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT2,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT1,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),

	/* SDMMC3 pinmux */
	DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),

	/* SDMMC4 pinmux */
	DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_RST_N,    RSVD1,           NORMAL,     TRISTATE,   INPUT),

	/* I2C1 pinmux */
	I2C_PINMUX(GEN1_I2C_SCL,	I2C1,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(GEN1_I2C_SDA,	I2C1,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* I2C2 pinmux */
	I2C_PINMUX(GEN2_I2C_SCL,	I2C2,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(GEN2_I2C_SDA,	I2C2,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* I2C3 pinmux */
	I2C_PINMUX(CAM_I2C_SCL,		I2C3,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(CAM_I2C_SDA,		I2C3,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* I2C4 pinmux */
	I2C_PINMUX(DDC_SCL,		I2C4,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(DDC_SDA,		I2C4,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* Power I2C pinmux */
	I2C_PINMUX(PWR_I2C_SCL,		I2CPWR,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(PWR_I2C_SDA,		I2CPWR,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	DEFAULT_PINMUX(ULPI_DATA3,      ULPI,            PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_DATA4,      SPI2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_DATA5,      SPI2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_DATA6,      SPI2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_DATA7,      SPI2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_CLK,        UARTD,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DIR,        UARTD,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_NXT,        UARTD,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_STP,        UARTD,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(DAP3_FS,         I2S2,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(DAP3_DIN,        I2S2,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_PWR2,        DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_SCK,         DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_PCLK,        DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_DE,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_HSYNC,       DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_VSYNC,       DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_M1,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D0,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D1,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D2,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D3,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D4,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D5,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D6,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D7,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D8,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D9,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D10,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D11,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D12,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D13,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D14,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D15,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D16,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D17,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_DC1,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(CRT_VSYNC,       CRT,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D0,           RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D1,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D2,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D3,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D4,           VI,              NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(VI_D5,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D7,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D10,          RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_MCLK,         VI,              PULL_UP,   NORMAL,     INPUT),

	DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART3_RXD,       UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PU0,        RSVD1,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PU2,        RSVD1,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PU3,        RSVD1,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PU4,        PWM1,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PU5,        PWM2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PU6,        RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_FS,         I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_DIN,        I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_DOUT,       I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_SCLK,       I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_WP_N,        GMI,             PULL_UP,   NORMAL,     INPUT),

	DEFAULT_PINMUX(GMI_A17,         SPI4,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GMI_A19,         SPI4,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CAM_MCLK,        VI_ALT2,         PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PCC1,       RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PBB0,       RSVD1,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB5,       VGP5,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PBB7,       I2S4,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(JTAG_RTCK,       RTCK,            NORMAL,    NORMAL,     OUTPUT),

	/*  KBC keys */
	DEFAULT_PINMUX(KB_COL0,         KBC,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL1,         KBC,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PV0,        RSVD,            NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(CLK_32K_OUT,     BLINK,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(CLK1_REQ,        DAP,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CLK1_OUT,        EXTPERIPH1,      NORMAL,    NORMAL,     OUTPUT),

	DEFAULT_PINMUX(DAP2_FS,         I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DIN,        I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(SPI2_CS1_N,      SPI2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SPI2_CS2_N,      SPI2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L0_PRSNT_N,  PCIE,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L0_RST_N,    PCIE,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(PEX_L0_CLKREQ_N, PCIE,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L1_PRSNT_N,  PCIE,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L1_RST_N,    PCIE,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(PEX_L1_CLKREQ_N, PCIE,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(HDMI_CEC,        CEC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(HDMI_INT,        RSVD0,           NORMAL,    NORMAL,     INPUT),

	/* Gpios */
	/* SDMMC1 CD gpio */
	DEFAULT_PINMUX(GMI_IORDY,       RSVD1,           NORMAL,    NORMAL,     INPUT),
	/* SDMMC1 WP gpio */
	DEFAULT_PINMUX(VI_D11,          RSVD1,           PULL_UP,   NORMAL,     INPUT),
	/* Touch panel GPIO */
	/* Touch IRQ */
	DEFAULT_PINMUX(GMI_AD12,        NAND,            NORMAL,    NORMAL,     INPUT),

	/* Touch RESET */
	DEFAULT_PINMUX(GMI_AD14,        NAND,            NORMAL,    NORMAL,     OUTPUT),

	/* Touch Power Control 0 */
	DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,    NORMAL,     INPUT),

	/* Power rails GPIO */
	DEFAULT_PINMUX(GPIO_PBB4,       VGP4,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT5,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT),
	DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT),

	VI_PINMUX(VI_D6,           VI,              NORMAL,    NORMAL,     OUTPUT, DISABLE, DISABLE),
	VI_PINMUX(VI_D8,           SDMMC2,          NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_D9,           SDMMC2,          NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_HSYNC,        RSVD1,           NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_VSYNC,        RSVD1,           NORMAL,    NORMAL,     INPUT,  DISABLE, DISABLE),
};

static __initdata struct tegra_pingroup_config txs03_pinmux[] = {
	DEFAULT_PINMUX(LCD_CS0_N,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_CS1_N,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           NORMAL,    NORMAL,     OUTPUT), /* gpio */
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            PULL_DOWN, NORMAL,     OUTPUT), /* LCD1_BL_PWM */
	DEFAULT_PINMUX(KB_ROW4,         KBC,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW5,         KBC,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW6,         KBC,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW7,         KBC,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW8,         KBC,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW9,         KBC,             PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(KB_ROW10,        KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PV1,        RSVD,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_RXD,       IRDA,            PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_TXD,       IRDA,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART2_RTS_N,     UARTA,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART2_CTS_N,     UARTA,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD13,        NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS1_N,       NAND,            PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS2_N,       NAND,            PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS3_N,       NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS4_N,       NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS6_N,       NAND,            PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_CS7_N,       NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_DQS,         NAND,            PULL_DOWN, NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GMI_WAIT,        NAND,            NORMAL,    NORMAL,     INPUT),
};

static __initdata struct tegra_pingroup_config unused_pins_txs03[] = {
	DEFAULT_PINMUX(SDMMC3_DAT6,     SDMMC3,          NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA0,      ULPI,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA1,      ULPI,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(ULPI_DATA2,      ULPI,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(DAP3_SCLK,       I2S2,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PV2,        OWR,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PV3,        RSVD1,           NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(CLK2_OUT,        EXTPERIPH2,      NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(CLK2_REQ,        DAP,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(LCD_PWR1,        DISPLAYA,        NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(LCD_SDIN,        DISPLAYA,        NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYA,        NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(LCD_WR_N,        DISPLAYA,        NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(LCD_DC0,         DISPLAYA,        NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(LCD_PWR0,        DISPLAYA,        NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(CRT_HSYNC,       CRT,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PU1,        RSVD1,           NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(CLK3_OUT,        EXTPERIPH3,      NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(CLK3_REQ,        DEV3,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD0,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD1,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD10,        NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD11,        NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD15,        NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD2,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD3,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD4,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD5,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD6,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD7,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_AD9,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_ADV_N,       NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_CLK,         NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_CS0_N,       NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_OE_N,        NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_WR_N,        NAND,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_A16,         SPI4,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_A18,         SPI4,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB3,       VGP3,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB6,       VGP6,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GPIO_PCC2,       I2S4,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW0,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW1,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW11,        KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW12,        KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW13,        KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW14,        KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW15,        KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW2,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_ROW3,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_COL3,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_COL4,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_COL5,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_COL6,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(KB_COL7,         KBC,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SYS_CLK_REQ,     SYSCLK,          NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(OWR,             OWR,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,    TRISTATE,   INPUT),
	DEFAULT_PINMUX(SPI2_MISO,       GMI,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI2_MOSI,       GMI,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI2_SCK,        GMI,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI1_SCK,        SPI1,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI1_MISO,       SPI1,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(PEX_WAKE_N,      PCIE,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(PEX_L2_PRSNT_N,  PCIE,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(PEX_L2_CLKREQ_N, PCIE,            NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPDIF_OUT,       SPDIF,           NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(SPI2_CS0_N,      GMI,             NORMAL,    TRISTATE,   OUTPUT),
	DEFAULT_PINMUX(GMI_RST_N,       RSVD3,           NORMAL,    TRISTATE,   OUTPUT),

	//	VI_PINMUX(VI_PCLK,         RSVD1,           PULL_UP,   TRISTATE,   INPUT,  DISABLE, DISABLE),

};

/* GEN2_I2C leak issue */
static int gen2_i2c_scl_handle = -1;
static int gen2_i2c_sda_handle = -1;

static struct tegra_pingroup_config txs03_pinmux_i2c_gen2_suspend_table[] = {
	/* I2C2 pinmux */
	I2C_PINMUX(GEN2_I2C_SCL,        I2C2,           NORMAL, NORMAL, INPUT,  DISABLE,        ENABLE),
	I2C_PINMUX(GEN2_I2C_SDA,        I2C2,           NORMAL, NORMAL, INPUT,  DISABLE,        ENABLE),
};
static struct tegra_pingroup_config txs03_pinmux_i2c_gen2_resume_table[] = {
	/* I2C2 pinmux */
	I2C_PINMUX(GEN2_I2C_SCL,        I2C2,           NORMAL, NORMAL, INPUT,  DISABLE,        ENABLE),
	I2C_PINMUX(GEN2_I2C_SDA,        I2C2,           NORMAL, NORMAL, INPUT,  DISABLE,        ENABLE),
};

int txs03_pinmux_i2c_gen2_suspend(void){
	tegra_pinmux_config_table(txs03_pinmux_i2c_gen2_suspend_table,
				  ARRAY_SIZE(txs03_pinmux_i2c_gen2_suspend_table));

	gen2_i2c_scl_handle = gpio_request(TEGRA_GPIO_PT5, "I2C GEN2 SCL");
	if(gen2_i2c_scl_handle<0) pr_err("gpio_request err gen2_i2c_scl\n");
	gen2_i2c_sda_handle = gpio_request(TEGRA_GPIO_PT6, "I2C GEN2 SDA");
	if(gen2_i2c_sda_handle<0) pr_err("gpio_request err gen2_i2c_sda\n");

	tegra_gpio_enable(TEGRA_GPIO_PT5);
	tegra_gpio_enable(TEGRA_GPIO_PT6);

	gpio_direction_output(TEGRA_GPIO_PT5, 0);
	gpio_direction_output(TEGRA_GPIO_PT6, 0);
	return 0;
}

int txs03_pinmux_i2c_gen2_resume(void){
	if(gen2_i2c_scl_handle>=0){
		tegra_gpio_disable(TEGRA_GPIO_PT5);
		gpio_free(TEGRA_GPIO_PT5);
		gen2_i2c_scl_handle = -1;
	}
	if(gen2_i2c_sda_handle>=0){
		tegra_gpio_disable(TEGRA_GPIO_PT6);
		gpio_free(TEGRA_GPIO_PT6);
		gen2_i2c_sda_handle = -1;
	}
	tegra_pinmux_config_table(txs03_pinmux_i2c_gen2_resume_table,
				  ARRAY_SIZE(txs03_pinmux_i2c_gen2_resume_table));
	return 0;
}

#define GPIO_INIT_PIN_MODE(_gpio, _is_input, _value)	\
	{					\
		.gpio_nr	= _gpio,	\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

/* NBX0313 */
static struct gpio_init_pin_info init_gpio_mode_nbx0313[] = {
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PH5, false, 0),
	//	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PH6, false, 1),
	//GPIO_INIT_PIN_MODE(TEGRA_GPIO_PN0, false, 1),
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PU5, true,  0),
	// GPIO for Audio
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PW3, true,  0), // CDC_IRQ
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PW2, true,  0), // CDC_SHRT
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PJ2, true,  0), // HP_DET
};

static void __init txs03_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_nbx0313);
	pins_info = init_gpio_mode_nbx0313;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init txs03_pinmux_init(void)
{
	txs03_gpio_init_configure();  /* FIXME */

	tegra_pinmux_config_table(txs03_pinmux_common, ARRAY_SIZE(txs03_pinmux_common));
	tegra_drive_pinmux_config_table(txs03_drive_pinmux,
					ARRAY_SIZE(txs03_drive_pinmux));

	tegra_pinmux_config_table(txs03_pinmux,
				  ARRAY_SIZE(txs03_pinmux));
	tegra_pinmux_config_table(unused_pins_txs03,
				  ARRAY_SIZE(unused_pins_txs03));

	return 0;
}

#define PIN_GPIO_LPM(_name, _gpio, _is_input, _value)	\
	{					\
		.name		= _name,	\
		.gpio_nr	= _gpio,	\
		.is_gpio	= true,		\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

/* E1198 without PM313 display board */

static void set_unused_pin_gpio(struct gpio_init_pin_info *lpm_pin_info,
		int list_count)
{
	int i;
	struct gpio_init_pin_info *pin_info;
	int ret;

	for (i = 0; i < list_count; ++i) {
		pin_info = (struct gpio_init_pin_info *)(lpm_pin_info + i);
		if (!pin_info->is_gpio)
			continue;

		ret = gpio_request(pin_info->gpio_nr, pin_info->name);
		if (ret < 0) {
			pr_err("%s() Error in gpio_request() for gpio %d\n",
					__func__, pin_info->gpio_nr);
			continue;
		}
		if (pin_info->is_input)
			ret = gpio_direction_input(pin_info->gpio_nr);
		else
			ret = gpio_direction_output(pin_info->gpio_nr,
							pin_info->value);
		if (ret < 0) {
			pr_err("%s() Error in setting gpio %d to in/out\n",
				__func__, pin_info->gpio_nr);
			gpio_free(pin_info->gpio_nr);
			continue;
		}
		tegra_gpio_enable(pin_info->gpio_nr);
	}
}

/* Initialize the pins to desired state as per power/asic/system-eng
 * recomendation */
int __init txs03_pins_state_init(void)
{
	return 0;
}
