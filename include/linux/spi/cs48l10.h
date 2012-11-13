/* 2012-07-20: File added and changed by Sony Corporation */
/*
 * cs48L10.h  --  CS48L10/CHAS DSP driver
 *
 * Copyright 2010 Cirrus Logic, Inc.
 *
 * Author: Georgi Vlaev, Nucleus Systems Ltd. <office@nucleusys.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __LINUX_SPI_CS48L10_H__
#define __LINUX_SPI_CS48L10_H__


/*
    Platform data / DSP GPIOs
*/
struct cs48l10_platform_data 
{
	int gpio_busy;
	int gpio_int;
	int gpio_reset;
};



#define CS48L10IO                       0xE0

#define CS48L10_INVALID_FORMAT          -1
#define CS48L10_FORMAT_DEFAULT          0
#define CS48L10_FORMAT_PCM              0x00000000

/* GPIO's for interrupt, reset and busy */
#include "../../../arch/arm/mach-tegra/gpio-names.h"

#define TEGRA_CS48L10_RESET_GPIO TEGRA_GPIO_PK6
#define TEGRA_CS48L10_BUSY_GPIO TEGRA_GPIO_PW0
#define TEGRA_CS48L10_INT_GPIO TEGRA_GPIO_PN4

/* Number of 32 bit words that can be transfered in a single write. */
#define DSP_WORD_TRASFER_SIZE 128

/* These two defines below must be defined to the desired SPI clock 
frequency for both loading firmware (prekickstart) and after FW is running */


/*Boot assist parameters */

#define CS48L10_BOOT_ASSIST_PROJECT_NAME "boot_assist"
//#define CS48L10_DEFAULT_PROJECT_ID	0
#define CS48L10_DEFAULT_PROJECT_ID	1  // for SonyFW64fs

#define CS48L10_MAX_SPI 25000000 /* 25MHz */
#define CS48L10_MIN_SPI 1000000  /* 1 MHz */

#define CS48L10_NOBUSY

/* Unsolicicited Messages */
#define CS48L10_MALLOC_ERR 0x81000000
#define CS48L10_PLL_ERR 0x81000002


struct cs48l10_hostif_config {
	__u32 packet_unit_size;
	__u32 data_buf_size;
	__u32 watermark_level;
};

/* project/firmware info */
#define CS48L10_GET_PROJECT     _IO(CS48L10IO,   0x10)
#define CS48L10_SET_PROJECT     _IO(CS48L10IO,   0x11)
#define CS48L10_GET_HOSTIF_CONFIG     _IOR(CS48L10IO, 0x12, \
		    struct cs48l10_hostif_config)
#define CS48L10_SET_HOSTIF_CONFIG     _IOW(CS48L10IO, 0x13, \
		    struct cs48l10_hostif_config)


#define DSP_BUFFER_SIZE                0x6000  /* dsp words */
#define DSP_MIN_PUS            16      /* min packet unit size (dsp words) */
#define DSP_MAX_PUS            256     /* max packet unit size (dsp words) */
#define CS48L10_BUFFER_SIZE    96*1024 /* 96 KB ...*/

/* DSP Modules */
#define DSP_OS         0x01    /* OS module */
#define DSP_AM         0x03    /* Audio Manager */

#define RW_WR          0       /* overwite current value */
#define RW_WROR                1       /* new value or'd with current value */
#define RW_WRAND       2       /* new value and'd with current value */
#define RW_RD          3       /* read value */

/* mod := dsp module id, rw := RW, wc := wordcount - 1*/
#define MK_OP(mod, rw, wc) \
       ((((mod & 0x7F) | 0x80) << 8) | ((rw & 3) << 6) | (wc & 0x1F))
#define MK_OPWR(mod)   MK_OP(mod, RW_WR, 0)
#define MK_OPRD(mod)   MK_OP(mod, RW_RD, 0)

#define MK_CMD(op,idx) ((op << 16) | (idx & 0xFFFF)) /* command - op, index*/
#define MK_RES(op)     (op & 0x7FFF)   /* response*/

/* OS Indexes */
#define KICKSTART      0x0000 /* KICKSTART */
#define IO_CONFIG      0x0001 /* IO_CONFIG*/
#define SOFTBOOT       0x0009 /* SOFTBOOT */

/* OP codes */
#define OP_OS_URES     0x8100 /* Unsol Response */



#endif /* __LINUX_SPI_CS48L10_H__ */
