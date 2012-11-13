/* 2012-07-20: File added and changed by Sony Corporation */
/*                                                                    
 * Audio  * cs48L10.c  --  CS48L10 DSP driver
 *
 * Copyright 2010 Cirrus Logic, Inc.
 * Copyright (C) 2012 Sony Corporation
 *
 * Author: Paul Handrigan <phandrigan@cirrus.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/cs48l10.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/time.h>

#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>


#define CS48L10_DRV_VERSION			"0.4.1"
#define CS48L10_SPI_WRITE_CMD		0x80
#define CS48L10_SPI_READ_CMD		0x81
#define CS48L10_SPI_BUSY_WAIT_USEC	(10)	/* usec */
#define CS48L10_SPI_INT_WAIT_MSEC	(2)	/* msec */

#define CS48L10_FIRMWARE_NAME		"cs48l10.bin"

#define MASTER_I2C              0x00
#define MASTER_SPI1             0x08
#define MASTER_SPI2             0x01
#define MASTER_SPI3             0x09
#define SLAVE_I2C               0x04
#define SLAVE_SPI               0x05

#define PREKICKSTART_CFG        0x00
#define KICKSTART_CFG           0x01
#define INITIAL_CFG		0x02

/* Chas response codes */
#define CS48L10_BOOT_START 		0x00000001
#define CS48L10_BOOT_SUCCESS 		0x00000002
#define CS48L10_APP_START 		0x00000004
#define CS48L10_BOOT_READY		0x00000005

#define CS48L10_DEFAULT_FIRMWARE_NAME		"cs48l10.bin"

#define CS48L10_SLEEP_SNAPSHOT_NAME             "sleep"

#define CS48L10_WAKE_SNAPSHOT_NAME             "wake"
#define CS48L10_MAX_SNAPSHOTS 100

#define FLASH_ADDRESS_INVALID (flash_address_t)0
#define FLASH_IMAGE_MARKER (u32)0x1A2B3C4D

/* slots for OS/MPM/VPM/PPM */
#define MAX_ULDS_PER_SET 5

/** maximum number of characters that can be used for display names.
 *
 * MAX_DISPLAY_LENGTH%sizeof(u32) must == 0, for checksum calculation to be correct
 */
#define MAX_DISPLAY_LENGTH 15

/* Address offset for the version id of the firmware. */
#define FW_VERSION_OFFSET 0x20

/* retry counter for fw load */
#define FW_LOAD_RETRY_COUNT 5

/* #undef CS48L10_ASYNC_FWLOADER */	
//#define CS48L10_ASYNC_FWLOADER	

/* DSP Boot Loader Write Messages: */
static const u32 Slave_Boot = 0x80000000;
static const u32 Soft_Reset = 0x40000000;
static const u32 Soft_Boot[] = { 0x81000009, 0x00000001 };

/* Sleep Mode Command */
static const u32 Sleep_cmd[] = {0x81000009, 0x00000100};

/* Wake up from sleep Command */
static const u32 Wakeup_cmd[] = {0x81000009, 0x00000000};

/* Mute Command */
static const u32 Mute_cmd[] = {0x83000001, 0x00000001};

/* Unmute Command */
static const u32 Unmute_cmd[] = {0x83000001, 0x00000000};

/* a flash image pointer to a .cfg set of words. */
struct cfg_ptr {
	u32 cfg_length;			
	u32 cfg_ptr;	
};

struct uld_ptr{
	u32 uld_length;		
	u32 uld_ptr;	
};

/* all the information about a specific project uld set in flash image. */
 
struct uld_set{
	struct uld_ptr uld_address[MAX_ULDS_PER_SET];
	u32 memory_configuration;	
	u32 memory_map;
};

struct project_str {
	u8 input_src;
	u8 input_fs;
	u8 output_fs;
	u8 reserved;
	struct uld_set uld_set;
	struct cfg_ptr snapshot[];	/* first two snapshots are prekick and initial */
};


struct project_master_list_str {
	u32 num_projects;
	u32 cfgs_per_project;
	u32 row_size;
	struct project_str project;
};

/* data structure containing the displayable name for the project and all snapshots.
 *
 *  Names for the prekick, kickstart, and initial snapshots are NOT considered displayable.
 */
struct project_display {
	char project_name[MAX_DISPLAY_LENGTH+1];
	char snapshot_name[CS48L10_MAX_SNAPSHOTS][MAX_DISPLAY_LENGTH+1];
};

/* the project display text master list.  */
struct project_display_list {
	u32 num_projects;	/* should be the same as project_master_list.num_projects */
	u32 max_snapshot_names;	/* will be less than project master_list */
	u32 row_size;	/* bytes per row == bytes per project */
	struct project_display display;
}; 

/* the initial index part of the flash image. */

 
struct updateable_flash_image_index {
	/* start marker of the flash image. */
	u32 start_marker;

	/* address of the end marker for the flash image. */
	u32 image_trailer_marker_ptr;	/* image_trailer_marker_t* */

	u32 image_checksum;	/* not currently used. */

	/* master pointer list of all ULDs in flash. */
	u32 project_master_list_size;	/* in bytes */
	u32 project_master_list_ptr;	/* project_master_list_t* */

	/* address of text display structure. */
	u32 project_display_ptr;	/* project_text_display_t* */

	/* words to use for HCMB boot message. If first word is zero, this is slave boot project */
	u32 hcmb_message[2];

	u32 fw_version_ptr;

};


struct cs48l10_firmware {
	u32 firmware_size;
	const u8 *firmware_data;
	u8 current_project;
	u8 current_snapshot;
	u8 number_of_snapshots_for_current_project;
	u8 active_project;

	/* structure of flash addresses to DSP code (.uld) and configurations (.cfg) */
	struct updateable_flash_image_index *pflash_image;
	struct project_master_list_str *pmaster_list;
	struct project_str *projects;
	struct project_display_list *pdisplay_list;
	struct project_display* project_names;
};


/* dsp cmd */
struct dsp_cmd
{
        u32		command;
	u32		mask; /* match mask */
	u32		value;
	struct completion	cmd_wait;
        struct list_head	link;
	int 			state;
};

struct unsol_message
{
        u32	data[2]; 
	int error_id;
        struct list_head	link;
};


/* cs48l10 private data */
struct cs48l10 {
	struct spi_device *spi;
	const struct firmware *osfw;	/* firmware class */
	struct cs48l10_firmware *fw;	/* parsed firmware */
	struct mutex lock;

	/* sleep mode: Sleep mode if non zero  */ 
	int sleep_mode;

	/* hibernate Mode.  Hibernate mode if non-zero */
	int hibernate;

	/* gpios from platform_data */
	int gpio_int;
#ifndef CS48L10_NOBUSY 
	int gpio_busy;
#endif 
	int gpio_reset;

   wait_queue_head_t wait;
   wait_queue_head_t rx_data_wait;
	int rx_data_available;
	struct wake_lock gpio_int_wake_wakelock;
	char *buffer;

	/* DSP */
	u32 dsp_packet_unit_size;
	u32 dsp_chunk_size; /* dsp_packet_unit_size aligned */ 
	int unsol_timeout;
#define DSP_MODE_CMD		0
#define DSP_MODE_CMD_DATA	1
#define DSP_MODE_DATA		2
	int dsp_mode;
#define DSP_STATE_LOW_POWER	0
#define DSP_STATE_STOP		1
#define DSP_STATE_PLAY		2
#define DSP_STATE_PAUSE		3
	int dsp_state;
	unsigned int dsp_reset_count;

	/* Unsolicited message handing */

	int sol_count; /* Number of solicited messages waiting */
	struct work_struct 	unsol_work;
   	struct list_head	unsol_list;
	int unsol_length;
	int error_id;
	spinlock_t              sol_spinlock;


   	struct work_struct 	int_work;
   	struct list_head	cmd_list;
	struct mutex		cmd_lock;

	struct dsp_cmd	cached_unsol;
	int 		has_cached_unsol;

	/* DSP module rendering the current stream */

	bool boot_assist;
	int boot_assist_project;
#define FW_VERSION_SIZE 20
	char firmware_version[FW_VERSION_SIZE];
};

#ifndef CS48L10_NOBUSY
static void cs48l10_busy(struct cs48l10 *cs48l10);
#endif 

static int cmp_micro_condenser_get_project_name
(
	struct cs48l10 *cs48l10,
	int project_id,
	char buffer[],
	int bufsiz
);


 /* cs48l10_spi_speed() - Set new SPI speed for all spi transactions */
static int cs48l10_spi_speed(struct spi_device* spi, int max_speed_hz)
{
	int ret;

	if (spi->max_speed_hz == max_speed_hz &&
		spi->bits_per_word == 8 && 
		spi->mode == SPI_MODE_0)
			return 0;

	if (max_speed_hz > CS48L10_MAX_SPI || max_speed_hz < CS48L10_MIN_SPI)
		max_speed_hz = CONFIG_CS48L10_SPI_PREBOOT_FREQ;
	    
       	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0; 		/* Normal clock, rising edge */
	spi->max_speed_hz = max_speed_hz; 	/* Max clock is 24Mhz */

        ret = spi_setup(spi);
        if (ret < 0) {
                dev_err(&spi->dev, "spi setup (%u Hz) failed\n", 
			max_speed_hz);
                return ret;
        }
        dev_info(&spi->dev, "New SPI speed: %u Hz\n", max_speed_hz);
	return 0;
}

static int cs48l10_write_dsp_word(struct cs48l10 *cs48l10, const u32 dsp_word)
{
	int ret;
	u8 buf[5];
	u8 *pbuf = &buf[1];

	buf[0] = CS48L10_SPI_WRITE_CMD;
	(*(u32 *) pbuf) = cpu_to_be32(dsp_word);

	if ((ret = spi_write(cs48l10->spi, buf, sizeof(buf))) < 0)
		dev_err(&cs48l10->spi->dev,
			": %s() write %d bytes failed: ret = %d\n",
			__FUNCTION__, sizeof(buf), ret);
	return ret;
}


static int cs48l10_read_dsp_word(struct cs48l10 *cs48l10, u32 * dsp_word)
{
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer tx, rx;
	u32 srx = 0;
	u8 stx = CS48L10_SPI_READ_CMD;

	memset(&tx, 0, sizeof(struct spi_transfer));
	memset(&rx, 0, sizeof(struct spi_transfer));

	/* tx xfer */
	tx.tx_buf = &stx;
	tx.len = sizeof(stx);
	tx.bits_per_word = 8;
	tx.speed_hz = cs48l10->spi->max_speed_hz;

	/* rx xfer */
	rx.rx_buf = &srx;
	rx.len = sizeof(srx);
	rx.bits_per_word = 8;
	rx.speed_hz = cs48l10->spi->max_speed_hz;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);

	if ((ret = spi_sync(cs48l10->spi, &msg)) < 0) {
		dev_err(&cs48l10->spi->dev,
			": %s() read %d bytes failed: ret = %d\n",
			__FUNCTION__, sizeof(srx), ret);
		return ret;
	}
	dev_dbg(&cs48l10->spi->dev,
		": %s() result: %08x\n", __FUNCTION__, cpu_to_be32(srx));

	*dsp_word = cpu_to_be32(srx);
	return ret;
}

static int cs48l10_write_dsp_word_ptr_buf(struct cs48l10 *cs48l10,
					  const u8 * dsp_word_ptr,
					  int dsp_word_buf_len)
{
	int ret;
	u8 buf[(DSP_WORD_TRASFER_SIZE * sizeof(u32)) + 1];
	int buf_len = DSP_WORD_TRASFER_SIZE * sizeof(u32);

	if (buf_len > dsp_word_buf_len) {
		buf_len = dsp_word_buf_len;
	}

	buf[0] = CS48L10_SPI_WRITE_CMD;
	memcpy((u8 *) & buf[1], dsp_word_ptr, buf_len);

	/* Write the buf length plus the write cmd */
	if ((ret = spi_write(cs48l10->spi, buf, buf_len+1)) < 0)
		dev_err(&cs48l10->spi->dev,
			": %s() write %d bytes failed: ret = %d\n",
			__FUNCTION__, buf_len, ret);
	return ret;
}


static int cs48l10_write_buf(struct cs48l10 *cs48l10, const u8 * buf, int len)
{
	int ret;
	int write_len = 0;
	int chunk_len = DSP_WORD_TRASFER_SIZE * sizeof(u32);

	len &= ~3;
	write_len = len;

	while (write_len) {

#ifndef CS48L10_NOBUSY
		cs48l10_busy(cs48l10);
#endif

		if (write_len < chunk_len)
			chunk_len = write_len;

		ret = cs48l10_write_dsp_word_ptr_buf(cs48l10, buf, chunk_len);

		if (ret < 0) {
			printk("cs48l10_write_buf: errret: %d\n", ret);
			return ret;
		}
		write_len -= chunk_len;
		buf += chunk_len;
	}
	//printk("cs48l10_write_buf: ok: %d\n", len);
	return len;
}


static int cs48l10_write_then_read(struct cs48l10* cs48l10,
				const u32* txwords, int txlen,
				u32* rxwords, int rxlen)
{
	int ret, rxread = 0;
	unsigned long flags;

	cs48l10->rx_data_available = 0;

	spin_lock_irqsave(&cs48l10->sol_spinlock, flags);
	cs48l10->sol_count++;
	spin_unlock_irqrestore(&cs48l10->sol_spinlock, flags);

        /* Send the DSP command words */
	while (txlen--) {
		if ((ret = cs48l10_write_dsp_word(cs48l10, *txwords)) < 0)
			return ret;
		txwords ++;
	}
	/*Increment the sol count to show the isr how many soliciated
	 messages are expecting */
	/* Wait for nINT */
        ret = wait_event_interruptible_timeout(cs48l10->rx_data_wait,
                         !gpio_get_value(cs48l10->gpio_int), 5 *  HZ);
        if (ret < 0)
                return ret;

        if (!ret) {
                dev_err(&cs48l10->spi->dev,
                        "%s(): Timeout while waiting for dsp response\n",
                        __FUNCTION__);
                return -ETIMEDOUT;
        }

	/*Decrement the sol count to show the isr how many soliciated messages 
	are expecting */
	spin_lock_irqsave(&cs48l10->sol_spinlock, flags);
	cs48l10->sol_count--;
	spin_unlock_irqrestore(&cs48l10->sol_spinlock, flags);

        /* Read DSP response */
	while (rxlen --) {
    		if ((ret = cs48l10_read_dsp_word(cs48l10, rxwords)) < 0)
            		return ret;
		rxwords ++; rxread ++;
		if (gpio_get_value(cs48l10->gpio_int))
		    break;
	}
	/* nINT still low ? */

	cs48l10->rx_data_available = 0;

	return rxread;
}

#define BUSY_RETRY	10
#define INT_RETRY	10


#ifndef CS48L10_NOBUSY
static void cs48l10_busy(struct cs48l10 *cs48l10)
{
	int retry = BUSY_RETRY;
	int dsp_busy;

	if (!cs48l10->gpio_busy) {
		udelay(CS48L10_SPI_BUSY_WAIT_USEC * BUSY_RETRY);
		return;
	}

	while (retry--) {
		if ((dsp_busy = gpio_get_value(cs48l10->gpio_busy)) != 0)
			return;

		udelay(CS48L10_SPI_BUSY_WAIT_USEC);
	}
}
#endif //CS48L10_NOBUSY


/* DSP hardware reset */
static int cs48l10_dsp_reset(struct cs48l10 *cs48l10)
{
	int ret;
	u32 dsp_result;
	unsigned long flags;

	dev_info(&cs48l10->spi->dev, "cs48l10_dsp_reset");

	if (!cs48l10->gpio_reset)
		return -EINVAL;

	spin_lock_irqsave(&cs48l10->sol_spinlock, flags);
	cs48l10->sol_count++;
	spin_unlock_irqrestore(&cs48l10->sol_spinlock, flags);

	gpio_set_value(cs48l10->gpio_reset, 0);
	udelay(100);
	cs48l10->dsp_reset_count++;
	gpio_set_value(cs48l10->gpio_reset, 1);
	udelay(100);

	/* Increment the sol count to show the isr how many soliciated messages
	 * are expecting 
	 */
	ret = wait_event_interruptible_timeout(cs48l10->rx_data_wait,
			!gpio_get_value(cs48l10->gpio_int), 5 * HZ);
	if (ret < 0)
		return ret;
	if (!ret) {
		dev_err(&cs48l10->spi->dev,
			"%s(): Timeout while wiating for BOOT_READY (%08x)\n",
			__FUNCTION__, CS48L10_BOOT_READY);
		return -ETIMEDOUT;
	}

	/* Decrement the sol count to show the isr how many soliciated 
	 * messages are expecting.
	 */
	spin_lock_irqsave(&cs48l10->sol_spinlock, flags);
	cs48l10->sol_count--;
	spin_unlock_irqrestore(&cs48l10->sol_spinlock, flags);

	if ((ret = cs48l10_read_dsp_word(cs48l10, &dsp_result)) < 0)
		return ret;

	/* After reset, chip is no longer in hibernate or sleep mode */
	cs48l10->hibernate = 0;
	cs48l10->sleep_mode = 0;


	/* BOOT_READY expected  */
	if (dsp_result != CS48L10_BOOT_READY) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n",
			dsp_result, CS48L10_BOOT_READY);
		return -EIO;
	}

	return 0;
}


/*
    Cirrus Logic Microcondenser Firmware Loader 
*/


static int cs48l10_fw_validate_ptr(struct cs48l10_firmware *fw,
                                         u32 ptr)
{
	return (ptr >= fw->firmware_size);
}

/*Loads a configuration/snapshot using the number specified. */
static int cs48l10_fw_load_config(struct cs48l10 *cs48l10, int cfg)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;
	u32 		cfg_addr;
	u32 		cfg_ptr_addr;
	struct cfg_ptr 			*pcfg;
	struct project_master_list_str 	*master_list;			
	struct project_str 		*project;

	const u8 *buf;
	u32 bufsize;

	/* get Snapshot[cfg] ptr and address from flash, only 1 snapshot ptr and
	 address is saved in the MCU RAM. */

	master_list = (struct project_master_list_str *)
                      (fw->firmware_data
		      + cpu_to_be32(fw->pflash_image->project_master_list_ptr));

	project = (struct project_str *)(((u8 *)&master_list->project) + 
	( be32_to_cpu(fw->pmaster_list->row_size) * fw->current_project));

	pcfg = (struct cfg_ptr *)&project->snapshot[cfg];
	cfg_addr =  (u8 *)pcfg - (u8 *)fw->firmware_data;
	
	if (cs48l10_fw_validate_ptr(fw, cfg_addr))
		return -EIO;

	bufsize 	= be32_to_cpu(project->snapshot[cfg].cfg_length);
	cfg_ptr_addr	= be32_to_cpu(project->snapshot[cfg].cfg_ptr);

	if (cs48l10_fw_validate_ptr(fw, cfg_ptr_addr + bufsize))
		return -EIO;

	if (bufsize && cfg_ptr_addr) {
		buf = fw->firmware_data + cfg_ptr_addr;

		if ((ret = cs48l10_write_buf(cs48l10, buf, bufsize)) < 0)
			return ret;
	}

	fw->current_snapshot = cfg;

	return 0;
}



/* Fills a buffer with the name of a specific project */
static int cmp_micro_condenser_get_project_name(struct cs48l10 *cs48l10, 
						int project_id, char *buffer, int bufsiz)
{
	int flash_addr;
	int row_size;
		
	struct project_display_list * display_info;
	struct project_display      * project_info;
	struct cs48l10_firmware *fw = cs48l10->fw;

	/* Fill the display_list data structure if it exists */
	if (fw->pflash_image->project_display_ptr) {
		flash_addr = 
		be32_to_cpu(fw->pflash_image->project_display_ptr);
		if(cs48l10_fw_validate_ptr(fw, flash_addr )) {
		
			printk(KERN_INFO "flash_addr check failed \n");
			return -EIO;
		}
		display_info = (struct project_display_list *)
		(((u8 *)cs48l10->fw->firmware_data) + flash_addr);

		row_size = be32_to_cpu(display_info->row_size);

		project_info = (struct project_display *)
		(( u8 *) &display_info->display + (row_size * project_id));		

		memcpy(buffer, project_info->project_name, MAX_DISPLAY_LENGTH);
	} else {
		snprintf(buffer, bufsiz, "%2d", project_id);
	}
	return 0;
}

/* Fills a buffer with the name of the snapshot */
static int cmp_micro_condenser_get_snapshot_name(struct cs48l10	*cs48l10, 
				int project_id, int snapshot_id, char *buffer, int bufsiz)
{ 

	int flash_addr;
	int row_size;
	int max_snapshot_names;
	struct project_display_list *display_info;
	struct project_display      *project_info;
	struct cs48l10_firmware *fw = cs48l10->fw;

	if (snapshot_id < INITIAL_CFG){
		snprintf((char *)buffer, bufsiz, "%2d", snapshot_id);
	} else if (snapshot_id == INITIAL_CFG) {
		strlcpy((char *)buffer, "Initial", bufsiz);
	} else if (fw->pflash_image->project_display_ptr) {
		flash_addr = be32_to_cpu(fw->pflash_image->project_display_ptr);
		
		/* Get the row size. */
		display_info = (struct project_display_list *)
		( ((u8 *)cs48l10->fw->firmware_data) + flash_addr);

		row_size = be32_to_cpu(display_info->row_size);

		/* Get the max number of snapshots per project. */
		max_snapshot_names =
			be32_to_cpu(display_info->max_snapshot_names);

		/* We need to use max_snapshot_names+3 here because the first
		 *three snapshots are pre-kickstart, kickstart, and initial
		 */
		
		if(snapshot_id > (max_snapshot_names + 3)) {
			
			printk("Invalid snapshot id %d\n", snapshot_id);
			snprintf((char *)buffer, bufsiz, "%2d", project_id);
			return -EIO;
		}

		project_info = (struct project_display *)
			((u8*) &display_info->display + 
				(row_size * project_id));
		/* We need to use snapshot_id-3 here because the first three
		 * snapshots are pre-kickstart, kickstart, and initial 
		 */
		memcpy(buffer, project_info->snapshot_name[snapshot_id-3],
			MAX_DISPLAY_LENGTH);
	} else {
		snprintf(buffer, bufsiz, "%2d", snapshot_id);
	}
	return 0;
}


/* This function boots the uld at the specified firmware offset. */
static int cs48l10_fw_slave_boot_uld(struct cs48l10 *cs48l10, u32 addr,
					u32 size)
{
	struct cs48l10_firmware *fw = cs48l10->fw;

	int ret;
	const u8 *buf;
	u32 dsp_result;
	unsigned long flags;

	/* Send SLAVE_BOOT message */
	spin_lock_irqsave(&cs48l10->sol_spinlock, flags);
	cs48l10->sol_count++;
	spin_unlock_irqrestore(&cs48l10->sol_spinlock, flags);

	if ((ret = cs48l10_write_then_read(cs48l10,
			&Slave_Boot, 1,
			&dsp_result, 1)) < 0)
		return ret;

	/* BOOT_START expected  */
	if (dsp_result != CS48L10_BOOT_START) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_BOOT_START);

		return -EIO;
	}

	dev_dbg(&cs48l10->spi->dev, "\t@ %08x, size %d\n", addr, size);

	if (cs48l10_fw_validate_ptr(fw, addr + size))
		return -EIO;

	buf = fw->firmware_data + addr;

   	cs48l10->rx_data_available = 0;

	if ((ret = cs48l10_write_buf(cs48l10, buf, size)) < 0)
		return ret;

	/* Increment the sol count to show the isr how many soliciated messages
	 * are expecting.
	 */
   	ret = wait_event_interruptible_timeout(cs48l10->rx_data_wait,
                        cs48l10->rx_data_available, 5 * HZ);
	if (ret < 0)
		return ret;
	if (!ret) {
		dev_err(&cs48l10->spi->dev,
			"%s(): Timeout while waiting for dsp response\n",
			__FUNCTION__);
		return -ETIMEDOUT;
	}
	/* Decrement the sol count to show the isr how many soliciated
	 * messages are expecting.
	 */
	spin_lock_irqsave(&cs48l10->sol_spinlock, flags);
	cs48l10->sol_count--;
	spin_unlock_irqrestore(&cs48l10->sol_spinlock, flags);

	/* Read DSP response */
	if ((ret = cs48l10_read_dsp_word(cs48l10, &dsp_result)) < 0) {
		printk("DSP read word error in %s()\n", __FUNCTION__);
		return ret;
	}

	/* BOOT_SUCCESS expected  */
	if (dsp_result != CS48L10_BOOT_SUCCESS) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_BOOT_SUCCESS);

		return -EIO;
	}

	dev_dbg(&cs48l10->spi->dev, "\t@ %08x, size %d [OK]\n", addr, size);

	return 0;
}


static int cs48l10_fw_load_project(struct cs48l10 *cs48l10, int project_id,
							int snapshot_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;

	int ret, i;
	struct project_str *pproject;
	struct project_display *pproject_name;
	u32 dsp_result = 0;

        pproject= (struct project_str *)((u8 *) fw->projects +
    		cpu_to_be32(fw->pmaster_list->row_size) * project_id);

        pproject_name = (struct project_display *)((u8 *) fw->project_names +
    		cpu_to_be32(fw->pdisplay_list->row_size) * project_id);

	dev_info(&cs48l10->spi->dev, "Project[%d] %s\n", project_id,
		pproject_name->project_name);

	dev_dbg(&cs48l10->spi->dev, "Project Input Src	%02x\n",
		 pproject->input_src);
	dev_dbg(&cs48l10->spi->dev, "Project Input FS	%02x\n",
		 pproject->input_fs);
	dev_dbg(&cs48l10->spi->dev, "Project Output FS	%02x\n",
		 pproject->output_fs);
	
	for (i = 0; i < MAX_ULDS_PER_SET; i++) {
		if (pproject->uld_set.uld_address[i].uld_length == 0)
			continue;

		/* slaveboot the .uld file from SPI_FLASH to the DSP */
		dev_info(&cs48l10->spi->dev, "ULD[%d]\n", i);
		if ((ret = cs48l10_fw_slave_boot_uld(cs48l10,
			cpu_to_be32(pproject->
				uld_set.
				uld_address[i].
				uld_ptr),
			cpu_to_be32(pproject->
				uld_set.
				uld_address[i].
				uld_length))) <
		    0)
	 return ret;
	}

	/* Send SOFT_RESET message */
	if ((ret = cs48l10_write_then_read(cs48l10,
			&Soft_Reset, 1, &dsp_result, 1)) < 0)
		return ret;
	

		
	if (dsp_result == CS48L10_BOOT_READY && cs48l10->boot_assist == true) {
		  /* We are in boot assist.  We are done here.*/
		return ret;
	} else if (dsp_result != CS48L10_APP_START) {
		  dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_APP_START);
			return -EIO;
	}

	/* If not in boot assist and CS48L10_APP_START is true, 
	 * then we load the approprite project.
	 */

	fw->current_project = project_id;
	fw->current_snapshot = snapshot_id;

	ret = cs48l10_spi_speed(cs48l10->spi, CONFIG_CS48L10_SPI_RUNTIME_FREQ);
	if (ret < 0)
		return ret;


	if ((ret = cs48l10_fw_load_config(cs48l10, PREKICKSTART_CFG)) < 0)
		return ret;

	if ((ret = cs48l10_fw_load_config(cs48l10, INITIAL_CFG)) < 0)
		return ret;

	if(snapshot_id != INITIAL_CFG)
	{
		if ((ret = cs48l10_fw_load_config(cs48l10, snapshot_id)) < 0)
			return ret;
	}

	if ((ret = cs48l10_fw_load_config(cs48l10, KICKSTART_CFG)) < 0)
		return ret;

	fw->active_project = project_id;
	mdelay(5);
	return 0;
}


static int cs48l10_fw_softboot_project(struct cs48l10 *cs48l10, 
				int project_id, int snapshot_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;
	u32 dsp_result;

	if (!fw)
		return -EINVAL;

	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Project %d is not in projects master list\n",
			 project_id);
		return -EINVAL;
	}

	if ((ret = cs48l10_write_then_read(cs48l10,
			Soft_Boot, ARRAY_SIZE(Soft_Boot),
			&dsp_result, 1)) < 0)
			return ret;

	if (dsp_result != CS48L10_BOOT_READY) {
		dev_err(&cs48l10->spi->dev,
			"DSP result = 0x%08x, expected %08x\n", dsp_result,
			CS48L10_BOOT_READY);
		return -EIO;
	}
	return cs48l10_fw_load_project(cs48l10, project_id, snapshot_id);
}


static int cs48l10_fw_boot_project(struct cs48l10 *cs48l10, int project_id,
							int snapshot_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;

	if (!fw) {
		dev_err(&cs48l10->spi->dev,
                         "Error occurred in cs48l10_boot_project\n");
		return -EINVAL;
	}

	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Project %d is not in projects master list\n",
			 project_id);
		return -EINVAL;
	}
	ret = cs48l10_spi_speed(cs48l10->spi, CONFIG_CS48L10_SPI_PREBOOT_FREQ);
	if (ret < 0)
		return ret;


	if ((ret = cs48l10_dsp_reset(cs48l10)) < 0) {
		dev_err(&cs48l10->spi->dev,
			"Error loading project [%d]. DSP reset failed\n",
			project_id);
		return ret;
	}

	return cs48l10_fw_load_project(cs48l10, project_id, snapshot_id);
}


/* Performs a hard reset, loads the boot assist project, and loads the 
 * default project.
 */
static int cs48l10_fw_bootassist_project(struct cs48l10 *cs48l10, int project_id, int snapshot_id)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret;
	int retry_count = 0;

	if (!fw) {
		dev_err(&cs48l10->spi->dev,
	        "Error occurred in cs48l10_boot_project\n");
		return -EINVAL;
	}

	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Project %d is not in projects master list\n",
			 project_id);
		return -EINVAL;
	}

retry:
	 /* The initial spi speed should be set as a fraction of the
	  * reference clock speed.
	  */
	ret = cs48l10_spi_speed(cs48l10->spi, CONFIG_CS48L10_SPI_PREBOOT_FREQ);
   	if (ret < 0)
      		return ret;


	if ((ret = cs48l10_dsp_reset(cs48l10)) < 0) {
		dev_err(&cs48l10->spi->dev,
			"Error loading project [%d]. DSP reset failed\n",
			project_id);
		return ret;
	}
	

	/* Load the boot assist project */
	ret = cs48l10_fw_load_project(cs48l10, cs48l10->boot_assist_project,
					INITIAL_CFG);
	if(ret < 0 )
		return ret;

	/* The PLL in the dsp should be locked.  We now can use the max 
	 * frequency of the SPI bus.
	 */
	ret = cs48l10_spi_speed(cs48l10->spi, CONFIG_CS48L10_SPI_BOOT_FREQ);
	if (ret < 0)
		return ret;

	cs48l10->boot_assist = false;
	/* Now, load the required project */
	ret = cs48l10_fw_load_project(cs48l10, project_id, snapshot_id);
	if (ret < 0 && ++retry_count < FW_LOAD_RETRY_COUNT) {
		mdelay(500);

		dev_err(&cs48l10->spi->dev,
			"cs48l10_fw_load_project : retry %d\n",
			retry_count);
		cs48l10->boot_assist = true;
		goto retry;
	}

	return ret;
}


static int cs48l10_fw_parse_master_list(struct cs48l10 *cs48l10)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	u32 projects_addr, project_addr_last;
	u32 project_names_addr, project_names_addr_last;

	fw->current_project = 0;
	fw->active_project = 0;

	projects_addr = cpu_to_be32(fw->pflash_image->project_master_list_ptr) +
	    sizeof(struct project_master_list_str) - sizeof(struct project_str);

	if (cs48l10_fw_validate_ptr(fw, projects_addr))
		return -EIO;

        project_addr_last = projects_addr +
    		(cpu_to_be32(fw->pmaster_list->row_size) * 
		(cpu_to_be32(fw->pmaster_list->num_projects) - 1));

	if (cs48l10_fw_validate_ptr(fw, project_addr_last))
		return -EIO;

	fw->projects = (struct project_str *) (fw->firmware_data + projects_addr);

	project_names_addr = cpu_to_be32(fw->pflash_image->project_display_ptr) +
		sizeof(struct project_display_list) - sizeof(struct project_display);

	if (cs48l10_fw_validate_ptr(fw, project_names_addr))
		return -EIO;

        project_names_addr_last = project_names_addr +
    		(cpu_to_be32(fw->pdisplay_list->row_size) * 
		(cpu_to_be32(fw->pdisplay_list->num_projects) - 1));

	if (cs48l10_fw_validate_ptr(fw, project_names_addr_last))
		return -EIO;

	fw->project_names = (struct project_display *) (
		fw->firmware_data + project_names_addr);

	return 0;
}


static int cs48l10_fw_parse_firmware(struct cs48l10 *cs48l10)
{
	struct cs48l10_firmware *fw = cs48l10->fw;
	u32 image_marker;

	fw->firmware_data = cs48l10->osfw->data;
	fw->firmware_size = cs48l10->osfw->size;

	if (cs48l10_fw_validate_ptr(fw, sizeof(struct updateable_flash_image_index)))
		return -EINVAL;

	fw->pflash_image = (struct updateable_flash_image_index *) fw->firmware_data;
	dev_dbg(&cs48l10->spi->dev,
		 "Start Marker:			%08x\n",
		 cpu_to_be32(fw->pflash_image->start_marker));

	if (FLASH_IMAGE_MARKER != cpu_to_be32(fw->pflash_image->start_marker)) {
		dev_info(&cs48l10->spi->dev, "Bad start Marker\n");
		return -EIO;
	}

	if (cs48l10_fw_validate_ptr
	    (fw, cpu_to_be32(fw->pflash_image->image_trailer_marker_ptr)))
		return -EIO;

	image_marker = *((u32 *) (fw->firmware_data +
					 cpu_to_be32(fw->pflash_image->
						     image_trailer_marker_ptr)));

	if (cpu_to_be32(image_marker) != FLASH_IMAGE_MARKER) {
		dev_err(&cs48l10->spi->dev, "Bad trailer Marker %08x\n",
			 cpu_to_be32(image_marker));
		return -EIO;
	}

	dev_dbg(&cs48l10->spi->dev, "Trailer Marker Ptr:		%08x\n",
		 cpu_to_be32(fw->pflash_image->image_trailer_marker_ptr));
	dev_dbg(&cs48l10->spi->dev,
		 "Image Chekcsum:			%08x\n",
		 cpu_to_be32(fw->pflash_image->image_checksum));
	dev_dbg(&cs48l10->spi->dev, "Project Master List Size:	%08x\n",
		 cpu_to_be32(fw->pflash_image->project_master_list_size));
	dev_info(&cs48l10->spi->dev, "Project Master Ptr:		%08x\n",
		 cpu_to_be32(fw->pflash_image->project_master_list_ptr));
	dev_info(&cs48l10->spi->dev, "Project Text Display Ptr:	%08x\n",
		 cpu_to_be32(fw->pflash_image->project_display_ptr));
	dev_dbg(&cs48l10->spi->dev,
		 "HCMB[0,1]:			%08x, %08x\n",
		 cpu_to_be32(fw->
			     pflash_image->hcmb_message
			     [0]),
		 cpu_to_be32(fw->pflash_image->hcmb_message[1]));

	if (cs48l10_fw_validate_ptr
	    (fw, cpu_to_be32(fw->pflash_image->project_master_list_ptr)))
		return -EINVAL;

	fw->pmaster_list = (struct project_master_list_str *)
	    (fw->firmware_data +
	     cpu_to_be32(fw->pflash_image->project_master_list_ptr));

	dev_info(&cs48l10->spi->dev, "Master List\n");
	dev_info(&cs48l10->spi->dev, "Number of Projects:	%d\n",
		 cpu_to_be32(fw->pmaster_list->num_projects));
	dev_info(&cs48l10->spi->dev, "Max Cfgs per Project:	%d\n",
		 cpu_to_be32(fw->pmaster_list->cfgs_per_project));
	dev_info(&cs48l10->spi->dev, "Row size:		%d\n",
		 cpu_to_be32(fw->pmaster_list->row_size));

	fw->pdisplay_list = (struct project_display_list *)
	    (fw->firmware_data +
	     cpu_to_be32(fw->pflash_image->project_display_ptr));

	dev_info(&cs48l10->spi->dev, "Display List\n");
	dev_info(&cs48l10->spi->dev, "Number of Projects(display):	%d\n",
		 cpu_to_be32(fw->pdisplay_list->num_projects));
	dev_info(&cs48l10->spi->dev, "Max snapshots per Project:	%d\n",
		 cpu_to_be32(fw->pdisplay_list->max_snapshot_names));
	dev_info(&cs48l10->spi->dev, "Row size:		%d\n",
		 cpu_to_be32(fw->pdisplay_list->row_size));

	strncpy(cs48l10->firmware_version, fw->firmware_data +
	   FW_VERSION_OFFSET, FW_VERSION_SIZE);
	
	cs48l10_fw_parse_master_list(cs48l10);

	return 0;
}

/* Microcondenser Firmware Loader */
#ifdef CS48L10_ASYNC_FWLOADER
/* Callback function for Async firmware load. */
static void cs48l10_fw_load_async_cont(const struct firmware *osfw,
				       void *context)
{
	struct cs48l10 *cs48l10 = (struct cs48l10 *)context;
	int ret;
	int i;
	struct cs48l10_firmware *fw;
	struct project_display *pproject_name;

	dev_dbg(&cs48l10->spi->dev, "cs48l10_fw_load_async_cont() ...\n");
	if (!cs48l10) 
		return;
	

	if (!osfw) {
		dev_err(&cs48l10->spi->dev, "Firmware not available\n");
		return;
	}

	if (cs48l10->fw)
		kfree(cs48l10->fw);

	if (cs48l10->osfw)
		release_firmware(cs48l10->osfw);

	cs48l10->osfw = osfw;

	cs48l10->fw = (struct cs48l10_firmware *)
	    kzalloc(sizeof(struct cs48l10_firmware), GFP_KERNEL);

	if (!cs48l10->fw) {
		release_firmware(cs48l10->osfw);
		cs48l10->osfw = NULL;
		mutex_unlock(&cs48l10->lock);
		return;
	}
	ret = cs48l10_fw_parse_firmware(cs48l10);
	if (ret < 0) {
		dev_err(&cs48l10->spi->dev,
			"firmware (%u bytes) failed to load, ret = %d\n",
			cs48l10->osfw->size, ret);

		kfree(cs48l10->fw);
		release_firmware(cs48l10->osfw);
		cs48l10->osfw = NULL;
		cs48l10->fw = NULL;
		return;
	}

	dev_info(&cs48l10->spi->dev,
		 "firmware (%u bytes) parsed successfully\n",
		 cs48l10->osfw->size);


	fw = cs48l10->fw;
	/*Set boot_assist_project to < 0 to identify that it was not set. */
	cs48l10->boot_assist_project = -1;  
	cs48l10->boot_assist = false;  

	for(i = 0; i < cpu_to_be32(fw->pmaster_list->num_projects); i++) {
		pproject_name = (struct project_display *)((u8 *)fw->project_names +
    		cpu_to_be32(fw->pdisplay_list->row_size) * i);
		
		if(!memcmp(pproject_name->project_name,
			CS48L10_BOOT_ASSIST_PROJECT_NAME,
			strlen(CS48L10_BOOT_ASSIST_PROJECT_NAME))) {
			cs48l10->boot_assist = true;  
			cs48l10->boot_assist_project = i;  
			break;
		}
	}
	if(cs48l10->boot_assist_project == CS48L10_DEFAULT_PROJECT_ID) {
		dev_err(&cs48l10->spi->dev,
		"Boot assist project can't be the same as the default.\n");
		return -EIO;
	}

	mutex_lock(&cs48l10->lock);

	if(cs48l10->boot_assist == true) {
		ret = cs48l10_fw_bootassist_project(cs48l10,
			CS48L10_DEFAULT_PROJECT_ID, INITIAL_CFG);
		if(ret < 0)
			dev_err(&cs48l10->spi->dev,
			"Boot assist project failed to start 0x%x\n", ret);

	} else {
		/* Boot the default project */
		ret = cs48l10_fw_boot_project(cs48l10,
			CS48L10_DEFAULT_PROJECT_ID, INITIAL_CFG);
		if (ret < 0)
			dev_err(&cs48l10->spi->dev,
				"Project %d failed to start, ret = %d\n",
				CS48L10_DEFAULT_PROJECT_ID, ret);
	}

	mutex_unlock(&cs48l10->lock);

}

/* Asynchronous load from a microcondenser image from /lib/firmware */
static int cs48l10_fw_load_async(struct cs48l10 *cs48l10, const char *filename)
{
	int ret;

	cs48l10->fw = NULL;
	cs48l10->osfw = NULL;

	ret =
	    request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG, filename,
				    &cs48l10->spi->dev, GFP_KERNEL, cs48l10,
				    cs48l10_fw_load_async_cont);

	if (ret != 0) {
		dev_err(&cs48l10->spi->dev,
			"request async load of %s failed. err = %d\n", 
			filename,
			ret);

		return ret;
	}

	return 0;
}
#else

/* Sync firmware load from a microcondenser image from /lib/firmware */
static int cs48l10_fw_load_sync(struct cs48l10 *cs48l10, const char *filename)
{
	int ret;
	ret = request_firmware(&cs48l10->osfw, filename, &cs48l10->spi->dev);

	if (ret != 0) {
		dev_err(&cs48l10->spi->dev,
			"firmware: %s not found. err = %d\n", filename, ret);

		return ret;
	}
	cs48l10->fw = (struct cs48l10_firmware *)
	    kzalloc(sizeof(struct cs48l10_firmware), GFP_KERNEL);

	if (!cs48l10->fw)
		return -ENOMEM;

	ret = cs48l10_fw_parse_firmware(cs48l10);
	if (ret < 0) {
		dev_err(&cs48l10->spi->dev,
			"firmware %s (%u bytes) was not parsed, ret = %d\n",
			filename, cs48l10->osfw->size, ret);

		kfree(cs48l10->fw);
		release_firmware(cs48l10->osfw);

		cs48l10->osfw = NULL;
		cs48l10->fw = NULL;

		return ret;
	}

	dev_info(&cs48l10->spi->dev,
		 "firmware %s (%u bytes) parsed successfully\n",
		 filename, cs48l10->osfw->size);

	return 0;
}

#endif  //CS48L10_ASYNC_FWLOADER

static ssize_t cs48l10_hibernate_mode_show(struct device *dev,
				       struct device_attribute *attr, char *buf)

{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", cs48l10->hibernate);
}

/* Shows the driver version number via the sysfs */
static ssize_t cs48l10_drv_ver_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CS48L10_DRV_VERSION);
}


static ssize_t cs48l10_drv_ver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return 0;
}

/* Shows the firmware version number via the sysfs */
static ssize_t cs48l10_fw_ver_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	if(strlen(cs48l10->firmware_version))
		return snprintf(buf, PAGE_SIZE, "%s\n", cs48l10->firmware_version);
	else
		return -EINVAL;
}


static ssize_t cs48l10_fw_ver_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return 0;
}


/* Shows the firmware version number via the sysfs */
static ssize_t cs48l10_err_mess_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	char tmp[128]; 
	struct cs48l10 *cs48l10;
	struct unsol_message *node;
	cs48l10 = dev_get_drvdata(dev);
	if(list_empty(&cs48l10->unsol_list)) {
		snprintf(buf, PAGE_SIZE, "%s", "No Messages\n");
		return strlen("No Messages\n")+1; 
	}
	memset(tmp, 0, 128);
	strlcpy(buf, "", PAGE_SIZE);
		
	list_for_each_entry(node, &cs48l10->unsol_list, link) {
		/* Add error ID */
		snprintf(tmp, sizeof(tmp), "id=%d ", node->error_id);
		strlcat(buf, tmp, PAGE_SIZE);
		switch(node->data[0]){
			case CS48L10_MALLOC_ERR:
				snprintf(tmp, sizeof(tmp), "MALLOC ERROR: ");
				strlcat(buf, tmp, PAGE_SIZE);
				break;
			case CS48L10_PLL_ERR:
				snprintf(tmp, sizeof(tmp), "MALLOC ERROR: ");
				strlcat(buf, tmp, PAGE_SIZE);
				break;

			default:
				snprintf(tmp, sizeof(tmp), "UNKNOWN MSG: ");
				strlcat(buf, tmp, PAGE_SIZE);
				break;
		}
		snprintf(tmp, sizeof(tmp), "0x%08x", node->data[0]);
		strlcat(buf, tmp, PAGE_SIZE);
		snprintf(tmp, sizeof(tmp), "%08x\n", node->data[1]);
		strlcat(buf, tmp, PAGE_SIZE);
	}
	return (strlen(buf)+1);
}


static ssize_t cs48l10_err_mess_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return -EINVAL;
}

 
static ssize_t cs48l10_hibernate_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)


{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	u32 Hibernate_Mode[2] = { 0x81000009, 0x00000011 };
	int ret = 0, i;
 
	mutex_lock(&cs48l10->lock);

	/* Send the Hibernate Mode command to the DSP */
	for (i = 0; i < ARRAY_SIZE(Hibernate_Mode); i++) 
	{
		if ((ret = cs48l10_write_dsp_word(cs48l10, 
						Hibernate_Mode[i])) < 0)
			break;
	} 
	cs48l10->hibernate = 1; 

	if(ret < 0)
	{
		mutex_unlock(&cs48l10->lock);
		return ret;
	}
	else
	{
		mutex_unlock(&cs48l10->lock);
		return count;
	}
}

static int cs48l10_hibernate_mode(struct cs48l10 *cs48l10)
{
 	//struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	u32 Hibernate_Mode[2] = { 0x81000009, 0x00000011 };
	int ret=1, i;
 
	mutex_lock(&cs48l10->lock);
 
	/* Send the Hibernate Mode command to the DSP */
	for (i = 0; i < ARRAY_SIZE(Hibernate_Mode); i++) {
		if ((ret = cs48l10_write_dsp_word(cs48l10,
					Hibernate_Mode[i])) < 0)
			break;
	}  
 
	mutex_unlock(&cs48l10->lock);

	return ret;
}


/* CS48L10_nINT irq handler */
static irqreturn_t cs48l10_int_handler(int irq, void *data)
{
        struct cs48l10 *cs48l10 = (struct cs48l10 *)data;
	if (cs48l10->dsp_state == DSP_STATE_PLAY)
		wake_lock(&cs48l10->gpio_int_wake_wakelock);
	cs48l10->rx_data_available = !gpio_get_value(cs48l10->gpio_int);
	wake_up_interruptible(&cs48l10->rx_data_wait);


	/* Checks to see if the message is soliciated or not.  If not,
	 * schedule the unsol message task.
	*/
	if(cs48l10->sol_count == 0) 
    	        schedule_work(&cs48l10->unsol_work);

/*
	if (!list_empty(&cs48l10->cmd_list)) {
    	        schedule_work(&cs48l10->int_work);
	}
*/

        return IRQ_HANDLED;
}

/* GPIO */
static int __devinit cs48l10_setup_int_gpio(struct spi_device *spi,
                                         struct cs48l10 *cs48l10)
{
	int ret;

	if (!cs48l10)
		return -EINVAL;

        if (!gpio_is_valid(TEGRA_CS48L10_INT_GPIO)) {
                dev_err(&spi->dev, "Invalid CS48L10_nINT gpio %d\n", 
			TEGRA_CS48L10_INT_GPIO);
                return -EINVAL;
        }

        ret = gpio_request(TEGRA_CS48L10_INT_GPIO, "CS48L10_nINT");
        if (ret) {
                dev_err(&spi->dev, 
			"gpio_request() CS48L10_nINT gpio %d failed\n",
			TEGRA_CS48L10_INT_GPIO);
		return -EINVAL;
	}

        ret = gpio_direction_input(TEGRA_CS48L10_INT_GPIO);
        if (ret) {
    		gpio_free(TEGRA_CS48L10_INT_GPIO);
                return ret;
	}

        ret = request_irq(gpio_to_irq(TEGRA_CS48L10_INT_GPIO),
                        cs48l10_int_handler,
                        IRQF_TRIGGER_FALLING,
                        "cs48l10_int",
                        cs48l10);
        if (ret) {
                dev_err(&spi->dev, 
			"request_irq CS48L10_nINT gpio %d -> irq %d failed\n",
			TEGRA_CS48L10_INT_GPIO,
			gpio_to_irq(TEGRA_CS48L10_INT_GPIO));
    		gpio_free(TEGRA_CS48L10_INT_GPIO);
		return ret;
	}

    cs48l10->gpio_int = TEGRA_CS48L10_INT_GPIO;
	enable_irq_wake(gpio_to_irq(cs48l10->gpio_int));


#ifdef CONFIG_GPIO_SYSFS
        /* Expose GPIO value over sysfs for diagnostic purposes */
        gpio_export(cs48l10->gpio_int, false);
#endif
	wake_lock_init(&cs48l10->gpio_int_wake_wakelock, WAKE_LOCK_SUSPEND,
		"cirrus_gpio_int_wake");
        dev_info(&spi->dev, "CS48L10_nINT gpio %d -> irq %d\n",
		cs48l10->gpio_int, gpio_to_irq(cs48l10->gpio_int));

	return 0;
}


static int __devinit cs48l10_setup_reset_gpio(struct spi_device *spi,
                                         struct cs48l10 *cs48l10)
{
	int ret;

	if (!cs48l10)
		return -EINVAL;

   if (!gpio_is_valid(TEGRA_CS48L10_RESET_GPIO)) {
       dev_err(&spi->dev, "Invalid CS48L10_nRESET gpio %d\n", 
			TEGRA_CS48L10_RESET_GPIO);
      return -EINVAL;
   }

   ret = gpio_request(TEGRA_CS48L10_RESET_GPIO, "CS48L10_nRESET");
   if (ret) {
       dev_err(&spi->dev, 
		"gpio_request() CS48L10_nRESET gpio %d failed\n",
			TEGRA_CS48L10_RESET_GPIO);
		return -EINVAL;
	}
   cs48l10->gpio_reset = TEGRA_CS48L10_RESET_GPIO;
	
   ret = gpio_direction_output(TEGRA_CS48L10_RESET_GPIO, 1);
   if (ret) {
 		gpio_free(TEGRA_CS48L10_RESET_GPIO);
      return ret;
	}

   cs48l10->gpio_reset = TEGRA_CS48L10_RESET_GPIO;
#ifdef CONFIG_GPIO_SYSFS
        /* Expose GPIO value over sysfs for diagnostic purposes */
        gpio_export(cs48l10->gpio_reset, false);
#endif
        dev_info(&spi->dev, "CS48L10_nRESET gpio %d\n", cs48l10->gpio_reset);

	return 0;
}


#ifndef CS48L10_NOBUSY
static int __devinit cs48l10_setup_busy_gpio(struct spi_device *spi,
                                         struct cs48l10 *cs48l10)
{
	int ret;

	if (!cs48l10)
		return -EINVAL;

        if (!gpio_is_valid(TEGRA_CS48L10_BUSY_GPIO)) {
                dev_err(&spi->dev, "Invalid CS48L10_nBUSY gpio %d\n", 
			TEGRA_CS48L10_BUSY_GPIO);
                return -EINVAL;
        }

        ret = gpio_request(TEGRA_CS48L10_BUSY_GPIO, "CS48L10_nBUSY");
        if (ret) {
                dev_err(&spi->dev, 
			"gpio_request() CS48L10_nBUSY gpio %d failed\n",
			TEGRA_CS48L10_BUSY_GPIO);
		return -EINVAL;
	}

        ret = gpio_direction_input(TEGRA_CS48L10_BUSY_GPIO);
        if (ret) {
    		gpio_free(TEGRA_CS48L10_BUSY_GPIO);
                return ret;
	}

        cs48l10->gpio_busy = TEGRA_CS48L10_BUSY_GPIO;
#ifdef CONFIG_GPIO_SYSFS
        /* Expose GPIO value over sysfs for diagnostic purposes */
        gpio_export(cs48l10->gpio_busy, false);
#endif
        dev_info(&spi->dev, "CS48L10_nBUSY gpio %d\n", cs48l10->gpio_busy);

	return 0;
}
#endif //CS48L10_NOBUSY

static int cs48l10_free_gpios(struct spi_device* spi, struct cs48l10* cs48l10)
{
	if (cs48l10->gpio_int) {
		wake_lock_destroy(&cs48l10->gpio_int_wake_wakelock);
		free_irq(gpio_to_irq(cs48l10->gpio_int), cs48l10);
    		gpio_free(cs48l10->gpio_int);
	}

	if (cs48l10->gpio_reset) 
    		gpio_free(cs48l10->gpio_reset);

#ifndef CS48L10_NOBUSY
	if (cs48l10->gpio_busy) 
    		gpio_free(cs48l10->gpio_busy);
#endif 

	return 0;
}


static int __devinit cs48l10_setup_gpios(struct spi_device *spi,
					 struct cs48l10 *cs48l10)
{
	struct cs48l10_platform_data *pdata = spi->dev.platform_data;
	int ret;

	if (!pdata)
		return -EINVAL;

	if((ret = cs48l10_setup_reset_gpio(spi, cs48l10)) != 0)
		return ret;
	if ((ret = cs48l10_setup_int_gpio(spi, cs48l10)) != 0)
		return ret;

#ifndef CS48L10_NOBUSY
	cs48l10_setup_busy_gpio(spi, cs48l10);
#endif

	return 0;
}

/* END GPIO */


/* Command dispatcher scheduled on interrupt. */

#define DSP_CMD_MATCH(res, mask, req) ((res & mask) == (req & mask))
static void dsp_cmd_dispatch(struct work_struct *work)
{
	int ret;
	struct dsp_cmd* c, *cmd = NULL;
	u32 response = 0;
        struct cs48l10 *cs48l10 = 
                container_of(work, struct cs48l10, int_work);
	int state = 0; /* cmd state 0 - command, 1 - data */


        /* Read DSP response. Drop unknown codes */

	/* 
	 *   IF we resume after stop the new fw will generate unsol
	 *   response, this still can produce race condition, because
	 *   the unsol readers are not locked togther with the cmd readers
	 *   e.g we issue read command before unsol reader is started.
	*/	

	mutex_lock(&cs48l10->cmd_lock);
	/*
	if (list_empty(&cs48l10->cmd_list)) {
		mutex_unlock(&cs48l10->cmd_lock);
		return;
	}
        */
        while (!gpio_get_value(cs48l10->gpio_int)) {
                if ((ret = cs48l10_read_dsp_word(cs48l10, &response)) < 0) {
			mutex_unlock(&cs48l10->cmd_lock);
                        return;
		}
		/* unsol prefix or response code */
	        if (state == 0) {
			/* Look for a pending command or unsol request */
	    		list_for_each_entry(c, &cs48l10->cmd_list, link) {
            			if (DSP_CMD_MATCH(response, c->mask,
							c->command)) {
					c->command = response;
					cmd = c;
					state = 1;
					break;
				}
			} /* list */
			if (state == 1) continue;
                } /* state */
		/* value */
                if (state == 1 && cmd) {
		    cmd->value = response;
		    cmd->state = 1; /* command complete */
		    state = 0;
		    list_del(&cmd->link);
		    complete(&cmd->cmd_wait);		    
		    cmd = NULL;
		    continue;
		}
    
		/* Unsol response caching ...
		 * We don't have pending requests and we don't want to lose
		 * unsol response. This is changed in latest firmwares, but 
		 * may create problems when sending cmds and writing date.
		 * Cache the last unsol response .. if we ever enter in this 
		 * condition.
		 */
		if (state == 0 && 
			(DSP_CMD_MATCH(response, 0xff000000, 0x81000000) ||
			DSP_CMD_MATCH(response, 0xff000000, 0x02000000))) {
			cs48l10->cached_unsol.command = response;
			state = 1;
			continue;	
		}

                if (state == 1) {
    		    cs48l10->cached_unsol.value = response;
		    cs48l10->has_cached_unsol = 1;
		    state = 0;
		    dev_info(&cs48l10->spi->dev, "%s() cached %08x, val %d\n",
			    __FUNCTION__,
			    cs48l10->cached_unsol.command, 
			    cs48l10->cached_unsol.value );
		}

        }
	mutex_unlock(&cs48l10->cmd_lock);
}                       


static void dsp_clear_queue(struct cs48l10 *cs48l10)
{
	struct dsp_cmd* c;
	mutex_lock(&cs48l10->cmd_lock);
	list_for_each_entry(c, &cs48l10->cmd_list, link) {
		c->command = 0;
		c->value = 0;
		c->state = 0;
		complete(&c->cmd_wait);
	} /* list */
	mutex_unlock(&cs48l10->cmd_lock);
}


static void dsp_queue_cmd(struct cs48l10 *cs48l10, struct dsp_cmd *cmd)
{
	mutex_lock(&cs48l10->cmd_lock);
        list_add_tail(&cmd->link, &cs48l10->cmd_list);
        if (!gpio_get_value(cs48l10->gpio_int))
    	        schedule_work(&cs48l10->int_work);
	mutex_unlock(&cs48l10->cmd_lock);
}

/*
 * Post a request for read of a DSP word at 
 * a given index. Waits for dispatch()
 */
#if 0
static int dsp_read_cmd_async(struct cs48l10* cs48l10, u16 opcode, 
	u16 index, u32 mask, u32* value)
{
	int ret;
        struct dsp_cmd* cmd = (struct dsp_cmd*) 
		kzalloc(sizeof(struct dsp_cmd), GFP_KERNEL);	

	u32 request = MK_CMD(opcode, index);

	if (!cmd)
		return -ENOMEM;

	cmd->command = request;
	cmd->mask = mask;
	cmd->value = 0;
	init_completion(&cmd->cmd_wait);
	dsp_queue_cmd(cs48l10, cmd);

	/* spi write */	
        if ((ret = cs48l10_write_dsp_word(cs48l10, request)) < 0) {
    		dev_err(&cs48l10->spi->dev, "read request %08x failed\n",
                                request);
                return ret;
        }

	/* wait for for dispatch */
	ret = wait_for_completion_interruptible_timeout(&cmd->cmd_wait, 1 * HZ);
	mutex_lock(&cs48l10->cmd_lock);
	if (ret > 0 && cmd->state == 1) {
		/* dispatched command are removed from the list */
		*value = cmd->value;
		kfree(cmd);
		dev_dbg(&cs48l10->spi->dev, 
			"%s(): rq:%08x mask:%08x, resp: %08x, %08x\n",
			__FUNCTION__, request, mask, cmd->command, cmd->value );
		mutex_unlock(&cs48l10->cmd_lock);
		return ret;    		
        }
	*value = 0;

	if (ret == 0)
		dev_err(&cs48l10->spi->dev, "%s timeout\n", __FUNCTION__);
	if (ret < 0)
		dev_err(&cs48l10->spi->dev, "%s error %d\n",__FUNCTION__, 
		ret);
	if (ret > 0 && cmd->state)
		dev_err(&cs48l10->spi->dev, "%s cmd dropped\n", __FUNCTION__);

    	list_del(&cmd->link);
	kfree(cmd);

	mutex_unlock(&cs48l10->cmd_lock);

	return ret;
}
#endif //#if 0
/* read/write by module */
/* #define dsp_read(cs48l10, mod, index, value) \
	dsp_read_cmd_async(cs48l10, MK_OPRD(mod), index, 0x7FFFFFFF, value); */

/* Unsolicited wait from the DSP */
 
#if 0
static int dsp_wait_unsol_async(struct cs48l10* cs48l10, 
	    u32 unsol_prefix, u32 mask,
	    u32* unsol_response, u32* unsol_value)
{
	int ret;
        struct dsp_cmd* cmd;

	/* check if we read something ... */
	mutex_lock (&cs48l10->cmd_lock);
	if (cs48l10->has_cached_unsol) {
        	if ((cs48l10->cached_unsol.command & mask) == 
			(unsol_prefix & mask)) {
            		*unsol_response = cs48l10->cached_unsol.command;
			*unsol_value = cs48l10->cached_unsol.value;
			cs48l10->has_cached_unsol = 0;
			mutex_unlock (&cs48l10->cmd_lock);
			return 1;
		}
	}

	mutex_unlock (&cs48l10->cmd_lock);

        cmd = (struct dsp_cmd*) kzalloc(sizeof(struct dsp_cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->command = unsol_prefix;
	cmd->mask = mask;
	cmd->value = 0;
	init_completion(&cmd->cmd_wait);
	dsp_queue_cmd(cs48l10, cmd);

	/* wait for for dispatch */
	ret = wait_for_completion_interruptible_timeout(&cmd->cmd_wait,
						cs48l10->unsol_timeout);
	mutex_lock(&cs48l10->cmd_lock);
	if (ret > 0 && cmd->state == 1) {
		/* dispatched command are removed from the list */
		*unsol_response = cmd->command;
		*unsol_value = cmd->value;
	
		kfree(cmd);
		dev_dbg(&cs48l10->spi->dev, "%s() unsol result %08x, %08x\n",
			__FUNCTION__, cmd->command, cmd->value );
		mutex_unlock(&cs48l10->cmd_lock);
		return ret;    		
        }
	*unsol_response = 0;
	*unsol_value = 0;

	if (ret == 0)
		dev_err(&cs48l10->spi->dev, "%s() timeout\n", __FUNCTION__);
	if (ret < 0)
		dev_err(&cs48l10->spi->dev, "%s() error %d\n",__FUNCTION__,
									ret);
	if (ret > 0 && cmd->state == 0) {
		dev_err(&cs48l10->spi->dev, "%s() cmd dropped\n", __FUNCTION__);
    		ret = -EINTR;
	}

        /* timeout or error */
    	list_del(&cmd->link);
	kfree(cmd);
	mutex_unlock(&cs48l10->cmd_lock);

	return ret;    		
}
#endif //#if 0 


#define CS48L10_MAX_UNSOL_MESS 1
#define CS48L10_MAX_LIST_SIZE 50
/* Back end task for handling unsolicited messages on interrupt */
static void unsol_dsp_mess(struct work_struct *work)
{
	int i;
	u32 dsp_data1;
	u32 dsp_data2;
	struct spi_device *spi_dev;

	struct cs48l10 *cs48l10 = 
                container_of(work, struct cs48l10, unsol_work);

	struct unsol_message *node = kzalloc(sizeof(struct unsol_message),
						GFP_KERNEL);
	if (node == NULL) { 
		dev_err(&cs48l10->spi->dev,
			"Cannot allocate memory in %s\n", __FUNCTION__);
		return;
	}
	spi_dev = cs48l10->spi;
	
	
	mutex_lock(&cs48l10->lock);

	for(i = 0; i < CS48L10_MAX_UNSOL_MESS; i++) {
		/* Unsolicited messages are two 32 bit words. */
    		cs48l10_read_dsp_word(cs48l10, &dsp_data1);
    		cs48l10_read_dsp_word(cs48l10, &dsp_data2);
		/* Unique error id */
		node->error_id = cs48l10->error_id;
		cs48l10->error_id++;
		node->data[0] = dsp_data1;
		node->data[1] = dsp_data2;
		/* if the return value is boot ready or app start we don't want it. */
		if((dsp_data1 == CS48L10_BOOT_READY && dsp_data2 == CS48L10_BOOT_READY) ||
		(dsp_data1 == CS48L10_APP_START && dsp_data2 == CS48L10_APP_START))
			break;

		if(cs48l10->unsol_length >= CS48L10_MAX_LIST_SIZE) {
			/*Remove the first element */
			struct unsol_message *del_node = list_first_entry(&cs48l10->unsol_list,
								struct unsol_message, link); 
			if(!del_node) {
				dev_err(&cs48l10->spi->dev,
				"There should be elements in the list %s()\n", __FUNCTION__);
				break;
			}
		    	list_del(&del_node->link);
			kfree(del_node);
		} else {
			cs48l10->unsol_length++;
		}
        	list_add_tail(&node->link, &cs48l10->unsol_list);
		if (gpio_get_value(cs48l10->gpio_int))
			break;
	}
	/* Send event to user space */
	kobject_uevent(&spi_dev->dev.kobj, KOBJ_CHANGE);
	mutex_unlock(&cs48l10->lock);
}


/* sysfs entries */
static ssize_t cs48l10_project_id_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;

	if (!fw || !fw->pmaster_list) {
		dev_err(&cs48l10->spi->dev, "Firmware is not loaded\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%u\n", fw->active_project);
}


static ssize_t cs48l10_reset_dsp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	gpio_set_value(cs48l10->gpio_reset, 0);
	udelay(100);
	cs48l10->dsp_reset_count++;
	gpio_set_value(cs48l10->gpio_reset, 1);
	udelay(100);

	/* After reset, no longer in hibernate or sleep mode */
	cs48l10->hibernate = 0; 
	cs48l10->sleep_mode = 0; 
	return count;
}



static ssize_t cs48l10_reset_dsp_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return -1;
}

static ssize_t cs48l10_reset_count_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return -EINVAL;
}
static ssize_t cs48l10_reset_count_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", cs48l10->dsp_reset_count);
}


static ssize_t cs48l10_sleep_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return -EINVAL;
}



static ssize_t cs48l10_sleep_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", cs48l10->sleep_mode);
}


static ssize_t cs48l10_project_id_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	unsigned long project_id = 0;
	int ret;

	if (!fw || !fw->pmaster_list)
		return -EINVAL;

	ret = strict_strtoul(buf, 10, &project_id);
	if (ret)
		return ret;
	
	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Invalid Project id = %d\n",
			 (int)project_id);
		return -EINVAL;
	}


	mutex_lock(&cs48l10->lock);

	if(cs48l10->boot_assist_project >= 0) {
		cs48l10->boot_assist = true;
		ret = cs48l10_fw_bootassist_project(cs48l10,
		project_id, INITIAL_CFG);
		if(ret < 0) 
			dev_err(&cs48l10->spi->dev,
			"Boot assist project failed to start 0x%x\n", ret);
	}
	else {
		ret = cs48l10_fw_softboot_project(cs48l10,
		project_id, INITIAL_CFG);
		if (ret >= 0) {
			mdelay(10);
			/* set low power mode (hibernate) */
			/* dsp_write(cs48l10, DSP_OS, SOFTBOOT, 0x11); */
			/* cs48l10->hibernate = 1; */
		} 
		else {
			dev_err(&cs48l10->spi->dev,
				"Project %d failed to start, ret = %d\n",
				CS48L10_DEFAULT_PROJECT_ID, ret);

		}
	}

	mutex_unlock(&cs48l10->lock);

	if (ret < 0) 
		return ret;

	return count;
}

/* This function shows the current snapshot id that is being used. */
static ssize_t cs48l10_snapshot_id_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;

	if (!fw || !fw->pmaster_list) {
		dev_err(&cs48l10->spi->dev, "Firmware is not loaded\n");
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", fw->current_snapshot);
}




/* This function changes the snapshot that is currently being used. */
static ssize_t cs48l10_snapshot_id_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	unsigned long snapshot_id = 0;
	int ret=0, wake_count;
	bool no_init_snapshot = false;
	char snapshot_name[MAX_DISPLAY_LENGTH];
	u32 dsp_result;

	memset(&snapshot_name[0], 0, MAX_DISPLAY_LENGTH);
	if (!fw || !fw->pmaster_list) {
		dev_err(&cs48l10->spi->dev, "Firmware is not loaded\n");
		return -EINVAL;
	}
	strict_strtoul(buf, 10, &snapshot_id);

	if (snapshot_id >= cpu_to_be32(fw->pmaster_list->cfgs_per_project)
		 || snapshot_id < INITIAL_CFG) {
		dev_err(&cs48l10->spi->dev,
			"Invalid snapshot_id %lu\n", snapshot_id);
		return count;
	}

	mutex_lock(&cs48l10->lock);
	cmp_micro_condenser_get_snapshot_name 
	(
		cs48l10,
		fw->active_project,
		snapshot_id,  
		snapshot_name,
		sizeof(snapshot_name)
	);
	if(!strncmp(CS48L10_SLEEP_SNAPSHOT_NAME, snapshot_name,
			strlen(CS48L10_SLEEP_SNAPSHOT_NAME))){
		no_init_snapshot = true;
	} else if(!strncmp(CS48L10_WAKE_SNAPSHOT_NAME, snapshot_name,
		strlen(CS48L10_WAKE_SNAPSHOT_NAME))) { 
		no_init_snapshot = true;
		for(wake_count = 0; wake_count < 3; wake_count++) {
			cs48l10_write_then_read(cs48l10,
			Wakeup_cmd, ARRAY_SIZE(Wakeup_cmd),
			&dsp_result, 1);
			if(dsp_result == CS48L10_APP_START)
				break;
		}
		if(wake_count == 3){
			dev_err(&cs48l10->spi->dev,
			"Failed to set wake up command\n");
			return count;
		}	
	} else {
		no_init_snapshot = false;
	}
	
	/* Check to see if the "wake" or "sleep" snapshots are being set. 
	 * If not, load the INITIAL_CFG snapshot first.
	 */
	if (snapshot_id != INITIAL_CFG && no_init_snapshot == false)
		ret = cs48l10_fw_load_config(cs48l10, INITIAL_CFG);
	
	if (!ret)
		ret = cs48l10_fw_load_config(cs48l10, snapshot_id);

	mutex_unlock(&cs48l10->lock);

	if (ret < 0) {
		dev_err(&cs48l10->spi->dev,
			"Snapshot %lu failed to load, ret = %d\n",
			snapshot_id, ret);
		fw->current_snapshot = -1;
		return ret;
	}
	fw->current_snapshot = snapshot_id;
	return count;
}

static ssize_t cs48l10_snapshot_name_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return 0;
}

static ssize_t cs48l10_snapshot_name_show(struct device *dev,
				       struct device_attribute *attr, 
                                       char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	
	if (!fw || !fw->pmaster_list) {
		dev_err(&cs48l10->spi->dev, "Firmware is not loaded\n");
		return -EINVAL;
	}

	cmp_micro_condenser_get_snapshot_name 
	(
		cs48l10,
		fw->active_project,
		fw->current_snapshot,
		buf,
		PAGE_SIZE
	);
	return strlen(buf);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief This function is a placeholder.  
/// 
/// You cannot change the project being used by this function.
///
/// \retval	-1	Return an errorcodes that this function is not available.
////////////////////////////////////////////////////////////////////////////////
static ssize_t cs48l10_project_name_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief This function shows the current project that is being used.
/// 
/// \param[in]		dev
/// \param[in]		attr
/// \param[out]		buf	The ascii string pointer that contains the current 
///                             project name.  
///
/// \retval	<0	Error occurred.
/// \retval	>0      String length of the project name.
////////////////////////////////////////////////////////////////////////////////
static ssize_t cs48l10_project_name_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
	struct cs48l10_firmware *fw = cs48l10->fw;

	//
	// Get the project name.
	//
	cmp_micro_condenser_get_project_name 
	(
		cs48l10,
		fw->current_project,
		buf,
		PAGE_SIZE
	);


	/* Return the last snapshot that was used */
	return strlen(buf);
}

static ssize_t cs48l10_urd(struct device *dev,
               struct device_attribute *attr, char *buf)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
        char        tmp_buffer[10] = "";
        u32         dsp_word;
        int         ret;

	//dev_info(&cs48l10->spi->dev,"CS48L10_URD\n");

        buf[0] = 0;

	/*TODO: Add interrupt wait here!!! */
               
	ret = cs48l10_read_dsp_word(cs48l10, &dsp_word);

	if(ret < 0) {
		dev_info(&cs48l10->spi->dev,"CS48L10: Read = NO DATA\n");
		return ret;            
        }

	//dev_info(&cs48l10->spi->dev,"CS48L10: Read = 0x%08x\n", dsp_word);

	snprintf(tmp_buffer, sizeof(tmp_buffer), "%08x",dsp_word);
	strlcat(buf, tmp_buffer, PAGE_SIZE);
        return strlen(buf);       
}


static ssize_t cs48l10_ucmd(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct cs48l10 *cs48l10 = dev_get_drvdata(dev);
    int     i, j;
    u8         c;
    u8         oc;
    u8          buffer[256];

    //dev_info(&cs48l10->spi->dev,"CS48L10_UCMD\n");

    j = 0;
    for(i = 0; i< count ; i++) {
	c = buf[i];
	if(c == ' ' )
	    continue;

    	if((j %2) == 0) 
		oc = 0;

	if(c >='0' && c <='9')
		oc |= c - '0';

    	else if (c >='A' && c <='F')
	     oc |= c - 'A' + 10;

    	else if (c >='a' && c <='f')
	      oc |= c -'a' + 10;

	if(j >256) {
		dev_info(&cs48l10->spi->dev,"CS48L10_UCMD: -EINVAL:%d\n", j);
		return -EINVAL;
	}
	
	      
    	if((j %2) == 0) 
	        oc = oc <<4;
	else
	       buffer[j>>1] = oc;
	j++;
    }

    if(j%8 != 1) {
        dev_err(&cs48l10->spi->dev,"CS48L10_UCMD: invalid hex format\n");
        return -EINVAL;
    }
    
    dev_dbg(&cs48l10->spi->dev,"CS48L10_UCMD: %d bytes\n", (j>>1));
    for ( i = 0 ; i < (j>>1) ; ++i ) {
	    dev_dbg(&cs48l10->spi->dev, "%02x ", buffer[i]);
    }
    dev_dbg(&cs48l10->spi->dev, "\n");
              
    i = cs48l10_write_buf(cs48l10, buffer, j >>1);
    if ( i != (j>>1) ) {
        dev_err(&cs48l10->spi->dev, "failed to cs48l10_write_buf (%d)\n", i);
    }

    return count;
}

static DEVICE_ATTR(ucmd_urd, 0400, cs48l10_urd, cs48l10_ucmd);
static DEVICE_ATTR(snapshot_name, 0400, cs48l10_snapshot_name_show,
		cs48l10_snapshot_name_store);
static DEVICE_ATTR(project_name, 0400, cs48l10_project_name_show,
		cs48l10_project_name_store);
static DEVICE_ATTR(snapshot_id, 0400,
	cs48l10_snapshot_id_show, cs48l10_snapshot_id_store);
static DEVICE_ATTR(project_id, 0400,
	cs48l10_project_id_show, cs48l10_project_id_store);
static DEVICE_ATTR(hibernate_mode, 0400, cs48l10_hibernate_mode_show,
	cs48l10_hibernate_mode_store); 
static DEVICE_ATTR(dsp_reset, 0400, cs48l10_reset_dsp_show,
	cs48l10_reset_dsp_store);
static DEVICE_ATTR(reset_count, 0400, cs48l10_reset_count_show,
	cs48l10_reset_count_store);
static DEVICE_ATTR(sleep_mode, 0400, cs48l10_sleep_mode_show,
	cs48l10_sleep_mode_store);
static DEVICE_ATTR(sw_version, 0400, cs48l10_drv_ver_show,
	cs48l10_drv_ver_store);
static DEVICE_ATTR(fw_version, 0400, cs48l10_fw_ver_show,
	cs48l10_fw_ver_store);
static DEVICE_ATTR(err_mess, 0400, cs48l10_err_mess_show,
	cs48l10_err_mess_store);

static struct attribute *cs48l10_attributes[] = {
    &dev_attr_project_id.attr,
    &dev_attr_snapshot_id.attr,
    &dev_attr_project_name.attr,
    &dev_attr_snapshot_name.attr,
    &dev_attr_ucmd_urd.attr,
    &dev_attr_hibernate_mode.attr,
	&dev_attr_dsp_reset.attr,
	&dev_attr_reset_count.attr,
	&dev_attr_sleep_mode.attr,
	&dev_attr_sw_version.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_err_mess.attr,
    NULL
};


static const struct attribute_group cs48l10_attr_group = {
	.attrs = cs48l10_attributes,
};

/* END sysfs */

static int __devinit cs48l10_probe(struct spi_device *spi)
{
	struct cs48l10 *cs48l10;
	int ret = 0;

	dev_info(&spi->dev, "CS48L10 driver %s loaded\n", CS48L10_DRV_VERSION);

	cs48l10 = kzalloc(sizeof(struct cs48l10), GFP_KERNEL);
	if (cs48l10 == NULL) {
		return -ENOMEM;
	}


	cs48l10->spi = spi;	/* cs48l10 to spi reference */

	mutex_init(&cs48l10->lock);
	mutex_init(&cs48l10->cmd_lock);
	init_waitqueue_head(&cs48l10->rx_data_wait);
	INIT_LIST_HEAD(&cs48l10->cmd_list);
	INIT_LIST_HEAD(&cs48l10->unsol_list);
	INIT_WORK(&cs48l10->int_work, dsp_cmd_dispatch);
	INIT_WORK(&cs48l10->unsol_work, unsol_dsp_mess);

	cs48l10->sol_spinlock = __SPIN_LOCK_UNLOCKED(cs48l10->sol_spinlock); 

	dev_set_drvdata(&spi->dev, cs48l10);	/* spi to cs48l10 reference */


	/* setup platform RESET/INT/BUSY functions */
	
	if ((ret = cs48l10_setup_gpios(spi, cs48l10)) < 0) {
		printk("Error: Can't set gpios. ret = %d\n", ret);
		kfree(cs48l10);
		return ret;
	}

	if ((ret = sysfs_create_group(&spi->dev.kobj,
		&cs48l10_attr_group)) < 0) {
		dev_err(&spi->dev, "failed to register sysfs\n");
		cs48l10_free_gpios(spi, cs48l10);
		kfree(cs48l10);
		return ret;
	}

	ret = cs48l10_spi_speed(cs48l10->spi, CONFIG_CS48L10_SPI_PREBOOT_FREQ);
	if (ret < 0)
		return ret;


#ifdef CS48L10_ASYNC_FWLOADER
	if ((ret = cs48l10_fw_load_async(cs48l10, 
		CS48L10_FIRMWARE_NAME)) < 0) {
		sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);
		cs48l10_free_gpios(spi, cs48l10);
		kfree(cs48l10);
		return ret;
	}
#else
	if ((ret = cs48l10_fw_load_sync(cs48l10, CS48L10_FIRMWARE_NAME)) < 0) {
		dev_err(&spi->dev,
			"Failed to load %s\n", CS48L10_FIRMWARE_NAME);

		sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);

		cs48l10_free_gpios(spi, cs48l10);

		kfree(cs48l10);

		return ret;
	}

	mutex_lock(&cs48l10->lock);

	/* Initialized to NOT be in sleep mode */
	cs48l10->sleep_mode = 0;
	cs48l10->hibernate = 0; 

	/* Get the name of the project that may have the boot assist and
	 * verify 
	 */
	struct cs48l10_firmware *fw = cs48l10->fw;
	struct project_display *pproject_name;
	int i;
	/* Set boot_assist_project to < 0 to identify that it was not set. */
	cs48l10->boot_assist_project = -1;  
	cs48l10->boot_assist = false;  

	for(i = 0; i < cpu_to_be32(fw->pmaster_list->num_projects); i++) {
		pproject_name = (struct project_display *)((u8 *) fw->project_names +
    		cpu_to_be32(fw->pdisplay_list->row_size) * i);
		if(!memcmp(pproject_name, CS48L10_BOOT_ASSIST_PROJECT_NAME, strlen(CS48L10_BOOT_ASSIST_PROJECT_NAME)))
		{
			cs48l10->boot_assist = true;  
			cs48l10->boot_assist_project = i;  
			break;
		}
	}

	if(cs48l10->boot_assist_project == CS48L10_DEFAULT_PROJECT_ID) {
		dev_err(&cs48l10->spi->dev,
		"Boot assist project can't be the same as the default.\n");
		return -EIO;
	}

	if(cs48l10->boot_assist) {
		ret = cs48l10_fw_bootassist_project(cs48l10,
		CS48L10_DEFAULT_PROJECT_ID, INITIAL_CFG);
		if(ret < 0) {
			dev_err(&cs48l10->spi->dev,
			"Boot assist project failed to start 0x%x\n", ret);
		}
	}
	else {
		ret = cs48l10_fw_boot_project(cs48l10,
		CS48L10_DEFAULT_PROJECT_ID, INITIAL_CFG);
		if (ret >= 0) {
			mdelay(10);
			/* set low power mode (hibernate) */
			/* dsp_write(cs48l10, DSP_OS, SOFTBOOT, 0x11); */
			/* cs48l10->hibernate = 1; */
		} 
		else {
			dev_err(&cs48l10->spi->dev,
				"Project %d failed to start, ret = %d\n",
				CS48L10_DEFAULT_PROJECT_ID, ret);

		}
	}

	mutex_unlock(&cs48l10->lock);
#endif

	return 0;
}

static int cs48l10_remove(struct spi_device *spi)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(&spi->dev);
	cs48l10_free_gpios(spi, cs48l10);

	sysfs_remove_group(&spi->dev.kobj, &cs48l10_attr_group);

	if (cs48l10->fw)
		kfree(cs48l10->fw);

	if (cs48l10->osfw)
		release_firmware(cs48l10->osfw);

	if (cs48l10)
		kfree(cs48l10);

	return 0;
}

/*
    Power management
*/
#ifdef CONFIG_PM
static int cs48l10_suspend(struct spi_device *spi, pm_message_t msg)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(&spi->dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret=0, i;
	char snapshot_name[MAX_DISPLAY_LENGTH];

	if (!fw || !fw->pmaster_list) {
		dev_err(&cs48l10->spi->dev, "Firmware is not loaded\n");
		return -EINVAL;
	}

	memset(&snapshot_name[0], 0, MAX_DISPLAY_LENGTH);
	if(cs48l10->hibernate)
		return 0;

	/* Start out at snapshot three because 0, 1, and 2 are 
	 * pre-kickstart, kickstart,
	 * and initial.
	 */
	for(i = 3; i < cpu_to_be32(fw->pmaster_list->cfgs_per_project); i++) {
		cmp_micro_condenser_get_snapshot_name 
		(
			cs48l10,
			fw->active_project,
			i,  
			snapshot_name,
			sizeof(snapshot_name)
		);
		if(!strncmp(CS48L10_SLEEP_SNAPSHOT_NAME, snapshot_name, 
				strlen(CS48L10_SLEEP_SNAPSHOT_NAME))) {
			/* Set the wakeup command first */
			ret = cs48l10_fw_load_config(cs48l10, i);
			if (ret < 0) {
				dev_err(&cs48l10->spi->dev,
				"Snapshot %d failed to load, ret = %d\n",
				i, ret);
				fw->current_snapshot = -1;
				goto err;
			}
			cs48l10->sleep_mode = 1;
			fw->current_snapshot = i;
			return 0;
		}
	}
	
	/* First, mute the DSP */
	for(i = 0; i < ARRAY_SIZE(Mute_cmd); i++) {
		if((ret = cs48l10_write_dsp_word(cs48l10,
			Mute_cmd[i])) < 0) {
			dev_err(&cs48l10->spi->dev,
			"Error during DSP write 0x%x\n", ret);
			goto err;
		}
	}//for

	/* Put the DSP to sleep. */
	for(i = 0; i < ARRAY_SIZE(Sleep_cmd); i++) {
		if((ret = cs48l10_write_dsp_word(cs48l10,
			Sleep_cmd[i])) < 0) {
			dev_err(&cs48l10->spi->dev,
			"Error during DSP write 0x%x\n", ret);
			goto err;
		}
	}//for

	cs48l10->sleep_mode = 1;
	return 0;
err:
	cs48l10->sleep_mode = 1;
	/* reset */
	gpio_set_value(cs48l10->gpio_reset, 0);
	udelay(100);
	cs48l10->dsp_reset_count++;

	return 0;
}

static int cs48l10_resume(struct spi_device *spi)
{
	struct cs48l10 *cs48l10 = dev_get_drvdata(&spi->dev);
	struct cs48l10_firmware *fw = cs48l10->fw;
	int ret=0;
	int project_id;

	if (!fw || !fw->pmaster_list) {
		dev_err(&cs48l10->spi->dev, "Firmware is not loaded\n");
		return -EINVAL;
	}

	if(cs48l10->hibernate)
		return 0;

	project_id = fw->current_project;
	if (project_id >= cpu_to_be32(fw->pmaster_list->num_projects)) {
		dev_err(&cs48l10->spi->dev,
			 "Invalid Project id = %d\n",
			 (int)project_id);
		return -EINVAL;
	}

	gpio_set_value(cs48l10->gpio_reset, 1); /* release reset */
	udelay(100);

	cs48l10->sleep_mode = 0;

	mutex_lock(&cs48l10->lock);

	if(cs48l10->boot_assist_project >= 0) {
		cs48l10->boot_assist = true;
		ret = cs48l10_fw_bootassist_project(cs48l10,
		project_id, INITIAL_CFG);
		if(ret < 0)
			dev_err(&cs48l10->spi->dev,
			"Boot assist project failed to start 0x%x\n", ret);
	}
	else {
		ret = cs48l10_fw_softboot_project(cs48l10,
		project_id, INITIAL_CFG);
		if (ret >= 0) {
			mdelay(10);
			/* set low power mode (hibernate) */
			/* dsp_write(cs48l10, DSP_OS, SOFTBOOT, 0x11); */
			/* cs48l10->hibernate = 1; */
		}
		else {
			dev_err(&cs48l10->spi->dev,
				"Project %d failed to start, ret = %d\n",
				CS48L10_DEFAULT_PROJECT_ID, ret);

		}
	}

	mutex_unlock(&cs48l10->lock);

	return 0;
}

#else
#define cs48l10_suspend NULL
#define cs48l10_resume  NULL
#endif

/*
    SPI driver
*/
static struct spi_driver cs48l10_driver = {
	.driver = {
		   .name = "cs48l10",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.suspend = cs48l10_suspend,
	.resume = cs48l10_resume,
	.probe = cs48l10_probe,
	.remove = __devexit_p(cs48l10_remove),
};

static int __init cs48l10_init(void)
{
	return spi_register_driver(&cs48l10_driver);
}

static void __exit cs48l10_exit(void)
{
	spi_unregister_driver(&cs48l10_driver);
}

module_init(cs48l10_init);
module_exit(cs48l10_exit);

MODULE_AUTHOR("Georgi Vlaev, Nucleus Systems, Ltd, <joe@nucleusys.com>");
MODULE_AUTHOR("Paul Handrigan, Cirrus Logic Inc., <phandrigan@cirrus.com>");
MODULE_DESCRIPTION("CS48L10 Audio DSP Driver");
MODULE_LICENSE("GPL");
