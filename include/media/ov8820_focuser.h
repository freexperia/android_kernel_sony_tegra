/* 2012-07-20: File changed by Sony Corporation */
/*
 * Copyright (C) 2011 NVIDIA Corporation.
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

#ifndef __OV8820_FOCUSER_H__
#define __OV8820_FOCUSER_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV8820_FOC_IOCTL_GET_CONFIG   _IOR('o', 1, struct ov8820_foc_config)
#define OV8820_FOC_IOCTL_SET_POSITION _IOW('o', 2, u32)
#define OV8820_FOC_IOCTL_READ_OTP     _IOR('o', 3, struct ov8820_foc_config)

struct ov8820_foc_config {
	__u32 settle_time;
	__u32 actuator_range;
	__u32 pos_low;
	__u32 pos_high;
	float focal_length;
	float fnumber;
	float max_aperture;
};

struct ov8820_focuser_platform_data {
	int cfg;
	int num;
	int sync;
	const char *dev_name;
	struct nvc_focus_nvc (*nvc);
	struct nvc_focus_cap (*cap);
	struct ov8820_focuser_pdata_info (*info);
	__u8 i2c_addr_rom;
	unsigned gpio_reset;
/* Due to a Linux limitation, a GPIO is defined to "enable" the device.  This
 * workaround is for when the device's power GPIO's are behind an I2C expander.
 * The Linux limitation doesn't allow the I2C GPIO expander to be ready for
 * use when this device is probed.
 * When this problem is solved, this driver needs to hard-code the regulator
 * names (vreg_vdd & vreg_i2c) and remove the gpio_en WAR.
 */
	unsigned gpio_en;
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif
/* __OV8820_FOCUSER_H__ */

