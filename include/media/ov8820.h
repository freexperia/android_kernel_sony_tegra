/* 2012-07-20: File changed by Sony Corporation */
/*
 * Copyright (C) 2010 Motorola, Inc.
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

#ifndef __OV8820_H__
#define __OV8820_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV8820_IOCTL_SET_MODE			_IOW('o', 1, struct ov8820_mode)
#define OV8820_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV8820_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV8820_IOCTL_SET_GAIN			_IOW('o', 4, __u16)
#define OV8820_IOCTL_GET_STATUS			_IOR('o', 5, __u8)
#define OV8820_IOCTL_SET_BINNING    	_IOW('o', 6, __u8)
#define OV8820_IOCTL_TEST_PATTERN		_IOW('o', 7, enum ov8820_test_pattern)
#define OV8820_IOCTL_SET_CAMERA_MODE	_IOW('o', 10, __u32)
#define OV8820_IOCTL_SYNC_SENSORS		_IOW('o', 11, __u32)
#define OV8820_IOCTL_SET_GROUP_HOLD	    _IOW('o', 12, struct tegra_camera_ae)
#define OV8820_IOCTL_GET_SENSOR_ID      _IOR('o', 13, __u8 *)

/* OV8820 registers */
#define OV8820_OTP_READ_NWRITE          (0x3D81)
#define OV8820_OTP_BANK_SELECT          (0x3D84)
#define OV8820_OTP_DATA_START           (0x3D00)
#define OV8820_OTP_DATA_END             (0x3D1F)
#define OV8820_OTP_BANK_EN_BIT          (1 << 3)
#define OV8820_OTP_READ_BIT             (1 << 0)
#define OV8820_FUSEID_REG0              (0x3D00)
#define OV8820_FUSEID_REG1              (0x3D01)
#define OV8820_FUSEID_REG2              (0x3D02)
#define OV8820_FUSEID_REG3              (0x3D03)
#define OV8820_FUSEID_REG4              (0x3D04)
#define OV8820_SRM_GRUP_ACCESS          (0x3212)
#define OV8820_GROUP_HOLD_BIT		(1 << 7)
#define OV8820_GROUP_LAUNCH_BIT		(1 << 5)
#define OV8820_GROUP_HOLD_END_BIT	(1 << 4)
#define OV8820_GROUP_ID(id)		(id)

enum ov8820_test_pattern {
	TEST_PATTERN_NONE,
	TEST_PATTERN_COLORBARS,
	TEST_PATTERN_CHECKERBOARD
};

struct ov8820_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

#ifdef __KERNEL__
struct ov8820_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	void (*synchronize_sensors)(void);
};
#endif /* __KERNEL__ */

#endif  /* __OV8820_H__ */

