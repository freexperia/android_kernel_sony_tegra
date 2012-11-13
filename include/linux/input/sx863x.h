/*
 * include/linux/input/sx863x.h
 *
 * SX863X devices are a line of devices used as cap buttons,
 * slider, wheel, and proximity. Different devices contain different
 * features listed above.
 * Start with SX8636 - 8 cap buttons w/ proximity sensing
 *
 * Copyright 2011 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */
#define FOXCONN_SX8636_SYSFS_ENABLED		1

#ifndef _SX863X_H_
#define _SX863X_H_

enum sx863x_proximity_status { CAP_SENSE_PROX, CAP_SENSE_NO_PROX };
extern enum sx863x_proximity_status proximity_status;

struct sx863x_button_platform_data {
	int keycode;
	unsigned char capstatelsb_Mask;
};

struct sx863x_platform_data {
	int button_num;
  int i2c_reg_num;
  int spm_cfg_num;
	struct sx863x_button_platform_data *button;
  struct sx863x_reg_platform_data *i2c_reg;
  struct sx863x_reg_platform_data *spm_cfg;
};
struct sx863x_reg_platform_data {
  unsigned char reg;
  unsigned char val;
};

#endif
