/*
 * Copyright (C) 2012 Sony Corporation
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef __TXS_ACC_DET_H_INCLUDED__
#define __TXS_ACC_DET_H_INCLUDED__

#define ACC_DET_DEBUG_PRINT

#define TXS_ACC_NOT_CONNECTED  (-1)
#define TXS_ACC_NOT_ASSIGN_0    (0)
#define TXS_ACC_CHARGING_CRADLE (1)
#define TXS_ACC_NOT_ASSIGN_2    (2)
#define TXS_ACC_DOCKING_STAND   (3)
#define TXS_ACC_NOT_ASSIGN_4    (4)
#define TXS_ACC_DOCK_SPEAKER    (5)
#define TXS_ACC_NOT_ASSIGN_6    (6)
#define TXS_ACC_NOT_ASSIGN_7    (7)
#define TXS_ACC_NOT_ASSIGN_8    (8)
#define TXS_ACC_DCOUT_RESERVED  (9)
#define TXS_ACC_HDMI_DONGLE    (10)


struct txs_acc_det_platform_data {
	int det_pin_num;
};
struct txs_acc_dcout_platform_data {
	int dcout_pin_num;
};

/*
 *  get current val.
 *
 *  ret : accessory number
 *        less than 0 is not initialied.
 */
int txs_acc_det_get_num(void);

/*
 *  register callback function. callback when lid switch changed.
 *
 *  arg : "changed", address of callback function.
 *
 *  ret : if non 0, error occur.
 */
int txs_acc_det_register_callback( void (*changed)(void) );

/*
 *  unregister callback function.
 *
 *  arg : "changed", address of callback function.
 */
void txs_acc_det_unregister_callback( void (*changed)(void) );

#endif /* __TXS_ACC_DET_H_INCLUDED__ */
