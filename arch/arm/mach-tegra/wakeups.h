/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __WAKEUPS_H_
#define __WAKEUPS_H_

/* sets 64-bit wake mask argument bits for wake sources given irq */
int tegra_irq_to_wake(int irq, u64 *wake_msk);
/*
 * given wake source index, returns irq number or negative value for error
 */
int tegra_wake_to_irq(int wake);
/* disable selected wake source setting for particular board */
int tegra_disable_wake_source(unsigned int wake_index);

#endif /* end __WAKEUPS_H_ */
