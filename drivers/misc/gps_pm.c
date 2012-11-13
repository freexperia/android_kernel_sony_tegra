/* 2011-06-10: File added and changed by Sony Corporation */
/*
 * drivers/misc/gps_pm.c
 *
 * Copyright (c) 2008-2010, NVIDIA Corporation.
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

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/percpu.h>

#include "wireless_power_control.h"

#define DRIVER_NAME "gps_pm"
#undef VERBOSE_DEBUG

#define dbg(STR, ARGS...)                                  \
        do {                                               \
            printk("%s: " STR "\n" , DRIVER_NAME, ##ARGS); \
        } while (0)                                        

#ifdef VERBOSE_DEBUG
#define vdbg(STR, ARGS...)                                            \
        do {                                                          \
            printk(KERN_DEBUG "%s: " STR "\n" , DRIVER_NAME, ##ARGS); \
        } while (0)                                                   
#else
#define vdbg(STR, ARGS...) {}
#endif

#if defined(CONFIG_PM)
//
// /sys/power/gps/notifier
//

int gps_power = 0;

struct kobject *gps_kobj;

// Reading blocks if the value is not available.
static ssize_t
gps_pm_notifier_show(struct kobject *kobj, struct kobj_attribute *attr,
                   char *buf)
{
    int nchar;

    // Return the value, and clear.
    vdbg("returning with gps_power = '%d'", gps_power);
    nchar = snprintf(buf, 2, "%d\n", gps_power);
    return nchar;
}

// Writing is no blocking.
static ssize_t
gps_pm_notifier_store(struct kobject *kobj, struct kobj_attribute *attr,
                    const char *buf, size_t count)
{
    sscanf(buf, "%d", &gps_power);
    vdbg("gps_power = %d", gps_power);
    wireless_power_control(WPC_MODULE_GPS, gps_power);
   
    return count;
}


static struct kobj_attribute gps_pm_notifier_attribute =
       __ATTR(notifier, 0400, gps_pm_notifier_show, gps_pm_notifier_store);


int gps_pm_notifier(struct notifier_block *nb,
                      unsigned long event, void *nouse)
{
    vdbg("start processing event=%lx", event);

    // Notify the event to GPS HAL.
    switch (event) {
    case PM_SUSPEND_PREPARE:
        wireless_power_control(WPC_MODULE_GPS, 0/*off*/);
        break;
    case PM_POST_SUSPEND:
        wireless_power_control(WPC_MODULE_GPS, gps_power);
        break;
    default:
        dbg("unknown event %ld", event);
        return NOTIFY_DONE;
    }

    vdbg("finished processing event=%ld", event);
    return NOTIFY_OK;
}
#endif


static int __init gps_pm_init(void)
{
    int ret = 0;
    vdbg("init");

    #if defined(CONFIG_PM)
    // Create /sys/power/gps/notifier.
    gps_kobj = kobject_create_and_add("gps", power_kobj);
    ret = sysfs_create_file(gps_kobj, &gps_pm_notifier_attribute.attr);
    if(ret) {
        dbg("entry with the given name already exists");
        return ret;
    }

    // Register PM notifier.
    pm_notifier(gps_pm_notifier, 1);
    #endif

    return ret;
}

static void __exit gps_pm_deinit(void)
{
    vdbg("deinit");
}

module_init(gps_pm_init);
module_exit(gps_pm_deinit);
