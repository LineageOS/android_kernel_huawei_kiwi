/*
 * drivers/power/huawei_charger.h
 *
 *huawei charger driver
 *
 * Copyright (C) 2012-2015 HUAWEI, Inc.
 * Author: HUAWEI, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/usb/msm_hsusb.h>

#ifndef _CHARGER_CORE
#define _CHARGER_CORE

/*options of charge states from chip*/
#define CHARGELOG_SIZE      (1024)

#define MAX_CHARGER_TYPE_STRING_LEN        20
#define CHARGER_IC_QCOM                    0
#define CHARGER_IC_NO_DPM                  1
#define CHARGER_IC_HAVE_DPM                2

#define QCOM_LINEAR_CHARGER        0
#define TI_BQ24152_CHARGER         1
#define TI_BQ24192_CHARGER         2
#define TI_BQ24296_CHARGER      3
#define MAX77819_CHARGER      4
#define UNKNOW_CHARGER             10
#define DEFAULT_AC_IIN_CURRENT             1000

#define PMU_ERR  1
#define PMU_WARN  2
#define PMU_INFO  3
#define PMU_DBG  4
extern int pmu_debug_mask;

#define pmu_log_err(x...) \
        _pmu_log_err(HWLOG_TAG,##x)

#define _pmu_log_err(TAG,x...) \
        __pmu_log_err(TAG,##x)

#define __pmu_log_err(TAG,fmt, ...) \
    do{ \
        if(pmu_debug_mask >= PMU_ERR) \
            pr_err(hw_fmt_tag(TAG,E) fmt,##__VA_ARGS__);    \
    }while(0)
	
#define pmu_log_warn(x...) \
        _pmu_log_warn(HWLOG_TAG,##x)

#define _pmu_log_warn(TAG,x...) \
        __pmu_log_warn(TAG,##x)

#define __pmu_log_warn(TAG,fmt, ...) \
    do{ \
        if(pmu_debug_mask >= PMU_WARN) \
            pr_err(hw_fmt_tag(TAG,W) fmt,##__VA_ARGS__);    \
    }while(0)
	
#define pmu_log_info(x...) \
        _pmu_log_info(HWLOG_TAG,##x)

#define _pmu_log_info(TAG,x...) \
        __pmu_log_info(TAG,##x)

#define __pmu_log_info(TAG,fmt, ...) \
    do{ \
        if(pmu_debug_mask >= PMU_INFO) \
            pr_err(hw_fmt_tag(TAG,I) fmt,##__VA_ARGS__);    \
    }while(0)

#define pmu_log_debug(x...) \
        _pmu_log_debug(HWLOG_TAG,##x)

#define _pmu_log_debug(TAG,x...) \
        __pmu_log_debug(TAG,##x)

#define __pmu_log_debug(TAG,fmt, ...) \
    do{ \
        if(pmu_debug_mask >= PMU_DBG) \
            pr_err(hw_fmt_tag(TAG,D) fmt,##__VA_ARGS__);    \
    }while(0)

#define hw_fmt_tag(TAG,LEVEL) "[" #LEVEL "/" #TAG "] "

/*************************struct define area***************************/
struct charger_ic_type{
    char *charger_type_string;
    int  charger_index;
};

struct charge_core_data{
    unsigned int iin_ac;
};

struct huawei_charger_irq
{
    int		irq;
    unsigned long	disabled;
    bool            is_wake;
};

struct charger_core_info{
    struct charger_ic_type  charger_type_info;
    struct charge_core_data data;
    struct qpnp_vadc_chip   *vadc_dev;
    struct huawei_charger_irq		irqs;
};

struct charge_device_ops
{
    int (*set_runningtest)(int value);   /* for running test charging control*/
    int (*set_enable_charger)(int value); /* for factory diag function */
    int (*set_in_thermal)(int value); /* for power genius to set current limit */
    int (*shutdown_q4)(int value); /* shutdown q4 */
    int (*shutdown_wd)(int value); /* shutdown watchdog */
    int (*set_usb_current)(int value); /* set usb input current */
    int (*dump_register)(char *reg_value); /* dump charger regs*/
    int (*get_ibus)(void); /* get ibus current */
    int (*charge_event_poll_register)(struct device *dev); /* register charge event poll node*/
    int  (*get_bat_status)(void);
};

/****************variable and function declarationn area******************/
extern struct charger_core_info *charge_core_get_params(void);
extern int charge_ops_register(struct charge_device_ops *ops);
#endif
