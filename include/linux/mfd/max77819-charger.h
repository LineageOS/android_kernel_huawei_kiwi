/*
 * Maxim MAX77819 Charger Driver Header
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77819_CHARGER_H__
#define __MAX77819_CHARGER_H__

// remove current macos

struct max77819_charger_platform_data {
    bool disable_interrupt;
    int irq;

    char *psy_name;
    char *ext_psy_name;

    char **supplied_to;
    size_t num_supplicants;

    u32 chg_dcin_current_max;        /* in uA; 0.00A ~ 1.80A */
    u32 chg_fast_current_max;        /* in uA; 0.00A ~ 1.80A */
    u32 charge_termination_voltage; /* in uV; 4.10V ~ 4.35V */
    u32 topoff_timer;               /* in min; 0min ~ 60min, infinite */
    u32 topoff_current;             /* in uA; 50mA ~ 400mA */
    u32 charge_restart_threshold;   /* in uV; 100mV ~ 150mV */

    /* Co-operating charger */
    bool enable_coop;
    char *coop_psy_name;

    /* Temperature regulation */
    bool enable_thermistor;

    /* AICL control */
    bool enable_aicl;
    u32 aicl_detection_voltage;     /* in uV; 3.9V ~ 4.8V  */
    u32 aicl_reset_threshold;       /* in uV; 100mV or 200mV */
};

/* for compatibility with kernel not from android/kernel/msm */
/* remove redefine power_supply prop menu macos */

#endif /* !__MAX77819_CHARGER_H__ */
