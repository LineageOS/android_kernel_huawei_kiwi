/*
 * drivers/power/bq24296m_charger.c
 *
 * BQ24296M charging driver
 *
 * Copyright (C) 2012-2015 HUAWEI, Inc.
 * Author: HUAWEI, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/power/bq24296m_charger.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <linux/power/huawei_charger.h>
#include <linux/charger_core.h>
/* move code to header file */
#define HWLOG_TAG  bq24296_charger
/* move code to header file */


static struct wake_lock chrg_lock;
static struct wake_lock stop_chrg_lock;
u32 wakeup_timer_seconds;

static unsigned int irq_int_active;
/* remove unused old code */
static unsigned int input_current_iin;
static unsigned int input_current_ichg;
static unsigned int iin_temp = 0;
static unsigned int max_currentmA_bak = 0;
static unsigned int ichg_temp = 0;
static unsigned int iin_limit_temp = 1;
static unsigned int ichg_limit_temp = 1;
static int regulation_voltage_changed = 0;
static unsigned int prior_shutdown_wd = 0;
static unsigned int prior_shutdown_q4 = 0;
static unsigned int usb_current = 0;
typedef enum VBUS_DPM_STATUS_TAG
{
    VBUS_DPM_STATUS_UNKNOWN, //vbus_out
    VBUS_DPM_STATUS_CHECKING, //vbus_in
    VBUS_DPM_STATUS_GOOD,
    VBUS_DPM_STATUS_POOR,
}VBUS_DPM_STATUS_TYPE;

struct dpm_point_tag{
       int bat_vol_max;
       int bat_vol_roll_back;
       int dpm_vol;
};

static struct mutex  dpm_status_access_lock;
static VBUS_DPM_STATUS_TYPE g_vbus_dpm_status = VBUS_DPM_STATUS_UNKNOWN;
#define VBUS_DPM_STATUS_CHECK_CNT (2)
#define DPM_POINT_CNT (4)

static struct dpm_point_tag dpm_point_tbl[DPM_POINT_CNT] = {
   {3900, 3850, 4280},
   {4000, 3950, 4440},
   {4200, 4150, 4520},
   {4250, 4200, 4600}};
/**just for test**/
/*************/

enum
{
    BATTERY_HEALTH_TEMPERATURE_NORMAL = 0,
    BATTERY_HEALTH_TEMPERATURE_OVERLOW,
    BATTERY_HEALTH_TEMPERATURE_LOW,
    BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH,
    BATTERY_HEALTH_TEMPERATURE_HIGH,
    BATTERY_HEALTH_TEMPERATURE_HIGH_HOT,
    BATTERY_HEALTH_TEMPERATURE_OVERHIGH,
    BATTERY_HEALTH_TEMPERATURE_10,
    BATTERY_HEALTH_TEMPERATURE_5,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP1,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP2,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP3,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP4,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP5,
};

/* remove unused old code */

enum
{
    NORMAL_TEMP_CONFIG = 0,  // (BATTERY_HEALTH_TEMPERATURE_NORMAL)
    NORMAL_HIGH_TEMP_CONFIG, // (BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH)
    HIGH_TEMP_CONFIG,        // (BATTERY_HEALTH_TEMPERATURE_HIGH)
    HIGH_HOT_CONFIG,
    COOL_CONFIG,
    TEMP_CONFIG_10,
    TEMP_CONFIG_5,
    NORMAL_SECOND_STAGE,//for two charging stage
    NORMAL_HIGH_TEMP_CONFIG_CP1,
    NORMAL_HIGH_TEMP_CONFIG_CP2,
    NORMAL_HIGH_TEMP_CONFIG_CP3,
    NORMAL_HIGH_TEMP_CONFIG_CP4,
};

/* remove old temp control code */

#define bq24296m_bms_psy(di)\
({\
    if (!di->bms_name)\
    {\
        di->bms_name = POWR_SUPPLY_BMS_NAME;\
    }\
    if (!di->bms_psy)\
    {\
        di->bms_psy = power_supply_get_by_name(di->bms_name);\
        if (!di->bms_psy)\
        {\
            pr_err("the battery power_supply is not got!!\n");\
        }\
    }\
    di->bms_psy;\
})

#define bq24296m_charger_get_ext_property(pwr_spy, pwr_psp, ptr_val)\
({\
    int rc = -ENXIO;\
    if ((pwr_spy) && (pwr_spy)->get_property) {\
        rc = (pwr_spy)->get_property((pwr_spy), (pwr_psp), (ptr_val));\
    }\
    if (unlikely(IS_ERR_VALUE(rc))) {\
        (ptr_val)->intval = 0;\
    }\
    rc;\
})

struct bq24296m_temp_control_info bq24296_temp_info = {
    .cold_bat_degree = 0, //default cold bat degree: 0 degree
    .cool_bat_degree = 100, //default cool bat degree: 10 degree
    .imaxma_cool_bat = 600, //default cool bat charge current: 600mA
    .vmaxmv_cool_bat = 4350, //default cool bat max voltage: 4350mV
    .warm_bat_degree = 420, //default warm bat degree: 42 degree
    .imaxma_warm_bat = 800, //default warm bat charge current: 600mA
    .vmaxmv_warm_bat = 4100, //default warm bat max voltage: 4100mV
    .hot_bat_degree = 520, //default hot bat degree: 52 degree
};

static struct bq24296m_device_info *bq_device = NULL;

/* move function bq24296m_charger_present to other place*/

static int bq24296_get_ibus(void);

static struct kobject *g_sysfs_poll = NULL;

static ssize_t get_poll_charge_start_event(struct device *dev,
                       struct device_attribute *attr, char *buf)
{
    struct bq24296m_device_info *di = bq_device;

    if(di)
    {
        return sprintf(buf, "%d\n", di->input_event);
    }
    else
    {
        return 0;
    }
}

static ssize_t set_poll_charge_event(struct device *dev,
                       struct device_attribute *attr, const char *buf, size_t count)
{
    struct bq24296m_device_info *di = bq_device;
    long val = 0;

    if(di)
    {
        if((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 3000))
        {
            return -EINVAL;
        }

        di->input_event = val;
        sysfs_notify(g_sysfs_poll, NULL, "poll_charge_start_event");
    }

    return count;
}

static DEVICE_ATTR(poll_charge_start_event, (S_IWUSR | S_IRUGO),
                        get_poll_charge_start_event,
                        set_poll_charge_event);

static int charge_event_poll_register(struct device *dev)
{
    int ret;

    ret = sysfs_create_file(&dev->kobj, &dev_attr_poll_charge_start_event.attr);
    if(ret)
    {
        pr_err("fail to create poll node for %s\n", dev->kobj.name);
        return ret;
    }
    g_sysfs_poll = &dev->kobj;

    return ret;
}
static void charge_event_notify(unsigned int event)
{
    struct bq24296m_device_info *di = bq_device;

    if(!di)
    {
        pmu_log_info("bq24296m device is not init, do nothing!\n");
        return;
    }
    /* avoid notify charge stop event continuously without charger inserted */
    if((di->input_event != event) || (event == BQ2419x_START_USB_CHARGING)
                                  || (event == BQ2419x_START_AC_CHARGING))
    {
        di->input_event = event;
        if(g_sysfs_poll)
        {
            sysfs_notify(g_sysfs_poll, NULL, "poll_charge_start_event");
        }
    }
}

static int bq24296m_get_battery_capacity(struct bq24296m_device_info *di)
{
    int rc = 0;
    union power_supply_propval bat_soc_val = {0};

    if (di->fake_battery_soc >= 0 && di->fake_battery_soc <= BATT_FULL)
    {
        /* return the user-defined battery soc */
        return di->fake_battery_soc;
    }
    if (!bq24296m_bms_psy(di))
    {
        return BATT_DEFAULT_SOC;
    }

    if (di->bms_psy)
        rc = di->bms_psy->get_property(di->bms_psy,
                                            POWER_SUPPLY_PROP_CAPACITY, &bat_soc_val);
    if (unlikely(IS_ERR_VALUE(rc)))
    {
        pr_err("get battery capacity failed! rc = %d \n", rc);
        return BATT_DEFAULT_SOC;
    }

    pmu_log_debug("get battery capacity =%d \n", bat_soc_val.intval);

    return bat_soc_val.intval;
}

static int bq24296m_parse_dts_charge_parameter(struct bq24296m_device_info *di)
{
    int ret = 0;
    struct device_node *np = di->client->dev.of_node;

    ret = of_property_read_u32(np, "ti,max_charger_currentmA",
                               &di->max_currentmA);
    if (ret)
        return 0;
    max_currentmA_bak = di->max_currentmA;

    ret = of_property_read_u32(np, "ti,max_charger_voltagemV",
                               &di->max_voltagemV);
    if (ret)
        return 0;

    ret = of_property_read_u32(np, "ti,max_cin_limit_currentmA",
                               &di->max_cin_currentmA);
    if (ret)
        return 0;

    di->gpio_cd_level_reverse = of_property_read_bool(np, "ti,gpio_cd_level_reverse");

    di->no_ibus_detect = of_property_read_bool(np, "ti,no_ibus_detect");
    ret = of_property_read_string(np, "bms-name", &di->bms_name);
    if (ret)
    {
        di->bms_name = POWR_SUPPLY_BMS_NAME;
    }
    pmu_log_info("use bms (%s) in charger driver\n", di->bms_name);
    return 1;
}

static int bq24296m_write_block(struct bq24296m_device_info *di, u8 *value, u8 reg, unsigned num_bytes)
{
    struct i2c_msg msg[1];
    int ret = 0;

    *value = reg;

    msg[0].addr = di->client->addr;
    msg[0].flags = 0;
    msg[0].buf = value;
    msg[0].len = num_bytes + 1;

    ret = i2c_transfer(di->client->adapter, msg, 1);

    /* i2c_transfer returns number of messages transferred */
    if (ret != 1)
    {
        pr_err("i2c_write failed to transfer all messages\n");
        if (ret < 0)
            return ret;
        else
            return -EIO;
    }
    else
    {
        return 0;
    }
}

static int bq24296m_read_block(struct bq24296m_device_info *di, u8 *value,u8 reg, unsigned num_bytes)
{
    struct i2c_msg msg[2];
    u8 buf = 0;
    int ret = 0;

    buf = reg;

    msg[0].addr = di->client->addr;
    msg[0].flags = 0;
    msg[0].buf = &buf;
    msg[0].len = 1;

    msg[1].addr = di->client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = value;
    msg[1].len = num_bytes;

    ret = i2c_transfer(di->client->adapter, msg, 2);

    /* i2c_transfer returns number of messages transferred */
    if (ret != 2)
    {
        pr_err("i2c_write failed to transfer all messages\n");
        if (ret < 0)
            return ret;
        else
            return -EIO;
    }
    else
    {
        return 0;
    }
}

static int bq24296m_write_byte(struct bq24296m_device_info *di, u8 value, u8 reg)
{
    /* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
    u8 temp_buffer[2] = { 0 };

    /* offset 1 contains the data */
    temp_buffer[1] = value;
    return bq24296m_write_block(di, temp_buffer, reg, 1);
}

static int bq24296m_read_byte(struct bq24296m_device_info *di, u8 *value, u8 reg)
{
    return bq24296m_read_block(di, value, reg, 1);
}

static int bq24296m_is_otg_enable(struct bq24296m_device_info *di)
{
    u8 read_reg = 0;
    int rc = 0;

    rc = bq24296m_read_byte(di, &read_reg, POWER_ON_CONFIG_REG01);
    if (rc)
    {
        pmu_log_err("failed to read POWER_ON_CONFIG_REG01\n");
        return 0;
    }
    pmu_log_debug("otg_en 0x%x\n", read_reg);
    return  (read_reg & BQ_OTG_EN) ? 1 : 0;
}

static bool bq24296m_charger_present(struct bq24296m_device_info *di)
{
    if(NULL == di){
        pmu_log_err("bq24296m device is not init, do nothing!\n");
        return false;
    }

    if(bq24296m_is_otg_enable(di)){
        return false;
    }else{
        return (1 == is_usb_chg_exist());
    }
}

int bq24296m_get_charge_type(struct bq24296m_device_info *di)
{
    int rc;
    u8 read_reg = 0;

    rc = bq24296m_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if (rc)
    {
        pr_err("failed to read SYS_STSTUS_REG08");
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }

    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_PRE_CHARGING)
        return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_FAST_CHARGING)
        return POWER_SUPPLY_CHARGE_TYPE_FAST;

    return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int bq24296m_get_battery_health(void)
{
    int temp = 0;
    temp = huawei_charger_get_battery_temperature();
    if(temp >= BATT_TEMP_MAX)
    {
        return POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    else
    {
        return POWER_SUPPLY_HEALTH_GOOD;
    }
}

static int bq24296m_config_hot_limit_current(struct bq24296m_device_info *di)
{
    /* thermal design will set this as 0 when cpu cools down*/

    if (di->hot_limit_current == 0)
    {
        mutex_lock(&di->hot_limit_lock);
        di->hot_limit_current = di->max_cin_currentmA;
        mutex_unlock(&di->hot_limit_lock);
    }

    return di->cin_limit <= di->hot_limit_current ? di->cin_limit : di->hot_limit_current;
}

static void bq24296m_config_input_source_reg(struct bq24296m_device_info *di)
{
    unsigned int vindpm = 0;
    unsigned int current_I_limit = 0;
    u8 Vdpm = 0;
    u8 Iin_limit = 0;
    unsigned int ma_limit_temp = 0;

    vindpm = di->cin_dpmmV;

    if(vindpm < VINDPM_MIN_3880)
        vindpm = VINDPM_MIN_3880;
    else if (vindpm > VINDPM_MAX_5080)
        vindpm = VINDPM_MAX_5080;

    if(usb_current && (POWER_SUPPLY_TYPE_USB == di->charger_source)
    && (!di->rt_discharge_flag))
    {
        current_I_limit = usb_current;
    }
    else
    {
        current_I_limit = bq24296m_config_hot_limit_current(di);
    }

    if(BQ24196 == di->bqchip_version){
        ma_limit_temp = IINLIM_1200; /* bq24196 use 1200MA threshold */
    }else{
        ma_limit_temp = IINLIM_1000; /* bq24296m use 1000MA threshold  */
    }

    if (current_I_limit <= IINLIM_100)
        Iin_limit = 0;
    else if (current_I_limit > IINLIM_100 && current_I_limit <= IINLIM_150)
        Iin_limit = 1;
    else if (current_I_limit > IINLIM_150 && current_I_limit <= IINLIM_500)
        Iin_limit = 2;
    else if (current_I_limit > IINLIM_500 && current_I_limit <= IINLIM_900)
        Iin_limit = 3;
    else if (current_I_limit > IINLIM_900 && current_I_limit <= ma_limit_temp)
        Iin_limit = 4;
    else if (current_I_limit > ma_limit_temp && current_I_limit <= IINLIM_1500)
        Iin_limit = 5;
    else if (current_I_limit > IINLIM_1500 && current_I_limit <= IINLIM_2000)
        Iin_limit = 6;
    else if (current_I_limit > IINLIM_2000 && current_I_limit <= IINLIM_3000)
        Iin_limit = 7;
    else
        Iin_limit = 4;

    Vdpm = (vindpm -VINDPM_MIN_3880)/VINDPM_STEP_80;

    di->input_source_reg00 = (di->hz_mode << BQ24296M_EN_HIZ_SHIFT)
                             | (Vdpm << BQ24296M_VINDPM_SHIFT) |Iin_limit;

    bq24296m_write_byte(di, di->input_source_reg00, INPUT_SOURCE_REG00);
    return;
}

static void bq24296m_config_power_on_reg(struct bq24296m_device_info *di)
{
    unsigned int sys_min = 0;
    u8 Sysmin = 0;

    sys_min = di->sys_minmV;


    if(sys_min < SYS_MIN_MIN_3000)
        sys_min = SYS_MIN_MIN_3000;
    else if (sys_min > SYS_MIN_MAX_3700)
        sys_min = SYS_MIN_MAX_3700;

    Sysmin = (sys_min -SYS_MIN_MIN_3000)/SYS_MIN_STEP_100;

    di->power_on_config_reg01 = WATCHDOG_TIMER_RST
                                | (di->chrg_config << BQ24296M_EN_CHARGER_SHIFT)
                                | (Sysmin << BQ24296M_SYS_MIN_SHIFT) | di->boost_lim;
    pmu_log_debug("power on reg  = %x\n",di->power_on_config_reg01);
    bq24296m_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
    return;
}

static void bq24296m_config_current_reg(struct bq24296m_device_info *di)
{
    unsigned int currentmA = 0;
    u8 Vichrg = 0;

    currentmA = di->currentmA;
    /* if currentmA is below ICHG_512, we can set the ICHG to 5*currentmA and
       set the FORCE_20PCT in REG02 to make the true current 20% of the ICHG*/

    if (currentmA < ICHG_1024)
    {
        currentmA = currentmA * 5;
        di->enable_low_chg = EN_FORCE_20PCT;
    }
    else
    {
        di->enable_low_chg = DIS_FORCE_20PCT;
    }
    if (currentmA < ICHG_512)
        currentmA = ICHG_512;
    else if(currentmA > ICHG_MAX)
        currentmA = ICHG_MAX;
    Vichrg = (currentmA - ICHG_512)/ICHG_STEP_64;

    di->charge_current_reg02 = (Vichrg << BQ24296M_ICHG_SHIFT) |(di->bcold_threshold<<BQ24296M_BCOLD_SHIFT ) | di->enable_low_chg;

    bq24296m_write_byte(di, di->charge_current_reg02, CHARGE_CURRENT_REG02);
    return;
}

static void bq24296m_config_prechg_term_current_reg(struct bq24296m_device_info *di)
{
    unsigned int precurrentmA = 0;
    unsigned int term_currentmA = 0;
    u8 Prechg = 0;
    u8 Viterm = 0;
    unsigned int ma_term_temp = 0;

    precurrentmA = di->prechrg_currentmA;
    term_currentmA = di->term_currentmA;

    if(BQ24196 == di->bqchip_version){
        ma_term_temp = ITERM_MAX_1024; /* bq24196 use max terminate 1024MA */
    }else{
        ma_term_temp = ITERM_MAX_512; /* bq24296m use max terminate 512MA  */
    }

    if(precurrentmA < IPRECHRG_MIN_128)
    {
        precurrentmA = IPRECHRG_MIN_128;
    }
    else if (precurrentmA > IPRECHRG_MAX_2048)
    {
        precurrentmA = IPRECHRG_MAX_2048;
    }

    if(term_currentmA < ITERM_MIN_128)
    {
        term_currentmA = ITERM_MIN_128;
    }
    else if (term_currentmA > ma_term_temp)
    {
        term_currentmA = ma_term_temp;
    }

    Prechg = (precurrentmA - IPRECHRG_MIN_128)/IPRECHRG_STEP_128;
    Viterm = (term_currentmA-ITERM_MIN_128)/ITERM_STEP_128;

    di->prechrg_term_current_reg03 = (Prechg <<  BQ24296M_IPRECHRG_SHIFT| Viterm);
    bq24296m_write_byte(di, di->prechrg_term_current_reg03, PRECHARGE_TERM_CURRENT_REG03);
    return;
}

static void bq24296m_config_voltage_reg(struct bq24296m_device_info *di)
{
    unsigned int voltagemV = 0;
    u8 Voreg = 0;

    voltagemV = di->voltagemV;
    if (voltagemV < VCHARGE_MIN_3504)
        voltagemV = VCHARGE_MIN_3504;
    else if (voltagemV > VCHARGE_MAX_4400)
        voltagemV = VCHARGE_MAX_4400;

    Voreg = (voltagemV - VCHARGE_MIN_3504)/VCHARGE_STEP_16;

    di->charge_voltage_reg04 = (Voreg << BQ24296M_VCHARGE_SHIFT) | BATLOWV_3000 |VRECHG_100;
    bq24296m_write_byte(di, di->charge_voltage_reg04, CHARGE_VOLTAGE_REG04);
    return;
}

static void bq24296m_config_term_timer_reg(struct bq24296m_device_info *di)
{
    if (prior_shutdown_wd)
    {
        di->watchdog_timer = WATCHDOG_DIS;
    }

    /* delete some code */
    di->term_timer_reg05 = (di->enable_iterm << BQ24296M_EN_TERM_SHIFT)
                           | di->watchdog_timer | (di->enable_timer << BQ24296M_EN_TIMER_SHIFT)
                           | di->chrg_timer;

    bq24296m_write_byte(di, di->term_timer_reg05, CHARGE_TERM_TIMER_REG05);
    return;
}

static void bq24296m_config_thernal_regulation_reg(struct bq24296m_device_info *di)
{
    u8 boostv = 0;
    unsigned int boost_vol = 0;
    u8 bhot = 0;

    boost_vol = di->boostv;
    if(boost_vol < BOOSTV_MIN_4550)
    {
        boost_vol = BOOSTV_MIN_4550;
    }
    else if(boost_vol > BOOSTV_MAX_5510)
    {
        boost_vol = BOOSTV_MAX_5510;
    }
    boostv = (boost_vol-BOOSTV_MIN_4550)/BOOSTV_SETP_64;

    if(BQ24296M == di->bqchip_version){
        di->thermal_regulation_reg06 = (boostv << BQ24296M_BOOSTV_SHIFT)
                                       |bhot |TREG_120;
    }else{
        di->thermal_regulation_reg06 = TREG_120;
    }

    bq24296m_write_byte(di, di->thermal_regulation_reg06, THERMAL_REGUALTION_REG06);
    return;
}

static void bq24296m_config_misc_operation_reg(struct bq24296m_device_info *di)
{
    if (prior_shutdown_q4)
    {
        di->enable_batfet = 1;
    }
    di->misc_operation_reg07 = (di->enable_dpdm << BQ24296M_DPDM_EN_SHIFT)
                               | TMR2X_EN |(di->enable_batfet<< BQ24296M_BATFET_EN_SHIFT)
                               | CHRG_FAULT_INT_EN |BAT_FAULT_INT_EN;

    bq24296m_write_byte(di, di->misc_operation_reg07, MISC_OPERATION_REG07);
    return;
}
static int bq24296m_get_vindpm_status(struct bq24296m_device_info *di)
{
  int ibus = 0;
  int dpm_status = 0;
  ibus = bq24296_get_ibus();

    if(di->no_ibus_detect)
    {
      pmu_log_info("not go to dpm status keep stay\n");
      dpm_status = 0;
    }
    else if (ibus < IINLIM_900)
    {
        pmu_log_info("stand charger go to dpm status keep stay\n");
        dpm_status = 1;
    }

    return dpm_status;
}

static void set_vbus_dpm_status(VBUS_DPM_STATUS_TYPE status)
{
    mutex_lock(&dpm_status_access_lock);
    g_vbus_dpm_status = status;
    mutex_unlock(&dpm_status_access_lock);
    return;
}
static VBUS_DPM_STATUS_TYPE get_vbus_dpm_status(void)
{
    VBUS_DPM_STATUS_TYPE status_ret = VBUS_DPM_STATUS_UNKNOWN;
    mutex_lock(&dpm_status_access_lock);
    status_ret = g_vbus_dpm_status;
    mutex_unlock(&dpm_status_access_lock);
    return status_ret;
}

static VBUS_DPM_STATUS_TYPE check_vbus_dpm_status(struct bq24296m_device_info *di)
{
    static int check_cnt = 0;
    VBUS_DPM_STATUS_TYPE status_ret = get_vbus_dpm_status();
    int is_in_dpm = 0;

    if(VBUS_DPM_STATUS_CHECKING == status_ret)
    {
        is_in_dpm = bq24296m_get_vindpm_status(di);
        pmu_log_info("dpm status %d\n",is_in_dpm);
        if(is_in_dpm)
        {
            status_ret = VBUS_DPM_STATUS_POOR;
            set_vbus_dpm_status(VBUS_DPM_STATUS_POOR);
            check_cnt = 0;
        }
        else
        {
            check_cnt++;
            if(VBUS_DPM_STATUS_CHECK_CNT <= check_cnt)
            {
              status_ret = VBUS_DPM_STATUS_GOOD;
              set_vbus_dpm_status(VBUS_DPM_STATUS_GOOD);
              check_cnt = 0;
            }
        }
    }
    else
    {
       check_cnt = 0;
    }
    return status_ret;
}
/*deal with poor charger when capacity is more than 90% ,if hardware does not
 use REGN for usb_int,pls delete the follow fuction*/
static void bq24296m_reset_vindpm(struct bq24296m_device_info *di)
{
    int battery_capacity = 0;
    static int cur_dpm_point_num = DPM_POINT_CNT - 1;
    int cur_bat_vol = 0, new_dpm_point_num = 0;
    int rise_num = 0;
    int fall_num = 0;
    static int first_flag = 1;
    int i=0;

    if (di->charger_source == POWER_SUPPLY_TYPE_USB)
    {
         battery_capacity = bq24296m_get_battery_capacity(di);
         if((battery_capacity <= CAPACITY_LEVEL_HIGH_THRESHOLD) && (di->cin_dpmmV != VINDPM_4520))
         {
            di->cin_dpmmV = VINDPM_4520;
         }
         else if ((battery_capacity > CAPACITY_LEVEL_HIGH_THRESHOLD) && (di->cin_dpmmV != VINDPM_4600))
         {
            di->cin_dpmmV = VINDPM_4600;
         }
    }
    else if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
    {
         cur_bat_vol = huawei_charger_get_battery_voltage_now();
         cur_bat_vol = cur_bat_vol /1000;
         if (di->cin_float_flag)
         {
           first_flag = 1;
           cur_dpm_point_num = DPM_POINT_CNT - 1;
           goto err_end1;
         }
         if(check_vbus_dpm_status(di) != VBUS_DPM_STATUS_POOR)
         {
            first_flag = 1;
            cur_dpm_point_num = DPM_POINT_CNT - 1;
            goto err_end1;
         }
         if(first_flag)
         {
            for(i=0;i<DPM_POINT_CNT;i++)
            {
               if(di->cin_dpmmV == dpm_point_tbl[i].dpm_vol)
                 cur_dpm_point_num = i;
            }
            first_flag = 0;
            pmu_log_info(" get current cin_dpm:%d cur_dpm_point_num:%d\n",di->cin_dpmmV,cur_dpm_point_num);
         }

        if(cur_bat_vol <= 3200)
        {
            cur_dpm_point_num = DPM_POINT_CNT - 1;
            di->cin_dpmmV = dpm_point_tbl[cur_dpm_point_num].dpm_vol;
            pmu_log_info(" get vol error %d \n",cur_bat_vol);
            goto err_end;
        }
        /*Dynamic adjust*/
        rise_num = (cur_dpm_point_num < DPM_POINT_CNT-1) ? (cur_dpm_point_num+1) : (DPM_POINT_CNT-1);
        fall_num = (cur_dpm_point_num > 0) ? (cur_dpm_point_num-1) : 0;
        if(cur_bat_vol > dpm_point_tbl[cur_dpm_point_num].bat_vol_max)
        {
            new_dpm_point_num = rise_num;
        }
        else if(cur_bat_vol < dpm_point_tbl[fall_num].bat_vol_roll_back)
        {
            new_dpm_point_num = fall_num;
        }
        else
        {
            new_dpm_point_num = cur_dpm_point_num;
        }
        cur_dpm_point_num = new_dpm_point_num;
        di->cin_dpmmV = dpm_point_tbl[cur_dpm_point_num].dpm_vol;

err_end1:
        battery_capacity = bq24296m_get_battery_capacity(di);
        if (battery_capacity > CAPACITY_LEVEL_HIGH_THRESHOLD)
        {
            di->cin_dpmmV = VINDPM_4600;
        }
        pmu_log_info("dpmmv = %d! cur_battery_vol = %d new_dpm_point_num:%d rise_num:%d fall_num:%d\n", di->cin_dpmmV,cur_bat_vol,new_dpm_point_num,rise_num,fall_num);
    }

err_end:
    bq24296m_config_input_source_reg(di);
    return;
}

/*0 = temperature less than 42,1 = temperature more than 42 and less 45,2 more than 45 */
static void bq24296m_calling_limit_ac_input_current(struct bq24296m_device_info *di,int flag)
{
    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
    {
        switch(flag)
        {
        case NORMAL_TEMP_CONFIG:
            if(di->calling_limit)
            {
                iin_temp = IINLIM_900;
                ichg_temp = ICHG_820;
            }
            else
            {
                if (di->cin_float_flag)
                    iin_temp = IINLIM_1000;
                else
                    iin_temp = di->max_cin_currentmA;
                ichg_temp = di->max_currentmA;
            }
            break;
        case NORMAL_HIGH_TEMP_CONFIG:
            if(di->calling_limit)
            {
                iin_temp = IINLIM_900;
                ichg_temp = ICHG_820;
            }
            else
            {
                /* delete redundant code */
                iin_temp = di->cin_limit;
                ichg_temp = di->currentmA;
            }
            break;
        case COOL_CONFIG:
            iin_temp = IINLIM_900;
            ichg_temp = di->temp_ctrl->imaxma_cool_bat;
            break;
        case HIGH_TEMP_CONFIG:
            iin_temp = IINLIM_900;
            ichg_temp = di->temp_ctrl->imaxma_warm_bat;
            break;
        case HIGH_HOT_CONFIG:
            iin_temp = IINLIM_900;
            ichg_temp = di->currentmA;
            break;
        case TEMP_CONFIG_5:
            if(di->calling_limit)
            {
                iin_temp = IINLIM_900;
                ichg_temp = di->design_capacity / 10 * di->charge_in_temp_5;
            }
            else
            {
                if (di->cin_float_flag)
                    iin_temp = IINLIM_1000;
                else
                    iin_temp = di->max_cin_currentmA;
                /* battery whose max_voltage is above 4.35V is easy to broken
                when the temperature is below 10��.
                So we need set the Current below 0.x * Capacity. */
                ichg_temp = di->design_capacity / 10 * di->charge_in_temp_5;
            }
            break;
        case TEMP_CONFIG_10:
            if(di->calling_limit)
            {
                iin_temp = IINLIM_900;
                ichg_temp = di->design_capacity / 10 * di->charge_in_temp_10;
            }
            else
            {
                if (di->cin_float_flag)
                    iin_temp = IINLIM_1000;
                else
                    iin_temp = di->max_cin_currentmA;
                ichg_temp = di->design_capacity / 10 * di->charge_in_temp_10;
            }
            break;
        case NORMAL_SECOND_STAGE:
            if(di->calling_limit)
            {
                iin_temp = IINLIM_900;
                ichg_temp = ICHG_820;
            }
            else
            {
                iin_temp = IINLIM_1200;
                ichg_temp = ICHG_1024;
            }
            break;
        case NORMAL_HIGH_TEMP_CONFIG_CP2:
            iin_temp = IINLIM_900;
            ichg_temp = ICHG_820;
            break;
        case NORMAL_HIGH_TEMP_CONFIG_CP4:
            iin_temp = IINLIM_500;
            ichg_temp = ICHG_512;
            break;
        default:
            break;
        }
    }
    else
    {
        iin_temp = IINLIM_500;
        ichg_temp = ICHG_512;
    }
    if(iin_temp > input_current_iin)
        iin_temp = input_current_iin;
    if(ichg_temp > input_current_ichg)
        ichg_temp = input_current_ichg;
    /* remove mhl code, we do not use mhl */
    return;
}

static void bq24296m_config_limit_temperature_parameter(struct bq24296m_device_info *di)
{
    di->temperature_cold = di->temp_ctrl->cold_bat_degree;
    di->temperature_cool = di->temp_ctrl->cool_bat_degree;
    di->temperature_warm = di->temp_ctrl->warm_bat_degree;
    di->temperature_hot  = di->temp_ctrl->hot_bat_degree;
    di->temperature_5    = BQ24296M_BATTERY_THRESHOLD_5;
    di->temperature_10   = BQ24296M_BATTERY_THRESHOLD_10;
    return;
}

static int bq24296m_get_bat_temp_area(struct bq24296m_device_info *di)
{
    int bat_temp = huawei_charger_get_battery_temperature();

    if (bat_temp <= di->temperature_cold) { /* cold: below cold threshold */
        return BATTERY_HEALTH_TEMPERATURE_OVERLOW;

    } else if((bat_temp > di->temperature_cold)
        && (bat_temp <  di->temperature_cool)){ /* cool: between cold and normal*/
        return BATTERY_HEALTH_TEMPERATURE_LOW;

    } else if ((bat_temp >= di->temperature_cool) /* normal */
        && (bat_temp < (di->temperature_warm - BQ24296_TEMPERATURE_OFFSET))){
       return BATTERY_HEALTH_TEMPERATURE_NORMAL;

    } else if ((bat_temp >= (di->temperature_warm - BQ24296_TEMPERATURE_OFFSET))
        && (bat_temp < di->temperature_warm)){  /* keep last state */
        return BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH;

    } else if ((bat_temp >= di->temperature_warm) /* warm: between normal and hot*/
        && (bat_temp < di->temperature_hot - BQ24296_TEMPERATURE_OFFSET)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH;

    } else if((bat_temp >= (di->temperature_hot - BQ24296_TEMPERATURE_OFFSET))
        && (bat_temp < di->temperature_hot)){ /* keep last state */
        return BATTERY_HEALTH_TEMPERATURE_HIGH_HOT;

    } else if (bat_temp >= di->temperature_hot){ /* hot: high than hot threshold*/
       return BATTERY_HEALTH_TEMPERATURE_OVERHIGH;

    } else {
       return BATTERY_HEALTH_TEMPERATURE_NORMAL;
    }
}

static void bq24296m_monitor_battery_ntc_charging(struct bq24296m_device_info *di)
{
    int battery_status = 0;
    u8 read_reg = 0;
    int batt_temp = huawei_charger_get_battery_temperature();

     /* return when battery absent or rt_discharge_flag is set */
    if((!di->battery_present) || di->rt_discharge_flag)
        return;
    bq24296m_reset_vindpm(di);

    di->battery_voltage = huawei_charger_get_battery_voltage_now();
    pmu_log_debug("battery temp is %d,battery_voltage is %d , pre status is %d\n",
             batt_temp, di->battery_voltage,di->battery_temp_status);
    if(di->not_limit_chrg_flag) /* if set this flag, ignore temp contrl*/
    {
        di->chrg_config = EN_CHARGER & di->factory_flag;
        bq24296m_config_power_on_reg(di);
        if(di->chrg_config){
            di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        } else {
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        bq24296m_read_byte(di, &read_reg, SYS_STATUS_REG08);
        if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_CHAEGE_DONE){
            return;
        }
        power_supply_changed(&di->charger);
        pmu_log_info("function return by not limit charge flag\n");
        return;
    }
    battery_status = bq24296m_get_bat_temp_area(di);

    switch (battery_status)
    {
    case BATTERY_HEALTH_TEMPERATURE_LOW: /* cool */
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->temp_ctrl->vmaxmv_cool_bat;
        if(di->battery_temp_status != battery_status)
        {
            bq24296m_calling_limit_ac_input_current(di,COOL_CONFIG);
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_NORMAL: /* normal */
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->max_voltagemV;
        if(di->battery_temp_status != battery_status)
        {
            bq24296m_calling_limit_ac_input_current(di,NORMAL_TEMP_CONFIG);
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH: /* normal high: keep last state*/
        di->chrg_config = EN_CHARGER;
        if (di->battery_temp_status != battery_status)
        {
            bq24296m_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG);
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_HIGH: /* warm */
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->temp_ctrl->vmaxmv_warm_bat;
        if (di->battery_temp_status != battery_status)
        {
            bq24296m_calling_limit_ac_input_current(di,HIGH_TEMP_CONFIG);
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_HIGH_HOT: /* high hot: keep last state*/
        if (di->battery_temp_status != battery_status)
        {
            bq24296m_calling_limit_ac_input_current(di,HIGH_HOT_CONFIG);
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_OVERLOW: /* cold: stop charging*/
    case BATTERY_HEALTH_TEMPERATURE_OVERHIGH: /* hot: stop charging*/
        di->chrg_config = DIS_CHARGER;
        di->voltagemV = di->max_voltagemV;
        break;
    default:
        break;
    }

    if(di->battery_temp_status != battery_status)
    {
        pmu_log_info("battery temp status changed\n");
    }
    di->battery_temp_status = battery_status;

    if(iin_limit_temp == 1)
    {
        if(di->cin_limit != iin_temp)
        {
            di->cin_limit = iin_temp;
        }
    }
    else if (di->cin_limit > iin_limit_temp)
    {
        di->cin_limit = iin_limit_temp;
    }
    bq24296m_config_input_source_reg(di);

    if(ichg_limit_temp == 1)
    {
        if(di->currentmA != ichg_temp)
        {
            di->currentmA = ichg_temp;
        }
    }
    else
    {
        di->currentmA = ichg_limit_temp;
    }
    bq24296m_config_current_reg(di);
    bq24296m_config_voltage_reg(di);

    di->chrg_config = (di->chrg_config & di->factory_flag) | di->not_stop_chrg_flag;
     if(di->chrg_config){
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else if(di->hz_mode == EN_HIZ){
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    bq24296m_config_power_on_reg(di);
    bq24296m_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_CHAEGE_DONE)
    {
        pmu_log_debug("function return by charge done\n");
        return;
    }
    pmu_log_debug("i_in = %d, charge_current = %d\n", di->cin_limit, di->currentmA);
    power_supply_changed(&di->charger);

    return;
}

static void bq24296m_start_usb_charger(struct bq24296m_device_info *di)
{
    unsigned int events;
    di->wakelock_enabled = 1;
    if (di->wakelock_enabled)
    {
        wake_lock(&chrg_lock);
    }
    if(di->rt_discharge_flag == RT_DISCHARGE)
    {
        /* force to discharge */
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
        /* set cin limit as 100mA when running test set discharging */
        di->cin_limit = IINLIM_100;
        di->chrg_config = DIS_CHARGER;
        /* delete some code, not used */
        events = BQ2419x_STOP_CHARGING;
    }
    else
    {
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        di->cin_limit = IINLIM_500;
        iin_temp = di->cin_limit;
        di->chrg_config = EN_CHARGER;
        /* delete some code, not used */
        events = BQ2419x_START_USB_CHARGING;
    }
    power_supply_changed(&di->charger);

    charge_event_notify(events);

    di->charger_source = POWER_SUPPLY_TYPE_USB;
    di->cin_dpmmV = VINDPM_4520;
    di->currentmA = ICHG_512;

    ichg_temp = di->currentmA;
    di->voltagemV = di->max_voltagemV;
    di->term_currentmA = ITERM_MIN_128;
    di->watchdog_timer = WATCHDOG_80;
    di->chrg_timer = CHG_TIMER_12;
    di->hz_mode = DIS_HIZ;
    /* remove redundant code */
    di->boost_lim = BOOST_LIM_1000;
    di->enable_low_chg = DIS_FORCE_20PCT;
    di->enable_iterm = DIS_TERM;
    di->enable_timer = di->enable_timer_temp;
    di->enable_batfet = EN_BATFET;
    di->calling_limit = 0;
    di->battery_temp_status = -1;
    di->enable_dpdm = DPDM_DIS;
    di->charge_full_count = 0;
    di->boostv = BOOSTV_DEFAULT;

    bq24296m_config_power_on_reg(di);
    msleep(500);
    bq24296m_config_input_source_reg(di);
    bq24296m_config_current_reg(di);
    bq24296m_config_prechg_term_current_reg(di);
    bq24296m_config_voltage_reg(di);
    bq24296m_config_term_timer_reg(di);
    bq24296m_config_thernal_regulation_reg(di);
    bq24296m_config_misc_operation_reg(di);

    gpio_set_value(di->gpio_cd, di->gpio_cd_level_reverse);

    schedule_delayed_work(&di->bq24296m_charger_work, msecs_to_jiffies(0));

    pmu_log_info(" ---->START USB CHARGING,battery current = %d mA,battery voltage = %d mV,cin_limit_current = %d mA\n"
            , di->currentmA, di->voltagemV,di->cin_limit);
    di->battery_present = huawei_charger_battery_is_exist();
    if (!di->battery_present)
    {
        pr_debug( "BATTERY NOT DETECTED!\n");
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        power_supply_changed(&di->charger);
    }

    return;
}


static void bq24296m_detect_ibus_work(struct work_struct* work)
{
    struct bq24296m_device_info* di =
        container_of(work, struct bq24296m_device_info, ibus_detect_work.work);

    static int ibus_err_count = 0;
    int ibus = 0;

    if (BQ24296M_IBUS_CONFIRM_COUNT <= ibus_err_count)
    {
        ibus_err_count = 0;
        ibus = IINLIM_1000;
    }
    else
    {
        /* get bus input current */
        ibus = bq24296_get_ibus();
    }

    pmu_log_info("detect input current, ibus = %d \n", ibus);
    if (ibus >= 0)
    {
        if ((IINLIM_1500 + BQ24296M_IBUS_MARGIN_MA) < ibus)
        {
            di->cin_limit = IINLIM_2000;
        }
        else if ((IINLIM_1000 + BQ24296M_IBUS_MARGIN_MA) < ibus)
        {
            di->cin_limit = IINLIM_1500;
        }
        else
        {
            di->cin_limit = IINLIM_1000;
        }
        pmu_log_info("ibus input limit %d \n", di->cin_limit);
        iin_temp = di->cin_limit;
        di->max_cin_currentmA = di->cin_limit;
        di->ibus_detecting = false;
        bq24296m_config_input_source_reg(di);

        if(IINLIM_1000 == di->cin_limit)
        {
            di->max_currentmA = ICHG_1088;
            di->currentmA = di->max_currentmA;
            ichg_temp = di->currentmA;
        }
        bq24296m_config_current_reg(di);
    }
    else
    {
        ibus_err_count++;
        pmu_log_err("get ibus error, will retry to detect, error count %d \n", ibus_err_count);
        schedule_delayed_work(&di->ibus_detect_work, BQ24296M_IBUS_WORK_DELAY_TIME);
    }
}

static void bq24296m_config_input_current_reg(struct bq24296m_device_info* di)
{
    int recod_hot_limit_ma = 0;

    if (di->cin_float_flag)
    {
        bq24296m_config_input_source_reg(di);
    }
    else if (di->no_ibus_detect || IINLIM_2000 > di->max_cin_cfg_currentmA)
    {
        /* config the default input current */
        bq24296m_config_input_source_reg(di);
        pmu_log_info("no ibus detect = %d , dtsi config max cin = %d \n ",
                di->no_ibus_detect, di->max_cin_cfg_currentmA);
    }
    else
    {
        /* remember the hot limit current right now */
        recod_hot_limit_ma = di->hot_limit_current;
        /* avoid the hot limit current & config max input current */
        di->cin_limit = IINLIM_2000;
        di->hot_limit_current = di->cin_limit;
        di->max_cin_currentmA = di->cin_limit;

        /* config the input current */
        bq24296m_config_input_source_reg(di);
        /* resume the hot limit current value */
        di->hot_limit_current = recod_hot_limit_ma;

        /* to do detect pdm work */
        di->ibus_detecting = true;
        schedule_delayed_work(&di->ibus_detect_work, BQ24296M_IBUS_WORK_DELAY_TIME);
    }
}

static void bq24296m_start_ac_charger(struct bq24296m_device_info *di)
{

    unsigned int events;
    di->wakelock_enabled = 1;
    if (di->wakelock_enabled)
    {
        wake_lock(&chrg_lock);
    }
    if(di->rt_discharge_flag == RT_DISCHARGE)
    {
        /* force to discharge */
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
        /* set cin limit as 100mA when running test set set discharging */
        di->cin_limit = IINLIM_100;
        di->chrg_config = DIS_CHARGER;
        /* delete some code, not used */
        events = BQ2419x_STOP_CHARGING;
    }
    else
    {
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        if (di->cin_float_flag)
            di->cin_limit = IINLIM_1000;
        else
            di->cin_limit = di->max_cin_currentmA;
        iin_temp = di->cin_limit;
        di->chrg_config = EN_CHARGER;
        /* delete some code, not used */
        events = BQ2419x_START_AC_CHARGING;
    }
    power_supply_changed(&di->charger);

    charge_event_notify(events);
    di->charger_source = POWER_SUPPLY_TYPE_MAINS;
    di->hz_mode = DIS_HIZ;
    di->cin_dpmmV = VINDPM_4520;
    di->voltagemV = di->max_voltagemV;
    di->currentmA = di->max_currentmA;
    ichg_temp = di->currentmA;
    di->term_currentmA = ITERM_MIN_128;
    di->watchdog_timer = WATCHDOG_80;
    di->chrg_timer = CHG_TIMER_12;

    di->boost_lim = BOOST_LIM_1000;/*500*/
    di->enable_low_chg = DIS_FORCE_20PCT;
    di->enable_iterm = DIS_TERM;
    di->enable_timer = EN_TIMER;
    di->enable_batfet = EN_BATFET;
    di->enable_dpdm = DPDM_DIS;
    di->boostv = BOOSTV_DEFAULT;
    di->calling_limit = 0;
    di->battery_temp_status = -1;
    di->charge_full_count = 0;

    bq24296m_config_power_on_reg(di);
    msleep(500);
    bq24296m_config_input_current_reg(di);
    if(IINLIM_1000 == di->cin_limit)
    {
        di->max_currentmA = ICHG_1088;
        di->currentmA = di->max_currentmA;
        ichg_temp = di->currentmA;
    }
    bq24296m_config_current_reg(di);
    bq24296m_config_prechg_term_current_reg(di);
    bq24296m_config_voltage_reg(di);
    bq24296m_config_term_timer_reg(di);
    bq24296m_config_thernal_regulation_reg(di);
    bq24296m_config_misc_operation_reg(di);

    gpio_set_value(di->gpio_cd, di->gpio_cd_level_reverse);

    schedule_delayed_work(&di->bq24296m_charger_work, msecs_to_jiffies(0));

    pmu_log_info("---->START AC CHARGING,battery current = %d mA,battery voltage = %d mV,cin_limit_current = %d mA\n"
            , di->currentmA, di->voltagemV,di->cin_limit);

    di->battery_present = huawei_charger_battery_is_exist();
    if (!di->battery_present)
    {
        pr_debug( "BATTERY NOT DETECTED!\n");
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        power_supply_changed(&di->charger);
    }

    return;
}

static void bq24296m_start_usb_otg(struct bq24296m_device_info *di)
{
    pmu_log_info("%s,---->USB_EVENT_OTG_ID<----\n", __func__);

    if(di->charger_source != POWER_SUPPLY_TYPE_BATTERY)
    {
        gpio_set_value(di->gpio_cd, !di->gpio_cd_level_reverse);
        return;
    }

    di->wakelock_enabled = 1;
    if (di->wakelock_enabled)
    {
        wake_lock(&chrg_lock);
    }

    di->hz_mode = DIS_HIZ;
    di->chrg_config = EN_CHARGER_OTG;
    di->boost_lim = BOOST_LIM_1000;

    bq24296m_config_power_on_reg(di);
    bq24296m_config_input_source_reg(di);

    gpio_set_value(di->gpio_cd, di->gpio_cd_level_reverse);
    if(irq_int_active == 0)
    {
        irq_int_active = 1;
        enable_irq(di->irq_int);
    }
    schedule_delayed_work(&di->bq24296m_usb_otg_work, msecs_to_jiffies(0));

    return;
}

static void bq24296m_stop_charger(struct bq24296m_device_info *di)
{
    if (!wake_lock_active(&chrg_lock))
    {
        wake_lock(&chrg_lock);
    }

    /*set gpio_xxx high level for CE pin to disable bq24296m IC */
    gpio_set_value(di->gpio_cd, !di->gpio_cd_level_reverse);
    if(irq_int_active == 1)
    {
        disable_irq(di->irq_int);
        irq_int_active = 0;
    }

    charge_event_notify(BQ2419x_STOP_CHARGING);
    cancel_delayed_work_sync(&di->bq24296m_charger_work);
    di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    power_supply_changed(&di->charger);

    pr_info("%s,---->STOP CHARGING\n", __func__);
    di->charger_source = POWER_SUPPLY_TYPE_BATTERY;

    di->enable_batfet = EN_BATFET;
    di->calling_limit = 0;
    /* delete some code, not used */
    di->hz_mode = DIS_HIZ;
    di->battery_temp_status = -1;
    di->chrg_config = DIS_CHARGER;
    di->enable_dpdm = DPDM_DIS;
    di->charge_full_count = 0;
    di->max_currentmA = max_currentmA_bak;

    bq24296m_config_power_on_reg(di);
    bq24296m_config_input_source_reg(di);
    bq24296m_config_misc_operation_reg(di);

    cancel_delayed_work_sync(&di->bq24296m_usb_otg_work);
    cancel_delayed_work_sync(&di->otg_int_work);

    /*set gpio_xxx high level for CE pin to disable bq24296m IC */
    gpio_set_value(di->gpio_cd, !di->gpio_cd_level_reverse);
    msleep(1000);

    di->wakelock_enabled = 1;
    if (di->wakelock_enabled)
    {
        wake_lock_timeout(&stop_chrg_lock, HZ);
        wake_unlock(&chrg_lock);
    }

    wakeup_timer_seconds = 0;
    return;
}

static int bq24296m_otg_regulator_enable(struct regulator_dev *rdev)
{
    struct bq24296m_device_info *di = rdev_get_drvdata(rdev);
    otg_enabled = true;
    di->event = USB_EVENT_OTG_ID;
    schedule_work(&di->usb_work);
    return 0;
}

static int bq24296m_otg_regulator_disable(struct regulator_dev *rdev)
{
    struct bq24296m_device_info *di = rdev_get_drvdata(rdev);
    otg_enabled = false;
    di->event = CHARGER_REMOVED;
    schedule_work(&di->usb_work);
    return 0;
}

static int bq24296m_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    u8 read_reg = 0;
    int rc = 0;
    struct bq24296m_device_info *di = rdev_get_drvdata(rdev);

    rc = bq24296m_read_byte(di, &read_reg, POWER_ON_CONFIG_REG01);
    if (rc)
    {
        pr_err("failed to read POWER_ON_CONFIG_REG01\n");
        return 0;
    }
    pr_debug("otg_en 0x%x\n", read_reg);
    return  (read_reg & BQ_OTG_EN) ? 1 : 0;
}

struct regulator_ops bq24296m_otg_reg_ops =
{
    .enable = bq24296m_otg_regulator_enable,
    .disable = bq24296m_otg_regulator_disable,
    .is_enabled = bq24296m_otg_regulator_is_enable,
};

static int bq24296m_regulator_init(struct bq24296m_device_info *di)
{
    int rc = 0;
    struct regulator_init_data *init_data;
    struct regulator_config cfg = {};

    init_data = of_get_regulator_init_data(di->dev, di->dev->of_node);
    if (!init_data)
    {
        dev_err(di->dev, "Unable to allocate memory\n");
        return -ENOMEM;
    }

    if (init_data->constraints.name)
    {
        di->otg_vreg.rdesc.owner = THIS_MODULE;
        di->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
        di->otg_vreg.rdesc.ops = &bq24296m_otg_reg_ops;
        di->otg_vreg.rdesc.name = init_data->constraints.name;

        cfg.dev = di->dev;
        cfg.init_data = init_data;
        cfg.driver_data = di;
        cfg.of_node = di->dev->of_node;

        init_data->constraints.valid_ops_mask
        |= REGULATOR_CHANGE_STATUS;

        di->otg_vreg.rdev = regulator_register(
                                &di->otg_vreg.rdesc, &cfg);
        if (IS_ERR(di->otg_vreg.rdev))
        {
            rc = PTR_ERR(di->otg_vreg.rdev);
            di->otg_vreg.rdev = NULL;
            if (rc != -EPROBE_DEFER)
                dev_err(di->dev,
                        "OTG reg failed, rc=%d\n", rc);
        }
    }

    return rc;
}

static void resume_charging_checking(struct bq24296m_device_info *di)
{
    int rc;
    u8 read_reg = 0;

    di->enable_iterm = DIS_TERM;
    bq24296m_config_term_timer_reg(di);

    /* if set discharging when running test, return*/
    if(di->rt_discharge_flag == RT_DISCHARGE)
        return;

    rc = bq24296m_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if (rc)
    {
        pmu_log_err("failed to read SYS_STSTUS_REG08");
        return;
    }

    /* if soc_resume_charging is true, but reg08 is still charge done*/
    /* disable charging and enable charging to recharge*/
    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_CHAEGE_DONE)
    {
        pmu_log_info("read_reg = 0x%x, resume charging is needed\n", read_reg);
        di->hz_mode = EN_HIZ;
        bq24296m_config_input_source_reg(di);
        msleep(100);
        di->hz_mode = DIS_HIZ;
        bq24296m_config_input_source_reg(di);
    }
}

static void bq24296m_check_battery_full(struct bq24296m_device_info *di)
{
    if(di->battery_present)
    {

        if(di->battery_full)
        {
            di->enable_iterm = EN_TERM;
            di->charge_full_count++;
            if(di->charge_full_count >= 20)
            {
                di->charge_full_count = 20;
            }
        }
        else
        {
            resume_charging_checking(di);
        }
    }
    else
    {
        di->enable_iterm = EN_TERM;
    }

    bq24296m_config_term_timer_reg(di);

    return;
}

static void bq24296m_config_status_reg(struct bq24296m_device_info *di)
{
    di->power_on_config_reg01 = di->power_on_config_reg01 |WATCHDOG_TIMER_RST;
    bq24296m_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
    return;
}

static void bq24296m_charger_update_status(struct bq24296m_device_info *di)
{
    u8 read_reg[11] = {0};
    unsigned int events = 0;

    di->timer_fault = 0;
    bq24296m_read_block(di, &read_reg[0], 0, 11);
    bq24296m_read_byte(di, &read_reg[9], CHARGER_FAULT_REG09);
    msleep(700);
    bq24296m_read_byte(di, &read_reg[9], CHARGER_FAULT_REG09);

    if ((read_reg[8] & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_CHAEGE_DONE)
    {
        pmu_log_debug("CHARGE DONE\n");
        //while battery is not present, the register show charging done.
        //avoid the issue, add condition here
        if(!di->battery_present)
        {
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            events = BQ2419x_STOP_CHARGING;
        }
        /* do not report battery full here */
        power_supply_changed(&di->charger);
    }

    if ((read_reg[8] & BQ2429M_PG_STAT) == BQ24296M_NOT_PG_STAT)
    {
        di->cfg_params = 1;
        pmu_log_info("not power good\n");
    }
    if ((read_reg[9] & BQ24296M_POWER_SUPPLY_OVP_MASK) == BQ24296M_POWER_SUPPLY_OVP)
    {
        pmu_log_err("POWER_SUPPLY_OVERVOLTAGE = %x\n", read_reg[9]);
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        power_supply_changed(&di->charger);
        events = BQ2419x_STOP_CHARGING;
    }

    if ((read_reg[9] & BQ24296M_WATCHDOG_FAULT) == BQ24296M_WATCHDOG_FAULT)
    {
        di->timer_fault = 1;
    }

    if (read_reg[9] & 0xFF)
    {
        di->cfg_params = 1;
        pmu_log_err("CHARGER STATUS %x\n", read_reg[9]);
    }
    /*when battery temp is over 42 degree,ignore ovp detect */
    if ((bq24296m_get_bat_temp_area(di) == BATTERY_HEALTH_TEMPERATURE_NORMAL)
        ||(bq24296m_get_bat_temp_area(di) == BATTERY_HEALTH_TEMPERATURE_LOW)
        ||(bq24296m_get_bat_temp_area(di) == BATTERY_HEALTH_TEMPERATURE_OVERLOW))
    {
        if ((read_reg[9] & BQ24296M_BAT_FAULT_OVP) == BQ24296M_BAT_FAULT_OVP)
        {
            pmu_log_err("BATTERY OVP = %x\n", read_reg[9]);
        }
    }

    if ((di->timer_fault == 1) || (di->cfg_params == 1))
    {
        bq24296m_write_byte(di, di->input_source_reg00, INPUT_SOURCE_REG00);
        bq24296m_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
        bq24296m_write_byte(di, di->charge_current_reg02, CHARGE_CURRENT_REG02);
        bq24296m_write_byte(di, di->prechrg_term_current_reg03, PRECHARGE_TERM_CURRENT_REG03);
        bq24296m_write_byte(di, di->charge_voltage_reg04, CHARGE_VOLTAGE_REG04);
        bq24296m_write_byte(di, di->term_timer_reg05, CHARGE_TERM_TIMER_REG05);
        bq24296m_write_byte(di, di->thermal_regulation_reg06, THERMAL_REGUALTION_REG06);
        di->cfg_params = 0;
    }

    /* reset 30 second timer */
    bq24296m_config_status_reg(di);
    if(!events)
    {
        if(di->chrg_config && di->battery_present)
        {
            if(di->event == CHARGER_TYPE_USB)
            {
                events = BQ2419x_START_USB_CHARGING;
            }
            if(di->event == CHARGER_TYPE_STANDARD)
            {
                events = BQ2419x_START_AC_CHARGING;
            }
        }
    }
    charge_event_notify(events);
}

int get_bq_charge_status()
{
    int rc;
    u8 read_reg = 0;

    if(NULL == bq_device){
        pmu_log_info("bq24296m device is not init, do nothing!\n");
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

    rc = bq24296m_read_byte(bq_device, &read_reg, SYS_STATUS_REG08);
    if (rc)
    {
        pmu_log_err("failed to read SYS_STSTUS_REG08");
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }

    if(bq_device->battery_full && bq24296m_charger_present(bq_device))
        return POWER_SUPPLY_STATUS_FULL;

    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_PRE_CHARGING)
        return POWER_SUPPLY_STATUS_CHARGING;
    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_FAST_CHARGING)
        return POWER_SUPPLY_STATUS_CHARGING;

    return POWER_SUPPLY_STATUS_DISCHARGING;
}


static void bq24296m_charger_work(struct work_struct *work)
{

    struct bq24296m_device_info *di = container_of(work,
                                      struct bq24296m_device_info, bq24296m_charger_work.work);
    di->battery_present = huawei_charger_battery_is_exist();

    /* remove the code of checking battery full here*/

    /* bq24296m_update_charge_current(di); */
    bq24296m_monitor_battery_ntc_charging(di);

    bq24296m_check_battery_full(di);

    bq24296m_charger_update_status(di);

    schedule_delayed_work(&di->bq24296m_charger_work,
                          msecs_to_jiffies(BQ24296M_WATCHDOG_TIMEOUT));
}

static void bq24296m_otg_int_work(struct work_struct *work)
{
    struct bq24296m_device_info *di = container_of(work,struct bq24296m_device_info, otg_int_work.work);
    u8 read_reg = 0;

    if(di->event != USB_EVENT_OTG_ID)
        return;
    pr_info("bq24296m_otg_int_work\n");

    msleep(100);
    bq24296m_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
    pr_info("reg09=0x%x first\n",read_reg);
    msleep(100);
    bq24296m_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
    pr_info("reg09=0x%x then\n", read_reg);
    if(read_reg & BQ24296M_OTG_FAULT)
    {
        di->chrg_config = DIS_CHARGER;
        bq24296m_config_power_on_reg(di);
        dev_err(di->dev, "VBUS overloaded in OTG read_reg[9]= %x\n", read_reg);
        return;
    }

    if(irq_int_active == 0)
    {
        irq_int_active = 1;
        enable_irq(di->irq_int);
    }
}
static irqreturn_t bq24296m_irq_int_interrupt(int irq, void *_di)
{
    struct bq24296m_device_info *di = _di;

    pr_info("OTG interrupt\n");
    if(irq_int_active == 1)
    {
        disable_irq_nosync(di->irq_int);
        irq_int_active = 0;
    }
    schedule_delayed_work(&di->otg_int_work,0);
    return IRQ_HANDLED;
}
static void bq24296m_usb_otg_work(struct work_struct *work)
{
    struct bq24296m_device_info *di = container_of(work,
                                      struct bq24296m_device_info, bq24296m_usb_otg_work.work);

    /* reset 30 second timer */
    bq24296m_config_power_on_reg(di);
    pr_debug("bq24296m_usb_otg_work\n");

    schedule_delayed_work(&di->bq24296m_usb_otg_work,
                          msecs_to_jiffies(BQ24296M_WATCHDOG_TIMEOUT));
}
static void judge_dpm_status(struct bq24296m_device_info *di)
{
    switch(di->event)
    {
        case CHARGER_TYPE_BC_USB:
        case CHARGER_TYPE_STANDARD:
            set_vbus_dpm_status(VBUS_DPM_STATUS_CHECKING);
            break;
        default:
            set_vbus_dpm_status(VBUS_DPM_STATUS_UNKNOWN);
            check_vbus_dpm_status(di);
            break;
    }
}
static void bq24296m_usb_charger_work(struct work_struct *work)
{
    struct bq24296m_device_info	*di =
        container_of(work, struct bq24296m_device_info, usb_work);

    switch (di->event)
    {
    case CHARGER_TYPE_USB:
        pr_info("case = CHARGER_TYPE_USB-> \n");
        bq24296m_start_usb_charger(di);
        break;
    case CHARGER_TYPE_NON_STANDARD:
        pr_info("case = CHARGER_TYPE_NON_STANDARD -> \n");
        bq24296m_start_usb_charger(di);
        break;
    case CHARGER_TYPE_BC_USB:
        pr_info("case = CHARGER_TYPE_BC_USB -> \n");
        bq24296m_start_ac_charger(di);
        break;
    case CHARGER_TYPE_STANDARD:
        pr_info("case = CHARGER_TYPE_STANDARD\n");
        bq24296m_start_ac_charger(di);
        break;
    case CHARGER_REMOVED:
        pr_info("case = USB_EVENT_NONE\n");
        bq24296m_stop_charger(di);
        break;
    case USB_EVENT_OTG_ID:
        pr_info("case = USB_EVENT_OTG_ID\n");
        bq24296m_start_usb_otg(di);
        break;
    default:
        break;
    }
    judge_dpm_status(di);

}

static void handle_charging_by_current_limit(struct bq24296m_device_info *di)
{
    /* usb has been suspended or reset, disable charging*/
    if (SUSPEND_CURRENT >= di->charge_current_limit)
    {
        di->chrg_config = DIS_CHARGER;
        bq24296m_config_power_on_reg(di);
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    }
    /* check usb current_max current, whether it is 500mA */
    else if (SDP_CURRENT_LIMIT >= di->charge_current_limit)
    {
        di->event = CHARGER_TYPE_USB;
        schedule_work(&di->usb_work);
    }
    /* check floated current, whether it is 1000mA */
    else if (IINLIM_1000 >= di->charge_current_limit)
    {
        di->cin_float_flag = true;
        di->event = CHARGER_TYPE_STANDARD;
        schedule_work(&di->usb_work);
    }
    /* check usb current_max current, whether it is 1500mA */
    else
    {
        di->event = CHARGER_TYPE_STANDARD;
        schedule_work(&di->usb_work);
    }

}

static void stop_charging(struct bq24296m_device_info *di)
{
    di->event = CHARGER_REMOVED;
    di->cin_float_flag = false;
    schedule_work(&di->usb_work);
}

/*
* set 1 --- discharging ; 0 --- charging
*/
static void bq24296_set_enable_rt_mode_for_factory(struct bq24296m_device_info *di, int val)
{
    u8 read_reg = 0;
    if((di == NULL) ||(val < 0) || (val > 1))
        return ;

    /* set cin limit as 100mA when running test set discharging */
    if(val){
        di->cin_limit = IINLIM_100;
    }else{
        di->cin_limit = iin_temp;
    }
    di->rt_discharge_flag = val;
    di->chrg_config = !val;
    di->factory_flag = !val;
    bq24296m_config_input_source_reg(di);
    bq24296m_config_power_on_reg(di);

    if(val)
    {
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    }
    else
    {
        if(bq24296m_charger_present(di))
        {
            di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        }
        else
        {
            di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
        }
    }
    pmu_log_info("set running test discharge flag %d\n", val);

    bq24296m_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_CHAEGE_DONE)
    {
        return;
    }

    power_supply_changed(&di->charger);
    return ;
}

/* set 1 --- enable_charger; 0 --- disable charger */
static void bq24296_set_enable_charger_for_factory_diag(struct bq24296m_device_info *di, int val)
{
    u8 read_reg = 0;
    if((di == NULL) ||(val < 0) || (val > 1))
        return ;

    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    di->chrg_config = val;
    di->factory_flag = val;
    if(di->factory_flag)
    {
        if(bq24296m_charger_present(di))
        {
            di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        }
        else
        {
            di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
        }
    }
    else
    {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    bq24296m_config_power_on_reg(di);
    pmu_log_info("set enable charger factory diag %d\n", val);

    bq24296m_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if ((read_reg & BQ24296M_CHGR_STAT_CHAEGE_DONE) == BQ24296M_CHGR_STAT_CHAEGE_DONE)
    {
        return;
    }

    power_supply_changed(&di->charger);
}

static void bq24296m_set_hot_limit_current(struct bq24296m_device_info *di, int ma)
{
    if (di->hot_limit_current != ma)
    {
        mutex_lock(&di->hot_limit_lock);
        di->hot_limit_current = ma;
        mutex_unlock(&di->hot_limit_lock);
        if (!di->ibus_detecting)
        {
            bq24296m_config_input_source_reg(di);
        }
        else
        {
            pmu_log_info("The charge is detecting ibus, hot current defers to limit \n");
        }
    }

}

/*
* set 1 --- hz_mode ; 0 --- not hz_mode
*/
static ssize_t bq24296m_set_enable_hz_mode(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->hz_mode= val;
    bq24296m_config_input_source_reg(di);

    return status;
}

static ssize_t bq24296m_show_enable_hz_mode(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned long val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->hz_mode;
    return snprintf(buf, PAGE_SIZE, "%lu\n", val);
}

static ssize_t bq24296m_set_dppm_voltage(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < VINDPM_MIN_3880)
            || (val > VINDPM_MAX_5080))
        return -EINVAL;

    di->cin_dpmmV = (unsigned int)val;
    bq24296m_config_input_source_reg(di);

    return status;
}

static ssize_t bq24296m_show_dppm_voltage(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->cin_dpmmV;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_cin_limit(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if(strict_strtol(buf, 10, &val) < 0)
    {
        return -EINVAL;
    }

    if(((val > 1) && (val < IINLIM_100)) || (val > IINLIM_3000))
    {
        return -EINVAL;
    }

    if(val == 1)
    {
        iin_limit_temp = (unsigned int)val;
        return status;
    }

    iin_limit_temp = val;
    if(di->cin_limit > iin_limit_temp)
    {
        di->cin_limit = iin_limit_temp;
        bq24296m_config_input_source_reg(di);
    }

    pr_info("set cin_limit = %d\n", iin_limit_temp);
    return status;
}

static ssize_t bq24296m_show_cin_limit(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->cin_limit;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_regulation_voltage(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < VCHARGE_MIN_3504)
            || (val > VCHARGE_MAX_4400))
        return -EINVAL;

    if(val < 4200)
        regulation_voltage_changed = 1;
    else
        regulation_voltage_changed = 0;
    di->voltagemV = (unsigned int)val;
    bq24296m_config_voltage_reg(di);

    return status;
}

static ssize_t bq24296m_show_regulation_voltage(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->voltagemV;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_charge_current(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if(strict_strtol(buf, 10, &val) < 0)
    {
        return -EINVAL;
    }

    if(((val > 1) && (val < ICHG_512)) || (val > ICHG_3000))
    {
        return -EINVAL;
    }

    if(val == 1)
    {
        ichg_limit_temp = val;
        return status;
    }

    ichg_limit_temp = (unsigned int)val;
    if(di->currentmA > ichg_limit_temp)
    {
        di->currentmA = ichg_limit_temp;
        bq24296m_config_current_reg(di);
    }

    pr_info("set currentmA = %d\n", ichg_limit_temp);
    return status;
}

static ssize_t bq24296m_show_charge_current(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->currentmA;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_precharge_current(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > IPRECHRG_MAX_2048))
        return -EINVAL;

    di->prechrg_currentmA = (unsigned int)val;
    bq24296m_config_prechg_term_current_reg(di);

    return status;
}

static ssize_t bq24296m_show_precharge_current(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->prechrg_currentmA;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_termination_current(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > ITERM_MAX_1024))
        return -EINVAL;

    di->term_currentmA = (unsigned int)val;
    bq24296m_config_prechg_term_current_reg(di);

    return status;
}

static ssize_t bq24296m_show_termination_current(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->term_currentmA;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_enable_itermination(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->enable_iterm = val;
    bq24296m_config_term_timer_reg(di);

    return status;
}

static ssize_t bq24296m_show_enable_itermination(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->enable_iterm;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

/* set 1 --- enable_charger; 0 --- disable charger */
static ssize_t bq24296m_set_enable_charger(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    pr_info("set factory_flag = %ld not_stop_chrg_flag = 0 not_limit_chrg_flag = 0\n", val);

    di->chrg_config = (unsigned int)val;
    di->factory_flag = (unsigned int)val;
    di->not_stop_chrg_flag = 0;
    di->not_limit_chrg_flag = 0;
    if(di->factory_flag)
    {
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    }
    else
    {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    bq24296m_config_power_on_reg(di);


    return status;
}

static ssize_t bq24296m_show_enable_charger(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->chrg_config ;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

/* set 1 --- enable_batfet; 0 --- disable batfet */
static ssize_t bq24296m_set_enable_batfet(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->enable_batfet = val ^ 0x01;
    bq24296m_config_misc_operation_reg(di);

    return status;
}

static ssize_t bq24296m_show_enable_batfet(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->enable_batfet ^ 0x01;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

/*
* set 1 --- enable bq24296m IC; 0 --- disable bq24296m IC
*
*/
static ssize_t bq24296m_set_enable_cd(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->cd_active = val ? di->gpio_cd_level_reverse : (!di->gpio_cd_level_reverse);
    gpio_set_value(di->gpio_cd, di->cd_active);
    return status;
}

static ssize_t bq24296m_show_enable_cd(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->cd_active ^ 0x1;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_charging(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if (strncmp(buf, "startac", 7) == 0)
    {
        if (di->charger_source == POWER_SUPPLY_TYPE_USB)
            bq24296m_stop_charger(di);
        bq24296m_start_ac_charger(di);
    }
    else if (strncmp(buf, "startusb", 8) == 0)
    {
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
            bq24296m_stop_charger(di);
        bq24296m_start_usb_charger(di);
    }
    else if (strncmp(buf, "stop" , 4) == 0)
    {
        bq24296m_stop_charger(di);
    }
    else if (strncmp(buf, "otg" , 3) == 0)
    {
        if (di->charger_source == POWER_SUPPLY_TYPE_BATTERY)
        {
            bq24296m_stop_charger(di);
            bq24296m_start_usb_otg(di);
        }
        else
        {
            return -EINVAL;
        }
    }
    else
        return -EINVAL;

    return status;
}

static ssize_t bq24296m_set_wakelock_enable(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    if ((val) && (di->charger_source != POWER_SUPPLY_TYPE_BATTERY))
        wake_lock(&chrg_lock);
    else
        wake_unlock(&chrg_lock);

    di->wakelock_enabled = (unsigned int)val;
    return status;
}

static ssize_t bq24296m_show_wakelock_enable(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->wakelock_enabled;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_show_chargelog_head(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    snprintf(buf, PAGE_SIZE, "Reg[00]    Reg[01]    Reg[02]    Reg[03]    Reg[04]    Reg[05]    Reg[06]    Reg[07]    Reg[08]    Reg[09]    Reg[0A]    \n");
    return strlen(buf);
}
static ssize_t bq24296m_show_chargelog(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    int i = 0;
    u8 read_reg[11] = {0};
    u8 buf_temp[26] = {0};
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    bq24296m_read_block(di, &read_reg[0], 0, 11);
    bq24296m_read_byte(di, &read_reg[9], CHARGER_FAULT_REG09);
    for(i=0; i<11; i++)
    {
        snprintf(buf_temp,26,"0x%-8.2x",read_reg[i]);
        strncat(buf, buf_temp, strlen(buf_temp));
		//use comma to generate .csv document,excel can open it directly
		strncat(buf,",",strlen(","));
    }
    strncat(buf, "\n", 1);
    return strlen(buf);
}

static ssize_t bq24296m_set_calling_limit(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{

    long val = 0;
    int status = count;

    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;
    di->calling_limit = (unsigned int) val;
    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
    {
        if(di->calling_limit)
        {
            di->cin_limit = IINLIM_900;
            di->currentmA = ICHG_820;
            bq24296m_config_input_source_reg(di);
            bq24296m_config_current_reg(di);
            pr_info("calling_limit_current = %d\n", di->cin_limit);
        }
        else
        {
            di->battery_temp_status = -1;
            if (di->cin_float_flag)
                di->cin_limit = IINLIM_1000;
            else
                di->cin_limit = di->max_cin_currentmA;
            di->currentmA = di->max_currentmA ;
            bq24296m_config_input_source_reg(di);
            bq24296m_config_current_reg(di);
        }
    }
    else
    {
        di->calling_limit = 0;
    }

    return status;
}

static ssize_t bq24296m_show_calling_limit(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->calling_limit;

    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_boostv(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > BOOSTV_MAX_5510))
        return -EINVAL;

    di->boostv = (unsigned int)val;
    bq24296m_config_thernal_regulation_reg(di);

    return status;
}

static ssize_t bq24296m_show_boostv(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->boostv;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_bhot(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 3))
        return -EINVAL;

    di->bhot= (unsigned int)val;
    bq24296m_config_thernal_regulation_reg(di);

    return status;
}

static ssize_t bq24296m_show_bhot(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->bhot;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_not_limit_chrg_flag(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    pr_info("set not_limit_chrg_flag = %ld\n", val);
    di->not_limit_chrg_flag = val;
    if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
        di->currentmA = di->max_currentmA;
        if (di->cin_float_flag)
            di->cin_limit = IINLIM_1000;
        else
            di->cin_limit = di->max_cin_currentmA;
        bq24296m_config_current_reg(di);
        bq24296m_config_power_on_reg(di);
        bq24296m_config_input_source_reg(di);
    }

    bq24296m_monitor_battery_ntc_charging(di);
    return status;
}

static ssize_t bq24296m_show_not_limit_chrg_flag(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->not_limit_chrg_flag;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_not_stop_chrg_flag(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    pr_info("set not_stop_chrg_flag = %ld\n", val);
    di->not_stop_chrg_flag = val;

    return status;
}

static ssize_t bq24296m_show_not_stop_chrg_flag(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->not_stop_chrg_flag;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_enable_timer(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    pr_info("set enable_timer = %ld\n", val);

    di->enable_timer_temp = val;
    if (CHARGER_TYPE_USB == di->event) {
        di->enable_timer = di->enable_timer_temp;
        bq24296m_config_term_timer_reg(di);
    }
    bq24296m_config_term_timer_reg(di);

    return status;
}

static ssize_t bq24296m_show_enable_timer(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    val = di->enable_timer_temp;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}
static ssize_t bq24296m_set_temperature_parameter(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 40)
            || (val > 70))
        return -EINVAL;

    di->temperature_warm = (int)val;
    di->temperature_hot  = (int)val + 5;

    bq24296m_monitor_battery_ntc_charging(di);

    return status;
}

static ssize_t bq24296m_show_temperature_parameter(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int read_reg[6] = {0};
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    read_reg[0] = di->temperature_cold;
    read_reg[1] = di->temperature_cool;
    read_reg[2] = di->temperature_5;
    read_reg[3] = di->temperature_10;
    read_reg[4] = di->temperature_warm;
    read_reg[5] = di->temperature_hot;

    snprintf(buf, PAGE_SIZE, "%-9d  %-9d  %-9d  %-9d  %-9d  %-9d",
             read_reg[0],read_reg[1],read_reg[2],read_reg[3],read_reg[4],read_reg[5]);

    return strlen(buf);
}

static ssize_t bq24296m_set_current(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < ICHG_512)
            || (val > ICHG_3000))
        return -EINVAL;

    di->max_currentmA = (int)val;
    max_currentmA_bak = di->max_currentmA;
    di->max_cin_currentmA = (int)val;

    return status;
}

static ssize_t bq24296m_set_temp_current_iin(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;

    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0)
            || (val > ICHG_3000))
        return -EINVAL;

    if(val == 0)
        input_current_iin = di->max_cin_currentmA;
    else if(val <= IINLIM_100)
        input_current_iin = IINLIM_100;
    else
        input_current_iin = (unsigned int)val;

    if(di->cin_limit > input_current_iin)
    {
        di->cin_limit = input_current_iin;
        bq24296m_config_input_source_reg(di);
    }

    pr_info("set temp_current_iin = %u\n", input_current_iin);

    return status;
}

static ssize_t bq24296m_show_temp_current_iin(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    val = input_current_iin;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_set_temp_current_ichg(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > ICHG_3000))
        return -EINVAL;

    if(val == 0)
        input_current_ichg = di->max_currentmA;
    else if(val <= ICHG_100)
        input_current_ichg = ICHG_100;
    else
        input_current_ichg = (unsigned int)val;

    if(di->currentmA > input_current_ichg)
    {
        di->currentmA = input_current_ichg;
        bq24296m_config_current_reg(di);
    }

    pr_info("set temp_current_ichg = %u\n", input_current_ichg);

    return status;
}
static ssize_t bq24296m_show_temp_current_ichg(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    val = input_current_ichg;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t bq24296m_show_shutdown_watchdog(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    val = prior_shutdown_wd;
    return sprintf(buf, "%u\n", val);
}
static ssize_t bq24296m_set_shutdown_watchdog(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;
    prior_shutdown_wd = (unsigned int)val;
    pr_info("set shutdown_wd = %u\n", prior_shutdown_wd);
    bq24296m_config_term_timer_reg(di);

    return status;
}

static ssize_t bq24296m_show_shutdown_q4(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned int val;
    val = prior_shutdown_q4;
    return sprintf(buf, "%u\n", val);
}
static ssize_t bq24296m_set_shutdown_q4(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;
    prior_shutdown_q4 = (unsigned int)val;
    pr_info("set shutdown_q4 = %u\n", prior_shutdown_q4);
    bq24296m_config_misc_operation_reg(di);
    return status;
}

static ssize_t bq24296m_set_usb_current(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
    long val = 0;
    int status = count;
    struct bq24296m_device_info *di = dev_get_drvdata(dev);

    if(strict_strtol(buf, 10, &val) < 0)
    {
        return -EINVAL;
    }

    if(((val > 1) && (val < IINLIM_100)) || (val > IINLIM_3000))
    {
        return -EINVAL;
    }

    usb_current = val;

    bq24296m_config_input_source_reg(di);
    /* delete some code */

    pmu_log_info("set cin_limit = %d\n", usb_current);
    return status;
}

static ssize_t bq24296m_show_usb_current(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    unsigned int val;

    val = usb_current;
    return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static DEVICE_ATTR(enable_hz_mode, S_IWUSR | S_IRUGO,
                   bq24296m_show_enable_hz_mode,
                   bq24296m_set_enable_hz_mode);
static DEVICE_ATTR(dppm_voltage, S_IWUSR | S_IRUGO,
                   bq24296m_show_dppm_voltage,
                   bq24296m_set_dppm_voltage);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO,
                   bq24296m_show_cin_limit,
                   bq24296m_set_cin_limit);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
                   bq24296m_show_regulation_voltage,
                   bq24296m_set_regulation_voltage);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
                   bq24296m_show_charge_current,
                   bq24296m_set_charge_current);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
                   bq24296m_show_termination_current,
                   bq24296m_set_termination_current);
static DEVICE_ATTR(precharge_current, S_IWUSR | S_IRUGO,
                   bq24296m_show_precharge_current,
                   bq24296m_set_precharge_current);
static DEVICE_ATTR(enable_itermination, S_IWUSR | S_IRUGO,
                   bq24296m_show_enable_itermination,
                   bq24296m_set_enable_itermination);
static DEVICE_ATTR(enable_charger, S_IWUSR | S_IRUGO,
                   bq24296m_show_enable_charger,
                   bq24296m_set_enable_charger);
static DEVICE_ATTR(enable_batfet, S_IWUSR | S_IRUGO,
                   bq24296m_show_enable_batfet,
                   bq24296m_set_enable_batfet);
static DEVICE_ATTR(enable_cd, S_IWUSR | S_IRUGO,
                   bq24296m_show_enable_cd,
                   bq24296m_set_enable_cd);
static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO,
                   NULL,
                   bq24296m_set_charging);
static DEVICE_ATTR(wakelock_enable, S_IWUSR | S_IRUGO,
                   bq24296m_show_wakelock_enable,
                   bq24296m_set_wakelock_enable);
static DEVICE_ATTR(chargelog_head, S_IWUSR | S_IRUGO,
                   bq24296m_show_chargelog_head,
                   NULL);
static DEVICE_ATTR(chargelog, S_IWUSR | S_IRUGO,
                   bq24296m_show_chargelog,
                   NULL);
static DEVICE_ATTR(calling_limit, S_IWUSR | S_IRUGO,
                   bq24296m_show_calling_limit,
                   bq24296m_set_calling_limit);
static DEVICE_ATTR(boostv, S_IWUSR | S_IRUGO,
                   bq24296m_show_boostv,
                   bq24296m_set_boostv);
static DEVICE_ATTR(bhot, S_IWUSR | S_IRUGO,
                   bq24296m_show_bhot,
                   bq24296m_set_bhot);
static DEVICE_ATTR(temperature_parameter, S_IWUSR | S_IRUGO,
                   bq24296m_show_temperature_parameter,
                   bq24296m_set_temperature_parameter);
static DEVICE_ATTR(set_current, S_IWUSR | S_IRUGO,
                   NULL,
                   bq24296m_set_current);
static DEVICE_ATTR(temp_current_iin, S_IWUSR | S_IRUGO,
                   bq24296m_show_temp_current_iin,
                   bq24296m_set_temp_current_iin);
static DEVICE_ATTR(temp_current_ichg, S_IWUSR | S_IRUGO,
                   bq24296m_show_temp_current_ichg,
                   bq24296m_set_temp_current_ichg);
static DEVICE_ATTR(not_limit_chrg_flag, S_IWUSR | S_IRUGO,
                   bq24296m_show_not_limit_chrg_flag,
                   bq24296m_set_not_limit_chrg_flag);
static DEVICE_ATTR(not_stop_chrg_flag, S_IWUSR | S_IRUGO,
                   bq24296m_show_not_stop_chrg_flag,
                   bq24296m_set_not_stop_chrg_flag);
static DEVICE_ATTR(enable_timer, S_IWUSR | S_IRUGO,
                   bq24296m_show_enable_timer,
                   bq24296m_set_enable_timer);
static DEVICE_ATTR(shutdown_watchdog, S_IWUSR | S_IRUGO,
                   bq24296m_show_shutdown_watchdog,
                   bq24296m_set_shutdown_watchdog);
static DEVICE_ATTR(shutdown_q4, S_IWUSR | S_IRUGO,
                   bq24296m_show_shutdown_q4,
                   bq24296m_set_shutdown_q4);
static DEVICE_ATTR(usb_current, S_IWUSR | S_IRUGO,
                   bq24296m_show_usb_current,
                   bq24296m_set_usb_current);
static struct attribute *bq24296m_attributes[] =
{
    &dev_attr_enable_hz_mode.attr,
    &dev_attr_dppm_voltage.attr,
    &dev_attr_cin_limit.attr,
    &dev_attr_regulation_voltage.attr,
    &dev_attr_charge_current.attr,
    &dev_attr_precharge_current.attr,
    &dev_attr_termination_current.attr,
    &dev_attr_enable_itermination.attr,
    &dev_attr_enable_charger.attr,
    &dev_attr_enable_batfet.attr,
    &dev_attr_enable_cd.attr,
    &dev_attr_charging.attr,
    &dev_attr_wakelock_enable.attr,
    &dev_attr_chargelog_head.attr,
    &dev_attr_boostv.attr,
    &dev_attr_bhot.attr,
    &dev_attr_chargelog.attr,
    &dev_attr_calling_limit.attr,
    &dev_attr_temperature_parameter.attr,
    &dev_attr_set_current.attr,
    &dev_attr_temp_current_iin.attr,
    &dev_attr_temp_current_ichg.attr,
    &dev_attr_not_limit_chrg_flag.attr,
    &dev_attr_not_stop_chrg_flag.attr,
    &dev_attr_enable_timer.attr,
    &dev_attr_shutdown_watchdog.attr,
    &dev_attr_shutdown_q4.attr,
    &dev_attr_usb_current.attr,
    NULL,
};

static const struct attribute_group bq24296m_attr_group =
{
    .attrs = bq24296m_attributes,
};

static int bq24296m_set_runningtest(int val)
{
    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }
    bq24296_set_enable_rt_mode_for_factory(bq_device, !val);
    return 0;
}

static int bq24296m_enable_charge(int val)
{
    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }
    bq24296_set_enable_charger_for_factory_diag(bq_device, val);
    return 0;
}

static int bq24296_set_in_thermal(int val)
{
    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }
    bq24296m_set_hot_limit_current(bq_device, val);
    return 0;
}

static int bq24296_shutdown_q4(int val)
{
    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }

    prior_shutdown_q4 = (unsigned int)val;
    bq24296m_config_misc_operation_reg(bq_device);
    return 0;
}

static int bq24296_shutdown_wd(int val)
{
    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }

    prior_shutdown_wd = (unsigned int)val;
    bq24296m_config_term_timer_reg(bq_device);
    return 0;
}

static int bq24296_set_usb_current(int val)
{
    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }

    /* if jig box set usb_current as 1000mA, should ignore 900mA*/
    if((IINLIM_1000 == usb_current) && (IINLIM_900 == val))
    {
        usb_current = IINLIM_1000;
    }
    else
    {
        usb_current = val;
    }
    bq24296m_config_input_source_reg(bq_device);
    bq24296m_config_term_timer_reg(bq_device);
    return 0;
}

static int bq24296_get_chargelog(char *reg_value)
{
    int i = 0;
    u8 read_reg[11] = {0};
    u8 buf_temp[26] = {0};

    if(!bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }

    memset(reg_value, 0, CHARGELOG_SIZE);

    bq24296m_read_block(bq_device, &read_reg[0], 0, 11);
    bq24296m_read_byte(bq_device, &read_reg[9], CHARGER_FAULT_REG09);
    for(i=0; i<11; i++)
    {
        snprintf(buf_temp,26,"0x%-8.2x",read_reg[i]);
        strncat(reg_value, buf_temp, strlen(buf_temp));
        //use comma to generate .csv document,excel can open it directly
        strncat(reg_value,",",strlen(","));
    }
    strncat(reg_value, "\n", 1);
    return 0;
}

static int  bq24296_get_ibus(void)
{
    int i;
    int cnt = 0;
    int V_temp;
    int sum = 0;
    int ibus = 0;

    if(NULL == bq_device){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }

    for (i = 0; i < SAMPLE_NUM; ++i)
    {
        V_temp = huawei_charger_get_ilimit_voltage();
        pmu_log_debug("V_temp %d\n", V_temp);
        if (V_temp >= 0)
        {
            sum += V_temp;
            ++cnt;
        }
        else
        {
            pmu_log_debug("get V_temp fail!\n");
        }
        msleep(SAMPLE_DELAY_TIME);
    }

    if (cnt > 0)
    {
        if(BQ24196 == bq_device->bqchip_version){
            ibus = ((sum * BQ24196_KILIM) / ( BQ24296M_RILIM * cnt ));
        } else {
            ibus = ((sum * BQ24296M_KILIM) / ( BQ24296M_RILIM * cnt ));
        }
    }
    else
    {
        ibus = -1; //not support ibus detect
    }

    return ibus;
}

struct charge_device_ops bq24296_ops = {
    .set_runningtest = bq24296m_set_runningtest,
    .set_enable_charger = bq24296m_enable_charge,
    .set_in_thermal = bq24296_set_in_thermal,
    .shutdown_q4 = bq24296_shutdown_q4,
    .shutdown_wd = bq24296_shutdown_wd,
    .set_usb_current = bq24296_set_usb_current,
    .dump_register = bq24296_get_chargelog,
    .get_ibus = bq24296_get_ibus,
    .charge_event_poll_register = charge_event_poll_register,
    .get_bat_status = get_bq_charge_status,
};

static enum power_supply_property bq24296m_power_supply_props[] =
    {
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_CHARGE_TYPE,
        POWER_SUPPLY_PROP_CAPACITY,
        POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_CHARGING_ENABLED,
        POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
        POWER_SUPPLY_PROP_TECHNOLOGY,
        POWER_SUPPLY_PROP_RESUME_CHARGING,
        POWER_SUPPLY_PROP_HOT_IBAT_LIMIT,
        POWER_SUPPLY_PROP_FACTORY_DIAG,
        POWER_SUPPLY_PROP_RUNNING_TEST_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
    };

static int bq24296m_power_supply_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    struct bq24296m_device_info *di = container_of(psy, struct bq24296m_device_info,
                                      charger);
    int ret = 0;
    bool usb_present = false;

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = di->charge_status;
        di->capacity = bq24296m_get_battery_capacity(di);
        usb_present = bq24296m_charger_present(di);
        if((BATT_FULL == di->capacity) && usb_present){
            val->intval = POWER_SUPPLY_STATUS_FULL;
        }
        if((POWER_SUPPLY_STATUS_CHARGING == di->charge_status)
            && (!usb_present)){
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }
        break;

    case POWER_SUPPLY_PROP_CHARGE_TYPE:
        val->intval = bq24296m_get_charge_type(di);
        break;

    case POWER_SUPPLY_PROP_TEMP:
        val->intval = huawei_charger_get_battery_temperature();
        break;

    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = huawei_charger_get_battery_voltage_now();
        break;

    case POWER_SUPPLY_PROP_CURRENT_NOW:
        bq24296m_charger_get_ext_property(di->bms_psy, POWER_SUPPLY_PROP_CURRENT_NOW, val);
        break;

    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = bq24296m_get_battery_health();
        break;

    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = huawei_charger_battery_is_exist();
        break;

    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        val->intval = !di->rt_discharge_flag;
        break;

    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = bq24296m_get_battery_capacity(di);
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        bq24296m_charger_get_ext_property(di->bms_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, val);
        break;

    case POWER_SUPPLY_PROP_RESUME_CHARGING:
        val->intval = di->soc_resume_charging;
        break;

    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
        break;

    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        val->intval = di->hot_limit_current;
        break;
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
        val->intval = di->factory_flag;
        break;

    case POWER_SUPPLY_PROP_RUNNING_TEST_STATUS:
        val->intval = get_running_test_status();
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        val->intval = di->max_voltagemV;
        break;

    default:
        return -EINVAL;
    }
    return ret;
}

static int bq24296m_power_supply_set_property(struct power_supply *psy,
        enum power_supply_property psp,
        const union power_supply_propval *val)
{
    struct bq24296m_device_info *di = container_of(psy, struct bq24296m_device_info,
                                      charger);
    int rc = 0;

    switch (psp)
    {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        bq24296_set_enable_rt_mode_for_factory(di, (!(val->intval)));
        set_running_test_flag(!val->intval);
        break;
    case POWER_SUPPLY_PROP_STATUS:
        if(POWER_SUPPLY_STATUS_FULL == val->intval){
            di->battery_full = true;
        }
        break;
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        pmu_log_info("set hot-ibat limit %d \n", val->intval);
        bq24296m_set_hot_limit_current(di, val->intval);
        break;
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
        bq24296_set_enable_charger_for_factory_diag(di, val->intval);
        break;
    case POWER_SUPPLY_PROP_RESUME_CHARGING:
        di->soc_resume_charging = val->intval;
        if(di->soc_resume_charging && di->battery_full){
            di->battery_full = false;
            if(di->battery_present && bq24296m_charger_present(di)){
                resume_charging_checking(di);
            }
        }
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        di->fake_battery_soc = val->intval;
        pmu_log_info("set user-defined battery fake soc %d \n", di->fake_battery_soc);
        power_supply_changed(&di->charger);
        break;
    default:
        return -EINVAL;
    }
    return rc;
}

static int bq24296m_property_is_writeable(struct power_supply *psy,
        enum power_supply_property psp)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
    case POWER_SUPPLY_PROP_RESUME_CHARGING:
    case POWER_SUPPLY_PROP_CAPACITY:
        return 1;
    default:
        break;
    }

    return 0;
}

static void bq24296m_external_power_changed(struct power_supply *psy)
{
    struct bq24296m_device_info *di = container_of(psy, struct bq24296m_device_info,
                                      charger);
    union power_supply_propval ret = {0,};
    int current_ma = 0, charger_present = 0;
    unsigned long flags;

    spin_lock_irqsave(&di->psy_lock, flags);
    if (!bq24296m_bms_psy(di))
    {
        pmu_log_err("the battery power supply is not got!!\n");
    }

    spin_unlock_irqrestore(&di->psy_lock, flags);

    charger_present = bq24296m_charger_present(di);

    if(charger_present)
    {
        di->usb_psy->get_property(di->usb_psy,
                                  POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
        current_ma = ret.intval / 1000;
        if(current_ma != di->charge_current_limit)
        {
            mutex_lock(&di->current_change_lock);
            di->charge_current_limit = current_ma;
            mutex_unlock(&di->current_change_lock);
            handle_charging_by_current_limit(di);
        }
    }

    if(charger_present ^ di->charger_present)
    {
        /* reset the max cin to the config value */
        di->max_cin_currentmA = di->max_cin_cfg_currentmA;
        di->ibus_detecting = false;
        di->charger_present = charger_present;
        if(!charger_present)
        {
            /* the charger is pluged out, stop the detect at once */
            cancel_delayed_work_sync(&di->ibus_detect_work);
            stop_charging(di);
        }
    }
}

static char *bq24296m_supplied_to[] =
{
    "bms",
    "ti-bms",
};

static int bq24296m_power_supply_init(struct bq24296m_device_info *di)
{
    int ret;
    di->charger.name = "battery";
    di->charger.type = POWER_SUPPLY_TYPE_BATTERY;
    di->charger.properties = bq24296m_power_supply_props;
    di->charger.num_properties = ARRAY_SIZE(bq24296m_power_supply_props);
    di->charger.get_property = bq24296m_power_supply_get_property;
    di->charger.set_property = bq24296m_power_supply_set_property;
    di->charger.property_is_writeable = bq24296m_property_is_writeable;
    di->charger.external_power_changed = bq24296m_external_power_changed;
    di->charger.supplied_to = bq24296m_supplied_to;
    di->charger.num_supplicants =ARRAY_SIZE(bq24296m_supplied_to);

    ret = power_supply_register(di->dev, &di->charger);
    if (ret)
    {
        return ret;
    }

    return 0;
}

static int  bq24296m_charger_probe(struct i2c_client *client,
                                   const struct i2c_device_id *id)
{
    struct bq24296m_device_info *di;
    struct charge_device_ops *ops = NULL;
    int ret = 0;
    u8 read_reg = 0;
    struct power_supply *usb_psy;

    mutex_init(&dpm_status_access_lock);
    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy)
    {
        pr_err("usb supply not found deferring probe\n");
        return -EPROBE_DEFER;
    }

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di)
    {
        pr_err("bq2419x_device_info is NULL!\n");
        return -ENOMEM;
    }

    di->dev = &client->dev;
    di->client = client;
    i2c_set_clientdata(client, di);
    di->usb_psy = usb_psy;
    ops = &bq24296_ops;
    ret = charge_ops_register(ops);
    if(ret)
    {
        pmu_log_err("register charge ops failed!\n");
        goto err_kfree;
    }

    ret = bq24296m_read_byte(di, &read_reg, PART_REVISION_REG0A);
    if (ret < 0)
    {
        pmu_log_err("chip not present at address %x\n",client->addr);
        ret = -EINVAL;
        goto err_kfree;
    }
    if (((read_reg & 0x20) == BQ24296M ) && (client->addr == 0x6B))
    {
        di->bqchip_version = BQ24296M;
    }
    if (((read_reg & BQ_VID_MASK) == BQ24196) && (client->addr == 0x6B))
    {
        di->bqchip_version = BQ24196;
    }
    pr_info("Charger IC is %x",di->bqchip_version);
    if (di->bqchip_version == 0)
    {
        pr_debug("unknown bq chip\n");
        pr_debug("Chip address %x", client->addr);
        pr_debug("bq chip version reg value %x", read_reg);
        ret = -EINVAL;
        goto err_kfree;
    }

    ret = bq24296m_parse_dts_charge_parameter(di);
    if(!ret)
    {
        di->max_voltagemV = 4208;
        di->max_currentmA = 1000;
        max_currentmA_bak  = di->max_currentmA;
        di->max_cin_currentmA = 1000;
        di->no_ibus_detect = true;
    }
    di->max_cin_cfg_currentmA = di->max_cin_currentmA;
    di->ibus_detecting = false;

    INIT_DELAYED_WORK(&di->otg_int_work, bq24296m_otg_int_work);

    di->gpio_cd = of_get_named_gpio(di->dev->of_node,"gpio_cd",0);
    if(!gpio_is_valid(di->gpio_cd))
    {
        pr_err("gpio_cd is not valid\n");
        ret = -EINVAL;
        goto err_kfree;
    }
    di->gpio_int = of_get_named_gpio(di->dev->of_node,"gpio_int",0);
    if(!gpio_is_valid(di->gpio_int))
    {
        pr_err("gpio_int is not valid\n");
        ret = -EINVAL;
        goto err_kfree;
    }

    pr_info("max charge current check: max_currentmA = %d, max_cin_currentmA = %d.\n", di->max_currentmA, di->max_cin_currentmA);

    /*set gpio_108 to control CD pin to disable/enable bq24296m IC*/
    ret = gpio_request(di->gpio_cd, "charger_cd");
    if (ret)
    {
        pmu_log_err("could not request gpio_cd\n");
        ret = -ENOMEM;
        goto err_io;
    }
    gpio_direction_output(di->gpio_cd, 0);/*GPIO output, low level enable charge*/

    ret = gpio_request(di->gpio_int,"charger_int");
    if(ret)
    {
        pmu_log_err("could not request gpio_int\n");
        goto err_gpio_int;
    }
    gpio_direction_input(di->gpio_int);/* interrupt */

    di->irq_int = gpio_to_irq(di->gpio_int);
    if(di->irq_int < 0)
    {
        pmu_log_err("could not map gpio_int to irq\n");
        goto err_int_map_irq;
    }
    ret = request_irq(di->irq_int,bq24296m_irq_int_interrupt,IRQF_TRIGGER_FALLING,"charger_int_irq",di);
    if(ret)
    {
        pmu_log_err("could not request irq_int\n");
        di->irq_int = -1;
        goto err_irq_int;
    }
    disable_irq(di->irq_int);
    irq_int_active = 0;

    ret = bq24296m_regulator_init(di);
    if (ret)
    {
        dev_err(&client->dev,"Couldn't initialize bq24296m regulator rc=%d\n", ret);
        goto err_irq_int;
    }

    di->voltagemV = di->max_voltagemV;/* regulation voltage */
    di->currentmA = ICHG_512 ;/* fast charge current */
    di->cin_dpmmV = VINDPM_4520;/* vindpm voltage threshold */
    di->cin_limit = IINLIM_500;/* input current limit */
    di->sys_minmV = SYS_MIN_3500;/* SYS voltage */
    di->prechrg_currentmA = IPRECHRG_256;/* precharge current */
    di->term_currentmA = ITERM_MIN_128;/* termination current */
    di->watchdog_timer = WATCHDOG_80;
    di->chrg_timer = CHG_TIMER_12;

    di->hz_mode = DIS_HIZ;
    di->chrg_config = EN_CHARGER;
    di->boost_lim = BOOST_LIM_1000;
    di->enable_low_chg = DIS_FORCE_20PCT;
    di->enable_iterm = EN_TERM;
    di->enable_timer = EN_TIMER;
    di->enable_timer_temp = EN_TIMER;
    di->enable_batfet = EN_BATFET;

    di->cd_active = 0;
    di->cfg_params = 1;
    di->enable_dpdm = DPDM_DIS;
    di->battery_full = false;
    di->soc_resume_charging = false;
    di->charge_full_count = 0;

    input_current_iin = di->max_cin_currentmA;
    input_current_ichg = di->max_currentmA;
    iin_temp = IINLIM_500;
    di->not_limit_chrg_flag = 0;
    di->not_stop_chrg_flag = 0;
    di->factory_flag = EN_CHARGER;
    di->charger_present = 0;
    di->rt_discharge_flag = 0;
    di->hot_limit_current = di->max_cin_currentmA;
    di->temp_ctrl = &bq24296_temp_info;
    di->capacity = BATT_DEFAULT_SOC;
    if(bq24296m_charger_present(di))
    {
        di->charger_present = 1;
    }
    di->fake_battery_soc = -EINVAL;

    bq24296m_config_power_on_reg(di);
    bq24296m_config_current_reg(di);
    bq24296m_config_prechg_term_current_reg(di);
    bq24296m_config_voltage_reg(di);
    bq24296m_config_term_timer_reg(di);
    bq24296m_config_thernal_regulation_reg(di);
    bq24296m_config_limit_temperature_parameter(di);

    wake_lock_init(&chrg_lock, WAKE_LOCK_SUSPEND, "bq24296m_chrg_wakelock");
    wake_lock_init(&stop_chrg_lock, WAKE_LOCK_SUSPEND, "bq24296m_stop_chrg_wakelock");
    mutex_init(&di->hot_limit_lock);
    mutex_init(&di->current_change_lock);
    spin_lock_init(&di->psy_lock);

    ret = bq24296m_power_supply_init(di);
    if (ret)
    {
        dev_err(di->dev, "failed to register power supply: %d\n", ret);
        goto err_irq_int;
    }

    di->design_capacity = BQ24296M_DEFAULT_CAPACITY;
    di->charge_in_temp_5 = DEFAULT_CHARGE_PARAM_LOW_TEMP;
    di->charge_in_temp_10 = DEFAULT_CHARGE_PARAM_LOW_TEMP;

    INIT_DELAYED_WORK(&di->bq24296m_charger_work,
                      bq24296m_charger_work);

    INIT_DELAYED_WORK(&di->bq24296m_usb_otg_work,
                      bq24296m_usb_otg_work);

    INIT_DELAYED_WORK(&di->ibus_detect_work,
                      bq24296m_detect_ibus_work);

    INIT_WORK(&di->usb_work, bq24296m_usb_charger_work);

    ret = sysfs_create_group(&client->dev.kobj, &bq24296m_attr_group);
    if (ret)
    {
        pr_debug("could not create sysfs files\n");
        goto err_sysfs;
    }

    if(!di->charger_present){
        di->event = CHARGER_REMOVED;
        schedule_work(&di->usb_work);
    }else{
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    }

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_CHARGER);
#endif
    bq_device = di;
    pr_info("bq24296m probe ok!\n");
    return 0;

err_sysfs:
    regulator_unregister(di->otg_vreg.rdev);
    power_supply_unregister(&di->charger);
    mutex_destroy(&di->current_change_lock);
    mutex_destroy(&di->hot_limit_lock);
    wake_lock_destroy(&chrg_lock);
    wake_lock_destroy(&stop_chrg_lock);
    free_irq(di->irq_int,di);
err_irq_int:
err_int_map_irq:
    gpio_free(di->gpio_int);
err_gpio_int:
    gpio_free(di->gpio_cd);
err_io:
err_kfree:
    kfree(di);
    di = NULL;

    return ret;
}

static int  bq24296m_charger_remove(struct i2c_client *client)
{
    struct bq24296m_device_info *di = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &bq24296m_attr_group);
    regulator_unregister(di->otg_vreg.rdev);
    wake_lock_destroy(&chrg_lock);
    wake_lock_destroy(&stop_chrg_lock);
    mutex_destroy(&dpm_status_access_lock);
    mutex_destroy(&di->current_change_lock);
    mutex_destroy(&di->hot_limit_lock);
    cancel_delayed_work_sync(&di->bq24296m_charger_work);
    cancel_delayed_work_sync(&di->bq24296m_usb_otg_work);
    cancel_delayed_work_sync(&di->ibus_detect_work);
    cancel_delayed_work_sync(&di->otg_int_work);
    gpio_set_value(di->gpio_cd, !di->gpio_cd_level_reverse);
    gpio_free(di->gpio_cd);
    free_irq(di->irq_int,di);
    gpio_free(di->gpio_int);
    kfree(di);
    return 0;
}

static void bq24296m_charger_shutdown(struct i2c_client *client)
{
    struct bq24296m_device_info *di = i2c_get_clientdata(client);
    if(bq24296m_charger_present(di))
    {
        di->chrg_config = DIS_CHARGER;
        bq24296m_config_power_on_reg(di);
    }
    cancel_delayed_work_sync(&di->bq24296m_charger_work);
    cancel_delayed_work_sync(&di->bq24296m_usb_otg_work);
    cancel_delayed_work_sync(&di->ibus_detect_work);
    cancel_delayed_work_sync(&di->otg_int_work);

    return;
}

static const struct i2c_device_id bq24296m_id[] =
{
    { "bq24296m_charger", 0 },
    {},
};

#ifdef CONFIG_PM
static int bq24296m_charger_suspend(struct i2c_client *client,
                                    pm_message_t state)
{
    struct bq24296m_device_info *di = i2c_get_clientdata(client);

    bq24296m_config_power_on_reg(di);
    return 0;
}

static int bq24296m_charger_resume(struct i2c_client *client)
{
    struct bq24296m_device_info *di = i2c_get_clientdata(client);

    bq24296m_config_power_on_reg(di);
    return 0;
}
#else
#define bq24296m_charger_suspend       NULL
#define bq24296m_charger_resume        NULL
#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(i2c, bq24296m);
static struct of_device_id bq24296m_charger_match_table[] =
{
    {
        .compatible = "huawei,bq24296m_charger",
        .data = NULL,
    },
    {
    },
};
static struct i2c_driver bq24296m_charger_driver =
{
    .probe = bq24296m_charger_probe,
    .remove = bq24296m_charger_remove,
    .suspend = bq24296m_charger_suspend,
    .resume = bq24296m_charger_resume,
    .shutdown = bq24296m_charger_shutdown,
    .id_table = bq24296m_id,
    .driver = {
        .owner = THIS_MODULE,
        .name = "bq24296m_charger",
        .of_match_table = of_match_ptr(bq24296m_charger_match_table),
    },
};

static int __init bq24296m_charger_init(void)
{
    return i2c_add_driver(&bq24296m_charger_driver);
}
device_initcall_sync(bq24296m_charger_init);

static void __exit bq24296m_charger_exit(void)
{
    i2c_del_driver(&bq24296m_charger_driver);
}
module_exit(bq24296m_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HW Inc");
