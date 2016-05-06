/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2012  Pali Rohár <pali.rohar@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Datasheets:
 * http://www.ti.com/product/bq24150
 * http://www.ti.com/product/bq24150a
 * http://www.ti.com/product/bq24152
 * http://www.ti.com/product/bq24153
 * http://www.ti.com/product/bq24153a
 * http://www.ti.com/product/bq24155
 */
//#define DEBUG
#define pr_fmt(fmt)	"Ti-CHARGER: %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/power/bq24152_charger.h>

#include <linux/power/huawei_charger.h>
#include <linux/charger_core.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <linux/power/bq27510_bms.h>
#include <linux/batterydata-lib.h>
#define STATUS_FULL 4
#define STATUS_DISCHARGING 0
#define STATUS_CHARGING 1
/*temperature hysteresis is 2 centigrade*/
#define TEMP_HYSTERESIS 20
#define  VOLTAGE_NOW_DEFULT  (1000)
#define  CURRENT_NOW_DEFULT  (-1)
#define  CHARGE_FULL_DESIGN_DEFULT  (1500)
#define  CAPACITY_DEFAULT  (50)
/*default temp is 9 centigrade*/
#define TEMP_DEFAULT 90
#define STATUS_MIGRATE 1
#define STATUS_KEEP 0
#define TERM_CUR (-100)
#define CHARGE_FULL_COUNT (30)
#define HOT_MAX_CURRENT 1440
#define HWLOG_TAG bq24152_charger
static int hot_current = HOT_MAX_CURRENT;

jeita_spec jeita_batt_param =
{
	.normal	= {100,420,1250,4340},
	.hot.t_high = -1
};
static int g_current_config = 0;
static bool current_config_changed = false;
static DEFINE_SPINLOCK(current_config_changed_lock);

static bool user_ctl_status = true;
#define BAT_CAPACITY_FULL    (100)
static struct bq2415x_device *g_bq;
extern int get_true_bms_soc(void);
extern int get_ui_bms_soc(void);
static int factory_diag_flag = 0;
static int factory_diag_last_current_ma = 0;
static int bq2415x_set_in_thermal(int val);
//delete dead code.


static void bq2415x_set_appropriate_jeita(struct bq2415x_device *bq);
static int jeita_find_running_zone(jeita_entry **r_entry,jeita_spec *batt_param);

/* timeout for resetting chip timer */
#define BQ2415X_TIMER_TIMEOUT		2
#define HYSTERTSIS_TIME 20  //sec
#define USB_CURRENT_LIMIT	540
#define USB_CHARGE_CURRENT	550
#define CURRENT_LIMIT_100MA	100
#define CURRENT_LIMIT_500MA	500
#define CURRENT_LIMIT_800MA	800
#define CURRENT_LIMIT_1800MA	1800
#define MAX_CHARGE_CURRENT	1150
#define BATTERY_VOL_THRESHOLD	3600
#define BQ24152_REG_0_FAULT_MASK	0x07
#define BQ24152_REG_0_STAT_MASK		0x30
#define BQ24152_CHG_STAT_FAULT	3
#define POOR_INPUT_FAULT_STATUS	3
#define CHARGE_CURRENT_STEP	100

#define DELAYED_TIME    2500
#define BQ2415X_REG_STATUS		0x00
#define BQ2415X_REG_CONTROL		0x01
#define BQ2415X_REG_VOLTAGE		0x02
#define BQ2415X_REG_VENDER		0x03
#define BQ2415X_REG_CURRENT		0x04

/* reset state for all registers */
#define BQ2415X_RESET_STATUS		BIT(6)
#define BQ2415X_RESET_CONTROL		(BIT(4)|BIT(5))
#define BQ2415X_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define BQ2415X_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/* status register */
#define BQ2415X_BIT_TMR_RST		7
#define BQ2415X_BIT_OTG			7
#define BQ2415X_BIT_EN_STAT		6
#define BQ2415X_MASK_STAT		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_STAT		4
#define BQ2415X_BIT_BOOST		3
#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT		0

/* control register */
#define BQ2415X_MASK_LIMIT		(BIT(6)|BIT(7))
#define BQ2415X_SHIFT_LIMIT		6
#define BQ2415X_MASK_VLOWV		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_VLOWV		4
#define BQ2415X_BIT_TE			3
#define BQ2415X_BIT_CE			2
#define BQ2415X_BIT_HZ_MODE		1
#define BQ2415X_BIT_OPA_MODE		0

/* voltage register */
#define BQ2415X_MASK_VO		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VO		2
#define BQ2415X_BIT_OTG_PL		1
#define BQ2415X_BIT_OTG_EN		0

/* vender register */
#define BQ2415X_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VENDER		5
#define BQ2415X_MASK_PN			(BIT(3)|BIT(4))
#define BQ2415X_SHIFT_PN		3
#define BQ2415X_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_REVISION		0

/* current register */
#define BQ2415X_MASK_RESET		BIT(7)
#define BQ2415X_MASK_VI_CHRG		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_VI_CHRG		4
/* N/A					BIT(3) */
#define BQ2415X_MASK_VI_TERM		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_VI_TERM		0


//move code to bq24152_charger.h

static char *bq2415x_chip_name[] = {
	"unknown",
	"bq24150",
	"bq24150a",
	"bq24151",
	"bq24151a",
	"bq24152",
	"bq24153",
	"bq24153a",
	"bq24155",
	"bq24156",
	"bq24156a",
	"bq24158",
};

//move code to bq24152_charger.h

/* each registered chip must have unique id */
static DEFINE_IDR(bq2415x_id);

static DEFINE_MUTEX(bq2415x_id_mutex);
static DEFINE_MUTEX(bq2415x_timer_mutex);
static DEFINE_MUTEX(bq2415x_i2c_mutex);

extern int huawei_charger_get_battery_temperature(void);
extern int huawei_charger_get_battery_voltage_now(void);
extern int set_running_test_flag(int value);
extern int is_usb_chg_exist(void);
extern int qpnp_lbc_is_in_vin_min_loop(void);
static int bq2415x_enable_charge(int val);
static int bq2415x_factory_diag(int val);

static int poor_input_enable = 0;

/**** i2c read functions ****/

/* read value from register */
static int bq2415x_i2c_read(struct bq2415x_device *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[2];
	u8 val;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&bq2415x_i2c_mutex);

	if (ret < 0)
		return ret;

	return val;
}



/* read value from register, apply mask and right shift it */
static int bq2415x_i2c_read_mask(struct bq2415x_device *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

/* read value from register and return one specified bit */
static int bq2415x_i2c_read_bit(struct bq2415x_device *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_read_mask(bq, reg, BIT(bit), bit);
}

/**** i2c write functions ****/

/* write value to register */
static int bq2415x_i2c_write(struct bq2415x_device *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	mutex_lock(&bq2415x_i2c_mutex);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&bq2415x_i2c_mutex);

	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}

/* read value from register, change it with mask left shifted and write back */
static int bq2415x_i2c_write_mask(struct bq2415x_device *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;
    int rc;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
    {
        pmu_log_info("bq24152 i2c read faild!\n");
		return ret;
    }

	ret &= ~mask;
	ret |= val << shift;

    rc = bq2415x_i2c_write(bq, reg, ret);
    if(rc < 0)
    {
        pmu_log_info("bq24152 i2c write faild!\n");
    }
    return rc;
}

/* change only one bit in register */
static int bq2415x_i2c_write_bit(struct bq2415x_device *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

/**** global functions ****/

/* exec command function */
static int bq2415x_exec_command(struct bq2415x_device *bq,
				enum bq2415x_command command)
{
	switch (command) {
	case BQ2415X_TIMER_RESET:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS,
				1, BQ2415X_BIT_TMR_RST);
	case BQ2415X_OTG_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_OTG);
	case BQ2415X_STAT_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 1,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 0,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_CHARGE_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
				BQ2415X_MASK_STAT, BQ2415X_SHIFT_STAT);
	case BQ2415X_BOOST_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_BOOST);
	case BQ2415X_FAULT_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
			BQ2415X_MASK_FAULT, BQ2415X_SHIFT_FAULT);

	case BQ2415X_CHARGE_TERMINATION_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_TE);
	case BQ2415X_CHARGER_STATUS:
		return !bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
			BQ2415X_BIT_CE);
	case BQ2415X_CHARGER_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_CE);
	case BQ2415X_CHARGER_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_CE);
	case BQ2415X_HIGH_IMPEDANCE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_BOOST_MODE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_OPA_MODE);

	case BQ2415X_OTG_LEVEL:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_HIGH:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_LOW:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_EN);

	case BQ2415X_VENDER_CODE:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_VENDER, BQ2415X_SHIFT_VENDER);
	case BQ2415X_PART_NUMBER:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
				BQ2415X_MASK_PN, BQ2415X_SHIFT_PN);
	case BQ2415X_REVISION:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_REVISION, BQ2415X_SHIFT_REVISION);
	}
	return -EINVAL;
}

/* detect chip type */
static enum bq2415x_chip bq2415x_detect_chip(struct bq2415x_device *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret = bq2415x_exec_command(bq, BQ2415X_PART_NUMBER);

	if (ret < 0)
		return ret;

	switch (client->addr) {
	case 0x6b:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24151A)
				return bq->chip;
			else
				return BQ24151;
		case 1:
			if (bq->chip == BQ24150A ||
				bq->chip == BQ24152 ||
				bq->chip == BQ24155)
				return bq->chip;
			else
				return BQ24150;
		case 2:
			if (bq->chip == BQ24153A)
				return bq->chip;
			else
				return BQ24153;
		default:
			return BQUNKNOWN;
		}
		break;

	case 0x6a:
		switch (ret) {
		case 0:
			if (bq->chip == BQ24156A)
				return bq->chip;
			else
				return BQ24156;
		case 2:
			return BQ24158;
		default:
			return BQUNKNOWN;
		}
		break;
	}

	return BQUNKNOWN;
}

/* detect chip revision */
static int bq2415x_detect_revision(struct bq2415x_device *bq)
{
	int ret = bq2415x_exec_command(bq, BQ2415X_REVISION);
	int chip = bq2415x_detect_chip(bq);

	if (ret < 0 || chip < 0)
		return -1;

	switch (chip) {
	case BQ24150:
	case BQ24150A:
	case BQ24151:
	case BQ24151A:
	case BQ24152:
		if (ret >= 0 && ret <= 3)
			return ret;
		else
			return -1;
	case BQ24153:
	case BQ24153A:
	case BQ24156:
	case BQ24156A:
	case BQ24158:
		if (ret == 3)
			return 0;
		else if (ret == 1)
			return 1;
		else
			return -1;
	case BQ24155:
		if (ret == 3)
			return 3;
		else
			return -1;
	case BQUNKNOWN:
		return -1;
	}

	return -1;
}

/* return chip vender code */
static int bq2415x_get_vender_code(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_exec_command(bq, BQ2415X_VENDER_CODE);
	if (ret < 0)
		return 0;

	/* convert to binary */
	return (ret & 0x1) +
	       ((ret >> 1) & 0x1) * 10 +
	       ((ret >> 2) & 0x1) * 100;
}

/* reset all chip registers to default state */
static void bq2415x_reset_chip(struct bq2415x_device *bq)
{
	bq2415x_i2c_write(bq, BQ2415X_REG_CURRENT, BQ2415X_RESET_CURRENT);
	bq2415x_i2c_write(bq, BQ2415X_REG_VOLTAGE, BQ2415X_RESET_VOLTAGE);
	bq2415x_i2c_write(bq, BQ2415X_REG_CONTROL, BQ2415X_RESET_CONTROL);
	bq2415x_i2c_write(bq, BQ2415X_REG_STATUS, BQ2415X_RESET_STATUS);
	bq->timer_error = NULL;
}

/**** properties functions ****/

/* set current limit in mA */
static int bq2415x_set_current_limit(struct bq2415x_device *bq, int mA)
{
	int val;

	if (mA <= 100)
		val = 0;
	else if (mA <= USB_CURRENT_LIMIT)
		val = 1;
	else if (mA <= 800)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
}

/* get current limit in mA */
static int bq2415x_get_current_limit(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CONTROL,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
	if (ret < 0)
		return ret;
	else if (ret == 0)
		return 100;
	else if (ret == 1)
		return 500;
	else if (ret == 2)
		return 800;
	else if (ret == 3)
		return 1800;
	return -EINVAL;
}

/* set weak battery voltage in mV */
static int bq2415x_set_weak_battery_voltage(struct bq2415x_device *bq, int mV)
{
	int val;

	/* round to 100mV */
	if (mV <= 3400 + 50)
		val = 0;
	else if (mV <= 3500 + 50)
		val = 1;
	else if (mV <= 3600 + 50)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
}

/* get weak battery voltage in mV */
static int bq2415x_get_weak_battery_voltage(struct bq2415x_device *bq)
{
	int ret;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CONTROL,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
	if (ret < 0)
		return ret;
	return 100 * (34 + ret);
}

/* set battery regulation voltage in mV */
static int bq2415x_set_battery_regulation_voltage(struct bq2415x_device *bq,
						  int mV)
{
	int val = (mV/10 - 350) / 2;

	if (val < 0)
		val = 0;
	else if (val > 47)
		return -EINVAL;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_VOLTAGE, val,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);
}

/* get battery regulation voltage in mV */
static int bq2415x_get_battery_regulation_voltage(struct bq2415x_device *bq)
{
	int ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_VOLTAGE,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);

	if (ret < 0)
		return ret;
	return 10 * (350 + 2*ret);
}

/* set charge current in mA (platform data must provide resistor sense) */
static int bq2415x_set_charge_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	val = (mA * bq->init_data.resistor_sense - 37400) / 6800;
    /* limit the max charge current as 1150 for msm8916*/
	if (val < 0)
		val = 0;
	else if (val > 6)
		val = 6;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_CHRG | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_CHRG);
}

/* get charge current in mA (platform data must provide resistor sense) */
static int bq2415x_get_charge_current(struct bq2415x_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
			BQ2415X_MASK_VI_CHRG, BQ2415X_SHIFT_VI_CHRG);
	if (ret < 0)
		return ret;
	return (37400 + 6800*ret) / bq->init_data.resistor_sense;
}

/* set termination current in mA (platform data must provide resistor sense) */
static int bq2415x_set_termination_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	val = (mA * bq->init_data.resistor_sense - 3400) / 3400;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_TERM | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_TERM);
}

/* get termination current in mA (platform data must provide resistor sense) */
static int bq2415x_get_termination_current(struct bq2415x_device *bq)
{
	int ret;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	ret = bq2415x_i2c_read_mask(bq, BQ2415X_REG_CURRENT,
			BQ2415X_MASK_VI_TERM, BQ2415X_SHIFT_VI_TERM);
	if (ret < 0)
		return ret;
	return (3400 + 3400*ret) / bq->init_data.resistor_sense;
}

/* set default value of property */
#define bq2415x_set_default_value(bq, prop) \
	do { \
		int ret = 0; \
		if (bq->init_data.prop != -1) \
			ret = bq2415x_set_##prop(bq, bq->init_data.prop); \
		if (ret < 0) \
			return ret; \
	} while (0)

/* set default values of all properties */
static int bq2415x_set_defaults(struct bq2415x_device *bq)
{
	int rc = 0;

	rc = bq2415x_exec_command(bq, BQ2415X_STAT_PIN_DISABLE);
	if(rc < 0)
	{
		pmu_log_info("disable pin failed\n");
		return rc;
	}

	rc = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);
	if(rc < 0)
	{
		pmu_log_info("boost mode set failed\n");
		return rc;
	}
	rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
	if(rc < 0)
	{
		pmu_log_info("charger disable set failed\n");
		return rc;
	}

	rc = bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_DISABLE);
	if(rc < 0)
	{
		pmu_log_info("charge termination disable set failed\n");
		return rc;
	}

	bq2415x_set_default_value(bq, current_limit);
	bq2415x_set_default_value(bq, weak_battery_voltage);
	bq2415x_set_default_value(bq, battery_regulation_voltage);

	if (bq->init_data.resistor_sense > 0) {
        /* remove the code of setting 1.25A charging current when driver init*/
		bq2415x_set_default_value(bq, termination_current);
		/* disable termination of charger */
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_DISABLE);
		if(rc < 0)
		{
			pmu_log_info("charge termination enable set failed\n");
			return rc;
		}
	}

	rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
	if(rc < 0)
	{
		pmu_log_info("charge enable set failed\n");
		return rc;
	}
	return 0;
}

/**** charger mode functions ****/

/* set charger mode */
static int bq2415x_set_mode(struct bq2415x_device *bq, enum bq2415x_mode mode)
{
	int ret = 0;
	int charger = 0;
	int boost = 0;

	if (mode == BQ2415X_MODE_BOOST)
		boost = 1;
	else if (mode != BQ2415X_MODE_OFF)
		charger = 1;

	if (!charger)
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);

	if (!boost)
		ret = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);

	if (ret < 0)
		return ret;

	switch (mode) {
	case BQ2415X_MODE_OFF:
		dev_dbg(bq->dev, "changing mode to: Offline\n");
		ret = bq2415x_set_current_limit(bq, 100);
		break;
	case BQ2415X_MODE_NONE:
		dev_dbg(bq->dev, "changing mode to: N/A\n");
		ret = bq2415x_set_current_limit(bq, 100);
		break;
	case BQ2415X_MODE_HOST_CHARGER:
		dev_dbg(bq->dev, "changing mode to: Host/HUB charger\n");
		ret = bq2415x_set_current_limit(bq, 500);
		break;
	case BQ2415X_MODE_DEDICATED_CHARGER:
		dev_dbg(bq->dev, "changing mode to: Dedicated charger\n");
		ret = bq2415x_set_current_limit(bq, 1800);
		break;
	case BQ2415X_MODE_BOOST: /* Boost mode */
		dev_dbg(bq->dev, "changing mode to: Boost\n");
		ret = bq2415x_set_current_limit(bq, 100);
		break;
	}

	if (ret < 0)
		return ret;

	if (charger)
		ret = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
	else if (boost)
		ret = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_ENABLE);

	if (ret < 0)
		return ret;

	bq2415x_set_default_value(bq, weak_battery_voltage);
	bq2415x_set_default_value(bq, battery_regulation_voltage);

	bq->mode = mode;
	sysfs_notify(&bq->charger.dev->kobj, NULL, "mode");

	return 0;

}

/*===========================================
FUNCTION: get_bq2415x_fault_status
DESCRIPTION: to get bq2415x Reg0 fault bits value
IPNUT: none
RETURN:	a int value, the error status of bq2415x
=============================================*/
int get_bq2415x_fault_status(void)
{
	int error = 0;
	if(g_bq == NULL)
	{
		pmu_log_info("%s:device not init,do nothing!\n",__func__);
		return -EINVAL;
	}
	error = bq2415x_exec_command(g_bq, BQ2415X_FAULT_STATUS);
	if (error < 0) {
		pmu_log_err ("Unknown error\n");
	}
	return error;
}
EXPORT_SYMBOL(get_bq2415x_fault_status);

/*===========================================
FUNCTION: is_bq24152_in_boost_mode
DESCRIPTION:
IPNUT: none
RETURN:	a int value, 1 means bq24152 is in boost mode, 0 means charger mode
=============================================*/
int is_bq24152_in_boost_mode(void)
{
	int ret = 0;
	if(g_bq == NULL)
	{
		pmu_log_info("%s:device not init,do nothing!\n",__func__);
		return -EINVAL;
	}
	ret = bq2415x_exec_command(g_bq, BQ2415X_BOOST_STATUS);
	if (ret < 0)
		return ret;
	if(0 == ret)
		return 0;
	if(1 == ret)
		return 1;
	return 0;
}

void bq24152_control_otg(struct bq2415x_device *bq,bool enable)
{
	int mode = 0;
	mode = is_bq24152_in_boost_mode();
	if(enable){
		pmu_log_info("bq2415x_set_mode: boost mode, enable=%d\n", enable);
		if(1 == mode)
			return;
		bq2415x_set_mode(bq, BQ2415X_MODE_BOOST);
	}else{
		pmu_log_info("bq2415x_set_mode: normal mode, enable=%d\n", enable);
		if(0 == mode)
			return;
		bq2415x_set_mode(bq, BQ2415X_MODE_OFF);
	}
}
/* hook function called by other driver which set reported mode */
static void bq2415x_hook_function(enum bq2415x_mode mode, void *data)
{
	struct bq2415x_device *bq = data;

	if (!bq)
		return;

	dev_dbg(bq->dev, "hook function was called\n");
	bq->reported_mode = mode;

	/* if automode is not enabled do not tell about reported_mode */
	if (bq->automode < 1)
		return;

	sysfs_notify(&bq->charger.dev->kobj, NULL, "reported_mode");
	bq2415x_set_mode(bq, bq->reported_mode);

}

/**** timer functions ****/

/* enable/disable auto resetting chip timer */
static void bq2415x_set_autotimer(struct bq2415x_device *bq, int state)
{
	mutex_lock(&bq2415x_timer_mutex);

	if (bq->autotimer == state) {
		mutex_unlock(&bq2415x_timer_mutex);
		return;
	}

	bq->autotimer = state;

	if (state) {
		schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
		bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
		bq->timer_error = NULL;
	} else {
		cancel_delayed_work_sync(&bq->work);
	}

	mutex_unlock(&bq2415x_timer_mutex);
}

/* called by bq2415x_timer_work on timer error */
#ifndef CONFIG_HUAWEI_KERNEL
static void bq2415x_timer_error(struct bq2415x_device *bq, const char *msg)
{
	bq->timer_error = msg;
	sysfs_notify(&bq->charger.dev->kobj, NULL, "timer");
	dev_err(bq->dev, "%s\n", msg);
	if (bq->automode > 0)
		bq->automode = 0;
	bq2415x_set_mode(bq, BQ2415X_MODE_OFF);
	bq2415x_set_autotimer(bq, 0);
}
#endif
static void bq2451x_check_charge_status_by_charger(struct bq2415x_device *bq)
{
    int rc;
    if(bq == NULL)
    {
        pmu_log_info("TI chip uninitialized\n");
        return;
    }
    if(!is_usb_chg_exist())
    {
        bq->charge_done_flag = 0;
        return;
    }
    if(bq->charge_done_flag==1)
    {
        rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
        if(rc < 0)
        {
            pmu_log_info("disable charger failed\n");
            return;
        }
        pmu_log_debug("charge full, stop charege!\n");
       if(bq->soc_resume_charging)
       {
            bq->charge_done_flag = 0;
       }
    }
    else
    {
        rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_STATUS);
        if(rc < 0)
        {
            pmu_log_info("get hzmode failed\n");
            return;
        }
        else if(rc == 0)
        {
            if(!bq->charge_disable)
            {
                //if battery is not full, enable charge
                rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
                if(rc < 0)
                {
                    pmu_log_info("charge enable set failed\n");
                    return;
                }
             }
         }
         else
         {
              pmu_log_info("High impedance mode!\n");
              bq->charge_done_flag = 0;
         }
    }
}
/* to check whether charge is done by gasgauge */
/*solve the problem of recharge condition is not accurate*/
static void bq2451x_check_charge_status(struct bq2415x_device *bq)
{
	int battery_full = 0;
	int rc;
    int real_capacity = 0;
    int term_cur = 0;
    static int charge_full_count = 0;
    static int enter_recharge_flag = true;
	if((g_bq == NULL) || (g_battery_measure_by_bq27510_device == NULL))
	{
		pmu_log_debug("TI chip uninitialized\n");
		return;
	}
	//remove reduntant code.
	//check if charger is online
	if(!is_usb_chg_exist())
	{
		bq->charge_done_flag = 0;
        charge_full_count = 0;
        enter_recharge_flag = true;
		return;
	}

	//get battery full status from gasgauge
	battery_full = is_bq27510_battery_full(g_battery_measure_by_bq27510_device);
    real_capacity = bq27510_battery_capacity(g_battery_measure_by_bq27510_device);
    term_cur = -1*bq27510_battery_current(g_battery_measure_by_bq27510_device);
	if(battery_full)
	{
        if((TERM_CUR < term_cur) && (term_cur < 0))
        {
            charge_full_count++;
        }
        if(( g_battery_measure_by_bq27510_device->capacity == CAPACITY_FULL) && (real_capacity > CAPACITY_CLEAR_FULL) \
            && (enter_recharge_flag == true) && (CHARGE_FULL_COUNT <= charge_full_count))
		{
			rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
			if(rc < 0)
			{
				pmu_log_info("disable charger failed\n");
				return;
			}
			bq->charge_done_flag = 1;
			pmu_log_debug("charge full, stop charege!\n");
            enter_recharge_flag = false;
            charge_full_count = 0;
		}
        else if(real_capacity <= CAPACITY_CLEAR_FULL)
            {
                rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
                if(rc < 0){
                            pmu_log_info("charge enable set failed\n");
                            return;
                    }
                enter_recharge_flag = true;
                charge_full_count = 0;
            }
	}
	else
	{
        charge_full_count = 0;
        enter_recharge_flag = true;
		//check hzmode for running test
		rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_STATUS);
		if(rc < 0)
		{
			pmu_log_info("get hzmode failed\n");
			return;
		}
		else if(rc == 0)
		{
			if(!bq->charge_disable)
			{
				//if battery is not full, enable charge
				rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
				if(rc < 0)
				{
					pmu_log_info("charge enable set failed\n");
					return;
				}
			}
			/* capacity under clear threshold, clear charge done flag */
			if(g_battery_measure_by_bq27510_device->capacity < CAPACITY_CLEAR_FULL)
			{
				bq->charge_done_flag = 0;
			}
		}
		else
		{
			pmu_log_debug("High impedance mode!\n");
			bq->charge_done_flag = 0;
		}
	}
}

/* delayed work function for auto resetting chip timer */
static void bq2415x_timer_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 work.work);
	int ret;
	int error;
	int boost;
	int now_charge_status = -1;
	static int last_charge_status = -1;
	ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
	if (ret < 0) {
		pmu_log_info("Resetting timer failed %d\n",ret);
	}

	boost = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_STATUS);
	if (boost < 0) {
		pmu_log_info("Unknown boost error %d\n",boost);
	}

	error = bq2415x_exec_command(bq, BQ2415X_FAULT_STATUS);
	if (error < 0) {
		pmu_log_info("Unknown error %d\n",error);
	}

	if (boost) {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 3: /* Battery voltage too low */
			dev_err(bq->dev, "Battery voltage to low\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			pmu_log_info("Overvoltage protection (chip fried)\n");
			break;
		case 2: /* Overload */
			pmu_log_info("Overload\n");
			break;
		case 4: /* Battery overvoltage protection */
			pmu_log_info("Battery overvoltage protection\n");
			break;
		case 5: /* Thermal shutdown (too hot) */
			pmu_log_info("Thermal shutdown (too hot)\n");
			break;
		case 7: /* N/A */
			pmu_log_info("Unknown error\n");
			break;
		}
	} else {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 2: /* Sleep mode */
			dev_err(bq->dev, "Sleep mode\n");
			break;
		case 3: /* Poor input source */
			dev_err(bq->dev, "Poor input source\n");
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			break;
		case 7: /* No battery */
			dev_err(bq->dev, "No battery\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			pmu_log_info("Overvoltage protection (chip fried)\n");
			break;
		case 4: /* Battery overvoltage protection */
			pmu_log_info("Battery overvoltage protection\n");
			break;
		case 5: /* Thermal shutdown (too hot) */
			pmu_log_info("Thermal shutdown (too hot)\n");
			break;
		}
	}
	bq2415x_set_appropriate_jeita(bq);
    if(!bq->use_only_charge)
    {
       bq2451x_check_charge_status(bq);
    }
    else
    {
       bq2451x_check_charge_status_by_charger(bq);
    }

	now_charge_status = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
	if(bq->charge_done_flag)
	{
		now_charge_status = STATUS_FULL;
	}
	if(now_charge_status == STATUS_DISCHARGING)
	{
		bq->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	if(now_charge_status == STATUS_CHARGING)
	{
		bq->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	//update charge status so that charge icon change.
	if(now_charge_status != last_charge_status)
	{
		power_supply_changed(&bq->charger);
	}
	last_charge_status = now_charge_status;

	schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
}

static void bq2415x_iusb_work(struct work_struct *work)
{
	int rc = 0;
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 iusb_work);
	pmu_log_info("usb current is %d\n",bq->iusb_limit);
	rc = bq2415x_set_current_limit(bq,bq->iusb_limit);
	if(rc < 0)
	{
		pmu_log_info("current limit setting failed\n");
		return;
	}
	if(bq->iusb_limit == 0 ){
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
		if(rc < 0){
			pmu_log_info("disable charger failed\n");
	//remove redundant code
			return;
		}
    }else{
        /* enable charging otherwise high/cold temperature or high impedance */
        if((!bq->charge_disable)
            && (!bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_STATUS))){
            rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
            if(rc < 0){
                pmu_log_info("charge enable set failed\n");
                return;
            }
        }
    }
    /* disable stat pin for turn off the led */
    rc = bq2415x_exec_command(bq, BQ2415X_STAT_PIN_DISABLE);
    if(rc < 0)
    {
        pmu_log_info("disable pin failed\n");
        return;
    }
    msleep(DELAYED_TIME);
    power_supply_changed(&bq->charger);
}

/*===========================================
FUNCTION: increase_usb_ma_step
DESCRIPTION: to increase charge current step by step
IPNUT:	bq2415x_device *bq
RETURN:	a int value: 0 means sucessful
=============================================*/
static int increase_usb_ma_step(struct bq2415x_device *bq)
{
	int usb_ma;
	int rc = 0;
	usb_ma = bq2415x_get_charge_current(bq);
	if(usb_ma < 0){
		rc = -EINVAL;
		return rc;
	}
	else if(usb_ma >= MAX_CHARGE_CURRENT)
	{
		return rc;
	}

	rc = bq2415x_set_charge_current(bq, usb_ma + CHARGE_CURRENT_STEP);
	if(rc < 0)
	{
		pmu_log_info("current limit setting failed\n");
		return rc;
	}

	return rc;
}
/*===========================================
FUNCTION: bq2415x_get_charge_type
DESCRIPTION: get charge type
IPNUT:bq2415x_device *bq
RETURN:NONE or FAST
=============================================*/
int bq2415x_get_charge_type(struct bq2415x_device *bq)
{
    int rc=POWER_SUPPLY_CHARGE_TYPE_NONE;
    rc = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
    if (rc<0)
    {
        pmu_log_err("failed to read status");
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }
    else if (rc==1)
    {
        return POWER_SUPPLY_CHARGE_TYPE_FAST;
    }
    else
    {
        return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }
}
/*===========================================
FUNCTION: bq24152_charger_present
DESCRIPTION: get charge present
IPNUT:NA
RETURN:1 or 0
=============================================*/
static bool bq24152_charger_present(void)
{
    if(is_bq24152_in_boost_mode()){
        return false;
    }else{
        return (1 == is_usb_chg_exist());
    }
}
/*===========================================
FUNCTION: bq24152_get_battery_health
DESCRIPTION: get battery health
IPNUT:NA
RETURN:good or overheat
=============================================*/
static int bq24152_get_battery_health(void)
{
    int temp = 0;
    temp = huawei_charger_get_battery_temperature();
    if(temp >= BATTERY_TEMP_MAX)
    {
        return POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    else
    {
        return POWER_SUPPLY_HEALTH_GOOD;
    }
}

/*===========================================
FUNCTION: bq2415x_usb_low_power_work
DESCRIPTION: to ajust charge current for low power(poor input) charger
IPNUT:	bq2415x_device *bq
RETURN:	N/A
=============================================*/
static void bq2415x_usb_low_power_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 lower_power_charger_work.work);
	int usb_ma, usb_present, rc ;
	int vbatt = 0;
	int chg_status = 0, chg_fault = 0;
	u8 reg_val = 0;
	union power_supply_propval val = {0};



	/* exit when usb absent*/
	usb_present = is_usb_chg_exist();
	if(!usb_present)
	{
		return;
	}

	/* set usb_init_ma as 500 */
	rc = bq2415x_set_current_limit(bq, CURRENT_LIMIT_500MA);
	if(rc < 0)
	{
		pmu_log_err("current limit setting failed\n");
		return;
	}

	/* set default charge current*/
	usb_ma = USB_CHARGE_CURRENT;
	rc = bq2415x_set_charge_current(bq, usb_ma);
	if(rc < 0)
	{
		pr_err("charge current setting failed\n");
		return;
	}
	spin_lock(&current_config_changed_lock);
	current_config_changed = true;
	spin_unlock(&current_config_changed_lock);
	g_current_config = usb_ma;
	/* enable charging otherwise high temperature or high impedance */
	if((!bq->charge_disable) 
		&& (!bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_STATUS)))
	{
		rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
		if(rc < 0){
			pr_err("bq2415x enable charging failed\n");
			return;
		}
	}

	/* Get battery voltage in mV*/
    if(bq->use_only_charge)
    {
        vbatt = huawei_charger_get_battery_voltage_now();
        vbatt = vbatt /1000;
    }
    else
    {
        if (bq->ti_bms_psy){
        rc = bq->ti_bms_psy->get_property(bq->ti_bms_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
        }
        vbatt = val.intval /1000;
    }

	/* if battery voltage low, use default current to charge */
	if(vbatt < BATTERY_VOL_THRESHOLD && !poor_input_enable)
	{
		pmu_log_info("battery voltage low, set input current as 500mA\n");
		schedule_delayed_work(&bq->lower_power_charger_work, HYSTERTSIS_TIME * HZ);
		return;
	}

	/* not limit input current */
	if(!poor_input_enable)
		bq2415x_set_current_limit(bq, CURRENT_LIMIT_1800MA);

	/*G760 va board Reg0 charge status update a little slowly, need at least 2 seconds */
	if(!poor_input_enable){
		msleep(2500);
	}

	/* increase charge current by step */
	while(usb_ma < MAX_CHARGE_CURRENT && usb_present)
	{
		if(poor_input_enable){
			pmu_log_info("poor input charger enable!\n");
			bq2415x_set_current_limit(bq, CURRENT_LIMIT_500MA);
			poor_input_enable = 0;
			break;
		}
		/* Increase charge current by step 100mA, from 550mA to 1250mA,*/
		increase_usb_ma_step(bq);
        msleep(200);

		/* check vin_min status */
        if(qpnp_lbc_is_in_vin_min_loop()){
            pmu_log_info("charger is in vin_min loop! final charge_current = %d\n", usb_ma);
            bq2415x_set_charge_current(bq, USB_CHARGE_CURRENT);
            msleep(200);
			break;
		}
		/* Read Reg0 bit0 to bit3(fault status) and bit 4 to bit5(chg_sts) */
		reg_val = bq2415x_i2c_read(bq, BQ2415X_REG_STATUS);
		chg_status = (reg_val & BQ24152_REG_0_STAT_MASK) >> 4;
		chg_fault = reg_val & BQ24152_REG_0_FAULT_MASK;

		pmu_log_info("reg_val = %x, chg_status = %d, chg_fault = %d, charge_current = %d\n",
				reg_val, chg_status, chg_fault, usb_ma);

		if((BQ24152_CHG_STAT_FAULT == chg_status)
			|| (POOR_INPUT_FAULT_STATUS == chg_fault)){
			pmu_log_info("find chg state fault or poor input fault!\n");
			poor_input_enable = 1;
			break;
		}

		usb_present = is_usb_chg_exist();
		/* Get the current limit */
		usb_ma = bq2415x_get_charge_current(bq);
	}

	/*Set output current for DCP charger*/
	rc = bq2415x_set_charge_current(bq, min(usb_ma, hot_current));
	if(rc < 0){
		pr_err("set charge current failed\n");
		return;
	}
    /* disable stat pin for turn off the led */
    rc = bq2415x_exec_command(bq, BQ2415X_STAT_PIN_DISABLE);
    if(rc < 0)
    {
         pmu_log_info("disable pin failed\n");
         return;
    }
	spin_lock(&current_config_changed_lock);
	current_config_changed = true;
	spin_unlock(&current_config_changed_lock);
	g_current_config = min(usb_ma, hot_current);
}

/**** power supply interface code ****/

static enum power_supply_property bq2415x_power_supply_props[] = {
	/* TODO: maybe add more power supply properties */
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CHARGE_TYPE,
    POWER_SUPPLY_PROP_RESUME_CHARGING,
    POWER_SUPPLY_PROP_RUNNING_TEST_STATUS,
    POWER_SUPPLY_PROP_FACTORY_DIAG,
    POWER_SUPPLY_PROP_HOT_IBAT_LIMIT,
};
static int bq2415x_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	int ret = 0;
    int result=0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		result = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
		if (result < 0)
		{
            pmu_log_err("bq24152 get charge status error\n");
			return result;
		}
		else if (result == 0) /* Ready */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (result == 1) /* Charge in progress */
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (result == 2) /* Charge done */
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

        if(bq->use_only_charge)
        {
            if(bq->qcom_bms_psy)
            {
               pmu_log_debug("get real soc :%d\n",get_true_bms_soc());
               if((BAT_CAPACITY_FULL == get_true_bms_soc())&&bq24152_charger_present())
               {
                   bq->charge_done_flag=1;
               }
               if((bq->charge_done_flag)||((get_ui_bms_soc() == BAT_CAPACITY_FULL)&&bq24152_charger_present()))
               {
                   val->intval = POWER_SUPPLY_STATUS_FULL;
               }
            }
        }
        else
        {
        if(bq->charge_done_flag \
            || ((g_battery_measure_by_bq27510_device == NULL ? 0 \
                : g_battery_measure_by_bq27510_device->capacity == CAPACITY_FULL) \
            && is_usb_chg_exist() && !is_bq24152_in_boost_mode()))
		{
			val->intval = POWER_SUPPLY_STATUS_FULL;
		}
       }
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_TEMP:
        if(!bq->use_only_charge)
        {
          if (bq->ti_bms_psy)
          {
			ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_TEMP,val);
          }
          else
          {
             val->intval = TEMP_DEFAULT;
               pmu_log_info("get error temp :%d\n",val->intval);
          }
        }
        else
        {
			val->intval = huawei_charger_get_battery_temperature();
        }
		 break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        if(!bq->use_only_charge)
        {
             if (bq->ti_bms_psy)
             {
                ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,val);
             }
             else
             {
                val->intval = VOLTAGE_NOW_DEFULT;
                pmu_log_info("get error voltage :%d\n",val->intval);
             }
        }
        else
        {
              val->intval = huawei_charger_get_battery_voltage_now();
        }
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
        if(!bq->use_only_charge)
        {
            if (bq->ti_bms_psy){
             ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_CURRENT_NOW,val);
            }
           else
           {
               val->intval = CURRENT_NOW_DEFULT;
               pmu_log_info("get error current :%d\n",val->intval);
           }
        }
        else
        {
           if (bq->qcom_bms_psy)
           {
            ret = bq->qcom_bms_psy->get_property(bq->qcom_bms_psy,POWER_SUPPLY_PROP_CURRENT_NOW,val);
           }
           else
           {
             val->intval = CURRENT_NOW_DEFULT;
               pmu_log_info("get error current :%d\n",val->intval);
           }
        }
		break;
	case POWER_SUPPLY_PROP_HEALTH:
        if(!bq->use_only_charge)
        {
            if (bq->ti_bms_psy)
               ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_HEALTH,val);
		//if ti fuel gauge is not ready,val return UNKNOWN.
            else
            val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
        }
        else
        {
           val->intval = bq24152_get_battery_health();

        }
		break;
	case POWER_SUPPLY_PROP_PRESENT:
        val->intval = huawei_charger_battery_is_exist();
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval= !(bq->charging_disabled);
		
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
        if(!bq->use_only_charge)
        {
            if(bq->ti_bms_psy){
                ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_CAPACITY,val);
                }
            else
            {
                val->intval=CAPACITY_DEFAULT;
                pmu_log_info("get error capacity :%d\n",val->intval);
            }
        }
        else
        {
             if(bq->qcom_bms_psy)
             {
                 ret = bq->qcom_bms_psy->get_property(bq->qcom_bms_psy,POWER_SUPPLY_PROP_CAPACITY,val);
             }
             else
             {
                 val->intval=CAPACITY_DEFAULT;
                 pmu_log_info("get error capacity :%d\n",val->intval);
             }
        }
        if(bq->charge_done_flag)
        {
            val->intval = BAT_CAPACITY_FULL;
        }
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        if(!bq->use_only_charge)
        {
           if(bq->ti_bms_psy)
           {
               ret = bq->ti_bms_psy->get_property(bq->ti_bms_psy,POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,val);
           }
           else
           {
               val->intval = CHARGE_FULL_DESIGN_DEFULT;
               pmu_log_info("get error full design:%d\n",val->intval);
           }
        }
        else
        {
           if(bq->qcom_bms_psy)
           {
                ret = bq->qcom_bms_psy->get_property(bq->qcom_bms_psy,POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,val);
           }
           else
           {
               val->intval = CHARGE_FULL_DESIGN_DEFULT;
               pmu_log_info("get error full design:%d\n",val->intval);
           }
        }
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
    case POWER_SUPPLY_PROP_CHARGE_TYPE:
        val->intval = bq2415x_get_charge_type(bq);
        break;
    case POWER_SUPPLY_PROP_RESUME_CHARGING:
         val->intval = bq->soc_resume_charging;
     break;
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        val->intval = hot_current;
    break;
    case POWER_SUPPLY_PROP_RUNNING_TEST_STATUS:
        val->intval = get_running_test_status();
    break;
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
        val->intval = !(factory_diag_flag);
    break;
	default:
        ret = -EINVAL;
        break;
	}
	return ret;
}

static int bq2415x_power_supply_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
								charger);
	int rc = 0;

	switch (psp) {

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        bq->charging_disabled = !(val->intval);
        pmu_log_info("set charging_enabled value is %d\n",val->intval);
        set_running_test_flag(bq->charging_disabled);
        if(!bq->charging_disabled)
			rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_DISABLE);
		else
			rc = bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_ENABLE);
		user_ctl_status = val->intval;
		break;
    case POWER_SUPPLY_PROP_RESUME_CHARGING:
        bq->soc_resume_charging = val->intval;
        break;
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
        bq2415x_set_in_thermal(val->intval);
        break;
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
        bq2415x_factory_diag(val->intval);
        break;
    case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_WARM_TEMP:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
         pmu_log_info("bq24152 no need to set this prop: %d\n",psp);
         break;
	default:
        rc = -EINVAL;
        break;
	}
	return rc;
}

static int bq2415x_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_WARM_TEMP:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
    case POWER_SUPPLY_PROP_RESUME_CHARGING:
    case POWER_SUPPLY_PROP_FACTORY_DIAG:
    case POWER_SUPPLY_PROP_HOT_IBAT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static void set_charging_by_current_limit(struct bq2415x_device *bq)
{
         if(bq->iusb_limit <= USB_CURRENT_LIMIT){
            schedule_work(&bq->iusb_work);
    }else{
            schedule_delayed_work(&bq->lower_power_charger_work, 0);
    }
}
#define OTG_RETRY_ENABLE_DELAY_MS	10
#define OTG_RETRY_ERR_COUNT			3
extern bool get_usb_id_status(void);
static void check_otg_id_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 otg_id_check_work.work);
	static unsigned int otg_err_count = 0;
	if (get_usb_id_status() == true)
	{
		otg_err_count = 0;
		return;
	}

	if (otg_err_count >= OTG_RETRY_ERR_COUNT)
	{
		pr_info("retry time out, open otg failed\n");
		otg_err_count = 0;
		return;
	}

	otg_err_count++;
	pr_info("retry to open otg, count is %d\n", otg_err_count);
	bq24152_control_otg(bq, true);
}
static void bq2415x_external_power_changed(struct power_supply *psy)
{
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
								charger);
	unsigned long flags;
    int current_ma = 0, charger_present = 0;
	int usb_present = 0;
    union power_supply_propval ret = {0,};
	spin_lock_irqsave(&bq->ibat_change_lock, flags);
    if(!bq->use_only_charge)
    {
       if (!bq->ti_bms_psy)
          bq->ti_bms_psy = power_supply_get_by_name("ti-bms");
    }
    else
    {
      if (!bq->qcom_bms_psy)
        bq->qcom_bms_psy = power_supply_get_by_name("bms");
    }

	spin_unlock_irqrestore(&bq->ibat_change_lock, flags);
	usb_present = is_usb_chg_exist();
	if ((usb_present == 0))
	{
		schedule_delayed_work(&bq->otg_id_check_work,
				msecs_to_jiffies(OTG_RETRY_ENABLE_DELAY_MS));
	}

    charger_present = bq24152_charger_present();
    if(charger_present)
    {
         bq->usb_psy->get_property(bq->usb_psy,
                                  POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
        current_ma = ret.intval / 1000;

        current_ma = min(current_ma, hot_current);
        if(current_ma != bq->iusb_limit)
        {
            mutex_lock(&bq->current_change_lock);
            bq->iusb_limit = current_ma;
            mutex_unlock(&bq->current_change_lock);
            pmu_log_info("get iusb_limit :%d\n",bq->iusb_limit);
            set_charging_by_current_limit(bq);
        }
    }
}
static char *bq2415x_supplied_to[] = {
	"ti-bms",
     "bms"
};
static int bq2415x_power_supply_init(struct bq2415x_device *bq)
{
	int ret;
	int chip;
	char revstr[8];

	bq->charger.name = "battery";
	bq->charger.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->charger.properties = bq2415x_power_supply_props;
	bq->charger.num_properties = ARRAY_SIZE(bq2415x_power_supply_props);
	bq->charger.get_property = bq2415x_power_supply_get_property;
	bq->charger.set_property = bq2415x_power_supply_set_property;
	bq->charger.property_is_writeable = bq2415x_property_is_writeable;
	bq->charger.external_power_changed = bq2415x_external_power_changed;
	bq->charger.supplied_to = bq2415x_supplied_to;
	bq->charger.num_supplicants =ARRAY_SIZE(bq2415x_supplied_to);

	ret = bq2415x_detect_chip(bq);
	if (ret < 0)
		chip = BQUNKNOWN;
	else
		chip = ret;

	ret = bq2415x_detect_revision(bq);
	if (ret < 0)
		strncpy(revstr, "unknown",sizeof revstr);
	else
		snprintf(revstr,sizeof revstr, "1.%d", ret);

	bq->model = kasprintf(GFP_KERNEL,
				"chip %s, revision %s, vender code %.3d",
				bq2415x_chip_name[chip], revstr,
				bq2415x_get_vender_code(bq));
	if (!bq->model) {
		dev_err(bq->dev, "failed to allocate model name\n");
		return -ENOMEM;
	}

	ret = power_supply_register(bq->dev, &bq->charger);
	if (ret) {
		kfree(bq->model);
		return ret;
	}

	return 0;
}

static void bq2415x_power_supply_exit(struct bq2415x_device *bq)
{
	bq->autotimer = 0;
	if (bq->automode > 0)
		bq->automode = 0;
	cancel_delayed_work_sync(&bq->work);
	power_supply_unregister(&bq->charger);
	kfree(bq->model);
}

/**** additional sysfs entries for power supply interface ****/

/* show *_status entries */
static ssize_t bq2415x_sysfs_show_status(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						charger);
	enum bq2415x_command command;
	int ret;

	if (strcmp(attr->attr.name, "otg_status") == 0)
		command = BQ2415X_OTG_STATUS;
	else if (strcmp(attr->attr.name, "charge_status") == 0)
		command = BQ2415X_CHARGE_STATUS;
	else if (strcmp(attr->attr.name, "boost_status") == 0)
		command = BQ2415X_BOOST_STATUS;
	else if (strcmp(attr->attr.name, "fault_status") == 0)
		command = BQ2415X_FAULT_STATUS;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE,"%d\n", ret);
}

/*
 * set timer entry:
 *    auto - enable auto mode
 *    off - disable auto mode
 *    (other values) - reset chip timer
 */
static ssize_t bq2415x_sysfs_set_timer(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						charger);
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0)
		bq2415x_set_autotimer(bq, 1);
	else if (strncmp(buf, "off", 3) == 0)
		bq2415x_set_autotimer(bq, 0);
	else
		ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);

	if (ret < 0)
		return ret;
	return count;
}

/* show timer entry (auto or off) */
static ssize_t bq2415x_sysfs_show_timer(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);

	if (bq->timer_error)
		return snprintf(buf, PAGE_SIZE,"%s\n", bq->timer_error);

	if (bq->autotimer)
		return snprintf(buf, PAGE_SIZE,"auto\n");
	return snprintf(buf, PAGE_SIZE,"off\n");
}

/*
 * set mode entry:
 *    auto - if automode is supported, enable it and set mode to reported
 *    none - disable charger and boost mode
 *    host - charging mode for host/hub chargers (current limit 500mA)
 *    dedicated - charging mode for dedicated chargers (unlimited current limit)
 *    boost - disable charger and enable boost mode
 */
static ssize_t bq2415x_sysfs_set_mode(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	enum bq2415x_mode mode;
	int ret = 0;

	if (strncmp(buf, "auto", 4) == 0) {
		if (bq->automode < 0)
			return -ENOSYS;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else if (strncmp(buf, "off", 3) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_OFF;
	} else if (strncmp(buf, "none", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_NONE;
	} else if (strncmp(buf, "host", 4) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_HOST_CHARGER;
	} else if (strncmp(buf, "dedicated", 9) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_DEDICATED_CHARGER;
	} else if (strncmp(buf, "boost", 5) == 0) {
		if (bq->automode > 0)
			bq->automode = 0;
		mode = BQ2415X_MODE_BOOST;
	} else if (strncmp(buf, "reset", 5) == 0) {
		bq2415x_reset_chip(bq);
		bq2415x_set_defaults(bq);
		if (bq->automode <= 0)
			return count;
		bq->automode = 1;
		mode = bq->reported_mode;
	} else {
		return -EINVAL;
	}

	ret = bq2415x_set_mode(bq, mode);
	if (ret < 0)
		return ret;
	return count;
}

/* show mode entry (auto, none, host, dedicated or boost) */
static ssize_t bq2415x_sysfs_show_mode(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						charger);
	ssize_t ret = 0;

	if (bq->automode > 0)
		ret += snprintf(buf+ret, PAGE_SIZE, "auto (");

	switch (bq->mode) {
	case BQ2415X_MODE_OFF:
		ret += snprintf(buf+ret, PAGE_SIZE, "off");
		break;
	case BQ2415X_MODE_NONE:
		ret += snprintf(buf+ret, PAGE_SIZE, "none");
		break;
	case BQ2415X_MODE_HOST_CHARGER:
		ret += snprintf(buf+ret, PAGE_SIZE, "host");
		break;
	case BQ2415X_MODE_DEDICATED_CHARGER:
		ret += snprintf(buf+ret, PAGE_SIZE, "dedicated");
		break;
	case BQ2415X_MODE_BOOST:
		ret += snprintf(buf+ret, PAGE_SIZE, "boost");
		break;
	}

	if (bq->automode > 0)
		ret += snprintf(buf+ret, PAGE_SIZE, ")");

	ret += snprintf(buf+ret, PAGE_SIZE,"\n");
	return ret;
}
/* show reported_mode entry (none, host, dedicated or boost) */
static ssize_t bq2415x_sysfs_show_reported_mode(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);

	if (bq->automode < 0)
		return -EINVAL;

	switch (bq->reported_mode) {
	case BQ2415X_MODE_OFF:
		return snprintf(buf, PAGE_SIZE,"off\n");
	case BQ2415X_MODE_NONE:
		return snprintf(buf, PAGE_SIZE,"none\n");
	case BQ2415X_MODE_HOST_CHARGER:
		return snprintf(buf, PAGE_SIZE,"host\n");
	case BQ2415X_MODE_DEDICATED_CHARGER:
		return snprintf(buf, PAGE_SIZE,"dedicated\n");
	case BQ2415X_MODE_BOOST:
		return snprintf(buf, PAGE_SIZE,"boost\n");
	}

	return -EINVAL;
}

	//remove redundant code
/* print value of chip register, format: 'register=value' */
static ssize_t bq2415x_sysfs_print_reg(struct bq2415x_device *bq,
				       u8 reg,
				       char *buf)
{
	int ret = bq2415x_i2c_read(bq, reg);

	if (ret < 0)
		return snprintf(buf, PAGE_SIZE, "%#.2x=error %d\n", reg, ret);	
	return snprintf(buf, PAGE_SIZE, "%#.2x ", ret);
}

/* show all raw values of chip register, format per line: 'register=value' */
static ssize_t bq2415x_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	ssize_t ret = 0;

	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_STATUS, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CONTROL, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VOLTAGE, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VENDER, buf+ret);
	ret += bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CURRENT, buf+ret);
	ret += snprintf(buf+ret, PAGE_SIZE,"\n");
	return ret;
}

/* set current and voltage limit entries (in mA or mV) */
static ssize_t bq2415x_sysfs_set_limit(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = bq2415x_set_current_limit(bq, val);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = bq2415x_set_weak_battery_voltage(bq, val);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = bq2415x_set_battery_regulation_voltage(bq, val);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = bq2415x_set_charge_current(bq, val);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = bq2415x_set_termination_current(bq, val);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return count;
}

/* show current and voltage limit entries (in mA or mV) */
static ssize_t bq2415x_sysfs_show_limit(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	int ret;

	if (strcmp(attr->attr.name, "current_limit") == 0)
		ret = bq2415x_get_current_limit(bq);
	else if (strcmp(attr->attr.name, "weak_battery_voltage") == 0)
		ret = bq2415x_get_weak_battery_voltage(bq);
	else if (strcmp(attr->attr.name, "battery_regulation_voltage") == 0)
		ret = bq2415x_get_battery_regulation_voltage(bq);
	else if (strcmp(attr->attr.name, "charge_current") == 0)
		ret = bq2415x_get_charge_current(bq);
	else if (strcmp(attr->attr.name, "termination_current") == 0)
		ret = bq2415x_get_termination_current(bq);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE,"%d\n", ret);
}

/* set *_enable entries */
static ssize_t bq2415x_sysfs_set_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	enum bq2415x_command command;
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = val ? BQ2415X_CHARGE_TERMINATION_ENABLE :
			BQ2415X_CHARGE_TERMINATION_DISABLE;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = val ? BQ2415X_HIGH_IMPEDANCE_ENABLE :
			BQ2415X_HIGH_IMPEDANCE_DISABLE;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = val ? BQ2415X_OTG_PIN_ENABLE :
			BQ2415X_OTG_PIN_DISABLE;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = val ? BQ2415X_STAT_PIN_ENABLE :
			BQ2415X_STAT_PIN_DISABLE;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return count;
}

/* show *_enable entries */
static ssize_t bq2415x_sysfs_show_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2415x_device *bq = container_of(psy, struct bq2415x_device,
						 charger);
	enum bq2415x_command command;
	int ret;

	if (strcmp(attr->attr.name, "charge_termination_enable") == 0)
		command = BQ2415X_CHARGE_TERMINATION_STATUS;
	else if (strcmp(attr->attr.name, "high_impedance_enable") == 0)
		command = BQ2415X_HIGH_IMPEDANCE_STATUS;
	else if (strcmp(attr->attr.name, "otg_pin_enable") == 0)
		command = BQ2415X_OTG_PIN_STATUS;
	else if (strcmp(attr->attr.name, "stat_pin_enable") == 0)
		command = BQ2415X_STAT_PIN_STATUS;
	else
		return -EINVAL;

	ret = bq2415x_exec_command(bq, command);
	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static DEVICE_ATTR(current_limit, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(weak_battery_voltage, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(battery_regulation_voltage, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_limit, bq2415x_sysfs_set_limit);

static DEVICE_ATTR(charge_termination_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(high_impedance_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(otg_pin_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);
static DEVICE_ATTR(stat_pin_enable, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_enable, bq2415x_sysfs_set_enable);

static DEVICE_ATTR(reported_mode, S_IRUGO,
		bq2415x_sysfs_show_reported_mode, NULL);
static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_mode, bq2415x_sysfs_set_mode);
static DEVICE_ATTR(timer, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_timer, bq2415x_sysfs_set_timer);

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bq2415x_sysfs_show_registers, NULL);

static DEVICE_ATTR(otg_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);
static DEVICE_ATTR(boost_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);
static DEVICE_ATTR(fault_status, S_IRUGO, bq2415x_sysfs_show_status, NULL);

static struct attribute *bq2415x_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use power supply class props.
	 */
	&dev_attr_current_limit.attr,
	&dev_attr_weak_battery_voltage.attr,
	&dev_attr_battery_regulation_voltage.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_termination_current.attr,

	&dev_attr_charge_termination_enable.attr,
	&dev_attr_high_impedance_enable.attr,
	&dev_attr_otg_pin_enable.attr,
	&dev_attr_stat_pin_enable.attr,

	&dev_attr_reported_mode.attr,
	&dev_attr_mode.attr,
	&dev_attr_timer.attr,

	&dev_attr_registers.attr,

	&dev_attr_otg_status.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_boost_status.attr,
	&dev_attr_fault_status.attr,
	NULL,
};

static int jeita_param_init_check(jeita_spec *batt_param)
{
	if(!batt_param)
	{
        pmu_log_info("jeita param init encounter bad address\n");
		return -EFAULT;
	}

	if(batt_param->hot.t_high == -1)
	{
		pmu_log_info("waiting for linear charger load battery data ...\n");
		return -EINVAL;
	}

	if(batt_param->hot.t_high == INT_MAX)
	{
		int i = 0;
		struct jeita_entry *loop = (struct jeita_entry *)batt_param;
		
		for(;i < ZONE_MAX;i++)
		{
			pmu_log_info("jeita [%d %d %d %d]\n",loop->t_low,loop->t_high, \
				loop->i_max,loop->v_max);
			loop++;
		}
		batt_param->hot.t_high = INT_MAX-1;
	}

	return 0;
}

static int jeita_select_zone(jeita_entry **r_entry,int temp,jeita_spec *batt_param)
{
	int i = 0;
	jeita_entry *entry = (struct jeita_entry *)batt_param;

	if(!entry || !r_entry)
	{
        pmu_log_info("jeita select zone encounter bad address\n");
		return -EFAULT;
	}

	for(i = 0; i < ZONE_MAX; i++)
	{
		if(is_between(entry[i].t_low,entry[i].t_high,temp))
			break;
	}

	if(i < ZONE_MAX)
	{
		*r_entry = entry + i;
		pmu_log_debug("select param [%d %d %d %d %d]\n",(*r_entry)->t_low, \
			(*r_entry)->t_high,(*r_entry)->i_max,(*r_entry)->v_max,(*r_entry)->selected);
	}
	else
	{
		pr_err("batt jeita param error\n");
		return -EINVAL;
	}

	return 0;
}
static int jeita_find_running_zone(jeita_entry **r_entry,jeita_spec *batt_param)
{
	int i = 0;
	jeita_entry *entry = (struct jeita_entry *)batt_param;

	if(!entry || !r_entry)
	{
        pmu_log_info("jeita find running zone encounter bad address\n");
		return -EFAULT;
	}

	for(i = 0; i < ZONE_MAX; i++)
	{
		if(entry[i].selected == 1)
			break;
	}

	if(i < ZONE_MAX)
	{
		*r_entry = entry + i;
	}
	else
	{
		*r_entry = (struct jeita_entry *)batt_param + COOL;
		(*r_entry)->selected = 1;
	}
	pmu_log_debug("running zone [%d %d %d %d]\n",(*r_entry)->t_low, \
		(*r_entry)->t_high,(*r_entry)->i_max,(*r_entry)->v_max);

	return 0;
}

static int jeita_check_status_migrate(jeita_entry **r_selected_zone, jeita_entry **r_running_zone, jeita_spec *batt_param)
{
	jeita_entry *running_zone = NULL;
	jeita_entry *selected_zone = NULL;
	int temp = 0;

	if(!batt_param || !r_selected_zone || !r_running_zone)
	{
		return -EFAULT;
	}

	jeita_param_init_check(batt_param);
	jeita_find_running_zone(&running_zone,batt_param);
	*r_running_zone = running_zone;
    if(!g_bq->use_only_charge)
    {
	   if(g_battery_measure_by_bq27510_device)
	   {
		    union power_supply_propval val;
		    g_battery_measure_by_bq27510_device->ti_bms_psy.get_property( \
			    &g_battery_measure_by_bq27510_device->ti_bms_psy,POWER_SUPPLY_PROP_TEMP,&val);
		    temp = val.intval;
	   }
	   else
	   {
		    temp = TEMP_DEFAULT;
       }
    }
    else
    {
       temp = huawei_charger_get_battery_temperature();
    }
	pmu_log_debug("current temp is [%d]\n",temp);

	jeita_select_zone(&selected_zone,temp,batt_param);
	*r_selected_zone = selected_zone;

	if(selected_zone < running_zone)
	{
		if(selected_zone == running_zone->last_zone)
		{
			if(temp < running_zone->t_low - TEMP_HYSTERESIS)
			{
				return STATUS_MIGRATE;
			}
			else
			{
				return STATUS_KEEP;
			}
		}
		else
		{
			return STATUS_MIGRATE;
		}
	}
	else if(selected_zone > running_zone)
	{
		return STATUS_MIGRATE;
	}
	else
	{
		return STATUS_KEEP;
	}
}

static void bq2415x_set_appropriate_jeita(struct bq2415x_device *bq)
{
	int ret = 0;
	bool changed = false;
	jeita_entry *running_zone = NULL;
	jeita_entry *selected_zone = NULL;

	spin_lock(&current_config_changed_lock);
	changed = current_config_changed;
	current_config_changed = false;
	spin_unlock(&current_config_changed_lock);
	
	if(jeita_check_status_migrate(&selected_zone, &running_zone, &jeita_batt_param) \
		== STATUS_MIGRATE || changed)
	{
		pmu_log_debug("need jeita adjust\n");

		/* cold or hot, stop charging directly,ignore IUSB worker, LOW_POWER worker current setting */
        if(NULL == selected_zone)
        {
            pmu_log_info("selected_zone is NULL!\n");
            return;
        }
		if(!selected_zone->i_max)
		{
			ret = bq2415x_exec_command(bq,BQ2415X_CHARGER_DISABLE);
			if(ret < 0)
			{
				pr_err("bq2415x enable charging failed\n");
				return;
			}
			bq->charge_disable = true;
			pmu_log_debug("temperature is hot or cold,stop charging\n");

			running_zone->selected = 0;
			selected_zone->selected = 1;
			selected_zone->last_zone = running_zone;
			return;
		}

		if(user_ctl_status)
		{
			ret = bq2415x_set_charge_current(bq, \
				min(min(selected_zone->i_max,hot_current),g_current_config));
			if(ret < 0)
				pr_err("set charge current failed\n");

			ret = bq2415x_set_battery_regulation_voltage(bq,selected_zone->v_max);
			if(ret < 0)
				pr_err("set battery regulation voltage failed\n");

			if(!bq->charge_done_flag)
			{
				ret = bq2415x_exec_command(bq,BQ2415X_CHARGER_ENABLE);
				if(ret < 0)
					pr_err("bq2415x enable charging failed\n");

			}

			bq->charge_disable = false;
			running_zone->selected = 0;
			selected_zone->selected = 1;
			selected_zone->last_zone = running_zone;
			pmu_log_debug("adjust current %d voltage %d\n", \
				min(min(selected_zone->i_max,hot_current),g_current_config), \
				selected_zone->v_max);
		}
	}
}
static const struct attribute_group bq2415x_sysfs_attr_group = {
	.name = "ti-charger-prop",
	.attrs = bq2415x_sysfs_attributes,
};

static int bq2415x_sysfs_init(struct bq2415x_device *bq)
{
	return sysfs_create_group(&bq->charger.dev->kobj,
			&bq2415x_sysfs_attr_group);
}

static void bq2415x_sysfs_exit(struct bq2415x_device *bq)
{
	sysfs_remove_group(&bq->charger.dev->kobj, &bq2415x_sysfs_attr_group);
}

/* Add charger driver interfaces for bq24152. */
static int bq2415x_set_runningtest(int val)
{
    if(!g_bq){
        pmu_log_info("cd_debug bq_device is null, do nothing\n");
        return -1;
    }

    return 0;
}

static int bq2415x_enable_charge(int val)
{
    int rc = 0;
    if(!g_bq){
        pmu_log_info("cd_debug bq_device is null, do nothing\n");
        return -1;
    }
    g_bq->charging_disabled = !(val);
    pmu_log_info("set hw_power charging_enabled value is %d\n",val);
    if(!g_bq->charging_disabled)
	    rc = bq2415x_exec_command(g_bq, BQ2415X_HIGH_IMPEDANCE_DISABLE);
    else
        rc = bq2415x_exec_command(g_bq, BQ2415X_HIGH_IMPEDANCE_ENABLE);
    user_ctl_status = val;
    return 0;
}
static int bq2415x_factory_diag(int val)
{
    union power_supply_propval val_factory_diag = {0,};
    if(!g_bq){
        pmu_log_info("cd_debug bq_device is null, do nothing\n");
        return -1;
    }

    factory_diag_flag = !val;
    /* if set discharging when PT, set 90mA, as lbc have no BATFET*/
    if (factory_diag_flag) {
        g_bq->usb_psy->get_property(g_bq->usb_psy,
               POWER_SUPPLY_PROP_CURRENT_MAX, &val_factory_diag);
        factory_diag_last_current_ma = val_factory_diag.intval;
        val_factory_diag.intval = 90;
        g_bq->usb_psy->set_property(g_bq->usb_psy,
        POWER_SUPPLY_PROP_CURRENT_MAX, &val_factory_diag);
        pmu_log_info("set factory diag to %d\n",val_factory_diag.intval);
    } else {
        if (factory_diag_last_current_ma) {
            val_factory_diag.intval = factory_diag_last_current_ma;
            g_bq->usb_psy->set_property(g_bq->usb_psy,
                  POWER_SUPPLY_PROP_CURRENT_MAX, &val_factory_diag);
            pmu_log_info("set factory diag to %d\n",val_factory_diag.intval);

        }
        factory_diag_last_current_ma = 0;
    }
	bq2415x_external_power_changed(&g_bq->charger);

    return 0;
}

static int bq2415x_set_in_thermal(int val)
{
    if(!g_bq){
        pmu_log_info("cd_debugbq_device is null, do nothing\n");
        return -1;
    }

    if (val) {
        hot_current = val;
    } else {
        hot_current = HOT_MAX_CURRENT;
    }
    pmu_log_info("bq24152 set thermal:%d\n",hot_current);
    bq2415x_external_power_changed(&g_bq->charger);
    return 0;
}

static int bq2415x_set_usb_current(int val)
{
    if(!g_bq){
        pmu_log_info("bq_device is null, do nothing\n");
        return -1;
    }

    return 0;
}
static int bq2415x_get_chargelog(char *reg_value)
{

    if(!g_bq){
        pmu_log_info("cd_debug bq_device is null, do nothing\n");
        return -1;
    }

    memset(reg_value, 0, CHARGELOG_SIZE);

    strncat(reg_value, "\n", 1);

    return 0;
}
static int bq2415x_get_battery_status(void)
{
    int result = 0;
    if(!g_bq){
        pmu_log_info("bq_device is null, do nothing\n");
        return POWER_SUPPLY_STATUS_UNKNOWN;
    }
        result = bq2415x_exec_command(g_bq, BQ2415X_CHARGE_STATUS);
        if (result < 0)
        {
            pmu_log_err("bq24152 get charge status error\n");
                return POWER_SUPPLY_STATUS_UNKNOWN;
        }
        if(g_bq->use_only_charge)
        {
            if(g_bq->qcom_bms_psy)
            {
               if((g_bq->charge_done_flag)||((get_ui_bms_soc()==BAT_CAPACITY_FULL)&&bq24152_charger_present()))
               {
                   return  POWER_SUPPLY_STATUS_FULL;
               }
            }
        }
        else
        {

           if(g_bq->charge_done_flag \
               || ((g_battery_measure_by_bq27510_device == NULL ? 0 \
                   : g_battery_measure_by_bq27510_device->capacity == CAPACITY_FULL) \
               && bq24152_charger_present()))
          {
                return POWER_SUPPLY_STATUS_FULL;
          }
       }
        if (result == 0) /* Ready */
            return POWER_SUPPLY_STATUS_DISCHARGING;
        else if (result == 1) /* Charge in progress */
            return POWER_SUPPLY_STATUS_CHARGING;
        else if (result == 2) /* Charge done */
            return POWER_SUPPLY_STATUS_FULL;
        else
          return POWER_SUPPLY_STATUS_DISCHARGING;


    return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int bq24152_otg_regulator_enable(struct regulator_dev *rdev)
{
    struct bq2415x_device *bq = rdev_get_drvdata(rdev);
    if(bq==NULL)
    {
       pmu_log_info("bq_device is null, do nothing\n");
       return -1;
    }
    otg_enabled = true;
    bq24152_control_otg(bq,otg_enabled);

    return 0;
}

static int bq24152_otg_regulator_disable(struct regulator_dev *rdev)
{
    struct bq2415x_device *bq = rdev_get_drvdata(rdev);
    if(bq==NULL)
    {
       pmu_log_info("bq_device is null, do nothing\n");
       return -1;
    }
    otg_enabled = false;
    bq24152_control_otg(bq,otg_enabled);
    return 0;
}

static int bq24152_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    int ret = 0;
    struct bq2415x_device *bq = rdev_get_drvdata(rdev);
    ret = bq2415x_exec_command(bq, BQ2415X_BOOST_STATUS);
    if (ret < 0)
        return ret;
    if(0 == ret)
        return 0;
    if(1 == ret)
        return 1;
    return 0;

}

struct charge_device_ops bq2415x_ops = {
    .set_runningtest = bq2415x_set_runningtest,
    .set_enable_charger = bq2415x_enable_charge,
    .set_in_thermal = bq2415x_set_in_thermal,
    .shutdown_q4 = NULL,
    .shutdown_wd = NULL,
    .set_usb_current = bq2415x_set_usb_current,
    .dump_register = bq2415x_get_chargelog,
    .get_ibus = NULL,
    .get_bat_status = bq2415x_get_battery_status,
};

struct regulator_ops bq24152_otg_reg_ops =
{
    .enable = bq24152_otg_regulator_enable,
    .disable = bq24152_otg_regulator_disable,
    .is_enabled = bq24152_otg_regulator_is_enable,
};

static int bq24152_regulator_init(struct bq2415x_device *bq)
{
    int rc = 0;
    struct regulator_init_data *init_data;
    struct regulator_config cfg = {};

    init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
    if (!init_data)
    {
        dev_err(bq->dev, "Unable to allocate memory\n");
        return -ENOMEM;
    }

    if (init_data->constraints.name)
    {
        bq->otg_vreg.rdesc.owner = THIS_MODULE;
        bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
        bq->otg_vreg.rdesc.ops = &bq24152_otg_reg_ops;
        bq->otg_vreg.rdesc.name = init_data->constraints.name;

        cfg.dev = bq->dev;
        cfg.init_data = init_data;
        cfg.driver_data = bq;
        cfg.of_node = bq->dev->of_node;

        init_data->constraints.valid_ops_mask
        |= REGULATOR_CHANGE_STATUS;

        bq->otg_vreg.rdev = regulator_register(
                                &bq->otg_vreg.rdesc, &cfg);
        if (IS_ERR(bq->otg_vreg.rdev))
        {
            rc = PTR_ERR(bq->otg_vreg.rdev);
            bq->otg_vreg.rdev = NULL;
            if (rc != -EPROBE_DEFER)
                dev_err(bq->dev,
                        "OTG reg failed, rc=%d\n", rc);
        }
    }

    return rc;
}

/* main bq2415x probe function */
static int bq2415x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	int num;
	char *name;
	struct bq2415x_device *bq;
	struct device_node *np = client->dev.of_node;
	struct power_supply *usb_psy;
    struct charge_device_ops *ops = NULL;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pmu_log_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	/* Get new ID for the new device */
	mutex_lock(&bq2415x_id_mutex);
	num = idr_alloc(&bq2415x_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&bq2415x_id_mutex);
	if (num < 0)
		return num;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		ret = -ENOMEM;
		goto error_1;
	}

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "failed to allocate device data\n");
		ret = -ENOMEM;
		goto error_2;
	}

	i2c_set_clientdata(client, bq);

	bq->usb_psy = usb_psy;
	bq->id = num;
	bq->dev = &client->dev;
	bq->chip = id->driver_data;
	bq->name = name;
	bq->mode = BQ2415X_MODE_OFF;
	bq->reported_mode = BQ2415X_MODE_OFF;
	bq->autotimer = 0;
	bq->automode = 0;
	bq->charge_done_flag = 0;
	bq->charge_disable = false;
    bq->soc_resume_charging = false;
    ops = &bq2415x_ops;
    ret = charge_ops_register(ops);
    if(ret)
    {
        pmu_log_err("register charge ops failed!\n");
        goto error_2;
    }
	spin_lock_init(&bq->ibat_change_lock);
    mutex_init(&bq->current_change_lock);
    ret = of_property_read_u32(np, "ti,use-only-charger",
       &bq->use_only_charge);
    pmu_log_info("use bq only charge :%d\n",bq->use_only_charge);
	ret = of_property_read_u32(np, "ti,current-limit",
			&bq->init_data.current_limit);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,weak-battery-voltage",
			&bq->init_data.weak_battery_voltage);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,battery-regulation-voltage",
			&bq->init_data.battery_regulation_voltage);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,charge-current",
			&bq->init_data.charge_current);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,termination-current",
			&bq->init_data.termination_current);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ti,resistor-sense",
			&bq->init_data.resistor_sense);
	if (ret)
		return ret;

	bq2415x_reset_chip(bq);
    ret = bq24152_regulator_init(bq);
    if (ret)
    {
        dev_err(&client->dev,"Couldn't initialize bq24152 regulator rc=%d\n", ret);
        goto error_2;
    }
	ret = bq2415x_power_supply_init(bq);
	if (ret) {
		dev_err(bq->dev, "failed to register power supply: %d\n", ret);
		goto error_2;
	}

	ret = bq2415x_sysfs_init(bq);
	if (ret) {
		dev_err(bq->dev, "failed to create sysfs entries: %d\n", ret);
		goto error_3;
	}

	ret = bq2415x_set_defaults(bq);
	if (ret) {
		dev_err(bq->dev, "failed to set default values: %d\n", ret);
		goto error_4;
	}

	if (bq->init_data.set_mode_hook) {
		if (bq->init_data.set_mode_hook(
				bq2415x_hook_function, bq)) {
			bq->automode = 1;
			bq2415x_set_mode(bq, bq->reported_mode);
			dev_info(bq->dev, "automode enabled\n");
		} else {
			bq->automode = -1;
			dev_info(bq->dev, "automode failed\n");
		}
	} else {
		bq->automode = -1;
		dev_info(bq->dev, "automode not supported\n");
	}

    g_bq=bq;
	INIT_DELAYED_WORK(&bq->otg_id_check_work, check_otg_id_work);
	INIT_DELAYED_WORK(&bq->work, bq2415x_timer_work);
	bq2415x_set_autotimer(bq, 1);
	INIT_WORK(&bq->iusb_work,bq2415x_iusb_work);
	INIT_DELAYED_WORK(&bq->lower_power_charger_work,
			bq2415x_usb_low_power_work);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_CHARGER);
#endif
	dev_info(bq->dev, "driver registered\n");
    pmu_log_info("bq24152 probe ok\n");

	return 0;

error_4:
	bq2415x_sysfs_exit(bq);
error_3:
	bq2415x_power_supply_exit(bq);
error_2:
	kfree(name);
error_1:
    mutex_destroy(&bq->current_change_lock);
	mutex_lock(&bq2415x_id_mutex);
	idr_remove(&bq2415x_id, num);
	mutex_unlock(&bq2415x_id_mutex);

	return ret;
}

/* main bq2415x remove function */

static int bq2415x_remove(struct i2c_client *client)
{
	struct bq2415x_device *bq = i2c_get_clientdata(client);

	if (bq->init_data.set_mode_hook)
		bq->init_data.set_mode_hook(NULL, NULL);
	cancel_delayed_work_sync(&bq->lower_power_charger_work);
	cancel_delayed_work_sync(&bq->otg_id_check_work);
    regulator_unregister(bq->otg_vreg.rdev);
	bq2415x_sysfs_exit(bq);
	bq2415x_power_supply_exit(bq);

	bq2415x_reset_chip(bq);

    mutex_destroy(&bq->current_change_lock);
	mutex_lock(&bq2415x_id_mutex);
	idr_remove(&bq2415x_id, bq->id);
	mutex_unlock(&bq2415x_id_mutex);

	dev_info(bq->dev, "driver unregistered\n");

	kfree(bq->name);

	return 0;
}
static int bq24152_suspend(struct i2c_client *client,pm_message_t state)
{
    struct bq2415x_device *bq = i2c_get_clientdata(client);
    cancel_delayed_work_sync(&bq->work);
    return 0;
}

static int bq24152_resume(struct i2c_client *client)
{
    struct bq2415x_device *bq = i2c_get_clientdata(client);
    schedule_delayed_work(&bq->work, 0);
    return 0;
}

static const struct i2c_device_id bq2415x_i2c_id_table[] = {
       { "bq2415x", BQUNKNOWN },
       { "bq24150", BQ24150 },
       { "bq24150a", BQ24150A },
       { "bq24151", BQ24151 },
       { "bq24151a", BQ24151A },
       { "bq24152", BQ24152 },
       { "bq24153", BQ24153 },
       { "bq24153a", BQ24153A },
       { "bq24155", BQ24155 },
       { "bq24156", BQ24156 },
       { "bq24156a", BQ24156A },
       { "bq24158", BQ24158 },
       {},
};
static void bq24152_shutdown(struct i2c_client *client)
{
    int rc =0;
    struct bq2415x_device *bq = i2c_get_clientdata(client);
    if(is_usb_chg_exist())
    {
        rc = bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
        if(rc < 0)
        {
            pmu_log_info("disable charger failed\n");
            return;
        }
    }
    return;
}

static struct of_device_id bq2419x_charger_match_table[] =
{
   { .compatible = "ti,bq24152",},
};

static struct i2c_driver bq2415x_driver = {
	.driver = {
		.name = "ti,bq24152",			
        .of_match_table = bq2419x_charger_match_table,
	},
	.probe = bq2415x_probe,
	.remove = bq2415x_remove,
	.suspend = bq24152_suspend,
	.resume = bq24152_resume,
    .shutdown = bq24152_shutdown,
	.id_table = bq2415x_i2c_id_table,
};
static int __init bq24152_charger_init(void)
{
    return i2c_add_driver(&bq2415x_driver);
}
device_initcall_sync(bq24152_charger_init);

static void __exit bq24152_charger_exit(void)
{
    i2c_del_driver(&bq2415x_driver);
}
module_exit(bq24152_charger_exit);

MODULE_AUTHOR("Pali Rohár <pali.rohar@gmail.com>");
MODULE_DESCRIPTION("bq2415x charger driver");
MODULE_LICENSE("GPL");
