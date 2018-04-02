//modify debug log in hw style
/*
 * Flash-led driver for Maxim MAX77819
 *
 * Copyright (C) 2013 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mfd/max77819.h>
#include <linux/gpio.h>

#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#include <linux/power_supply.h>

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "max77819_flash"
#include <linux/hw_camera_common.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#define M2SH  __CONST_FFS

/* Registers */
#define MAX77819_IFLASH			0x00
#define MAX77819_ITORCH			0x02
#define MAX77819_TORCH_TMR		0x03
#define MAX77819_FLASH_TMR		0x04
#define MAX77819_FLASH_EN		0x05
#define MAX77819_MAX_FLASH1		0x06
#define MAX77819_MAX_FLASH2		0x07
#define MAX77819_MAX_FLASH3		0x08
#define MAX77819_VOUT_CNTL		0x0A
#define MAX77819_VOUT_FLASH		0x0B
#define MAX77819_FLASH_INT		0x0E
#define MAX77819_FLASH_INT_MASK		0x0F
#define MAX77819_FLASH_STATUS		0x10

/* MAX77819_IFLASH */
#define MAX77819_FLASH_I		0x3F

/* MAX77819_ITORCH */
#define MAX77819_TORCH_I		0x0F

/* MAX77819_TORCH_TMR */
#define MAX77819_TORCH_TMR_DUR		0x0F
#define MAX77819_DIS_TORCH_TMR		0x40
#define MAX77819_TORCH_TMR_MODE		0x80
#define MAX77819_TORCH_TMR_ONESHOT	0x00
#define MAX77819_TORCH_TMR_MAXTIMER	0x80

/* MAX77819_FLASH_TMR */
#define MAX77819_FLASH_TMR_DUR		0x0F
#define MAX77819_FLASH_TMR_MODE		0x80
#define MAX77819_FLASH_TMR_ONESHOT	0x00
#define MAX77819_FLASH_TMR_MAXTIMER	0x80

/* MAX77819_FLASH_EN */
#define MAX77819_TORCH_FLED_EN		0x0C
#define MAX77819_FLASH_FLED_EN		0xC0
#define MAX77819_OFF			0x00
#define MAX77819_BY_FLASHEN		0x01
#define MAX77819_BY_TORCHEN		0x02
#define MAX77819_BY_I2C			0X03

/* MAX77819_MAX_FLASH1 */
#define MAX77819_MAX_FLASH_HYS		0x03
#define MAX77819_MAX_FLASH_TH		0x7C
#define MAX77819_MAX_FLASH_TH_FROM_VOLTAGE(uV) \
		((((uV) - 2400000) / 33333) << M2SH(MAX77819_MAX_FLASH_TH))
#define MAX77819_MAX_FL_EN		0x80

/* MAX77819_MAX_FLASH2 */
#define MAX77819_LB_TMR_F		0x07
#define MAX77819_LB_TMR_R		0x38
#define MAX77819_LB_TME_FROM_TIME(uSec) ((uSec) / 256)

/* MAX77819_MAX_FLASH3 */
#define MAX77819_FLED_MIN_OUT		0x3F
#define MAX77819_FLED_MIN_MODE		0x80

/* MAX77819_VOUT_CNTL */
#define MAX77819_BOOST_FLASH_MDOE	0x07
#define MAX77819_BOOST_FLASH_MODE_OFF	0x00
#define MAX77819_BOOST_FLASH_MODE_ADAPTIVE	0x01
#define MAX77819_BOOST_FLASH_MODE_FIXED	0x04

/* MAX77819_VOUT_FLASH */
#define MAX77819_BOOST_VOUT_FLASH 	0x7F
#define MAX77819_BOOST_VOUT_FLASH_FROM_VOLTAGE(uV)				\
		((uV) <= 3300000 ? 0x00 :					\
		((uV) <= 5500000 ? (((mV) - 3300000) / 25000 + 0x0C) : 0x7F))

/* MAX77819_FLASH_INT_MASK */
#define MAX77819_FLED_OPEN_M		0x04
#define MAX77819_FLED_SHORT_M		0x08
#define MAX77819_MAX_FLASH_M		0x10
#define MAX77819_FLED_FAIL_M		0x20

/* MAX77819_FLASH_STATAUS */
#define MAX77819_TORCH_ON_STAT		0x04
#define MAX77819_FLASH_ON_STAT		0x08

#define MAX_FLASH_CURRENT	1000	// 1000mA(0x1f)
#define MAX_TORCH_CURRENT	250	// 250mA(0x0f)   
#define MAX_FLASH_DRV_LEVEL	63	/* 15.625 + 15.625*63 mA */
#define MAX_TORCH_DRV_LEVEL	15	/* 15.625 + 15.625*15 mA */

#define CAM_FLASH_PINCTRL_STATE_SLEEP "cam_flash_suspend"
#define CAM_FLASH_PINCTRL_STATE_DEFAULT "cam_flash_default"

#define MAX77819_POWR_SUPPLY_BATTERY_NAME "battery"
#define MAX77819_POWR_SUPPLY_CHARGER_NAME "max77819-charger"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
/******for max77819 registers acess***/

struct max77819_flash{
	struct regmap *regmap;
	struct power_supply *max77819_charger;
	struct power_supply *max77819_battery;
};

static struct msm_led_flash_ctrl_t fctrl;

//charger is present return 1
//charger is not present return 0
//error return < 0
static int max77819_flash_get_charger_present(struct max77819_flash *flash)
{
	int rc = 0;
	union power_supply_propval prop_val = {0};
	CMR_LOGD("%s:%d called\n", __func__, __LINE__);

	if(!flash)
	{
		CMR_LOGE("%s:flash is null\n",__func__);
		return (-1);
	}

	//get max77819 charger ctrl node,to get charger status
	if (!flash->max77819_charger)
	{
		flash->max77819_charger = power_supply_get_by_name(MAX77819_POWR_SUPPLY_CHARGER_NAME);
		if (!flash->max77819_charger)
		{
			CMR_LOGE("%s:the max77819 power_supply charger is not got!!\n",__func__);
			return (-1);
		}
	}

	if(!flash->max77819_charger->get_property)
	{
		CMR_LOGE("%s:flash->max77819_charger->get_property is null\n",__func__);
		return (-1);
	}

	rc = flash->max77819_charger->get_property(flash->max77819_charger, POWER_SUPPLY_PROP_PRESENT, &prop_val);
	if (unlikely(IS_ERR_VALUE(rc)))
	{
		CMR_LOGE("%s:get power supply present error! rc = %d \n",__func__, rc);
		return (-1);
	}
	CMR_LOGE("%s:get power supply present=%d \n",__func__, prop_val.intval);
	return prop_val.intval;
}


static int max77819_flash_set_charger_enable(struct max77819_flash *flash, int on)
{
	int rc = 0;
	union power_supply_propval prop_val = {0};
	CMR_LOGD("%s:%d called\n", __func__, __LINE__);

	if(!flash)
	{
		CMR_LOGE("%s:flash is null\n",__func__);
		return (-1);
	}

	//get max77819 battery ctrl node,to control charger when flash is on
	if (!flash->max77819_battery)
	{
		flash->max77819_battery = power_supply_get_by_name(MAX77819_POWR_SUPPLY_BATTERY_NAME);
		if (!flash->max77819_battery)
		{
			CMR_LOGE("%s:the max77819 battery power_supply is not got!!\n",__func__);
			return (-1);
		}
	}

	prop_val.intval = (on)?(1):(0);

	if(!flash->max77819_battery->set_property)
	{
		CMR_LOGE("%s:flash->max77819_battery->set_property is null\n",__func__);
		return (-1);
	}

	rc = flash->max77819_battery->set_property(flash->max77819_battery, POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop_val);
	CMR_LOGE("%s:%d set charger enable %d\n", __func__, __LINE__,prop_val.intval);
	if (unlikely(IS_ERR_VALUE(rc)))
	{
		CMR_LOGE("%s:set power supply battery enable error! rc = %d \n", __func__, rc);
		return (-1);
	}

	return 0;
}

static int max77819_flash_led_init(struct msm_led_flash_ctrl_t *fctrl, 
	struct max77819_flash *flash)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	unsigned int value;

	CMR_LOGE("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		CMR_LOGE("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		CMR_LOGE("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc)
			CMR_LOGE("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	msleep(2);

	/*Clear status register*/
	rc = regmap_read(flash->regmap, MAX77819_FLASH_INT, &value);

	//move set flash led enable after gpio set
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	/*Enable Flash LED*/
	rc = regmap_update_bits(flash->regmap, MAX77819_FLASH_EN,MAX77819_FLASH_FLED_EN,
			(MAX77819_BY_FLASHEN << M2SH(MAX77819_FLASH_FLED_EN)));

	//clear the err and unlock IC, this function must be called before read and write register
	//msm_flash_clear_err_and_unlock(fctrl);

	return rc;
}

static int max77819_flash_led_release(struct msm_led_flash_ctrl_t *fctrl, 
	struct max77819_flash *flash)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CMR_LOGE("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		CMR_LOGE("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	rc = regmap_update_bits(flash->regmap, MAX77819_FLASH_EN, MAX77819_FLASH_FLED_EN|MAX77819_TORCH_FLED_EN, 0);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		CMR_LOGE("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (rc)
			CMR_LOGE("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}

	return 0;
}

static int max77819_flash_led_off(struct msm_led_flash_ctrl_t *fctrl, 
	struct max77819_flash *flash)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	unsigned int value;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	
	CMR_LOGE("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		CMR_LOGE("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	/*Clear status register*/
	rc = regmap_read(flash->regmap, MAX77819_FLASH_INT, &value);
	
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	rc = regmap_update_bits(flash->regmap, MAX77819_FLASH_EN, MAX77819_FLASH_FLED_EN|MAX77819_TORCH_FLED_EN, 0);
	return rc;
}

static int  max77819_flash_led_low(struct msm_led_flash_ctrl_t *fctrl, 
	struct max77819_flash *flash)
{
	int rc = 0;
	unsigned int brightness = 0x06;//0x06=164.05mA
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CMR_LOGE("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CMR_LOGE("%s:%d set brightness 0x%x\n", __func__, __LINE__,brightness);
//do not use flash_led mode,
//now replaced by torch_led mode
//To prevent flash auto shut down if the focusing time more than 1s
#if 1
	rc = regmap_update_bits(flash->regmap, MAX77819_ITORCH, MAX77819_TORCH_I,
			brightness << M2SH(MAX77819_TORCH_I));

	/*Enable Torch LED*/
	//rc = regmap_update_bits(flash->regmap, MAX77819_FLASH_EN,MAX77819_TORCH_FLED_EN,
	//		(MAX77819_BY_FLASHEN << M2SH(MAX77819_TORCH_FLED_EN))); //MAX77819_BY_I2C
	rc = regmap_write(flash->regmap, MAX77819_FLASH_EN,0x04);//set to torch mode ,disable flash mode

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#else
	rc = regmap_update_bits(flash->regmap, MAX77819_IFLASH, MAX77819_FLASH_I,
			brightness << M2SH(MAX77819_FLASH_I));

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif

	//clear the err and unlock IC, this function must be called before read and write register
	//msm_flash_clear_err_and_unlock(fctrl);
	
	return rc;
}

static int max77819_flash_led_high(struct msm_led_flash_ctrl_t *fctrl,
	struct max77819_flash *flash)
{
	int rc = 0;
	int rc_present = 0;
	unsigned int brightness = 0x1F; //0x1F=749.95mA
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CMR_LOGE("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	rc_present = max77819_flash_get_charger_present(flash);
	CMR_LOGI("%s:%d rc_present %d\n", __func__, __LINE__,rc_present);
	if(rc_present == 0)//cable is not present
	{
		brightness = 0x1F; //0x1F=749.95mA
		CMR_LOGD("%s:%d set brightness current 749mA\n", __func__, __LINE__);
	}
	else if(rc_present == 1)//cable is present set current to 500mA
	{
		brightness = 0x14; //0x14=492.16mA
		CMR_LOGD("%s:%d set brightness current 492mA\n", __func__, __LINE__);
	}
	else
	{
		brightness = 0x14; //0x14=492.16mA
		CMR_LOGD("%s:%d check present error set current to 492mA\n", __func__, __LINE__);
	}
	CMR_LOGE("%s:%d set brightness 0x%x\n", __func__, __LINE__,brightness);

	rc = regmap_update_bits(flash->regmap, MAX77819_IFLASH, MAX77819_FLASH_I,
			brightness << M2SH(MAX77819_FLASH_I));

	/*Enable Flash LED*/
	//rc = regmap_update_bits(flash->regmap, MAX77819_FLASH_EN,MAX77819_FLASH_FLED_EN,
	//		(MAX77819_BY_FLASHEN << M2SH(MAX77819_FLASH_FLED_EN)));
	rc = regmap_write(flash->regmap, MAX77819_FLASH_EN,0x40);//open flash mode, disable torch mode

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	//clear the err and unlock IC, this function must be called before read and write register
	//msm_flash_clear_err_and_unlock(fctrl);
	
	return rc;
}

/****************************************************************************
* FunctionName: msm_torch_max77819_flash_led_on;
* Description : set torch func ;
***************************************************************************/


static int max77819_torch_led_on(struct msm_led_flash_ctrl_t *fctrl,
	struct max77819_flash *flash)
{
	int rc = 0;
	unsigned int brightness = 0x06; //0x06=164.05mA 0x03=93.744mA
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CMR_LOGE("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CMR_LOGE("%s:%d set brightness 0x%x\n", __func__, __LINE__,brightness);
	rc = regmap_update_bits(flash->regmap, MAX77819_ITORCH, MAX77819_TORCH_I,
			brightness << M2SH(MAX77819_TORCH_I));

	/*Enable Torch LED*/
	//change torch led control by gpio FLASH_NOW
	//rc = regmap_update_bits(flash->regmap, MAX77819_FLASH_EN,MAX77819_TORCH_FLED_EN,
	//		(MAX77819_BY_FLASHEN << M2SH(MAX77819_TORCH_FLED_EN))); //MAX77819_BY_I2C
	rc = regmap_write(flash->regmap, MAX77819_FLASH_EN,0x04);//enable torch mode, disable flash mode

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
	/*
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	*/
	
	return rc;
}

static int msm_flash_pinctrl_init(struct msm_led_flash_ctrl_t *ctrl)
{
	struct msm_pinctrl_info *flash_pctrl = NULL;
	flash_pctrl = &ctrl->pinctrl_info;
	if (flash_pctrl->use_pinctrl != true) {
		CMR_LOGE("%s: %d PINCTRL is not enables in Flash driver node\n",
			__func__, __LINE__);
		return 0;
	}
	flash_pctrl->pinctrl = devm_pinctrl_get(&ctrl->pdev->dev);

	if (IS_ERR_OR_NULL(flash_pctrl->pinctrl)) {
		CMR_LOGE("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_active = pinctrl_lookup_state(
					       flash_pctrl->pinctrl,
					       CAM_FLASH_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_active)) {
		CMR_LOGE("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_suspend = pinctrl_lookup_state(
						flash_pctrl->pinctrl,
						CAM_FLASH_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_suspend)) {
		CMR_LOGE("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int32_t msm_led_get_dt_data(struct device_node *of_node,
		struct msm_led_flash_ctrl_t *fctrl)
{
	int32_t rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct device_node *flash_src_node = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3];

	CMR_LOGD("called\n");

	if (!of_node) {
		CMR_LOGE("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->flashdata) {
		CMR_LOGE("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		CMR_LOGE("failed\n");
		return -EINVAL;
	}

	CMR_LOGD("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&flashdata->sensor_name);
	CMR_LOGD("%s label %s, rc %d\n", __func__,
		flashdata->sensor_name, rc);
	if (rc < 0) {
		CMR_LOGE("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_i2c_master);
	CMR_LOGD("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	fctrl->pinctrl_info.use_pinctrl = false;
	fctrl->pinctrl_info.use_pinctrl = of_property_read_bool(of_node,
						"qcom,enable_pinctrl");
	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CMR_LOGD("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			CMR_LOGE("failed\n");
			return -EINVAL;
		}
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				CMR_LOGE("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				CMR_LOGE("failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CMR_LOGD("default trigger %s\n",
				 fctrl->flash_trigger_name[i]);

			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				CMR_LOGE("failed rc %d\n", rc);
				of_node_put(flash_src_node);
				continue;
			}

			of_node_put(flash_src_node);

			CMR_LOGD("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}

	} else { /*Handle LED Flash Ctrl by GPIO*/
		power_info->gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!power_info->gpio_conf) {
			CMR_LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			return rc;
		}
		gconf = power_info->gpio_conf;

		gpio_array_size = of_gpio_count(of_node);
		CMR_LOGD("%s gpio count %d\n", __func__, gpio_array_size);

		if (gpio_array_size) {
			gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
				GFP_KERNEL);
			if (!gpio_array) {
				CMR_LOGE("%s failed %d\n", __func__, __LINE__);
				rc = -ENOMEM;
				goto ERROR4;
			}
			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				CMR_LOGD("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				CMR_LOGE("%s failed %d\n", __func__, __LINE__);
				goto ERROR4;
			}

			rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				CMR_LOGE("%s failed %d\n", __func__, __LINE__);
				goto ERROR5;
			}

			rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				CMR_LOGE("%s failed %d\n", __func__, __LINE__);
				goto ERROR6;
			}
		}

		flashdata->slave_info =
			kzalloc(sizeof(struct msm_camera_slave_info),
				GFP_KERNEL);
		if (!flashdata->slave_info) {
			CMR_LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto ERROR8;
		}

		rc = of_property_read_u32_array(of_node, "qcom,slave-id",
			id_info, 3);
		if (rc < 0) {
			CMR_LOGE("%s failed %d\n", __func__, __LINE__);
			goto ERROR9;
		}
		fctrl->flashdata->slave_info->sensor_slave_addr = id_info[0];
		fctrl->flashdata->slave_info->sensor_id_reg_addr = id_info[1];
		fctrl->flashdata->slave_info->sensor_id = id_info[2];

		kfree(gpio_array);
		return rc;
ERROR9:
		kfree(fctrl->flashdata->slave_info);
ERROR8:
		kfree(fctrl->flashdata->power_info.gpio_conf->gpio_num_info);
ERROR6:
		kfree(gconf->cam_gpio_set_tbl);
ERROR5:
		kfree(gconf->cam_gpio_req_tbl);
ERROR4:
		kfree(gconf);
ERROR1:
		kfree(fctrl->flashdata);
		kfree(gpio_array);
	}
	return rc;
}

static int max77819_flash_hw_setup(struct max77819_io *io)
{
	struct regmap *regmap = io->regmap;
	//unsigned int value;
	int ret = 0;

	/*Disable flash and torch by default*/
	regmap_write(regmap, MAX77819_IFLASH, 0);
	regmap_write(regmap, MAX77819_ITORCH, 0);
	
	/* Torch Safty Timer Disabled, run for MAX timer */
	ret = regmap_write(regmap, MAX77819_TORCH_TMR,
			MAX77819_TORCH_TMR_DUR | MAX77819_DIS_TORCH_TMR | MAX77819_TORCH_TMR_MODE);
	if (IS_ERR_VALUE(ret))
		return ret;

	/* Flash Safty Timer = 1000ms, run for MAX timer */
	ret = regmap_write(regmap, MAX77819_FLASH_TMR,
			MAX77819_FLASH_TMR_DUR | MAX77819_FLASH_TMR_MAXTIMER);
	if (IS_ERR_VALUE(ret))
		return ret;

	/*flash mode setting*/
	//ret = regmap_write(regmap, MAX77819_FLASH_EN,0x48);
	ret = regmap_write(regmap, MAX77819_FLASH_EN,0x40);//default flash mode

	if (IS_ERR_VALUE(ret))
		return ret;

	/* Max Flash setting */
	//ret = regmap_write(regmap, MAX77819_MAX_FLASH1, 0xfc);
	//0xe0 1 11000 00 3192mV : 0xe4 1 11001 00 3225mV : 0xcc 1 10011 00 3027mV : 0x00 close protect
	//0xce 1 10011 10 3027mV detection hysteresis 300mV:
	//0xe7 1 11001 11 3225mV 0x03 = Hysteresis disabled. Flash current is only reduced.:
	//0xe3 1 11000 11 3192mV 0x03 = Hysteresis disabled. Flash current is only reduced.:
	ret = regmap_write(regmap, MAX77819_MAX_FLASH1, 0xe3);
	if (IS_ERR_VALUE(ret))
		return ret;

	/* Low battery mask timer for Max Flash */

	//0x3f 00 111 111 rising 2048us falling 2048us
	//0x38 00 111 000 rising 2048us falling 0us
	ret = regmap_write(regmap, MAX77819_MAX_FLASH2, 0x3f);

	if (IS_ERR_VALUE(ret))
		return ret;

 	/* recommended boost mode: adaptive*/
 	ret = regmap_write(regmap, MAX77819_VOUT_CNTL, MAX77819_BOOST_FLASH_MODE_ADAPTIVE);
	    
	return ret;
}


/**********************************************************************************/
static int32_t max77819_flash_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		CMR_LOGE("failed\n");
		return -EINVAL;
	}
	
	*subdev_id = fctrl->subdev_id;

	CMR_LOGI("MAX77819 subdev_id %d\n", *subdev_id);

	return 0;

}


int32_t max77819_flash_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	struct max77819_flash *flash = NULL;

	CMR_LOGI("MAX77819 called led_state %d\n", cfg->cfgtype);

	if (!fctrl) {
		CMR_LOGE("failed\n");
		return -EINVAL;
	}

	flash = dev_get_drvdata(&fctrl->pdev->dev);
	if (!flash){
		CMR_LOGE("Failed to get private data\n");
		return -EINVAL;
	}	
	
	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_INIT:
		rc = max77819_flash_led_init(fctrl, flash);
		break;

	case MSM_CAMERA_LED_RELEASE:
		rc = max77819_flash_led_release(fctrl, flash);
		max77819_flash_set_charger_enable(flash,1);
		break;

	case MSM_CAMERA_LED_OFF:
		rc = max77819_flash_led_off(fctrl, flash);
		max77819_flash_set_charger_enable(flash,1);
		break;

	case MSM_CAMERA_LED_LOW:
		max77819_flash_set_charger_enable(flash,0);
		rc = max77819_flash_led_low(fctrl, flash);
		break;

	case MSM_CAMERA_LED_HIGH:
		max77819_flash_set_charger_enable(flash,0);
		rc = max77819_flash_led_high(fctrl, flash);
		break;

	case MSM_CAMERA_LED_TORCH:
		max77819_flash_set_charger_enable(flash,0);
		rc = max77819_torch_led_on(fctrl, flash);
		break;

	default:
		rc = -EFAULT;
		break;
	}
	
	CMR_LOGD("flash_set_led_state: return %d\n", rc);
	return rc;
}


/**********************************************************************************/
#ifdef CONFIG_OF
static struct of_device_id max77819_flash_of_ids[] = {
    { .compatible = "maxim,"MAX77819_FLASH_NAME, .data = &fctrl },
    { },
};
MODULE_DEVICE_TABLE(of, max77819_flash_of_ids);
#endif /* CONFIG_OF */

static void max77819_flash_shutdown(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct max77819_flash *flash = NULL;
	struct msm_led_flash_ctrl_t *fctrl = NULL;

	CMR_LOGI("%s:%d called\n", __func__, __LINE__);

	match = of_match_device(max77819_flash_of_ids, &pdev->dev);
	if (!match){
		CMR_LOGE("%s: match is null failed\n", __func__);
		goto ERROR_TO_EXIT;
	}

	fctrl = (struct msm_led_flash_ctrl_t *)match->data;

	if (!fctrl) {
		CMR_LOGE("%s: fctrl is null failed\n", __func__);
		goto ERROR_TO_EXIT;
	}

	flash = dev_get_drvdata(&pdev->dev);
	if (!flash){
		CMR_LOGE("%s: Failed to get private data\n", __func__);
		goto ERROR_TO_EXIT;
	}

	max77819_flash_set_charger_enable(flash,1);
	max77819_flash_led_off(fctrl, flash);
	max77819_flash_led_release(fctrl, flash);

ERROR_TO_EXIT:
	CMR_LOGI("%s:%d exit\n", __func__, __LINE__);
}

static struct platform_driver max77819_flash_driver = {
	.driver = {
		.name = MAX77819_FLASH_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table  = max77819_flash_of_ids,
#endif /* CONFIG_OF */	
	},
	.shutdown = max77819_flash_shutdown,
};


static int32_t max77819_flash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct max77819_flash *max77819_flash;
	struct max77819_dev *chip = dev_get_drvdata(dev->parent);
	struct max77819_io *io = max77819_get_io(chip);
	struct device_node *nproot = dev->parent->of_node;
	struct msm_led_flash_ctrl_t *fctrl = NULL;
	struct device_node *of_node = NULL;
	int rc = 0;

	CMR_LOGI("in max77819_flash_probe\n");
	if((NULL == chip)||(NULL == io))
	{
		CMR_LOGE("chip or io is NULL\n");
		return -EFAULT;
	}
	match = of_match_device(max77819_flash_of_ids, &pdev->dev);
	if (!match)
	{
		CMR_LOGE("of_match_device error, match is NULL\n");
		return -EFAULT;
	}

	max77819_flash = devm_kzalloc(dev, sizeof(struct max77819_flash), GFP_KERNEL);
	if (unlikely(!max77819_flash))
	{
		CMR_LOGE("alloc memory error\n");
		return -ENOMEM;
	}

	max77819_flash->regmap = io->regmap;
	
	of_node = of_find_node_by_name(nproot, "qcom,led-flash");
	if (!of_node) {
		CMR_LOGE("of_node NULL\n");
		goto probe_failure;
	}

	fctrl = (struct msm_led_flash_ctrl_t *)match->data;
	fctrl->pdev = pdev;
	
	rc = msm_led_get_dt_data(of_node, fctrl);
	if (rc < 0) {
		CMR_LOGE("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		goto probe_failure;
	}
	
	msm_flash_pinctrl_init(fctrl);
	
	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
			"%s", fctrl->flashdata->sensor_name);
	/* Set device type as Platform*/
	fctrl->flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	
	rc = max77819_flash_hw_setup(io);
	if (IS_ERR_VALUE(rc))
	{
		CMR_LOGE("hw init error\n");
		goto probe_failure;
	}
	rc = msm_led_flash_create_v4lsubdev(pdev, fctrl);
	if (IS_ERR_VALUE(rc))
	{
		CMR_LOGE("%s: call msm_led_flash_create_v4lsubdev error\n", __func__);
		goto probe_failure;
	}

	rc = dev_set_drvdata(&pdev->dev, max77819_flash);
	if (IS_ERR_VALUE(rc))
	{
		CMR_LOGE("%s: call dev_set_drvdata error\n", __func__);
		goto probe_failure;
	}

	#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_FLASH);
	#endif

	CMR_LOGE("%s: probe success\n", __func__);
	return rc;
	
probe_failure:
	devm_kfree(dev, max77819_flash);
	CMR_LOGE("%s probe failed\n", __func__);
	return rc;

}

static int __init max77819_flash_add_driver(void)
{
	CMR_LOGD("%s called\n", __func__);
	return platform_driver_probe(&max77819_flash_driver,
		max77819_flash_probe);
}

static struct msm_flash_fn_t max77819_flash_func_tbl = {
	.flash_get_subdev_id = max77819_flash_get_subdev_id,
	.flash_led_config = max77819_flash_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &max77819_flash_func_tbl,
};


module_init(max77819_flash_add_driver);
MODULE_DESCRIPTION("Maxim77819 FLASH LED");
MODULE_LICENSE("GPL v2");
