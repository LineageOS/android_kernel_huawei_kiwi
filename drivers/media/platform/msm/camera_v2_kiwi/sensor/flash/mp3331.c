/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
//#include <mach/gpiomux.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#ifdef CONFIG_HUAWEI_DSM
#include "msm_camera_dsm.h"
#endif

#define FLASH_NAME "mps,mp3331"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define MP3331_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define MP3331_DBG(fmt, args...)
#endif
#define FLASH_CHIP_ID_REG    0x00
#define FLASH_CHIP_ID             0x18
#define FLASH_CHIP_ID_MASK 0xF8

#define FLASH_FLAG_REGISTER   0x0B
#define FLASH_FAULT_REGISTER 0x0C

#ifdef CONFIG_HUAWEI_DSM
#define FT_VOSC       1<<5  //VOUT-GND short
#define FT_LEDSC     1<<4  //LED-GND short
#define FT_OTP         1<<3  //over temperature
#define LED_SHORT                                 (FT_VOSC|FT_LEDSC)
#define LED_THERMAL_SHUTDOWN        FT_OTP
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver mp3331_i2c_driver;

/*Enable IFVM and set it threshold to 3.2v ,also set flash ramp time to 512us */
static struct msm_camera_i2c_reg_array mp3331_init_array[] = {
    {0x01, 0xA0},/*mode setting, standby mode*/
    {0x02, 0x03},/*streching down fs from 1M when vin is close to vout*/
    {0x03, 0xB0},/*fs:4Mhz, reset to standby automaticly when torch or flash off, flash timer:600ms*/
    {0x04, 0x5F},/*low battery voltage: 3.2V*/
    {0x05, 0xF4},/*over-temperature:130,if temp is over 130, current will be decrease to I_TX; VTS_PAS:450mV*/
    {0x06, 0x18},/*flash current:760.8mA*/
    {0x07, 0x08},/*I_TX:253.6mA*/
    {0x0a, 0x05},/*torch current:158.5mA*/
};


static struct msm_camera_i2c_reg_array mp3331_off_array[] = {
    {0x01, 0xA0},/*set to standby mode*/
};

static struct msm_camera_i2c_reg_array mp3331_release_array[] = {
    {0x01, 0xA0},/*set to standby mode*/
};

static struct msm_camera_i2c_reg_array mp3331_low_array[] = {
    {0x01, 0xA4},/*set to assist mode*/
    {0x0A, 0x08},/*set current:31.7*8=253.6mA*/
    {0x01, 0xB4},/*enable current source*/
};

static struct msm_camera_i2c_reg_array mp3331_high_array[] = {
    {0x01, 0xA6},/*set to flash mode, hardware*/
    {0x06, 0x18},/*set current:31.7*24 = 760.8mA*/
    {0x03, 0xB0},/*flash timer:600ms, fs:4Mhz*/
    {0x01, 0xB6},/*enable current source*/
};

static struct msm_camera_i2c_reg_array mp3331_torch_array[] = {
    {0x0A, 0x05},/*set current:31.7*5=158.5mA*/
    {0x01, 0xB4},/*enable current source*/
};


static const struct of_device_id mp3331_i2c_trigger_dt_match[] = {
    {.compatible = "mps,mp3331"},
    {}
};

MODULE_DEVICE_TABLE(of, mp3331_i2c_trigger_dt_match);
static const struct i2c_device_id mp3331_i2c_id[] = {
    {FLASH_NAME, (kernel_ulong_t)&fctrl},
    { }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
    MP3331_DBG("%s:%d, value:%d called\n", __func__, __LINE__,value);

    if (value > LED_OFF) {
        if(fctrl.func_tbl->flash_led_low)
            fctrl.func_tbl->flash_led_low(&fctrl);
    } else {
        if(fctrl.func_tbl->flash_led_off)
            fctrl.func_tbl->flash_led_off(&fctrl);
    }
};

static struct led_classdev msm_torch_led = {
    .name                   = "torch-light",
    .brightness_set      = msm_led_torch_brightness_set,
    .brightness            = LED_OFF,
};

static int32_t msm_mp3331_torch_create_classdev(struct device *dev , void *data)
{
    int rc;
    MP3331_DBG("%s:%d called\n", __func__, __LINE__);
    msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
    rc = led_classdev_register(dev, &msm_torch_led);
    if (rc) {
        pr_err("Failed to register led dev. rc = %d\n", rc);
        return rc;
    }

    return 0;
};

#ifdef CONFIG_HUAWEI_DSM
static void camera_report_flash_mp3331_dsm_err(uint16_t flash_reg, int rc_value)
{
    ssize_t len = 0;

    MP3331_DBG("%s:%d called, flash_reg:0x%x\n", __func__, __LINE__,flash_reg);

    memset(camera_dsm_log_buff, 0, MSM_CAMERA_DSM_BUFFER_SIZE);

    if (rc_value < 0)
    {
        pr_err("%s. i2c read error, no dsm error\n",__func__);
        return;
    }

    /* camera record error info according to err type */
    if (flash_reg & LED_THERMAL_SHUTDOWN)
    {
        MP3331_DBG("%s:%d called, flash_reg:0x%x\n", __func__, __LINE__,flash_reg);

        len += snprintf(camera_dsm_log_buff+len, MSM_CAMERA_DSM_BUFFER_SIZE-len, "[msm_camera]Flash temperature reached thermal shutdown value.\n");
        if ((len < 0) || (len >= MSM_CAMERA_DSM_BUFFER_SIZE -1))
        {
            pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
            return ;
        }
        camera_report_dsm_err(DSM_CAMERA_LED_FLASH_OVER_TEMPERATURE, flash_reg, camera_dsm_log_buff);
    }
    else if (flash_reg & LED_SHORT)
    {
        MP3331_DBG("%s:%d called, flash_reg:0x%x\n", __func__, __LINE__,flash_reg);

        len += snprintf(camera_dsm_log_buff+len, MSM_CAMERA_DSM_BUFFER_SIZE-len, "[msm_camera]Flash led short detected.\n");
        if ((len < 0) || (len >= MSM_CAMERA_DSM_BUFFER_SIZE -1))
        {
            pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
            return ;
        }
        camera_report_dsm_err(DSM_CAMERA_LED_FLASH_CIRCUIT_ERR, flash_reg, camera_dsm_log_buff);
    }
    else
    {
        MP3331_DBG("%s:%d called, flash_reg:0x%x\n", __func__, __LINE__,flash_reg);

        len += snprintf(camera_dsm_log_buff+len, MSM_CAMERA_DSM_BUFFER_SIZE-len, "[msm_camera]Flash led voltage error.\n");
        if ((len < 0) || (len >= MSM_CAMERA_DSM_BUFFER_SIZE -1))
        {
            pr_err("%s %d. write camera_dsm_log_buff error\n",__func__, __LINE__);
            return ;
        }
        camera_report_dsm_err(DSM_CAMERA_LED_FLASH_VOUT_ERROR, flash_reg, camera_dsm_log_buff);
    }

    return;
}
#endif


/****************************************************************************
* FunctionName: msm_flash_clear_err_and_unlock;
* Description : clear the error and unlock the IC ;
* NOTE: this funtion must be called before register is read and write
***************************************************************************/
static int msm_flash_mp3331_clear_err_and_unlock(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;
    uint16_t flag_value=0;
    uint16_t fault_value=0;
    uint16_t reg_value = 0;

    MP3331_DBG("%s entry\n", __func__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    if (fctrl->flash_i2c_client) {
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
                        fctrl->flash_i2c_client,
                        FLASH_FLAG_REGISTER,&flag_value, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0){
            pr_err("clear err and unlock %s:%d failed\n", __func__, __LINE__);
        }
        MP3331_DBG("clear flag err and unlock success:flag_value:%02x\n",flag_value);

        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
                        fctrl->flash_i2c_client,
                        FLASH_FAULT_REGISTER,&flag_value, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0){
            pr_err("clear err and unlock %s:%d failed\n", __func__, __LINE__);
        }
        MP3331_DBG("clear fault err and unlock success:fault_value%02x\n",fault_value);

        reg_value = (fault_value<<8)|flag_value;

#ifdef CONFIG_HUAWEI_DSM
        if (reg_value != 0){
            camera_report_flash_mp3331_dsm_err(reg_value, rc);
        }
#endif
    }else{
        pr_err("%s:%d flash_i2c_client NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    return 0;
}

int msm_flash_mp3331_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;
    struct msm_camera_sensor_board_info *flashdata = NULL;
    struct msm_camera_power_ctrl_t *power_info = NULL;

    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    flashdata = fctrl->flashdata;
    power_info = &flashdata->power_info;

    gpio_set_value_cansleep(
                power_info->gpio_conf->gpio_num_info->
                gpio_num[SENSOR_GPIO_FL_NOW],
                GPIO_OUT_LOW);
    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_mp3331_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                                   fctrl->flash_i2c_client,
                                   fctrl->reg_setting->init_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }

    return rc;
}

int msm_flash_mp3331_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;
    struct msm_camera_sensor_board_info *flashdata = NULL;
    struct msm_camera_power_ctrl_t *power_info = NULL;

    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    flashdata = fctrl->flashdata;
    power_info = &flashdata->power_info;

    gpio_set_value_cansleep(
                power_info->gpio_conf->gpio_num_info->
                gpio_num[SENSOR_GPIO_FL_NOW],
                GPIO_OUT_LOW);

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_mp3331_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                    fctrl->flash_i2c_client,
                    fctrl->reg_setting->release_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }
    return 0;
}

int msm_flash_mp3331_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;
    struct msm_camera_sensor_board_info *flashdata = NULL;
    struct msm_camera_power_ctrl_t *power_info = NULL;
    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    flashdata = fctrl->flashdata;
    power_info = &flashdata->power_info;

    gpio_set_value_cansleep(
                power_info->gpio_conf->gpio_num_info->
                gpio_num[SENSOR_GPIO_FL_NOW],
                GPIO_OUT_LOW);

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_mp3331_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                    fctrl->flash_i2c_client,
                    fctrl->reg_setting->off_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }

    return rc;
}

int msm_flash_mp3331_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;

    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_mp3331_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
            fctrl->flash_i2c_client,
            fctrl->reg_setting->low_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }

    return rc;
}

int msm_flash_mp3331_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;
    struct msm_camera_sensor_board_info *flashdata = NULL;
    struct msm_camera_power_ctrl_t *power_info = NULL;

    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    flashdata = fctrl->flashdata;
    power_info = &flashdata->power_info;

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_mp3331_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
        pr_err("%s:%d  high mode\n", __func__, __LINE__);
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                    fctrl->flash_i2c_client,
                    fctrl->reg_setting->high_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }

    gpio_set_value_cansleep(
                power_info->gpio_conf->gpio_num_info->
                gpio_num[SENSOR_GPIO_FL_NOW],
                GPIO_OUT_HIGH);

    return rc;
}

/****************************************************************************
* FunctionName: msm_torch_mp3331_led_on;
* Description : set torch func ;
***************************************************************************/
int msm_torch_mp3331_led_on(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;

    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_mp3331_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
                    fctrl->flash_i2c_client,
                    fctrl->reg_setting->torch_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }
    
    return rc;
}

static int32_t msm_flash_mp3331_match_id(struct msm_led_flash_ctrl_t *fctrl)
{
    int32_t rc = 0;
    int32_t i = 0;
    uint16_t id_val = 0;
    MP3331_DBG("%s:%d called\n", __func__, __LINE__);

    if (!fctrl ||  !fctrl->flash_i2c_client) {
        pr_err("%s:%d  NULL PTR\n", __func__, __LINE__);
        return -EINVAL;
    }

    for(i = 0; i < 3; i++){
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
                    fctrl->flash_i2c_client,FLASH_CHIP_ID_REG,&id_val, MSM_CAMERA_I2C_BYTE_DATA);
        if(rc < 0){
            pr_err("%s: FLASHCHIP READ I2C error!\n", __func__);
            continue;
        }

        if ( FLASH_CHIP_ID == (id_val & FLASH_CHIP_ID_MASK) ){
            pr_info("%s, mp3331 match success, id:0x%x\n",__func__, id_val );
            break;
        }
    }

    if( i >= 3 ){
        pr_err("%s failed\n",__func__);
        rc = -ENODEV;
    }
    
    return rc;

}


static int msm_flash_mp3331_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct msm_camera_sensor_board_info *flashdata = NULL;
    struct msm_camera_power_ctrl_t *power_info = NULL;
    int rc = 0 ;
    MP3331_DBG("%s entry\n", __func__);
    if (!id) {
        pr_err("msm_flash_mp3331_i2c_probe: id is NULL");
        id = mp3331_i2c_id;
    }
    rc = msm_flash_i2c_probe(client, id);
    if( rc < 0 ){		
        pr_err("%s: msm_flash_i2c_probe fail\n", __func__);
        return rc;
    } else {
        /*when the flashlight open background, hold power key for more than 10s*/
        /*would enter HW reset, without turn off the light. So we need to close*/
        /*light after we reboot*/
        msm_flash_mp3331_led_off(&fctrl);
    }

    if(fctrl.flash_high_current){
        fctrl.reg_setting->high_setting->reg_setting[1].reg_data = fctrl.flash_high_current;
    }

    flashdata = fctrl.flashdata;
    power_info = &flashdata->power_info;

    rc = msm_camera_request_gpio_table(
                power_info->gpio_conf->cam_gpio_req_tbl,
                power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
    if (rc < 0) {
        pr_err("%s: request gpio failed\n", __func__);
        return rc;
    }

    if (fctrl.pinctrl_info.use_pinctrl == true) {
        pr_err("%s:%d PC:: flash pins setting to active state",
        __func__, __LINE__);
        rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
                    fctrl.pinctrl_info.gpio_state_active);
        if (rc)
            pr_err("%s:%d cannot set pin to active state",
            __func__, __LINE__);
    }
    
    if (!rc)
    msm_mp3331_torch_create_classdev(&(client->dev),NULL);
    return rc;
}

static int msm_flash_mp3331_i2c_remove(struct i2c_client *client)
{
    struct msm_camera_sensor_board_info *flashdata = NULL;
    struct msm_camera_power_ctrl_t *power_info = NULL;
    int rc = 0 ;
    MP3331_DBG("%s entry\n", __func__);
    flashdata = fctrl.flashdata;
    power_info = &flashdata->power_info;

    rc = msm_camera_request_gpio_table(
                power_info->gpio_conf->cam_gpio_req_tbl,
                power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
    if (rc < 0) {
        pr_err("%s: request gpio failed\n", __func__);
        return rc;
    }

    if (fctrl.pinctrl_info.use_pinctrl == true) {
    rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
                fctrl.pinctrl_info.gpio_state_suspend);
    if (rc)
        pr_err("%s:%d cannot set pin to suspend state",
                    __func__, __LINE__);
    }
    return rc;
}

static void mp3331_shutdown(struct i2c_client * client)
{
    pr_err("[%s],[%d]\n", __func__, __LINE__);
    msm_flash_led_off(&fctrl);
    msm_flash_led_release(&fctrl);
}

static struct i2c_driver mp3331_i2c_driver = {
    .id_table = mp3331_i2c_id,
    .probe  = msm_flash_mp3331_i2c_probe,
    .remove = msm_flash_mp3331_i2c_remove,
    .driver = {
        .name = FLASH_NAME,
        .owner = THIS_MODULE,
        .of_match_table = mp3331_i2c_trigger_dt_match,
    },
    .shutdown = mp3331_shutdown,
};

static int __init msm_flash_mp3331_init(void)
{
    MP3331_DBG("%s entry\n", __func__);
    return i2c_add_driver(&mp3331_i2c_driver);
}

static void __exit msm_flash_mp3331_exit(void)
{
    MP3331_DBG("%s entry\n", __func__);
    i2c_del_driver(&mp3331_i2c_driver);
    return;
}


static struct msm_camera_i2c_client mp3331_i2c_client = {
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting mp3331_init_setting = {
    .reg_setting = mp3331_init_array,
    .size = ARRAY_SIZE(mp3331_init_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
};

static struct msm_camera_i2c_reg_setting mp3331_off_setting = {
    .reg_setting = mp3331_off_array,
    .size = ARRAY_SIZE(mp3331_off_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
};

static struct msm_camera_i2c_reg_setting mp3331_release_setting = {
    .reg_setting = mp3331_release_array,
    .size = ARRAY_SIZE(mp3331_release_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
};

static struct msm_camera_i2c_reg_setting mp3331_low_setting = {
    .reg_setting = mp3331_low_array,
    .size = ARRAY_SIZE(mp3331_low_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
};

static struct msm_camera_i2c_reg_setting mp3331_high_setting = {
    .reg_setting = mp3331_high_array,
    .size = ARRAY_SIZE(mp3331_high_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
};

static struct msm_camera_i2c_reg_setting mp3331_torch_setting = {
    .reg_setting = mp3331_torch_array,
    .size = ARRAY_SIZE(mp3331_torch_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
};

static struct msm_led_flash_reg_t mp3331_regs = {
    .init_setting = &mp3331_init_setting,
    .off_setting = &mp3331_off_setting,
    .low_setting = &mp3331_low_setting,
    .high_setting = &mp3331_high_setting,
    .release_setting = &mp3331_release_setting,
    .torch_setting = &mp3331_torch_setting,
};

static struct msm_flash_fn_t mp3331_func_tbl = {
    .flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
    .flash_led_config = msm_led_i2c_trigger_config,
    .flash_led_init = msm_flash_mp3331_led_init,
    .flash_led_release = msm_flash_mp3331_led_release,
    .flash_led_off = msm_flash_mp3331_led_off,
    .flash_led_low = msm_flash_mp3331_led_low,
    .flash_led_high = msm_flash_mp3331_led_high,
    .torch_led_on = msm_torch_mp3331_led_on,
    .flash_match_id = msm_flash_mp3331_match_id,
};

static struct msm_led_flash_ctrl_t fctrl = {
    .flash_i2c_client = &mp3331_i2c_client,
    .reg_setting = &mp3331_regs,
    .func_tbl = &mp3331_func_tbl,
    // Set default flash high current.
    .flash_high_current = 0x1B,
};

module_init(msm_flash_mp3331_init);
module_exit(msm_flash_mp3331_exit);
MODULE_DESCRIPTION("mp3331 FLASH");
MODULE_LICENSE("GPL v2");
