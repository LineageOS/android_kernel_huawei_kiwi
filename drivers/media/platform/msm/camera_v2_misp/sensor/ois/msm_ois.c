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
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_ois.h"
#include "msm_cci.h"
/*mini-isp do the ois init*/
#include "mini_isp.h"
#include "../sensor_otp_common.h"
DEFINE_MSM_MUTEX(msm_ois_mutex);
/*#define MSM_OIS_DEBUG*/
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define MAX_POLL_COUNT 100

/*add camera ois driver*/
#define _USE_MINI_ISP
static struct v4l2_file_operations msm_ois_v4l2_subdev_fops;
static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl);
static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl);

extern bool huawei_cam_is_factory_mode(void);

static struct i2c_driver msm_ois_i2c_driver;

#ifdef USE_MINI_ISP_INIT_OIS
static int msm_ois_check_mini_isp_init_status(void);
#endif

static int32_t msm_ois_write_settings(struct msm_ois_ctrl_t *o_ctrl,
	uint16_t size, struct reg_settings_ois_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	CDBG("Enter\n");

	for (i = 0; i < size; i++) {
		switch (settings[i].i2c_operation) {
		case MSM_OIS_WRITE: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&o_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data,
					settings[i].data_type);
				break;
			case MSM_CAMERA_I2C_DWORD_DATA:
				reg_setting.reg_addr = settings[i].reg_addr;
				reg_setting.reg_data[0] = (uint8_t)
					((settings[i].reg_data &
					0xFF000000) >> 24);
				reg_setting.reg_data[1] = (uint8_t)
					((settings[i].reg_data &
					0x00FF0000) >> 16);
				reg_setting.reg_data[2] = (uint8_t)
					((settings[i].reg_data &
					0x0000FF00) >> 8);
				reg_setting.reg_data[3] = (uint8_t)
					(settings[i].reg_data & 0x000000FF);
				reg_setting.reg_data_size = 4;
				rc = o_ctrl->i2c_client.i2c_func_tbl->
					i2c_write_seq(&o_ctrl->i2c_client,
					reg_setting.reg_addr,
					reg_setting.reg_data,
					reg_setting.reg_data_size);
				if (rc < 0)
					return rc;
				break;

			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
		}
			break;

		case MSM_OIS_POLL: {
			int32_t poll_count = 0;
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				do {
					rc = o_ctrl->i2c_client.i2c_func_tbl
						->i2c_poll(&o_ctrl->i2c_client,
						settings[i].reg_addr,
						settings[i].reg_data,
						settings[i].data_type);

					if (poll_count++ > MAX_POLL_COUNT) {
						pr_err("MSM_OIS_POLL failed");
						break;
					}
				} while (rc != 0);
				break;

			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
		}
		}

		if (settings[i].delay > 20)
			msleep(settings[i].delay);
		else if (0 != settings[i].delay)
			usleep_range(settings[i].delay * 1000,
				(settings[i].delay * 1000) + 1000);

		if (rc < 0)
			break;
	}

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_vreg_control(struct msm_ois_ctrl_t *o_ctrl,
							int config)
{
/*add camera ois driver*/
#ifndef _USE_MINI_ISP
	int rc = 0, i, cnt;
	struct msm_ois_vreg *vreg_cfg;

	vreg_cfg = &o_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= MSM_OIS_MAX_VREGS) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(o_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
 #else
 
    return 0;
 #endif
}

static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (o_ctrl->ois_state != OIS_POWER_DOWN) {

		rc = msm_ois_vreg_control(o_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		o_ctrl->i2c_tbl_index = 0;
		o_ctrl->ois_state = OIS_POWER_DOWN;
	}
	CDBG("Exit\n");
	return rc;
}

/*add camera ois driver*/
/*delete invalid codes*/

/*optimize ois turn on/off interface in driver*/
static int32_t msm_ois_turn_onoff(struct msm_ois_ctrl_t *o_ctrl, uint32_t onoff)
{
    int32_t rc = 0;
	pr_info("%s: Enter onoff=%d\n",__func__, onoff);

	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	/*delete invalit code*/
    o_ctrl->ois_onoff = onoff;

    if((o_ctrl->delay_write_gain == 1) && (o_ctrl->ois_onoff == 1))
    {
        pr_info("%s: delay write gyro gain: %x %x\n",__func__,
                o_ctrl->towrite_gyro_xgain,o_ctrl->towrite_gyro_ygain);
        o_ctrl->delay_write_gain = 0;
        mini_isp_ois_setGyroGain(o_ctrl->module_id,o_ctrl->towrite_gyro_xgain,o_ctrl->towrite_gyro_ygain);
    }

    //send miniisp cmd to init ois otp data
    rc = mini_isp_ois_turn_on(o_ctrl->module_id,onoff, o_ctrl->turn_on_type);
    //rc = 0;
    pr_info("%s: Exit\n",__func__);
	return rc;
}

/*adjust back camera resolution and optimize ois driver*/
static int32_t msm_ois_running_test(struct msm_ois_ctrl_t *o_ctrl, int times)
{
	int rc = 0;
    if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
    
    pr_info("%s: Enter times=%d\n",__func__,times);

    //for(i = 0; i<times; i++)
    {
        rc = mini_isp_ois_start_running_test(o_ctrl->module_id);
    }

    if(rc != 0)
    {
        pr_err("%s: fail \n",__func__);
        o_ctrl->check_ois = 0;
    }
    else
    {
        o_ctrl->check_ois = 1;
    }
    o_ctrl->ois_wait_end = 1;
    wake_up_interruptible(&o_ctrl->wait_check_ois);

    pr_info("%s: Exit\n",__func__);

    return 0;
}
/*delete msm_ois_mmi_test function, no use*/
static int32_t msm_ois_mag_test(struct msm_ois_ctrl_t *o_ctrl, struct msm_ois_cfg_data *cdata)
{
    int32_t ret = 0;

    if (!o_ctrl || !cdata) {
        pr_err("failed\n");
        return -EINVAL;
    }

    pr_info("%s: Enter module_id=%d\n",__func__,o_ctrl->module_id);

    ret = mini_isp_ois_start_mag_test(o_ctrl->module_id, cdata);
    ret = msm_ois_turn_onoff(o_ctrl,1);

    pr_info("%s: Exit\n",__func__);
    return ret;
}


static int32_t msm_ois_control(struct msm_ois_ctrl_t *o_ctrl,
	struct msm_ois_set_info_t *set_info)
{
	struct reg_settings_ois_t *settings = NULL;
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = o_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->ois_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = o_ctrl->cci_master;
	} else {
		o_ctrl->i2c_client.client->addr =
			set_info->ois_params.i2c_addr;
	}
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;


	if (set_info->ois_params.setting_size > 0 &&
		set_info->ois_params.setting_size
		< MAX_OIS_REG_SETTINGS) {
		settings = kmalloc(
			sizeof(struct reg_settings_ois_t) *
			(set_info->ois_params.setting_size),
			GFP_KERNEL);
		if (settings == NULL) {
			pr_err("Error allocating memory\n");
			return -EFAULT;
		}
		if (copy_from_user(settings,
			(void *)set_info->ois_params.settings,
			set_info->ois_params.setting_size *
			sizeof(struct reg_settings_ois_t))) {
			kfree(settings);
			pr_err("Error copying\n");
			return -EFAULT;
		}

		rc = msm_ois_write_settings(o_ctrl,
			set_info->ois_params.setting_size,
			settings);
		kfree(settings);
		if (rc < 0) {
			pr_err("Error\n");
			return -EFAULT;
		}
	}

	CDBG("Exit\n");

	return rc;
}

/*delete invalid codes*/
/*add camera ois driver*/
static int32_t msm_ois_config(struct msm_ois_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_ois_cfg_data *cdata =
		(struct msm_ois_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(o_ctrl->ois_mutex);
	pr_info("Enter\n");
	pr_info("type %d\n", cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_OIS_INIT:
	    pr_info("start ois init work vendor=0x%0x \n",cdata->vendor_id);
        mini_isp_init_exit_flag(0);
        /*mini-isp do the ois init*/
		o_ctrl->ois_onoff = 0;
        o_ctrl->module_id = cdata->vendor_id;
		/*no need turn on ois in here, instead of apk param*/
		o_ctrl->gyro_xgain = o_ctrl->gyro_ygain = 0;
		o_ctrl->towrite_gyro_xgain = o_ctrl->towrite_gyro_ygain = 0;
		o_ctrl->delay_write_gain = 0;
		break;
	case CFG_OIS_POWERDOWN:
		/*adjust back camera resolution and optimize ois driver*/
		pr_info("stop ois work start\n");
		mini_isp_init_exit_flag(1);
/*mini-isp do the ois init*/
#ifdef USE_MINI_ISP_INIT_OIS
		misp_set_ois_initstatus(0);
#endif
		cancel_delayed_work_sync(&o_ctrl->ois_init_work);
		pr_info("stop ois work end\n");
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_power_down failed %d\n", rc);
		break;
	case CFG_OIS_POWERUP:
		rc = msm_ois_power_up(o_ctrl);
		if (rc < 0)
			pr_err("Failed ois power up%d\n", rc);
		break;
	case CFG_OIS_CONTROL:
		rc = msm_ois_control(o_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("Failed ois control%d\n", rc);
		break;

#ifdef _USE_MINI_ISP
    case CFG_OIS_SETOTP:
    /*mini-isp do the ois init*/
        break;
    case CFG_OIS_TURN_ON_LIN:
        o_ctrl->turn_on_type = 1;
#ifdef USE_MINI_ISP_INIT_OIS
        if(!misp_get_ois_initstatus())
#endif
        {
            pr_err("CFG_OIS_TURN_ON ois don't init complete! \n");
            pr_info("CFG_OIS_TURN_ON when ois no init, delay do turn on\n");
            o_ctrl->ois_work_type = OIS_TURN_ON_WORK;
            schedule_delayed_work(&o_ctrl->ois_init_work, msecs_to_jiffies(10));
            break;
        }
        mini_isp_init_exit_flag(0);
        rc = msm_ois_turn_onoff(o_ctrl,1);
        if (rc < 0)
            pr_err("msm_ois_turn_on failed %d\n", rc);
        break;
    case CFG_OIS_TURN_ON:
/*mini-isp do the ois init*/
o_ctrl->turn_on_type = 0;
#ifdef USE_MINI_ISP_INIT_OIS
		if(!misp_get_ois_initstatus())
#endif
        {
            pr_err("CFG_OIS_TURN_ON ois don't init complete! \n");
			/*optimize ois turn on/off interface in driver*/
            /*mini-isp do the ois init*/
			/*when need turn on ois if no init done. delay to check init done statu*/
			pr_info("CFG_OIS_TURN_ON when ois no init, delay do turn on\n");
			o_ctrl->ois_work_type = OIS_TURN_ON_WORK;
			schedule_delayed_work(&o_ctrl->ois_init_work, msecs_to_jiffies(10));
            break;
        }
        /*adjust back camera resolution and optimize ois driver*/
        mini_isp_init_exit_flag(0);
        rc = msm_ois_turn_onoff(o_ctrl,1);
        if (rc < 0)
			pr_err("msm_ois_turn_on failed %d\n", rc);
        break;
    case CFG_OIS_TURN_OFF:
/*mini-isp do the ois init*/
#ifdef USE_MINI_ISP_INIT_OIS
	 if(!misp_get_ois_initstatus())
#endif
        {
            pr_err("%s: CFG_OIS_TURN_OFF ois don't init complete! \n",__func__);
			/*optimize ois turn on/off interface in driver*/
            /*mini-isp do the ois init*/
            break;
        }
        /*adjust back camera resolution and optimize ois driver*/
        mini_isp_init_exit_flag(0);
        rc = msm_ois_turn_onoff(o_ctrl,0);
        if (rc < 0)
			pr_err("msm_ois_turn_off failed %d\n", rc);
        break;
    case CFG_OIS_START_RUNNINGTEST:
/*mini-isp do the ois init*/
        pr_info("vendor_id = %d \n",cdata->vendor_id);
        o_ctrl->ois_work_type = OIS_RUNNING_TEST_WORK;
        o_ctrl->runningtest_parm = cdata->vendor_id;
        schedule_delayed_work(&o_ctrl->ois_init_work, msecs_to_jiffies(10));
        break;
    case CFG_OIS_START_MMI_CHECK:
        /*now CFG_OIS_START_MMI_CHECK no use*/
        break;
    case CFG_OIS_START_MAG_CHECK:
#ifdef USE_MINI_ISP_INIT_OIS
        if(!msm_ois_check_mini_isp_init_status())
#endif
        {
            pr_err("%s: CFG_OIS_START_MAG_CHECK ois don't init complete! \n",__func__);
            break;
        }
        mini_isp_init_exit_flag(0);
        rc = msm_ois_mag_test(o_ctrl, cdata);
        if (rc < 0)
            pr_err("CFG_OIS_START_MAG_CHECK failed %d\n", rc);
        break;
#endif
    case CFG_OIS_GET_GYROGAIN:
        {
            struct msm_ois_gyrogain_info_t gain_info = {0};
            void *ptr_dest = NULL;
            struct msm_sensor_otp_info * otp_info = NULL;
            uint8_t *data = NULL;
            if(!misp_get_ois_initstatus())
            {
                pr_info("%s: CFG_OIS_GET_GYROGAIN ois don't init complete! get gain from otp\n",__func__);
                otp_info = get_otp_info_by_moduleid(o_ctrl->module_id);
                if(!otp_info)
                {
                    pr_err("%s: can't get otp info fail!\n",__func__);
                    break;
                }

                data = (uint8_t *)otp_info->ois_otp.aucOIS;
                gain_info.gyrogainx = (data[25] << 24)|(data[26] << 16)|(data[27] << 8)|data[28];// gxzoom UlGxgVal
                gain_info.gyrogainy = (data[29] << 24)|(data[30] << 16)|(data[31] << 8)|data[32];// gyzoom UlGygVal 
            }
            else
            {
                mini_isp_init_exit_flag(0);
                mini_isp_ois_getGyroGain(o_ctrl->module_id,&gain_info.gyrogainx,&gain_info.gyrogainy);
            }

            pr_info("%s: get gyro gain %x %x\n",__func__,gain_info.gyrogainx,gain_info.gyrogainy);
            
            ptr_dest = (void *) cdata->cfg.gyro_gain_info;
            rc  = copy_to_user((void __user *)ptr_dest, &gain_info, sizeof(gain_info));
            if(rc)
            {
                pr_err("%s cannot copy error rc=%d \n", __func__, rc);
            }
        }
        break;
    case CFG_OIS_SET_GYROGAIN:
        {
            o_ctrl->towrite_gyro_xgain = cdata->cfg.gyro_gain_info->towritegyrogainx;
            o_ctrl->towrite_gyro_ygain = cdata->cfg.gyro_gain_info->towritegyrogainy;

            o_ctrl->delay_write_gain = 0;
            if(!misp_get_ois_initstatus())
            {
                o_ctrl->delay_write_gain = 1;
                pr_err("%s: CFG_OIS_SET_GYROGAIN ois don't init complete! \n",__func__);
                break;
            }

            pr_info("%s: to write new xgain=0x%08x ygain=0x%08x \n",__func__,
                o_ctrl->towrite_gyro_xgain,o_ctrl->towrite_gyro_xgain);

            mini_isp_init_exit_flag(0);
            mini_isp_ois_setGyroGain(o_ctrl->module_id,o_ctrl->towrite_gyro_xgain,o_ctrl->towrite_gyro_ygain);
        }
        break;
	case CFG_OIS_I2C_WRITE_SEQ_TABLE: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			memcpy(&conf_array,
				(void *)cdata->cfg.settings,
				sizeof(struct msm_camera_i2c_seq_reg_setting));
		} else
#endif
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.settings,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = o_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_seq_table(&o_ctrl->i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}
	default:
		break;
	}
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_get_subdev_id(struct msm_ois_ctrl_t *o_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = o_ctrl->pdev->id;
	else
		*subdev_id = o_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll = msm_camera_qup_i2c_poll,
};

static int msm_ois_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_ois_ctrl_t *o_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
/*add camera ois driver*/
#ifndef _USE_MINI_ISP
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&o_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
#endif
	CDBG("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_ois_internal_ops = {
	.close = msm_ois_close,
};

static long msm_ois_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d o_ctrl %p argp %p\n", __func__, __LINE__, o_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_ois_get_subdev_id(o_ctrl, argp);
	case VIDIOC_MSM_OIS_CFG:
		return msm_ois_config(o_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		msm_ois_close(sd, NULL);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	rc = msm_ois_vreg_control(o_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	o_ctrl->ois_state = OIS_POWER_UP;
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	mutex_lock(o_ctrl->ois_mutex);
	if (on)
		rc = msm_ois_power_up(o_ctrl);
	else
		rc = msm_ois_power_down(o_ctrl);
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_ois_subdev_core_ops = {
	.ioctl = msm_ois_subdev_ioctl,
	.s_power = msm_ois_power,
};

static struct v4l2_subdev_ops msm_ois_subdev_ops = {
	.core = &msm_ois_subdev_core_ops,
};

static const struct i2c_device_id msm_ois_i2c_id[] = {
	{"qcom,ois", (kernel_ulong_t)NULL},
	{ }
};

static int32_t msm_ois_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_ois_ctrl_t *ois_ctrl_t = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("msm_ois_i2c_probe: client is null\n");
		rc = -EINVAL;
		goto probe_failure;
	}

	ois_ctrl_t = kzalloc(sizeof(struct msm_ois_ctrl_t),
		GFP_KERNEL);
	if (!ois_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	CDBG("client = 0x%p\n",  client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&ois_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", ois_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	ois_ctrl_t->i2c_driver = &msm_ois_i2c_driver;
	ois_ctrl_t->i2c_client.client = client;
	/* Set device type as I2C */
	ois_ctrl_t->ois_device_type = MSM_CAMERA_I2C_DEVICE;
	ois_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	ois_ctrl_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
	ois_ctrl_t->ois_mutex = &msm_ois_mutex;

	/* Assign name for sub device */
	snprintf(ois_ctrl_t->msm_sd.sd.name, sizeof(ois_ctrl_t->msm_sd.sd.name),
		"%s", ois_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&ois_ctrl_t->msm_sd.sd,
		ois_ctrl_t->i2c_client.client,
		ois_ctrl_t->ois_v4l2_subdev_ops);
	v4l2_set_subdevdata(&ois_ctrl_t->msm_sd.sd, ois_ctrl_t);
	ois_ctrl_t->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	ois_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&ois_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	ois_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	ois_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	ois_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&ois_ctrl_t->msm_sd);
	ois_ctrl_t->ois_state = OIS_POWER_DOWN;
	pr_info("msm_ois_i2c_probe: succeeded\n");
	CDBG("Exit\n");

probe_failure:
	return rc;
}

#ifdef CONFIG_COMPAT
static long msm_ois_subdev_ioctl32(
	struct file *file, unsigned int cmd, void *arg)
{
	long rc = 0;
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct msm_ois_cfg_data32 *u32 =
		(struct msm_ois_cfg_data32 *)arg;
	struct msm_ois_cfg_data ois_data;
	void *parg = arg;
	struct msm_camera_i2c_seq_reg_setting settings;
	struct msm_camera_i2c_seq_reg_setting32 settings32;

	ois_data.cfgtype = u32->cfgtype;
/*add camera ois driver*/
    ois_data.vendor_id = u32->vendor_id;

	switch (cmd) {
	case VIDIOC_MSM_OIS_CFG32:
		cmd = VIDIOC_MSM_OIS_CFG;

		switch (u32->cfgtype) {
		case CFG_OIS_CONTROL:
			ois_data.cfg.set_info.ois_params.setting_size =
				u32->cfg.set_info.ois_params.setting_size;
			ois_data.cfg.set_info.ois_params.i2c_addr =
				u32->cfg.set_info.ois_params.i2c_addr;
			ois_data.cfg.set_info.ois_params.i2c_addr_type =
				u32->cfg.set_info.ois_params.i2c_addr_type;
			ois_data.cfg.set_info.ois_params.i2c_data_type =
				u32->cfg.set_info.ois_params.i2c_data_type;
			ois_data.cfg.set_info.ois_params.settings =
				compat_ptr(u32->cfg.set_info.ois_params.
				settings);
			parg = &ois_data;
			break;
		case CFG_OIS_I2C_WRITE_SEQ_TABLE:
			if (copy_from_user(&settings32,
				(void *)compat_ptr(u32->cfg.settings),
				sizeof(
				struct msm_camera_i2c_seq_reg_setting32))) {
				pr_err("copy_from_user failed\n");
				return -EFAULT;
			}

			settings.addr_type = settings32.addr_type;
			settings.delay = settings32.delay;
			settings.size = settings32.size;
			settings.reg_setting =
				compat_ptr(settings32.reg_setting);

			ois_data.cfgtype = u32->cfgtype;
			ois_data.cfg.settings = &settings;
			parg = &ois_data;
			break;
		case CFG_OIS_START_MAG_CHECK:
			ois_data.cfg.mag_info = compat_ptr(u32->cfg.mag_info);
			parg = &ois_data;
			break;
		case CFG_OIS_SET_GYROGAIN:
		case CFG_OIS_GET_GYROGAIN:
			ois_data.cfg.gyro_gain_info = compat_ptr(u32->cfg.gyro_gain_info);
			parg = &ois_data;
			break;
		default:
			parg = &ois_data;
			break;
		}
	}
	rc = msm_ois_subdev_ioctl(sd, cmd, parg);

	return rc;
}
static long msm_ois_subdev_fops_ioctl32(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_ois_subdev_ioctl32);
}
#endif

//for OT adjust gyro gain
static ssize_t msm_mmi_ois_gyrogain_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t msm_mmi_ois_gyrogain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ois_gyrogain, 0644, msm_mmi_ois_gyrogain_show, msm_mmi_ois_gyrogain_store);
static ssize_t msm_mmi_ois_gyrogain_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)

{
	/*format: "%d %d" */
	int ret = 0;
	struct msm_ois_ctrl_t *pdata = NULL;
	const char *pos;

	pdata = (struct msm_ois_ctrl_t *)dev_get_drvdata(dev);
	if (NULL==pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}

	pr_info("%s - enter", __func__);

	if (count == 0)
		return 0;

	if(!misp_get_ois_initstatus())
	{
		pr_err("%s: ois don't init complete! \n",__func__);
		return (ssize_t)count;
	}

	pos = buf;

	ret = sscanf(pos, "%8x %8x", &pdata->towrite_gyro_xgain, &pdata->towrite_gyro_ygain);
	if(ret <= 0)		
	{
		pr_err("%s: sscanf fail\n",__func__);
		return (ssize_t)count;
	}

	if(pdata->towrite_gyro_xgain != 0 && pdata->towrite_gyro_ygain != 0)
	{
		ret = mini_isp_ois_setGyroGain(pdata->module_id,pdata->towrite_gyro_xgain,pdata->towrite_gyro_ygain);
	}

	pr_info("%s: towrite_gyro_xgain=%8x towrite_gyro_ygain=%8x \n",__func__,pdata->towrite_gyro_xgain,pdata->towrite_gyro_ygain);
	pr_info("%s - X", __func__);
	return (ssize_t)count;
}

static ssize_t msm_mmi_ois_gyrogain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct msm_ois_ctrl_t *pdata = NULL;
	char *offset;

	pr_info("%s - enter", __func__);

	pdata = (struct msm_ois_ctrl_t *)dev_get_drvdata(dev);
	if (NULL==pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}

	if(!misp_get_ois_initstatus())
	{
		pr_err("%s: ois don't init complete! \n",__func__);
		return 0;
	}

	/* store gyro gain buff */
	offset = buf;

	ret = mini_isp_ois_getGyroGain(pdata->module_id, &pdata->gyro_xgain, &pdata->gyro_ygain);
	if(ret)
	{
		pr_err("mini_isp_ois_getGyroGain error! ");
	}

	ret = snprintf(offset, PAGE_SIZE, "%8x %8x", pdata->gyro_xgain,pdata->gyro_ygain);

	offset += ret;

	pr_info("%s - X", __func__);

	return (offset-buf);
}

/*add camera ois driver*/
/*
 * ois device node for mmi test to check ois result
*/
static ssize_t msm_mmi_ois_pixel_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t msm_mmi_ois_pixel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ois_pixel, 0644, msm_mmi_ois_pixel_show, msm_mmi_ois_pixel_store);

static ssize_t msm_mmi_ois_pixel_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)

{
    /*format: "%d %d" */
    int ret = 0;
	struct msm_ois_ctrl_t *pdata = NULL;
    const char *pos;

    pdata = (struct msm_ois_ctrl_t *)dev_get_drvdata(dev);
	if (NULL==pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}
    
    pr_info("%s: enter!\n",__func__);

    if (count == 0)
		return 0;

    pos = buf;

    ret = sscanf(pos, "%d %d", &pdata->x_width, &pdata->y_height);		
    if(ret <= 0)		
    {  
       pr_err("%s: sscanf fail\n",__func__);
       return 0;       
    }

    pr_info("%s: x=%d y=%d \n",__func__,pdata->x_width,pdata->y_height);
    
    return (ssize_t)count;
}

static ssize_t msm_mmi_ois_pixel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	struct msm_ois_ctrl_t *pdata = NULL;
	char *offset;

	pr_info("%s - enter", __func__);

	pdata = (struct msm_ois_ctrl_t *)dev_get_drvdata(dev);
	if (NULL==pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}
    
	/* show mmi ois result */
	offset = buf;

    pr_err("%s: x=%d y=%d \n",__func__,pdata->x_width,pdata->y_height);

    ret = snprintf(offset, PAGE_SIZE, "%d %d", pdata->x_width,pdata->y_height);

    offset += ret;

    pr_info("%s - X", __func__);

    return (offset-buf);
    
}

/*
 * ois device node for running test to check ois result
*/
static ssize_t msm_check_ois_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t msm_check_ois_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(check_ois, 0644, msm_check_ois_show, msm_check_ois_store);
#define MISP_CHECK_OIS_TIMEOUT (HZ*8)

static ssize_t msm_check_ois_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)

{
    
    pr_info("%s: nothing to do!\n",__func__);
    if (count == 0)
		return 0;

    return (ssize_t)count;
}

static ssize_t msm_check_ois_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	struct msm_ois_ctrl_t *pdata = NULL;
	char *offset;

	pr_info("%s - enter", __func__);

	pdata = (struct msm_ois_ctrl_t *)dev_get_drvdata(dev);
	if (NULL==pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}

    /*
        pdata->runningtest_check_ois:
        1 ---> ois check pass
        2 ---> ois check fail
    */
    ret = wait_event_interruptible_timeout(pdata->wait_check_ois,\
        (pdata->ois_wait_end != 0),\
        MISP_CHECK_OIS_TIMEOUT);

	/* show check ois result */
	offset = buf;

    pr_err("%s: check_ois=%u ret=%d \n",__func__,pdata->check_ois,ret);

    if(!ret)
    {
        pdata->check_ois = 0; //notify apk ois test timeout
    }

    ret = snprintf(offset, PAGE_SIZE, "%d", pdata->check_ois);

    offset += ret;

    pdata->ois_wait_end = 0;

    pr_info("%s - X", __func__);

    return (offset-buf);
}

#ifdef USE_MINI_ISP_INIT_OIS
static int msm_ois_check_mini_isp_init_status(void)
{
	int32_t retry_count = 5;
	int32_t sleep_time = 500;

	while(retry_count)
	{
	if(misp_get_ois_initstatus())
		return 1;

	msleep(sleep_time);
	retry_count--;
	}
	return 0;
}
#endif

static void msm_ois_init_work_handler(struct work_struct *work)
{
    int32_t rc = 0;
    struct msm_ois_ctrl_t *o_ctrl = container_of(work, struct msm_ois_ctrl_t,ois_init_work.work);
    
    if(!o_ctrl)
        return;

    /*adjust back camera resolution and optimize ois driver*/
    mini_isp_init_exit_flag(0);
/*mini-isp do the ois init*/
#ifdef USE_MINI_ISP_INIT_OIS
	 //now need to default open ois 
	 if(!msm_ois_check_mini_isp_init_status())
		{
			//dump the misp log
            if(huawei_cam_is_factory_mode())
            {
                misp_flush_log();
            }
			pr_err("%s until now mini-isp init ois uncompleted!\n", __func__);
			return;
		}

	/*fix ois running test fail issue*/
	if(o_ctrl->ois_work_type == OIS_TURN_OFF_WORK)
	{
		rc = msm_ois_turn_onoff(o_ctrl,0);
		if (rc < 0)
		{
			pr_err("%s msm_ois_turn_on failed %d\n", __func__, rc);
		}
		else
		{
			pr_err("%s msm_ois_turn_on success %d\n", __func__, rc);
		}
	}
	else
	{
		rc = msm_ois_turn_onoff(o_ctrl,1);
		if (rc < 0)
		{
			pr_err("%s msm_ois_turn_on failed %d\n", __func__, rc);
		}
		else
		{
			pr_err("%s msm_ois_turn_on success %d\n", __func__, rc);
		}
	}
    /*delete invalid codes*/
#endif
		if(o_ctrl->ois_work_type == OIS_RUNNING_TEST_WORK)
    {
        rc = msm_ois_running_test(o_ctrl, o_ctrl->runningtest_parm);
    }
}
/*add camera ois driver*/
static int32_t msm_ois_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_ois_ctrl_t *msm_ois_t = NULL;
	struct msm_ois_vreg *vreg_cfg;
	pr_info("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	msm_ois_t = kzalloc(sizeof(struct msm_ois_ctrl_t),
		GFP_KERNEL);
	if (!msm_ois_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		kfree(msm_ois_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_ois_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_ois_t->cci_master, rc);
	if (rc < 0) {
		kfree(msm_ois_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	if (of_find_property((&pdev->dev)->of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &msm_ois_t->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data((&pdev->dev)->of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(msm_ois_t);
			pr_err("failed rc %d\n", rc);
			return rc;
		}
	}

	msm_ois_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
	msm_ois_t->ois_mutex = &msm_ois_mutex;

	/* Set platform device handle */
	msm_ois_t->pdev = pdev;
	/* Set device type as platform device */
	msm_ois_t->ois_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_ois_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_ois_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_ois_t->i2c_client.cci_client) {
		kfree(msm_ois_t->vreg_cfg.cam_vreg);
		kfree(msm_ois_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = msm_ois_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = MASTER_MAX;
	v4l2_subdev_init(&msm_ois_t->msm_sd.sd,
		msm_ois_t->ois_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_ois_t->msm_sd.sd, msm_ois_t);
	msm_ois_t->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	msm_ois_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_ois_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_ois_t->msm_sd.sd.name), "msm_ois");
	media_entity_init(&msm_ois_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_ois_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_ois_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	msm_ois_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_ois_t->msm_sd);
	msm_ois_t->ois_state = OIS_POWER_DOWN;
	msm_ois_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_ois_v4l2_subdev_fops.compat_ioctl32 =
		msm_ois_subdev_fops_ioctl32;
#endif
	msm_ois_t->msm_sd.sd.devnode->fops =
		&msm_ois_v4l2_subdev_fops;

    INIT_DELAYED_WORK(&msm_ois_t->ois_init_work, msm_ois_init_work_handler);

    init_waitqueue_head(&msm_ois_t->wait_check_ois);
    msm_ois_t->ois_wait_end = 0;

    //create ois runningtest device node
    rc = device_create_file(&pdev->dev, &dev_attr_check_ois);
	if (rc) {
		pr_err("probe - create check ois dev attr file fail");
		rc = 0;
	}
    
    //create ois OT pixel device node
    rc = device_create_file(&pdev->dev, &dev_attr_ois_pixel);
	if (rc) {
		pr_err("probe - create ois pixel dev attr file fail");
		rc = 0;
	}

	//create ois gyro gain device node
	rc = device_create_file(&pdev->dev, &dev_attr_ois_gyrogain);
	if (rc) {
		pr_err("probe - create ois gyro gain dev attr file fail");
		rc = 0;
	}
	/* set driver_data to device */
    platform_set_drvdata(pdev, msm_ois_t);

	pr_info("Exit\n");
	return rc;
}

static const struct of_device_id msm_ois_i2c_dt_match[] = {
	{.compatible = "qcom,ois"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_ois_i2c_dt_match);

static struct i2c_driver msm_ois_i2c_driver = {
	.id_table = msm_ois_i2c_id,
	.probe  = msm_ois_i2c_probe,
	.remove = __exit_p(msm_ois_i2c_remove),
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_ois_i2c_dt_match,
	},
};

static const struct of_device_id msm_ois_dt_match[] = {
	{.compatible = "qcom,ois", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_ois_dt_match);

static struct platform_driver msm_ois_platform_driver = {
	.probe = msm_ois_platform_probe,
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_ois_dt_match,
	},
};

static int __init msm_ois_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_register(&msm_ois_platform_driver);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_ois_i2c_driver);
}

static void __exit msm_ois_exit_module(void)
{
	platform_driver_unregister(&msm_ois_platform_driver);
	i2c_del_driver(&msm_ois_i2c_driver);
	return;
}

module_init(msm_ois_init_module);
module_exit(msm_ois_exit_module);
MODULE_DESCRIPTION("MSM OIS");
MODULE_LICENSE("GPL v2");
