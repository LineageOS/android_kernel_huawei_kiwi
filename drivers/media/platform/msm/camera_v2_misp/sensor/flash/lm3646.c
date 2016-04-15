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

#define FLASH_NAME "ti,lm3646"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define LM3646_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LM3646_DBG(fmt, args...)
#endif

#define FLASH_CHIP_ID_MASK 0x38
#define FLASH_CHIP_ID 0x10

//Register Addr
#define ENABLE_REGISTER 0x01
#define REG_FLASH_TIMEOUT		0x04
#define REG_MAX_CURRENT         0x05
#define REG_LED1_FLASH_CURRENT_CONTROL		0x06
#define REG_LED1_TORCH_CURRENT_CONTROL		0x07
#define REG_FLAGS1			0x08
#define REG_FLAGS2			0x09

//Enable Register Mode
#define MODE_BIT_MASK 0x03
#define MODE_BIT_STANDBY_DEFAULT 0x00
#define MODE_BIT_STANDBY 0x01
#define MODE_BIT_TORCH 0x02
#define MODE_BIT_FLASH 0x03

//LED1 Flash Current Control Register (0x06)
#define ENABLE_BIT_FLASH 0x7F //enable bit control

//LED1 Torch Current Control Register (0x07)
#define ENABLE_BIT_TORCH 0x7F //enable bit control

#define SET_TORCH_MODE (INDUCTOR_CURRENT_LIMMIT | MODE_BIT_TORCH)
#define SET_FLASH_MODE (INDUCTOR_CURRENT_LIMMIT | MODE_BIT_FLASH)
#define SET_MAX_CURRENT 0x7c
#define FLASH_TIMEOUT_TIME        0x47    //400ms

#define INDUCTOR_CURRENT_LIMMIT 0xe0

#define FLASH_CURRENT_MAX 335
#define FLASH_CURRENT_MIN 0

//delete some lines

struct hw_lm3646_flash_level_matrix{
        unsigned int flash_level_min;
        unsigned int flash_level_max;

        unsigned char max_current_flash;
        unsigned char max_current_torch;
};

/*
预闪总电流(mA)	强闪总电流(mA)	电流组合数
187.2					1500					128
140.2					1124.9					96
93.4						749.8					64
46.5						374.7					32
23.1						187.2					16
*/
static struct hw_lm3646_flash_level_matrix hw_lm3646_flash_level[5] = {
    {0,       15,       1,        0},
    {16,     47,       3,         1},
    {48,    111,       7,        3},
    {112,  207,       11,       5},
    {208,  335,       15,       7},
};

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3646_i2c_driver;
static bool dual_leds_test_mode = false;
static char dual_leds_max_node = 0;


static struct msm_camera_i2c_reg_array lm3646_init_array[] = {
	{ENABLE_REGISTER, INDUCTOR_CURRENT_LIMMIT | MODE_BIT_STANDBY},
};

static struct msm_camera_i2c_reg_array lm3646_off_array[] = {
	{ENABLE_REGISTER, INDUCTOR_CURRENT_LIMMIT | MODE_BIT_STANDBY},
};

static struct msm_camera_i2c_reg_array lm3646_release_array[] = {
	{ENABLE_REGISTER, INDUCTOR_CURRENT_LIMMIT | MODE_BIT_STANDBY_DEFAULT},
};

static const struct of_device_id lm3646_i2c_trigger_dt_match[] = {
	{.compatible = "ti,lm3646"},
	{}
};

//dummy E
static struct msm_camera_i2c_reg_array lm3646_low_array[] = {
		{ENABLE_REGISTER, INDUCTOR_CURRENT_LIMMIT | MODE_BIT_STANDBY},
};

static struct msm_camera_i2c_reg_array lm3646_high_array[] = {
		{ENABLE_REGISTER, INDUCTOR_CURRENT_LIMMIT | MODE_BIT_STANDBY},
};

static struct msm_camera_i2c_reg_array lm3646_torch_array[] = {
		{ENABLE_REGISTER, INDUCTOR_CURRENT_LIMMIT | MODE_BIT_STANDBY},
};
//dummy X

MODULE_DEVICE_TABLE(of, lm3646_i2c_trigger_dt_match);
static const struct i2c_device_id lm3646_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF) 
	{
		if(fctrl.func_tbl->torch_led_on)
		{
			fctrl.torch_max_current[LED_FLASH] = value;
			fctrl.func_tbl->torch_led_on(&fctrl);
		}
	} 
	else 
	{
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static void msm_led_dual_leds_max_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if ((value & 0x80) > LED_OFF) 
	{
		dual_leds_test_mode = true;
		dual_leds_max_node = ((dual_leds_max_node&0x00) | (value&0x7f));
	}
	else
	{
		dual_leds_test_mode = false;
	}
};

//0:OFF 
//1~128:Torch 
//129~255:Flash
static void msm_led_dual_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	unsigned char mode = 0;
	int32_t currentindex = 0;
	struct msm_camera_i2c_client *i2c_client =NULL;
	struct msm_camera_i2c_fn_t * i2c_func =NULL;

	LM3646_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (value > LED_OFF) 
	{
		//struct msm_camera_led_cfg_t data = {0};
		LM3646_DBG("%s:flash_led_on\n", __func__);

		if (NULL != fctrl.flash_i2c_client)
		{
			i2c_client = fctrl.flash_i2c_client;
			
			if (NULL != fctrl.flash_i2c_client->i2c_func_tbl)
			{
				i2c_func = fctrl.flash_i2c_client->i2c_func_tbl;
			}
			else
			{
				pr_err("%s:%d fctrl.flash_i2c_client->i2c_func_tb is NULL.\n", __func__, __LINE__);
				return;
			}
		}
		else
		{
			pr_err("%s:%d fctrl.flash_i2c_client is NULL.\n", __func__, __LINE__);
			return;
		}

		mode = SET_FLASH_MODE;

		if (value <= LED_HALF+1)
		{
			mode = SET_TORCH_MODE;
			currentindex = (int32_t)value-1;
		}
		else if (value < LED_FULL)
		{
			currentindex = (int32_t)value -LED_HALF -2;
		}
		else
		{
			currentindex = (int32_t)LED_HALF;
		}

		LM3646_DBG("%s:mode=0x%x, current=%d.\n", __func__, mode, currentindex);

		i2c_func->i2c_write(i2c_client, REG_FLASH_TIMEOUT, FLASH_TIMEOUT_TIME, MSM_CAMERA_I2C_BYTE_DATA);
		i2c_func->i2c_write(i2c_client, REG_LED1_TORCH_CURRENT_CONTROL, currentindex&0x7f, MSM_CAMERA_I2C_BYTE_DATA);
		i2c_func->i2c_write(i2c_client, REG_LED1_FLASH_CURRENT_CONTROL, currentindex&0x7f, MSM_CAMERA_I2C_BYTE_DATA);

		if (dual_leds_test_mode)
		{
			i2c_func->i2c_write(i2c_client, REG_MAX_CURRENT, dual_leds_max_node, MSM_CAMERA_I2C_BYTE_DATA);
		}
		else
		{
			i2c_func->i2c_write(i2c_client, REG_MAX_CURRENT, SET_MAX_CURRENT, MSM_CAMERA_I2C_BYTE_DATA);
		}
		
		i2c_func->i2c_write(i2c_client, ENABLE_REGISTER, mode, MSM_CAMERA_I2C_BYTE_DATA);
	} 
	else
	{
		LM3646_DBG("%s:flash_led_off\n", __func__);
		
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
}


static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

//"/sys/class/leds/dual_leds"
static struct led_classdev msm_dual_leds = {
	.name			= "dual_leds",
	.brightness_set	= msm_led_dual_torch_brightness_set,
	.brightness		= LED_OFF,
};

//"Max Current set"
static struct led_classdev msm_dual_leds_max = {
	.name			= "dual_leds_max",
	.brightness_set	= msm_led_dual_leds_max_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3646_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};


static int32_t msm_lm3646_dual_leds_max_create_classdev(struct device *dev ,
				void *data)
{
	int rc;

	rc = led_classdev_register(dev, &msm_dual_leds_max);
	
	if (rc) 
	{
		pr_err("Failed to register dual_leds_max dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

//"/sys/class/leds/dual_leds"
static int32_t msm_lm3646_dual_leds_create_classdev(struct device *dev ,
				void *data)
{
	int rc;

	rc = led_classdev_register(dev, &msm_dual_leds);
	
	if (rc) 
	{
		pr_err("Failed to register dual_leds dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};


/****************************************************************************
* FunctionName: msm_lm3646_clear_err_and_unlock;
* Description : clear the error and unlock the IC ;
* NOTE: this funtion must be called before register is read and write
***************************************************************************/
int msm_lm3646_clear_err_and_unlock(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	uint16_t reg_value1=0;
	uint16_t reg_value2=0;

	LM3646_DBG("%s entry\n", __func__);

	if (!fctrl) 
	{
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}


	if (fctrl->flash_i2c_client) 
	{
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client,
													REG_FLAGS1,
													&reg_value1, 
													MSM_CAMERA_I2C_BYTE_DATA);
		
		if (rc < 0)
		{
			pr_err("clear err and unlock %s:%d REG_FLAGS1 failed\n", __func__, __LINE__);
		}

		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(fctrl->flash_i2c_client,
													REG_FLAGS2,
													&reg_value2, 
													MSM_CAMERA_I2C_BYTE_DATA);
		
		if (rc < 0)
		{
			pr_err("clear err and unlock %s:%d REG_FLAGS2 failed\n", __func__, __LINE__);
		}

		LM3646_DBG("clear err and unlock success:REG_FLAGS1 is %02x\n",reg_value1);
		LM3646_DBG("clear err and unlock success:REG_FLAGS2 is %02x\n",reg_value2);
	}
	else
	{
		pr_err("%s:%d flash_i2c_client NULL\n", __func__, __LINE__);
		return -EINVAL;
	}


	return 0;

}

int msm_flash_lm3646_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	LM3646_DBG("%s:%d called\n", __func__, __LINE__);

	//clear the err and unlock IC, this function must be called before read and write register
	msm_lm3646_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) 
	{
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(fctrl->flash_i2c_client,
															fctrl->reg_setting->init_setting);

		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	
	return rc;
}

int msm_flash_lm3646_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	LM3646_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (!fctrl) 
	{
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_lm3646_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) 
	{
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(fctrl->flash_i2c_client,
															fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	
	return 0;
}

int msm_flash_lm3646_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	LM3646_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) 
	{
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_lm3646_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) 
	{
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(fctrl->flash_i2c_client,
															fctrl->reg_setting->off_setting);
		
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_lm3646_led_on(struct msm_led_flash_ctrl_t *fctrl, void * pdata)
{
	int i = 0;
	int rc = -1;
	int current_index = 0;
	unsigned char mode = 0;
	unsigned char regmaxcurrent = 0;
	unsigned char regcurrentflash = 0;
	unsigned char regcurrenttorch = 0;
	struct hw_lm3646_flash_level_matrix* matrix = NULL;
	struct msm_camera_i2c_client *i2c_client =NULL;
	struct msm_camera_i2c_fn_t * i2c_func =NULL;

	struct msm_camera_led_cfg_t *data = (struct msm_camera_led_cfg_t *)pdata;

	LM3646_DBG("%s:%d called\n", __func__, __LINE__);
	
	if ( (NULL == fctrl) || (NULL == data) )
	{
		pr_err("%s:%d fctrl or data is NULL.\n", __func__, __LINE__);
		return rc;
	}

	LM3646_DBG("%s:E DualLedMode=%d, CurrentIndex=%d.\n", __func__, data->DualLedMode, data->CurrentIndex);

	if (NULL != fctrl->flash_i2c_client)
	{
		i2c_client = fctrl->flash_i2c_client;
		
		if (NULL != fctrl->flash_i2c_client->i2c_func_tbl)
		{
			i2c_func = fctrl->flash_i2c_client->i2c_func_tbl;
		}
		else
		{
			pr_err("%s:%d fctrl->flash_i2c_client->i2c_func_tbl is NULL.\n", __func__, __LINE__);
			return rc;
		}
	}
	else
	{
		pr_err("%s:%d fctrl->flash_i2c_client is NULL.\n", __func__, __LINE__);
		return rc;
	}

	current_index = data->CurrentIndex;
	if(current_index > FLASH_CURRENT_MAX)
	{
		current_index = FLASH_CURRENT_MAX;
		pr_err("%s user set flash CurrentIndex is out of range %d,set to maxval",__func__, data->CurrentIndex);
       }

	if(current_index < FLASH_CURRENT_MIN)
	{
		current_index = FLASH_CURRENT_MIN;
		pr_err("%s user set flash CurrentIndex is out of range %d,set to minval",__func__, data->CurrentIndex);
       }

       for(i = 0;i<5;i++)
	{
		if(hw_lm3646_flash_level[i].flash_level_min<= current_index 
			&& hw_lm3646_flash_level[i].flash_level_max >= current_index)
		{
			matrix = &hw_lm3646_flash_level[i];
			LM3646_DBG("%s:Select hw_lm3646_flash_level[%d].\n", __func__, i);
			break;
		}
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_lm3646_clear_err_and_unlock(fctrl);       
	if(DUAL_LED_MODE_FLASH == data->DualLedMode)
	{
		mode = SET_FLASH_MODE;
		regcurrentflash = ENABLE_BIT_FLASH & (matrix->flash_level_max - current_index);
		regmaxcurrent = matrix->max_current_flash;
		LM3646_DBG("%s:mode is SET_FLASH_MODE.\n", __func__);
	}
	else
	{
		mode = SET_TORCH_MODE;
		regcurrenttorch = ENABLE_BIT_TORCH & (matrix->flash_level_max - current_index);
		regmaxcurrent = matrix->max_current_torch<<4;
		LM3646_DBG("%s:mode is SET_TORCH_MODE.\n", __func__);
	}

	//delete some lines
	
	rc = i2c_func->i2c_write(i2c_client, REG_FLASH_TIMEOUT, FLASH_TIMEOUT_TIME, MSM_CAMERA_I2C_BYTE_DATA);
	rc = i2c_func->i2c_write(i2c_client, REG_LED1_FLASH_CURRENT_CONTROL, regcurrentflash, MSM_CAMERA_I2C_BYTE_DATA);
	rc = i2c_func->i2c_write(i2c_client, REG_LED1_TORCH_CURRENT_CONTROL, regcurrenttorch, MSM_CAMERA_I2C_BYTE_DATA);
	rc = i2c_func->i2c_write(i2c_client, REG_MAX_CURRENT, regmaxcurrent, MSM_CAMERA_I2C_BYTE_DATA);
	rc = i2c_func->i2c_write(i2c_client, ENABLE_REGISTER, mode, MSM_CAMERA_I2C_BYTE_DATA);

	LM3646_DBG("%s:X mode=0x%x.\n", __func__, mode);
	LM3646_DBG("%s regmaxcurrent = 0x%x regcurrentflash&0x7f =0x%x regcurrenttorch&0x7f = 0x%x\n",__func__,regmaxcurrent,regcurrentflash&0x7f,regcurrenttorch&0x7f);
	LM3646_DBG("%s regmaxcurrent = 0x%x regcurrentflash =0x%x regcurrenttorch = 0x%x\n",__func__,regmaxcurrent,regcurrentflash,regcurrenttorch);


	if (rc < 0)
		pr_err("%s:%d i2c_write error: %d\n", __func__, __LINE__, rc);

	return rc;
}

int msm_flash_lm3646_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	
	struct msm_camera_led_cfg_t data = {0};

	LM3646_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (NULL == fctrl)
	{
		pr_err("%s:%d fctrl is NULL.\n", __func__, __LINE__);
		return rc;
	}
	
	data.DualLedMode = DUAL_LED_MODE_TORCH;
	data.CurrentIndex = 335;

	LM3646_DBG("%s:E DualLedMode=%d, CurrentIndex=%d.\n", __func__, data.DualLedMode, data.CurrentIndex);
	
	rc = msm_flash_lm3646_led_on(fctrl, &data);

	return rc;
}
int msm_flash_lm3646_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	
	struct msm_camera_led_cfg_t data = {0};

	LM3646_DBG("%s:%d called\n", __func__, __LINE__);
	
	if (NULL == fctrl)
	{
		pr_err("%s:%d fctrl is NULL.\n", __func__, __LINE__);
		return rc;
	}
	
	data.DualLedMode = DUAL_LED_MODE_FLASH;
	data.CurrentIndex = 280;

	LM3646_DBG("%s:E DualLedMode=%d, CurrentIndex=%d.\n", __func__, data.DualLedMode, data.CurrentIndex);
	
	rc = msm_flash_lm3646_led_on(fctrl, &data);

	return rc;
}

int msm_flash_lm3646_torch(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = -1;
	unsigned char val = 0;
	struct msm_camera_i2c_client *i2c_client =NULL;
	struct msm_camera_i2c_fn_t * i2c_func =NULL;
	unsigned char maxcurrent = SET_MAX_CURRENT;

	LM3646_DBG("%s:%d called\n", __func__, __LINE__);

	if (NULL == fctrl)
	{
		pr_err("%s:%d fctrl is NULL.\n", __func__, __LINE__);
		return rc;
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_lm3646_clear_err_and_unlock(fctrl);

	if (NULL != fctrl->flash_i2c_client)
	{
		i2c_client = fctrl->flash_i2c_client;
		
		if (NULL != fctrl->flash_i2c_client->i2c_func_tbl)
		{
			i2c_func = fctrl->flash_i2c_client->i2c_func_tbl;
		}
		else
		{
			pr_err("%s:%d fctrl->flash_i2c_client->i2c_func_tbl is NULL.\n", __func__, __LINE__);
			return rc;
		}
	}
	else
	{
		pr_err("%s:%d fctrl->flash_i2c_client is NULL.\n", __func__, __LINE__);
		return rc;
	}

	LM3646_DBG("%s fctrl->df_torch_type = %d\n", __func__, fctrl->df_torch_type);

	switch (fctrl->df_torch_type)
	{
		case DUAL_LED_TORCH_WARM:
		{
			LM3646_DBG("%s fctrl->df_torch_type=DUAL_LED_TORCH_WARM\n", __func__);
			/* REG_CURRENT_CONTROL[3:0] control flash current */
			val = ((val & 0x80) | (0x7f));
		}
		break;

		case DUAL_LED_TORCH_BOTH:
		{
			LM3646_DBG("%s  fctrl->df_torch_type =DUAL_LED_TORCH_BOTH\n", __func__);
			/* REG_CURRENT_CONTROL[3:0] control flash current */
			val = ((val & 0x80) | (0x3f));
		}
		break;

		case DUAL_LED_TORCH_COLD:
			LM3646_DBG("%s fctrl->df_torch_type=DUAL_LED_TORCH_COLD\n", __func__);
			/* REG_CURRENT_CONTROL[3:0] control flash current */
			val = ((val & 0x80) | (0x0));
			break;
		default://camera torch mode
		{
			struct msm_camera_led_cfg_t cfg = {
				.DualLedMode = DUAL_LED_MODE_TORCH,
				.CurrentIndex = fctrl->df_torch_type,
				};

			if ( fctrl->flip_id )
			{
				cfg.CurrentIndex = 0x06;
				LM3646_DBG("%s: flip_id=1.\n", __func__);
			}
			return msm_flash_lm3646_led_on(fctrl, &cfg);
		}
	}

	if  ( fctrl->flip_id )
	{
		val = ( 0x80 | 0x06);
		maxcurrent = 0x0c;
		LM3646_DBG("%s:fctrl->flip_id=%d\n", __func__, fctrl->flip_id);
	}

	rc = i2c_func->i2c_write(i2c_client, REG_MAX_CURRENT, maxcurrent, MSM_CAMERA_I2C_BYTE_DATA);
	rc = i2c_func->i2c_write(i2c_client, REG_LED1_TORCH_CURRENT_CONTROL, val, MSM_CAMERA_I2C_BYTE_DATA);	
	rc = i2c_func->i2c_write(i2c_client, ENABLE_REGISTER, SET_TORCH_MODE, MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
		pr_err("%s:%d i2c_write error: %d", __func__, __LINE__, rc);
		
	return rc;
}

static int msm_flash_lm3646_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3646_DBG("%s entry\n", __func__);
	
	if (!id) {
		pr_err("msm_flash_lm3646_i2c_probe: id is NULL");
		id = lm3646_i2c_id;
	}
	//match ID inside
	rc = msm_flash_i2c_probe(client, id);

	//Create node 
	if (!rc)
	{
		//For Torch
		rc = msm_lm3646_torch_create_classdev(&(client->dev),NULL);

		//For dual_leds mode
		rc = msm_lm3646_dual_leds_create_classdev(&(client->dev),NULL);

		//For dual_leds_max
		rc = msm_lm3646_dual_leds_max_create_classdev(&(client->dev),NULL);
	}

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);

	//
	if (fctrl.pinctrl_info.use_pinctrl == true) 
	{
		pr_err("%s:%d PC:: flash pins setting to active state",__func__, __LINE__);
		
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state", __func__, __LINE__);
	}

	return rc;
}

static int msm_flash_lm3646_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3646_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(power_info->gpio_conf->cam_gpio_req_tbl,
										power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) 
	{
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) 
	{
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
								fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state", __func__, __LINE__);
	}
	return rc;
}

static void lm3624_shutdown(struct i2c_client * client)
{
    pr_err("[%s],[%d]\n", __func__, __LINE__);
    msm_flash_led_off(&fctrl);
    msm_flash_led_release(&fctrl);
}

static struct i2c_driver lm3646_i2c_driver = {
	.id_table = lm3646_i2c_id,
	.probe  = msm_flash_lm3646_i2c_probe,
	.remove = msm_flash_lm3646_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3646_i2c_trigger_dt_match,
	},
	.shutdown = lm3624_shutdown,
};

static int __init msm_flash_lm3646_init(void)
{
	LM3646_DBG("%s entry\n", __func__);
	return i2c_add_driver(&lm3646_i2c_driver);
}

static void __exit msm_flash_lm3646_exit(void)
{
	LM3646_DBG("%s entry\n", __func__);
	i2c_del_driver(&lm3646_i2c_driver);
	return;
}


static struct msm_camera_i2c_client lm3646_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3646_init_setting = {
	.reg_setting = lm3646_init_array,
	.size = ARRAY_SIZE(lm3646_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_off_setting = {
	.reg_setting = lm3646_off_array,
	.size = ARRAY_SIZE(lm3646_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_release_setting = {
	.reg_setting = lm3646_release_array,
	.size = ARRAY_SIZE(lm3646_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

//dummy E
static struct msm_camera_i2c_reg_setting lm3646_low_setting = {
	.reg_setting = lm3646_low_array,
	.size = ARRAY_SIZE(lm3646_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3646_high_setting = {
	.reg_setting = lm3646_high_array,
	.size = ARRAY_SIZE(lm3646_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};


static struct msm_camera_i2c_reg_setting lm3646_torch_setting = {
	.reg_setting = lm3646_torch_array,
	.size = ARRAY_SIZE(lm3646_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
//dummy X

static struct msm_led_flash_reg_t lm3646_regs = {
	.init_setting = &lm3646_init_setting,
	.off_setting = &lm3646_off_setting,
	.release_setting = &lm3646_release_setting,
	.low_setting = &lm3646_low_setting,//dummy
	.high_setting = &lm3646_high_setting,//dummy
	.torch_setting = &lm3646_torch_setting,//dummy
};

static struct msm_flash_fn_t lm3646_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3646_led_init,
	.flash_led_release = msm_flash_lm3646_led_release,
	.flash_led_off = msm_flash_lm3646_led_off,
	.flash_led_on = msm_flash_lm3646_led_on,
	.torch_led_on = msm_flash_lm3646_torch,
	//Only for Test
	.flash_led_low = msm_flash_lm3646_led_low,
	.flash_led_high = msm_flash_lm3646_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3646_i2c_client,
	.reg_setting = &lm3646_regs,
	.func_tbl = &lm3646_func_tbl,
};

module_init(msm_flash_lm3646_init);
module_exit(msm_flash_lm3646_exit);
MODULE_DESCRIPTION("lm3646 FLASH");
MODULE_LICENSE("GPL v2");
