/*
 *  apds9251.c - Linux kernel modules for RGB sensor
 *
 *  Copyright (C) 2015 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2015 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <huawei_platform/sensor/apds9251.h>
#include <huawei_platform/sensor/hw_sensor_info.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <misc/app_info.h>
#include <linux/debugfs.h>

#define PARSE_DTSI_NUMBER                               (34)

enum tp_color_id{
	GOLD = 0,
	WHITE,
	BLACK,
	BLUE,
	TP_COLOR_NUMBER,
};
#define MODULE_MANUFACTURE_NUMBER		3
#define VALID_FLAG							0x5555
#define APDS9251_DRV_NAME	"apds9251"
#define DRIVER_VERSION		"1.0.0"
#define APDS9251_REG_LEN 0x28
#define APDS9251_I2C_RETRY_COUNT		3 	/* Number of times to retry i2c */
#define APDS9251_I2C_RETRY_TIMEOUT	3	/* Timeout between retry (miliseconds) */
#define APDS9251_I2C_BYTE 0
#define APDS9251_I2C_WORD 1

static unsigned short apds9251_als_meas_rate_tb[] = {25, 50, 100, 200, 400};
static unsigned char apds9251_als_gain_tb[] = { 1, 3, 6, 9, 18 };
static long CCT_M = 6914;
static long CCT_N = 1735;
static long cct_compensator_H = 913;
static long cct_compensator_A = 1010;
static long cct_compensator_D = 1076;
static long LUX_P = 746;
static long LUX_Q = 714;
static long LUX_R = 641;
static long avago_cofficient[3] = {27, 18, 10};

/*dynamic debug mask to control log print,you can echo value to rgb_apds9251_debug to control*/
static int rgb_apds9251_debug_mask= 1;
module_param_named(rgb_apds9251_debug, rgb_apds9251_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define APDS9251_ERR(x...) do {\
    if (rgb_apds9251_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)

#define APDS9251_INFO(x...) do {\
    if (rgb_apds9251_debug_mask >=1) \
        printk(KERN_ERR x);\
    } while (0)
#define APDS9251_FLOW(x...) do {\
    if (rgb_apds9251_debug_mask >=2) \
        printk(KERN_ERR x);\
    } while (0)

static const char *data_array_name[MODULE_MANUFACTURE_NUMBER] = {
	[0] = "apds9251,cal_data0",
	[1] = "apds9251,cal_data1",
	[2] = "apds9251,cal_data2"
};

struct apds9251_lux_cal_parameter{
	long cct_m;
	long cct_n;
	long avago_h;
	long avago_a;
	long avago_d;
	long lux_p;
	long lux_q;
	long lux_r;
	long avago_cofficient[3];
}apds9251_lux_cal_parameter;
struct apds9251_tp_lx_cal_parameter{
	long tp_module_id;
	struct apds9251_lux_cal_parameter  gold_apds9251_lux_cal_parameter;
	struct apds9251_lux_cal_parameter  white_apds9251_lux_cal_parameter;
	struct apds9251_lux_cal_parameter  black_apds9251_lux_cal_parameter;
	struct apds9251_lux_cal_parameter  blue_apds9251_lux_cal_parameter;
}apds9251_tp_lx_cal_parameter;
static struct apds9251_tp_lx_cal_parameter apds9251_tp_module_parameter[MODULE_MANUFACTURE_NUMBER] = {{.tp_module_id = 0x55},{.tp_module_id = 0x55},{.tp_module_id = 0x55}};

typedef struct rgb_apds9251_rgb_data {
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    unsigned int ir;
    unsigned int lx;
    unsigned int cct;
} rgb_apds9251_rgb_data;

struct rgb_apds9251_data {
	struct i2c_client *client;
	/*to protect the i2c read and write operation*/
	struct mutex update_lock;
	/*to protect only one thread to control the device register*/
	struct mutex single_lock;
	struct work_struct	als_dwork;	/* for ALS polling */

	struct input_dev *input_dev_als;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	/*sensor class for als and ps*/
	struct sensors_classdev als_cdev;
	struct rgb_apds9251_platform_data *platform_data;
	struct rgb_apds9251_rgb_data rgb_data;
	int irq;
	struct hrtimer timer;
	unsigned int enable;
	unsigned int irq_control;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pers;
	unsigned int config;
	unsigned int control;
	unsigned int measure_time;
	/* control flag from HAL */
	unsigned int enable_als_sensor;
	/*to record the open or close state of als before suspend*/
	unsigned int enable_als_state;
	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	int als_prev_lux;		/* to store previous lux value */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	bool device_exist;

	unsigned int als_res_index;		/* storage for als integratiion time */
	unsigned int als_gain_index;	/* storage for als GAIN */
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "apds9251-light",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "10000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct workqueue_struct *rgb_apds9251_workqueue = NULL;
/*init the register of device function for probe and every time the chip is powered on*/
static int rgb_apds9251_init_client(struct i2c_client *client);
/*we use the unified the function for i2c write and read operation*/
static int rgb_apds9251_i2c_write(struct i2c_client*client, u8 reg, u16 value,bool flag)
{
	int err,loop;

	struct rgb_apds9251_data *data = i2c_get_clientdata(client);

	loop = APDS9251_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_write_byte_data,1 is i2c_smbus_write_word_data*/
		if(flag == APDS9251_I2C_BYTE)
		{
			err = i2c_smbus_write_byte_data(client, reg, (u8)value);
		}
		else if(flag == APDS9251_I2C_WORD)
		{
			err = i2c_smbus_write_word_data(client, reg, value);
		}
		else
		{
			APDS9251_ERR("%s,line %d:attention: i2c write wrong flag\n",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			msleep(APDS9251_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		APDS9251_ERR("%s,line %d:attention:i2c write err = %d\n",__func__,__LINE__,err);
	}

	return err;
}

static int rgb_apds9251_i2c_read(struct i2c_client*client, u8 reg,bool flag)
{
	int err,loop;

	struct rgb_apds9251_data *data = i2c_get_clientdata(client);

	loop = APDS9251_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_read_byte_data,1 is i2c_smbus_read_word_data*/
		if(flag == APDS9251_I2C_BYTE)
		{
			err = i2c_smbus_read_byte_data(client, reg);
		}
		else if(flag == APDS9251_I2C_WORD)
		{
			err = i2c_smbus_read_word_data(client, reg);
		}
		else
		{
			APDS9251_ERR("%s,line %d:attention: i2c read wrong flag\n",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			msleep(APDS9251_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		APDS9251_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x\n",__func__,__LINE__,err,reg);
	}

	return err;
}
#if 0
/*
*	print the registers value with proper format
*/
static int dump_reg_buf(struct rgb_apds9251_data *data,char *buf, int size,int enable)
{
	int i=0;

	if(enable)
		APDS9251_INFO("[enable]");
	else
		APDS9251_INFO("[disable]");
	APDS9251_INFO(" reg_buf= ");
	for(i = 0;i < size; i++){
		APDS9251_INFO("0x%2x  ",buf[i]);
	}
	
	APDS9251_INFO("\n");
	return 0;
}

static int rgb_apds9251_regs_debug_print(struct rgb_apds9251_data *data,int enable)
{
	int i=0;
	char reg_buf[APDS9251_REG_LEN];
	u8 reg = 0;
	struct i2c_client *client = data->client;

	/* read registers[0x0~0x1a] value*/
	for(i = 0; i < APDS9251_REG_LEN; i++ )
	{
		reg = 0x50+i;
		reg_buf[i] = rgb_apds9251_i2c_read(client, reg, APDS9251_I2C_BYTE);

		if(reg_buf[i] <0){
			APDS9251_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg_buf[i] ;
		}
	}

	/* print the registers[0x0~0x1a] value in proper format*/
	dump_reg_buf(data,reg_buf,APDS9251_REG_LEN,enable);

	return 0;
}
#endif
static void rgb_apds9251_dump_register(struct i2c_client *client)
{
	int main_ctl,als_meas_rate,als_gain,part_id,int_cfg;
	main_ctl = rgb_apds9251_i2c_read(client, APDS9251_DD_MAIN_CTRL_ADDR,APDS9251_I2C_BYTE);
	als_meas_rate= rgb_apds9251_i2c_read(client, APDS9251_DD_ALS_MEAS_RATE_ADDR,APDS9251_I2C_BYTE);
	als_gain =rgb_apds9251_i2c_read(client, APDS9251_DD_ALS_GAIN_ADDR,APDS9251_I2C_BYTE);
	part_id=rgb_apds9251_i2c_read(client, APDS9251_DD_PART_ID_ADDR,APDS9251_I2C_BYTE);
	int_cfg = rgb_apds9251_i2c_read(client, APDS9251_DD_INT_CFG_ADDR,APDS9251_I2C_BYTE);
	APDS9251_INFO("%s,line %d:main_ctl = 0x%x,als_meas_rate=0x%x,als_gain=0x%x\n",__func__,__LINE__,main_ctl,als_meas_rate,als_gain);
	APDS9251_INFO("%s,line %d:part_id = 0x%x,int_cfg =0x%x\n",__func__,__LINE__,part_id,int_cfg);
}

static int LuxCalculation(struct i2c_client *client)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	long long int lux = 0;
	long long int cct = 0;
	long long int ir_r_ratio = 0;
	long long int b_r_ratio = 0;
	
	if (data->rgb_data.red > 0)
	{
		ir_r_ratio = data->rgb_data.ir*1000/data->rgb_data.red;
		b_r_ratio = data->rgb_data.blue*1000/data->rgb_data.red;
	}
	cct = (CCT_M*b_r_ratio)/1000 + CCT_N;
	
	if (ir_r_ratio/100 > avago_cofficient[0]){
		cct = cct_compensator_H * cct/1000;
		lux = data->rgb_data.green*LUX_R/((apds9251_als_meas_rate_tb[data->als_res_index])*apds9251_als_gain_tb[4]);
	}else if (ir_r_ratio/100 > avago_cofficient[1]){
		cct = cct_compensator_A * cct/1000;
		lux = data->rgb_data.green*LUX_R/((apds9251_als_meas_rate_tb[data->als_res_index])*apds9251_als_gain_tb[4]);
	}else if (ir_r_ratio/100 > avago_cofficient[2]){
		cct = cct_compensator_D * cct/1000;
		lux = data->rgb_data.green*LUX_Q/((apds9251_als_meas_rate_tb[data->als_res_index])*apds9251_als_gain_tb[4]);
	}else{
		lux = data->rgb_data.green*LUX_P/((apds9251_als_meas_rate_tb[data->als_res_index])*apds9251_als_gain_tb[4]);
	}
	APDS9251_FLOW("%s:apds9251_als_meas_rate_tb[data->als_res_index]=%d apds9251_als_gain_tb[4]=%d \n", __FUNCTION__,apds9251_als_meas_rate_tb[data->als_res_index],apds9251_als_gain_tb[4]);
	data->rgb_data.lx = lux;
	data->rgb_data.cct = cct;
	return 0;
}

static int apds9251_dd_set_main_ctrl(struct i2c_client *client, int main_ctrl)
{
	return rgb_apds9251_i2c_write(client, APDS9251_DD_MAIN_CTRL_ADDR, main_ctrl,APDS9251_I2C_BYTE);
}

static int apds9251_dd_set_als_meas_rate(struct i2c_client *client, int als_meas)
{
	return rgb_apds9251_i2c_write(client, APDS9251_DD_ALS_MEAS_RATE_ADDR, als_meas,APDS9251_I2C_BYTE);
}

static int apds9251_dd_set_als_gain(struct i2c_client *client, int als_gain)
{
	return rgb_apds9251_i2c_write(client, APDS9251_DD_ALS_GAIN_ADDR, als_gain,APDS9251_I2C_BYTE);
}
#if 0
static int apds9251_dd_get_als_meas_rate(struct i2c_client *client)
{
	return rgb_apds9251_i2c_read(client, APDS9251_DD_ALS_MEAS_RATE_ADDR,APDS9251_I2C_BYTE);
}

static int apds9251_dd_get_als_gain(struct i2c_client *client)
{
	return rgb_apds9251_i2c_read(client, APDS9251_DD_ALS_GAIN_ADDR,APDS9251_I2C_BYTE);
}
#endif
static void apds9251_change_als_threshold(struct i2c_client *client)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	unsigned char i2c_data[12];
	int status;

	status = i2c_smbus_read_i2c_block_data(client, APDS9251_DD_IR_DATA_ADDR, APDS_READ_BLOCK_DATA_SIZE, (unsigned char*)i2c_data);
	
	if (status < 0) {
		APDS9251_ERR("%s:i2c read block data fail\n",__FUNCTION__);
		return;
	}
	
	if (status != APDS_READ_BLOCK_DATA_SIZE) {
		APDS9251_ERR("%s:i2c read block data does not match\n",__FUNCTION__);
		return;
	}
	
	data->rgb_data.ir = (i2c_data[2] << 16) | (i2c_data[1] << 8) | i2c_data[0];
	data->rgb_data.green = (i2c_data[5] << 16) | (i2c_data[4] << 8) | i2c_data[3];
	data->rgb_data.blue = (i2c_data[8] << 16) | (i2c_data[7] << 8) | i2c_data[6];
	data->rgb_data.red = (i2c_data[11] << 16) | (i2c_data[10] << 8) | i2c_data[9];

	LuxCalculation(client);
	#if 0
	gain = apds9251_dd_get_als_gain(client);
	if (gain < 0) {
		APDS9251_ERR("%s:i2c read gain fail\n",__FUNCTION__);
		return;
	}

	rate = apds9251_dd_get_als_meas_rate(client);
	if (rate < 0) {
		APDS9251_ERR("%s:i2c read meas rate fail\n",__FUNCTION__);
		return;
	}
	#endif
	APDS9251_FLOW("%s:lux=%d  red=%d green=%d blue=%d ir=%d \n", __FUNCTION__,data->rgb_data.lx,data->rgb_data.red,
	data->rgb_data.green,data->rgb_data.blue,data->rgb_data.ir);

	// report to HAL
	data->rgb_data.lx = (data->rgb_data.lx>30000) ? 30000 : data->rgb_data.lx;
	data->rgb_data.cct = (data->rgb_data.cct<10000) ? data->rgb_data.cct : 10000;
	input_report_abs(data->input_dev_als, ABS_MISC, data->rgb_data.lx);
	input_sync(data->input_dev_als);
	//we does not use cct
	//input_report_abs(data->input_dev_als, ABS_CCT, data->cct);
	APDS9251_FLOW("%s:report to HAL Lux = %d data->rgb_data.cct=%d\n",__FUNCTION__, data->rgb_data.lx,data->rgb_data.cct);
}

/* ALS polling routine */
static void rgb_apds9251_als_polling_work_handler(struct work_struct *work)
{
	struct rgb_apds9251_data *data = container_of(work, struct rgb_apds9251_data,als_dwork);
	struct i2c_client *client=data->client;
	int status;
	int loop_count=0;
	int ret;
	status = rgb_apds9251_i2c_read(client, APDS9251_DD_MAIN_STATUS_ADDR, APDS9251_I2C_BYTE);

	if ((status & APDS9251_DD_ALS_DATA_STATUS) && data->enable_als_sensor) {
		apds9251_change_als_threshold(client);
	}else{
		/* wait the most another 200ms */
		while (!(status & APDS9251_DD_ALS_DATA_STATUS) && loop_count < APDS_LOOP_COUNT)
		{
			status = rgb_apds9251_i2c_read(client, APDS9251_DD_MAIN_STATUS_ADDR,APDS9251_I2C_BYTE);
			msleep(20);
			loop_count++;
			APDS9251_FLOW("%s: enter work hadler loop_count=%d\n",__FUNCTION__,loop_count);
		}
	
		if (loop_count < APDS_LOOP_COUNT){
			apds9251_change_als_threshold(client);
			loop_count = 0;
		}
	}
	
	ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
	if (ret != 0) {
		APDS9251_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
	}
}

/*****************************************************************
Parameters    :  timer
Return        :  HRTIMER_NORESTART
Description   :  hrtimer_start call back function,
				 use to report als data
*****************************************************************/
static enum hrtimer_restart rgb_apds9251_als_timer_func(struct hrtimer *timer)
{
	struct rgb_apds9251_data* data = container_of(timer,struct rgb_apds9251_data,timer);
	
	queue_work(rgb_apds9251_workqueue, &data->als_dwork);
	return HRTIMER_NORESTART;
}

static int rgb_apds9251_enable_als_sensor(struct i2c_client *client, int val)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	struct rgb_apds9251_platform_data *pdata = data->platform_data;
	int ret;
	APDS9251_FLOW("%s,line %d:enable als val=%d data->enable_als_sensor=%d\n",__func__,__LINE__,val,data->enable_als_sensor);
	mutex_lock(&data->single_lock);
	if (val == 1) {
		/* turn on light  sensor */
		APDS9251_INFO("%s:%d pdata->panel_id = %d pdata->tp_color = %d\n", __func__,__LINE__,pdata->panel_id,pdata->tp_color);
		APDS9251_INFO("%s:%d lux cal parameter from dtsi  is CCT_M[%ld], CCT_N[%ld], cct_compensator_H[%ld], cct_compensator_A[%ld] , cct_compensator_D[%ld], LUX_P[%ld], LUX_Q[%ld],LUX_R[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n",
		__FUNCTION__, __LINE__,CCT_M, CCT_N,cct_compensator_H,cct_compensator_A,cct_compensator_D,LUX_P,LUX_Q,LUX_R,avago_cofficient[0],avago_cofficient[1],avago_cofficient[2]);
		// turn on light  sensor
		if (data->enable_als_sensor==APDS_DISABLE_ALS) {
			/* Power on and initalize the device */
			if (pdata->power_on)
				pdata->power_on(true,data);
			
			ret = rgb_apds9251_init_client(client);
			if (ret) {
				APDS9251_ERR("%s:line:%d,Failed to init rgb_apds9251\n", __func__, __LINE__);
				mutex_unlock(&data->single_lock);
				return ret;
			}
			data->enable_als_sensor = val;
			apds9251_dd_set_main_ctrl(client, APDS9251_DD_CS_MODE|APDS9251_DD_ALS_EN);

			ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
			if (ret != 0) {
				APDS9251_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
			}
		}
	} else {
		if(data->enable_als_sensor == 1){
			//turn off light sensor
			data->enable_als_sensor = APDS_DISABLE_ALS;
			apds9251_dd_set_main_ctrl(client, 0);

			/* disable als sensor, cancne data report hrtimer */
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->als_dwork);
			/*avoid hrtimer restart in data->als_dwork*/
			hrtimer_cancel(&data->timer);
		}
	}
	/* Vote off  regulators if light sensor are off */
	if ((data->enable_als_sensor == 0)&&(pdata->power_on)){
		pdata->power_on(false,data);
	}
	mutex_unlock(&data->single_lock);
	return 0;
}

 static int rgb_apds9251_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	static int als_enalbe_count=0;
	
	struct rgb_apds9251_data *data = container_of(sensors_cdev,struct rgb_apds9251_data, als_cdev);
	struct i2c_client *client = data->client;

	if ((enable != 0) && (enable != 1)) {
		APDS9251_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	APDS9251_FLOW("%s,line %d:rgb apds9251 als enable=%d\n",__func__,__LINE__,enable);

	/*for debug and print registers value when enable/disable the als every time*/
	if(enable == 0)
	{
		rgb_apds9251_enable_als_sensor(data->client, enable);

		if(rgb_apds9251_debug_mask >= 1){
			APDS9251_FLOW("%s:attention:before als_disable %d times\n", __FUNCTION__,als_enalbe_count);
			#if 0
			rgb_apds9251_regs_debug_print(data,enable);
			#endif
		}
		rgb_apds9251_dump_register(client);
	}else{

		rgb_apds9251_enable_als_sensor(data->client, enable);

		if(rgb_apds9251_debug_mask >= 1){
			APDS9251_FLOW("%s:attention: after als_enable %d times\n",__FUNCTION__,++als_enalbe_count);
		}
		rgb_apds9251_dump_register(client);		
	 }
	 return ret;
}

/*use this function to reset the poll_delay time(ms),val is the time parameter*/
static int rgb_apds9251_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	int ret;
	int als_res_index=0;
	int als_meas_rate=0;

	if (val <= 25){
		data->als_poll_delay = 25;
		//25ms conversion time
		als_res_index = APDS9251_DD_ALS_RES_16BIT;
		als_meas_rate = APDS9251_DD_ALS_MEAS_RES_16_BIT|APDS9251_DD_ALS_MEAS_RATE_25_MS;
	}else if (val <= 50){
		data->als_poll_delay = 50;
		//50ms conversion time
		als_res_index = APDS9251_DD_ALS_RES_17BIT;
		als_meas_rate = APDS9251_DD_ALS_MEAS_RES_17_BIT|APDS9251_DD_ALS_MEAS_RATE_50_MS;
	}else if (val <= 100){
		data->als_poll_delay = 100;
		//100ms conversion time
		als_res_index = APDS9251_DD_ALS_RES_18BIT;
		als_meas_rate = APDS9251_DD_ALS_MEAS_RES_18_BIT|APDS9251_DD_ALS_MEAS_RATE_100_MS;
	}else if (val <= 200){
		data->als_poll_delay = 200;
		//200ms conversion time
		als_res_index = APDS9251_DD_ALS_RES_19BIT;
		als_meas_rate = APDS9251_DD_ALS_MEAS_RES_19_BIT|APDS9251_DD_ALS_MEAS_RATE_200_MS;
	}else {
		data->als_poll_delay = 100;
		//100ms conversion time
		als_res_index = APDS9251_DD_ALS_RES_18BIT;
		als_meas_rate = APDS9251_DD_ALS_MEAS_RES_18_BIT|APDS9251_DD_ALS_MEAS_RATE_100_MS;
	}
	
	ret = apds9251_dd_set_als_meas_rate(client, als_meas_rate);
	if (ret < 0){
		APDS9251_ERR("%s: ret=%d\n",__FUNCTION__,ret);
	}
	data->als_res_index = als_res_index;
	
	APDS9251_FLOW("%s:poll delay %d, als_res_index %d, meas_rate %d\n", __FUNCTION__,data->als_poll_delay, als_res_index, als_meas_rate);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_work_sync(&data->als_dwork);
	hrtimer_cancel(&data->timer);
	ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
	if (ret != 0) {
		APDS9251_ERR("%s,line%d: hrtimer_start fail! nsec=%d\n", __func__, __LINE__,data->als_poll_delay);
		return ret;
	}
	return 0;
}

static int rgb_apds9251_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct rgb_apds9251_data *data = container_of(sensors_cdev,
			struct rgb_apds9251_data, als_cdev);
	rgb_apds9251_set_als_poll_delay(data->client, delay_msec);
	return 0;
}

static ssize_t rgb_apds9251_show_ir_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	unsigned char i2c_data[3];
	
	i2c_smbus_read_i2c_block_data(client, APDS9251_DD_IR_DATA_ADDR, 3, (unsigned char*)i2c_data);
	
	data->rgb_data.red = (((int)i2c_data[2] << 16) | ((short)i2c_data[1] << 8) | i2c_data[0]);

	return snprintf(buf,32,"%d\n", data->rgb_data.ir);
}
static DEVICE_ATTR(ir_data, S_IRUGO, rgb_apds9251_show_ir_data, NULL);

static ssize_t rgb_apds9251_show_red_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	unsigned char i2c_data[3];
	
	i2c_smbus_read_i2c_block_data(client, APDS9251_DD_RED_DATA_ADDR, 3, (unsigned char*)i2c_data);
	
	data->rgb_data.red = (((int)i2c_data[2] << 16) | ((short)i2c_data[1] << 8) | i2c_data[0]);

	return snprintf(buf,32,"%d\n", data->rgb_data.red);
}
static DEVICE_ATTR(red_data, S_IRUGO, rgb_apds9251_show_red_data, NULL);

static ssize_t rgb_apds9251_show_green_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	unsigned char i2c_data[3];

	i2c_smbus_read_i2c_block_data(client, APDS9251_DD_GREEN_DATA_ADDR, 3, (unsigned char*)i2c_data);
	
	data->rgb_data.green= (((int)i2c_data[2] << 16) | ((short)i2c_data[1] << 8) | i2c_data[0]);
	
	return snprintf(buf,32, "%d\n", data->rgb_data.green);
}
static DEVICE_ATTR(green_data, S_IRUGO, rgb_apds9251_show_green_data, NULL);

static ssize_t rgb_apds9251_show_blue_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	unsigned char i2c_data[3];

	i2c_smbus_read_i2c_block_data(client, APDS9251_DD_BLUE_DATA_ADDR, 3, (unsigned char*)i2c_data);
	
	data->rgb_data.blue= (((int)i2c_data[2] << 16) | ((short)i2c_data[1] << 8) | i2c_data[0]);
	
	return snprintf(buf,32, "%d\n", data->rgb_data.blue);
 }
static DEVICE_ATTR(blue_data, S_IRUGO, rgb_apds9251_show_blue_data, NULL);

/*
* set the register's value from userspace
* Usage: echo "0x08|0x12" > dump_reg
*			"reg_address|reg_value"
*/
static ssize_t rgb_apds9251_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val_len_max = 4;
	char* input_str =NULL;
	char reg_addr_str[10]={'\0'};
	char reg_val_str[10]={'\0'};
	long reg_addr,reg_val;
	int addr_lenth=0,value_lenth=0,buf_len=0,ret = -1;
	char* strtok=NULL;

	buf_len = strlen(buf);
	input_str = kzalloc(buf_len, GFP_KERNEL);
	if (!input_str)
	{
		APDS9251_ERR("%s:kmalloc fail!\n",__func__);
		return -ENOMEM;
	}

	snprintf(input_str, 10,"%s", buf);
	/*Split the string when encounter "|", for example "0x08|0x12" will be splited "0x18" "0x12" */
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		addr_lenth = strlen(strtok);
		memcpy(reg_addr_str,strtok,((addr_lenth > (val_len_max))?(val_len_max):addr_lenth));
	}
	else
	{
		APDS9251_ERR("%s: buf name Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		value_lenth = strlen(strtok);
		memcpy(reg_val_str,strtok,((value_lenth > (val_len_max))?(val_len_max):value_lenth));
	}
	else
	{
		APDS9251_ERR("%s: buf value Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}
	/* transform string to long int */
	ret = kstrtol(reg_addr_str,16,&reg_addr);
	if(ret)
		goto parse_fail_exit;

	ret = kstrtol(reg_val_str,16,&reg_val);
	if(ret)
		goto parse_fail_exit;

	/* write the parsed value in the register*/
	ret = rgb_apds9251_i2c_write(client,(char)reg_addr,(char)reg_val,APDS9251_I2C_BYTE);
	if (ret < 0){
		goto parse_fail_exit;
	}
	return count;

parse_fail_exit:
	if (input_str)
		kfree(input_str);

	return ret;
}

/*
* show all registers' value to userspace
*/
static ssize_t rgb_apds9251_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i;
	char reg[APDS9251_REG_LEN];
	struct i2c_client *client = to_i2c_client(dev);

	/* read all register value and print to user*/
	for(i = 0; i < APDS9251_REG_LEN; i++ )
	{
		if ((i >= 0x1) && (i <= 0x3))
			continue;
		if ((i >= 0x8) && (i <= 0x9))
			continue;
		if ((i >= 0x1B) && (i <= 0x20))
			continue;
		reg[i] = rgb_apds9251_i2c_read(client, i, APDS9251_I2C_BYTE);
		if(reg[i] <0){
			APDS9251_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg[i] ;
		}
	}

	return snprintf(buf,512,"reg[0x0,0x04~0x7,0x0a~0x0e]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
									 "reg[0x0f~0x18]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
						    "reg[0x19~0x1a,0x21~0x27]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n",
			reg[0x00],reg[0x04],reg[0x05],reg[0x06],reg[0x07],reg[0x0a],reg[0x0b],reg[0x0c],reg[0x0d],reg[0x0e],
			reg[0x0f],reg[0x10],reg[0x11],reg[0x12],reg[0x13],reg[0x14],reg[0x15],reg[0x16],reg[0x17],reg[0x18],
			reg[0x19],reg[0x1a],reg[0x21],reg[0x22],reg[0x23],reg[0x24],reg[0x25],reg[0x26]);
}
static DEVICE_ATTR(dump_reg ,S_IRUGO|S_IWUSR|S_IWGRP, rgb_apds9251_print_reg_buf, rgb_apds9251_write_reg);
/*
 *	panel_id represent junda ofilm jdi.
 *	tp_color represent golden white black blue
 *
 */
static ssize_t write_module_tpcolor(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	struct rgb_apds9251_platform_data *pdata = data->platform_data;
	int err;
	u32 val;
	int valid_flag;
	int i;
	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		APDS9251_ERR("%s:%d kstrtoint failed\n", __func__,__LINE__);
		return count;
	}
	valid_flag = val & 0xffff;
	pdata->panel_id = (val >> 16) & 0xff;
	pdata->tp_color = (val >> 24) & 0xff;
	if (valid_flag != VALID_FLAG){
		APDS9251_ERR("%s:%d  valid flag error\n", __func__,__LINE__);
		return count;
	}

	APDS9251_INFO("%s:%d panel_id = %d pdata->tp_color = %d\n", __FUNCTION__, __LINE__,pdata->panel_id,pdata->tp_color);
	for (i = 0;i < MODULE_MANUFACTURE_NUMBER;i++){
		if (pdata->panel_id == apds9251_tp_module_parameter[i].tp_module_id){
			if (pdata->tp_color == GOLD){
				CCT_M = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.cct_m;
				CCT_N = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.cct_n;
				LUX_P = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.lux_p;
				LUX_Q = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.lux_q;
				LUX_R = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.lux_r;
				cct_compensator_A = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.avago_a;
				cct_compensator_D = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.avago_d;
				cct_compensator_H = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.avago_h;
				avago_cofficient[0] = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.avago_cofficient[0];
				avago_cofficient[1] = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.avago_cofficient[1];
				avago_cofficient[2] = apds9251_tp_module_parameter[i].gold_apds9251_lux_cal_parameter.avago_cofficient[2];
			}else if (pdata->tp_color == WHITE){
				CCT_M = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.cct_m;
				CCT_N = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.cct_n;
				LUX_P = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.lux_p;
				LUX_Q = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.lux_q;
				LUX_R = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.lux_r;
				cct_compensator_A = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.avago_a;
				cct_compensator_D = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.avago_d;
				cct_compensator_H = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.avago_h;
				avago_cofficient[0] = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.avago_cofficient[0];
				avago_cofficient[1] = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.avago_cofficient[1];
				avago_cofficient[2] = apds9251_tp_module_parameter[i].white_apds9251_lux_cal_parameter.avago_cofficient[2];
			}else if (pdata->tp_color == BLACK){
				CCT_M = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.cct_m;
				CCT_N = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.cct_n;
				LUX_P = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.lux_p;
				LUX_Q = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.lux_q;
				LUX_R = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.lux_r;
				cct_compensator_A = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_a;
				cct_compensator_D = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_d;
				cct_compensator_H = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_h;
				avago_cofficient[0] = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_cofficient[0];
				avago_cofficient[1] = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_cofficient[1];
				avago_cofficient[2] = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_cofficient[2];
			}else if (pdata->tp_color == BLUE){
				CCT_M = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.cct_m;
				CCT_N = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.cct_n;
				LUX_P = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.lux_p;
				LUX_Q = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.lux_q;
				LUX_R = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.lux_r;
				cct_compensator_A = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_a;
				cct_compensator_D = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_d;
				cct_compensator_H = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_h;
				avago_cofficient[0] = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_cofficient[0];
				avago_cofficient[1] = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_cofficient[1];
				avago_cofficient[2] = apds9251_tp_module_parameter[i].black_apds9251_lux_cal_parameter.avago_cofficient[2];
			}
		}
	}
	APDS9251_INFO("%s:%d lux cal  parameter from dtsi  is M[%ld], N[%ld], H[%ld], A[%ld], D[%ld] , P[%ld], Q[%ld], R[%ld] avago_cofficient0[%ld] avago_cofficient1[%ld] avago_cofficient2[%ld]\n", __FUNCTION__, __LINE__,
	CCT_M, CCT_N,cct_compensator_H,cct_compensator_A,cct_compensator_D,LUX_P,LUX_Q,LUX_R,avago_cofficient[0],avago_cofficient[1],avago_cofficient[2]);
	return count;
}
static DEVICE_ATTR(module_tpcolor ,S_IRUGO|S_IWUSR, NULL, write_module_tpcolor);

static ssize_t read_tp_parameters(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf,2048,"golden0 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "white0 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "black0 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "blue0 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "golden1 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "white1 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "black1 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "blue1 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "golden2 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "white2 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "black2 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n"
	                                    "blue2 cct_m[%ld],cct_n[%ld],avago_h[%ld],avago_a[%ld],avago_d[%ld],lux_p[%ld],lux_q[%ld],lux_r[%ld],avago_cofficient0[%ld],avago_cofficient1[%ld],avago_cofficient2[%ld]\n",
	apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[0].gold_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[0].white_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[0].black_apds9251_lux_cal_parameter.avago_cofficient[2],

	apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[0].blue_apds9251_lux_cal_parameter.avago_cofficient[2],

	apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[1].gold_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[1].white_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[1].black_apds9251_lux_cal_parameter.avago_cofficient[2],

	apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[1].blue_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[2].gold_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[2].white_apds9251_lux_cal_parameter.avago_cofficient[2],
	
	apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[2].black_apds9251_lux_cal_parameter.avago_cofficient[2],

	apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.cct_m,apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.cct_n,apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.avago_h,
	apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.avago_a,apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.avago_d,apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.lux_p,apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.lux_q,
	apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.lux_r,apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.avago_cofficient[0],apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.avago_cofficient[1],apds9251_tp_module_parameter[2].blue_apds9251_lux_cal_parameter.avago_cofficient[2]);
}
static DEVICE_ATTR(dump_tp_parameters ,S_IRUGO, read_tp_parameters, NULL);

static ssize_t rgb_apds9251_show_light_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	
	return snprintf(buf, 32,"%d\n", data->enable_als_sensor);
}

static ssize_t rgb_apds9251_store_light_enable(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	APDS9251_FLOW("%s: enable als sensor ( %ld)\n", __func__, val);
	
	if ((val != APDS_DISABLE_ALS) && 
		(val != APDS_ENABLE_ALS_WITH_INT) && 
		(val != APDS_ENABLE_ALS_NO_INT) &&
		(val != APDS_ENABLE_ALS_CALIBRATION)) {
		
		APDS9251_FLOW("**%s: store invalid value=%ld\n", __func__, val);
		return count;
	}

	rgb_apds9251_enable_als_sensor(client, val); 

	return count;
}
static DEVICE_ATTR(light_enable, S_IWUGO | S_IRUGO,rgb_apds9251_show_light_enable, rgb_apds9251_store_light_enable);

static struct attribute *rgb_apds9251_attributes[] = {
	&dev_attr_ir_data.attr,
	&dev_attr_red_data.attr,
	&dev_attr_green_data.attr,
	&dev_attr_blue_data.attr,
	&dev_attr_dump_reg.attr,
	&dev_attr_module_tpcolor.attr,
	&dev_attr_dump_tp_parameters.attr,
	&dev_attr_light_enable.attr,
	NULL
};

static const struct attribute_group rgb_apds9251_attr_group = {
	.attrs = rgb_apds9251_attributes,
};

/*
 * Initialization function
 */
static int rgb_apds9251_read_device_id(struct i2c_client *client)
{
	int id;
	int err;
	id    = rgb_apds9251_i2c_read(client, APDS9251_DD_PART_ID_ADDR, APDS9251_I2C_BYTE);
	id &= 0xff;
	if (id == 0xb2) {
		APDS9251_INFO("%s: AVAGO APDS9251\n", __func__);
		err = app_info_set("L-Sensor", "APDS9251");
		if (err < 0)/*failed to add app_info*/
		{
		    APDS9251_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
		}
	} else {
		APDS9251_INFO("%s: AVAGO APDS9251 Does not exist \n", __func__);
		return -ENODEV;
	}
	return 0;
}

static int rgb_apds9251_init_client(struct i2c_client *client)
{
	int err;
	
	err = apds9251_dd_set_main_ctrl(client, 0);
	if (err < 0){
		APDS9251_ERR("%s,line%d:MAIN ctrl FAIL ",__func__,__LINE__);
		return err;
	}
	/* ALS_MEAS_RATE */
	err = apds9251_dd_set_als_meas_rate(client, APDS9251_DD_ALS_DEFAULT_RES|APDS9251_DD_ALS_DEFAULT_MEAS_RATE);
	if (err < 0){
		APDS9251_ERR("%s,line%d:ALS_MEAS_RATE FAIL ",__func__,__LINE__);
		return err;
	}
	/* ALS_GAIN */
	err = apds9251_dd_set_als_gain(client, APDS9251_DD_ALS_DEFAULT_GAIN);
	if (err < 0){
		APDS9251_ERR("%s,line%d:ALS_GAIN FAIL ",__func__,__LINE__);
		return err;
	}
	
	return 0;
}
/*qualcom updated the regulator configure functions and we add them all*/
static int sensor_regulator_configure(struct rgb_apds9251_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, APDS9251_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, APDS9251_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			APDS9251_ERR("%s,line%d:Regulator get failed vdd rc=%d\n",__func__,__LINE__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, APDS9251_VDD_MIN_UV, APDS9251_VDD_MAX_UV);
			if (rc) {
				APDS9251_ERR("%s,line%d:Regulator set failed vdd rc=%d\n",__func__,__LINE__,rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			APDS9251_ERR("%s,line%d:Regulator get failed vio rc=%d\n",__func__,__LINE__, rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				APDS9251_VIO_MIN_UV, APDS9251_VIO_MAX_UV);
			if (rc) {
				APDS9251_ERR("%s,line%d:Regulator set failed vio rc=%d\n",__func__,__LINE__, rc);
				goto reg_vio_put;
			}
		}

	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, APDS9251_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
/*In suspend and resume function,we only control the als,leave pls alone*/
static int rgb_apds9251_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	int rc;
	APDS9251_INFO("%s,line%d:APDS9251 SUSPEND\n",__func__,__LINE__);

	data->enable_als_state = data->enable_als_sensor;
	if(data->enable_als_sensor){
		APDS9251_INFO("%s,line%d:APDS9251 SUSPEND and disable als\n",__func__,__LINE__);
		rc = rgb_apds9251_enable_als_sensor(data->client, 0);
		if (rc){
			APDS9251_ERR("%s,line%d:Disable rgb light sensor fail! rc=%d\n",__func__,__LINE__, rc);
		}
	}

	return 0;
}

static int rgb_apds9251_resume(struct i2c_client *client)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	int ret = 0;

	APDS9251_INFO("%s,line%d:APDS9251 RESUME\n",__func__,__LINE__);

	if (data->enable_als_state) {
		ret = rgb_apds9251_enable_als_sensor(data->client, 1);
		if (ret){
			APDS9251_ERR("%s,line%d:enable rgb  light sensor fail! rc=%d\n",__func__,__LINE__, ret);
		}
	}

	return 0;
}
/*pamameter subfunction of probe to reduce the complexity of probe function*/
static int rgb_apds9251_sensorclass_init(struct rgb_apds9251_data *data,struct i2c_client* client)
{
	int err;
	/* Register to sensors class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = rgb_apds9251_als_set_enable;
	data->als_cdev.sensors_poll_delay = rgb_apds9251_als_poll_delay;

	err = sensors_classdev_register(&data ->input_dev_als ->dev, &data->als_cdev);
	if (err) {
		APDS9251_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
	}

	return err;
}
static void rgb_apds9251_parameter_init(struct rgb_apds9251_data *data)
{
	struct rgb_apds9251_platform_data *pdata = data->platform_data;
	data->enable_als_sensor = 0;
	data->als_poll_delay = 200;
	data->als_prev_lux = 300;
	//100ms conversion time
	data->als_res_index = APDS9251_DD_ALS_RES_18BIT;
	//18x GAIN
	data->als_gain_index = APDS9251_DD_ALS_GAIN_18X;
	pdata->panel_id = -1;
	pdata->tp_color = -1;
}
/*input init subfunction of probe to reduce the complexity of probe function*/
static int rgb_apds9251_input_init(struct rgb_apds9251_data *data)
{
	int err = 0;
	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		APDS9251_ERR("%s: Failed to allocate input device als\n", __func__);
		goto exit;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);

	data->input_dev_als->name = "light";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		APDS9251_ERR("%s: Unable to register input device als: %s\n",
				__func__, data->input_dev_als->name);
		goto input_register_err;
	}
	return err;

input_register_err:
	input_free_device(data->input_dev_als);
exit:
	return err;
}

static int sensor_regulator_power_on(struct rgb_apds9251_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			APDS9251_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			APDS9251_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			rc = regulator_enable(data->vdd);
			APDS9251_ERR("%s:Regulator vio re-enabled rc=%d\n",__func__, rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			APDS9251_ERR("%s:Regulator vdd enable failed rc=%d\n",__func__, rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			APDS9251_ERR("%s:Regulator vio enable failed rc=%d\n", __func__,rc);
			rc = regulator_disable(data->vdd);
			return rc;
		}
	}
enable_delay:
	msleep(130);
	APDS9251_FLOW("%s:Sensor regulator power on =%d\n",__func__, on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on,struct rgb_apds9251_data *data)
{
	int err = 0;

	if (data->power_on != on) {
		err = sensor_regulator_power_on(data, on);
		if (err)
			APDS9251_ERR("%s,line%d:Can't configure regulator!\n", __func__, __LINE__);
		else
			data->power_on = on;
	}

	return err;
}

static int sensor_platform_hw_init(struct rgb_apds9251_data *data)
{
	int error;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		APDS9251_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(struct rgb_apds9251_data *data)
{
	int error;
	error = sensor_regulator_configure(data, false);
	if (error < 0) {
		APDS9251_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
	}
}

static int sensor_parse_dt(struct device *dev,
		struct rgb_apds9251_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int tp_moudle_count = 0;
	int index =0;
	int rc = 0;
	int array_len = 0;
	int retval = 0;
	int i = 0;
	const char *raw_data0_dts = NULL;
	long *ptr = NULL;

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	rc = of_property_read_u32(np, "apds9251,tp_moudle_count", &tmp);
	if (rc) {
		APDS9251_ERR("%s,line %d:Unable to read ga_a_value\n",__func__,__LINE__);
		return rc;
	}
	tp_moudle_count = tmp;

	APDS9251_FLOW("%s:%d read lux cal parameter count from dtsi  is %d\n", __FUNCTION__, __LINE__, tp_moudle_count);

	if(tp_moudle_count > MODULE_MANUFACTURE_NUMBER){
		APDS9251_ERR("%s,line %d:tp_moudle_count from dtsi too large: %d\n",__func__,__LINE__, tp_moudle_count);
		return  -EINVAL;
	}

	for(i=0; i<tp_moudle_count; i++){
		array_len = of_property_count_strings(np, data_array_name[i]);
		if (array_len != PARSE_DTSI_NUMBER) {
			APDS9251_ERR("%s:%d apds9251,junda_data0 length invaild or dts number is larger than:%d\n",__FUNCTION__,__LINE__,array_len);
			return array_len;
		}
		APDS9251_FLOW("%s:%d read lux cal parameter count from dtsi  is %d\n", __FUNCTION__, __LINE__, array_len);

		ptr = (long *)&apds9251_tp_module_parameter[i];

		for(index = 0; index < array_len; index++){
			retval = of_property_read_string_index(np, data_array_name[i], index, &raw_data0_dts);
			if (retval) {
				APDS9251_ERR("%s:%d read index = %d,raw_data0_dts = %s,retval = %d error,\n",__FUNCTION__,__LINE__,index, raw_data0_dts, retval);
				return retval;
			}
			ptr[index]  = simple_strtol(raw_data0_dts, NULL, 10);
			APDS9251_FLOW("%s:%d lux cal parameter from dtsi  is %ld\n", __FUNCTION__, __LINE__, ptr[index]);
		}
	}

	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver rgb_apds9251_driver;
static int rgb_apds9251_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rgb_apds9251_data *data;
	struct rgb_apds9251_platform_data *pdata;
	int err = 0;

	APDS9251_INFO("%s,line %d:PROBE START.\n",__func__,__LINE__);

	if (client->dev.of_node) {
		/*Memory allocated with this function is automatically freed on driver detach.*/
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct rgb_apds9251_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			APDS9251_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
			err =-ENOMEM;
			goto exit;
		}

		client->dev.platform_data = pdata;
		err = sensor_parse_dt(&client->dev, pdata);
		if (err) {
			APDS9251_ERR("%s: sensor_parse_dt() err\n", __func__);
			goto exit_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			APDS9251_ERR("%s,line %d:No platform data\n",__func__,__LINE__);
			err = -ENODEV;
			goto exit;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		APDS9251_ERR("%s,line %d:Failed to i2c_check_functionality\n",__func__,__LINE__);
		err = -EIO;
		goto exit_parse_dt;
	}

	data = kzalloc(sizeof(struct rgb_apds9251_data), GFP_KERNEL);
	if (!data) {
		APDS9251_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
		err = -ENOMEM;
		goto exit_parse_dt;
	}

	data->platform_data = pdata;
	data->client = client;
	data->device_exist = false;

	/* h/w initialization */
	if (pdata->init)
		err = pdata->init(data);

	if (pdata->power_on)
		err = pdata->power_on(true,data);

	i2c_set_clientdata(client, data);
	rgb_apds9251_parameter_init(data);

	mutex_init(&data->update_lock);
	mutex_init(&data->single_lock);

	INIT_WORK(&data->als_dwork, rgb_apds9251_als_polling_work_handler);
	
	/* Initialize the APDS9251 chip and judge who am i*/
	err = rgb_apds9251_read_device_id(client);
	if (err) {
		APDS9251_ERR("%s: Failed to read rgb_apds9251\n", __func__);
		goto exit_power_off;
	}
	
	err = rgb_apds9251_init_client(client);
	if (err) {
		APDS9251_ERR("%s: Failed to init rgb_apds9251\n", __func__);
		goto exit_power_off;
	}

	err = rgb_apds9251_input_init(data);
	if(err)
		goto exit_power_off;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rgb_apds9251_attr_group);
	if (err)
		goto exit_unregister_dev_als;

	err = rgb_apds9251_sensorclass_init(data,client);
	if (err) {
		APDS9251_ERR("%s: Unable to register to sensors class: %d\n", __func__, err);
		goto exit_remove_sysfs_group;
	}

	rgb_apds9251_workqueue = create_workqueue("rgb_apds9251_work_queue");
	if (!rgb_apds9251_workqueue)
	{
		APDS9251_ERR("%s: Create ps_workqueue fail.\n", __func__);
		goto exit_unregister_sensorclass;
	}

	/* init hrtimer and call back function */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rgb_apds9251_als_timer_func;
	set_sensors_list(L_SENSOR);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	err = set_sensor_input(ALS, data->input_dev_als->dev.kobj.name);
	if (err) {
		APDS9251_ERR("%s set_sensor_input ALS failed\n", __func__);
	}

	if (pdata->power_on)
		err = pdata->power_on(false,data);

	APDS9251_INFO("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);
	data->device_exist = true;
	return 0;

exit_unregister_sensorclass:
	sensors_classdev_unregister(&data->als_cdev);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &rgb_apds9251_attr_group);
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_power_off:
	if (pdata->power_on)
		pdata->power_on(false,data);
	if (pdata->exit)
		pdata->exit(data);
	kfree(data);

exit_parse_dt:
exit:
	return err;
}

static int rgb_apds9251_remove(struct i2c_client *client)
{
	struct rgb_apds9251_data *data = i2c_get_clientdata(client);
	struct rgb_apds9251_platform_data *pdata = data->platform_data;
	/* Power down the device */
	apds9251_dd_set_main_ctrl(client, 0);

	sysfs_remove_group(&client->dev.kobj, &rgb_apds9251_attr_group);

	input_unregister_device(data->input_dev_als);

	hrtimer_cancel(&data->timer);

	if (pdata->power_on)
		pdata->power_on(false,data);

	if (pdata->exit)
		pdata->exit(data);

	kfree(data);

	return 0;
}

static const struct i2c_device_id rgb_apds9251_id[] = {
	{ "apds9251", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rgb_apds9251_id);

static struct of_device_id rgb_apds9251_match_table[] = {
	{ .compatible = "avago,apds9251",},
	{ },
};

static struct i2c_driver rgb_apds9251_driver = {
	.driver = {
		.name   = APDS9251_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = rgb_apds9251_match_table,
	},
	.probe  = rgb_apds9251_probe,
	.remove = rgb_apds9251_remove,
	.suspend = rgb_apds9251_suspend,
	.resume = rgb_apds9251_resume,
	.id_table = rgb_apds9251_id,
};

static int __init rgb_apds9251_init(void)
{
	return i2c_add_driver(&rgb_apds9251_driver);
}

static void __exit rgb_apds9251_exit(void)
{
	if (rgb_apds9251_workqueue) {
		destroy_workqueue(rgb_apds9251_workqueue);
		rgb_apds9251_workqueue = NULL;
	}

	i2c_del_driver(&rgb_apds9251_driver);
}

MODULE_DESCRIPTION("APDS9251 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rgb_apds9251_init);
module_exit(rgb_apds9251_exit);
