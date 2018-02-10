/*Add for huawei TP*/
/*
 * Copyright (c) 2014 Huawei Device Company
 *
 * This file provide common requeirment for different touch IC.
 * 
 * 2014-01-04:Add "tp_get_touch_screen_obj" by sunlibin
 *
 */
#include <linux/module.h>
#include <huawei_platform/touchscreen/hw_tp_common.h>
#include <linux/atomic.h>
/*echo debug_level > sys/module/hw_tp_common/paramers/hw_tp_common_debug_mask */
int hw_tp_common_debug_mask = TP_INFO;
module_param_named(hw_tp_common_debug_mask, hw_tp_common_debug_mask, int, 0664);
bool pt_test_enable_tp = 0;
module_param_named(pt_test_enable_tp, pt_test_enable_tp, bool, 0664);
static int g_tp_type = UNKNOW_PRODUCT_MODULE;
static struct kobject *touch_screen_kobject_ts = NULL;
static struct kobject *touch_glove_func_ts = NULL;
struct kobject *virtual_key_kobject_ts = NULL;
/*init atomic touch_detected_flag*/
atomic_t touch_detected_flag = ATOMIC_INIT(0);

/**
 * set_touch_probe_flag - to set the touch_detected_flag
 *
 * @detected: the value of touch_detected_flag,if detected is 1, set touch_detected_flag to 1;
 *                  if detected is 0, set touch_detected_flag to 0.
 *
 * This function to set the touch_detected_flag,notice it is atomic.
 *
 * This function have not returned value.
 */
static void set_touch_probe_flag(int detected)
{
	if(detected >= 0)
	{
		atomic_set(&touch_detected_flag, 1);
	}
	else
	{
		atomic_set(&touch_detected_flag, 0);
	}

	return;
}

/**
 * read_touch_probe_flag - to read the touch_detected_flag
 *
 * @no input value
 *
 * This function to read the touch_detected_flag,notice it is atomic.
 *
 * The atomic of touch_detected_flag will be returned.
 */
static int read_touch_probe_flag(void)
{
	return atomic_read(&touch_detected_flag);
}

/*It is a whole value inoder to set of read touch_detected_flag status*/
struct touch_hw_platform_data touch_hw_data =
{
	.set_touch_probe_flag = set_touch_probe_flag,
	.read_touch_probe_flag = read_touch_probe_flag,
};

/**
 * tp_get_touch_screen_obj - it is a common function,tp can call it to creat /sys/touch_screen file node
 *
 * @no input value
 *
 * This function is tp call it to creat /sys/touch_screen file node.
 *
 * The kobject of touch_screen_kobject_ts will be returned,notice it is static.
 */
struct kobject* tp_get_touch_screen_obj(void)
{
	if( NULL == touch_screen_kobject_ts )
	{
		touch_screen_kobject_ts = kobject_create_and_add("touch_screen", NULL);
		if (!touch_screen_kobject_ts)
		{
			tp_log_err("%s: create touch_screen kobjetct error!\n", __func__);
			return NULL;
		}
		else
		{
			tp_log_debug("%s: create sys/touch_screen successful!\n", __func__);
		}
	}
	else
	{
		tp_log_debug("%s: sys/touch_screen already exist!\n", __func__);
	}

	return touch_screen_kobject_ts;
}
/**
 * tp_get_virtual_key_obj - it is a common function,tp can call it to creat virtual_key file node in /sys/
 *
 * @no input value
 *
 * This function is tp call it to creat virtual_key file node in /sys/
 *
 * The kobject of virtual_key_kobject_ts will be returned
 */
struct kobject* tp_get_virtual_key_obj(char *name)
{
	if( NULL == virtual_key_kobject_ts )
	{
		virtual_key_kobject_ts = kobject_create_and_add(name, NULL);
		if (!virtual_key_kobject_ts)
		{
			tp_log_err("%s: create virtual_key kobjetct error!\n", __func__);
			return NULL;
		}
		else
		{
			tp_log_debug("%s: create virtual_key successful!\n", __func__);
		}
	}
	else
	{
		tp_log_debug("%s: virtual_key already exist!\n", __func__);
	}

	return virtual_key_kobject_ts;
}
/**
 * tp_get_glove_func_obj - it is a common function,tp can call it to creat glove_func file node in /sys/touch_screen/
 *
 * @no input value
 *
 * This function is tp call it to creat glove_func file node in /sys/touch_screen/.
 *
 * The kobject of touch_glove_func_ts will be returned,notice it is static.
 */
struct kobject* tp_get_glove_func_obj(void)
{
	struct kobject *properties_kobj;
	
	properties_kobj = tp_get_touch_screen_obj();
	if( NULL == properties_kobj )
	{
		tp_log_err("%s: Error, get kobj failed!\n", __func__);
		return NULL;
	}
	
	if( NULL == touch_glove_func_ts )
	{
		touch_glove_func_ts = kobject_create_and_add("glove_func", properties_kobj);
		if (!touch_glove_func_ts)
		{
			tp_log_err("%s: create glove_func kobjetct error!\n", __func__);
			return NULL;
		}
		else
		{
			tp_log_debug("%s: create sys/touch_screen/glove_func successful!\n", __func__);
		}
	}
	else
	{
		tp_log_debug("%s: sys/touch_screen/glove_func already exist!\n", __func__);
	}

	return touch_glove_func_ts;
}
/*add api func for sensor to get TP module*/
/*tp type define in enum f54_product_module_name*/
/*tp type store in global variable g_tp_type */
/*get_tp_type:sensor use to get tp type*/
int get_tp_type(void)
{
	return g_tp_type;
}
/*set_tp_type:drivers use to set tp type*/
void set_tp_type(int type)
{
	g_tp_type = type;
	tp_log_err("%s:tp_type=%d\n",__func__,type);
}

