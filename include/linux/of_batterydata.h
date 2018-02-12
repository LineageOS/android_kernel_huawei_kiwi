/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include <linux/of.h>
#include <linux/batterydata-lib.h>

#ifdef CONFIG_OF_BATTERYDATA
/**
 * of_batterydata_read_data() - Populate battery data from the device tree
 * @container_node: pointer to the battery-data container device node
 *		containing the profile nodes.
 * @batt_data: pointer to an allocated bms_battery_data structure that the
 *		loaded profile will be written to.
 * @batt_id_uv: ADC voltage of the battery id line used to differentiate
 *		between different battery profiles. If there are multiple
 *		battery data in the device tree, the one with the closest
 *		battery id resistance will be automatically loaded.
 *
 * This routine loads the closest match battery data from device tree based on
 * the battery id reading. Then, it will try to load all the relevant data from
 * the device tree battery data profile.
 *
 * If any of the lookup table pointers are NULL, this routine will skip trying
 * to read them.
 */
int of_batterydata_read_data(struct device_node *container_node,
				struct bms_battery_data *batt_data,
				int batt_id_uv);

#ifdef CONFIG_HUAWEI_KERNEL
/* read dts property */
#define OF_HW_READ_PROPERTY_VAL(node, prop, getVal)\
do{\
	u32 val;\
	if (of_property_read_u32(node, prop, &val))\
	{\
		pr_err("property %s is read failed, the value is not changed!! \n", prop);\
	}\
	else\
	{\
		(getVal) = val;\
		pr_debug("property %s is read ok, value=%d \n", prop, val);\
	}\
}while(0)

#define OF_HW_READ_PROPERTY_STRING(node, prop, pgetStr)\
do{\
	const char* pstr = "";\
	if (of_property_read_string(node, prop, &pstr))\
	{\
		pr_err("property %s is read failed, the value is not changed!! \n", prop);\
	}\
	else\
	{\
		(pgetStr) = (char*)pstr;\
		pr_debug("property %s is read ok, value=%s \n", prop, pstr);\
	}\
}while(0)

#define OF_HW_READ_PROPERTY_IDX_STRING(node, prop, idx, pgetStr)\
do{\
	const char* pstr = "";\
	if (of_property_read_string_index(node, prop, idx, &pstr))\
	{\
		pr_err("property %s is read idx %d failed !! \n", prop, idx);\
	}\
	else\
	{\
		(pgetStr) = (char*)pstr;\
		pr_debug("property %s is read idx %d ok, value=%s \n", prop, idx, pstr);\
	}\
}while(0)

#define OF_HW_READ_PROPERTY_BOOL(node, prop, getBool)\
do{\
	getBool = of_property_read_bool(node, prop);\
	pr_debug("property %s is read ok, value=%d \n", prop, getBool);\
}while(0)

int of_batterydata_read_fgauge_data_maxim(struct device_node *pmain_node,
				struct max17048_batt_data *pbatt_data,
				int batt_id_uv);

int of_batterydata_read_tempctrl_info_maxim(struct device_node *pmain_node,
				struct max77819_temp_control_info *pctrl_info,
				int batt_id_uv);

#endif

#else
/**
 * of_batterydata_get_best_profile() - Find matching battery data device node
 * @batterydata_container_node: pointer to the battery-data container device
 *		node containing the profile nodes.
 * @psy_name: Name of the power supply which holds the
 *		POWER_SUPPLY_RESISTANCE_ID value to be used to match
 *		against the id resistances specified in the corresponding
 *		battery data profiles.
 * @batt_type: Battery type which we want to force load the profile.
 *
 * This routine returns a device_node pointer to the closest match battery data
 * from device tree based on the battery id reading.
 */
struct device_node *of_batterydata_get_best_profile(
		struct device_node *batterydata_container_node,
		const char *psy_name, const char *batt_type);

static inline int of_batterydata_read_data(struct device_node *container_node,
				struct bms_battery_data *batt_data,
				int batt_id_uv);
{
	return -ENXIO;
}
static inline struct device_node *of_batterydata_get_best_profile(
		struct device_node *batterydata_container_node,
		struct device_node *best_node, const char *psy_name)
{
	return -ENXIO;
}
#endif /* CONFIG_OF_QPNP */
