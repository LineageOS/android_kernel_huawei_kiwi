
#include "sensor_otp_common.h"


#define  IMX214_OTP_LENGTH     1219
#define  IMX214_AF_OTP_START   1214
#define  IMX214_AF_OTP_LEN   4
#define  IMX214_AWB_OTP_START 5
#define  IMX214_AWB_OTP_LEN 8

#define  IMX278_OTP_LENGTH        1273 //1252  //for sunny and liteon
//#define  IMX278_OTP_LENGTH_LG     1272
#define  IMX278_AF_OTP_START      1214
#define  IMX278_AF_OTP_LEN   4
#define  IMX278_AWB_OTP_START 5
#define  IMX278_AWB_OTP_LEN 8
#define  IMX278_OIS_OTP_START 1218
#define  IMX278_OIS_OTP_LEN 53
//#define  IMX278_OIS_OTP_LEN_LG 53


#define  OV5648_OTP_LENGTH     12
#define  OV5648_AWB_OTP_START 5
#define  OV5648_AWB_OTP_LEN 6

#define  S5k4E1_OTP_LENGTH     10
#define  S5k4E1_AWB_OTP_START 5
#define  S5k4E1_AWB_OTP_LEN 4
#define OTP_AXIS_ANGLE_HORIZON    0
#define OTP_AXIS_ANGLE_VERTICAL   1
#define OTP_POSE_OFFSET_DEFAULT 0x40

static struct msm_sensor_otp_info imx214_otp_info = {
	.otp_size = IMX214_OTP_LENGTH,
	.af_otp = {
		.index_start = IMX214_AF_OTP_START,
		.af_size = IMX214_AF_OTP_LEN,
		.start_dist = 500,  //500cm
		.end_dist = 7, //7cm
		.axis_angle = OTP_AXIS_ANGLE_HORIZON,
		.pose_offset = OTP_POSE_OFFSET_DEFAULT,
	},
	.common_otp = {
		//default start index is zero
		.common_size = ALTEK_MAX_COMMON_OTP_LEN,
	},
	.awb_otp = {
		.index_start = IMX214_AWB_OTP_START,
		.awb_size = IMX214_AWB_OTP_LEN,
	},
};

static struct msm_sensor_otp_info imx278_otp_info = {
	.otp_size = IMX278_OTP_LENGTH,
	.af_otp = {
		.index_start = IMX278_AF_OTP_START,
		.af_size = IMX278_AF_OTP_LEN,
		.start_dist = 500,  //500cm
		.end_dist = 7, //7cm
		.axis_angle = OTP_AXIS_ANGLE_HORIZON,
		.pose_offset = OTP_POSE_OFFSET_DEFAULT,
	},
	.common_otp = {
		//default start index is zero
		.common_size = ALTEK_MAX_COMMON_OTP_LEN,
	},
	.awb_otp = {
		.index_start = IMX278_AWB_OTP_START,
		.awb_size = IMX278_AWB_OTP_LEN,
	},
	.ois_otp = {
		.index_start = IMX278_OIS_OTP_START,
		.ois_size = IMX278_OIS_OTP_LEN,
	},
};

static struct msm_sensor_otp_info imx278_lg_otp_info = {
	.otp_size = IMX278_OTP_LENGTH,
	.af_otp = {
		.index_start = IMX278_AF_OTP_START,
		.af_size = IMX278_AF_OTP_LEN,
		.start_dist = 300,  //300cm
		.end_dist = 7, //7cm
		.axis_angle = OTP_AXIS_ANGLE_VERTICAL,
		.pose_offset = OTP_POSE_OFFSET_DEFAULT,
	},
	.common_otp = {
		//default start index is zero
		.common_size = ALTEK_MAX_COMMON_OTP_LEN,
	},
	.awb_otp = {
		.index_start = IMX278_AWB_OTP_START,
		.awb_size = IMX278_AWB_OTP_LEN,
	},
	.ois_otp = {
		.index_start = IMX278_OIS_OTP_START,
		.ois_size = IMX278_OIS_OTP_LEN,
	},
};

static struct msm_sensor_otp_info ov5648_otp_info = {
	.otp_size = OV5648_OTP_LENGTH,
	.common_otp = {
		//default start index is zero
		.common_size = ALTEK_MAX_COMMON_OTP_LEN,
	},
	.awb_otp = {
		.index_start = OV5648_AWB_OTP_START,
		.awb_size = OV5648_AWB_OTP_LEN,
	},
};

static struct msm_sensor_otp_info s5k4e1_otp_info = {
	.otp_size = S5k4E1_OTP_LENGTH,
	.common_otp = {
		//default start index is zero
		.common_size = ALTEK_MAX_COMMON_OTP_LEN,
	},
	.awb_otp = {
		.index_start = S5k4E1_AWB_OTP_START,
		.awb_size = S5k4E1_AWB_OTP_LEN,
	},
};

struct otp_data_t otp_data_lists[]=
{
	{
		"imx214_sunny",
		&imx214_otp_info,
	},
	{
		"imx214_liteon",
		&imx214_otp_info,
	},
	{
		"altek6010_imx278_sunny",
		&imx278_otp_info,
	},	
	{
		"altek6010_imx278_liteon",
		&imx278_otp_info,
	},	
	{
		"altek6010_imx278_lg",
		&imx278_lg_otp_info,
	},
	{
		"altek6010_ov5648",
		&ov5648_otp_info,
	},
	{
		"altek6010_s5k4e1",
		&s5k4e1_otp_info,
	},
};

uint8_t get_otp_index( struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t i = 0;

	pr_err("%s: sensor name=%s", __func__, s_ctrl->sensordata->sensor_name);
	for (i=0; i<(sizeof(otp_data_lists)/sizeof(otp_data_lists[0])); i++)
	{
		if (0 == strncmp(s_ctrl->sensordata->sensor_name, 
			otp_data_lists[i].sensor_name, strlen(s_ctrl->sensordata->sensor_name)))
		{
			pr_err("%s success i = %d\n", __func__, i);
			return i;
		}
	}
	return -1;
}
struct msm_sensor_otp_info * get_otp_info_by_moduleid(int32_t module_id)
{
	uint8_t i = 0;
	int32_t vendor_id = -1;

	for (i=0; i<(sizeof(otp_data_lists)/sizeof(otp_data_lists[0])); i++)
	{
		vendor_id = otp_data_lists[i].otp_info->common_otp.common_info[4];
		vendor_id = (vendor_id >> 4);
		if(vendor_id == module_id)
		{
			pr_info("find vendor_id = %02x module\n",vendor_id);
			return otp_data_lists[i].otp_info;
		}
	}

	return NULL;
}
