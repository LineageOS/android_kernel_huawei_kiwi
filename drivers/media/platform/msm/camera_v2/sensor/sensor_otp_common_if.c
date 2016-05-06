

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_common_if"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

/***** 23060132(ov5648/s5k4e1), 2015/07/14 begin *****/
#define OV5648_FOXCONN_132_RG_RATIO_TYPICAL  0x278
#define OV5648_FOXCONN_132_BG_RATIO_TYPICAL  0x2d2

#define OV5648_OFILM_132_RG_RATIO_TYPICAL    0x02e8
#define OV5648_OFILM_132_BG_RATIO_TYPICAL    0x0312

#define S5K4E1_SUNNY_132_RG_RATIO_TYPICAL     0x30c
#define S5K4E1_SUNNY_132_BG_RATIO_TYPICAL     0x2b1
/***** 23060132(ov5648/s5k4e1), 2015/07/14 end *****/


/***** 23060167(0v13850/imx328), 2015/07/14 begin *****/
#define OV13850_OFILM_167_RG_RATIO_TYPICAL    0x248
#define OV13850_OFILM_167_BG_RATIO_TYPICAL    0x257

#define IMX328_SUNNY_167_RG_RATIO_TYPICAL      0x01ED
#define IMX328_SUNNY_167_BG_RATIO_TYPICAL      0x0172

#define OV13850_LITEON_167_RG_RATIO_TYPICAL    0x248
#define OV13850_LITEON_167_BG_RATIO_TYPICAL    0x257
/***** 23060167(0v13850/imx328), 2015/07/14 end *****/


/***** 23060193(0v13850/ar1335), 2015/07/14 begin *****/
/* average of only two golden values(20150519) */
#define OV13850_OFILM_193_RG_RATIO_TYPICAL  0x264
#define OV13850_OFILM_193_BG_RATIO_TYPICAL  0x24c

/* average of 5 golden values OK(20150519) */
#define OV13850_LITEON_193_RG_RATIO_TYPICAL    0x242
#define OV13850_LITEON_193_BG_RATIO_TYPICAL    0x233

#define AR1335_SUNNY_193_RG_RATIO_TYPICAL    0x26D
#define AR1335_SUNNY_193_BG_RATIO_TYPICAL    0x267
/***** 23060193(0v13850/ar1335), 2015/07/14 end *****/

/********* 23060156(imx214), 2015/07/16 begin ************/
#define IMX214_FOXCONN_RG_RATIO_TYPICAL 0x1E0
#define IMX214_FOXCONN_BG_RATIO_TYPICAL 0x179
#define IMX214_SUNNY_RG_RATIO_TYPICAL 0x1DD
#define IMX214_SUNNY_BG_RATIO_TYPICAL 0x192
#define IMX214_OFILM_RG_RATIO_TYPICAL 0x1E1
#define IMX214_OFILM_BG_RATIO_TYPICAL 0x1A3
/********* 23060156(imx214), 2015/07/16 end ************/

#define OV8858_FOXCONN_PAD_RG_RATIO_TYPICAL 0x24B
#define OV8858_FOXCONN_PAD_BG_RATIO_TYPICAL 0x274

#define IMX219_LITEON_PAD_RG_RATIO_TYPICAL 0x23A
#define IMX219_LITEON_PAD_BG_RATIO_TYPICAL 0x251

#define IMX219_OFILM_PAD_RG_RATIO_TYPICAL 0x271
#define IMX219_OFILM_PAD_BG_RATIO_TYPICAL 0x2ba

#define S5K3M2_SUNNY_RG_RATIO_TYPICAL 0x1FE
#define S5K3M2_SUNNY_BG_RATIO_TYPICAL 0x224

struct otp_function_t otp_function_lists []=
{
	#include "sensor_otp_kiw.h"
	#include "sensor_otp_chm.h"
	#include "sensor_otp_ale.h"
	#include "sensor_otp_cherry.h"
	/* #include "sensor_otp_t2.h" */
};

/*************************************************
  Function    : is_exist_otp_function
  Description: Detect the otp we support
  Calls:
  Called By  : msm_sensor_config
  Input       : s_ctrl
  Output     : index
  Return      : true describe the otp we support
                false describe the otp we don't support

*************************************************/
bool is_exist_otp_function( struct msm_sensor_ctrl_t *s_ctrl, int32_t *index)
{
	int32_t i = 0;

	for (i=0; i<(sizeof(otp_function_lists)/sizeof(otp_function_lists[0])); ++i)
	{
        if(strlen(s_ctrl->sensordata->sensor_name) != strlen(otp_function_lists[i].sensor_name))
            continue;
		if (0 == strncmp(s_ctrl->sensordata->sensor_name, otp_function_lists[i].sensor_name, strlen(s_ctrl->sensordata->sensor_name)))
		{
			*index = i;
			CMR_LOGI("is_exist_otp_function success i = %d\n", i);
			return true;
		}
	}
	return false;
}

