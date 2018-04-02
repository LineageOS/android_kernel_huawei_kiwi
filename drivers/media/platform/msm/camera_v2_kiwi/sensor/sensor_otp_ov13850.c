/************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  FileName: sensor_otp_ov13850.c
  Author:  jwx206032
  Version :Initial Draft
  Date: 2014/07/08
  Description:    this file contion several functions to detect otp_ov5648 properties
  Version:         Initial Draft
  History:
   History        :
   1.Date        : 2014/07/08
   Author        : jwx206032
   Modification : Created function
***********************************************************/

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_ov13850"

#include <linux/hw_camera_common.h>
#include <media/msm_cam_sensor.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

//R/G and B/G value of Golden Samples.
static  uint32_t rg_ratio_typical = 0x248;   //the average of 4 Golden samples' RG ratio
static  uint32_t bg_ratio_typical = 0x257;   //the average of 4 Golden samples' BG ratio

//#define OV13850_OTP_READ_TIME_PRINT
#define OV13850_SLAVE_ADDR       0x20
#define OTP_SLAVE_ADDR1             0xa4
#define OTP_SLAVE_ADDR2             0xa6
#define OTP_ID_REG 		              0x00
#define OTP_AWB_REG 	              0x05
#define OTP_LSC_1_REG 	              0x0b
#define OTP_LSC_2_REG	              0x00
#define OTP_VCM_REG 	              0x73
#define OTP_CHECKSUM_REG           0x77
#define OV13850_OTP_ID_READ	          (1 << 0)
#define OV13850_OTP_VCM_READ             (1 << 1)
#define OV13850_OTP_LSC_READ              (1 << 2)
#define OV13850_OTP_AWB_READ             (1 << 3)
#define OV13850_OTP_CHECKSUM_READ   (1 << 4)
#define OV13850_OTP_CHECKSUM_ERR      (1 << 5)
#define OV13850_OTP_FAIL_FLAG             (1 << 6)

#define OV1385_MODULE_HUAWEI_193_ID      0xC1 //23060193   KIW 
#define OV1385_MODULE_HUAWEI_167_ID      0xA7 //23060167   CHM/ALE/CHERRY
#define OV13850_OTP_LSC_SIZE             360

#define OV13850_MMI_OTP_VCM_FLAG          (1 << 0)
#define OV13850_MMI_OTP_AWB_FLAG          (1 << 1)
#define OV13850_MMI_OTP_MODULE_INFO_FLAG  (1 << 2)
#define OV13850_MMI_OTP_LSC_FLAG          (1 << 3)
#define OV13850_MMI_OTP_CHECKSUM_FLAG     (1 << 4)
#define OV13850_MMI_OTP_SUMVAL_FLAG       (1 << 5)

#define MMI_OTP_FLAG_FAILED_MASK           0x3F

//the value used for vcm effect, maybe modified by others
#define OV13850_LITEON_167_OTP_VCM_OFFSET_VALUE    100
#define OV13850_OFILM_167_OTP_VCM_OFFSET_VALUE     100
#define OV13850_LITEON_193_OTP_VCM_OFFSET_VALUE    100
#define OV13850_OFILM_193_OTP_VCM_OFFSET_VALUE     100
#define OV13850_OTP_VCM_END_MAX                    1023

typedef enum {
	LITEON_MODULE_167_NUM,
	LITEON_MODULE_193_NUM,
	OFILM_MODULE_167_NUM,
	OFILM_MODULE_193_NUM
}camera_module_vendor_num;

typedef enum {
	SUNNY_MODULE_VENDOR_ID = 1,
	FOXCONN_MODULE_VENDOR_ID,
	LITEON_MODULE_VENDOR_ID,
	SEMCO_MODULE_VENDOR_ID,
	BYD_MODULE_VENDOR_ID,
	OFILM_MODULE_VENDOR_ID
}camera_module_vendor_id;

typedef struct ov13850_otp_struct_type
{
	u16 rg_ratio;
	u16 bg_ratio;
	u16 grgb;
	u16 vcm_start;
	u16 vcm_end;
	u8  lsc[OV13850_OTP_LSC_SIZE];
}ov13850_otp_struct;

static uint8_t ov13850_otp_vendor_module = 0xFF;

static ov13850_otp_struct ov13850_otp;
static u32 OTPSUMVAL         = 0;
static uint8_t  ov13850_otp_flag  = 0;

static bool ov13850_get_otp_from_sensor(struct msm_sensor_ctrl_t *s_ctrl);
static void ov13850_otp_set_lsc(struct msm_sensor_ctrl_t *s_ctrl);
static void ov13850_otp_set_awb(struct msm_sensor_ctrl_t *s_ctrl);
static int ov13850_module_i2c_opt_interface(struct msm_sensor_ctrl_t * s_ctrl,uint16_t i2c_addr,enum msm_camera_i2c_reg_addr_type addr_type);

/****************************************************************************
* FunctionName: ov5648_sunny_p5v18g_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t ov13850_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, u16 data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_sunny_p5v18g_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t ov13850_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&temp_data, MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	}

	*data = temp_data;

	return rc;
}

/****************************************************************************
* FunctionName: ov13850_read_otp;
* Description : i2c api used to read information from eeprom.
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool ov13850_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t i2c_addr,u16 reg,u8 *buf,u16 count)
{
	u16 i   = 0;
	int ret = 0;
	u16 val = 0;

	ret = ov13850_module_i2c_opt_interface(s_ctrl,i2c_addr,MSM_CAMERA_I2C_BYTE_ADDR);
	if (ret < 0)
	{
		return false;
	}

	for (i=0; i<count; i++)
	{
		ret =ov13850_cci_i2c_read(s_ctrl,(reg+i) ,&val);
		if (ret !=0)
		{
			CMR_LOGE("%s fail to read otp with error code %d, i2c_addr=0x%x reg_addr=0x%x\n", __func__,ret,i2c_addr,reg+i);
			return false;
		}
		buf[i] = (val&0xff);
		OTPSUMVAL += buf[i];
	}

	return true;
}

/****************************************************************************
* FunctionName: ov13850_otp_set_flag;
* Description    : set the ov13850 otp flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static void ov13850_otp_set_flag(uint8_t flag)
{
	ov13850_otp_flag |= flag;
	return;
}

/****************************************************************************
* FunctionName: ov13850_otp_get_flag;
* Description : get the ov13850 flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static uint8_t ov13850_otp_get_flag(void)
{
	return ov13850_otp_flag;
}

/****************************************************************************
* FunctionName: ov13850_otp_read_checksum;
* Description : Get check etc. parameters from eeprom.
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool ov13850_otp_read_checksum(struct msm_sensor_ctrl_t *s_ctrl,u8 *checksum)
{
	return ov13850_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_CHECKSUM_REG,checksum,1);
}

/****************************************************************************
* FunctionName: ov13850_otp_read_id;
* Description : Get id etc. parameters from eeprom.
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool ov13850_otp_read_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	u8 buf[5] = {0};
	u8 vendor_id = 0;

	ov13850_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_ID_REG,buf,5);
	CMR_LOGW("module info year 20%02d month %d day %d, SNO. 0x%x  vendor id&version 0x%x\n", buf[0],buf[1],buf[2],buf[3],buf[4]);
	vendor_id = (buf[4]>>4)&0x0F;

	if (vendor_id == LITEON_MODULE_VENDOR_ID && buf[3] == OV1385_MODULE_HUAWEI_167_ID)
	{
		ov13850_otp_set_flag(OV13850_OTP_ID_READ);
		ov13850_otp_vendor_module = LITEON_MODULE_167_NUM;
		return true;
	}
	else if (vendor_id == OFILM_MODULE_VENDOR_ID && buf[3] == OV1385_MODULE_HUAWEI_167_ID)
	{
		ov13850_otp_set_flag(OV13850_OTP_ID_READ);
		ov13850_otp_vendor_module = OFILM_MODULE_167_NUM;
		return true;
	}
	else if (vendor_id == LITEON_MODULE_VENDOR_ID && buf[3] == OV1385_MODULE_HUAWEI_193_ID)
	{
		ov13850_otp_set_flag(OV13850_OTP_ID_READ);
		ov13850_otp_vendor_module = LITEON_MODULE_193_NUM;
		return true;
	}
	else if (vendor_id == OFILM_MODULE_VENDOR_ID && buf[3] == OV1385_MODULE_HUAWEI_193_ID)
	{
		ov13850_otp_set_flag(OV13850_OTP_ID_READ);
		ov13850_otp_vendor_module = OFILM_MODULE_193_NUM;
		return true;
	}
	else
	{
		CMR_LOGE("%s OTP data is worng for with wrong vender id!!!\n",__func__);
		return false;
	}
}

/****************************************************************************
* FunctionName: ov13850_otp_read_awb;
* Description : Get awb parameters from eeprom.
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool ov13850_otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl)
{
	u8  buf[6] = {0};

	ov13850_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_AWB_REG,buf,6);
	//CMR_LOGD("%s OTP data are Rg_high=%x, Rg_low=%x, Bg_high=%x, Bg_low=%x, gbgr_high=%x, gbgr_low=%x!!!\n", __func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	ov13850_otp.rg_ratio = buf[0];
	ov13850_otp.rg_ratio <<= 8;
	ov13850_otp.rg_ratio += buf[1];
	ov13850_otp.bg_ratio = buf[2];
	ov13850_otp.bg_ratio <<= 8;
	ov13850_otp.bg_ratio += buf[3];
	ov13850_otp.grgb = buf[4];
	ov13850_otp.grgb <<= 8;
	ov13850_otp.grgb += buf[5];

	CMR_LOGD("%s OTP data are rg_ratio=0x%x, bg_ratio=0x%x, grgb=0x%x\n",__func__, ov13850_otp.rg_ratio, ov13850_otp.bg_ratio, ov13850_otp.grgb);

       //if awb value read is error for zero, abnormal branch deal
	if (0 == ov13850_otp.rg_ratio || 0 ==ov13850_otp.bg_ratio || 0 == ov13850_otp.grgb)
	{
		CMR_LOGE("%s OTP data is worng!!!\n",__func__);
		return false;
	}
	ov13850_otp_set_flag(OV13850_OTP_AWB_READ);

	return true;
}

/****************************************************************************
* FunctionName: ov13850_otp_set_awb;
* Description : Set AWB parameters to sensor registers.
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static void ov13850_otp_set_awb(struct msm_sensor_ctrl_t *s_ctrl)
{
	u16 R_gain = 0;
	u16 G_gain = 0;
	u16 B_gain = 0;
	u16 Base_gain = 0;
	u16  temp = 0;

      /*calculate the expect reg gain form the otp awb data by ov provided algorithm*/
	R_gain = (rg_ratio_typical*1000) / ov13850_otp.rg_ratio;
	B_gain = (bg_ratio_typical*1000) / ov13850_otp.bg_ratio;
	G_gain = 1000;

	if (R_gain < 1000 || B_gain < 1000)
	{
		if (R_gain < B_gain)
		{
			Base_gain = R_gain;
		}
		else
		{
			Base_gain = B_gain;
		}
	}
	else
	{
		Base_gain = G_gain;
	}

	R_gain = 0x400 * R_gain / (Base_gain);
	B_gain = 0x400 * B_gain / (Base_gain);
	G_gain = 0x400 * G_gain / (Base_gain);
	CMR_LOGD("ov13850_get_otp_from_sensor() ~ %s R_gain=0x%x, B_gain=0x%x, G_gain=0x%x\n",__func__, R_gain, B_gain, G_gain);
       /* awb set flag */
	ov13850_cci_i2c_read(s_ctrl,0x5001, &temp);
	temp = 0x02 | temp;
	ov13850_cci_i2c_write(s_ctrl,0x5001, temp);

	if (R_gain>0x400)
	{
		ov13850_cci_i2c_write(s_ctrl,0x5056, R_gain>>8);
		ov13850_cci_i2c_write(s_ctrl,0x5057, R_gain & 0x00ff);
	}

	if (G_gain>0x400)
	{
		ov13850_cci_i2c_write(s_ctrl,0x5058, G_gain>>8);
		ov13850_cci_i2c_write(s_ctrl,0x5059, G_gain & 0x00ff);
	}

	if (B_gain>0x400)
	{
		ov13850_cci_i2c_write(s_ctrl,0x505a, B_gain>>8);
		ov13850_cci_i2c_write(s_ctrl,0x505b, B_gain & 0x00ff);
	}

	return;
}

/****************************************************************************
* FunctionName: ov13850_otp_read_lsc;
* Description : Get lens shading parameters from eeprom.
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool ov13850_otp_read_lsc(struct msm_sensor_ctrl_t *s_ctrl)
{
	//LSC 0xa0:0b--0xff & 0xa2:00--0x72  total = 360
	ov13850_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_LSC_1_REG,ov13850_otp.lsc,0xff-0x0b+1);
	ov13850_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_LSC_2_REG,&ov13850_otp.lsc[0xff-0x0b+1],0x72+1);
	CMR_LOGD("%s LCS[0]=%x, LSC[244]=%x, LSC[245]=%x, LSC[359]=%x\n",__func__,
		ov13850_otp.lsc[0],ov13850_otp.lsc[244],ov13850_otp.lsc[245],ov13850_otp.lsc[359]);
       ov13850_otp_set_flag(OV13850_OTP_LSC_READ);

	return true;
}

/****************************************************************************
* FunctionName: ov13850_otp_set_lsc;
* Description : Set lens shading parameters to sensor registers.
* Input       : NA;
* Output      : NA;
* ReturnValue : NONE;
* Other       : NA;
***************************************************************************/
static void ov13850_otp_set_lsc(struct msm_sensor_ctrl_t *s_ctrl)
{
	u16 i = 0;
	u16  temp = 0;

       /* lsc set flag */
	ov13850_cci_i2c_read(s_ctrl,0x5000, &temp);
	temp = 0x01 | temp;
	ov13850_cci_i2c_write(s_ctrl,0x5000, temp);

	for (i=0; i<OV13850_OTP_LSC_SIZE; i++)
	{
		ov13850_cci_i2c_write(s_ctrl,0x5200+i, ov13850_otp.lsc[i]);
	}
	CMR_LOGD("%s, set OTP LSC to sensor OK.\n", __func__);
	return;
}

/****************************************************************************
* FunctionName: ov13850_otp_read_vcm;
* Description : Get AF motor parameters from EEPROM.;
* Input       : NA;
* Output      : ov13850_otp;
* ReturnValue :ture-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool ov13850_otp_read_vcm(struct msm_sensor_ctrl_t *s_ctrl)
{
	u8  buf[4] = {0};
	u16 vcm_start = 0;
	u16 vcm_end =0;
	uint8_t	vcm_offset_value = 0;

	ov13850_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_VCM_REG,buf,4);
	vcm_start = buf[0];
	vcm_start <<= 8;
	vcm_start += buf[1];
	vcm_end   = buf[2];
	vcm_end   <<= 8;
	vcm_end   += buf[3];
	if ((vcm_start != vcm_end ) &&(vcm_end > vcm_start) && vcm_start != 0 && vcm_end  != 0)
	{
	    ov13850_otp_set_flag(OV13850_OTP_VCM_READ);
		ov13850_otp.vcm_start = vcm_start;
		ov13850_otp.vcm_end = vcm_end;
		CMR_LOGW("%s vcm_start=0x%x, vcm_end=0x%x \n",__func__, ov13850_otp.vcm_start,ov13850_otp.vcm_end);

		switch (ov13850_otp_vendor_module)
		{
			case LITEON_MODULE_167_NUM:
			{
				CMR_LOGW("ov13850 otp is liteon module and huawei code num is 167.\n");
				vcm_offset_value = OV13850_LITEON_167_OTP_VCM_OFFSET_VALUE;
				break;
			}
			case OFILM_MODULE_167_NUM:
			{
				CMR_LOGW("ov13850 otp is ofilm module and huawei code num is 167.\n");
				vcm_offset_value = OV13850_OFILM_167_OTP_VCM_OFFSET_VALUE;
				break;
			}
			case LITEON_MODULE_193_NUM:
			{
				CMR_LOGW("ov13850 otp is liteon module and huawei code num is 193.\n");
				vcm_offset_value = OV13850_LITEON_193_OTP_VCM_OFFSET_VALUE;
				break;
			}
			case OFILM_MODULE_193_NUM:
			{
				CMR_LOGW("ov13850 otp is ofilm module and huawei code num is 193.\n");
				vcm_offset_value = OV13850_OFILM_193_OTP_VCM_OFFSET_VALUE;
				break;
			}
			default:
			{
				CMR_LOGE("%s ov13850_otp_vendor_module = %d is wrong!\n",__func__,ov13850_otp_vendor_module);
				return false;
			}
		}

		CMR_LOGW("%s vcm_offset_value = %d\n",__func__,vcm_offset_value);

		if (ov13850_otp.vcm_start <= vcm_offset_value)
		{
			CMR_LOGE("%s, ov13850_otp.vcm_start = 0x%x\n", __func__,ov13850_otp.vcm_start);
			ov13850_otp.vcm_start = 0;
		}
		else
		{
			ov13850_otp.vcm_start -= vcm_offset_value;
		}

		ov13850_otp.vcm_end += vcm_offset_value;

		if (ov13850_otp.vcm_end >= OV13850_OTP_VCM_END_MAX)
		{
			ov13850_otp.vcm_end = OV13850_OTP_VCM_END_MAX;
		}

		s_ctrl->afc_otp_info.starting_dac = ov13850_otp.vcm_start;
		s_ctrl->afc_otp_info.infinity_dac = ov13850_otp.vcm_start;
		s_ctrl->afc_otp_info.macro_dac = ov13850_otp.vcm_end;

		CMR_LOGW(" s_ctrl->afc_otp_info.starting_dac = 0x%x\n", s_ctrl->afc_otp_info.starting_dac);
		CMR_LOGW(" s_ctrl->afc_otp_info.infinity_dac = 0x%x\n", s_ctrl->afc_otp_info.starting_dac);
		CMR_LOGW(" s_ctrl->afc_otp_info.macro_dac = 0x%x\n", s_ctrl->afc_otp_info.macro_dac);
	}
	else //Abnormal branch deal
	{
		ov13850_otp.vcm_start = 0;
		ov13850_otp.vcm_end = 0;
		CMR_LOGE("%s VCM OTP data is worng! vcm_start=0x%x, vcm_end=0x%x\n",__func__, ov13850_otp.vcm_start,ov13850_otp.vcm_end);
		return false;
	}
	return true;
}

/*
 **************************************************************************
 * FunctionName: ov13850_get_otp_from_sensor;
 * Description : the api to use ov13850 sensor module OTP function;
 * Input         : NULL;
 * Output       : the ov13850_otp;
 * ReturnValue : true-OTP can be used, false-OTP is error, cannot be used;
 * Other         : NA;
 **************************************************************************
*/
static bool ov13850_get_otp_from_sensor(struct msm_sensor_ctrl_t *s_ctrl)
{
	u8   sum = 0;
	u8   otpCheckSumVal = 0;
	bool retVal = false;
	uint8_t otpflag = 0;
	uint16_t tmp_mmi_otp_flag = MMI_OTP_FLAG_FAILED_MASK; //set all mmi otp flag mask ,default:fail

#ifdef OV13850_OTP_READ_TIME_PRINT
	u16 otpReadUsed = 0;
	struct timeval otpReadBegin, otpReadEnd;
#endif

	CMR_LOGD("%s enters!\n",__func__);

        otpflag = ov13850_otp_get_flag();

	//Just check OTP once whether success or not
	if((otpflag & OV13850_OTP_FAIL_FLAG) == OV13850_OTP_FAIL_FLAG)
	{
		CMR_LOGE("%s OTP data is worng, ov13850_otp_flag=0x%04x\n", __func__,otpflag);
		return false;
	}
	else if((otpflag & OV13850_OTP_CHECKSUM_READ) == OV13850_OTP_CHECKSUM_READ)
	{
		CMR_LOGD("%s OTP has been read success already, ov13850_otp_flag=0x%01x\n", __func__,otpflag);
		return true;
	}

#ifdef OV13850_OTP_READ_TIME_PRINT
	memset(&otpReadBegin,0,sizeof(otpReadBegin));
	memset(&otpReadEnd,0,sizeof(otpReadEnd));
	do_gettimeofday(&otpReadBegin);
#endif

	retVal = ov13850_otp_read_checksum(s_ctrl,&otpCheckSumVal);
	if ((otpCheckSumVal == 0xFF) || (false == retVal))
	{
		CMR_LOGE("%s OTP data has not flashed!\n", __func__);
		goto OTP_FAIL;
	}

       //initial ov13850_otp
	memset(&ov13850_otp,0,sizeof(ov13850_otp));
	tmp_mmi_otp_flag &= ~OV13850_MMI_OTP_CHECKSUM_FLAG;

	retVal = ov13850_otp_read_id(s_ctrl);
	if (false == retVal)
	{
		CMR_LOGE("%s ov13850_otp_read_id() failed!\n",__func__);
		goto OTP_FAIL;
	}
	tmp_mmi_otp_flag &= ~OV13850_MMI_OTP_MODULE_INFO_FLAG;
	
	retVal = ov13850_otp_read_awb(s_ctrl);
	if (false == retVal)
	{
		CMR_LOGE("%s ov13850_otp_read_awb() failed!\n",__func__);
		goto OTP_FAIL;
	}
	tmp_mmi_otp_flag &= ~OV13850_MMI_OTP_AWB_FLAG;

	retVal = ov13850_otp_read_vcm(s_ctrl);
	if (false == retVal)
	{
		CMR_LOGE("%s ov13850_otp_read_vcm() failed!\n",__func__);
		goto OTP_FAIL;
	}
	tmp_mmi_otp_flag &= ~OV13850_MMI_OTP_VCM_FLAG;
	
	retVal = ov13850_otp_read_lsc(s_ctrl);
	if (false == retVal)
	{
		CMR_LOGE("%s ov13850_otp_read_vcm() failed!\n",__func__);
		goto OTP_FAIL;
	}
	tmp_mmi_otp_flag &= ~OV13850_MMI_OTP_LSC_FLAG;

	sum = (OTPSUMVAL - otpCheckSumVal) % 0xff;

#ifdef OV13850_OTP_READ_TIME_PRINT
	do_gettimeofday(&otpReadEnd);
	otpReadUsed = (otpReadEnd.tv_sec - otpReadBegin.tv_sec) * 1000 + (otpReadEnd.tv_usec - otpReadBegin.tv_usec) / 1000;
	CMR_LOGD("%s used time is %d ms\n", __func__,otpReadUsed);
#endif

	if (otpCheckSumVal == sum)
	{
		ov13850_otp_set_flag(OV13850_OTP_CHECKSUM_READ);
		otpflag = ov13850_otp_get_flag();
		tmp_mmi_otp_flag &= ~OV13850_MMI_OTP_SUMVAL_FLAG;
		CMR_LOGD("%s success, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d, ov13850_otp_flag=0x%x\n", __func__, OTPSUMVAL, otpCheckSumVal,sum,otpflag);
		s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
		CMR_LOGI("%s ov13850_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
		return true;
	}
	else
	{
	       ov13850_otp_set_flag(OV13850_OTP_CHECKSUM_ERR);
		CMR_LOGE("%s fail, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d \n", __func__, OTPSUMVAL, otpCheckSumVal,sum);
	}

OTP_FAIL:
	ov13850_otp_set_flag(OV13850_OTP_FAIL_FLAG);
	otpflag = ov13850_otp_get_flag();
	CMR_LOGD("%s ov13850_otp_flag=0x%x \n", __func__,otpflag);
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	CMR_LOGE("%s ov13850_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
	return false;
}
/*
 **************************************************************************
 * FunctionName: ov13850_set_otp_to_sensor;
 * Description : the api to use ov13850 sensor module OTP function;
 * Input         : NULL;
 * Output       : NA;
 * ReturnValue:NA;
 * Other         : NA;
 **************************************************************************
*/
static void ov13850_set_otp_to_sensor(struct msm_sensor_ctrl_t * s_ctrl)
{
	uint8_t otpflag = ov13850_otp_get_flag();

	if (( otpflag&OV13850_OTP_FAIL_FLAG) != OV13850_OTP_FAIL_FLAG)
	{
		ov13850_otp_set_lsc(s_ctrl);
		ov13850_otp_set_awb(s_ctrl);
	}

	return;
}

/*
 **************************************************************************
 * FunctionName: ov13850_module_i2c_opt_interface;
 * Description : the i2c api to use ov13850 module;
 * Input         : NULL;
 * Output       : NA;
 * ReturnValue :NONEl ;
 * Other         : NA;
 **************************************************************************
*/
static int ov13850_module_i2c_opt_interface(struct msm_sensor_ctrl_t * s_ctrl,uint16_t i2c_addr,enum msm_camera_i2c_reg_addr_type addr_type)
{
	int rc = 0;
	CMR_LOGD("%s enters! i2c_addr = 0x%x.\n",__func__,i2c_addr);
	s_ctrl->sensor_i2c_client->addr_type = addr_type;

	if (s_ctrl->sensor_i2c_client->cci_client)
	{
		s_ctrl->sensor_i2c_client->cci_client->sid = (i2c_addr >> 1);
	}
	else if (s_ctrl->sensor_i2c_client->client)
	{
		s_ctrl->sensor_i2c_client->client->addr = (i2c_addr >> 1);
	}
	else
	{
		CMR_LOGE("%s: error: no i2c/cci client found.", __func__);
		rc = -EFAULT;
	}

	return rc;

}

/******************************************************************************
Function   :  ov13850_otp_func
Description:  read the otp info
******************************************************************************/
int ov13850_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index)
{
	 int rc = 0;
	CMR_LOGD("%s enters!\n",__func__);
	if (otp_function_lists[index].rg_ratio_typical)
	{
		rg_ratio_typical = otp_function_lists[index].rg_ratio_typical;
	}

	if (otp_function_lists[index].bg_ratio_typical)
	{
		bg_ratio_typical = otp_function_lists[index].bg_ratio_typical;
	}

	/* i2c opt in EEPROM module */
	rc = ov13850_get_otp_from_sensor(s_ctrl);

       /* i2c opt in ov13850 sensor module */
	ov13850_module_i2c_opt_interface(s_ctrl,OV13850_SLAVE_ADDR,MSM_CAMERA_I2C_WORD_ADDR);

	if (false == rc)
	{
		return -1;
	}

	ov13850_set_otp_to_sensor(s_ctrl);
	CMR_LOGD("%s, rg_ratio_typical=%04x,bg_ratio_typical=%04x\n", __func__,rg_ratio_typical,bg_ratio_typical);
	CMR_LOGD("%s, the OTP read and set end.\n", __func__);
	return rc;
}

