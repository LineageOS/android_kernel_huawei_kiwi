
#define HW_CMR_LOG_TAG "sensor_otp_s5k3m2"

#include <linux/hw_camera_common.h>
#include <media/msm_cam_sensor.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"
#include <linux/slab.h>

#define S5K3M2_SLAVE_ADDR 0x5A //sensor i2c addr
#define S5K3M2_MODULE_HUAWEI_ID 0xCE
#define S5K3M2_GROUP_FLAG_NUM 3

#define S5K3M2_MI_AWB 0
#define S5K3M2_VCM 1
#define S5K3M2_LSC 2

static uint16_t group1_flag_addr[S5K3M2_GROUP_FLAG_NUM] = {0x0A04, 0x0A14, 0x0A1A};
static uint16_t group2_flag_addr[S5K3M2_GROUP_FLAG_NUM] = {0x0A24, 0x0A34, 0x0A3A};

static  uint32_t rg_ratio_typical = 0x1FE;   //the average of 4 Golden samples' RG ratio
static  uint32_t bg_ratio_typical = 0x224;   //the average of 4 Golden samples' BG ratio

typedef struct s5k3m2_otp_struct_type
{
	uint16_t product_year;
	uint16_t product_mouth;
	uint16_t product_date;
	uint16_t module_huawei_id;
	uint16_t module_vendor_id;
	uint16_t module_version;

	uint16_t awb_rg;
	uint16_t awb_bg;
	uint16_t awb_grgb;
	uint16_t vcm_start;
	uint16_t vcm_end;
}s5k3m2_otp_struct;

static  s5k3m2_otp_struct s5k3m2_otp;

typedef struct s5k3m2_otp_addr_struct_type
{
	uint16_t module_id_addr;
	uint16_t awb_addr;
	uint16_t vcm_addr;
	uint16_t checksum_addr[3];
} s5k3m2_otp_addr_struct;

static  s5k3m2_otp_addr_struct s5k3m2_otp_addr;

static  s5k3m2_otp_addr_struct s5k3m2_otp_group1_addr =
{
	.module_id_addr = 0x0A06,
	.awb_addr = 0x0A0E,
	.vcm_addr = 0x0A16,
	.checksum_addr = {0x0A05, 0x0A15, 0x0A1B},
};

static  s5k3m2_otp_addr_struct s5k3m2_otp_group2_addr =
{
	.module_id_addr = 0x0A26,
	.awb_addr = 0x0A2E,
	.vcm_addr = 0x0A36,
	.checksum_addr = {0x0A25, 0x0A35, 0x0A3B},
};

typedef struct s5k3m2_lsc_block_t
{
	uint16_t start_reg;
	uint16_t len;
	uint16_t page;
} s5k3m2_lsc_block;

static s5k3m2_lsc_block lsc_block_array1[9] =
{
	{0x0A2C, 24, 0x0100},
	{0x0A04, 64, 0x0200},
	{0x0A04, 64, 0x0300},
	{0x0A04, 64, 0x0400},
	{0x0A04, 64, 0x0500},
	{0x0A04, 64, 0x0600},
	{0x0A04, 64, 0x0700},
	{0x0A04, 64, 0x0800},
	{0x0A04, 12, 0x0900},
};

static s5k3m2_lsc_block lsc_block_array2[8] =
{
	{0x0A10, 52, 0x0900},
	{0x0A04, 64, 0x0A00},
	{0x0A04, 64, 0x0B00},
	{0x0A04, 64, 0x0C00},
	{0x0A04, 64, 0x0D00},
	{0x0A04, 64, 0x0E00},
	{0x0A04, 64, 0x0F00},
	{0x0A04, 48, 0x1000},
};

static uint16_t s5k3m2_otp_lsc_group;

/*OTP READ STATUS*/
#define S5K3M2_OTP_ID_READ            (1 << 0)
#define S5K3M2_OTP_AF_READ            (1 << 1)
#define S5K3M2_OTP_AWB_READ           (1 << 2)
#define S5K3M2_OTP_LSC_READ           (1 << 3)
#define S5K3M2_OTP_CHECKSUM_READ      (1 << 4)
#define S5K3M2_OTP_CHKSUM_SUCC        (1 << 6)
#define S5K3M2_OTP_FAIL_FLAG          (1 << 7)

#define S5K3M2_OTP_SUCCESS (S5K3M2_OTP_ID_READ|S5K3M2_OTP_AF_READ| \
S5K3M2_OTP_AWB_READ|S5K3M2_OTP_LSC_READ|S5K3M2_OTP_CHECKSUM_READ|S5K3M2_OTP_CHKSUM_SUCC)

#define S5K3M2_MMI_OTP_VCM_FLAG          (1 << 0)
#define S5K3M2_MMI_OTP_AWB_FLAG          (1 << 1)
#define S5K3M2_MMI_OTP_MODULE_INFO_FLAG  (1 << 2)
#define S5K3M2_MMI_OTP_LSC_FLAG          (1 << 3)
#define S5K3M2_MMI_OTP_CHECKSUM_FLAG     (1 << 4)
#define S5K3M2_MMI_OTP_SUMVAL_FLAG       (1 << 5)

static uint32_t OTPSUMVAL[3];

static uint8_t  s5k3m2_otp_flag   = 0;

/****************************************************************************
* FunctionName: s5k3m2_otp_i2c_read;
* Description : read data to sensor registers.
* Input       : s_ctrl:the struct of sensor controller;
                    addr:the address of register
                    date:the value read from register
* Output      : NA;
* ReturnValue : rc<0 write faill;
* Other       : NA;
***************************************************************************/
static int32_t s5k3m2_otp_i2c_read(struct msm_sensor_ctrl_t* s_ctrl, uint32_t addr, uint8_t* data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		addr,
		&temp_data,
		MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	}

	*data = temp_data;

	return rc;
}
/****************************************************************************
* FunctionName: s5k3m2_otp_i2c_write;
* Description : write data to sensor registers.
* Input       : s_ctrl:the struct of sensor controller;
                    addr:the address of register
                    date:the value write to register
* Output      : NA;
* ReturnValue : rc<0 write faill;
* Other       : NA;
***************************************************************************/
static int32_t s5k3m2_otp_i2c_write(struct msm_sensor_ctrl_t* s_ctrl, uint32_t addr, uint16_t data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		addr, data,
		MSM_CAMERA_I2C_WORD_DATA);

	if (rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: s5k3m2_otp_get_chksum;
* Description : Get checksum OTP from sensor;
* Input       : s_ctrl:the struct of sensor controller;
                    checksum:the temp address to save value from checksum register
                    i:the number of checksum group
* Output      : NA;
* ReturnValue: true - Success
                       false - Fail
* Other       : ;
***************************************************************************/
static bool s5k3m2_otp_get_chksum(struct msm_sensor_ctrl_t* s_ctrl, uint8_t* checksum ,int i)
{
	int rc = 0;

	rc = s5k3m2_otp_i2c_read(s_ctrl, s5k3m2_otp_addr.checksum_addr[i], checksum);

	if ( (rc < 0) || (0 == *checksum))
	{
		CMR_LOGE("read checksum[%d] ,addr:0x%x failed" , i , s5k3m2_otp_addr.checksum_addr[i]);
		return false;
	}

	CMR_LOGW("%s checksum[%d] = 0x%x\n", __func__, i ,*checksum);
	return true;
}

/****************************************************************************
* FunctionName: s5k3m2_otp_get_module_id;
* Description : Get module id OTP from sensor;
* Input       : s_ctrl:the struct of sensor controller;
* Output      : NA;
* ReturnValue: true - Success
                       false - Fail
* Other       : ;
***************************************************************************/
static bool s5k3m2_otp_get_module_id(struct msm_sensor_ctrl_t* s_ctrl)
{
	uint8_t buf[8] = {0};
	int rc = 0;
	int i = 0;

	for ( i = 0; i < 8; i++)
	{
		rc = s5k3m2_otp_i2c_read(s_ctrl, s5k3m2_otp_addr.module_id_addr + i , &buf[i]);
		if ( rc < 0 )
		{
			CMR_LOGE("%s, read module id fail addr:0x%x\n", __func__ ,s5k3m2_otp_addr.module_id_addr + i);
			return false;
		}

		OTPSUMVAL[S5K3M2_MI_AWB] += buf[i];
	}

	CMR_LOGW("%s module info year 20%02d month %d day %d. huawei_id 0x%x,  vendor id&version 0x%x\n",
			__func__, buf[0], buf[1], buf[2], buf[3], buf[4]);
	CMR_LOGW("%s reserved reg value:0x%x,0x%x,0x%x\n", __func__, buf[5], buf[6], buf[7]);

	if (buf[3] != S5K3M2_MODULE_HUAWEI_ID)
	{
		CMR_LOGE("%s, huawei_id is error!module matching failed\n", __func__);
		return false;
	}

	return true;
}

/****************************************************************************
* FunctionName: s5k3m2_otp_get_awb;
* Description : Get AWB OTP from sensor;
* Input       : s_ctrl:the struct of sensor controller;
* Output      : NA;
* ReturnValue: true - Success
                       false - Fail
* Other       : ;
***************************************************************************/
static bool s5k3m2_otp_get_awb(struct msm_sensor_ctrl_t* s_ctrl)
{
	uint8_t buf[6] = {0};
	int rc = 0;
	int i = 0;

	for (i = 0; i < 6 ; i++)
	{
		rc = s5k3m2_otp_i2c_read(s_ctrl, s5k3m2_otp_addr.awb_addr + i, &buf[i]);
		if ( rc < 0)
		{
			CMR_LOGE("%s, read awb fail, addr:0x%x\n", __func__ , s5k3m2_otp_addr.awb_addr + i);
			return false;
		}

		OTPSUMVAL[S5K3M2_MI_AWB] += buf[i];
	}

	s5k3m2_otp.awb_rg = buf[0];
	s5k3m2_otp.awb_rg <<= 8;
	s5k3m2_otp.awb_rg += buf[1];

	s5k3m2_otp.awb_bg = buf[2];
	s5k3m2_otp.awb_bg <<= 8;
	s5k3m2_otp.awb_bg += buf[3];

	s5k3m2_otp.awb_grgb = buf[4];
	s5k3m2_otp.awb_grgb <<= 8;
	s5k3m2_otp.awb_grgb += buf[5];

	CMR_LOGW("%s awb OTP data: R/G=0x%x, B/G=0x%x, Gb/Gr=0x%x\n", __func__,
				s5k3m2_otp.awb_rg, s5k3m2_otp.awb_bg, s5k3m2_otp.awb_grgb);

	if (0 == s5k3m2_otp.awb_rg || 0 == s5k3m2_otp.awb_bg || 0 == s5k3m2_otp.awb_grgb)
	{
		CMR_LOGE("%s awb OTP must not be zero, error!\n", __func__);
		return false;
	}

	return true;
}

/****************************************************************************
* FunctionName: s5k3m2_otp_get_lsc;
* Description : Get LSC OTP from sensor;
* Input       : s_ctrl:the struct of sensor controller;
* Output      : NA;
* ReturnValue: true - Success
                       false - Fail
* Other       : S5K3M2 can auto load LSC, so only read the LSC value and calculate checksum here;
***************************************************************************/
static bool s5k3m2_otp_get_lsc(struct msm_sensor_ctrl_t* s_ctrl)
{
	int rc  = 0;
	int i , j = 0;
	uint8_t val = 0;
	s5k3m2_lsc_block *otp_lsc = NULL;
	uint8_t size = 0;

	if (s5k3m2_otp_lsc_group == 1)
	{
		otp_lsc = lsc_block_array1;
		size = ARRAY_SIZE(lsc_block_array1);
	} else
	{
		otp_lsc = lsc_block_array2;
		size = ARRAY_SIZE(lsc_block_array2);
	}

	//S5K3M2 can auto load LSC, so only read the LSC value and calculate checksum here
	for ( i = 0 ; i < size ; i++)
	{
		rc = s5k3m2_otp_i2c_write(s_ctrl, 0x0A02, (otp_lsc + i)->page);
		if (rc < 0)
		{
			CMR_LOGE("write page 0x%x to 0x0A02 failed", (otp_lsc + i)->page);
			return false;
		}
		rc = s5k3m2_otp_i2c_write(s_ctrl, 0x0A00, 0x0100);
		if (rc < 0)
		{
			CMR_LOGE("write enable failed ,page:0x%x", (otp_lsc + i)->page);
			return false;
		}

		msleep(10);//delay 10ms , NVM data is available in RAM and may be read

		for ( j = 0 ; j < (otp_lsc + i)->len ; j++)
		{
			rc = s5k3m2_otp_i2c_read(s_ctrl, ((otp_lsc + i)->start_reg + j) , &val);
			if (rc < 0)
			{
				CMR_LOGE("read page 0x%x reg 0x%x failed",
					(otp_lsc + i)->page ,(otp_lsc + i)->start_reg + j);
				return false;
			}

			OTPSUMVAL[S5K3M2_LSC] += val;
		}
	}

	return true;
}

/****************************************************************************
* FunctionName: s5k3m2_otp_get_af;
* Description : Get VCM OTP from sensor;
* Input       : s_ctrl:the struct of sensor controller;
* Output      : NA;
* ReturnValue: true - Success
                       false - Fail
* Other       : NA;
***************************************************************************/
static bool s5k3m2_otp_get_vcm(struct msm_sensor_ctrl_t* s_ctrl)
{
	uint8_t  buf[4] = {0};
	uint16_t vcm_start = 0;
	uint16_t vcm_end = 0;
	int rc = 0;
	int i = 0;

	for ( i = 0; i < 4 ; i++)
	{
		rc = s5k3m2_otp_i2c_read(s_ctrl, s5k3m2_otp_addr.vcm_addr + i, &buf[i]);
		if ( rc < 0 )
		{
			CMR_LOGE("%s, read vcm fail ,addr:0x%x\n", __func__, s5k3m2_otp_addr.vcm_addr + i);
			return false;
		}

		OTPSUMVAL[S5K3M2_VCM] += buf[i];
	}

	vcm_start = buf[0];
	vcm_start <<= 8;
	vcm_start += buf[1];
	vcm_end   = buf[2];
	vcm_end   <<= 8;
	vcm_end   += buf[3];

	if ((vcm_end > vcm_start) && (vcm_start != 0) && ( vcm_end  != 0))
	{
		s5k3m2_otp.vcm_start = vcm_start;
		s5k3m2_otp.vcm_end= vcm_end;
		CMR_LOGW("%s vcm_start=0x%x, vcm_end=0x%x \n", __func__,
				s5k3m2_otp.vcm_start, s5k3m2_otp.vcm_end );
	}
	else
	{
		s5k3m2_otp.vcm_start = 0;
		s5k3m2_otp.vcm_end = 0;
		CMR_LOGE("%s VCM OTP data is worng! vcm_start=0x%x, vcm_end=0x%x\n", __func__,
				s5k3m2_otp.vcm_start, s5k3m2_otp.vcm_end);
		return false;
	}

	return true;
}

/*
**************************************************************************
* FunctionName: s5k3m2_prepare_to_read_otp;
* Description : init PLL setting ,prepare to read;
* Input         : s_ctrl:the struct of sensor controller;
* Output       : NA;
* ReturnValue: true - Success
                       false - Fail
* Other         : NA;
**************************************************************************
*/
static bool s5k3m2_prepare_to_read_otp(struct msm_sensor_ctrl_t* s_ctrl)
{
	int rc = 0;
	int i = 0;

	struct msm_camera_i2c_reg_array pll_init_setting[] =
	{
		{0x0136,0x1800},
		{0x0304,0x0006},
		{0x0306,0x0073},
		{0x030C,0x0004},
		{0x030E,0x0064},
		{0x0302,0x0001},
		{0x0300,0x0004},
		{0x030A,0x0001},
		{0x0308,0x0008},
		{0x0100,0x0100},//stream on
	};

	for (i = 0; i < ARRAY_SIZE(pll_init_setting); i++ )
	{
		rc = s5k3m2_otp_i2c_write(s_ctrl, pll_init_setting[i].reg_addr, pll_init_setting[i].reg_data);
		if (rc < 0)
		{
			CMR_LOGE("write 0x%x %d failed",pll_init_setting[i].reg_addr, i);
			return false;
		}
	}

	msleep(10);//delay 10ms after stream on

	rc = s5k3m2_otp_i2c_write(s_ctrl, 0x0A02, 0x1F00);
	if (rc < 0)
	{
		CMR_LOGE("write page num 0x1F00 failed");
		return false;
	}
	rc = s5k3m2_otp_i2c_write(s_ctrl, 0x0A00, 0x0100);
	if (rc < 0)
	{
		CMR_LOGE("NVM enable failed at page 0x1F00");
		return false;
	}

	msleep(10);//delay 10ms , NVM data is available in RAM and may be read

	return true;
}

/*
**************************************************************************
* FunctionName: s5k3m2_otp_group_select;
* Description : select the valid group of 3 blocks;
* Input         :  s_ctrl:the struct of sensor controller;
* Output       : NA;
* ReturnValue: true - Success
                       false - Fail
* Other         : NA;
**************************************************************************
*/
static bool s5k3m2_otp_group_select(struct msm_sensor_ctrl_t* s_ctrl)
{
	int rc = 0;
	int i = 0;
	uint8_t val = 0;

	for (i = 0; i < S5K3M2_GROUP_FLAG_NUM ; i++)
	{
		//read the group flag from group2
		rc = s5k3m2_otp_i2c_read(s_ctrl, group2_flag_addr[i], &val);
		if (rc != 0)
		{
			CMR_LOGE("%s i2c read fail addr:0x%x \n", __func__, group2_flag_addr[i]);
			return false;
		}

		//the otp is bad if group2_flag[i] is 0x1X
		if ( (val & 0x10) != 0)
		{
			CMR_LOGE("%s ,All Group:%d invalid, otp flag error:0x%x!",__func__ , i, val);
			return false;
		}

		//if the group2_flag[i] is 0x00, select the group1's data
		if ( val == 0x00 )
		{
			CMR_LOGW("%s ,Group2:%d is empty , select Group1:%d!",__func__ , i, i);
			rc = s5k3m2_otp_i2c_read(s_ctrl, group1_flag_addr[i], &val);
			if (rc != 0)
			{
				CMR_LOGE("%s i2c read fail addr:0x%x \n", __func__, group1_flag_addr[i]);
				return false;
			}

			if ( (val & 0x10) != 0 )
			{
				CMR_LOGE("%s ,Group1:%d invalid, otp flag error:0x%x!",__func__ , i, val);
				return false;
			}

			if ( val == 0x00 )
			{
				CMR_LOGE("%s ,Group1:%d is empty too,all otp empty:%d!",__func__ , i, i);
				return false;
			}

			if (i == S5K3M2_MI_AWB)//module ID and AWB
			{
				s5k3m2_otp_addr.module_id_addr = s5k3m2_otp_group1_addr.module_id_addr;
				s5k3m2_otp_addr.awb_addr= s5k3m2_otp_group1_addr.awb_addr;
				s5k3m2_otp_addr.checksum_addr[0] = s5k3m2_otp_group1_addr.checksum_addr[0];
			}
			else if (i == S5K3M2_VCM)//VCM
			{
				s5k3m2_otp_addr.vcm_addr= s5k3m2_otp_group1_addr.vcm_addr;
				s5k3m2_otp_addr.checksum_addr[1] = s5k3m2_otp_group1_addr.checksum_addr[1];
			}
			else//i == S5K3M2_LSC
			{
				s5k3m2_otp_lsc_group = 1;
				s5k3m2_otp_addr.checksum_addr[2] = s5k3m2_otp_group1_addr.checksum_addr[2];
			}

			continue;
		}

		//if the group2_flag[i] is 0x01, select the group2's data
		if ( val == 0x01)
		{
			CMR_LOGW("%s ,Group2:%d  is valid , select Group2!",__func__ , i);
			if (i == S5K3M2_MI_AWB)//module ID and AWB
			{
				s5k3m2_otp_addr.module_id_addr = s5k3m2_otp_group2_addr.module_id_addr;
				s5k3m2_otp_addr.awb_addr= s5k3m2_otp_group2_addr.awb_addr;
				s5k3m2_otp_addr.checksum_addr[0] = s5k3m2_otp_group2_addr.checksum_addr[0];
			}
			else if (i == S5K3M2_VCM)//VCM
			{
				s5k3m2_otp_addr.vcm_addr= s5k3m2_otp_group2_addr.vcm_addr;
				s5k3m2_otp_addr.checksum_addr[1] = s5k3m2_otp_group2_addr.checksum_addr[1];
			}
			else//i == S5K3M2_LSC
			{
				s5k3m2_otp_lsc_group = 2;
				s5k3m2_otp_addr.checksum_addr[2] = s5k3m2_otp_group2_addr.checksum_addr[2];
			}
		}
	}

	return true;
}

/****************************************************************************
* FunctionName: s5k3m2_otp_checksum_calc;
* Description : calculate the checksum .
* Input       : checksum value;
* Output      : NA;
* ReturnValue (checksum % 0xFF) +1:
* Other       : NA;
***************************************************************************/
static uint8_t s5k3m2_otp_checksum_calc(uint32_t checksum)
{
	return (uint8_t)((checksum % 255) + 1);
}

/****************************************************************************
* FunctionName: s5k3m2_get_otp_from_sensor;
* Description : Get all the OTP info from sensor;
* Input       : s_ctrl:the struct of sensor controller;
* Output      : NA;
* ReturnValue :true-Success
                       false-Fail;
* Other       : NA;
***************************************************************************/
static bool s5k3m2_get_otp_from_sensor(struct msm_sensor_ctrl_t* s_ctrl)
{
	uint8_t checksum[3] = {0};
	int i = 0;
	bool rc = false;
	uint16_t tmp_mmi_otp_flag = 0x3F;//set all mmi otp flag mask ,default:fail

	CMR_LOGD("%s, enter\n", __func__);

	if (S5K3M2_OTP_FAIL_FLAG == (s5k3m2_otp_flag & S5K3M2_OTP_FAIL_FLAG))
	{
		CMR_LOGE("%s, S5K3M2_OTP_FAIL_FLAG\n", __func__);
		return false;
	}
	else if (S5K3M2_OTP_SUCCESS == s5k3m2_otp_flag)
	{
		CMR_LOGW("%s, S5K3M2_OTP_COMPLETE, no need to read again\n", __func__);
		return true;
	}

	/* prepare to read otp */
	rc = s5k3m2_prepare_to_read_otp(s_ctrl);
	if (false == rc)
	{
		CMR_LOGE("%s init PLL failed \n", __func__);
		goto GET_OTP_FAIL;
	}

	/* select group */
	rc = s5k3m2_otp_group_select(s_ctrl);
	if (false == rc)
	{
		CMR_LOGE("%s otp group select failed\n", __func__);
		goto GET_OTP_FAIL;
	}

	s5k3m2_otp_flag = 0;
	OTPSUMVAL[S5K3M2_MI_AWB] = 0;
	OTPSUMVAL[S5K3M2_VCM] = 0;
	OTPSUMVAL[S5K3M2_LSC] = 0;

	/*read checksum register*/
	for ( i = 0; i < S5K3M2_GROUP_FLAG_NUM ; i++)
	{
		rc = s5k3m2_otp_get_chksum(s_ctrl, &checksum[i], i);
		if ( false == rc )
		{
			goto GET_OTP_FAIL;
		}
	}

	s5k3m2_otp_flag |= S5K3M2_OTP_CHECKSUM_READ;
	tmp_mmi_otp_flag &= ~S5K3M2_MMI_OTP_CHECKSUM_FLAG;

	/*read module id register*/
	rc = s5k3m2_otp_get_module_id(s_ctrl);
	if ( false == rc )
	{
		goto GET_OTP_FAIL;
	}

	/*read awb registers */
	rc = s5k3m2_otp_get_awb(s_ctrl);
	if ( false == rc )
	{
		goto GET_OTP_FAIL;
	}

	/* verify checksum for MODULE ID and AWB */
	if( checksum[S5K3M2_MI_AWB] != s5k3m2_otp_checksum_calc(OTPSUMVAL[S5K3M2_MI_AWB]))
	{
		CMR_LOGE("%s MODULE ID and AWB checksum error, checksum_reg_value:0x%x, SUMVAL=%d",
			__func__ , checksum[S5K3M2_MI_AWB] , OTPSUMVAL[S5K3M2_MI_AWB]);
		goto GET_OTP_FAIL;
	}

	s5k3m2_otp_flag |= S5K3M2_OTP_ID_READ;
	tmp_mmi_otp_flag &= ~S5K3M2_MMI_OTP_MODULE_INFO_FLAG;
	s5k3m2_otp_flag |= S5K3M2_OTP_AWB_READ;
	tmp_mmi_otp_flag &= ~S5K3M2_MMI_OTP_AWB_FLAG;

	/*read vcm registers*/
	rc = s5k3m2_otp_get_vcm(s_ctrl);
	if ( false == rc )
	{
		goto GET_OTP_FAIL;
	}

	/* verify checksum for VCM */
	if( checksum[S5K3M2_VCM] != s5k3m2_otp_checksum_calc(OTPSUMVAL[S5K3M2_VCM]))
	{
		CMR_LOGE("%s VCM checksum verify error, checksum_reg_value:0x%x, SUMVAL=%d",
			__func__ , checksum[S5K3M2_VCM] , OTPSUMVAL[S5K3M2_VCM]);
		goto GET_OTP_FAIL;
	}

	s5k3m2_otp_flag |= S5K3M2_OTP_AF_READ;
	tmp_mmi_otp_flag &= ~S5K3M2_MMI_OTP_VCM_FLAG;

	/*read lsc registers*/
	rc = s5k3m2_otp_get_lsc(s_ctrl);
	if ( false == rc )
	{
		goto GET_OTP_FAIL;
	}

	/* verify checksum for VCM */
	if( checksum[S5K3M2_LSC] != s5k3m2_otp_checksum_calc(OTPSUMVAL[S5K3M2_LSC]))
	{
		CMR_LOGE("%s LSC checksum verify error, checksum_reg_value:0x%x, SUMVAL=%d",
			__func__ , checksum[S5K3M2_LSC] , OTPSUMVAL[S5K3M2_LSC]);
		goto GET_OTP_FAIL;
	}

	CMR_LOGW("%s LSC verifying succeed\n", __func__);

	s5k3m2_otp_flag |= S5K3M2_OTP_LSC_READ;
	tmp_mmi_otp_flag &= ~S5K3M2_MMI_OTP_LSC_FLAG;

	s5k3m2_otp_flag |= S5K3M2_OTP_CHKSUM_SUCC;
	tmp_mmi_otp_flag &= ~S5K3M2_MMI_OTP_SUMVAL_FLAG;

	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	CMR_LOGW("%s s5k3m2_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);

	rc = s5k3m2_otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	if (rc < 0)
	{
		CMR_LOGE("%s NVM control failed", __func__);
		goto NVM_CONTROL_FAIL;
	}

	return true;

GET_OTP_FAIL:
	CMR_LOGE("%s fail \n", __func__);
	rc = s5k3m2_otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	if (rc < 0)
	{
		CMR_LOGE("%s NVM control failed", __func__);
	}
	s5k3m2_otp_flag |= S5K3M2_OTP_FAIL_FLAG;
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	CMR_LOGE("%s s5k3m2_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
	return false;
NVM_CONTROL_FAIL:
	s5k3m2_otp_flag |= S5K3M2_OTP_FAIL_FLAG;
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	CMR_LOGE("%s s5k3m2_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
	return false;
}

/*
**************************************************************************
* FunctionName: s5k3m2_set_otp_to_platform;
* Description : set the otp info into sctrl;
* Input         : s_ctrl:the struct of sensor controller;
* Output       : NA;
* ReturnValue:NA;
* Other         : NA;
**************************************************************************
*/
static void s5k3m2_set_otp_to_platform(struct msm_sensor_ctrl_t* s_ctrl)
{

	if (S5K3M2_OTP_FAIL_FLAG == (s5k3m2_otp_flag & S5K3M2_OTP_FAIL_FLAG))
	{
		CMR_LOGE("%s invalid otp info!\n", __func__);
		return;
	}

	/*set AWB*/
	s_ctrl->awb_otp_info.RG = s5k3m2_otp.awb_rg;
	s_ctrl->awb_otp_info.BG = s5k3m2_otp.awb_bg;
	s_ctrl->awb_otp_info.typical_RG = rg_ratio_typical;
	s_ctrl->awb_otp_info.typical_BG= bg_ratio_typical;

	/*set VCM*/
	s_ctrl->afc_otp_info.starting_dac = s5k3m2_otp.vcm_start;
	s_ctrl->afc_otp_info.infinity_dac = s5k3m2_otp.vcm_start;
	s_ctrl->afc_otp_info.macro_dac = s5k3m2_otp.vcm_end;

	return;
}

/*
**************************************************************************
* FunctionName: s5k3m2_otp_func;
* Description : Get OTP info from sensor, and set the info into sctrl.
* Input         : s_ctrl:the struct of sensor controller
                      index:the index of sensor list
* Output       : NA;
* ReturnValue :-1 fail, 0 success;
* Other         : NA;
**************************************************************************
*/
int s5k3m2_otp_func(struct msm_sensor_ctrl_t* s_ctrl, int index)
{
	int rc = false;
	CMR_LOGW("%s enters!\n", __func__);

	/* read otp info */
	rc = s5k3m2_get_otp_from_sensor(s_ctrl);

	if (false == rc)
	{
		CMR_LOGE("%s get otp failed!\n", __func__);
		return -1;
	}

	/*set otp to platform*/
	s5k3m2_set_otp_to_platform(s_ctrl);

	CMR_LOGW("%s OTP operation succeed.\n", __func__ );

	return 0;
}
