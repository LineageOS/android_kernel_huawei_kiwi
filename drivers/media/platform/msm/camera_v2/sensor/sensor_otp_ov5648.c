

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_ov5648"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"


struct ov5648_otp_struct {
    uint16_t product_year;//product year
    uint16_t product_month;//product month
    uint16_t product_date;//product date
    uint16_t camera_id;//hw camera id
    uint16_t supplier_version_id;//supplier version code
    uint16_t version;//camera version
    uint16_t wb_rg_h;//rg high bits
    uint16_t wb_rg_l;// rg low bits
    uint16_t wb_bg_h;//bg hight bits
    uint16_t wb_bg_l;//bg low bits
    uint16_t wb_gbgr_h;//gr/gb high bits
    uint16_t wb_gbgr_l;//gb/gr low bits
};

//R/G and B/G value of Golden Samples.
static  uint32_t rg_ratio_typical = 0x26c;//the average of 4 Golden samples' RG ratio
static  uint32_t bg_ratio_typical = 0x2ed;//the average of 4 Golden samples' BG ratio

//Final r/g/b gain
static uint32_t gr_gain = 0;
static uint32_t gg_gain = 0;
static uint32_t gb_gain = 0;

//OTP read indications
static  uint32_t ov5648_otp_read_flag = 0 ;

//OV5648 opt has 2 bank
#define OV5648_OTP_BANK0    0
#define OV5648_OTP_BANK1    1

//bit0:module info, bit1:AWB
#define OV5648_OTP_MODULE_INFO          1
#define OV5648_OTP_AWB                  2
#define OV5648_OTP_ALL  (OV5648_OTP_MODULE_INFO | OV5648_OTP_AWB)

#define OV5648_MMI_OTP_AWB_FLAG          (1 << 1)
#define OV5648_MMI_OTP_MODULE_INFO_FLAG  (1 << 2)

/******************************************************************************
OTP memory Map:
                          Group 1
0x05~0x09    Bank 0:0x3d05~0x3d09    Module info
0x0a~0x0f    Bank 0:0x3d0a~0x3d0f    AWB
                          Group 2
0x10~0x14    Bank 1:0x3d00~0x3d04    Module info
0x15~0x1a    Bank 1:0x3d05~0x3d0a    AWB
******************************************************************************/


/****************************************************************************
* FunctionName: ov5648_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t ov5648_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

	if(rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}


/****************************************************************************
* FunctionName: ov5648_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t ov5648_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&temp_data, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	}
	*data = temp_data;
	return rc;
}

/******************************************************************************
Function   :    ov5648_otp_debug
Description:    Only for debug use
******************************************************************************/
void ov5648_otp_debug(struct ov5648_otp_struct otp_ptr)
{
    CMR_LOGD("%s,otp_ptr.product_year:%d\n",__func__,otp_ptr.product_year);
    CMR_LOGD("%s,otp_ptr.product_month:%d\n",__func__,otp_ptr.product_month);
    CMR_LOGD("%s,otp_ptr.product_date:%d\n",__func__,otp_ptr.product_date);
    CMR_LOGD("%s,otp_ptr.camera_id:0x%x\n",__func__,otp_ptr.camera_id);
    CMR_LOGD("%s,otp_ptr.supplier_version_id:0x%x\n",__func__,otp_ptr.supplier_version_id);
    CMR_LOGD("%s,otp_ptr.wb_rg:0x%x\n",__func__, (otp_ptr.wb_rg_h << 8) + otp_ptr.wb_rg_l);
    CMR_LOGD("%s,otp_ptr.wb_bg:0x%x\n",__func__, (otp_ptr.wb_bg_h << 8) + otp_ptr.wb_bg_l);
    CMR_LOGD("%s,otp_ptr.wb_gbgr:0x%x\n",__func__,(otp_ptr.wb_gbgr_h << 8) + otp_ptr.wb_gbgr_l);
}

/******************************************************************************
Function   :    ov5648_otp_clear
Description:    Disable  OTP read and clear otp buffers, must called wehen read anothe bank.
******************************************************************************/
void ov5648_otp_clear(struct msm_sensor_ctrl_t * s_ctrl)
{
    int i = 0;
    //Disable OTP read
    ov5648_cci_i2c_write(s_ctrl, 0x3d81, 0x00);

    for ( i = 0; i < 16; i++)//16 regs in on bank
    {
        //Clear otp buffers
        ov5648_cci_i2c_write(s_ctrl, 0x3d00 + i, 0x00);//0x3d00~0x3d0f
    }

    return;
}

/******************************************************************************
Function   :    ov5648_select_bank
Description:    Select the specific bankNum. OTP will be read into IC's internel men.
******************************************************************************/
static int ov5648_select_bank(struct msm_sensor_ctrl_t *s_ctrl, int bankNum)
{
    CMR_LOGD("%s, select bank num :%d\n",__func__, bankNum);
    if (OV5648_OTP_BANK0 == bankNum)
    {
        ov5648_cci_i2c_write(s_ctrl, 0x3d84, 0xc0);
        ov5648_cci_i2c_write(s_ctrl, 0x3d85, 0x00);//bank0
        ov5648_cci_i2c_write(s_ctrl, 0x3d86, 0x0f);
        ov5648_cci_i2c_write(s_ctrl, 0x3d81, 0x01);//start read

        mdelay(3);//must delay 3ms
        return 0;
    }
    else if (OV5648_OTP_BANK1 == bankNum)
    {
        ov5648_cci_i2c_write(s_ctrl, 0x3d84, 0xc0);
        ov5648_cci_i2c_write(s_ctrl, 0x3d85, 0x10);//bank1
        ov5648_cci_i2c_write(s_ctrl, 0x3d86, 0x1f);
        ov5648_cci_i2c_write(s_ctrl, 0x3d81, 0x01);//start read

        mdelay(3);//must delay 3ms
        return 0;
    }
    else
    {
        CMR_LOGE("%s, Error bank number!\n",__func__);
        return -1;
    }

    return 0;
}

/******************************************************************************
Function   :    ov5648_otp_read_module_info
Description:    read module info OTP.
******************************************************************************/
static int ov5648_otp_read_module_info(struct msm_sensor_ctrl_t *s_ctrl,
                                                                   struct ov5648_otp_struct * otp_ptr)
{
    //module info from group2,bank1 first
    if (-1 == ov5648_select_bank(s_ctrl, OV5648_OTP_BANK1))
    {
        return -1;
    }
    //Module info:  Bank 1:0x3d02~0x3d06
    ov5648_cci_i2c_read(s_ctrl, 0x3d00, &(otp_ptr->product_year));
    ov5648_cci_i2c_read(s_ctrl, 0x3d01, &(otp_ptr->product_month));
    ov5648_cci_i2c_read(s_ctrl, 0x3d02, &(otp_ptr->product_date));
    ov5648_cci_i2c_read(s_ctrl, 0x3d03, &(otp_ptr->camera_id));
    ov5648_cci_i2c_read(s_ctrl, 0x3d04, &(otp_ptr->supplier_version_id));
    CMR_LOGW("%s,group 2 otp_ptr.product_year:%d\n",__func__,otp_ptr->product_year);
    CMR_LOGW("%s,group 2 otp_ptr.product_month:%d\n",__func__,otp_ptr->product_month);
    CMR_LOGW("%s,group 2 otp_ptr.product_date:%d\n",__func__,otp_ptr->product_date);
    CMR_LOGW("%s,group 2 otp_ptr.camera_id:%d\n",__func__,otp_ptr->camera_id);
    CMR_LOGW("%s,group 2 otp_ptr.supplier_version_id:%d\n",__func__,otp_ptr->supplier_version_id);

    ov5648_otp_clear(s_ctrl);

    if (0 != otp_ptr->product_year)// year is not zero, we assume module info is correct
    {
        CMR_LOGW("%s, Module info lays in group 2\n",__func__);
        return 0;
    }
    //group2 fail, check group1, bank0
    if (-1 == ov5648_select_bank(s_ctrl, OV5648_OTP_BANK0))
    {
        return -1;
    }
    //Module info: Bank 0:0x3d05~0x3d09
    ov5648_cci_i2c_read(s_ctrl, 0x3d05, &(otp_ptr->product_year));
    ov5648_cci_i2c_read(s_ctrl, 0x3d06, &(otp_ptr->product_month));
    ov5648_cci_i2c_read(s_ctrl, 0x3d07, &(otp_ptr->product_date));
    ov5648_cci_i2c_read(s_ctrl, 0x3d08, &(otp_ptr->camera_id));
    ov5648_cci_i2c_read(s_ctrl, 0x3d09, &(otp_ptr->supplier_version_id));

    CMR_LOGW("%s,group 2 otp_ptr.product_year:%d\n",__func__,otp_ptr->product_year);
    CMR_LOGW("%s,group 2 otp_ptr.product_month:%d\n",__func__,otp_ptr->product_month);
    CMR_LOGW("%s,group 2 otp_ptr.product_date:%d\n",__func__,otp_ptr->product_date);
    CMR_LOGW("%s,group 2 otp_ptr.camera_id:%d\n",__func__,otp_ptr->camera_id);
    CMR_LOGW("%s,group 2 otp_ptr.supplier_version_id:%d\n",__func__,otp_ptr->supplier_version_id);

    ov5648_otp_clear(s_ctrl);

    if (0 != otp_ptr->product_year) // year is not zero, we assume module info is correct
    {
        CMR_LOGW("%s, Module info lays in group 1\n",__func__);
        return 0;//otp read sucess
    }
    else
    {
        CMR_LOGE("%s,  Error !No Module inf found in OTP\n",__func__);
        return -1;//otp read fail
    }
    return 0;
}

/******************************************************************************
Function   :    ov5648_otp_read_awb
Description:    read awb  info OTP.
******************************************************************************/
static int ov5648_otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl,
                                                   struct ov5648_otp_struct * otp_ptr)
{
    uint32_t rg = 0;
    uint32_t bg = 0;

    //AWB info from group2,bank1 first
    if (-1 == ov5648_select_bank(s_ctrl, OV5648_OTP_BANK1))
         return -1;

    //AWB:Bank 1:0x3d07~0x3d0c
    ov5648_cci_i2c_read(s_ctrl, 0x3d05, &(otp_ptr->wb_rg_h));
    ov5648_cci_i2c_read(s_ctrl, 0x3d06, &(otp_ptr->wb_rg_l));

    ov5648_cci_i2c_read(s_ctrl, 0x3d07, &(otp_ptr->wb_bg_h));
    ov5648_cci_i2c_read(s_ctrl, 0x3d08, &(otp_ptr->wb_bg_l));

    ov5648_cci_i2c_read(s_ctrl, 0x3d09, &(otp_ptr->wb_gbgr_h));
    ov5648_cci_i2c_read(s_ctrl, 0x3d0a, &(otp_ptr->wb_gbgr_l));

    rg = (otp_ptr->wb_rg_h << 8) + otp_ptr->wb_rg_l;
    bg = (otp_ptr->wb_bg_h << 8) + otp_ptr->wb_bg_l;

    ov5648_otp_clear(s_ctrl);
    //rG and bG all not zero, so awb info is correct
    if ((0 != rg) && (0 != bg))
    {
        CMR_LOGW("%s, AWB info lays in group 2\n",__func__);
        return 0;//otp read sucess
    }
    //one of rG and bG is not zero, so group 2 in not correct
    else if (((0 == rg) && ( 0 != bg )) || ((0 != rg) && ( 0 == bg )))
    {
        CMR_LOGE("%s,  Error !Group 2 AWB info not correct\n",__func__);
        return -1;//otp read fail
    }
    //group2 fail, check group1, bank0
     if (-1 == ov5648_select_bank(s_ctrl, OV5648_OTP_BANK0))
         return -1;

    //AWB : Bank 0:0x3d0a~0x3d0f
    ov5648_cci_i2c_read(s_ctrl, 0x3d0a, &(otp_ptr->wb_rg_h));
    ov5648_cci_i2c_read(s_ctrl, 0x3d0b, &(otp_ptr->wb_rg_l));

    ov5648_cci_i2c_read(s_ctrl, 0x3d0c, &(otp_ptr->wb_bg_h));
    ov5648_cci_i2c_read(s_ctrl, 0x3d0d, &(otp_ptr->wb_bg_l));

    ov5648_cci_i2c_read(s_ctrl, 0x3d0e, &(otp_ptr->wb_gbgr_h));
    ov5648_cci_i2c_read(s_ctrl, 0x3d0f, &(otp_ptr->wb_gbgr_l));

    rg = (otp_ptr->wb_rg_h << 8) + otp_ptr->wb_rg_l;
    bg = (otp_ptr->wb_bg_h<< 8) + otp_ptr->wb_bg_l;

    ov5648_otp_clear(s_ctrl);
    if ((0 != rg) && (0 != bg))//rG and bG all not zero, so awb info is correct
    {
        CMR_LOGW("%s, AWB info lays in group 1\n",__func__);
        return 0;//otp read sucess
    }
    else
    {
        CMR_LOGE("%s,  Error! No AWB info found in OTP\n",__func__);
        return -1;//otp read fail
    }
    return 0;
}




/******************************************************************************
Function   :    ov5648_read_otp
Description:    read module info , awb otp separeately,
                     and set coressponding bits
******************************************************************************/
void ov5648_read_otp(struct msm_sensor_ctrl_t *s_ctrl,
                                             struct ov5648_otp_struct * current_otp)
{
    uint8_t tmp_mmi_otp_flag = (OV5648_MMI_OTP_MODULE_INFO_FLAG | OV5648_MMI_OTP_AWB_FLAG);
	
    if (0 == ov5648_otp_read_module_info(s_ctrl, current_otp))
    {
        ov5648_otp_read_flag |= OV5648_OTP_MODULE_INFO;
		tmp_mmi_otp_flag &= ~OV5648_MMI_OTP_MODULE_INFO_FLAG;
    }
    if (0 == ov5648_otp_read_awb(s_ctrl, current_otp))
    {
        ov5648_otp_read_flag |= OV5648_OTP_AWB;
		tmp_mmi_otp_flag &= ~OV5648_MMI_OTP_AWB_FLAG;
    }
    CMR_LOGD("%s, ov5648_otp_read_flag:0x%x\n",__func__, ov5648_otp_read_flag);
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	
	CMR_LOGI("mmi_otp_check_flag = 0x%x\n", s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
	
    return;
}

/******************************************************************************
Function   :  ov5648_update_otp
Description:  Check, read OTP
******************************************************************************/
static int ov5648_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
    struct ov5648_otp_struct current_otp;

    uint32_t r_gain = 0;
    uint32_t g_gain = 0;
    uint32_t b_gain = 0;
    uint32_t g_gain_r = 0;
    uint32_t g_gain_b = 0;
    uint32_t rg = 0;
    uint32_t bg = 0;

    memset(&current_otp, 0, sizeof(struct ov5648_otp_struct));

    // R/G and B/G of current camera module is read out from sensor OTP
    ov5648_read_otp(s_ctrl, &current_otp);

    ov5648_otp_debug(current_otp);

    rg = (current_otp.wb_rg_h << 8) + current_otp.wb_rg_l;
    bg = (current_otp.wb_bg_h << 8) + current_otp.wb_bg_l;
    //check if AWB OTP is read correctly
    if (0 ==(ov5648_otp_read_flag & OV5648_OTP_AWB))
    {
        CMR_LOGE("%s Error!no AWB data in OTP!\n", __func__);
        return -1;
    }
    if ((0 == rg) || (0 == bg))//make sure rg and bg are not zero
    {
        CMR_LOGE("%s, Error Zero rg or bg, rg:%d, bg:%d\n", __func__, rg, bg);
        return -1;
    }

    CMR_LOGD("%s, r/g:0x%x, b/g:0x%x\n", __func__, rg, bg);
    //calculate G gain
    //0x400 = 1x gain
    if(bg < bg_ratio_typical)
    {
        if (rg < rg_ratio_typical)
        {
            // current_otp.bg_ratio < BG_Ratio_typical &&
            // current_otp.rg_ratio < RG_Ratio_typical
            g_gain = 0x400;
            b_gain = (0x400 * bg_ratio_typical) / bg;
            r_gain = (0x400 * rg_ratio_typical) / rg;
        }
        else
        {
            // current_otp.bg_ratio < BG_Ratio_typical &&
            // current_otp.rg_ratio >= RG_Ratio_typical
            r_gain = 0x400;
            g_gain = (0x400 * rg) / rg_ratio_typical;
            b_gain = (g_gain * bg_ratio_typical) / bg;
        }
    }
    else
    {
        if (rg < rg_ratio_typical)
        {
            // current_otp.bg_ratio >= BG_Ratio_typical &&
            // current_otp.rg_ratio < RG_Ratio_typical
            b_gain = 0x400;
            g_gain = (0x400 * bg) / bg_ratio_typical;
            r_gain = (g_gain * rg_ratio_typical) / rg;
        }
        else
        {
            // current_otp.bg_ratio >= BG_Ratio_typical &&
            // current_otp.rg_ratio >= RG_Ratio_typical
            g_gain_b = (0x400 * bg) / bg_ratio_typical;
            g_gain_r = (0x400 * rg) / rg_ratio_typical;

            if(g_gain_b > g_gain_r )
            {
                b_gain = 0x400;
                g_gain = g_gain_b;
                r_gain = (g_gain * rg_ratio_typical) /rg;
            }
            else
            {
                r_gain = 0x400;
                g_gain = g_gain_r;
                b_gain = (g_gain * bg_ratio_typical) / bg;
            }
        }
    }
    CMR_LOGD("%s, R_gain:0x%x,G_gain:0x%x,B_gain:0x%x\n", __func__, r_gain, g_gain, b_gain);
    gr_gain = r_gain;
    gg_gain = g_gain;
    gb_gain = b_gain;

    return 0;

}

/******************************************************************************
Function   :  ov5648_update_awb_gain
Description:  Update the RGB AWB gain to sensor
******************************************************************************/
void ov5648_update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl,
                                                        uint32_t r_gain,
                                                        uint32_t g_gain,
                                                        uint32_t b_gain)
{
    CMR_LOGD("%s, R_gain:0x%x,G_gain:0x%x,B_gain:0x%x\n",__func__, r_gain, g_gain, b_gain);
    if (r_gain > 0x400) {
        ov5648_cci_i2c_write(s_ctrl, 0x5186, r_gain >> 8);
        ov5648_cci_i2c_write(s_ctrl, 0x5187, r_gain & 0x00ff);
    }

    if (g_gain > 0x400) {
        ov5648_cci_i2c_write(s_ctrl, 0x5188, g_gain >> 8);
        ov5648_cci_i2c_write(s_ctrl, 0x5189, g_gain & 0x00ff);
    }

    if (b_gain > 0x400) {
        ov5648_cci_i2c_write(s_ctrl, 0x518a, b_gain >> 8);
        ov5648_cci_i2c_write(s_ctrl, 0x518b, b_gain & 0x00ff);
    }

    return;
}

/******************************************************************************
Function   :  ov5648_otp_func
Description:  read the otp info
******************************************************************************/
int ov5648_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index)
{

	if(otp_function_lists[index].rg_ratio_typical)
	{
		rg_ratio_typical = otp_function_lists[index].rg_ratio_typical;
	}

	if(otp_function_lists[index].bg_ratio_typical)
	{
		bg_ratio_typical = otp_function_lists[index].bg_ratio_typical;
	}
	CMR_LOGD("%s, rg_ratio_typical=%04x,bg_ratio_typical=%04x\n", __func__,rg_ratio_typical,bg_ratio_typical);


	if (0 == (ov5648_otp_read_flag & OV5648_OTP_AWB))
	{
		if(0 != ov5648_update_otp(s_ctrl))
		{
			//todo need to check whether stop the sensor
			CMR_LOGE("%s : No OTP info! return error!\n", __func__);
			return -1;
		}
	}
	else
	{
		CMR_LOGD("%s :AWB OTP has already been read out!\n",__func__);
	}
    ov5648_update_awb_gain(s_ctrl, gr_gain, gg_gain, gb_gain);

    return 0;
}
