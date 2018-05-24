
#define HW_CMR_LOG_TAG "sensor_otp_imx328_sunny_p13n10a"
#include <linux/hw_camera_common.h>
#include <media/msm_cam_sensor.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"



#define IMX328_SLAVE_ADDR 0x20

#define OTP_SLAVE_ADDR1       0xA0
#define OTP_SLAVE_ADDR2       0xAC

#define OTP_ID_REG            0x00
#define OTP_AWB_REG           0x05
#define OTP_VCM_REG           0x00
#define OTP_CHECKSUM_REG      0x05

/*OTP READ STATUS*/
#define IMX328_OTP_ID_READ        (1 << 0)
#define IMX328_OTP_VCM_READ       (1 << 1)
#define IMX328_OTP_AWB_READ       (1 << 2)
#define IMX328_OTP_CHECKSUM_READ  (1 << 3)
#define IMX328_OTP_CHECKSUM_ERR   (1 << 4)
#define IMX328_OTP_FAIL_FLAG      (1 << 5)

#define IMX328_SUNNY_MODULE_VENDOR_ID    0x01//SUNNY
#define IMX328_SUNNY_MODULE_HUAWEI_ID    0xA7//23060167

#define IMX328_MMI_OTP_VCM_FLAG          (1 << 0)
#define IMX328_MMI_OTP_AWB_FLAG          (1 << 1)
#define IMX328_MMI_OTP_MODULE_INFO_FLAG  (1 << 2)
#define IMX328_MMI_OTP_LSC_FLAG          (1 << 3)
#define IMX328_MMI_OTP_CHECKSUM_FLAG     (1 << 4)
#define IMX328_MMI_OTP_SUMVAL_FLAG       (1 << 5)

#define DIGITAL_GAIN_CALC_BASE 0x1000
#define DIGITAL_GAIN_BASE 0x100

#define IMX328_DIG_GAIN_GR_REG_H     (0x020E)
#define IMX328_DIG_GAIN_GR_REG_L     (0x020F)
#define IMX328_DIG_GAIN_GB_REG_H     (0x0214)
#define IMX328_DIG_GAIN_GB_REG_L     (0x0215)
#define IMX328_DIG_GAIN_R_REG_H      (0x0210)
#define IMX328_DIG_GAIN_R_REG_L      (0x0211)
#define IMX328_DIG_GAIN_B_REG_H      (0x0212)
#define IMX328_DIG_GAIN_B_REG_L      (0x0213)

//the value used for vcm effect, maybe modified by others
#define IMX328_SUNNY_OTP_VCM_OFFSET_VALUE    100
#define IMX328_SUNNY_OTP_VCM_END_MAX        1023

typedef struct {
    uint32_t rgain;
    uint32_t ggain;
    uint32_t bgain;
} awb_gain;

static awb_gain awb_gain_typical[] = {
    {0xe9d, 0x794, 0xaf5}, //sunny
};

typedef struct imx328_otp_struct_type {
    uint32_t rgain;
    uint32_t bgain;
    uint32_t ggain;
    uint32_t vcm_start;
    uint32_t vcm_end;
}imx328_otp_struct;

static imx328_otp_struct imx328_otp;

static uint32_t OTPSUMVAL         = 0;
static uint8_t  imx328_otp_flag   = 0;

static int32_t imx328_otp_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
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
static int32_t imx328_otp_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t data)
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
* FunctionName: imx328_otp_set_i2c_address;
* Description : set i2c address.
* Input       : NA;
* Output      : NA;
* ReturnValue : 
                -EFAULT Set FAIL;
* Other       : NA;
***************************************************************************/
static int imx328_otp_set_i2c_address(struct msm_sensor_ctrl_t * s_ctrl,uint16_t i2c_addr,enum msm_camera_i2c_reg_addr_type addr_type)
{
    int rc = 0;

    CMR_LOGI("%s enter! i2c_addr = 0x%x.\n",__func__,i2c_addr);

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

/****************************************************************************
* FunctionName: imx328_read_otp;
* Description : used to read OTP information from eeprom,
                and output the info in buf.
* Input       :
* Output      : sonyimx328_otp;
* ReturnValue :
               true, read successfully.
               false, read failed.
* Other       : NA;
***************************************************************************/
static bool imx328_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t i2c_addr,uint16_t reg,uint8_t *buf,uint16_t count)
{
    uint16_t i = 0;
    int ret = 0;
    uint16_t val = 0;
    bool rc = false;
    //set i2c address to read otp info
    ret = imx328_otp_set_i2c_address(s_ctrl,i2c_addr,MSM_CAMERA_I2C_BYTE_ADDR);
    if (ret < 0)
    {
        CMR_LOGE("%s fail to set i2c address\n", __func__);
        rc = false;
        goto OUT;
    }

    for (i=0; i<count; i++)
    {
        ret =imx328_otp_i2c_read(s_ctrl,(reg+i) ,&val);
        if (ret !=0)
        {
            CMR_LOGE("%s fail to read otp with error code %d, i2c_addr=0x%x reg_addr=0x%x\n", __func__,ret,i2c_addr,reg+i);
            rc = false;
            goto OUT;
        }
        buf[i] = (val&0xff);
        OTPSUMVAL += buf[i];
    }
    rc = true;

OUT:
    //resume camsensor i2c address no matter rc == true or false
    imx328_otp_set_i2c_address(s_ctrl,IMX328_SLAVE_ADDR,MSM_CAMERA_I2C_WORD_ADDR);
    return rc;
}

/****************************************************************************
* FunctionName: imx328_read_otp;
* Description : used to read OTP checksum value from eeprom,
* Input       :
* Output      : checksum;
* ReturnValue :
               true, read successfully.
               false, read failed.
* Other       : NA;
***************************************************************************/
static bool imx328_otp_read_checksum(struct msm_sensor_ctrl_t *s_ctrl, uint8_t *checksum)
{
    return imx328_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_CHECKSUM_REG,checksum,1);
}
/****************************************************************************
* FunctionName: imx328_read_otp;
* Description : used to read moudle id info value from eeprom
* Input       :
* Output      :
* ReturnValue :
                true, read successfully.
                false, read failed.
* Other       : NA;
***************************************************************************/
static bool imx328_otp_read_id(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t buf[5] = {0};
    uint8_t vendor_id = 0;

    imx328_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_ID_REG,buf,5);

    CMR_LOGI("%s module info year 20%02d month %d day %d, SNO. 0x%x  vendor id&version 0x%x\n",__func__,buf[0],buf[1],buf[2],buf[3],buf[4]);

    vendor_id = (buf[4]>>4)&0x0F;

    //Sunny 0x01 & huaweiModuleCode is 23060167(0n167 = 0xA7)
    if((vendor_id == IMX328_SUNNY_MODULE_VENDOR_ID) && (buf[3] == IMX328_SUNNY_MODULE_HUAWEI_ID))
    {
        imx328_otp_flag |= IMX328_OTP_ID_READ;
        return true;
    } else {
        CMR_LOGE("%s OTP data is worng for with wrong vender id!!!\n",__func__);
        return false;
    }
}

/****************************************************************************
* FunctionName: imx328_otp_read_awb;
* Description : Get awb parameters from eeprom.
* Input       : NA;
* Output      : imx328_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool imx328_otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t buf[8] = {0};

    imx328_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_AWB_REG,buf,8);

    imx328_otp.rgain = buf[2];
    imx328_otp.rgain <<= 8;
    imx328_otp.rgain += buf[3];
    imx328_otp.ggain = buf[4];
    imx328_otp.ggain <<= 8;
    imx328_otp.ggain += buf[5];
    imx328_otp.bgain = buf[6];
    imx328_otp.bgain <<= 8;
    imx328_otp.bgain += buf[7];
    CMR_LOGW("%s OTP data are rgain=0x%x, ggain=0x%x, bgain=0x%x\n",__func__, imx328_otp.rgain, imx328_otp.ggain, imx328_otp.bgain);
    if(0 == imx328_otp.rgain || 0 ==imx328_otp.ggain || 0 == imx328_otp.bgain){
        //if awb value read is error for zero, abnormal branch deal
        CMR_LOGE("%s OTP data is worng!!!\n",__func__);
        return false;
    }

    imx328_otp_flag |= IMX328_OTP_AWB_READ;

    return true;
}
/****************************************************************************
* FunctionName: imx328_otp_read_vcm;
* Description : Get AF motor parameters from EEPROM.;
* Input       : NA;
* Output      : imx328_otp;
* ReturnValue :true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool imx328_otp_read_vcm(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t  buf[4] = {0};
    uint16_t vcm_start = 0;
    uint16_t vcm_end =0;

    imx328_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_VCM_REG,buf,4);

    vcm_start = buf[0];
    vcm_start <<= 8;
    vcm_start += buf[1];
    vcm_end   = buf[2];
    vcm_end   <<= 8;
    vcm_end   += buf[3];

    if((vcm_end > vcm_start) && (vcm_start != 0) && ( vcm_end  != 0))
    {
        imx328_otp_flag |= IMX328_OTP_VCM_READ;
        imx328_otp.vcm_start = vcm_start;
        imx328_otp.vcm_end = vcm_end;
        CMR_LOGI("%s vcm_start=0x%x, vcm_end=0x%x \n",__func__, imx328_otp.vcm_start,imx328_otp.vcm_end);
    } else {
        //Abnormal branch deal
        imx328_otp.vcm_start = 0;
        imx328_otp.vcm_end = 0;
        CMR_LOGE("%s VCM OTP data is worng! vcm_start=0x%x, vcm_end=0x%x\n",__func__, imx328_otp.vcm_start,imx328_otp.vcm_end);
        return false;
    }

	if (imx328_otp.vcm_start <= IMX328_SUNNY_OTP_VCM_OFFSET_VALUE)
	{
		CMR_LOGE("%s, imx328_otp.vcm_start = 0x%x\n", __func__,imx328_otp.vcm_start);
		imx328_otp.vcm_start = 0;
	}
	else
	{
		imx328_otp.vcm_start -= IMX328_SUNNY_OTP_VCM_OFFSET_VALUE;
	}

	imx328_otp.vcm_end += IMX328_SUNNY_OTP_VCM_OFFSET_VALUE;

	if (imx328_otp.vcm_end >= IMX328_SUNNY_OTP_VCM_END_MAX)
	{
		imx328_otp.vcm_end = IMX328_SUNNY_OTP_VCM_END_MAX;
	}

	s_ctrl->afc_otp_info.starting_dac = imx328_otp.vcm_start;
	s_ctrl->afc_otp_info.infinity_dac = imx328_otp.vcm_start;
	s_ctrl->afc_otp_info.macro_dac = imx328_otp.vcm_end;

	CMR_LOGW(" s_ctrl->afc_otp_info.starting_dac = 0x%x\n", s_ctrl->afc_otp_info.starting_dac);
	CMR_LOGW(" s_ctrl->afc_otp_info.infinity_dac = 0x%x\n", s_ctrl->afc_otp_info.starting_dac);
	CMR_LOGW(" s_ctrl->afc_otp_info.macro_dac = 0x%x\n", s_ctrl->afc_otp_info.macro_dac);

    return true;
}
/****************************************************************************
* FunctionName: imx328_get_otp_from_sensor;
* Description : Get all the OTP info from EEPROM.;
* Input       : NA;
* Output      : imx328_otp;
* ReturnValue :true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool imx328_get_otp_from_sensor(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t   sum = 0;
    uint8_t   otpCheckSumVal = 0xFF;
    bool retVal = false;
    uint16_t tmp_mmi_otp_flag = 0x3F & ~IMX328_MMI_OTP_LSC_FLAG;//set all mmi otp flag mask(No LSC) ,default:fail

    CMR_LOGI("%s enters!\n",__func__);
    //Just check OTP once whether success or not
    if((imx328_otp_flag & IMX328_OTP_FAIL_FLAG) == IMX328_OTP_FAIL_FLAG) {
        //read fail
        CMR_LOGE("%s OTP data is worng, imx328_otp_flag=0x%x\n", __func__,imx328_otp_flag);
        return false;
    }else if((imx328_otp_flag & IMX328_OTP_CHECKSUM_READ) == IMX328_OTP_CHECKSUM_READ){
        //read success, no need read more
        CMR_LOGI("%s OTP has been read success already, imx328_otp_flag=0x%x\n", __func__,imx328_otp_flag);
        return true;
    }

    imx328_otp_read_checksum(s_ctrl, &otpCheckSumVal);
    if(otpCheckSumVal == 0xFF){
        CMR_LOGE("%s OTP data has not flashed!\n", __func__);
        goto OTP_FAIL;
    }
    tmp_mmi_otp_flag &= ~IMX328_MMI_OTP_CHECKSUM_FLAG;

    memset(&imx328_otp,0,sizeof(imx328_otp));//initial imx328_otp

    retVal = imx328_otp_read_id(s_ctrl);
    if(false == retVal){
        CMR_LOGE("%s imx328_otp_read_id() failed!\n",__func__);
        goto OTP_FAIL;
    }
    tmp_mmi_otp_flag &= ~IMX328_MMI_OTP_MODULE_INFO_FLAG;

    retVal = imx328_otp_read_awb(s_ctrl);
    if(false == retVal){
        CMR_LOGE("%s imx328_otp_read_awb() failed!\n",__func__);
        goto OTP_FAIL;
    }
    tmp_mmi_otp_flag &= ~IMX328_MMI_OTP_AWB_FLAG;

    retVal = imx328_otp_read_vcm(s_ctrl);
    if(false == retVal){
        CMR_LOGE("%s imx328_otp_read_vcm() failed!\n",__func__);
        goto OTP_FAIL;
    }
    tmp_mmi_otp_flag &= ~IMX328_MMI_OTP_VCM_FLAG;

    sum = (OTPSUMVAL - otpCheckSumVal) % 0xff;

    if(otpCheckSumVal == sum){
        imx328_otp_flag |= IMX328_OTP_CHECKSUM_READ;
        CMR_LOGI("%s success, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d, imx328_otp_flag=0x%x\n",
                __func__, OTPSUMVAL, otpCheckSumVal,sum,imx328_otp_flag);
		tmp_mmi_otp_flag &= ~IMX328_MMI_OTP_SUMVAL_FLAG;
		s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
		CMR_LOGW("%s imx328_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
        return true;
    } else {
        imx328_otp_flag |= IMX328_OTP_CHECKSUM_ERR;
        CMR_LOGE("%s fail, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d \n", __func__, OTPSUMVAL, otpCheckSumVal,sum);
    }

OTP_FAIL:
    imx328_otp_flag |= IMX328_OTP_FAIL_FLAG;
    CMR_LOGE("%s imx328_otp_flag=0x%x \n", __func__,imx328_otp_flag);
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	CMR_LOGE("%s imx328_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
    return false;
}

/****************************************************************************
* FunctionName: imx328_otp_get_flag;
* Description : get the imx328 flag;
* Input           : NA;
* Output         : NA;
* ReturnValue : imx328_otp_flag;
* Other           : NA;
***************************************************************************/
static uint8_t imx328_otp_get_flag(void)
{
    return imx328_otp_flag;
}
/****************************************************************************
* FunctionName: imx328_otp_set_awb;
* Description : Calc and set the otp awb param to sensor;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static void imx328_otp_set_awb(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint32_t rg_ratio, bg_ratio;
    uint32_t rg_ratio_typical, bg_ratio_typical;

    //digital gain write to sensor
    uint32_t r_gain, b_gain, g_gain;

    if(0 == imx328_otp.ggain|| 0 == imx328_otp.rgain|| 0 == imx328_otp.bgain) {
        CMR_LOGE("%s otp value error.", __func__);
        return;
    }

    rg_ratio = DIGITAL_GAIN_CALC_BASE * imx328_otp.rgain/ imx328_otp.ggain;
    bg_ratio = DIGITAL_GAIN_CALC_BASE * imx328_otp.bgain/ imx328_otp.ggain;

    rg_ratio_typical = DIGITAL_GAIN_CALC_BASE * awb_gain_typical[0].rgain / awb_gain_typical[0].ggain;
    bg_ratio_typical = DIGITAL_GAIN_CALC_BASE * awb_gain_typical[0].bgain / awb_gain_typical[0].ggain;

    r_gain = DIGITAL_GAIN_CALC_BASE;
    g_gain = DIGITAL_GAIN_CALC_BASE;
    b_gain = DIGITAL_GAIN_CALC_BASE;

    if (rg_ratio > rg_ratio_typical && bg_ratio > bg_ratio_typical) {
        g_gain = DIGITAL_GAIN_CALC_BASE;
        r_gain = DIGITAL_GAIN_CALC_BASE * rg_ratio / rg_ratio_typical;
        b_gain = DIGITAL_GAIN_CALC_BASE * bg_ratio / bg_ratio_typical;
    } else {
        /* select smaller divider as base 0x100 */
        if (rg_ratio * bg_ratio_typical < bg_ratio * rg_ratio_typical) {
            r_gain = DIGITAL_GAIN_CALC_BASE;
            g_gain = DIGITAL_GAIN_CALC_BASE * rg_ratio_typical / rg_ratio;
            b_gain = g_gain * bg_ratio / bg_ratio_typical;
        } else {
            b_gain = DIGITAL_GAIN_CALC_BASE;
            g_gain = DIGITAL_GAIN_CALC_BASE * bg_ratio_typical / bg_ratio;
            r_gain = g_gain * rg_ratio / rg_ratio_typical;
        }
    }

    /* change CALC_BASE to BASE for more accurate */
    r_gain = (r_gain + 0x8) / 0x10;
    g_gain = (g_gain + 0x8) / 0x10;
    b_gain = (b_gain + 0x8) / 0x10;
    /* MSB digital gain range is 1 to 15, check if the gain is valid */
    if(((r_gain&0xF000)|(g_gain&0xF000)|(b_gain&0xF000))!= 0 ){
        CMR_LOGE("%s awb gain is invalid.r_gan: %d, b_gain: %d, g_gain: %d\n",__func__,r_gain,b_gain,g_gain);
        return;
    }
    /* write awb gain to sensor*/
    if( g_gain >= 0x100 ){
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_GR_REG_H, (g_gain >> 8) & 0xff);
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_GR_REG_L, g_gain & 0xff);
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_GB_REG_H, (g_gain >> 8) & 0xff);
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_GB_REG_L, g_gain & 0xff);
    }
    if( r_gain >= 0x100 ){
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_R_REG_H, (r_gain >> 8) & 0xff);
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_R_REG_L, r_gain & 0xff);
    }
    if( b_gain >= 0x100 ){
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_B_REG_H, (b_gain >> 8) & 0xff);
        imx328_otp_i2c_write(s_ctrl,IMX328_DIG_GAIN_B_REG_L, b_gain & 0xff);
    }
    CMR_LOGI("%s set sensor digital r_gain: %d, b_gain: %d, g_gain: %d\n",__func__,r_gain,b_gain,g_gain);
}

/*
 **************************************************************************
 * FunctionName: imx328_set_otp_to_sensor;
 * Description : set the all otp params to sensor;
 * Input         : NULL;
 * Output       : NA;
 * ReturnValue:NA;
 * Other         : NA;
 **************************************************************************
*/
static void imx328_set_otp_to_sensor(struct msm_sensor_ctrl_t * s_ctrl)
{
    uint8_t otpflag = imx328_otp_get_flag();
    if (( otpflag&IMX328_OTP_FAIL_FLAG) != IMX328_OTP_FAIL_FLAG){
        imx328_otp_set_awb(s_ctrl);
    }
    return;
}

/*
 **************************************************************************
 * FunctionName: imx328_sunny_p13n10a_otp_func;
 * Description : Get OTP info from eeprom, and set the info to sensor.
 * Input         : NULL;
 * Output       : NA;
 * ReturnValue :-1 fail, 0 success;
 * Other         : NA;
 **************************************************************************
*/
int imx328_sunny_p13n10a_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index)
{
    int rc = 0;
    CMR_LOGI("%s enters!\n",__func__);

    /* read otp info */
    rc = imx328_get_otp_from_sensor(s_ctrl);

    if (false == rc)
    {
        return -1;
    }
    /*set otp to sensor*/
    imx328_set_otp_to_sensor(s_ctrl);

    CMR_LOGI("%s, the OTP read and set end.\n", __func__);

    return rc;
}
