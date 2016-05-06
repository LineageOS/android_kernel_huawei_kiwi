/*******************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  File name         : sensor_otp_imx214.c
  Author            : w00345499
  Version           : Initial Draft
  Date              : 2015/07/16
  Description       : this file contains the functions to detect and read
                      imx214 camera's OTP memory info.
  Function List     :
            imx214_otp_func
  History           :
  1.Date          : 2015/07/16
    Author        : w00345499
    Modification  : Created File
********************************************************************/
#define HW_CMR_LOG_TAG "sensor_otp_imx214"

#include <linux/hw_camera_common.h>
#include <media/msm_cam_sensor.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

#define IMX214_SLAVE_ADDR 0x34 //sensor i2c addr

#define OTP_SLAVE_ADDR1       0xA0
#define OTP_SLAVE_ADDR2       0xA4
#define OTP_SLAVE_ADDR3       0xA6
#define OTP_SLAVE_ADDR4       0xA8
#define OTP_SLAVE_ADDR5       0xAA
#define OTP_SLAVE_ADDR6       0xAC

#define OTP_ID_REG            0x00
#define OTP_AWB_REG           0x05
#define OTP_VCM_REG           0x00
#define OTP_CHECKSUM_REG      0x04

/*OTP READ STATUS*/
#define IMX214_OTP_ID_READ            (1 << 0)
#define IMX214_OTP_AF_READ            (1 << 1)
#define IMX214_OTP_AWB_READ           (1 << 2)
#define IMX214_OTP_LSC_READ           (1 << 3)
#define IMX214_OTP_CHECKSUM_READ      (1 << 4)
#define IMX214_OTP_CHKSUM_SUCC        (1 << 6)
#define IMX214_OTP_FAIL_FLAG          (1 << 7)

#define IMX214_OTP_SUCCESS (IMX214_OTP_ID_READ|IMX214_OTP_AF_READ| \
                            IMX214_OTP_AWB_READ|IMX214_OTP_LSC_READ|IMX214_OTP_CHECKSUM_READ|IMX214_OTP_CHKSUM_SUCC)

#define IMX214_SUNNY_MODULE_VENDOR_ID    0x01
#define IMX214_FOXCONN_MODULE_VENDOR_ID  0x02
#define IMX214_OFILM_MODULE_VENDOR_ID 0x06
#define IMX214_MODULE_HUAWEI_ID          0x9c//23060156
#define IMX214_SUNNY_MODULE_VCM_OFFSET    200
#define IMX214_FOXCONN_MODULE_VCM_OFFSET  170
#define IMX214_OFILM_MODULE_VCM_OFFSET    140
#define IMX214_MODULE_AF_END_MAX 1023

#define DIGITAL_GAIN_CALC_BASE 0x1000
#define DIGITAL_GAIN_BASE      0x100

#define IMX214_DIG_GAIN_GR_REG_H     (0x020E)
#define IMX214_DIG_GAIN_GR_REG_L     (0x020F)
#define IMX214_DIG_GAIN_GB_REG_H     (0x0214)
#define IMX214_DIG_GAIN_GB_REG_L     (0x0215)
#define IMX214_DIG_GAIN_R_REG_H      (0x0210)
#define IMX214_DIG_GAIN_R_REG_L      (0x0211)
#define IMX214_DIG_GAIN_B_REG_H      (0x0212)
#define IMX214_DIG_GAIN_B_REG_L      (0x0213)

#define IMX214_MMI_OTP_VCM_FLAG          (1 << 0)
#define IMX214_MMI_OTP_AWB_FLAG          (1 << 1)
#define IMX214_MMI_OTP_MODULE_INFO_FLAG  (1 << 2)
#define IMX214_MMI_OTP_LSC_FLAG          (1 << 3)
#define IMX214_MMI_OTP_CHECKSUM_FLAG     (1 << 4)
#define IMX214_MMI_OTP_SUMVAL_FLAG       (1 << 5)

typedef struct __lsc_block_t
{
    uint16_t addr;
    uint16_t start_reg;
    uint16_t size;
} lsc_block_type;

lsc_block_type lsc_block_array[] =
{
    {OTP_SLAVE_ADDR1, 0x0D, 243},
    {OTP_SLAVE_ADDR2, 0x00, 256},
    {OTP_SLAVE_ADDR3, 0x00, 256},
    {OTP_SLAVE_ADDR4, 0x00, 256},
    {OTP_SLAVE_ADDR5, 0x00, 190},
};
typedef enum
{
    SUNNY   = 0x01,
    FOXCONN = 0x02,
    OFILM   = 0x06,
    INVALID,
} module_type;
typedef struct
{
    uint16_t rgain;
    uint16_t ggain;
    uint16_t bgain;
} awb_gain;

static awb_gain awb_gain_typical[] =
{
    {0, 0, 0}, //dummy
    {0xccc, 0x6dc, 0xac7}, //sunny
    {0xe15, 0x781, 0xb0f},//{0xe16, 0x789, 0xb3e}, //foxconn #360
    {0, 0, 0},//dummy
    {0, 0, 0},//dummy
    {0, 0, 0},//dummy
    {0xdf7, 0x76b, 0xc25},//ofilm

};

static uint16_t af_offset_defined[] =
{
    00,   //dummy
    IMX214_SUNNY_MODULE_VCM_OFFSET, //sunny module vcm offset
    IMX214_FOXCONN_MODULE_VCM_OFFSET, //foxconn module vcm offset
    00,
    00,
    00,
    IMX214_OFILM_MODULE_VCM_OFFSET,//ofilm module vcm offset
};

typedef struct imx214_otp_struct_type
{
    uint16_t rgain;
    uint16_t bgain;
    uint16_t ggain;
    uint16_t af_start;
    uint16_t af_end;
    module_type module_id;
} imx214_otp_struct;

static imx214_otp_struct imx214_otp;

static uint32_t OTPSUMVAL         = 0;

static uint8_t  imx214_otp_flag   = 0;

static int32_t imx214_otp_i2c_read(struct msm_sensor_ctrl_t* s_ctrl, uint32_t addr, uint16_t* data)
{
    int32_t rc = -EFAULT;
    uint16_t temp_data = 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
             s_ctrl->sensor_i2c_client,
             addr,
             &temp_data, MSM_CAMERA_I2C_BYTE_DATA);

    if (rc < 0){
        CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
    }

    *data = temp_data;

    return rc;
}
/****************************************************************************
* FunctionName: imx214_otp_i2c_write;
* Description : write data to sensor reg.
* Input       : NA;
* Output      : NA;
* ReturnValue : <0 write faill;
* Other       : NA;
***************************************************************************/
static int32_t imx214_otp_i2c_write(struct msm_sensor_ctrl_t* s_ctrl, uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
             s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

    if (rc < 0){
        CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
    }

    return rc;
}
/****************************************************************************
* FunctionName: imx214_otp_set_i2c_address;
* Description : set i2c address.
* Input       : NA;
* Output      : NA;
* ReturnValue :
                -EFAULT Set FAIL;
* Other       : NA;
***************************************************************************/
static int imx214_otp_set_i2c_address(struct msm_sensor_ctrl_t* s_ctrl, uint16_t i2c_addr, enum msm_camera_i2c_reg_addr_type addr_type)
{
    int rc = 0;

    CMR_LOGW("%s enter! i2c_addr = 0x%x.\n", __func__, i2c_addr);

    s_ctrl->sensor_i2c_client->addr_type = addr_type;

    if (s_ctrl->sensor_i2c_client->cci_client){
        s_ctrl->sensor_i2c_client->cci_client->sid = (i2c_addr >> 1);
    } else if (s_ctrl->sensor_i2c_client->client) {
        s_ctrl->sensor_i2c_client->client->addr = (i2c_addr >> 1);
    } else {
        CMR_LOGE("%s: error: no i2c/cci client found.", __func__);
        rc = -EFAULT;
    }

    return rc;
}

/****************************************************************************
* FunctionName: imx214_read_otp;
* Description : used to read OTP information from eeprom,
                and output the info in buf.
* Input       :
* Output      : buf:read val;
* ReturnValue :
               true, read successfully.
               false, read failed.
* Other       : NA;
***************************************************************************/
static bool imx214_read_otp(struct msm_sensor_ctrl_t* s_ctrl, uint16_t i2c_addr, uint16_t reg, uint8_t* buf, uint16_t count)
{
    uint16_t i = 0;
    int ret = 0;
    uint16_t val = 0;
    bool rc = false;
    //set i2c address to read otp info
    ret = imx214_otp_set_i2c_address(s_ctrl, i2c_addr, MSM_CAMERA_I2C_BYTE_ADDR);

    if (ret < 0){
        CMR_LOGE("%s fail to set i2c address\n", __func__);
        rc = false;
        goto OUT;
    }

    for (i = 0; i < count; i++){
        ret = imx214_otp_i2c_read(s_ctrl, (reg + i) , &val);

        if (ret != 0){
            CMR_LOGE("%s fail to read otp with error code %d, i2c_addr=0x%x reg_addr=0x%x\n", __func__, ret, i2c_addr, reg + i);
            rc = false;
            goto OUT;
        }

        buf[i] = (val & 0xff);
        OTPSUMVAL += buf[i];
        CMR_LOGD("%s: %d, i2c_addr=0x%x reg_addr=0x%x, reg_val:0x%x, otpsum:0x%x\n",
                 __func__, ret, i2c_addr, reg + i, val, OTPSUMVAL);
    }

    rc = true;

OUT:
    //resume camsensor i2c address no matter rc == true or false
    imx214_otp_set_i2c_address(s_ctrl, IMX214_SLAVE_ADDR, MSM_CAMERA_I2C_WORD_ADDR);
    return rc;
}

/****************************************************************************
* FunctionName: imx214_otp_get_chksum;
* Description : used to read OTP checksum value from eeprom,
* Input       :
* Output      : checksum;
* ReturnValue :
               true, read successfully.
               false, read failed.
* Other       : NA;
***************************************************************************/
static bool imx214_otp_get_chksum(struct msm_sensor_ctrl_t* s_ctrl, uint8_t* checksum)
{
    return imx214_read_otp(s_ctrl, OTP_SLAVE_ADDR6, OTP_CHECKSUM_REG, checksum, 1);
}
/****************************************************************************
* FunctionName: imx214_otp_get_module_id;
* Description : used to read moudle id info value from eeprom
* Input       :
* Output      imx214_otp:
* ReturnValue :
                true, read successfully.
                false, read failed.
* Other       : NA;
***************************************************************************/
static bool imx214_otp_get_module_id(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint8_t buf[5] = {0};
    uint8_t vendor_id = 0;
    bool rc = false;

    rc = imx214_read_otp(s_ctrl, OTP_SLAVE_ADDR1, OTP_ID_REG, buf, 5);
    if ( false == rc ){
        CMR_LOGE("%s, read module id fail\n", __func__);
        return false;
    }
    CMR_LOGW("%s module info year 20%02d month %d day %d. huawei_id 0x%x,  vendor id&version 0x%x\n", __func__, buf[0], buf[1], buf[2], buf[3], buf[4]);

    if (buf[3] != IMX214_MODULE_HUAWEI_ID){
        CMR_LOGE("%s, huawei_id is err!\n", __func__);
        return false;
    }
    vendor_id = (buf[4] >> 4) & 0x0F;
    if ( IMX214_SUNNY_MODULE_VENDOR_ID == vendor_id ){
        CMR_LOGW("%s sunny_module\n", __func__);
        imx214_otp.module_id = SUNNY;
    } else if (IMX214_FOXCONN_MODULE_VENDOR_ID == vendor_id){
        CMR_LOGW("%s foxconn_module\n", __func__);
        imx214_otp.module_id = FOXCONN;
    } else if (IMX214_OFILM_MODULE_VENDOR_ID == vendor_id){
        CMR_LOGW("%s ofilm_module\n", __func__);
        imx214_otp.module_id = OFILM;
    } else {
        CMR_LOGE("%s module_id is wrong\n", __func__);
        return false;
    }

    return true;
}

/****************************************************************************
* FunctionName: imx214_otp_get_awb;
* Description : Get awb parameters from eeprom.
* Input       : NA;
* Output      : imx214_otp;
* ReturnValue : true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool imx214_otp_get_awb(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint8_t buf[8] = {0};
    bool rc = false;

    rc = imx214_read_otp(s_ctrl, OTP_SLAVE_ADDR1, OTP_AWB_REG, buf, 8);
    if ( false == rc ){
        CMR_LOGE("%s, read awb fail\n", __func__);
        return false;
    }

    imx214_otp.rgain = buf[2];
    imx214_otp.rgain <<= 8;
    imx214_otp.rgain += buf[3];

    imx214_otp.ggain = buf[4];
    imx214_otp.ggain <<= 8;
    imx214_otp.ggain += buf[5];

    imx214_otp.bgain = buf[6];
    imx214_otp.bgain <<= 8;
    imx214_otp.bgain += buf[7];

    CMR_LOGW("%s OTP data are rgain=0x%x, ggain=0x%x, bgain=0x%x\n", __func__, imx214_otp.rgain, imx214_otp.ggain, imx214_otp.bgain);

    if (0 == imx214_otp.rgain || 0 == imx214_otp.ggain || 0 == imx214_otp.bgain){
        //if awb value read is error for zero, abnormal branch deal
        CMR_LOGE("%s OTP awb is wrong!!!\n", __func__);
        return false;
    }

    return true;
}
/****************************************************************************
* FunctionName: imx214_otp_get_lsc;
* Description : Get lsc parameters from eeprom.
* Input       : NA;
* Output      : NA;
* ReturnValue : true-success,false-fail;
* Other       : this sensor no use lsc calibration data, only read out for
                     checksum, because the lsc format doesn't apply to qc
                     platform,sensor only have G chanel LSC;
***************************************************************************/
static bool imx214_otp_get_lsc(struct msm_sensor_ctrl_t* s_ctrl)
{
    bool rc  = false;
    int i = 0;
    uint16_t i2c_addr = 0;
    uint16_t start_reg = 0;
    uint16_t block_size = 0;
    uint8_t  buf[256] = {0};

    for ( i = 0; i < ARRAY_SIZE(lsc_block_array); i++ ){

        i2c_addr = lsc_block_array[i].addr;
        start_reg = lsc_block_array[i].start_reg;
        block_size = lsc_block_array[i].size;
        memset(buf, 0, 256);

        rc = imx214_read_otp(s_ctrl, i2c_addr, start_reg, buf, block_size);
        if (false == rc){
            return false;
        }
    }

    return true;
}

/****************************************************************************
* FunctionName: imx214_otp_get_af;
* Description : Get AF motor parameters from EEPROM.;
* Input       : NA;
* Output      : imx214_otp;
* ReturnValue :true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool imx214_otp_get_af(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint8_t  buf[4] = {0};
    uint16_t af_start = 0;
    uint16_t af_end = 0;
    bool rc = false;

    rc = imx214_read_otp(s_ctrl, OTP_SLAVE_ADDR6, OTP_VCM_REG, buf, 4);
    if ( false == rc ){
        CMR_LOGE("%s, read af fail\n", __func__);
        return false;
    }

    af_start = buf[0];
    af_start <<= 8;
    af_start += buf[1];
    af_end   = buf[2];
    af_end   <<= 8;
    af_end   += buf[3];

    if ((af_end > af_start) && (af_start != 0) && ( af_end  != 0)){
        imx214_otp.af_start = af_start;
        imx214_otp.af_end = af_end;
        CMR_LOGW("%s vcm_start=0x%x, vcm_end=0x%x \n", __func__, imx214_otp.af_start, imx214_otp.af_end );
    }else{
        //Abnormal branch deal
        imx214_otp.af_start = 0;
        imx214_otp.af_end = 0;
        CMR_LOGE("%s VCM OTP data is worng! vcm_start=0x%x, vcm_end=0x%x\n", __func__,  imx214_otp.af_start, imx214_otp.af_end);
        return false;
    }

    return true;
}
/****************************************************************************
* FunctionName: imx214_get_otp_from_sensor;
* Description : Get all the OTP info from EEPROM.;
* Input       : NA;
* Output      : imx214_otp;
* ReturnValue :true-success,false-fail;
* Other       : NA;
***************************************************************************/
static bool imx214_get_otp_from_sensor(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint8_t checksum = 0;
    uint32_t sum = 0;
    bool rc = false;
    uint16_t tmp_mmi_otp_flag = 0x3F;//set all mmi otp flag mask ,default:fail

    CMR_LOGD("%s, enter\n", __func__);

    if (IMX214_OTP_FAIL_FLAG == (imx214_otp_flag & IMX214_OTP_FAIL_FLAG)){
        CMR_LOGE("%s, IMX214_OTP_FAIL_FLAG\n", __func__);
        return false;
    } else if (IMX214_OTP_SUCCESS == imx214_otp_flag){
        CMR_LOGW("%s, IMX214_OTP_COMPLETE\n", __func__);
        return true;
    }

    imx214_otp_flag = 0;
    OTPSUMVAL = 0;
    /*read checksum*/
    rc = imx214_otp_get_chksum(s_ctrl, &checksum);
    if ( false == rc ){
        goto GET_OTP_FAIL;
    }
    imx214_otp_flag |= IMX214_OTP_CHECKSUM_READ;
    tmp_mmi_otp_flag &= ~IMX214_MMI_OTP_CHECKSUM_FLAG;

    /*read module id*/
    rc = imx214_otp_get_module_id(s_ctrl);
    if ( false == rc ){
        goto GET_OTP_FAIL;
    }
    imx214_otp_flag |= IMX214_OTP_ID_READ;
    tmp_mmi_otp_flag &= ~IMX214_MMI_OTP_MODULE_INFO_FLAG;

    /*read awb info*/
    rc = imx214_otp_get_awb(s_ctrl);
    if ( false == rc ){
        goto GET_OTP_FAIL;
    }
    imx214_otp_flag |= IMX214_OTP_AWB_READ;
    tmp_mmi_otp_flag &= ~IMX214_MMI_OTP_AWB_FLAG;

    /*read lsc info*/
    rc = imx214_otp_get_lsc(s_ctrl);
    if ( false == rc ){
        goto GET_OTP_FAIL;
    }
    imx214_otp_flag |= IMX214_OTP_LSC_READ;
    tmp_mmi_otp_flag &= ~IMX214_MMI_OTP_LSC_FLAG;

    /*read af info*/
    rc = imx214_otp_get_af(s_ctrl);
    if ( false == rc ){
        goto GET_OTP_FAIL;
    }
    imx214_otp_flag |= IMX214_OTP_AF_READ;
    tmp_mmi_otp_flag &= ~IMX214_MMI_OTP_VCM_FLAG;

    sum = OTPSUMVAL - checksum;
    if (checksum == (sum % 255 + 1) ){
        imx214_otp_flag |= IMX214_OTP_CHKSUM_SUCC;
        tmp_mmi_otp_flag &= ~IMX214_MMI_OTP_SUMVAL_FLAG;
        CMR_LOGW("%s success, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d, imx214_otp_flag=0x%x\n", __func__, OTPSUMVAL,
                 checksum, sum, imx214_otp_flag);
        s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
        CMR_LOGI("%s imx214_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
        return true;
    }else{
        CMR_LOGE("%s fail, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d \n", __func__, OTPSUMVAL, checksum, sum);
    }

GET_OTP_FAIL:
    CMR_LOGE("%s fail \n", __func__);
    imx214_otp_flag |= IMX214_OTP_FAIL_FLAG;
    s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
    CMR_LOGE("%s imx214_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
    return false;
}

/****************************************************************************
* FunctionName: imx214_otp_set_awb;
* Description : Calc and set the otp awb param to sensor;
* Input           : NA;
* Output         : NA;
* ReturnValue : NA;
* Other           : NA;
***************************************************************************/
static void imx214_otp_set_awb(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint32_t rg_ratio, bg_ratio;
    uint32_t rg_ratio_typical, bg_ratio_typical;

    //digital gain write to sensor
    uint32_t r_gain, b_gain, g_gain;
    module_type module;

    if (0 == imx214_otp.ggain || 0 == imx214_otp.rgain || 0 == imx214_otp.bgain){
        CMR_LOGE("%s otp value error.", __func__);
        return;
    }

    module = imx214_otp.module_id;

    rg_ratio = DIGITAL_GAIN_CALC_BASE * imx214_otp.rgain / imx214_otp.ggain;
    bg_ratio = DIGITAL_GAIN_CALC_BASE * imx214_otp.bgain / imx214_otp.ggain;

    rg_ratio_typical = DIGITAL_GAIN_CALC_BASE * awb_gain_typical[module].rgain / awb_gain_typical[module].ggain;
    bg_ratio_typical = DIGITAL_GAIN_CALC_BASE * awb_gain_typical[module].bgain / awb_gain_typical[module].ggain;

    r_gain = DIGITAL_GAIN_CALC_BASE;
    g_gain = DIGITAL_GAIN_CALC_BASE;
    b_gain = DIGITAL_GAIN_CALC_BASE;

    if (rg_ratio > rg_ratio_typical && bg_ratio > bg_ratio_typical){
        g_gain = DIGITAL_GAIN_CALC_BASE;
        r_gain = DIGITAL_GAIN_CALC_BASE * rg_ratio / rg_ratio_typical;
        b_gain = DIGITAL_GAIN_CALC_BASE * bg_ratio / bg_ratio_typical;
    } else {
        /* select smaller divider as base 0x100 */
        if (rg_ratio * bg_ratio_typical < bg_ratio * rg_ratio_typical){
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
    if (((r_gain & 0xF000) | (g_gain & 0xF000) | (b_gain & 0xF000)) != 0 ){
        CMR_LOGE("%s awb gain is invalid.r_gan: 0x%x, b_gain: 0x%x, g_gain: 0x%x\n", __func__, r_gain, b_gain, g_gain);
        return;
    }

    /* write awb gain to sensor*/
    if ( g_gain >= 0x100 ){
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_GR_REG_H, (g_gain >> 8) & 0xff);
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_GR_REG_L, g_gain & 0xff);
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_GB_REG_H, (g_gain >> 8) & 0xff);
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_GB_REG_L, g_gain & 0xff);
    }

    if ( r_gain >= 0x100 ){
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_R_REG_H, (r_gain >> 8) & 0xff);
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_R_REG_L, r_gain & 0xff);
    }

    if ( b_gain >= 0x100 ){
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_B_REG_H, (b_gain >> 8) & 0xff);
        imx214_otp_i2c_write(s_ctrl, IMX214_DIG_GAIN_B_REG_L, b_gain & 0xff);
    }

    CMR_LOGW("%s set sensor digital complete. r_gain: 0x%x, b_gain: 0x%x, g_gain: 0x%x\n", __func__, r_gain, b_gain, g_gain);
}

static void imx214_otp_set_af(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint16_t af_start =  imx214_otp.af_start;
    uint16_t af_end = imx214_otp.af_end;
    uint16_t af_offset = af_offset_defined[imx214_otp.module_id];

    if (af_start <= af_offset){
        af_start = 0;
    } else {
        af_start -= af_offset;
    }

    af_end += af_offset;

    if (af_end > IMX214_MODULE_AF_END_MAX){
        CMR_LOGW("%s: af_end:%d is larger than max\n", __func__, af_end);
        af_end = IMX214_MODULE_AF_END_MAX;
    }

    s_ctrl->afc_otp_info.starting_dac = af_start;
    s_ctrl->afc_otp_info.infinity_dac = af_start;
    s_ctrl->afc_otp_info.macro_dac = af_end;

    CMR_LOGW("%s: starting_dac = 0x%x,  infinity_dac = 0x%x, macro_dac = 0x%x\n", __func__,
             s_ctrl->afc_otp_info.starting_dac,
             s_ctrl->afc_otp_info.starting_dac,
             s_ctrl->afc_otp_info.macro_dac);

    return;
}

/*
**************************************************************************
* FunctionName: imx214_set_otp_to_sensor;
* Description : set the all otp params to sensor;
* Input         : NULL;
* Output       : NA;
* ReturnValue:NA;
* Other         : NA;
**************************************************************************
*/
static void imx214_set_otp_to_sensor(struct msm_sensor_ctrl_t* s_ctrl)
{

    CMR_LOGD("%s, enter\n", __func__);

    if (IMX214_OTP_FAIL_FLAG == (imx214_otp_flag & IMX214_OTP_FAIL_FLAG)){
        CMR_LOGE("%s invalid otp info!\n", __func__);
        return;
    }

    /*set awb*/
    imx214_otp_set_awb(s_ctrl);

    /*set af*/
    imx214_otp_set_af(s_ctrl);

    return;
}

/*
 **************************************************************************
 * FunctionName: imx214_otp_func;
 * Description : Get OTP info from eeprom, and set the info to sensor.
 * Input         : NULL;
 * Output       : NA;
 * ReturnValue :-1 fail, 0 success;
 * Other         : NA;
 **************************************************************************
*/
int imx214_otp_func(struct msm_sensor_ctrl_t* s_ctrl, int index)
{
    int rc = 0;
    CMR_LOGD("%s enters!\n", __func__);

    /* read otp info */
    rc = imx214_get_otp_from_sensor(s_ctrl);

    if (false == rc)
    {
        CMR_LOGE("%s get otp failed!\n", __func__);
        return -1;
    }

    /*set otp to sensor*/
    imx214_set_otp_to_sensor(s_ctrl);

    CMR_LOGD("%s, the OTP read and set end.\n", __func__);

    return rc;
}

