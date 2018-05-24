

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_ar1335_sunny_f13m01f"

#include <linux/hw_camera_common.h>
#include <media/msm_cam_sensor.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

//R/G and B/G value of Golden Samples.
static  uint32_t rg_ratio_typical = 0x26D;   //the average of 4 Golden samples' RG ratio
static  uint32_t bg_ratio_typical = 0x267;   //the average of 4 Golden samples' BG ratio

#define AR1335_OTP_ID_READ             (1 << 0)
#define AR1335_OTP_VCM_READ          (1 << 1)
#define AR1335_OTP_LSC_READ           (1 << 2)
#define AR1335_OTP_AWB_READ          (1 << 3)
#define AR1335_OTP_FAIL_FLAG          (1 << 7)
#define AR1335_OTP_COMPLETED (AR1335_OTP_ID_READ|AR1335_OTP_VCM_READ|AR1335_OTP_LSC_READ|AR1335_OTP_AWB_READ)

#define AR1335_MODULE_VENDOR_ID         0x01  //SUNNY
#define AR1335_MODULE_HUAWEI_ID        0xC1  //23060193
#define AR1335_OTP_LSC_SIZE                 115     //length in word


#define AR1335_OTP_TYPE_VCM_BEGIN 0x31
#define AR1335_OTP_TYPE_VCM_END 0x32

#define AR1335_OTP_TYPE_LSC_BEGIN 0x34
#define AR1335_OTP_TYPE_LSC_END 0x36

#define AR1335_OTP_TYPE_AWB_BEGIN 0x37
#define AR1335_OTP_TYPE_AWB_END 0x39

#define AR1335_OTP_TYPE_MODULE_ID 0x50

#define AR1335_OTP_BUF_REG 0x3800

#define I2C_COMPARE_MATCH 0
#define I2C_COMPARE_MISMATCH 1
#define I2C_POLL_MAX_ITERATION 5

#define AR1335_OTP_LSC_BLOCK_NUM 7

//the value used for vcm effect, maybe modified by others
#define AR1335_OTP_VCM_OFFSET_VALUE            200

#define AR1335_OTP_LSC_ENABLE_REG 0x3780

struct _ar1335_lsc_reg_block_t
{
	uint16_t addr;/*block start address*/
	uint16_t size;/*block size in words*/
};
/*LSC register blocks definition*/
struct _ar1335_lsc_reg_block_t ar1335_lsc_block[AR1335_OTP_LSC_BLOCK_NUM] = {
	{0x3600, 20},
	{0x3640, 20},
	{0x3680, 20},
	{0x36c0, 20},
	{0x3700, 20},
	{0x3782,  2},
	{0x37c0,  4},
};

typedef struct ar1335_otp_struct_type
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
	uint16_t  lsc[AR1335_OTP_LSC_SIZE];
}ov1335_otp_struct;

ov1335_otp_struct ar1335_otp;

static uint8_t  ar1335_otp_flag  = 0;


/****************************************************************************
* FunctionName: ar1335_otp_i2c_read;
* Description : read otp data.
* Input       : NA;
* Output      : data;
* ReturnValue : <0 -fail;
* Other       : NA;
***************************************************************************/
static int32_t ar1335_otp_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&temp_data, MSM_CAMERA_I2C_WORD_DATA);

	if (rc < 0){
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	} else {
		*data = temp_data;
	}

	return rc;
}

/****************************************************************************
* FunctionName: ar1335_otp_i2c_write;
* Description : write data to sensor reg.
* Input       : NA;
* Output      : NA;
* ReturnValue : <0 write faill;
* Other       : NA;
***************************************************************************/
static int32_t ar1335_otp_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, u16 data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_WORD_DATA);

	if (rc < 0){
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;

}
/****************************************************************************
* FunctionName: ar1335_otp_i2c_poll;
* Description : polling to the addr's val,  successfully if the val equal the taget data.
* Input       : addr: poll reg;
		    data: target val;
		    bitmask:taget val mask;
		    max_iteration: poll times;
* Output      : NA;
* ReturnValue : I2C_COMPARE_MISMATCH: poll faill;
			I2C_COMPARE_MATCH: poll success;
* Other       : NA;
***************************************************************************/
static int32_t ar1335_otp_i2c_poll(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr,
	uint16_t data, uint16_t bitmask, uint16_t max_iteration)
{
	int32_t rc = I2C_COMPARE_MISMATCH;
	int32_t i = 0;
	uint16_t read_data = 0;

	for (i = 0; i < max_iteration; i++) {

		if( ar1335_otp_i2c_read(s_ctrl, addr, &read_data) < 0 ){
			CMR_LOGE("%s read reg:%d fail", __func__, addr);
			break;
		}
		/*if any of the bit masked by bitmask is non-zero, then break the loop*/
		if ((read_data&bitmask) != 0){
			CMR_LOGW("%s iteration:%d read_data:0x%x\n", __func__,i, read_data);
			break;
		}

		usleep_range(5000, 6000);
	}

	if( (read_data&bitmask) == data ){
		rc = I2C_COMPARE_MATCH;
	} else {
		rc = I2C_COMPARE_MISMATCH;
	}
	return rc;
}
/****************************************************************************
* FunctionName: ar1335_otp_read_prepare;
* Description : prepare to read otp, operates before read.
* Input       : NA;
* Output      : NA;
* ReturnValue : <0 poll faill;
* Other       : NA;
***************************************************************************/
static int32_t ar1335_otp_read_prepare(struct msm_sensor_ctrl_t *s_ctrl, uint16_t record_type)
{
	int32_t rc = -EFAULT;

	uint16_t data = 0;

	CMR_LOGD("%s ENTER\n", __func__);

	/*read out r0x301a*/
	rc = ar1335_otp_i2c_read(s_ctrl, 0x301a, &data);
	if( rc < 0 ){
		CMR_LOGE("%s fail to read R0x301a\n", __func__);
		goto PREPARE_OUT;
	}
	CMR_LOGD("%s read R0x301a: 0x%x\n", __func__, data);
	/*set R0x301a bit[0] to 0x01, reset the sensor*/
	rc = ar1335_otp_i2c_write(s_ctrl, 0x301a, data|0x01);
	if( rc < 0 ){
		CMR_LOGE("%s fail toreset sensor\n", __func__);
		goto PREPARE_OUT;
	}
	/*After the reset bit set to be 1, the sensor needs to delay about 15000+1032 MCLKs,
	    668us when mclk is 24M. We choose to delay 1ms here*/
	usleep_range(1000,1200);
	/*set the sensor to standby mode, disable streaming*/
	rc = ar1335_otp_i2c_write(s_ctrl, 0x301a,0x218);
	if( rc < 0 ){
		CMR_LOGE("%s fail to disable streaming\n", __func__);
		goto PREPARE_OUT;
	}

	/*write the read purpose record type*/
	rc = ar1335_otp_i2c_write(s_ctrl, 0x304c, (record_type<<8)&0xff00);
	if( rc < 0 ){
		CMR_LOGE("%s fail to write record type\n", __func__);
		goto PREPARE_OUT;
	}

	/*enable the read from the otp mem to the buffer registers*/
	rc = ar1335_otp_i2c_write(s_ctrl, 0x3054, 0x400);
	if( rc < 0 ){
		CMR_LOGE("%s fail to enable the read to buffer registers\n", __func__);
		goto PREPARE_OUT;
	}

	/*auto read start, otp mem to buffer registers*/
	rc = ar1335_otp_i2c_write(s_ctrl, 0x304a, 0x210);
	if( rc < 0 ){
		CMR_LOGE("%s fail to start auto read\n", __func__);
		goto PREPARE_OUT;
	}

	/*poll r0x304a bit[5]&bit[6] to ensure the auto read result*/
	rc = ar1335_otp_i2c_poll(s_ctrl, 0x304a, 0x60, 0x60, I2C_POLL_MAX_ITERATION);
	if(rc != I2C_COMPARE_MATCH){
		CMR_LOGE("%s block:0x%x read to buffer registers failed. rc:%d \n", __func__,record_type, rc);
		rc = -EFAULT;
	}

PREPARE_OUT:
	return rc;
}
/****************************************************************************
* FunctionName: ar1335_otp_read;
* Description : read the purpuse otp reg.
* Input       : reg_addr: start reg addr;
		    lens: regs number in words.
* Output      :  data: regs' vals;
		       sum: sum of the val which has been read out, not necessary.
* ReturnValue : <0 read faill;
* Other       : sum is not necessary, if sum is NULL, no effect on the funcion;
***************************************************************************/
static int32_t ar1335_otp_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t reg_addr, uint16_t *data, uint16_t lens, uint32_t *sum)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	uint32_t temp_addr = reg_addr;
	uint32_t temp_sum = 0;
	uint16_t val = 0;

	CMR_LOGD("%s ENTER. data lens:%d\n", __func__, lens);

	if(NULL == data){
		CMR_LOGE("%s data is NULL ptr\n", __func__);
		return rc;
	}
	for( i =0; i< lens; i++){
		/*both the i2c reg type and data type are word, so reg addr need +2 everytime*/
		rc = ar1335_otp_i2c_read(s_ctrl, temp_addr, &val);
		if( rc < 0){
			CMR_LOGE("%s reg[0x%x] read  failed \n", __func__, temp_addr );
			break;
		}
		data[i] = val;
		temp_sum += val;
		temp_addr += 2;
		CMR_LOGD("%s read reg[0x%x]:data[%d]: 0x%x, temp_sum:0x%x \n", __func__,  temp_addr,i, data[i],temp_sum);
	}

	if(sum != NULL){
		*sum = temp_sum;
	}
	CMR_LOGD("%s EXIT\n", __func__);
	return rc;
}
/*
 **************************************************************************
 * FunctionName: ar1335_otp_get_module_id;
 * Description : Read back module id from OTP memory, and check whether the info is correct;
 * Input         : s_ctrl: sensor struct ptr;
 * Output       :module id
 * ReturnValue : true-module id read success and correct, false-module id read failed or is wrong;
 * Other         : NA;
 **************************************************************************
*/
static bool	ar1335_otp_get_module_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -EFAULT;
	uint16_t buf[5] = {0};

	CMR_LOGD("%s ENTER\n", __func__);
	rc = ar1335_otp_read_prepare( s_ctrl, AR1335_OTP_TYPE_MODULE_ID );
	if(rc < 0){
		CMR_LOGE("%s read prepare failed \n", __func__);
		return false;
	}

	rc = ar1335_otp_read(s_ctrl, AR1335_OTP_BUF_REG, buf, 5, NULL);
	if( rc < 0 ){
		CMR_LOGE("%s read MOUDLE ID failed \n", __func__);
		return false;
	}

	ar1335_otp.product_year = buf[0];
	ar1335_otp.product_mouth = buf[1];
	ar1335_otp.product_date = buf[2];
	ar1335_otp.module_huawei_id = buf[3];
	ar1335_otp.module_vendor_id = (buf[4]>>4)& 0x0f;
	ar1335_otp.module_version = buf[4]&0x0f;

	CMR_LOGW("%s 20%d year, %d mouth %d day, huawei_id:%d, vendor_id:%d, version:%d\n", __func__,  ar1335_otp.product_year,
		ar1335_otp.product_mouth, ar1335_otp.product_date, ar1335_otp.module_huawei_id, ar1335_otp.module_vendor_id, ar1335_otp.module_version);
	if((ar1335_otp.module_huawei_id != AR1335_MODULE_HUAWEI_ID) ||
		ar1335_otp.module_vendor_id != AR1335_MODULE_VENDOR_ID){
		CMR_LOGE("%s MOUDLE ID mismatch\n", __func__);
		return false;
	}
	CMR_LOGD("%s EXIT\n", __func__);
	return true;
}
/*
 **************************************************************************
 * FunctionName: ar1335_otp_get_awb;
 * Description : Read back awb calibration data from OTP memory;
 * Input         : s_ctrl: sensor struct ptr;
 * Output       : awb info:awb calibration data. store in ar1335_otp;
 * ReturnValue : true-awb data read success, false-awb data is invalid;
 * Other         : NA;
 **************************************************************************
*/
static bool	ar1335_otp_get_awb(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -EFAULT;
	bool ret = false;
	int32_t i = 0;

	uint16_t buf[6] = {0};

	uint16_t temp_rg = 0;
	uint16_t temp_bg = 0;
	uint16_t temp_grgb = 0;
	CMR_LOGD("%s ENTER\n", __func__);
	/*Read the OTP block in reversed order with write order, from the end to begin, */
	for( i = AR1335_OTP_TYPE_AWB_END; i>=AR1335_OTP_TYPE_AWB_BEGIN; i--){
		temp_rg = 0;
		temp_bg = 0;
		temp_grgb = 0;
		/*prepare to read the i th block*/
		rc = ar1335_otp_read_prepare(s_ctrl, i);
		if(rc < 0 ){
			CMR_LOGD("%s awb otp read prepared fail, record type: 0x%x\n", __func__, i);
			continue;
		}

		rc = ar1335_otp_read(s_ctrl, AR1335_OTP_BUF_REG, buf, 6, NULL);
		if(rc < 0 ){
			CMR_LOGE("%s awb otp read fail, record type: 0x%x\n", __func__, i);
			break;
		}
		/*B/G R/G and then GR/GB*/
		temp_bg    = ((buf[0]<<8)&0xff00) | (buf[1]&0x00ff);//R0X3800.R0X3802
		temp_rg    = ((buf[2]<<8)&0xff00) | (buf[3]&0x00ff);//R0x3804.R0X3806
		temp_grgb = ((buf[4]<<8)&0xff00) | (buf[5]&0x00ff);//0X3808.0X380A
		if(temp_rg || temp_bg || temp_grgb){
			/*one of awb info is not zero, means this block is valid, then break the loop*/
			break;
		}
	}

	CMR_LOGW("%s record_type:0x%x, rg:0x%x, bg:0x%x, grgb:0x%x\n", __func__, i, temp_rg, temp_bg, temp_grgb);
	/*check if the awb vals are correct*/
	if ( temp_rg && temp_bg && temp_grgb ) {
		/*valid data is stored when all the rg bg and grgb are non-zero*/
		ar1335_otp.awb_rg    = temp_rg;
		ar1335_otp.awb_bg    = temp_bg;
		ar1335_otp.awb_grgb = temp_grgb;
		ret = true;
	} else {
		CMR_LOGE("%s awb otp read fail, last block is 0x%x\n", __func__, i);
		ret = false;
	}

	CMR_LOGD("%s EXIT\n", __func__);

	return ret;
}
/*
 **************************************************************************
 * FunctionName: ar1335_otp_get_vcm;
 * Description : Read back af(vcm) calibrate data from OTP memory;
 * Input         : s_ctrl: sensor struct ptr;
 * Output       : vcm_start:start code of vcm. store in ar1335_otp.vcm_start;
 			vcm_end: end code of vcm. store in ar1335_otp.vcm_end;
 * ReturnValue : true-vcm data read success, false-vcm data is invalid;
 * Other         : NA;
 **************************************************************************
*/
static bool	ar1335_otp_get_vcm(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -EFAULT;
	bool ret = false;
	int i = 0;
	int16_t buf[4] = {0};

	int16_t vcm_start = 0;
	int16_t vcm_end   = 0;
	CMR_LOGD("%s ENTER\n", __func__);
	/*Read the OTP block in reversed order with write order, from the end to begin, */
	for( i = AR1335_OTP_TYPE_VCM_END; i>=AR1335_OTP_TYPE_VCM_BEGIN; i--){
		vcm_start = 0;
		vcm_end   = 0;
		rc = ar1335_otp_read_prepare(s_ctrl, i);
		if(rc < 0 ){
			CMR_LOGD("%s vcm otp read prepared fail, record type: 0x%x\n", __func__, i);
			continue;
		}

		rc = ar1335_otp_read(s_ctrl, AR1335_OTP_BUF_REG, buf, 4, NULL);
		if(rc < 0 ){
			CMR_LOGE("%s vcm otp read fail, record type: 0x%x\n", __func__, i);
			break;
		}

		vcm_start = ((buf[0]<<8)&0xff00) | (buf[1]&0x00ff);
		vcm_end  = ((buf[2]<<8)&0xff00) | (buf[3]&0x00ff);

		if( vcm_start || vcm_end){
			/*if vcm info is not zero, then break the loop*/
			break;
		}
	}

	CMR_LOGW("%s record_type:0x%x, start:0x%x, end:0x%x\n", __func__, i, vcm_start, vcm_end);

	if ((vcm_end > vcm_start) && vcm_start != 0 && vcm_end  != 0){
		/*vcm_end must be larger than vcm start, and both of them are not zero, or the value is invalid*/
		if (vcm_start <= AR1335_OTP_VCM_OFFSET_VALUE){
			vcm_start = 0;
		}else{
			vcm_start -= AR1335_OTP_VCM_OFFSET_VALUE;
		}

		vcm_end += AR1335_OTP_VCM_OFFSET_VALUE;

		ar1335_otp.vcm_start = vcm_start;
		ar1335_otp.vcm_end = vcm_end;

		ret = true;
	} else {
		/* Set the result to be false when the values read back are invalid*/
		CMR_LOGE("%s fail, last block is 0x%x\n", __func__, i);
		ret = false;
	}
	CMR_LOGD("%s EXIT\n", __func__);

	return ret;
}

/*
 **************************************************************************
 * FunctionName: ar1335_otp_get_lsc;
 * Description : Read back LSC calibrate data from OTP memory, and check if the data is valid;
 * Input         : s_ctrl: sensor struct ptr;
 * Output       : lsc data: store in ar1335_otp.lsc;
 * ReturnValue : true-LSC data read success, false-LSC data is invalid, or no correct data canbe read;
 * Other         : NA;
 **************************************************************************
*/
static bool	ar1335_otp_get_lsc(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -EFAULT;
	bool ret = false;
	int i = 0;
	uint16_t *buf = NULL;
	uint16_t temp_buf[3] = {0};
	uint32_t sum = 0;
	uint16_t chksum_val = 0;

	CMR_LOGD("%s ENTER\n", __func__);

	buf = ar1335_otp.lsc;
	/*Read the OTP block in reversed order with write order, from the end to begin, */
	for( i = AR1335_OTP_TYPE_LSC_END; i>=AR1335_OTP_TYPE_LSC_BEGIN; i--){

		sum = 0;
		chksum_val = 0;
		memset(buf,  0, AR1335_OTP_LSC_SIZE*sizeof(uint16_t));

		rc = ar1335_otp_read_prepare(s_ctrl, i);
		if(rc < 0 ){
			CMR_LOGD("%s lsc otp read prepared fail, record type: 0x%x\n", __func__, i);
			continue;
		}
		/*read back the checksum*/
		rc = ar1335_otp_read(s_ctrl, 0x38e6, &chksum_val, 1, NULL);
		if(rc<0){
			CMR_LOGE("%s lsc otp checksum read fail, record type: 0x%x\n", __func__, i);
			goto LSC_OUT;
		}
		/*read back the first three reg, to check if this block is valid*/
		rc = ar1335_otp_read(s_ctrl, AR1335_OTP_BUF_REG, temp_buf, 3, NULL);
		if(rc<0){
			CMR_LOGE("%s lsc otp read fail, record type: 0x%x\n", __func__, i);
			goto LSC_OUT;
		}

		if( chksum_val || temp_buf[0]||temp_buf[1]||temp_buf[2]){
			rc = ar1335_otp_read(s_ctrl, AR1335_OTP_BUF_REG, buf, AR1335_OTP_LSC_SIZE, &sum);
			if(rc < 0 ){
				CMR_LOGE("%s awb otp read fail, record type: 0x%x\n", __func__, i);
				goto LSC_OUT;
			}

			if( chksum_val == (sum%255+1) ){
				ret = true;
			}

			break;
		} else {
			CMR_LOGW("%s the record type:0x%x is invalid, continue read the pre-block\n", __func__, i);
		}
	}

LSC_OUT:
	CMR_LOGW("%s record_type:0x%x, chksum:0x%x, sum:0x%x\n", __func__, i,chksum_val ,sum);
	return ret;
}
/*
 **************************************************************************
 * FunctionName: ar1335_get_otp_from_sensor;
 * Description : the api to use ar1335 sensor module OTP function;
 * Input         : NULL;
 * Output       : the ar1335_otp;
 * ReturnValue : true-OTP can be used, false-OTP is error, cannot be used;
 * Other         : NA;
 **************************************************************************
*/
static bool ar1335_get_otp_from_sensor(struct msm_sensor_ctrl_t *s_ctrl)
{
	bool ret = false;
	uint8_t otp_flag = 0;

	CMR_LOGW("%s enter\n", __func__);

	/*OTP has been read out, but failed*/
	if((ar1335_otp_flag &AR1335_OTP_FAIL_FLAG) != 0){
		CMR_LOGE("%s ar1335_otp_flag, AR1335_OTP_FAIL_FLAG\n", __func__);
		return false;
	}
	/*OTP has been read out, and all the info is correct, no need to read again*/
	if( AR1335_OTP_COMPLETED == ar1335_otp_flag ){
		CMR_LOGE("%s ar1335_otp_flag, AR1335_OTP_COMPLETED\n", __func__);
		return true;
	}

	ar1335_otp_flag = 0;
	/*read module id*/
	ret = ar1335_otp_get_module_id(s_ctrl);
	if( false == ret ){
		CMR_LOGE("%s ar1335_otp_get_module_id, failed\n", __func__);
		goto OTP_FAILED;
	}
	otp_flag |= AR1335_OTP_ID_READ;

	/*read awb*/
	ret = ar1335_otp_get_awb(s_ctrl);
	if( false == ret ){
		CMR_LOGE("%s ar1335_otp_get_awb, failed\n", __func__);
		goto OTP_FAILED;
	}
	otp_flag |= AR1335_OTP_AWB_READ;

	/*read af*/
	ret = ar1335_otp_get_vcm(s_ctrl);
	if( false == ret ){
		CMR_LOGE("%s ar1335_otp_get_vcm, failed\n", __func__);
		goto OTP_FAILED;
	}
	otp_flag |= AR1335_OTP_VCM_READ;

	/*read lsc*/
	ret = ar1335_otp_get_lsc(s_ctrl);
	if( false == ret ){
		CMR_LOGE("%s ar1335_otp_get_lsc, failed\n", __func__);
		goto OTP_FAILED;
	}
	otp_flag |= AR1335_OTP_LSC_READ;
	ret = true;

	ar1335_otp_flag = otp_flag;
	CMR_LOGW("%s exit\n", __func__);

	return ret;

OTP_FAILED:
	otp_flag |= AR1335_OTP_FAIL_FLAG;
	ar1335_otp_flag = otp_flag;
	ret = false;
	CMR_LOGE("%s failed\n", __func__);

	return ret;
}

/*
 **************************************************************************
 * FunctionName: ar1335_set_otp_awb;
 * Description : send awb otp info to the user;
 * Input         : NULL;
 * Output       : the awb otp;
 * ReturnValue :NA;
 * Other         : Make sure ar1335_otp_get_awb has been called and successfully before use this func ;
 **************************************************************************
*/
static void ar1335_set_otp_awb(struct msm_sensor_ctrl_t* s_ctrl)
{
	s_ctrl->awb_otp_info.RG = ar1335_otp.awb_rg;
	s_ctrl->awb_otp_info.BG = ar1335_otp.awb_bg;
	s_ctrl->awb_otp_info.typical_RG = rg_ratio_typical;
	s_ctrl->awb_otp_info.typical_BG= bg_ratio_typical;

	CMR_LOGW(" %s: RG = 0x%x,  BG = 0x%x, typical_RG = 0x%x, typical_BG=0x%x\n", __func__,
		s_ctrl->awb_otp_info.RG,
		s_ctrl->awb_otp_info.BG,
		s_ctrl->awb_otp_info.typical_RG,
		s_ctrl->awb_otp_info.typical_BG);

	return;
}
/*
 **************************************************************************
 * FunctionName: ar1335_set_otp_vcm;
 * Description : send vcm otp info to the user;
 * Input         : NULL;
 * Output       : the vcm otp;
 * ReturnValue :NA;
 * Other         : Make sure ar1335_otp_get_vcm has been called and successfully before use this func ;
 **************************************************************************
*/
static void ar1335_set_otp_vcm(struct msm_sensor_ctrl_t* s_ctrl)
{
	s_ctrl->afc_otp_info.starting_dac = ar1335_otp.vcm_start;
	s_ctrl->afc_otp_info.infinity_dac = ar1335_otp.vcm_start;
	s_ctrl->afc_otp_info.macro_dac = ar1335_otp.vcm_end;

	CMR_LOGW("%s: starting_dac = 0x%x,  infinity_dac = 0x%x, macro_dac = 0x%x\n", __func__,
		s_ctrl->afc_otp_info.starting_dac,
		s_ctrl->afc_otp_info.starting_dac,
		s_ctrl->afc_otp_info.macro_dac);

	return;
}
/*
 **************************************************************************
 * FunctionName: ar1335_set_otp_vcm;
 * Description : send lsc otp calibration parameters to the sensor;
 * Input         : NA;
 * Output       : NA;
 * ReturnValue :NA;
 * Other         : Make sure ar1335_otp_get_lsc has been called and successfully before use this func ;
 **************************************************************************
*/
static void ar1335_set_otp_lsc(struct msm_sensor_ctrl_t* s_ctrl)
{

	uint16_t *lsc_buf = NULL;
	uint32_t lsc_total_num = 0;
	int i = 0, j=0;
	uint16_t lsc_reg = 0;
	CMR_LOGW("%s ENTER\n",__func__);

	lsc_buf = ar1335_otp.lsc;
	/*write lsc parameters to sensor's corresponding block*/
	for( i = 0; i < ARRAY_SIZE(ar1335_lsc_block); i++){

		lsc_reg = ar1335_lsc_block[i].addr;
		lsc_total_num += ar1335_lsc_block[i].size;

		if( lsc_total_num >= AR1335_OTP_LSC_SIZE ){
			CMR_LOGE("%s set LSC OTP size is:%d,  larger than the buf!\n",__func__, lsc_total_num);
			break;
		}

		for( j = 0; j < ar1335_lsc_block[i].size; j++ ){

			CMR_LOGD("%s set LSC OTP writting: reg[0x%x]: 0x%x!\n",__func__, lsc_reg,  lsc_buf[j]);
			ar1335_otp_i2c_write(s_ctrl, lsc_reg, lsc_buf[j]);
			lsc_reg += 2;//next reg
		}

		lsc_buf +=  ar1335_lsc_block[i].size;

	}
	/*enable lsc calbiration function*/
	ar1335_otp_i2c_write(s_ctrl, AR1335_OTP_LSC_ENABLE_REG, 0x8000);
	CMR_LOGW("%s EXIT\n",__func__);

	return;
}
/*
 **************************************************************************
 * FunctionName: ar1335_set_otp_to_sensor;
 * Description : set otp info to the target user;
 * Input         : NA;
 * Output       : NA;
 * ReturnValue :NA;
 * Other         : Make sure otp has been read out completely and successfully before the func called,
                        or the func will return derectly.
 **************************************************************************
*/
static void ar1335_set_otp_to_sensor(struct msm_sensor_ctrl_t* s_ctrl)
{
	CMR_LOGW("%s ENTER!\n",__func__);

	//check if otp read successfully and completely.
	if(ar1335_otp_flag != AR1335_OTP_COMPLETED){
		CMR_LOGE("%s invalid otp info!\n",__func__);
		return;
	}
	//calc af and set to the sensor
	ar1335_set_otp_vcm(s_ctrl);

	//calc awb and set to the sensor
	ar1335_set_otp_awb(s_ctrl);

	//set the lsc info to sensor
	ar1335_set_otp_lsc(s_ctrl);

	CMR_LOGW("%s EXIT!\n",__func__);

	return;
}

/******************************************************************************
Function   :  ov13850_sunny_p13v01h_otp_func
Description:  read the otp info
******************************************************************************/
int ar1335_sunny_f13m01f_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index)
{
	 int rc = 0;
	CMR_LOGW("%s ENTER!\n",__func__);
	if (otp_function_lists[index].rg_ratio_typical){
		rg_ratio_typical = otp_function_lists[index].rg_ratio_typical;
	}

	if (otp_function_lists[index].bg_ratio_typical){
		bg_ratio_typical = otp_function_lists[index].bg_ratio_typical;
	}

	/* get otp info from ar1335 otp memory */
	rc = ar1335_get_otp_from_sensor(s_ctrl);
	if (false == rc){
		CMR_LOGE("%s get otp info failed!\n",__func__);
		return  -EFAULT;
	}
	/* set otp info to the user*/
	ar1335_set_otp_to_sensor(s_ctrl);

	CMR_LOGW("%s EXIT!\n",__func__);

	return rc;
}

