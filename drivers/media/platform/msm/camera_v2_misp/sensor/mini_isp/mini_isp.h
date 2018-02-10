#ifndef _MINI_ISP_H_
#define _MINI_ISP_H_

#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <media/minib_isp.h> 
#include "../msm_sensor.h"

#define FWVER_INFOSIZE_MAX		34
#define AEZONECFG_MAXGROUP		5
#define AFZONECFG_MAXGROUP		5
#define FACEAREACFG_MAXGROUP	6

// Command error code
#define T_ERROR_NO_ERROR							0
#define T_ERROR_FAILURE								0x50
#define T_ERROR_COMMAND_NOT_SUPPORTED				0x51
#define T_ERROR_DATA_INVALID						0x52
#define T_ERROR_DATA_OUT_OF_RANGE					0x53
#define T_ERROR_SYSTEM_ALREADY_IN_STATE				0x54
#define T_ERROR_SYSTEM_IN_INVALIDE_STATE			0x55

//constants for Sensors' control to switch on
#define T_REAR1_SENSOR_ON		0
#define T_REAR2_SENSOR_ON		1
#define T_REAR1_REAR2_SENSOR_ON	2
#define T_FRONT_SENSOR_ON		3
#define T_FRONT_REAR1_SENSOR_ON	4
#define T_FRONT_REAR2_SENSOR_ON	5

//constants for Sensor mode for Sensor mode control
#define T_SENSOR_BINMODE		0
#define T_SENSOR_FRMODE			1
#define T_SENSOR_FRBINMODE		2
#define T_SENSOR_HDMODE			3

//constants for Preview mode for Change mode control
#define T_PREVIEW_TEST			0
#define T_PREVIEW_STILL			1
#define T_PREVIEW_RSVD0			2
#define T_PREVIEW_RSVD1			3
#define T_PREVIEW_BYPASS		4
#define T_PREVIEW_PWRDWN		5

//constants for anti-flicker mode
#define T_FREQ_AUTO				0
#define T_FREQ_OFF				1
#define T_FREQ_50HZ				2
#define T_FREQ_60HZ				3

//constants for auto-exposure mode
#define T_EXP_AUTO				0
#define T_EXP_DISABLE			1

//constants for exposure compensation
#define T_COMPEXPIDX_MIN		0
#define T_COMPEXPIDX_MAX		12
#define T_COMPEXPIDX_DEFAULT	6
#define T_COMPEXPVALUE_MIN		(-2)
#define T_COMPEXPVALUE_MAX		(2)

//constants for white balance
#define T_WB_AUTO				0
#define T_WB_INCANDESCENT		1
#define T_WB_FLUORESCENT		2
#define T_WB_WARM_FLUORESCENT	3
#define T_WB_DAYLIGHT			4
#define T_WB_CLOUDY_DAYLIGHT	5
#define T_WB_TWILIGHT			6
#define T_WB_SHADE				7

//constants for focus mode
#define T_AF_FIXED				0
#define T_AF_AUTO				1
#define T_AF_INFINITY			2
#define T_AF_MACRO				3
#define T_AF_CONTINUEVIDEO		4
#define T_AF_CONTINUEPICTURE	5
#define T_AF_EDOF				6

//constants for ISO level
#define T_ISO_AUTO				0
#define T_ISO_100				1
#define T_ISO_200				2
#define T_ISO_400				3
#define T_ISO_800				4
#define T_ISO_1600				5
#define T_ISO_3200				6
#define T_ISO_6400				7
#define T_ISO_12800				8

//constants for Scene mode
#define T_SCENE_AUTO            0
#define T_SCENE_ACTION          1
#define T_SCENE_PORTRAIT        2
#define T_SCENE_LANDSCAPE       3
#define T_SCENE_NIGHT           4
#define T_SCENE_NIGHTPORTRAIT   5
#define T_SCENE_THEATER         6
#define T_SCENE_BEACH           7
#define T_SCENE_SNOW            8
#define T_SCENE_SUNSET          9
#define T_SCENE_STEADYPHOTO     10
#define T_SCENE_FIREWORKS       11
#define T_SCENE_SPORTS          12
#define T_SCENE_PARTY           13
#define T_SCENE_CANDLELIGHT     14

//Take mode
#define T_TAKEPIC_SINGLE		0
#define T_TAKEPIC_CONTINUE		1
#define T_TAKEPIC_STOP			2

//AE metering mode
#define T_AE_AUTO				0 
#define T_AE_AVERAGE 			1
#define T_AE_CENTRWEIGHT		2
#define T_AE_SPOT				3
//move misp_state into include/media/minib_isp.h
struct misp_askdata_setting{
	u16 opcode;	//opcode
	u8 *askparam;	//ask cmd
	u32 asklen;	//ask cmd lens
	u8 *recvparam;	//recv data
	u32 recvlen;	//recv data lens
	bool is_block_data;	//the recv data is block or normal
	misp_state wait_state;	//wait for which state after send ask cmd
};

u32 misp_construct_opcode(u32 opcode, u32 set_flag, u32 len);
int misp_exec_cmd(u32 opcode, u8 *param);
int misp_exec_inout_cmd(u16 cmd, u8 *in, u32 in_len, u8 *out, u32 out_len);
int misp_load_fw(struct msm_sensor_ctrl_t *s_ctrl);
/*mini-isp do the ois init*/
int misp_get_ois_initstatus(void);
int misp_set_ois_initstatus(int status);
int misp_flush_log(void);
int misp_flush_reg(void);
int misp_reset_vcm(void);
int misp_set_reset_gpio(int value);
int misp_set_cs_gpio(int value);
int misp_write_cmd(u16 opcode, u8 *param, u32 len);
int misp_askdata_cmd(struct misp_askdata_setting);
int misp_execmd_array(struct msm_camera_spi_reg_settings );
int misp_get_otp_data(struct msm_sensor_otp_info *otp_info, uint8_t cam_pos);
//int misp_get_otp(struct msm_sensor_otp_info *otp_info, uint8_t cam_pos);
/*add camera ois driver*/
int misp_exec_write_block(u16 cmd, u8 *in, u32 in_len, u8 *out, u32 out_len);
int misp_exec_bidir_cmd(u16 cmd, u8 *in, u32 in_len, bool out_to_block, u8 *out, u32 out_len);
#endif
