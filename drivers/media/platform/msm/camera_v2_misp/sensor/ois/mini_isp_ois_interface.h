/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#ifndef _MINIISP_OIS_INTERFACE_HEAD
#define _MINIISP_OIS_INTERFACE_HEAD

#define SUNNY_MODULE_ID   0x1
#define LITEON_MODULE_ID  0x3
#define LGIT_MODULE_ID    0x7

typedef enum
{
    OIS_PREVIEW,
	OIS_CAPTURE,
}OIS_HALL_LIMIT_TYPE;

#define		X_CURRENT_LMT_PREVIEW	0x3F666666//0.9
#define		Y_CURRENT_LMT_PREVIEW	0x3F666666//0.9
#define		X_CURRENT_LMT_CAPTURE	0x3F666666// 0.9
#define		Y_CURRENT_LMT_CAPTURE	0x3F666666// 0.9


//delete ois otp
int32_t mini_isp_ois_init(int32_t module_id);
int32_t mini_isp_ois_turn_on(int32_t module_id, uint32_t onoff, int32_t turn_on_type);
int mini_isp_ois_start_running_test(int32_t module_id);
/*delete mini_isp_ois_start_mmi_test function, no use*/
int32_t mini_isp_ois_start_mag_test(int32_t module_id, void *userdata);
void mini_isp_init_exit_flag(int flag);
int32_t mini_isp_ois_setGyroGain(int32_t module_id, int32_t new_xgain, int32_t new_ygain);
int32_t mini_isp_ois_getGyroGain(int32_t module_id, int32_t* xgain, int32_t* ygain);
    
int mini_isp_ois_RamWrite32A(uint32_t addr, uint32_t data);
int mini_isp_ois_RamRead32A(uint16_t addr, void* data);
int mini_isp_ois_RamWrite16A(uint16_t addr, uint16_t data);
int mini_isp_ois_RamRead16A(uint16_t addr, uint16_t *data);
int mini_isp_ois_RegWrite8A(uint16_t addr, uint16_t data);
int mini_isp_ois_RegRead8A(uint16_t addr, uint8_t *data);
void mini_isp_ois_WitTim(uint16_t delay);
int mini_isp_ois_RamWrite16Burst(uint8_t data[], uint16_t length);
int mini_isp_ois_RegWrite8Burst(uint8_t data[], uint16_t length);
int mini_isp_ois_RamWrite32Burst(uint8_t data[], uint16_t length);

#endif
