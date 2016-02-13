



#ifndef _SENSOR_OTP_COMMON_IF_H
#define _SENSOR_OTP_COMMON_IF_H


struct otp_function_t {
	char sensor_name[32];
	int (*sensor_otp_function) (struct msm_sensor_ctrl_t *s_ctrl, int index);
	uint32_t rg_ratio_typical;//r/g ratio
	uint32_t bg_ratio_typical;//b/g ratio
	bool is_boot_load; //whether load otp info when booting.
};


extern bool is_exist_otp_function(struct msm_sensor_ctrl_t *sctl, int32_t *index);
extern int s5k4e1_liteon_13p1_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
extern int ov5648_sunny_p5v18g_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
extern int ov8858_foxconn_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
extern int ov5648_foxconn_sc0602_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
extern int s5k4e1_sunny_p5s07a_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);

extern int ov13850_sunny_p13v01h_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
extern int imx328_sunny_p13n10a_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);

extern struct otp_function_t otp_function_lists [];


#endif
