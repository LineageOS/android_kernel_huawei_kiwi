ccflags-y += -Idrivers/media/platform/msm/camera_v2_kiwi
ccflags-y += -Idrivers/media/platform/msm/camera_v2_kiwi/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2_kiwi/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2_kiwi/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2_kiwi/sensor/cci
obj-$(CONFIG_MSMB_CAMERA_KIWI) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/

obj-$(CONFIG_MSM_CAMERA_SENSOR) += \
	msm_sensor_init.o \
	msm_sensor_driver.o \
	msm_sensor.o \
	sensor_otp_ar1335_sunny_f13m01f.o \
	sensor_otp_common_if.o \
	sensor_otp_imx214.o \
	sensor_otp_imx219.o \
	sensor_otp_imx328_sunny_p13n10a.o \
	sensor_otp_ov5648.o \
	sensor_otp_ov5648_foxconn_sc0602.o \
	sensor_otp_ov5648_sunny_p5v18g.o \
	sensor_otp_ov8858_foxconn.o \
	sensor_otp_ov13850.o \
	sensor_otp_s5k3m2.o \
	sensor_otp_s5k4e1.o \
	sensor_otp_s5k4e1_liteon_13p1.o \
	sensor_otp_s5k4e1_sunny_p5s07a.o \

obj-$(CONFIG_MT9M114) += mt9m114.o

obj-$(CONFIG_OV5645) += \
	ov5645.o \
	ov7695.o
