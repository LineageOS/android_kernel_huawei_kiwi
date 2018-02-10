ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/sensor
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/codecs
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/isps
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/pproc
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/jpeg_10
ccflags-y += -Idrivers/media/platform/msm/camera_v2_misp/fd

obj-$(CONFIG_MSMB_CAMERA) += msm.o
obj-$(CONFIG_MSMB_CAMERA) += hw_camera_common.o
obj-$(CONFIG_MSMB_CAMERA) += msm_camera_dsm.o
obj-$(CONFIG_MSMB_CAMERA) += camera/
obj-$(CONFIG_MSMB_CAMERA) += msm_vb2/
obj-$(CONFIG_MSMB_CAMERA) += sensor/
obj-$(CONFIG_MSMB_CAMERA) += pproc/
obj-$(CONFIG_MSMB_CAMERA) += isp/
obj-$(CONFIG_MSMB_CAMERA) += ispif/
obj-$(CONFIG_MSMB_JPEG) += jpeg_10/
obj-$(CONFIG_MSMB_CAMERA) += msm_buf_mgr/
obj-$(CONFIG_MSMB_CAMERA) += hw_camera_log.o

