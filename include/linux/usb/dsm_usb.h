#ifdef CONFIG_HUAWEI_USB_DSM
#ifndef LINUX_USB_DSM_USB_H
#define LINUX_USB_DSM_USB_H
#include <dsm/dsm_pub.h>

/* define a 1024 size of array as buffer */
#define USB_DSM_BUFFER_SIZE  1024
#define USB_MSG_MAX_SIZE 200
/* Error code, decimal[5]: 0 is input, 1 is output, 2 I&O
 * decimal[4:3]: 13 is for usb,
 * decimal[2:1]: for different error code.
 */
enum DSM_USB_ERR
{
	DSM_USB_PHY_PMICVBUS_ERR 		= 21301,
	DSM_USB_PHY_DCD_ERR,
	DSM_USB_HOST_BUS_ERR,
	DSM_USB_HOST_MS_TR_ERR,
	DSM_USB_DEVICE_EMU__ERR,
	DSM_USB_DEVICE_MS_SCSI_ERR,
	DSM_USB_DEVICE_MS_CMD_ERR,
	DSM_USB_DEVICE_MS_REPLY_ERR,
	DSM_USB_DEVICE_MTP_RD_ERR,
	DSM_USB_DEVICE_MTP_WR_ERR,
	DSM_USB_DEVICE_ADB_OFFLINE_ERR,
};


enum DSM_USB_SUBSYSTEM
{
	DSM_USB_PHY 		= 1,
	DSM_USB_HOST,
	DSM_USB_DEVICE,
};
struct usb_dsm_log {
	char usb_dsm_log[USB_DSM_BUFFER_SIZE];
	spinlock_t lock;	/* mutex */
};

extern struct dsm_client *usb_dclient;

/*buffer for transffering to device radar*/
extern struct usb_dsm_log g_usb_dsm_log;

extern int dsm_usb_get_log(int sub_sys_type, void *p_sub, int err_num, char *err_msg);

/*Transfer the msg to device radar*/
#define DSM_USB_LOG(sub_sys_type, sub_sys, error_num, fmt, a...) \
	do { \
		char msg[USB_MSG_MAX_SIZE]; \
		snprintf(msg, USB_MSG_MAX_SIZE-1, fmt, ## a); \
		spin_lock(&g_usb_dsm_log.lock); \
		if(dsm_usb_get_log((sub_sys_type), (sub_sys), (error_num), (msg))){ \
			if(!dsm_client_ocuppy(usb_dclient)) { \
				dsm_client_copy(usb_dclient,g_usb_dsm_log.usb_dsm_log, strlen(g_usb_dsm_log.usb_dsm_log) + 1); \
				dsm_client_notify(usb_dclient, error_num); } \
		} \
		spin_unlock(&g_usb_dsm_log.lock); \
	}while(0)

#endif /* LINUX_USB_DSM_USB_H */
#endif /* CONFIG_HUAWEI_USB_DSM */
