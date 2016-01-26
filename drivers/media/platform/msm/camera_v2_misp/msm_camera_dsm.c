#include "msm_camera_dsm.h"

/* buffer size for dsm camera client */
#define CAMERA_RADAR_BUF_MAX	4096

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

//extern struct msm_sensor_ctrl_t;

struct dsm_client *camera_dsm_client = NULL;
//struct msm_sensor_ctrl_t *ps_ctrl = NULL;

//int camera_dsm_dump (int type, void *buff, int size);

/* add camera dump_func */
struct dsm_client_ops camera_dsm_ops={
	.poll_state = NULL,
//	.dump_func = camera_dsm_dump,
};

/* dsm client for camera */
struct dsm_dev camera_dsm_dev = {
	.name = "dsm_camera",	// dsm client name
	.fops = &camera_dsm_ops,		// options
	.buff_size = CAMERA_RADAR_BUF_MAX,		// buffer size
};

struct dsm_client* camera_dsm_get_client(void)
{
	if (!camera_dsm_client) 
	{
		camera_dsm_client = dsm_register_client(&camera_dsm_dev);

		if ( NULL == camera_dsm_client)
		{
			CDBG("%s : camera register dsm client failed!!!\n", __func__);
		}
	}
	
	return camera_dsm_client;
}



ssize_t camera_dsm_record_basic_info(struct dsm_client *pcamera_client , int type, int err_num , const char* str)
{

	ssize_t size = 0;
	ssize_t total_size = 0;

	CDBG("%s: entry!\n", __func__);

	/* Camera basic info */
	CDBG("%s: record basic info!\n", __func__);

	size =dsm_client_record(pcamera_client, 
				"[Camera info] DSM Camera Error Type:%d, Error No:%d, %s\n",
				type,
				err_num,
				str);

	total_size += size;

	return total_size;

}


int camera_report_dsm_err( int type, int err_num, const char* str )
{
#ifdef CONFIG_HUAWEI_DSM
	struct dsm_client *pcamera_client = camera_dsm_get_client();

	CDBG("%s: entry! type:%d\n", __func__, type);

	if( NULL == pcamera_client )
	{
		CDBG("%s: there is not pcamera_client!\n", __func__);
		return -1;
	}

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(pcamera_client))
	{
		/* buffer is busy */
		CDBG("%s: buffer is busy!\n", __func__);
		return -1;
	}

	/* report camera basic infomation */
	camera_dsm_record_basic_info(pcamera_client, type, err_num, str);

	dsm_client_notify(pcamera_client, type);

#endif
	return 0;

}

