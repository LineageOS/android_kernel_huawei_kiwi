/**********************************************************
 * Filename:    bluetooth-dsm.c
 *
 * Discription: bluetooth device state monitor driver
 *
 * Copyright: (C) 2015 huawei.
 *
 * Author:
 *
**********************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <dsm/dsm_pub.h>
#include <linux/errno.h>


static struct dsm_client *bt_dclient = NULL;

static struct dsm_dev dsm_bt = {
    .name = "dsm_bluetooth",
    .fops = NULL,
    .buff_size = 1024,
};


#define DSM_BLUETOOTH_DM_OPEN                 1
#define DSM_BLUETOOTH_DM_GET_ADDR             2
#define DSM_BLUETOOTH_A2DP_CONNECT            3
#define DSM_BLUETOOTH_A2DP_AUDIO              4
#define DSM_BLUETOOTH_HFP_CONNECT             5
#define DSM_BLUETOOTH_HFP_SCO_CONNECT         6
#define DSM_BLUETOOTH_BLE_CONNECT             7

static int bluetooth_open(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static int bluetooth_close(struct inode *inode, struct file *file)
{
    return 0;
}

static long bluetooth_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static ssize_t bluetooth_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    return 0;
}

static ssize_t bluetooth_write(struct file *filp, const char __user *buf,
                size_t count, loff_t *offset)
{
    int ret = 0;
    int val = 0;
    int error_no = 0;
    pr_info("%s enter\n",__func__);

    if(NULL == buf){
        pr_err("%s: input para buf is NULL!\n", __func__);
        return -EINVAL;
    }

    pr_info("%s: dsm report op:%s\n", __func__, buf);

    if (1 != sscanf(buf, "%2d", &val)){
        pr_err("%s : get buf failed: %s\n", __func__,buf);
        return -EINVAL;
    }
    if(!bt_dclient){
        pr_err("%s: bt_dclient is NULL!\n", __func__);
        return -1;
    }
    if(dsm_client_ocuppy(bt_dclient)){
        pr_err("%s: buffer is busy!\n", __func__);
        return -1;
    }
    switch(val)
    {
        case DSM_BLUETOOTH_DM_OPEN:
            error_no = DSM_BLUETOOTH_DM_OPEN_ERROR;
            ret = dsm_client_record(bt_dclient, "bluetooth open fail");
            break;
        case DSM_BLUETOOTH_DM_GET_ADDR:
            error_no = DSM_BLUETOOTH_DM_GET_ADDR_ERROR;
            ret = dsm_client_record(bt_dclient, "get bt addr fail");
            break;
        case DSM_BLUETOOTH_A2DP_CONNECT:
            error_no = DSM_BLUETOOTH_A2DP_CONNECT_ERROR;
            ret = dsm_client_record(bt_dclient, "a2dp connect fail");
            break;
        case DSM_BLUETOOTH_A2DP_AUDIO:
            error_no = DSM_BLUETOOTH_A2DP_AUDIO_ERROR;
            ret = dsm_client_record(bt_dclient, "a2dp audio fail");
            break;
        case DSM_BLUETOOTH_HFP_CONNECT:
            error_no = DSM_BLUETOOTH_HFP_CONNECT_ERROR;
            ret = dsm_client_record(bt_dclient, "hfp connect fail");
            break;
        case DSM_BLUETOOTH_HFP_SCO_CONNECT:
            error_no = DSM_BLUETOOTH_HFP_SCO_CONNECT_ERROR;
            ret = dsm_client_record(bt_dclient, "hfp sco connect fail");
            break;
        case DSM_BLUETOOTH_BLE_CONNECT:
            error_no = DSM_BLUETOOTH_BLE_CONNECT_ERROR;
            ret = dsm_client_record(bt_dclient, "ble connect fail");
            break;
        default :
            break;
    }
    if(!ret){
        pr_err("%s: no need report! error no:%d\n", __func__,error_no );
        return 0;
    }

    /*call dsm report interface*/
    dsm_client_notify(bt_dclient, error_no); 
    return 0;
}

static const struct file_operations bluetooth_fops = {
    .owner     = THIS_MODULE,
    .llseek    = no_llseek,
    .open      = bluetooth_open,
    .release   = bluetooth_close,
    .read      = bluetooth_read,
    .write     = bluetooth_write,
    .unlocked_ioctl = bluetooth_ioctl,
};

static struct miscdevice bluetooth_miscdev = {
    .minor     = MISC_DYNAMIC_MINOR,
    .name      = "bluetooth",
    .fops      = &bluetooth_fops,
};

static int __init bluetooth_dsm_init(void)
{
    int retval = -EIO;
    
    if(!bt_dclient){
       bt_dclient = dsm_register_client (&dsm_bt);
    }

    retval = misc_register(&bluetooth_miscdev);
    if (retval) {
        pr_err("%s: misc_register err %d\n",
            __func__, retval);
        return retval;
    }
    
    return 0;
}

static void __exit bluetooth_dsm_exit(void)
{
    dsm_unregister_client(bt_dclient,&dsm_bt);
}

module_init(bluetooth_dsm_init);

module_exit(bluetooth_dsm_exit);

MODULE_DESCRIPTION("BT DEVICE MONITOR");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
