/*
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.


   Copyright (C) 2011-2013  Huawei Corporation
*/
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <asm/uaccess.h>
#include <linux/wcnss_wlan.h>

#define WLAN_DEVICE_VERSION		"1.1"
#define PROC_DIR	"wlan_feature"

#define WLAN_CHIP_TYPE_UNKNOWN "Unknown"

struct proc_dir_entry    *wlan_dir;

static int wlan_featuretrans_remove(struct platform_device *pdev)
{
    printk("wlan devicefeature removed.");
    return 0;
}

static struct platform_driver wlan_featuretrans_driver = {
    .remove = wlan_featuretrans_remove,
    .driver = {
        .name = "wlanfeaturetrans",
        .owner = THIS_MODULE,
    },
};


const void *get_wifi_device_type(void)
{
    int wifi_device_type_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
         printk("%s:device is not available!\n",__func__);
         return WLAN_CHIP_TYPE_UNKNOWN;
    }

    return  of_get_property(dp,"wifi,chiptype", &wifi_device_type_len);
}
EXPORT_SYMBOL(get_wifi_device_type);


static int devtype_proc_show(struct seq_file *m, void *v)
{
    const char *wifi_device_type = NULL;

    wifi_device_type = get_wifi_device_type();
    if( NULL != wifi_device_type )
    {
        printk("%s enter wifi_device_type:%s;\n", __func__,wifi_device_type);
    }
    else
    {
        printk("%s get wifi_device_type failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wifi_device_type);

    return 0;
}

static int devtype_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, devtype_proc_show, NULL);
}

static const struct file_operations devtype_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= devtype_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


const void *get_hw_wifi_pubfile_id(void)
{
    int wifi_pubfile_id_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
         printk("%s:device is not available!\n",__func__);
         return NULL;
    }

    return  of_get_property(dp,"wifi,pubfile_id", &wifi_pubfile_id_len);
}
EXPORT_SYMBOL(get_hw_wifi_pubfile_id);

static int pubfd_proc_show(struct seq_file *m, void *v)
{
    const char *wifi_pubfile_id = NULL;

    wifi_pubfile_id = get_hw_wifi_pubfile_id();
    if( NULL != wifi_pubfile_id )
    {
        printk("%s enter wifi_pubfile_id:%s;\n", __func__,wifi_pubfile_id);
    }
    else
    {
        printk("%s get pubfd failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wifi_pubfile_id);

    return 0;
}

static int pubfd_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pubfd_proc_show, NULL);
}

static const struct file_operations pubfd_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= pubfd_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


const void *get_wlan_fw_ver(void)
{
    int wlan_fw_ver_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
        printk("%s:device is not available!\n",__func__);
         return NULL;
    }

    return  of_get_property(dp,"wifi,fw_ver", &wlan_fw_ver_len);
}

static int wlan_fw_ver_proc_show(struct seq_file *m, void *v)
{
    const char *wlan_fw_ver = NULL;

    wlan_fw_ver = get_wlan_fw_ver();
    if( NULL !=wlan_fw_ver )
    {
         printk("%s enter wlan_fw_ver:%s;\n", __func__,wlan_fw_ver);
    }
    else
    {
        printk("%s get wlan_fw_ver failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wlan_fw_ver);

    return 0;
}

static int wlan_fw_ver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlan_fw_ver_proc_show, NULL);
}

 static const struct file_operations wlan_fw_ver_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= wlan_fw_ver_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*parse the new feature of softap feature added in the dtsi*/
const void *get_wlan_softap_feature(void)
{
    int wlan_softap_feature_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
        printk("%s:device is not available!\n",__func__);
        return NULL;
    }

    return  of_get_property(dp,"wifi,softap_feature", &wlan_softap_feature_len);
}
EXPORT_SYMBOL(get_wlan_softap_feature);

static int wlan_softap_feature_show(struct seq_file *m, void *v)
{
    const char *wlan_softap_feature = NULL;

    wlan_softap_feature = get_wlan_softap_feature();
    if( NULL !=wlan_softap_feature )
    {
        printk("%s enter wlan_softap_feature:%s;\n", __func__,wlan_softap_feature);
    }
    else
    {
        printk("%s get wlan_softap_feature failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wlan_softap_feature);

    return 0;
}

static int wlan_softap_feature_open(struct inode *inode, struct file *file)
{
    printk("%s enter;\n", __func__);
	return single_open(file, wlan_softap_feature_show, NULL);
}

 static const struct file_operations wlan_softap_feature_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= wlan_softap_feature_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*parse the new feature of operator added in the dtsi*/
const void *get_wlan_ap_operator(void)
{
    int wlan_ap_operator_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
        printk("%s:device is not available!\n",__func__);
        return NULL;
    }

    return  of_get_property(dp,"wifi,ap_operator", &wlan_ap_operator_len);
}
EXPORT_SYMBOL(get_wlan_ap_operator);

static int wlan_ap_operator_show(struct seq_file *m, void *v)
{
    const char *wlan_ap_operator = NULL;

    wlan_ap_operator = get_wlan_ap_operator();
    if( NULL !=wlan_ap_operator )
    {
        printk("%s enter wlan_ap_operator:%s;\n", __func__,wlan_ap_operator);
    }
    else
    {
        printk("%s get wlan_ap_operator failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wlan_ap_operator);

    return 0;
}

static int wlan_ap_operator_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlan_ap_operator_show, NULL);
}

 static const struct file_operations wlan_ap_operator_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= wlan_ap_operator_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


const void *get_fem_check_flag(void)
{
    int get_fem_check_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
        printk("%s:device is not available!\n",__func__);
        return NULL;
    }

    return  of_get_property(dp,"wifi,fem_check_flag", &get_fem_check_len);
}
EXPORT_SYMBOL(get_fem_check_flag);

static int fen_check_flag_show(struct seq_file *m, void *v)
{
    const char *fem_check_flag = NULL;

    fem_check_flag = get_fem_check_flag();
    if( NULL !=fem_check_flag )
    {
        printk("%s enter fem_check_flag:%s;\n", __func__,fem_check_flag);
    }
    else
    {
        printk("%s get fem_check_flag failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",fem_check_flag);

    return 0;
}

static int fem_check_flag_open(struct inode *inode, struct file *file)
{
	return single_open(file, fen_check_flag_show, NULL);
}

 static const struct file_operations fem_check_flag_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= fem_check_flag_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init wlanfeaturetrans_init(void)
{
    int retval = 0;
    struct proc_dir_entry *ent = NULL;

    printk("%s:WIFI DEVICE FEATURE VERSION: %s",__func__,WLAN_DEVICE_VERSION);

    /* Driver Register */
    retval = platform_driver_register(&wlan_featuretrans_driver);
    if (0 != retval)
    {
        printk("%s:featurntransfer driver register fail.", __func__);
        return retval;
    }

    /* create device_feature directory for wifi chip info */
    wlan_dir = proc_mkdir("wlan_feature", NULL);
    if (NULL == wlan_dir)
    {
        printk("%s:Unable to create /proc/wlan_feature directory",__func__);
        retval =  -ENOMEM;
        goto error_dir;
    }

    /* Creating read/write "devtype" entry*/
    ent= proc_create("devtype", 0, wlan_dir, &devtype_proc_fops);
    if (!ent)
    {
        printk("%s:Unable to create /proc/%s/devtype entry",__func__,PROC_DIR);
        retval =  -ENOMEM;
        goto error_devtype;
    }

    /* Creating read/write "pubfd" entry*/
    ent= proc_create("pubfd", 0, wlan_dir, &pubfd_proc_fops);
    if (!ent)
    {
        printk("%s:Unable to create /proc/%s/pubfd entry",__func__,PROC_DIR);
        retval =  -ENOMEM;
        goto error_pubfd;
    }

    /* Creating read/write "wlan_fw_ver" entry for wifi chip type*/
    ent= proc_create("wlan_fw_ver", 0, wlan_dir, &wlan_fw_ver_proc_fops);
    if (!ent)
    {
        printk("%s:Unable to create /proc/%s/wlan_fw_ver entry",__func__,PROC_DIR);
        retval =  -ENOMEM;
        goto error_wlan_fw_ver;
    }

    /* Creating read/write "softap_wps" entry for wifi chip type*/
    ent= proc_create("softap_feature", 0, wlan_dir, &wlan_softap_feature_proc_fops);
    if (!ent)
    {
        printk("%s:Unable to create /proc/%s/softap_feature entry",__func__,PROC_DIR);
        retval =  -ENOMEM;
        goto error_wlan_softap_wps;
    }

    /* Creating read/write "wlan_ap_operator" entry for wifi chip type*/
    ent= proc_create("wlan_ap_operator", 0, wlan_dir, &wlan_ap_operator_proc_fops);
    if (!ent)
    {
        printk("%s:Unable to create /proc/%s/wlan_ap_operator entry",__func__,PROC_DIR);
        retval =  -ENOMEM;
        goto error_wlan_ap_operator;
    }

    /* Creating read/write "fem_check_flag" entry for wifi chip type*/
    ent= proc_create("fem_check_flag", 0, wlan_dir, &fem_check_flag_proc_fops);
    if (!ent)
    {
        printk("%s:Unable to create /proc/%s/fem_check_flag entry",__func__,PROC_DIR);
        retval =  -ENOMEM;
        goto error_fem_check_flag;
    }

    return 0;
error_fem_check_flag:
    remove_proc_entry("fem_check_flag", wlan_dir);
error_wlan_ap_operator:
    remove_proc_entry("wlan_ap_operator", wlan_dir);
error_wlan_softap_wps:
    remove_proc_entry("softap_feature", wlan_dir);
error_wlan_fw_ver:
    remove_proc_entry("wlan_fw_ver", wlan_dir);
error_pubfd:
    remove_proc_entry("pubfd", wlan_dir);
error_devtype:
    remove_proc_entry("devtype", wlan_dir);
error_dir:
    remove_proc_entry("wlan_feature", NULL);
    platform_driver_unregister(&wlan_featuretrans_driver);

    return retval;
}


/**
 * Cleans up the module.
 */
static void __exit wlanfeaturetrans_exit(void)
{
    platform_driver_unregister(&wlan_featuretrans_driver);
    remove_proc_entry("devtype", wlan_dir);
    remove_proc_entry("pubfd", wlan_dir);
    remove_proc_entry("wlan_fw_ver", wlan_dir);
    remove_proc_entry("wlan_ap_operator", wlan_dir);
    remove_proc_entry("fem_check_flag", wlan_dir);
    remove_proc_entry("softap_feature", wlan_dir);
    remove_proc_entry("wlan_feature", 0);
}


module_init(wlanfeaturetrans_init);
module_exit(wlanfeaturetrans_exit);

MODULE_DESCRIPTION("WIFI DEVICE FEATURE VERSION: %s " WLAN_DEVICE_VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
