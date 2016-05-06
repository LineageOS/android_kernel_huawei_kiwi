/*
 * Copyright (C) 2012-2014 HUAWEI, Inc.
 * Author: HUAWEI, Inc.
 */

#define pr_fmt(fmt)	"Ii-BMS: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/firmware.h>
#include <linux/power/bq27510_bms.h>
#include <linux/power/bq24152_charger.h>
#include <linux/power/bq24296m_charger.h>
#include <linux/of_gpio.h>
#include <linux/charger_core.h>
#include <linux/of_batterydata.h>
#include <linux/qpnp/qpnp-adc.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#ifdef CONFIG_HUAWEI_PMU_DSM
#include <linux/power/huawei_dsm_charger.h>
#include <linux/rtc.h>
#endif
#include <linux/charger_core.h>
#define HWLOG_TAG bq27510_bms
#define ID_LEN   12
extern int is_usb_chg_exist(void);
#if defined(PRODUCTION_ALE_KERNEL) || defined(PRODUCTION_G760_KERNEL)
struct bq2415x_device *bq_device;
#else
struct bq24296m_device_info *bq_device;
atomic_t battery_full_flag = ATOMIC_INIT(0);
#endif

static bool use_ti_coulometer=false;
static bool use_filter_mode = false;
static DEFINE_MUTEX(bq27510_battery_mutex);

struct firmware_header
{
    unsigned int magic_number;
    char file_name[48];
    unsigned int offset;
    unsigned int length;
};

struct i2c_client* g_battery_measure_by_bq27510_i2c_client = NULL;
struct bq27510_device_info* g_battery_measure_by_bq27510_device = NULL;
static struct i2c_driver bq27510_battery_driver;
static unsigned int gBq27510DownloadFirmwareFlag = BSP_NORMAL_MODE;
static bool force_update_flag = false;
#define FW_UPD_OK            0
#define FW_UPD_FAIL          1
#define FW_UPD_PROCESSING    2
static char fw_version_export[ID_LEN] = {0};
static int fw_update_ok = FW_UPD_PROCESSING;
static bool battery_alarm_enabled = false;
#define HIGH_TEMP		55

enum
{
    BQ27510_NORMAL_MODE,
    BQ27510_UPDATE_FIRMWARE,
    BQ27510_LOCK_MODE
};

struct bq27510_context
{
    unsigned int temperature;
    unsigned int capacity;
    unsigned int volt;
    unsigned int bat_current;
    unsigned int remaining_capacity;
    unsigned int full_capacity;
    unsigned int battery_present;
    unsigned int battery_health;
    unsigned char state;
    unsigned long locked_timeout_jiffies;//after updating firmware, device can not be accessed immediately
    unsigned int i2c_error_count;
    unsigned int lock_count;
};

struct bq27510_context gauge_context =
{
    .temperature = 2820,//9 degree
    .capacity = 88,     //50 percent
    .volt = 3700,       // 3.7 V
    .bat_current = 200,// 200 mA
    .remaining_capacity = 800,//mAH
    .full_capacity = 1800,
    .battery_present = 1,
    .battery_health = POWER_SUPPLY_HEALTH_GOOD,
    .state = BQ27510_NORMAL_MODE,
    .i2c_error_count = 0
};


#define GAS_GAUGE_I2C_ERR_STATICS() ++gauge_context.i2c_error_count
#define GAS_GAUGE_LOCK_STATICS() ++gauge_context.lock_count


static bool factory_flag = false;
static int __init early_parse_factory_flag(char * p)
{
    if(p)
    {
        if(!strcmp(p,"factory"))
        {
            factory_flag = true;
        }
    }
    return 0;
}
early_param("androidboot.huawei_swtype",early_parse_factory_flag);

static int bq27510_is_accessible(void)
{
    if(gauge_context.state == BQ27510_UPDATE_FIRMWARE)
    {
        pr_debug("bq27510 isn't accessible,It's updating firmware!\n");
        GAS_GAUGE_LOCK_STATICS();
        return 0;
    }
    else if(gauge_context.state == BQ27510_NORMAL_MODE)
        return 1;
    else
    {
        if(time_is_before_jiffies(gauge_context.locked_timeout_jiffies))
        {
            gauge_context.state = BQ27510_NORMAL_MODE;
            return 1;
        }
        else
        {
            pr_debug("bq27510 isn't accessible after firmware updated immediately!\n");
            return 0;
        }
    }
}


static int bq27510_i2c_read_word(struct bq27510_device_info *di,u8 reg)
{
    int err = 0;
    int i = 0;
    if(NULL == di)
    {
        return err;
    }
    mutex_lock(&bq27510_battery_mutex);
    for(i = 0; i < 5; i++)
    {
        err = i2c_smbus_read_word_data(di->client,reg);
        if (err < 0)
        {
            GAS_GAUGE_I2C_ERR_STATICS();
            pr_info("[%s,%d] i2c_smbus_read_byte_data failed\n",__FUNCTION__,__LINE__);
        }
        else
        {
            break;
        }
        msleep(5);
    }
    mutex_unlock(&bq27510_battery_mutex);

    return err;
}


static int bq27510_i2c_word_write(struct i2c_client *client, u8 reg, u16 value)
{
    int err = 0;
    int i = 0;

    mutex_lock(&bq27510_battery_mutex);
    for(i = 0; i < 5; i++)
    {
        err = i2c_smbus_write_word_data(client, reg, value);
        if (err < 0)
        {
            GAS_GAUGE_I2C_ERR_STATICS();
            pr_debug("i2c_smbus_write_word_data failed\n");
        }
        else
        {
            break;
        }
        msleep(5);
    }
    mutex_unlock(&bq27510_battery_mutex);

    return err;
}

static int bq27510_i2c_bytes_write(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
    int i2c_ret = 0, i = 0,j = 0;
    u8 *p;

    p = pBuf;

    mutex_lock(&bq27510_battery_mutex);
    for(i=0; i<len; i+=I2C_SMBUS_BLOCK_MAX)
    {
        j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);
        i2c_ret = i2c_smbus_write_i2c_block_data(client, reg+i, j, p+i);

        if (i2c_ret < 0)
        {
            GAS_GAUGE_I2C_ERR_STATICS();
            pr_debug("i2c_transfer failed\n");
            break;
        }
    }
    mutex_unlock(&bq27510_battery_mutex);

    return i2c_ret;
}

static int bq27510_i2c_bytes_read(struct i2c_client *client, u8 reg, u8 *pBuf, u16 len)
{
    int i2c_ret = 0, i = 0, j = 0;
    u8 *p;

    p = pBuf;

    mutex_lock(&bq27510_battery_mutex);
    for(i=0; i<len; i+=I2C_SMBUS_BLOCK_MAX)
    {
        j = ((len - i) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - i);
        i2c_ret = i2c_smbus_read_i2c_block_data(client, reg+i, j, p+i);
        if (i2c_ret < 0)
        {
            GAS_GAUGE_I2C_ERR_STATICS();
            pr_debug("i2c_transfer failed\n");
            break;
        }
    }
    mutex_unlock(&bq27510_battery_mutex);

    return i2c_ret;
}

static int bq27510_i2c_bytes_read_and_compare(struct i2c_client *client, u8 reg, u8 *pSrcBuf, u8 *pDstBuf, u16 len)
{
    int i2c_ret = 0;

    i2c_ret = bq27510_i2c_bytes_read(client, reg, pSrcBuf, len);
    if(i2c_ret < 0)
    {
        GAS_GAUGE_I2C_ERR_STATICS();
        pr_debug("bq27510_i2c_bytes_read failed\n");
        return i2c_ret;
    }

    i2c_ret = strncmp(pDstBuf, pSrcBuf, len);

    return i2c_ret;
}

int bq27510_battery_temperature(struct bq27510_device_info *di)
{
    int data = -1;

    if(bq27510_is_accessible())
    {
        data = bq27510_i2c_read_word(di,BQ27510_REG_TEMP);
        if(data < 0)
            pr_debug("i2c error in reading temperature!");
        else
            gauge_context.temperature = data;
    }

    if(data < 0)
        data = gauge_context.temperature;

    data = (data-CONST_NUM_2730)/CONST_NUM_10;
    pr_debug("read temperature result = %d Celsius\n",data);
    return data ;
}

int bq27510_battery_voltage(struct bq27510_device_info *di)
{
    int data = -1;

    if(!bq27510_is_accessible())
        return gauge_context.volt;

    data = bq27510_i2c_read_word(di,BQ27510_REG_VOLT);
    if(data < 0)
    {
        pr_debug("i2c error in reading voltage!");
        data = gauge_context.volt;
    }
    else
        gauge_context.volt = data;

    pr_debug("read voltage result = %d mVolts\n",data);
    return data;//adapt android upper layer unit: mV
}

short bq27510_battery_current(struct bq27510_device_info *di)
{
    int data = -1;
    short nCurr = 0;

    if(!bq27510_is_accessible())
        return gauge_context.bat_current;

    data = bq27510_i2c_read_word(di,BQ27510_REG_AI);
    if(data < 0)
    {
        pr_debug("i2c error in reading current!");
        data = gauge_context.bat_current;
    }
    else
        gauge_context.bat_current = data;


    nCurr = (signed short)data;

    pr_debug("read current result = %d mA\n", nCurr);

    return nCurr;
}

int bq27510_battery_capacity(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return gauge_context.capacity;
    data = bq27510_i2c_read_word(di,di->soc_reg);

    pr_debug("---> capacity = %d\n",data);
    if((data < 0)||(data > 100))
    {
        pr_debug("capacity is out of 0~100");
        data = gauge_context.capacity;
    }
    else
    {
        gauge_context.capacity = data;
    }

    pr_debug("read soc result = %d Hundred Percents\n",data);
    return data;
}

int bq27510_battery_rm(struct bq27510_device_info *di)
{
    int data = -1;

    if(!bq27510_is_accessible())
        return gauge_context.remaining_capacity;

    data = bq27510_i2c_read_word(di,BQ27510_REG_RM);
    if(data < 0)
    {
        pr_debug("i2c error in reading remain capacity!");
        data = gauge_context.remaining_capacity;
    }
    else
        gauge_context.remaining_capacity = data;


    pr_debug("read rm result = %d mAh\n",data);
    return data;
}

int bq27510_battery_fcc(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FCC);
    if(data < 0)
    {
        pr_debug("i2c error in reading FCC!");
        data = gauge_context.full_capacity;
    }
    else
        gauge_context.full_capacity = data;

    pr_debug("read fcc result = %d mAh\n",data);
    return data;
}

static int bq27510_get_gasgauge_qmax(struct bq27510_device_info *di)
{
    int control_status = 0,qmax = 0,qmax1 = 0;
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    mutex_lock(&bq27510_battery_mutex);
    i2c_smbus_write_word_data(di->client,BQ27510_REG_CTRL,BQ27510_REG_CTRS);
    mdelay(2);
    control_status  = i2c_smbus_read_word_data(di->client,BQ27510_REG_CTRL);
    mdelay(2);
    i2c_smbus_write_word_data(di->client,BQ27510_REG_DFCLS,BQ27510_REG_CLASS_ID);
    mdelay(2);
    qmax = i2c_smbus_read_byte_data(di->client,BQ27510_REG_QMAX);
    mdelay(2);
    qmax1 = i2c_smbus_read_byte_data(di->client,BQ27510_REG_QMAX1);
    mutex_unlock(&bq27510_battery_mutex);
    data = (qmax << 8) | qmax1;

    return data;
}


int bq27510_battery_tte(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_TTE);
    if(data < 0)
    {
        pr_debug("i2c error in reading TTE!");
        return 0;
    }
    pr_debug("read tte result = %d minutes\n",data);
    return data;
}

int bq27510_battery_cyc(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_CYC);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading CYC!");
        return 0;
    }

    pmu_log_debug("read cyc result = %d minutes\n",data);
    return data;
}
int bq27510_battery_ttf(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_TTF);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading TTF!");
        return 0;
    }

    pmu_log_debug("read ttf result = %d minutes\n",data);
    return data;
}
int bq27510_battery_ufrm(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_UFRM);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading UFRM!");
        return 0;
    }
    pmu_log_debug("read bq27510_battery_ufrm result = %d mAh\n",data);
    return data;
}

int bq27510_battery_frm(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FRM);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading FRM!");
        return 0;
    }
    pmu_log_debug("read bq27510_battery_frm result = %d mAh\n",data);
    return data;
}

int bq27510_battery_uffcc(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_UFFCC);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading UFFCC!");
        return 0;
    }
    pmu_log_debug("read bq27510_battery_uffcc result = %d mAh\n",data);
    return data;
}

int bq27510_battery_ffcc(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FFCC);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading FFCC!");
        return 0;
    }
    pmu_log_debug("read bq27510_battery_ffcc result = %d mAh\n",data);
    return data;
}

int bq27510_battery_ufsoc(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_UFSOC);
    if(data < 0)
    {
        pmu_log_debug("i2c error in reading UFSOC!");
        return 0;
    }
    pmu_log_debug("read bq27510_battery_ufsoc result = %d Hundred Percents\n",data);
    return data;
}
int is_bq27510_battery_full(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    if(data < 0)
    {
        pr_debug("i2c error in reading battery charging full bit!");
        data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
        if(data < 0)
            return 0;
    }

    pr_debug("read flags result = 0x%x \n",data);
    return (data & BQ27510_FLAG_FC);
}

int is_bq27510_battery_discharging(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    if(data < 0)
    {
        pr_debug("i2c error in reading battery discharging!");
        return 0;
    }

    pr_debug("read flags result = 0x%x \n",data);
    return (data & BQ27510_FLAG_DSG);
}

int is_bq27510_battery_exist(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return gauge_context.battery_present;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    pr_debug("is_exist flags result = 0x%x \n",data);
    if(data < 0)
    {
        pr_debug("i2c error in reading battery_exist!");
        data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    }

    if(data >= 0)
    {
        gauge_context.battery_present = !!(data & BQ27510_FLAG_DET);
    }
    else
    {
        pr_info("i2c read BQ27510_REG_FLAGS error = %d \n",data);
    }

    return gauge_context.battery_present;
}


int is_bq27510_battery_reach_threshold(struct bq27510_device_info *di)
{
    int data = 0;

    if(!bq27510_is_accessible())
        return 0;

    if(!is_bq27510_battery_exist(di))
        return 0;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    if(data < 0)
        return 0;

    return data;
}

#define STATE_HARDWARE_ERR 2
static int bq27510_get_firmware_version_by_i2c(struct i2c_client *client)
{
    unsigned int data = 0;
    int id = 0;
    int ver = 0;

    mutex_lock(&bq27510_battery_mutex);
    if (0 != i2c_smbus_write_word_data(client,BQ27510_REG_CTRL,BQ27510_REG_FIRMWARE_ID))
    {
        mutex_unlock(&bq27510_battery_mutex);
        return -1;
    }
    mdelay(2);
    id = i2c_smbus_read_word_data(client,BQ27510_REG_CTRL);
    mdelay(2);
    if (0 != i2c_smbus_write_word_data(client,BQ27510_REG_DFCLS,BQ27510_REG_FIRMWARE_VERSION))
    {
        mutex_unlock(&bq27510_battery_mutex);
        return -1;
    }
    mdelay(2);
    ver = i2c_smbus_read_byte_data(client,BQ27510_REG_FLASH);
    mdelay(2);
    mutex_unlock(&bq27510_battery_mutex);
    if (id < 0 || ver < 0)
        return -1;
    data = (id << 16) | ver;
    return data;
}


int bq27510_battery_status(struct bq27510_device_info *di)
{
    int data=0;
    int status =0;
    short m_current = 0;

    if(!bq27510_is_accessible())
        return 0;

    /*Use the current direction to decide charging or discharging*/
    m_current = bq27510_battery_current(di);
    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    if(data < 0)
    {
        pr_debug("i2c error in reading battery status!");
        return 0;
    }
    pr_debug("read status result = %d minutes\n",data);

    if (data & BQ27510_FLAG_FC)
        status = POWER_SUPPLY_STATUS_FULL;
    else if ( (data & BQ27510_FLAG_DSG) && m_current < 0 )
        status = POWER_SUPPLY_STATUS_DISCHARGING;
    else if ( m_current > 0)
        status = POWER_SUPPLY_STATUS_CHARGING;
    else if ( 0 == m_current )
        status = POWER_SUPPLY_STATUS_NOT_CHARGING;

    return status;
}

int bq27510_battery_health(struct bq27510_device_info *di)
{
    int data=0;
    int status =0;


    if(!bq27510_is_accessible())
        return gauge_context.battery_health;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    if(data < 0)
    {
        pr_debug("i2c error in reading battery health!");
        return POWER_SUPPLY_HEALTH_GOOD;
    }

    pr_debug("read health result = %d \n",data);

    if (data & (BQ27510_FLAG_OTC | BQ27510_FLAG_OTD))
        status = POWER_SUPPLY_HEALTH_OVERHEAT;

    else
        status = POWER_SUPPLY_HEALTH_GOOD;

    gauge_context.battery_health = status;
    return status;
}

int bq27510_battery_capacity_level(struct bq27510_device_info *di)
{
    int data=0;
    int data_capacity = 0;
    int status =0;

    if(!bq27510_is_accessible())
        return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

    data = bq27510_i2c_read_word(di,BQ27510_REG_FLAGS);
    data_capacity = bq27510_battery_capacity(di);

    pr_debug("read capactiylevel result = %d \n",data);
    if(data < 0)
        return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

    if (data & BQ27510_FLAG_SOCF )
        status = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
    else if (data & BQ27510_FLAG_SOC1 )
        status = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
    else if (data & BQ27510_FLAG_FC )
        status = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
    else if( data_capacity > 95)
        status = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
    else
        status = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

    return status;
}

static int bq27510_atoi(const char *s)
{
    int k = 0;

    k = 0;
    while (*s != '\0' && *s >= '0' && *s <= '9')
    {
        k = 10 * k + (*s - '0');
        s++;
    }
    return k;
}

static unsigned long bq27510_strtoul(const char *cp, unsigned int base)
{
    unsigned long result = 0,value;

    while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp-'0' : (islower(*cp)
                                     ? toupper(*cp) : *cp)-'A'+10) < base)
    {
        result = result*base + value;
        cp++;
    }

    return result;
}


static int bq27510_firmware_program(struct i2c_client *client, const unsigned char *pgm_data, unsigned int filelen)
{
    unsigned int i = 0, j = 0, ulDelay = 0, ulReadNum = 0;
    unsigned int ulCounter = 0, ulLineLen = 0;
    unsigned char temp = 0;
    unsigned char *p_cur;
    unsigned char pBuf[BSP_MAX_ASC_PER_LINE] = { 0 };
    unsigned char p_src[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
    unsigned char p_dst[BSP_I2C_MAX_TRANSFER_LEN] = { 0 };
    unsigned char ucTmpBuf[16] = { 0 };

bq27510_firmware_program_begin:
    if(ulCounter > 10)
    {
        return -1;
    }

    p_cur = (unsigned char *)pgm_data;

    while(1)
    {
        while (*p_cur == '\r' || *p_cur == '\n')
        {
            p_cur++;
        }

        if((p_cur - pgm_data) >= filelen)
        {
            printk("Download success\n");
            break;
        }

        i = 0;
        ulLineLen = 0;

        memset(p_src, 0x00, sizeof(p_src));
        memset(p_dst, 0x00, sizeof(p_dst));
        memset(pBuf, 0x00, sizeof(pBuf));

        while(i < BSP_MAX_ASC_PER_LINE)
        {
            temp = *p_cur++;
            i++;
            if(('\r' == temp) || ('\n' == temp))
            {
                break;
            }
            if(' ' != temp)
            {
                pBuf[ulLineLen++] = temp;
            }
        }

        p_src[0] = pBuf[0];
        p_src[1] = pBuf[1];

        if(('W' == p_src[0]) || ('C' == p_src[0]))
        {
            for(i=2,j=0; i<ulLineLen; i+=2,j++)
            {
                memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
                memcpy(ucTmpBuf, pBuf+i, 2);
                p_src[2+j] = bq27510_strtoul(ucTmpBuf, 16);
            }

            temp = (ulLineLen -2)/2;
            ulLineLen = temp + 2;
        }
        else if('X' == p_src[0])
        {
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			if(ulLineLen <= 2)
				return -1;
            memcpy(ucTmpBuf, pBuf+2, ulLineLen-2);
            ulDelay = bq27510_atoi(ucTmpBuf);
        }
        else if('R' == p_src[0])
        {
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
            memcpy(ucTmpBuf, pBuf+2, 2);
            p_src[2] = bq27510_strtoul(ucTmpBuf, 16);
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
            memcpy(ucTmpBuf, pBuf+4, 2);
            p_src[3] = bq27510_strtoul(ucTmpBuf, 16);
            memset(ucTmpBuf, 0x00, sizeof(ucTmpBuf));
			if(ulLineLen <= 6)
				return -1;
            memcpy(ucTmpBuf, pBuf+6, ulLineLen-6);
            ulReadNum = bq27510_atoi(ucTmpBuf);
        }

        if(':' == p_src[1])
        {
            switch(p_src[0])
            {
            case 'W' :
                if(bq27510_i2c_bytes_write(client, p_src[3], &p_src[4], ulLineLen-4) < 0)
                {
                    printk(KERN_ERR "[%s,%d] bq27510_i2c_bytes_write failed len=%d\n",__FUNCTION__,__LINE__,ulLineLen-4);
                }
                break;
            case 'R' :
                if(bq27510_i2c_bytes_read(client, p_src[3], p_dst, ulReadNum) < 0)
                {
                    printk(KERN_ERR "[%s,%d] bq27510_i2c_bytes_read failed\n",__FUNCTION__,__LINE__);
                }
                break;
            case 'C' :
                if(bq27510_i2c_bytes_read_and_compare(client, p_src[3], p_dst, &p_src[4], ulLineLen-4))
                {
                    ulCounter++;
                    printk(KERN_ERR "[%s,%d] bq27510_i2c_bytes_read_and_compare failed\n",__FUNCTION__,__LINE__);
                    goto bq27510_firmware_program_begin;
                }
                break;
            case 'X' :
                msleep(ulDelay);
                break;
            default:
                return 0;
            }
        }

    }
    return 0;
}

static int bq27510_firmware_download(struct i2c_client *client, const unsigned char *pgm_data, unsigned int len)
{
    int iRet;

    gauge_context.state = BQ27510_UPDATE_FIRMWARE;

    /*Enter Rom Mode */
    iRet = bq27510_i2c_word_write(client, BSP_ENTER_ROM_MODE_CMD, BSP_ENTER_ROM_MODE_DATA);
    if(iRet)
    {
        pr_info("bq27510_i2c_word_write failed when update firmware\n");
    }
    msleep(10);

    /*change i2c addr*/
    g_battery_measure_by_bq27510_i2c_client->addr = BSP_ROM_MODE_I2C_ADDR;

    /*program bqfs*/
    iRet = bq27510_firmware_program(client, pgm_data, len);
    if(iRet)
    {
        pr_info("bq27510_firmware_program failed\n");

    }

    /*change i2c addr*/
    g_battery_measure_by_bq27510_i2c_client->addr = BSP_NORMAL_MODE_I2C_ADDR;

    gauge_context.locked_timeout_jiffies =  jiffies + msecs_to_jiffies(5000);
    gauge_context.state = BQ27510_LOCK_MODE;

    if (bq27510_i2c_word_write(client,BQ27510_REG_CTRL,0x0041))/* reset cmd*/
        pr_info("write reset failed\n");
    else
        pr_info("bq27510 download reset\n");

    return iRet;
}


int get_gas_version_id(char * id, char * name)
{
    char *end, *start;
    char *temp;
    int i = 0;
    start = strrchr(name, '_');
    end = strchr(name, '.');
    if (start == NULL || end == NULL || (start > end) )
        return 1;
    start++;
    temp = id;
    while ((i < (ID_LEN - 1)) && (start != end))
    {
        *temp++ = *start++;
        i++;
    }
    *temp = '\0';
    return 0;
}

static void bq27510_firmware_update(struct work_struct *work)
{
    int rc = 0;
    char fw_name[32] = {0};
    char version[ID_LEN] = {0};
    char chip_fw_version[ID_LEN] = {0};
    const struct firmware *fw = NULL;
    const char *product_name = NULL;
/* remove useless code */
    struct firmware_header *fw_header = NULL;
    struct bq27510_device_info *di = NULL;
    struct device_node *np = NULL;
	fw_update_ok = FW_UPD_PROCESSING;

    di = container_of(work, struct bq27510_device_info, update_work.work);

    np = di->client->dev.of_node;

    //get product id
    rc = of_property_read_string(np,"product-name",&product_name);
    if(rc)
    {
        pr_info("need product name\n");
        fw_update_ok = FW_UPD_FAIL;
        return;
    }
    pr_info("product name is %s\n",product_name);

    //get batt id
    di->battery_type = huawei_charger_batt_type();
    if(NULL == di->battery_type)
    {
        schedule_delayed_work(&di->update_work, msecs_to_jiffies(3000));
        return;
    }

    //battery firmware name
    snprintf(fw_name,sizeof fw_name,"%s-%s.fw",product_name, di->battery_type);
    pr_info("find battery firmware %s\n",fw_name);

    //call user space for firmware
    rc = request_firmware(&fw,fw_name,&di->client->dev);
    if(rc)
    {
        pr_err("request firmware failed \n");
        fw_update_ok = FW_UPD_FAIL;
        snprintf(fw_version_export,sizeof fw_version_export,"%x", \
                bq27510_get_firmware_version_by_i2c(di->client));
        return ;
    }

    fw_header = (struct firmware_header *)fw->data;

    pr_info("fw version of fs is %s\n",fw_header->file_name);

    if(get_gas_version_id(version,fw_header->file_name))
    {
        pr_info("failed to get firmware version\n");
        release_firmware(fw);
        fw_update_ok = FW_UPD_FAIL;
        return;
    }
    pr_info("fw version of bqfs is %s\n",version);

    snprintf(chip_fw_version,sizeof chip_fw_version,"%x", \
             bq27510_get_firmware_version_by_i2c(di->client));

    pr_info("fw version of chip is %s\n",chip_fw_version);

    if (!strncasecmp(version, chip_fw_version, ID_LEN) && !force_update_flag)
    {
        pr_info("no need update\n");
        release_firmware(fw);
        fw_update_ok = FW_UPD_OK;
        memcpy(fw_version_export ,chip_fw_version,ID_LEN);
        return;
    }
    else
    {
        bq27510_firmware_download(di->client,(unsigned char *)fw_header+fw_header->offset,fw_header->length);
    }
    snprintf(fw_version_export,sizeof fw_version_export,"%x", \
             bq27510_get_firmware_version_by_i2c(di->client));

    if(!strncasecmp(version, fw_version_export, ID_LEN))
    {
        fw_update_ok = FW_UPD_OK;
    }
    else
    {
        fw_update_ok = FW_UPD_FAIL;
    }
    pr_info("update success\n");
    release_firmware(fw);

    return;
}


static void bq27510_update_firmware_work(struct work_struct *work)
{
    gBq27510DownloadFirmwareFlag = BSP_FIRMWARE_DOWNLOAD_MODE;
    bq27510_firmware_update(work);
    gBq27510DownloadFirmwareFlag = BSP_NORMAL_MODE;
}


/*
 * Use BAT_LOW not BAT_GD. When battery capacity is below SOC1, BAT_LOW PIN will pull up and cause a
 * interrput, this is the interrput callback.
 */
static irqreturn_t bq27510_abnormal_status_interrupt(int irq, void *_di)
{
    struct bq27510_device_info *di = _di;

    schedule_delayed_work(&di->notifier_work, 0);

    return IRQ_HANDLED;
}

static void interrupt_notifier_work(struct work_struct *work)
{
    struct bq27510_device_info *di = container_of(work,
                                     struct bq27510_device_info, notifier_work.work);
    int capacity = 0;

    capacity =  bq27510_battery_capacity(di);
    pmu_log_info("battery is low: interrupt_notifier_work is invoked, capacity = %d\n", capacity);
    if(capacity > CUTOFF_LEVEL){
        pm_wakeup_event(&di->client->dev,100);
    }else{
        pm_stay_awake(&di->client->dev);
        battery_alarm_enabled = true;
    }

    power_supply_changed(&di->ti_bms_psy);
    return;
}

static int bq27510_dt_parse(struct device *dev, struct bq27510_device_info *di)
{
    int rc = 0;
    struct device_node *node = di->client->dev.of_node;

    di->client->irq = of_get_named_gpio(node, "qcom,batt-low-gpio", 0);
    if (di->client->irq < 0){
        pmu_log_err("batt-low-gpio is not available\n");
    }
    return rc;
}

#if !defined(PRODUCTION_ALE_KERNEL) && !defined(PRODUCTION_G760_KERNEL)
static void resume_charge_check(struct bq27510_device_info *di)
{
    union power_supply_propval ret = {0,};
    int curr_capacity = 0 ;

    if (!atomic_read(&battery_full_flag))
        return;

    curr_capacity = bq27510_battery_capacity(di);

    if(NULL == di->batt_psy)
    {
        di->batt_psy = power_supply_get_by_name("battery");
    }
    if (di->batt_psy) {
        if (curr_capacity <= 99) {
            ret.intval = true;
        }
        else {
            ret.intval = false;
        }
        di->batt_psy->set_property(di->batt_psy,
            POWER_SUPPLY_PROP_RESUME_CHARGING, &ret);
    }
    pmu_log_info("resume_charging_check is %d with real SoC %d\n", ret.intval, curr_capacity);
}
#endif

static int get_charge_status(struct bq27510_device_info *di)
{
#if defined(PRODUCTION_ALE_KERNEL) || defined(PRODUCTION_G760_KERNEL)
    return bq_device->charge_status;
#else
    union power_supply_propval val = {0,};
    if (!di->batt_psy)
        di->batt_psy = power_supply_get_by_name("battery");
    if (di->batt_psy)
        di->batt_psy->get_property(di->batt_psy,
                POWER_SUPPLY_PROP_STATUS, &val);
    return val.intval;
#endif
}

static void set_charge_status(struct bq27510_device_info *di, int status)
{
#if defined(PRODUCTION_ALE_KERNEL) || defined(PRODUCTION_G760_KERNEL)
    bq_device->charge_status = status;
#else
    union power_supply_propval val = {0,};

    if (!di->batt_psy)
        di->batt_psy = power_supply_get_by_name("battery");
    if (di->batt_psy) {
        val.intval = status;
        di->batt_psy->set_property(di->batt_psy,
                POWER_SUPPLY_PROP_STATUS, &val);
    }
    if (POWER_SUPPLY_STATUS_FULL == status)
        atomic_set(&battery_full_flag, 1);
#endif
}

static int battery_monitor_capacity_changed(struct bq27510_device_info *di)
{
    int curr_capacity = 0;
    int curr_temperature = 0;
    int bat_exist = 1;
    /* Remove the code: int battery_voltage = 0 */
    static int zero_level_count = 0;
    int voltage_now = 0;
    union power_supply_propval val = {0};
    int ramp_step = 0;
#if !defined(PRODUCTION_ALE_KERNEL) && !defined(PRODUCTION_G760_KERNEL)
    static int ramp_count = 0;
#endif

    if(di->batt_psy)
    {
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_PRESENT, &val);
        bat_exist = val.intval;
    }
    if (!bat_exist)
    {
        curr_capacity = NO_BATTERY_CAPACITY;
        curr_temperature = 0;
    }
    else
    {
        curr_capacity = bq27510_battery_capacity(di);
        /* If battery alarm is enabled, but we charge the battery again at 2%,*/
        /* we should release the pm lock to allow the phone to sleep when >=3% */
        if(battery_alarm_enabled && (FAKE_CUTOFF_LEVEL <= curr_capacity)){
            pm_relax(&di->client->dev);
            battery_alarm_enabled = false;
        }
        if(true == factory_flag  &&  0 == curr_capacity)
        {
            pmu_log_info("do not report zero in factory mode \n");
            curr_capacity = 1;
        }
        if ((bq_device != NULL) && (get_charge_status(di) == POWER_SUPPLY_STATUS_DISCHARGING))
        {
            /* Remove the code of get battery voltage and fake cutoff capacity*/
            if(curr_capacity <= CUTOFF_LEVEL)
            {
                curr_capacity = CUTOFF_LEVEL;
            }
        }
    }

    pmu_log_info("bat_exist value is %d,curr_capacity is %d,prev_capacity is %d\n",bat_exist,curr_capacity,di->prev_capacity);

    /* Debouncing of power on init. */
    if (di->capacity == -1)
    {
        di->capacity = curr_capacity;
        di->prev_capacity = curr_capacity;
        di->capacity_debounce_count = 0;

        return 1;
    }

    /*change monitoring interval from 30s to 5s when capacity less than 10%*/
    if(curr_capacity < REACH_EMPTY_RESAMPLE_THRESHOLD)
    {
        di->monitoring_interval = REACH_EMPTY_SAMPLE_INTERVAL;
    }
    else
    {
        di->monitoring_interval = DEFALUT_MONITOR_TIME;
    }

#if defined(PRODUCTION_ALE_KERNEL) || defined(PRODUCTION_G760_KERNEL)
    if (is_usb_chg_exist() == CHARGER_ONLINE && di->capacity == CAPACITY_FULL && bq_device)
        set_charge_status(di, POWER_SUPPLY_STATUS_FULL);
#else
    if((di->capacity == CAPACITY_FULL) && (curr_capacity == CAPACITY_FULL) &&
            (is_usb_chg_exist() == CHARGER_ONLINE))
    {
        set_charge_status(di, POWER_SUPPLY_STATUS_FULL);
    }

    if (atomic_read(&battery_full_flag) && (curr_capacity > CAPACITY_RAMP_DOWN))
    {
        curr_capacity = CAPACITY_FULL;
    }
    else
    {
        atomic_set(&battery_full_flag, 0);
    }
#endif

    if(bq_device && (get_charge_status(di) == POWER_SUPPLY_STATUS_CHARGING) && (di->capacity > CAPACITY_NEAR_FULL))
    {
        di->charge_full_count++;
        if(bq_device->charge_full_count >= CHARGE_FULL_TIME)
        {
            di->charge_full_count = CHARGE_FULL_TIME;
        }
    }
    else if(bq_device && (get_charge_status(di) == POWER_SUPPLY_STATUS_FULL))
    {
        di->charge_full_count = CHARGE_FULL_TIME;
    }
    else
    {
        di->charge_full_count = 0;
    }

    if ((bq_device != NULL) && (get_charge_status(di) == POWER_SUPPLY_STATUS_CHARGING)
        && (ZERO_LEVEL == curr_capacity)){
        voltage_now = bq27510_battery_voltage(di);
        if(ZERO_LEVEL_VOLTAGE <= voltage_now){
            /* if capacity read from soc reg is 0% when charging and voltage is over 3350mV */
            /* modify the capacity as 1% */
            curr_capacity = 1;
            zero_level_count = 0;
            pmu_log_info("curr_capacity = %d voltage_now = %d\n", curr_capacity, voltage_now);
        }else{
            if(ZERO_LEVEL_COUNT > zero_level_count++){
                /* if zero_level_count is less than 10 times */
                /* modify the capacity as 1% */
                pmu_log_info("zero_level_count is %d\n", zero_level_count);
                curr_capacity = 1;
            }else{
                curr_capacity = ZERO_LEVEL;
                zero_level_count = 0;
            }
       }
    }else{
        zero_level_count = 0;
    }
    /*Only availability if the capacity changed*/
    if (curr_capacity != di->prev_capacity)
    {
        if (abs(di->prev_capacity -curr_capacity) >= CAPACITY_JUMP_THRESHOLD)
        {
            dev_info(di->dev,"prev_capacity = %d \n"
                     "curr_capacity = %d \n" "curr_voltage = %d \n",
                     di->prev_capacity, curr_capacity,bq27510_battery_voltage(di));
        }
        /* capacity should not increase during discharging*/
        if(bq_device && (get_charge_status(di) == POWER_SUPPLY_STATUS_DISCHARGING)
                && (di->prev_capacity < curr_capacity))
        {
            dev_info(di->dev,"capacity increase during discharge: prev_capacity = %d \n"
                     "curr_capacity = %d \n" "curr_voltage = %d \n",
                     di->prev_capacity, curr_capacity,bq27510_battery_voltage(di));
            return 0;
        }
        if(!di->last_soc_unbound)
        {
            ramp_step = MAX_SOC_CHANGE;
#if !defined(PRODUCTION_ALE_KERNEL) && !defined(PRODUCTION_G760_KERNEL)
            if(abs(curr_capacity - di->prev_capacity) > MAX_SOC_CHANGE)
            {
                if(++ramp_count < 2)
                {
                    ramp_step = 0;
                }
                else
                {
                    ramp_count = 0;
                }
            }
            else
            {
                ramp_count = 0;
            }
#endif
            if(curr_capacity > di->prev_capacity)
            {
                curr_capacity = di->prev_capacity + ramp_step;
            }
            else if(curr_capacity < di->prev_capacity && curr_capacity != 0)
            {
                curr_capacity = di->prev_capacity - ramp_step;
            }
            else
            {
                dev_info(di->dev,"shutdown capacity: prev_capacity = %d \n"
                         "curr_capacity = %d \n" "curr_voltage = %d \n",
                         di->prev_capacity, curr_capacity,bq27510_battery_voltage(di));
            }
        }
        else
        {
            di->last_soc_unbound = false;
        }
        di->prev_capacity = curr_capacity;
        di->capacity_debounce_count = 0;
    }
    else if (++di->capacity_debounce_count >= CAPACITY_DEBOUNCE_MAX)
    {
        if (bq_device && is_usb_chg_exist() == CHARGER_ONLINE && (di->charge_full_count >= CHARGE_FULL_TIME))
        {
            curr_capacity = CAPACITY_FULL;
            set_charge_status(di, POWER_SUPPLY_STATUS_FULL);
        }

        di->capacity = curr_capacity;
        di->capacity_debounce_count = 0;
        if(bq_device && (get_charge_status(di) == POWER_SUPPLY_STATUS_FULL))
        {
            di->capacity = CAPACITY_FULL;
        }
        return 1;
    }
    else
    {
        return 0;
    }

    di->prev_capacity = curr_capacity;
    di->capacity = curr_capacity;

    return 1;
}

static void battery_monitor_work(struct work_struct *work)
{
    struct bq27510_device_info *di = container_of(work,
                                     struct bq27510_device_info, battery_monitor_work.work);
    int current_temp = 0;

#if !defined(PRODUCTION_ALE_KERNEL) && !defined(PRODUCTION_G760_KERNEL)
    if (CHARGER_ONLINE != is_usb_chg_exist())
        atomic_set(&battery_full_flag, 0);
#endif

    current_temp = bq27510_battery_temperature(di);

    if (battery_monitor_capacity_changed(di) || ((di->prev_temp != current_temp) && (current_temp >= HIGH_TEMP)))
    {
        /*the capacity changed and updata the info of the battery*/
        power_supply_changed(&di->ti_bms_psy);

    }

#if !defined(PRODUCTION_ALE_KERNEL) && !defined(PRODUCTION_G760_KERNEL)
    resume_charge_check(di);
#endif

    di->prev_temp = current_temp;

    /* schedule the soc calculating work more frequently when other related module doesn't init well */
    /* Delete qcom_charger_psy */
    if (!di->batt_psy || !bq_device) {
            di->monitoring_interval = BOOT_MONITOR_TIME;
    }
    schedule_delayed_work(&di->battery_monitor_work,
                          msecs_to_jiffies(1000 * di->monitoring_interval));
}

static enum power_supply_property ti_bms_power_props[] =
    {
        POWER_SUPPLY_PROP_CAPACITY,
        POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    };

static int ti_bms_power_get_property(struct power_supply *psy,
                                     enum power_supply_property psp,
                                     union power_supply_propval *val)
{
    switch (psp)
    {
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = g_battery_measure_by_bq27510_device->capacity;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = bq27510_battery_temperature(g_battery_measure_by_bq27510_device) * 10;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = 1000*bq27510_battery_voltage(g_battery_measure_by_bq27510_device);
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = -1000*bq27510_battery_current(g_battery_measure_by_bq27510_device);
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = bq27510_battery_health(g_battery_measure_by_bq27510_device);
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = bq27510_get_gasgauge_qmax(g_battery_measure_by_bq27510_device)*1000;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static void ti_bms_external_power_changed(struct power_supply *psy)
{
    g_battery_measure_by_bq27510_device->batt_psy = power_supply_get_by_name("battery");
    if(g_battery_measure_by_bq27510_device->batt_psy)
    {
#if defined(PRODUCTION_ALE_KERNEL) || defined(PRODUCTION_G760_KERNEL)
        bq_device = container_of(g_battery_measure_by_bq27510_device->batt_psy, struct bq2415x_device, charger);
#else
        bq_device = container_of(g_battery_measure_by_bq27510_device->batt_psy, struct bq24296m_device_info, charger);
        if (CHARGER_ONLINE != is_usb_chg_exist())
            atomic_set(&battery_full_flag, 0);
#endif
    }
}

static char *bq27510_supplied_to[] =
{
    "battery",
};

static ssize_t bq27510_show_gaugelog(struct device_driver *driver, char *buf)
{
    int temp = 0, voltage = 0, capacity = 100, rm = 0, fcc = 0;
#ifndef PRODUCTION_G760_KERNEL
	int ufrm = 0,frm = 0,uffcc = 0,ffcc = 0,ufsoc = 100;
#endif
    int cur = 0,cyc = 0,si = 0;
    u16 flag = 0,control_status = 0;
    int qmax = 0;

    struct bq27510_device_info* di = g_battery_measure_by_bq27510_device;

    if(NULL == buf)
    {
        return -1;
    }
    if(!bq27510_is_accessible())
        return snprintf(buf, PAGE_SIZE, "bq27510 is busy because of updating(%d)",gauge_context.state);
    if(BSP_NORMAL_MODE != gBq27510DownloadFirmwareFlag)
    {
        return -1;
    }

    temp =  bq27510_battery_temperature(di);
    mdelay(2);
    voltage = bq27510_battery_voltage(di);
    mdelay(2);
    cur = (-1)*bq27510_battery_current(di);
    mdelay(2);
    capacity = bq27510_battery_capacity(di);
    mdelay(2);
    flag = is_bq27510_battery_reach_threshold(di);
    mdelay(2);
    rm =  bq27510_battery_rm(di);
    mdelay(2);
    fcc =  bq27510_battery_fcc(di);
    mdelay(2);
    cyc = bq27510_i2c_read_word(di,BQ27510_REG_CYC);
    mdelay(2);
    si = bq27510_i2c_read_word(di,BQ27510_REG_SI);

    mutex_lock(&bq27510_battery_mutex);
    i2c_smbus_write_word_data(di->client,BQ27510_REG_CTRL,BQ27510_REG_CTRS);
    mdelay(2);
    control_status  = i2c_smbus_read_word_data(di->client,BQ27510_REG_CTRL);
    mdelay(2);
    mutex_unlock(&bq27510_battery_mutex);
    mdelay(2);
    qmax =  bq27510_get_gasgauge_qmax(di);
    mdelay(2);
#ifndef PRODUCTION_G760_KERNEL
    ufrm = bq27510_battery_ufrm(di);
    mdelay(2);
    frm = bq27510_battery_frm(di);
    mdelay(2);
    uffcc = bq27510_battery_uffcc(di);
    mdelay(2);
    ffcc = bq27510_battery_ffcc(di);
    mdelay(2);
    ufsoc = bq27510_battery_ufsoc(di);
    mdelay(2);
#endif
    if(qmax < 0)
    {
        return snprintf(buf, PAGE_SIZE, "%s", "Coulometer Damaged or Firmware Error \n");
    }
#ifndef PRODUCTION_G760_KERNEL
    else
    {
        snprintf(buf, PAGE_SIZE, "%-9d  %-9d  %-4d  %-5d  %-6d  %-6d  %-6d  %-6d  0x%-5.4x  0x%-5.4x  %-6d  %-5d  %-5d  %-6d  %-6d  %-4d  ",
                voltage,  (signed short)cur, capacity, rm, fcc, (signed short)cyc, (signed short)si, temp, flag, control_status, qmax, ufrm, frm, uffcc, ffcc, ufsoc);
    }
#else
	else
	{
        snprintf(buf, PAGE_SIZE, "%-9d  %-9d  %-4d  %-5d  %-6d  %-6d  %-6d  %-6d  0x%-5.4x  0x%-5.4x  %-6d  ",
                voltage,  (signed short)cur, capacity, rm, fcc, (signed short)cyc, (signed short)si, temp, flag, control_status, qmax);
	}
#endif
    return strlen(buf);
}

static DRIVER_ATTR(gaugelog, S_IRUGO, bq27510_show_gaugelog,NULL);

static ssize_t force_update_trigger(struct device_driver *driver,const char *buf, size_t count)
{
    force_update_flag = true;
    schedule_delayed_work(&g_battery_measure_by_bq27510_device->update_work, msecs_to_jiffies(1000));
    return count;
}

static DRIVER_ATTR(force_update_trigger, S_IWUSR | S_IWGRP, NULL,force_update_trigger);


static ssize_t remaining_capacity_show(struct device_driver *driver, char *buf)
{
	int rm = 0;

	if(!g_battery_measure_by_bq27510_device)
		return 0;
	
    rm =  bq27510_battery_rm(g_battery_measure_by_bq27510_device);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", rm);
}

static DRIVER_ATTR(remaining_capacity, S_IRUGO, remaining_capacity_show, NULL);
static ssize_t firmware_version_show(struct device_driver *driver, char *buf)
{
    return snprintf(buf, ID_LEN, "%s\n", fw_version_export);
}

static DRIVER_ATTR(firmware_version, S_IRUGO, firmware_version_show, NULL);

static ssize_t firmware_update_success_show(struct device_driver *driver, char *buf)
{
    return snprintf(buf, ID_LEN, "%d\n", fw_update_ok);
}

static DRIVER_ATTR(firmware_update_success, S_IRUGO, firmware_update_success_show, NULL);
static struct attribute *bq27510_sysfs_attributes[] =
{
    &driver_attr_gaugelog.attr,
    &driver_attr_force_update_trigger.attr,
    &driver_attr_remaining_capacity.attr,
	&driver_attr_firmware_version.attr,
	&driver_attr_firmware_update_success.attr,
    NULL,
};

static umode_t ti_bms_prop_is_visible(struct kobject *a,struct attribute *b, int i)
{
    if(use_ti_coulometer)
    {
        return b->mode;
    }
	else
		return 0;
}

static const struct attribute_group bq27510_sysfs_attr_group =
{
    .name = "ti-bms-prop",
    .attrs = bq27510_sysfs_attributes,
    .is_visible = ti_bms_prop_is_visible
};

static const struct attribute_group *driver_groups[] =
{
    &bq27510_sysfs_attr_group,
    NULL
};


static int bq27510_battery_probe(struct i2c_client *client,
                                 const struct i2c_device_id *id)
{
    int retval = 0;
    struct bq27510_device_info *di = NULL;

    i2c_smbus_write_word_data(client,0x00,0x0008);
    msleep(2);
    retval = i2c_smbus_read_word_data(client,0x00);
    if(retval < 0)
    {
        pr_info("coulometer damaged or firmware error\n");
		//if i2c read word fail,we will not prob TI.
        //if i2c read fail go on TI probe,otherwith there wil be NULL pointer problem
		//return -ENODEV;
    }
    else
    {
        pr_info("normal mode. firmware version=%04x\n", retval);
    }

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di)
    {
        pr_info("failed to allocate device info data\n");
        return -ENOMEM;
    }

    di->id = 0;
    i2c_set_clientdata(client, di);
    di->dev = &client->dev;
    di->client = client;

    di->timeout_jiffies = 0;//jiffies + msecs_to_jiffies(DISABLE_ACCESS_TIME);// Maybe lost interrupts in 2 seconds after power on,Zheng
    di->prev_capacity = 0;
    di->prev_temp = 0;
    di->capacity_debounce_count = 0;
    di->monitoring_interval = DEFALUT_MONITOR_TIME;
    di->capacity = -1;
    di->charge_full_count = 0;
    di->last_soc_unbound = true;
    use_ti_coulometer = of_property_read_bool(di->client->dev.of_node, "ti,use-ti-coulometer");
    use_filter_mode = of_property_read_bool(di->client->dev.of_node, "ti,use-ti-filter-mode");
    pmu_log_info("use_ti_coulometer=%d,use_filter_mode=%d\n",use_ti_coulometer,use_filter_mode);
    if(!use_filter_mode)
      di->soc_reg = BQ27510_REG_SOC;
    else
      di->soc_reg = BQ27510_REG_SOC_FILTER;

    device_init_wakeup(&di->client->dev,1);

    INIT_DELAYED_WORK(&di->notifier_work,interrupt_notifier_work);
    INIT_DELAYED_WORK(&di->update_work,bq27510_update_firmware_work);
    INIT_DELAYED_WORK(&di->battery_monitor_work,battery_monitor_work);

    bq27510_dt_parse(&client->dev, di);

    if (client->irq > 0)
    {
        retval = gpio_request(client->irq, "battary-alarm");
        if(retval < 0)
        {
            pr_info("battary-alarm gpio request failed\n");
            goto failed_1;
        }
        else
        {
            gpio_direction_input(client->irq);
        }

        /* request battery_low interruption */
        retval = request_irq(gpio_to_irq(client->irq), bq27510_abnormal_status_interrupt, IRQF_TRIGGER_FALLING,
                             "battary-alarm", di);
        if (retval)
        {
            pr_err("could not request irq %d, status %d\n", gpio_to_irq(client->irq), retval);
            goto failed_2;
        }
        else
        {
            enable_irq_wake(gpio_to_irq(client->irq));
        }
    }

	//remove redundant code
    g_battery_measure_by_bq27510_i2c_client = client;
    g_battery_measure_by_bq27510_device = di;


#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    set_hw_dev_flag(DEV_I2C_BATTERY);
#endif

    di->ti_bms_psy.name = "ti-bms";
    di->ti_bms_psy.type = POWER_SUPPLY_TYPE_BMS;
    di->ti_bms_psy.properties = ti_bms_power_props;
    di->ti_bms_psy.num_properties = ARRAY_SIZE(ti_bms_power_props);
    di->ti_bms_psy.get_property = ti_bms_power_get_property;
    di->ti_bms_psy.supplied_to = bq27510_supplied_to;
    di->ti_bms_psy.num_supplicants =ARRAY_SIZE(bq27510_supplied_to);
    //di->ti_bms_psy.set_property = ti_bms_power_set_property;
    //di->ti_bms_psy.property_is_writeable = ti_bms_property_is_writeable;
    di->ti_bms_psy.external_power_changed = ti_bms_external_power_changed;

    retval= power_supply_register(di->dev, &di->ti_bms_psy);

    if (retval < 0)
    {
        pr_err("ti bms psy failed to register rc=%d\n", retval);
        if(client->irq)
            goto failed_3;
        else
            goto failed_1;
    }
    //request firmware and run update task.
    schedule_delayed_work(&di->update_work, 0);
    //start to calculate capacity 5 sencond later,for battery exist only can only dectected by pmic
    schedule_delayed_work(&di->battery_monitor_work,msecs_to_jiffies(START_CAPACITY_CACL));
    pmu_log_info("bq27510 probe ok\n");
    return retval;

failed_3:
    free_irq(gpio_to_irq(di->client->irq), di);
failed_2:
    gpio_free(client->irq);
failed_1:
    device_wakeup_disable(&di->client->dev);
    kfree(di);
    di = NULL;

    return retval;

}
static int bq27510_battery_suspend(struct i2c_client *client,
                                   pm_message_t state)
{
    struct bq27510_device_info *di = i2c_get_clientdata(client);
    cancel_delayed_work_sync(&di->battery_monitor_work);
    /*soc changes by step of 1% in battery_monitor_work after resume */
    /*so remove the code: di->last_soc_unbound = true; */
    return 0;
}

static int bq27510_battery_resume(struct i2c_client *client)
{
    struct bq27510_device_info *di = i2c_get_clientdata(client);
    schedule_delayed_work(&di->battery_monitor_work, 0);
    return 0;
}

static struct of_device_id bq27510_battery_match_table[] =
{
    {.compatible = "ti,bq27510"}
};

static const struct i2c_device_id bq27510_id[] =
{
    {"ti,bq27510",0},
    {},
};

static struct i2c_driver bq27510_battery_driver =
{
    .driver = {
        .name = "ti,bq27510",
        .of_match_table = bq27510_battery_match_table,
        .groups = driver_groups
    },
    .probe = bq27510_battery_probe,
    .suspend = bq27510_battery_suspend,
    .resume = bq27510_battery_resume,
    .id_table = bq27510_id,
};

module_i2c_driver(bq27510_battery_driver);
static void __exit ti_bms_exit(void)
{
    i2c_del_driver(&bq27510_battery_driver);
}
module_exit(ti_bms_exit);
MODULE_AUTHOR("HUAWEI");
MODULE_DESCRIPTION("BQ27510 battery monitor driver");
MODULE_LICENSE("GPL");
