/*
 * Copyright 2010 HUAWEI Tech. Co., Ltd.
 */
#define I2C_ADDR_BQ27510        (0x55)

#define DRIVER_VERSION          "1.0.0"

#define BQ27510_REG_TEMP    (0x06)
#define BQ27510_REG_VOLT    (0x08)
#define BQ27510_REG_FLAGS   (0x0a)
/*Time to Empty*/
#define BQ27510_REG_TTE     (0x16)
/*Time to Full*/
#define BQ27510_REG_TTF     (0x18)
/*Cycle Count*/
#define BQ27510_REG_CYC     (0x1e)
/* State-of-Charge */
//register has been changed in new fw version
#define BQ27510_REG_SOC_FILTER     (0x20)
#define BQ27510_REG_SOC     (0x2c)
/*Average Current*/
#define BQ27510_REG_AI      (0x14)
/*Remainning Capacity*/
#define BQ27510_REG_RM      (0x10)
/*Full Charge Capacity*/
#define BQ27510_REG_FCC     (0x12)
/*Standby Current*/
#define BQ27510_REG_SI      (0x18)
/*DesignCapacity*/
#ifdef CONFIG_HUAWEI_DSM
#define BQ27510_DESIGN_CAPACITY		(0x2e)
#endif
/*Control*/
#define BQ27510_REG_CTRL    (0x00)
/*Control Status*/
#define BQ27510_REG_CTRS    (0x0000)
/*Data Flash Class*/
#define BQ27510_REG_DFCLS      (0x3e)
#define BQ27510_REG_CLASS_ID     (82)
#define BQ27510_REG_QMAX       (0x42)
#define BQ27510_REG_QMAX1      (0x43)
#define BQ27510_REG_FLASH             (0x40)
#define BQ27510_REG_FIRMWARE_ID      (0x0008)
#define BQ27510_REG_FIRMWARE_VERSION (0x0039)
#define BQ27510_REG_UFRM          (0x6c)
#define BQ27510_REG_FRM           (0x6e)
#define BQ27510_REG_UFFCC         (0x70)
#define BQ27510_REG_FFCC          (0x72)
#define BQ27510_REG_UFSOC         (0x74)

/* both words and bytes are LSB*/
/* Full-charged bit */
#define BQ27510_FLAG_FC               (1<<9)
#define BQ27510_FLAG_DET              (1<<3)
/* Over-Temperature-Charge bit */
#define BQ27510_FLAG_OTC              (1<<15)
/* Over-Temperature-Discharge bit */
#define BQ27510_FLAG_OTD              (1<<14)
/* State-of-Charge-Threshold 1 bit */
#define BQ27510_FLAG_SOC1             (1<<2)
/* State-of-Charge-Threshold Final bit */
#define BQ27510_FLAG_SOCF             (1<<1)
#define BQ27510_FLAG_LOCK       (BQ27510_FLAG_SOC1 | BQ27510_FLAG_SOCF)
/* Discharging detected bit */
#define BQ27510_FLAG_DSG              (1<<0)

#define CONST_NUM_10                   (10)
#define CONST_NUM_2730               (2730)

#define BSP_ROM_MODE_I2C_ADDR               (0x0B)
#define BSP_NORMAL_MODE_I2C_ADDR            (0x55)
#define BSP_FIRMWARE_FILE_SIZE              (400*1024)
#define BSP_I2C_MAX_TRANSFER_LEN            (128)
#define BSP_MAX_ASC_PER_LINE                (400)
#define BSP_ENTER_ROM_MODE_CMD              (0x00)
#define BSP_ENTER_ROM_MODE_DATA             (0x0F00)
#define BSP_FIRMWARE_DOWNLOAD_MODE          (0xDDDDDDDD)
#define BSP_NORMAL_MODE                     (0x00)

#define DISABLE_ACCESS_TIME                 (2000)

#define DEFALUT_MONITOR_TIME	30
#define NO_BATTERY_CAPACITY	40
#define CHARGE_FULL_TIME		80
#define START_CAPACITY_CACL 	0
#define REACH_EMPTY_RESAMPLE_THRESHOLD  (10)
#define REACH_EMPTY_SAMPLE_INTERVAL     (5)
#define BOOT_MONITOR_TIME	2
#define CHARGER_ONLINE 			1
#define CAPACITY_FULL 			100
#define CAPACITY_CLEAR_FULL 	98
#define CAPACITY_NEAR_FULL 	96
#define CAPACITY_JUMP_THRESHOLD 	5
#define CAPACITY_DEBOUNCE_MAX	4
#define CUTOFF_LEVEL	2
#define FAKE_CUTOFF_LEVEL	3
#define CAPACITY_RAMP_DOWN    97
/* Remove 3450mV cutoff voltage */
#define ZERO_LEVEL    0
#define ZERO_LEVEL_COUNT    10
#define MAX_SOC_CHANGE	1
#define ZERO_LEVEL_VOLTAGE        3350

struct bq27510_device_info
{
    struct device  *dev;
    int             id;
    const char*    battery_type;
    struct power_supply    ti_bms_psy;
    struct power_supply    *batt_psy;
    struct i2c_client      *client;
    struct delayed_work     notifier_work;
    struct delayed_work     update_work;
    struct delayed_work     battery_monitor_work;
    unsigned long           timeout_jiffies;
    const char *config_name;
    int prev_capacity;
    int prev_temp;
    int capacity;
    int capacity_debounce_count;
    int charge_full_count;
    int monitoring_interval;
    int last_soc_unbound;
#ifdef CONFIG_HUAWEI_DSM
    int saved_soc;
#endif
    u8 soc_reg;
};

extern struct bq27510_device_info* g_battery_measure_by_bq27510_device;
extern int is_bq27510_battery_full(struct bq27510_device_info *di);
extern int is_usb_chg_exist(void);
extern int bq27510_battery_capacity(struct bq27510_device_info *di);
extern short bq27510_battery_current(struct bq27510_device_info *di);
extern char *huawei_charger_batt_type(void);
