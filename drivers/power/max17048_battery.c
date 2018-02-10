/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/irqreturn.h>
#include <linux/of_gpio.h>
#include <linux/of_batterydata.h>
#include <linux/mfd/max77819.h>
#include <linux/interrupt.h>

/*******************************************************************************/
/* Debugging log maco defines */
#define log_level 0

#undef pr_err
#undef pr_info
#undef pr_debug
#undef  log_fmt

#define pr_err								log_err
#define pr_info								log_info
#define pr_debug							log_dbg
#define pr_vdebug							log_vdbg
#define log_fmt(format)						"Maxim:%s:%d: " format, __func__, __LINE__
/******************************************************************************/

/* registor address */
#define MAX17048_VCELL_REG					0x02
#define MAX17048_VCELL_LSB					0x03
#define MAX17048_SOC_REG					0x04
#define MAX17048_SOC_LSB					0x05
#define MAX17048_MODE_REG					0x06
#define MAX17048_MODE_LSB					0x07
#define MAX17048_VER_REG					0x08
#define MAX17048_VER_LSB					0x09
#define MAX17048_HIBRT_REG					0x0A
#define MAX17048_RCOMP_REG					0x0C
#define MAX17048_RCOMP_LSB					0x0D
#define MAX17048_CMD_REG					0xFE
#define MAX17048_CMD_LSB					0xFF
#define MAX17048_OCV_REG					0x0E
#define MAX17048_VRESET_REG					0x18
#define MAX17048_STATUS_REG					0x1A
#define MAX17048_MODEL_ACCESS_REG			0x3E
#define MAX17048_MODEL_DATA_REG_ADRR		0x40
#define MAX17048_MODEL_RCOMSEG_ADDR			0x80

#define MAX17048_MODEL_ACCESS_UNLOCK		0x4A57
#define MAX17048_MODEL_ACCESS_LOCK			0x0
#define MAX17048_MODEL_DATA_SIZE			64 		/*MAX17048_MODEL_DATA_REG_ADRR bytes (0x40 ~ 0x7f)*/
#define MAX17048_MODEL_RCOMSEG_SIZE			0x20 	/* MAX17048_MODEL_RCOMSEG_ADDR bytes (0x80 ~ 0x9f)*/
#define MAX17048_IC_STATUS_POR				0x0100	/* MAX17048_STATUS_REG por bit */
#define MAX17048_RCOMP_TEMP_CONST			200		/* 20 du */
#define MAX17048_RCOMP_TEMP_FACTORIAL		10
#define MAX17048_REG_INVALID_VALUE			0xFF
#define MAX17048_SOC_FULL					100
#define MAX17048_ALERT_SOC_LOWEST			1
#define MAX17048_ALERT_SOC_HIGHEST			32
#define MAX17048_VRESET_LOWEST_MV			2280	/* Soc reset, VRESET register value */
#define MAX17048_VRESET_HIGHEST_MV			3480
#define MAX17048_VRESET_STEP_MV				40		/* mv */
#define MAX17048_BATT_UV_PER_CELL			625/8	/* 78125/1000  = 625/8 * uv */

#define MAX17048_TIME_WORK_DELAY			msecs_to_jiffies(15000)
#define MAX17048_HANDLE_MODEL_DELAY			msecs_to_jiffies(1000*3600) /* one hour */
#define MAX17048_SOC_ALRM_IRQ_NAME			"battery-alarm"
#define QCOM_POWR_SUPPLY_CAHRGER_NAME		"battery"

/* max times to retry to load model data if it is failed */
#define MAX17048_LOADMODLE_MAXTIMES			3
#define MAX17048_RCOMP_FACTORIAL			10000
#define MAX17048_VERIFY_FIX_MODEL				1
#define MAX17048_LOAD_MODEL					(!(MAX17048_VERIFY_FIX_MODEL))

#define MAX17048_CALC_SOC(socReg, bitType)\
	(((bitType) == 18) ? ((socReg) / 256) : ((socReg)/512))

/* VRESET regadrr = MAX17048_VRESET_REG */
/* VRESET regvalue use bit [7:1], step mv = MAX17048_VRESET_STEP_MV*/
#define MAX17048_VRESET_MV_TO_REGVAL(mv)\
	((((mv) /MAX17048_VRESET_STEP_MV) << 1))
	
#define MAX17048_VRESET_REGVAL_TO_MV(reg)\
	(((reg) >> 1) * MAX17048_VRESET_STEP_MV)

#define MAX17048_VBATT_REGVAL_TO_UV(reg)\
	(((u32)(reg)) * MAX17048_BATT_UV_PER_CELL)
	
#define MAX17048_RIGHT_VERSION(ver)\
	(((u16)(ver) == 0x0011) || ((u16)(ver) == 0x0012))

#define MAXIM_CURRENT_NOW_UV_TO_UA(vichguv)\
	((int)(vichguv) * 100 / 141)

/* the vichg real value is 0 when the adc read value is less than 50mv */
#define VADC_VICHG_ZERO_CHECK(vichguv)\
	((vichguv) = ((vichguv) <= 50000) ? 0 : (vichguv))

/* huawei system param define */
#define HW_VOLTAGE_FACTORIAL					1000
#define HW_ALERT_LOW_SOC						5
#define HW_SYS_POWRUP_TIME					30		/* seconds */
#define HW_AVRG_BATT_MV_COUNT				3
#define HW_PROTECT_BATT_BAD_MV 				3300	/* mv */
#define HW_PROTECT_BATT_BAD_CNT				10		/* continue times */
#define HW_CUTOFF_BATT_SOC					2
#define HW_CUTOFF_BATT_MV						3450	/* mv */
#define HW_CUTOFF_BATT_CNT					10

#define HIGH_WORD_VALUE(val)					((u8)((((u16)(val))&(0xFF00))>>8))
#define LOW_WORD_VALUE(val)					((u8)(((u16)(val))&(0x00FF)))
#define TWO_U8_TO_U16(high, low)				(((((u16)(high)) << 8)&(0xFF00))|((u16)(low)))

#define __lock(_chip)    mutex_lock(&(_chip)->lock)
#define __unlock(_chip)  mutex_unlock(&(_chip)->lock)
#define __lock_register(_chip)			max17048_write_reg((_chip)->client, MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_LOCK);
#define __unlock_register(_chip)		max17048_write_reg((_chip)->client, MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_UNLOCK);

#define loop_schedule_delayed_work(ptr_work, delaytime) \
		if (likely(!delayed_work_pending(ptr_work))){\
			schedule_delayed_work(ptr_work, delaytime);\
		}

static int max17048_write_reg(struct i2c_client *client, u8 reg, u16 value);
static u16 max17048_read_reg(struct i2c_client *client, u8 reg);

extern bool get_max77819_charger_present(void);

struct max17048_platform_data{
	unsigned int			irq_gpio;
	u8						alert_soc;
	int						reset_mvlolt;
	u16						cutoff_batt_mv;
	u8						cutoff_batt_soc;
	u16						bad_batt_mv;
	struct max17048_batt_data*		pbat_data;
};

struct max17048_chip {
	struct mutex						lock;
	struct i2c_client*					client;
	struct delayed_work					work;
	struct delayed_work					hand_work;
	struct delayed_work					notifier_work;

	struct power_supply					fgbattery;
	struct power_supply*				psy_qcom_charger;

	struct qpnp_vadc_chip*				vadc_dev;

	struct max17048_platform_data*		pdata;

	bool	bbatt_alrm;
	int		irq;
	/* battery voltage */
	int		vcell;
	/* battery capacity */
	u8		soc;
};

static struct max17048_chip* global_chip = NULL;
/* check whether the system boots up by factory mode */
static bool factory_flag = false;

static struct max17048_batt_data gbat_ini_default = {
	.full_capacity = 3000000,	/* uAh */
	.ini_rcompseg = 0x0080,
	.ini_rcomp = 97,
	.ini_tempco_up = -1000,		/* -0.1 * 10000 */
	.ini_tempco_dwon = 74750,	/* 7.475 * 10000 */
	.ini_soccheck_a = 226,
	.ini_soccheck_b = 228,
	.ini_ocvtest = 57952,
	.ini_bits = 19,
	.model_data = {
		0xA9, 0xD0, 0xB7, 0x20, 0xB8, 0x90, 0xBA, 0xB0,
		0xBB, 0xD0, 0xBC, 0xE0, 0xBE, 0x00, 0xBF, 0x00,
		0xC0, 0x40, 0xC2, 0x30, 0xC4, 0xA0, 0xC6, 0x60,
		0xCA, 0x00, 0xCD, 0x70, 0xD2, 0x20, 0xD8, 0x60,
		0x02, 0x80, 0x26, 0x60, 0x1B, 0xB0, 0x27, 0xE0,
		0x3C, 0x80, 0x3B, 0xF0, 0x2B, 0xC0, 0x24, 0xD0,
		0x10, 0xE0, 0x16, 0xD0, 0x14, 0x30, 0x11, 0x80,
		0x10, 0x10, 0x0F, 0xB0, 0x0B, 0x00, 0x0B, 0x00,
	},
};

static struct max17048_platform_data gplatform_data_default = {
	.irq_gpio = 109,
	.alert_soc = HW_ALERT_LOW_SOC,
	.reset_mvlolt = 2800, /* 2.8v */
	.cutoff_batt_mv = HW_CUTOFF_BATT_MV,
	.cutoff_batt_soc = HW_CUTOFF_BATT_SOC,
	.bad_batt_mv = HW_PROTECT_BATT_BAD_MV,
	.pbat_data = &gbat_ini_default,
};

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int __init early_parse_factory_flag(char* p)
{
	if(p && !strcmp(p,"factory"))
	{
		factory_flag = true;
	}
	return 0;
}
early_param("androidboot.huawei_swtype",early_parse_factory_flag);

/* Use another mothed to reduce the power consumption of FTM mode. */

static int max17048_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		pr_err("reg %d err %d\n", reg, ret);

	return ret;
}

static u16 max17048_read_reg(struct i2c_client *client, u8 reg)
{
	u16 ret;

	ret = (u16)i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
		pr_err("reg %d err %d\n", reg, ret);

	return swab16(ret);
}

static int max17048_get_battery_temp(struct max17048_chip* chip)
{
	int rc = 0;
	int temp = 250;
	union power_supply_propval val = {0};
	if (!chip->psy_qcom_charger)
	{
		chip->psy_qcom_charger = 
			power_supply_get_by_name(QCOM_POWR_SUPPLY_CAHRGER_NAME);
	}
	if (chip->psy_qcom_charger)
	{
		rc = chip->psy_qcom_charger->get_property(
			chip->psy_qcom_charger, POWER_SUPPLY_PROP_TEMP, &val);
		if (likely(!IS_ERR_VALUE(rc)))
		{
			temp = val.intval;
		}
		else
		{
			pr_err("get battery temp failed!!\n");
		}
	}
	return temp;
}

u16 max17048_prepare_load_model(struct max17048_chip* chip)
{
	u8 OCV_1,OCV_2;
	u16 msb = -1;
	u16 check_times = 0;
	do {
		msleep(100);
		//Step1:unlock model access, enable access to OCV and table registers
		max17048_write_reg(chip->client, 
			MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_UNLOCK);

		//Step2:Read OCV, verify Model Access Unlocked  
		msb = max17048_read_reg(chip->client, MAX17048_OCV_REG);//read OCV
		OCV_1 = HIGH_WORD_VALUE(msb);//"big endian":low byte save to MSB
		OCV_2 = LOW_WORD_VALUE(msb);

		if(check_times++ >= MAX17048_LOADMODLE_MAXTIMES) {//avoid of while(1)
			pr_info("read ocv reg time out ...");
			return (-EIO);
		}
	}while ((OCV_1==MAX17048_REG_INVALID_VALUE)&&
		(OCV_2==MAX17048_REG_INVALID_VALUE));//verify Model Access Unlocked
	return msb;
}

static void max17048_dump_model_regs(struct max17048_chip* chip)
{
	u16 value = 0;
	u8 k = 0;

	for (k=0; k < MAX17048_MODEL_DATA_SIZE; k+=2)
	{
		value = max17048_read_reg(chip->client, (MAX17048_MODEL_DATA_REG_ADRR+k));
		pr_debug("model_data[0x%02x] = 0x%04x \n", (MAX17048_MODEL_DATA_REG_ADRR+k), value);
	}

	//read RCOMPSeg (for MAX17048/MAX17049 only)
	for (k=0; k < MAX17048_MODEL_RCOMSEG_SIZE; k+=2)
	{
		value = max17048_read_reg(chip->client, (MAX17048_MODEL_RCOMSEG_ADDR+k));
		pr_debug("rcomp_seg[0x%02x] = 0x%04x \n", (MAX17048_MODEL_RCOMSEG_ADDR+k), value);
	}
}

void max17048_load_model(struct max17048_chip* chip) {	
   	/******************************************************************************
	Step 5. Write the Model
	Once the model is unlocked, the host software must write the 64 byte model
	to the device. The model is located between memory 0x40 and 0x7F.
	The model is available in the INI file provided with your performance
	report. See the end of this document for an explanation of the INI file.
	Note that the table registers are write-only and will always read
	0xFF. Step 9 will confirm the values were written correctly.
	*/
	u8 k=0;
	u16 value = 0;
	u8* pmodel_data = chip->pdata->pbat_data->model_data;
	u16 rcomseg = chip->pdata->pbat_data->ini_rcompseg;
	//Once the model is unlocked, the host software must write the 64 bytes model to the device
	for (k=0; k < MAX17048_MODEL_DATA_SIZE; k+=2)
	{
		value = TWO_U8_TO_U16(pmodel_data[k], pmodel_data[k+1]);
		//The model is located between memory 0x40 and 0x7F
		max17048_write_reg(chip->client, 
			(MAX17048_MODEL_DATA_REG_ADRR+k), value);
	}

	//Write RCOMPSeg (for MAX17048/MAX17049 only)
	for (k=0; k < MAX17048_MODEL_RCOMSEG_SIZE; k+=2)
	{
	    max17048_write_reg(chip->client,
			(MAX17048_MODEL_RCOMSEG_ADDR+k), rcomseg);
	}

	if (log_level >= 2)
	{
		max17048_dump_model_regs(chip);
	}
}

bool max17048_verify_model_is_correct(struct max17048_chip* chip) 
{
	u8 SOC_1, SOC_2;
	u16 msb;

	msleep(200);//Delay at least 150ms(max17048/1/3/4 only)

	//Step 7. Write OCV:write(reg[0x0E], INI_OCVTest_High_Byte, INI_OCVTest_Low_Byte)
	max17048_write_reg(chip->client,MAX17048_OCV_REG, 
		chip->pdata->pbat_data->ini_ocvtest);

	//Step 7.1 Disable Hibernate (MAX17048/49 only)
	max17048_write_reg(chip->client,MAX17048_HIBRT_REG,0x0);

	//Step 7.2. Lock Model Access (MAX17048/49/58/59 only)
	max17048_write_reg(chip->client, MAX17048_MODEL_ACCESS_REG, 
		MAX17048_MODEL_ACCESS_LOCK);

	//Step 8: Delay between 150ms and 600ms, delaying beyond 600ms could cause the verification to fail
	msleep(500);
 
	//Step 9. Read SOC register and compare to expected result
	msb = max17048_read_reg(chip->client, MAX17048_SOC_REG);

	SOC_1 = HIGH_WORD_VALUE(msb);//"big endian":low byte save MSB
	SOC_2 = LOW_WORD_VALUE(msb);

	pr_debug("soc1=%d, soc2=%d, checka=%d, checkb=%d\n", SOC_1,SOC_2,
			chip->pdata->pbat_data->ini_soccheck_a,
			chip->pdata->pbat_data->ini_soccheck_b);
	if(SOC_1 >= chip->pdata->pbat_data->ini_soccheck_a &&
	SOC_1 <= chip->pdata->pbat_data->ini_soccheck_b) {
		pr_info("model data was loaded successfully!\n");
		return true;
	}
	else {
		pr_info("model data was NOT loaded successfully!\n");
		return false; 
	}
}

void max17048_cleanup_model_load(struct max17048_chip* chip, u16 original_ocv) 
{
	u16 recom_alrt = TWO_U8_TO_U16(
		(chip->pdata->pbat_data->ini_rcomp),
		(MAX17048_ALERT_SOC_HIGHEST - chip->pdata->alert_soc));

	//step9.1, Unlock Model Access (MAX17048/49/58/59 only): To write OCV, requires model access to be unlocked
	max17048_write_reg(chip->client,
		MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_UNLOCK);

	//step 10 Restore CONFIG and OCV: write(reg[0x0C], INI_RCOMP, Your_Desired_Alert_Configuration)
	max17048_write_reg(chip->client, MAX17048_RCOMP_REG,  recom_alrt);//RCOMP0=94 , battery empty Alert threshold = 4% -> 0x1C
	max17048_write_reg(chip->client, MAX17048_OCV_REG, original_ocv); 

	//step 10.1 Restore Hibernate (MAX17048/49 only)
	/* disable Hibernate */
	max17048_write_reg(chip->client, MAX17048_HIBRT_REG, 0x0);

	//step 11 Lock Model Access
	max17048_write_reg(chip->client,
		MAX17048_MODEL_ACCESS_REG, MAX17048_MODEL_ACCESS_LOCK);
	//step 12,//delay at least 150ms before reading SOC register
	mdelay(200); 
}

/*
RI (reset indicator) is set when the device powers
up. Any time this bit is set, the IC is not configured,
so the model should be loaded and the bit should
be cleared.
*/
bool max17048_check_por(struct max17048_chip* chip)
{
	u16 status;
	//firstly check POR
	status = max17048_read_reg(chip->client, MAX17048_STATUS_REG);
	return (!!(status & MAX17048_IC_STATUS_POR));  //if por is not set,do nothing
}

void max17048_clear_por(struct max17048_chip* chip)
{
	u16 status;
	status = max17048_read_reg(chip->client, MAX17048_STATUS_REG);
	status = status & (~MAX17048_IC_STATUS_POR);
	max17048_write_reg(chip->client,MAX17048_STATUS_REG,status);
}

bool max17048_handle_model(struct max17048_chip* chip, int load_or_verify) 
{
	bool model_load_ok = false;
	u16 check_times = 0;
	u16 msb_original;
	
	//remember the OCV
	msb_original = max17048_prepare_load_model(chip);
	if (msb_original < 0){
		pr_err("read the original OCV failed!! \n");
		goto out;
	}

	check_times = 0;
	do {
		// Steps 1-4		
		max17048_prepare_load_model(chip);
		if (load_or_verify == MAX17048_LOAD_MODEL) {		
			// Step 5
			max17048_load_model(chip);
		}

		// Steps 6-9
		model_load_ok = max17048_verify_model_is_correct(chip);
		if (!model_load_ok) {
			load_or_verify = MAX17048_LOAD_MODEL;
		}

		if (check_times++ >= MAX17048_LOADMODLE_MAXTIMES) {
			pr_err("max17048 handle model :time out1...");
			goto out;
		}
	} while (!model_load_ok);

	// Steps 10-12
	max17048_cleanup_model_load(chip, msb_original);

	//clear up por
		//firstly check POR
	if(max17048_check_por(chip)){
		max17048_clear_por(chip);
	}
out:
	return model_load_ok;
}

void max17048_update_rcomp(struct max17048_chip *chip) 
{
	u16 cfg=0;
	int temp = 250;

	//int NewRCOMP = INI_RCOMP;
	u8 new_rcomp = 0;
	u8 ini_rcomp = chip->pdata->pbat_data->ini_rcomp;
	int ini_tempco_up = chip->pdata->pbat_data->ini_tempco_up;
	int ini_tempco_down = chip->pdata->pbat_data->ini_tempco_dwon;
	
	/*get battery temperature*/
	temp = max17048_get_battery_temp(chip);

	if(temp > MAX17048_RCOMP_TEMP_CONST) {
		new_rcomp = (u8)(ini_rcomp + (temp -MAX17048_RCOMP_TEMP_CONST) *
			ini_tempco_up/(MAX17048_RCOMP_FACTORIAL * MAX17048_RCOMP_TEMP_FACTORIAL));
	} 
	else if(temp < MAX17048_RCOMP_TEMP_CONST) {
		new_rcomp = (u8)(ini_rcomp + (temp -MAX17048_RCOMP_TEMP_CONST) * 
			ini_tempco_down/(MAX17048_RCOMP_FACTORIAL * MAX17048_RCOMP_TEMP_FACTORIAL));
	} 

	if(new_rcomp > MAX17048_REG_INVALID_VALUE){
		new_rcomp = MAX17048_REG_INVALID_VALUE;
	}
	else if(new_rcomp < 0){
		new_rcomp = 0;
	}
	/*now update it to register*/
	cfg = max17048_read_reg(chip->client, MAX17048_RCOMP_REG);
	cfg = TWO_U8_TO_U16(new_rcomp, LOW_WORD_VALUE(cfg));
	max17048_write_reg(chip->client, MAX17048_RCOMP_REG, cfg);
	msleep(150);
}

static int max17048_get_vcell_batt_uv(struct max17048_chip *chip)
{
	u16 fg_vcell = 0;
	u32 vcell_uV = 0;

	fg_vcell = max17048_read_reg(chip->client, MAX17048_VCELL_REG);
	if (fg_vcell >= 0)
	{
		vcell_uV = MAX17048_VBATT_REGVAL_TO_UV(fg_vcell);
		chip->vcell = vcell_uV;
		log_vdbg("max17048:chip->vcell = %duV\n", vcell_uV);
		return vcell_uV;
	}
	pr_err("max17048:chip get vcell error rc = %d \n", fg_vcell);
	return fg_vcell;
}

static int max17048_avarage_vbat_mv(struct max17048_chip *chip, int avrg_count)
{
#define HW_AVRG_BATMV_COUNT_RIGHT(cnt)	((cnt) = min(128, max((cnt), 1)))

	int cnt = 0;
	int vbat_uv = 0;
	int sum = 0;

	/* sample_count scope is 1 -128 */
	HW_AVRG_BATMV_COUNT_RIGHT(avrg_count);

	for(cnt = 0; cnt < avrg_count ; cnt++)
	{
		vbat_uv = max17048_get_vcell_batt_uv(chip);
		if (vbat_uv < 0)
		{
			pr_debug("get batt mv failed, break count = %d \n", cnt);
			break;
		}
		sum += vbat_uv;
	}
	
	return ((0 == cnt) ? (-EIO) :  (sum / cnt / HW_VOLTAGE_FACTORIAL));
}

static void max17048_low_soc_protecting(struct max17048_chip *chip)
{
	int vbat_mv = 0;
	struct timespec kernel_time;
	static bool power_up_flag = false;
	static u8 bad_mv_count = 0;

	/*  check voltage only after the phone power up */
	if(!power_up_flag)
	{
		/*  kernel time is less than VOLTAE_CHECK_BEGAIN_SECOND,   return soc
		     kernel time is greater than VOLTAE_CHECK_BEGAIN_SECOND, set flag true*/
		ktime_get_ts(&kernel_time);
		if(kernel_time.tv_sec < HW_SYS_POWRUP_TIME)
		{
			log_vdbg(" power_up time do not check voltage of battery,\
				soc is %d ,kernel_time is %ld \n",  chip->soc, kernel_time.tv_sec);
			return;
		}
		else
		{
			pr_debug("set power_up flag true \n");
			power_up_flag = true;
		}		
	}

	/* get vbatt uv based on */
	vbat_mv = max17048_avarage_vbat_mv(chip, HW_AVRG_BATT_MV_COUNT);

	if (vbat_mv <= 0)
	{
		pr_err(" get avarage battery mv failed!!\n");
		return;
	}
	else if(vbat_mv < chip->pdata->bad_batt_mv)
	{
		bad_mv_count += 1;
	}
	else
	{
		bad_mv_count = 0;
	}
	
	if(bad_mv_count >= HW_PROTECT_BATT_BAD_CNT)
	{
		pr_info("Voltage is too low,change soc to zero\n ");
		chip->soc = 0;
		bad_mv_count = HW_PROTECT_BATT_BAD_CNT;
	}
	else if(0 == chip->soc)
	{
		pr_info("Voltage is higher than %d ,change soc from zero to 1 \n ",
					chip->pdata->bad_batt_mv);
		chip->soc = 1;
	}

	log_vdbg("soc %d, bad_voltage_count %d,avarage vbat_uv %d \n ", 
		chip->soc, bad_mv_count, vbat_mv);
}

static void max17048_low_soc_adjust(struct max17048_chip *chip)
{
	static u8 vcutoff_mv_count = 0;
	int vbat_mv;
	int cutoff_mv = chip->pdata->cutoff_batt_mv;

	vbat_mv = max17048_avarage_vbat_mv(chip, HW_AVRG_BATT_MV_COUNT);
	if (chip->soc <= chip->pdata->cutoff_batt_soc)
	{
		if (vbat_mv > cutoff_mv)
		{
			chip->soc = chip->pdata->cutoff_batt_soc + 1;
			pr_info("soc is low to %d, but battery mv is higher than cutoff mv, soc+1 !\n",
				chip->pdata->cutoff_batt_soc);
		}
		vcutoff_mv_count = 0;
	}
	else
	{
		if (vbat_mv <= cutoff_mv)
		{
			vcutoff_mv_count++;
		}
		else
		{
			vcutoff_mv_count = 0;
		}

		/* if the battery mv is lower than cuttof batt mv continue  */
		if (vcutoff_mv_count >= HW_CUTOFF_BATT_CNT)
		{
			vcutoff_mv_count = HW_CUTOFF_BATT_CNT;
			chip->soc = chip->pdata->cutoff_batt_soc;
			pr_info("battery mv is lower than cutoff mv, set soc to cutoff soc %d !\n",
				chip->pdata->cutoff_batt_soc);
		}
	}
}

static u8 max17048_get_soc(struct max17048_chip *chip)
{
	u16 fg_soc = 0;
	int ini_bits = chip->pdata->pbat_data->ini_bits;

	log_vdbg("max17048 Fuel-Gauge Previous SOC:%d, bits:%d\n", chip->soc, ini_bits);
	fg_soc = max17048_read_reg(chip->client, MAX17048_SOC_REG);

	fg_soc = MAX17048_CALC_SOC(fg_soc, ini_bits);

	chip->soc = min((int)fg_soc, MAX17048_SOC_FULL);
	log_vdbg("max17048:updated chip->soc = %d, fg_soc = %d\n", chip->soc, fg_soc);

	/* check the low capacity */
	if (get_max77819_charger_present())
	{
		max17048_low_soc_protecting(chip);

		/* In factory system mode, reporting the soc 1% when the soc fall to 0% */
		if (factory_flag && 0 == chip->soc)
		{
			chip->soc = 1;
			pr_info("In factory system mode, reporting the soc 1 when the soc fall to 0 \n");
		}
	}
	else
	{
		max17048_low_soc_adjust(chip);
	}

	return chip->soc;
}

static int max17048_get_charge_current_now(struct max17048_chip *chip)
{
#define MAXIM_VICHG_ADC_CHAN P_MUX2_1_3
	int rc;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, MAXIM_VICHG_ADC_CHAN, &results);
	if (rc) {
		pr_err("Unable to read charge current now !! rc=%d\n", rc);
		return -1;
	}
	VADC_VICHG_ZERO_CHECK(results.physical);

	return MAXIM_CURRENT_NOW_UV_TO_UA(results.physical);
}

static int max17048_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int rc = 0;
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, fgbattery);

	__lock(chip);
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		max17048_get_vcell_batt_uv(chip);
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		max17048_get_soc(chip);
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->pdata->pbat_data->full_capacity;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max17048_get_charge_current_now(chip);
		break;
	default:
		rc = -EINVAL;
	}
	__unlock(chip);
	return rc;
}

static u16 max17048_get_version(struct i2c_client *client)
{
	u16 chip_version = 0;

	chip_version = max17048_read_reg(client, MAX17048_VER_REG);

	pr_info("max17048 Fuel-Gauge Ver 0x%04x\n", chip_version);
	return chip_version;
}

static void max17048_init(struct max17048_chip *chip)
{
	u8 ini_rcomp = 0;	
	u8 init_alert = 0;
	u16 init_cfg = 0;
	int mvolt = 0;
	u16  vreset = 0;

	if (!chip->pdata){
		pr_err("max17048_platform_data is not inited!!\n");
		return;
	}
	/*config rcomp and alert register*/
	ini_rcomp = chip->pdata->pbat_data->ini_rcomp;
	init_alert = chip->pdata->alert_soc;
	if((init_alert > MAX17048_ALERT_SOC_HIGHEST)||
		(init_alert < MAX17048_ALERT_SOC_LOWEST))
	{
		init_alert = HW_ALERT_LOW_SOC;
	}

	init_cfg = TWO_U8_TO_U16(ini_rcomp, (MAX17048_ALERT_SOC_HIGHEST-init_alert));
	max17048_write_reg(chip->client, MAX17048_RCOMP_REG, init_cfg);	

	/*config vreset */
	mvolt = chip->pdata->reset_mvlolt;
	mvolt = min(mvolt, MAX17048_VRESET_HIGHEST_MV);
	mvolt = max(mvolt, MAX17048_VRESET_LOWEST_MV);
	
	vreset = MAX17048_VRESET_MV_TO_REGVAL(mvolt);
	vreset = vreset << 8;
	max17048_write_reg(chip->client, MAX17048_VRESET_REG, vreset);
}

static void max17048_report_soc(struct max17048_chip *chip)
{
	static u8 presoc = 0;
	
	if (presoc != chip->soc)
	{
		power_supply_changed(&(chip->fgbattery));
		presoc = chip->soc;
	}

	/* If battery alarm is enabled, but we charge the battery again at alert_soc,*/
	/* we should release the pm lock to allow the phone to sleep when >=alert_soc */
	if (chip->bbatt_alrm &&
		((chip->soc) > (chip->pdata->alert_soc)))
	{
		chip->bbatt_alrm = false;
		pm_relax(&(chip->client->dev));
	}
}

static void max17048_dump_register(struct max17048_chip* chip)
{
	static u8	g_dump_reg_adrrs[] = {
		0x02,0x04,0x06,0x0a,0x0c,0x14,0x16,0x18,0x1a,
	};
	static u8	g_dump_lock_reg_addrs[] = {
		0x0e,
	};
	int i, num = 0;
	u16 value;

	num = sizeof(g_dump_reg_adrrs) / sizeof(g_dump_reg_adrrs[0]);
	for (i = 0; i < num; i++)
	{
		value = max17048_read_reg(chip->client, g_dump_reg_adrrs[i]);
		pr_debug("reg[0x%02x] = 0x%04x \n", g_dump_reg_adrrs[i], value);
	}

	num = sizeof(g_dump_lock_reg_addrs) / sizeof(g_dump_lock_reg_addrs[0]);
	/* unlock */
	__unlock_register(chip)
	for (i = 0; i < num; i++)
	{
		value = max17048_read_reg(chip->client, g_dump_lock_reg_addrs[i]);
		pr_debug("reg[0x%02x] = 0x%04x \n", g_dump_lock_reg_addrs[i], value);
	}
	__lock_register(chip)
	/* lock */
}

static void max17048_debug_log(struct max17048_chip* chip)
{
	int temp = 0;

	if (log_level >= 2)
	{
		max17048_dump_register(chip);

		temp = max17048_get_battery_temp(chip);
		pr_debug("battery temp = %d \n", temp);
	}
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;
	chip = container_of(work, struct max17048_chip, work.work);
	__lock(chip);

	max17048_update_rcomp(chip);		//update rcomp periodically
	max17048_report_soc(chip);

	max17048_debug_log(chip);

	loop_schedule_delayed_work(&chip->work, MAX17048_TIME_WORK_DELAY);
	__unlock(chip);
}

static void max17048_handle_work(struct work_struct *work)
{
	struct max17048_chip *chip;
	chip = container_of(work, struct max17048_chip, hand_work.work);
	__lock(chip);
	max17048_handle_model(chip, MAX17048_VERIFY_FIX_MODEL);
	loop_schedule_delayed_work(&chip->hand_work, MAX17048_HANDLE_MODEL_DELAY);
	__unlock(chip);
}

static irqreturn_t max17048_low_bat_interrupt(int irq, void *thechip)
{
	struct max17048_chip* chip = (struct max17048_chip*)thechip;

	schedule_delayed_work(&chip->notifier_work, 0);

	return IRQ_HANDLED;
}

static void max17048_interrupt_notifier_work(struct work_struct *work)
{
	struct max17048_chip *chip;
	int capacity = 0;
	chip = container_of(work, struct max17048_chip, notifier_work.work);
	__lock(chip);

	capacity =  max17048_get_soc(chip);
	pr_info("battery is low: interrupt_notifier_work is invoked, capacity = %d\n", capacity);
	if(capacity > chip->pdata->alert_soc)
	{
		pm_wakeup_event(&chip->client->dev,100);
	}
	else
	{
		pm_stay_awake(&chip->client->dev);
		chip->bbatt_alrm = true;
	}
	power_supply_changed(&chip->fgbattery);
	
	__unlock(chip);
	return;
}

static int max17048_soc_alarm_irq_init(struct max17048_chip* chip)
{
	int ret = 0;
	
	if (!gpio_is_valid(chip->pdata->irq_gpio)) {
		pr_err("irq gpio %d not provided\n", chip->pdata->irq_gpio);
		return -1;
	}

	/* configure max17048 irq gpio */
	ret = gpio_request(chip->pdata->irq_gpio,MAX17048_SOC_ALRM_IRQ_NAME);
	pr_debug("max17048 gpio_request\n");
	if (ret)
	{
		pr_err("unable to request gpio %d\n",	chip->pdata->irq_gpio);
		goto err_gpio;
	}
	else
	{
		gpio_direction_input(chip->pdata->irq_gpio);
		pr_debug("max17048 gpio_direction set to input\n");
    }
	chip->irq = chip->client->irq = gpio_to_irq(chip->pdata->irq_gpio);
	pr_debug("max17048 gpio = %d, aler irq = %d \n",chip->pdata->irq_gpio,chip->irq);

	/* request battery_low interruption */
	ret = request_irq(chip->irq, max17048_low_bat_interrupt, IRQF_TRIGGER_FALLING,
			MAX17048_SOC_ALRM_IRQ_NAME, chip);
	if (ret) {
		pr_err("could not request irq %d, status %d\n", chip->irq, ret);
		goto err_irq;
	}
	else
	{
		enable_irq_wake(chip->client->irq);
	}
	pr_debug("max17048 irq init ok,chip->irq = %d\n",chip->irq);
	return 0;

err_irq:
	free_irq(chip->irq, chip);
err_gpio:
	gpio_free(chip->pdata->irq_gpio);
	return -EIO;
}

static void max17048_soc_alarm_irq_free(struct max17048_chip* chip)
{
	free_irq(chip->irq, chip);
	gpio_free(chip->pdata->irq_gpio);
}

static int max17048_battery_id(struct max17048_chip *chip)
{
	int rc = 0;
	static int64_t vbatt_id = -1;
	struct qpnp_vadc_result results;

	if(-1 != vbatt_id)
	{
		return vbatt_id;
	}
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &results);
	if (rc) {
		pr_err("Unable to read battery_id rc=%d\n", rc);
		return -1;
	}
	vbatt_id = results.physical;
	return vbatt_id;
}

int maxim_get_battery_id_uv(void)
{
	if (global_chip)
	{
		return max17048_battery_id(global_chip);
	}
	return -1;
}
EXPORT_SYMBOL(maxim_get_battery_id_uv);

static int max17048_get_platform_data(struct max17048_chip* chip)
{
	int rc = 0;
	struct device_node *np = chip->client->dev.of_node;
	int vbatt_id = 0;

	/* get fgauge platform data info */
	//maxim,irq-soc-alarm-gpio;
	gplatform_data_default.irq_gpio = of_get_named_gpio(np, "maxim,irq-soc-alarm-gpio", 0);
	if (gplatform_data_default.irq_gpio <= 0){
		pr_err("batt-low-gpio is not available\n");
	}
	pr_debug("batt-low-gpio %d is assigned\n" ,gplatform_data_default.irq_gpio);

	OF_HW_READ_PROPERTY_VAL(np, "maxim,alert-soc", gplatform_data_default.alert_soc);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,reset-mvlolt", gplatform_data_default.reset_mvlolt);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,cutoff_batt_mv", gplatform_data_default.cutoff_batt_mv);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,cutoff_batt_soc", gplatform_data_default.cutoff_batt_soc);
	OF_HW_READ_PROPERTY_VAL(np, "maxim,bad_batt_mv", gplatform_data_default.bad_batt_mv);
	
	/* get battery data info */
	vbatt_id = max17048_battery_id(chip);
	pr_debug("battery id uv %d \n", vbatt_id);
	rc = of_batterydata_read_fgauge_data_maxim(np, gplatform_data_default.pbat_data, vbatt_id);

	chip->pdata = &gplatform_data_default;

	return rc;
}

static int max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);			
	struct max17048_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		pr_err("Failed: I2C bus function is not correct\n");		
		return -EIO;
	}
	ret = max17048_get_version(client);
	if (!MAX17048_RIGHT_VERSION(ret)) {
		pr_err("fail to get max17048 Fuel-Gauge Ver, exit!\n");
		return -EIO;
	}
	
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->lock);

	chip->vadc_dev = qpnp_get_vadc(&(chip->client->dev), "maxim-bms");
	if (IS_ERR(chip->vadc_dev)) {
		ret = PTR_ERR(chip->vadc_dev);
		if (ret != -EPROBE_DEFER) {
			pr_err("vadc prop missing rc=%d\n", ret);
		}
		else{
			pr_err("no found dts adc node rc=%d\n", ret);
		}
		goto err_out1;
	}

	max17048_get_platform_data(chip);
	max17048_init(chip);

    /*low soc alert interupt support*/
	chip->bbatt_alrm = false;
	if (max17048_soc_alarm_irq_init(chip) < 0){
		ret = -EIO;
		goto err_out1; 
	}

	chip->fgbattery.name		= "max17048_fgauge";
	chip->fgbattery.type		= POWER_SUPPLY_TYPE_BMS;
	chip->fgbattery.get_property	= max17048_get_property;
	chip->fgbattery.properties	= max17048_battery_props;
	chip->fgbattery.num_properties	= ARRAY_SIZE(max17048_battery_props);

	ret = power_supply_register(&client->dev, &chip->fgbattery);
	if (ret) {
		pr_err("failed: power supply register\n");
		goto err_out2;
	}

	max17048_handle_model(chip,MAX17048_LOAD_MODEL);

	INIT_DELAYED_WORK(&chip->notifier_work,max17048_interrupt_notifier_work);
	INIT_DELAYED_WORK(&chip->work, max17048_work);
	INIT_DELAYED_WORK(&chip->hand_work, max17048_handle_work);

	schedule_delayed_work(&chip->hand_work,MAX17048_HANDLE_MODEL_DELAY);
	schedule_delayed_work(&chip->work, 0);

	global_chip = chip;
	return 0;

err_out2:
	max17048_soc_alarm_irq_free(chip);
err_out1:
	mutex_destroy(&chip->lock);
	kfree(chip);
	return ret;
}

static int max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->hand_work);
	cancel_delayed_work(&chip->work);
	cancel_delayed_work(&chip->notifier_work);
	
	power_supply_unregister(&chip->fgbattery);
	max17048_soc_alarm_irq_free(chip);
	
	mutex_destroy(&chip->lock);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17048_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	__lock(chip);

	cancel_delayed_work(&chip->hand_work);
	cancel_delayed_work(&chip->work);
	
	__unlock(chip);
	return 0;
}

static int max17048_resume(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	__lock(chip);

	schedule_delayed_work(&chip->hand_work,MAX17048_HANDLE_MODEL_DELAY);
	schedule_delayed_work(&chip->work, 0);

	__unlock(chip);
	return 0;
}

#else

#define max17048_suspend NULL
#define max17048_resume NULL

#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static struct of_device_id max17048_of_ids[] = {
	{ .compatible = "maxim,max17048" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17048_of_ids);
#endif /* CONFIG_OF */

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
#ifdef CONFIG_OF
    		.of_match_table  = max17048_of_ids,
#endif /* CONFIG_OF */				
	},
	.probe		= max17048_probe,
	.remove		= max17048_remove,
	.suspend		= max17048_suspend,
	.resume		= max17048_resume,
	.id_table	= max17048_id,
};

module_i2c_driver(max17048_i2c_driver);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("max17048 Fuel Gauge");
MODULE_LICENSE("GPL");
