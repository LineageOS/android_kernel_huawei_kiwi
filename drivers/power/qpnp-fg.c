/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"FG: %s: " fmt, __func__

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/rtc.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/spmi.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/power_supply.h>
#include <linux/of_batterydata.h>
#include <linux/string_helpers.h>

/* Register offsets */

/* Interrupt offsets */
#define INT_RT_STS(base)			(base + 0x10)
#define INT_EN_CLR(base)			(base + 0x16)

/* SPMI Register offsets */
#define SOC_MONOTONIC_SOC	0x09
#define MEM_INTF_CFG		0x40
#define MEM_INTF_CTL		0x41
#define MEM_INTF_ADDR_LSB	0x42
#define MEM_INTF_ADDR_MSB	0x43
#define MEM_INTF_WR_DATA0	0x48
#define MEM_INTF_WR_DATA1	0x49
#define MEM_INTF_WR_DATA2	0x4A
#define MEM_INTF_WR_DATA3	0x4B
#define MEM_INTF_RD_DATA0	0x4C
#define MEM_INTF_RD_DATA1	0x4D
#define MEM_INTF_RD_DATA2	0x4E
#define MEM_INTF_RD_DATA3	0x4F
#define OTP_CFG1		0xE2
#define SOC_BOOT_MOD		0x50
#define SOC_RESTART		0x51

#define REG_OFFSET_PERP_SUBTYPE	0x05

/* RAM register offsets */
#define RAM_OFFSET		0x400

/* Bit/Mask definitions */
#define FULL_PERCENT		0xFF
#define MAX_TRIES_SOC		5
#define MA_MV_BIT_RES		39
#define MSB_SIGN		BIT(7)
#define IBAT_VBAT_MASK		0x7F
#define NO_OTP_PROF_RELOAD	BIT(6)
#define REDO_FIRST_ESTIMATE	BIT(3)
#define RESTART_GO		BIT(0)

/* SUBTYPE definitions */
#define FG_SOC			0x9
#define FG_BATT			0xA
#define FG_ADC			0xB
#define FG_MEMIF		0xC

#define QPNP_FG_DEV_NAME "qcom,qpnp-fg"
#define MEM_IF_TIMEOUT_MS	5000

/* Debug Flag Definitions */
enum {
	FG_SPMI_DEBUG_WRITES		= BIT(0), /* Show SPMI writes */
	FG_SPMI_DEBUG_READS		= BIT(1), /* Show SPMI reads */
	FG_IRQS				= BIT(2), /* Show interrupts */
	FG_MEM_DEBUG_WRITES		= BIT(3), /* Show SRAM writes */
	FG_MEM_DEBUG_READS		= BIT(4), /* Show SRAM reads */
	FG_POWER_SUPPLY			= BIT(5), /* Show POWER_SUPPLY */
	FG_STATUS			= BIT(6), /* Show FG status changes */
};

struct fg_mem_setting {
	u16	address;
	u8	offset;
	int	value;
};

struct fg_mem_data {
	u16	address;
	u8	offset;
	unsigned int len;
	int	value;
};

/* FG_MEMIF setting index */
enum fg_mem_setting_index {
	FG_MEM_SOFT_COLD = 0,
	FG_MEM_SOFT_HOT,
	FG_MEM_HARD_COLD,
	FG_MEM_HARD_HOT,
	FG_MEM_RESUME_SOC,
	FG_MEM_SETTING_MAX,
};

/* FG_MEMIF data index */
enum fg_mem_data_index {
	FG_DATA_BATT_TEMP = 0,
	FG_DATA_OCV,
	FG_DATA_VOLTAGE,
	FG_DATA_CURRENT,
	FG_DATA_BATT_ESR,
	FG_DATA_BATT_ID,
	FG_DATA_BATT_ID_INFO,
	FG_DATA_MAX,
};

#define SETTING(_idx, _address, _offset, _value)	\
	[FG_MEM_##_idx] = {				\
		.address = _address,			\
		.offset = _offset,			\
		.value = _value,			\
	}						\

static struct fg_mem_setting settings[FG_MEM_SETTING_MAX] = {
	/*       ID                    Address, Offset, Value*/
	SETTING(SOFT_COLD,       0x454,   0,      100),
	SETTING(SOFT_HOT,        0x454,   1,      400),
	SETTING(HARD_COLD,       0x454,   2,      50),
	SETTING(HARD_HOT,        0x454,   3,      450),
	SETTING(RESUME_SOC,      0x45C,   1,      0),
};

#define DATA(_idx, _address, _offset, _length,  _value)	\
	[FG_DATA_##_idx] = {				\
		.address = _address,			\
		.offset = _offset,			\
		.len = _length,			\
		.value = _value,			\
	}						\

static struct fg_mem_data fg_data[FG_DATA_MAX] = {
	/*       ID           Address, Offset, Length, Value*/
	DATA(BATT_TEMP,       0x550,   2,      2,     -EINVAL),
	DATA(OCV,             0x588,   3,      2,     -EINVAL),
	DATA(VOLTAGE,         0x5CC,   1,      2,     -EINVAL),
	DATA(CURRENT,         0x5CC,   3,      2,     -EINVAL),
	DATA(BATT_ESR,        0x554,   2,      2,     -EINVAL),
	DATA(BATT_ID,         0x594,   1,      1,     -EINVAL),
	DATA(BATT_ID_INFO,    0x594,   3,      1,     -EINVAL),
};

static int fg_debug_mask;
module_param_named(
	debug_mask, fg_debug_mask, int, S_IRUSR | S_IWUSR
);

static int fg_sense_type = -EINVAL;

static int fg_est_dump;
module_param_named(
	first_est_dump, fg_est_dump, int, S_IRUSR | S_IWUSR
);

static char *fg_batt_type;
module_param_named(
	battery_type, fg_batt_type, charp, S_IRUSR | S_IWUSR
);

struct fg_irq {
	int			irq;
	unsigned long		disabled;
};

enum fg_soc_irq {
	HIGH_SOC,
	LOW_SOC,
	FULL_SOC,
	EMPTY_SOC,
	DELTA_SOC,
	FIRST_EST_DONE,
	SW_FALLBK_OCV,
	SW_FALLBK_NEW_BATTRT_STS,
	FG_SOC_IRQ_COUNT,
};

enum fg_batt_irq {
	JEITA_SOFT_COLD,
	JEITA_SOFT_HOT,
	VBATT_LOW,
	BATT_IDENTIFIED,
	BATT_ID_REQ,
	BATTERY_UNKNOWN,
	BATT_MISSING,
	BATT_MATCH,
	FG_BATT_IRQ_COUNT,
};

enum fg_mem_if_irq {
	FG_MEM_AVAIL,
	TA_RCVRY_SUG,
	FG_MEM_IF_IRQ_COUNT,
};

#define THERMAL_COEFF_N_BYTES		6
struct fg_chip {
	struct device		*dev;
	struct spmi_device	*spmi;
	u16			soc_base;
	u16			batt_base;
	u16			mem_base;
	u16			vbat_adc_addr;
	u16			ibat_adc_addr;
	atomic_t		memif_user_cnt;
	struct fg_irq		soc_irq[FG_SOC_IRQ_COUNT];
	struct fg_irq		batt_irq[FG_BATT_IRQ_COUNT];
	struct fg_irq		mem_irq[FG_MEM_IF_IRQ_COUNT];
	struct completion	sram_access;
	struct completion	batt_id_avail;
	struct power_supply	bms_psy;
	struct mutex		rw_lock;
	struct work_struct	batt_profile_init;
	struct work_struct	dump_sram;
	struct power_supply	*batt_psy;
	bool			profile_loaded;
	bool			use_otp_profile;
	bool			battery_missing;
	bool			power_supply_registered;
	struct delayed_work	update_jeita_setting;
	struct delayed_work	update_sram_data;
	char			*batt_profile;
	u8			thermal_coefficients[THERMAL_COEFF_N_BYTES];
	bool			use_thermal_coefficients;
	unsigned int		batt_profile_len;
	const char		*batt_type;
	unsigned long		last_sram_update_time;
};

/* FG_MEMIF DEBUGFS structures */
#define ADDR_LEN	4	/* 3 byte address + 1 space character */
#define CHARS_PER_ITEM	3	/* Format is 'XX ' */
#define ITEMS_PER_LINE	4	/* 4 data items per line */
#define MAX_LINE_LENGTH  (ADDR_LEN + (ITEMS_PER_LINE * CHARS_PER_ITEM) + 1)
#define MAX_REG_PER_TRANSACTION	(8)

static const char *DFS_ROOT_NAME	= "fg_memif";
static const mode_t DFS_MODE = S_IRUSR | S_IWUSR;
static const char *default_batt_type	= "Unknown Battery";
static const char *loading_batt_type	= "Loading Battery Data";
static const char *missing_batt_type	= "Disconnected Battery";

/* Log buffer */
struct fg_log_buffer {
	size_t rpos;	/* Current 'read' position in buffer */
	size_t wpos;	/* Current 'write' position in buffer */
	size_t len;	/* Length of the buffer */
	char data[0];	/* Log buffer */
};

/* transaction parameters */
struct fg_trans {
	u32 cnt;	/* Number of bytes to read */
	u16 addr;	/* 12-bit address in SRAM */
	u32 offset;	/* Offset of last read data + byte offset */
	struct fg_chip *chip;
	struct fg_log_buffer *log; /* log buffer */
	u8 *data;	/* fg data that is read */
	struct mutex memif_dfs_lock; /* Prevent thread concurrency */
};

struct fg_dbgfs {
	u32 cnt;
	u32 addr;
	struct fg_chip *chip;
	struct dentry *root;
	struct mutex  lock;
	struct debugfs_blob_wrapper help_msg;
};

static struct fg_dbgfs dbgfs_data = {
	.lock = __MUTEX_INITIALIZER(dbgfs_data.lock),
	.help_msg = {
	.data =
"FG Debug-FS support\n"
"\n"
"Hierarchy schema:\n"
"/sys/kernel/debug/fg_memif\n"
"       /help            -- Static help text\n"
"       /address  -- Starting register address for reads or writes\n"
"       /count    -- Number of registers to read (only used for reads)\n"
"       /data     -- Initiates the SRAM read (formatted output)\n"
"\n",
	},
};

static const struct of_device_id fg_match_table[] = {
	{	.compatible = QPNP_FG_DEV_NAME, },
	{}
};

static char *fg_supplicants[] = {
	"battery"
};

#define DEBUG_PRINT_BUFFER_SIZE 64
static void fill_string(char *str, size_t str_len, u8 *buf, int buf_len)
{
	int pos = 0;
	int i;

	for (i = 0; i < buf_len; i++) {
		pos += scnprintf(str + pos, str_len - pos, "%02X", buf[i]);
		if (i < buf_len - 1)
			pos += scnprintf(str + pos, str_len - pos, " ");
	}
}

static int fg_write(struct fg_chip *chip, u8 *val, u16 addr, int len)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	char str[DEBUG_PRINT_BUFFER_SIZE];

	if ((addr & 0xff00) == 0) {
		pr_err("addr cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, len);
	if (rc) {
		pr_err("write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	if (!rc && (fg_debug_mask & FG_SPMI_DEBUG_WRITES)) {
		str[0] = '\0';
		fill_string(str, DEBUG_PRINT_BUFFER_SIZE, val, len);
		pr_info("write(0x%04X), sid=%d, len=%d; %s\n",
			addr, spmi->sid, len, str);
	}

	return rc;
}

static int fg_read(struct fg_chip *chip, u8 *val, u16 addr, int len)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	char str[DEBUG_PRINT_BUFFER_SIZE];

	if ((addr & 0xff00) == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, len);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n", addr,
				spmi->sid, rc);
		return rc;
	}

	if (!rc && (fg_debug_mask & FG_SPMI_DEBUG_READS)) {
		str[0] = '\0';
		fill_string(str, DEBUG_PRINT_BUFFER_SIZE, val, len);
		pr_info("read(0x%04x), sid=%d, len=%d; %s\n",
			addr, spmi->sid, len, str);
	}

	return rc;
}

static int fg_masked_write(struct fg_chip *chip, u16 addr,
		u8 mask, u8 val, int len)
{
	int rc;
	u8 reg;

	rc = fg_read(chip, &reg, addr, len);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	pr_debug("addr = 0x%x read 0x%x\n", addr, reg);

	reg &= ~mask;
	reg |= val & mask;

	pr_debug("Writing 0x%x\n", reg);

	rc = fg_write(chip, &reg, addr, len);
	if (rc) {
		pr_err("spmi write failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}

	return rc;
}

#define RIF_MEM_ACCESS_REQ	BIT(7)
static inline bool fg_check_sram_access(struct fg_chip *chip)
{
	int rc;
	u8 mem_if_sts;

	rc = fg_read(chip, &mem_if_sts, INT_RT_STS(chip->mem_base), 1);
	if (rc) {
		pr_err("failed to read mem status rc=%d\n", rc);
		return 0;
	}

	if ((mem_if_sts & BIT(FG_MEM_AVAIL)) == 0)
		return false;

	rc = fg_read(chip, &mem_if_sts, chip->mem_base + MEM_INTF_CFG, 1);
	if (rc) {
		pr_err("failed to read mem status rc=%d\n", rc);
		return 0;
	}

	if ((mem_if_sts & RIF_MEM_ACCESS_REQ) == 0)
		return false;

	return true;
}

#define RIF_MEM_ACCESS_REQ	BIT(7)
static inline int fg_assert_sram_access(struct fg_chip *chip)
{
	int rc;
	u8 mem_if_sts;

	rc = fg_read(chip, &mem_if_sts, INT_RT_STS(chip->mem_base), 1);
	if (rc) {
		pr_err("failed to read mem status rc=%d\n", rc);
		return rc;
	}

	if ((mem_if_sts & BIT(FG_MEM_AVAIL)) == 0) {
		pr_err("mem_avail not high: %02x\n", mem_if_sts);
		return -EINVAL;
	}

	rc = fg_read(chip, &mem_if_sts, chip->mem_base + MEM_INTF_CFG, 1);
	if (rc) {
		pr_err("failed to read mem status rc=%d\n", rc);
		return rc;
	}

	if ((mem_if_sts & RIF_MEM_ACCESS_REQ) == 0) {
		pr_err("mem_avail not high: %02x\n", mem_if_sts);
		return -EINVAL;
	}

	return 0;
}

#define INTF_CTL_BURST		BIT(7)
#define INTF_CTL_WR_EN		BIT(6)
static int fg_config_access(struct fg_chip *chip, bool write,
		bool burst, bool otp)
{
	int rc;
	u8 intf_ctl = 0;

	if (otp) {
		/* Configure OTP access */
		rc = fg_masked_write(chip, chip->mem_base + OTP_CFG1,
				0xFF, 0x00, 1);
		if (rc) {
			pr_err("failed to set OTP cfg\n");
			return -EIO;
		}
	}

	intf_ctl = (write ? INTF_CTL_WR_EN : 0) | (burst ? INTF_CTL_BURST : 0);

	rc = fg_write(chip, &intf_ctl, chip->mem_base + MEM_INTF_CTL, 1);
	if (rc) {
		pr_err("failed to set mem access bit\n");
		return -EIO;
	}

	return rc;
}

static int fg_req_and_wait_access(struct fg_chip *chip, int timeout)
{
	int rc = 0, ret = 0;
	bool tried_again = false;

	if (!fg_check_sram_access(chip)) {
		rc = fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
				RIF_MEM_ACCESS_REQ, RIF_MEM_ACCESS_REQ, 1);
		if (rc) {
			pr_err("failed to set mem access bit\n");
			return -EIO;
		}
	}

wait:
	/* Wait for MEM_AVAIL IRQ. */
	ret = wait_for_completion_interruptible_timeout(&chip->sram_access,
			msecs_to_jiffies(timeout));
	/* If we were interrupted wait again one more time. */
	if (ret == -ERESTARTSYS && !tried_again) {
		tried_again = true;
		goto wait;
	} else if (ret <= 0) {
		rc = -ETIMEDOUT;
		pr_err("transaction timed out rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static void fg_release_access_if_necessary(struct fg_chip *chip)
{
	mutex_lock(&chip->rw_lock);
	if (atomic_sub_return(1, &chip->memif_user_cnt) <= 0) {
		fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
				RIF_MEM_ACCESS_REQ, 0, 1);
		INIT_COMPLETION(chip->sram_access);
	}
	mutex_unlock(&chip->rw_lock);
}

static int fg_set_ram_addr(struct fg_chip *chip, u16 *address)
{
	int rc;

	rc = fg_write(chip, (u8 *) address,
			chip->mem_base + MEM_INTF_ADDR_LSB, 2);
	if (rc) {
		pr_err("spmi write failed: addr=%03X, rc=%d\n",
				chip->mem_base + MEM_INTF_ADDR_LSB, rc);
		return rc;
	}

	return rc;
}

#define BUF_LEN		4
static int fg_mem_read(struct fg_chip *chip, u8 *val, u16 address, int len,
		int offset, bool keep_access)
{
	int rc = 0, user_cnt = 0, total_len = 0, orig_address = address;
	u8 *rd_data = val;
	bool otp;
	char str[DEBUG_PRINT_BUFFER_SIZE];

	if (address < RAM_OFFSET)
		otp = 1;

	if (offset > 3) {
		pr_err("offset too large %d\n", offset);
		return -EINVAL;
	}

	address = ((orig_address + offset) / 4) * 4;
	offset = (orig_address + offset) % 4;

	user_cnt = atomic_add_return(1, &chip->memif_user_cnt);
	if (fg_debug_mask & FG_MEM_DEBUG_READS)
		pr_info("user_cnt %d\n", user_cnt);
	mutex_lock(&chip->rw_lock);
	if (!fg_check_sram_access(chip)) {
		rc = fg_req_and_wait_access(chip, MEM_IF_TIMEOUT_MS);
		if (rc)
			goto out;
	}

	rc = fg_config_access(chip, 0, (len > 4), otp);
	if (rc)
		goto out;

	rc = fg_set_ram_addr(chip, &address);
	if (rc)
		goto out;

	if (fg_debug_mask & FG_MEM_DEBUG_READS)
		pr_info("length %d addr=%02X keep_access %d\n",
				len, address, keep_access);

	total_len = len;
	while (len > 0) {
		if (!offset) {
			rc = fg_read(chip, rd_data,
					chip->mem_base + MEM_INTF_RD_DATA0,
					(len > BUF_LEN) ? BUF_LEN : len);
		} else {
			rc = fg_read(chip, rd_data,
				chip->mem_base + MEM_INTF_RD_DATA0 + offset,
				BUF_LEN - offset);

			/* manually set address to allow continous reads */
			address += BUF_LEN;

			rc = fg_set_ram_addr(chip, &address);
			if (rc)
				goto out;
		}
		if (rc) {
			pr_err("spmi read failed: addr=%03x, rc=%d\n",
				chip->mem_base + MEM_INTF_RD_DATA0, rc);
			goto out;
		}
		rd_data += (BUF_LEN - offset);
		len -= (BUF_LEN - offset);
		offset = 0;
	}

	if (fg_debug_mask & FG_MEM_DEBUG_READS) {
		fill_string(str, DEBUG_PRINT_BUFFER_SIZE, val, total_len);
		pr_info("data: %s\n", str);
	}

out:
	user_cnt = atomic_sub_return(1, &chip->memif_user_cnt);
	if (fg_debug_mask & FG_MEM_DEBUG_READS)
		pr_info("user_cnt %d\n", user_cnt);

	fg_assert_sram_access(chip);

	if (!keep_access && (user_cnt == 0) && !rc) {
		rc = fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
				RIF_MEM_ACCESS_REQ, 0, 1);
		if (rc)
			pr_err("failed to set mem access bit\n");
		INIT_COMPLETION(chip->sram_access);
	}

	mutex_unlock(&chip->rw_lock);
	return rc;
}

static int fg_mem_write(struct fg_chip *chip, u8 *val, u16 address,
		unsigned int len, unsigned int offset, bool keep_access)
{
	int rc = 0, user_cnt = 0, orig_address = address;
	u8 *wr_data = val;

	if (address < RAM_OFFSET)
		return -EINVAL;

	if (offset > 3)
		return -EINVAL;

	address = ((orig_address + offset) / 4) * 4;
	offset = (orig_address + offset) % 4;

	user_cnt = atomic_add_return(1, &chip->memif_user_cnt);
	if (fg_debug_mask & FG_MEM_DEBUG_WRITES)
		pr_info("user_cnt %d\n", user_cnt);
	mutex_lock(&chip->rw_lock);
	if (!fg_check_sram_access(chip)) {
		rc = fg_req_and_wait_access(chip, MEM_IF_TIMEOUT_MS);
		if (rc)
			goto out;
	}

	rc = fg_config_access(chip, 1, (len > 4), 0);
	if (rc)
		goto out;

	rc = fg_set_ram_addr(chip, &address);
	if (rc)
		goto out;

	if (fg_debug_mask & FG_MEM_DEBUG_WRITES)
		pr_info("length %d addr=%02X\n", len, address);

	while (len > 0) {
		if (offset)
			rc = fg_write(chip, wr_data,
				chip->mem_base + MEM_INTF_WR_DATA0 + offset,
				(len > 4) ? 4 : len);
		else
			rc = fg_write(chip, wr_data,
				chip->mem_base + MEM_INTF_WR_DATA0,
				(len > 4) ? 4 : len);
		if (rc) {
			pr_err("spmi read failed: addr=%03x, rc=%d\n",
				chip->mem_base + MEM_INTF_RD_DATA0, rc);
			goto out;
		}
		if (offset) {
			wr_data += 4-offset;
			if (len >= 4)
				len -= 4-offset;
			else
				len = 0;
		} else {
			wr_data += 4;
			if (len >= 4)
				len -= 4;
			else
				len = 0;
		}
	}

out:
	user_cnt = atomic_sub_return(1, &chip->memif_user_cnt);
	if (fg_debug_mask & FG_MEM_DEBUG_WRITES)
		pr_info("user_cnt %d\n", user_cnt);

	fg_assert_sram_access(chip);

	if (!keep_access && (user_cnt == 0) && !rc) {
		rc = fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
				RIF_MEM_ACCESS_REQ, 0, 1);
		if (rc) {
			pr_err("failed to set mem access bit\n");
			rc = -EIO;
		}
		INIT_COMPLETION(chip->sram_access);
	}

	mutex_unlock(&chip->rw_lock);
	return rc;
}

static int fg_mem_masked_write(struct fg_chip *chip, u16 addr,
		u8 mask, u8 val, u8 offset)
{
	int rc = 0;
	u8 reg[4];
	char str[DEBUG_PRINT_BUFFER_SIZE];

	rc = fg_mem_read(chip, reg, addr, 4, 0, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}

	reg[offset] &= ~mask;
	reg[offset] |= val & mask;

	str[0] = '\0';
	fill_string(str, DEBUG_PRINT_BUFFER_SIZE, reg, 4);
	pr_debug("Writing %s address %03x, offset %d\n", str, addr, offset);

	rc = fg_mem_write(chip, reg, addr, 4, 0, 0);
	if (rc) {
		pr_err("spmi write failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}

	return rc;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define DEFAULT_CAPACITY 50
#define MISSING_CAPACITY 100
static int get_prop_capacity(struct fg_chip *chip)
{
	u8 cap[2];
	int rc, capacity = 0, tries = 0;

	if (chip->battery_missing)
		return MISSING_CAPACITY;
	if (!chip->profile_loaded && !chip->use_otp_profile)
		return DEFAULT_CAPACITY;

	while (tries < MAX_TRIES_SOC) {
		rc = fg_read(chip, cap,
				chip->soc_base + SOC_MONOTONIC_SOC, 2);
		if (rc) {
			pr_err("spmi read failed: addr=%03x, rc=%d\n",
				chip->soc_base + SOC_MONOTONIC_SOC, rc);
			return rc;
		}

		if (cap[0] == cap[1])
			break;

		tries++;
	}

	if (tries == MAX_TRIES_SOC) {
		pr_err("shadow registers do not match\n");
		return -EINVAL;
	}

	if (cap[0] > 0)
		capacity = (cap[0] * 100 / FULL_PERCENT);

	if (fg_debug_mask & FG_POWER_SUPPLY)
		pr_info_ratelimited("capacity: %d, raw: 0x%02x\n",
				capacity, cap[0]);
	return capacity;
}

#define HIGH_BIAS	3
#define MED_BIAS	BIT(1)
#define LOW_BIAS	BIT(0)
static u8 bias_ua[] = {
	[HIGH_BIAS] = 150,
	[MED_BIAS] = 15,
	[LOW_BIAS] = 5,
};

static int64_t get_batt_id(unsigned int battery_id_uv, u8 bid_info)
{
	u64 battery_id_ohm;

	if (!(bid_info & 0x3) >= 1) {
		pr_err("can't determine battery id %d\n", bid_info);
		return -EINVAL;
	}

	battery_id_ohm = div_u64(battery_id_uv, bias_ua[bid_info & 0x3]);

	return battery_id_ohm;
}

#define DEFAULT_TEMP_DEGC	250
static int get_sram_prop_now(struct fg_chip *chip, unsigned int type)
{
	if (fg_debug_mask & FG_POWER_SUPPLY)
		pr_info("addr 0x%02X, offset %d value %d\n",
			fg_data[type].address, fg_data[type].offset,
			fg_data[type].value);

	if (type == FG_DATA_BATT_ID)
		return get_batt_id(fg_data[type].value,
				fg_data[FG_DATA_BATT_ID_INFO].value);

	return fg_data[type].value;
}

#define MIN_TEMP_DEGC	-300
#define MAX_TEMP_DEGC	970
static int get_prop_jeita_temp(struct fg_chip *chip, unsigned int type)
{
	if (fg_debug_mask & FG_POWER_SUPPLY)
		pr_info("addr 0x%02X, offset %d\n", settings[type].address,
			settings[type].offset);

	return settings[type].value;
}

static int set_prop_jeita_temp(struct fg_chip *chip,
				unsigned int type, int decidegc)
{
	int rc = 0;

	if (fg_debug_mask & FG_POWER_SUPPLY)
		pr_info("addr 0x%02X, offset %d temp%d\n",
			settings[type].address,
			settings[type].offset, decidegc);

	settings[type].value = decidegc;

	cancel_delayed_work_sync(
		&chip->update_jeita_setting);
	schedule_delayed_work(
		&chip->update_jeita_setting, 0);

	return rc;
}

#define EXTERNAL_SENSE_SELECT		0x4AC
#define EXTERNAL_SENSE_OFFSET		0x2
#define EXTERNAL_SENSE_BIT		BIT(2)
static int set_prop_sense_type(struct fg_chip *chip, int ext_sense_type)
{
	int rc;

	rc = fg_mem_masked_write(chip, EXTERNAL_SENSE_SELECT,
			EXTERNAL_SENSE_BIT,
			ext_sense_type ? EXTERNAL_SENSE_BIT : 0,
			EXTERNAL_SENSE_OFFSET);
	if (rc) {
		pr_err("failed to write profile rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#define EXPONENT_MASK		0xF800
#define MANTISSA_MASK		0x3FF
#define SIGN			BIT(10)
#define EXPONENT_SHIFT		11
#define MICRO_UNIT		1000000ULL
static int64_t float_decode(u16 reg)
{
	int64_t final_val, exponent_val, mantissa_val;
	int exponent, mantissa, n;
	bool sign;

	exponent = (reg & EXPONENT_MASK) >> EXPONENT_SHIFT;
	mantissa = (reg & MANTISSA_MASK);
	sign = !!(reg & SIGN);

	pr_debug("exponent=%d mantissa=%d sign=%d\n", exponent, mantissa, sign);

	mantissa_val = mantissa * MICRO_UNIT;

	n = exponent - 15;
	if (n < 0)
		exponent_val = MICRO_UNIT >> -n;
	else
		exponent_val = MICRO_UNIT << n;

	n = n - 10;
	if (n < 0)
		mantissa_val >>= -n;
	else
		mantissa_val <<= n;

	final_val = exponent_val + mantissa_val;

	if (sign)
		final_val *= -1;

	return final_val;
}

#define BATT_IDED	BIT(3)
static int fg_is_batt_id_valid(struct fg_chip *chip)
{
	u8 fg_batt_sts;
	int rc;

	rc = fg_read(chip, &fg_batt_sts,
				 INT_RT_STS(chip->batt_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->batt_base), rc);
		return rc;
	}

	if (fg_debug_mask & FG_IRQS)
		pr_info("fg batt sts 0x%x\n", fg_batt_sts);

	return (fg_batt_sts & BATT_IDED) ? 1 : 0;
}

#define LSB_16B		153
#define LSB_8B		9800
#define TEMP_LSB_16B	625
#define DECIKELVIN	2730
#define SRAM_PERIOD_UPDATE_MS		30000
#define SRAM_PERIOD_NO_ID_UPDATE_MS	100
static void update_sram_data(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work,
				struct fg_chip,
				update_sram_data.work);
	int i, resched_ms, rc = 0;
	u8 reg[2];
	s16 temp;
	int battid_valid = fg_is_batt_id_valid(chip);

	for (i = 0; i < FG_DATA_MAX; i++) {
		rc = fg_mem_read(chip, reg, fg_data[i].address,
			fg_data[i].len, fg_data[i].offset,
			(i+1 == FG_DATA_MAX) ? 0 : 1);
		if (rc) {
			pr_err("Failed to update sram data\n");
			break;
		}

		if (fg_data[i].len > 1)
			temp = reg[0] | (reg[1] << 8);

		switch (i) {
		case FG_DATA_BATT_TEMP:
			fg_data[i].value = (temp * TEMP_LSB_16B / 1000)
				- DECIKELVIN;
			break;
		case FG_DATA_OCV:
		case FG_DATA_VOLTAGE:
			fg_data[i].value = ((u16) temp) * LSB_16B;
			break;
		case FG_DATA_CURRENT:
			fg_data[i].value = temp * LSB_16B;
			break;
		case FG_DATA_BATT_ESR:
			fg_data[i].value = float_decode((u16) temp);
			break;
		case FG_DATA_BATT_ID:
			if (battid_valid)
				fg_data[i].value = reg[0] * LSB_8B;
			break;
		case FG_DATA_BATT_ID_INFO:
			if (battid_valid)
				fg_data[i].value = reg[0];
			break;
		};

		if (fg_debug_mask & FG_MEM_DEBUG_READS)
			pr_info("%d %d %d\n", i, temp, fg_data[i].value);
	}

	if (battid_valid) {
		complete_all(&chip->batt_id_avail);
		resched_ms = SRAM_PERIOD_UPDATE_MS;
	} else {
		resched_ms = SRAM_PERIOD_NO_ID_UPDATE_MS;
	}
	get_current_time(&chip->last_sram_update_time);
	schedule_delayed_work(
		&chip->update_sram_data,
		msecs_to_jiffies(resched_ms));
}

static void update_jeita_setting(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work,
				struct fg_chip,
				update_jeita_setting.work);
	u8 reg[4];
	int i, rc;

	for (i = 0; i < 4; i++)
		reg[i] = (settings[FG_MEM_SOFT_COLD + i].value / 10) + 30;

	rc = fg_mem_write(chip, reg, settings[FG_MEM_SOFT_COLD].address,
			4, settings[FG_MEM_SOFT_COLD].offset, 0);
	if (rc)
		pr_err("failed to update JEITA setting rc=%d\n", rc);
}

#define FG_ALG_SYSCTL_1	0x4B0
#define SYSCTL_OFFSET		1
#define AUTO_RCHG_BIT		BIT(1)

static int fg_set_auto_recharge(struct fg_chip *chip)
{
	int rc;

	rc = fg_mem_masked_write(chip, FG_ALG_SYSCTL_1, AUTO_RCHG_BIT,
			AUTO_RCHG_BIT, SYSCTL_OFFSET);

	if (rc)
		pr_err("write failed rc=%d\n", rc);
	else
		pr_info("setting auto recharge\n");

	return rc;
}

static int fg_set_resume_soc(struct fg_chip *chip, u8 threshold)
{
	u16 address;
	int offset, rc;

	address = settings[FG_MEM_RESUME_SOC].address;
	offset = settings[FG_MEM_RESUME_SOC].offset;

	rc = fg_mem_masked_write(chip, address, 0xFF, threshold, offset);

	if (rc)
		pr_err("write failed rc=%d\n", rc);
	else
		pr_info("setting resume-soc to %x\n", threshold);

	return rc;
}

static enum power_supply_property fg_power_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_COOL_TEMP,
	POWER_SUPPLY_PROP_WARM_TEMP,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
};

static int fg_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct fg_chip *chip = container_of(psy, struct fg_chip, bms_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		val->strval = chip->batt_type;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_sram_prop_now(chip, FG_DATA_CURRENT);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_sram_prop_now(chip, FG_DATA_VOLTAGE);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = get_sram_prop_now(chip, FG_DATA_OCV);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_sram_prop_now(chip, FG_DATA_BATT_TEMP);
		break;
	case POWER_SUPPLY_PROP_COOL_TEMP:
		val->intval = get_prop_jeita_temp(chip, FG_MEM_SOFT_COLD);
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		val->intval = get_prop_jeita_temp(chip, FG_MEM_SOFT_HOT);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = get_sram_prop_now(chip, FG_DATA_BATT_ESR);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = get_sram_prop_now(chip, FG_DATA_BATT_ID);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fg_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct fg_chip *chip = container_of(psy, struct fg_chip, bms_psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_COOL_TEMP:
		rc = set_prop_jeita_temp(chip,
				FG_MEM_SOFT_COLD, val->intval);
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		rc = set_prop_jeita_temp(chip,
				FG_MEM_SOFT_HOT, val->intval);
		break;
	default:
		return -EINVAL;
	};

	return rc;
};

static int fg_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_WARM_TEMP:
		return 1;
	default:
		break;
	}

	return 0;
}

#define SRAM_DUMP_START		0x400
#define SRAM_DUMP_LEN		0x200
static void dump_sram(struct work_struct *work)
{
	int i, rc;
	u8 *buffer;
	char str[16];
	struct fg_chip *chip = container_of(work,
				struct fg_chip,
				dump_sram);

	buffer = devm_kzalloc(chip->dev, SRAM_DUMP_LEN, GFP_KERNEL);
	if (buffer == NULL) {
		pr_err("Can't allocate buffer\n");
		return;
	}

	rc = fg_mem_read(chip, buffer, SRAM_DUMP_START, SRAM_DUMP_LEN, 0, 0);
	if (rc) {
		pr_err("dump failed: rc = %d\n", rc);
		return;
	}

	for (i = 0; i < SRAM_DUMP_LEN; i += 4) {
		str[0] = '\0';
		fill_string(str, DEBUG_PRINT_BUFFER_SIZE, buffer + i, 4);
		pr_info("%03X %s\n", SRAM_DUMP_START + i, str);
	}
	devm_kfree(chip->dev, buffer);
}

#define BATT_MISSING_STS BIT(6)
static bool is_battery_missing(struct fg_chip *chip)
{
	int rc;
	u8 fg_batt_sts;

	rc = fg_read(chip, &fg_batt_sts,
				 INT_RT_STS(chip->batt_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->batt_base), rc);
		return false;
	}

	return (fg_batt_sts & BATT_MISSING_STS) ? true : false;
}

static irqreturn_t fg_batt_missing_irq_handler(int irq, void *_chip)
{
	struct fg_chip *chip = _chip;
	bool batt_missing = is_battery_missing(chip);

	if (batt_missing) {
		chip->battery_missing = true;
		chip->batt_type = missing_batt_type;
	} else {
		if (!chip->use_otp_profile) {
			INIT_COMPLETION(chip->batt_id_avail);
			schedule_work(&chip->batt_profile_init);
			cancel_delayed_work(&chip->update_sram_data);
			schedule_delayed_work(
				&chip->update_sram_data,
				msecs_to_jiffies(0));
		} else {
			chip->battery_missing = false;
		}
	}

	if (fg_debug_mask & FG_IRQS)
		pr_info("batt-missing triggered: %s\n",
				batt_missing ? "missing" : "present");

	if (chip->power_supply_registered)
		power_supply_changed(&chip->bms_psy);
	return IRQ_HANDLED;
}

static irqreturn_t fg_mem_avail_irq_handler(int irq, void *_chip)
{
	struct fg_chip *chip = _chip;
	u8 mem_if_sts;
	int rc;

	rc = fg_read(chip, &mem_if_sts, INT_RT_STS(chip->mem_base), 1);
	if (rc) {
		pr_err("failed to read mem status rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (fg_check_sram_access(chip)) {
		if (fg_debug_mask & FG_IRQS)
			pr_info("sram access granted\n");
		complete_all(&chip->sram_access);
	} else {
		if (fg_debug_mask & FG_IRQS)
			pr_info("sram access revoked\n");
	}

	if (!rc && (fg_debug_mask & FG_IRQS))
		pr_info("mem_if sts 0x%02x\n", mem_if_sts);

	return IRQ_HANDLED;
}

static irqreturn_t fg_soc_irq_handler(int irq, void *_chip)
{
	struct fg_chip *chip = _chip;
	u8 soc_rt_sts;
	int rc;

	rc = fg_read(chip, &soc_rt_sts, INT_RT_STS(chip->soc_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->soc_base), rc);
	}

	if (fg_debug_mask & FG_IRQS)
		pr_info("triggered 0x%x\n", soc_rt_sts);

	if (chip->power_supply_registered)
		power_supply_changed(&chip->bms_psy);
	return IRQ_HANDLED;
}

static irqreturn_t fg_first_soc_irq_handler(int irq, void *_chip)
{
	struct fg_chip *chip = _chip;

	if (fg_debug_mask & FG_IRQS)
		pr_info("triggered\n");

	if (fg_est_dump)
		schedule_work(&chip->dump_sram);

	if (chip->power_supply_registered)
		power_supply_changed(&chip->bms_psy);
	return IRQ_HANDLED;
}

#define OF_READ_SETTING(type, qpnp_dt_property, retval, optional)	\
do {									\
	if (retval)							\
		break;							\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," qpnp_dt_property,	\
					&settings[type].value);		\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		pr_err("Error reading " #qpnp_dt_property		\
				" property rc = %d\n", rc);		\
} while (0)

#define LOW_LATENCY	BIT(6)
#define PROFILE_LOAD_TIMEOUT_MS		5000
#define BATT_PROFILE_OFFSET		0x4C0
#define PROFILE_INTEGRITY_REG		0x53C
#define PROFILE_INTEGRITY_BIT		BIT(0)
#define FIRST_EST_DONE_BIT		BIT(5)
#define MAX_TRIES_FIRST_EST		3
#define FIRST_EST_WAIT_MS		2000
static int fg_batt_profile_init(struct fg_chip *chip)
{
	int rc = 0, ret;
	int len, tries;
	struct device_node *node = chip->spmi->dev.of_node;
	struct device_node *batt_node, *profile_node;
	const char *data, *batt_type_str, *old_batt_type;
	bool tried_again = false;
	u8 reg = 0;

wait:
	ret = wait_for_completion_interruptible_timeout(&chip->batt_id_avail,
			msecs_to_jiffies(PROFILE_LOAD_TIMEOUT_MS));
	/* If we were interrupted wait again one more time. */
	if (ret == -ERESTARTSYS && !tried_again) {
		tried_again = true;
		pr_debug("interrupted, waiting again\n");
		goto wait;
	} else if (ret <= 0) {
		rc = -ETIMEDOUT;
		pr_err("profile loading timed out rc=%d\n", rc);
		return rc;
	}

	batt_node = of_find_node_by_name(node, "qcom,battery-data");
	if (!batt_node) {
		pr_warn("No available batterydata, using OTP defaults\n");
		return 0;
	}

	profile_node = of_batterydata_get_best_profile(batt_node, "bms",
							fg_batt_type);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -ENODATA;
	}

	data = of_get_property(profile_node, "qcom,fg-profile-data", &len);
	if (!data) {
		pr_err("no battery profile loaded\n");
		return 0;
	}

	rc = of_property_read_string(profile_node, "qcom,battery-type",
					&batt_type_str);
	if (rc) {
		pr_err("Could not find battery data type: %d\n", rc);
		return 0;
	}

	if (!chip->batt_profile)
		chip->batt_profile = devm_kzalloc(chip->dev,
				sizeof(char) * len, GFP_KERNEL);

	if (!chip->batt_profile) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	rc = fg_mem_read(chip, &reg, PROFILE_INTEGRITY_REG, 1, 0, 1);
	if (rc) {
		pr_err("failed to read profile integrity rc=%d\n", rc);
		return rc;
	}

	rc = fg_mem_read(chip, chip->batt_profile, BATT_PROFILE_OFFSET,
			len, 0, 1);
	if (rc) {
		pr_err("failed to read profile rc=%d\n", rc);
		return rc;
	}

	if ((reg & PROFILE_INTEGRITY_BIT)
			&& memcmp(chip->batt_profile, data, len - 4) == 0) {
		if (fg_debug_mask & FG_STATUS)
			pr_info("Battery profiles same, using default\n");
		if (fg_est_dump)
			schedule_work(&chip->dump_sram);
		goto done;
	} else {
		if (fg_debug_mask & FG_STATUS) {
			pr_info("Battery profiles differ, using new profile\n");
			print_hex_dump(KERN_INFO, "FG: loaded profile: ",
					DUMP_PREFIX_NONE, 16, 1,
					chip->batt_profile, len, false);
		}
		old_batt_type = chip->batt_type;
		chip->batt_type = loading_batt_type;
		if (chip->power_supply_registered)
			power_supply_changed(&chip->bms_psy);
	}

	memcpy(chip->batt_profile, data, len);

	chip->batt_profile_len = len;

	if (fg_debug_mask & FG_MEM_DEBUG_WRITES)
		print_hex_dump(KERN_INFO, "FG: new profile: ",
				DUMP_PREFIX_NONE, 16, 1, chip->batt_profile,
				chip->batt_profile_len, false);

	/*
	 * release the sram access and configure the correct settings
	 * before re-requesting access.
	 */
	mutex_lock(&chip->rw_lock);
	fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
			RIF_MEM_ACCESS_REQ, 0, 1);
	INIT_COMPLETION(chip->sram_access);

	rc = fg_masked_write(chip, chip->soc_base + SOC_BOOT_MOD,
			NO_OTP_PROF_RELOAD, 0, 1);
	if (rc) {
		pr_err("failed to set no otp reload bit\n");
		goto unlock_and_fail;
	}

	/* unset the restart bits so the fg doesn't continuously restart */
	reg = REDO_FIRST_ESTIMATE | RESTART_GO;
	rc = fg_masked_write(chip, chip->soc_base + SOC_RESTART,
			reg, 0, 1);
	if (rc) {
		pr_err("failed to unset fg restart: %d\n", rc);
		goto unlock_and_fail;
	}

	rc = fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
				LOW_LATENCY, LOW_LATENCY, 1);
	if (rc) {
		pr_err("failed to set low latency access bit\n");
		goto unlock_and_fail;
	}
	mutex_unlock(&chip->rw_lock);

	/* read once to get a fg cycle in */
	rc = fg_mem_read(chip, &reg, PROFILE_INTEGRITY_REG, 1, 0, 0);
	if (rc) {
		pr_err("failed to read profile integrity rc=%d\n", rc);
		goto fail;
	}

	/*
	 * If this is not the first time a profile has been loaded, sleep for
	 * 3 seconds to make sure the NO_OTP_RELOAD is cleared in memory
	 */
	if (chip->profile_loaded)
		msleep(3000);

	mutex_lock(&chip->rw_lock);
	fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
			RIF_MEM_ACCESS_REQ, 0, 1);
	INIT_COMPLETION(chip->sram_access);
	rc = fg_masked_write(chip, chip->mem_base + MEM_INTF_CFG,
				LOW_LATENCY, 0, 1);
	if (rc) {
		pr_err("failed to set low latency access bit\n");
		goto unlock_and_fail;
	}

	atomic_add_return(1, &chip->memif_user_cnt);
	mutex_unlock(&chip->rw_lock);

	/* write the battery profile */
	rc = fg_mem_write(chip, chip->batt_profile, BATT_PROFILE_OFFSET,
			chip->batt_profile_len, 0, 1);
	if (rc) {
		pr_err("failed to write profile rc=%d\n", rc);
		goto sub_and_fail;
	}

	/* write the integrity bits and release access */
	rc = fg_mem_masked_write(chip, PROFILE_INTEGRITY_REG,
			PROFILE_INTEGRITY_BIT, PROFILE_INTEGRITY_BIT, 0);
	if (rc) {
		pr_err("failed to write profile rc=%d\n", rc);
		goto sub_and_fail;
	}

	/* decrement the user count so that memory access can be released */
	fg_release_access_if_necessary(chip);
	/*
	 * set the restart bits so that the next fg cycle will reload
	 * the profile
	 */
	rc = fg_masked_write(chip, chip->soc_base + SOC_BOOT_MOD,
			NO_OTP_PROF_RELOAD, NO_OTP_PROF_RELOAD, 1);
	if (rc) {
		pr_err("failed to set no otp reload bit\n");
		goto fail;
	}

	reg = REDO_FIRST_ESTIMATE | RESTART_GO;
	rc = fg_masked_write(chip, chip->soc_base + SOC_RESTART,
			reg, reg, 1);
	if (rc) {
		pr_err("failed to set fg restart: %d\n", rc);
		goto fail;
	}

	/* wait for the first estimate to complete */
	for (tries = 0; tries < MAX_TRIES_FIRST_EST; tries++) {
		msleep(FIRST_EST_WAIT_MS);

		rc = fg_read(chip, &reg, INT_RT_STS(chip->soc_base), 1);
		if (rc) {
			pr_err("spmi read failed: addr=%03X, rc=%d\n",
					INT_RT_STS(chip->soc_base), rc);
		}
		if (reg & FIRST_EST_DONE_BIT)
			break;
		else
			if (fg_debug_mask & FG_STATUS)
				pr_info("waiting for est, tries = %d\n", tries);
	}
	if ((reg & FIRST_EST_DONE_BIT) == 0)
		pr_err("Battery profile reloading failed, no first estimate\n");

	rc = fg_masked_write(chip, chip->soc_base + SOC_BOOT_MOD,
			NO_OTP_PROF_RELOAD, 0, 1);
	if (rc) {
		pr_err("failed to set no otp reload bit\n");
		goto fail;
	}
	/* unset the restart bits so the fg doesn't continuously restart */
	reg = REDO_FIRST_ESTIMATE | RESTART_GO;
	rc = fg_masked_write(chip, chip->soc_base + SOC_RESTART,
			reg, 0, 1);
	if (rc) {
		pr_err("failed to unset fg restart: %d\n", rc);
		goto fail;
	}
done:
	if (fg_batt_type)
		chip->batt_type = fg_batt_type;
	else
		chip->batt_type = batt_type_str;
	chip->profile_loaded = true;
	chip->battery_missing = is_battery_missing(chip);
	if (chip->power_supply_registered)
		power_supply_changed(&chip->bms_psy);
	return rc;
unlock_and_fail:
	mutex_unlock(&chip->rw_lock);
	goto fail;
sub_and_fail:
	fg_release_access_if_necessary(chip);
	goto fail;
fail:
	chip->batt_type = old_batt_type;
	if (chip->power_supply_registered)
		power_supply_changed(&chip->bms_psy);
	return rc;
}

static void batt_profile_init(struct work_struct *work)
{
	struct fg_chip *chip = container_of(work,
				struct fg_chip,
				batt_profile_init);

	if (fg_batt_profile_init(chip))
		pr_err("failed to initialize profile\n");
}

static int fg_of_init(struct fg_chip *chip)
{
	int rc = 0, sense_type, len = 0;
	const char *data;

	OF_READ_SETTING(FG_MEM_SOFT_HOT, "warm-bat-decidegc", rc, 1);
	OF_READ_SETTING(FG_MEM_SOFT_COLD, "cool-bat-decidegc", rc, 1);
	OF_READ_SETTING(FG_MEM_HARD_HOT, "hot-bat-decidegc", rc, 1);
	OF_READ_SETTING(FG_MEM_HARD_COLD, "cold-bat-decidegc", rc, 1);
	data = of_get_property(chip->spmi->dev.of_node,
			"qcom,thermal-coefficients", &len);
	if (data && len == THERMAL_COEFF_N_BYTES) {
		memcpy(chip->thermal_coefficients, data, len);
		chip->use_thermal_coefficients = true;
	}
	OF_READ_SETTING(FG_MEM_RESUME_SOC, "resume-soc", rc, 1);

	/* Get the use-otp-profile property */
	chip->use_otp_profile = of_property_read_bool(
			chip->spmi->dev.of_node,
			"qcom,use-otp-profile");

	sense_type = of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,ext-sense-type");
	if (rc == 0) {
		if (fg_sense_type < 0)
			fg_sense_type = sense_type;

		if (fg_debug_mask & FG_STATUS) {
			if (fg_sense_type == 0)
				pr_info("Using internal sense\n");
			else if (fg_sense_type == 1)
				pr_info("Using external sense\n");
			else
				pr_info("Using default sense\n");
		}
	} else {
		rc = 0;
	}

	return rc;
}

static int fg_init_irqs(struct fg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("fg: spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		if ((resource->start == chip->vbat_adc_addr) ||
				(resource->start == chip->ibat_adc_addr))
			continue;

		rc = fg_read(chip, &subtype,
				resource->start + REG_OFFSET_PERP_SUBTYPE, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			return rc;
		}

		switch (subtype) {
		case FG_SOC:
			chip->soc_irq[FULL_SOC].irq = spmi_get_irq_byname(
					chip->spmi, spmi_resource, "full-soc");
			if (chip->soc_irq[FULL_SOC].irq < 0) {
				pr_err("Unable to get full-soc irq\n");
				return rc;
			}
			chip->soc_irq[EMPTY_SOC].irq = spmi_get_irq_byname(
					chip->spmi, spmi_resource, "empty-soc");
			if (chip->soc_irq[EMPTY_SOC].irq < 0) {
				pr_err("Unable to get low-soc irq\n");
				return rc;
			}
			chip->soc_irq[DELTA_SOC].irq = spmi_get_irq_byname(
					chip->spmi, spmi_resource, "delta-soc");
			if (chip->soc_irq[DELTA_SOC].irq < 0) {
				pr_err("Unable to get delta-soc irq\n");
				return rc;
			}
			chip->soc_irq[FIRST_EST_DONE].irq = spmi_get_irq_byname(
				chip->spmi, spmi_resource, "first-est-done");
			if (chip->soc_irq[FIRST_EST_DONE].irq < 0) {
				pr_err("Unable to get first-est-done irq\n");
				return rc;
			}

			rc |= devm_request_irq(chip->dev,
				chip->soc_irq[FULL_SOC].irq,
				fg_soc_irq_handler, IRQF_TRIGGER_RISING,
				"full-soc", chip);
			if (rc < 0) {
				pr_err("Can't request %d full-soc: %d\n",
					chip->soc_irq[FULL_SOC].irq, rc);
				return rc;
			}
			rc |= devm_request_irq(chip->dev,
				chip->soc_irq[EMPTY_SOC].irq,
				fg_soc_irq_handler, IRQF_TRIGGER_RISING,
				"empty-soc", chip);
			if (rc < 0) {
				pr_err("Can't request %d empty-soc: %d\n",
					chip->soc_irq[EMPTY_SOC].irq, rc);
				return rc;
			}
			rc |= devm_request_irq(chip->dev,
				chip->soc_irq[DELTA_SOC].irq,
				fg_soc_irq_handler, IRQF_TRIGGER_RISING,
				"delta-soc", chip);
			if (rc < 0) {
				pr_err("Can't request %d delta-soc: %d\n",
					chip->soc_irq[DELTA_SOC].irq, rc);
				return rc;
			}
			rc |= devm_request_irq(chip->dev,
				chip->soc_irq[FIRST_EST_DONE].irq,
				fg_first_soc_irq_handler, IRQF_TRIGGER_RISING,
				"first-est-done", chip);
			if (rc < 0) {
				pr_err("Can't request %d delta-soc: %d\n",
					chip->soc_irq[FIRST_EST_DONE].irq, rc);
				return rc;
			}

			enable_irq_wake(chip->soc_irq[DELTA_SOC].irq);
			enable_irq_wake(chip->soc_irq[FULL_SOC].irq);
			enable_irq_wake(chip->soc_irq[EMPTY_SOC].irq);
			break;
		case FG_MEMIF:
			chip->mem_irq[FG_MEM_AVAIL].irq = spmi_get_irq_byname(
					chip->spmi, spmi_resource, "mem-avail");
			if (chip->mem_irq[FG_MEM_AVAIL].irq < 0) {
				pr_err("Unable to get mem-avail irq\n");
				return rc;
			}
			rc |= devm_request_irq(chip->dev,
					chip->mem_irq[FG_MEM_AVAIL].irq,
					fg_mem_avail_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"mem-avail", chip);
			if (rc < 0) {
				pr_err("Can't request %d mem-avail: %d\n",
					chip->mem_irq[FG_MEM_AVAIL].irq, rc);
				return rc;
			}
			break;
		case FG_BATT:
			chip->batt_irq[BATT_MISSING].irq = spmi_get_irq_byname(
					chip->spmi, spmi_resource,
					"batt-missing");
			if (chip->batt_irq[BATT_MISSING].irq < 0) {
				pr_err("Unable to get batt-missing irq\n");
				rc = -EINVAL;
				return rc;
			}
			rc |= devm_request_irq(chip->dev,
					chip->batt_irq[BATT_MISSING].irq,
					fg_batt_missing_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"batt-missing", chip);
			if (rc < 0) {
				pr_err("Can't request %d batt-missing: %d\n",
					chip->batt_irq[BATT_MISSING].irq, rc);
				return rc;
			}
		case FG_ADC:
			break;
		default:
			pr_err("subtype %d\n", subtype);
			return -EINVAL;
		}
	}

	return rc;
}

static int fg_remove(struct spmi_device *spmi)
{
	struct fg_chip *chip = dev_get_drvdata(&spmi->dev);

	mutex_destroy(&chip->rw_lock);
	cancel_delayed_work_sync(&chip->update_jeita_setting);
	cancel_work_sync(&chip->batt_profile_init);
	cancel_work_sync(&chip->dump_sram);
	power_supply_unregister(&chip->bms_psy);
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static int fg_memif_data_open(struct inode *inode, struct file *file)
{
	struct fg_log_buffer *log;
	struct fg_trans *trans;
	u8 *data_buf;

	size_t logbufsize = SZ_4K;
	size_t databufsize = SZ_4K;

	if (!dbgfs_data.chip) {
		pr_err("Not initialized data\n");
		return -EINVAL;
	}

	/* Per file "transaction" data */
	trans = kzalloc(sizeof(*trans), GFP_KERNEL);
	if (!trans) {
		pr_err("Unable to allocate memory for transaction data\n");
		return -ENOMEM;
	}

	/* Allocate log buffer */
	log = kzalloc(logbufsize, GFP_KERNEL);

	if (!log) {
		kfree(trans);
		pr_err("Unable to allocate memory for log buffer\n");
		return -ENOMEM;
	}

	log->rpos = 0;
	log->wpos = 0;
	log->len = logbufsize - sizeof(*log);

	/* Allocate data buffer */
	data_buf = kzalloc(databufsize, GFP_KERNEL);

	if (!data_buf) {
		kfree(trans);
		kfree(log);
		pr_err("Unable to allocate memory for data buffer\n");
		return -ENOMEM;
	}

	trans->log = log;
	trans->data = data_buf;
	trans->cnt = dbgfs_data.cnt;
	trans->addr = dbgfs_data.addr;
	trans->chip = dbgfs_data.chip;
	trans->offset = trans->addr;
	mutex_init(&trans->memif_dfs_lock);

	file->private_data = trans;
	return 0;
}

static int fg_memif_dfs_close(struct inode *inode, struct file *file)
{
	struct fg_trans *trans = file->private_data;

	if (trans && trans->log && trans->data) {
		file->private_data = NULL;
		mutex_destroy(&trans->memif_dfs_lock);
		kfree(trans->log);
		kfree(trans->data);
		kfree(trans);
	}

	return 0;
}

/**
 * print_to_log: format a string and place into the log buffer
 * @log: The log buffer to place the result into.
 * @fmt: The format string to use.
 * @...: The arguments for the format string.
 *
 * The return value is the number of characters written to @log buffer
 * not including the trailing '\0'.
 */
static int print_to_log(struct fg_log_buffer *log, const char *fmt, ...)
{
	va_list args;
	int cnt;
	char *buf = &log->data[log->wpos];
	size_t size = log->len - log->wpos;

	va_start(args, fmt);
	cnt = vscnprintf(buf, size, fmt, args);
	va_end(args);

	log->wpos += cnt;
	return cnt;
}

/**
 * write_next_line_to_log: Writes a single "line" of data into the log buffer
 * @trans: Pointer to SRAM transaction data.
 * @offset: SRAM address offset to start reading from.
 * @pcnt: Pointer to 'cnt' variable.  Indicates the number of bytes to read.
 *
 * The 'offset' is a 12-bit SRAM address.
 *
 * On a successful read, the pcnt is decremented by the number of data
 * bytes read from the SRAM.  When the cnt reaches 0, all requested bytes have
 * been read.
 */
static int
write_next_line_to_log(struct fg_trans *trans, int offset, size_t *pcnt)
{
	int i, j;
	u8 data[ITEMS_PER_LINE];
	struct fg_log_buffer *log = trans->log;

	int cnt = 0;
	int padding = offset % ITEMS_PER_LINE;
	int items_to_read = min(ARRAY_SIZE(data) - padding, *pcnt);
	int items_to_log = min(ITEMS_PER_LINE, padding + items_to_read);

	/* Buffer needs enough space for an entire line */
	if ((log->len - log->wpos) < MAX_LINE_LENGTH)
		goto done;

	memcpy(data, trans->data + (offset - trans->addr), items_to_read);

	*pcnt -= items_to_read;

	/* Each line starts with the aligned offset (12-bit address) */
	cnt = print_to_log(log, "%3.3X ", offset & 0xfff);
	if (cnt == 0)
		goto done;

	/* If the offset is unaligned, add padding to right justify items */
	for (i = 0; i < padding; ++i) {
		cnt = print_to_log(log, "-- ");
		if (cnt == 0)
			goto done;
	}

	/* Log the data items */
	for (j = 0; i < items_to_log; ++i, ++j) {
		cnt = print_to_log(log, "%2.2X ", data[j]);
		if (cnt == 0)
			goto done;
	}

	/* If the last character was a space, then replace it with a newline */
	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

done:
	return cnt;
}

/**
 * get_log_data - reads data from SRAM and saves to the log buffer
 * @trans: Pointer to SRAM transaction data.
 *
 * Returns the number of "items" read or SPMI error code for read failures.
 */
static int get_log_data(struct fg_trans *trans)
{
	int cnt, rc;
	int last_cnt;
	int items_read;
	int total_items_read = 0;
	u32 offset = trans->offset;
	size_t item_cnt = trans->cnt;
	struct fg_log_buffer *log = trans->log;

	if (item_cnt == 0)
		return 0;

	if (item_cnt > SZ_4K) {
		pr_err("Reading too many bytes\n");
		return -EINVAL;
	}

	rc = fg_mem_read(trans->chip, trans->data,
			trans->addr, trans->cnt, 0, 0);
	if (rc) {
		pr_err("dump failed: rc = %d\n", rc);
		return rc;
	}
	/* Reset the log buffer 'pointers' */
	log->wpos = log->rpos = 0;

	/* Keep reading data until the log is full */
	do {
		last_cnt = item_cnt;
		cnt = write_next_line_to_log(trans, offset, &item_cnt);
		items_read = last_cnt - item_cnt;
		offset += items_read;
		total_items_read += items_read;
	} while (cnt && item_cnt > 0);

	/* Adjust the transaction offset and count */
	trans->cnt = item_cnt;
	trans->offset += total_items_read;

	return total_items_read;
}

/**
 * fg_memif_dfs_reg_read: reads value(s) from SRAM and fills user's buffer a
 *  byte array (coded as string)
 * @file: file pointer
 * @buf: where to put the result
 * @count: maximum space available in @buf
 * @ppos: starting position
 * @return number of user bytes read, or negative error value
 */
static ssize_t fg_memif_dfs_reg_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct fg_trans *trans = file->private_data;
	struct fg_log_buffer *log = trans->log;
	size_t ret;
	size_t len;

	mutex_lock(&trans->memif_dfs_lock);
	/* Is the the log buffer empty */
	if (log->rpos >= log->wpos) {
		if (get_log_data(trans) <= 0) {
			len = 0;
			goto unlock_mutex;
		}
	}

	len = min(count, log->wpos - log->rpos);

	ret = copy_to_user(buf, &log->data[log->rpos], len);
	if (ret == len) {
		pr_err("error copy sram register values to user\n");
		len = -EFAULT;
		goto unlock_mutex;
	}

	/* 'ret' is the number of bytes not copied */
	len -= ret;

	*ppos += len;
	log->rpos += len;

unlock_mutex:
	mutex_unlock(&trans->memif_dfs_lock);
	return len;
}

/**
 * fg_memif_dfs_reg_write: write user's byte array (coded as string) to SRAM.
 * @file: file pointer
 * @buf: user data to be written.
 * @count: maximum space available in @buf
 * @ppos: starting position
 * @return number of user byte written, or negative error value
 */
static ssize_t fg_memif_dfs_reg_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	int bytes_read;
	int data;
	int pos = 0;
	int cnt = 0;
	u8  *values;
	size_t ret = 0;
	char *kbuf;
	u32 offset;

	struct fg_trans *trans = file->private_data;

	mutex_lock(&trans->memif_dfs_lock);
	offset = trans->offset;

	/* Make a copy of the user data */
	kbuf = kmalloc(count + 1, GFP_KERNEL);
	if (!kbuf) {
		ret = -ENOMEM;
		goto unlock_mutex;
	}

	ret = copy_from_user(kbuf, buf, count);
	if (ret == count) {
		pr_err("failed to copy data from user\n");
		ret = -EFAULT;
		goto free_buf;
	}

	count -= ret;
	*ppos += count;
	kbuf[count] = '\0';

	/* Override the text buffer with the raw data */
	values = kbuf;

	/* Parse the data in the buffer.  It should be a string of numbers */
	while (sscanf(kbuf + pos, "%i%n", &data, &bytes_read) == 1) {
		pos += bytes_read;
		values[cnt++] = data & 0xff;
	}

	if (!cnt)
		goto free_buf;

	pr_info("address %x, count %d\n", offset, cnt);
	/* Perform the write(s) */

	ret = fg_mem_write(trans->chip, values, offset,
				cnt, 0, 0);
	if (ret) {
		pr_err("SPMI write failed, err = %zu\n", ret);
	} else {
		ret = count;
		trans->offset += cnt > 4 ? 4 : cnt;
	}

free_buf:
	kfree(kbuf);
unlock_mutex:
	mutex_unlock(&trans->memif_dfs_lock);
	return ret;
}

static const struct file_operations fg_memif_dfs_reg_fops = {
	.open		= fg_memif_data_open,
	.release	= fg_memif_dfs_close,
	.read		= fg_memif_dfs_reg_read,
	.write		= fg_memif_dfs_reg_write,
};

/**
 * fg_dfs_create_fs: create debugfs file system.
 * @return pointer to root directory or NULL if failed to create fs
 */
static struct dentry *fg_dfs_create_fs(void)
{
	struct dentry *root, *file;

	pr_debug("Creating FG_MEM debugfs file-system\n");
	root = debugfs_create_dir(DFS_ROOT_NAME, NULL);
	if (IS_ERR_OR_NULL(root)) {
		pr_err("Error creating top level directory err:%ld",
			(long)root);
		if (PTR_ERR(root) == -ENODEV)
			pr_err("debugfs is not enabled in the kernel");
		return NULL;
	}

	dbgfs_data.help_msg.size = strlen(dbgfs_data.help_msg.data);

	file = debugfs_create_blob("help", S_IRUGO, root, &dbgfs_data.help_msg);
	if (!file) {
		pr_err("error creating help entry\n");
		goto err_remove_fs;
	}
	return root;

err_remove_fs:
	debugfs_remove_recursive(root);
	return NULL;
}

/**
 * fg_dfs_get_root: return a pointer to FG debugfs root directory.
 * @return a pointer to the existing directory, or if no root
 * directory exists then create one. Directory is created with file that
 * configures SRAM transaction, namely: address, and count.
 * @returns valid pointer on success or NULL
 */
struct dentry *fg_dfs_get_root(void)
{
	if (dbgfs_data.root)
		return dbgfs_data.root;

	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return NULL;
	/* critical section */
	if (!dbgfs_data.root) { /* double checking idiom */
		dbgfs_data.root = fg_dfs_create_fs();
	}
	mutex_unlock(&dbgfs_data.lock);
	return dbgfs_data.root;
}

/*
 * fg_dfs_create: adds new fg_mem if debugfs entry
 * @return zero on success
 */
int fg_dfs_create(struct fg_chip *chip)
{
	struct dentry *root;
	struct dentry *file;

	root = fg_dfs_get_root();
	if (!root)
		return -ENOENT;

	dbgfs_data.chip = chip;

	file = debugfs_create_u32("count", DFS_MODE, root, &(dbgfs_data.cnt));
	if (!file) {
		pr_err("error creating 'count' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_x32("address", DFS_MODE,
			root, &(dbgfs_data.addr));
	if (!file) {
		pr_err("error creating 'address' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data", DFS_MODE, root, &dbgfs_data,
							&fg_memif_dfs_reg_fops);
	if (!file) {
		pr_err("error creating 'data' entry\n");
		goto err_remove_fs;
	}

	return 0;

err_remove_fs:
	debugfs_remove_recursive(root);
	return -ENOMEM;
}

static int soc_to_setpoint(int soc)
{
	return DIV_ROUND_CLOSEST(soc * 255, 100);
}

#define SOC_CNFG	0x450
#define SOC_DELTA_OFFSET	3
#define DELTA_SOC_PERCENT	1
#define BATT_TEMP_OFFSET	3
#define BATT_TEMP_CNTRL_MASK	0x17
#define BATT_TEMP_ON		0x16
#define BATT_TEMP_OFF		0x01
#define THERMAL_COEFF_ADDR	0x444
#define THERMAL_COEFF_OFFSET	0x2
static int fg_hw_init(struct fg_chip *chip)
{
	u8 resume_soc;
	int rc = 0;

	rc = fg_set_auto_recharge(chip);
	if (rc) {
		pr_err("Couldn't set auto recharge in FG\n");
		return rc;
	}

	resume_soc = settings[FG_MEM_RESUME_SOC].value;
	if (resume_soc > 0) {
		resume_soc = resume_soc * 255 / 100;
		rc = fg_set_resume_soc(chip, resume_soc);
		if (rc) {
			pr_err("Couldn't set resume SOC for FG\n");
			return rc;
		}
	} else {
		pr_info("FG auto recharge threshold not specified in DT\n");
	}

	if (fg_sense_type >= 0) {
		rc = set_prop_sense_type(chip, fg_sense_type);
		if (rc) {
			pr_err("failed to config sense type %d rc=%d\n",
					fg_sense_type, rc);
			return rc;
		}
	}

	rc = fg_mem_masked_write(chip, EXTERNAL_SENSE_SELECT,
			BATT_TEMP_CNTRL_MASK,
			BATT_TEMP_ON,
			BATT_TEMP_OFFSET);
	if (rc) {
		pr_err("failed to write to memif rc=%d\n", rc);
		return rc;
	}

	rc = fg_mem_masked_write(chip, SOC_CNFG,
			0xFF,
			soc_to_setpoint(DELTA_SOC_PERCENT),
			SOC_DELTA_OFFSET);
	if (rc) {
		pr_err("failed to write to memif rc=%d\n", rc);
		return rc;
	}

	if (chip->use_thermal_coefficients) {
		fg_mem_write(chip, chip->thermal_coefficients,
			THERMAL_COEFF_ADDR, 2,
			THERMAL_COEFF_OFFSET, 0);
		fg_mem_write(chip, chip->thermal_coefficients + 2,
			THERMAL_COEFF_ADDR + 4, 4, 0, 0);
	}

	return 0;
}

#define INIT_JEITA_DELAY_MS 1000
static int fg_probe(struct spmi_device *spmi)
{
	struct device *dev = &(spmi->dev);
	struct fg_chip *chip;
	struct spmi_resource *spmi_resource;
	struct resource *resource;
	u8 subtype, reg;
	int rc = 0;

	if (!spmi) {
		pr_err("no valid spmi pointer\n");
		return -ENODEV;
	}

	if (!spmi->dev.of_node) {
		pr_err("device node missing\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(dev, sizeof(struct fg_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("Can't allocate fg_chip\n");
		return -ENOMEM;
	}

	chip->spmi = spmi;
	chip->dev = &(spmi->dev);

	mutex_init(&chip->rw_lock);
	INIT_DELAYED_WORK(&chip->update_jeita_setting,
			update_jeita_setting);
	INIT_DELAYED_WORK(&chip->update_sram_data, update_sram_data);
	INIT_WORK(&chip->batt_profile_init,
			batt_profile_init);
	INIT_WORK(&chip->dump_sram, dump_sram);
	init_completion(&chip->sram_access);
	init_completion(&chip->batt_id_avail);
	dev_set_drvdata(&spmi->dev, chip);

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("qpnp_chg: spmi resource absent\n");
			rc = -ENXIO;
			goto of_init_fail;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto of_init_fail;
		}

		if (strcmp("qcom,fg-adc-vbat",
					spmi_resource->of_node->name) == 0) {
			chip->vbat_adc_addr = resource->start;
			continue;
		} else if (strcmp("qcom,fg-adc-ibat",
					spmi_resource->of_node->name) == 0) {
			chip->ibat_adc_addr = resource->start;
			continue;
		}

		rc = fg_read(chip, &subtype,
				resource->start + REG_OFFSET_PERP_SUBTYPE, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto of_init_fail;
		}

		switch (subtype) {
		case FG_SOC:
			chip->soc_base = resource->start;
			break;
		case FG_MEMIF:
			chip->mem_base = resource->start;
			break;
		case FG_BATT:
			chip->batt_base = resource->start;
			break;
		default:
			pr_err("Invalid peripheral subtype=0x%x\n", subtype);
			rc = -EINVAL;
		}
	}

	rc = fg_of_init(chip);
	if (rc) {
		pr_err("failed to parse devicetree rc%d\n", rc);
		goto of_init_fail;
	}

	reg = 0xFF;
	rc = fg_write(chip, &reg, INT_EN_CLR(chip->mem_base), 1);
	if (rc) {
		pr_err("failed to clear interrupts %d\n", rc);
		goto of_init_fail;
	}

	/* hold memory access until initialization finishes */
	atomic_add_return(1, &chip->memif_user_cnt);

	rc = fg_init_irqs(chip);
	if (rc) {
		pr_err("failed to request interrupts %d\n", rc);
		goto power_supply_unregister;
	}

	chip->batt_type = default_batt_type;

	chip->bms_psy.name = "bms";
	chip->bms_psy.type = POWER_SUPPLY_TYPE_BMS;
	chip->bms_psy.properties = fg_power_props;
	chip->bms_psy.num_properties = ARRAY_SIZE(fg_power_props);
	chip->bms_psy.get_property = fg_power_get_property;
	chip->bms_psy.set_property = fg_power_set_property;
	chip->bms_psy.supplied_to = fg_supplicants;
	chip->bms_psy.num_supplicants = ARRAY_SIZE(fg_supplicants);
	chip->bms_psy.property_is_writeable = fg_property_is_writeable;

	rc = power_supply_register(chip->dev, &chip->bms_psy);
	if (rc < 0) {
		pr_err("batt failed to register rc = %d\n", rc);
		goto of_init_fail;
	}
	chip->power_supply_registered = true;

	if (chip->mem_base) {
		rc = fg_dfs_create(chip);
		if (rc < 0) {
			pr_err("failed to create debugfs rc = %d\n", rc);
			goto power_supply_unregister;
		}
	}

	schedule_delayed_work(
		&chip->update_jeita_setting,
		msecs_to_jiffies(INIT_JEITA_DELAY_MS));

	if (chip->last_sram_update_time == 0)
		update_sram_data(&chip->update_sram_data.work);

	rc = fg_hw_init(chip);
	if (rc) {
		pr_err("failed to hw init rc = %d\n", rc);
		goto cancel_jeita_work;
	}

	/* release memory access if necessary */
	fg_release_access_if_necessary(chip);

	if (!chip->use_otp_profile)
		schedule_work(&chip->batt_profile_init);

	pr_info("probe success\n");

	return rc;

cancel_jeita_work:
	cancel_delayed_work_sync(&chip->update_jeita_setting);
power_supply_unregister:
	power_supply_unregister(&chip->bms_psy);
of_init_fail:
	mutex_destroy(&chip->rw_lock);
	return rc;
}

static int fg_suspend(struct device *dev)
{
	struct fg_chip *chip = dev_get_drvdata(dev);
	ktime_t  enter_time;
	ktime_t  total_time;
	int total_time_ms;
	int rc;

	enter_time = ktime_get();
	rc = fg_mem_masked_write(chip, EXTERNAL_SENSE_SELECT,
			BATT_TEMP_CNTRL_MASK,
			BATT_TEMP_OFF,
			BATT_TEMP_OFFSET);
	if (rc)
		pr_err("failed to write to memif rc=%d\n", rc);

	total_time = ktime_sub(ktime_get(), enter_time);
	total_time_ms = ktime_to_ms(total_time);

	if ((total_time_ms > 1500) && (fg_debug_mask & FG_STATUS))
		pr_info("spent %dms configuring rbias\n", total_time_ms);

	return 0;
}

static int fg_resume(struct device *dev)
{
	struct fg_chip *chip = dev_get_drvdata(dev);
	ktime_t  enter_time;
	ktime_t  total_time;
	int total_time_ms;
	unsigned long current_time = 0, next_update_time, time_left;
	int rc;

	enter_time = ktime_get();
	rc = fg_mem_masked_write(chip, EXTERNAL_SENSE_SELECT,
			BATT_TEMP_CNTRL_MASK,
			BATT_TEMP_ON,
			BATT_TEMP_OFFSET);
	if (rc)
		pr_err("failed to write to memif rc=%d\n", rc);
	total_time = ktime_sub(ktime_get(), enter_time);
	total_time_ms = ktime_to_ms(total_time);

	if ((total_time_ms > 1500) && (fg_debug_mask & FG_STATUS))
		pr_info("spent %dms configuring rbias\n", total_time_ms);

	/*
	 * this may fail, but current time should be still 0,
	 * triggering an immediate update.
	 */
	get_current_time(&current_time);

	next_update_time = chip->last_sram_update_time
		+ (SRAM_PERIOD_UPDATE_MS / 1000);
	if (next_update_time > current_time)
		time_left = next_update_time - current_time;
	else
		time_left = 0;

	cancel_delayed_work_sync(
		&chip->update_sram_data);
	schedule_delayed_work(
		&chip->update_sram_data, msecs_to_jiffies(time_left * 1000));
	return 0;
}

static const struct dev_pm_ops qpnp_fg_pm_ops = {
	.suspend	= fg_suspend,
	.resume		= fg_resume,
};

static int fg_sense_type_set(const char *val, const struct kernel_param *kp)
{
	int rc;
	struct power_supply *bms_psy;
	struct fg_chip *chip;
	int old_fg_sense_type = fg_sense_type;

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("Unable to set fg_sense_type: %d\n", rc);
		return rc;
	}

	if (fg_sense_type != 0 && fg_sense_type != 1) {
		pr_err("Bad value %d\n", fg_sense_type);
		fg_sense_type = old_fg_sense_type;
		return -EINVAL;
	}

	if (fg_debug_mask & FG_STATUS)
		pr_info("fg_sense_type set to %d\n", fg_sense_type);

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("bms psy not found\n");
		return 0;
	}

	chip = container_of(bms_psy, struct fg_chip, bms_psy);
	rc = set_prop_sense_type(chip, fg_sense_type);
	return rc;
}

static struct kernel_param_ops fg_sense_type_ops = {
	.set = fg_sense_type_set,
	.get = param_get_int,
};

module_param_cb(sense_type, &fg_sense_type_ops, &fg_sense_type, 0644);

static struct spmi_driver fg_driver = {
	.driver		= {
		.name	= QPNP_FG_DEV_NAME,
		.of_match_table	= fg_match_table,
		.pm	= &qpnp_fg_pm_ops,
	},
	.probe		= fg_probe,
	.remove		= fg_remove,
};

static int __init fg_init(void)
{
	return spmi_driver_register(&fg_driver);
}

static void __exit fg_exit(void)
{
	return spmi_driver_unregister(&fg_driver);
}

module_init(fg_init);
module_exit(fg_exit);

MODULE_DESCRIPTION("QPNP Fuel Gauge Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_FG_DEV_NAME);
