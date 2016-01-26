/*
 * A driver for the mini isp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/ctype.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/config_interface.h>
#include <linux/vmalloc.h>
#include <linux/random.h>
#include "mini_isp.h"
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>


#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "mini_isp"
#endif

#define MINI_ISP_SPI_SPEED_BOOT		(12000000)
#define MINI_ISP_SPI_SPEED_NORMAL	(40000000)
#define MINI_ISP_WAIT_TIMEOUT	(40)
#define MINI_ISP_WAIT_TIMEOUT_STREAMOFF (HZ*2)
#define MINI_ISP_POLL_TIMES		(20)
#define MINI_ISP_MAX_ERR_CODE	(10)
#define MINI_ISP_CMD_DELAY_US	(2500)

#define MINI_ISP_FW_LOG_BUF		(4096)

#define SPI_TX_BUF_SIZE			(64)
#define SPI_RX_BUF_SIZE			(64)
/*if want to change ALLOC_PAGES_SIZE or SPI_BLOCK_BUF_SIZE,just change ALLOC_GET_ORDER_SIZE is ok*/
/*ALLOC_GET_ORDER_SIZE can be 1,2,4,8.....*/
#define ALLOC_GET_ORDER_SIZE	256
/*get free pages size of tx_buf*/
#define ALLOC_PAGES_SIZE		get_order(ALLOC_GET_ORDER_SIZE*PAGE_SIZE)
/*spi block buf size*/
#define SPI_BLOCK_BUF_SIZE		(4096*ALLOC_GET_ORDER_SIZE)	
#define ISPCMD_DUMMYBYTES		(4)
#define ISPCMD_LENFLDBYTES		(2)
#define ISPCMD_OPCODEBYTES		(2)
#define ISPCMD_CKSUMBYTES		(2) 
#define ISPCMD_HDRSIZE			(ISPCMD_LENFLDBYTES+ISPCMD_OPCODEBYTES)
#define ISPCMD_HDRSIZEWDUMMY	(ISPCMD_LENFLDBYTES+ISPCMD_OPCODEBYTES+ISPCMD_DUMMYBYTES)
/*delete the static whole stack*/
//static unsigned char fw_buf[1024*1024]={0};
enum msm_dt_entry_type {
DT_U32,
DT_GPIO,
DT_BOOL,
};
/*dt map struct getting config from dts*/
struct msm_dt_to_pdata_map {
	const char                  *dt_name;
	void                        *ptr_data;
	enum msm_dt_entry_type   type;
	int                          default_val;
};

struct misp_plat_data {
	int spi_cs_gpio;
	int irq_gpio;
	int reset_gpio;
	int power_gpio;
};

struct misp_fw_ver_info {
	char	info[16];
	u32		product;
	u16		major;
	u16		minor;
	char	user[9];
};

struct misp_data {
	struct spi_device	*spi;
	struct mutex		busy_lock;
	struct misp_plat_data	*plat_data;
	struct workqueue_struct	*work_queue;
	wait_queue_head_t	wait_queue;
	wait_queue_head_t	sync_queue;
	wait_queue_head_t	pwdn_queue;
	/*add camera ois driver*/
	wait_queue_head_t   ois_queue;
	/*mini-isp do the ois init*/
	int ois_init_status;
	int		state;
	u32		last_error_code[MINI_ISP_MAX_ERR_CODE];
	/*
	 * NOTE: All buffers should be dma-safe buffers
	 * ext_buf:used only for 1-byte ctrl_byte in font of tx or rx
	 * tx_buf :used for 64-Bytes CMD-send or 8K-Bytes Block-send
	 * rx_buf :used for 64-Bytes CMD-recv or 4K-Bytes Block-recv
	 */
	u8		*tx_buf;
	u8		*rx_buf;
	u8		*ext_buf;
	/*
	 * NOTE:
	 * all these tx/rx/ext buffers above used with mutex lock, but
	 * irq cannot use mutex lock, so a special buffer is supplied
	 */
	u8		*irq_buf;
	/* debug config & debug info */
	u32		debug_config;
	u32		debug_log_level;
	struct work_struct	dump_err_work;
	struct work_struct	dump_log_work;
	/*fix ois running test fail issue*/
	struct work_struct  ois_init_done_work;
	struct misp_fw_ver_info fw_info;
	struct misp_fw_ver_info boot_info;
	u32 	flush_reg_flag;
};

typedef enum {
    USPICTRL_MS_CB_ORG                  = (0x00<<6),    /*!< Ctrl-Byte, original command */
    USPICTRL_MS_CB_STS                  = (0x01<<6),    /*!< Ctrl-Byte, polling status */
    USPICTRL_MS_CB_RSP                  = (0x02<<6),    /*!< Ctrl-Byte, get response */
    USPICTRL_MS_CB_DIS                  = (0x03<<6),    /*!< Ctrl-Byte, disable Ctrl-Byte mode */

    USPICTRL_MS_ORG                     = 0x5A,     /*!< Ctrl-Byte, original command */
    USPICTRL_MS_RSP                     = 0xA5,     /*!< Ctrl-Byte, get response */
    USPICTRL_MS_BK0                     = 0xB5,     /*!< Ctrl-Byte, send bulk for 1~(N-1) one */
    USPICTRL_MS_BKN                     = 0xBB,     /*!< Ctrl-Byte, senc bulk for last one */
    USPICTRL_MS_DIS                     = 0x05,     /*!< Ctrl-Byte, disable get response */
} USPICTRL_MS_CB_ID;
//delete here
enum {
	MINI_ISP_IRQ_READY_DONE			= (1U<<0),
	MINI_ISP_IRQ_CMD_ERR			= (1U<<1),
	MINI_ISP_IRQ_OTHRE_ERR			= (1U<<2),
	MINI_ISP_IRQ_LOG_FULL			= (1U<<3),
	MINI_ISP_IRQ_SYNC				= (1U<<4),
	MINI_ISP_IRQ_PWDN				= (1U<<5),
	/*mini-isp do the ois init*/
	MINI_ISP_IRQ_OIS_INITDONE       = (1U<<6),
	/*add camera ois driver*/
	MINI_ISP_IRQ_OIS                = (1U<<7), //0x0080 
};

enum {
	MINI_ISP_DEBUG_SPI_DATA			= (1U<<0),
	MINI_ISP_DEBUG_LOG_FILE			= (1U<<1),
};

enum {
	MINI_ISP_LOG_LEVEL_NONE			= (0x0),
	MINI_ISP_LOG_LEVEL_SW1			= (0x00000001),
	MINI_ISP_LOG_LEVEL_SW2			= (0x00000002),
	MINI_ISP_LOG_LEVEL_SW3			= (0x00000004),
	MINI_ISP_LOG_LEVEL_IQ1			= (0x00000010),
	MINI_ISP_LOG_LEVEL_IQ2			= (0x00000020),
	MINI_ISP_LOG_LEVEL_IQ3			= (0x00000040),
	MINI_ISP_LOG_LEVEL_ISR			= (0x00000100),
};

enum {
	MINI_ISP_FLUSH_REG_NONE			= (0U),
	MINI_ISP_FLUSH_REG_ALREADY		= (1U),
};

static struct misp_data *misp_drv_data;
static u32 misp_al6010_reg_table [] =
{    
// Rx0
// Start = 0xfffa1000, End = 0xfffa11f4
    0xfffa1000,
    0xfffa1028,
    0xfffa1050,
    0xfffa1078,
    0xfffa10a0,
    0xfffa10c8,
    0xfffa10f0,
    0xfffa1118,
    0xfffa1140,
    0xfffa1168,
    0xfffa1190,
    0xfffa11b8,
    0xfffa11e0,
    
// Rx1
// Start = 0xfffa2000, End = 0xfffa21f4
    0xfffa2000,
    0xfffa2028,
    0xfffa2050,
    0xfffa2078,
    0xfffa20a0,
    0xfffa20c8,
    0xfffa20f0,
    0xfffa2118,
    0xfffa2140,
    0xfffa2168,
    0xfffa2190,
    0xfffa21b8,
    0xfffa21e0,
    
// Rx0_phy
// Start = 0xfffa5000, End = 0xfffa510c
    0xfffa5000,
    0xfffa5028,
    0xfffa5050,
    0xfffa5078,
    0xfffa50a0,
    0xfffa50c8,
    0xfffa50f0,
    
// Rx1_phy
// Start = 0xfffa0000, End = 0xfffa010c
    0xfffa0000,
    0xfffa0028,
    0xfffa0050,
    0xfffa0078,
    0xfffa00a0,
    0xfffa00c8,
    0xfffa00f0,
    
// RxLS
// Start = 0xfffa9000, End = 0xfffa9164
    0xfffa9000,
    0xfffa9028,
    0xfffa9050,
    0xfffa9078,
    0xfffa90a0,
    0xfffa90c8,
    0xfffa90f0,
    0xfffa9118,
    0xfffa9140,
    
// TxLM_a
// Start = 0xffede000, End = 0xffede394
    0xffede000,
    0xffede028,
    0xffede050,
    0xffede078,
    0xffede0a0,
    0xffede0c8,
    0xffede0f0,
    0xffede118,
    0xffede140,
    0xffede168,
    0xffede190,
    0xffede1b8,
    0xffede1e0,
    0xffede208,
    0xffede230,
    0xffede258,
    0xffede280,
    0xffede2a8,
    0xffede2d0,
    0xffede2f8,
    0xffede320,
    0xffede348,
    0xffede370,
    
// TxLM_b
// Start = 0xffedd000, End = 0xffedd394
    0xffedd000,
    0xffedd028,
    0xffedd050,
    0xffedd078,
    0xffedd0a0,
    0xffedd0c8,
    0xffedd0f0,
    0xffedd118,
    0xffedd140,
    0xffedd168,
    0xffedd190,
    0xffedd1b8,
    0xffedd1e0,
    0xffedd208,
    0xffedd230,
    0xffedd258,
    0xffedd280,
    0xffedd2a8,
    0xffedd2d0,
    0xffedd2f8,
    0xffedd320,
    0xffedd348,
    0xffedd370,
    
// TxTop
// Start = 0xfffae000, End = 0xfffae16c
    0xfffae000,
    0xfffae028,
    0xfffae050,
    0xfffae078,
    0xfffae0a0,
    0xfffae0c8,
    0xfffae0f0,
    0xfffae118,
    0xfffae140,
    0xfffae168,
    
// Tx
// Start = 0xffeb0000, End = 0xffeb01e4
    0xffeb0000,
    0xffeb0028,
    0xffeb0050,
    0xffeb0078,
    0xffeb00a0,
    0xffeb00c8,
    0xffeb00f0,
    0xffeb0118,
    0xffeb0140,
    0xffeb0168,
    0xffeb0190,
    0xffeb01b8,
    0xffeb01e0,
    
// Tx_Phy
// Start = 0xffeb2000, End = 0xffeb2104
    0xffeb2000,
    0xffeb2028,
    0xffeb2050,
    0xffeb2078,
    0xffeb20a0,
    0xffeb20c8,
    0xffeb20f0,
    
// RawTop1
// Start = 0xfffab000, End = 0xfffab038
    0xfffab000,
    0xfffab028,
    
// RawTop2
// Start = 0xfffab100, End = 0xfffab11c
    0xfffab100,
    
// RawTop3
// Start = 0xfffab150, End = 0xfffab154
    0xfffab150,
    
// CLK1
// Start = 0xffe80200, End = 0xffe80a04
    0xffe80200,
    0xffe80228,
    0xffe80250,
    0xffe80278,
    0xffe802a0,
    0xffe802c8,
    0xffe802f0,
    0xffe80318,
    0xffe80340,
    0xffe80368,
    0xffe80390,
    0xffe803b8,
    0xffe803e0,
    0xffe80408,
    0xffe80430,
    0xffe80458,
    0xffe80480,
    0xffe804a8,
    0xffe804d0,
    0xffe804f8,
    0xffe80520,
    0xffe80548,
    0xffe80570,
    0xffe80598,
    0xffe805c0,
    0xffe805e8,
    0xffe80610,
    0xffe80638,
    0xffe80660,
    0xffe80688,
    0xffe806b0,
    0xffe806d8,
    0xffe80700,
    0xffe80728,
    0xffe80750,
    0xffe80778,
    0xffe807a0,
    0xffe807c8,
    0xffe807f0,
    0xffe80818,
    0xffe80840,
    0xffe80868,
    0xffe80890,
    0xffe808b8,
    0xffe808e0,
    0xffe80908,
    0xffe80930,
    0xffe80958,
    0xffe80980,
    0xffe809a8,
    0xffe809d0,
    0xffe809f8,
    
// CLK2
// Start = 0xffe81000, End = 0xffe81130
    0xffe81000,
    0xffe81028,
    0xffe81050,
    0xffe81078,
    0xffe810a0,
    0xffe810c8,
    0xffe810f0,
    0xffe81118,
    
// CAP
// Start = 0xfff10000, End = 0xfff10fcc
    0xfff10000,
    0xfff10028,
    0xfff10050,
    0xfff10078,
    0xfff100a0,
    0xfff100c8,
    0xfff100f0,
    0xfff10118,
    0xfff10140,
    0xfff10168,
    0xfff10190,
    0xfff101b8,
    0xfff101e0,
    0xfff10208,
    0xfff10230,
    0xfff10258,
    0xfff10280,
    0xfff102a8,
    0xfff102d0,
    0xfff102f8,
    0xfff10320,
    0xfff10348,
    0xfff10370,
    0xfff10398,
    0xfff103c0,
    0xfff103e8,
    0xfff10410,
    0xfff10438,
    0xfff10460,
    0xfff10488,
    0xfff104b0,
    0xfff104d8,
    0xfff10500,
    0xfff10528,
    0xfff10550,
    0xfff10578,
    0xfff105a0,
    0xfff105c8,
    0xfff105f0,
    0xfff10618,
    0xfff10640,
    0xfff10668,
    0xfff10690,
    0xfff106b8,
    0xfff106e0,
    0xfff10708,
    0xfff10730,
    0xfff10758,
    0xfff10780,
    0xfff107a8,
    0xfff107d0,
    0xfff107f8,
    0xfff10820,
    0xfff10848,
    0xfff10870,
    0xfff10898,
    0xfff108c0,
    0xfff108e8,
    0xfff10910,
    0xfff10938,
    0xfff10960,
    0xfff10988,
    0xfff109b0,
    0xfff109d8,
    0xfff10a00,
    0xfff10a28,
    0xfff10a50,
    0xfff10a78,
    0xfff10aa0,
    0xfff10ac8,
    0xfff10af0,
    0xfff10b18,
    0xfff10b40,
    0xfff10b68,
    0xfff10b90,
    0xfff10bb8,
    0xfff10be0,
    0xfff10c08,
    0xfff10c30,
    0xfff10c58,
    0xfff10c80,
    0xfff10ca8,
    0xfff10cd0,
    0xfff10cf8,
    0xfff10d20,
    0xfff10d48,
    0xfff10d70,
    0xfff10d98,
    0xfff10dc0,
    0xfff10de8,
    0xfff10e10,
    0xfff10e38,
    0xfff10e60,
    0xfff10e88,
    0xfff10eb0,
    0xfff10ed8,
    0xfff10f00,
    0xfff10f28,
    0xfff10f50,
    0xfff10f78,
    0xfff10fa0,
    0xfff10fc8,
    
// RDN
// Start = 0xfff86000, End = 0xfff86a58
    0xfff86000,
    0xfff86028,
    0xfff86050,
    0xfff86078,
    0xfff860a0,
    0xfff860c8,
    0xfff860f0,
    0xfff86118,
    0xfff86140,
    0xfff86168,
    0xfff86190,
    0xfff861b8,
    0xfff861e0,
    0xfff86208,
    0xfff86230,
    0xfff86258,
    0xfff86280,
    0xfff862a8,
    0xfff862d0,
    0xfff862f8,
    0xfff86320,
    0xfff86348,
    0xfff86370,
    0xfff86398,
    0xfff863c0,
    0xfff863e8,
    0xfff86410,
    0xfff86438,
    0xfff86460,
    0xfff86488,
    0xfff864b0,
    0xfff864d8,
    0xfff86500,
    0xfff86528,
    0xfff86550,
    0xfff86578,
    0xfff865a0,
    0xfff865c8,
    0xfff865f0,
    0xfff86618,
    0xfff86640,
    0xfff86668,
    0xfff86690,
    0xfff866b8,
    0xfff866e0,
    0xfff86708,
    0xfff86730,
    0xfff86758,
    0xfff86780,
    0xfff867a8,
    0xfff867d0,
    0xfff867f8,
    0xfff86820,
    0xfff86848,
    0xfff86870,
    0xfff86898,
    0xfff868c0,
    0xfff868e8,
    0xfff86910,
    0xfff86938,
    0xfff86960,
    0xfff86988,
    0xfff869b0,
    0xfff869d8,
    0xfff86a00,
    0xfff86a28,
    0xfff86a50,
    
// IDD
// Start = 0xfffa8000, End = 0xfffa8258
    0xfffa8000,
    0xfffa8028,
    0xfffa8050,
    0xfffa8078,
    0xfffa80a0,
    0xfffa80c8,
    0xfffa80f0,
    0xfffa8118,
    0xfffa8140,
    0xfffa8168,
    0xfffa8190,
    0xfffa81b8,
    0xfffa81e0,
    0xfffa8208,
    0xfffa8230,
    
// 3ARLM
// Start = 0xfffac000, End = 0xfffac068
    0xfffac000,
    0xfffac028,
    0xfffac050,
    
// HW3A
// Start = 0xfff22000, End = 0xfff22594
    0xfff22000,
    0xfff22028,
    0xfff22050,
    0xfff22078,
    0xfff220a0,
    0xfff220c8,
    0xfff220f0,
    0xfff22118,
    0xfff22140,
    0xfff22168,
    0xfff22190,
    0xfff221b8,
    0xfff221e0,
    0xfff22208,
    0xfff22230,
    0xfff22258,
    0xfff22280,
    0xfff222a8,
    0xfff222d0,
    0xfff222f8,
    0xfff22320,
    0xfff22348,
    0xfff22370,
    0xfff22398,
    0xfff223c0,
    0xfff223e8,
    0xfff22410,
    0xfff22438,
    0xfff22460,
    0xfff22488,
    0xfff224b0,
    0xfff224d8,
    0xfff22500,
    0xfff22528,
    0xfff22550,
    0xfff22578,
 
};

/* MISP SELF-TEST BEGIN */
enum {
	EXTISP_NULL = 1<<0,
	EXTISP_AL6045 = 1<<1,
	EXTISP_AL6010 = 1<<2,
};

enum altek6045_pipe_test_mode{
	ALTEK6045_PIPE_0 = 0,
	ALTEK6045_PIPE_1 = 1,
	ALTEK6045_PIPE_DUAL = 2,
	ALTEK6045_PIPE_MAX = 3,
};
enum altek6045_pipe_test_stage {
	ALTEK6045_PIPE_UNTESTED = 0,
	ALTEK6045_PIPE_TESTING = 1,
	ALTEK6045_PIPE_TEST_CMD_ERR = 2,
	ALTEK6045_PIPE_GET_CMD_ERR = 3,
	ALTEK6045_PIPE_TEST_BAD = 4,
	ALTEK6045_PIPE_TEST_DONE = 5,
	ALTEK6045_PIPE_TEST_MAX = 6,
};

//static int misp_cmd_filter=0;
static int test_result[ALTEK6045_PIPE_MAX];
#define set_test_result(pipe, result) \
	do { \
		test_result[pipe] = result;\
	} while(0)

static const char *test_report[ALTEK6045_PIPE_TEST_MAX] =
{
	[ALTEK6045_PIPE_UNTESTED] = "untested",
	[ALTEK6045_PIPE_TESTING] = "still in testing",
	[ALTEK6045_PIPE_TEST_CMD_ERR] = "test cmd error",
	[ALTEK6045_PIPE_GET_CMD_ERR] = "get cmd error",
	[ALTEK6045_PIPE_TEST_BAD] = "bad",
	[ALTEK6045_PIPE_TEST_DONE] = "ok",
};
/* MISP SELF-TEST END */

static struct misp_data* get_misp_data(void);
static int misp_send_cmd(struct misp_data *devdata, u16 opcode, u8 *param, u32 len);
static int misp_recv_data(struct misp_data *devdata, u8 *param, u32 len);
static int misp_write_block(struct misp_data *devdata, u8 *param, u32 len);
static u16 calculate_checksum(u8 *buf, u16 size);
static void spi_data_debug(const void *buf, int data_len, int dbg_len);
static void misp_dump_err(struct work_struct *work);
static void misp_dump_log(struct work_struct *work);
static int misp_spi_send(struct misp_data *devdata, u32 len);
static int misp_spi_recv(struct misp_data *devdata, u32 len, u8 ctrl_byte);
static int misp_recv_block(struct misp_data *devdata, u8 *out, u32 len);
static int misp_poll_status(struct misp_data *devdata);
static int misp_load_boot_code(struct msm_sensor_ctrl_t *s_ctrl, struct misp_data *devdata);
static int misp_load_basic_code(struct msm_sensor_ctrl_t *s_ctrl, struct misp_data *devdata);

static void misp_config_spi(struct misp_data *pdata, u32 mode, u32 bits, u32 speed);
static void misp_config_log(struct misp_data *pdata, u32 new_config, u32 new_log_level);
static void misp_dump_reg(struct misp_data *pdata, u32* reg_table, u32 reg_table_size);
static void misp_dump_ram(struct misp_data *pdata, u32 start, u32 total, u32 mode);
static void misp_write_reg(struct misp_data *pdata, u32 addr, u32 val);
static void misp_test_spi(struct misp_data *pdata);
extern bool huawei_cam_is_factory_mode(void);
/* 
 * mini isp export functions, export to other driver modules to use.
 * include :
 * @applicaton-layer spi cmd
 * @init and exit function
 * @power control funciont
 */

u32 misp_construct_opcode(u32 opcode, u32 set_flag, u32 len)
{
	u32 cmd;
	cmd = (set_flag<<31)+ (len<<16) + opcode;
	pr_err("cmd=0x%08x", cmd);
	return cmd;	
}
EXPORT_SYMBOL(misp_construct_opcode);
int misp_exec_cmd(u32 opcode, u8 *param)
{
	struct misp_data *devdata = get_misp_data();
	int err = 0;
	u8 set_flag = 0;
	u8 len = 0;
	u16 cmd;

	set_flag = opcode>>31;
	len = (opcode>>16)&0x7fff;
	cmd = opcode&0x0000ffff;

	if (!devdata) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	err = mutex_lock_interruptible(&devdata->busy_lock);
	if(err)
		return err;
	if (set_flag == 1) {
		devdata->state = MINI_ISP_STATE_IDLE;
		err = misp_send_cmd(devdata, cmd, param, len);
		if (err)
			goto out;

		/* special cmd <stream off> need to wait until done*/
		if ((cmd == ISPCMD_CAMERA_PREVIEWSTREAMONOFF) &&
			(param[0]==0 && param[1]==0 && param[2]==0)) {

			wait_event_interruptible_timeout(devdata->sync_queue,\
						(devdata->state == MINI_ISP_STATE_SYNC),\
						MINI_ISP_WAIT_TIMEOUT);

			if (devdata->state != MINI_ISP_STATE_SYNC) {
				pr_err("%s wait sync signal timeout", __func__);
				err = -EAGAIN;
				goto out;
			}
		}
	} else {
		devdata->state = MINI_ISP_STATE_IDLE;
		err = misp_send_cmd(devdata, opcode, NULL, 0);
		if (err)
			goto out;

		wait_event_interruptible_timeout(devdata->wait_queue,\
					(devdata->state == MINI_ISP_STATE_READY),\
					MINI_ISP_WAIT_TIMEOUT);

		if (devdata->state != MINI_ISP_STATE_READY) {
			pr_err("%s wait ready signal timeout", __func__);
			err = -EAGAIN;
			goto out;
		}

		err = misp_recv_data(devdata, param, len);
	}
out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return err;
}
EXPORT_SYMBOL(misp_exec_cmd);
int misp_execmd_array(struct msm_camera_spi_reg_settings settings)
{
	struct misp_data *devdata = get_misp_data();
	int rc = 0;
	struct msm_camera_spi_array* spi_cmd;
	int i = 0;
	if (!devdata) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}
	rc = mutex_lock_interruptible(&devdata->busy_lock);
	if(rc)
		return rc;
	devdata->state = MINI_ISP_STATE_IDLE;
	// handle cammands in settings
	for(i=0;i<settings.size;i++)
	{
		spi_cmd = settings.reg_settings + i;
		if(spi_cmd->is_recv){
		// recv data
			if(spi_cmd->is_block_data){
				rc = misp_recv_block(devdata,spi_cmd->param, spi_cmd->size);
			}else{
				rc = misp_recv_data(devdata,spi_cmd->param, spi_cmd->size);
			}
		}else{
		//send data
			if (spi_cmd->is_block_data){
				rc = misp_write_block(devdata,spi_cmd->param, spi_cmd->size);
			}else{
				rc = misp_send_cmd(devdata, spi_cmd->opcode, spi_cmd->param, spi_cmd->size);
			}
		}
		if (rc)
			goto out;
		if(spi_cmd->is_wait_state){
			wait_event_interruptible_timeout(devdata->wait_queue,\
	                   (devdata->state == spi_cmd->wait_state),MINI_ISP_WAIT_TIMEOUT);

			if (devdata->state != spi_cmd->wait_state) {
				pr_err("%s wait ready signal timeout %d\n", __func__,spi_cmd->wait_state);
				rc = -EAGAIN;
				goto out;
			}
		}
	}
out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return rc;
}
EXPORT_SYMBOL(misp_execmd_array);
int misp_askdata_cmd(struct misp_askdata_setting setting)
{
	struct misp_data *devdata = get_misp_data();
	int rc = 0;
	if (!devdata) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}
	rc = mutex_lock_interruptible(&devdata->busy_lock);
	if(rc)
		return rc;

	devdata->state = MINI_ISP_STATE_IDLE;
	if((0 == setting.asklen) && (setting.askparam != NULL)){
		pr_err("setting error asklen and askparam not match\n");
		return -1;
	}
	rc = misp_send_cmd(devdata, setting.opcode, setting.askparam, setting.asklen);
	if (rc)
		goto out;
	wait_event_interruptible_timeout(devdata->wait_queue,\
                                (devdata->state == setting.wait_state),\
                                MINI_ISP_WAIT_TIMEOUT);

	if (devdata->state != setting.wait_state) {
                pr_err("%s wait ready signal timeout", __func__);
                rc = -EAGAIN;
                goto out;
	}
	if(setting.is_block_data)
		rc = misp_recv_block(devdata,setting.recvparam, setting.recvlen);
	else
		rc = misp_recv_data(devdata, setting.recvparam, setting.recvlen);
out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return rc;
}
EXPORT_SYMBOL(misp_askdata_cmd);
int misp_write_cmd(u16 opcode, u8 *param, u32 len)
{
	struct misp_data *devdata = get_misp_data();
	int rc = 0;

	if (!devdata) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	rc = mutex_lock_interruptible(&devdata->busy_lock);
	if(rc)
		return rc;

	devdata->state = MINI_ISP_STATE_IDLE;
	rc = misp_send_cmd(devdata, opcode, param, len);
	if (rc)
		goto out;
	/* special cmd <stream off> need to wait until done*/
	if ((opcode == ISPCMD_CAMERA_PREVIEWSTREAMONOFF) &&
		(param[0]==0 && param[1]==0 && param[2]==0)) {
		wait_event_interruptible_timeout(devdata->sync_queue,\
					(devdata->state == MINI_ISP_STATE_SYNC),\
					MINI_ISP_WAIT_TIMEOUT_STREAMOFF);
		if (devdata->state != MINI_ISP_STATE_SYNC) {
			pr_err("%s stream off wait sync signal timeout", __func__);
			rc = -EAGAIN;
			goto out;
		}
	}

out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return rc;
}
EXPORT_SYMBOL(misp_write_cmd);
/*add camera ois driver*/
static int misp_recv_data_block(struct misp_data *devdata, u8 *out, u32 len)
{
	struct spi_transfer	xfer[2];
	struct spi_message	m;
	u8 ctrl_byte = USPICTRL_MS_CB_DIS;
	int ret = 0;

	if (len > PAGE_SIZE) {
		pr_err("%s invalid block len=%d \n", __func__, len);
		return -EINVAL;
	}

	pr_debug("%s - recv block addr=%p len=%d \n", __func__, out, len);

	/*one message only, incase broken by spi interrupt */
	memset(xfer, 0, sizeof(xfer));
	devdata->ext_buf[0] = ctrl_byte;
	xfer[0].tx_buf = devdata->ext_buf;
	xfer[0].len = sizeof(ctrl_byte);
	xfer[0].cs_change = 1;
	xfer[0].delay_usecs = 1;
	xfer[1].rx_buf = devdata->rx_buf;
	xfer[1].len = len;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	ret = spi_sync(devdata->spi, &m);

	memcpy(out, devdata->rx_buf, len);
	return ret;
}
static int misp_write_block(struct misp_data *devdata, u8 *param, u32 len)
{
	int err = 0;
	u8 ctrl_byte = USPICTRL_MS_CB_DIS;
	struct spi_transfer	xfer[2] = {{0},{0}};
	struct spi_message	m;
	int status;
	pr_debug("Enter: %s \n", __func__);

	if(!devdata) {
	  pr_err("%s no driver data", __func__);
	  return -ENODEV;
	}
	/*one message only, incase broken by spi interrupt */
	devdata->ext_buf[0] = ctrl_byte;
	xfer[0].tx_buf = devdata->ext_buf;
	xfer[0].len = sizeof(ctrl_byte);
	xfer[0].cs_change = 1;
	xfer[0].delay_usecs = 1;

	memcpy(devdata->tx_buf, param, len);
	xfer[1].tx_buf = devdata->tx_buf;
	xfer[1].len = len;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(devdata->spi, &m);
	if (status) {
	  pr_err("%s - sync error: status=%d \n", __func__, status);
	}
	if (devdata->debug_config & MINI_ISP_DEBUG_SPI_DATA)
		spi_data_debug(devdata->tx_buf, SPI_TX_BUF_SIZE, len);
	return err;
}
int misp_exec_bidir_cmd(u16 cmd, u8 *in, u32 in_len,
						bool out_to_block, u8 *out, u32 out_len)
{
	struct misp_data *devdata = get_misp_data();
	int err = 0, ret = 0;
    pr_debug("Enter: %s \n", __func__);

	if (!devdata) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	if ((in == NULL) || (out == NULL)) {
		pr_err("%s in out buf invalid", __func__);
		return -EINVAL;
	}

	err = mutex_lock_interruptible(&devdata->busy_lock);
	if (err)
		return err;

	devdata->state = MINI_ISP_STATE_IDLE;
	err = misp_send_cmd(devdata, cmd, in, in_len);
	if (err)
		goto out;

	ret = wait_event_interruptible_timeout(devdata->wait_queue,\
						(devdata->state == MINI_ISP_STATE_READY),\
						MINI_ISP_WAIT_TIMEOUT);
	if (ret <= 0) {
		pr_err("%s wait ready signal timeout \n", __func__);
		err = -EAGAIN;
		goto out;
	}

	if (out_to_block)
		err = misp_recv_data_block(devdata, out, out_len);
	else
		err = misp_recv_data(devdata, out, out_len);

out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return err;
}
EXPORT_SYMBOL(misp_exec_bidir_cmd);


int misp_exec_write_block(u16 cmd, u8 *in, u32 in_len, u8 *out, u32 out_len)
{
    struct misp_data *devdata = get_misp_data();
    int err = 0;
	u8 ctrl_byte = USPICTRL_MS_CB_DIS;
	struct spi_transfer	xfer[2];
	struct spi_message	m;
	int status;
    pr_debug("Enter: %s \n", __func__);

    if(!devdata) {
        pr_err("%s no driver data", __func__);
        return -ENODEV;
    }
	memset(xfer, 0, sizeof(xfer));

    err = mutex_lock_interruptible(&devdata->busy_lock);
    if(err)
        return err;
	devdata->state = MINI_ISP_STATE_IDLE;
	if(misp_send_cmd(devdata, cmd, in, in_len))
		goto out;

	wait_event_interruptible_timeout(devdata->wait_queue,\
				(devdata->state == MINI_ISP_STATE_READY),\
				MINI_ISP_WAIT_TIMEOUT);

	if (devdata->state != MINI_ISP_STATE_READY) {
		pr_err("%s wait ready signal timeout \n", __func__);
		err = -EAGAIN;
		goto out;
	}

    /*one message only, incase broken by spi interrupt */
	devdata->ext_buf[0] = ctrl_byte;
    memcpy(devdata->tx_buf, out, out_len);
	xfer[0].tx_buf = devdata->ext_buf;
	xfer[0].len = sizeof(ctrl_byte);
	xfer[0].cs_change = 1;
	xfer[0].delay_usecs = 1;
	xfer[1].tx_buf = devdata->tx_buf;
	xfer[1].len = out_len;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(devdata->spi, &m);
	if (status) {
		pr_err("%s - sync error: status=%d \n", __func__, status);
	}

    //spi_data_debug(devdata->tx_buf, SPI_TX_BUF_SIZE, out_len);

	/*add hdr and mutiframe interface base on mini isp*/
	if(cmd != ISPCMD_BULK_WRITE_START_ZSL_SNAPSHOT) {
		wait_event_interruptible_timeout(devdata->ois_queue,\
				(devdata->state == MINI_ISP_STATE_OIS),\
				MINI_ISP_WAIT_TIMEOUT);

		if (devdata->state != MINI_ISP_STATE_OIS) {
			pr_err("%s wait ois signal timeout \n", __func__);
			err = -EAGAIN;
			goto out;
		}
	}

    err = 0;

out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return err;
}
EXPORT_SYMBOL(misp_exec_write_block);


int misp_exec_inout_cmd(u16 cmd, u8 *in, u32 in_len,
										u8 *out, u32 out_len)
{
	struct misp_data *devdata = get_misp_data();
	int err = 0, ret = 0;

	if (!devdata) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	if ((in == NULL) || (out == NULL)) {
		pr_err("%s in out buf invalid", __func__);
		return -EINVAL;
	}

	err = mutex_lock_interruptible(&devdata->busy_lock);
	if(err)
		return err;

	devdata->state = MINI_ISP_STATE_IDLE;
	err = misp_send_cmd(devdata, cmd, in, in_len);
	if (err)
		goto out;

	ret = wait_event_interruptible_timeout(devdata->wait_queue,\
						(devdata->state == MINI_ISP_STATE_READY),\
						MINI_ISP_WAIT_TIMEOUT);
	if (ret <= 0) {
		pr_err("%s wait ready signal timeout", __func__);
		err = -EAGAIN;
		goto out;
	}

	err = misp_recv_data(devdata, out, out_len);
out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);
	return err;
}
EXPORT_SYMBOL(misp_exec_inout_cmd);

int misp_flush_log(void)
{
	struct misp_data *drv_data = get_misp_data();
	pr_info("%s enter \n", __func__);

	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	/* force to record last log when exit */
	if (drv_data->debug_config & MINI_ISP_DEBUG_LOG_FILE) {
		queue_work(drv_data->work_queue, &drv_data->dump_log_work);
		flush_work(&drv_data->dump_log_work);
	}

	return 0;
}
EXPORT_SYMBOL(misp_flush_log);


int misp_reset_vcm(void)
{
	struct misp_data *drv_data = get_misp_data();
	u8 mode = 5;
	int ret = 0, err= 0;

	pr_info("%s enter", __func__);

	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	if(mutex_lock_interruptible(&drv_data->busy_lock))
		return -1;

	drv_data->state = MINI_ISP_STATE_IDLE;
	err = misp_send_cmd(drv_data, ISPCMD_SYSTEM_CHANGEMODE, &mode, sizeof(mode));

	ret = wait_event_interruptible_timeout(drv_data->pwdn_queue,\
						(drv_data->state == MINI_ISP_STATE_PWDN),\
						msecs_to_jiffies(500));
	if (ret <= 0) {
		pr_err("%s wait vcm pwdn signal timeout", __func__);
		err = -EAGAIN;
	}
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&drv_data->busy_lock);

	return err;
}
EXPORT_SYMBOL(misp_reset_vcm);

int misp_flush_reg(void)
{
	struct misp_data *drv_data = get_misp_data();
	u32 *addr_table = NULL;
	u32 size = 0;
	pr_info("%s enter", __func__);

	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}
	addr_table = misp_al6010_reg_table;
	size = ARRAY_SIZE(misp_al6010_reg_table);

	/* don't dump twice */
	if (drv_data->flush_reg_flag == MINI_ISP_FLUSH_REG_ALREADY)
		return 0;

	/* mark register already dumped */
	drv_data->flush_reg_flag = MINI_ISP_FLUSH_REG_ALREADY;

	misp_dump_reg(drv_data, addr_table, size);

	return 0;
}
EXPORT_SYMBOL(misp_flush_reg);
int misp_set_cs_gpio(int value)
{
	struct misp_data *drv_data = get_misp_data();
	if(gpio_direction_output( drv_data->spi->cs_gpio,value ))
	{
		pr_err("cs_gpio high failed");
	}
	return 0;
}
EXPORT_SYMBOL(misp_set_cs_gpio);
int misp_set_reset_gpio(int value)
{
	struct misp_data *drv_data = get_misp_data();
	struct misp_plat_data *plat_data = NULL;
	pr_info("%s enter", __func__);
	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}

	plat_data = drv_data->plat_data;


	if(gpio_direction_output(plat_data->reset_gpio,value ))
        {
                pr_err("reset  high fa");
        }
        return 0;
}
EXPORT_SYMBOL(misp_set_reset_gpio);
int misp_load_fw(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct misp_data *drv_data = get_misp_data();
	struct misp_plat_data *plat_data = NULL;
	int ret = 0;
    u32 new_config = 3;
    u32 new_log_level = 1;

	pr_err("%s enter", __func__);
	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return -ENODEV;
	}
	plat_data = drv_data->plat_data;
	ret=mutex_lock_interruptible(&drv_data->busy_lock);
	if (ret) {
		pr_err("%s mutex lock error", __func__);
		goto out;
	}
	ret = gpio_direction_output(plat_data->reset_gpio,0 );
	if(ret) {
		pr_err("%s reset_gpio to 0 failed",__func__);
		goto unlock;
	}
	msleep(1);
	ret = gpio_direction_output(plat_data->reset_gpio,1 );
	if(ret) {
		pr_err("%s reset_gpio to 1 failed",__func__);
		goto unlock;
	}
    msleep(5);
	/* load boot code firmware */
	drv_data->spi->max_speed_hz = MINI_ISP_SPI_SPEED_BOOT;
	ret = spi_setup(drv_data->spi);
	if (ret < 0) {
		pr_err("%s - failed to setup spi speed:%u", __func__,
										drv_data->spi->max_speed_hz);
		goto unlock;
	}
	ret = misp_load_boot_code(s_ctrl, drv_data);
	if (ret)
		goto unlock;
	msleep(1);

	/* load main code firmware */
	drv_data->spi->max_speed_hz = MINI_ISP_SPI_SPEED_NORMAL;
	ret = spi_setup(drv_data->spi);
	if (ret < 0) {
		pr_err("%s - failed to setup spi speed:%u", __func__,
										drv_data->spi->max_speed_hz);
		goto unlock;
	}
	drv_data->state = MINI_ISP_STATE_IDLE;
	ret |= misp_load_basic_code(s_ctrl, drv_data);
	if (ret)
		goto unlock;

	wait_event_interruptible_timeout(drv_data->wait_queue,\
					(drv_data->state == MINI_ISP_STATE_READY),\
					MINI_ISP_WAIT_TIMEOUT);
	if (drv_data->state != MINI_ISP_STATE_READY){
		pr_err("%s wait ready signal timeout", __func__);
	}
	else
		pr_err("load firmware ok\n");
unlock:
	mutex_unlock(&drv_data->busy_lock);
out:

    if(huawei_cam_is_factory_mode())
    {
        misp_config_log(drv_data,new_config,new_log_level);
    }
    
	return ret;
}
EXPORT_SYMBOL(misp_load_fw);
/*mini-isp do the ois init*/
int misp_get_ois_initstatus(void)
{
	struct misp_data *drv_data = get_misp_data();
	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return 0;
	}
	return drv_data->ois_init_status;
}
int misp_set_ois_initstatus(int status)
{
	struct misp_data *drv_data = get_misp_data();
	if (!drv_data) {
		pr_err("%s no driver data", __func__);
		return -1;
	}
	drv_data->ois_init_status = status;
	return drv_data->ois_init_status;
}
EXPORT_SYMBOL(misp_get_ois_initstatus);
EXPORT_SYMBOL(misp_set_ois_initstatus);


/*
 * mini isp local functions, no need to export to other driver modules.
 * include :
 * @lowlevel spi transfer
 * @firmware load
 * @interrupte handle...
 */
static struct misp_data* get_misp_data(void)
{
	if (likely(misp_drv_data))
		return misp_drv_data;
	else
		return NULL;
}

static int misp_send_cmd(struct misp_data *devdata, u16 opcode, u8 *param, u32 len)
{
    u16 *p_len, *p_opcode, total_len;
    int err = 0;
	u16 calc_chk_sum;
	memset(devdata->tx_buf, 0, SPI_TX_BUF_SIZE);

	p_len = (u16 *)&devdata->tx_buf[0];
	p_opcode = (u16 *)&devdata->tx_buf[ISPCMD_OPCODEBYTES];

	total_len = ISPCMD_HDRSIZE + len;
	*p_len      = total_len - ISPCMD_LENFLDBYTES;

	*p_opcode	= opcode;
	memcpy(&devdata->tx_buf[ISPCMD_HDRSIZE], param, len);

   /* calculate checksum */
   calc_chk_sum = calculate_checksum(devdata->tx_buf, total_len);
   memcpy(&devdata->tx_buf[total_len], &calc_chk_sum, ISPCMD_CKSUMBYTES);
   /* add bytes for checksum */
   total_len += ISPCMD_CKSUMBYTES;

   /* send command to slave */
   err = misp_spi_send(devdata,  total_len);

   return err;
}

static int misp_recv_block(struct misp_data *devdata, u8 *out, u32 len)
{
	int rc = 0;

	if (len > PAGE_SIZE) {
                pr_err("%s invalid block len=%d", __func__, len);
                return -EINVAL;
	}
	//len is not bigger than PAGE_SIZE. and block is bigger than SPI_RX_BUF_SIZE
	memset(devdata->rx_buf, 0, len);
	/* get data via spi bus */
	rc = misp_spi_recv(devdata, len, USPICTRL_MS_CB_DIS);
	if (rc)
		return rc;

	memcpy(out, devdata->rx_buf, len);
	return rc;
}
static int misp_recv_data(struct misp_data *devdata, u8 *param, u32 len)
{
    int err = 0;
	u32 total_len;
	u16  calc_chk_sum, recv_chk_sum;

	memset(devdata->rx_buf, 0, SPI_RX_BUF_SIZE);
    total_len = len + ISPCMD_HDRSIZEWDUMMY + ISPCMD_CKSUMBYTES;

    /* get data via spi bus */
    err = misp_spi_recv(devdata, total_len,USPICTRL_MS_CB_RSP);
	if (err)
		return err;

    /* calculate checksum */
    memcpy(&recv_chk_sum, &devdata->rx_buf[(total_len - ISPCMD_CKSUMBYTES)], ISPCMD_CKSUMBYTES);
	calc_chk_sum = calculate_checksum(devdata->rx_buf, (total_len - ISPCMD_CKSUMBYTES));
	if(calc_chk_sum != recv_chk_sum) {
		pr_err("%s recicev data check sum not match", __func__);
		return -1;
	}

    /* copy param data to caller */
    memcpy(param, &devdata->rx_buf[ISPCMD_HDRSIZEWDUMMY], len);

    return 0;

}

static u16 calculate_checksum(u8 *buf, u16 size)
{
    u16 index;
	u32 sum = 0;
	u16 sumvalue;

	for (index=0 ; index < size ; index++) {
	  	if( 0 == (index%2))
			sum += buf[index];
		else
			sum += (buf[index] << 8);
	}
	sumvalue = (u16) ( 65536 - (sum & 0x0000FFFF));
	return sumvalue;
}

static void spi_data_debug(const void *buf, int data_len, int dbg_len)
{
	int len=0, pos=0;
	unsigned char *cbuf = (unsigned char *)buf;

	len = (dbg_len>data_len)?data_len:dbg_len;
	len =  roundup(len, 16);
	len = (len<16)?16:len;

	pos = 0;
	while (len > 0) {
		pr_info("buf[%04d]:%02x %02x %02x %02x %02x %02x %02x %02x "
							 "%02x %02x %02x %02x %02x %02x %02x %02x\n",
					pos,
					cbuf[pos],   cbuf[pos+1], cbuf[pos+2], cbuf[pos+3],
					cbuf[pos+4], cbuf[pos+5], cbuf[pos+6], cbuf[pos+7],
					cbuf[pos+8], cbuf[pos+9], cbuf[pos+10],cbuf[pos+11],
					cbuf[pos+12],cbuf[pos+13],cbuf[pos+14],cbuf[pos+15]);
		len -= 16;
		pos += 16;
	}
}

static irqreturn_t misp_irq_thread(int irq, void *handle)
{
	struct misp_data *drv_data = (struct misp_data *)handle;
	u16	irq_code = 0;
	u8	ctrl_byte = USPICTRL_MS_CB_STS;
	struct spi_message	m;
	struct spi_transfer	xfer[2] = {
		[0] = {
			.tx_buf = drv_data->irq_buf,
			.rx_buf = NULL,
			.bits_per_word = 8,
			.len = sizeof(ctrl_byte),
		},
		[1] = {
			.tx_buf = NULL,
			.rx_buf = drv_data->irq_buf,
			.bits_per_word = 8,
			.len = sizeof(irq_code),
		},
	};

	drv_data->irq_buf[0] = ctrl_byte;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	spi_sync(drv_data->spi, &m);
	memcpy(&irq_code, drv_data->irq_buf, sizeof(irq_code));
	pr_err(" irq misp irq state=0x%04x", irq_code);
	if (irq_code ==0xffff || irq_code ==0x0000){	
		return IRQ_HANDLED;
	}
	/* handle ready done */
	if (irq_code & MINI_ISP_IRQ_READY_DONE) {
		drv_data->state = MINI_ISP_STATE_READY;
		wake_up_interruptible(&drv_data->wait_queue);
	}

	/* handle sync signal */
	if (irq_code & MINI_ISP_IRQ_SYNC) {
		drv_data->state = MINI_ISP_STATE_SYNC;
		wake_up_interruptible(&drv_data->sync_queue);
	}

	/* handle pwdn signal */
	if (irq_code & MINI_ISP_IRQ_PWDN) {
		drv_data->state = MINI_ISP_STATE_PWDN;
		wake_up_interruptible(&drv_data->pwdn_queue);
	}

	/*mini-isp do the ois init*/
	/* handle camera OIS init  */
	if (irq_code & MINI_ISP_IRQ_OIS_INITDONE){
		 misp_set_ois_initstatus(1);
		/*fix ois running test fail issue*/
		queue_work(drv_data->work_queue, &drv_data->ois_init_done_work);
		 pr_err("mini isp init ois success");
	}

	/*add camera ois driver*/
	if(irq_code & MINI_ISP_IRQ_OIS) {
		drv_data->state = MINI_ISP_STATE_OIS;
		wake_up_interruptible(&drv_data->ois_queue);
	}

	/* handle cmd error */
	if (irq_code & MINI_ISP_IRQ_CMD_ERR)
		pr_err("mini isp critical error, cmd not respond");

	/* handle other error */
	if (irq_code & MINI_ISP_IRQ_OTHRE_ERR)
	{
		pr_err("mini isp MINI_ISP_IRQ_OTHRE_ERR \n");
		queue_work(drv_data->work_queue, &drv_data->dump_err_work);
	}

	/* handle dump log */
	if (irq_code & MINI_ISP_IRQ_LOG_FULL) {
		if (drv_data->debug_config & MINI_ISP_DEBUG_LOG_FILE)
			queue_work(drv_data->work_queue, &drv_data->dump_log_work);
	}

	return IRQ_HANDLED;
}
/*fix ois running test fail issue*/
static void misp_ois_init_done_work(struct work_struct *work)
{
	struct misp_data *drv_data =
		container_of(work, struct misp_data, ois_init_done_work);
	u16 param = 0x0040;
	u16 code = 0x0111;
	int err = 0;

	if(!drv_data)
		return;

	pr_info("%s enter\n", __func__);

	if(mutex_lock_interruptible(&drv_data->busy_lock))
		return;

	/* clear ois init done irq statu code: 0x0040*/
	err = misp_send_cmd(drv_data, code, (u8*)&param, sizeof(param));
	if(err)
	{
		pr_err("%s: clear ois init done irq status Fail!\n",__func__);
	}

	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&drv_data->busy_lock);
	pr_info("%s X\n", __func__);
}

static void misp_dump_err(struct work_struct *work)
{
	struct misp_data *drv_data =
		container_of(work, struct misp_data, dump_err_work);
	int err = 0, index = 0;

	pr_info("%s enter", __func__);

	/* send get error cmd*/
	if(mutex_lock_interruptible(&drv_data->busy_lock))
		return ;
	err = misp_send_cmd(drv_data, ISPCMD_SYSTEM_GET_ERRORCODE, NULL, 0);
	if (err)
		return;
	err = misp_poll_status(drv_data);
	if (err)
		goto out;
	err = misp_recv_data(drv_data, (u8*)drv_data->last_error_code,
						sizeof(u32)*MINI_ISP_MAX_ERR_CODE);
out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&drv_data->busy_lock);
    
	/* dump last error code */
	if (!err) {
		for (index=0; index<MINI_ISP_MAX_ERR_CODE; index++) {
			pr_info("misp last error code[%d] = 0x%08x",
					index, drv_data->last_error_code[index]);
		}
	}
    if(huawei_cam_is_factory_mode()){
        pr_err("%s: misp err, dump reg\n", __func__);
        misp_flush_reg();
    }

}

static void misp_dump_log(struct work_struct *work)
{
	int err = 0;
	struct misp_data *devdata =
			container_of(work, struct misp_data, dump_log_work);
	struct file *fp = NULL;
	mm_segment_t oldfs = get_fs();
	u8 param[8], ctrl_byte = USPICTRL_MS_CB_DIS;
	u32 *psize, *pblock;
	size_t len;
	struct spi_transfer	xfer[2];
	struct spi_message	m;
    static int log_count = 0;
    char log_path[256] = {0};

	pr_info("%s enter \n", __func__);

	psize  = (u32*)&param[0];
	pblock = (u32*)&param[sizeof(u32)];
	*psize = MINI_ISP_FW_LOG_BUF;
	*pblock= SPI_BLOCK_BUF_SIZE;
	memset(xfer, 0, sizeof(xfer));

	/* save log to buffer */
	if(mutex_lock_interruptible(&devdata->busy_lock))
		return;

	err = misp_send_cmd(devdata, ISPCMD_BULK_LOG_DUMP, param, sizeof(param));
	if (err)
		goto cmd_out;

	err = misp_poll_status(devdata);
	if (err)
		goto cmd_out;

	/*one message only, incase broken by spi interrupt */
	devdata->ext_buf[0] = ctrl_byte;
	xfer[0].tx_buf = devdata->ext_buf;
	xfer[0].bits_per_word = 8;
	xfer[0].len = sizeof(ctrl_byte);
	xfer[0].cs_change = 1;
	xfer[1].rx_buf = devdata->rx_buf;
	xfer[1].bits_per_word = 8;
	xfer[1].len = MINI_ISP_FW_LOG_BUF;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	spi_sync(devdata->spi, &m);

	/* dump log buffer to file */
	devdata->rx_buf[MINI_ISP_FW_LOG_BUF-1] = 0;
	len = strlen((char *)(devdata->rx_buf));
	if (len == 0) {
		pr_err("%s no data in log buf", __func__);
		goto cmd_out;
	}

    snprintf(log_path, sizeof(log_path), "/data/misp/misp_%d.log", log_count++);

	set_fs(KERNEL_DS);
	fp = filp_open(log_path, O_CREAT|O_APPEND|O_WRONLY, 0666);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s open log file error", __func__);
		goto file_out;
	}
	if (fp->f_pos < (4*1024*1024L)) {
		vfs_write(fp, devdata->rx_buf, len, &fp->f_pos);
	} else {
		pr_err("%s log file is larger than 4MB", __func__);
		filp_close(fp, NULL);
		fp = filp_open(log_path, O_CREAT|O_TRUNC|O_WRONLY, 0666);
		if (IS_ERR_OR_NULL(fp)) {
			pr_err("%s create new log file error", __func__);
			goto file_out;
		}
		vfs_write(fp, devdata->rx_buf, len, &fp->f_pos);
	}

file_out:
	if (!IS_ERR_OR_NULL(fp))
		filp_close(fp, NULL);
	set_fs(oldfs);
cmd_out:
	udelay(MINI_ISP_CMD_DELAY_US);
	mutex_unlock(&devdata->busy_lock);

    pr_info("%s X \n", __func__);
}

static int misp_spi_send(struct misp_data *devdata, u32 len)
{
	int status;
	u8 ctrl_byte = USPICTRL_MS_CB_ORG;
	struct spi_transfer	xfer[2] = {
		[0] = {
			.tx_buf = devdata->ext_buf,
			.rx_buf = NULL,
			.bits_per_word = 8,
			.len = 1,
		},
		[1] = {
			.tx_buf = devdata->tx_buf,
			.rx_buf = NULL,
			.bits_per_word = 8,
			.len = len,
		},
	};
	struct spi_message	m;

	pr_info("%s - send buf len=%d:", __func__, len);

	if ((!devdata) || (len > SPI_TX_BUF_SIZE)) {
		pr_err("%s - invalid arg devdata=%p,len=%d",__func__, devdata, len);
		return -EINVAL;
	}
	devdata->ext_buf[0] = ctrl_byte;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(devdata->spi, &m);
	if (status) {
		pr_err("%s - sync error: status=%d", __func__, status);
		return status;
	}

	if (devdata->debug_config & MINI_ISP_DEBUG_SPI_DATA)
		spi_data_debug(devdata->tx_buf, SPI_TX_BUF_SIZE, len);

	return status;
}

static int misp_spi_recv(struct misp_data *devdata, u32 len,u8 ctrl_byte)
{
	int status;
	struct spi_message m;
	struct spi_transfer	xfer[2] = {
		[0] = {
			.tx_buf = devdata->ext_buf,
			.rx_buf = NULL,
			.bits_per_word = 8,
			.len = 1,
		},
		[1] = {
			.tx_buf = NULL,
			.rx_buf = devdata->rx_buf,
			.bits_per_word = 8,
			.len = len,
		},
	};
	switch(ctrl_byte){
	case USPICTRL_MS_CB_RSP:
		xfer[0].cs_change=0;
		break;
	case USPICTRL_MS_CB_DIS:
		xfer[0].cs_change=1;
		break;
	default :
		pr_err("%s no such ctrl byte\n",__func__);
		return -1;
	}

	pr_info("%s - recv buf len=%d:", __func__, len);

	// some time len is bigger than SPI_RX_BUF_SIZE
	if ((!devdata)) {
		pr_err("%s - invalid arg devdata=%p", __func__, devdata);
		return -EINVAL;
	}
	devdata->ext_buf[0] = ctrl_byte;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);
	status = spi_sync(devdata->spi, &m);
	if (status) {
		pr_err("%s - sync error: status=%d", __func__, status);
		return status;
	}

	if (devdata->debug_config & MINI_ISP_DEBUG_SPI_DATA)
		spi_data_debug(devdata->rx_buf, SPI_RX_BUF_SIZE, len);

	return status;
}

static int misp_poll_status(struct misp_data *devdata)
{
	int status, trys;
	u8	ctrl_byte = USPICTRL_MS_CB_STS;
	u16 ack = 0;
	struct spi_transfer	xfer[2] = {
		[0] = {
			.tx_buf = devdata->ext_buf,
			.rx_buf = NULL,
			.bits_per_word = 8,
			.len = 1,
		},
		[1] = {
			.tx_buf = NULL,
			.rx_buf = devdata->rx_buf,
			.bits_per_word = 8,
			.len = 2,
		},
	};
	struct spi_message	m;

	if (!devdata) {
		pr_err("%s - invalid arg devdata=%p", __func__, devdata);
		return -EINVAL;
	}

	devdata->ext_buf[0] = ctrl_byte;
	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);

	for (trys=0 ; trys<MINI_ISP_POLL_TIMES; trys++) {
		status = spi_sync(devdata->spi, &m);
		if (status) {
			pr_err("%s - sync error: status=%d", __func__, status);
			return status;
		}
		memcpy(&ack, devdata->rx_buf, sizeof(ack));
		if (ack == MINI_ISP_IRQ_READY_DONE)
			break;

		msleep(5);
	}

	if (ack == MINI_ISP_IRQ_READY_DONE) {
		pr_info("%s - try times=%d", __func__, trys);
		return 0;
	} else {
		pr_err("%s - timeout ack=0x%04x", __func__, ack);
		return -EAGAIN;
	}
}

static int misp_load_boot_code(struct msm_sensor_ctrl_t *s_ctrl, struct misp_data *devdata)
{
	int rc = 0;
	u8 ctrl_byte = USPICTRL_MS_CB_DIS;
	const struct firmware *fw_entry = NULL;	
	int totol_size;
	u8* fw_buf;
	struct spi_transfer	xfer[2] = {
		[0] = {
			.tx_buf = devdata->ext_buf,
			.rx_buf = NULL,
			.bits_per_word = 8,
			.len = 1,
		},
		[1] = {
			.rx_buf = NULL,
			.bits_per_word = 8,
			.speed_hz = MINI_ISP_SPI_SPEED_BOOT,
		},
	};

	struct spi_message m;
	rc = request_firmware(&fw_entry, s_ctrl->misp_boot_firmware_name, &devdata->spi->dev);
	if (rc != 0) {
		pr_err("%s:%d Firmware image miniBoot is not available\n", __func__, __LINE__);
		return rc;
	}
	pr_err("%s:%d Firmware miniBoot size:%ld\n", __func__, __LINE__,fw_entry->size);

	fw_buf = devm_kzalloc(&devdata->spi->dev, fw_entry->size, GFP_KERNEL);
	if ( !fw_buf ) {
		pr_err("fail to allocate firmware buffer\n");
		rc = -ENOMEM;
		goto  release_firmware;
	}
	totol_size=fw_entry->size;
	memcpy(fw_buf,fw_entry->data,totol_size);
	
	devdata->ext_buf[0] = ctrl_byte;
        //get the data
	xfer[1].tx_buf = fw_buf;
	xfer[1].len = totol_size;
	spi_message_init(&m);
	//TODO:make sure ctrl byte not needed
	//spi_message_add_tail(&xfer[0],&m);
	spi_message_add_tail(&xfer[1],&m);
	rc= spi_sync(devdata->spi,&m);
       	if (rc != 0)
 	{
	 	pr_err( "%s failed!!  status: %d", __func__,  rc);
    	}
    	pr_err("%s exit\n", __func__ );
	
	devm_kfree(&devdata->spi->dev,fw_buf);
	
release_firmware:
	release_firmware(fw_entry);
	return rc;
}

static int misp_load_basic_code(struct msm_sensor_ctrl_t *s_ctrl, struct misp_data *devdata)
{
	mm_segment_t oldfs = get_fs();
	struct file *fp = NULL;
	int one_size, left, block, ret;
	loff_t pos;
	//u8* fw_buf;

	u8 param[16];
	u8 misp_fw_filepath[128];
	u32 *p_addr = (u32 *)&param[0];
	u32 *p_total_size =  (u32 *)&param[sizeof(u32)];
	u32 *p_block_size =  (u32 *)&param[sizeof(u32)*2]; 
	u32 *p_chk_sum =  (u32 *)&param[sizeof(u32)*3];
	u8 ctrl_byte = USPICTRL_MS_CB_DIS;
	pr_debug("%s write fw begin\n", __func__);
	snprintf(misp_fw_filepath, sizeof(misp_fw_filepath), "/system/etc/firmware/%s", s_ctrl->misp_firmware_name);
	
	set_fs(KERNEL_DS);
	fp = filp_open(misp_fw_filepath, O_RDONLY, 0644);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s open misp_fw_file error", __func__);
		set_fs(oldfs);
		return -EINVAL;
	}

	vfs_llseek(fp, 0L, SEEK_SET);
	pos = fp->f_pos;
	vfs_read(fp, (char *)p_addr,		4, &pos);
	vfs_read(fp, (char *)p_total_size,	4, &pos);
	vfs_read(fp, (char *)p_chk_sum,		4, &pos);
	vfs_read(fp, (char *)p_block_size,	4, &pos);


	pr_err("%s,start sending basic code... size=0x%08x,p_block_size=%d\n", __func__,*p_total_size,*p_block_size);
	pr_err("basic code info:addr=0x%08x chksum=0x%08x\n", *p_addr, *p_chk_sum);
	*p_block_size = SPI_BLOCK_BUF_SIZE;

	/* need to send a cmd to tell mini isp how many blocks to send*/
	ret = misp_send_cmd(devdata, ISPCMD_BULK_WRITE_BASICCODE, param, sizeof(param));
	if (ret) {
		pr_err("%s send cmd error", __func__);
		goto out;
	}

	/* disable other control bytes */
	/* todo: better to be one message with tx buf */
	devdata->ext_buf[0] = ctrl_byte;
	spi_write(devdata->spi, devdata->ext_buf, 1);

	/* basic code actually start with 16bytes offset */
	vfs_llseek(fp, 16L, SEEK_SET);
	pos = fp->f_pos;
	for (left = *p_total_size, block = 0; left > 0; left -= SPI_BLOCK_BUF_SIZE) {
		/* memset(devdata->ext_buf, 0, SPI_BLOCK_BUF_SIZE); */
		one_size = (left > SPI_BLOCK_BUF_SIZE)? SPI_BLOCK_BUF_SIZE : left;

		ret = vfs_read(fp, devdata->tx_buf, one_size, &pos);
		if (ret < 0) {
			pr_err("%s vfs read error %d", __func__, ret);
			break;
		}

		ret = spi_write(devdata->spi, devdata->tx_buf, one_size);
		if (ret < 0) {
			pr_err("%s spi send error %d", __func__, ret);
			break;
		}
		block++;
	}
	pr_err("%s succeed to send %d blocks", __func__, block);

	/* get firmware version info */
	vfs_llseek(fp, -32L, SEEK_END);
	vfs_read(fp, (char *)&devdata->fw_info, 32, &fp->f_pos);
	devdata->fw_info.info[15] = '\0';
	devdata->fw_info.user[8]  = '\0';
	pr_err("%s firmware version: %01u.%04u user=%s", __func__, \
											devdata->fw_info.major,\
											devdata->fw_info.minor,\
											devdata->fw_info.user);
out:
	filp_close(fp, 0);
	set_fs(oldfs);
	pr_debug("%s write fw end\n", __func__);
	return ret;
}

/*
 * mini isp debug functions, used to debug.
 * include :
 * [spi config] item
 * [debug config] item
 *
 */
static ssize_t misp_config_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t misp_config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(misp_config, 0644, misp_config_show, misp_config_store);

static ssize_t misp_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0, index;
	struct misp_data *pdata = NULL;
	char *offset;

	pr_info("%s - enter", __func__);

	pdata = (struct misp_data *)dev_get_drvdata(dev);
	if (NULL==pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}

	/* show [spi config info] item */
	offset = buf;
	ret = snprintf(offset, PAGE_SIZE, "[spi config info]\n");

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  mode=%d, bit=%d, speed=%d\n",
					pdata->spi->mode,
					pdata->spi->bits_per_word,
					pdata->spi->max_speed_hz);

	/* show [debug config info] item */
	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "[debug config info]\n");

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  debug=0x%08x\n", pdata->debug_config);

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  log level=0x%08x\n", pdata->debug_log_level);

	/* show [boot version info] item */
	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "[boot version info]\n");

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  info=%s\n", pdata->boot_info.info);

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  version=%01u.%04u\n",\
										pdata->boot_info.major,\
										pdata->boot_info.minor);

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  user=%s\n", pdata->boot_info.user);

	/* show [fw version info] item */
	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "[fw version info]\n");

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  info=%s\n", pdata->fw_info.info);

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  version=%01u.%04u\n",\
										pdata->fw_info.major,\
										pdata->fw_info.minor);

	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "  user=%s\n", pdata->fw_info.user);

	/* show [last error info] item */
	offset += ret;
	ret = snprintf(offset, PAGE_SIZE, "[last error info]\n");

	offset += ret;
	for (index=0; index<MINI_ISP_MAX_ERR_CODE; index++) {
		ret = snprintf(offset, PAGE_SIZE, "  error code(%d)=0x%08x\n",
						index, pdata->last_error_code[index]);
		offset += ret;
	}
	return (offset-buf);
}

static void misp_config_spi(struct misp_data *pdata, u32 mode, u32 bits, u32 speed)
{
	int err;

	if (((mode & 0x03UL) > 3) || (bits > 32) || (speed > 48000000)) {
		pr_err("%s - invalid spi config [mode=0x%x bits=%d speed=%d]",
				__func__, mode, bits, speed);
		return ;
	}

	pdata->spi->mode= mode;
	pdata->spi->max_speed_hz = speed;
	pdata->spi->bits_per_word = bits;

	/* handle spi config */
	err = spi_setup(pdata->spi);
	if (!err) {
		pr_info("%s - setup spi [mode=%d bits=%d speed=%d]",
					__func__, mode, bits, speed);
		return;
	}

	/* invalid spi config , set default spi config */
	pr_err("%s - setup spi failed. use default config", __func__);
	pdata->spi->mode = SPI_MODE_3;
	pdata->spi->max_speed_hz = MINI_ISP_SPI_SPEED_BOOT;
	pdata->spi->bits_per_word = 8;
	spi_setup(pdata->spi);
}

static void misp_config_log(struct misp_data *pdata, u32 new_config, u32 new_log_level)
{
	u32 old_config = pdata->debug_config;

	/* config misp linux driver log */
	if ( (old_config & MINI_ISP_DEBUG_LOG_FILE) &&
		!(new_config & MINI_ISP_DEBUG_LOG_FILE)) {
		queue_work(pdata->work_queue, &pdata->dump_log_work);
		flush_work(&pdata->dump_log_work);
	}
	pdata->debug_config = new_config;

	/* config misp internal firmware log */
	pdata->debug_log_level = new_log_level;
	if(mutex_lock_interruptible(&pdata->busy_lock))
		return ;
	misp_send_cmd(pdata, ISPCMD_SYSTEM_SET_LOG_LEVEL, (u8*)&new_log_level, 4);
	mutex_unlock(&pdata->busy_lock);
}

static void misp_dump_reg(struct misp_data *pdata, u32* reg_table, u32 reg_table_size)
{
	mm_segment_t oldfs = get_fs();
	struct file *fp = NULL;
	u8 param[8], reg[48], reg_head[16], version[16];
	u32 *pstart=(u32*)&param[0], *pcount=(u32*)&param[4], index;
	int ret;
	char name[64];

	if (reg_table_size == 0) {
		pr_err("%s invalid reg table size", __func__);
		return;
	}

	pr_info("%s reg start=0x%x, reg end=0x%x", __func__,
				reg_table[0], reg_table[reg_table_size-1]);

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "/data/misp/misp_0x%08x.regx",
													reg_table[0]);

	/* create reg file */
	set_fs(KERNEL_DS);
	fp = filp_open(name, O_CREAT|O_TRUNC|O_WRONLY, 0666);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s create reg bin file error", __func__);
		set_fs(oldfs);
		return ;
	}

	/* add firmware version in dump file */
	memset(version, 0, sizeof(version));
	snprintf(version, sizeof(version), "%01u.%04u%s",
									pdata->fw_info.major,
									pdata->fw_info.minor,
									pdata->fw_info.user);
	vfs_write(fp, version, sizeof(version), &fp->f_pos);
	pr_info("%s firmware version: %01u.%04u user=%s",
										__func__,
									pdata->fw_info.major,
									pdata->fw_info.minor,
									pdata->fw_info.user);

	/* get reg from spi cmd */
	if(mutex_lock_interruptible(&pdata->busy_lock))
		goto FLIP_CLOSE;

	for (index = 0; index < reg_table_size; index++) {
		*pstart = reg_table[index];
		*pcount = 10;
		memset(reg_head, 0, sizeof(reg_head));
		snprintf(reg_head, sizeof(reg_head)-1, "[0x%08x]", reg_table[index]);

		pdata->state = MINI_ISP_STATE_IDLE;
		ret= misp_send_cmd(pdata, ISPCMD_SYSTEM_GET_ISPREGISTER, param, sizeof(param));
		if (ret)
			break;

		ret = wait_event_interruptible_timeout(pdata->wait_queue,\
						(pdata->state == MINI_ISP_STATE_READY),\
						MINI_ISP_WAIT_TIMEOUT);

		if (ret <= 0) {
			pr_err("%s wait ready signal timeout", __func__);
			break;
		}

		ret = misp_recv_data(pdata, reg, sizeof(reg));
		if (ret)
			break;

		/* save reg head to file    : 16bytes */
		vfs_write(fp, reg_head, sizeof(reg_head), &fp->f_pos);
		/* save reg context to file : 48bytes */
		vfs_write(fp, reg, sizeof(reg), &fp->f_pos);
	}

	mutex_unlock(&pdata->busy_lock);
FLIP_CLOSE:
	filp_close(fp, NULL);
	set_fs(oldfs);
	return ;
}


static void paser_reg_table(char *buf, u32 **out_ptr, u32 *out_count)
{
	u32  *reg_buf = NULL, reg_count = 0, reg_addr = 0, index = 0;
	char *pos = NULL;

	/* calulate how many reg table need to dump */
	pos = buf;
	reg_count = 0;
	while(*pos) {
		if (*pos == ']')
			break;
		if (pos[0] == '0' && tolower(pos[1]) == 'x') {
			reg_count++;
			pos += 2;
		} else
			pos++;
	}

	/* alloc buf and fill the buf with reg_address*/
	if (reg_count == 0) {
		reg_buf = NULL;
	} else {
		reg_buf = kmalloc(sizeof(u32) * reg_count, GFP_KERNEL);
		pos = buf;
		index = 0;
		while ((*pos) && (index < reg_count)) {
			if (*pos == ']')
				break;

			while (!isdigit(*pos))
				pos++;

			reg_addr = simple_strtoul(pos, &pos, 0);
			reg_buf[index] = reg_addr;

			index++;
		}
	}

	pr_info("%s reg_count=%d", __func__, reg_count);
	*out_ptr = reg_buf;
	*out_count = reg_count;
}

int misp_get_otp_data(struct msm_sensor_otp_info *otp_info, uint8_t cam_pos)
{
	struct misp_data *pdata= get_misp_data();
	int i = 0;
	u8 param[9];
	u8 ctrl_byte = USPICTRL_MS_CB_DIS;
	// Byte0: 0:Rear OTP data; 1:Front OTP data
	// Byte1~4: Total size
	// Byte5~8: SPI block size
	u8 *paddr=(u8*)&param[0];
	u32 *ptotal=(u32*)&param[1];
	u32 *pblock=(u32*)&param[5];
	int ret, one_size, left;
	u32 otp_length = otp_info->otp_size;
	u8 *otp_data = NULL;

	pr_debug("%s enter", __func__);
	if (!pdata) {
		pr_err("%s no driver data", __func__);
		return -1;
	}

	pr_info("%s cam_pos=0x%x, otp_length=%u\n", __func__, cam_pos, otp_length);

	if (otp_length == 0) {
		otp_length = 936*1024;
	}

	otp_data = kzalloc(otp_length, GFP_KERNEL);
	if(!otp_data){
		pr_err("%s failed: no memory for otp_data\n", __func__);
		return -1;
	}

	memset(param, 0, sizeof(param));
	*paddr = (u8)cam_pos;
	*ptotal = otp_length;
	*pblock = SPI_BLOCK_BUF_SIZE;

	if(mutex_lock_interruptible(&pdata->busy_lock))
	{
		pr_err("%s: mutex_lock_interruptible fail\n",__func__);
		kfree(otp_data);
		otp_data = NULL;
		return -1;
	}

	pdata->state = MINI_ISP_STATE_IDLE;
	ret= misp_send_cmd(pdata, ISPCMD_BULK_GET_OTP_DATA, param, sizeof(param));
	if (ret)
	{
		pr_err("%s: misp_send_cmd fail\n",__func__);
		goto out;
	}

	ret = wait_event_interruptible_timeout(pdata->wait_queue,\
						(pdata->state == MINI_ISP_STATE_READY),\
						MINI_ISP_WAIT_TIMEOUT);

	if (ret <= 0) {
		pr_err("%s wait ready signal timeout", __func__);
		goto out;
	}

	/* disable interrpt to avoid messages broken */
	disable_irq(pdata->spi->irq);
	pdata->ext_buf[0] = ctrl_byte;
	spi_write(pdata->spi, pdata->ext_buf, sizeof(ctrl_byte));
	for (left = otp_length; left > 0; left -= SPI_BLOCK_BUF_SIZE) {
		one_size = (left > SPI_BLOCK_BUF_SIZE)? SPI_BLOCK_BUF_SIZE : left;
		spi_read(pdata->spi, pdata->rx_buf, one_size);
	}
	enable_irq(pdata->spi->irq);
	//*vendor_id = (int)(pdata->rx_buf)[4];
	memcpy(otp_data, pdata->rx_buf, otp_length);
	spi_data_debug(pdata->rx_buf, 16, 16);

	pr_info("%s: otp data: DATE=20%d.%d.%d, huawei ID=%d, moudle ID=%d \n",
		__func__, otp_data[0], otp_data[1], otp_data[2], otp_data[3], otp_data[4]);
	pr_info("%s, is otp data vaild=%d \n", __func__, otp_data[otp_length-1]);

	otp_info->otp_vaild = 1; //otp_data[otp_length-1]; TODO later

	if(otp_info->otp_vaild)
	{
		//copy common info
		if(otp_info->common_otp.common_size)
		{
			memcpy(otp_info->common_otp.common_info,otp_data,otp_info->common_otp.common_size);
			pr_info("common otp info[%d]:\n",otp_info->common_otp.common_size);
			for(i = 0; i<otp_info->common_otp.common_size; i++)
			{
				pr_info("0x%02x ",otp_info->common_otp.common_info[i]);
			}
			pr_info("\n");
		}

		//copy awb info
		if(otp_info->awb_otp.awb_size)
		{
			memcpy(otp_info->awb_otp.aucISO_AWBCalib,&otp_data[otp_info->awb_otp.index_start],
				otp_info->awb_otp.awb_size);
			pr_info("awb otp info[%d]:\n",otp_info->awb_otp.awb_size);
			for(i = 0; i<otp_info->awb_otp.awb_size; i++)
			{
				pr_info("0x%02x ",otp_info->awb_otp.aucISO_AWBCalib[i]);
			}
			pr_info("\n");
		}

		//copy af info
		if(otp_info->af_otp.af_size){
			otp_info->af_otp.start_code = 
			otp_data[otp_info->af_otp.index_start] << 8
				| otp_data[otp_info->af_otp.index_start +1];
			otp_info->af_otp.max_code = 
			otp_data[otp_info->af_otp.index_start + 2] << 8
				| otp_data[otp_info->af_otp.index_start + 3];
			pr_err("vcm current:  start=%d, max=%d", 
				otp_info->af_otp.start_code,
				otp_info->af_otp.max_code);
		}

		//copy ois info
		if(otp_info->ois_otp.ois_size)
		{
			memcpy(otp_info->ois_otp.aucOIS,&otp_data[otp_info->ois_otp.index_start],
				otp_info->ois_otp.ois_size);
			pr_info("ois otp info[%d]:\n",otp_info->ois_otp.ois_size);
			for(i = 0; i<otp_info->ois_otp.ois_size; i++)
			{
				pr_info("0x%02x ",otp_info->ois_otp.aucOIS[i]);
			}
			pr_info("\n");

			pr_info("ucDataValid =0x%02x \n", otp_data[otp_length - 2]);
			pr_info("ucHallLimitDataValid = 0x%02x \n",otp_data[otp_length - 1]);
		}

	}

	ret = 0;
out:
	if(otp_data)
	{
		kfree(otp_data);
		otp_data = NULL;
	}
	mutex_unlock(&pdata->busy_lock);
	return ret;
}
EXPORT_SYMBOL(misp_get_otp_data);

#if 0
void misp_get_otp(struct msm_sensor_otp_info *otp_info, uint8_t cam_pos)
{
	int rc = 0;
	uint32_t cmd_len = 9, block_size = 8192, otp_length = 0;
	uint8_t in_buf[9] = {0};
	uint8_t *otp_data = NULL;

	if(!otp_info)
	{
		pr_err("%s: otp info is NULL !\n", __func__);
		return ;
	}

	otp_length = otp_info->otp_size;
	otp_data = kzalloc(otp_length, GFP_KERNEL);
	if(!otp_data){
		pr_err("%s failed: no memory for otp_info", __func__);
		return ;
	}

	in_buf[0] = cam_pos;
	memcpy(&(in_buf[1]), &otp_length, sizeof(otp_length));
	memcpy(&(in_buf[5]), &block_size, sizeof(block_size));

	rc = misp_exec_bidir_cmd(ISPCMD_BULK_GET_OTP_DATA,
								in_buf,
								cmd_len,
								1,
								otp_data,
								otp_length);
	if(rc)
	{
		pr_err("%s, %d, get otp cmd failed \n", __func__, __LINE__);
		goto free_otp_data;
	}

	pr_debug("%s: otp data: DATE=20%d.%d.%d, huawei ID=%d, moudle ID=%d",
		__func__, otp_data[0], otp_data[1], otp_data[2], otp_data[3], otp_data[4]);
	pr_err("%s, is otp data vaild=%d \n", __func__, otp_data[otp_length-1]);

	otp_info->otp_vaild = 1; //otp_data[otp_length-1]; TODO later
	if(otp_info->otp_vaild)
	{
		otp_info->af_otp.start_code = 
			otp_data[otp_info->af_otp.index_start] << 8
			| otp_data[otp_info->af_otp.index_start +1];
		otp_info->af_otp.max_code = 
			otp_data[otp_info->af_otp.index_start + 2] << 8
			| otp_data[otp_info->af_otp.index_start + 3];
		pr_err("vcm current:  start=%d, max=%d", 
			otp_info->af_otp.start_code,
			otp_info->af_otp.max_code);
	}

free_otp_data:
	kfree(otp_data);

	return ;
}
#endif

static void misp_dump_ram(struct misp_data *pdata,u32 start, u32 total, u32 mode)
{
	mm_segment_t oldfs = get_fs();
	struct file *fp = NULL;
	u8 param[16], ctrl_byte = USPICTRL_MS_CB_DIS;
	u32 *paddr=(u32*)&param[0], *ptotal=(u32*)&param[4], *pblock=(u32*)&param[8], *pmode=(u32*)&param[12];
	int ret, one_size, left;
	char name[64];

	pr_info("%s start=0x%x, total=%u, mode=%u", __func__, start, total, mode);

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "/data/k3_camera/misp_0x%08x.ram", start);

	set_fs(KERNEL_DS);
	fp = filp_open(name, O_CREAT|O_TRUNC|O_WRONLY, 0666);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s create ram bin file error", __func__);
		set_fs(oldfs);
		return;
	}

	if (total == 0)
		total = 936*1024;
	memset(param, 0, sizeof(param));
	*paddr = start;
	*ptotal = total;
	*pblock = SPI_BLOCK_BUF_SIZE;
	*pmode = mode;

	if(mutex_lock_interruptible(&pdata->busy_lock))
		return ;

	pdata->state = MINI_ISP_STATE_IDLE;
	ret= misp_send_cmd(pdata, ISPCMD_BULK_READ_MEMORY, param, sizeof(param));
	if (ret)
		goto out;

	ret = wait_event_interruptible_timeout(pdata->wait_queue,\
						(pdata->state == MINI_ISP_STATE_READY),\
						MINI_ISP_WAIT_TIMEOUT);

	if (ret <= 0) {
		pr_err("%s wait ready signal timeout", __func__);
		goto out;
	}

	/* disable interrpt to avoid messages broken */
	disable_irq(pdata->spi->irq);
	pdata->ext_buf[0] = ctrl_byte;
	spi_write(pdata->spi, pdata->ext_buf, sizeof(ctrl_byte));
	for (left = total; left > 0; left -= SPI_BLOCK_BUF_SIZE) {
		one_size = (left > SPI_BLOCK_BUF_SIZE)? SPI_BLOCK_BUF_SIZE : left;
		spi_read(pdata->spi, pdata->rx_buf, one_size);
		vfs_write(fp, pdata->rx_buf, one_size, &fp->f_pos);
	}
	enable_irq(pdata->spi->irq);

out:
	mutex_unlock(&pdata->busy_lock);
	filp_close(fp, NULL);
	set_fs(oldfs);
	return;
}

static void misp_write_reg(struct misp_data *pdata,
										u32 addr, u32 val)
{
	u8 param[8];
	u32 *paddr=(u32*)&param[0], *pval=(u32*)&param[4];

	pr_info("%s addr=0x%x, val=0x%x", __func__, addr, val);

	*paddr = addr;
	*pval = val;
	if(mutex_lock_interruptible(&pdata->busy_lock))
		return ;
	misp_send_cmd(pdata, ISPCMD_SYSTEM_SET_ISPREGISTER, param, sizeof(param));
	mutex_unlock(&pdata->busy_lock);
}

static void misp_test_spi(struct misp_data *pdata)
{
	int index;
	struct spi_transfer	xfer = {
		.tx_buf = pdata->tx_buf,
		.rx_buf = pdata->rx_buf,
		.bits_per_word = 8,
		.len = SPI_TX_BUF_SIZE,
	};
	struct spi_message m;

	if(mutex_lock_interruptible(&pdata->busy_lock))
		return ;

	for (index = 0; index < SPI_TX_BUF_SIZE; index++) {
		get_random_bytes(&(pdata->tx_buf[index]), 1);
		pdata->rx_buf[index] = 0;
	}

	spi_message_init(&m);
	spi_message_add_tail(&xfer, &m);
	spi_sync(pdata->spi, &m);

	mutex_unlock(&pdata->busy_lock);

	if (pdata->debug_config & MINI_ISP_DEBUG_SPI_DATA) {
		pr_info("%s - send buf len=%d:", __func__, SPI_TX_BUF_SIZE);
		spi_data_debug(pdata->tx_buf, SPI_TX_BUF_SIZE, SPI_TX_BUF_SIZE);
		pr_info("%s - recv buf len=%d:", __func__, SPI_RX_BUF_SIZE);
		spi_data_debug(pdata->rx_buf, SPI_TX_BUF_SIZE, SPI_TX_BUF_SIZE);
	}
}

static ssize_t misp_config_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	u32 mode = 0, bits = 0, speed = 0;
	u32 new_log_config = 0, new_log_level = 0;
	u32 dump_reg_table_count = 0, *dump_reg_table = NULL;
	u32 *default_reg_table;
	u32 default_reg_table_count;
	u32 dump_ram_start = 0, dump_ram_total = 0, dump_ram_mode = 0;
	u32 write_reg_addr = 0, write_reg_val = 0;
	struct misp_data *pdata = NULL;
	char *pos;

	pr_info("%s - enter", __func__);

	pdata = (struct misp_data *)dev_get_drvdata(dev);
	if (NULL == pdata) {
		pr_err("%s - get pdata error", __func__);
		return 0;
	}
   	default_reg_table = misp_al6010_reg_table;
	default_reg_table_count = ARRAY_SIZE(misp_al6010_reg_table);

	if (count == 0)
		return 0;

	pos = (char*)buf;
	if (0 == strncmp("spi_config", pos, strlen("spi_config"))) {
		while ((*pos) && (!isdigit(*pos)))
			pos++;
		sscanf(pos, "0x%x %u %u", &mode, &bits, &speed);
		misp_config_spi(pdata, mode, bits, speed);
	} else if (0 == strncmp("log_config", pos, strlen("log_config"))) {
		while ((*pos) && (!isdigit(*pos)))
			pos++;
		sscanf(pos, "%u %u", &new_log_config, &new_log_level);
		misp_config_log(pdata, new_log_config, new_log_level);
	} else if (0 == strncmp("dump_reg", pos, strlen("dump_reg"))) {
		pos += strlen("dump_reg");
		if (*pos == '=')
			paser_reg_table(pos, &dump_reg_table, &dump_reg_table_count);
		if (dump_reg_table == NULL) {
			misp_dump_reg(pdata, default_reg_table, default_reg_table_count);
		} else {
			misp_dump_reg(pdata, dump_reg_table, dump_reg_table_count);
			kfree(dump_reg_table);
		}
	} else if (0 == strncmp("dump_ram", pos, strlen("dump_ram"))) {
		while ((*pos) && (!isdigit(*pos)))
			pos++;
		sscanf(pos, "0x%x %u %u", &dump_ram_start, &dump_ram_total, &dump_ram_mode);
		misp_dump_ram(pdata, dump_ram_start, dump_ram_total, dump_ram_mode);
	} else if (0 == strncmp("write_reg", pos, strlen("write_reg"))) {
		while ((*pos) && (!isdigit(*pos)))
			pos++;
		sscanf(pos, "0x%x 0x%x", &write_reg_addr, &write_reg_val);
		misp_write_reg(pdata, write_reg_addr, write_reg_val);
	} else if (0 == strncmp("test_spi", pos, strlen("test_spi"))) {
		misp_test_spi(pdata);
	}

	return (ssize_t)count;
}

/* MISP SELF-TEST BEGIN */
static ssize_t misp_test_pipe_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t misp_test_pipe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(test_pipe, 0664, misp_test_pipe_show, misp_test_pipe_store);

static ssize_t misp_test_pipe_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0, test_pipe_id = -1, index = 0;
	u8 in_buf[7], out_buf[53];
	u8 sizeout = 0;
	u16 opcode = 0;
	const char *pos = buf;
	int  extisp_type = EXTISP_NULL;
	struct misp_askdata_setting out_setting;

	pr_err("%s enter %s", __func__, buf);
	//extisp_type = misp_get_chipid();
	extisp_type = EXTISP_AL6010;         //set to 6010 until get_chipid func OK.

	if (0 == strncmp("DBC_Begin", pos, strlen("DBC_Begin")))
	{
		//misp_cmd_filter = 1;
		pr_err("%s DBC BDTEST Begin!\n",__func__);
		return count;
	}

	if (0 == strncmp("DBC_End", pos, strlen("DBC_End")))
	{
		//misp_cmd_filter = 0;
		pr_err("%s DBC BDTEST End!\n",__func__);
		return count;
	}



	/* input:test_pipe=0 test_pipe=1 test_pipe=2 */
	if (0 == strncmp("test_pipe", pos, strlen("test_pipe"))) {
		while (*pos) {
			if (isdigit(*pos))
				break;
			else
				pos++;
		}
	}

	if (*pos == '0' ) {
		test_pipe_id = ALTEK6045_PIPE_0;
	} else if (*pos == '1') {
		test_pipe_id = ALTEK6045_PIPE_1;
	} else if (*pos == '2') {
		test_pipe_id = ALTEK6045_PIPE_DUAL;
	} else {
		//	test_pipe_id = -1;
		pr_err("%s invalid argument\n", __func__);
		goto err;
	}

	set_test_result(test_pipe_id, ALTEK6045_PIPE_TESTING);

	/* start test mode */
	memset(in_buf, 0, sizeof(in_buf));
	if (test_pipe_id == ALTEK6045_PIPE_DUAL) {
		in_buf[0] = in_buf[2] = 1;
		in_buf[1] = in_buf[3] = 99;
	} else {
		if(extisp_type == EXTISP_AL6045){
			in_buf[test_pipe_id * 2] = 1;
			in_buf[test_pipe_id * 2 + 1] = 99;
			sizeout = 33;
		}else{
			in_buf[0] = 1;
			if(test_pipe_id == ALTEK6045_PIPE_0) {
				in_buf[1]=101;
			} else {
				in_buf[1]=100;
			}
			sizeout = 53;
		}
	}
	opcode = ISPCMD_CAMERA_SET_SENSORMODE;
	ret = misp_write_cmd(opcode,  in_buf, sizeof(in_buf));
	pr_err("%s misp_write_cmd: opcode = 0x%x,   ret = %d", __func__, opcode, ret);
	spi_data_debug(in_buf, 7, sizeof(in_buf));

	if (ret) {
		set_test_result(test_pipe_id, ALTEK6045_PIPE_TEST_CMD_ERR);
		pr_err("%s set test mode cmd failed ret:%d\n", __func__, ret);
		goto err;
	}

	msleep(1000);

	/* get test mode */
	memset(&out_setting, 0, sizeof(struct misp_askdata_setting));
	memset(out_buf, 0, sizeout);
	out_setting.opcode = ISPCMD_GET_BULK_CHIPTEST_REPORT;
	out_setting.asklen = 0;
	out_setting.askparam = NULL;
	out_setting.is_block_data = false;
	out_setting.recvparam = out_buf;
	out_setting.recvlen = sizeout;
	out_setting.wait_state = MINI_ISP_STATE_READY;

	ret = misp_askdata_cmd(out_setting);
	pr_err("%s misp_askdata_cmd: opcode = 0x%x, with no in_param,  ret = %d\n", __func__, out_setting.opcode, ret );
	spi_data_debug(out_buf, sizeout, sizeout);
	if (ret) {
		set_test_result(test_pipe_id, ALTEK6045_PIPE_GET_CMD_ERR);
		pr_err("%s get test result cmd failed ret:%d", __func__, ret);
		goto err;
	}

	for (index = 0; index < sizeout; index++) {
		if(out_buf[index] != 1) {
			set_test_result(test_pipe_id, ALTEK6045_PIPE_TEST_BAD);
			goto err;
		}
	}
	set_test_result(test_pipe_id, ALTEK6045_PIPE_TEST_DONE);

err:
	msleep(100);
	pr_err("%s exit", __func__);
	return count;
}


static ssize_t misp_test_pipe_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *offset = buf;
	int ret, index, stage;
	const char *report;

	pr_err("%s enter", __func__);

	for (index = ALTEK6045_PIPE_0; index < ALTEK6045_PIPE_MAX; index++) {
		stage = test_result[index];
		report = test_report[stage];
		ret = snprintf(offset, PAGE_SIZE, "[PIPE%d:%s] ", index, report);
		offset += ret;
	}
	pr_err("%s:%s",  __func__, buf);
	memset(test_result, 0, sizeof(test_result));


	ret = snprintf(offset, PAGE_SIZE, "\n");
	offset += ret;

	pr_err("%s exit", __func__);
	return (offset - buf);
}
/* MISP SELF-TEST END */

static const struct of_device_id msm_miniisp_dt_match[] = {
	{
		.compatible = "hw,mini_isp",
	},
	{}
};
MODULE_DEVICE_TABLE(of, msm_miniisp_dt_match);


static int  msm_dt_to_pdata_populate(struct  device_node *of_node,
					struct msm_dt_to_pdata_map  *itr)
{
	int  ret, err = 0;
	struct device_node *node = of_node;

	for (; itr->dt_name; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(node, itr->dt_name, 0);
			if (ret >= 0) {
				*((int*) itr->ptr_data) = ret;
				ret = 0;
			}
			pr_err("DT entry ret:%d name:%s val:%d\n",	ret, itr->dt_name, *((int *)itr->ptr_data));
			break;
		case DT_U32:
			ret = of_property_read_u32(node, itr->dt_name, (u32 *) itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) =of_property_read_bool(node, itr->dt_name);
			ret = 0;
			break;
		default:
			pr_err("%d is an unknown DT entry type\n",	itr->type);
			ret = -EBADE;
		}
		if (ret) {
			pr_err("Missing '%s' DT entry\n",itr->dt_name);
		}
	}
	return err;
}


/*get isp config from dts include spi config*/
static int32_t mini_isp_driver_get_dt_data(struct device_node *of_node, struct misp_data* mispdata  )	
{
	struct device_node *node=of_node;
	/*load miniisp config*/
	struct msm_dt_to_pdata_map mini_map[] ={
	{"hw,isp_gpio_reset",&mispdata->plat_data->reset_gpio, DT_GPIO, -1},
	{NULL,  NULL,       0,        0},
	};
	if (msm_dt_to_pdata_populate(node , mini_map)) {
		return -1;	
	}
	/*load spi config*/
	return 0;
}

static int mini_isp_probe(struct spi_device *spi)
{
	struct misp_data *drv_data = NULL;
	struct misp_plat_data *plat_data = NULL;
	const struct of_device_id *match_dev;
	int ret = 0;

	plat_data = kmalloc(sizeof(struct misp_plat_data), GFP_KERNEL);
	spi->dev.platform_data=(void*)plat_data;
	if (!plat_data) {
		pr_err("probe - no plat data");
		return -ENODEV;
	}

	/* step 1: alloc driver data struct */
	drv_data = kmalloc(sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data) {
		pr_err("probe - can not alloc driver data");
		return -ENOMEM;
	}
	memset(drv_data, 0, sizeof(*drv_data));
	drv_data->plat_data = plat_data;

	if (spi->dev.of_node) {
		match_dev = of_match_device(msm_miniisp_dt_match, &spi->dev);
		if (!match_dev) {
			pr_err("%s: No vfe hardware info\n", __func__);
			return -EINVAL;
		}
		mini_isp_driver_get_dt_data(spi->dev.of_node,drv_data);
	}
	
	/*change tx_buf  to 16K in order to raise performance*/
	drv_data->tx_buf  = (void *)__get_free_pages(GFP_KERNEL|GFP_DMA, ALLOC_PAGES_SIZE);
	drv_data->rx_buf  = (void *)__get_free_pages(GFP_KERNEL|GFP_DMA, get_order(PAGE_SIZE));
	drv_data->ext_buf = (void *)__get_free_pages(GFP_KERNEL|GFP_DMA, get_order(PAGE_SIZE));
	drv_data->irq_buf = (void *)__get_free_pages(GFP_KERNEL|GFP_DMA, get_order(PAGE_SIZE));
	pr_err("probe - tx_buf=0x%p rx_buf=0x%p ext_buf=0x%p, irq_buf=0x%p\n",
		drv_data->tx_buf, drv_data->rx_buf, drv_data->ext_buf, drv_data->irq_buf);
	if ((!drv_data->tx_buf) || (!drv_data->rx_buf) ||
		(!drv_data->ext_buf)|| (!drv_data->irq_buf)) {
		pr_err("probe - can not alloc dma buf page");
		ret = -ENOMEM;
		goto err_alloc_buf;
	}

	/* step 2: init driver data */
	
	drv_data->spi = spi;
	drv_data->state = MINI_ISP_STATE_IDLE;
	drv_data->debug_config = 0;
	mutex_init(&drv_data->busy_lock);
	INIT_WORK(&drv_data->dump_log_work, misp_dump_log);
	INIT_WORK(&drv_data->dump_err_work, misp_dump_err);
	/*fix ois running test fail issue*/
	INIT_WORK(&drv_data->ois_init_done_work, misp_ois_init_done_work);
	init_waitqueue_head(&drv_data->wait_queue);
	init_waitqueue_head(&drv_data->sync_queue);
	init_waitqueue_head(&drv_data->pwdn_queue);
	/*add camera ois driver*/
	init_waitqueue_head(&drv_data->ois_queue);
	drv_data->work_queue = create_singlethread_workqueue(
									dev_name(&spi->dev));
	if (!drv_data->work_queue) {
		pr_err("probe - create workqueue error");
		ret = -EBUSY;
		goto err_create_queue;
	}

	/* step 3: setup spi */
	spi->max_speed_hz = MINI_ISP_SPI_SPEED_BOOT;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	pr_err("%s: irq[%d] cs[%x] csgpio[%d] CPHA[%x] CPOL[%x] CS_HIGH[%x]\n",
                        __func__, spi->irq,spi->chip_select,spi->cs_gpio, (spi->mode & SPI_CPHA) ? 1 : 0, (spi->mode & SPI_CPOL) ? 1 : 0,  (spi->mode & SPI_CS_HIGH) ? 1 : 0);
	
	if (ret<0) {
		pr_err("probe - setup spi error");
		goto err_spi_setup;
	}

	ret = gpio_request(plat_data->reset_gpio, "mini_isp_reset");
	if (ret) {
		pr_err("probe - request reset gpio error");
		goto err_reset_gpio;
	}

	ret = request_threaded_irq(spi->irq, NULL, misp_irq_thread,
							IRQF_ONESHOT | IRQF_TRIGGER_RISING,
							"mini_isp", drv_data);
	if (ret) {
		pr_err("probe - request irq error");
		goto err_irq_config;
	}
	/* setp 5:other additional config */
	ret = device_create_file(&spi->dev, &dev_attr_misp_config);
	if (ret) {
	    pr_err("probe - create dev attr file fail");
		goto err_dev_attr;
	}
	/* MISP SELF-TEST BEGIN */
	ret = device_create_file(&spi->dev, &dev_attr_test_pipe);
	if (ret) {
	    pr_err("probe - create dev attr file fail");
		goto err_dev_attr;
	}
	/* MISP SELF-TEST END */
	/* setp 6: set driver_data to device */
	spi_set_drvdata(spi, drv_data);

	misp_drv_data = drv_data;

	pr_err("mini isp probe success\n ");
	return ret;

err_dev_attr:
	free_irq(spi->irq, drv_data);
err_irq_config:	
gpio_free(plat_data->reset_gpio);
err_reset_gpio:	
err_spi_setup:
	destroy_workqueue(drv_data->work_queue);
err_create_queue:
	free_pages((unsigned long)drv_data->tx_buf,  ALLOC_PAGES_SIZE);
	free_pages((unsigned long)drv_data->rx_buf,  get_order(PAGE_SIZE));
	free_pages((unsigned long)drv_data->ext_buf, get_order(PAGE_SIZE));
	free_pages((unsigned long)drv_data->irq_buf, get_order(PAGE_SIZE));
err_alloc_buf:
	kfree(drv_data);
	misp_drv_data = NULL;
	return ret;
}


static struct spi_driver mini_isp_drv = {
	.driver = {
		.name =         "mini_isp",
		.owner =        THIS_MODULE,
		.of_match_table = msm_miniisp_dt_match,
	},
	.probe =        mini_isp_probe,
	.remove =       NULL,
};

static int __init mini_isp_init(void)
{
	return spi_register_driver(&mini_isp_drv);
}

static void __exit mini_isp_exit(void)
{
	spi_unregister_driver(&mini_isp_drv);
}

module_init(mini_isp_init);
module_exit(mini_isp_exit);
MODULE_LICENSE("Huawei/GPL");
