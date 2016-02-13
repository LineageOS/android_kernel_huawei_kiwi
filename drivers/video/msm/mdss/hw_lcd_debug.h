
/* kernel\drivers\video\msm\lcd_hw_debug.h
 * this file is used by the driver team to change the 
 * LCD init parameters by putting a config file in the mobile,
 * this function can make the LCD parameter debug easier.
 * 
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2010/12/10
 * By genghua
 * 
 */

#ifndef __HW_LCD_DEBUG__
#define __HW_LCD_DEBUG__
#include <linux/syscalls.h>
#include <linux/slab.h>
#include "mdss_dsi.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define HW_LCD_INIT_TEST_PARAM "/data/hw_lcd_init_param.txt"
#define HW_LCD_CONFIG_TABLE_MAX_NUM 2*PAGE_SIZE
int mdss_dsi_check_panel_status_n(struct mdss_panel_data *pdata);
int hw_parse_dsi_cmds(struct dsi_panel_cmds *pcmds);
bool hw_free_dsi_cmds(struct dsi_panel_cmds *pcmds);
#ifdef CONFIG_DEBUG_FS
void lcd_dbg_set_dsi_ctrl_pdata(struct mdss_dsi_ctrl_pdata *ctrl);
struct mdss_dsi_ctrl_pdata *lcd_dbg_get_dsi_ctrl_pdata(void);
int lcd_dbg_mipi_prcess_ic_reg(int op_type,int reg, int cmd_type, int param_num, char *param_buf,int *read_value, int delay_ms);
int lcd_debugfs_init(void);
#endif

#endif 


