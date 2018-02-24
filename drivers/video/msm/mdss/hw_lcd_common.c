/* add cmi lcd driver */
/* Copyright (c) 2009, Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include<linux/init.h>
#include<linux/module.h>

#include <linux/hw_lcd_common.h>
#include <mdss_dsi.h>
#include <linux/of.h>
#include "mdss_panel.h"
#include <linux/sched.h>
#include <asm/setup.h>

#ifdef CONFIG_DEBUG_FS

static int g_tp_gesture_enable_status = false;
static struct mdss_dsi_ctrl_pdata *g_lcd_dbg_dsi_ctrl_pdata = NULL;    // global mdss_dsi_ctrl_pdata

/*
*merge qcom patch from 02098626: 08_20_patches.zip
*add delay time before vddio-incell enable. if vddio-incell pull down time is smaller then 80ms.
*/
static unsigned long vddio_incell_poweroff_time = 0;

/* FPC unlock can't light lcd backlight */
static int lcd_power_delay_time = false;

unsigned long get_tp_vddio_poweroff_time(void)
{
	return vddio_incell_poweroff_time;
}
void set_tp_vddio_poweroff_time(unsigned long jz)
{
	vddio_incell_poweroff_time = jz;
}

/* get tp_gesture_enable or not */
int get_tp_gesture_enable_status(void)
{
	return g_tp_gesture_enable_status;
}
/*tp_gesture_enable:drivers use to set tp_gesture_enable*/
void set_tp_gesture_enable_status(int type)
{
	g_tp_gesture_enable_status = type;
	pr_info("%s:tp_type=%d\n",__func__,type);
}

/* FPC unlock can't light lcd backlight */
/* get lcd power delay time */
int get_lcd_power_delay_time(void)
{
	return lcd_power_delay_time;
}
/* set lcd power delay time */
void set_lcd_power_delay_time(int delay_timeValue)
{
	lcd_power_delay_time = delay_timeValue;
	pr_info("%s:lcd power delay time is : %d\n",__func__,delay_timeValue);
}

/* set global ctrl_pdata pointer */
void lcd_dbg_set_dsi_ctrl_pdata(struct mdss_dsi_ctrl_pdata *ctrl)
{
	static char already_set = 0;

	/* judge if already set or not*/
	if (already_set)
	{
		pr_err("%s: already set\n", __func__);
	}
	else
	{
		g_lcd_dbg_dsi_ctrl_pdata = ctrl;
		already_set = 1;
	}

	return;
}

/* get global ctrl_pdata pointer */
struct mdss_dsi_ctrl_pdata *lcd_dbg_get_dsi_ctrl_pdata(void)
{
	return g_lcd_dbg_dsi_ctrl_pdata;
}

/* check whether mipi input is legal or not */
/* return: 0 - success, negative - fail */
static int is_mipi_input_legal(int op_type,int ic_reg, int cmd_type, int param_num,char *buf)
{
	int ret = 0;
	if( (op_type != OPER_READ) && (op_type != OPER_WRITE))
	{
		ret = -1;
	}
	/* ic_reg must in [0x00, 0xff] */
	if (!((unsigned int)ic_reg >= 0 && (unsigned int)ic_reg <= 0xff))
	{
		ret = -1;
	}

	/* cmd_type must be 0x01 or 0x04 */
	if ((cmd_type != MIPI_DCS_COMMAND) &&(cmd_type != MIPI_GEN_COMMAND))
	{
		ret = -1;
	}
	/* param_num must larger or equal 0 */
	if (param_num < 0)
	{
		ret = -1;
	}

	if(NULL == buf)
	{
		ret = -1;
	}
	return ret;
}


/**********************************************************************************
*function:process read and write commands  for lcd reg debug
*op_type:	read or write
*reg:		lcd register
*cmd_type:	DCS or GEN
*param_num:	the count of prameters to tranfer
*param_buf:	prameters
*read_value:  value from lcd panel
*delay_ms:	delay time
*return: 0 - success, negative - fail
**********************************************************************************/
int lcd_dbg_mipi_prcess_ic_reg(int op_type,int reg, int cmd_type, int param_num, char *param_buf,int *read_value, int delay_ms)
{
	static struct dcs_cmd_req cmdreq;
	static struct dsi_cmd_desc dsi_cmd;                  // dsi cmd struct
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	/* check if input legal */
	if (is_mipi_input_legal(op_type,reg, cmd_type, param_num,param_buf))
	{
		pr_err("%s, input illegal.\n", __func__);
		return -1;
	}

	ctrl = lcd_dbg_get_dsi_ctrl_pdata();
	/* translate cmd_type from huawei to qcom's format */
	switch (param_num)
	{
		case 0:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_READ;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_WRITE;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_WRITE1;
				}
			}
			break;
		}

		case 1:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					pr_err("%s ,not support this kind of dcs read! \n",__func__);
					return -1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ1;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_WRITE1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_WRITE2;
				}
			}
			break;
		}

		default:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					pr_err("%s ,not support this kind of dcs read! \n",__func__);
					return -1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ2;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_LWRITE;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_LWRITE;
				}
			}
			break;
		}
	}

	/* insert reg into param_buf's beginning */
	memmove(param_buf + 1, param_buf, param_num);
	param_buf[0] = reg;
	param_num++;

	dsi_cmd.dchdr.dtype = cmd_type;
	dsi_cmd.dchdr.last =1;
	dsi_cmd.dchdr.vc = 0;
	dsi_cmd.dchdr.dlen = param_num;
	dsi_cmd.payload = param_buf;
	memset(&cmdreq, 0, sizeof(cmdreq));
	switch(op_type)
	{
		case OPER_READ:
			dsi_cmd.dchdr.ack = 1;
			dsi_cmd.dchdr.wait = 5;//5 ms
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			cmdreq.rbuf = (char*)read_value;
			break;
		case OPER_WRITE:
			dsi_cmd.dchdr.ack = 0;
			dsi_cmd.dchdr.wait = delay_ms;//5 ms
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rbuf = NULL;
			break;
	}
	cmdreq.cmds = &dsi_cmd;
	cmdreq.cmds_cnt = 1;
	if (ctrl->long_read_flag)
	{
		cmdreq.rlen = 3;
	}else
	{
		cmdreq.rlen = 1;
	}
	cmdreq.cb = NULL; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return 0;
}
#endif
