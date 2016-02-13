/*
 * miniisp sensor agent
 *
 *  Author: 	Zhoujie (zhou.jie1981@163.com)
 *  Date:  	2013/10/28
 *  Version:	1.0
 *  History:	2013/10/28      Frist add driver for miniisp
 *
 * ----------------------------------------------------------------------------
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include "camera_agent.h"
#include "mini_isp.h"

#define LOG_TAG "camera_agent"
#define DEBUG_DEBUG 1


/*
 **************************************************************************
 * FunctionName: camera_agent_stream_on;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
#define ERR_SUCCESS                     0 // No error
#define SET_CMD					1
#define GET_CMD					0
int camera_agent_streamonoff(int32_t pos , u8 state)
{
	u16 OpCode = ISPCMD_CAMERA_PREVIEWSTREAMONOFF;
	u32 cmd;
	u8 buf[3] = {0,0,0};
	int errorcode;

	pr_err("Enter Function:%s=[%s]", __func__, state?"on":"off");

	cmd = misp_construct_opcode(OpCode,SET_CMD,sizeof(buf));

	buf[pos] = state;

	errorcode = misp_exec_cmd(cmd,buf);
	if (errorcode) {
		pr_err("%s fail, error code = %d", __func__, errorcode);
		return -1;
	}
	return 0;
}


