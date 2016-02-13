/*
 *  miniisp camera driver head file
 *
 *  Author: 	Zhoujie (zhou.jie1981@163.com)
 *  Date:  	2013/10/29
 *  Version:	1.0
 *  History:	2013/10/29      Frist add driver for miniisp 
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

/************************** END ***************************/

#ifndef __CAMERA_AGENT_H__
#define __CAMERA_AGENT_H__

#include <linux/workqueue.h>
#include "mini_isp.h"

int camera_agent_streamonoff(int32_t pos, u8 state);
#endif
