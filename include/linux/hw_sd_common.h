/* Copyright (c), Code HUAWEI. All rights reserved.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef HW_EMMCSD_COMMON_H
#define HW_EMMCSD_COMMON_H

/* Add dynamic_log interface */
#define EMMC_SD_ERR  1
#define EMMC_SD_INFO 2
#define EMMC_SD_DBG  3

extern int emmcsd_debug_mask;

#ifndef EMMCSD_LOG_ERR
#define EMMCSD_LOG_ERR( x...)					\
do{											\
	if( emmcsd_debug_mask >= EMMC_SD_ERR )			\
	{										\
		printk(KERN_ERR "[EMMC_SD_ERR] " x);	\
	}										\
											\
}while(0)
#endif

#ifndef EMMCSD_LOG_INFO
#define EMMCSD_LOG_INFO( x...)					\
do{											\
	if( emmcsd_debug_mask >= EMMC_SD_INFO )		\
	{										\
		printk(KERN_ERR "[EMMC_SD_INFO] " x);	\
	}										\
											\
}while(0)
#endif

#ifndef EMMCSD_LOG_DBG
#define EMMCSD_LOG_DBG( x...)					\
do{											\
	if( emmcsd_debug_mask >= EMMC_SD_DBG )			\
	{										\
		printk(KERN_ERR "[EMMC_SD_DBG] " x);	\
	}										\
											\
}while(0)
#endif
#endif
