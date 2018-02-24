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

#ifndef HW_LCD_COMMON_H
#define HW_LCD_COMMON_H

#define OPER_READ  (1)
#define OPER_WRITE (2)
#define MIPI_DCS_COMMAND (1<<0)
#define MIPI_GEN_COMMAND 4
#define MIPI_PATH_OPEN		1
#define MIPI_PATH_CLOSE	0

int get_tp_gesture_enable_status(void);
void set_tp_gesture_enable_status(int type);

/*add power status error judge,avoid red screen*/
struct lcd_pwr_status_t
{
	int panel_power_on;
	int lcd_dcm_pwr_status;
	struct tm tm_unblank;
	struct timeval tvl_unblank;
	struct tm tm_lcd_on;
	struct timeval tvl_lcd_on;
	struct tm tm_set_frame;
	struct timeval tvl_set_frame;
	struct tm tm_backlight;
	struct timeval tvl_backlight;
};
extern int lcd_dcm_pwr_status;
extern struct lcd_pwr_status_t lcd_pwr_status;

#define  LCD_PWR_STAT_GOOD 0x000f
#define  LCD_PWR_STAT_IS_GOOD 0x0000

/* LCD_MDELAY will select mdelay or msleep according value */
#define LCD_MDELAY(time_ms)   	\
	do							\
	{ 							\
		if (time_ms>10)			\
			msleep(time_ms);	\
		else					\
			mdelay(time_ms);	\
	}while(0)

#endif
