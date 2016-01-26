/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#include <linux/init.h>
#include <linux/io.h>

#include "imx278_lgit_ois.h"
#include "imx278_lgit_ois_data.h"
#include "../mini_isp_ois_interface.h"

#if 0
#define RamWrite32A         mini_isp_ois_RamWrite32A
#define RamRead32A          mini_isp_ois_RamRead32A
#define RamWriteA           mini_isp_ois_RamWrite16A
#define RamReadA            mini_isp_ois_RamRead16A
#define RegWriteA           mini_isp_ois_RegWrite8A
#define RegReadA            mini_isp_ois_RegRead8A
#define WitTim              mini_isp_ois_WitTim
#define CntWrtRam           mini_isp_ois_RamWrite16Burst
#define CntWrtRag           mini_isp_ois_RegWrite8Burst
#define CntWrtRam32         mini_isp_ois_RamWrite32Burst
#endif

/*
    all interface:
        success return 0
        fail return -1
*/
#define RamWrite32A(a,b)   do{ \
    if(mini_isp_ois_RamWrite32A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RamRead32A(a,b)          do{ \
    if(mini_isp_ois_RamRead32A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RamWriteA(a,b)           do{ \
    if(mini_isp_ois_RamWrite16A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RamReadA(a,b)            do{ \
    if(mini_isp_ois_RamRead16A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RegWriteA(a,b)           do{ \
    if(mini_isp_ois_RegWrite8A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RegReadA(a,b)            do{ \
    if(mini_isp_ois_RegRead8A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define WitTim              mini_isp_ois_WitTim

#define CntWrtRam(a,b)            do{ \
    if(mini_isp_ois_RamWrite16Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define CntWrtRag(a,b)           do{ \
    if(mini_isp_ois_RegWrite8Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define CntWrtRam32(a,b)         do{ \
    if(mini_isp_ois_RamWrite32Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define CHECK_RETURN(a) do{ \
    if(a != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0);

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 *
 *
 *      Source from Ois.h
 *
 *
 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
/****************************** Define説明 ******************************/
/*	USE_3WIRE_DGYRO		Digital Gyro I/F 3線Mode使用					*/
/*	USE_INVENSENSE		Invensense Digital Gyro使用						*/
/*		USE_IDG2020		Inv IDG-2020使用								*/
/*	STANDBY_MODE		Standby制御使用(未確認)							*/
/*	GAIN_CONT			:Gain control機能使用							*/
/*		(disable)		DSC			:三脚Mode使用						*/
/************************************************************************/

/**************** Select Gyro Sensor **************/
//#define 	USE_3WIRE_DGYRO    //for D-Gyro SPI interface

#define		USE_INVENSENSE				// INVENSENSE
//#define		USE_STMICRO_L2G2IS			// STMicro-L2G2IS


#ifdef USE_INVENSENSE
//			#define		FS_SEL		0		/* ±262LSB/°/s  */
//			#define		FS_SEL		1		/* ±131LSB/°/s  */
			#define		FS_SEL		2		/* ±65.5LSB/°/s  */
//          #define		FS_SEL		3		/* ±32.8LSB/°/s  */

//			#define		GYROSTBY			/* Sleep+STBY */
#endif

/**************** Model name *****************/
//#define MDL_VER         0x09		// P13N12A  

/**************** FW version *****************/
//#define	FW_VER			0x02        

/**************** Select Mode **************/
#define		STANDBY_MODE		// STANDBY Mode
#define		GAIN_CONT			// Gain Control Mode
#define		PWM_BREAK			// PWM mode select (disable standby)

//#define		DEF_SET				// default value re-setting
//#define		USE_EXTCLK_ALL		// USE Ext clk for ALL
//#define		USE_EXTCLK_PWM		// USE Ext clk for PWM
//#define		USE_VH_SYNC			// USE V/H Sync for PWM
//#define		PWM_CAREER_TEST		// PWM_CAREER_TEST
#define		NEUTRAL_CENTER		// Upper Position Current 0mA Measurement
#define		H1COEF_CHANGER		/* H1 coef lvl chage */
#define		MONITOR_OFF			// default Monitor output
//#define		MODULE_CALIBRATION		// for module maker   use float
//#define		AF_PWMMODE			// AF Driver PWM mode
//#define		CATCHMODE               // Catch mode --Rex.Tang 20141029
#ifdef	CATCHMODE
  #define	CORRECT_1DEG			// Correct 1deg   disable 0.5deg
  #define	G_45G_INI	0x3E3FECFF	// 32.8/175 = 0.187428571 = -14.543[dB]
#endif	//CATCHMODE
#define		ACCEPTANCE					// Examination of Acceptance



// Command Status
#define		EXE_END		0x02		// Execute End (Adjust OK)
#define		EXE_HXADJ	0x06		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0A		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x12		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x22		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x42		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x82		// Adjust NG : Y Gyro NG (offset)
#define		EXE_CXADJ	0x102		// Adjust NG : X Lens Center NG
#define		EXE_CYADJ	0x202		// Adjust NG : Y Lens Center NG
#define		EXE_OCADJ	0x402		// Adjust NG : OSC Clock NG
#define		EXE_ERR		0x99		// Execute Error End

#ifdef	ACCEPTANCE
 // Hall Examination of Acceptance
 #define		EXE_HXMVER	0x06		// X Err
 #define		EXE_HYMVER	0x0A		// Y Err

 // Gyro Examination of Acceptance
 #define		EXE_GXABOVE	0x06		// X Above
 #define		EXE_GXBELOW	0x0A		// X Below
 #define		EXE_GYABOVE	0x12		// Y Above
 #define		EXE_GYBELOW	0x22		// Y Below
#endif	//ACCEPTANCE

// Common Define
#define	SUCCESS			0x00		// Success
#define	FAILURE			0x01		// Failure

#ifndef ON
 #define	ON				0x01		// ON
 #define	OFF				0x00		// OFF
#endif
 #define	SPC				0x02		// Special Mode

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction
#define	X2_DIR			0x10		// X Direction
#define	Y2_DIR			0x11		// Y Direction

//s#define	NOP_TIME		0.00004166F

#define		X_CURRENT_LMT	0x3E6B851F//0x3ECCCCCD	// 0.4	X-axis current limit
#define		Y_CURRENT_LMT	0x3E6B851F//0x3ECCCCCD	// 0.4	X-axis current limit	

#ifdef STANDBY_MODE
 // Standby mode
 #define		STB1_ON		0x00		// Standby1 ON
 #define		STB1_OFF	0x01		// Standby1 OFF
 #define		STB2_ON		0x02		// Standby2 ON
 #define		STB2_OFF	0x03		// Standby2 OFF
 #define		STB3_ON		0x04		// Standby3 ON
 #define		STB3_OFF	0x05		// Standby3 OFF
 #define		STB4_ON		0x06		// Standby4 ON			/* for Digital Gyro Read */
 #define		STB4_OFF	0x07		// Standby4 OFF
 #define		STB2_OISON	0x08		// Standby2 ON (only OIS)
 #define		STB2_OISOFF	0x09		// Standby2 OFF(only OIS)
 #define		STB2_AFON	0x0A		// Standby2 ON (only AF)
 #define		STB2_AFOFF	0x0B		// Standby2 OFF(only AF)
#endif


// OIS Adjust Parameter
 #define		DAHLXO_INI		0x0451		// Hall X Offset
 #define		DAHLXB_INI		0xF267		// Hall X Bias
 #define		DAHLYO_INI		0x0A74		// Hall Y Offset
 #define		DAHLYB_INI		0xE4EE		// Hall Y Bias
 #define		SXGAIN_INI		0x4185		// Hall X Loop Gain
 #define		SYGAIN_INI		0x4149		// Hall Y Loop Gain
 #define		HXOFF0Z_INI		0xF460		// Hall X AD Offset
 #define		HYOFF1Z_INI		0xEC50		// Hall Y AD Offset 
/* OSC Init */
 #define		OSC_INI			0x50		/* VDD=2.8V */
// Digital Gyro offset Initial value 
#define		DGYRO_OFST_XH	0x00 
#define		DGYRO_OFST_XL	0x00 
#define		DGYRO_OFST_YH	0x00 
#define		DGYRO_OFST_YL	0x00
 #define		GXGAIN_INI		0x3E94C4C5	// Gyro X Zoom Value 
 #define		GYGAIN_INI		0x3E8FF43F	// Gyro Y Zoom Value
 #define		BIAS_CUR_OIS     0x22        //1.0mA/1.0mA
 #define        AMP_GAIN_X      0x02         //x50
 #define        AMP_GAIN_Y      0x02         //x50


/* AF Open para */
#define			RWEXD1_L_AF		0x7FFF                 //
#define			RWEXD2_L_AF		0x6ACE                 //
#define			RWEXD3_L_AF		0x7000                 //
#define			FSTCTIME_AF		0x96                   //

#define			FSTMODE_AF		0x00                   //

 /* (0.3750114X^3+0.5937681X)*(0.3750114X^3+0.5937681X) 6.5ohm*/
// #define		A3_IEXP3		0x3EC0017F
// #define		A1_IEXP1		0x3F180130
/* (0.3750114X^3+0.55X)*(0.3750114X^3+0.55X) 9.2ohm*/
//#define         A3_IEXP3        0x3EC0017F
#define         A1_IEXP1        0x3F0CCCCD
 /* (0.425X^3+0.55X)*(0.425X^3+0.55X) 10.2ohm*/
/* (0.425X^3+0.55X)*(0.425X^3+0.55X) 10.5ohm*/
#define       A3_IEXP3        0x3ED9999A
  /* (0.4531388X^3+0.4531388X)*(0.4531388X^3+0.4531388X) 15ohm*/
//#define			A3_IEXP3		0x3EE801CF
//#define			A1_IEXP1		0x3EE801CF

/* AF adjust parameter */
//#define		DAHLZB_INI		0x8001		// abe 2014.06.03
//#define		DAHLZO_INI		0x0000
#define		BIAS_CUR_AF		0x00		//0.25mA
#define		AMP_GAIN_AF		0x00		//x6

#define		SXGAIN_LOP		0x3000
#define		SYGAIN_LOP		0x3000

//#define		TCODEH_ADJ		0x0000

#ifdef	CATCHMODE

 /************* Wide *************/
 #define		GYRLMT3_S1_W	0x3F0CCCCD		//0.55F
 #define		GYRLMT3_S2_W	0x3F0CCCCD		//0.55F
 #define		GYRLMT4_S1_W	0x40300000		//2.75F
 #define		GYRLMT4_S2_W	0x40300000		//2.75F
 #define		GYRISTP_W		0x3A51B700		/* -62dB */
 //#define		GYRA12_HGH_W	0x40333333		/* 2.80F */
 #define		GYRA12_HGH_W	0x402CCCCD		/* 2.70F */
 
 /************* Narrow *************/
 #define		GYRLMT3_S1		0x3ECCCCCD		//0.40F
 #define		GYRLMT3_S2		0x3ECCCCCD		//0.40F
 #define		GYRLMT4_S1		0x40000000		//2.0F
 #define		GYRLMT4_S2		0x40000000		//2.0F 
 #define		GYRISTP			0x39D1B700		/* -68dB */
 #define		GYRA12_HGH		0x3FE00000		/* 1.75F */

 /**********************************/
 #define		GYRLMT1H		0x3F800000		//1.0F

 #define		GYRA12_MID		0x3C23D70A		/* 0.01F */
 #define		GYRA34_HGH		0x3F000000		/* 0.5F */
 #define		GYRA34_MID		0x3C23D70A		/* 0.01F */

 #define		GYRB12_HGH		0x3E4CCCCD		/* 0.20F */
 #define		GYRB12_MID		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_HGH		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_MID		0x3C23D70A		/* 0.001F */

#else	//CATCHMODE

 /*** Hall, Gyro Parameter Setting ***/
 #define		GYRLMT1H		0x3E4CCCCD	// 0.2F
 
 #define		GYRLMT3_S1		0x3EDDB22D	// 0.433F  pm 1.4[deg]
 #define		GYRLMT3_S2		0x3EDDB22D	// 0.433F  pm 1.4[deg]
 
 #define		GYRLMT4_S1		0x41180000 	// 9.500F  pm 2.5[deg]
 #define		GYRLMT4_S2		0x41180000 	// 9.500F  pm 2.5[deg]
 
 #define		GYRA12_HGH		0x4108CCCD	// 8.550F  pm 2.25[deg]
 #define		GYRA12_MID		0x3F980000	// 1.188F  pm 0.313[deg]
 #define		GYRA34_HGH		0x40B00E0B	// 6.888F  pm 1.813[deg]
 #define		GYRA34_MID		0x40733333	// 3.800F  pm 1.000[deg]
 #define		GYRA34_MID_M	0x3F980000	// 1.188F  pm 0.313[deg] 
 #define		GYRB12_HGH		0x3E40B3DD	// 0.1882F  pm 94.000[dps]
 #define		GYRB12_MID		0x38D1EC3E	// 0.0001F  pm  0.05 [dps]
 #define		GYRB34_HGH		0x3E40B3DD	// 0.1882F  pm 94.000[dps]
 #define		GYRB34_MID		0x38D1EC3E	// 0.0001F  pm  0.05 [dps]

 #define		GYRISTP			0x39D1B700		/* -68dB */

#endif	//CATCHMODE
//#define		OPTCEN_X		0x0000
//#define		OPTCEN_Y		0x0000

#define			SXQ_INI			0x3F800000		/* Hall Filter Connection Setting(sxq, syq) */
#define			SYQ_INI			0x3F800000		// 0x3F800000 -> Positive, 0xBF800000 -> negative


#define			GYROX_INI		0x43	// Gyro X axis select
#define			GYROY_INI		0x45	// Gyro Y axis select

#define			GXHY_GYHX		0

/* Optical Center & Gyro Gain for Mode */
#define	VAL_SET				0x00		// Setting mode
#define	VAL_FIX				0x01		// Fix Set value
#define	VAL_SPC				0x02		// Special mode

#define	MEASSTR		0x01
#define	MEASCNT		0x08
#define	MEASFIX		0x80


#define		PWMMOD_CVL	0x00		// CVL PWM MODE
#define		PWMMOD_PWM	0x01		// PWM MODE

#define		INIT_PWMMODE	PWMMOD_CVL		// initial output mode

#define	CVER122		0x93		 // LC898122
#define	CVER122A	0xA1		 // LC898122A

// Prottype Declation
#define CLR_FRAM0		 	0x01
#define CLR_FRAM1 			0x02
#define CLR_ALL_RAM 		0x03

#define		SINEWAVE	0
#define		XHALWAVE	1
#define		YHALWAVE	2
#define		XACTTEST	10
#define		YACTTEST	11
#define		CIRCWAVE	255

#define		HALL_H_VAL	0x3F800000			/* 1.0 */
//#define		HALL_H_VAL	0x3F666666			/* 0.9 */

#define		PTP_BEFORE		0
#define		PTP_AFTER		1
#define		PTP_ACCEPT		2

#define		Mlnp		0					// Linear PWM
#define		Mpwm		1					// PWM

/* mode */
#define		S2MODE		0x40
#define		ACTMODE		0x80
#ifdef	CATCHMODE
 #define		MOVMODE		0xFE
 #define		MOVMODE_W	0xFF
 #define		STILLMODE	0x00
 #define		STILLMODE_W	0x01
#else
 #define		MOVMODE		0xFF
#endif 

#define		ACT_CHK_LVL		0x3ECCCCCD		// 0.4
#define		ACT_THR			0x6000			// 28dB 20log(4/(0.4*256))
#define		GEA_DIF_HIG		0x0010
#define		GEA_DIF_LOW		0x0001

// Dead Lock Check
#define READ_COUNT_NUM	3

static uint8_t     UcOscAdjFlg ;       // For Measure trigger
static uint16_t    UsCntXof ;              /* OPTICAL Center Xvalue */
static uint16_t    UsCntYof ;              /* OPTICAL Center Yvalue */
 
 //    uint8_t   UcCvrCod ;              /* CverCode */
 
// static uint32_t    UlH1Coefval ;       // H1 coefficient value
static uint8_t     UcH1LvlMod ;        // H1 level coef mode
 
//static uint16_t    UsStpSiz ;                          // Bias Step Size
//static uint16_t    UsErrBia ;
//static uint16_t    UsErrOfs ;
//static uint16_t    UsValBef ;
//static uint16_t    UsValNow ;
static uint16_t    UsvalFw  ;

static stAdjPar    StAdjPar ;              // Execute Command Parameter
static uint8_t     UcCvrCod ;              /* CverCode */
static uint8_t     UcPwmMod ;              /* PWM MODE */


/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 *
 *
 *      Source from OisCmd.h
 *
 *
 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
//**************************
//	define
//**************************
#define		MES_XG1			0								// LXG1 Measure Mode
#define		MES_XG2			1								// LXG2 Measure Mode

#define		HALL_ADJ		0
#define		LOOPGAIN		1
#define		THROUGH			2
#define		NOISE			3

// Measure Mode

 #define		TNE 			80								// Waiting Time For Movement

 /******* Hall calibration Type 1 *******/
 #define		MARJIN			0x0300							// Marjin
 #define		BIAS_ADJ_BORDER	0x1998							// HALL_MAX_GAP < BIAS_ADJ_BORDER < HALL_MIN_GAP(80%)

 #define		HALL_MAX_GAP	BIAS_ADJ_BORDER - MARJIN
 #define		HALL_MIN_GAP	BIAS_ADJ_BORDER + MARJIN
 /***************************************/

 #define		BIAS_LIMIT		0xFFFF							// HALL BIAS LIMIT
 #define		OFFSET_DIV		2								// Divide Difference For Offset Step
 #define		TIME_OUT		40								// Time Out Count

 /******* Hall calibration Type 2 *******/
 #define		MARGIN			0x0300							// Margin

 #define		BIAS_ADJ_OVER	0xD998							// 85%
 #define		BIAS_ADJ_RANGE	0xCCCC							// 80%
 #define		BIAS_ADJ_SKIP	0xBFFF							// 75%
 #define		HALL_MAX_RANGE	BIAS_ADJ_RANGE + MARGIN
 #define		HALL_MIN_RANGE	BIAS_ADJ_RANGE - MARGIN

 #define		DECRE_CAL		0x0100							// decrease value
 /***************************************/
#ifdef	CATCHMODE
// #ifdef	CORRECT_1DEG
  #define		MAXLMT_W		0x40400000				// 3.0
  #define		MINLMT_W		0x4019999A				// 2.4
  #define		CHGCOEF_W		0xB9855555				// 
  #define		MINLMT_MOV_W	0x4019999A				// 2.4
  #define		CHGCOEF_MOV_W	0xB9200000
//  #define		CHGCOEF_MOV_W	0xB9BA9D55
// #else
  #define		MAXLMT			0x40000000				// 2.0
  #define		MINLMT			0x3F99999A				// 1.2
  #define		CHGCOEF			0xB9480000				// 
  #define		MINLMT_MOV		0x3F99999A				// 1.2
  #define		CHGCOEF_MOV		0xB8F00000
#else
  #define		MAXLMT		0x4091B359				// 3.5  // modified by abe 140814
  #define		MINLMT		0x00000000				// 2.2	// modified by abe 140814
  #define		CHGCOEF		0xB6A8ACB9				//
  #define		MAXLMT_MOV	0x40A9FBE7			//  5.312  pm 1.400
  #define		MINLMT_MOV	0x00000000				// 0.0
  #define		CHGCOEF_MOV	0xB7B4B90F
#endif /* CATCHMODE */


/* --------------------------------------------------------
  *
  *  common data
  *
  * ------------------------------------------------------*/
 // gxzoom Setting Value
 #define		ZOOMTBL	16
static const uint32_t	ClGyxZom[ ZOOMTBL ]	= {
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000
     } ;

 // gyzoom Setting Value
static const uint32_t	ClGyyZom[ ZOOMTBL ]	= {
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000,
         0x3F800000
     } ;

 // DI Coefficient Setting Value
 #define		COEFTBL	7
 #define		DIFIL_S2		0x3F7FFD80
static const uint32_t	ClDiCof[ COEFTBL ]	= {
         DIFIL_S2,		/* 0 */
         DIFIL_S2,		/* 1 */
         DIFIL_S2,		/* 2 */
         DIFIL_S2,		/* 3 */
         DIFIL_S2,		/* 4 */
         DIFIL_S2,		/* 5 */
         DIFIL_S2		/* 6 */
     } ;

 /********* Parameter Setting *********/
 /* Servo Sampling Clock		=	23.4375kHz						*/
 /* Freq						=	CmSinFreq*Fs/65536/16			*/
 /* 05 00 XX MM 				XX:Freq MM:Sin or Circle */
static const uint16_t	CucFreqVal[ 17 ]	= {
     0xFFFF,				//  0:  Stop
     0x002C,				//  1: 0.983477Hz
     0x0059,				//  2: 1.989305Hz
     0x0086,				//  3: 2.995133Hz
     0x00B2,				//  4: 3.97861Hz
     0x00DF,				//  5: 4.984438Hz
     0x010C,				//  6: 5.990267Hz
     0x0139,				//  7: 6.996095Hz
     0x0165,				//  8: 7.979572Hz
     0x0192,				//  9: 8.9854Hz
     0x01BF,				//  A: 9.991229Hz
     0x01EC,				//  B: 10.99706Hz
     0x0218,				//  C: 11.98053Hz
     0x0245,				//  D: 12.98636Hz
     0x0272,				//  E: 13.99219Hz
     0x029F,				//  F: 14.99802Hz
     0x02CB				// 10: 15.9815Hz
 } ;

 #define	RRATETABLE	8
 #define	CRATETABLE	16
static const int8_t	ScRselRate[ RRATETABLE ]	= {
     -12,			/* -12% */
      -9,			/*  -9% */
      -6,			/*  -6% */
      -3,			/*  -3% */
       0,			/*   0% */
       3,			/*   3% */
       7,			/*   7% */
      11				/*  11% */
 } ;
static const int8_t	ScCselRate[ CRATETABLE ]	= {
     -14,			/* -14% */
     -12,			/* -12% */
     -10,			/* -10% */
      -8,			/*  -8% */
      -6,			/*  -6% */
      -4,			/*  -4% */
      -2,			/*  -2% */
       0,			/*   0% */
       0,			/*   0% */
       2,			/*   2% */
       4,			/*   4% */
       6,			/*   6% */
       8,			/*   8% */
      10,			/*  10% */
      12,			/*  12% */
      14				/*  14% */
 } ;

 //********************************************************************************
 // Function Name    : ChkCvr
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Check Cver function
 // History          : First edition                         2013.10.03 Y.Shigeoka
 //********************************************************************************
 static int ChkCvr( void )
 {
     RegReadA( CVER ,    &UcCvrCod );        // 0x027E
     //RegWriteA( MDLREG ,   MDL_VER );          // 0x00FF   Model
     RegWriteA( VRREG ,  UsvalFw );          // 0x02D0   Version
     return 0;
 }


 //********************************************************************************
 // Function Name    : IniClk
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Clock Setting
 // History          : First edition                         2013.01.08 Y.Shigeoka
 //********************************************************************************
 static int IniClk( void )
 {
     CHECK_RETURN(ChkCvr() );                              /* Read Cver */
     
     /*OSC Enables*/
     UcOscAdjFlg = 0 ;                   // Osc adj flag 
 
     /*Clock Enables*/
     RegWriteA( CLKON,       0x1F ) ;            // 0x020B 
     return 0;
 }

 //********************************************************************************
// Function Name 	: AfDrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: AF Driver Mode setting function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
static int AfDrvSw( uint8_t UcDrvSw )
{
	if( UcDrvSw == ON )
	{
		RegWriteA( DRVFCAF	, 0x20 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2 
		RegWriteA( CCAAF,   0x80 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
	else
	{
		RegWriteA( CCAAF,   0x00 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
    return 0;
}

//********************************************************************************
// Function Name 	: AfVcmCod
// Return Value		: NON
// Argument Value	: AF Code
// Explanation		: VCM AF Code setting function
// History			: First edition 						2013.12.23 YS.Kim
//********************************************************************************
static int AfVcmCod(short UsCodVal)
{
	uint8_t	ucFlg = 0x00;
	uint8_t	UcCnt = 0x00;
	
	if(UsCodVal<0) UsCodVal=0;
	else if(UsCodVal>1023) UsCodVal=1023;
	
	RegReadA( OPAFST, &ucFlg ) ;		// 0x0335
	
	while( (ucFlg & 0x80) != 0x80 )
	{
		RegReadA( OPAFST, &ucFlg ) ;	// 0x0335
		WitTim( 1 );
		UcCnt = UcCnt + 1;
		if(UcCnt > 60){
			break;
		}
	}
	
	RamWriteA( TREG_H, UsCodVal << 6 );	// Code Setting
	return 0;
}

 //********************************************************************************
 // Function Name    : IniAf
 // Function Name    : IniAf
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Open AF Initial Setting
 // History          : First edition                         2013.09.12 Y.Shigeoka
 //********************************************************************************
 static int IniAf( void )
 {
     uint8_t UcStbb0 ;
     
     CHECK_RETURN(AfDrvSw( OFF ) );                                /* AF Drvier Block Ena=0 */
     RegWriteA( DRVFCAF  , 0x20 );                   // 0x0081   Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
     RegWriteA( DRVFC3AF , 0x00 );                   // 0x0083   DGAINDAF    Gain 0
     RegWriteA( DRVFC4AF , 0x80 );                   // 0x0084   DOFSTDAF
     RegWriteA( PWMAAF,    0x00 ) ;                  // 0x0090   AF PWM standby
     RegWriteA( AFFC,   0x80 ) ;                     // 0x0088   OpenAF/-/-
     
     RegWriteA( PWMFCAF,     0x01 ) ;                // 0x0091   AF VREF , Carrier , MODE1
     RegWriteA( PWMPERIODAF, 0x20 ) ;                // 0x0099   AF none-synchronism
     RegWriteA( CCFCAF,   0x40 ) ;                   // 0x00A1   GND/-
     
     RegReadA( STBB0     , &UcStbb0 );       // 0x0250   [ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
     UcStbb0 &= 0x7F ;
     RegWriteA( STBB0, UcStbb0 ) ;           // 0x0250   OIS standby
     RegWriteA( STBB1, 0x00 ) ;              // 0x0264   All standby
     
     /* AF Initial setting */
     RegWriteA( FSTMODE,     FSTMODE_AF ) ;      // 0x0302
     RamWriteA( RWEXD1_L,    RWEXD1_L_AF ) ;     // 0x0396 - 0x0397 (Register continuos write)
     RamWriteA( RWEXD2_L,    RWEXD2_L_AF ) ;     // 0x0398 - 0x0399 (Register continuos write)
     RamWriteA( RWEXD3_L,    RWEXD3_L_AF ) ;     // 0x039A - 0x039B (Register continuos write)
     RegWriteA( FSTCTIME,    FSTCTIME_AF ) ;     // 0x0303   
     RegWriteA( TCODEH,      0x04 );                 //
     RamWriteA( LTHDH,       0x00A0 ) ;      // 0x0306 - 0x0307 (Register continuos write) 2014.05.19
     CHECK_RETURN(AfVcmCod( 0x0000 ));                         //
     //AfVcmCod( 250 );
 
     UcStbb0 |= 0x80 ;
     RegWriteA( STBB0, UcStbb0 ) ;           // 0x0250   
     RegWriteA( STBB1    , 0x05 ) ;          // 0x0264   [ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
 
     CHECK_RETURN(AfDrvSw( ON ) );                             /* AF Driver Block Ena=1 */
     return 0;
 }

 //********************************************************************************
 // Function Name    : IniSetAf
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Initial AF Setting Function
 // History          : First edition                         2013.09.12 Y.Shigeoka
 //********************************************************************************
 static int IniSetAf( void )
 {
     // Command Execute Process Initial
 //TRACE("Initial CMD \n") ;
     //IniCmd() ;
     // Clock Setting
 //TRACE("Initial CLK \n") ;
     CHECK_RETURN(IniClk() );
     // AF Initial Setting
 //TRACE("Initial AF \n") ;
     CHECK_RETURN(IniAf() );
     return 0;
 }

 //********************************************************************************
 // Function Name    : IniIop
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : I/O Port Initial Setting
 // History          : First edition                         2013.01.08 Y.Shigeoka
 //********************************************************************************
 static int IniIop( void )
 {
     RegWriteA( IOP1SEL,     0x00 );     // 0x0231   IOP1 : DGDATAIN (ATT:0236h[0]=1) 
     return 0;
 }

 //********************************************************************************
 // Function Name    : ClrGyr
 // Retun Value      : NON
 // Argment Value    : UsClrFil - Select filter to clear.  If 0x0000, clears entire filter
 //                    UcClrMod - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
 // Explanation      : Gyro RAM clear function
 // History          : First edition                         2013.01.09 Y.Shigeoka
 //********************************************************************************
 static int ClrGyr( uint16_t UsClrFil , uint8_t UcClrMod )
 {
     uint8_t UcRamClr;
     uint8_t count = 0; 
 
     /*Select Filter to clear*/
     RegWriteA( WC_RAMDLYMOD1,   (uint8_t)(UsClrFil >> 8) ) ;        // 0x018F       FRAM Initialize Hbyte
     RegWriteA( WC_RAMDLYMOD0,   (uint8_t)UsClrFil ) ;               // 0x018E       FRAM Initialize Lbyte
 
     /*Enable Clear*/
     RegWriteA( WC_RAMINITON , UcClrMod ) ;  // 0x0102   [ - | - | - | - ][ - | - | ??Clr | 學?Clr ]
     
     /*Check RAM Clear complete*/
 //TRACE("  RAM Clear Start.....Fil = %04xh\n", UsClrFil );
     do{
         RegReadA( WC_RAMINITON, &UcRamClr );
         UcRamClr &= UcClrMod;
 
         if( count++ >= 100 ){
             break;
         }
     }while( UcRamClr != 0x00 );
     return 0;
 }

 //********************************************************************************
 // Function Name    : RamAccFixMod
 // Retun Value      : NON
 // Argment Value    : 0:OFF  1:ON
 // Explanation      : Ram Access Fix Mode setting function
 // History          : First edition                         2013.05.21 Y.Shigeoka
 //********************************************************************************
 static int RamAccFixMod( uint8_t UcAccMod )
 {
     switch ( UcAccMod ) {
         case OFF :
             RegWriteA( WC_RAMACCMOD,    0x00 ) ;        // 0x018C       GRAM Access Float32bit
             break ;
         case ON :
             RegWriteA( WC_RAMACCMOD,    0x31 ) ;        // 0x018C       GRAM Access Fix32bit
             break ;
     }

     return 0;
 }

 //********************************************************************************
 // Function Name    : DrvSw
 // Retun Value      : NON
 // Argment Value    : 0:OFF  1:ON
 // Explanation      : Driver Mode setting function
 // History          : First edition                         2012.04.25 Y.Shigeoka
 //********************************************************************************
 static int DrvSw( uint8_t UcDrvSw )
 {
     if( UcDrvSw == ON )
     {
         if( UcPwmMod == PWMMOD_CVL ) {
             RegWriteA( DRVFC    , 0xF0 );           // 0x0001   Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
         } else {
#ifdef	PWM_BREAK
             RegWriteA( DRVFC    , 0x00 );           // 0x0001   Drv.MODE=0,Drv.BLK=0,MODE0B
#else
             RegWriteA( DRVFC    , 0xC0 );           // 0x0001   Drv.MODE=1,Drv.BLK=1,MODE1
#endif
         }
     }
     else
     {
         if( UcPwmMod == PWMMOD_CVL ) {
             RegWriteA( DRVFC    , 0x30 );               // 0x0001   Drvier Block Ena=0
         } else {
             RegWriteA( DRVFC    , 0x00 );               // 0x0001   Drv.MODE=0,Drv.BLK=0,MODE0B
         }
     }

     return 0;
 }

 //********************************************************************************
 // Function Name    : IniSrv
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Servo Initial Setting
 // History          : First edition                         2013.01.08 Y.Shigeoka
 //********************************************************************************
 static int IniSrv( void )
 {
     uint8_t UcStbb0 ;
 
     UcPwmMod = INIT_PWMMODE ;                   // Driver output mode
 
     RegWriteA( WC_EQON,     0x00 ) ;                // 0x0101       Filter Calcu
     RegWriteA( WC_RAMINITON,0x00 ) ;                // 0x0102       
     CHECK_RETURN(ClrGyr( 0x0000 , CLR_ALL_RAM ));                 // All Clear
     
     RegWriteA( WH_EQSWX,    0x02 ) ;                // 0x0170       [ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
     RegWriteA( WH_EQSWY,    0x02 ) ;                // 0x0171       [ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
     
     CHECK_RETURN(RamAccFixMod( OFF )) ;                           // 32bit Float mode
     
     /* Hall output limitter */
     RamWrite32A( sxlmta1,   0x3F800000 ) ;          // 0x10E6       Hall X output Limit
     RamWrite32A( sylmta1,   0x3F800000 ) ;          // 0x11E6       Hall Y output Limit
     
     /* Emargency Stop */
     RegWriteA( WH_EMGSTPON, 0x00 ) ;                // 0x0178       Emargency Stop OFF
     RegWriteA( WH_EMGSTPTMR,0xFF ) ;                // 0x017A       255*(16/23.4375kHz)=174ms
     
     RamWrite32A( sxemglev,   0x3F800000 ) ;         // 0x10EC       Hall X Emargency threshold
     RamWrite32A( syemglev,   0x3F800000 ) ;         // 0x11EC       Hall Y Emargency threshold
     
     /* Hall Servo smoothing */
     RegWriteA( WH_SMTSRVON, 0x00 ) ;                // 0x017C       Smooth Servo OFF
     RegWriteA( WH_SMTSRVSMP,0x06 ) ;                // 0x017D       2.7ms=2^06/23.4375kHz
     RegWriteA( WH_SMTTMR,   0x01 ) ;                // 0x017E       1.3ms=(1+1)*16/23.4375kHz
     
     RamWrite32A( sxsmtav,   0xBC800000 ) ;          // 0x10ED       1/64 X smoothing ave coefficient
     RamWrite32A( sysmtav,   0xBC800000 ) ;          // 0x11ED       1/64 Y smoothing ave coefficient
     RamWrite32A( sxsmtstp,  0x3AE90466 ) ;          // 0x10EE       0.001778 X smoothing offset
     RamWrite32A( sysmtstp,  0x3AE90466 ) ;          // 0x11EE       0.001778 Y smoothing offset
     
     /* High-dimensional correction  */
     RegWriteA( WH_HOFCON,   0x11 ) ;                // 0x0174       OUT 3x3
     
     /* Front */
     RamWrite32A( sxiexp3,   A3_IEXP3 ) ;            // 0x10BA       
     RamWrite32A( sxiexp2,   0x00000000 ) ;          // 0x10BB       
     RamWrite32A( sxiexp1,   A1_IEXP1 ) ;            // 0x10BC       
     RamWrite32A( sxiexp0,   0x00000000 ) ;          // 0x10BD       
     RamWrite32A( sxiexp,    0x3F800000 ) ;          // 0x10BE       
 
     RamWrite32A( syiexp3,   A3_IEXP3 ) ;            // 0x11BA       
     RamWrite32A( syiexp2,   0x00000000 ) ;          // 0x11BB       
     RamWrite32A( syiexp1,   A1_IEXP1 ) ;            // 0x11BC       
     RamWrite32A( syiexp0,   0x00000000 ) ;          // 0x11BD       
     RamWrite32A( syiexp,    0x3F800000 ) ;          // 0x11BE       
 
     /* Back */
     RamWrite32A( sxoexp3,   A3_IEXP3 ) ;            // 0x10FA       
     RamWrite32A( sxoexp2,   0x00000000 ) ;          // 0x10FB       
     RamWrite32A( sxoexp1,   A1_IEXP1 ) ;            // 0x10FC       
     RamWrite32A( sxoexp0,   0x00000000 ) ;          // 0x10FD       
     RamWrite32A( sxoexp,    0x3F800000 ) ;          // 0x10FE       
 
     RamWrite32A( syoexp3,   A3_IEXP3 ) ;            // 0x11FA       
     RamWrite32A( syoexp2,   0x00000000 ) ;          // 0x11FB       
     RamWrite32A( syoexp1,   A1_IEXP1 ) ;            // 0x11FC       
     RamWrite32A( syoexp0,   0x00000000 ) ;          // 0x11FD       
     RamWrite32A( syoexp,    0x3F800000 ) ;          // 0x11FE       
 
#ifdef	CATCHMODE
     RegWriteA( WC_DPI1ADD0,     0x2F ) ;                // 0x01B0       Data Pass
     RegWriteA( WC_DPI1ADD1,     0x00 ) ;                // 0x01B1       0x142F(PXMBZ2) --> 0x1406(GXI2Z2)
     RegWriteA( WC_DPI2ADD0,     0xAF ) ;                // 0x01B2       
     RegWriteA( WC_DPI2ADD1,     0x00 ) ;                // 0x01B3       0x14AF(PYMBZ2) --> 0x1486(GYI2Z2)
     RegWriteA( WC_DPI3ADD0,     0x38 ) ;                // 0x01B4       
     RegWriteA( WC_DPI3ADD1,     0x00 ) ;                // 0x01B5       0x1438(GXK1Z2) --> 0x143A(GXK2Z1)
     RegWriteA( WC_DPI4ADD0,     0xB8 ) ;                // 0x01B6       
     RegWriteA( WC_DPI4ADD1,     0x00 ) ;                // 0x01B7       0x14B8(GYK1Z2) --> 0x14BA(GYK2Z1)
     
     RegWriteA( WC_DPO1ADD0,     0x06 ) ;                // 0x01B8       Data Pass
     RegWriteA( WC_DPO1ADD1,     0x00 ) ;                // 0x01B9       
     RegWriteA( WC_DPO2ADD0,     0x86 ) ;                // 0x01BA       
     RegWriteA( WC_DPO2ADD1,     0x00 ) ;                // 0x01BB       
     RegWriteA( WC_DPO3ADD0,     0x3A ) ;                // 0x01BC       
     RegWriteA( WC_DPO3ADD1,     0x00 ) ;                // 0x01BD       
     RegWriteA( WC_DPO4ADD0,     0xBA ) ;                // 0x01BE       
     RegWriteA( WC_DPO4ADD1,     0x00 ) ;                // 0x01BF       
     
     RegWriteA( WC_DPON,         0x0F ) ;                // 0x0105       Data pass ON
#endif	//CATCHMODE
 
     /* Ram Access */
     CHECK_RETURN(RamAccFixMod( OFF ) );                           // 32bit float mode
 
     // PWM Signal Generate
     CHECK_RETURN(DrvSw( OFF ) );                                  /* 0x0070   Drvier Block Ena=0 */
     RegWriteA( DRVFC2   , 0x90 );                   // 0x0002   Slope 3, Dead Time = 30 ns
     RegWriteA( DRVSELX  , 0xFF );                   // 0x0003   PWM X drv max current  DRVSELX[7:0]
     RegWriteA( DRVSELY  , 0xFF );                   // 0x0004   PWM Y drv max current  DRVSELY[7:0]
 
#ifdef	PWM_BREAK
         RegWriteA( PWMFC,   0x3D ) ;                    // 0x0011   VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
#else
         RegWriteA( PWMFC,   0x21 ) ;                    // 0x0011   VREF, PWMCLK/128, MODE0S, 12Bit Accuracy  
#endif
 
     RegWriteA( PWMA,    0x00 ) ;                    // 0x0010   PWM X/Y standby
     RegWriteA( PWMDLYX,  0x04 ) ;                   // 0x0012   X Phase Delay Setting
     RegWriteA( PWMDLYY,  0x04 ) ;                   // 0x0013   Y Phase Delay Setting
     
     RegWriteA( PWMPERIODX,  0x00 ) ;                // 0x0018       PWM Carrier Freq
     RegWriteA( PWMPERIODX2, 0x00 ) ;                // 0x0019       PWM Carrier Freq
     RegWriteA( PWMPERIODY,  0x00 ) ;                // 0x001A       PWM Carrier Freq
     RegWriteA( PWMPERIODY2, 0x00 ) ;                // 0x001B       PWM Carrier Freq
     
     /* Linear PWM circuit setting */
     RegWriteA( CVA      , 0xC0 );           // 0x0020   Linear PWM mode enable
     RegWriteA( CVFC2    , 0x80 );           // 0x0022
 
     RegReadA( STBB0     , &UcStbb0 );       // 0x0250   [ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
     UcStbb0 &= 0x80 ;
     RegWriteA( STBB0, UcStbb0 ) ;           // 0x0250   OIS standby
     return 0;
 }

//********************************************************************************
// Function Name 	: SelectPtRange
// Retun Value		: NON
// Argment Value	: OFF:Narrow  ON:Wide
// Explanation		: Pan/Tilt parameter Range function
// History			: First edition 						2014.04.08 Y.Shigeoka
//********************************************************************************
static int SelectPtRange( uint8_t UcSelRange )
{
	switch ( UcSelRange ) {
		case OFF :
			RamWrite32A( gxlmt3HS0, GYRLMT3_S1 ) ;		// 0x1029
			RamWrite32A( gylmt3HS0, GYRLMT3_S1 ) ;		// 0x1129
			
			RamWrite32A( gxlmt3HS1, GYRLMT3_S2 ) ;		// 0x102A
			RamWrite32A( gylmt3HS1, GYRLMT3_S2 ) ;		// 0x112A

			RamWrite32A( gylmt4HS0, GYRLMT4_S1 ) ;		//0x112B	Y軸Limiter4 High閾値0
			RamWrite32A( gxlmt4HS0, GYRLMT4_S1 ) ;		//0x102B	X軸Limiter4 High閾値0
			
			RamWrite32A( gxlmt4HS1, GYRLMT4_S2 ) ;		//0x102C	X軸Limiter4 High閾値1
			RamWrite32A( gylmt4HS1, GYRLMT4_S2 ) ;		//0x112C	Y軸Limiter4 High閾値1
		
			RamWrite32A( Sttx12aH, 	GYRA12_HGH );		// 0x105F
			RamWrite32A( Stty12aH, 	GYRA12_HGH );		// 0x115F

			break ;
#ifdef	CATCHMODE	
		case ON :
			RamWrite32A( gxlmt3HS0, GYRLMT3_S1_W ) ;		// 0x1029
			RamWrite32A( gylmt3HS0, GYRLMT3_S1_W ) ;		// 0x1129
			
			RamWrite32A( gxlmt3HS1, GYRLMT3_S2_W ) ;		// 0x102A
			RamWrite32A( gylmt3HS1, GYRLMT3_S2_W ) ;		// 0x112A

			RamWrite32A( gylmt4HS0, GYRLMT4_S1_W ) ;		//0x112B	Y軸Limiter4 High閾値0
			RamWrite32A( gxlmt4HS0, GYRLMT4_S1_W ) ;		//0x102B	X軸Limiter4 High閾値0
			
			RamWrite32A( gxlmt4HS1, GYRLMT4_S2_W ) ;		//0x102C	X軸Limiter4 High閾値1
			RamWrite32A( gylmt4HS1, GYRLMT4_S2_W ) ;		//0x112C	Y軸Limiter4 High閾値1
		
			RamWrite32A( Sttx12aH, 	GYRA12_HGH_W );			// 0x105F
			RamWrite32A( Stty12aH, 	GYRA12_HGH_W );			// 0x115F

			break ;
#endif // CATCHMODE
	}

    return 0;
}

//********************************************************************************
// Function Name 	: IniPtMovMod
// Retun Value		: NON
// Argment Value	: OFF:Still  ON:Movie
// Explanation		: Pan/Tilt parameter setting by mode function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
static int IniPtMovMod( uint8_t UcPtMod )
{
#ifdef	CATCHMODE
	switch ( UcPtMod ) {
		case OFF :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x04 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x00 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158

			break ;
		case ON :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x00 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x00 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158
			break ;
	}
#else
	switch ( UcPtMod ) {
		case OFF :
			// State 3 -> 1
			RegWriteA( WG_PANSTT31JUG0, 	0x01 );			// 0x0142
			RegWriteA( WG_PANSTT31JUG1, 	0x00 );			// 0x0143
			// State 4 -> 1
			RegWriteA( WG_PANSTT41JUG0, 	0x00 );			// 0x0144
			RegWriteA( WG_PANSTT41JUG1, 	0x00 );			// 0x0145
			// State 1 -> 3
			RegWriteA( WG_PANSTT13JUG0, 	0x00 );			// 0x0148
			RegWriteA( WG_PANSTT13JUG1, 	0x07 );			// 0x0149
			// State 4 -> 3
			RegWriteA( WG_PANSTT43JUG0, 	0x00 );			// 0x014C
			RegWriteA( WG_PANSTT43JUG1, 	0x00 );			// 0x014D
			// State 3 -> 4
			RegWriteA( WG_PANSTT34JUG0, 	0x00 );			// 0x014E
			RegWriteA( WG_PANSTT34JUG1, 	0x00 );			// 0x014F
			
			RegWriteA( WG_PANSTTXXXTH,	0x0F );				// 0x015A
			RamWrite32A( Sttx34aM, GYRA34_MID ) ;		// 0x108F
			RamWrite32A( Stty34aM, GYRA34_MID ) ;		// 0x118F

			// I Filter X							// 2s
			RamWrite32A( gxia_1, 0x3860DE00 ) ;		// 0x1043	0.4Hz
			RamWrite32A( gxib_1, 0xB261CF49 ) ;		// 0x1044	Down
			RamWrite32A( gxic_1, 0x3261CF49 ) ;		// 0x1045	Up

			RamWrite32A( gxia_a, 0x3860DE00 ) ;		// 0x1046	0.4Hz
			RamWrite32A( gxib_a, 0xB261CF49 ) ;		// 0x1047	Down
			RamWrite32A( gxic_a, 0x3261CF49 ) ;		// 0x1048	Up

			RamWrite32A( gxia_b, 0x3A2F91C0 ) ;		// 0x1049	5Hz
			RamWrite32A( gxib_b, 0xB261CF49 ) ;		// 0x104A	Down
			RamWrite32A( gxic_b, 0x3F800000 ) ;		// 0x104B	Up

			RamWrite32A( gxia_c, 0x3860DE00 ) ;		// 0x104C	0.4Hz
			RamWrite32A( gxib_c, 0xB261CF49 ) ;		// 0x104D	Down
			RamWrite32A( gxic_c, 0x3261CF49 ) ;		// 0x104E	Up

			// I Filter Y
			RamWrite32A( gyia_1, 0x3860DE00 ) ;		// 0.4Hz
			RamWrite32A( gyib_1, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_1, 0x3261CF49 ) ;		// Up

			RamWrite32A( gyia_a, 0x3860DE00 ) ;		// 0.4Hz
			RamWrite32A( gyib_a, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_a, 0x3261CF49 ) ;		// Up

			RamWrite32A( gyia_b, 0x3A2F91C0 ) ;		// 5Hz
			RamWrite32A( gyib_b, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_b, 0x3F800000 ) ;		// Up

			RamWrite32A( gyia_c, 0x3860DE00 ) ;		// 0.4Hz
			RamWrite32A( gyib_c, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_c, 0x3261CF49 ) ;		// Up
			break ;
		case ON :
			// State 3 -> 1
			RegWriteA( WG_PANSTT31JUG0, 	0x00 );			// 0x0142
			RegWriteA( WG_PANSTT31JUG1, 	0x00 );			// 0x0143
			// State 4 -> 1
			RegWriteA( WG_PANSTT41JUG0, 	0x07 );			// 0x0144
			RegWriteA( WG_PANSTT41JUG1, 	0x00 );			// 0x0145
			// State 1 -> 3
			RegWriteA( WG_PANSTT13JUG0, 	0x00 );			// 0x0148
			RegWriteA( WG_PANSTT13JUG1, 	0x07 );			// 0x0149
			// State 4 -> 3
			RegWriteA( WG_PANSTT43JUG0, 	0x00 );			// 0x014C
			RegWriteA( WG_PANSTT43JUG1, 	0x07 );			// 0x014D
			// State 3 -> 4
			RegWriteA( WG_PANSTT34JUG0, 	0x01 );			// 0x014E
			RegWriteA( WG_PANSTT34JUG1, 	0x00 );			// 0x014F
			
			RegWriteA( WG_PANSTTXXXTH,	0xF0 );				// 0x015A
			RamWrite32A( Sttx34aM, GYRA34_MID_M ) ;		// 0x108F
			RamWrite32A( Stty34aM, GYRA34_MID_M ) ;		// 0x118F
				
			// I Filter X							// 2s
			RamWrite32A( gxia_1, 0x3860DE00 ) ;		// 0x1043	0.4Hz
			RamWrite32A( gxib_1, 0xB261CF49 ) ;		// 0x1044	Down
			RamWrite32A( gxic_1, 0x3261CF49 ) ;		// 0x1045	Up

			RamWrite32A( gxia_a, 0x3860DE00 ) ;		// 0x1046	0.4Hz
			RamWrite32A( gxib_a, 0xB261CF49 ) ;		// 0x1047	Down
			RamWrite32A( gxic_a, 0x3261CF49 ) ;		// 0x1048	Up

			RamWrite32A( gxia_b, 0x3860DE00 ) ;		// 0x1049	0.4Hz
			RamWrite32A( gxib_b, 0xB261CF49 ) ;		// 0x104A	Down
			RamWrite32A( gxic_b, 0x3261CF49 ) ;		// 0x104B	Up

			RamWrite32A( gxia_c, 0x3A2F91C0 ) ;		// 0x104C	5Hz
			RamWrite32A( gxib_c, 0xB261CF49 ) ;		// 0x104D	Down
			RamWrite32A( gxic_c, 0x3F800000 ) ;		// 0x104E	Up
			
			// I Filter Y
			RamWrite32A( gyia_1, 0x3860DE00 ) ;		// 0.4Hz
			RamWrite32A( gyib_1, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_1, 0x3261CF49 ) ;		// Up

			RamWrite32A( gyia_a, 0x3860DE00 ) ;		// 0.4Hz
			RamWrite32A( gyib_a, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_a, 0x3261CF49 ) ;		// Up

			RamWrite32A( gyia_b, 0x3860DE00 ) ;		// 0.4Hz
			RamWrite32A( gyib_b, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_b, 0x3261CF49 ) ;		// Up

			RamWrite32A( gyia_c, 0x3A2F91C0 ) ;		// 5Hz
			RamWrite32A( gyib_c, 0xB261CF49 ) ;		// Down
			RamWrite32A( gyic_c, 0x3F800000 ) ;		// Up
			break ;
	}
#endif	
    return 0;
}

//********************************************************************************
// Function Name 	: AutoGainContIni
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gain Control initial function
// History			: First edition 						2014.09.16 Y.Shigeoka
//********************************************************************************
#ifdef	GAIN_CONT
//********************************************************************************
// Function Name 	: AutoGainContIni
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gain Control initial function
// History			: First edition 						2014.09.16 Y.Shigeoka
//********************************************************************************
#ifdef	CATCHMODE
  #define	TRI_LEVEL		0x3A031280		/* 0.0005 */
  #define	TIMELOW			0x50			/* */
  #define	TIMEHGH			0x05			/* */
  #define	TIMEBSE			0x5D			/* 3.96ms */
  #define	MONADR			GXXFZ
  #define	GANADR			gxadj
  #define	XMINGAIN		0x00000000
  #define	XMAXGAIN		0x3F800000
  #define	YMINGAIN		0x00000000
  #define	YMAXGAIN		0x3F800000
  #define	XSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	XSTEPDN			0xBD4CCCCD		/* -0.05 	 */
  #define	YSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	YSTEPDN			0xBD4CCCCD		/* -0.05 	 */
#else	//CATCHMODE
  #define	TRI_LEVEL		0x3B23D70A		/* 0.0025 */		//140314
  #define	TIMELOW			0x40			/* */				//140314
  #define	TIMEHGH			0x01			/* */
  #define	TIMEBSE			0x5D			/* 3.96ms */
  #define	MONADR			GXXFZ
  #define	GANADR			gxadj
  #define	XMINGAIN		0x00000000
  #define	XMAXGAIN		0x3F800000
  #define	YMINGAIN		0x00000000
  #define	YMAXGAIN		0x3F800000
  #define	XSTEPUP			0x3F800000		/* 1.0	 */
  #define	XSTEPDN			0xBF80000D		/* -1.0 	 */
  #define	YSTEPUP			0x3F800000		/* 1.0	 */
  #define	YSTEPDN			0xBF800000		/* -1.0 	 */
#endif	//CATCHMODE
static int AutoGainContIni( void )
{
	RamWrite32A( gxlevlow, TRI_LEVEL );					// 0x10AE	Low Th
	RamWrite32A( gylevlow, TRI_LEVEL );					// 0x11AE	Low Th
	RamWrite32A( gxadjmin, XMINGAIN );					// 0x1094	Low gain
	RamWrite32A( gxadjmax, XMAXGAIN );					// 0x1095	Hgh gain
	RamWrite32A( gxadjdn, XSTEPDN );					// 0x1096	-step
	RamWrite32A( gxadjup, XSTEPUP );					// 0x1097	+step
	RamWrite32A( gyadjmin, YMINGAIN );					// 0x1194	Low gain
	RamWrite32A( gyadjmax, YMAXGAIN );					// 0x1195	Hgh gain
	RamWrite32A( gyadjdn, YSTEPDN );					// 0x1196	-step
	RamWrite32A( gyadjup, YSTEPUP );					// 0x1197	+step
	
	RegWriteA( WG_LEVADD, (uint8_t)MONADR );		// 0x0120	Input signal
	RegWriteA( WG_LEVTMR, 		TIMEBSE );				// 0x0123	Base Time
	RegWriteA( WG_LEVTMRLOW, 	TIMELOW );				// 0x0121	X Low Time
	RegWriteA( WG_LEVTMRHGH, 	TIMEHGH );				// 0x0122	X Hgh Time
	RegWriteA( WG_ADJGANADD, (uint8_t)GANADR );		// 0x0128	control address
	return 0;
}

//********************************************************************************
// Function Name 	: AutoGainControlSw
// Retun Value		: NON
// Argment Value	: 0 :OFF  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
static int AutoGainControlSw( uint8_t UcModeSw )
{

	if( UcModeSw == OFF )
	{
		RegWriteA( WG_ADJGANGXATO, 	0xA0 );					// 0x0129	X exe off
		RegWriteA( WG_ADJGANGYATO, 	0xA0 );					// 0x012A	Y exe off
		RamWrite32A( GANADR			 , XMAXGAIN ) ;			// Gain Through
		RamWrite32A( GANADR | 0x0100 , YMAXGAIN ) ;			// Gain Through
	}
	else
	{
		RegWriteA( WG_ADJGANGXATO, 	0xA3 );					// 0x0129	X exe on
		RegWriteA( WG_ADJGANGYATO, 	0xA3 );					// 0x012A	Y exe on
	}
    return 0;
}

#endif //GAIN_CONT

static int IniGyr( void )
 {
#ifdef	CATCHMODE
         /* CPU control */
     RegWriteA( WC_CPUOPEON , 0x11 );    // 0x0103       CPU control
     RegWriteA( WC_CPUOPE1ADD , 0x06 );  // 0x018A       
     RegWriteA( WC_CPUOPE2ADD , 0x3A );  // 0x018B       
     RegWriteA( WG_EQSW  , 0x43 );       // 0x0110       [ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
#else	
     /*Gyro Filter Setting*/
     RegWriteA( WG_EQSW  , 0x03 );       // 0x0110       [ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
#endif	
     
     /*Gyro Filter Down Sampling*/
     
     RegWriteA( WG_SHTON , 0x10 );       // 0x0107       [ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
                                         //              CmShtOpe[1:0] 00: ???僞乕OFF, 01: ???僞乕ON, 1x:????
     RegWriteA( WG_SHTMOD , 0x06 );      // 0x0116       Shutter Hold mode
 
     // Limiter
     RamWrite32A( gxlmt1H, GYRLMT1H ) ;          // 0x1028
     RamWrite32A( gylmt1H, GYRLMT1H ) ;          // 0x1128
 
     RamWrite32A( Sttx12aM,  GYRA12_MID );               // 0x104F
     //RamWrite32A( Sttx12aH,    GYRA12_HGH );               // 0x105F
     RamWrite32A( Sttx12bM,  GYRB12_MID );               // 0x106F
     RamWrite32A( Sttx12bH,  GYRB12_HGH );               // 0x107F
     RamWrite32A( Sttx34aM,  GYRA34_MID );               // 0x108F
     RamWrite32A( Sttx34aH,  GYRA34_HGH );               // 0x109F
     RamWrite32A( Sttx34bM,  GYRB34_MID );               // 0x10AF
     RamWrite32A( Sttx34bH,  GYRB34_HGH );               // 0x10BF
     RamWrite32A( Stty12aM,  GYRA12_MID );               // 0x114F
     //RamWrite32A( Stty12aH,    GYRA12_HGH );               // 0x115F
     RamWrite32A( Stty12bM,  GYRB12_MID );               // 0x116F
     RamWrite32A( Stty12bH,  GYRB12_HGH );               // 0x117F
     RamWrite32A( Stty34aM,  GYRA34_MID );               // 0x118F
     RamWrite32A( Stty34aH,  GYRA34_HGH );               // 0x119F
     RamWrite32A( Stty34bM,  GYRB34_MID );               // 0x11AF
     RamWrite32A( Stty34bH,  GYRB34_HGH );               // 0x11BF
 
#ifdef	CATCHMODE
#ifdef	CORRECT_1DEG
     CHECK_RETURN(SelectPtRange( ON ) );
#else
     CHECK_RETURN(SelectPtRange( OFF ) );
#endif
 
     RegWriteA( WG_PANADDA,      0x12 );     // 0x0130   GXH2Z2/GYH2Z2 Select
     RegWriteA( WG_PANADDB,      0x3B );     // 0x0131   GXK2Z2/GYK2Z2 Select
#else	//CATCHMODE
     CHECK_RETURN(SelectPtRange( OFF ) );  
     /* Pan/Tilt parameter */
     RegWriteA( WG_PANADDA,      0x12 );                 // 0x0130   GXH1Z2/GYH1Z2 Select
     RegWriteA( WG_PANADDB,      0x09 );                 // 0x0131   GXIZ/GYIZ Select
#endif
      //Threshold
     RamWrite32A( SttxHis,   0x00000000 );           // 0x1226
     RamWrite32A( SttxaL,    0x00000000 );           // 0x109D
     RamWrite32A( SttxbL,    0x00000000 );           // 0x109E
     RamWrite32A( SttyaL,    0x00000000 );           // 0x119D
     RamWrite32A( SttybL,    0x00000000 );           // 0x119E       
     // Pan level
     RegWriteA( WG_PANLEVABS,        0x00 );     // 0x0133
     
     // Average parameter are set IniAdj
 
#ifdef	CATCHMODE
     // Phase Transition Setting
     // State 2 -> 1
     RegWriteA( WG_PANSTT21JUG0,     0x07 );     // 0x0140
     RegWriteA( WG_PANSTT21JUG1,     0x00 );     // 0x0141
     // State 3 -> 1
     RegWriteA( WG_PANSTT31JUG0,     0x00 );     // 0x0142
     RegWriteA( WG_PANSTT31JUG1,     0x00 );     // 0x0143
     // State 4 -> 1
     RegWriteA( WG_PANSTT41JUG0,     0x00 );     // 0x0144
     RegWriteA( WG_PANSTT41JUG1,     0x00 );     // 0x0145
     // State 1 -> 2
     RegWriteA( WG_PANSTT12JUG0,     0x00 );     // 0x0146
     RegWriteA( WG_PANSTT12JUG1,     0x07 );     // 0x0147
     // State 1 -> 3
     RegWriteA( WG_PANSTT13JUG0,     0x00 );     // 0x0148
     RegWriteA( WG_PANSTT13JUG1,     0x00 );     // 0x0149
     // State 2 -> 3
     RegWriteA( WG_PANSTT23JUG0,     0x00 );     // 0x014A
     RegWriteA( WG_PANSTT23JUG1,     0x00 );     // 0x014B
     // State 4 -> 3
     RegWriteA( WG_PANSTT43JUG0,     0x00 );     // 0x014C
     RegWriteA( WG_PANSTT43JUG1,     0x00 );     // 0x014D
     // State 3 -> 4
     RegWriteA( WG_PANSTT34JUG0,     0x00 );     // 0x014E
     RegWriteA( WG_PANSTT34JUG1,     0x00 );     // 0x014F
     // State 2 -> 4
     RegWriteA( WG_PANSTT24JUG0,     0x00 );     // 0x0150
     RegWriteA( WG_PANSTT24JUG1,     0x00 );     // 0x0151
     // State 4 -> 2
     RegWriteA( WG_PANSTT42JUG0,     0x00 );     // 0x0152
     RegWriteA( WG_PANSTT42JUG1,     0x00 );     // 0x0153
 
     // State Timer
     RegWriteA( WG_PANSTT1LEVTMR,    0x00 );     // 0x015B
     RegWriteA( WG_PANSTT2LEVTMR,    0x00 );     // 0x015C
     RegWriteA( WG_PANSTT3LEVTMR,    0x00 );     // 0x015D
     RegWriteA( WG_PANSTT4LEVTMR,    0x00 );     // 0x015E
     
     // Control filter
     RegWriteA( WG_PANTRSON0,        0x1B );     // 0x0132   USE iSTP
     
     // State Setting
     CHECK_RETURN(IniPtMovMod( OFF ) );                            // Pan/Tilt setting (Still)
     
     // Hold
     RegWriteA( WG_PANSTTSETILHLD,   0x00 );     // 0x015F
     
     
     // State2,4 Step Time Setting
     RegWriteA( WG_PANSTT2TMR0,  0x01 );     // 0x013C
     RegWriteA( WG_PANSTT2TMR1,  0x00 );     // 0x013D   
     RegWriteA( WG_PANSTT4TMR0,  0x01 );     // 0x013E
     RegWriteA( WG_PANSTT4TMR1,  0x00 );     // 0x013F   
     
     RegWriteA( WG_PANSTTXXXTH,  0x00 );     // 0x015A
 
     CHECK_RETURN(AutoGainContIni() );
     /* exe function */
     CHECK_RETURN(AutoGainControlSw( ON )) ;                           /* Auto Gain Control Mode ON  */
 
#else
     // Phase Transition Setting
     // State 2 -> 1
     RegWriteA( WG_PANSTT21JUG0,     0x00 );     // 0x0140
     RegWriteA( WG_PANSTT21JUG1,     0x00 );     // 0x0141
     // State 3 -> 1
     RegWriteA( WG_PANSTT31JUG0,     0x01 );     // 0x0142
     RegWriteA( WG_PANSTT31JUG1,     0x00 );     // 0x0143
     // State 4 -> 1
     RegWriteA( WG_PANSTT41JUG0,     0x00 );     // 0x0144
     RegWriteA( WG_PANSTT41JUG1,     0x00 );     // 0x0145
     // State 1 -> 2
     RegWriteA( WG_PANSTT12JUG0,     0x00 );     // 0x0146
     RegWriteA( WG_PANSTT12JUG1,     0x00 );     // 0x0147
     // State 1 -> 3
     RegWriteA( WG_PANSTT13JUG0,     0x00 );     // 0x0148
     RegWriteA( WG_PANSTT13JUG1,     0x07 );     // 0x0149
     // State 2 -> 3
     RegWriteA( WG_PANSTT23JUG0,     0x00 );     // 0x014A
     RegWriteA( WG_PANSTT23JUG1,     0x00 );     // 0x014B
     // State 4 -> 3
     RegWriteA( WG_PANSTT43JUG0,     0x00 );     // 0x014C
     RegWriteA( WG_PANSTT43JUG1,     0x00 );     // 0x014D
     // State 3 -> 4
     RegWriteA( WG_PANSTT34JUG0,     0x00 );     // 0x014E
     RegWriteA( WG_PANSTT34JUG1,     0x00 );     // 0x014F
     // State 2 -> 4
     RegWriteA( WG_PANSTT24JUG0,     0x00 );     // 0x0150
     RegWriteA( WG_PANSTT24JUG1,     0x00 );     // 0x0151
     // State 4 -> 2
     RegWriteA( WG_PANSTT42JUG0,     0x00 );     // 0x0152
     RegWriteA( WG_PANSTT42JUG1,     0x00 );     // 0x0153
     
     // Control filter
     RegWriteA( WG_PANTRSON0,        0x91 );     // 0x0132   USE I12/iSTP/Gain-Filter
 
     RegWriteA( WG_PANSTTSETGYRO,    0x00 );             // 0x0154
     RegWriteA( WG_PANSTTSETGAIN,    0x10 );             // 0x0155
     RegWriteA( WG_PANSTTSETISTP,    0x10 );             // 0x0156
     RegWriteA( WG_PANSTTSETIFTR,    0x10 );             // 0x0157
     RegWriteA( WG_PANSTTSETLFTR,    0x00 );             // 0x0158
     
     // Hold
     RegWriteA( WG_PANSTTSETILHLD,   0x00 );     // 0x015F
     
     
     // State2,4 Step Time Setting
     RegWriteA( WG_PANSTT2TMR0,  0xEA );     // 0x013C
     RegWriteA( WG_PANSTT2TMR1,  0x00 );     // 0x013D   
     RegWriteA( WG_PANSTT4TMR0,  0x92 );     // 0x013E
     RegWriteA( WG_PANSTT4TMR1,  0x04 );     // 0x013F   
     
     RegWriteA( WG_PANSTTXXXTH,  0x0F );     // 0x015A
 
  #ifdef	GAIN_CONT
     CHECK_RETURN(AutoGainContIni() );
     /* exe function */
     CHECK_RETURN(AutoGainControlSw( ON )) ;                           /* Auto Gain Control Mode OFF */
  #endif	//GAIN_CONT
#endif	/* CATCHMODE */	
    return 0;
 }

 //********************************************************************************
 // Function Name    : IniFil
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Gyro Filter Initial Parameter Setting
 // History          : First edition                         2009.07.30 Y.Tashita
 //********************************************************************************
 static int IniFil( void )
 {
#ifdef INIT_FAST
     unsigned char   UcAryId ;
     unsigned short  UsDatId, UsDatNum ;
 
     RegWriteA( WC_RAMACCXY, 0x01 ) ;            // 0x018D   Filter copy on
     // Filter Registor Parameter Setting
     UcAryId = 0 ;
     UsDatNum = 0 ;
     UsDatId = 0 ;
     while( CsFilRegBurst[ UcAryId ] != 0xFF )
     {
         UsDatNum    = CsFilRegBurst[ UcAryId ];
         CntWrtRag( ( unsigned char * )&CsFilRegDat[ UsDatId ], UsDatNum ) ;
         UcAryId++ ;
         UsDatId += UsDatNum ;
     }
     // Filter X-axis Ram Parameter Setting  
     UcAryId = 0 ;
     UsDatNum = 0 ;
     UsDatId = 0 ;
     while( CsFilRamBurst[ UcAryId ] != 0xFF )
     {
         UsDatNum    = CsFilRamBurst[ UcAryId ];
         CntWrtRam32( ( unsigned char * )&CsFilRamDat[ UsDatId ], UsDatNum ) ;
         UsDatId += UsDatNum ;
         UcAryId++ ;
     }
     // Filter Y-axis Ram Parameter Setting  
     UcAryId = 0 ;
     UsDatNum = 0 ;
     UsDatId = 0 ;
     while( CsFilRamY[ UcAryId ] != 0xFF )
     {
         UsDatNum    = CsFilRamY[ UcAryId ];
         CntWrtRam32( ( unsigned char * )&CsFilRamYDat[ UsDatId ], UsDatNum ) ;
         UsDatId += UsDatNum ;
         UcAryId++ ;
     }
     RegWriteA( WC_RAMACCXY, 0x00 ) ;
#else
     unsigned short  UsAryId ;
     // Filter Registor Parameter Setting
     RegWriteA( WC_RAMACCXY, 0x01 ) ;
     UsAryId = 0 ;
     while( CsFilReg[ UsAryId ].UsRegAdd != 0xFFFF )
     {
         RegWriteA( CsFilReg[ UsAryId ].UsRegAdd, CsFilReg[ UsAryId ].UcRegDat ) ;
         UsAryId++ ;
     }
     // Filter Ram Parameter Setting
     UsAryId = 0 ;
     while( CsFilRam[ UsAryId ].UsRamAdd != 0xFFFF )
     {
         RamWrite32A( CsFilRam[ UsAryId ].UsRamAdd, CsFilRam[ UsAryId ].UlRamDat ) ;
         UsAryId++ ;
     }
     RegWriteA( WC_RAMACCXY, 0x00 ) ;
#endif	

    return 0;
 }

 //********************************************************************************
 // Function Name    : AccWit
 // Retun Value      : NON
 // Argment Value    : Trigger Register Data
 // Explanation      : Acc Wait Function
 // History          : First edition                         2010.12.27 Y.Shigeoka
 //********************************************************************************
 static int AccWit( uint8_t UcTrgDat )
 {
     uint8_t UcFlgVal ;
     uint8_t UcCntPla ;
     UcFlgVal    = 1 ;
     UcCntPla    = 0 ;
     
     do{
         RegReadA( GRACC, &UcFlgVal ) ;
         UcFlgVal    &= UcTrgDat ;
         UcCntPla++ ;
     } while( UcFlgVal && ( UcCntPla < 6 ) ) ;

     return 0;
 }
 
 //********************************************************************************
 // Function Name    : GyOutSignal
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Select Gyro Signal Function
 // History          : First edition                         2010.12.27 Y.Shigeoka
 //********************************************************************************
 static int GyOutSignal( void )
 {
 
     RegWriteA( GRADR0,  GYROX_INI ) ;           // 0x0283   Set Gyro XOUT H~L
     RegWriteA( GRADR1,  GYROY_INI ) ;           // 0x0284   Set Gyro YOUT H~L
     
     /*Start OIS Reading*/
     RegWriteA( GRSEL    , 0x02 );           // 0x0280   [ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]
     return 0;
 }

 //********************************************************************************
 // Function Name    : IniDgy
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Digital Gyro Initial Setting
 // History          : First edition                         2013.01.08 Y.Shigeoka
 //********************************************************************************
 static int IniDgy( void )
 { 
     uint8_t UcGrini ;  
     RegWriteA( SPIM     , 0x01 );                           // 0x028F   [ - | - | - | - ][ - | - | - | DGSPI4 ] 
  
     /*Set to Command Mode*/
     RegWriteA( GRSEL    , 0x01 );                           // 0x0280   [ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]
 
     /*Digital Gyro Read settings*/
     RegWriteA( GRINI    , 0x80 );                           // 0x0281   [ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
  
     RegReadA( GRINI , &UcGrini );                   // 0x0281   [ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
     RegWriteA( GRINI, ( UcGrini | SLOWMODE) );      // 0x0281   [ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
 
     RegWriteA( GRADR0,  0x6B ) ;                    // 0x0283   Set I2C_DIS
     RegWriteA( GSETDT,  0x01 ) ;                    // 0x028A   Set Write Data
     RegWriteA( GRACC,   0x10 ) ;                    /* 0x0282   Set Trigger ON              */
     CHECK_RETURN(AccWit( 0x10 ) );                                /* Digital Gyro busy wait               */
 
     RegWriteA( GRADR0,  0x1B ) ;                    // 0x0283   Set GYRO_CONFIG
     RegWriteA( GSETDT,  ( FS_SEL << 3) ) ;          // 0x028A   Set Write Data
     RegWriteA( GRACC,   0x10 ) ;                    /* 0x0282   Set Trigger ON              */
     CHECK_RETURN(AccWit( 0x10 ) );                                /* Digital Gyro busy wait               */
 
     RegWriteA( GRADR0,  0x1A ) ;                    // 0x0283   Set DLPF_CFG
     RegWriteA( GSETDT,  0x00 ) ;                    // 0x028A   Set Write Data
     RegWriteA( GRACC,   0x10 ) ;                    /* 0x0282   Set Trigger ON              */
     CHECK_RETURN(AccWit( 0x10 ) );
     
     RegWriteA( GRADR0,  0x6A ) ;                    // 0x0283   Set I2C_IF_DIS, SIG_COND_RESET
     RegWriteA( GSETDT,  0x11 ) ;                    // 0x028A   Set Write Data
     RegWriteA( GRACC,   0x10 ) ;                    /* 0x0282   Set Trigger ON              */
     CHECK_RETURN(AccWit( 0x10 ) );
 
     RegReadA( GRINI , &UcGrini );                   // 0x0281   [ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
     RegWriteA( GRINI, (uint8_t)(UcGrini & ~SLOWMODE) );     // 0x0281   [ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ] 
     RegWriteA( RDSEL,   0x7C ) ;                // 0x028B   RDSEL(Data1 and 2 for continuos mode)
     
     CHECK_RETURN(GyOutSignal() );
     return 0;
 }

 //********************************************************************************
 // Function Name    : SelectIstpMod
 // Retun Value      : NON
 // Argment Value    : OFF:Narrow  ON:Wide
 // Explanation      : Pan/Tilt parameter Range function
 // History          : First edition                         2014.04.08 Y.Shigeoka
 //********************************************************************************
 static int  SelectIstpMod( uint8_t UcSelRange )
 {
     switch ( UcSelRange ) {
         case OFF :
             RamWrite32A( gxistp_1, GYRISTP ) ;      // 0x1083
             RamWrite32A( gyistp_1, GYRISTP ) ;      // 0x1183
             break ;
#ifdef	CATCHMODE		
         case ON :
             RamWrite32A( gxistp_1, GYRISTP_W ) ;    // 0x1083
             RamWrite32A( gyistp_1, GYRISTP_W ) ;    // 0x1183
             break ;
#endif // CATCHMODE
     }

     return 0;
 }

 //********************************************************************************
 // Function Name    : IniPtAve
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Pan/Tilt Average parameter setting function
 // History          : First edition                         2013.09.26 Y.Shigeoka
 //********************************************************************************
 static int IniPtAve( void )
 {
#ifdef	CATCHMODE
     RamWrite32A( st1mean, 0x3f800000 );     // 0x1235
     RamWrite32A( st2mean, 0x3f800000 );     // 0x1236
     RamWrite32A( st3mean, 0x3f800000 );     // 0x1237
     RamWrite32A( st4mean, 0x3f800000 );     // 0x1238
#else
     RamWrite32A( st1mean, 0x3f800000 );     // 0x1235
     RamWrite32A( st2mean, 0x3f800000 );     // 0x1236   1/400
     RamWrite32A( st3mean, 0x3f800000 );     // 0x1237   1/100
     RamWrite32A( st4mean, 0x3f800000 );     // 0x1238
#endif	

    return 0;
 }

 //********************************************************************************
 // Function Name    : SetPanTiltMode
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Pan-Tilt Enable/Disable
 // History          : First edition                         2013.01.09 Y.Shigeoka
 //********************************************************************************
 static int SetPanTiltMode( uint8_t UcPnTmod )
 {
     switch ( UcPnTmod ) {
         case OFF :
             RegWriteA( WG_PANON, 0x00 ) ;           // 0x0109   X,Y Pan/Tilt Function OFF
             break ;
         case ON :
             RegWriteA( WG_PANON, 0x01 ) ;           // 0x0109   X,Y Pan/Tilt Function ON
             break ;
     }
    return 0;
 }

#ifdef H1COEF_CHANGER
 //********************************************************************************
 // Function Name    : SetH1cMod
 // Retun Value      : NON
 // Argment Value    : Command Parameter
 // Explanation      : Set H1C coefficient Level chang Function
 // History          : First edition                         2013.04.18 Y.Shigeoka
 //********************************************************************************
 static int SetH1cMod( uint8_t UcSetNum )
 {
 
     switch( UcSetNum ){
     case ( ACTMODE ):               // initial
         // Limit value Value Setting
         RamWrite32A( gxlmt6L, MINLMT ) ;        /* 0x102D L-Limit */
         RamWrite32A( gxlmt6H, MAXLMT ) ;        /* 0x102E H-Limit */
 
         RamWrite32A( gylmt6L, MINLMT ) ;        /* 0x112D L-Limit */
         RamWrite32A( gylmt6H, MAXLMT ) ;        /* 0x112E H-Limit */
 
         RamWrite32A( gxmg, CHGCOEF ) ;          /* 0x10AA Change coefficient gain */
         RamWrite32A( gymg, CHGCOEF ) ;          /* 0x11AA Change coefficient gain */
 
         RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;     /* 0x100E Base Coef */
         RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;     /* 0x110E Base Coef */
         
         RamWrite32A( gxh1c, DIFIL_S2 ) ;        /* 0x1012 Base Coef */
         RamWrite32A( gyh1c, DIFIL_S2 ) ;        /* 0x1112 Base Coef */
 
         RegWriteA( WG_HCHR, 0x12 ) ;            // 0x011B   GmHChrOn[1]=1 Sw ON
         break ;
 
     case( S2MODE ):             // cancel lvl change mode
         RegWriteA( WG_HCHR, 0x10 ) ;            // 0x011B   GmHChrOn[1]=0 Sw OFF
         break ;
 
     case( MOVMODE ):            // Movie mode
         CHECK_RETURN(IniPtMovMod( OFF )) ;                            // Pan/Tilt setting (Still) 2014.05.19
 
         RamWrite32A( gxlmt6L, MINLMT_MOV ) ;    /* 0x102D L-Limit */
         RamWrite32A( gxlmt6H, MAXLMT_MOV ) ;    /* 0x102E H-Limit */
         RamWrite32A( gylmt6L, MINLMT_MOV ) ;    /* 0x112D L-Limit */
         RamWrite32A( gylmt6H, MAXLMT_MOV ) ;    /* 0x112E H-Limit */
 
         RamWrite32A( gxmg, CHGCOEF_MOV ) ;      /* 0x10AA Change coefficient gain */
         RamWrite32A( gymg, CHGCOEF_MOV ) ;      /* 0x11AA Change coefficient gain */
 
         RegWriteA( WG_HCHR, 0x12 ) ;            // 0x011B   GmHChrOn[1]=1 Sw ON
         break ;
 
     default :
         CHECK_RETURN(IniPtMovMod( OFF )) ;                            // Pan/Tilt setting (Still)
 
         UcH1LvlMod = UcSetNum ;
 
         RamWrite32A( gxlmt6L, MINLMT ) ;        /* 0x102D L-Limit */
         RamWrite32A( gxlmt6H, MAXLMT ) ;        /* 0x102E H-Limit */
         RamWrite32A( gylmt6L, MINLMT ) ;        /* 0x112D L-Limit */
         RamWrite32A( gylmt6H, MAXLMT ) ;        /* 0x112E H-Limit */
 
         RamWrite32A( gxmg,  CHGCOEF ) ;         /* 0x10AA Change coefficient gain */
         RamWrite32A( gymg,  CHGCOEF ) ;         /* 0x11AA Change coefficient gain */
         RegWriteA( WG_HCHR, 0x12 ) ;            // 0x011B   GmHChrOn[1]=1 Sw ON
         break ;
     }

     return 0;
 }
#endif

 //********************************************************************************
// Function Name 	: IniAdj
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
static int IniAdj( void )
{
#ifdef	CATCHMODE
 #ifdef	CORRECT_1DEG
	CHECK_RETURN(SelectIstpMod( ON ) );
 #else
	CHECK_RETURN(SelectIstpMod( OFF ) );
 #endif
#else
	CHECK_RETURN(SelectIstpMod( OFF )) ;
#endif// CATCHMODE
	CHECK_RETURN(IniPtAve( ) );								// Average setting
	
	/* OIS */
	RegWriteA( CMSDAC0, BIAS_CUR_OIS ) ;		// 0x0251	Hall Dac??
	RegWriteA( OPGSEL0, AMP_GAIN_X ) ;			// 0x0253	Hall amp Gain X
	RegWriteA( OPGSEL1, AMP_GAIN_Y ) ;			// 0x0254	Hall amp Gain Y
	/* AF */
	RegWriteA( CMSDAC1, BIAS_CUR_AF ) ;			// 0x0252	Hall Dac??
	RegWriteA( OPGSEL2, AMP_GAIN_AF ) ;			// 0x0255	Hall amp Gain AF

	RegWriteA( OSCSET, OSC_INI ) ;				// 0x0257	OSC ini
	
	/* adjusted value */
	RegWriteA( IZAH,	DGYRO_OFST_XH ) ;	// 0x02A0		Set Offset High byte
	RegWriteA( IZAL,	DGYRO_OFST_XL ) ;	// 0x02A1		Set Offset Low byte
	RegWriteA( IZBH,	DGYRO_OFST_YH ) ;	// 0x02A2		Set Offset High byte
	RegWriteA( IZBL,	DGYRO_OFST_YL ) ;	// 0x02A3		Set Offset Low byte
	
	/* Ram Access */
	CHECK_RETURN(RamAccFixMod( ON ) );							// 16bit Fix mode
	
	/* OIS adjusted parameter */
	RamWriteA( DAXHLO,		DAHLXO_INI ) ;		// 0x1479
	RamWriteA( DAXHLB,		DAHLXB_INI ) ;		// 0x147A
	RamWriteA( DAYHLO,		DAHLYO_INI ) ;		// 0x14F9
	RamWriteA( DAYHLB,		DAHLYB_INI ) ;		// 0x14FA
//	RamWriteA( OFF0Z,		HXOFF0Z_INI ) ;		// 0x1450
//	RamWriteA( OFF1Z,		HYOFF1Z_INI ) ;		// 0x14D0
	RamWriteA( sxg,			SXGAIN_INI ) ;		// 0x10D3
	RamWriteA( syg,			SYGAIN_INI ) ;		// 0x11D3
//	UsCntXof = OPTCEN_X ;						/* Clear Optical center X value */
//	UsCntYof = OPTCEN_Y ;						/* Clear Optical center Y value */
//	RamWriteA( SXOFFZ1,		UsCntXof ) ;		// 0x1461
//	RamWriteA( SYOFFZ1,		UsCntYof ) ;		// 0x14E1

	/* AF adjusted parameter */
//	RamWriteA( DAZHLO,		DAHLZO_INI ) ;		// 0x1529
//	RamWriteA( DAZHLB,		DAHLZB_INI ) ;		// 0x152A

	/* Ram Access */
	CHECK_RETURN(RamAccFixMod( OFF ) );							// 32bit Float mode
	
	RamWrite32A( gxzoom, GXGAIN_INI ) ;		// 0x1020 Gyro X axis Gain adjusted value
	RamWrite32A( gyzoom, GYGAIN_INI ) ;		// 0x1120 Gyro Y axis Gain adjusted value

	CHECK_RETURN(RamAccFixMod( ON )) ;						// 16bit Fix mode

	RamWriteA( OFF0Z,	HXOFF0Z_INI ) ;				// 0x1450
	RamWriteA( OFF1Z,	HYOFF1Z_INI ) ;				// 0x14D0
	
	CHECK_RETURN(RamAccFixMod( OFF ) );						// 32bit Float mode

	RamWrite32A( sxq, SXQ_INI ) ;			// 0x10E5	X axis output direction initial value
	RamWrite32A( syq, SYQ_INI ) ;			// 0x11E5	Y axis output direction initial value

#ifdef	CATCHMODE
	RamWrite32A( gx45g, G_45G_INI ) ;			// 0x1000
	RamWrite32A( gy45g, G_45G_INI ) ;			// 0x1100
	CHECK_RETURN(ClrGyr( 0x00FF , CLR_FRAM1 ));		// Gyro Delay RAM Clear
#endif	//CATCHMODE	
	
	RegWriteA( PWMA 	, 0xC0 );			// 0x0010		PWM enable

	RegWriteA( STBB0 	, 0xDF );			// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA( WC_EQSW	, 0x02 ) ;			// 0x01E0
	RegWriteA( WC_MESLOOP1	, 0x02 ) ;		// 0x0193
	RegWriteA( WC_MESLOOP0	, 0x00 ) ;		// 0x0192
	RegWriteA( WC_AMJLOOP1	, 0x02 ) ;		// 0x01A3
	RegWriteA( WC_AMJLOOP0	, 0x00 ) ;		// 0x01A2

#ifdef	CATCHMODE
	CHECK_RETURN(SetPanTiltMode( OFF ) );					/* Pan/Tilt OFF */

	CHECK_RETURN(SetH1cMod( ACTMODE ) );					/* Lvl Change Active mode */
	
	CHECK_RETURN(DrvSw( ON ) );							/* 0x0001		Driver Mode setting */
	
	RegWriteA( WC_EQON, 0x01 ) ;			// 0x0101	Filter ON
#else	//CATCHMODE	
	// I Filter X    2014.05.19				// 2s
	RamWrite32A( gxia_1, 0x3860DE00 ) ;		// 0.4Hz
	RamWrite32A( gxib_1, 0xB332EF82 ) ;		// Down
	RamWrite32A( gxic_1, 0x3332EF82 ) ;		// Up

	RamWrite32A( gxia_a, 0x3860DE00 ) ;		// 0.4Hz
	RamWrite32A( gxib_a, 0xB332EF82 ) ;		// Down
	RamWrite32A( gxic_a, 0x3332EF82 ) ;		// Up

	RamWrite32A( gxia_b, 0x3B038040 ) ;		// 15Hz	 2014.05.27
	RamWrite32A( gxib_b, 0xB332EF82 ) ;		// Down
	RamWrite32A( gxic_b, 0x3F800000 ) ;		// Up

	RamWrite32A( gxia_c, 0x3860DE00 ) ;		// 0.4Hz
	RamWrite32A( gxib_c, 0xB332EF82 ) ;		// Down
	RamWrite32A( gxic_c, 0x3332EF82 ) ;		// Up

	// I Filter Y    2014.05.19
	RamWrite32A( gyia_1, 0x3860DE00 ) ;		// 0.4Hz
	RamWrite32A( gyib_1, 0xB332EF82 ) ;		// Down
	RamWrite32A( gyic_1, 0x3332EF82 ) ;		// Up

	RamWrite32A( gyia_a, 0x3860DE00 ) ;		// 0.4Hz
	RamWrite32A( gyib_a, 0xB332EF82 ) ;		// Down
	RamWrite32A( gyic_a, 0x3332EF82 ) ;		// Up

	RamWrite32A( gyia_b, 0x3B038040 ) ;		// 15Hz	 2014.05.27
	RamWrite32A( gyib_b, 0xB332EF82 ) ;		// Down
	RamWrite32A( gyic_b, 0x3F800000 ) ;		// Up

	RamWrite32A( gyia_c, 0x3860DE00 ) ;		// 0.4Hz
	RamWrite32A( gyib_c, 0xB332EF82 ) ;		// Down
	RamWrite32A( gyic_c, 0x3332EF82 ) ;		// Up

	// gxgain
	RamWrite32A( gxgain_1, 0x3F800000 ) ;		// 0x1073	1
	RamWrite32A( gxgain_1d, 0xB7B2F402 ) ;		// 0x1074	Down
	RamWrite32A( gxgain_1u, 0x37B2F402 ) ;		// 0x1075	Up	//2.0s	//140314
	
	RamWrite32A( gxgain_a, 0x3F800000 ) ;		// 0x1076	1
	RamWrite32A( gxgain_2d, 0xB8DFB102 ) ;		// 0x1077	Down
	RamWrite32A( gxgain_2u, 0x38DFB102 ) ;		// 0x1078	Up
	
	RamWrite32A( gxgain_b, 0x00000000 ) ;		// 0x1079	Cut Off
	RamWrite32A( gxgain_3d, 0xBF800000 ) ;		// 0x107A	Down	//0.0s
	RamWrite32A( gxgain_3u, 0x38DFB102 ) ;		// 0x107B	Up
	
	RamWrite32A( gxgain_c, 0x3F800000 ) ;		// 0x107C	1
	RamWrite32A( gxgain_4d, 0xB8DFB102 ) ;		// 0x107D	Down	//0.4s
	RamWrite32A( gxgain_4u, 0x38DFB102 ) ;		// 0x107E	Up	//0.4s

	// gygain
	RamWrite32A( gygain_1, 0x3F800000 ) ;		// 1
	RamWrite32A( gygain_1d, 0xB7B2F402 ) ;		// Down
	RamWrite32A( gygain_1u, 0x37B2F402 ) ;		// Up	//2.0s	//140314
	
	RamWrite32A( gygain_a, 0x3F800000 ) ;		// 1
	RamWrite32A( gygain_2d, 0xB8DFB102 ) ;		// Down
	RamWrite32A( gygain_2u, 0x38DFB102 ) ;		// Up
	
	RamWrite32A( gygain_b, 0x00000000 ) ;		// Cut Off
	RamWrite32A( gygain_3d, 0xBF800000 ) ;		// Down	//0.0s
	RamWrite32A( gygain_3u, 0x38DFB102 ) ;		// Up
	
	RamWrite32A( gygain_c, 0x3F800000 ) ;		// 1
	RamWrite32A( gygain_4d, 0xB8DFB102 ) ;		// Down	//0.4s
	RamWrite32A( gygain_4u, 0x38DFB102 ) ;		// Up	//0.4s

	// gxistp								// 
	RamWrite32A( gxistp_1, 0x00000000 ) ;		// 0x1083	Cut Off
	RamWrite32A( gxistp_1d, 0xBF800000 ) ;		// 0x1084	Down
	RamWrite32A( gxistp_1u, 0x3F800000 ) ;		// 0x1085	Up
	
	RamWrite32A( gxistp_a, 0x00000000 ) ;		// 0x1086	Cut Off
	RamWrite32A( gxistp_2d, 0xBF800000 ) ;		// 0x1087	Down
	RamWrite32A( gxistp_2u, 0x3F800000 ) ;		// 0x1088	Up
	
	RamWrite32A( gxistp_b, 0x38D1B700 ) ;		// 0x1089	-80dB
	RamWrite32A( gxistp_3d, 0xBF800000 ) ;		// 0x108A	Down
	RamWrite32A( gxistp_3u, 0x3F800000 ) ;		// 0x108B	Up
	
	RamWrite32A( gxistp_c, 0x00000000 ) ;		// 0x108C	Cut Off
	RamWrite32A( gxistp_4d, 0xBF800000 ) ;		// 0x108D	Down
	RamWrite32A( gxistp_4u, 0x3F800000 ) ;		// 0x108E	Up

	// gyistp
	RamWrite32A( gyistp_1, 0x00000000 ) ;		// Cut Off
	RamWrite32A( gyistp_1d, 0xBF800000 ) ;		// Down
	RamWrite32A( gyistp_1u, 0x3F800000 ) ;		// Up
	
	RamWrite32A( gyistp_a, 0x00000000 ) ;		// Cut Off
	RamWrite32A( gyistp_2d, 0xBF800000 ) ;		// Down
	RamWrite32A( gyistp_2u, 0x3F800000 ) ;		// Up
	
	RamWrite32A( gyistp_b, 0x38D1B700 ) ;		// -80dB
	RamWrite32A( gyistp_3d, 0xBF800000 ) ;		// Down
	RamWrite32A( gyistp_3u, 0x3F800000 ) ;		// Up
	
	RamWrite32A( gyistp_c, 0x00000000 ) ;		// Cut Off
	RamWrite32A( gyistp_4d, 0xBF800000 ) ;		// Down
	RamWrite32A( gyistp_4u, 0x3F800000 ) ;		// Up

#ifdef H1COEF_CHANGER
	CHECK_RETURN(SetH1cMod( ACTMODE ) );					/* Lvl Change Active mode */
#endif
	
	CHECK_RETURN(DrvSw( ON ) );							/* 0x0001		Driver Mode setting */
	
	RegWriteA( WC_EQON, 0x01 ) ;			// 0x0101	Filter ON
	
	RegWriteA( WG_NPANST12BTMR, 0x01 ) ;		// 0x0167
	CHECK_RETURN(SetPanTiltMode( ON ) );						// Pan/Tilt
	RegWriteA( WG_PANSTT6, 0x44 ) ;				// 0x010A
	RegWriteA( WG_PANSTT6, 0x11 ) ;				// 0x010A
#endif //catchmode
#ifndef MODULE_CALIBRATION
	RamWrite32A( sxlmta2, X_CURRENT_LMT );		// 0.4 --Hall limitter rex.tang 20141107
	RamWrite32A( sylmta2, Y_CURRENT_LMT );      // 0.4 --Hall limitter rex.tang 20141107
#endif

    return 0;
}

 /*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 *
 *
 *      Source from OisIni.h
 *
 *
 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
//********************************************************************************
// Function Name 	: IniSetAll
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
static int IniSetAll( void )
{
//TRACE("Initial CMD \n") ;
	//IniCmd();
	// Clock Setting
//TRACE("Initial CLK \n") ;
	CHECK_RETURN(IniClk());
	CHECK_RETURN(IniAf());
	// I/O Port Initial Setting
//TRACE("Initial IOP \n") ;
	CHECK_RETURN(IniIop());
	// Monitor & Other Initial Setting
//TRACE("Initial MON \n") ;
	//IniMon();
	// Servo Initial Setting
//TRACE("Initial SRV \n") ;
	CHECK_RETURN(IniSrv());
	// Gyro Filter Initial Setting
//TRACE("Initial GYR \n") ;
	CHECK_RETURN(IniGyr());
	// Gyro Filter Initial Setting
//TRACE("Initial GFL \n") ;
	CHECK_RETURN(IniFil());
	// DigitalGyro Initial Setting
//TRACE("Initial DGY \n") ;
	CHECK_RETURN(IniDgy());
	// Adjust Fix Value Setting
//TRACE("Initial ADJ \n") ;
	CHECK_RETURN(IniAdj());

    return 0;
}

 //********************************************************************************
 // Function Name    : GyrCon
 // Retun Value      : NON
 // Argment Value    : Gyro Filter ON or OFF
 // Explanation      : Gyro Filter Control Function
 // History          : First edition                         2013.01.15 Y.Shigeoka
 //********************************************************************************
 static int GyrCon( uint8_t    UcGyrCon )
 {
     uint8_t UcAccMod ;
 
     RegReadA( WC_RAMACCMOD, &UcAccMod ) ;
 
     // Return HPF Setting
     RegWriteA( WG_SHTON, 0x00 ) ;                                   // 0x0107
     
     if( UcGyrCon == ON ) {                                              // Gyro ON
         CHECK_RETURN(ClrGyr( 0x000E , CLR_FRAM1 ));           // Gyro Delay RAM Clear
 
         if( UcAccMod == 0x31 ){
             RamWriteA( sxggf, 0x7FFF ) ;        // 0x10B5
             RamWriteA( syggf, 0x7FFF ) ;        // 0x11B5
             
             CHECK_RETURN(RamAccFixMod( OFF ) );
             RamWrite32A( gxib_1, 0xBF800000 ) ;     // Down
             RamWrite32A( gyib_1, 0xBF800000 ) ;     // Down
 //          RamWrite32A( gxib_1, 0xB261CF49 ) ;     // Down
 //          RamWrite32A( gyib_1, 0xB261CF49 ) ;     // Down
             CHECK_RETURN(IniPtMovMod( OFF ));                     //2014.05.19
             RamAccFixMod( ON ) ;
         }else{
             RamWrite32A( sxggf, 0x3F800000 ) ;  // 0x10B5
             RamWrite32A( syggf, 0x3F800000 ) ;  // 0x11B5
             
             RamWrite32A( gxib_1, 0xBF800000 ) ;     // Down
             RamWrite32A( gyib_1, 0xBF800000 ) ;     // Down
 //          RamWrite32A( gxib_1, 0xB261CF49 ) ;     // Down
 //          RamWrite32A( gyib_1, 0xB261CF49 ) ;     // Down
             CHECK_RETURN(IniPtMovMod( OFF ));                     //2014.05.19
         }
         RegWriteA( WG_PANSTT6, 0x00 ) ;             // 0x010A
         //SetPanTiltMode(OFF) ;                     // Pantilt OFF
     } else if( UcGyrCon == SPC ) {                  // Gyro ON for LINE
         RamWrite32A( sxggf, 0x3F800000 ) ;  // 0x10B5
         RamWrite32A( syggf, 0x3F800000 ) ;  // 0x11B5
     } else {                                                        // Gyro OFF
         if( UcAccMod == 0x31 ){
             RamWriteA( sxggf, 0x0000 ) ;        // 0x10B5
             RamWriteA( syggf, 0x0000 ) ;        // 0x11B5
         }else{
             RamWrite32A( sxggf, 0x00000000 ) ;  // 0x10B5
             RamWrite32A( syggf, 0x00000000 ) ;  // 0x11B5
         }
     }

     return 0;
 }

 /*---------------------------------------------
  Function: Auto focus driver set position
  Argument: pos
  Return: None
  History: First edition  Rex.Tang 20141023
 ----------------------------------------------*/
 static int AF_SetPos( short pos )
 {
     //#define    SW_FSTMODE     0x0400  // Fast Stable Mode
     //RamWriteA( TCODEH , pos | SW_FSTMODE);
     RamWriteA(TREG_H , pos << 6);
     return 0;
 }

 //********************************************************************************
 // Function Name    : CmdRdChk
 // Retun Value      : 1 : ERROR
 // Argment Value    : NON
 // Explanation      : Check Cver function
 // History          : First edition                         2043.02.27 K.abe
 //********************************************************************************
 static uint8_t CmdRdChk( void )
 {
     uint8_t UcTestRD;
     uint8_t UcCount;
 
     for(UcCount=0; UcCount < READ_COUNT_NUM; UcCount++){
         RegReadA( TESTRD ,  &UcTestRD );                    // 0x027F
         if( UcTestRD == 0xAC){
             return(0);
         }
     }
     //TRACE("***** Command Line Error !! Can't Read Data *****\n" ) ;
     //TRACE_ERROR();
     return(1);
 }

 static int SrvCon( uint8_t    UcDirSel, uint8_t   UcSwcCon );

 //********************************************************************************
 // Function Name    : StbOnn
 // Retun Value      : NON
 // Argment Value    : NON
 // Explanation      : Stabilizer For Servo On Function
 // History          : First edition                         2013.01.09 Y.Shigeoka
 //********************************************************************************
 static int StbOnn( void )
 {
     uint8_t UcRegValx,UcRegValy;                    // Registor value
     uint8_t UcRegIni ;
     uint8_t UcRegIniCnt = 0;
     uint8_t SumCnt = 0;
 
     RegReadA( WH_EQSWX , &UcRegValx ) ;         // 0x0170
     RegReadA( WH_EQSWY , &UcRegValy ) ;         // 0x0171
 
     if( (( UcRegValx & 0x01 ) != 0x01 ) && (( UcRegValy & 0x01 ) != 0x01 ))
     {
 
         RegWriteA( WH_SMTSRVON, 0x01 ) ;                // 0x017C       Smooth Servo ON
 
         CHECK_RETURN(SrvCon( X_DIR, ON )) ;
         CHECK_RETURN(SrvCon( Y_DIR, ON ) );
 
         UcRegIni = 0x11;
         while( (UcRegIni & 0x77) != 0x66 )
         {
             RegReadA( RH_SMTSRVSTT, &UcRegIni ) ;       // 0x01F8       Smooth Servo phase read
             if( CmdRdChk() !=0 )    break;              // Dead Lock check (responce check)
             if((UcRegIni & 0x77 ) == 0 )    UcRegIniCnt++ ;
             if( UcRegIniCnt > 10 ){
                 break ;         // Status Error
             }
             WitTim(3);
             SumCnt++;
             if(SumCnt > 20) {
                 pr_err("%s:Can not smooth move break while",__func__);
                 break;
             }
         }
         RegWriteA( WH_SMTSRVON, 0x00 ) ;                // 0x017C       Smooth Servo OFF
 
     }
     else
     {
         CHECK_RETURN(SrvCon( X_DIR, ON )) ;
         CHECK_RETURN(SrvCon( Y_DIR, ON )) ;
 //TRACE( " Not Slope \n" ) ;
     }
     return 0;
 }
//********************************************************************************
// Function Name    : StbOnnN
// Retun Value      : NON
// Argment Value    : NON
// Explanation      : Stabilizer For Servo On Function
// History          : First edition                         2013.10.09 Y.Shigeoka
//********************************************************************************
int imx278_lgit_ois_StbOnnN( uint8_t UcStbY , uint8_t UcStbX )
{
    uint8_t UcRegIni ;
    uint8_t UcSttMsk = 0 ;
    uint8_t UcRegIniCnt = 0;

    RegWriteA( WH_SMTSRVON, 0x01 ) ;// 0x017C Smooth Servo ON
    if( UcStbX == ON ) UcSttMsk |= 0x07 ;
    if( UcStbY == ON ) UcSttMsk |= 0x70 ;

    CHECK_RETURN(SrvCon( X_DIR, UcStbX )) ;
    CHECK_RETURN(SrvCon( Y_DIR, UcStbY )) ;

    UcRegIni = 0x11;
    while( (UcRegIni & UcSttMsk) != ( 0x66 & UcSttMsk ) )
    {
        RegReadA( RH_SMTSRVSTT, &UcRegIni ) ;// 0x01F8 Smooth Servo phase read

        if( CmdRdChk() !=0 ) break;// Dead Lock check (responce check)
        if((UcRegIni & 0x77 ) == 0 ) UcRegIniCnt++ ;
        if( UcRegIniCnt > 10 ){
            break ;// Status Error
        }

    }
    RegWriteA( WH_SMTSRVON,	0x00 ) ;// 0x017C Smooth Servo OFF
    return 0;
}

 //********************************************************************************
 // Function Name    : SrvCon
 // Retun Value      : NON
 // Argment Value    : X or Y Select, Servo ON/OFF
 // Explanation      : Servo ON,OFF Function
 // History          : First edition                         2013.01.09 Y.Shigeoka
 //********************************************************************************
 static int SrvCon( uint8_t    UcDirSel, uint8_t   UcSwcCon )
 {
     uint8_t UcAccMod ;
     
     RegReadA( WC_RAMACCMOD, &UcAccMod ) ;
     if( UcSwcCon ) {
         if( !UcDirSel ) {                       // X Direction
             RegWriteA( WH_EQSWX , 0x03 ) ;          // 0x0170
             if( UcAccMod == 0x31 ){ 
                 RamWriteA( sxggf, 0x0000 ) ;            // 0x10B5
             }else{
                 RamWrite32A( sxggf, 0x00000000 ) ;      // 0x10B5
             }        
         } else {                                // Y Direction
             RegWriteA( WH_EQSWY , 0x03 ) ;          // 0x0171
             if( UcAccMod == 0x31 ){ 
                 RamWriteA( syggf, 0x0000 ) ;            // 0x11B5
             }else{
                 RamWrite32A( syggf, 0x00000000 ) ;      // 0x11B5
             }
         }
     } else {
         if( !UcDirSel ) {                       // X Direction
             RegWriteA( WH_EQSWX , 0x02 ) ;          // 0x0170
             if( UcAccMod == 0x31 ){ 
                 RamWriteA( SXLMT, 0x0000 ) ;            // 0x1477
             }else{
                 RamWrite32A( SXLMT, 0x00000000 ) ;      // 0x1477
             }
         } else {                                // Y Direction
             RegWriteA( WH_EQSWY , 0x02 ) ;          // 0x0171
             if( UcAccMod == 0x31 ){ 
                 RamWriteA( SYLMT, 0x0000 ) ;            // 0x14F7
             }else{
                 RamWrite32A( SYLMT, 0x00000000 ) ;      // 0x14F7
             }
         }
     }

     return 0;
 }

 //********************************************************************************
 // Function Name    : RtnCen
 // Retun Value      : Command Status
 // Argment Value    : Command Parameter
 // Explanation      : Return to center Command Function
 // History          : First edition                         2013.01.15 Y.Shigeoka
 //********************************************************************************
 static int RtnCen( uint8_t UcCmdPar )
 {
     //uint8_t UcCmdSts ;
 
     //UcCmdSts    = EXE_END ;
 
     CHECK_RETURN(GyrCon( OFF ) );                                         // Gyro OFF
 
     if( !UcCmdPar ) {                                       // X,Y Centering
 
         CHECK_RETURN(StbOnn() );                                          // Slope Mode
 
     } else if( UcCmdPar == 0x01 ) {                         // X Centering Only
 
         CHECK_RETURN(SrvCon( X_DIR, ON ) );                               // X only Servo ON
         CHECK_RETURN(SrvCon( Y_DIR, OFF )) ;
     } else if( UcCmdPar == 0x02 ) {                         // Y Centering Only
 
         CHECK_RETURN(SrvCon( X_DIR, OFF ) );                              // Y only Servo ON
         CHECK_RETURN(SrvCon( Y_DIR, ON ) );
     }
 
     return( 0 ) ;
 }

#ifdef	ACCEPTANCE
 //********************************************************************************
 // Function Name    : MesFil
 // Retun Value      : NON
 // Argment Value    : Measure Filter Mode
 // Explanation      : Measure Filter Setting Function
 // History          : First edition                         2009.07.31  Y.Tashita
 //********************************************************************************
 static int MesFil( uint8_t    UcMesMod )
 {
#ifdef	USE_EXTCLK_ALL	// 24MHz
     if( !UcMesMod ) {                               // Hall Bias&Offset Adjust
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3D1E5A40 ) ;     // 0x10F0   LPF150Hz
         RamWrite32A( mes1ab, 0x3D1E5A40 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x3F6C34C0 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3F800000 ) ;     // 0x10F5   Through
         RamWrite32A( mes1bb, 0x00000000 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x00000000 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3D1E5A40 ) ;     // 0x11F0   LPF150Hz
         RamWrite32A( mes2ab, 0x3D1E5A40 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x3F6C34C0 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3F800000 ) ;     // 0x11F5   Through
         RamWrite32A( mes2bb, 0x00000000 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x00000000 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
 
     } else if( UcMesMod == LOOPGAIN ) {             // Loop Gain Adjust
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3E587E00 ) ;     // 0x10F0   LPF1000Hz
         RamWrite32A( mes1ab, 0x3E587E00 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x3F13C100 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3F7DF500 ) ;     // 0x10F5   HPF30Hz
         RamWrite32A( mes1bb, 0xBF7DF500 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x3F7BEA40 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3E587E00 ) ;     // 0x11F0   LPF1000Hz
         RamWrite32A( mes2ab, 0x3E587E00 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x3F13C100 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3F7DF500 ) ;     // 0x11F5   HPF30Hz
         RamWrite32A( mes2bb, 0xBF7DF500 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x3F7BEA40 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
 
     } else if( UcMesMod == THROUGH ) {              // for Through
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3F800000 ) ;     // 0x10F0   Through
         RamWrite32A( mes1ab, 0x00000000 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x00000000 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3F800000 ) ;     // 0x10F5   Through
         RamWrite32A( mes1bb, 0x00000000 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x00000000 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3F800000 ) ;     // 0x11F0   Through
         RamWrite32A( mes2ab, 0x00000000 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x00000000 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3F800000 ) ;     // 0x11F5   Through
         RamWrite32A( mes2bb, 0x00000000 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x00000000 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
 
     } else if( UcMesMod == NOISE ) {                // SINE WAVE TEST for NOISE
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3D1E5A40 ) ;     // 0x10F0   LPF150Hz
         RamWrite32A( mes1ab, 0x3D1E5A40 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x3F6C34C0 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3D1E5A40 ) ;     // 0x10F5   LPF150Hz
         RamWrite32A( mes1bb, 0x3D1E5A40 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x3F6C34C0 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3D1E5A40 ) ;     // 0x11F0   LPF150Hz
         RamWrite32A( mes2ab, 0x3D1E5A40 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x3F6C34C0 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3D1E5A40 ) ;     // 0x11F5   LPF150Hz
         RamWrite32A( mes2bb, 0x3D1E5A40 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x3F6C34C0 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
     }
#else
     if( !UcMesMod ) {                               // Hall Bias&Offset Adjust
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3CA175C0 ) ;     // 0x10F0   LPF150Hz
         RamWrite32A( mes1ab, 0x3CA175C0 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x3F75E8C0 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3F800000 ) ;     // 0x10F5   Through
         RamWrite32A( mes1bb, 0x00000000 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x00000000 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3CA175C0 ) ;     // 0x11F0   LPF150Hz
         RamWrite32A( mes2ab, 0x3CA175C0 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x3F75E8C0 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3F800000 ) ;     // 0x11F5   Through
         RamWrite32A( mes2bb, 0x00000000 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x00000000 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
 
     } else if( UcMesMod == LOOPGAIN ) {             // Loop Gain Adjust
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3DF21080 ) ;     // 0x10F0   LPF1000Hz
         RamWrite32A( mes1ab, 0x3DF21080 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x3F437BC0 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3F7EF980 ) ;     // 0x10F5   HPF30Hz
         RamWrite32A( mes1bb, 0xBF7EF980 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x3F7DF300 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3DF21080 ) ;     // 0x11F0   LPF1000Hz
         RamWrite32A( mes2ab, 0x3DF21080 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x3F437BC0 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3F7EF980 ) ;     // 0x11F5   HPF30Hz
         RamWrite32A( mes2bb, 0xBF7EF980 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x3F7DF300 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
 
     } else if( UcMesMod == THROUGH ) {              // for Through
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3F800000 ) ;     // 0x10F0   Through
         RamWrite32A( mes1ab, 0x00000000 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x00000000 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3F800000 ) ;     // 0x10F5   Through
         RamWrite32A( mes1bb, 0x00000000 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x00000000 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3F800000 ) ;     // 0x11F0   Through
         RamWrite32A( mes2ab, 0x00000000 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x00000000 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3F800000 ) ;     // 0x11F5   Through
         RamWrite32A( mes2bb, 0x00000000 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x00000000 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
 
     } else if( UcMesMod == NOISE ) {                // SINE WAVE TEST for NOISE
         // Measure Filter1 Setting
         RamWrite32A( mes1aa, 0x3CA175C0 ) ;     // 0x10F0   LPF150Hz
         RamWrite32A( mes1ab, 0x3CA175C0 ) ;     // 0x10F1
         RamWrite32A( mes1ac, 0x3F75E8C0 ) ;     // 0x10F2
         RamWrite32A( mes1ad, 0x00000000 ) ;     // 0x10F3
         RamWrite32A( mes1ae, 0x00000000 ) ;     // 0x10F4
         RamWrite32A( mes1ba, 0x3CA175C0 ) ;     // 0x10F5   LPF150Hz
         RamWrite32A( mes1bb, 0x3CA175C0 ) ;     // 0x10F6
         RamWrite32A( mes1bc, 0x3F75E8C0 ) ;     // 0x10F7
         RamWrite32A( mes1bd, 0x00000000 ) ;     // 0x10F8
         RamWrite32A( mes1be, 0x00000000 ) ;     // 0x10F9
 
         // Measure Filter2 Setting
         RamWrite32A( mes2aa, 0x3CA175C0 ) ;     // 0x11F0   LPF150Hz
         RamWrite32A( mes2ab, 0x3CA175C0 ) ;     // 0x11F1
         RamWrite32A( mes2ac, 0x3F75E8C0 ) ;     // 0x11F2
         RamWrite32A( mes2ad, 0x00000000 ) ;     // 0x11F3
         RamWrite32A( mes2ae, 0x00000000 ) ;     // 0x11F4
         RamWrite32A( mes2ba, 0x3CA175C0 ) ;     // 0x11F5   LPF150Hz
         RamWrite32A( mes2bb, 0x3CA175C0 ) ;     // 0x11F6
         RamWrite32A( mes2bc, 0x3F75E8C0 ) ;     // 0x11F7
         RamWrite32A( mes2bd, 0x00000000 ) ;     // 0x11F8
         RamWrite32A( mes2be, 0x00000000 ) ;     // 0x11F9
     }
#endif
    return 0;
 }

 //********************************************************************************
 // Function Name    : BsyWit
 // Retun Value      : NON
 // Argment Value    : Trigger Register Address, Trigger Register Data
 // Explanation      : Busy Wait Function
 // History          : First edition                         2013.01.09 Y.Shigeoka
 //********************************************************************************
 static int BsyWit( uint16_t   UsTrgAdr, uint8_t   UcTrgDat )
 {
     uint8_t UcFlgVal ;
 
     RegWriteA( UsTrgAdr, UcTrgDat ) ;   // Trigger Register Setting
 
     UcFlgVal    = 1 ;
 
     while( UcFlgVal ) {
 
         RegReadA( UsTrgAdr, &UcFlgVal ) ;
         UcFlgVal    &=  ( UcTrgDat & 0x0F ) ;
 
         if( CmdRdChk() !=0 )    break;      // Dead Lock check (responce check)
 
     } ;

     return 0;
}
 //********************************************************************************
 // Function Name    : GenMes
 // Retun Value      : A/D Convert Result
 // Argment Value    : Measure Filter Input Signal Ram Address
 // Explanation      : General Measure Function
 // History          : First edition                         2013.01.10 Y.Shigeoka
 //********************************************************************************
 static short GenMes( uint16_t  UsRamAdd, uint8_t   UcMesMod )
 {
     short   SsMesRlt ;
 
     RegWriteA( WC_MES1ADD0, (uint8_t)UsRamAdd ) ;                           // 0x0194
     RegWriteA( WC_MES1ADD1, (uint8_t)(( UsRamAdd >> 8 ) & 0x0001 ) ) ;  // 0x0195
     RamWrite32A( MSABS1AV, 0x00000000 ) ;               // 0x1041   Clear
 
     if( !UcMesMod ) {
         RegWriteA( WC_MESLOOP1, 0x04 ) ;                // 0x0193
         RegWriteA( WC_MESLOOP0, 0x00 ) ;                // 0x0192   1024 Times Measure
         RamWrite32A( msmean , 0x3A7FFFF7 );             // 0x1230   1/CmMesLoop[15:0]
     } else {
         RegWriteA( WC_MESLOOP1, 0x01 ) ;                // 0x0193
         RegWriteA( WC_MESLOOP0, 0x00 ) ;                // 0x0192   1 Times Measure
         RamWrite32A( msmean , 0x3F800000 );             // 0x1230   1/CmMesLoop[15:0]
     }
 
     RegWriteA( WC_MESABS, 0x00 ) ;                      // 0x0198   none ABS
     CHECK_RETURN(BsyWit( WC_MESMODE, 0x01 ) );                        // 0x0190   normal Measure
 
     CHECK_RETURN(RamAccFixMod( ON ));                            // Fix mode
 
     RamReadA( MSABS1AV, ( uint16_t * )&SsMesRlt ) ; // 0x1041
 
     CHECK_RETURN(RamAccFixMod( OFF )) ;                           // Float mode
 
     return( SsMesRlt ) ;
 }

 //********************************************************************************
 // Function Name    : RunGea
 // Retun Value      : Result
 // Argment Value    : NON
 // Explanation      : Gyro Examination of Acceptance
 // History          : First edition                         2014.02.13 T.Tokoro
 //********************************************************************************
 static int RunGea( void )
 {
     uint8_t     UcRst, UcCnt, UcXLowCnt, UcYLowCnt, UcXHigCnt, UcYHigCnt ;
     uint16_t    UsGxoVal[10], UsGyoVal[10], UsDif;
     short genmes_value = 0;
 
     UcRst = EXE_END ;
     UcXLowCnt = UcYLowCnt = UcXHigCnt = UcYHigCnt = 0 ;
 
     CHECK_RETURN(MesFil( THROUGH ) );             // 測定用フィルターを設定する。
 
     for( UcCnt = 0 ; UcCnt < 10 ; UcCnt++ )
     {
         // X
         RegWriteA( WC_MES1ADD0, 0x00 ) ;        // 0x0194
         RegWriteA( WC_MES1ADD1, 0x00 ) ;        // 0x0195
         CHECK_RETURN(ClrGyr( 0x1000 , CLR_FRAM1 ));                           // Measure Filter RAM Clear
         genmes_value = GenMes( GX45Z, 0 );
         if(genmes_value < 0)
         {
             pr_err("%s:%d fail \n",__func__,__LINE__);
             return -1;
         }
 //        UsGxoVal[UcCnt] = (uint16_t)GenMes( AD2Z, 0 );    // 64回の平均値測定 GYRMON1(0x1110) <- GXADZ(0x144A)
         UsGxoVal[UcCnt] = (uint16_t)genmes_value; // 64回の平均値測定 GYRMON1(0x1110) <- GX45Z(0x1401)        
 
         // Y
         RegWriteA( WC_MES1ADD0, 0x00 ) ;        // 0x0194
         RegWriteA( WC_MES1ADD1, 0x00 ) ;        // 0x0195
         CHECK_RETURN(ClrGyr( 0x1000 , CLR_FRAM1 ));                           // Measure Filter RAM Clear
         genmes_value = GenMes( GY45Z, 0 );
         if(genmes_value < 0)
         {
             pr_err("%s:%d fail \n",__func__,__LINE__);
             return -1;
         }
 //        UsGyoVal[UcCnt] = (uint16_t)GenMes( AD3Z, 0 );    // 64回の平均値測定 GYRMON2(0x1111) <- GYADZ(0x14CA)
         UsGyoVal[UcCnt] = (uint16_t)genmes_value; // 64回の平均値測定 GYRMON2(0x1111) <- GY45Z(0x1481)
 
 //TRACE("UcCnt = %02x, UsGxoVal[UcCnt] = %04x\n", UcCnt, UsGxoVal[UcCnt] ) ;
 //TRACE("UcCnt = %02x, UsGyoVal[UcCnt] = %04x\n", UcCnt, UsGyoVal[UcCnt] ) ;
 
 //      WitTim( 100 ) ;                         // Wait 100ms
 
         if( UcCnt > 0 )
         {
             if ( (short)UsGxoVal[0] > (short)UsGxoVal[UcCnt] ) {
                 UsDif = (uint16_t)((short)UsGxoVal[0] - (short)UsGxoVal[UcCnt]) ;
             } else {
                 UsDif = (uint16_t)((short)UsGxoVal[UcCnt] - (short)UsGxoVal[0]) ;
             }
#if 0
             if( UsDif > GEA_DIF_HIG ) {
                 //UcRst = UcRst | EXE_GXABOVE ;
                 UcXHigCnt ++ ;
             }
#endif
             if( UsDif < GEA_DIF_LOW ) {
                 //UcRst = UcRst | EXE_GXBELOW ;
                 UcXLowCnt ++ ;
             }
 
             if ( (short)UsGyoVal[0] > (short)UsGyoVal[UcCnt] ) {
                 UsDif = (uint16_t)((short)UsGyoVal[0] - (short)UsGyoVal[UcCnt]) ;
             } else {
                 UsDif = (uint16_t)((short)UsGyoVal[UcCnt] - (short)UsGyoVal[0]) ;
             }
#if 0
             if( UsDif > GEA_DIF_HIG ) {
                 //UcRst = UcRst | EXE_GYABOVE ;
                 UcYHigCnt ++ ;
             }
#endif
             if( UsDif < GEA_DIF_LOW ) {
                 //UcRst = UcRst | EXE_GYBELOW ;
                 UcYLowCnt ++ ;
             }
         }
     }
 
     if( UcXHigCnt >= 1 ) {
         UcRst = UcRst | EXE_GXABOVE ;
     }
     if( UcXLowCnt > 8 ) {
         UcRst = UcRst | EXE_GXBELOW ;
     }
 
     if( UcYHigCnt >= 1 ) {
         UcRst = UcRst | EXE_GYABOVE ;
     }
     if( UcYLowCnt > 8 ) {
         UcRst = UcRst | EXE_GYBELOW ;
     }
 
 //TRACE("UcRst = %02x\n", UcRst ) ;
 
     return (int)UcRst ;
 }

#define		USE_SINLPF			/* if sin or circle movement is used LPF , this define has to enable */
  
  static int SetSinWavePara( uint8_t UcTableVal ,  uint8_t UcMethodVal )
  {
      uint16_t    UsFreqDat ;
      uint8_t UcEqSwX , UcEqSwY ;
  
  
      if(UcTableVal > 0x10 )
          UcTableVal = 0x10 ;         /* Limit */
      UsFreqDat = CucFreqVal[ UcTableVal ] ;
  
      if( UcMethodVal == SINEWAVE) {
          RegWriteA( WC_SINPHSX, 0x00 ) ;                 /* 0x0183   */
          RegWriteA( WC_SINPHSY, 0x00 ) ;                 /* 0x0184   */
      }else if( UcMethodVal == CIRCWAVE ){
          RegWriteA( WC_SINPHSX,  0x00 ) ;                /* 0x0183   */
          RegWriteA( WC_SINPHSY,  0x20 ) ;                /* 0x0184   */
      }else{
          RegWriteA( WC_SINPHSX, 0x00 ) ;                 /* 0x0183   */
          RegWriteA( WC_SINPHSY, 0x00 ) ;                 /* 0x0184   */
      }
  
#ifdef	USE_SINLPF
      if(( UcMethodVal == CIRCWAVE ) || ( UcMethodVal == SINEWAVE )) {
          CHECK_RETURN(MesFil( NOISE )) ;           /* LPF */
      }
#endif
  
      if( UsFreqDat == 0xFFFF )           /* Sine波中止 */
      {
  
          RegReadA( WH_EQSWX, &UcEqSwX ) ;                /* 0x0170   */
          RegReadA( WH_EQSWY, &UcEqSwY ) ;                /* 0x0171   */
          UcEqSwX &= ~EQSINSW ;
          UcEqSwY &= ~EQSINSW ;
          RegWriteA( WH_EQSWX, UcEqSwX ) ;                /* 0x0170   */
          RegWriteA( WH_EQSWY, UcEqSwY ) ;                /* 0x0171   */
  
#ifdef	USE_SINLPF
          if(( UcMethodVal == CIRCWAVE ) || ( UcMethodVal == SINEWAVE ) || ( UcMethodVal == XACTTEST ) || ( UcMethodVal == YACTTEST )) {
              RegWriteA( WC_DPON,     0x00 ) ;            /* 0x0105   Data pass off */
              RegWriteA( WC_DPO1ADD0, 0x00 ) ;            /* 0x01B8   output initial */
              RegWriteA( WC_DPO1ADD1, 0x00 ) ;            /* 0x01B9   output initial */
              RegWriteA( WC_DPO2ADD0, 0x00 ) ;            /* 0x01BA   output initial */
              RegWriteA( WC_DPO2ADD1, 0x00 ) ;            /* 0x01BB   output initial */
              RegWriteA( WC_DPI1ADD0, 0x00 ) ;            /* 0x01B0   input initial */
              RegWriteA( WC_DPI1ADD1, 0x00 ) ;            /* 0x01B1   input initial */
              RegWriteA( WC_DPI2ADD0, 0x00 ) ;            /* 0x01B2   input initial */
              RegWriteA( WC_DPI2ADD1, 0x00 ) ;            /* 0x01B3   input initial */
  
              /* Ram Access */
              CHECK_RETURN(RamAccFixMod( ON )) ;                            // Fix mode
  
              RamWriteA( SXOFFZ1, UsCntXof ) ;            /* 0x1461   set optical value */
              RamWriteA( SYOFFZ1, UsCntYof ) ;            /* 0x14E1   set optical value */
  
              /* Ram Access */
              CHECK_RETURN(RamAccFixMod( OFF )) ;                           // Float mode
  
              RegWriteA( WC_MES1ADD0,  0x00 ) ;           /* 0x0194   */
              RegWriteA( WC_MES1ADD1,  0x00 ) ;           /* 0x0195   */
              RegWriteA( WC_MES2ADD0,  0x00 ) ;           /* 0x0196   */
              RegWriteA( WC_MES2ADD1,  0x00 ) ;           /* 0x0197   */
  
          }
#endif
          RegWriteA( WC_SINON,     0x00 ) ;           /* 0x0180   Sine wave  */
  
      }
      else
      {
  
          RegReadA( WH_EQSWX, &UcEqSwX ) ;                /* 0x0170   */
          RegReadA( WH_EQSWY, &UcEqSwY ) ;                /* 0x0171   */
  
          if(( UcMethodVal == CIRCWAVE ) || ( UcMethodVal == SINEWAVE )) {
#ifdef	USE_SINLPF
              RegWriteA( WC_DPI1ADD0,  ( uint8_t )MES1BZ2 ) ;                     /* 0x01B0   input Meas-Fil */
              RegWriteA( WC_DPI1ADD1,  ( uint8_t )(( MES1BZ2 >> 8 ) & 0x0001 ) ) ;    /* 0x01B1   input Meas-Fil */
              RegWriteA( WC_DPI2ADD0,  ( uint8_t )MES2BZ2 ) ;                     /* 0x01B2   input Meas-Fil */
              RegWriteA( WC_DPI2ADD1,  ( uint8_t )(( MES2BZ2 >> 8 ) & 0x0001 ) ) ;    /* 0x01B3   input Meas-Fil */
              RegWriteA( WC_DPO1ADD0, ( uint8_t )SXOFFZ1 ) ;                      /* 0x01B8   output SXOFFZ1 */
              RegWriteA( WC_DPO1ADD1, ( uint8_t )(( SXOFFZ1 >> 8 ) & 0x0001 ) ) ; /* 0x01B9   output SXOFFZ1 */
              RegWriteA( WC_DPO2ADD0, ( uint8_t )SYOFFZ1 ) ;                      /* 0x01BA   output SYOFFZ1 */
              RegWriteA( WC_DPO2ADD1, ( uint8_t )(( SYOFFZ1 >> 8 ) & 0x0001 ) ) ; /* 0x01BA   output SYOFFZ1 */
  
              RegWriteA( WC_MES1ADD0,  ( uint8_t )SINXZ ) ;                           /* 0x0194   */
              RegWriteA( WC_MES1ADD1,  ( uint8_t )(( SINXZ >> 8 ) & 0x0001 ) ) ;  /* 0x0195   */
              RegWriteA( WC_MES2ADD0,  ( uint8_t )SINYZ ) ;                           /* 0x0196   */
              RegWriteA( WC_MES2ADD1,  ( uint8_t )(( SINYZ >> 8 ) & 0x0001 ) ) ;  /* 0x0197   */
  
              RegWriteA( WC_DPON,     0x03 ) ;            /* 0x0105   Data pass[1:0] on */
  
              UcEqSwX &= ~EQSINSW ;
              UcEqSwY &= ~EQSINSW ;
#else
              UcEqSwX |= 0x08 ;
              UcEqSwY |= 0x08 ;
#endif
          } else if(( UcMethodVal == XACTTEST ) || ( UcMethodVal == YACTTEST )) {
              RegWriteA( WC_DPI2ADD0,  ( uint8_t )MES2BZ2 ) ;                     /* 0x01B2   input Meas-Fil */
              RegWriteA( WC_DPI2ADD1,  ( uint8_t )(( MES2BZ2 >> 8 ) & 0x0001 ) ) ;    /* 0x01B3   input Meas-Fil */
              if( UcMethodVal == XACTTEST ){
                  RegWriteA( WC_DPO2ADD0, ( uint8_t )SXOFFZ1 ) ;                      /* 0x01BA   output SXOFFZ1 */
                  RegWriteA( WC_DPO2ADD1, ( uint8_t )(( SXOFFZ1 >> 8 ) & 0x0001 ) ) ; /* 0x01BB   output SXOFFZ1 */
                  RegWriteA( WC_MES2ADD0,  ( uint8_t )SINXZ ) ;                           /* 0x0196   */
                  RegWriteA( WC_MES2ADD1,  ( uint8_t )(( SINXZ >> 8 ) & 0x0001 ) ) ;  /* 0x0197   */
              } else {
                  RegWriteA( WC_DPO2ADD0, ( uint8_t )SYOFFZ1 ) ;                      /* 0x01BA   output SYOFFZ1 */
                  RegWriteA( WC_DPO2ADD1, ( uint8_t )(( SYOFFZ1 >> 8 ) & 0x0001 ) ) ; /* 0x01BB   output SYOFFZ1 */
                  RegWriteA( WC_MES2ADD0,  ( uint8_t )SINYZ ) ;                           /* 0x0196   */
                  RegWriteA( WC_MES2ADD1,  ( uint8_t )(( SINYZ >> 8 ) & 0x0001 ) ) ;  /* 0x0197   */
              }
  
              RegWriteA( WC_DPON,     0x02 ) ;            /* 0x0105   Data pass[1] on */
  
              UcEqSwX &= ~EQSINSW ;
              UcEqSwY &= ~EQSINSW ;
  
          }else{
              if( UcMethodVal == XHALWAVE ){
                  UcEqSwX = 0x22 ;                /* SW[5] */
  //              UcEqSwY = 0x03 ;
              }else{
  //              UcEqSwX = 0x03 ;
                  UcEqSwY = 0x22 ;                /* SW[5] */
              }
          }
  
          RegWriteA( WC_SINFRQ0,  (uint8_t)UsFreqDat ) ;              // 0x0181       Freq L
          RegWriteA( WC_SINFRQ1,  (uint8_t)(UsFreqDat >> 8) ) ;           // 0x0182       Freq H
          RegWriteA( WC_MESSINMODE,     0x00 ) ;          /* 0x0191   Sine 0 cross  */
  
          RegWriteA( WH_EQSWX, UcEqSwX ) ;                /* 0x0170   */
          RegWriteA( WH_EQSWY, UcEqSwY ) ;                /* 0x0171   */
  
          RegWriteA( WC_SINON,     0x01 ) ;           /* 0x0180   Sine wave  */
  
      }
  
    return 0;
  }

 
 static int TstActMov( uint8_t UcDirSel )
 {
     int UcRsltSts;
     uint16_t    UsMsppVal ;
 
     CHECK_RETURN(MesFil( NOISE ) );
 
     if ( !UcDirSel ) {
         RamWrite32A( sxsin , ACT_CHK_LVL );                                             // 0x10D5
         RamWrite32A( sysin , 0x000000 );                                                // 0x11D5
         CHECK_RETURN(SetSinWavePara( 0x05 , XACTTEST ));
     }else{
         RamWrite32A( sxsin , 0x000000 );                                                // 0x10D5
         RamWrite32A( sysin , ACT_CHK_LVL );                                             // 0x11D5
         CHECK_RETURN(SetSinWavePara( 0x05 , YACTTEST ));
     }
 
     if ( !UcDirSel ) {                  // AXIS X
         RegWriteA( WC_MES1ADD0,  ( uint8_t )SXINZ1 ) ;                          /* 0x0194   */
         RegWriteA( WC_MES1ADD1,  ( uint8_t )(( SXINZ1 >> 8 ) & 0x0001 ) ) ;     /* 0x0195   */
     } else {                            // AXIS Y
         RegWriteA( WC_MES1ADD0,  ( uint8_t )SYINZ1 ) ;                          /* 0x0194   */
         RegWriteA( WC_MES1ADD1,  ( uint8_t )(( SYINZ1 >> 8 ) & 0x0001 ) ) ;     /* 0x0195   */
     }
 
     RegWriteA( WC_MESSINMODE, 0x00 ) ;          // 0x0191   0 cross
     RegWriteA( WC_MESLOOP1, 0x00 ) ;                // 0x0193
     RegWriteA( WC_MESLOOP0, 0x02 ) ;                // 0x0192   2 Times Measure
     RamWrite32A( msmean , 0x3F000000 );             // 0x10AE   1/CmMesLoop[15:0]/2
     RegWriteA( WC_MESABS, 0x00 ) ;              // 0x0198   none ABS
     CHECK_RETURN(BsyWit( WC_MESMODE, 0x02 ) );                // 0x0190       Sine wave Measure
 
     CHECK_RETURN(RamAccFixMod( ON ) );                            // Fix mode
     RamReadA( MSPP1AV, &UsMsppVal ) ;
     CHECK_RETURN(RamAccFixMod( OFF )) ;                           // Float mode
 
     if ( !UcDirSel ) {                  // AXIS X
         CHECK_RETURN(SetSinWavePara( 0x00 , XACTTEST ));  /* STOP */
     }else{
         CHECK_RETURN(SetSinWavePara( 0x00 , YACTTEST ));  /* STOP */
     }
 
 //TRACE(" DIR = %04x, PP = %04x, ", UcDirSel, UsMsppVal ) ;
 
     UcRsltSts = EXE_END ;
     if( UsMsppVal > ACT_THR ){
         if ( !UcDirSel ) {                  // AXIS X
             UcRsltSts = EXE_HXMVER ;
         }else{                              // AXIS Y
             UcRsltSts = EXE_HYMVER ;
         }
     }
 
     return( UcRsltSts ) ;
 }
 
 //********************************************************************************
 // Function Name    : RunHea
 // Retun Value      : Result
 // Argment Value    : NON
 // Explanation      : Hall Examination of Acceptance
 // History          : First edition                         2014.02.26 Y.Shigeoka
 //********************************************************************************
 static int RunHea( void )
 {
     int     UcRst ;
     int     data;
 
     UcRst = EXE_END ;

     data = TstActMov( X_DIR) ;
     if(data < 0)
     {
        pr_err("%s:%d fail\n",__func__,__LINE__);
        return -1;
     }
     UcRst |= data ;
     
     data = TstActMov( Y_DIR) ;
     if(data < 0)
     {
        pr_err("%s:%d fail\n",__func__,__LINE__);
        return -1;
     }
     UcRst |= data ;
 
 //TRACE("UcRst = %02x\n", UcRst ) ;
 
     return( UcRst ) ;
 }
#endif	//ACCEPTANCE

 //********************************************************************************
 // Function Name    : OisEna
 // Retun Value      : NON
 // Argment Value    : Command Parameter
 // Explanation      : OIS Enable Control Function
 // History          : First edition                         2013.01.15 Y.Shigeoka
 //********************************************************************************
 static int OisEna( void )
 {
     // Servo ON
     //SrvCon( X_DIR, ON ) ;
     //SrvCon( Y_DIR, ON ) ;
 
     CHECK_RETURN(GyrCon( ON )) ;

     return 0;
 }

static int setOISdata(void)
{
    pr_info("%s:willow begin \n",__func__);
    pr_info("%s:willow osc=%x DAXHLO=%2x \n",__func__,StAdjPar.UcOscVal,StAdjPar.StHalAdj.UsHlxOff);
    if((StAdjPar.UcOscVal != 0x00) && (StAdjPar.UcOscVal != 0xFF))
        RegWriteA( OSCSET, StAdjPar.UcOscVal ) ;
    CHECK_RETURN(RamAccFixMod( ON ));
    if((StAdjPar.StHalAdj.UsHlxOff != 0x0000) && (StAdjPar.StHalAdj.UsHlxOff != 0xFFFF))
        RamWriteA( DAXHLO, StAdjPar.StHalAdj.UsHlxOff );
    if((StAdjPar.StHalAdj.UsHlxGan != 0x0000) && (StAdjPar.StHalAdj.UsHlxGan != 0xFFFF))
        RamWriteA( DAXHLB, StAdjPar.StHalAdj.UsHlxGan );
    if((StAdjPar.StHalAdj.UsHlyOff != 0x0000) && (StAdjPar.StHalAdj.UsHlyOff != 0xFFFF))
        RamWriteA( DAYHLO, StAdjPar.StHalAdj.UsHlyOff );
    if((StAdjPar.StHalAdj.UsHlyGan != 0x0000) && (StAdjPar.StHalAdj.UsHlyGan != 0xFFFF))
        RamWriteA( DAYHLB, StAdjPar.StHalAdj.UsHlyGan );
    if((StAdjPar.StHalAdj.UsAdxOff != 0x0000) && (StAdjPar.StHalAdj.UsAdxOff != 0xFFFF))
        RamWriteA( OFF0Z,  StAdjPar.StHalAdj.UsAdxOff );
    if((StAdjPar.StHalAdj.UsAdyOff != 0x0000) && (StAdjPar.StHalAdj.UsAdyOff != 0xFFFF))		
        RamWriteA( OFF1Z,  StAdjPar.StHalAdj.UsAdyOff );
    if((StAdjPar.StLopGan.UsLxgVal != 0x0000) && (StAdjPar.StLopGan.UsLxgVal != 0xFFFF))
        RamWriteA( sxg,    StAdjPar.StLopGan.UsLxgVal );			// X Loop Gain
    if((StAdjPar.StLopGan.UsLygVal != 0x0000) && (StAdjPar.StLopGan.UsLygVal != 0xFFFF))
        RamWriteA( syg,    StAdjPar.StLopGan.UsLygVal );			// Y Loop Gain
    if((StAdjPar.StGvcOff.UsGxoVal != 0x0000) && (StAdjPar.StGvcOff.UsGxoVal != 0xFFFF))
        RamWriteA( IZAH,   StAdjPar.StGvcOff.UsGxoVal );			// X Gyro Offset
    if((StAdjPar.StGvcOff.UsGyoVal != 0x0000) && (StAdjPar.StGvcOff.UsGyoVal != 0xFFFF))
        RamWriteA( IZBH,   StAdjPar.StGvcOff.UsGyoVal );			// Y Gyro Offset
    CHECK_RETURN(RamAccFixMod( OFF ));
    if((StAdjPar.StGvcOff.UlGxgVal != 0x00000000) && (StAdjPar.StGvcOff.UlGxgVal != 0xFFFFFFFF))
        RamWrite32A( gxzoom, StAdjPar.StGvcOff.UlGxgVal );
    if((StAdjPar.StGvcOff.UlGygVal != 0x00000000) && (StAdjPar.StGvcOff.UlGygVal != 0xFFFFFFFF))
        RamWrite32A( gyzoom, StAdjPar.StGvcOff.UlGygVal );

    return 0;
}


int imx278_lgit_ois_init(void)
{
    unsigned char DrvVer, MdlVer, FwVer;
    
    pr_info("%s: enter\n",__func__);
    CHECK_RETURN(IniSetAll());

    CHECK_RETURN(setOISdata());

    //version check    
    RegReadA(0x027E, &DrvVer);   
    RegReadA(0x00FF, &MdlVer);    
    RegReadA(0x02D0, &FwVer);

    pr_info("LC298122 OIS Driver IC version: 0x%02x \n", DrvVer);    
    pr_info("LC298122 OIS Module version: 0x%02x \n", MdlVer);    
    pr_info("LC298122 OIS Firmware version: 0x%02x \n", FwVer);

    //move lens outside from AF dead area
    //m_manager.m_oisModule->AF_SetPos(100);
    //open OIS
    //lens centering
    CHECK_RETURN(RtnCen(0x00));
    //default off

    WitTim(20);
    //Pan/Tilt ON
    CHECK_RETURN(SetPanTiltMode(1));
    WitTim(10);

    /*init ois is closed*/
    CHECK_RETURN(OisEna());

    return 0;
}

int imx278_lgit_ois_initSpecial(void)
{
     return IniSetAf();
}
int imx278_lgit_ois_change_mode(uint32_t mode)
{
	if(mode == OIS_PREVIEW)
	{
    	RamWrite32A( sxlmta1, 	X_CURRENT_LMT_PREVIEW );		// 0x10E7 		Hall X output Limit
    	RamWrite32A( sylmta1, 	Y_CURRENT_LMT_PREVIEW ); 
	}
	else if (mode == OIS_CAPTURE)
	{
    	RamWrite32A( sxlmta1, 	X_CURRENT_LMT_CAPTURE );		// 0x10E7 		Hall X output Limit
    	RamWrite32A( sylmta1, 	Y_CURRENT_LMT_CAPTURE ); 
	}
    return 0;
}


int imx278_lgit_ois_turn_onoff(uint32_t on)
{
    if(on)
    {
        CHECK_RETURN(imx278_lgit_ois_change_mode(OIS_CAPTURE));
        CHECK_RETURN(OisEna());
    }
    else
    {
    	CHECK_RETURN(imx278_lgit_ois_change_mode(OIS_PREVIEW));
        CHECK_RETURN(RtnCen(0x00));
    }

    return 0;
}
int imx278_lgit_ois_turn_onoff_lin(uint32_t on)
{
    if(on)
    {
        CHECK_RETURN(imx278_lgit_ois_change_mode(OIS_CAPTURE));
        CHECK_RETURN(OisEna());
    }
    else
    {
        CHECK_RETURN(imx278_lgit_ois_change_mode(OIS_PREVIEW));
        CHECK_RETURN(RtnCen(0x00));
    }
    return 0;
}

void imx278_lgit_InitOISData(void *oisdata, uint32_t oislenth)
 {
    uint8_t *data = (uint8_t *)oisdata;
    StAdjPar.StHalAdj.UsHlxOff = (data[20] << 8)|data[21];    //DAXHLO Hall offset X
    StAdjPar.StHalAdj.UsHlyOff = (data[22] << 8)|data[23];    //DAYHLO Hall offset Y
    StAdjPar.StHalAdj.UsHlxGan = (data[16] << 8)|data[17];    //DAXHLB Hall bias X
    StAdjPar.StHalAdj.UsHlyGan = (data[18] << 8)|data[19];    //DAYHLB Hall bias Y
    StAdjPar.StHalAdj.UsAdxOff = (data[28] << 8)|data[29];    //HXOFF0Z_INI Hall AD offset X
    StAdjPar.StHalAdj.UsAdyOff = (data[30] << 8)|data[31];  //HYOFF1Z_INI Hall AD offset Y
    StAdjPar.StLopGan.UsLxgVal = (data[24] << 8)|data[25];  //sxg  SXGAIN_INI Loop gain X 
    StAdjPar.StLopGan.UsLygVal = (data[26] << 8)|data[27];  //syg  SYGAIN_INI Loop gain Y
    UsCntXof                   = (data[16] << 8)|data[17];  //optical center X
    UsCntYof                   = (data[18] << 8)|data[19];  //optical center Y
    StAdjPar.StGvcOff.UsGxoVal = (data[32] << 8)|data[33];  //IZAH DGYRO_OFST_XH Gyro offset X
    StAdjPar.StGvcOff.UsGyoVal = (data[34] << 8)|data[35];  //IZBH DGYRO_OFST_YH Gyro offset Y
    StAdjPar.UcOscVal          = data[36];
    StAdjPar.StGvcOff.UlGxgVal = (data[43] << 24)|(data[44] << 16)|(data[45] << 8)|data[46];// gxzoom UlGxgVal
    StAdjPar.StGvcOff.UlGygVal = (data[47] << 24)|(data[48] << 16)|(data[49] << 8)|data[50];// gyzoom UlGygVal
    UsvalFw                    = (data[51] << 8)|data[52];
 }

 //for test interface
int imx278_lgit_ois_AF_SetPos(short data)
{
    return AF_SetPos(data);
}
int imx278_lgit_ois_RamAccFixMod(uint8_t data)
{
    return RamAccFixMod(data);
}
int imx278_lgit_ois_RtnCen(uint8_t data)
{
    return RtnCen(data);
}
int imx278_lgit_ois_SetPanTiltMode(uint8_t data)
{
    return SetPanTiltMode(data);
}
int imx278_lgit_ois_RunGea(void)
{
    return RunGea();
}
int imx278_lgit_ois_RunHea(void)
{
    return RunHea();
}
int imx278_lgit_ois_OisEna(void)
{
    return OisEna();
}
int imx278_lgit_ois_MagnetismRead(int32_t *otpcenX, int32_t *otpcenY, int32_t *srvoffX, int32_t *srvoffY)
{
    RamRead32A(OFF0Z ,otpcenX);
    RamRead32A(OFF1Z ,otpcenY);
    RamRead32A(AD0OFFZ, srvoffX);
    RamRead32A(AD1OFFZ, srvoffY);
    return 0;
}

