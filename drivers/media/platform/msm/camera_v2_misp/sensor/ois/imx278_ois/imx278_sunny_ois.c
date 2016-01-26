/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#include <linux/init.h>
#include <linux/io.h>

#include "imx278_sunny_ois.h"
#include "imx278_sunny_ois_data.h"
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

#define RamRead32A(a,b)          do { \
    if(mini_isp_ois_RamRead32A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RamWriteA(a,b)           do { \
    if(mini_isp_ois_RamWrite16A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RamReadA(a,b)            do { \
if(mini_isp_ois_RamRead16A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RegWriteA(a,b)   do { \
 if(mini_isp_ois_RegWrite8A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define RegReadA(a,b)            do { \
    if(mini_isp_ois_RegRead8A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define WitTim              mini_isp_ois_WitTim

#define CntWrtRam(a,b)            do { \
    if(mini_isp_ois_RamWrite16Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define CntWrtRag(a,b)           do { \
    if(mini_isp_ois_RegWrite8Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define CntWrtRam32(a,b)         do { \
    if(mini_isp_ois_RamWrite32Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

#define CHECK_RETURN(a) do { \
    if(a != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 *
 *
 *      Source from Ois.h
 *
 *
 * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

/**************** Select Gyro Sensor **************/
//#define 	USE_3WIRE_DGYRO    //for D-Gyro SPI interface

#define		USE_INVENSENSE				// INVENSENSE
//#define		USE_STMICRO_L2G2IS			// STMicro-L2G2IS


#ifdef USE_INVENSENSE
//			#define		FS_SEL		0		/* Å}262LSB/Åã/s  */
//			#define		FS_SEL		1		/* Å}131LSB/Åã/s  */
//			#define		FS_SEL		2		/* Å}65.5LSB/Åã/s  */
            #define		FS_SEL		3		/* Å}32.8LSB/Åã/s  */

//			#define		GYROSTBY			/* Sleep+STBY */
#endif

/**************** Model name *****************/
#define MDL_VER         0x09		// P13N12A  

/**************** FW version *****************/
#define	FW_VER			0x04

/**************** Select Mode **************/
#define		STANDBY_MODE			// STANDBY Mode

//#define		PWM_BREAK			// PWM mode select (disable standby)
#define		NEUTRAL_CENTER			// Upper Position Current 0mA Measurement

//#define		MODULE_CALIBRATION		// for module maker   use float
#define		CORRECT_1DEG			// Correct 1deg   disable 0.5deg
#define		CATCHMODE               // Catch mode --Rex.Tang 20141029
#define		ACCEPTANCE					// Examination of Acceptance

//#define		DEF_SET				// default value re-setting
#define		MONITOR_OFF				// default Monitor output at Standby mode


// Command Status
#define		EXE_END		0x02		// Execute End (Adjust OK)
#define		EXE_HXADJ	0x06		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0A		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x12		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x22		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x42		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x82		// Adjust NG : Y Gyro NG (offset)
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


#undef ON
#undef OFF
#ifndef ON
 #define	ON				0x01		// ON
 #define	OFF				0x00		// OFF
#endif
 #define	SPC				0x02		// Special Mode

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction
#define	X2_DIR			0x10		// X Direction
#define	Y2_DIR			0x11		// Y Direction

#ifdef MODULE_CALIBRATION  
#define		X_CURRENT_LMT	0x3F800000	// 1.0	X-axis current limit  
#define		Y_CURRENT_LMT	0x3F800000  // 1.0	Y-axis current limit  
#else
#define		X_CURRENT_LMT	0x3E6B851F	// 0.23	X-axis current limit  
#define		Y_CURRENT_LMT	0x3E6B851F  // 0.23	Y-axis current limit  
#endif


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
 #define		DAHLXO_INI		0x1772
 #define		DAHLXB_INI		0xD790
 #define		DAHLYO_INI		0x146C
 #define		DAHLYB_INI		0xD320
 #define		SXGAIN_INI		0x344A
 #define		SYGAIN_INI		0x3236
 #define		HXOFF0Z_INI		0x0000//0xF85B
 #define		HYOFF1Z_INI		0x0000//0xF19D
/* OSC Init */
 #define		OSC_INI			0x2E		/* VDD=2.8V */

// Digital Gyro offset Initial value
#define		DGYRO_OFST_XH	0x00
#define		DGYRO_OFST_XL	0x00
#define		DGYRO_OFST_YH	0x00
#define		DGYRO_OFST_YL	0x00
#define			GXGAIN_INI		0xBF17999A	// Gyro X Zoom Value
#define			GYGAIN_INI		0x3F0F999A	// Gyro Y Zoom Value

 #define		BIAS_CUR_OIS     0x22        //1.0mA/1.0mA
 #define        AMP_GAIN_X      0x02         //x50
 #define        AMP_GAIN_Y      0x02         //x50



/* AF Open para */
#define			RWEXD1_L_AF		0x7FFF                 //
#define			RWEXD2_L_AF		0x6697                 //
#define			RWEXD3_L_AF		0x7414                 //
#define			FSTCTIME_AF		0x38                   //

#define			FSTMODE_AF		0x00                   //

/* (0.3750114X^3+0.55X)*(0.3750114X^3+0.55X) 9.2ohm*/
#define         A3_IEXP3        0x3EC0017F
#define         A1_IEXP1        0x3F0CCCCD

/* AF adjust parameter */
#define		DAHLZB_INI		0x8001		// abe 2014.06.03
#define		DAHLZO_INI		0x0000
#define		BIAS_CUR_AF		0x00		//0.25mA
#define		AMP_GAIN_AF		0x00		//x6

#define		SXGAIN_LOP		0x3000
#define		SYGAIN_LOP		0x3000

#define		TCODEH_ADJ		0x0000

#ifdef	CATCHMODE
/************* Wide *************/
 #define		GYRLMT3_S1_W	0x3F0CCCCD		//0.55F
 #define		GYRLMT3_S2_W	0x3F0CCCCD		//0.55F
 #define		GYRLMT4_S1_W	0x40300000		//2.75F
 #define		GYRLMT4_S2_W	0x40300000		//2.75F
 #define		GYRISTP_W		0x3A51B700		/* -62dB */
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

#else /* CATCHMODE */
#ifdef	CORRECT_1DEG
 #define		GYRLMT1H		0x3DCCCCCD		//0.1F

 #define		GYRLMT3_S1		0x3F19999A		//0.60F
 #define		GYRLMT3_S2		0x3F19999A		//0.60F

 #define		GYRLMT4_S1		0x40400000		//3.0F
 #define		GYRLMT4_S2		0x40400000		//3.0F

 #define		GYRA12_HGH		0x402CCCCD		/* 2.70F */		// modified by abe 140814
 #define		GYRA12_MID		0x3F800000		/* 1.0F */
 #define		GYRA34_HGH		0x3F000000		/* 0.5F */
 #define		GYRA34_MID		0x3DCCCCCD		/* 0.1F */

 #define		GYRB12_HGH		0x3E4CCCCD		/* 0.20F */
 #define		GYRB12_MID		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_HGH		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_MID		0x3C23D70A		/* 0.001F */

 #define		GYRISTP		0x39D0B900			/* -68dB */
#else /* CORRECT_1DEG */
 #define		GYRLMT3_S1		0x3ECCCCCD		//0.40F
 #define		GYRLMT3_S2		0x3ECCCCCD		//0.40F

 #define		GYRLMT4_S1		0x40000000		//2.0F
 #define		GYRLMT4_S2		0x40000000		//2.0F

 #define		GYRA12_HGH		0x3FC00000		/* 1.50F */
 #define		GYRA12_MID		0x3F800000		/* 1.0F */
 #define		GYRA34_HGH		0x3F000000		/* 0.5F */
 #define		GYRA34_MID		0x3DCCCCCD		/* 0.1F */

 #define		GYRB12_HGH		0x3E4CCCCD		/* 0.20F */
 #define		GYRB12_MID		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_HGH		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_MID		0x3C23D70A		/* 0.001F */

 #define		GYRISTP			0x38D1B700		/* -80dB */
#endif /* CORRECT_1DEG */
#endif /* CATCHMODE */

//#define		OPTCEN_X		0x0000
//#define		OPTCEN_Y		0x0000

#define			SXQ_INI			0x3F800000		/* Hall Filter Connection Setting(sxq, syq) */
#define			SYQ_INI			0x3F800000		// 0x3F800000 -> Positive, 0xBF800000 -> negative


#define			GYROX_INI		0x45	// Gyro X axis select
#define			GYROY_INI		0x43	// Gyro Y axis select

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
#define		STILLMODE	0x00
#ifdef	CATCHMODE
 #define		MOVMODE		0xFE
 #define		MOVMODE_W	0xFF
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


 /*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  *
  *
  *      Source from OisCmd.h
  *
  *
  * @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
 //**************************
 //  define
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
  #define		MAXLMT_W		0x40400000				// 3.0
  #define		MINLMT_W		0x4019999A				// 2.4
  #define		CHGCOEF_W		0xB9855555				// 
  #define		MINLMT_MOV_W	0x4019999A				// 2.4
  #define		CHGCOEF_MOV_W	0xB9200000
  #define		MAXLMT			0x40000000				// 2.0
  #define		MINLMT			0x3F99999A				// 1.2
  #define		CHGCOEF			0xB9480000				// 
  #define		MINLMT_MOV		0x3F99999A				// 1.2
  #define		CHGCOEF_MOV		0xB8F00000
#else
#ifdef	CORRECT_1DEG
  #define		MAXLMT		0x40600000				// 3.5  // modified by abe 140814
  #define		MINLMT		0x400CCCCD				// 2.2	// modified by abe 140814
  #define		CHGCOEF		0xBA0D89D9				//
 
  #define		MINLMT_MOV	0x00000000				// 0.0
  #define		CHGCOEF_MOV	0xB8A49249
 #else
  #define		MAXLMT			0x40000000				// 2.0
  #define		MINLMT			0x3F8CCCCD				// 1.1  
  #define		CHGCOEF			0xBA4C71C7				//   
  #define		MINLMT_MOV		0x00000000				// 0.0
  #define		CHGCOEF_MOV		0xB9700000  
#endif	/*	CORRECT_1DEG */  
#endif /* CATCHMODE */


 /*--------------------------------------------------------
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
 #define		DIFIL_S2		0x3F7FFE00
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

static stAdjPar StAdjPar ;              // Execute Command Parameter
static uint8_t UcCvrCod ;              /* CverCode */
static uint8_t UcPwmMod ;              /* PWM MODE */

static uint8_t	UcOscAdjFlg ;		// For Measure trigger
static uint16_t	UsCntXof ;				/* OPTICAL Center Xvalue */
static uint16_t	UsCntYof ;				/* OPTICAL Center Yvalue */


//uint8_t	UcCvrCod ;				/* CverCode */

//static uint32_t	UlH1Coefval ;		// H1 coefficient value --remove for compile
static uint8_t	UcH1LvlMod ;		// H1 level coef mode

//static uint16_t	UsStpSiz ;		    // Bias Step Size --remove for compile
//static uint16_t	UsErrBia ; //--remove for compile
//static uint16_t  UsErrOfs ; //--remove for compile
//static uint16_t	UsValBef ; //--remove for compile
//static uint16_t  UsValNow ; //--remove for compile

static uint8_t CmdRdChk( void );
//static void MemClr( uint8_t	*NcTgtPtr, uint16_t	UsClrSiz );


#if 0
//********************************************************************************
// Function Name 	: IniCmd
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
static void IniCmd( void )
{

	MemClr( ( uint8_t * )&StAdjPar, sizeof( stAdjPar ) ) ;	// Adjust Parameter Clear
	
}
#endif

//********************************************************************************
// Function Name 	: BsyWit
// Retun Value		: NON
// Argment Value	: Trigger Register Address, Trigger Register Data
// Explanation		: Busy Wait Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
static int BsyWit( uint16_t	UsTrgAdr, uint8_t	UcTrgDat )
{
	uint8_t	UcFlgVal ;

	RegWriteA( UsTrgAdr, UcTrgDat ) ;	// Trigger Register Setting

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {

		RegReadA( UsTrgAdr, &UcFlgVal ) ;
		UcFlgVal	&= 	( UcTrgDat & 0x0F ) ;

		if( CmdRdChk() !=0 )	break;		// Dead Lock check (responce check)

	} ;

    return 0;
}

#if 0
//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
static void MemClr( uint8_t	*NcTgtPtr, uint16_t	UsClrSiz )
{
	uint16_t	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}
#endif

//********************************************************************************
// Function Name 	: ChkCvr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2013.10.03 Y.Shigeoka
//********************************************************************************
static int ChkCvr( void )
{
	RegReadA( CVER ,	&UcCvrCod );		// 0x027E
	pr_info("%s: UcCvrCod=%d \n",__func__,(int)UcCvrCod);
	RegWriteA( MDLREG ,	MDL_VER );			// 0x00FF	Model
	RegWriteA( VRREG ,	FW_VER );			// 0x02D0	Version

    return 0;
}

//********************************************************************************
// Function Name 	: IniClk
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
static int IniClk( void )
{
    CHECK_RETURN(ChkCvr());             /* Read Cver */
    
	/*OSC Enables*/
	UcOscAdjFlg	= 0 ;					// Osc adj flag 
	
#ifdef	DEF_SET
	/*OSC ENABLE*/
    RegWriteA( OSCSTOP,		0x00 ) ;			// 0x0256
	RegWriteA( OSCSET,		0x90 ) ;			// 0x0257	OSC ini
	RegWriteA( OSCCNTEN,	0x00 ) ;			// 0x0258	OSC Cnt disable
#endif
	/*Clock Enables*/
	RegWriteA( CLKON,		0x1F ) ;			// 0x020B

#ifdef	DEF_SET
	RegWriteA( CLKSEL,		0x00 ) ;			// 0x020C	

	RegWriteA( PWMDIV,		0x00 ) ;			// 0x0210	48MHz/1
	RegWriteA( SRVDIV,		0x00 ) ;			// 0x0211	48MHz/1
	RegWriteA( GIFDIV,		0x03 ) ;			// 0x0212	48MHz/3 = 16MHz
	RegWriteA( AFPWMDIV,	0x02 ) ;			// 0x0213	48MHz/2 = 24MHz
	RegWriteA( OPAFDIV,		0x04 ) ;			// 0x0214	48MHz/4 = 12MHz
#endif

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
// Function Name 	: IniAf
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Open AF Initial Setting
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
static int IniAf( void )
{
	uint8_t	UcStbb0 ;
	
	CHECK_RETURN(AfDrvSw( OFF ) );								/* AF Drvier Block Ena=0 */

	RegWriteA( DRVFCAF	, 0x20 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
	RegWriteA( DRVFC4AF	, 0x80 );					// 0x0084	DOFSTDAF
	RegWriteA( AFFC,   0x80 ) ;						// 0x0088	OpenAF/-/-
#ifdef	DEF_SET
	RegWriteA( DRVFC3AF	, 0x00 );					// 0x0083	DGAINDAF	Gain 0
	RegWriteA( PWMAAF,    0x00 ) ;					// 0x0090	AF PWM standby

	RegWriteA( DRVFC2AF,    0x00 ) ;				// 0x0082	AF slope0
	RegWriteA( DRVCH3SEL,   0x00 ) ;				// 0x0085	AF H bridge control
#endif
	RegWriteA( PWMFCAF,     0x01 ) ;				// 0x0091	AF VREF , Carrier , MODE1
	RegWriteA( PWMPERIODAF, 0x20 ) ;				// 0x0099	AF none-synchronism
	RegWriteA( CCFCAF,   0x40 ) ;					// 0x00A1	GND/-
	
	RegReadA( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x7F ;
	RegWriteA( STBB0, UcStbb0 ) ;			// 0x0250	OIS standby
	RegWriteA( STBB1, 0x00 ) ;				// 0x0264	All standby
	
	/* AF Initial setting */
	RegWriteA( FSTMODE,		FSTMODE_AF ) ;		// 0x0302
	RamWriteA( RWEXD1_L,	RWEXD1_L_AF ) ;		// 0x0396 - 0x0397 (Register continuos write)
	RamWriteA( RWEXD2_L,	RWEXD2_L_AF ) ;		// 0x0398 - 0x0399 (Register continuos write)
	RamWriteA( RWEXD3_L,	RWEXD3_L_AF ) ;		// 0x039A - 0x039B (Register continuos write)
	RegWriteA( FSTCTIME,	FSTCTIME_AF ) ;		// 0x0303 	
	RamWriteA( TCODEH,		0x0400 ) ;			// 0x0304 - 0x0305 (Register continuos write)
	
	UcStbb0 |= 0x80 ;
	RegWriteA( STBB0, UcStbb0 ) ;			// 0x0250	
	RegWriteA( STBB1	, 0x05 ) ;			// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]

	CHECK_RETURN(AfDrvSw( ON ) );								/* AF Drvier Block Ena=1 */

    return 0;
}

//********************************************************************************
// Function Name 	: IniIop
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
static int IniIop( void )
{
#ifdef	DEF_SET
	/*set IOP direction*/
	RegWriteA( P0LEV,		0x00 ) ;	// 0x0220	[ - 	| - 	| WLEV5 | WLEV4 ][ WLEV3 | WLEV2 | WLEV1 | WLEV0 ]
	RegWriteA( P0DIR,		0x00 ) ;	// 0x0221	[ - 	| - 	| DIR5	| DIR4	][ DIR3  | DIR2  | DIR1  | DIR0  ]
	/*set pull up/down*/
	RegWriteA( P0PON,		0x0F ) ;	// 0x0222	[ -    | -	  | PON5 | PON4 ][ PON3  | PON2 | PON1 | PON0 ]
	RegWriteA( P0PUD,		0x0F ) ;	// 0x0223	[ -    | -	  | PUD5 | PUD4 ][ PUD3  | PUD2 | PUD1 | PUD0 ]
#endif
	/*select IOP signal*/
#ifdef	USE_3WIRE_DGYRO
	RegWriteA( IOP1SEL,		0x02 ); 	// 0x0231	IOP1 : IOP1
#else
	RegWriteA( IOP1SEL,		0x00 ); 	// 0x0231	IOP1 : DGDATAIN (ATT:0236h[0]=1)
#endif
#ifdef	DEF_SET
	RegWriteA( IOP0SEL,		0x02 ); 	// 0x0230	IOP0 : IOP0
	RegWriteA( IOP2SEL,		0x02 ); 	// 0x0232	IOP2 : IOP2
	RegWriteA( IOP3SEL,		0x00 ); 	// 0x0233	IOP3 : DGDATAOUT
	RegWriteA( IOP4SEL,		0x00 ); 	// 0x0234	IOP4 : DGSCLK
	RegWriteA( IOP5SEL,		0x00 ); 	// 0x0235	IOP5 : DGSSB
	RegWriteA( DGINSEL,		0x00 ); 	// 0x0236	DGDATAIN 0:IOP1 1:IOP2
	RegWriteA( I2CSEL,		0x00 );		// 0x0248	I2C noise reduction ON
	RegWriteA( DLMODE,		0x00 );		// 0x0249	Download OFF
#endif

    return 0;
}

//********************************************************************************
// Function Name 	: AccWit
// Retun Value		: NON
// Argment Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
static int AccWit( uint8_t UcTrgDat )
{
	uint8_t	UcFlgVal ;
	uint8_t	UcCntPla ;
	UcFlgVal	= 1 ;
	UcCntPla	= 0 ;
	
	do{
		RegReadA( GRACC, &UcFlgVal ) ;
		UcFlgVal	&= UcTrgDat ;
		UcCntPla++ ;
	} while( UcFlgVal && ( UcCntPla < 6 ) ) ;

    return 0;
}

//********************************************************************************
// Function Name 	: GyOutSignal
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
static int GyOutSignal( void )
{

	RegWriteA( GRADR0,	GYROX_INI ) ;			// 0x0283	Set Gyro XOUT H~L
	RegWriteA( GRADR1,	GYROY_INI ) ;			// 0x0284	Set Gyro YOUT H~L
	
	/*Start OIS Reading*/
	RegWriteA( GRSEL	, 0x02 );			// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

    return 0;
}


//********************************************************************************
// Function Name 	: IniDgy
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
static int IniDgy( void )
{
 #ifdef USE_INVENSENSE
	uint8_t	UcGrini ;
 #endif
	
	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
 #ifdef USE_3WIRE_DGYRO
	RegWriteA( SPIM 	, 0x00 );							// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #else
	RegWriteA( SPIM 	, 0x01 );							// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #endif
															//				DGSPI4	0: 3-wire SPI, 1: 4-wire SPI

	/*Set to Command Mode*/
	RegWriteA( GRSEL	, 0x01 );							// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA( GRINI	, 0x80 );							// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

 #ifdef USE_INVENSENSE

	RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	
	RegWriteA( GRADR0,	0x6A ) ;					// 0x0283	Set I2C_DIS
	RegWriteA( GSETDT,	0x10 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	CHECK_RETURN(AccWit( 0x10 ) );								/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x1B ) ;					// 0x0283	Set GYRO_CONFIG
	RegWriteA( GSETDT,	( FS_SEL << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	CHECK_RETURN(AccWit( 0x10 ) );								/* Digital Gyro busy wait 				*/

	RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

 #endif

 #ifdef	USE_STMICRO_L2G2IS
  	RegWriteA( LSBF	, 0x03 );						// 0x028D
 	
	/*Set Digital Gyro Settings*/
	RegWriteA( GRADR0	, 0x0B );					// 0x0283
  #ifdef USE_3WIRE_DGYRO
	RegWriteA( GSETDT	, 0x1B );					// 0x028A	Enable = 3-wire SPI, ODU, PW[Normal mode]
  #else
	RegWriteA( GSETDT	, 0x0B );					// 0x028A	Enable = 4-wire SPI, ODU, PW[Normal mode]
  #endif
	RegWriteA( GRACC	, 0x10 );					// 0x0282	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	CHECK_RETURN(AccWit( 0x10 ) );								/* Digital Gyro busy wait 				*/

	/*Set Digital Gyro Settings*/
	RegWriteA( GRADR0	, 0x0C );					// 0x0283
	RegWriteA( GSETDT	, 0x80 );					// 0x028A	LPF_O = 1st order
	RegWriteA( GRACC	, 0x10 );					// 0x0282	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	CHECK_RETURN(AccWit( 0x10 ) );								/* Digital Gyro busy wait 				*/
	
	/*Set Digital Gyro Settings*/
	RegWriteA( GRADR0	, 0x0D );					// 0x0283
	RegWriteA( GSETDT	, 0x00 );					// 0x028A	LPF=415Hz
	RegWriteA( GRACC	, 0x10 );					// 0x0282	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	CHECK_RETURN(AccWit( 0x10 ) );								/* Digital Gyro busy wait 				*/

	/*Set Digital Gyro Settings*/
	RegWriteA( GRADR0	, 0x1F );					// 0x0283
	RegWriteA( GSETDT	, 0x00 );					// 0x028A	Full-scale selection=100dps 	262LSb/dps
//	RegWriteA( GSETDT	, 0x08 );					// 0x028A	Full-scale selection=200dps	131LSb/dps
	RegWriteA( GRACC	, 0x10 );					// 0x0282	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	CHECK_RETURN(AccWit( 0x10 ) );								/* Digital Gyro busy wait 				*/
 #endif //USE_STMICRO_L2G2IS
 	
	RegWriteA( RDSEL,	0x7C ) ;				// 0x028B	RDSEL(Data1 and 2 for continuos mode)
	
	CHECK_RETURN(GyOutSignal() );

    return 0;
}

//********************************************************************************
// Function Name 	: IniMon
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
static int IniMon( void )
{
#ifndef	MONITOR_OFF	
	RegWriteA( PWMMONA, 0x00 ) ;				// 0x0030	0:off

	RegWriteA( MONSELA, 0x5C ) ;				// 0x0270	DLYMON1
	RegWriteA( MONSELB, 0x5D ) ;				// 0x0271	DLYMON2
	RegWriteA( MONSELC, 0x00 ) ;				// 0x0272	
	RegWriteA( MONSELD, 0x00 ) ;				// 0x0273	
	// Monitor Circuit
	RegWriteA( WC_PINMON1,	0x00 ) ;			// 0x01C0		Filter Monitor
	RegWriteA( WC_PINMON2,	0x00 ) ;			// 0x01C1		
	RegWriteA( WC_PINMON3,	0x00 ) ;			// 0x01C2		
	RegWriteA( WC_PINMON4,	0x00 ) ;			// 0x01C3		
	/* Delay Monitor */
	RegWriteA( WC_DLYMON11,	0x04 ) ;			// 0x01C5		DlyMonAdd1[10:8]
	RegWriteA( WC_DLYMON10,	0x40 ) ;			// 0x01C4		DlyMonAdd1[ 7:0]
	RegWriteA( WC_DLYMON21,	0x04 ) ;			// 0x01C7		DlyMonAdd2[10:8]
	RegWriteA( WC_DLYMON20,	0xC0 ) ;			// 0x01C6		DlyMonAdd2[ 7:0]
	RegWriteA( WC_DLYMON31,	0x00 ) ;			// 0x01C9		DlyMonAdd3[10:8]
	RegWriteA( WC_DLYMON30,	0x00 ) ;			// 0x01C8		DlyMonAdd3[ 7:0]
	RegWriteA( WC_DLYMON41,	0x00 ) ;			// 0x01CB		DlyMonAdd4[10:8]
	RegWriteA( WC_DLYMON40,	0x00 ) ;			// 0x01CA		DlyMonAdd4[ 7:0]
	/* Monitor */
	RegWriteA( PWMMONA, 0x80 ) ;				// 0x0030	1:on 
#endif

    return 0;
}

//********************************************************************************
// Function Name 	: ClrGyr
// Retun Value		: NON
// Argment Value	: UsClrFil - Select filter to clear.  If 0x0000, clears entire filter
//					  UcClrMod - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
static int ClrGyr( uint16_t UsClrFil , uint8_t UcClrMod )
{
	uint8_t	UcRamClr;
	uint8_t	count = 0; 

	/*Select Filter to clear*/
	RegWriteA( WC_RAMDLYMOD1,	(uint8_t)(UsClrFil >> 8) ) ;		// 0x018F		FRAM Initialize Hbyte
	RegWriteA( WC_RAMDLYMOD0,	(uint8_t)UsClrFil ) ;				// 0x018E		FRAM Initialize Lbyte

	/*Enable Clear*/
	RegWriteA( WC_RAMINITON	, UcClrMod ) ;	// 0x0102	[ - | - | - | - ][ - | - | ??Clr | õ{?Clr ]
	
	/*Check RAM Clear complete*/
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
// Function Name 	: RamAccFixMod
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Ram Access Fix Mode setting function
// History			: First edition 						2013.05.21 Y.Shigeoka
//********************************************************************************
static int RamAccFixMod( uint8_t UcAccMod )
{
	switch ( UcAccMod ) {
		case OFF :
			RegWriteA( WC_RAMACCMOD,	0x00 ) ;		// 0x018C		GRAM Access Float32bit
			break ;
		case ON :
			RegWriteA( WC_RAMACCMOD,	0x31 ) ;		// 0x018C		GRAM Access Fix32bit
			break ;
	}

    return 0;
}

//********************************************************************************
// Function Name 	: DrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
static int DrvSw( uint8_t UcDrvSw )
{
	if( UcDrvSw == ON )
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
		} else {
			RegWriteA( DRVFC	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
		}
	}
	else
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0x30 );				// 0x0001	Drvier Block Ena=0
		} else {
			RegWriteA( DRVFC	, 0x00 );				// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
		}
	}

    return 0;
}


//********************************************************************************
// Function Name 	: IniSrv
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
static int IniSrv( void )
{
	uint8_t	UcStbb0 ;

	UcPwmMod = INIT_PWMMODE ;					// Driver output mode

	RegWriteA( WC_EQON,		0x00 ) ;				// 0x0101		Filter Calcu
	RegWriteA( WC_RAMINITON,0x00 ) ;				// 0x0102		
	CHECK_RETURN(ClrGyr( 0x0000 , CLR_ALL_RAM ));					// All Clear
	
	RegWriteA( WH_EQSWX,	0x02 ) ;				// 0x0170		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	RegWriteA( WH_EQSWY,	0x02 ) ;				// 0x0171		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]

	/* Ram Access */	
	CHECK_RETURN(RamAccFixMod( OFF ) );							// 32bit Float mode
#ifndef	MONITOR_OFF	
	/* Monitor Gain */
	RamWrite32A( dm1g, 0x3F800000 ) ;				// 0x1239
	RamWrite32A( dm2g, 0x3F800000 ) ;				// 0x123A
	RamWrite32A( dm3g, 0x3F800000 ) ;				// 0x123B
	RamWrite32A( dm4g, 0x3F800000 ) ;				// 0x123C
#endif	
	/* Hall output limitter */
	RamWrite32A( sxlmta1,   0x3F800000 ) ;			// 0x10E6		Hall X output Limit
	RamWrite32A( sylmta1,   0x3F800000 ) ;			// 0x11E6		Hall Y output Limit
	RamWrite32A( sxlmta2, 	X_CURRENT_LMT );		// 0x10E7 		Hall X output Limit
	RamWrite32A( sylmta2, 	Y_CURRENT_LMT );      	// 0x11E7		Hall Y output Limit

	/* Emargency Stop */
	RegWriteA( WH_EMGSTPON,	0x00 ) ;				// 0x0178		Emargency Stop OFF
	RegWriteA( WH_EMGSTPTMR,0xFF ) ;				// 0x017A		255*(16/23.4375kHz)=174ms
	
	RamWrite32A( sxemglev,   0x3F800000 ) ;			// 0x10EC		Hall X Emargency threshold
	RamWrite32A( syemglev,   0x3F800000 ) ;			// 0x11EC		Hall Y Emargency threshold
	
	/* Hall Servo smoothing */
	RegWriteA( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF
	RegWriteA( WH_SMTSRVSMP,0x06 ) ;				// 0x017D		2.7ms=2^06/23.4375kHz
	RegWriteA( WH_SMTTMR,	0x0F ) ;				// 0x017E		10ms=(15+1)*16/23.4375kHz
	
	RamWrite32A( sxsmtav,   0xBC800000 ) ;			// 0x10ED		1/64 X smoothing ave coefficient
	RamWrite32A( sysmtav,   0xBC800000 ) ;			// 0x11ED		1/64 Y smoothing ave coefficient
	RamWrite32A( sxsmtstp,  0x3AE90466 ) ;			// 0x10EE		0.001778 X smoothing offset
	RamWrite32A( sysmtstp,  0x3AE90466 ) ;			// 0x11EE		0.001778 Y smoothing offset
	
	/* High-dimensional correction  */
	RegWriteA( WH_HOFCON,	0x11 ) ;				// 0x0174		OUT 3x3
	
	/* Front */
	RamWrite32A( sxiexp3,   A3_IEXP3 ) ;			// 0x10BA		
	RamWrite32A( sxiexp2,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A( sxiexp1,   A1_IEXP1 ) ;			// 0x10BC		
	RamWrite32A( sxiexp0,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A( sxiexp,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A( syiexp3,   A3_IEXP3 ) ;			// 0x11BA		
	RamWrite32A( syiexp2,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A( syiexp1,   A1_IEXP1 ) ;			// 0x11BC		
	RamWrite32A( syiexp0,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A( syiexp,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A( sxoexp3,   A3_IEXP3 ) ;			// 0x10FA		
	RamWrite32A( sxoexp2,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A( sxoexp1,   A1_IEXP1 ) ;			// 0x10FC		
	RamWrite32A( sxoexp0,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A( sxoexp,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A( syoexp3,   A3_IEXP3 ) ;			// 0x11FA		
	RamWrite32A( syoexp2,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A( syoexp1,   A1_IEXP1 ) ;			// 0x11FC		
	RamWrite32A( syoexp0,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A( syoexp,    0x3F800000 ) ;			// 0x11FE		
	
	/* Sine wave */
#ifdef	DEF_SET
	RegWriteA( WC_SINON,	0x00 ) ;				// 0x0180		Sin Wave off
	RegWriteA( WC_SINFRQ0,	0x00 ) ;				// 0x0181		
	RegWriteA( WC_SINFRQ1,	0x60 ) ;				// 0x0182		
	RegWriteA( WC_SINPHSX,	0x00 ) ;				// 0x0183		
	RegWriteA( WC_SINPHSY,	0x00 ) ;				// 0x0184		
	
	/* AD over sampling */
	RegWriteA( WC_ADMODE,	0x06 ) ;				// 0x0188		AD Over Sampling
	
	/* Measure mode */
	RegWriteA( WC_MESMODE,		0x00 ) ;				// 0x0190		Measurement Mode
	RegWriteA( WC_MESSINMODE,	0x00 ) ;				// 0x0191		
	RegWriteA( WC_MESLOOP0,		0x08 ) ;				// 0x0192		
	RegWriteA( WC_MESLOOP1,		0x02 ) ;				// 0x0193		
	RegWriteA( WC_MES1ADD0,		0x00 ) ;				// 0x0194		
	RegWriteA( WC_MES1ADD1,		0x00 ) ;				// 0x0195		
	RegWriteA( WC_MES2ADD0,		0x00 ) ;				// 0x0196		
	RegWriteA( WC_MES2ADD1,		0x00 ) ;				// 0x0197		
	RegWriteA( WC_MESABS,		0x00 ) ;				// 0x0198		
	RegWriteA( WC_MESWAIT,		0x00 ) ;				// 0x0199		
	
	/* auto measure */
	RegWriteA( WC_AMJMODE,		0x00 ) ;				// 0x01A0		Automatic measurement mode
	
	RegWriteA( WC_AMJLOOP0,		0x08 ) ;				// 0x01A2		Self-Aadjustment
	RegWriteA( WC_AMJLOOP1,		0x02 ) ;				// 0x01A3		
	RegWriteA( WC_AMJIDL0,		0x02 ) ;				// 0x01A4		
	RegWriteA( WC_AMJIDL1,		0x00 ) ;				// 0x01A5		
	RegWriteA( WC_AMJ1ADD0,		0x00 ) ;				// 0x01A6		
	RegWriteA( WC_AMJ1ADD1,		0x00 ) ;				// 0x01A7		
	RegWriteA( WC_AMJ2ADD0,		0x00 ) ;				// 0x01A8		
	RegWriteA( WC_AMJ2ADD1,		0x00 ) ;				// 0x01A9		
#endif
#ifdef	CATCHMODE	
	RegWriteA( WC_DPI1ADD0,		0x2F ) ;				// 0x01B0		Data Pass
	RegWriteA( WC_DPI1ADD1,		0x00 ) ;				// 0x01B1		
	RegWriteA( WC_DPI2ADD0,		0xAF ) ;				// 0x01B2		
	RegWriteA( WC_DPI2ADD1,		0x00 ) ;				// 0x01B3		
	RegWriteA( WC_DPI3ADD0,		0x38 ) ;				// 0x01B4		
	RegWriteA( WC_DPI3ADD1,		0x00 ) ;				// 0x01B5		
	RegWriteA( WC_DPI4ADD0,		0xB8 ) ;				// 0x01B6		
	RegWriteA( WC_DPI4ADD1,		0x00 ) ;				// 0x01B7		
	RegWriteA( WC_DPO1ADD0,		0x06 ) ;				// 0x01B8		Data Pass
	RegWriteA( WC_DPO1ADD1,		0x00 ) ;				// 0x01B9		
	RegWriteA( WC_DPO2ADD0,		0x86 ) ;				// 0x01BA		
	RegWriteA( WC_DPO2ADD1,		0x00 ) ;				// 0x01BB		
	RegWriteA( WC_DPO3ADD0,		0x3A ) ;				// 0x01BC		
	RegWriteA( WC_DPO3ADD1,		0x00 ) ;				// 0x01BD		
	RegWriteA( WC_DPO4ADD0,		0xBA ) ;				// 0x01BE		
	RegWriteA( WC_DPO4ADD1,		0x00 ) ;				// 0x01BF		
	RegWriteA( WC_DPON,			0x0F ) ;				// 0x0105		Data pass ON
#else	
 #ifdef	DEF_SET  
	/* Data Pass */
	RegWriteA( WC_DPI1ADD0,		0x00 ) ;				// 0x01B0		Data Pass
	RegWriteA( WC_DPI1ADD1,		0x00 ) ;				// 0x01B1		
	RegWriteA( WC_DPI2ADD0,		0x00 ) ;				// 0x01B2		
	RegWriteA( WC_DPI2ADD1,		0x00 ) ;				// 0x01B3		
	RegWriteA( WC_DPI3ADD0,		0x00 ) ;				// 0x01B4		
	RegWriteA( WC_DPI3ADD1,		0x00 ) ;				// 0x01B5		
	RegWriteA( WC_DPI4ADD0,		0x00 ) ;				// 0x01B6		
	RegWriteA( WC_DPI4ADD1,		0x00 ) ;				// 0x01B7		
	RegWriteA( WC_DPO1ADD0,		0x00 ) ;				// 0x01B8		Data Pass
	RegWriteA( WC_DPO1ADD1,		0x00 ) ;				// 0x01B9		
	RegWriteA( WC_DPO2ADD0,		0x00 ) ;				// 0x01BA		
	RegWriteA( WC_DPO2ADD1,		0x00 ) ;				// 0x01BB		
	RegWriteA( WC_DPO3ADD0,		0x00 ) ;				// 0x01BC		
	RegWriteA( WC_DPO3ADD1,		0x00 ) ;				// 0x01BD		
	RegWriteA( WC_DPO4ADD0,		0x00 ) ;				// 0x01BE		
	RegWriteA( WC_DPO4ADD1,		0x00 ) ;				// 0x01BF		
	RegWriteA( WC_DPON,			0x00 ) ;				// 0x0105		Data pass OFF
 #endif
#endif	
	/* Interrupt Flag */
	RegWriteA( WC_INTMSK,	0xFF ) ;				// 0x01CE		All Mask
	
	// PWM Signal Generate
	CHECK_RETURN(DrvSw( OFF ) );									/* 0x0070	Drvier Block Ena=0 */
	RegWriteA( DRVFC2	, 0x90 );					// 0x0002	Slope 3, Dead Time = 30 ns
	RegWriteA( DRVSELX	, 0xFF );					// 0x0003	PWM X drv max current  DRVSELX[7:0]
	RegWriteA( DRVSELY	, 0xFF );					// 0x0004	PWM Y drv max current  DRVSELY[7:0]

#ifdef	PWM_BREAK
	if( UcCvrCod == CVER122 ) {
		RegWriteA( PWMFC,   0x2D ) ;					// 0x0011	VREF, PWMCLK/256, MODE0B, 12Bit Accuracy
	} else {
		RegWriteA( PWMFC,   0x3D ) ;					// 0x0011	VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
	}
#else
	if( UcCvrCod == CVER122 ) {
		RegWriteA( PWMFC,   0x29 ) ;					// 0x0011	VREF, PWMCLK/256, MODE0S, 12Bit Accuracy  
	} else {
		RegWriteA( PWMFC,   0x39 ) ;					// 0x0011	VREF, PWMCLK/128, MODE0S, 12Bit Accuracy  
	}
#endif

#ifdef	DEF_SET
	RegWriteA( PWMA,    0x00 ) ;					// 0x0010	PWM X/Y standby
#endif	
	RegWriteA( PWMDLYX,  0x04 ) ;					// 0x0012	X Phase Delay Setting
	RegWriteA( PWMDLYY,  0x04 ) ;					// 0x0013	Y Phase Delay Setting
	
#ifdef	DEF_SET
	RegWriteA( DRVCH1SEL,	0x00 ) ;				// 0x0005	OUT1/OUT2	X axis
	RegWriteA( DRVCH2SEL,	0x00 ) ;				// 0x0006	OUT3/OUT4	Y axis
	
	RegWriteA( PWMDLYTIMX,	0x00 ) ;				// 0x0014		PWM Timing
	RegWriteA( PWMDLYTIMY,	0x00 ) ;				// 0x0015		PWM Timing
	
	if( UcCvrCod == CVER122 ) {
		RegWriteA( PWMPERIODY,	0x00 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA( PWMPERIODY2,	0x00 ) ;				// 0x001B		PWM Carrier Freq
	} else {
		RegWriteA( PWMPERIODX,	0x00 ) ;				// 0x0018		PWM Carrier Freq
		RegWriteA( PWMPERIODX2,	0x00 ) ;				// 0x0019		PWM Carrier Freq
		RegWriteA( PWMPERIODY,	0x00 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA( PWMPERIODY2,	0x00 ) ;				// 0x001B		PWM Carrier Freq
	}
#endif	
	/* Linear PWM circuit setting */
	RegWriteA( CVA		, 0xC0 );			// 0x0020	Linear PWM mode enable

	if( UcCvrCod == CVER122 ) {
		RegWriteA( CVFC 	, 0x22 );			// 0x0021	
	}
	
#ifdef	PWM_BREAK 
	RegWriteA( CVFC2 	, 0x80 );			// 0x0022
#else
	RegWriteA( CVFC2 	, 0x00 );			// 0x0022
#endif
	if( UcCvrCod == CVER122 ) {
		RegWriteA( CVSMTHX	, 0x00 );			// 0x0023	smooth off
		RegWriteA( CVSMTHY	, 0x00 );			// 0x0024	smooth off
	}

	RegReadA( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x80 ;
	RegWriteA( STBB0, UcStbb0 ) ;			// 0x0250	OIS standby

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

			RamWrite32A( gylmt4HS0, GYRLMT4_S1 ) ;		//0x112B	Yé≤Limiter4 HighËáíl0
			RamWrite32A( gxlmt4HS0, GYRLMT4_S1 ) ;		//0x102B	Xé≤Limiter4 HighËáíl0
			
			RamWrite32A( gxlmt4HS1, GYRLMT4_S2 ) ;		//0x102C	Xé≤Limiter4 HighËáíl1
			RamWrite32A( gylmt4HS1, GYRLMT4_S2 ) ;		//0x112C	Yé≤Limiter4 HighËáíl1
		
			RamWrite32A( Sttx12aH, 	GYRA12_HGH );		// 0x105F
			RamWrite32A( Stty12aH, 	GYRA12_HGH );		// 0x115F
			break ;
			
#ifdef	CATCHMODE		
		case ON :
			RamWrite32A( gxlmt3HS0, GYRLMT3_S1_W ) ;		// 0x1029
			RamWrite32A( gylmt3HS0, GYRLMT3_S1_W ) ;		// 0x1129
			
			RamWrite32A( gxlmt3HS1, GYRLMT3_S2_W ) ;		// 0x102A
			RamWrite32A( gylmt3HS1, GYRLMT3_S2_W ) ;		// 0x112A

			RamWrite32A( gylmt4HS0, GYRLMT4_S1_W ) ;		//0x112B	Yé≤Limiter4 HighËáíl0
			RamWrite32A( gxlmt4HS0, GYRLMT4_S1_W ) ;		//0x102B	Xé≤Limiter4 HighËáíl0
			
			RamWrite32A( gxlmt4HS1, GYRLMT4_S2_W ) ;		//0x102C	Xé≤Limiter4 HighËáíl1
			RamWrite32A( gylmt4HS1, GYRLMT4_S2_W ) ;		//0x112C	Yé≤Limiter4 HighËáíl1
		
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
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x54 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158

			break ;
		case ON :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158
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
	RegWriteA( WG_ADJGANGO, 		0x00 );					// 0x0108	manual off

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
//TRACE(" AGC = Off \n") ;
		RamWrite32A( GANADR			 , XMAXGAIN ) ;			// Gain Through
		RamWrite32A( GANADR | 0x0100 , YMAXGAIN ) ;			// Gain Through
	}
	else
	{
		RegWriteA( WG_ADJGANGXATO, 	0xA3 );					// 0x0129	X exe on
		RegWriteA( WG_ADJGANGYATO, 	0xA3 );					// 0x012A	Y exe on
//TRACE(" AGC = ON \n") ;
	}

    return 0;
}

//********************************************************************************
// Function Name 	: IniGyr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
static int IniGyr( void )
{
#ifdef	CATCHMODE
		/* CPU control */
	RegWriteA( WC_CPUOPEON , 0x11 );	// 0x0103	 	CPU control
	RegWriteA( WC_CPUOPE1ADD , 0x06 );	// 0x018A	 	
	RegWriteA( WC_CPUOPE2ADD , 0x3A );	// 0x018B	 	
	RegWriteA( WG_EQSW	, 0x43 );		// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
#else	
	/*Gyro Filter Setting*/
	RegWriteA( WG_EQSW	, 0x03 );		// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
#endif	
	
	/*Gyro Filter Down Sampling*/
	RegWriteA( WG_SHTON	, 0x10 );		// 0x0107		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
										//				CmShtOpe[1:0] 00: ???ôEÂhOFF, 01: ???ôEÂhON, 1x:????
										
#ifdef	DEF_SET
	RegWriteA( WG_SHTDLYTMR , 0x00 );	// 0x0117	 	Shutter Delay
	RegWriteA( WG_GADSMP, 	  0x00 );	// 0x011C		Sampling timing
	RegWriteA( WG_HCHR, 	  0x00 );	// 0x011B		H-filter limitter control not USE
	RegWriteA( WG_LMT3MOD , 0x00 );		// 0x0118 	[ - | - | - | - ][ - | - | - | CmLmt3Mod ]
										//				CmLmt3Mod	0: ??ôP??ôEÂh?õ∏, 1: ????ôP??ôEÂh?õ∏
	RegWriteA( WG_VREFADD , 0x12 );		// 0x0119	 	??ôEÂh????˙x??RAM?åX???öÿ?6ôJ???(default 0x12 = GXH1Z2/GYH1Z2)
#endif
	RegWriteA( WG_SHTMOD , 0x06 );		// 0x0116	 	Shutter Hold mode

	// Limiter
	RamWrite32A( gxlmt1H, GYRLMT1H ) ;		// 0x1028
	RamWrite32A( gylmt1H, GYRLMT1H ) ;		// 0x1128

	RamWrite32A( Sttx12aM, 	GYRA12_MID );	// 0x104F
	RamWrite32A( Sttx12bM, 	GYRB12_MID );	// 0x106F
	RamWrite32A( Sttx12bH, 	GYRB12_HGH );	// 0x107F
	RamWrite32A( Sttx34aM, 	GYRA34_MID );	// 0x108F
	RamWrite32A( Sttx34aH, 	GYRA34_HGH );	// 0x109F
	RamWrite32A( Sttx34bM, 	GYRB34_MID );	// 0x10AF
	RamWrite32A( Sttx34bH, 	GYRB34_HGH );	// 0x10BF
	RamWrite32A( Stty12aM, 	GYRA12_MID );	// 0x114F
	RamWrite32A( Stty12bM, 	GYRB12_MID );	// 0x116F
	RamWrite32A( Stty12bH, 	GYRB12_HGH );	// 0x117F
	RamWrite32A( Stty34aM, 	GYRA34_MID );	// 0x118F
	RamWrite32A( Stty34aH, 	GYRA34_HGH );	// 0x119F
	RamWrite32A( Stty34bM, 	GYRB34_MID );	// 0x11AF
	RamWrite32A( Stty34bH, 	GYRB34_HGH );	// 0x11BF

#ifdef	CATCHMODE
#ifdef	CORRECT_1DEG
	CHECK_RETURN(SelectPtRange( ON ) );
#else
	CHECK_RETURN(SelectPtRange( OFF ) );
#endif
	/* Pan/Tilt parameter */
	RegWriteA( WG_PANADDA, 		0x12 );		// 0x0130	GXH2Z2/GYH2Z2 Select
	RegWriteA( WG_PANADDB, 		0x3B );		// 0x0131	GXK2Z2/GYK2Z2 Select
	
#else
	CHECK_RETURN(SelectPtRange( OFF ) );
	
	/* Pan/Tilt parameter */
	RegWriteA( WG_PANADDA, 		0x12 );		// 0x0130	GXH1Z2/GYH1Z2 Select
	RegWriteA( WG_PANADDB, 		0x09 );		// 0x0131	GXIZ/GYIZ Select
	
#endif /* CATCHMODE */	

	 //Threshold
	RamWrite32A( SttxHis, 	0x00000000 );			// 0x1226
	RamWrite32A( SttxaL, 	0x00000000 );			// 0x109D
	RamWrite32A( SttxbL, 	0x00000000 );			// 0x109E
	RamWrite32A( SttyaL, 	0x00000000 );			// 0x119D
	RamWrite32A( SttybL, 	0x00000000 );			// 0x119E
	// Pan level
	RegWriteA( WG_PANLEVABS, 		0x00 );		// 0x0133
	
	// Average parameter are set IniAdj

#ifdef	CATCHMODE
	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA( WG_PANSTT21JUG0, 	0x07 );		// 0x0140
	RegWriteA( WG_PANSTT21JUG1, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA( WG_PANSTT31JUG0, 	0x00 );		// 0x0142
	RegWriteA( WG_PANSTT31JUG1, 	0x00 );		// 0x0143
	// State 4 -> 1
	RegWriteA( WG_PANSTT41JUG0, 	0x00 );		// 0x0144
	RegWriteA( WG_PANSTT41JUG1, 	0x00 );		// 0x0145
	// State 1 -> 2
	RegWriteA( WG_PANSTT12JUG0, 	0x00 );		// 0x0146
	RegWriteA( WG_PANSTT12JUG1, 	0x07 );		// 0x0147
	// State 1 -> 3
	RegWriteA( WG_PANSTT13JUG0, 	0x00 );		// 0x0148
	RegWriteA( WG_PANSTT13JUG1, 	0x00 );		// 0x0149
	// State 2 -> 3
	RegWriteA( WG_PANSTT23JUG0, 	0x00 );		// 0x014A
	RegWriteA( WG_PANSTT23JUG1, 	0x00 );		// 0x014B
	// State 4 -> 3
	RegWriteA( WG_PANSTT43JUG0, 	0x00 );		// 0x014C
	RegWriteA( WG_PANSTT43JUG1, 	0x00 );		// 0x014D
	// State 3 -> 4
	RegWriteA( WG_PANSTT34JUG0, 	0x00 );		// 0x014E
	RegWriteA( WG_PANSTT34JUG1, 	0x00 );		// 0x014F
	// State 2 -> 4
	RegWriteA( WG_PANSTT24JUG0, 	0x00 );		// 0x0150
	RegWriteA( WG_PANSTT24JUG1, 	0x00 );		// 0x0151
	// State 4 -> 2
	RegWriteA( WG_PANSTT42JUG0, 	0x00 );		// 0x0152
	RegWriteA( WG_PANSTT42JUG1, 	0x00 );		// 0x0153

	// State Timer
	RegWriteA( WG_PANSTT1LEVTMR, 	0x00 );		// 0x015B
	RegWriteA( WG_PANSTT2LEVTMR, 	0x00 );		// 0x015C
	RegWriteA( WG_PANSTT3LEVTMR, 	0x00 );		// 0x015D
	RegWriteA( WG_PANSTT4LEVTMR, 	0x00 );		// 0x015E
	
	// Control filter
	RegWriteA( WG_PANTRSON0, 		0x1B );		// 0x0132	USE iSTP
	
	// State Setting
	CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA( WG_PANSTTSETILHLD,	0x00 );		// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA( WG_PANSTT2TMR0,	0x01 );		// 0x013C
	RegWriteA( WG_PANSTT2TMR1,	0x00 );		// 0x013D	
	RegWriteA( WG_PANSTT4TMR0,	0x01 );		// 0x013E
	RegWriteA( WG_PANSTT4TMR1,	0x00 );		// 0x013F	
	
	RegWriteA( WG_PANSTTXXXTH,	0x00 );		// 0x015A

	CHECK_RETURN(AutoGainContIni() );
	/* exe function */
	CHECK_RETURN(AutoGainControlSw( ON ) );							/* Auto Gain Control Mode ON  */

#else
	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA( WG_PANSTT21JUG0, 	0x00 );		// 0x0140
	RegWriteA( WG_PANSTT21JUG1, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA( WG_PANSTT31JUG0, 	0x00 );		// 0x0142
	RegWriteA( WG_PANSTT31JUG1, 	0x00 );		// 0x0143
	// State 4 -> 1
	RegWriteA( WG_PANSTT41JUG0, 	0x01 );		// 0x0144
	RegWriteA( WG_PANSTT41JUG1, 	0x00 );		// 0x0145
	// State 1 -> 2
	RegWriteA( WG_PANSTT12JUG0, 	0x00 );		// 0x0146
	RegWriteA( WG_PANSTT12JUG1, 	0x07 );		// 0x0147
	// State 1 -> 3
	RegWriteA( WG_PANSTT13JUG0, 	0x00 );		// 0x0148
	RegWriteA( WG_PANSTT13JUG1, 	0x00 );		// 0x0149
	// State 2 -> 3
	RegWriteA( WG_PANSTT23JUG0, 	0x11 );		// 0x014A
	RegWriteA( WG_PANSTT23JUG1, 	0x00 );		// 0x014B
	// State 4 -> 3
	RegWriteA( WG_PANSTT43JUG0, 	0x00 );		// 0x014C
	RegWriteA( WG_PANSTT43JUG1, 	0x00 );		// 0x014D
	// State 3 -> 4
	RegWriteA( WG_PANSTT34JUG0, 	0x01 );		// 0x014E
	RegWriteA( WG_PANSTT34JUG1, 	0x00 );		// 0x014F
	// State 2 -> 4
	RegWriteA( WG_PANSTT24JUG0, 	0x00 );		// 0x0150
	RegWriteA( WG_PANSTT24JUG1, 	0x00 );		// 0x0151
	// State 4 -> 2
	RegWriteA( WG_PANSTT42JUG0, 	0x44 );		// 0x0152
	RegWriteA( WG_PANSTT42JUG1, 	0x04 );		// 0x0153

	// State Timer
	RegWriteA( WG_PANSTT1LEVTMR, 	0x00 );		// 0x015B
	RegWriteA( WG_PANSTT2LEVTMR, 	0x00 );		// 0x015C
	RegWriteA( WG_PANSTT3LEVTMR, 	0x00 );		// 0x015D
	RegWriteA( WG_PANSTT4LEVTMR, 	0x03 );		// 0x015E
	
	// Control filter
	RegWriteA( WG_PANTRSON0, 		0x11 );		// 0x0132	USE I12/iSTP/Gain-Filter
	
	// State Setting
	CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA( WG_PANSTTSETILHLD,	0x00 );		// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA( WG_PANSTT2TMR0,	0x01 );		// 0x013C
	RegWriteA( WG_PANSTT2TMR1,	0x00 );		// 0x013D	
	RegWriteA( WG_PANSTT4TMR0,	0x02 );		// 0x013E
	RegWriteA( WG_PANSTT4TMR1,	0x07 );		// 0x013F	
	
	RegWriteA( WG_PANSTTXXXTH,	0x00 );		// 0x015A

	CHECK_RETURN(AutoGainContIni() );  
	/* exe function */
	CHECK_RETURN(AutoGainControlSw( ON ) );							/* Auto Gain Control Mode OFF */
#endif	/* CATCHMODE */	
    return 0;
}

//********************************************************************************
// Function Name 	: IniFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
static int IniFil( void )
{
#ifdef INIT_FAST
	unsigned char	UcAryId ;
	unsigned short	UsDatId, UsDatNum ;

	RegWriteA( WC_RAMACCXY, 0x01 ) ;			// 0x018D	Filter copy on
	// Filter Registor Parameter Setting
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( CsFilRegBurst[ UcAryId ] != 0xFF )
	{
		UsDatNum	= CsFilRegBurst[ UcAryId ];
		CntWrtRag( ( unsigned char * )&CsFilRegDat[ UsDatId ], UsDatNum ) ;
		UcAryId++ ;
		UsDatId	+= UsDatNum ;
	}
	// Filter X-axis Ram Parameter Setting	
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( CsFilRamBurst[ UcAryId ] != 0xFF )
	{
		UsDatNum	= CsFilRamBurst[ UcAryId ];
		CntWrtRam32( ( unsigned char * )&CsFilRamDat[ UsDatId ], UsDatNum ) ;
		UsDatId	+= UsDatNum ;
		UcAryId++ ;
	}
	// Filter Y-axis Ram Parameter Setting	
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( CsFilRamY[ UcAryId ] != 0xFF )
	{
		UsDatNum	= CsFilRamY[ UcAryId ];
		CntWrtRam32( ( unsigned char * )&CsFilRamYDat[ UsDatId ], UsDatNum ) ;
		UsDatId	+= UsDatNum ;
		UcAryId++ ;
	}	
#else
 	unsigned short	UsAryId ;
	// Filter Registor Parameter Setting
	UsAryId	= 0 ;
	while( CsFilReg[ UsAryId ].UsRegAdd != 0xFFFF )
	{
		RegWriteA( CsFilReg[ UsAryId ].UsRegAdd, CsFilReg[ UsAryId ].UcRegDat ) ;
		UsAryId++ ;
	}
	// Filter Ram Parameter Setting
	UsAryId	= 0 ;
	while( CsFilRam[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWrite32A( CsFilRam[ UsAryId ].UsRamAdd, CsFilRam[ UsAryId ].UlRamDat ) ;
		UsAryId++ ;
	}
#endif	

    return 0;
}

//********************************************************************************
// Function Name 	: SelectIstpMod
// Retun Value		: NON
// Argment Value	: OFF:Narrow  ON:Wide
// Explanation		: Pan/Tilt parameter Range function
// History			: First edition 						2014.04.08 Y.Shigeoka
//********************************************************************************
static int SelectIstpMod( uint8_t UcSelRange )
{
	switch ( UcSelRange ) {
		case OFF :
			RamWrite32A( gxistp_1, GYRISTP ) ;		// 0x1083
			RamWrite32A( gyistp_1, GYRISTP ) ;		// 0x1183
			break ;
#ifdef	CATCHMODE				
		case ON :
			RamWrite32A( gxistp_1, GYRISTP_W ) ;	// 0x1083
			RamWrite32A( gyistp_1, GYRISTP_W ) ;	// 0x1183
			break ;
#endif // CATCHMODE  
	}

    return 0;
}

//********************************************************************************
// Function Name 	: IniPtAve
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan/Tilt Average parameter setting function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
static int IniPtAve( void )
{
#ifdef	CATCHMODE
	RegWriteA( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA( WG_PANSTT2DWNSMP0, 0x00 );		// 0x0136
	RegWriteA( WG_PANSTT2DWNSMP1, 0x00 );		// 0x0137
	RegWriteA( WG_PANSTT3DWNSMP0, 0x00 );		// 0x0138
	RegWriteA( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B

	RamWrite32A( st1mean, 0x3f800000 );		// 0x1235
	RamWrite32A( st2mean, 0x3f800000 );		// 0x1236
	RamWrite32A( st3mean, 0x3f800000 );		// 0x1237
	RamWrite32A( st4mean, 0x3f800000 );		// 0x1238
#else
	RegWriteA( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA( WG_PANSTT2DWNSMP0, 0x90 );		// 0x0136 400
	RegWriteA( WG_PANSTT2DWNSMP1, 0x01 );		// 0x0137
	RegWriteA( WG_PANSTT3DWNSMP0, 0x64 );		// 0x0138 100
	RegWriteA( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B

	RamWrite32A( st1mean, 0x3f800000 );		// 0x1235
	RamWrite32A( st2mean, 0x3B23D700 );		// 0x1236	1/400
	RamWrite32A( st3mean, 0x3C23D700 );		// 0x1237	1/100
	RamWrite32A( st4mean, 0x3f800000 );		// 0x1238
#endif	

    return 0;
}

//********************************************************************************
// Function Name 	: SetZsp
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Zoom Step parameter Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
static int SetZsp( uint8_t	UcZoomStepDat )
{
    uint32_t	UlGyrZmx, UlGyrZmy, UlGyrZrx, UlGyrZry ;


    /* Zoom Step */
    if(UcZoomStepDat > (ZOOMTBL - 1))
        UcZoomStepDat = (ZOOMTBL -1) ;										/* è„å¿ÇZOOMTBL-1Ç…ê›íËÇ∑ÇÈ */

    if( UcZoomStepDat == 0 )				/* initial setting	*/
    {
        UlGyrZmx	= ClGyxZom[ 0 ] ;		// Same Wide Coefficient
        UlGyrZmy	= ClGyyZom[ 0 ] ;		// Same Wide Coefficient
        /* Initial Rate value = 1 */
//TRACE("Initial ZoomX = %08xh", (unsigned int)UlGyrZmx ) ;
//TRACE("        ZoomY = %08xh\n", (unsigned int)UlGyrZmy ) ;
    }
    else
    {
        UlGyrZmx	= ClGyxZom[ UcZoomStepDat ] ;
        UlGyrZmy	= ClGyyZom[ UcZoomStepDat ] ;


//TRACE("   Step ZoomX = %08xh", (unsigned int)UlGyrZmx ) ;
//TRACE("        ZoomY = %08xh", (unsigned int)UlGyrZmy ) ;
//TRACE("        Step = %d\n", UcZoomStepDat ) ;
    }

    // Zoom Value Setting
    RamWrite32A( gxlens, UlGyrZmx ) ;		/* 0x1022 */
    RamWrite32A( gylens, UlGyrZmy ) ;		/* 0x1122 */

    RamRead32A( gxlens, &UlGyrZrx ) ;		/* 0x1022 */
    RamRead32A( gylens, &UlGyrZry ) ;		/* 0x1122 */

    // Zoom Value Setting Error Check
    if( UlGyrZmx != UlGyrZrx ) {
        RamWrite32A( gxlens, UlGyrZmx ) ;		/* 0x1022 */
    }

    if( UlGyrZmy != UlGyrZry ) {
        RamWrite32A( gylens, UlGyrZmy ) ;		/* 0x1122 */
    }

    return 0;
}

//********************************************************************************
// Function Name 	: SetPanTiltMode
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
static int SetPanTiltMode( uint8_t UcPnTmod )
{
    switch ( UcPnTmod ) {
        case OFF :
            RegWriteA( WG_PANON, 0x00 ) ;			// 0x0109	X,Y Pan/Tilt Function OFF
//TRACE(" PanTilt OFF\n");
            break ;
        case ON :
            RegWriteA( WG_PANON, 0x01 ) ;			// 0x0109	X,Y Pan/Tilt Function ON
//			RegWriteA( WG_PANON, 0x10 ) ;			// 0x0109	X,Y New Pan/Tilt Function ON
//TRACE(" PanTilt ON\n");
            break ;
    }

    return 0;
}

//********************************************************************************
// Function Name 	: SetH1cMod
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set H1C coefficient Level chang Function
// History			: First edition 						2013.04.18 Y.Shigeoka
//********************************************************************************
static int SetH1cMod( uint8_t	UcSetNum )
{

switch( UcSetNum ){

#ifdef	CATCHMODE
    case ( ACTMODE ):				// initial
        CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)

        /* enable setting */
        UcH1LvlMod = UcSetNum  ;

        // Limit value Value Setting
#ifdef	CORRECT_1DEG  
        RamWrite32A( gxlmt6L, MINLMT_W ) ;		/* 0x102D L-Limit */
        RamWrite32A( gxlmt6H, MAXLMT_W ) ;		/* 0x102E H-Limit */
        RamWrite32A( gylmt6L, MINLMT_W ) ;		/* 0x112D L-Limit */
        RamWrite32A( gylmt6H, MAXLMT_W ) ;		/* 0x112E H-Limit */
        RamWrite32A( gxmg,	CHGCOEF_W ) ;		/* 0x10AA Change coefficient gain */
        RamWrite32A( gymg,	CHGCOEF_W ) ;		/* 0x11AA Change coefficient gain */
#else
        RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
        RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
        RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
        RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
        RamWrite32A( gxmg,	CHGCOEF ) ;		/* 0x10AA Change coefficient gain */
        RamWrite32A( gymg,	CHGCOEF ) ;		/* 0x11AA Change coefficient gain */
#endif
        RamWrite32A( gxhc_tmp, 	DIFIL_S2) ;	/* 0x100E Base Coef */
        RamWrite32A( gyhc_tmp, 	DIFIL_S2) ;	/* 0x110E Base Coef */

        RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
        break ;

    case( S2MODE ):				// cancel lvl change mode
        RegWriteA( WG_HCHR, 0x10 ) ;			// 0x011B	GmHChrOn[1]=0 Sw OFF
        break ;

	case( MOVMODE ):			// Movie mode 
		CHECK_RETURN(IniPtMovMod( ON ) );						// Pan/Tilt setting (Movie)
		CHECK_RETURN(SelectPtRange( OFF ) );					// Range narrow
		CHECK_RETURN(SelectIstpMod( OFF ) );					// Range narrow
		
		RamWrite32A( gxlmt6L, MINLMT_MOV ) ;	/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT_MOV ) ;	/* 0x112D L-Limit */

		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, CHGCOEF_MOV ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF_MOV ) ;		/* 0x11AA Change coefficient gain */
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( MOVMODE_W ):			// Movie mode (wide)
		CHECK_RETURN(IniPtMovMod( ON ) );							// Pan/Tilt setting (Movie)
		CHECK_RETURN(SelectPtRange( ON ) );					// Range wide
		CHECK_RETURN(SelectIstpMod( ON ) );					// Range wide
		
		RamWrite32A( gxlmt6L, MINLMT_MOV_W ) ;	/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT_MOV_W ) ;	/* 0x112D L-Limit */

		RamWrite32A( gxlmt6H, MAXLMT_W ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT_W ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, CHGCOEF_MOV_W ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF_MOV_W ) ;		/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( STILLMODE ):				// Still mode 
		CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)
		CHECK_RETURN(SelectPtRange( OFF ) );					// Range narrow
		CHECK_RETURN(SelectIstpMod( OFF ) );					// Range narrow
		
		UcH1LvlMod = UcSetNum ;
			
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, 	CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 	CHGCOEF ) ;			/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( STILLMODE_W ):			// Still mode (Wide)
		CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)
		CHECK_RETURN(SelectPtRange( ON ) );					// Range wide
		CHECK_RETURN(SelectIstpMod( ON ) );					// Range wide
		
		UcH1LvlMod = UcSetNum ;
			
		RamWrite32A( gxlmt6L, MINLMT_W ) ;		/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT_W ) ;		/* 0x112D L-Limit */
		
		RamWrite32A( gxlmt6H, MAXLMT_W ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT_W ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, 	CHGCOEF_W ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 	CHGCOEF_W ) ;			/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	default :
		CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)
		CHECK_RETURN(SelectPtRange( OFF ) );					// Range narrow
		CHECK_RETURN(SelectIstpMod( OFF ) );					// Range narrow
		
		UcH1LvlMod = UcSetNum ;
			
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, 	CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 	CHGCOEF ) ;			/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
#else	/* CATCHMODE */
    case ( ACTMODE ):				// initial
        CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)

        /* enable setting */
        UcH1LvlMod = UcSetNum  ;

        // Limit value Value Setting
        RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
        RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */

        RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
        RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */

        RamWrite32A( gxhc_tmp, 	DIFIL_S2 ) ;	/* 0x100E Base Coef */
        RamWrite32A( gxmg, 		CHGCOEF ) ;		/* 0x10AA Change coefficient gain */

        RamWrite32A( gyhc_tmp, 	DIFIL_S2 ) ;	/* 0x110E Base Coef */
        RamWrite32A( gymg, 		CHGCOEF ) ;		/* 0x11AA Change coefficient gain */

        RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
        break ;

    case( S2MODE ):				// cancel lvl change mode
        RegWriteA( WG_HCHR, 0x10 ) ;			// 0x011B	GmHChrOn[1]=0 Sw OFF
        break ;

    case( MOVMODE ):			// Movie mode
        CHECK_RETURN(IniPtMovMod( ON ) );							// Pan/Tilt setting (Movie)

        RamWrite32A( gxlmt6L, MINLMT_MOV ) ;	/* 0x102D L-Limit */
        RamWrite32A( gylmt6L, MINLMT_MOV ) ;	/* 0x112D L-Limit */

        RamWrite32A( gxmg, CHGCOEF_MOV ) ;		/* 0x10AA Change coefficient gain */
        RamWrite32A( gymg, CHGCOEF_MOV ) ;		/* 0x11AA Change coefficient gain */

        RamWrite32A( gxhc_tmp, UlH1Coefval ) ;		/* 0x100E Base Coef */
        RamWrite32A( gyhc_tmp, UlH1Coefval ) ;		/* 0x110E Base Coef */

        RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
        break ;

    default :
        CHECK_RETURN(IniPtMovMod( OFF ) );							// Pan/Tilt setting (Still)

        UcH1LvlMod = UcSetNum ;

        RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
        RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */

        RamWrite32A( gxmg, 	CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
        RamWrite32A( gymg, 	CHGCOEF ) ;			/* 0x11AA Change coefficient gain */

        RamWrite32A( gxhc_tmp, UlH1Coefval ) ;		/* 0x100E Base Coef */
        RamWrite32A( gyhc_tmp, UlH1Coefval ) ;		/* 0x110E Base Coef */

        RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
        break ;
#endif	/* CATCHMODE */
    }

    return 0;
}


//********************************************************************************
// Function Name 	: IniAdj
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
static int IniAdj( void )
{
	RegWriteA( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Filter copy off

#ifdef	CATCHMODE
 #ifdef	CORRECT_1DEG
	CHECK_RETURN(SelectIstpMod( ON ) );
 #else
	CHECK_RETURN(SelectIstpMod( OFF ) );
 #endif
#else
	CHECK_RETURN(SelectIstpMod( OFF ) );   
#endif
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
	RamWriteA( OFF0Z,		HXOFF0Z_INI ) ;		// 0x1450
	RamWriteA( OFF1Z,		HYOFF1Z_INI ) ;		// 0x14D0
	RamWriteA( sxg,			SXGAIN_INI ) ;		// 0x10D3
	RamWriteA( syg,			SYGAIN_INI ) ;		// 0x11D3
//	UsCntXof = OPTCEN_X ;						/* Clear Optical center X value */
//	UsCntYof = OPTCEN_Y ;						/* Clear Optical center Y value */
//	RamWriteA( SXOFFZ1,		UsCntXof ) ;		// 0x1461
//	RamWriteA( SYOFFZ1,		UsCntYof ) ;		// 0x14E1

	/* AF adjusted parameter */
	RamWriteA( DAZHLO,		DAHLZO_INI ) ;		// 0x1529
	RamWriteA( DAZHLB,		DAHLZB_INI ) ;		// 0x152A

	/* Ram Access */
	CHECK_RETURN(RamAccFixMod( OFF ) );							// 32bit Float mode
	
	RamWrite32A( gxzoom, GXGAIN_INI ) ;		// 0x1020 Gyro X axis Gain adjusted value
	RamWrite32A( gyzoom, GYGAIN_INI ) ;		// 0x1120 Gyro Y axis Gain adjusted value

	RamWrite32A( sxq, SXQ_INI ) ;			// 0x10E5	X axis output direction initial value
	RamWrite32A( syq, SYQ_INI ) ;			// 0x11E5	Y axis output direction initial value
	
	if( GXHY_GYHX ){			/* GX -> HY , GY -> HX */
		RamWrite32A( sxgx, 0x00000000 ) ;			// 0x10B8
		RamWrite32A( sxgy, 0x3F800000 ) ;			// 0x10B9
		
		RamWrite32A( sygy, 0x00000000 ) ;			// 0x11B8
		RamWrite32A( sygx, 0x3F800000 ) ;			// 0x11B9
	}
	
	CHECK_RETURN(SetZsp(0) );								// Zoom coefficient Initial Setting
	
	RegWriteA( PWMA 	, 0xC0 );			// 0x0010		PWM enable

	RegWriteA( STBB0 	, 0xDF );			// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA( WC_EQSW	, 0x02 ) ;			// 0x01E0
	RegWriteA( WC_MESLOOP1	, 0x02 ) ;		// 0x0193
	RegWriteA( WC_MESLOOP0	, 0x00 ) ;		// 0x0192
	RegWriteA( WC_AMJLOOP1	, 0x02 ) ;		// 0x01A3
	RegWriteA( WC_AMJLOOP0	, 0x00 ) ;		// 0x01A2
	
	
	CHECK_RETURN(SetPanTiltMode( OFF ) );					/* Pan/Tilt OFF */

	CHECK_RETURN(SetH1cMod( ACTMODE ) );					/* Lvl Change Active mode */
	
	CHECK_RETURN(DrvSw( ON ) );							/* 0x0001		Driver Mode setting */
	
	RegWriteA( WC_EQON, 0x01 ) ;			// 0x0101	Filter ON

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
	// Command Execute Process Initial
	//IniCmd() ;
	// Clock Setting
	CHECK_RETURN(IniClk() );
	// AF Initial Setting
	CHECK_RETURN(IniAf() );
	// I/O Port Initial Setting
	CHECK_RETURN(IniIop() );
	// DigitalGyro Initial Setting
	CHECK_RETURN(IniDgy() );
	// Monitor & Other Initial Setting
	CHECK_RETURN(IniMon() );
	// Servo Initial Setting
	CHECK_RETURN(IniSrv() );
	// Gyro Filter Initial Setting
	CHECK_RETURN(IniGyr() );
	// Gyro Filter Initial Setting
	CHECK_RETURN(IniFil() );
	// Adjust Fix Value Setting
	CHECK_RETURN(IniAdj() );

    return 0;
}

//********************************************************************************
// Function Name 	: GyrCon
// Retun Value		: NON
// Argment Value	: Gyro Filter ON or OFF
// Explanation		: Gyro Filter Control Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
static int GyrCon( uint8_t	UcGyrCon )
{
    // Return HPF Setting
    RegWriteA( WG_SHTON, 0x00 ) ;									// 0x0107

    if( UcGyrCon == ON ) {												// Gyro ON
        CHECK_RETURN(ClrGyr( 0x000E , CLR_FRAM1 ));		// Gyro Delay RAM Clear
        RamWrite32A( sxggf, 0x3F800000 ) ;	// 0x10B5
        RamWrite32A( syggf, 0x3F800000 ) ;	// 0x11B5

    } else if( UcGyrCon == SPC ) {										// Gyro ON for LINE
        RamWrite32A( sxggf, 0x3F800000 ) ;	// 0x10B5
        RamWrite32A( syggf, 0x3F800000 ) ;	// 0x11B5
    } else {															// Gyro OFF
        RamWrite32A( sxggf, 0x00000000 ) ;	// 0x10B5
        RamWrite32A( syggf, 0x00000000 ) ;	// 0x11B5
    }

    return 0;
}

//********************************************************************************
// Function Name 	: SrvCon
// Retun Value		: NON
// Argment Value	: X or Y Select, Servo ON/OFF
// Explanation		: Servo ON,OFF Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
static int SrvCon( uint8_t	UcDirSel, uint8_t	UcSwcCon )
{
    if( UcSwcCon ) {
        if( !UcDirSel ) {						// X Direction
            RegWriteA( WH_EQSWX , 0x03 ) ;			// 0x0170
            RamWrite32A( sxggf, 0x00000000 ) ;		// 0x10B5
        } else {								// Y Direction
            RegWriteA( WH_EQSWY , 0x03 ) ;			// 0x0171
            RamWrite32A( syggf, 0x00000000 ) ;		// 0x11B5
        }
    } else {
        if( !UcDirSel ) {						// X Direction
            RegWriteA( WH_EQSWX , 0x02 ) ;			// 0x0170
            RamWrite32A( SXLMT, 0x00000000 ) ;		// 0x1477
        } else {								// Y Direction
            RegWriteA( WH_EQSWY , 0x02 ) ;			// 0x0171
            RamWrite32A( SYLMT, 0x00000000 ) ;		// 0x14F7
        }
    }

    return 0;
}

//********************************************************************************
// Function Name 	: CmdRdChk
// Retun Value		: 1 : ERROR
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2043.02.27 K.abe
//********************************************************************************

static uint8_t CmdRdChk( void )
{
    uint8_t UcTestRD;
    uint8_t UcCount;

    for(UcCount=0; UcCount < READ_COUNT_NUM; UcCount++){
        RegReadA( TESTRD ,	&UcTestRD );					// 0x027F
        if( UcTestRD == 0xAC){
            return(0);
        }
    }
    //TRACE("***** Command Line Error !! Can't Read Data *****\n" ) ;
    //TRACE_ERROR();
    return(1);
}


//********************************************************************************
// Function Name 	: StbOnn
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stabilizer For Servo On Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
static int StbOnn( void )
{
    uint8_t	UcRegValx,UcRegValy;					// Registor value
    uint8_t	UcRegIni ;
    uint8_t	UcRegIniCnt = 0;
    uint8_t SumCnt = 0;

    RegReadA( WH_EQSWX , &UcRegValx ) ;			// 0x0170
    RegReadA( WH_EQSWY , &UcRegValy ) ;			// 0x0171

    if( (( UcRegValx & 0x01 ) != 0x01 ) && (( UcRegValy & 0x01 ) != 0x01 ))
    {
//TRACE( " SMOOTH S \n" ) ;

        RegWriteA( WH_SMTSRVON,	0x01 ) ;				// 0x017C		Smooth Servo ON

        CHECK_RETURN(SrvCon( X_DIR, ON ) );
        CHECK_RETURN(SrvCon( Y_DIR, ON ) );

        UcRegIni = 0x11;
        while( (UcRegIni & 0x77) != 0x66 )
        {
            RegReadA( RH_SMTSRVSTT,	&UcRegIni ) ;		// 0x01F8		Smooth Servo phase read
            if( CmdRdChk() !=0 )	break;				// Dead Lock check (responce check)
            if((UcRegIni & 0x77 ) == 0 )	UcRegIniCnt++ ;
            if( UcRegIniCnt > 10 ){
//TRACE( " Slope Error \n" ) ;
                break ;			// Status Error
            }
            WitTim(3);
            SumCnt++;
            if(SumCnt > 20) {
                pr_err("%s:Can not smooth move break while \n",__func__);
                break;
            }
        }
        RegWriteA( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF

    }
    else
    {
        CHECK_RETURN(SrvCon( X_DIR, ON ) );
        CHECK_RETURN(SrvCon( Y_DIR, ON ) );
//TRACE( " Not Slope \n" ) ;
    }

    return 0;
}
//********************************************************************************
// Function Name     : StbOnnN
// Retun Value       : NON
// Argment Value     : NON
// Explanation       : Stabilizer For Servo On Function
// History           : First edition                        2013.10.09 Y.Shigeoka
//********************************************************************************
int imx278_sunny_ois_StbOnnN( uint8_t UcStbY , uint8_t UcStbX )
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
    RegWriteA( WH_SMTSRVON, 0x00 ) ;// 0x017C Smooth Servo OFF
    return 0;
}

//********************************************************************************
// Function Name 	: RtnCen
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
static int RtnCen( uint8_t	UcCmdPar )
{
    //uint8_t	UcCmdSts ;

    //UcCmdSts	= EXE_END ;

    CHECK_RETURN(GyrCon( OFF ) );											// Gyro OFF

    if( !UcCmdPar ) {										// X,Y Centering

        CHECK_RETURN(StbOnn() );											// Slope Mode

    } else if( UcCmdPar == 0x01 ) {							// X Centering Only

        CHECK_RETURN(SrvCon( X_DIR, ON ) );								// X only Servo ON
        CHECK_RETURN(SrvCon( Y_DIR, OFF ) );
    } else if( UcCmdPar == 0x02 ) {							// Y Centering Only

        CHECK_RETURN(SrvCon( X_DIR, OFF ) );								// Y only Servo ON
        CHECK_RETURN(SrvCon( Y_DIR, ON ) );
    }

    return( 0 ) ;
}

//********************************************************************************
// Function Name 	: OisEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
static int OisEna( void )
{
    // Servo ON
    CHECK_RETURN(SrvCon( X_DIR, ON ) );
    CHECK_RETURN(SrvCon( Y_DIR, ON ) );

    CHECK_RETURN(GyrCon( ON ) );

    return 0;
}

//********************************************************************************
// Function Name 	: OisEnaLin
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function for Line adjustment
// History			: First edition 						2013.09.05 Y.Shigeoka
//********************************************************************************
static int OisEnaLin( void )
{
    // Servo ON
    CHECK_RETURN(SrvCon( X_DIR, ON ) );
    CHECK_RETURN(SrvCon( Y_DIR, ON )) ;

    CHECK_RETURN(GyrCon( SPC )) ;
    return 0;
}

//********************************************************************************
// Function Name 	: IniSetAf
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial AF Setting Function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
static int IniSetAf( void )
{
	// Command Execute Process Initial
	//IniCmd() ;
	// Clock Setting
	CHECK_RETURN(IniClk() );
	// AF Initial Setting
	CHECK_RETURN(IniAf() );

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
    //#define	 SW_FSTMODE		0x0400	// Fast Stable Mode
    //RamWriteA( TCODEH , pos | SW_FSTMODE);
    RamWriteA( TREG_H  , pos << 6 );  
    return 0;
}

//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition 						2009.07.31  Y.Tashita
//********************************************************************************
static int MesFil( uint8_t	UcMesMod )
{
    if( !UcMesMod ) {								// Hall Bias&Offset Adjust
        // Measure Filter1 Setting
        RamWrite32A( mes1aa, 0x3CA175C0 ) ;		// 0x10F0	LPF150Hz
        RamWrite32A( mes1ab, 0x3CA175C0 ) ;		// 0x10F1
        RamWrite32A( mes1ac, 0x3F75E8C0 ) ;		// 0x10F2
        RamWrite32A( mes1ad, 0x00000000 ) ;		// 0x10F3
        RamWrite32A( mes1ae, 0x00000000 ) ;		// 0x10F4
        RamWrite32A( mes1ba, 0x3F800000 ) ;		// 0x10F5	Through
        RamWrite32A( mes1bb, 0x00000000 ) ;		// 0x10F6
        RamWrite32A( mes1bc, 0x00000000 ) ;		// 0x10F7
        RamWrite32A( mes1bd, 0x00000000 ) ;		// 0x10F8
        RamWrite32A( mes1be, 0x00000000 ) ;		// 0x10F9

        // Measure Filter2 Setting
        RamWrite32A( mes2aa, 0x3CA175C0 ) ;		// 0x11F0	LPF150Hz
        RamWrite32A( mes2ab, 0x3CA175C0 ) ;		// 0x11F1
        RamWrite32A( mes2ac, 0x3F75E8C0 ) ;		// 0x11F2
        RamWrite32A( mes2ad, 0x00000000 ) ;		// 0x11F3
        RamWrite32A( mes2ae, 0x00000000 ) ;		// 0x11F4
        RamWrite32A( mes2ba, 0x3F800000 ) ;		// 0x11F5	Through
        RamWrite32A( mes2bb, 0x00000000 ) ;		// 0x11F6
        RamWrite32A( mes2bc, 0x00000000 ) ;		// 0x11F7
        RamWrite32A( mes2bd, 0x00000000 ) ;		// 0x11F8
        RamWrite32A( mes2be, 0x00000000 ) ;		// 0x11F9

    } else if( UcMesMod == LOOPGAIN ) {				// Loop Gain Adjust
        // Measure Filter1 Setting
        RamWrite32A( mes1aa, 0x3DF21080 ) ;		// 0x10F0	LPF1000Hz
        RamWrite32A( mes1ab, 0x3DF21080 ) ;		// 0x10F1
        RamWrite32A( mes1ac, 0x3F437BC0 ) ;		// 0x10F2
        RamWrite32A( mes1ad, 0x00000000 ) ;		// 0x10F3
        RamWrite32A( mes1ae, 0x00000000 ) ;		// 0x10F4
        RamWrite32A( mes1ba, 0x3F7EF980 ) ;		// 0x10F5	HPF30Hz
        RamWrite32A( mes1bb, 0xBF7EF980 ) ;		// 0x10F6
        RamWrite32A( mes1bc, 0x3F7DF300 ) ;		// 0x10F7
        RamWrite32A( mes1bd, 0x00000000 ) ;		// 0x10F8
        RamWrite32A( mes1be, 0x00000000 ) ;		// 0x10F9

        // Measure Filter2 Setting
        RamWrite32A( mes2aa, 0x3DF21080 ) ;		// 0x11F0	LPF1000Hz
        RamWrite32A( mes2ab, 0x3DF21080 ) ;		// 0x11F1
        RamWrite32A( mes2ac, 0x3F437BC0 ) ;		// 0x11F2
        RamWrite32A( mes2ad, 0x00000000 ) ;		// 0x11F3
        RamWrite32A( mes2ae, 0x00000000 ) ;		// 0x11F4
        RamWrite32A( mes2ba, 0x3F7EF980 ) ;		// 0x11F5	HPF30Hz
        RamWrite32A( mes2bb, 0xBF7EF980 ) ;		// 0x11F6
        RamWrite32A( mes2bc, 0x3F7DF300 ) ;		// 0x11F7
        RamWrite32A( mes2bd, 0x00000000 ) ;		// 0x11F8
        RamWrite32A( mes2be, 0x00000000 ) ;		// 0x11F9

    } else if( UcMesMod == THROUGH ) {				// for Through
        // Measure Filter1 Setting
        RamWrite32A( mes1aa, 0x3F800000 ) ;		// 0x10F0	Through
        RamWrite32A( mes1ab, 0x00000000 ) ;		// 0x10F1
        RamWrite32A( mes1ac, 0x00000000 ) ;		// 0x10F2
        RamWrite32A( mes1ad, 0x00000000 ) ;		// 0x10F3
        RamWrite32A( mes1ae, 0x00000000 ) ;		// 0x10F4
        RamWrite32A( mes1ba, 0x3F800000 ) ;		// 0x10F5	Through
        RamWrite32A( mes1bb, 0x00000000 ) ;		// 0x10F6
        RamWrite32A( mes1bc, 0x00000000 ) ;		// 0x10F7
        RamWrite32A( mes1bd, 0x00000000 ) ;		// 0x10F8
        RamWrite32A( mes1be, 0x00000000 ) ;		// 0x10F9

        // Measure Filter2 Setting
        RamWrite32A( mes2aa, 0x3F800000 ) ;		// 0x11F0	Through
        RamWrite32A( mes2ab, 0x00000000 ) ;		// 0x11F1
        RamWrite32A( mes2ac, 0x00000000 ) ;		// 0x11F2
        RamWrite32A( mes2ad, 0x00000000 ) ;		// 0x11F3
        RamWrite32A( mes2ae, 0x00000000 ) ;		// 0x11F4
        RamWrite32A( mes2ba, 0x3F800000 ) ;		// 0x11F5	Through
        RamWrite32A( mes2bb, 0x00000000 ) ;		// 0x11F6
        RamWrite32A( mes2bc, 0x00000000 ) ;		// 0x11F7
        RamWrite32A( mes2bd, 0x00000000 ) ;		// 0x11F8
        RamWrite32A( mes2be, 0x00000000 ) ;		// 0x11F9

    } else if( UcMesMod == NOISE ) {				// SINE WAVE TEST for NOISE
        // Measure Filter1 Setting
        RamWrite32A( mes1aa, 0x3CA175C0 ) ;		// 0x10F0	LPF150Hz
        RamWrite32A( mes1ab, 0x3CA175C0 ) ;		// 0x10F1
        RamWrite32A( mes1ac, 0x3F75E8C0 ) ;		// 0x10F2
        RamWrite32A( mes1ad, 0x00000000 ) ;		// 0x10F3
        RamWrite32A( mes1ae, 0x00000000 ) ;		// 0x10F4
        RamWrite32A( mes1ba, 0x3CA175C0 ) ;		// 0x10F5	LPF150Hz
        RamWrite32A( mes1bb, 0x3CA175C0 ) ;		// 0x10F6
        RamWrite32A( mes1bc, 0x3F75E8C0 ) ;		// 0x10F7
        RamWrite32A( mes1bd, 0x00000000 ) ;		// 0x10F8
        RamWrite32A( mes1be, 0x00000000 ) ;		// 0x10F9

        // Measure Filter2 Setting
        RamWrite32A( mes2aa, 0x3CA175C0 ) ;		// 0x11F0	LPF150Hz
        RamWrite32A( mes2ab, 0x3CA175C0 ) ;		// 0x11F1
        RamWrite32A( mes2ac, 0x3F75E8C0 ) ;		// 0x11F2
        RamWrite32A( mes2ad, 0x00000000 ) ;		// 0x11F3
        RamWrite32A( mes2ae, 0x00000000 ) ;		// 0x11F4
        RamWrite32A( mes2ba, 0x3CA175C0 ) ;		// 0x11F5	LPF150Hz
        RamWrite32A( mes2bb, 0x3CA175C0 ) ;		// 0x11F6
        RamWrite32A( mes2bc, 0x3F75E8C0 ) ;		// 0x11F7
        RamWrite32A( mes2bd, 0x00000000 ) ;		// 0x11F8
        RamWrite32A( mes2be, 0x00000000 ) ;		// 0x11F9
    }

    return 0;
}

//********************************************************************************
// Function Name 	: GenMes
// Retun Value		: A/D Convert Result
// Argment Value	: Measure Filter Input Signal Ram Address
// Explanation		: General Measure Function
// History			: First edition 						2013.01.10 Y.Shigeoka
//********************************************************************************
static short GenMes( uint16_t	UsRamAdd, uint8_t	UcMesMod )
{
    short	SsMesRlt ;

    RegWriteA( WC_MES1ADD0, (uint8_t)UsRamAdd ) ;							// 0x0194
    RegWriteA( WC_MES1ADD1, (uint8_t)(( UsRamAdd >> 8 ) & 0x0001 ) ) ;	// 0x0195
    RamWrite32A( MSABS1AV, 0x00000000 );				// 0x1041	Clear

    if( !UcMesMod ) {
        RegWriteA( WC_MESLOOP1, 0x04 ) ;				// 0x0193
        RegWriteA( WC_MESLOOP0, 0x00 ) ;				// 0x0192	1024 Times Measure
        RamWrite32A( msmean	, 0x3A7FFFF7 );				// 0x1230	1/CmMesLoop[15:0]
    } else {
        RegWriteA( WC_MESLOOP1, 0x00 ) ;				// 0x0193
        RegWriteA( WC_MESLOOP0, 0x01 ) ;				// 0x0192	1 Times Measure
        RamWrite32A( msmean	, 0x3F800000 );				// 0x1230	1/CmMesLoop[15:0]
    }

    RegWriteA( WC_MESABS, 0x00 ) ;						// 0x0198	none ABS
    CHECK_RETURN(BsyWit( WC_MESMODE, 0x01 ) );						// 0x0190	normal Measure

    CHECK_RETURN(RamAccFixMod( ON ) );							// Fix mode

    RamReadA( MSABS1AV, ( uint16_t * )&SsMesRlt ) ;	// 0x1041

    CHECK_RETURN(RamAccFixMod( OFF ) );							// Float mode

    return( SsMesRlt ) ;
}


//********************************************************************************
// Function Name 	: SetSinWavePara
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave Test Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
#define		USE_SINLPF			/* if sin or circle movement is used LPF , this define has to enable */

/* êUïùÇÕsxsin(0x10D5),sysin(0x11D5)Ç≈í≤êÆ */
static int SetSinWavePara( uint8_t UcTableVal ,  uint8_t UcMethodVal )
{
    uint16_t	UsFreqDat ;
    uint8_t	UcEqSwX , UcEqSwY ;


    if(UcTableVal > 0x10 )
        UcTableVal = 0x10 ;			/* Limit */
    UsFreqDat = CucFreqVal[ UcTableVal ] ;

    if( UcMethodVal == SINEWAVE) {
        RegWriteA( WC_SINPHSX, 0x00 ) ;					/* 0x0183	*/
        RegWriteA( WC_SINPHSY, 0x00 ) ;					/* 0x0184	*/
    }else if( UcMethodVal == CIRCWAVE ){
        RegWriteA( WC_SINPHSX,	0x00 ) ;				/* 0x0183	*/
        RegWriteA( WC_SINPHSY,	0x20 ) ;				/* 0x0184	*/
    }else{
        RegWriteA( WC_SINPHSX, 0x00 ) ;					/* 0x0183	*/
        RegWriteA( WC_SINPHSY, 0x00 ) ;					/* 0x0184	*/
    }

#ifdef	USE_SINLPF
    if(( UcMethodVal == CIRCWAVE ) || ( UcMethodVal == SINEWAVE )) {
        CHECK_RETURN(MesFil( NOISE ) );			/* LPF */
    }
#endif

    if( UsFreqDat == 0xFFFF )			/* SineîgíÜé~ */
    {

        RegReadA( WH_EQSWX, &UcEqSwX ) ;				/* 0x0170	*/
        RegReadA( WH_EQSWY, &UcEqSwY ) ;				/* 0x0171	*/
        UcEqSwX &= ~EQSINSW ;
        UcEqSwY &= ~EQSINSW ;
        RegWriteA( WH_EQSWX, UcEqSwX ) ;				/* 0x0170	*/
        RegWriteA( WH_EQSWY, UcEqSwY ) ;				/* 0x0171	*/

#ifdef	USE_SINLPF
        if(( UcMethodVal == CIRCWAVE ) || ( UcMethodVal == SINEWAVE ) || ( UcMethodVal == XACTTEST ) || ( UcMethodVal == YACTTEST )) {
            RegWriteA( WC_DPON,     0x00 ) ;			/* 0x0105	Data pass off */
            RegWriteA( WC_DPO1ADD0, 0x00 ) ;			/* 0x01B8	output initial */
            RegWriteA( WC_DPO1ADD1, 0x00 ) ;			/* 0x01B9	output initial */
            RegWriteA( WC_DPO2ADD0, 0x00 ) ;			/* 0x01BA	output initial */
            RegWriteA( WC_DPO2ADD1, 0x00 ) ;			/* 0x01BB	output initial */
            RegWriteA( WC_DPI1ADD0, 0x00 ) ;			/* 0x01B0	input initial */
            RegWriteA( WC_DPI1ADD1, 0x00 ) ;			/* 0x01B1	input initial */
            RegWriteA( WC_DPI2ADD0, 0x00 ) ;			/* 0x01B2	input initial */
            RegWriteA( WC_DPI2ADD1, 0x00 ) ;			/* 0x01B3	input initial */

            /* Ram Access */
            CHECK_RETURN(RamAccFixMod( ON ) );							// Fix mode

            RamWriteA( SXOFFZ1, UsCntXof ) ;			/* 0x1461	set optical value */
            RamWriteA( SYOFFZ1, UsCntYof ) ;			/* 0x14E1	set optical value */

            /* Ram Access */
            CHECK_RETURN(RamAccFixMod( OFF ) );							// Float mode

            RegWriteA( WC_MES1ADD0,  0x00 ) ;			/* 0x0194	*/
            RegWriteA( WC_MES1ADD1,  0x00 ) ;			/* 0x0195	*/
            RegWriteA( WC_MES2ADD0,  0x00 ) ;			/* 0x0196	*/
            RegWriteA( WC_MES2ADD1,  0x00 ) ;			/* 0x0197	*/

        }
#endif
        RegWriteA( WC_SINON,     0x00 ) ;			/* 0x0180	Sine wave  */

    }
    else
    {

        RegReadA( WH_EQSWX, &UcEqSwX ) ;				/* 0x0170	*/
        RegReadA( WH_EQSWY, &UcEqSwY ) ;				/* 0x0171	*/

        if(( UcMethodVal == CIRCWAVE ) || ( UcMethodVal == SINEWAVE )) {
#ifdef	USE_SINLPF
            RegWriteA( WC_DPI1ADD0,  ( uint8_t )MES1BZ2 ) ;						/* 0x01B0	input Meas-Fil */
            RegWriteA( WC_DPI1ADD1,  ( uint8_t )(( MES1BZ2 >> 8 ) & 0x0001 ) ) ;	/* 0x01B1	input Meas-Fil */
            RegWriteA( WC_DPI2ADD0,  ( uint8_t )MES2BZ2 ) ;						/* 0x01B2	input Meas-Fil */
            RegWriteA( WC_DPI2ADD1,  ( uint8_t )(( MES2BZ2 >> 8 ) & 0x0001 ) ) ;	/* 0x01B3	input Meas-Fil */
            RegWriteA( WC_DPO1ADD0, ( uint8_t )SXOFFZ1 ) ;						/* 0x01B8	output SXOFFZ1 */
            RegWriteA( WC_DPO1ADD1, ( uint8_t )(( SXOFFZ1 >> 8 ) & 0x0001 ) ) ;	/* 0x01B9	output SXOFFZ1 */
            RegWriteA( WC_DPO2ADD0, ( uint8_t )SYOFFZ1 ) ;						/* 0x01BA	output SYOFFZ1 */
            RegWriteA( WC_DPO2ADD1, ( uint8_t )(( SYOFFZ1 >> 8 ) & 0x0001 ) ) ;	/* 0x01BA	output SYOFFZ1 */

            RegWriteA( WC_MES1ADD0,  ( uint8_t )SINXZ ) ;							/* 0x0194	*/
            RegWriteA( WC_MES1ADD1,  ( uint8_t )(( SINXZ >> 8 ) & 0x0001 ) ) ;	/* 0x0195	*/
            RegWriteA( WC_MES2ADD0,  ( uint8_t )SINYZ ) ;							/* 0x0196	*/
            RegWriteA( WC_MES2ADD1,  ( uint8_t )(( SINYZ >> 8 ) & 0x0001 ) ) ;	/* 0x0197	*/

            RegWriteA( WC_DPON,     0x03 ) ;			/* 0x0105	Data pass[1:0] on */

            UcEqSwX &= ~EQSINSW ;
            UcEqSwY &= ~EQSINSW ;
#else
            UcEqSwX |= 0x08 ;
            UcEqSwY |= 0x08 ;
#endif
        } else if(( UcMethodVal == XACTTEST ) || ( UcMethodVal == YACTTEST )) {
            RegWriteA( WC_DPI2ADD0,  ( uint8_t )MES2BZ2 ) ;						/* 0x01B2	input Meas-Fil */
            RegWriteA( WC_DPI2ADD1,  ( uint8_t )(( MES2BZ2 >> 8 ) & 0x0001 ) ) ;	/* 0x01B3	input Meas-Fil */
            if( UcMethodVal == XACTTEST ){
                RegWriteA( WC_DPO2ADD0, ( uint8_t )SXOFFZ1 ) ;						/* 0x01BA	output SXOFFZ1 */
                RegWriteA( WC_DPO2ADD1, ( uint8_t )(( SXOFFZ1 >> 8 ) & 0x0001 ) ) ;	/* 0x01BB	output SXOFFZ1 */
                RegWriteA( WC_MES2ADD0,  ( uint8_t )SINXZ ) ;							/* 0x0196	*/
                RegWriteA( WC_MES2ADD1,  ( uint8_t )(( SINXZ >> 8 ) & 0x0001 ) ) ;	/* 0x0197	*/
            } else {
                RegWriteA( WC_DPO2ADD0, ( uint8_t )SYOFFZ1 ) ;						/* 0x01BA	output SYOFFZ1 */
                RegWriteA( WC_DPO2ADD1, ( uint8_t )(( SYOFFZ1 >> 8 ) & 0x0001 ) ) ;	/* 0x01BB	output SYOFFZ1 */
                RegWriteA( WC_MES2ADD0,  ( uint8_t )SINYZ ) ;							/* 0x0196	*/
                RegWriteA( WC_MES2ADD1,  ( uint8_t )(( SINYZ >> 8 ) & 0x0001 ) ) ;	/* 0x0197	*/
            }

            RegWriteA( WC_DPON,     0x02 ) ;			/* 0x0105	Data pass[1] on */

            UcEqSwX &= ~EQSINSW ;
            UcEqSwY &= ~EQSINSW ;

        }else{
            if( UcMethodVal == XHALWAVE ){
                UcEqSwX = 0x22 ;				/* SW[5] */
//		    	UcEqSwY = 0x03 ;
            }else{
//				UcEqSwX = 0x03 ;
                UcEqSwY = 0x22 ;				/* SW[5] */
            }
        }

        RegWriteA( WC_SINFRQ0,	(uint8_t)UsFreqDat ) ;				// 0x0181		Freq L
        RegWriteA( WC_SINFRQ1,	(uint8_t)(UsFreqDat >> 8) ) ;			// 0x0182		Freq H
        RegWriteA( WC_MESSINMODE,     0x00 ) ;			/* 0x0191	Sine 0 cross  */

        RegWriteA( WH_EQSWX, UcEqSwX ) ;				/* 0x0170	*/
        RegWriteA( WH_EQSWY, UcEqSwY ) ;				/* 0x0171	*/

        RegWriteA( WC_SINON,     0x01 ) ;			/* 0x0180	Sine wave  */

    }

    return 0;
}


#ifdef	ACCEPTANCE
static int TstActMov( uint8_t UcDirSel );

//********************************************************************************
// Function Name 	: RunHea
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Hall Examination of Acceptance
// History			: First edition 						2014.02.26 Y.Shigeoka
//********************************************************************************
static int RunHea( void )
{
    int 	UcRst ;
    int data = 0;

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
    UcRst |= data;

//TRACE("UcRst = %02x\n", UcRst ) ;

    return( UcRst ) ;
}

static int TstActMov( uint8_t UcDirSel )
{
    int UcRsltSts;
    uint16_t	UsMsppVal ;

    CHECK_RETURN(MesFil( NOISE ) );					// ë™íËópÉtÉBÉãÉ^Å[Çê›íËÇ∑ÇÈÅB

    if ( !UcDirSel ) {
        RamWrite32A( sxsin , ACT_CHK_LVL );												// 0x10D5
        RamWrite32A( sysin , 0x000000 );												// 0x11D5
        CHECK_RETURN(SetSinWavePara( 0x05 , XACTTEST ));
    }else{
        RamWrite32A( sxsin , 0x000000 );												// 0x10D5
        RamWrite32A( sysin , ACT_CHK_LVL );												// 0x11D5
        CHECK_RETURN(SetSinWavePara( 0x05 , YACTTEST ));
    }

    if ( !UcDirSel ) {					// AXIS X
        RegWriteA( WC_MES1ADD0,  ( uint8_t )SXINZ1 ) ;							/* 0x0194	*/
        RegWriteA( WC_MES1ADD1,  ( uint8_t )(( SXINZ1 >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
    } else {							// AXIS Y
        RegWriteA( WC_MES1ADD0,  ( uint8_t )SYINZ1 ) ;							/* 0x0194	*/
        RegWriteA( WC_MES1ADD1,  ( uint8_t )(( SYINZ1 >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
    }

    RegWriteA( WC_MESSINMODE, 0x00 ) ;			// 0x0191	0 cross
    RegWriteA( WC_MESLOOP1, 0x00 ) ;				// 0x0193
    RegWriteA( WC_MESLOOP0, 0x02 ) ;				// 0x0192	2 Times Measure
    RamWrite32A( msmean	, 0x3F000000 );				// 0x10AE	1/CmMesLoop[15:0]/2
    RegWriteA( WC_MESABS, 0x00 ) ;				// 0x0198	none ABS
    CHECK_RETURN(BsyWit( WC_MESMODE, 0x02 ) );				// 0x0190		Sine wave Measure

    CHECK_RETURN(RamAccFixMod( ON ) );							// Fix mode
    RamReadA( MSPP1AV, &UsMsppVal ) ;
    CHECK_RETURN(RamAccFixMod( OFF ) );							// Float mode

    if ( !UcDirSel ) {					// AXIS X
        CHECK_RETURN(SetSinWavePara( 0x00 , XACTTEST )); 	/* STOP */
    }else{
        CHECK_RETURN(SetSinWavePara( 0x00 , YACTTEST )); 	/* STOP */
    }

//TRACE(" DIR = %04x, PP = %04x, ", UcDirSel, UsMsppVal ) ;


    UcRsltSts = EXE_END ;
    if( UsMsppVal > ACT_THR ){
        if ( !UcDirSel ) {					// AXIS X
            UcRsltSts = EXE_HXMVER ;
        }else{								// AXIS Y
            UcRsltSts = EXE_HYMVER ;
        }
    }

    return( UcRsltSts ) ;
}


//********************************************************************************
// Function Name 	: RunGea
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Examination of Acceptance
// History			: First edition 						2014.02.13 T.Tokoro
//********************************************************************************
static int RunGea( void )
{
    uint8_t 	UcRst, UcCnt, UcXLowCnt, UcYLowCnt, UcXHigCnt, UcYHigCnt ;
    uint16_t	UsGxoVal[10], UsGyoVal[10], UsDif;

    short genmes_value = 0;

    UcRst = EXE_END ;
    UcXLowCnt = UcYLowCnt = UcXHigCnt = UcYHigCnt = 0 ;

    CHECK_RETURN(MesFil( THROUGH ) );				// ë™íËópÉtÉBÉãÉ^Å[Çê›íËÇ∑ÇÈÅB

    for( UcCnt = 0 ; UcCnt < 10 ; UcCnt++ )
    {
        // X
        RegWriteA( WC_MES1ADD0, 0x00 ) ;		// 0x0194
        RegWriteA( WC_MES1ADD1, 0x00 ) ;		// 0x0195
        CHECK_RETURN(ClrGyr( 0x1000 , CLR_FRAM1 ));							// Measure Filter RAM Clear
        genmes_value = GenMes( GX45Z, 0 );
        if(genmes_value < 0)
        {
            pr_err("%s:%d fail \n",__func__,__LINE__);
            return -1;
        }
        
		UsGxoVal[UcCnt] = (uint16_t)genmes_value;	// 64âÒÇÃïΩãœílë™íË	GYRMON1(0x1110) <- GX45Z(0x1401)		

        // Y
        RegWriteA( WC_MES1ADD0, 0x00 ) ;		// 0x0194
        RegWriteA( WC_MES1ADD1, 0x00 ) ;		// 0x0195
        CHECK_RETURN(ClrGyr( 0x1000 , CLR_FRAM1 ));							// Measure Filter RAM Clear
        genmes_value = GenMes( GY45Z, 0 );
        if(genmes_value < 0)
        {
            pr_err("%s:%d fail \n",__func__,__LINE__);
            return -1;
        }
        UsGyoVal[UcCnt] = (uint16_t)genmes_value;	// 64âÒÇÃïΩãœílë™íË	GYRMON2(0x1111) <- GY45Z(0x1481)

//TRACE("UcCnt = %02x, UsGxoVal[UcCnt] = %04x\n", UcCnt, UsGxoVal[UcCnt] ) ;
//TRACE("UcCnt = %02x, UsGyoVal[UcCnt] = %04x\n", UcCnt, UsGyoVal[UcCnt] ) ;

//		WitTim( 100 ) ;							// Wait 100ms

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

    return (int)UcRst;
}
#endif	//ACCEPTANCE


static int setOISdata(void)
{
    pr_info("%s:willow begin\n",__func__);
    pr_info("%s:willow osc=%x DAXHLO=%2x\n",__func__,StAdjPar.UcOscVal,StAdjPar.StHalAdj.UsHlxOff);
    RegWriteA( OSCSET, StAdjPar.UcOscVal ) ;
    CHECK_RETURN(RamAccFixMod( ON ));
    RamWriteA( DAXHLO, StAdjPar.StHalAdj.UsHlxOff );
    RamWriteA( DAXHLB, StAdjPar.StHalAdj.UsHlxGan );
    RamWriteA( DAYHLO, StAdjPar.StHalAdj.UsHlyOff );
    RamWriteA( DAYHLB, StAdjPar.StHalAdj.UsHlyGan );
    RamWriteA( OFF0Z,  StAdjPar.StHalAdj.UsAdxOff );
    RamWriteA( OFF1Z,  StAdjPar.StHalAdj.UsAdyOff );
    RamWriteA( sxg,    StAdjPar.StLopGan.UsLxgVal );			// X Loop Gain
    RamWriteA( syg,    StAdjPar.StLopGan.UsLygVal );			// Y Loop Gain
    RamWriteA( IZAH,   StAdjPar.StGvcOff.UsGxoVal );			// X Gyro Offset
    RamWriteA( IZBH,   StAdjPar.StGvcOff.UsGyoVal );			// Y Gyro Offset
    CHECK_RETURN(RamAccFixMod( OFF ));
    RamWrite32A( gxzoom, StAdjPar.StGvcOff.UlGxgVal );
    RamWrite32A( gyzoom, StAdjPar.StGvcOff.UlGygVal );

    return 0;
}

int imx278_sunny_ois_setGyroGain(int32_t new_xgain, int32_t new_ygain)
{
    pr_info("%s: towrite xgain = %8x ygain = %8x \n", __func__,new_xgain, new_ygain);
    if(new_xgain!=0 && new_ygain!=0)
    {
        RamWrite32A( gxzoom, new_xgain );
        RamWrite32A( gyzoom, new_ygain );
    }
    
    return 0;
}

int imx278_sunny_ois_getGyroGain(int32_t* xgain, int32_t* ygain)
{
    RamRead32A( gxzoom, xgain );
    RamRead32A( gyzoom, ygain );
    pr_info("%s: form register read out xgain = %8x ygain = %8x \n", __func__, *xgain, *ygain);
    
    return 0;
}

int imx278_sunny_ois_init(void)
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

int imx278_sunny_ois_initSpecial(void)
{
    CHECK_RETURN(IniSetAf());
    return 0;
}
int imx278_sunny_ois_change_mode(uint32_t mode)
{
	if(mode == OIS_PREVIEW)
	{	
    	RamWrite32A( sxlmta2, 	X_CURRENT_LMT_PREVIEW );		// 0x10E7 		Hall X output Limit
	    RamWrite32A( sylmta2, 	Y_CURRENT_LMT_PREVIEW ); 
	}
	else if (mode == OIS_CAPTURE)
	{
    	RamWrite32A( sxlmta2, 	X_CURRENT_LMT_CAPTURE );		// 0x10E7 		Hall X output Limit
	    RamWrite32A( sylmta2, 	Y_CURRENT_LMT_CAPTURE ); 
	}
    return 0;
}


int imx278_sunny_ois_turn_onoff(uint32_t on)
{
    if(on)
    {
    	CHECK_RETURN(imx278_sunny_ois_change_mode(OIS_CAPTURE));
        CHECK_RETURN(OisEna());
    }
    else
    {
    	CHECK_RETURN(imx278_sunny_ois_change_mode(OIS_PREVIEW));
        CHECK_RETURN(RtnCen(0x00));
    }

    return 0;
}
int imx278_sunny_ois_turn_onoff_lin(uint32_t on)
{
    if(on)
    {
        CHECK_RETURN(imx278_sunny_ois_change_mode(OIS_CAPTURE));
        CHECK_RETURN(OisEnaLin());
    }
    else
    {
        CHECK_RETURN(imx278_sunny_ois_change_mode(OIS_PREVIEW));
        CHECK_RETURN(RtnCen(0x00));
    }

    return 0;
}

void imx278_sunny_InitOISData(void *oisdata, uint32_t oislenth)
{
    uint8_t *data = (uint8_t *)oisdata;
    //memcpy((void*)&data, oisdata, oislenth);
    StAdjPar.StHalAdj.UsHlxOff = (data[0] << 8)|data[1];    //DAXHLO Hall offset X
    StAdjPar.StHalAdj.UsHlyOff = (data[2] << 8)|data[3];    //DAYHLO Hall offset Y
    StAdjPar.StHalAdj.UsHlxGan = (data[4] << 8)|data[5];    //DAXHLB Hall bias X
    StAdjPar.StHalAdj.UsHlyGan = (data[6] << 8)|data[7];    //DAYHLB Hall bias Y
    StAdjPar.StHalAdj.UsAdxOff = (data[8] << 8)|data[9];    //HXOFF0Z_INI Hall AD offset X
    StAdjPar.StHalAdj.UsAdyOff = (data[10] << 8)|data[11];  //HYOFF1Z_INI Hall AD offset Y
    StAdjPar.StLopGan.UsLxgVal = (data[12] << 8)|data[13];  //sxg  SXGAIN_INI Loop gain X 
    StAdjPar.StLopGan.UsLygVal = (data[14] << 8)|data[15];  //syg  SYGAIN_INI Loop gain Y
    UsCntXof                   = (data[16] << 8)|data[17];  //optical center X
    UsCntYof                   = (data[18] << 8)|data[19];  //optical center Y
    StAdjPar.StGvcOff.UsGxoVal = (data[20] << 8)|data[21];  //IZAH DGYRO_OFST_XH Gyro offset X
    StAdjPar.StGvcOff.UsGyoVal = (data[22] << 8)|data[23];  //IZBH DGYRO_OFST_YH Gyro offset Y
    StAdjPar.UcOscVal          = data[24];
    StAdjPar.StGvcOff.UlGxgVal = (data[25] << 24)|(data[26] << 16)|(data[27] << 8)|data[28];// gxzoom UlGxgVal
    StAdjPar.StGvcOff.UlGygVal = (data[29] << 24)|(data[30] << 16)|(data[31] << 8)|data[32];// gyzoom UlGygVal
}

//for test interface
int imx278_sunny_ois_AF_SetPos(short data)
{
    CHECK_RETURN(AF_SetPos(data));
    return 0;
}
int imx278_sunny_ois_RamAccFixMod(uint8_t data)
{
    CHECK_RETURN(RamAccFixMod(data));
    return 0;
}
int imx278_sunny_ois_RtnCen(uint8_t data)
{
    return RtnCen(data);
}
int imx278_sunny_ois_SetPanTiltMode(uint8_t data)
{
    CHECK_RETURN(SetPanTiltMode(data));
    return 0;
}
int imx278_sunny_ois_RunGea(void)
{
    return RunGea();
}
int imx278_sunny_ois_RunHea(void)
{
    return RunHea();
}
int imx278_sunny_ois_OisEna(void)
{
    CHECK_RETURN(OisEna());
    return 0;
}
int imx278_sunny_ois_MagnetismRead(int32_t *otpcenX, int32_t *otpcenY, int32_t *srvoffX, int32_t *srvoffY)
{
    RamRead32A(OFF0Z ,otpcenX);
    RamRead32A(OFF1Z ,otpcenY);
    RamRead32A(AD0OFFZ, srvoffX);
    RamRead32A(AD1OFFZ, srvoffY);
    return 0;
}

