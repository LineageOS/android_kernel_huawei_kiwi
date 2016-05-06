/*add camera ois driver*/
#ifndef _IMX278_LGIT_OIS_DATA_HEAD
#define _IMX278_LGIT_OIS_DATA_HEAD
#define LAST_UPDATE  "15-05-21"	//LC898122A FW
#define	OISFW_RELEASE_VER (0xD1D0)	//LC898122A FW

#ifndef	K7_OIS_H
#define	K7_OIS_H
//==============================================================================
// Ois.h Code START (2012. 12. 02)
//==============================================================================
#ifdef	K7_OISINI
	#define	K7_OISINI__
#else
	#define	K7_OISINI__		extern
#endif
#define		CATCHMODE               // Catch mode
#ifdef	CATCHMODE
  #define	CORRECT_1DEG			// Correct 1deg   disable 0.5deg
  #define	G_45G_INI	0x3E3FECFF	// 32.8/175 = 0.187428571 = -14.543[dB]
  //#define	G_45G_INI	0x3F004DC0	// 32.8/65.5 = 0.5 = -6[dB]
#endif	//CATCHMODE

#define	OIS_FW_POLLING_PASS		0
#define	OIS_FW_POLLING_FAIL		-1

#define	AGING_TEST			// Aging Test

// EEPROM Address Define	0900_0940
#define		E2POFST				0x0900

/****************** Hall Limiter ******************/
#define		HALL_LIMIT_X			(0x0000 + E2POFST)		// 4byte setting
#define		HALL_LIMIT_Y			(0x0004 + E2POFST)		// 4byte setting
/****************** Hall Adjust Stroke ******************/
#define		MAX_HALL_AFTER_X		(0x0008 + E2POFST)		// X Max Hall Adjust Value After
#define		MIN_HALL_AFTER_X		(0x000A + E2POFST)		// X Min Hall Adjust Value After
#define		MAX_HALL_AFTER_Y		(0x000C + E2POFST)		// Y Max Hall Adjust Value After
#define		MIN_HALL_AFTER_Y		(0x000E + E2POFST)		// Y Min Hall Adjust Value After
/****************** page ******************/
// Hall Adjust
#define		HALL_BIAS_X				(0x0010 + E2POFST)		// X Hall Bias
#define		HALL_BIAS_Y				(0x0012 + E2POFST)		// Y Hall Bias
#define		HALL_OFFSET_X			(0x0014 + E2POFST)		// X Hall Offset
#define		HALL_OFFSET_Y			(0x0016 + E2POFST)		// Y Hall Offset
// Loop Gain Adjust
#define		LOOP_GAIN_X				(0x0018 + E2POFST)
#define		LOOP_GAIN_Y				(0x001A + E2POFST)
// LENS CENTER FINAL
#define		LENS_CENTER_FINAL_X		(0x001C + E2POFST)		
#define		LENS_CENTER_FINAL_Y		(0x001E + E2POFST)		
/****************** page ******************/
// Gyro A/D Offset
#define		GYRO_AD_OFFSET_X		(0x0020 + E2POFST)
#define		GYRO_AD_OFFSET_Y		(0x0022 + E2POFST)
// OSC VALUE
#define		OSC_CLK_VAL				(0x0024 + E2POFST)		// 1byte Setting
// Adjustment Completion Flag
#define		ADJ_HALL_FLAG			(0x0025 + E2POFST)
#define		ADJ_GYRO_FLAG			(0x0027 + E2POFST)
#define		ADJ_LENS_FLAG			(0x0029 + E2POFST)
// Gyro GAIN 
#define		GYRO_GAIN_X				(0x002B + E2POFST)		// 4byte Setting
#define		GYRO_GAIN_Y				(0x002F + E2POFST)		// 4byte Setting

/****************** page ******************/
// F/W Version Information
#define		FW_VERSION_INFO			(0x0033 + E2POFST)
/////////////////////////////////////////////////////////////////////////////
#define		MSB_CHKSUM		(0x0035 + E2POFST)		// 1byte Setting
#define     LSB_CHKSUM		(0x0036 + E2POFST)		// 1byte Setting
#define		SUCCESS_OIS_CAL			(0x0040 + E2POFST)		// 1byte

// EEPROM Address Define END

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

#ifndef ON
 #define	ON			0x01		// ON
 #define	OFF			0x00		// OFF
#endif
 #define	SPC			0x02		// Special Mode
#define		X_DIR		0x00		// X Direction
#define		Y_DIR		0x01		// Y Direction

#define		CLR_FRAM0	0x01
#define		CLR_FRAM1 	0x02
#define		CLR_ALL_RAM 0x03

#define		Mlnp		0			// Linear PWM
#define		Mpwm		1			// PWM

/* OIS Calibration Parameter */
 #define	DAHLXO_INI		0x0000		// Hall X Offset
 #define	DAHLXB_INI		0xF000		// Hall X Bias
 #define	DAHLYO_INI		0x0000		// Hall Y Offset
 #define	DAHLYB_INI		0xF000		// Hall X Bias
 #define	HXOFF0Z_INI		0x0000		// Hall X AD Offset
 #define	HYOFF1Z_INI		0x0000		// Hall Y AD Offset
 #define	SXGAIN_INI		0x4000		// Hall X Loop Gain		//20141014 Komori
 #define	SYGAIN_INI		0x4000		// Hall Y Loop Gain
 #define	DGYRO_OFST_XH	0x00		// Digital Gyro X AD Offset H
 #define	DGYRO_OFST_XL	0x00		// Digital Gyro X AD Offset L
 #define	DGYRO_OFST_YH	0x00		// Digital Gyro Y AD Offset H
 #define	DGYRO_OFST_YL	0x00		// Digital Gyro Y AD Offset L
//#ifdef	CATCHMODE
// #define	GXGAIN_INI		0xBF147AE1	// Gyro X Zoom Value	-0.58
// #define	GYGAIN_INI		0xBF147AE1	// Gyro Y Zoom Value	-0.58
//#else	//CATCHMODE
 #define	GXGAIN_INI		0xBEA8F5C0	// Gyro X Zoom Value	-0.33
 #define	GYGAIN_INI		0xBE999980	// Gyro Y Zoom Value	-0.3
//#endif	//CATCHMODE

/* Actuator Select */
// TDK Actuator for Huawei		//20141014 Komori
#define		BIAS_CUR_OIS	0x22		// 1.0mA/1.0mA
#define		AMP_GAIN_X		0x02		// x50
#define		AMP_GAIN_Y		0x02		// x50
#define		OSC_INI			0x2E		/* OSC Init, VDD=2.8V */

/* AF Open parameter */
// TDK Actuator for Huawei		//20141016 Komori
#define		RWEXD1_L_AF		0x7FFF	//0x0DDA      //0x7FFF		//
#define		RWEXD2_L_AF		0x6ACE  //0x3e4e		//0x5A00
#define		RWEXD3_L_AF		0x7000	//0x1762  	//0x7FFF		//0x7000
#define		FSTCTIME_AF		0x96	//0xd6        //0xd4		//0x5F
#define		FSTMODE_AF		0x00	//
#define		LTHDHL_AF		0x00A0

/* AF adjust parameter */
#define		BIAS_CUR_AF		0x00		//0.25mA
#define		AMP_GAIN_AF		0x00		//x6

#define		GYROX_INI		0x43	// Gyro X axis select
#define		GYROY_INI		0x45	// Gyro Y axis select

#define		STANDBY_MODE			// STANDBY Mode
#define		GAIN_CONT				// Gain Control Mode

#define		H1COEF_CHANGER			/* H1 coef lvl chage */

#ifdef	CATCHMODE
	#define		DIFIL_S2		0x3F7FFE00
#else	//CATCHMODE
	#define		DIFIL_S2		0x3F7FFD80
#endif	//CATCHMODE

// TDK Actuator for Huawei		//20141014 Komori
#define		SXQ_INI			0x3F800000	
#define		SYQ_INI			0x3F800000

#define AFMODE_DIRECT	0x00	// DIRECT MODE ON (FAST MODE OFF)
#define AFMODE_FAST		0x04	// FAST MODE ON
#define AFMODE_AUTO		0x0C	// FAST MODE ON & AUTO PARAMETER

#define		INI_SHORT1     // BURST WRITING Define
#define		INI_SHORT2
#define		INI_SHORT3

// Define According To Usage
/************************ Description of Define *************************/
/*	GAIN_CONT			Use Gain control								*/
/*		(disable)		Use Tripod Mode									*/
/*	INIT_PWMMODE		Select Driver mode PWM or CVL-PWM				*/
/*							PWMMOD_CVL -> Use CVL-PWM MODE				*/
/*	SXQ_INI, SYQ_INI	Check Hall Filter polarity						*/
/************************************************************************/
/* Driver Setting */
#define		PWMMOD_CVL	0x00					// CVL PWM MODE
#define		PWMMOD_PWM	0x01					// PWM MODE
#define		INIT_PWMMODE	PWMMOD_CVL			/* PWM/CVL MODE select */
#define		PWM_BREAK							// PWM mode select (disable zero cross)

/* Select Gyro Sensor */
//#define	FS_SEL		0		/* 700LSB  /DPS  */
//#define	FS_SEL		1		/* 350LSB  /DPS  */
#define		FS_SEL		2		/* 175LSB  /DPS  */
//#define	FS_SEL		3		/* 87.5LSB /DPS  */

#ifdef	CATCHMODE

/************* Wide *************/

 #define		GYRLMT3_S1_W	0x3EE66640		//0.45F
 #define		GYRLMT3_S2_W	0x3EE66640		//0.45F
 //#define		GYRLMT3_S1_W	0x3F0CCCCD		//0.55F
 //#define		GYRLMT3_S2_W	0x3F0CCCCD		//0.55F
 #define		GYRLMT4_S1_W	0x40300000		//2.75F
 #define		GYRLMT4_S2_W	0x40300000		//2.75F
 #define		GYRISTP_W		0x3A51B700		/* -62dB */
 #define		GYRA12_HGH_W	0x402CCCCD		/* 2.70F */
 #define		GYRA34_HGH_W	0x4019999A		/* 2.4	 */
 
 /************* Narrow *************/
 #define		GYRLMT3_S1		0x3ECCCCCD		//0.40F
 #define		GYRLMT3_S2		0x3ECCCCCD		//0.40F
 #define		GYRLMT4_S1		0x40000000		//2.0F
 #define		GYRLMT4_S2		0x40000000		//2.0F
 #define		GYRISTP			0x39D1B700		/* -68dB */
 #define		GYRA12_HGH		0x3FE00000		/* 1.75F */
 #define		GYRA34_HGH		0x3F99999A		/* 1.2	 */
 /**********************************/
 #define		GYRLMT1H		0x3F800000		//1.0F

 #define		GYRA12_MID		0x3C23D70A		/* 0.01F */
 #define		GYRA34_MID		0x3C23D70A		/* 0.01F */

 #define		GYRB12_HGH		0x3E4CCCCD		/* 0.20F */
 #define		GYRB12_MID		0x3CA3D70A		/* 0.02F */
 #define        GYRB34_HGH      0x3F800000      /* 1.0F */
 #define        GYRB34_MID      0x3DCCCCCD      /* 0.1F */

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

#ifdef	STANDBY_MODE
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
#endif	//STANDBY_MODE

struct STHALREG {
	unsigned short	UsRegAdd ;
	unsigned char	UcRegDat ;
} ;												// Hall Register Data Table

struct STHALFIL {
	unsigned short	UsRamAdd ;
	unsigned short	UsRamDat ;
} ;												// Hall Filter Coefficient Table

struct STGYRFIL {
	unsigned short	UsRamAdd ;
	unsigned long	UlRamDat ;
} ;												// Gyro Filter Coefficient Table

typedef struct STCALDAT {

	struct {

		unsigned short	UsHlxGan ;				// Hall Gain Value
		unsigned short	UsHlyGan ;				// Hall Gain Value
		unsigned short	UsHlxOff ;				// Hall Offset Value
		unsigned short	UsHlyOff ;				// Hall Offset Value

	} StHalAdj ;

	struct {
		unsigned short	UsLxgVal ;				// Loop Gain X
		unsigned short	UsLygVal ;				// Loop Gain Y
	} StLopGan ;

	struct {
		unsigned short	UsLsxVal ;				// Lens Center X
		unsigned short	UsLsyVal ;				// Lens Center Y
	} StLenCen ;
	
	struct {
		unsigned short	UsGxoVal ;				// Gyro A/D Offset X
		unsigned short	UsGyoVal ;				// Gyro A/D Offset Y
	} StGvcOff ;

	unsigned char		UcOscVal ;				// OSC value

	unsigned short		UsAdjHallF ;			// Adjustment Completion Flag
	unsigned short		UsAdjGyroF ;			// Adjustment Completion Flag
	unsigned short		UsAdjLensF ;			// Adjustment Completion Flag

	unsigned long		UlGxgVal ;				// Gyro Zoom Coefficient Value
	unsigned long		UlGygVal ;				// Gyro Zoom Coefficient Value

	//Hall Limiter Add by Bamtol.Lee at 2015.02.02
	unsigned long		UlHlxLmt;
	unsigned long		UlHlyLmt;
	//END at 2015.02.02 02

	unsigned short		UsVerDat ;				// Version Information

} stCalDat ;

struct STFILREG {
	unsigned short	UsRegAdd ;
	unsigned char	UcRegDat ;
} ;												// Register Data Table

struct STFILRAM {
	unsigned short	UsRamAdd ;
	unsigned long	UlRamDat ;
} ;												// Filter Coefficient Table

/*** caution [little-endian] ***/

// Float Data Union
#if 0//we no use
union	FLTVAL {
	float			SfFltVal ;
	unsigned long	UlLngVal ;
	unsigned short	UsDwdVal[ 2 ] ;
	struct {
		unsigned short	UsLowVal ;
		unsigned short	UsHigVal ;
	} StFltVal ;
} ;

typedef union FLTVAL	UnFltVal ;
#endif


K7_OISINI__ unsigned char	UcOscAdjFlg ;			// For Measure trigger

K7_OISINI__ unsigned short	UsCntXof ;				/* OPTICAL Center Xvalue */
K7_OISINI__ unsigned short	UsCntYof ;				/* OPTICAL Center Yvalue */

K7_OISINI__ unsigned char	UcPwmMod ;				/* PWM MODE */
K7_OISINI__ unsigned char	UcCvrCod ;				/* CverCode LC898122A = 0xA1, LC898122 = 0x93*/	
K7_OISINI__ stCalDat		StCalDat ;				// Execute Command Parameter

K7_OISINI__ unsigned char	UcVerHig;				// System Version
K7_OISINI__ unsigned char	UcVerLow;				// Filter Version

K7_OISINI__ int    OnsemiI2CCheck(void);
K7_OISINI__ int	IniSet( void ) ;				// Initial Top Function
K7_OISINI__ void	E2pDat( void ) ;				// Read calibration data from E2PROM
K7_OISINI__ int		VerInf( void ) ;				// Version Check
K7_OISINI__ void	IniClk( void ) ;				// Clock Setting
K7_OISINI__ void	IniIop( void ) ;				// I/O Port Initial Setting
K7_OISINI__ void	IniMon( void ) ;				// Monitor & Other Initial Setting
K7_OISINI__ void	IniSrv( void ) ;				// Servo Register Initial Setting
K7_OISINI__ void	IniGyr( void ) ;				// Gyro Filter Register Initial Setting
K7_OISINI__ int	IniFil( void ) ;
K7_OISINI__ void	IniAdj( void ) ;				// Adjust Fix Value Setting
K7_OISINI__ int	IniDgy( void ) ;				// Digital Gyro Initial Setting
K7_OISINI__ void	GyrCon( unsigned char ) ;								// Gyro Filter Control
K7_OISINI__ int	ClrGyr( unsigned short, unsigned char );					// Clear Gyro RAM

K7_OISINI__ void	WitTim( unsigned short ) ;								// Wait
K7_OISINI__ void	MemClr( unsigned char *, unsigned short ) ;				// Memory Clear Function
K7_OISINI__ int	AccWit( unsigned char ) ;								// Acc Wait Function
K7_OISINI__ void	AutoGainContIni( void );
K7_OISINI__ void	AutoGainControlSw( unsigned char ) ;					// Auto Gain Control Sw
#ifdef	CATCHMODE
K7_OISINI__ void	StartUpGainContIni( void );
K7_OISINI__ unsigned char	InitGainControl( unsigned char );
K7_OISINI__ void	SetDCoffsetContValue( unsigned char UcSelRange ) ;
#endif	//CATCHMODE

K7_OISINI__ void	DrvSw( unsigned char UcDrvSw ) ;						// Driver Mode setting function

K7_OISINI__ void			SrvCon( unsigned char, unsigned char ) ;					// Servo ON/OFF
K7_OISINI__ unsigned char	RtnCen( unsigned char ) ;									// Return to Center Function
K7_OISINI__ void			OisEna( void ) ;											// OIS Enable Function
K7_OISINI__ void			OisEnaLin( void ) ;											// OIS Enable Function for Line adjustment
K7_OISINI__ void			OisOff( void ) ;											// Ois Off
K7_OISINI__ void 			StbOnn( void ) ;								// Servo ON Slope mode
K7_OISINI__ void			SetPanTiltMode( unsigned char ) ;							/* Pan_Tilt control Function */
K7_OISINI__ void	RamAccFixMod( unsigned char ) ;							// Ram Access Fix Mode setting function	
K7_OISINI__ void	IniPtAve( void ) ;
K7_OISINI__ void	IniPtMovMod( unsigned char ) ;							// Pan/Tilt parameter setting by mode function
K7_OISINI__ void	SelectPtRange( unsigned char ) ;
K7_OISINI__ void	SelectIstpMod( unsigned char ) ;
K7_OISINI__ void	IniAf( void ) ;			// Open AF Initial Setting
K7_OISINI__ void	AfDrvSw( unsigned char  ) ;						// AF Driver Mode setting function
K7_OISINI__ void	AfVcmMod( unsigned char );						// AF Mode Write
K7_OISINI__ void	AfVcmCod( unsigned short );						// AF Code Write

K7_OISINI__ void			S2cPro( unsigned char ) ;									// S2 Command Process Function
K7_OISINI__ void			SetGcf( unsigned char ) ;									// Set DI filter coefficient Function
K7_OISINI__ unsigned long	UlH1Coefval ;		// H1 coefficient value

#ifdef H1COEF_CHANGER
 K7_OISINI__	unsigned char	UcH1LvlMod ;		// H1 level coef mode
 K7_OISINI__	void			SetH1cMod( unsigned char ) ;							// Set H1C coefficient Level chang Function
 #define		S2MODE		0x40
 #define		ACTMODE		0x80
 
 #ifdef	CATCHMODE
  #define		MOVMODE		0xFE
  #define		MOVMODE_W	0xFF
  #define		STILLMODE	0x00
  #define		STILLMODE_W	0x01
 #else	//CATCHMODE
  #define		MOVMODE		0xFF
  #define		STILLMODE	0x00
 #endif  //CATCHMODE
 
#ifdef	CATCHMODE
  #define		MAXLMT_W		0x40400000				// 3.0
  #define		MINLMT_W		0x4019999A				// 2.4
//  #define		CHGCOEF_W		0xBB480000				// -50dB
  #define		CHGCOEF_W		0xB9D1B717				// -68dB
//  #define		CHGCOEF_W		0xB9855555				// -72dB
  #define		MINLMT_MOV_W	0x4019999A				// 2.4
  #define		CHGCOEF_MOV_W	0xB9200000 
  #define		MAXLMT			0x40000000				// 2.0
  #define		MINLMT			0x3F99999A				// 1.2
  #define		CHGCOEF			0xB9480000				// -74.3dB
  #define		MINLMT_MOV		0x3F99999A				// 1.2
  #define		CHGCOEF_MOV		0xB8F00000				// -78.8dB
#else	//CATCHMODE
 #define		MAXLMT		0x4091B359			//  4.553  pm 1.200
 #define		MINLMT		0x00000000			//  0.000  pm 0.000
 #define		CHGCOEF		0xB6A8ACB9			// -0.000005
 
 #define		MAXLMT_MOV	0x40A9FBE7			//  5.312  pm 1.400
 #define		MINLMT_MOV	0x00000000			//  0.000  pm 0.000
 #define		CHGCOEF_MOV	0xB7B4B90F			// -0.000022
#endif	//CATCHMODE

#endif	//H1COEF_CHANGER

#ifdef	STANDBY_MODE
K7_OISINI__ void	SelectGySleep( unsigned char ) ;						// Select Gyro Mode Function
K7_OISINI__ void	SetStandby( unsigned char ) ;							// Standby control
K7_OISINI__ void	GyOutSignal( void ) ;									// Slect Gyro Output signal Function
K7_OISINI__ void	GyOutSignalCont( void ) ;								// Slect Gyro Output Continuos Function
#endif	//STANDBY_MODE

K7_OISINI__ short	GenMesMobile( unsigned short, unsigned char );
K7_OISINI__ int		TneGvcMobile( void );
//K7_OISINI__ void	power_off( void ) ;
//K7_OISINI__ void		E2pRed( unsigned short, unsigned char, unsigned char * ) ;	// E2P ROM Data Read
//K7_OISINI__ void		E2pWrt( unsigned short, unsigned char, unsigned char * ) ;	// E2P ROM Data Write

K7_OISINI__	void	GyroEllipseLmt( unsigned char );							// Gyro Ellipse limit


																				
/*******************************************************************************
 *  OisDef.h - Header file for LC898122
 *
 *  ON Semiconductor
 *
 *  REVISION:
 *      2013/01/07 - First Edition, Y.Shigeoka
 ******************************************************************************/

//==============================================================================
//PWM Register
//==============================================================================
//#define							0x0000
#define		DRVFC				0x0001
#define		DRVFC2				0x0002
#define		DRVSELX				0x0003
#define		DRVSELY				0x0004
#define		DRVCH1SEL			0x0005
#define		DRVCH2SEL			0x0006
//#define							0x0007
//#define							0x0008
//#define							0x0009
//#define							0x000A
//#define							0x000B
//#define							0x000C
//#define							0x000D
//#define							0x000E
//#define							0x000F
#define		PWMA				0x0010
#define		PWMFC				0x0011
#define		PWMDLYX				0x0012
#define		PWMDLYY				0x0013
#define		PWMDLYTIMX			0x0014
#define		PWMDLYTIMY			0x0015
#define		PWMFC2				0x0016
//#define							0x0017
#define		PWMPERIODX			0x0018	// none (122)
#define		PWMPERIODX2			0x0019	// none (122)
#define		PWMPERIODY			0x001A	// PWMPERIODX (122)
#define		PWMPERIODY2			0x001B	// PWMPERIODY (122)
#define		STROBEFC			0x001C
#define		STROBEDLYX			0x001D
#define		STROBEDLYY			0x001E
//#define							0x001F
#define		CVA					0x0020
#define		CVFC				0x0021
#define		CVFC2				0x0022
#define		CVSMTHX				0x0023
#define		CVSMTHY				0x0024
//#define							0x0025
//#define							0x0026
//#define							0x0027
//#define							0x0028
//#define							0x0029
//#define							0x002A
//#define							0x002B
//#define							0x002C
//#define							0x002D
//#define							0x002E
//#define							0x002F
#define		PWMMONA				0x0030
#define		PWMMONFC			0x0031
#define		DACMONFC			0x0032
//#define							0x0033
//#define							0x0034
//#define							0x0035
//#define							0x0036
//#define							0x0037
//#define							0x0038
//#define							0x0039
//#define							0x003A
//#define							0x003B
//#define							0x003C
//#define							0x003D
//#define							0x003E
//#define							0x003F
#define		DACSLVADD			0x0040
#define		DACMSTCODE			0x0041
#define		DACFSCKRATE			0x0042
#define		DACHSCKRATE			0x0043
#define		DACI2CFC			0x0044
#define		DACI2CA				0x0045
//#define							0x0046
//#define							0x0047
//#define							0x0048
//#define							0x0049
//#define							0x004A
//#define							0x004B
//#define							0x004C
//#define							0x004D
//#define							0x004E
//#define							0x004F
//#define							0x0050
//#define							0x0051
//#define							0x0052
//#define							0x0053
//#define							0x0054
//#define							0x0055
//#define							0x0056
//#define							0x0057
//#define							0x0058
//#define							0x0059
//#define							0x005A
//#define							0x005B
//#define							0x005C
//#define							0x005D
//#define							0x005E
//#define							0x005F
//#define							0x0060
//#define							0x0061
//#define							0x0062
//#define							0x0063
//#define							0x0064
//#define							0x0065
//#define							0x0066
//#define							0x0067
//#define							0x0068
//#define							0x0069
//#define							0x006A
//#define							0x006B
//#define							0x006C
//#define							0x006D
//#define							0x006E
//#define							0x006F
//#define							0x0070
//#define							0x0071
//#define							0x0072
//#define							0x0073
//#define							0x0074
//#define							0x0075
//#define							0x0076
//#define							0x0077
//#define							0x0078
//#define							0x0079
//#define							0x007A
//#define							0x007B
//#define							0x007C
//#define							0x007D
//#define							0x007E
//#define							0x007F
//#define							0x0080
#define		DRVFCAF				0x0081
#define		DRVFC2AF			0x0082
#define		DRVFC3AF			0x0083
#define		DRVFC4AF			0x0084
#define		DRVCH3SEL			0x0085
//#define							0x0086
//#define							0x0087
#define		AFFC				0x0088
//#define							0x0089
//#define							0x008A
//#define							0x008B
//#define							0x008C
//#define							0x008D
//#define							0x008E
//#define							0x008F
#define		PWMAAF				0x0090
#define		PWMFCAF				0x0091
#define		PWMDLYAF			0x0092
#define		PWMDLYTIMAF			0x0093
//#define							0x0094
//#define							0x0095
//#define							0x0096
//#define							0x0097
//#define							0x0098
#define		PWMPERIODAF			0x0099
//#define							0x009A
//#define							0x009B
//#define							0x009C
//#define							0x009D
//#define							0x009E
//#define							0x009F
#define		CCAAF				0x00A0
#define		CCFCAF				0x00A1
//#define							0x00A2
//#define							0x00A3
//#define							0x00A4
//#define							0x00A5
//#define							0x00A6
//#define							0x00A7
//#define							0x00A8
//#define							0x00A9
//#define							0x00AA
//#define							0x00AB
//#define							0x00AC
//#define							0x00AD
//#define							0x00AE
//#define							0x00AF
//#define							0x00B0
//#define							0x00B1
//#define							0x00B2
//#define							0x00B3
//#define							0x00B4
//#define							0x00B5
//#define							0x00B6
//#define							0x00B7
//#define							0x00B8
//#define							0x00B9
//#define							0x00BA
//#define							0x00BB
//#define							0x00BC
//#define							0x00BD
//#define							0x00BE
//#define							0x00BF
//#define							0x00C0
//#define							0x00C1
//#define							0x00C2
//#define							0x00C3
//#define							0x00C4
//#define							0x00C5
//#define							0x00C6
//#define							0x00C7
//#define							0x00C8
//#define							0x00C9
//#define							0x00CA
//#define							0x00CB
//#define							0x00CC
//#define							0x00CD
//#define							0x00CE
//#define							0x00CF
//#define							0x00D0
//#define							0x00D1
//#define							0x00D2
//#define							0x00D3
//#define							0x00D4
//#define							0x00D5
//#define							0x00D6
//#define							0x00D7
//#define							0x00D8
//#define							0x00D9
//#define							0x00DA
//#define							0x00DB
//#define							0x00DC
//#define							0x00DD
//#define							0x00DE
//#define							0x00DF
//#define							0x00E0
//#define							0x00E1
//#define							0x00E2
//#define							0x00E3
//#define							0x00E4
//#define							0x00E5
//#define							0x00E6
//#define							0x00E7
//#define							0x00E8
//#define							0x00E9
//#define							0x00EA
//#define							0x00EB
//#define							0x00EC
//#define							0x00ED
//#define							0x00EE
//#define							0x00EF
//#define							0x00F0
//#define							0x00F1
//#define							0x00F2
//#define							0x00F3
//#define							0x00F4
//#define							0x00F5
//#define							0x00F6
//#define							0x00F7
//#define							0x00F8
//#define							0x00F9
//#define							0x00FA
//#define							0x00FB
//#define							0x00FC
//#define							0x00FD
//#define							0x00FE
//#define							0x00FF


//==============================================================================
//Filter Register
//==============================================================================
//#define							0x0100
#define		WC_EQON				0x0101
#define		WC_RAMINITON		0x0102
#define		WC_CPUOPEON			0x0103
#define		WC_VMON				0x0104
#define		WC_DPON				0x0105
//#define							0x0106
#define		WG_SHTON			0x0107
#define		WG_ADJGANGO			0x0108
#define		WG_PANON			0x0109
#define		WG_PANSTT6			0x010A
#define		WG_NPANSTFRC		0x010B
#define		WG_CNTPICGO			0x010C
#define		WG_NPANINION		0x010D
#define		WG_NPANSTOFF		0x010E
//#define							0x010F
#define		WG_EQSW				0x0110
#define		WG_DWNSMP1			0x0111
#define		WG_DWNSMP2			0x0112
#define		WG_DWNSMP3			0x0113
#define		WG_DWNSMP4			0x0114
//#define							0x0115
#define		WG_SHTMOD			0x0116
#define		WG_SHTDLYTMR		0x0117
#define		WG_LMT3MOD			0x0118
#define		WG_VREFADD			0x0119
//#define							0x011A
#define		WG_HCHR				0x011B
#define		WG_GADSMP			0x011C
//#define							0x011D
//#define							0x011E
//#define							0x011F
#define		WG_LEVADD			0x0120
#define		WG_LEVTMRLOW		0x0121
#define		WG_LEVTMRHGH		0x0122
#define		WG_LEVTMR			0x0123
//#define							0x0124
//#define							0x0125
//#define							0x0126
//#define							0x0127
#define		WG_ADJGANADD		0x0128
#define		WG_ADJGANGXATO		0x0129
#define		WG_ADJGANGYATO		0x012A
//#define							0x012B
//#define							0x012C
//#define							0x012D
//#define							0x012E
//#define							0x012F
#define		WG_PANADDA			0x0130
#define		WG_PANADDB			0x0131
#define		WG_PANTRSON0		0x0132
#define		WG_PANLEVABS		0x0133
#define		WG_PANSTT1DWNSMP0	0x0134
#define		WG_PANSTT1DWNSMP1	0x0135
#define		WG_PANSTT2DWNSMP0	0x0136
#define		WG_PANSTT2DWNSMP1	0x0137
#define		WG_PANSTT3DWNSMP0	0x0138
#define		WG_PANSTT3DWNSMP1	0x0139
#define		WG_PANSTT4DWNSMP0	0x013A
#define		WG_PANSTT4DWNSMP1	0x013B
#define		WG_PANSTT2TMR0		0x013C
#define		WG_PANSTT2TMR1		0x013D
#define		WG_PANSTT4TMR0		0x013E
#define		WG_PANSTT4TMR1		0x013F
#define		WG_PANSTT21JUG0		0x0140
#define		WG_PANSTT21JUG1		0x0141
#define		WG_PANSTT31JUG0		0x0142
#define		WG_PANSTT31JUG1		0x0143
#define		WG_PANSTT41JUG0		0x0144
#define		WG_PANSTT41JUG1		0x0145
#define		WG_PANSTT12JUG0		0x0146
#define		WG_PANSTT12JUG1		0x0147
#define		WG_PANSTT13JUG0		0x0148
#define		WG_PANSTT13JUG1		0x0149
#define		WG_PANSTT23JUG0		0x014A
#define		WG_PANSTT23JUG1		0x014B
#define		WG_PANSTT43JUG0		0x014C
#define		WG_PANSTT43JUG1		0x014D
#define		WG_PANSTT34JUG0		0x014E
#define		WG_PANSTT34JUG1		0x014F
#define		WG_PANSTT24JUG0		0x0150
#define		WG_PANSTT24JUG1		0x0151
#define		WG_PANSTT42JUG0		0x0152
#define		WG_PANSTT42JUG1		0x0153
#define		WG_PANSTTSETGYRO	0x0154
#define		WG_PANSTTSETGAIN	0x0155
#define		WG_PANSTTSETISTP	0x0156
#define		WG_PANSTTSETIFTR	0x0157
#define		WG_PANSTTSETLFTR	0x0158
//#define							0x0159
#define		WG_PANSTTXXXTH		0x015A
#define		WG_PANSTT1LEVTMR	0x015B
#define		WG_PANSTT2LEVTMR	0x015C
#define		WG_PANSTT3LEVTMR	0x015D
#define		WG_PANSTT4LEVTMR	0x015E
#define		WG_PANSTTSETILHLD	0x015F
#define		WG_STT3MOD			0x0160
#define		WG_STILMOD			0x0161
#define		WG_PLAYON			0x0162
#define		WG_NPANJ2DWNSMP		0x0163
#define		WG_NPANTST0			0x0164
#define		WG_NPANDWNSMP		0x0165
#define		WG_NPANST3RTMR		0x0166
#define		WG_NPANST12BTMR		0x0167
#define		WG_NPANST12TMRX		0x0168
#define		WG_NPANST12TMRY		0x0169
#define		WG_NPANST3TMRX		0x016A
#define		WG_NPANST3TMRY		0x016B
#define		WG_NPANST4TMRX		0x016C
#define		WG_NPANST4TMRY		0x016D
#define		WG_NPANFUN			0x016E
#define		WG_NPANINITMR		0x016F
#define		WH_EQSWX			0x0170
#define		WH_EQSWY			0x0171
	#define		EQSINSW				0x3C
#define		WH_DWNSMP1			0x0172
#define		WH_G2SDLY			0x0173
#define		WH_HOFCON			0x0174
//#define							0x0175
//#define							0x0176
//#define							0x0177
#define		WH_EMGSTPON			0x0178
//#define							0x0179
#define		WH_EMGSTPTMR		0x017A
//#define							0x017B
#define		WH_SMTSRVON			0x017C
#define		WH_SMTSRVSMP		0x017D
#define		WH_SMTTMR			0x017E
//#define							0x017F
#define		WC_SINON			0x0180
#define		WC_SINFRQ0			0x0181
#define		WC_SINFRQ1			0x0182
#define		WC_SINPHSX			0x0183
#define		WC_SINPHSY			0x0184
//#define							0x0185
//#define							0x0186
//#define							0x0187
#define		WC_ADMODE			0x0188
//#define							0x0189
#define		WC_CPUOPE1ADD		0x018A
#define		WC_CPUOPE2ADD		0x018B
#define		WC_RAMACCMOD		0x018C
#define		WC_RAMACCXY			0x018D
#define		WC_RAMDLYMOD0		0x018E
#define		WC_RAMDLYMOD1		0x018F
#define		WC_MESMODE			0x0190
#define		WC_MESSINMODE		0x0191
#define		WC_MESLOOP0			0x0192
#define		WC_MESLOOP1			0x0193
#define		WC_MES1ADD0			0x0194
#define		WC_MES1ADD1			0x0195
#define		WC_MES2ADD0			0x0196
#define		WC_MES2ADD1			0x0197
#define		WC_MESABS			0x0198
#define		WC_MESWAIT			0x0199
//#define							0x019A
//#define							0x019B
//#define							0x019C
#define		RC_MESST			0x019D
#define		RC_MESLOOP0			0x019E
#define		RC_MESLOOP1			0x019F
#define		WC_AMJMODE			0x01A0
#define		WC_AMJDF			0x01A1
#define		WC_AMJLOOP0			0x01A2
#define		WC_AMJLOOP1			0x01A3
#define		WC_AMJIDL0			0x01A4
#define		WC_AMJIDL1			0x01A5
#define		WC_AMJ1ADD0			0x01A6
#define		WC_AMJ1ADD1			0x01A7
#define		WC_AMJ2ADD0			0x01A8
#define		WC_AMJ2ADD1			0x01A9
//#define							0x01AA
//#define							0x01AB
#define		RC_AMJST			0x01AC
#define		RC_AMJERROR			0x01AD
#define		RC_AMJLOOP0			0x01AE
#define		RC_AMJLOOP1			0x01AF
#define		WC_DPI1ADD0			0x01B0
#define		WC_DPI1ADD1			0x01B1
#define		WC_DPI2ADD0			0x01B2
#define		WC_DPI2ADD1			0x01B3
#define		WC_DPI3ADD0			0x01B4
#define		WC_DPI3ADD1			0x01B5
#define		WC_DPI4ADD0			0x01B6
#define		WC_DPI4ADD1			0x01B7
#define		WC_DPO1ADD0			0x01B8
#define		WC_DPO1ADD1			0x01B9
#define		WC_DPO2ADD0			0x01BA
#define		WC_DPO2ADD1			0x01BB
#define		WC_DPO3ADD0			0x01BC
#define		WC_DPO3ADD1			0x01BD
#define		WC_DPO4ADD0			0x01BE
#define		WC_DPO4ADD1			0x01BF
#define		WC_PINMON1			0x01C0
#define		WC_PINMON2			0x01C1
#define		WC_PINMON3			0x01C2
#define		WC_PINMON4			0x01C3
#define		WC_DLYMON10			0x01C4
#define		WC_DLYMON11			0x01C5
#define		WC_DLYMON20			0x01C6
#define		WC_DLYMON21			0x01C7
#define		WC_DLYMON30			0x01C8
#define		WC_DLYMON31			0x01C9
#define		WC_DLYMON40			0x01CA
#define		WC_DLYMON41			0x01CB
//#define							0x01CC
//#define							0x01CD
#define		WC_INTMSK			0x01CE
//#define							0x01CF
#define		WC_FRCAD			0x01D0
#define		WC_FRCADEN			0x01D1
#define		WC_ADRES			0x01D2
#define		WC_TSTMON			0x01D3
#define		WC_RAMACCTM0		0x01D4
#define		WC_RAMACCTM1		0x01D5
//#define							0x01D6
//#define							0x01D7
//#define							0x01D8
//#define							0x01D9
//#define							0x01DA
//#define							0x01DB
//#define							0x01DC
//#define							0x01DD
//#define							0x01DE
//#define							0x01DF
#define		WC_EQSW				0x01E0
#define		WC_STPMV			0x01E1
#define		WC_STPMVMOD			0x01E2
#define		WC_DWNSMP1			0x01E3
#define		WC_DWNSMP2			0x01E4
#define		WC_DWNSMP3			0x01E5
#define		WC_LEVTMP			0x01E6
#define		WC_DIFTMP			0x01E7
#define		WC_L10				0x01E8
#define		WC_L11				0x01E9
//#define							0x01EA
//#define							0x01EB
//#define							0x01EC
//#define							0x01ED
//#define							0x01EE
//#define							0x01EF
#define		RG_XPANFIL			0x01F0
#define		RG_YPANFIL			0x01F1
#define		RG_XPANRAW			0x01F2
#define		RG_YPANRAW			0x01F3
#define		RG_LEVJUGE			0x01F4
#define		RG_NXPANST			0x01F5
#define		RC_RAMACC			0x01F6
#define		RH_EMLEV			0x01F7
#define		RH_SMTSRVSTT		0x01F8
#define		RC_CNTPIC			0x01F9
#define		RC_LEVDIF			0x01FA
//#define							0x01FB
//#define							0x01FC
//#define							0x01FD
#define		RC_FLG0				0x01FE
#define		RC_INT				0x01FF


//==============================================================================
//System Register
//==============================================================================
//#define							0x0200
//#define							0x0201
//#define							0x0202
//#define							0x0203
//#define							0x0204
//#define							0x0205
//#define							0x0206
//#define							0x0207
//#define							0x0208
//#define							0x0209
#define		CLKTST				0x020A
#define		CLKON				0x020B
#define		CLKSEL				0x020C
//#define							0x020D
//#define							0x020E
//#define							0x020F
#define		PWMDIV				0x0210
#define		SRVDIV				0x0211
#define		GIFDIV				0x0212
#define		AFPWMDIV			0x0213
#define		OPAFDIV				0x0214
//#define							0x0215
//#define							0x0216
//#define							0x0217
//#define							0x0218
//#define							0x0219
//#define							0x021A
//#define							0x021B
//#define							0x021C
//#define							0x021D
//#define							0x021E
//#define							0x021F
#define		P0LEV				0x0220
#define		P0DIR				0x0221
#define		P0PON				0x0222
#define		P0PUD				0x0223
//#define							0x0224
//#define							0x0225
//#define							0x0226
//#define							0x0227
//#define							0x0228
//#define							0x0229
//#define							0x022A
//#define							0x022B
//#define							0x022C
//#define							0x022D
//#define							0x022E
//#define							0x022F
#define		IOP0SEL				0x0230
#define		IOP1SEL				0x0231
#define		IOP2SEL				0x0232
#define		IOP3SEL				0x0233
#define		IOP4SEL				0x0234
#define		IOP5SEL				0x0235
#define		DGINSEL				0x0236
//#define							0x0237
#define		IOP_CNT				0x0238
#define		OUT56MON			0x0239
//#define							0x023A
//#define							0x023B
//#define							0x023C
//#define							0x023D
//#define							0x023E
//#define							0x023F
#define		BSYSEL				0x0240
//#define							0x0241
//#define							0x0242
//#define							0x0243
//#define							0x0244
//#define							0x0245
//#define							0x0246
//#define							0x0247
#define		I2CSEL				0x0248
#define		DLMODE				0x0249
//#define							0x024A
//#define							0x024B
//#define							0x024C
//#define							0x024D
#define		TSTREG0				0x024E
#define		TSTREG1				0x024F
#define		STBB0				0x0250
#define		CMSDAC0				0x0251
#define		CMSDAC1				0x0252
#define		OPGSEL0				0x0253
#define		OPGSEL1				0x0254
#define		OPGSEL2				0x0255
#define		OSCSTOP				0x0256
#define		OSCSET				0x0257
#define		OSCCNTEN			0x0258
#define		LDO_C_SET			0x0259
#define		VGA_SW0				0x025A
#define		VGA_SW1				0x025B
#define		RSTRLSCNTL			0x025C
#define		RSTRLSCNTH			0x025D
#define		OSCCK_CNTR0			0x025E
#define		OSCCK_CNTR1			0x025F
#define		EXTCNTEN			0x0260
#define		EXTCLKLOW			0x0261
#define		ADCTEST				0x0262
#define		LDSTB				0x0263
#define		STBB1				0x0264
//#define							0x0265
//#define							0x0266
//#define							0x0267
//#define							0x0268
//#define							0x0269
//#define							0x026A
//#define							0x026B
//#define							0x026C
//#define							0x026D
//#define							0x026E
//#define							0x026F
#define		MONSELA				0x0270
#define		MONSELB				0x0271
#define		MONSELC				0x0272
#define		MONSELD				0x0273
#define		CmMonTst			0x0274
//#define							0x0275
//#define							0x0276
//#define							0x0277
#define		SOFTRES1			0x0278
#define		SOFTRES2			0x0279
//#define							0x027A
//#define							0x027B
//#define							0x027C
//#define							0x027D
#define		CVER				0x027E
#define		TESTRD				0x027F


//==============================================================================
//Digital Gyro I/F Register
//==============================================================================
#define		GRSEL				0x0280
#define		GRINI				0x0281
	#define		SLOWMODE			0x04			/* 0:4MHz	1:1MHz	*/
#define		GRACC				0x0282
#define		GRADR0				0x0283
#define		GRADR1				0x0284
#define		GRADR2				0x0285
#define		GRADR3				0x0286
#define		GRADR4				0x0287
#define		GRADR5				0x0288
#define		GRADR6				0x0289
#define		GSETDT				0x028A
#define		RDSEL				0x028B
#define		REVB7				0x028C
#define		LSBF				0x028D
#define		PANAM				0x028E
#define		SPIM				0x028F
#define		GRDAT0H				0x0290
#define		GRDAT0L				0x0291
#define		GRDAT1H				0x0292
#define		GRDAT1L				0x0293
#define		GRDAT2H				0x0294
#define		GRDAT2L				0x0295
#define		GRDAT3H				0x0296
#define		GRDAT3L				0x0297
#define		GRDAT4H				0x0298
#define		GRDAT4L				0x0299
#define		GRDAT5H				0x029A
#define		GRDAT5L				0x029B
#define		GRDAT6H				0x029C
#define		GRDAT6L				0x029D
//#define							0x029E
//#define							0x029F
#define		IZAH				0x02A0
#define		IZAL				0x02A1
#define		IZBH				0x02A2
#define		IZBL				0x02A3
//#define							0x02A4
//#define							0x02A5
//#define							0x02A6
//#define							0x02A7
//#define							0x02A8
//#define							0x02A9
//#define							0x02AA
//#define							0x02AB
//#define							0x02AC
//#define							0x02AD
//#define							0x02AE
//#define							0x02AF
//#define							0x02B0
//#define							0x02B1
//#define							0x02B2
//#define							0x02B3
//#define							0x02B4
//#define							0x02B5
//#define							0x02B6
//#define							0x02B7
#define		GRFLG0				0x02B8
#define		GRFLG1				0x02B9
//#define							0x02BA
//#define							0x02BB
//#define							0x02BC
//#define							0x02BD
//#define							0x02BE
//#define							0x02BF
//#define							0x02C0
#define		DGSTAT0				0x02C1
#define		DGSTAT1				0x02C2
//#define							0x02C3
//#define							0x02C4
//#define							0x02C5
//#define							0x02C6
//#define							0x02C7
//#define							0x02C8
//#define							0x02C9
//#define							0x02CA
//#define							0x02CB
//#define							0x02CC
//#define							0x02CD
//#define							0x02CE
//#define							0x02CF
//#define							0x02D0
//#define							0x02D1
//#define							0x02D2
//#define							0x02D3
//#define							0x02D4
//#define							0x02D5
//#define							0x02D6
//#define							0x02D7
//#define							0x02D8
//#define							0x02D9
//#define							0x02DA
//#define							0x02DB
//#define							0x02DC
//#define							0x02DD
//#define							0x02DE
//#define							0x02DF
//#define							0x02E0
//#define							0x02E1
//#define							0x02E2
//#define							0x02E3
//#define							0x02E4
//#define							0x02E5
//#define							0x02E6
//#define							0x02E7
//#define							0x02E8
//#define							0x02E9
//#define							0x02EA
//#define							0x02EB
//#define							0x02EC
//#define							0x02ED
//#define							0x02EE
//#define							0x02EF
//#define							0x02F0
//#define							0x02F1
//#define							0x02F2
//#define							0x02F3
//#define							0x02F4
//#define							0x02F5
//#define							0x02F6
//#define							0x02F7
//#define							0x02F8
//#define							0x02F9
//#define							0x02FA
//#define							0x02FB
//#define							0x02FC
//#define							0x02FD
//#define							0x02FE
//#define							0x02FF


//==============================================================================
//Open AF Register
//==============================================================================
//#define							0x0300
//#define							0x0301
#define		FSTMODE				0x0302
#define		FSTCTIME			0x0303
#define		TCODEH				0x0304
#define		TCODEL				0x0305
#define		LTHDH				0x0306
#define		LTHDL				0x0307
//#define							0x0308
//#define							0x0309
//#define							0x030A
//#define							0x030B
//#define							0x030C
//#define							0x030D
//#define							0x030E
//#define							0x030F
#define		FSTOPTION			0x0310
//#define							0x0311
//#define							0x0312
//#define							0x0313
//#define							0x0314
//#define							0x0315
//#define							0x0316
//#define							0x0317
//#define							0x0318
//#define							0x0319
//#define							0x031A
//#define							0x031B
//#define							0x031C
//#define							0x031D
//#define							0x031E
//#define							0x031F
#define		OPAFEN				0x0320
//#define							0x0321
//#define							0x0322
//#define							0x0323
//#define							0x0324
//#define							0x0325
//#define							0x0326
//#define							0x0327
//#define							0x0328
//#define							0x0329
//#define							0x032A
//#define							0x032B
//#define							0x032C
//#define							0x032D
//#define							0x032E
//#define							0x032F
#define		OPAFSW				0x0330
//#define							0x0331
//#define							0x0332
//#define							0x0333
//#define							0x0334
#define		OPAFST				0x0335

#define		TREG_H				0x0380
#define		TREG_L				0x0381
//#define							0x0382
//#define							0x0383
//#define							0x0384
//#define							0x0385
//#define							0x0386
//#define							0x0387
//#define							0x0388
//#define							0x0389
//#define							0x038A
//#define							0x038B
//#define							0x038C
//#define							0x038D
//#define							0x038E
//#define							0x038F
//#define							0x0390
//#define							0x0391
//#define							0x0392
//#define							0x0393
//#define							0x0394
//#define							0x0395
#define		RWEXD1_L			0x0396		// 2Byte access
//#define							0x0397
#define		RWEXD2_L			0x0398		// 2Byte access
//#define							0x0399
#define		RWEXD3_L			0x039A		// 2Byte access
//#define							0x039B
//#define							0x039C
//#define							0x039D
//#define							0x039E
//#define							0x039F

//==============================================================================
//FILTER COEFFICIENT RAM
//==============================================================================
#define		gx45g				0x1000
#define		gx45x				0x1001
#define		gx45y				0x1002
#define		gxgyro				0x1003
#define		gxia				0x1004
#define		gxib				0x1005
#define		gxic				0x1006
#define		gxggain				0x1007
#define		gxigain				0x1008
#define		gxggain2			0x1009
#define		gx2x4xf				0x100A
#define		gxadj				0x100B
#define		gxgain				0x100C
#define		gxl3				0x100D
#define		gxhc_tmp			0x100E
#define		npxlev1				0x100F
#define		gxh1a				0x1010
#define		gxh1b				0x1011
#define		gxh1c				0x1012
#define		gxh2a				0x1013
#define		gxh2b				0x1014
#define		gxh2c				0x1015
#define		gxh3a				0x1016
#define		gxh3b				0x1017
#define		gxh3c				0x1018
#define		gxla				0x1019
#define		gxlb				0x101A
#define		gxlc				0x101B
#define		gxhgain				0x101C
#define		gxlgain				0x101D
#define		gxigainstp			0x101E
#define		npxlev2				0x101F
#define		gxzoom				0x1020
#define		gx2x4xb				0x1021
#define		gxlens				0x1022
#define		gxta				0x1023
#define		gxtb				0x1024
#define		gxtc				0x1025
#define		gxtd				0x1026
#define		gxte				0x1027
#define		gxlmt1H				0x1028
#define		gxlmt3HS0			0x1029
#define		gxlmt3HS1			0x102A
#define		gxlmt4HS0			0x102B
#define		gxlmt4HS1			0x102C
#define		gxlmt6L				0x102D
#define		gxlmt6H				0x102E
#define		npxlev3				0x102F
#define		gxj1a				0x1030
#define		gxj1b				0x1031
#define		gxj1c				0x1032
#define		gxj2a				0x1033
#define		gxj2b				0x1034
#define		gxj2c				0x1035
#define		gxk1a				0x1036
#define		gxk1b				0x1037
#define		gxk1c				0x1038
#define		gxk2a				0x1039
#define		gxk2b				0x103A
#define		gxk2c				0x103B
#define		gxoa				0x103C
#define		gxob				0x103D
#define		gxoc				0x103E
#define		npxlev4				0x103F
#define		MSABS1				0x1040
#define		MSABS1AV			0x1041
#define		MSPP1AV				0x1042
#define		gxia_1				0x1043
#define		gxib_1				0x1044
#define		gxic_1				0x1045
#define		gxia_a				0x1046
#define		gxib_a				0x1047
#define		gxic_a				0x1048
#define		gxia_b				0x1049
#define		gxib_b				0x104A
#define		gxic_b				0x104B
#define		gxia_c				0x104C
#define		gxib_c				0x104D
#define		gxic_c				0x104E
#define		Sttx12aM			0x104F
#define		MSMAX1				0x1050
#define		MSMAX1AV			0x1051
#define		MSCT1AV				0x1052
#define		gxla_1				0x1053
#define		gxlb_1				0x1054
#define		gxlc_1				0x1055
#define		gxla_a				0x1056
#define		gxlb_a				0x1057
#define		gxlc_a				0x1058
#define		gxla_b				0x1059
#define		gxlb_b				0x105A
#define		gxlc_b				0x105B
#define		gxla_c				0x105C
#define		gxlb_c				0x105D
#define		gxlc_c				0x105E
#define		Sttx12aH			0x105F
#define		MSMIN1				0x1060
#define		MSMIN1AV			0x1061
#define		MS1AV				0x1062
#define		gxgyro_1			0x1063
#define		gxgyro_1d			0x1064
#define		gxgyro_1u			0x1065
#define		gxgyro_a			0x1066
#define		gxgyro_2d			0x1067
#define		gxgyro_2u			0x1068
#define		gxgyro_b			0x1069
#define		gxgyro_3d			0x106A
#define		gxgyro_3u			0x106B
#define		gxgyro_c			0x106C
#define		gxgyro_4d			0x106D
#define		gxgyro_4u			0x106E
#define		Sttx12bM			0x106F
#define		HOStp				0x1070
#define		HOMin				0x1071
#define		HOMax				0x1072
#define		gxgain_1			0x1073
#define		gxgain_1d			0x1074
#define		gxgain_1u			0x1075
#define		gxgain_a			0x1076
#define		gxgain_2d			0x1077
#define		gxgain_2u			0x1078
#define		gxgain_b			0x1079
#define		gxgain_3d			0x107A
#define		gxgain_3u			0x107B
#define		gxgain_c			0x107C
#define		gxgain_4d			0x107D
#define		gxgain_4u			0x107E
#define		Sttx12bH			0x107F
#define		HBStp				0x1080
#define		HBMin				0x1081
#define		HBMax				0x1082
#define		gxistp_1			0x1083
#define		gxistp_1d			0x1084
#define		gxistp_1u			0x1085
#define		gxistp_a			0x1086
#define		gxistp_2d			0x1087
#define		gxistp_2u			0x1088
#define		gxistp_b			0x1089
#define		gxistp_3d			0x108A
#define		gxistp_3u			0x108B
#define		gxistp_c			0x108C
#define		gxistp_4d			0x108D
#define		gxistp_4u			0x108E
#define		Sttx34aM			0x108F
#define		LGStp				0x1090
#define		LGMin				0x1091
#define		LGMax				0x1092
#define		gxistp				0x1093
#define		gxadjmin			0x1094
#define		gxadjmax			0x1095
#define		gxadjdn				0x1096
#define		gxadjup				0x1097
#define		gxog3				0x1098
#define		gxog5				0x1099
#define		gxog7				0x109A
#define		npxlev8				0x109B
#define		sxlmtb1				0x109C
#define		SttxaL				0x109D
#define		SttxbL				0x109E
#define		Sttx34aH			0x109F
#define		sxlmtb2				0x10A0
#define		pxmaa				0x10A1
#define		pxmab				0x10A2
#define		pxmac				0x10A3
#define		pxmba				0x10A4
#define		pxmbb				0x10A5
#define		pxmbc				0x10A6
#define		gxma				0x10A7
#define		gxmb				0x10A8
#define		gxmc				0x10A9
#define		gxmg				0x10AA
#define		gxleva				0x10AB
#define		gxlevb				0x10AC
#define		gxlevc				0x10AD
#define		gxlevlow			0x10AE
#define		Sttx34bM			0x10AF
#define		sxria				0x10B0
#define		sxrib				0x10B1
#define		sxric				0x10B2
#define		sxinx				0x10B3
#define		sxiny				0x10B4
#define		sxggf				0x10B5
#define		sxag				0x10B6
#define		sxpr				0x10B7
#define		sxgx				0x10B8
#define		sxgy				0x10B9
#define		sxiexp3				0x10BA
#define		sxiexp2				0x10BB
#define		sxiexp1				0x10BC
#define		sxiexp0				0x10BD
#define		sxiexp				0x10BE
#define		Sttx34bH			0x10BF
#define		sxda				0x10C0
#define		sxdb				0x10C1
#define		sxdc				0x10C2
#define		sxea				0x10C3
#define		sxeb				0x10C4
#define		sxec				0x10C5
#define		sxua				0x10C6
#define		sxub				0x10C7
#define		sxuc				0x10C8
#define		sxia				0x10C9
#define		sxib				0x10CA
#define		sxic				0x10CB
#define		sxja				0x10CC
#define		sxjb				0x10CD
#define		sxjc				0x10CE
#define		npxlev1_i			0x10CF
#define		sxfa				0x10D0
#define		sxfb				0x10D1
#define		sxfc				0x10D2
#define		sxg					0x10D3
#define		sxg2				0x10D4
#define		sxsin				0x10D5
#define		sxggf_tmp			0x10D6
#define		sxsa				0x10D7
#define		sxsb				0x10D8
#define		sxsc				0x10D9
#define		sxoa				0x10DA
#define		sxob				0x10DB
#define		sxoc				0x10DC
#define		sxod				0x10DD
#define		sxoe				0x10DE
#define		npxlev2_i			0x10DF
#define		sxpa				0x10E0
#define		sxpb				0x10E1
#define		sxpc				0x10E2
#define		sxpd				0x10E3
#define		sxpe				0x10E4
#define		sxq					0x10E5
#define		sxlmta1				0x10E6
#define		sxlmta2				0x10E7
#define		smxga				0x10E8
#define		smxgb				0x10E9
#define		smxa				0x10EA
#define		smxb				0x10EB
#define		sxemglev			0x10EC
#define		sxsmtav				0x10ED
#define		sxsmtstp			0x10EE
#define		npxlev3_i			0x10EF
#define		mes1aa				0x10F0
#define		mes1ab				0x10F1
#define		mes1ac				0x10F2
#define		mes1ad				0x10F3
#define		mes1ae				0x10F4
#define		mes1ba				0x10F5
#define		mes1bb				0x10F6
#define		mes1bc				0x10F7
#define		mes1bd				0x10F8
#define		mes1be				0x10F9
#define		sxoexp3				0x10FA
#define		sxoexp2				0x10FB
#define		sxoexp1				0x10FC
#define		sxoexp0				0x10FD
#define		sxoexp				0x10FE
#define		npxlev4_i			0x10FF
#define		gy45g				0x1100
#define		gy45y				0x1101
#define		gy45x				0x1102
#define		gygyro				0x1103
#define		gyia				0x1104
#define		gyib				0x1105
#define		gyic				0x1106
#define		gyggain				0x1107
#define		gyigain				0x1108
#define		gyggain2			0x1109
#define		gy2x4xf				0x110A
#define		gyadj				0x110B
#define		gygain				0x110C
#define		gyl3				0x110D
#define		gyhc_tmp			0x110E
#define		npylev1				0x110F
#define		gyh1a				0x1110
#define		gyh1b				0x1111
#define		gyh1c				0x1112
#define		gyh2a				0x1113
#define		gyh2b				0x1114
#define		gyh2c				0x1115
#define		gyh3a				0x1116
#define		gyh3b				0x1117
#define		gyh3c				0x1118
#define		gyla				0x1119
#define		gylb				0x111A
#define		gylc				0x111B
#define		gyhgain				0x111C
#define		gylgain				0x111D
#define		gyigainstp			0x111E
#define		npylev2				0x111F
#define		gyzoom				0x1120
#define		gy2x4xb				0x1121
#define		gylens				0x1122
#define		gyta				0x1123
#define		gytb				0x1124
#define		gytc				0x1125
#define		gytd				0x1126
#define		gyte				0x1127
#define		gylmt1H				0x1128
#define		gylmt3HS0			0x1129
#define		gylmt3HS1			0x112A
#define		gylmt4HS0			0x112B
#define		gylmt4HS1			0x112C
#define		gylmt6L				0x112D
#define		gylmt6H				0x112E
#define		npylev3				0x112F
#define		gyj1a				0x1130
#define		gyj1b				0x1131
#define		gyj1c				0x1132
#define		gyj2a				0x1133
#define		gyj2b				0x1134
#define		gyj2c				0x1135
#define		gyk1a				0x1136
#define		gyk1b				0x1137
#define		gyk1c				0x1138
#define		gyk2a				0x1139
#define		gyk2b				0x113A
#define		gyk2c				0x113B
#define		gyoa				0x113C
#define		gyob				0x113D
#define		gyoc				0x113E
#define		npylev4				0x113F
#define		MSABS2				0x1140
#define		MSABS2AV			0x1141
#define		MSPP2AV				0x1142
#define		gyia_1				0x1143
#define		gyib_1				0x1144
#define		gyic_1				0x1145
#define		gyia_a				0x1146
#define		gyib_a				0x1147
#define		gyic_a				0x1148
#define		gyia_b				0x1149
#define		gyib_b				0x114A
#define		gyic_b				0x114B
#define		gyia_c				0x114C
#define		gyib_c				0x114D
#define		gyic_c				0x114E
#define		Stty12aM			0x114F
#define		MSMAX2				0x1150
#define		MSMAX2AV			0x1151
#define		MSCT2AV				0x1152
#define		gyla_1				0x1153
#define		gylb_1				0x1154
#define		gylc_1				0x1155
#define		gyla_a				0x1156
#define		gylb_a				0x1157
#define		gylc_a				0x1158
#define		gyla_b				0x1159
#define		gylb_b				0x115A
#define		gylc_b				0x115B
#define		gyla_c				0x115C
#define		gylb_c				0x115D
#define		gylc_c				0x115E
#define		Stty12aH			0x115F
#define		MSMIN2				0x1160
#define		MSMIN2AV			0x1161
#define		MS2AV				0x1162
#define		gygyro_1			0x1163
#define		gygyro_1d			0x1164
#define		gygyro_1u			0x1165
#define		gygyro_a			0x1166
#define		gygyro_2d			0x1167
#define		gygyro_2u			0x1168
#define		gygyro_b			0x1169
#define		gygyro_3d			0x116A
#define		gygyro_3u			0x116B
#define		gygyro_c			0x116C
#define		gygyro_4d			0x116D
#define		gygyro_4u			0x116E
#define		Stty12bM			0x116F
#define		GGStp				0x1170
#define		GGMin				0x1171
#define		GGMax				0x1172
#define		gygain_1			0x1173
#define		gygain_1d			0x1174
#define		gygain_1u			0x1175
#define		gygain_a			0x1176
#define		gygain_2d			0x1177
#define		gygain_2u			0x1178
#define		gygain_b			0x1179
#define		gygain_3d			0x117A
#define		gygain_3u			0x117B
#define		gygain_c			0x117C
#define		gygain_4d			0x117D
#define		gygain_4u			0x117E
#define		Stty12bH			0x117F
#define		GGStp2				0x1180
#define		GGMin2				0x1181
#define		GGMax2				0x1182
#define		gyistp_1			0x1183
#define		gyistp_1d			0x1184
#define		gyistp_1u			0x1185
#define		gyistp_a			0x1186
#define		gyistp_2d			0x1187
#define		gyistp_2u			0x1188
#define		gyistp_b			0x1189
#define		gyistp_3d			0x118A
#define		gyistp_3u			0x118B
#define		gyistp_c			0x118C
#define		gyistp_4d			0x118D
#define		gyistp_4u			0x118E
#define		Stty34aM			0x118F
#define		vma					0x1190
#define		vmb					0x1191
#define		vmc					0x1192
#define		gyistp				0x1193
#define		gyadjmin			0x1194
#define		gyadjmax			0x1195
#define		gyadjdn				0x1196
#define		gyadjup				0x1197
#define		gyog3				0x1198
#define		gyog5				0x1199
#define		gyog7				0x119A
#define		npylev8				0x119B
#define		sylmtb1				0x119C
#define		SttyaL				0x119D
#define		SttybL				0x119E
#define		Stty34aH			0x119F
#define		sylmtb2				0x11A0
#define		pymaa				0x11A1
#define		pymab				0x11A2
#define		pymac				0x11A3
#define		pymba				0x11A4
#define		pymbb				0x11A5
#define		pymbc				0x11A6
#define		gyma				0x11A7
#define		gymb				0x11A8
#define		gymc				0x11A9
#define		gymg				0x11AA
#define		gyleva				0x11AB
#define		gylevb				0x11AC
#define		gylevc				0x11AD
#define		gylevlow			0x11AE
#define		Stty34bM			0x11AF
#define		syria				0x11B0
#define		syrib				0x11B1
#define		syric				0x11B2
#define		syiny				0x11B3
#define		syinx				0x11B4
#define		syggf				0x11B5
#define		syag				0x11B6
#define		sypr				0x11B7
#define		sygy				0x11B8
#define		sygx				0x11B9
#define		syiexp3				0x11BA
#define		syiexp2				0x11BB
#define		syiexp1				0x11BC
#define		syiexp0				0x11BD
#define		syiexp				0x11BE
#define		Stty34bH			0x11BF
#define		syda				0x11C0
#define		sydb				0x11C1
#define		sydc				0x11C2
#define		syea				0x11C3
#define		syeb				0x11C4
#define		syec				0x11C5
#define		syua				0x11C6
#define		syub				0x11C7
#define		syuc				0x11C8
#define		syia				0x11C9
#define		syib				0x11CA
#define		syic				0x11CB
#define		syja				0x11CC
#define		syjb				0x11CD
#define		syjc				0x11CE
#define		npylev1_i			0x11CF
#define		syfa				0x11D0
#define		syfb				0x11D1
#define		syfc				0x11D2
#define		syg					0x11D3
#define		syg2				0x11D4
#define		sysin				0x11D5
#define		syggf_tmp			0x11D6
#define		sysa				0x11D7
#define		sysb				0x11D8
#define		sysc				0x11D9
#define		syoa				0x11DA
#define		syob				0x11DB
#define		syoc				0x11DC
#define		syod				0x11DD
#define		syoe				0x11DE
#define		npylev2_i			0x11DF
#define		sypa				0x11E0
#define		sypb				0x11E1
#define		sypc				0x11E2
#define		sypd				0x11E3
#define		sype				0x11E4
#define		syq					0x11E5
#define		sylmta1				0x11E6
#define		sylmta2				0x11E7
#define		smyga				0x11E8
#define		smygb				0x11E9
#define		smya				0x11EA
#define		smyb				0x11EB
#define		syemglev			0x11EC
#define		sysmtav				0x11ED
#define		sysmtstp			0x11EE
#define		npylev3_i			0x11EF
#define		mes2aa				0x11F0
#define		mes2ab				0x11F1
#define		mes2ac				0x11F2
#define		mes2ad				0x11F3
#define		mes2ae				0x11F4
#define		mes2ba				0x11F5
#define		mes2bb				0x11F6
#define		mes2bc				0x11F7
#define		mes2bd				0x11F8
#define		mes2be				0x11F9
#define		syoexp3				0x11FA
#define		syoexp2				0x11FB
#define		syoexp1				0x11FC
#define		syoexp0				0x11FD
#define		syoexp				0x11FE
#define		npylev4_i			0x11FF
#define		afsin				0x1200
#define		afing				0x1201
#define		afstmg				0x1202
#define		afag				0x1203
#define		afda				0x1204
#define		afdb				0x1205
#define		afdc				0x1206
#define		afea				0x1207
#define		afeb				0x1208
#define		afec				0x1209
#define		afua				0x120A
#define		afub				0x120B
#define		afuc				0x120C
#define		afia				0x120D
#define		afib				0x120E
#define		afic				0x120F
#define		afja				0x1210
#define		afjb				0x1211
#define		afjc				0x1212
#define		affa				0x1213
#define		affb				0x1214
#define		affc				0x1215
#define		afg					0x1216
#define		afg2				0x1217
#define		afpa				0x1218
#define		afpb				0x1219
#define		afpc				0x121A
#define		afpd				0x121B
#define		afpe				0x121C
#define		afstma				0x121D
#define		afstmb				0x121E
#define		afstmc				0x121F
#define		aflmt				0x1220
#define		aflmt2				0x1221
#define		afssmv1				0x1222
#define		afssmv2				0x1223
#define		afsjlev				0x1224
#define		afsjdif				0x1225
#define		SttxHis				0x1226
#define		tmpa				0x1227
#define		af_cc				0x1228
#define		a_df				0x1229
#define		b_df				0x122A
#define		c_df				0x122B
#define		d_df				0x122C
#define		e_df				0x122D
#define		f_df				0x122E
#define		pi					0x122F
#define		msmean				0x1230
#define		vmlevhis			0x1231
#define		vmlev				0x1232
#define		vmtl				0x1233
#define		vmth				0x1234
#define		st1mean				0x1235
#define		st2mean				0x1236
#define		st3mean				0x1237
#define		st4mean				0x1238
#define		dm1g				0x1239
#define		dm2g				0x123A
#define		dm3g				0x123B
#define		dm4g				0x123C
#define		zero				0x123D
#define		com10				0x123E
#define		cop10				0x123F

//==============================================================================
//FILTER DELAY RAM
//==============================================================================
#define		SINXZ				0x1400
#define		GX45Z				0x1401
#define		GXINZ				0x1402
#define		GXI1Z1				0x1403
#define		GXI1Z2				0x1404
#define		GXI2Z1				0x1405
#define		GXI2Z2				0x1406
#define		GXMZ1				0x1407
#define		GXMZ2				0x1408
#define		GXIZ				0x1409
#define		GXXFZ				0x140A
#define		GXADJZ				0x140B
#define		GXGAINZ				0x140C
#define		GXLEV1Z1			0x140D
#define		GXLEV1Z2			0x140E
#define		TMPX				0x140F
#define		SXDOFFZ2			0x1410
#define		GXH1Z1				0x1411
#define		GXH1Z2				0x1412
//#define							0x1413
#define		GXH2Z1				0x1414
#define		GXH2Z2				0x1415
#define		GXLEV2Z1			0x1416
#define		GXH3Z1				0x1417
#define		GXH3Z2				0x1418
#define		GXL1Z1				0x1419
#define		GXL1Z2				0x141A
#define		GXL2Z1				0x141B
#define		GXL2Z2				0x141C
#define		GXL3Z				0x141D
#define		GXLZ				0x141E
#define		GXI3Z				0x141F
#define		GXZOOMZ				0x1420
#define		GXXBZ				0x1421
#define		GXLENSZ				0x1422
#define		GXLMT3Z				0x1423
#define		GXTZ1				0x1424
#define		GXTZ2				0x1425
#define		GXTZ3				0x1426
#define		GXTZ4				0x1427
#define		GX2SXZ				0x1428
#define		SXOVRZ				0x1429
#define		PXAMZ				0x142A
#define		PXMAZ1				0x142B
#define		PXMAZ2				0x142C
#define		PXBMZ				0x142D
#define		PXMBZ1				0x142E
#define		PXMBZ2				0x142F
#define		DAXHLOtmp			0x1430
#define		GXJ1Z1				0x1431
#define		GXJ1Z2				0x1432
#define		SXINZ1				0x1433
#define		GXJ2Z1				0x1434
#define		GXJ2Z2				0x1435
#define		SXINZ2				0x1436
#define		GXK1Z1				0x1437
#define		GXK1Z2				0x1438
#define		SXTRZ				0x1439
#define		GXK2Z1				0x143A
#define		GXK2Z2				0x143B
#define		SXIEXPZ				0x143C
#define		GXOZ1				0x143D
#define		GXOZ2				0x143E
#define		GXLEV2Z2			0x143F
#define		AD0Z				0x1440
#define		SXRIZ1				0x1441
#define		SXRIZ2				0x1442
#define		SXAGZ				0x1443
#define		SXSMTZ				0x1444
#define		MES1AZ1				0x1445
#define		MES1AZ2				0x1446
#define		MES1AZ3				0x1447
#define		MES1AZ4				0x1448
#define		SXTRZ1				0x1449
#define		AD2Z				0x144A
#define		MES1BZ1				0x144B
#define		MES1BZ2				0x144C
#define		MES1BZ3				0x144D
#define		MES1BZ4				0x144E
#define		AD4Z				0x144F
#define		OFF0Z				0x1450
#define		SXDZ1				0x1451
#define		SXDZ2				0x1452
#define		NPXDIFZ				0x1453
#define		SXEZ1				0x1454
#define		SXEZ2				0x1455
#define		SX2HXZ2				0x1456
#define		SXUZ1				0x1457
#define		SXUZ2				0x1458
#define		SXTRZ2				0x1459
#define		OFF2Z				0x145A
#define		SXIZ1				0x145B
#define		SXIZ2				0x145C
#define		SXJZ1				0x145D
#define		SXJZ2				0x145E
#define		OFF4Z				0x145F
#define		AD0OFFZ				0x1460
#define		SXOFFZ1				0x1461
#define		SXOFFZ2				0x1462
#define		SXFZ				0x1463
#define		SXGZ				0x1464
#define		NPXTMPZ				0x1465
#define		SXG3Z				0x1466
#define		SXSZ1				0x1467
#define		SXSZ2				0x1468
#define		SXTRZ3				0x1469
#define		AD2OFFZ				0x146A
#define		SXOZ1				0x146B
#define		SXOZ2				0x146C
#define		SXOZ3				0x146D
#define		SXOZ4				0x146E
#define		AD4OFFZ				0x146F
#define		SXDOFFZ				0x1470
#define		SXPZ1				0x1471
#define		SXPZ2				0x1472
#define		SXPZ3				0x1473
#define		SXPZ4				0x1474
#define		SXQZ				0x1475
#define		SXOEXPZ				0x1476
#define		SXLMT				0x1477
#define		SX2HXZ				0x1478
#define		DAXHLO				0x1479
#define		DAXHLB				0x147A
#define		TMPX2				0x147B
#define		TMPX3				0x147C
//#define							0x147D
//#define							0x147E
//#define							0x147F
#define		SINYZ				0x1480
#define		GY45Z				0x1481
#define		GYINZ				0x1482
#define		GYI1Z1				0x1483
#define		GYI1Z2				0x1484
#define		GYI2Z1				0x1485
#define		GYI2Z2				0x1486
#define		GYMZ1				0x1487
#define		GYMZ2				0x1488
#define		GYIZ				0x1489
#define		GYXFZ				0x148A
#define		GYADJZ				0x148B
#define		GYGAINZ				0x148C
#define		GYLEV1Z1			0x148D
#define		GYLEV1Z2			0x148E
#define		TMPY				0x148F
#define		SYDOFFZ2			0x1490
#define		GYH1Z1				0x1491
#define		GYH1Z2				0x1492
//#define							0x1493
#define		GYH2Z1				0x1494
#define		GYH2Z2				0x1495
#define		GYLEV2Z1			0x1496
#define		GYH3Z1				0x1497
#define		GYH3Z2				0x1498
#define		GYL1Z1				0x1499
#define		GYL1Z2				0x149A
#define		GYL2Z1				0x149B
#define		GYL2Z2				0x149C
#define		GYL3Z				0x149D
#define		GYLZ				0x149E
#define		GYI3Z				0x149F
#define		GYZOOMZ				0x14A0
#define		GYXBZ				0x14A1
#define		GYLENSZ				0x14A2
#define		GYLMT3Z				0x14A3
#define		GYTZ1				0x14A4
#define		GYTZ2				0x14A5
#define		GYTZ3				0x14A6
#define		GYTZ4				0x14A7
#define		GY2SYZ				0x14A8
#define		SYOVRZ				0x14A9
#define		PYAMZ				0x14AA
#define		PYMAZ1				0x14AB
#define		PYMAZ2				0x14AC
#define		PYBMZ				0x14AD
#define		PYMBZ1				0x14AE
#define		PYMBZ2				0x14AF
#define		DAYHLOtmp			0x14B0
#define		GYJ1Z1				0x14B1
#define		GYJ1Z2				0x14B2
#define		SYINZ1				0x14B3
#define		GYJ2Z1				0x14B4
#define		GYJ2Z2				0x14B5
#define		SYINZ2				0x14B6
#define		GYK1Z1				0x14B7
#define		GYK1Z2				0x14B8
#define		SYTRZ				0x14B9
#define		GYK2Z1				0x14BA
#define		GYK2Z2				0x14BB
#define		SYIEXPZ				0x14BC
#define		GYOZ1				0x14BD
#define		GYOZ2				0x14BE
#define		GYLEV2Z2			0x14BF
#define		AD1Z				0x14C0
#define		SYRIZ1				0x14C1
#define		SYRIZ2				0x14C2
#define		SYAGZ				0x14C3
#define		SYSMTZ				0x14C4
#define		MES2AZ1				0x14C5
#define		MES2AZ2				0x14C6
#define		MES2AZ3				0x14C7
#define		MES2AZ4				0x14C8
#define		SYTRZ1				0x14C9
#define		AD3Z				0x14CA
#define		MES2BZ1				0x14CB
#define		MES2BZ2				0x14CC
#define		MES2BZ3				0x14CD
#define		MES2BZ4				0x14CE
#define		AD5Z				0x14CF
#define		OFF1Z				0x14D0
#define		SYDZ1				0x14D1
#define		SYDZ2				0x14D2
#define		NPYDIFZ				0x14D3
#define		SYEZ1				0x14D4
#define		SYEZ2				0x14D5
#define		SY2HYZ2				0x14D6
#define		SYUZ1				0x14D7
#define		SYUZ2				0x14D8
#define		SYTRZ2				0x14D9
#define		OFF3Z				0x14DA
#define		SYIZ1				0x14DB
#define		SYIZ2				0x14DC
#define		SYJZ1				0x14DD
#define		SYJZ2				0x14DE
#define		OFF5Z				0x14DF
#define		AD1OFFZ				0x14E0
#define		SYOFFZ1				0x14E1
#define		SYOFFZ2				0x14E2
#define		SYFZ				0x14E3
#define		SYGZ				0x14E4
#define		NPYTMPZ				0x14E5
#define		SYG3Z				0x14E6
#define		SYSZ1				0x14E7
#define		SYSZ2				0x14E8
#define		SYTRZ3				0x14E9
#define		AD3OFFZ				0x14EA
#define		SYOZ1				0x14EB
#define		SYOZ2				0x14EC
#define		SYOZ3				0x14ED
#define		SYOZ4				0x14EE
#define		AD5OFFZ				0x14EF
#define		SYDOFFZ				0x14F0
#define		SYPZ1				0x14F1
#define		SYPZ2				0x14F2
#define		SYPZ3				0x14F3
#define		SYPZ4				0x14F4
#define		SYQZ				0x14F5
#define		SYOEXPZ				0x14F6
#define		SYLMT				0x14F7
#define		SY2HYZ				0x14F8
#define		DAYHLO				0x14F9
#define		DAYHLB				0x14FA
#define		TMPY2				0x14FB
#define		TMPY3				0x14FC
//#define							0x14FD
//#define							0x14FE
//#define							0x14FF
#define		AFSINZ				0x1500
#define		AFDIFTMP			0x1501
#define		AFINZ				0x1502
#define		AFINZ2				0x1503
#define		AFAGZ				0x1504
#define		AFDZ1				0x1505
#define		AFDZ2				0x1506
#define		AFSTMGTSS			0x1507
#define		AFEZ1				0x1508
#define		AFEZ2				0x1509
#define		OFSTAFZ				0x150A
#define		AFUZ1				0x150B
#define		AFUZ2				0x150C
#define		AD4OFFZ2			0x150D
#define		AFIZ1				0x150E
#define		AFIZ2				0x150F
#define		OFF6Z				0x1510
#define		AFJZ1				0x1511
#define		AFJZ2				0x1512
#define		AFSTMTGT			0x1513
#define		AFSTMSTP			0x1514
#define		AFSTMTGTtmp			0x1515
#define		AFFZ				0x1516
#define		AFGZ				0x1517
#define		AFG3Z				0x1518
#define		AFPZ1				0x1519
#define		AFPZ2				0x151A
#define		AFPZ3				0x151B
#define		AFPZ4				0x151C
#define		AFLMTZ				0x151D
#define		AF2PWM				0x151E
#define		AFSTMZ2				0x151F
#define		VMXYZ				0x1520
#define		VMZ1				0x1521
#define		VMZ2				0x1522
//#define							0x1523
#define		OAFTHL				0x1524
#define		PR					0x1525
#define		AFRATO1				0x1526
#define		ADRATO2				0x1527
#define		AFRATO3				0x1528
#define		DAZHLO				0x1529
#define		DAZHLB				0x152A
#define		AFL1Z				0x152B
#define		AFL2Z				0x152C
#define		AFDFZ				0x152D
#define		pi_L1				0x152E
#define		pi_L2				0x152F






























































// 
// //==============================================================================
// // OisDef.h Code START
// //==============================================================================
// #define         LCXFC                   0x0001
// #define         LCX1INADD               0x0002
// #define         LCX1OUTADD              0x0003
// #define         LCX2INADD               0x0004
// #define         LCX2OUTADD              0x0005
// #define         LCYFC                   0x0006
// #define         LCY1INADD               0x0007
// #define         LCY1OUTADD              0x0008
// #define         LCY2INADD               0x0009
// #define         LCY2OUTADD              0x000A
// #define         PRAJFC4                 0x000B
// #define         PRTBLX                  0x000C
// #define         GVFEN                   0x000D
// #define         GVFDS                   0x000E
// #define         GVFX                    0x000F
// #define         DASCEN                  0x0010
// #define         DASCDS                  0x0011
// #define         GYINFC2                 0x0012
// #define         LXEQFC3                 0x0013
// #define         LYEQFC3                 0x0014
// #define         AJTA2                   0x0015
// #define         PWMFRQSPX               0x0060
// #define         PWMFRQSPY               0x0061
// //#define         PWMPERIODX              0x0062
// //#define         PWMPERIODY              0x0063
// #define         STROBEFCX               0x0064
// //#define         STROBEDLYX              0x0065
// #define         STROBEFCY               0x0066
// //#define         STROBEDLYY              0x0067
// //#define         DRVFC2                  0x0068
// #define         LNFC2                   0x0069
// #define         DRIFFC                  0x006E
// #define         DRIFTIME                0x006F
// //#define 	    DRVFC                   0x0070
// //#define 	    DRVSELX                 0x0071
// //#define 	    DRVSELY                 0x0072
// //#define 	    PWMA                    0x0074
// //#define 	    PWMFC                   0x0075
// #define 	    PWMDLY1                 0x0076
// #define 	    PWMDLY2                 0x0077
// #define 	    LNA                     0x0078
// #define		    LNFC                    0x0079
// #define		    LNSMTHX                 0x007A
// #define		    LNSMTHY                 0x007B
// #define		    PCLKNUM1                0x007C
// #define		    PCLKNUM2                0x007D
// #define         GEPWMFC                 0x007E
// #define         GEPWMDLY                0x007F
// #define         ADS4ADD                 0x0080
// #define         ADS4FC                  0x0081
// #define         LSVFC1                  0x0082
// #define         LXEQFC2                 0x0083
// #define         LXEQEN                  0x0084
// #define         LXEQFC                  0x0085
// #define         HXSFT                   0x0086
// #define         LXX1                    0x0087
// #define         LXX2                    0x0088
// #define         LXX3                    0x0089
// #define         LXSC1                   0x008A
// #define         LXSC2                   0x008B
// #define         LXSC3                   0x008C
// #define         LYEQFC2                 0x008D
// #define         LYEQEN                  0x008E
// #define         LYEQFC                  0x008F
// #define         HYSFT                   0x0090
// #define         LYX1                    0x0091
// #define         LYX2                    0x0092
// #define         LYX3                    0x0093
// #define         LYSC1                   0x0094
// #define         LYSC2                   0x0095
// #define         LYSC3                   0x0096
// #define         SSSEN                   0x0097
// #define         SSSFC1                  0x0098
// #define         SSSFC2                  0x0099
// #define         SSSFC3                  0x009A
// #define         SEOEN                   0x009B
// #define         SEOFC1                  0x009C
// #define         SEOFC2                  0x009D
// #define         GNEQEN                  0x009E
// #define         GNEQFC                  0x009F
// #define         GNX1                    0x00A0
// #define         GNX2                    0x00A1
// #define         GNINADD                 0x00A2
// #define         GNOUTADD                0x00A3
// #define         GDPXFC                  0x00A4
// #define         GDPX1INADD              0x00A5
// #define         GDPX1OUTADD             0x00A6
// #define         GDPX2INADD              0x00A7
// #define         GDPX2OUTADD             0x00A8
// #define         GDPX3INADD              0x00A9
// #define         GDPX3OUTADD             0x00AA
// #define         GDPYFC                  0x00AB
// #define         GDPY1INADD              0x00AC
// #define         GDPY1OUTADD             0x00AD
// #define         GDPY2INADD              0x00AE
// #define         GDPY2OUTADD             0x00AF
// #define         GDPY3INADD              0x00B0
// #define         GDPY3OUTADD             0x00B1
// #define         FFXEN                   0x00B2
// #define         FFXFC                   0x00B3
// #define         FFXDS                   0x00B4
// #define         FFXX                    0x00B5
// #define         FFXSOF                  0x00B6
// #define         FXINADD                 0x00B7
// #define         FXOUTADD                0x00B8
// #define         FFYEN                   0x00B9
// #define         FFYFC                   0x00BA
// #define         FFYDS                   0x00BB
// #define         FFYX                    0x00BC
// #define         FFYSOF                  0x00BD
// #define         FYINADD                 0x00BE
// #define         FYOUTADD                0x00BF
// #define         MSF1EN                  0x00C0
// #define         MSF1SOF                 0x00C1
// #define         MS1INADD                0x00C2
// #define         MS1OUTADD               0x00C3
// #define         MSF2EN                  0x00C4
// #define         MSF2SOF                 0x00C5
// #define         MS2INADD                0x00C6
// #define         MS2OUTADD               0x00C7
// #define         MSFDS                   0x00C8
// #define         MSMA                    0x00C9
// #define         MSMPLNSL                0x00CA
// #define         MSMPLNSH                0x00CB
// #define         MSMPLNL                 0x00CC
// #define         MSMPLNH                 0x00CD
// #define         RTXADD                  0x00CE
// #define         AJTA                    0x00CF
// #define         AJWAIT                  0x00D0
// #define         AJTIMEOUT               0x00D1
// #define         AJSTATUS                0x00D2
// #define         AJNUM                   0x00D3
// #define         PRVFADD                 0x00D4
// #define         PRAJEN                  0x00D5
// #define         PRAJFC1                 0x00D6
// #define         PRAJFC2                 0x00D7
// #define         PRAJFC3                 0x00D8
// #define         PRAJSR                  0x00D9
// #define         GYINFC                  0x00DA
// #define         SWEN                    0x00DB
// #define         SWSR                    0x00DC
// #define         SWFC1                   0x00DD
// #define         SWFC2                   0x00DE
// #define         SWFC3                   0x00DF
// #define         SWFC4                   0x00E0
// #define         SWFC5                   0x00E1
// #define         SWSEL                   0x00E2
// #define         SINXADD                 0x00E3
// #define         SINYADD                 0x00E4
// #define         MDLY1ADD                0x00E5
// #define         MDLY2ADD                0x00E6
// #define         HLXOADD                 0x00E7
// #define         RTYADD                  0x00E8
// #define         LOGEN                   0x00E9
// #define         LOGFC1                  0x00EA
// #define         LOGFC2                  0x00EB
// #define         DCOMPFC                 0x00EC
// #define         DCOMPEN                 0x00ED
// #define         DLYCLR                  0x00EE
// #define         DLYCLR2                 0x00EF
// #define         TIMERA                  0x00F0
// #define         TIMERBL                 0x00F1
// #define         TIMERBH                 0x00F2
// #define         TIMERCTL                0x00F3
// //#define         PWMMONFC                0x00F4
// #define         DAMONFC                 0x00F5
// #define         FCSW                    0x00F6
// #define         FCSW2                   0x00F7
// #define         FLGM                    0x00F8
// #define         FLGIST                  0x00F9
// #define         FLGIM2                  0x00FA
// #define         FLGIST2                 0x00FB
// #define         SVMONA                  0x00FC	
// #define         SVMONB                  0x00FD		
// #define         SVMONC                  0x00FE	
// #define         SVMOND                  0x00FF	
// #define         GEQON                   0x0100
// #define         GEQSW                   0x0101
// #define         GSHAKEON                0x0102
// #define         GRAMINITON              0x0103
// #define         GSHTON                  0x0104
// #define         GCPUOPEON               0x0105
// #define         G2NDCEFON0              0x0106
// #define         G2NDCEFON1              0x0107
// #define         GADJGANGO               0x0108
// #define         GPANON                  0x0109
// #define         GPANSTTFRCE             0x010A
// #define         GGENHPSGO               0x010B
// #define         GGSRVOFSTGO             0x010C
// #define         GSLWGANGO               0x010D
// #define         GSLWFFCGO               0x010E
// #define         GSTPGO                  0x010F
// #define         GDWNSMP1                0x0110
// #define         GDWNSMP2                0x0111
// #define         GDWNSMP3                0x0112
// #define         GADMEANON               0x0113
// #define         GVREFADD                0x0114
// #define         GSHTMOD                 0x0115
// #define         GLMT3MOD                0x0116
// #define         GLMT3SEL                0x0117
// #define         GCPUOPE1ADD             0x0118
// #define         GCPUOPE2ADD             0x0119
// #define         GRAMACCMOD              0x011A
// #define         GRAMDLYMOD              0x011B
// #define         GGADON                  0x011C
// #define         GGADSMP0                0x011D
// #define         GGADSMP1                0x011E
// #define         GGADSMPT                0x011F
// #define         GLEVGXADD               0x0120
// #define         GLEVTMRLOWGX            0x0121
// #define         GLEVTMRMIDGX            0x0122
// #define         GLEVTMRHGHGX            0x0123
// #define         GLEVGYADD               0x0124
// #define         GLEVTMRLOWGY            0x0125
// #define         GLEVTMRMIDGY            0x0126
// #define         GLEVTMRHGHGY            0x0127
// #define         GLEVTMR                 0x0128
// #define         GLEVFILMOD              0x0129
// #define         GADJGANADD              0x012A
// #define         GADJGANGXMOD            0x012B
// #define         GADJGANGYMOD            0x012C
// #define         GADJGANLNK              0x012D
// #define         GSLWGANADD              0x012E
// #define         GSLWFFCMOD              0x012F
// #define         GPANADDA                0x0130
// #define         GPANADDB                0x0131
// #define         GPANTRSON0              0x0132
// #define         GPANTRSON1              0x0133
// #define         GPANSTT1DWNSMP0         0x0134
// #define         GPANSTT1DWNSMP1         0x0135
// #define         GPANSTT2DWNSMP0         0x0136
// #define         GPANSTT2DWNSMP1         0x0137
// #define         GPANSTT3DWNSMP0         0x0138
// #define         GPANSTT3DWNSMP1         0x0139
// #define         GPANSTT4DWNSMP0         0x013A
// #define         GPANSTT4DWNSMP1         0x013B
// #define         GPANSTT2TMR0            0x013C
// #define         GPANSTT2TMR1            0x013D
// #define         GPANSTT4TMR0            0x013E
// #define         GPANSTT4TMR1            0x013F
// #define         GPANSTT21JUG0           0x0140
// #define         GPANSTT21JUG1           0x0141
// #define         GPANSTT31JUG0           0x0142
// #define         GPANSTT31JUG1           0x0143
// #define         GPANSTT41JUG0           0x0144
// #define         GPANSTT41JUG1           0x0145
// #define         GPANSTT12JUG0           0x0146
// #define         GPANSTT12JUG1           0x0147
// #define         GPANSTT13JUG0           0x0148
// #define         GPANSTT13JUG1           0x0149
// #define         GPANSTT23JUG0           0x014A
// #define         GPANSTT23JUG1           0x014B
// #define         GPANSTT43JUG0           0x014C
// #define         GPANSTT43JUG1           0x014D
// #define         GPANSTT34JUG0           0x014E
// #define         GPANSTT34JUG1           0x014F
// #define         GPANSTT24JUG0           0x0150
// #define         GPANSTT24JUG1           0x0151
// #define         GPANSTT42JUG0           0x0152
// #define         GPANSTT42JUG1           0x0153
// #define         GPANSTTSETGYRO          0x0154
// #define         GPANSTTSETGAIN          0x0155
// #define         GPANSTTSETISTP          0x0156
// #define         GPANSTTSETI1FTR         0x0157
// #define         GPANSTTSETI2FTR         0x0158
// #define         GPANSTTSETL2FTR         0x0159
// #define         GPANSTTSETL3FTR         0x015A
// #define         GPANSTTSETL4FTR         0x015B
// #define         GPANSTTSETHPS           0x015C
// #define         GPANSTTXXXTH            0x015D
// #define         GMEANAUTO               0x015E
// #define         GPANSTT3MOD             0x015F
// #define         GPANSTT1LEVTMR          0x0160
// #define         GPANSTT2LEVTMR          0x0161
// #define         GPANSTT3LEVTMR          0x0162
// #define         GPANSTT4LEVTMR          0x0163
// #define         GPANLEVABS              0x0164
// #define         GSTILMOD                0x0165
// #define         GPLAYON                 0x0166
// #define         GPANFILMOD              0x0167
// #define         GPANSTTSETILHLD         0x0168
// #define         GHPSACT                 0x0169
// #define         GPANHPSTMR0             0x016A
// #define         GPANHPSTMR1             0x016B
// #define         GGENHPSTMR0             0x016C
// #define         GGENHPSTMR1             0x016D
// #define         GGENSETHPSTMR           0x016E
// #define         GHPSMOD                 0x016F
// #define         GDPI1ADD0               0x0170
// #define         GDPI1ADD1               0x0171
// #define         GDPO1ADD0               0x0172
// #define         GDPO1ADD1               0x0173
// #define         GDPI2ADD0               0x0174
// #define         GDPI2ADD1               0x0175
// #define         GDPO2ADD0               0x0176
// #define         GDPO2ADD1               0x0177
// #define         GPOSCOEF                0x0178
// #define         GLSEL                   0x017A
// #define         GHCHR                   0x017B
// #define         GDWNSMP4                0x017D
// #define         GPINMON1                0x0180
// #define         GPINMON2                0x0181
// #define         GPINMON3                0x0182
// #define         GPINMON4                0x0183
// #define         GDLYMON10               0x0184
// #define         GDLYMON11               0x0185
// #define         GDLYMON20               0x0186
// #define         GDLYMON21               0x0187
// #define         GDLYMON30               0x0188
// #define         GDLYMON31               0x0189
// #define         GDLYMON40               0x018A
// #define         GDLYMON41               0x018B
// #define         GINTMSK                 0x018C
// #define         GCPUINTMOD              0x018D
// #define         GSINTST                 0x018F
// #define         GFRCAD                  0x0190
// #define         GFRCADEN                0x0191
// #define         GADRES                  0x0192
// #define         GTSTGMON                0x0193
// #define         GFRAC                   0x019A
// #define         GFL2FXLMT               0x019B
// #define         GEXPLMTH                0x019C
// #define         GEXPLMTL                0x019D
// #define         GRAMACCTM0              0x019E
// #define         GRAMACCTM1              0x019F
// #define         GXPANFIL                0x01F0
// #define         GYPANFIL                0x01F1
// #define         GXPANRAW                0x01F2
// #define         GYPANRAW                0x01F3
// #define         GXPANFIL0               0x01F4
// #define         GYPANFIL0               0x01F5
// #define         GXPANRAW0               0x01F6
// #define         GYPANRAW0               0x01F7
// #define         GPANSTATE               0x01F8
// #define         GXPANLEV                0x01F9
// #define         GYPANLEV                0x01FA
// #define         GLEVJUGE                0x01FB
// #define         GHPS                    0x01FC
// #define         GRAMACC                 0x01FD
// #define         GFLG                    0x01FE
// #define         GINT                    0x01FF
// #define         CLKTST                  0x020A
// #define         CLKON                   0x020B
// #define         EEPDIV                  0x0210
// #define         SRVDIV                  0x0211
// //#define         PWMDIV                  0x0212
// #define         TSTDIV                  0x0213
// //#define         GIFDIV                  0x0214
// #define         CALDIV                  0x0215
// #define         P0LEV0                  0x0220
// #define         P0LEV1                  0x0221
// #define         P0DIR0                  0x0222
// #define         P0DIR1                  0x0223
// #define         P0PON0                  0x0224
// #define         P0PON1                  0x0225
// #define         P0PUD0                  0x0226
// #define         P0PUD1                  0x0227
// #define         IOP0SEL                 0x0230
// #define         IOP1SEL                 0x0231
// #define         IOP2SEL                 0x0232
// #define         IOP3SEL                 0x0233
// #define         IOP4SEL                 0x0234
// #define         IOP5SEL                 0x0235
// #define         IOP6SEL                 0x0236
// #define         IOP7SEL                 0x0237
// #define         IOP8SEL                 0x0238
// #define         BSYSEL                  0x0240
// #define         SPIMD3                  0x0248
// #define         TSTREG                  0x024F
// //#define         I2CSEL                  0x0250
// #define         SRMODE                  0x0251
// #define         EEPMODE                 0x0252
// #define         SASTREG                 0x0253
// #define         SASTENB                 0x0254
// #define         SASTDAT                 0x0255
// #define         SASTREV                 0x0256
// #define         AFSTREG                 0x0257
// #define         AFSTENB                 0x0258
// #define         AFSTDAT                 0x0259
// #define         AFSTREV                 0x025A
// #define         STBB                    0x0260
// #define         CMSDAC                  0x0261
// #define         OPGSEL                  0x0262
// //#define         OSCSTOP                 0x0263
// //#define         OSCSET                  0x0264
// //#define         OSCCNTEN                0x0265
// //#define         LDO_C_SET               0x0266
// #define         VGA_SET                 0x0267
// //#define         RSTRLSCNTL              0x0268
// //#define         RSTRLSCNTH              0x0269
// //#define         EXTCLKLOW               0x026A
// //#define         OSCCK_CNTR0             0x026B
// //#define         OSCCK_CNTR1             0x026C
// //#define         EXTCNTEN                0x026D
// #define         DACOFST                 0x026E
// #define         MONSELA                 0x0270
// #define         MONSELB                 0x0271
// #define         MONSELC                 0x0272
// #define         MONSELD                 0x0273
// #define         CmMonTst                0x0274
// #define         SOFRES1                 0x0278
// #define         SOFRES2                 0x0279
// #define         CVER                    0x027E
// #define         TESTRD                  0x027F
// #define         E2ACC					0x0280
// #define         E2L						0x0281
// #define         E2H						0x0282
// #define         E2DAT0					0x0283
// #define         E2DAT1					0x0284
// #define         E2DAT2					0x0285
// #define         E2DAT3					0x0286
// #define			E2SRT					0x0291
// //#define         GRSEL                   0x0380
// //#define         GRINI                   0x0381
// #define			SLOWMODE				0x04
// //#define         GRACC                   0x0382
// //#define         GRADR0                  0x0383
// //#define         GRADR1                  0x0384
// //#define         GRADR2                  0x0385
// //#define         GRADR3                  0x0386
// //#define         GRADR4                  0x0387
// //#define         GRADR5                  0x0388
// //#define         GRADR6                  0x0389
// //#define         GSETDT                  0x038A
// //#define         RDSEL                   0x038B
// //#define         REVB7                   0x038C
// #define         HPSM                    0x038D
// //#define         PANAM                   0x038E
// //#define         SPIM                    0x038F
// #define         GRADT0H                 0x0390
// #define         GRADT0L                 0x0391
// #define         GRADT1H                 0x0392
// #define         GRADT1L                 0x0393
// #define         GRADT2H                 0x0394
// #define         GRADT2L                 0x0395
// #define         GRADT3H                 0x0396
// #define         GRADT3L                 0x0397
// #define         GRADT4H                 0x0398
// #define         GRADT4L                 0x0399
// #define         GRADT5H                 0x039A
// #define         GRADT5L                 0x039B
// #define         GRADT6H                 0x039C
// #define         GRADT6L                 0x039D
// //#define         IZAH                    0x03A0
// //#define         IZAL                    0x03A1
// //#define         IZBH                    0x03A2
// //#define         IZBL                    0x03A3
// #define         GRINT                   0x03B0
// #define         HPS0AD                  0x03B1
// #define         HPS0DAT                 0x03B2
// #define         HPS1AD                  0x03B3
// #define         HPS1DAT                 0x03B4
// #define         HPS2AD                  0x03B5
// #define         HPS2DAT                 0x03B6
// //#define         GRFLG0                  0x03B8
// //#define         GRFLG1                  0x03B9
// #define         HPSWAIT                 0x03BA
// #define         SLVA                    0x03C0
// //#define         DGSTAT0                 0x03C1
// //#define         DGSTAT1                 0x03C2
// #define         BURST                   0x03D0
// #define         SVTR_100                0x1100
// #define         HXIDAT                  0x1101
// #define         ADHXOFF                 0x1102
// #define         ADHXI0                  0x1103
// #define         HYIDAT                  0x1104
// #define         ADHYOFF                 0x1105
// #define         ADHYI0                  0x1106
// #define         GYXIDAT                 0x1107
// #define         ADGXOFF                 0x1108
// #define         ADGYXI                  0x1109
// #define         GYYIDAT                 0x110A
// #define         ADGYOFF                 0x110B
// #define         ADGYYI                  0x110C
// #define         SAD4DAT                 0x110D
// #define         ADSAD4OFF               0x110E
// #define         ADSAD4                  0x110F
// #define         GYRMON1                 0x1110
// #define         GYRMON2                 0x1111
// #define         GYRMON3                 0x1112
// #define         GYRMON4                 0x1113
// #define         DAHLXO                  0x1114      
// #define         DAHLXB                  0x1115      
// #define         DAHLYO                  0x1116      
// #define         DAHLYB                  0x1117      
// #define         HLXBINI                 0x1118
// #define         HLYBINI                 0x1119
// #define         HLXBOG                  0x111A
// #define         HLYBOG                  0x111B
// #define         VFINI                   0x111C
// #define         VFN0                    0x111D
// #define         VFN1                    0x111E
// #define         VFLMT                   0x111F
// #define         HXSMSTP                 0x1120
// #define         HXINTRH1                0x1121
// #define         HXINTRH2                0x1122
// #define         HXSEPT1                 0x1123
// #define         HXSEPT2                 0x1124
// #define         HXSEPT3                 0x1125
// #define         HXDCIN                  0x1126
// #define         HXINOD                  0x1127
// #define         hxinog                  0x1128
// #define         HXIN                    0x1129
// #define         LXRIZ1                  0x112A
// #define         LXRIZ2                  0x112B
// #define         LXGZF                   0x112C
// #define         Z1LXGZF                 0x112D
// #define         Z2LXGZF                 0x112E
// #define         LXDZ1                   0x112F
// #define         LXDZ2                   0x1130
// #define         LXDZ3                   0x1131
// #define         LXDZ4                   0x1132
// #define         LXEZ1                   0x1133
// #define         LXEZ2                   0x1134
// #define         LXEZ3                   0x1135
// #define         LXEZ4                   0x1136
// #define         LXDZO                   0x1137
// #define         LXPZO                   0x1138
// #define         LXUZ1                   0x1139
// #define         LXUZ2                   0x113A
// #define         LXUZ3                   0x113B
// #define         LXUZ4                   0x113C
// #define         LXIZ1                   0x113D
// #define         LXIZ2                   0x113E
// #define         LXIZ3                   0x113F
// #define         LXIZ4                   0x1140
// #define         LXJZ1                   0x1141
// #define         LXJZ2                   0x1142
// #define         LXJZ3                   0x1143
// #define         LXJZ4                   0x1144
// #define         LXIZO                   0x1145
// #define         LXC1                    0x1146
// #define         LXC2                    0x1147
// #define         LXDX                    0x1148
// #define         LXGZB                   0x1149
// #define         LXDOBZ                  0x114A
// #define         LXFZF                   0x114B
// #define         LXSZ1                   0x114C
// #define         LXSZ2                   0x114D
// #define         LXOZ1                   0x114E
// #define         LXOZ2                   0x114F
// #define         LXOZ3                   0x1150
// #define         LXOZ4                   0x1151
// #define         LXPZ1                   0x1152
// #define         LXPZ2                   0x1153
// #define         LXPZ3                   0x1154
// #define         LXPZ4                   0x1155
// #define         LXFZB                   0x1156
// #define         LXLMT                   0x1157
// #define         LXLMT2                  0x1158
// #define         LXLMTSD                 0x1159
// #define         LXDODAT                 0x115A
// #define         PLXOFF                  0x115B
// #define         PLXDO                   0x115C
// #define         HXTMP                   0x115D
// #define         HXSMTMP                 0x115E
// #define         LXLMTTMP                0x115F
// #define         HYSMSTP                 0x1160
// #define         HYINTRH1                0x1161
// #define         HYINTRH2                0x1162
// #define         HYSEPT1                 0x1163
// #define         HYSEPT2                 0x1164
// #define         HYSEPT3                 0x1165
// #define         HYDCIN                  0x1166
// #define         HYINOD                  0x1167
// #define         hyinog                  0x1168
// #define         HYIN                    0x1169
// #define         LYRIZ1                  0x116A
// #define         LYRIZ2                  0x116B
// #define         LYGZF                   0x116C
// #define         Z1LYGZF                 0x116D
// #define         Z2LYGZF                 0x116E
// #define         LYDZ1                   0x116F
// #define         LYDZ2                   0x1170
// #define         LYDZ3                   0x1171
// #define         LYDZ4                   0x1172
// #define         LYEZ1                   0x1173
// #define         LYEZ2                   0x1174
// #define         LYEZ3                   0x1175
// #define         LYEZ4                   0x1176
// #define         LYDZO                   0x1177
// #define         LYPZO                   0x1178
// #define         LYUZ1                   0x1179
// #define         LYUZ2                   0x117A
// #define         LYUZ3                   0x117B
// #define         LYUZ4                   0x117C
// #define         LYIZ1                   0x117D
// #define         LYIZ2                   0x117E
// #define         LYIZ3                   0x117F
// #define         LYIZ4                   0x1180
// #define         LYJZ1                   0x1181
// #define         LYJZ2                   0x1182
// #define         LYJZ3                   0x1183
// #define         LYJZ4                   0x1184
// #define         LYIZO                   0x1185
// #define         LYC1                    0x1186
// #define         LYC2                    0x1187
// #define         LYDX                    0x1188
// #define         LYGZB                   0x1189
// #define         LYDOBZ                  0x118A
// #define         LYFZF                   0x118B
// #define         LYSZ1                   0x118C
// #define         LYSZ2                   0x118D
// #define         LYOZ1                   0x118E
// #define         LYOZ2                   0x118F
// #define         LYOZ3                   0x1190
// #define         LYOZ4                   0x1191
// #define         LYPZ1                   0x1192
// #define         LYPZ2                   0x1193
// #define         LYPZ3                   0x1194
// #define         LYPZ4                   0x1195
// #define         LYFZB                   0x1196
// #define         LYLMT                   0x1197
// #define         LYLMT2                  0x1198
// #define         LYLMTSD                 0x1199
// #define         LYDODAT                 0x119A
// #define         PLYOFF                  0x119B
// #define         PLYDO                   0x119C
// #define         HYTMP                   0x119D
// #define         HYSMTMP                 0x119E
// #define         LYLMTTMP                0x119F
// #define         GOFF                    0x11A0
// #define         GDZ1                    0x11A1
// #define         GDZ2                    0x11A2
// #define         GEZ1                    0x11A3
// #define         GEZ2                    0x11A4
// #define         GUZ1                    0x11A5
// #define         GUZ2                    0x11A6
// #define         GIZ1                    0x11A7
// #define         GIZ2                    0x11A8
// #define         GJZ1                    0x11A9
// #define         GJZ2                    0x11AA
// #define         GHDO                    0x11AB
// #define         GNLMT                   0x11AC
// #define         GC1                     0x11AD
// #define         GC2                     0x11AE
// #define         GDX                     0x11AF
// #define         GDOFFSET                0x11B0
// #define         GOZ1                    0x11B1
// #define         GOZ2                    0x11B2
// #define         FFXAZ1                  0x11B3
// #define         FFXAZ2                  0x11B4
// #define         FFXAZ3                  0x11B5
// #define         FFXAZ4                  0x11B6
// #define         FFXBZ1                  0x11B7
// #define         FFXBZ2                  0x11B8
// #define         FFXBZ3                  0x11B9
// #define         FFXBZ4                  0x11BA
// #define         FFX45Y                  0x11BB
// #define         FFY45X                  0x11BC
// #define         FFYAZ1                  0x11BD
// #define         FFYAZ2                  0x11BE
// #define         FFYAZ3                  0x11BF
// #define         FFYAZ4                  0x11C0
// #define         FFYBZ1                  0x11C1
// #define         FFYBZ2                  0x11C2
// #define         FFYBZ3                  0x11C3
// #define         FFYBZ4                  0x11C4
// #define         MS1AZ1                  0x11C5
// #define         MS1AZ2                  0x11C6
// #define         MS1AZ3                  0x11C7
// #define         MS1AZ4                  0x11C8
// #define         MS1BZ1                  0x11C9
// #define         MS1BZ2                  0x11CA
// #define         MS1BZ3                  0x11CB
// #define         MS1BZ4                  0x11CC
// #define         MS2AZ1                  0x11CD
// #define         MS2AZ2                  0x11CE
// #define         MS2AZ3                  0x11CF
// #define         MS2AZ4                  0x11D0
// #define         MS2BZ1                  0x11D1
// #define         MS2BZ2                  0x11D2
// #define         MS2BZ3                  0x11D3
// #define         MS2BZ4                  0x11D4
// #define         WAVXO                   0x11D5
// #define         WAVYO                   0x11D6
// #define         LXCFIN                  0x11D7
// #define         LYCFIN                  0x11D8
// #define         C3A                     0x11D9
// #define         C3B                     0x11DA
// #define         C3XDAT                  0x11DB
// #define         SCMP1                   0x11DC
// #define         SCMP2                   0x11DD
// #define         SCMP3                   0x11DE
// #define         SCMP4                   0x11DF
// #define         gdpx1g                  0x11E0
// #define         gdpx2g                  0x11E1
// #define         gdpx3g                  0x11E2
// #define         gdpy1g                  0x11E3
// #define         gdpy2g                  0x11E4
// #define         gdpy3g                  0x11E5
// #define         VFDZ                    0x11E6
// #define         VFMZ                    0x11E7
// #define         HLXBTMP                 0x11E8
// #define         HLYBTMP                 0x11E9
// #define         LXDOIN                  0x11EA
// #define         LYDOIN                  0x11EB
// #define         XRT                     0x11EC
// #define         YRT                     0x11ED
// #define         RDO1DAT                 0x11EE
// #define         RDO2DAT                 0x11EF
// #define         DLY00                   0x11F0
// #define         DLY01                   0x11F1
// #define         DLY02                   0x11F2
// #define         DLY03                   0x11F3
// #define         LXADOIN                 0x11F4
// #define         LYADOIN                 0x11F9
// #define         HLXOIN1                 0x11FE
// #define         HLYOIN1                 0x11FF
// #define         MSAVAC2L                0x1200
// #define         MSAVAC2H                0x1201
// #define         MSAV2                   0x1202
// #define         MSAVAC1L                0x1203
// #define         MSAVAC1H                0x1204
// #define         MSAV1                   0x1205
// #define         MSAVCNT                 0x1206
// #define         MSCMAX                  0x1207
// #define         MSCMIN                  0x1208
// #define         MSCCT                   0x1209
// #define         MSCAP                   0x120A
// #define         MSSMAX                  0x120B
// #define         MSSMIN                  0x120C
// #define         MSSCTACL                0x120D
// #define         MSSCTACH                0x120E
// #define         MSSCTAV                 0x120F
// #define         MSSAPACL                0x1210
// #define         MSSAPACH                0x1211
// #define         MSSAPAV                 0x1212
// #define         MSSMAXACL               0x1213
// #define         MSSMAXACH               0x1214
// #define         MSSMAXAV                0x1215
// #define         MSSMINACL               0x1216
// #define         MSSMINACH               0x1217
// #define         MSSMINAV                0x1218
// #define         MSSCNT                  0x1219
// #define         LXOLMT	                0x121A
// #define         LYOLMT	                0x121B
// #define         AJCTMAX                 0x121C
// #define         AJCTMIN                 0x121D
// #define         AJAPMAX                 0x121E
// #define         AJAPMIN                 0x121F
// #define         AJAVMAX                 0x1220
// #define         AJAVMIN                 0x1221
// #define         AJLPMAX                 0x1222
// #define         AJLPMIN                 0x1223
// #define         AJHOFFD                 0x1224
// #define         AJHGAIND                0x1225
// #define         AJPRGD                  0x1226
// #define         AJLPGD                  0x1227
// #define         AJCT2MAX                0x1228
// #define         AJCT2MIN                0x1229
// #define         AJAP2MAX                0x122A
// #define         AJAP2MIN                0x122B
// #define         SC3TRL                  0x122C
// #define         SC3TRH                  0x122D
// #define         SC4TRL                  0x122E
// #define         SC4TRH                  0x122F
// #define         XLOG00                  0x1230
// #define         XLOG01                  0x1231
// #define         XLOG02                  0x1232
// #define         XLOG03                  0x1233
// #define         XLOG04                  0x1234
// #define         XLOG05                  0x1235
// #define         XLOG06                  0x1236
// #define         XLOG07                  0x1237
// #define         XLOG08                  0x1238
// #define         XLOG09                  0x1239
// #define         XLOG10                  0x123A
// #define         XLOG11                  0x123B
// #define         XLOG12                  0x123C
// #define         XLOG13                  0x123D
// #define         XLOG14                  0x123E
// #define         XLOG15                  0x123F
// #define         XLOG16                  0x1240
// #define         XLOG17                  0x1241
// #define         XLOG18                  0x1242
// #define         XLOG19                  0x1243
// #define         XLOG20                  0x1244
// #define         XLOG21                  0x1245
// #define         XLOG22                  0x1246
// #define         XLOG23                  0x1247
// #define         XLOG24                  0x1248
// #define         XLOG25                  0x1249
// #define         XLOG26                  0x124A
// #define         XLOG27                  0x124B
// #define         XLOG28                  0x124C
// #define         XLOG29                  0x124D
// #define         XLOG30                  0x124E
// #define         XLOG31                  0x124F
// #define         XLOG32                  0x1250
// #define         XLOG33                  0x1251
// #define         XLOG34                  0x1252
// #define         XLOG35                  0x1253
// #define         XLOG36                  0x1254
// #define         XLOG37                  0x1255
// #define         XLOG38                  0x1256
// #define         XLOG39                  0x1257
// #define         XLOG40                  0x1258
// #define         XLOG41                  0x1259
// #define         XLOG42                  0x125A
// #define         XLOG43                  0x125B
// #define         XLOG44                  0x125C
// #define         XLOG45                  0x125D
// #define         XLOG46                  0x125E
// #define         XLOG47                  0x125F
// #define         YLOG00                  0x1260
// #define         YLOG01                  0x1261
// #define         YLOG02                  0x1262
// #define         YLOG03                  0x1263
// #define         YLOG04                  0x1264
// #define         YLOG05                  0x1265
// #define         YLOG06                  0x1266
// #define         YLOG07                  0x1267
// #define         YLOG08                  0x1268
// #define         YLOG09                  0x1269
// #define         YLOG10                  0x126A
// #define         YLOG11                  0x126B
// #define         YLOG12                  0x126C
// #define         YLOG13                  0x126D
// #define         YLOG14                  0x126E
// #define         YLOG15                  0x126F
// #define         YLOG16                  0x1270
// #define         YLOG17                  0x1271
// #define         YLOG18                  0x1272
// #define         YLOG19                  0x1273
// #define         YLOG20                  0x1274
// #define         YLOG21                  0x1275
// #define         YLOG22                  0x1276
// #define         YLOG23                  0x1277
// #define         YLOG24                  0x1278
// #define         YLOG25                  0x1279
// #define         YLOG26                  0x127A
// #define         YLOG27                  0x127B
// #define         YLOG28                  0x127C
// #define         YLOG29                  0x127D
// #define         YLOG30                  0x127E
// #define         YLOG31                  0x127F
// #define         YLOG32                  0x1280
// #define         YLOG33                  0x1281
// #define         YLOG34                  0x1282
// #define         YLOG35                  0x1283
// #define         YLOG36                  0x1284
// #define         YLOG37                  0x1285
// #define         YLOG38                  0x1286
// #define         YLOG39                  0x1287
// #define         YLOG40                  0x1288
// #define         YLOG41                  0x1289
// #define         YLOG42                  0x128A
// #define         YLOG43                  0x128B
// #define         YLOG44                  0x128C
// #define         YLOG45                  0x128D
// #define         YLOG46                  0x128E
// #define         YLOG47                  0x128F
// #define         WAVD00                  0x1290
// #define         WAVD01                  0x1291
// #define         WAVD02                  0x1292
// #define         WAVD03                  0x1293
// #define         WAVD04                  0x1294
// #define         WAVD05                  0x1295
// #define         WAVD06                  0x1296
// #define         WAVD07                  0x1297
// #define         WAVD08                  0x1298
// #define         WAVD09                  0x1299
// #define         WAVD10                  0x129A
// #define         WAVD11                  0x129B
// #define         WAVD12                  0x129C
// #define         WAVD13                  0x129D
// #define         WAVD14                  0x129E
// #define         WAVD15                  0x129F
// #define         WAVD16                  0x12A0
// #define         WAVD17                  0x12A1
// #define         WAVD18                  0x12A2
// #define         WAVD19                  0x12A3
// #define         WAVD20                  0x12A4
// #define         WAVD21                  0x12A5
// #define         WAVD22                  0x12A6
// #define         WAVD23                  0x12A7
// #define         WAVD24                  0x12A8
// #define         MSCMAX2                 0x12A9
// #define         MSCMIN2                 0x12AA
// #define         MSCCT2                  0x12AB
// #define         MSCAP2                  0x12AC
// #define         XAP                     0x12AD
// #define         YAP                     0x12AE
// #define         IOROCX00                0x12B0
// #define         IOROCX01                0x12B1
// #define         IOROCX02                0x12B2
// #define         IOROCX03                0x12B3
// #define         IOROCX04                0x12B4
// #define         IOROCX05                0x12B5
// #define         IOROCX06                0x12B6
// #define         IOROCX07                0x12B7
// #define         IOROCX08                0x12B8
// #define         IOROCX09                0x12B9
// #define         IOROCX0A                0x12BA
// #define         IOROCX0B                0x12BB
// #define         IOROCX0C                0x12BC
// #define         IOROCX0D                0x12BD
// #define         IOROCX0E                0x12BE
// #define         IOROCX0F                0x12BF
// #define         IOROCY00                0x12C0
// #define         IOROCY01                0x12C1
// #define         IOROCY02                0x12C2
// #define         IOROCY03                0x12C3
// #define         IOROCY04                0x12C4
// #define         IOROCY05                0x12C5
// #define         IOROCY06                0x12C6
// #define         IOROCY07                0x12C7
// #define         IOROCY08                0x12C8
// #define         IOROCY09                0x12C9
// #define         IOROCY0A                0x12CA
// #define         IOROCY0B                0x12CB
// #define         IOROCY0C                0x12CC
// #define         IOROCY0D                0x12CD
// #define         IOROCY0E                0x12CE
// #define         IOROCY0F                0x12CF
// #define         LXSEOLMT                0x12D0
// #define         LYSEOLMT                0x12D1
// #define         LCX1A0                  0x12D2
// #define         LCX1A1                  0x12D3
// #define         LCX1A2                  0x12D4
// #define         LCX1A3                  0x12D5
// #define         LCX1A4                  0x12D6
// #define         LCX1A5                  0x12D7
// #define         LCX1A6                  0x12D8
// #define         LCX2A0                  0x12D9
// #define         LCX2A1                  0x12DA
// #define         LCX2A2                  0x12DB
// #define         LCX2A3                  0x12DC
// #define         LCX2A4                  0x12DD
// #define         LCX2A5                  0x12DE
// #define         LCX2A6                  0x12DF
// #define         PZXPCNTD                0x12F0
// #define         PZYPCNTD                0x12F1
// #define         LCY1A0                  0x12F2
// #define         LCY1A1                  0x12F3
// #define         LCY1A2                  0x12F4
// #define         LCY1A3                  0x12F5
// #define         LCY1A4                  0x12F6
// #define         LCY1A5                  0x12F7
// #define         LCY1A6                  0x12F8
// #define         LCY2A0                  0x12F9
// #define         LCY2A1                  0x12FA
// #define         LCY2A2                  0x12FB
// #define         LCY2A3                  0x12FC
// #define         LCY2A4                  0x12FD
// #define         LCY2A5                  0x12FE
// #define         LCY2A6                  0x12FF
// #define         SVTR_300                0x1300
// #define         hx0g                    0x1301
// #define         hxslp1                  0x1302
// #define         hxslp2                  0x1303
// #define         hxslp3                  0x1304
// #define         lxria                   0x1305
// #define         lxrib                   0x1306
// #define         lxiric                  0x1307
// #define         lxggf                   0x1308
// #define         lxag                    0x1309
// #define         lxdgh                   0x130A
// #define         lxpgh                   0x130B
// #define         lxdgi                   0x130C
// #define         lxda                    0x130D
// #define         lxdb                    0x130E
// #define         lxdc                    0x130F
// #define         lxdd                    0x1310
// #define         lxde                    0x1311
// #define         lxea                    0x1312
// #define         lxeb                    0x1313
// #define         lxec                    0x1314
// #define         lxed                    0x1315
// #define         lxee                    0x1316
// #define         lxdgo                   0x1317
// #define         lxpgi                   0x1318
// #define         lxpgo                   0x1319
// #define         lxua                    0x131A
// #define         lxub                    0x131B
// #define         lxuc                    0x131C
// #define         lxud                    0x131D
// #define         lxue                    0x131E
// #define         lxia                    0x131F
// #define         lxib                    0x1320
// #define         lxic                    0x1321
// #define         lxid                    0x1322
// #define         lxie                    0x1323
// #define         lxja                    0x1324
// #define         lxjb                    0x1325
// #define         lxjc                    0x1326
// #define         lxjd                    0x1327
// #define         lxje                    0x1328
// #define         lxigo                   0x1329
// #define         lxgain                  0x132A  
// #define         lxxg                    0x132B
// #define         lxggb                   0x132C
// #define         lxdobg                  0x132D
// #define         lxfgf                   0x132E
// #define         lxsa                    0x132F
// #define         lxsb                    0x1330
// #define         lxsc                    0x1331
// #define         lxoa                    0x1332
// #define         lxob                    0x1333
// #define         lxoc                    0x1334
// #define         lxod                    0x1335
// #define         lxoe                    0x1336
// #define         lxpa                    0x1337
// #define         lxpb                    0x1338
// #define         lxpc                    0x1339
// #define         lxpd                    0x133A
// #define         lxpe                    0x133B
// #define         pzgxp                   0x133C
// #define         pzgxm                   0x133D
// #define         lxfgb                   0x133E
// #define         plxg                    0x133F
// #define         c3sin                   0x1340
// #define         hy0g                    0x1341
// #define         hyslp1                  0x1342
// #define         hyslp2                  0x1343
// #define         hyslp3                  0x1344
// #define         lyria                   0x1345
// #define         lyrib                   0x1346
// #define         lyiric                  0x1347
// #define         lyggf                   0x1348
// #define         lyag                    0x1349
// #define         lydgh                   0x134A
// #define         lypgh                   0x134B
// #define         lydgi                   0x134C
// #define         lyda                    0x134D
// #define         lydb                    0x134E
// #define         lydc                    0x134F
// #define         lydd                    0x1350
// #define         lyde                    0x1351
// #define         lyea                    0x1352
// #define         lyeb                    0x1353
// #define         lyec                    0x1354
// #define         lyed                    0x1355
// #define         lyee                    0x1356
// #define         lydgo                   0x1357
// #define         lypgi                   0x1358
// #define         lypgo                   0x1359
// #define         lyua                    0x135A
// #define         lyub                    0x135B
// #define         lyuc                    0x135C
// #define         lyud                    0x135D
// #define         lyue                    0x135E
// #define         lyia                    0x135F
// #define         lyib                    0x1360
// #define         lyic                    0x1361
// #define         lyid                    0x1362
// #define         lyie                    0x1363
// #define         lyja                    0x1364
// #define         lyjb                    0x1365
// #define         lyjc                    0x1366
// #define         lyjd                    0x1367
// #define         lyje                    0x1368
// #define         lyigo                   0x1369
// #define         lygain                  0x136A  
// #define         lyxg                    0x136B
// #define         lyggb                   0x136C
// #define         lydobg                  0x136D
// #define         lyfgf                   0x136E
// #define         lysa                    0x136F
// #define         lysb                    0x1370
// #define         lysc                    0x1371
// #define         lyoa                    0x1372
// #define         lyob                    0x1373
// #define         lyoc                    0x1374
// #define         lyod                    0x1375
// #define         lyoe                    0x1376
// #define         lypa                    0x1377
// #define         lypb                    0x1378
// #define         lypc                    0x1379
// #define         lypd                    0x137A
// #define         lype                    0x137B
// #define         pzgyp                   0x137C
// #define         pzgym                   0x137D
// #define         lyfgb                   0x137E
// #define         plyg                    0x137F
// #define         gag                     0x1380
// #define         gda                     0x1381
// #define         gdb                     0x1382
// #define         gdc                     0x1383
// #define         gea                     0x1384
// #define         geb                     0x1385
// #define         gec                     0x1386
// #define         gdg                     0x1387
// #define         gua                     0x1388
// #define         gub                     0x1389
// #define         guc                     0x138A
// #define         gia                     0x138B
// #define         gib                     0x138C
// #define         gic                     0x138D
// #define         gja                     0x138E
// #define         gjb                     0x138F
// #define         gjc                     0x1390
// #define         gngain                  0x1391
// #define         gxg                     0x1392
// #define         goa                     0x1393
// #define         gob                     0x1394
// #define         goc                     0x1395
// #define         ffxag                   0x1396
// #define         ffxaa                   0x1397
// #define         ffxab                   0x1398
// #define         ffxac                   0x1399
// #define         ffxad                   0x139A
// #define         ffxae                   0x139B
// #define         ffxba                   0x139C
// #define         ffxbb                   0x139D
// #define         ffxbc                   0x139E
// #define         ffxbd                   0x139F
// #define         ffxbe                   0x13A0
// #define         ffxgain                 0x13A1
// #define         ffrt                    0x13A2
// #define         ffyag                   0x13A3
// #define         ffyaa                   0x13A4
// #define         ffyab                   0x13A5
// #define         ffyac                   0x13A6
// #define         ffyad                   0x13A7
// #define         ffyae                   0x13A8
// #define         ffyba                   0x13A9
// #define         ffybb                   0x13AA
// #define         ffybc                   0x13AB
// #define         ffybd                   0x13AC
// #define         ffybe                   0x13AD
// #define         ffygain                 0x13AE
// #define         ms1aa                   0x13AF
// #define         ms1ab                   0x13B0
// #define         ms1ac                   0x13B1
// #define         ms1ad                   0x13B2
// #define         ms1ae                   0x13B3
// #define         ms1ba                   0x13B4
// #define         ms1bb                   0x13B5
// #define         ms1bc                   0x13B6
// #define         ms1bd                   0x13B7
// #define         ms1be                   0x13B8
// #define         ms2aa                   0x13B9
// #define         ms2ab                   0x13BA
// #define         ms2ac                   0x13BB
// #define         ms2ad                   0x13BC
// #define         ms2ae                   0x13BD
// #define         ms2ba                   0x13BE
// #define         ms2bb                   0x13BF
// #define         ms2bc                   0x13C0
// #define         ms2bd                   0x13C1
// #define         ms2be                   0x13C2
// #define         wavxg                   0x13C3
// #define         wavyg                   0x13C4
// #define         msav                    0x13C5
// #define         va                      0x13C6
// #define         varcp                   0x13C7
// #define         vakix                   0x13C8
// #define         vakiy                   0x13C9
// #define         lxggf2                  0x13CA
// #define         lxag2                   0x13CB
// #define         lxda2                   0x13CC
// #define         lxdb2                   0x13CD
// #define         lxdc2                   0x13CE
// #define         lxdd2                   0x13CF
// #define         lxde2                   0x13D0
// #define         lxea2                   0x13D1
// #define         lxeb2                   0x13D2
// #define         lxec2                   0x13D3
// #define         lxed2                   0x13D4
// #define         lxee2                   0x13D5
// #define         lxdgo2                  0x13D6
// #define         lxpgo2                  0x13D7
// #define         lxia2                   0x13D8
// #define         lxib2                   0x13D9
// #define         lxic2                   0x13DA
// #define         lxid2                   0x13DB
// #define         lxie2                   0x13DC
// #define         lxja2                   0x13DD
// #define         lxjb2                   0x13DE
// #define         lxjc2                   0x13DF
// #define         lxjd2                   0x13E0
// #define         lxje2                   0x13E1
// #define         lxigo2                  0x13E2
// #define         lxgain2                 0x13E3
// #define         lxggb2                  0x13E4
// #define         lyggf2                  0x13E5
// #define         lyag2                   0x13E6
// #define         lyda2                   0x13E7
// #define         lydb2                   0x13E8
// #define         lydc2                   0x13E9
// #define         lydd2                   0x13EA
// #define         lyde2                   0x13EB
// #define         lyea2                   0x13EC
// #define         lyeb2                   0x13ED
// #define         lyec2                   0x13EE
// #define         lyed2                   0x13EF
// #define         lyee2                   0x13F0
// #define         lydgo2                  0x13F1
// #define         lypgo2                  0x13F2
// #define         lyia2                   0x13F3
// #define         lyib2                   0x13F4
// #define         lyic2                   0x13F5
// #define         lyid2                   0x13F6
// #define         lyie2                   0x13F7
// #define         lyja2                   0x13F8
// #define         lyjb2                   0x13F9
// #define         lyjc2                   0x13FA
// #define         lyjd2                   0x13FB
// #define         lyje2                   0x13FC
// #define         lyigo2                  0x13FD
// #define         lygain2                 0x13FE
// #define         lyggb2                  0x13FF
// //#define         gxgyro                  0x1800
// #define         gxgma                   0x1801
// #define         gxgmc                   0x1802
// #define         gxi1a                   0x1803
// #define         gxi1b                   0x1804
// #define         gxi1c                   0x1805
// #define         gxgmb                   0x1806
// #define         gxigain1                0x1807
// #define         gxlmt4H0                0x1808
// #define         gxlmt4H1                0x1809
// #define         gxi2a                   0x180A
// #define         gxi2b                   0x180B
// #define         gxi2c                   0x180C
// #define         gxggain3                0x180D
// #define         gxigain3                0x180E
// #define         gxx2x4f                 0x180F
// //#define         gxadj                   0x1810
// //#define         gxgain                  0x1811
// //#define         gxh1a                   0x1812
// //#define         gxh1b                   0x1813
// //#define         gxh1c                   0x1814
// //#define         gxh2a                   0x1815
// //#define         gxh2b                   0x1816
// //#define         gxh2c                   0x1817
// #define         gxl1a                   0x1818
// #define         gxl1b                   0x1819
// #define         gxl1c                   0x181A
// #define         gxhgain1                0x181B
// #define         gxlgain1                0x181C
// #define         gxl2a                   0x181D
// #define         gxl2b                   0x181E
// #define         gxl2c                   0x181F
// #define         gxl3a                   0x1820
// #define         gxl3b                   0x1821
// #define         gxl3c                   0x1822
// #define         gxl4a                   0x1823
// #define         gxl4b                   0x1824
// #define         gxl4c                   0x1825
// #define         gxhgain2                0x1826
// #define         gxlgain2                0x1827
// //#define         gxzoom                  0x1828
// #define         gxx2x4b                 0x1829
// //#define         gxlens                  0x182A
// //#define         gx45x                   0x182B
// //#define         gx45y                   0x182C
// #define         gxlevmid                0x182D
// #define         gxlevhgh                0x182E
// #define         gxrevofst               0x182F
// //#define         gxta                    0x1830
// //#define         gxtb                    0x1831
// //#define         gxtc                    0x1832
// //#define         gxtd                    0x1833
// //#define         gxte                    0x1834
// //#define         gxistp                  0x1835
// //#define         st1mean                 0x1836
// //#define         st2mean                 0x1837
// #define         pxmean                  0x1838
// //#define         pxmaa                   0x1839
// //#define         pxmab                   0x183A
// //#define         pxmac                   0x183B
// //#define         pxmba                   0x183C
// //#define         pxmbb                   0x183D
// //#define         pxmbc                   0x183E
// //#define         SttxHis                 0x183F
// #define         gxi1a_2                 0x1840
// #define         gxi1b_2                 0x1841
// #define         gxi1c_2                 0x1842
// #define         gxi1a_1                 0x1843
// #define         gxi1b_1                 0x1844
// #define         gxi1c_1                 0x1845
// #define         gxi1a_a                 0x1846
// #define         gxi1b_a                 0x1847
// #define         gxi1c_a                 0x1848
// #define         gxi1a_b                 0x1849
// #define         gxi1b_b                 0x184A
// #define         gxi1c_b                 0x184B
// #define         gxi1a_c                 0x184C
// #define         gxi1b_c                 0x184D
// #define         gxi1c_c                 0x184E
// //#define         Sttx12aM                0x184F
// #define         gxi2a_2                 0x1850
// #define         gxi2b_2                 0x1851
// #define         gxi2c_2                 0x1852
// #define         gxi2a_1                 0x1853
// #define         gxi2b_1                 0x1854
// #define         gxi2c_1                 0x1855
// #define         gxi2a_a                 0x1856
// #define         gxi2b_a                 0x1857
// #define         gxi2c_a                 0x1858
// #define         gxi2a_b                 0x1859
// #define         gxi2b_b                 0x185A
// #define         gxi2c_b                 0x185B
// #define         gxi2a_c                 0x185C
// #define         gxi2b_c                 0x185D
// #define         gxi2c_c                 0x185E
// //#define         Sttx12aH                0x185F
// #define         gxl2a_2                 0x1860
// #define         gxl2b_2                 0x1861
// #define         gxl2c_2                 0x1862
// #define         gxl2a_1                 0x1863
// #define         gxl2b_1                 0x1864
// #define         gxl2c_1                 0x1865
// #define         gxl2a_a                 0x1866
// #define         gxl2b_a                 0x1867
// #define         gxl2c_a                 0x1868
// #define         gxl2a_b                 0x1869
// #define         gxl2b_b                 0x186A
// #define         gxl2c_b                 0x186B
// #define         gxl2a_c                 0x186C
// #define         gxl2b_c                 0x186D
// #define         gxl2c_c                 0x186E
// //#define         Sttx12bM                0x186F
// #define         gxl3a_2                 0x1870
// #define         gxl3b_2                 0x1871
// #define         gxl3c_2                 0x1872
// #define         gxl3a_1                 0x1873
// #define         gxl3b_1                 0x1874
// #define         gxl3c_1                 0x1875
// #define         gxl3a_a                 0x1876
// #define         gxl3b_a                 0x1877
// #define         gxl3c_a                 0x1878
// #define         gxl3a_b                 0x1879
// #define         gxl3b_b                 0x187A
// #define         gxl3c_b                 0x187B
// #define         gxl3a_c                 0x187C
// #define         gxl3b_c                 0x187D
// #define         gxl3c_c                 0x187E
// //#define         Sttx12bH                0x187F
// #define         gxl4a_2                 0x1880
// #define         gxl4b_2                 0x1881
// #define         gxl4c_2                 0x1882
// #define         gxl4a_1                 0x1883
// #define         gxl4b_1                 0x1884
// #define         gxl4c_1                 0x1885
// #define         gxl4a_a                 0x1886
// #define         gxl4b_a                 0x1887
// #define         gxl4c_a                 0x1888
// #define         gxl4a_b                 0x1889
// #define         gxl4b_b                 0x188A
// #define         gxl4c_b                 0x188B
// #define         gxl4a_c                 0x188C
// #define         gxl4b_c                 0x188D
// #define         gxl4c_c                 0x188E
// //#define         Sttx34aM                0x188F
// #define         gxgyro_2                0x1890
// #define         gxgain_2                0x1891
// #define         gxistp_2                0x1892
// //#define         gxgyro_1                0x1893
// //#define         gxgain_1                0x1894
// //#define         gxistp_1                0x1895
// //#define         gxgyro_a                0x1896
// //#define         gxgain_a                0x1897
// //#define         gxistp_a                0x1898
// //#define         gxgyro_b                0x1899
// //#define         gxgain_b                0x189A
// //#define         gxistp_b                0x189B
// //#define         gxgyro_c                0x189C
// //#define         gxgain_c                0x189D
// //#define         gxistp_c                0x189E
// //#define         Sttx34aH                0x189F
// #define         gxh1a_2                 0x18A0  
// #define         gxh1b_2                 0x18A1   
// #define         gxh1c_2                 0x18A2   
// #define         gxh2a_2                 0x18A3   
// #define         gxh2b_2                 0x18A4   
// #define         gxh2c_2                 0x18A5
// #define         gxl1a_2                 0x18A6  
// #define         gxl1b_2                 0x18A7   
// #define         gxl1c_2                 0x18A8  
// #define         gxgma_2                 0x18A9
// #define         gxzoom_2                0x18AA
// #define         gxlens_2                0x18AB
// #define         gdm1g                   0x18AC
// #define         gdm3g                   0x18AD
// //#define         SttxaL                  0x18AE
// //#define         Sttx34bM                0x18AF
// #define         gxlmt1L                 0x18B0
// //#define         gxlmt1H                 0x18B1
// #define         gxlmt2L                 0x18B2
// #define         gxlmt2H                 0x18B3
// #define         gxlmt3H0                0x18B4
// #define         gxlmt3H1                0x18B5
// #define         gxlmt3g                 0x18B6
// //#define         gxleva                  0x18B7
// //#define         gxlevb                  0x18B8
// #define         gxlebc                  0x18B9
// //#define         gxadjmin                0x18BA
// //#define         gxadjmax                0x18BB
// //#define         gxadjdn                 0x18BC
// //#define         gxadjup                 0x18BD
// //#define         SttxbL                  0x18BE
// //#define         Sttx34bH                0x18BF
// #define         GYADOFFZ                0x18C0
// #define         GYLMT1Z                 0x18C1
// //#define         GYINZ                   0x18C2
// #define         GYGMZ2                  0x18C3
// //#define         GYI1Z1                  0x18C4
// //#define         GYI1Z2                  0x18C5
// #define         GYI1Z                   0x18C6
// #define         GYDM1GZ                 0x18C7
// #define         GYDM2GZ                 0x18C8
// #define         GYDM3GZ                 0x18C9
// #define         GYDM4GZ                 0x18CA
// //#define         GYI2Z1                  0x18CB
// //#define         GYI2Z2                  0x18CC
// #define         SttxLtmp                0x18CD
// //#define         GYI3Z                   0x18CE
// //#define         GYXFZ                   0x18CF
// //#define         GYADJZ                  0x18D0
// //#define         GYGAINZ                 0x18D1
// #define         GYLMT2Z                 0x18D2
// //#define         GYH1Z1                  0x18D3
// //#define         GYH1Z2                  0x18D4
// #define         GYCPUZ1                 0x18D5
// //#define         GYH2Z1                  0x18D6
// //#define         GYH2Z2                  0x18D7
// #define         gxtmp                   0x18D8
// //#define         GYL1Z1                  0x18D9
// //#define         GYL1Z2                  0x18DA
// #define         GYLEV1Z                 0x18DB
// #define         GYLEV2Z                 0x18DC
// #define         SttxMtmp                0x18DD
// //#define         GYL2Z1                  0x18DE
// //#define         GYL2Z2                  0x18DF
// #define         GYOFFZ                  0x18E0
// #define         GYL3Z1                  0x18E1
// #define         GYL3Z2                  0x18E2
// #define         gxlmt3Hx                0x18E3
// #define         GYL4Z1                  0x18E4
// #define         GYL4Z2                  0x18E5
// #define         GYL4Z5                  0x18E6
// #define         GYS2Z                   0x18E7
// //#define         GYZOOMZ                 0x18E8
// //#define         GYXBZ                   0x18E9
// //#define         GYLENSZ                 0x18EA
// #define         TMPY1Z                  0x18EB
// //#define         GY45Z                   0x18EC
// #define         SttxHtmp                0x18ED
// //#define         GYLMT3Z                 0x18EE
// #define         GYREFZ                  0x18F0
// //#define         GYTZ1                   0x18F1
// //#define         GYTZ2                   0x18F2
// //#define         GYTZ3                   0x18F3
// //#define         GYTZ4                   0x18F4
// #define         GYADZ                   0x18F5
// #define         GYL4Z6                  0x18F6
// #define         GY2SXZ                  0x18F7
// #define         GYZOOMZ2                0x18F8
// //#define         PYAMZ                   0x18F9
// //#define         PYMAZ1                  0x18FA
// //#define         PYMAZ2                  0x18FB
// //#define         PYBMZ                   0x18FC
// //#define         PYMBZ1                  0x18FD
// //#define         PYMBZ2                  0x18FE
// //#define         gygyro                  0x1900
// #define         gygma                   0x1901
// #define         gygmc                   0x1902
// #define         gyi1a                   0x1903
// #define         gyi1b                   0x1904
// #define         gyi1c                   0x1905
// #define         gygmb                   0x1906
// #define         gyigain1                0x1907
// #define         gylmt4H0                0x1908
// #define         gylmt4H1                0x1909
// #define         gyi2a                   0x190A
// #define         gyi2b                   0x190B
// #define         gyi2c                   0x190C
// #define         gyggain3                0x190D
// #define         gyigain3                0x190E
// #define         gyx2x4f                 0x190F
// //#define         gyadj                   0x1910
// //#define         gygain                  0x1911
// //#define         gyh1a                   0x1912
// //#define         gyh1b                   0x1913
// //#define         gyh1c                   0x1914
// //#define         gyh2a                   0x1915
// //#define         gyh2b                   0x1916
// //#define         gyh2c                   0x1917
// #define         gyl1a                   0x1918
// #define         gyl1b                   0x1919
// #define         gyl1c                   0x191A
// #define         gyhgain1                0x191B
// #define         gylgain1                0x191C
// #define         gyl2a                   0x191D
// #define         gyl2b                   0x191E
// #define         gyl2c                   0x191F
// #define         gyl3a                   0x1920
// #define         gyl3b                   0x1921
// #define         gyl3c                   0x1922
// #define         gyl4a                   0x1923
// #define         gyl4b                   0x1924
// #define         gyl4c                   0x1925
// #define         gyhgain2                0x1926
// #define         gylgain2                0x1927
// //#define         gyzoom                  0x1928
// #define         gyx2x4b                 0x1929
// //#define         gylens                  0x192A
// //#define         gy45x                   0x192B
// //#define         gy45y                   0x192C
// #define         gylevmid                0x192D
// #define         gylevhgh                0x192E
// #define         gyrevofst               0x192F
// //#define         gyta                    0x1930
// //#define         gytb                    0x1931
// //#define         gytc                    0x1932
// //#define         gytd                    0x1933
// //#define         gyte                    0x1934
// //#define         gyistp                  0x1935
// //#define         st3mean                 0x1936
// //#define         st4mean                 0x1937
// #define         pymean                  0x1938
// //#define         pymaa                   0x1939
// //#define         pymab                   0x193A
// //#define         pymac                   0x193B
// //#define         pymba                   0x193C
// //#define         pymbb                   0x193D
// //#define         pymbc                   0x193E
// #define         SttyHis                 0x193F
// #define         gyi1a_2                 0x1940
// #define         gyi1b_2                 0x1941
// #define         gyi1c_2                 0x1942
// #define         gyi1a_1                 0x1943
// #define         gyi1b_1                 0x1944
// #define         gyi1c_1                 0x1945
// #define         gyi1a_a                 0x1946
// #define         gyi1b_a                 0x1947
// #define         gyi1c_a                 0x1948
// #define         gyi1a_b                 0x1949
// #define         gyi1b_b                 0x194A
// #define         gyi1c_b                 0x194B
// #define         gyi1a_c                 0x194C
// #define         gyi1b_c                 0x194D
// #define         gyi1c_c                 0x194E
// //#define         Stty12aM                0x194F
// #define         gyi2a_2                 0x1950
// #define         gyi2b_2                 0x1951
// #define         gyi2c_2                 0x1952
// #define         gyi2a_1                 0x1953
// #define         gyi2b_1                 0x1954
// #define         gyi2c_1                 0x1955
// #define         gyi2a_a                 0x1956
// #define         gyi2b_a                 0x1957
// #define         gyi2c_a                 0x1958
// #define         gyi2a_b                 0x1959
// #define         gyi2b_b                 0x195A
// #define         gyi2c_b                 0x195B
// #define         gyi2a_c                 0x195C
// #define         gyi2b_c                 0x195D
// #define         gyi2c_c                 0x195E
// //#define         Stty12aH                0x195F
// #define         gyl2a_2                 0x1960
// #define         gyl2b_2                 0x1961
// #define         gyl2c_2                 0x1962
// #define         gyl2a_1                 0x1963
// #define         gyl2b_1                 0x1964
// #define         gyl2c_1                 0x1965
// #define         gyl2a_a                 0x1966
// #define         gyl2b_a                 0x1967
// #define         gyl2c_a                 0x1968
// #define         gyl2a_b                 0x1969
// #define         gyl2b_b                 0x196A
// #define         gyl2c_b                 0x196B
// #define         gyl2a_c                 0x196C
// #define         gyl2b_c                 0x196D
// #define         gyl2c_c                 0x196E
// //#define         Stty12bM                0x196F
// #define         gyl3a_2                 0x1970
// #define         gyl3b_2                 0x1971
// #define         gyl3c_2                 0x1972
// #define         gyl3a_1                 0x1973
// #define         gyl3b_1                 0x1974
// #define         gyl3c_1                 0x1975
// #define         gyl3a_a                 0x1976
// #define         gyl3b_a                 0x1977
// #define         gyl3c_a                 0x1978
// #define         gyl3a_b                 0x1979
// #define         gyl3b_b                 0x197A
// #define         gyl3c_b                 0x197B
// #define         gyl3a_c                 0x197C
// #define         gyl3b_c                 0x197D
// #define         gyl3c_c                 0x197E
// //#define         Stty12bH                0x197F
// #define         gyl4a_2                 0x1980
// #define         gyl4b_2                 0x1981
// #define         gyl4c_2                 0x1982
// #define         gyl4a_1                 0x1983
// #define         gyl4b_1                 0x1984
// #define         gyl4c_1                 0x1985
// #define         gyl4a_a                 0x1986
// #define         gyl4b_a                 0x1987
// #define         gyl4c_a                 0x1988
// #define         gyl4a_b                 0x1989
// #define         gyl4b_b                 0x198A
// #define         gyl4c_b                 0x198B
// #define         gyl4a_c                 0x198C
// #define         gyl4b_c                 0x198D
// #define         gyl4c_c                 0x198E
// //#define         Stty34aM                0x198F
// #define         gygyro_2                0x1990
// #define         gygain_2                0x1991
// #define         gyistp_2                0x1992
// //#define         gygyro_1                0x1993
// //#define         gygain_1                0x1994
// //#define         gyistp_1                0x1995
// //#define         gygyro_a                0x1996
// //#define         gygain_a                0x1997
// //#define         gyistp_a                0x1998
// //#define         gygyro_b                0x1999
// //#define         gygain_b                0x199A
// //#define         gyistp_b                0x199B
// //#define         gygyro_c                0x199C
// //#define         gygain_c                0x199D
// //#define         gyistp_c                0x199E
// //#define         Stty34aH                0x199F
// #define         gyh1a_2                 0x19A0  
// #define         gyh1b_2                 0x19A1  
// #define         gyh1c_2                 0x19A2  
// #define         gyh2a_2                 0x19A3 
// #define         gyh2b_2                 0x19A4  
// #define         gyh2c_2                 0x19A5
// #define         gyl1a_2                 0x19A6  
// #define         gyl1b_2                 0x19A7  
// #define         gyl1c_2                 0x19A8  
// #define         gygma_2                 0x19A9
// #define         gyzoom_2                0x19AA
// #define         gylens_2                0x19AB
// #define         gdm2g                   0x19AC
// #define         gdm4g                   0x19AD
// //#define         SttyaL                  0x19AE
// //#define         Stty34bM                0x19AF
// #define         gylmt1L                 0x19B0
// //#define         gylmt1H                 0x19B1
// #define         gylmt2L                 0x19B2
// #define         gylmt2H                 0x19B3
// #define         gylmt3H0                0x19B4
// #define         gylmt3H1                0x19B5
// #define         gylmt3g                 0x19B6
// //#define         gyleva                  0x19B7
// //#define         gylevb                  0x19B8
// #define         gylebc                  0x19B9
// //#define         gyadjmin                0x19BA
// //#define         gyadjmax                0x19BB
// //#define         gyadjdn                 0x19BC
// //#define         gyadjup                 0x19BD
// //#define         SttybL                  0x19BE
// //#define         Stty34bH                0x19BF
// #define         GXADOFFZ                0x19C0
// #define         GXLMT1Z                 0x19C1
// //#define         GXINZ                   0x19C2
// #define         GXGMZ2                  0x19C3
// //#define         GXI1Z1                  0x19C4
// //#define         GXI1Z2                  0x19C5
// #define         GXI1Z                   0x19C6
// #define         GXDM1GZ                 0x19C7
// #define         GXDM2GZ                 0x19C8
// #define         GXDM3GZ                 0x19C9
// #define         GXDM4GZ                 0x19CA
// //#define         GXI2Z1                  0x19CB
// //#define         GXI2Z2                  0x19CC
// #define         SttyLtmp                0x19CD
// //#define         GXI3Z                   0x19CE
// //#define         GXXFZ                   0x19CF
// //#define         GXADJZ                  0x19D0
// //#define         GXGAINZ                 0x19D1
// #define         GXLMT2Z                 0x19D2
// //#define         GXH1Z1                  0x19D3
// //#define         GXH1Z2                  0x19D4
// #define         GXCPUZ1                 0x19D5
// //#define         GXH2Z1                  0x19D6
// //#define         GXH2Z2                  0x19D7
// #define         gytmp                   0x19D8
// //#define         GXL1Z1                  0x19D9
// //#define         GXL1Z2                  0x19DA
// #define         GXLEV1Z                 0x19DB
// #define         GXLEV2Z                 0x19DC
// #define         SttyMtmp                0x19DD
// //#define         GXL2Z1                  0x19DE
// //#define         GXL2Z2                  0x19DF
// #define         GXOFFZ                  0x19E0
// #define         GXL3Z1                  0x19E1
// #define         GXL3Z2                  0x19E2
// #define         gylmt3Hx                0x19E3
// #define         GXL4Z1                  0x19E4
// #define         GXL4Z2                  0x19E5
// #define         GXL4Z5                  0x19E6
// #define         GXS2Z                   0x19E7
// //#define         GXZOOMZ                 0x19E8
// //#define         GXXBZ                   0x19E9
// //#define         GXLENSZ                 0x19EA
// #define         TMPX1Z                  0x19EB
// //#define         GX45Z                   0x19EC
// #define         SttyHtmp                0x19ED
// //#define         GXLMT3Z                 0x19EE
// #define         GXREFZ                  0x19F0
// //#define         GXTZ1                   0x19F1
// //#define         GXTZ2                   0x19F2
// //#define         GXTZ3                   0x19F3
// //#define         GXTZ4                   0x19F4
// #define         GXADZ                   0x19F5
// #define         GXL4Z6                  0x19F6
// //#define         GX2SXZ                  0x19F7
// #define         GXZOOMZ2                0x19F8
// //#define         PXAMZ                   0x19F9
// //#define         PXMAZ1                  0x19FA
// //#define         PXMAZ2                  0x19FB
// //#define         PXBMZ                   0x19FC
// //#define         PXMBZ1                  0x19FD
// //#define         PXMBZ2                  0x19FE
// 
// 
// 
// 
// 
// 
// 


#endif


//==============================================================================
// ois.h Code END
//==============================================================================

#endif
