/*add camera ois driver*/
#ifndef _IMX278_LGIT_OIS_DATA_HEAD
#define _IMX278_LGIT_OIS_DATA_HEAD
//********************************************************************************
//
//		LC8981xxx data
//
//	    Program Name	: OisDataSunny.h
//		Design			: Rex.Tang
//		History			: First edition						2014.10.24 Rex.Tang
//
//		Description		: Filter & common data
//********************************************************************************

struct STFILREG {
    unsigned short	UsRegAdd ;
    unsigned char	UcRegDat ;
} ;													// Register Data Table

struct STFILRAM {
    unsigned short	UsRamAdd ;
    unsigned long	UlRamDat ;
} ;													// Filter Coefficient Table
#define		INIT_FAST				// Muitl Byte trans mode for OIS filter intialize
/*--------------------------------------------------------
 *
 *  Filter data
 *
 * ------------------------------------------------------*/
/*Filter Calculator Version 4.02*/
/*the time and date : 2014/10/29 21:50:20*/
/*FC filename : LC898122_TDK95_V0007_d*/
/*fs,23438Hz*/
/*LSI No.,LC898122*/
/*Comment,*/
#ifndef INIT_FAST
/* 8bit */
static const struct STFILREG	CsFilReg[]	= {
		{ 0x0111,	0x00},		/*00,0111*/
		{ 0x0113,	0x00},		/*00,0113*/
		{ 0x0114,	0x00},		/*00,0114*/
		{ 0x0172,	0x00},		/*00,0172*/
		{ 0x01E3,	0x00},		/*00,01E3*/
		{ 0x01E4,	0x00},		/*00,01E4*/
		{ 0xFFFF,	0xFF }
	} ;
/* 32bit */
static const struct STFILRAM	CsFilRam[]	= {
		{ 0x1000,	0x3F800000},		/*3F800000,1000,0dB,invert=0*/
		{ 0x1001,	0x3F800000},		/*3F800000,1001,0dB,invert=0*/
		{ 0x1002,	0x00000000},		/*00000000,1002,Cutoff,invert=0*/
		{ 0x1003,	0x3EBFED40},		/*3EBFED40,1003,-8.5227dB,invert=0*/
		{ 0x1004,	0x3860DE00},		/*3860DE00,1004,LPF,0.4Hz,0dB,fs/1,invert=0*/
		{ 0x1005,	0x3860DE00},		/*3860DE00,1005,LPF,0.4Hz,0dB,fs/1,invert=0*/
		{ 0x1006,	0x3F7FF900},		/*3F7FF900,1006,LPF,0.4Hz,0dB,fs/1,invert=0*/
		{ 0x1007,	0x3F800000},		/*3F800000,1007,0dB,invert=0*/
		{ 0x1008,	0xBF800000},		/*BF800000,1008,0dB,invert=1*/
		{ 0x1009,	0x3F800000},		/*3F800000,1009,0dB,invert=0*/
		{ 0x100A,	0x3F800000},		/*3F800000,100A,0dB,invert=0*/
		{ 0x100B,	0x3F800000},		/*3F800000,100B,0dB,invert=0*/
		{ 0x100C,	0x3F800000},		/*3F800000,100C,0dB,invert=0*/
		{ 0x100E,	0x3F800000},		/*3F800000,100E,0dB,invert=0*/
		{ 0x1010,	0x3DA2ADC0},		/*3DA2ADC0,1010*/
		{ 0x1011,	0x00000000},		/*00000000,1011,Free,fs/1,invert=0*/
		{ 0x1012,	0x3F7FFD80},		/*3F7FFD80,1012,Free,fs/1,invert=0*/
		{ 0x1013,	0x3F76D280},		/*3F76D280,1013,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
		{ 0x1014,	0xBF723C00},		/*BF723C00,1014,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
		{ 0x1015,	0x3F690E80},		/*3F690E80,1015,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
		{ 0x1016,	0x40FE2B00},		/*40FE2B00,1016,HPF,0.5Hz,18dB,fs/1,invert=0*/
		{ 0x1017,	0xC0FE2B00},		/*C0FE2B00,1017,HPF,0.5Hz,18dB,fs/1,invert=0*/
		{ 0x1018,	0x3F7FF740},		/*3F7FF740,1018,HPF,0.5Hz,18dB,fs/1,invert=0*/
		{ 0x1019,	0x3ED1D880},		/*3ED1D880,1019,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
		{ 0x101A,	0xBED1CFC0},		/*BED1CFC0,101A,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
		{ 0x101B,	0x3F7FFB80},		/*3F7FFB80,101B,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
		{ 0x101C,	0x00000000},		/*00000000,101C,Cutoff,invert=0*/
		{ 0x101D,	0x3F800000},		/*3F800000,101D,0dB,invert=0*/
		{ 0x101E,	0x3F800000},		/*3F800000,101E,0dB,invert=0*/
		{ 0x1020,	0x3F800000},		/*3F800000,1020,0dB,invert=0*/
		{ 0x1021,	0x3F800000},		/*3F800000,1021,0dB,invert=0*/
		{ 0x1022,	0x3F800000},		/*3F800000,1022,0dB,invert=0*/
		{ 0x1023,	0x3F800000},		/*3F800000,1023,Through,0dB,fs/1,invert=0*/
		{ 0x1024,	0x00000000},		/*00000000,1024,Through,0dB,fs/1,invert=0*/
		{ 0x1025,	0x00000000},		/*00000000,1025,Through,0dB,fs/1,invert=0*/
		{ 0x1026,	0x00000000},		/*00000000,1026,Through,0dB,fs/1,invert=0*/
		{ 0x1027,	0x00000000},		/*00000000,1027,Through,0dB,fs/1,invert=0*/
		{ 0x1030,	0x3F800000},		/*3F800000,1030,Through,0dB,fs/1,invert=0*/
		{ 0x1031,	0x00000000},		/*00000000,1031,Through,0dB,fs/1,invert=0*/
		{ 0x1032,	0x00000000},		/*00000000,1032,Through,0dB,fs/1,invert=0*/
		{ 0x1033,	0x3F800000},		/*3F800000,1033,Through,0dB,fs/1,invert=0*/
		{ 0x1034,	0x00000000},		/*00000000,1034,Through,0dB,fs/1,invert=0*/
		{ 0x1035,	0x00000000},		/*00000000,1035,Through,0dB,fs/1,invert=0*/
		{ 0x1036,	0x3F800000},		/*3F800000,1036,Through,0dB,fs/1,invert=0*/
		{ 0x1037,	0x00000000},		/*00000000,1037,Through,0dB,fs/1,invert=0*/
		{ 0x1038,	0x00000000},		/*00000000,1038,Through,0dB,fs/1,invert=0*/
		{ 0x1039,	0x3F800000},		/*3F800000,1039,Through,0dB,fs/1,invert=0*/
		{ 0x103A,	0x00000000},		/*00000000,103A,Through,0dB,fs/1,invert=0*/
		{ 0x103B,	0x00000000},		/*00000000,103B,Through,0dB,fs/1,invert=0*/
		{ 0x103C,	0x3F800000},		/*3F800000,103C,Through,0dB,fs/1,invert=0*/
		{ 0x103D,	0x00000000},		/*00000000,103D,Through,0dB,fs/1,invert=0*/
		{ 0x103E,	0x00000000},		/*00000000,103E,Through,0dB,fs/1,invert=0*/
		{ 0x1043,	0x39D2BD40},		/*39D2BD40,1043,LPF,3Hz,0dB,fs/1,invert=0*/
		{ 0x1044,	0x39D2BD40},		/*39D2BD40,1044,LPF,3Hz,0dB,fs/1,invert=0*/
		{ 0x1045,	0x3F7FCB40},		/*3F7FCB40,1045,LPF,3Hz,0dB,fs/1,invert=0*/
		{ 0x1046,	0x388C8A40},		/*388C8A40,1046,LPF,0.5Hz,0dB,fs/1,invert=0*/
		{ 0x1047,	0x388C8A40},		/*388C8A40,1047,LPF,0.5Hz,0dB,fs/1,invert=0*/
		{ 0x1048,	0x3F7FF740},		/*3F7FF740,1048,LPF,0.5Hz,0dB,fs/1,invert=0*/
		{ 0x1049,	0x390C87C0},		/*390C87C0,1049,LPF,1Hz,0dB,fs/1,invert=0*/
		{ 0x104A,	0x390C87C0},		/*390C87C0,104A,LPF,1Hz,0dB,fs/1,invert=0*/
		{ 0x104B,	0x3F7FEE80},		/*3F7FEE80,104B,LPF,1Hz,0dB,fs/1,invert=0*/
		{ 0x104C,	0x398C8300},		/*398C8300,104C,LPF,2Hz,0dB,fs/1,invert=0*/
		{ 0x104D,	0x398C8300},		/*398C8300,104D,LPF,2Hz,0dB,fs/1,invert=0*/
		{ 0x104E,	0x3F7FDCC0},		/*3F7FDCC0,104E,LPF,2Hz,0dB,fs/1,invert=0*/
		{ 0x1053,	0x3F800000},		/*3F800000,1053,Through,0dB,fs/1,invert=0*/
		{ 0x1054,	0x00000000},		/*00000000,1054,Through,0dB,fs/1,invert=0*/
		{ 0x1055,	0x00000000},		/*00000000,1055,Through,0dB,fs/1,invert=0*/
		{ 0x1056,	0x3F800000},		/*3F800000,1056,Through,0dB,fs/1,invert=0*/
		{ 0x1057,	0x00000000},		/*00000000,1057,Through,0dB,fs/1,invert=0*/
		{ 0x1058,	0x00000000},		/*00000000,1058,Through,0dB,fs/1,invert=0*/
		{ 0x1059,	0x3F800000},		/*3F800000,1059,Through,0dB,fs/1,invert=0*/
		{ 0x105A,	0x00000000},		/*00000000,105A,Through,0dB,fs/1,invert=0*/
		{ 0x105B,	0x00000000},		/*00000000,105B,Through,0dB,fs/1,invert=0*/
		{ 0x105C,	0x3F800000},		/*3F800000,105C,Through,0dB,fs/1,invert=0*/
		{ 0x105D,	0x00000000},		/*00000000,105D,Through,0dB,fs/1,invert=0*/
		{ 0x105E,	0x00000000},		/*00000000,105E,Through,0dB,fs/1,invert=0*/
		{ 0x1063,	0x3F800000},		/*3F800000,1063,0dB,invert=0*/
		{ 0x1066,	0x3F800000},		/*3F800000,1066,0dB,invert=0*/
		{ 0x1069,	0x3F800000},		/*3F800000,1069,0dB,invert=0*/
		{ 0x106C,	0x3F800000},		/*3F800000,106C,0dB,invert=0*/
		{ 0x1073,	0x00000000},		/*00000000,1073,Cutoff,invert=0*/
		{ 0x1076,	0x3F800000},		/*3F800000,1076,0dB,invert=0*/
		{ 0x1079,	0x3F800000},		/*3F800000,1079,0dB,invert=0*/
		{ 0x107C,	0x3F800000},		/*3F800000,107C,0dB,invert=0*/
		{ 0x1083,	0x38D1B700},		/*38D1B700,1083,-80dB,invert=0*/
		{ 0x1086,	0x00000000},		/*00000000,1086,Cutoff,invert=0*/
		{ 0x1089,	0x00000000},		/*00000000,1089,Cutoff,invert=0*/
		{ 0x108C,	0x00000000},		/*00000000,108C,Cutoff,invert=0*/
		{ 0x1093,	0x00000000},		/*00000000,1093,Cutoff,invert=0*/
		{ 0x1098,	0x3F800000},		/*3F800000,1098,0dB,invert=0*/
		{ 0x1099,	0x3F800000},		/*3F800000,1099,0dB,invert=0*/
		{ 0x109A,	0x3F800000},		/*3F800000,109A,0dB,invert=0*/
		{ 0x10A1,	0x3C58B440},		/*3C58B440,10A1,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x10A2,	0x3C58B440},		/*3C58B440,10A2,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x10A3,	0x3F793A40},		/*3F793A40,10A3,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x10A4,	0x3C58B440},		/*3C58B440,10A4,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x10A5,	0x3C58B440},		/*3C58B440,10A5,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x10A6,	0x3F793A40},		/*3F793A40,10A6,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x10A7,	0x3F800000},		/*3F800000,10A7,Through,0dB,fs/1,invert=0*/
		{ 0x10A8,	0x00000000},		/*00000000,10A8,Through,0dB,fs/1,invert=0*/
		{ 0x10A9,	0x00000000},		/*00000000,10A9,Through,0dB,fs/1,invert=0*/
		{ 0x10AA,	0x00000000},		/*00000000,10AA,Cutoff,invert=0*/
		{ 0x10AB,	0x3BDA2580},		/*3BDA2580,10AB,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10AC,	0x3BDA2580},		/*3BDA2580,10AC,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10AD,	0x3F7C9780},		/*3F7C9780,10AD,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10B0,	0x3E8081C0},		/*3E8081C0,10B0,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10B1,	0x3E8081C0},		/*3E8081C0,10B1,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10B2,	0x3EFEFC80},		/*3EFEFC80,10B2,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10B3,	0x3F800000},		/*3F800000,10B3,0dB,invert=0*/
		{ 0x10B4,	0x00000000},		/*00000000,10B4,Cutoff,invert=0*/
		{ 0x10B5,	0x00000000},		/*00000000,10B5,Cutoff,invert=0*/
		{ 0x10B6,	0x3F800000},		/*3F800000,10B6,0dB,invert=0*/
		{ 0x10B8,	0x3F800000},		/*3F800000,10B8,0dB,invert=0*/
		{ 0x10B9,	0x00000000},		/*00000000,10B9,Cutoff,invert=0*/
		{ 0x10C0,	0x3F696080},		/*3F696080,10C0,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x10C1,	0xBF6509C0},		/*BF6509C0,10C1,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x10C2,	0x3F4E6A40},		/*3F4E6A40,10C2,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x10C3,	0x3F800000},		/*3F800000,10C3,Through,0dB,fs/1,invert=0*/
		{ 0x10C4,	0x00000000},		/*00000000,10C4,Through,0dB,fs/1,invert=0*/
		{ 0x10C5,	0x00000000},		/*00000000,10C5,Through,0dB,fs/1,invert=0*/
		{ 0x10C6,	0x3D506F00},		/*3D506F00,10C6,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x10C7,	0x3D506F00},		/*3D506F00,10C7,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x10C8,	0x3F65F240},		/*3F65F240,10C8,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x10C9,	0x3C62BC00},		/*3C62BC00,10C9,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x10CA,	0x3C62BC00},		/*3C62BC00,10CA,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x10CB,	0x3F7FE940},		/*3F7FE940,10CB,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x10CC,	0x3DB5DCC0},		/*3DB5DCC0,10CC,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x10CD,	0xBDB41D80},		/*BDB41D80,10CD,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x10CE,	0x3F7F7380},		/*3F7F7380,10CE,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x10D0,	0x3FFF64C0},		/*3FFF64C0,10D0,6dB,invert=0*/
		{ 0x10D1,	0x00000000},		/*00000000,10D1,Cutoff,invert=0*/
		{ 0x10D2,	0x3F800000},		/*3F800000,10D2,0dB,invert=0*/
		{ 0x10D3,	0x3F004DC0},		/*3F004DC0,10D3,-6dB,invert=0*/
		{ 0x10D4,	0x3F800000},		/*3F800000,10D4,0dB,invert=0*/
		{ 0x10D5,	0x3F800000},		/*3F800000,10D5,0dB,invert=0*/
		{ 0x10D7,	0x417D9540},		/*417D9540,10D7,Through,24dB,fs/1,invert=0*/
		{ 0x10D8,	0x00000000},		/*00000000,10D8,Through,24dB,fs/1,invert=0*/
		{ 0x10D9,	0x00000000},		/*00000000,10D9,Through,24dB,fs/1,invert=0*/
		{ 0x10DA,	0x3F792280},		/*3F792280,10DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DB,	0xBFEBE280},		/*BFEBE280,10DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DC,	0x3FEBE280},		/*3FEBE280,10DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DD,	0x3F6B5700},		/*3F6B5700,10DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DE,	0xBF6479C0},		/*BF6479C0,10DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10E0,	0x3E8081C0},		/*3E8081C0,10E0,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10E1,	0x3E8081C0},		/*3E8081C0,10E1,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10E2,	0x3EFEFC80},		/*3EFEFC80,10E2,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10E3,	0x00000000},		/*00000000,10E3,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10E4,	0x00000000},		/*00000000,10E4,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x10E5,	0x3F800000},		/*3F800000,10E5,0dB,invert=0*/
		{ 0x10E8,	0x3F800000},		/*3F800000,10E8,0dB,invert=0*/
		{ 0x10E9,	0x00000000},		/*00000000,10E9,Cutoff,invert=0*/
		{ 0x10EA,	0x00000000},		/*00000000,10EA,Cutoff,invert=0*/
		{ 0x10EB,	0x00000000},		/*00000000,10EB,Cutoff,invert=0*/
		{ 0x10F0,	0x3F800000},		/*3F800000,10F0,Through,0dB,fs/1,invert=0*/
		{ 0x10F1,	0x00000000},		/*00000000,10F1,Through,0dB,fs/1,invert=0*/
		{ 0x10F2,	0x00000000},		/*00000000,10F2,Through,0dB,fs/1,invert=0*/
		{ 0x10F3,	0x00000000},		/*00000000,10F3,Through,0dB,fs/1,invert=0*/
		{ 0x10F4,	0x00000000},		/*00000000,10F4,Through,0dB,fs/1,invert=0*/
		{ 0x10F5,	0x3F800000},		/*3F800000,10F5,Through,0dB,fs/1,invert=0*/
		{ 0x10F6,	0x00000000},		/*00000000,10F6,Through,0dB,fs/1,invert=0*/
		{ 0x10F7,	0x00000000},		/*00000000,10F7,Through,0dB,fs/1,invert=0*/
		{ 0x10F8,	0x00000000},		/*00000000,10F8,Through,0dB,fs/1,invert=0*/
		{ 0x10F9,	0x00000000},		/*00000000,10F9,Through,0dB,fs/1,invert=0*/

		{ 0x1200,	0x00000000},		/*00000000,1200,Cutoff,invert=0*/
		{ 0x1201,	0x3F800000},		/*3F800000,1201,0dB,invert=0*/
		{ 0x1202,	0x3F800000},		/*3F800000,1202,0dB,invert=0*/
		{ 0x1203,	0x3F800000},		/*3F800000,1203,0dB,invert=0*/
		{ 0x1204,	0x3F800000},		/*3F800000,1204,Through,0dB,fs/1,invert=0*/
		{ 0x1205,	0x00000000},		/*00000000,1205,Through,0dB,fs/1,invert=0*/
		{ 0x1206,	0x00000000},		/*00000000,1206,Through,0dB,fs/1,invert=0*/
		{ 0x1207,	0x3F800000},		/*3F800000,1207,Through,0dB,fs/1,invert=0*/
		{ 0x1208,	0x00000000},		/*00000000,1208,Through,0dB,fs/1,invert=0*/
		{ 0x1209,	0x00000000},		/*00000000,1209,Through,0dB,fs/1,invert=0*/
		{ 0x120A,	0x3F800000},		/*3F800000,120A,Through,0dB,fs/1,invert=0*/
		{ 0x120B,	0x00000000},		/*00000000,120B,Through,0dB,fs/1,invert=0*/
		{ 0x120C,	0x00000000},		/*00000000,120C,Through,0dB,fs/1,invert=0*/
		{ 0x120D,	0x3F800000},		/*3F800000,120D,Through,0dB,fs/1,invert=0*/
		{ 0x120E,	0x00000000},		/*00000000,120E,Through,0dB,fs/1,invert=0*/
		{ 0x120F,	0x00000000},		/*00000000,120F,Through,0dB,fs/1,invert=0*/
		{ 0x1210,	0x3F800000},		/*3F800000,1210,Through,0dB,fs/1,invert=0*/
		{ 0x1211,	0x00000000},		/*00000000,1211,Through,0dB,fs/1,invert=0*/
		{ 0x1212,	0x00000000},		/*00000000,1212,Through,0dB,fs/1,invert=0*/
		{ 0x1213,	0x3F800000},		/*3F800000,1213,0dB,invert=0*/
		{ 0x1214,	0x3F800000},		/*3F800000,1214,0dB,invert=0*/
		{ 0x1215,	0x3F800000},		/*3F800000,1215,0dB,invert=0*/
		{ 0x1216,	0x3F800000},		/*3F800000,1216,0dB,invert=0*/
		{ 0x1217,	0x3F800000},		/*3F800000,1217,0dB,invert=0*/
		{ 0x1218,	0x00000000},		/*00000000,1218,Cutoff,fs/1,invert=0*/
		{ 0x1219,	0x00000000},		/*00000000,1219,Cutoff,fs/1,invert=0*/
		{ 0x121A,	0x00000000},		/*00000000,121A,Cutoff,fs/1,invert=0*/
		{ 0x121B,	0x00000000},		/*00000000,121B,Cutoff,fs/1,invert=0*/
		{ 0x121C,	0x00000000},		/*00000000,121C,Cutoff,fs/1,invert=0*/
		{ 0x121D,	0x3F800000},		/*3F800000,121D,0dB,invert=0*/
		{ 0x121E,	0x3F800000},		/*3F800000,121E,0dB,invert=0*/
		{ 0x121F,	0x3F800000},		/*3F800000,121F,0dB,invert=0*/
		{ 0x1235,	0x3F800000},		/*3F800000,1235,0dB,invert=0*/
		{ 0x1236,	0x3F800000},		/*3F800000,1236,0dB,invert=0*/
		{ 0x1237,	0x3F800000},		/*3F800000,1237,0dB,invert=0*/
		{ 0x1238,	0x3F800000},		/*3F800000,1238,0dB,invert=0*/
		{ 0xFFFF,	0xFFFFFFFF}
};
#else
/*Filter Calculator Version 4.02*/
/*the time and date : 2014/10/29 21:50:20*/
/*FC filename : LC898122_TDK95_V0007_d*/
/*fs,23438Hz*/
/*LSI No.,LC898122*/
/*Comment,*/
/* 8bit */
static const uint8_t CsFilRegDat[] = { 
	0x01, 0x11, 	/* 0x0111 */
	 0x00, 			 /*00,0111*/
	0x01, 0x13, 	/* 0x0113 */
	 0x00, 			 /*00,0113*/
	 0x00, 			 /*00,0114*/
	0x01, 0x72, 	/* 0x0172 */
	 0x00, 			 /*00,0172*/
	0x01, 0xE3, 	/* 0x01E3 */
	 0x00, 			 /*00,01E3*/
	 0x00, 			 /*00,01E4*/
} ;

static const uint8_t CsFilRegBurst[] = {
	 3, 4, 3, 4, 0xFF 
}; 

/* 32bit */
static const uint8_t CsFilRamDat[] = { 
	0x10, 0x00, 	/* 0x1000 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1000,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1001,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1002,Cutoff,invert=0*/
	0x10, 0x03, 	/* 0x1003 */
	 0x3E, 0xBF, 0xED, 0x40, 	 /*3EBFED40,1003,-8.5227dB,invert=0*/
	 0x38, 0x60, 0xDE, 0x00, 	 /*3860DE00,1004,LPF,0.4Hz,0dB,fs/1,invert=0*/
	 0x38, 0x60, 0xDE, 0x00, 	 /*3860DE00,1005,LPF,0.4Hz,0dB,fs/1,invert=0*/
	0x10, 0x06, 	/* 0x1006 */
	 0x3F, 0x7F, 0xF9, 0x00, 	 /*3F7FF900,1006,LPF,0.4Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1007,0dB,invert=0*/
	 0xBF, 0x80, 0x00, 0x00, 	 /*BF800000,1008,0dB,invert=1*/
	0x10, 0x09, 	/* 0x1009 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1009,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,100A,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,100B,0dB,invert=0*/
	0x10, 0x0C, 	/* 0x100C */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,100C,0dB,invert=0*/
	0x10, 0x0E, 	/* 0x100E */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,100E,0dB,invert=0*/
	0x10, 0x10, 	/* 0x1010 */
	 0x3D, 0xA2, 0xAD, 0xC0, 	 /*3DA2ADC0,1010*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1011,Free,fs/1,invert=0*/
	 0x3F, 0x7F, 0xFD, 0x80, 	 /*3F7FFD80,1012,Free,fs/1,invert=0*/
	0x10, 0x13, 	/* 0x1013 */
	 0x3F, 0x76, 0xD2, 0x80, 	 /*3F76D280,1013,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
	 0xBF, 0x72, 0x3C, 0x00, 	 /*BF723C00,1014,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x69, 0x0E, 0x80, 	 /*3F690E80,1015,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
	0x10, 0x16, 	/* 0x1016 */
	 0x40, 0xFE, 0x2B, 0x00, 	 /*40FE2B00,1016,HPF,0.5Hz,18dB,fs/1,invert=0*/
	 0xC0, 0xFE, 0x2B, 0x00, 	 /*C0FE2B00,1017,HPF,0.5Hz,18dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xF7, 0x40, 	 /*3F7FF740,1018,HPF,0.5Hz,18dB,fs/1,invert=0*/
	0x10, 0x19, 	/* 0x1019 */
	 0x3E, 0xD1, 0xD8, 0x80, 	 /*3ED1D880,1019,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
	 0xBE, 0xD1, 0xCF, 0xC0, 	 /*BED1CFC0,101A,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xFB, 0x80, 	 /*3F7FFB80,101B,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
	0x10, 0x1C, 	/* 0x101C */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,101C,Cutoff,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,101D,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,101E,0dB,invert=0*/
	0x10, 0x20, 	/* 0x1020 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1020,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1021,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1022,0dB,invert=0*/
	0x10, 0x23, 	/* 0x1023 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1023,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1024,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1025,Through,0dB,fs/1,invert=0*/
	0x10, 0x26, 	/* 0x1026 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1026,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1027,Through,0dB,fs/1,invert=0*/
	0x10, 0x30, 	/* 0x1030 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1030,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1031,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1032,Through,0dB,fs/1,invert=0*/
	0x10, 0x33, 	/* 0x1033 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1033,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1034,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1035,Through,0dB,fs/1,invert=0*/
	0x10, 0x36, 	/* 0x1036 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1036,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1037,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1038,Through,0dB,fs/1,invert=0*/
	0x10, 0x39, 	/* 0x1039 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1039,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,103A,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,103B,Through,0dB,fs/1,invert=0*/
	0x10, 0x3C, 	/* 0x103C */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,103C,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,103D,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,103E,Through,0dB,fs/1,invert=0*/
	0x10, 0x43, 	/* 0x1043 */
	 0x39, 0xD2, 0xBD, 0x40, 	 /*39D2BD40,1043,LPF,3Hz,0dB,fs/1,invert=0*/
	 0x39, 0xD2, 0xBD, 0x40, 	 /*39D2BD40,1044,LPF,3Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xCB, 0x40, 	 /*3F7FCB40,1045,LPF,3Hz,0dB,fs/1,invert=0*/
	0x10, 0x46, 	/* 0x1046 */
	 0x38, 0x8C, 0x8A, 0x40, 	 /*388C8A40,1046,LPF,0.5Hz,0dB,fs/1,invert=0*/
	 0x38, 0x8C, 0x8A, 0x40, 	 /*388C8A40,1047,LPF,0.5Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xF7, 0x40, 	 /*3F7FF740,1048,LPF,0.5Hz,0dB,fs/1,invert=0*/
	0x10, 0x49, 	/* 0x1049 */
	 0x39, 0x0C, 0x87, 0xC0, 	 /*390C87C0,1049,LPF,1Hz,0dB,fs/1,invert=0*/
	 0x39, 0x0C, 0x87, 0xC0, 	 /*390C87C0,104A,LPF,1Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xEE, 0x80, 	 /*3F7FEE80,104B,LPF,1Hz,0dB,fs/1,invert=0*/
	0x10, 0x4C, 	/* 0x104C */
	 0x39, 0x8C, 0x83, 0x00, 	 /*398C8300,104C,LPF,2Hz,0dB,fs/1,invert=0*/
	 0x39, 0x8C, 0x83, 0x00, 	 /*398C8300,104D,LPF,2Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xDC, 0xC0, 	 /*3F7FDCC0,104E,LPF,2Hz,0dB,fs/1,invert=0*/
	0x10, 0x53, 	/* 0x1053 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1053,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1054,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1055,Through,0dB,fs/1,invert=0*/
	0x10, 0x56, 	/* 0x1056 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1056,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1057,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1058,Through,0dB,fs/1,invert=0*/
	0x10, 0x59, 	/* 0x1059 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1059,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,105A,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,105B,Through,0dB,fs/1,invert=0*/
	0x10, 0x5C, 	/* 0x105C */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,105C,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,105D,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,105E,Through,0dB,fs/1,invert=0*/
	0x10, 0x63, 	/* 0x1063 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1063,0dB,invert=0*/
	0x10, 0x66, 	/* 0x1066 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1066,0dB,invert=0*/
	0x10, 0x69, 	/* 0x1069 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1069,0dB,invert=0*/
	0x10, 0x6C, 	/* 0x106C */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,106C,0dB,invert=0*/
	0x10, 0x73, 	/* 0x1073 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1073,Cutoff,invert=0*/
	0x10, 0x76, 	/* 0x1076 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1076,0dB,invert=0*/
	0x10, 0x79, 	/* 0x1079 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1079,0dB,invert=0*/
	0x10, 0x7C, 	/* 0x107C */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,107C,0dB,invert=0*/
	0x10, 0x83, 	/* 0x1083 */
	 0x38, 0xD1, 0xB7, 0x00, 	 /*38D1B700,1083,-80dB,invert=0*/
	0x10, 0x86, 	/* 0x1086 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1086,Cutoff,invert=0*/
	0x10, 0x89, 	/* 0x1089 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1089,Cutoff,invert=0*/
	0x10, 0x8C, 	/* 0x108C */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,108C,Cutoff,invert=0*/
	0x10, 0x93, 	/* 0x1093 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1093,Cutoff,invert=0*/
	0x10, 0x98, 	/* 0x1098 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1098,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1099,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,109A,0dB,invert=0*/
	0x10, 0xA1, 	/* 0x10A1 */
	 0x3C, 0x58, 0xB4, 0x40, 	 /*3C58B440,10A1,LPF,100Hz,0dB,fs/1,invert=0*/
	 0x3C, 0x58, 0xB4, 0x40, 	 /*3C58B440,10A2,LPF,100Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x79, 0x3A, 0x40, 	 /*3F793A40,10A3,LPF,100Hz,0dB,fs/1,invert=0*/
	0x10, 0xA4, 	/* 0x10A4 */
	 0x3C, 0x58, 0xB4, 0x40, 	 /*3C58B440,10A4,LPF,100Hz,0dB,fs/1,invert=0*/
	 0x3C, 0x58, 0xB4, 0x40, 	 /*3C58B440,10A5,LPF,100Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x79, 0x3A, 0x40, 	 /*3F793A40,10A6,LPF,100Hz,0dB,fs/1,invert=0*/
	0x10, 0xA7, 	/* 0x10A7 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10A7,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10A8,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10A9,Through,0dB,fs/1,invert=0*/
	0x10, 0xAA, 	/* 0x10AA */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10AA,Cutoff,invert=0*/
	 0x3B, 0xDA, 0x25, 0x80, 	 /*3BDA2580,10AB,LPF,50Hz,0dB,fs/1,invert=0*/
	 0x3B, 0xDA, 0x25, 0x80, 	 /*3BDA2580,10AC,LPF,50Hz,0dB,fs/1,invert=0*/
	0x10, 0xAD, 	/* 0x10AD */
	 0x3F, 0x7C, 0x97, 0x80, 	 /*3F7C9780,10AD,LPF,50Hz,0dB,fs/1,invert=0*/
	0x10, 0xB0, 	/* 0x10B0 */
	 0x3E, 0x80, 0x81, 0xC0, 	 /*3E8081C0,10B0,LPF,2500Hz,0dB,fs/1,invert=0*/
	 0x3E, 0x80, 0x81, 0xC0, 	 /*3E8081C0,10B1,LPF,2500Hz,0dB,fs/1,invert=0*/
	 0x3E, 0xFE, 0xFC, 0x80, 	 /*3EFEFC80,10B2,LPF,2500Hz,0dB,fs/1,invert=0*/
	0x10, 0xB3, 	/* 0x10B3 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10B3,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10B4,Cutoff,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10B5,Cutoff,invert=0*/
	0x10, 0xB6, 	/* 0x10B6 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10B6,0dB,invert=0*/
	0x10, 0xB8, 	/* 0x10B8 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10B8,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10B9,Cutoff,invert=0*/
	0x10, 0xC0, 	/* 0x10C0 */
	 0x3F, 0x69, 0x60, 0x80, 	 /*3F696080,10C0,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
	 0xBF, 0x65, 0x09, 0xC0, 	 /*BF6509C0,10C1,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x4E, 0x6A, 0x40, 	 /*3F4E6A40,10C2,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
	0x10, 0xC3, 	/* 0x10C3 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10C3,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10C4,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10C5,Through,0dB,fs/1,invert=0*/
	0x10, 0xC6, 	/* 0x10C6 */
	 0x3D, 0x50, 0x6F, 0x00, 	 /*3D506F00,10C6,LPF,400Hz,0dB,fs/1,invert=0*/
	 0x3D, 0x50, 0x6F, 0x00, 	 /*3D506F00,10C7,LPF,400Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x65, 0xF2, 0x40, 	 /*3F65F240,10C8,LPF,400Hz,0dB,fs/1,invert=0*/
	0x10, 0xC9, 	/* 0x10C9 */
	 0x3C, 0x62, 0xBC, 0x00, 	 /*3C62BC00,10C9,LPF,1.3Hz,38dB,fs/1,invert=0*/
	 0x3C, 0x62, 0xBC, 0x00, 	 /*3C62BC00,10CA,LPF,1.3Hz,38dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xE9, 0x40, 	 /*3F7FE940,10CB,LPF,1.3Hz,38dB,fs/1,invert=0*/
	0x10, 0xCC, 	/* 0x10CC */
	 0x3D, 0xB5, 0xDC, 0xC0, 	 /*3DB5DCC0,10CC,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
	 0xBD, 0xB4, 0x1D, 0x80, 	 /*BDB41D80,10CD,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0x73, 0x80, 	 /*3F7F7380,10CE,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
	0x10, 0xD0, 	/* 0x10D0 */
	 0x3F, 0xFF, 0x64, 0xC0, 	 /*3FFF64C0,10D0,6dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10D1,Cutoff,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10D2,0dB,invert=0*/
	0x10, 0xD3, 	/* 0x10D3 */
	 0x3F, 0x00, 0x4D, 0xC0, 	 /*3F004DC0,10D3,-6dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10D4,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10D5,0dB,invert=0*/
	0x10, 0xD7, 	/* 0x10D7 */
	 0x41, 0x7D, 0x95, 0x40, 	 /*417D9540,10D7,Through,24dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10D8,Through,24dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10D9,Through,24dB,fs/1,invert=0*/
	0x10, 0xDA, 	/* 0x10DA */
	 0x3F, 0x79, 0x22, 0x80, 	 /*3F792280,10DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	 0xBF, 0xEB, 0xE2, 0x80, 	 /*BFEBE280,10DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	 0x3F, 0xEB, 0xE2, 0x80, 	 /*3FEBE280,10DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	0x10, 0xDD, 	/* 0x10DD */
	 0x3F, 0x6B, 0x57, 0x00, 	 /*3F6B5700,10DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	 0xBF, 0x64, 0x79, 0xC0, 	 /*BF6479C0,10DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	0x10, 0xE0, 	/* 0x10E0 */
	 0x3E, 0x80, 0x81, 0xC0, 	 /*3E8081C0,10E0,LPF,2500Hz,0dB,fs/1,invert=0*/
	 0x3E, 0x80, 0x81, 0xC0, 	 /*3E8081C0,10E1,LPF,2500Hz,0dB,fs/1,invert=0*/
	 0x3E, 0xFE, 0xFC, 0x80, 	 /*3EFEFC80,10E2,LPF,2500Hz,0dB,fs/1,invert=0*/
	0x10, 0xE3, 	/* 0x10E3 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E3,LPF,2500Hz,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E4,LPF,2500Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10E5,0dB,invert=0*/
	0x10, 0xE8, 	/* 0x10E8 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10E8,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E9,Cutoff,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10EA,Cutoff,invert=0*/
	0x10, 0xEB, 	/* 0x10EB */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10EB,Cutoff,invert=0*/
	0x10, 0xF0, 	/* 0x10F0 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10F0,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F1,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F2,Through,0dB,fs/1,invert=0*/
	0x10, 0xF3, 	/* 0x10F3 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F3,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F4,Through,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10F5,Through,0dB,fs/1,invert=0*/
	0x10, 0xF6, 	/* 0x10F6 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F6,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F7,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F8,Through,0dB,fs/1,invert=0*/
	0x10, 0xF9, 	/* 0x10F9 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10F9,Through,0dB,fs/1,invert=0*/
	0x12, 0x00, 	/* 0x1200 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1200,Cutoff,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1201,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1202,0dB,invert=0*/
	0x12, 0x03, 	/* 0x1203 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1203,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1204,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1205,Through,0dB,fs/1,invert=0*/
	0x12, 0x06, 	/* 0x1206 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1206,Through,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1207,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1208,Through,0dB,fs/1,invert=0*/
	0x12, 0x09, 	/* 0x1209 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1209,Through,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,120A,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,120B,Through,0dB,fs/1,invert=0*/
	0x12, 0x0C, 	/* 0x120C */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,120C,Through,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,120D,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,120E,Through,0dB,fs/1,invert=0*/
	0x12, 0x0F, 	/* 0x120F */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,120F,Through,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1210,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1211,Through,0dB,fs/1,invert=0*/
	0x12, 0x12, 	/* 0x1212 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1212,Through,0dB,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1213,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1214,0dB,invert=0*/
	0x12, 0x15, 	/* 0x1215 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1215,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1216,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1217,0dB,invert=0*/
	0x12, 0x18, 	/* 0x1218 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1218,Cutoff,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1219,Cutoff,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,121A,Cutoff,fs/1,invert=0*/
	0x12, 0x1B, 	/* 0x121B */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,121B,Cutoff,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,121C,Cutoff,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,121D,0dB,invert=0*/
	0x12, 0x1E, 	/* 0x121E */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,121E,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,121F,0dB,invert=0*/
	0x12, 0x35, 	/* 0x1235 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1235,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1236,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1237,0dB,invert=0*/
	0x12, 0x38, 	/* 0x1238 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1238,0dB,invert=0*/
} ;

static const uint8_t CsFilRamBurst[] = {
	 14, 14, 14, 14, 6, 6, 14, 14, 14, 14, 14, 14, 14, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 14, 14, 14, 14, 14, 6, 14, 14, 6, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14, 10, 14, 14, 14, 6, 14, 14, 14, 6, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 10, 14, 6, 0xFF 
}; 

static const uint8_t CsFilRamYDat[] ={
} ;

static const uint8_t CsFilRamY[] ={ 
	 0xFF 
}; 
#endif

#endif
