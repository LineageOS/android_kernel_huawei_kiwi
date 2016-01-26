/*add camera ois driver*/
#ifndef _IMX278_SUNNY_OIS_DATA_HEAD
#define _IMX278_SUNNY_OIS_DATA_HEAD

#define		INIT_FAST				// Muitl Byte trans mode for OIS filter intialize

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
		{ 0x1003,	0x3F800000},		/*3F800000,1003,0dB,invert=0*/
		{ 0x1004,	0x00000000},		/*00000000,1004,Cutoff,fs/1,invert=0*/
		{ 0x1005,	0x00000000},		/*00000000,1005,Cutoff,fs/1,invert=0*/
		{ 0x1006,	0x00000000},		/*00000000,1006,Cutoff,fs/1,invert=0*/
		{ 0x1007,	0x00000000},		/*00000000,1007,Cutoff,invert=0*/
		{ 0x1008,	0xBF800000},		/*BF800000,1008,0dB,invert=1*/
		{ 0x1009,	0x3F800000},		/*3F800000,1009,0dB,invert=0*/
		{ 0x100A,	0x3F800000},		/*3F800000,100A,0dB,invert=0*/
		{ 0x100B,	0x3F800000},		/*3F800000,100B,0dB,invert=0*/
		{ 0x100C,	0x3F800000},		/*3F800000,100C,0dB,invert=0*/
		{ 0x100E,	0x3F800000},		/*3F800000,100E,0dB,invert=0*/
		{ 0x1010,	0x3DA2AD80},		/*3DA2AD80,1010*/
		{ 0x1011,	0x00000000},		/*00000000,1011,Free,fs/1,invert=0*/
		{ 0x1012,	0x3F7FFE00},		/*3F7FFE00,1012,Free,fs/1,invert=0*/
		{ 0x1013,	0x402C5840},		/*402C5840,1013,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
		{ 0x1014,	0xC02A0D00},		/*C02A0D00,1014,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
		{ 0x1015,	0x3F65F240},		/*3F65F240,1015,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
		{ 0x1016,	0x3F530B40},		/*3F530B40,1016,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
		{ 0x1017,	0xBF52FF00},		/*BF52FF00,1017,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
		{ 0x1018,	0x3F7FF400},		/*3F7FF400,1018,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
		{ 0x1019,	0x3F800000},		/*3F800000,1019,Through,0dB,fs/1,invert=0*/
		{ 0x101A,	0x00000000},		/*00000000,101A,Through,0dB,fs/1,invert=0*/
		{ 0x101B,	0x00000000},		/*00000000,101B,Through,0dB,fs/1,invert=0*/
		{ 0x101C,	0x3F800000},		/*3F800000,101C,0dB,invert=0*/
		{ 0x101D,	0x00000000},		/*00000000,101D,Cutoff,invert=0*/
		{ 0x101E,	0x3F800000},		/*3F800000,101E,0dB,invert=0*/
		{ 0x1020,	0x3F800000},		/*3F800000,1020,0dB,invert=0*/
		{ 0x1021,	0x3F800000},		/*3F800000,1021,0dB,invert=0*/
		{ 0x1022,	0x3F800000},		/*3F800000,1022,0dB,invert=0*/
		{ 0x1023,	0x3F800000},		/*3F800000,1023,Through,0dB,fs/1,invert=0*/
		{ 0x1024,	0x00000000},		/*00000000,1024,Through,0dB,fs/1,invert=0*/
		{ 0x1025,	0x00000000},		/*00000000,1025,Through,0dB,fs/1,invert=0*/
		{ 0x1026,	0x00000000},		/*00000000,1026,Through,0dB,fs/1,invert=0*/
		{ 0x1027,	0x00000000},		/*00000000,1027,Through,0dB,fs/1,invert=0*/
		{ 0x1030,	0x3CA175C0},		/*3CA175C0,1030,LPF,150Hz,0dB,fs/1,invert=0*/
		{ 0x1031,	0x3CA175C0},		/*3CA175C0,1031,LPF,150Hz,0dB,fs/1,invert=0*/
		{ 0x1032,	0x3F75E8C0},		/*3F75E8C0,1032,LPF,150Hz,0dB,fs/1,invert=0*/
		{ 0x1033,	0x3F800000},		/*3F800000,1033,Through,0dB,fs/1,invert=0*/
		{ 0x1034,	0x00000000},		/*00000000,1034,Through,0dB,fs/1,invert=0*/
		{ 0x1035,	0x00000000},		/*00000000,1035,Through,0dB,fs/1,invert=0*/
		{ 0x1036,	0x4F7FFFC0},		/*4F7FFFC0,1036,Free,fs/1,invert=0*/
		{ 0x1037,	0x00000000},		/*00000000,1037,Free,fs/1,invert=0*/
		{ 0x1038,	0x00000000},		/*00000000,1038,Free,fs/1,invert=0*/
		{ 0x1039,	0x00000000},		/*00000000,1039,Free,fs/1,invert=0*/
		{ 0x103A,	0x30800000},		/*30800000,103A,Free,fs/1,invert=0*/
		{ 0x103B,	0x00000000},		/*00000000,103B,Free,fs/1,invert=0*/
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
		{ 0x10A1,	0x3BDA2580},		/*3BDA2580,10A1,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10A2,	0x3BDA2580},		/*3BDA2580,10A2,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10A3,	0x3F7C9780},		/*3F7C9780,10A3,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10A4,	0x00000000},		/*00000000,10A4,Free,fs/1,invert=0*/
		{ 0x10A5,	0x3A031240},		/*3A031240,10A5,Free,fs/1,invert=0*/
		{ 0x10A6,	0x3F800000},		/*3F800000,10A6,Free,fs/1,invert=0*/
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
		{ 0x10C0,	0x3F708000},		/*3F708000,10C0,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
		{ 0x10C1,	0xBF6C5880},		/*BF6C5880,10C1,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
		{ 0x10C2,	0x3F5CD8C0},		/*3F5CD8C0,10C2,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
		{ 0x10C3,	0x3F800000},		/*3F800000,10C3,Through,0dB,fs/1,invert=0*/
		{ 0x10C4,	0x00000000},		/*00000000,10C4,Through,0dB,fs/1,invert=0*/
		{ 0x10C5,	0x00000000},		/*00000000,10C5,Through,0dB,fs/1,invert=0*/
		{ 0x10C6,	0x3D506F00},		/*3D506F00,10C6,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x10C7,	0x3D506F00},		/*3D506F00,10C7,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x10C8,	0x3F65F240},		/*3F65F240,10C8,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x10C9,	0x3C62BC00},		/*3C62BC00,10C9,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x10CA,	0x3C62BC00},		/*3C62BC00,10CA,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x10CB,	0x3F7FE940},		/*3F7FE940,10CB,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x10CC,	0x3DF7D400},		/*3DF7D400,10CC,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
		{ 0x10CD,	0xBDF5A540},		/*BDF5A540,10CD,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
		{ 0x10CE,	0x3F7F5080},		/*3F7F5080,10CE,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
		{ 0x10D0,	0x3FFF64C0},		/*3FFF64C0,10D0,6dB,invert=0*/
		{ 0x10D1,	0x00000000},		/*00000000,10D1,Cutoff,invert=0*/
		{ 0x10D2,	0x3F800000},		/*3F800000,10D2,0dB,invert=0*/
		{ 0x10D3,	0x3ECBD4C0},		/*3ECBD4C0,10D3,-8dB,invert=0*/
		{ 0x10D4,	0x3F800000},		/*3F800000,10D4,0dB,invert=0*/
		{ 0x10D5,	0x3F800000},		/*3F800000,10D5,0dB,invert=0*/
		{ 0x10D7,	0x40B3CE80},		/*40B3CE80,10D7,LPF,2500Hz,27dB,fs/1,invert=0*/
		{ 0x10D8,	0x40B3CE80},		/*40B3CE80,10D8,LPF,2500Hz,27dB,fs/1,invert=0*/
		{ 0x10D9,	0x3EFEFC80},		/*3EFEFC80,10D9,LPF,2500Hz,27dB,fs/1,invert=0*/
		{ 0x10DA,	0x3F792280},		/*3F792280,10DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DB,	0xBFEBE280},		/*BFEBE280,10DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DC,	0x3FEBE280},		/*3FEBE280,10DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DD,	0x3F6B5700},		/*3F6B5700,10DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10DE,	0xBF6479C0},		/*BF6479C0,10DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x10E0,	0x3F800000},		/*3F800000,10E0,Through,0dB,fs/1,invert=0*/
		{ 0x10E1,	0x00000000},		/*00000000,10E1,Through,0dB,fs/1,invert=0*/
		{ 0x10E2,	0x00000000},		/*00000000,10E2,Through,0dB,fs/1,invert=0*/
		{ 0x10E3,	0x00000000},		/*00000000,10E3,Through,0dB,fs/1,invert=0*/
		{ 0x10E4,	0x00000000},		/*00000000,10E4,Through,0dB,fs/1,invert=0*/
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
		{ 0x1100,	0x3F800000},		/*3F800000,1100,0dB,invert=0*/
		{ 0x1101,	0x3F800000},		/*3F800000,1101,0dB,invert=0*/
		{ 0x1102,	0x00000000},		/*00000000,1102,Cutoff,invert=0*/
		{ 0x1103,	0x3F800000},		/*3F800000,1103,0dB,invert=0*/
		{ 0x1104,	0x00000000},		/*00000000,1104,Cutoff,fs/1,invert=0*/
		{ 0x1105,	0x00000000},		/*00000000,1105,Cutoff,fs/1,invert=0*/
		{ 0x1106,	0x00000000},		/*00000000,1106,Cutoff,fs/1,invert=0*/
		{ 0x1107,	0x00000000},		/*00000000,1107,Cutoff,invert=0*/
		{ 0x1108,	0xBF800000},		/*BF800000,1108,0dB,invert=1*/
		{ 0x1109,	0x3F800000},		/*3F800000,1109,0dB,invert=0*/
		{ 0x110A,	0x3F800000},		/*3F800000,110A,0dB,invert=0*/
		{ 0x110B,	0x3F800000},		/*3F800000,110B,0dB,invert=0*/
		{ 0x110C,	0x3F800000},		/*3F800000,110C,0dB,invert=0*/
		{ 0x110E,	0x3F800000},		/*3F800000,110E,0dB,invert=0*/
		{ 0x1110,	0x3DA2AD80},		/*3DA2AD80,1110*/
		{ 0x1111,	0x00000000},		/*00000000,1111,Free,fs/1,invert=0*/
		{ 0x1112,	0x3F7FFE00},		/*3F7FFE00,1112,Free,fs/1,invert=0*/
		{ 0x1113,	0x402C5840},		/*402C5840,1113,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
		{ 0x1114,	0xC02A0D00},		/*C02A0D00,1114,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
		{ 0x1115,	0x3F65F240},		/*3F65F240,1115,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
		{ 0x1116,	0x3F530B40},		/*3F530B40,1116,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
		{ 0x1117,	0xBF52FF00},		/*BF52FF00,1117,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
		{ 0x1118,	0x3F7FF400},		/*3F7FF400,1118,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
		{ 0x1119,	0x3F800000},		/*3F800000,1119,Through,0dB,fs/1,invert=0*/
		{ 0x111A,	0x00000000},		/*00000000,111A,Through,0dB,fs/1,invert=0*/
		{ 0x111B,	0x00000000},		/*00000000,111B,Through,0dB,fs/1,invert=0*/
		{ 0x111C,	0x3F800000},		/*3F800000,111C,0dB,invert=0*/
		{ 0x111D,	0x00000000},		/*00000000,111D,Cutoff,invert=0*/
		{ 0x111E,	0x3F800000},		/*3F800000,111E,0dB,invert=0*/
		{ 0x1120,	0x3F800000},		/*3F800000,1120,0dB,invert=0*/
		{ 0x1121,	0x3F800000},		/*3F800000,1121,0dB,invert=0*/
		{ 0x1122,	0x3F800000},		/*3F800000,1122,0dB,invert=0*/
		{ 0x1123,	0x3F800000},		/*3F800000,1123,Through,0dB,fs/1,invert=0*/
		{ 0x1124,	0x00000000},		/*00000000,1124,Through,0dB,fs/1,invert=0*/
		{ 0x1125,	0x00000000},		/*00000000,1125,Through,0dB,fs/1,invert=0*/
		{ 0x1126,	0x00000000},		/*00000000,1126,Through,0dB,fs/1,invert=0*/
		{ 0x1127,	0x00000000},		/*00000000,1127,Through,0dB,fs/1,invert=0*/
		{ 0x1130,	0x3CA175C0},		/*3CA175C0,1130,LPF,150Hz,0dB,fs/1,invert=0*/
		{ 0x1131,	0x3CA175C0},		/*3CA175C0,1131,LPF,150Hz,0dB,fs/1,invert=0*/
		{ 0x1132,	0x3F75E8C0},		/*3F75E8C0,1132,LPF,150Hz,0dB,fs/1,invert=0*/
		{ 0x1133,	0x3F800000},		/*3F800000,1133,Through,0dB,fs/1,invert=0*/
		{ 0x1134,	0x00000000},		/*00000000,1134,Through,0dB,fs/1,invert=0*/
		{ 0x1135,	0x00000000},		/*00000000,1135,Through,0dB,fs/1,invert=0*/
		{ 0x1136,	0x4F7FFFC0},		/*4F7FFFC0,1136,Free,fs/1,invert=0*/
		{ 0x1137,	0x00000000},		/*00000000,1137,Free,fs/1,invert=0*/
		{ 0x1138,	0x00000000},		/*00000000,1138,Free,fs/1,invert=0*/
		{ 0x1139,	0x00000000},		/*00000000,1139,Free,fs/1,invert=0*/
		{ 0x113A,	0x30800000},		/*30800000,113A,Free,fs/1,invert=0*/
		{ 0x113B,	0x00000000},		/*00000000,113B,Free,fs/1,invert=0*/
		{ 0x113C,	0x3F800000},		/*3F800000,113C,Through,0dB,fs/1,invert=0*/
		{ 0x113D,	0x00000000},		/*00000000,113D,Through,0dB,fs/1,invert=0*/
		{ 0x113E,	0x00000000},		/*00000000,113E,Through,0dB,fs/1,invert=0*/
		{ 0x1143,	0x39D2BD40},		/*39D2BD40,1143,LPF,3Hz,0dB,fs/1,invert=0*/
		{ 0x1144,	0x39D2BD40},		/*39D2BD40,1144,LPF,3Hz,0dB,fs/1,invert=0*/
		{ 0x1145,	0x3F7FCB40},		/*3F7FCB40,1145,LPF,3Hz,0dB,fs/1,invert=0*/
		{ 0x1146,	0x388C8A40},		/*388C8A40,1146,LPF,0.5Hz,0dB,fs/1,invert=0*/
		{ 0x1147,	0x388C8A40},		/*388C8A40,1147,LPF,0.5Hz,0dB,fs/1,invert=0*/
		{ 0x1148,	0x3F7FF740},		/*3F7FF740,1148,LPF,0.5Hz,0dB,fs/1,invert=0*/
		{ 0x1149,	0x390C87C0},		/*390C87C0,1149,LPF,1Hz,0dB,fs/1,invert=0*/
		{ 0x114A,	0x390C87C0},		/*390C87C0,114A,LPF,1Hz,0dB,fs/1,invert=0*/
		{ 0x114B,	0x3F7FEE80},		/*3F7FEE80,114B,LPF,1Hz,0dB,fs/1,invert=0*/
		{ 0x114C,	0x398C8300},		/*398C8300,114C,LPF,2Hz,0dB,fs/1,invert=0*/
		{ 0x114D,	0x398C8300},		/*398C8300,114D,LPF,2Hz,0dB,fs/1,invert=0*/
		{ 0x114E,	0x3F7FDCC0},		/*3F7FDCC0,114E,LPF,2Hz,0dB,fs/1,invert=0*/
		{ 0x1153,	0x3F800000},		/*3F800000,1153,Through,0dB,fs/1,invert=0*/
		{ 0x1154,	0x00000000},		/*00000000,1154,Through,0dB,fs/1,invert=0*/
		{ 0x1155,	0x00000000},		/*00000000,1155,Through,0dB,fs/1,invert=0*/
		{ 0x1156,	0x3F800000},		/*3F800000,1156,Through,0dB,fs/1,invert=0*/
		{ 0x1157,	0x00000000},		/*00000000,1157,Through,0dB,fs/1,invert=0*/
		{ 0x1158,	0x00000000},		/*00000000,1158,Through,0dB,fs/1,invert=0*/
		{ 0x1159,	0x3F800000},		/*3F800000,1159,Through,0dB,fs/1,invert=0*/
		{ 0x115A,	0x00000000},		/*00000000,115A,Through,0dB,fs/1,invert=0*/
		{ 0x115B,	0x00000000},		/*00000000,115B,Through,0dB,fs/1,invert=0*/
		{ 0x115C,	0x3F800000},		/*3F800000,115C,Through,0dB,fs/1,invert=0*/
		{ 0x115D,	0x00000000},		/*00000000,115D,Through,0dB,fs/1,invert=0*/
		{ 0x115E,	0x00000000},		/*00000000,115E,Through,0dB,fs/1,invert=0*/
		{ 0x1163,	0x3F800000},		/*3F800000,1163,0dB,invert=0*/
		{ 0x1166,	0x3F800000},		/*3F800000,1166,0dB,invert=0*/
		{ 0x1169,	0x3F800000},		/*3F800000,1169,0dB,invert=0*/
		{ 0x116C,	0x3F800000},		/*3F800000,116C,0dB,invert=0*/
		{ 0x1173,	0x00000000},		/*00000000,1173,Cutoff,invert=0*/
		{ 0x1176,	0x3F800000},		/*3F800000,1176,0dB,invert=0*/
		{ 0x1179,	0x3F800000},		/*3F800000,1179,0dB,invert=0*/
		{ 0x117C,	0x3F800000},		/*3F800000,117C,0dB,invert=0*/
		{ 0x1183,	0x38D1B700},		/*38D1B700,1183,-80dB,invert=0*/
		{ 0x1186,	0x00000000},		/*00000000,1186,Cutoff,invert=0*/
		{ 0x1189,	0x00000000},		/*00000000,1189,Cutoff,invert=0*/
		{ 0x118C,	0x00000000},		/*00000000,118C,Cutoff,invert=0*/
		{ 0x1193,	0x00000000},		/*00000000,1193,Cutoff,invert=0*/
		{ 0x1198,	0x3F800000},		/*3F800000,1198,0dB,invert=0*/
		{ 0x1199,	0x3F800000},		/*3F800000,1199,0dB,invert=0*/
		{ 0x119A,	0x3F800000},		/*3F800000,119A,0dB,invert=0*/
		{ 0x11A1,	0x3BDA2580},		/*3BDA2580,11A1,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11A2,	0x3BDA2580},		/*3BDA2580,11A2,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11A3,	0x3F7C9780},		/*3F7C9780,11A3,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11A4,	0x00000000},		/*00000000,11A4,Free,fs/1,invert=0*/
		{ 0x11A5,	0x3A031240},		/*3A031240,11A5,Free,fs/1,invert=0*/
		{ 0x11A6,	0x3F800000},		/*3F800000,11A6,Free,fs/1,invert=0*/
		{ 0x11A7,	0x3F800000},		/*3F800000,11A7,Through,0dB,fs/1,invert=0*/
		{ 0x11A8,	0x00000000},		/*00000000,11A8,Through,0dB,fs/1,invert=0*/
		{ 0x11A9,	0x00000000},		/*00000000,11A9,Through,0dB,fs/1,invert=0*/
		{ 0x11AA,	0x00000000},		/*00000000,11AA,Cutoff,invert=0*/
		{ 0x11AB,	0x3BDA2580},		/*3BDA2580,11AB,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11AC,	0x3BDA2580},		/*3BDA2580,11AC,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11AD,	0x3F7C9780},		/*3F7C9780,11AD,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11B0,	0x3E8081C0},		/*3E8081C0,11B0,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11B1,	0x3E8081C0},		/*3E8081C0,11B1,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11B2,	0x3EFEFC80},		/*3EFEFC80,11B2,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11B3,	0x3F800000},		/*3F800000,11B3,0dB,invert=0*/
		{ 0x11B4,	0x00000000},		/*00000000,11B4,Cutoff,invert=0*/
		{ 0x11B5,	0x00000000},		/*00000000,11B5,Cutoff,invert=0*/
		{ 0x11B6,	0x3F800000},		/*3F800000,11B6,0dB,invert=0*/
		{ 0x11B8,	0x3F800000},		/*3F800000,11B8,0dB,invert=0*/
		{ 0x11B9,	0x00000000},		/*00000000,11B9,Cutoff,invert=0*/
		{ 0x11C0,	0x3F708000},		/*3F708000,11C0,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
		{ 0x11C1,	0xBF6C5880},		/*BF6C5880,11C1,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
		{ 0x11C2,	0x3F5CD8C0},		/*3F5CD8C0,11C2,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
		{ 0x11C3,	0x3F800000},		/*3F800000,11C3,Through,0dB,fs/1,invert=0*/
		{ 0x11C4,	0x00000000},		/*00000000,11C4,Through,0dB,fs/1,invert=0*/
		{ 0x11C5,	0x00000000},		/*00000000,11C5,Through,0dB,fs/1,invert=0*/
		{ 0x11C6,	0x3D506F00},		/*3D506F00,11C6,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C7,	0x3D506F00},		/*3D506F00,11C7,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C8,	0x3F65F240},		/*3F65F240,11C8,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C9,	0x3C62BC00},		/*3C62BC00,11C9,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CA,	0x3C62BC00},		/*3C62BC00,11CA,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CB,	0x3F7FE940},		/*3F7FE940,11CB,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CC,	0x3DF7D400},		/*3DF7D400,11CC,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
		{ 0x11CD,	0xBDF5A540},		/*BDF5A540,11CD,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
		{ 0x11CE,	0x3F7F5080},		/*3F7F5080,11CE,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
		{ 0x11D0,	0x3FFF64C0},		/*3FFF64C0,11D0,6dB,invert=0*/
		{ 0x11D1,	0x00000000},		/*00000000,11D1,Cutoff,invert=0*/
		{ 0x11D2,	0x3F800000},		/*3F800000,11D2,0dB,invert=0*/
		{ 0x11D3,	0x3ECBD4C0},		/*3ECBD4C0,11D3,-8dB,invert=0*/
		{ 0x11D4,	0x3F800000},		/*3F800000,11D4,0dB,invert=0*/
		{ 0x11D5,	0x3F800000},		/*3F800000,11D5,0dB,invert=0*/
		{ 0x11D7,	0x40B3CE80},		/*40B3CE80,11D7,LPF,2500Hz,27dB,fs/1,invert=0*/
		{ 0x11D8,	0x40B3CE80},		/*40B3CE80,11D8,LPF,2500Hz,27dB,fs/1,invert=0*/
		{ 0x11D9,	0x3EFEFC80},		/*3EFEFC80,11D9,LPF,2500Hz,27dB,fs/1,invert=0*/
		{ 0x11DA,	0x3F792280},		/*3F792280,11DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DB,	0xBFEBE280},		/*BFEBE280,11DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DC,	0x3FEBE280},		/*3FEBE280,11DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DD,	0x3F6B5700},		/*3F6B5700,11DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DE,	0xBF6479C0},		/*BF6479C0,11DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11E0,	0x3F800000},		/*3F800000,11E0,Through,0dB,fs/1,invert=0*/
		{ 0x11E1,	0x00000000},		/*00000000,11E1,Through,0dB,fs/1,invert=0*/
		{ 0x11E2,	0x00000000},		/*00000000,11E2,Through,0dB,fs/1,invert=0*/
		{ 0x11E3,	0x00000000},		/*00000000,11E3,Through,0dB,fs/1,invert=0*/
		{ 0x11E4,	0x00000000},		/*00000000,11E4,Through,0dB,fs/1,invert=0*/
		{ 0x11E5,	0x3F800000},		/*3F800000,11E5,0dB,invert=0*/
		{ 0x11E8,	0x3F800000},		/*3F800000,11E8,0dB,invert=0*/
		{ 0x11E9,	0x00000000},		/*00000000,11E9,Cutoff,invert=0*/
		{ 0x11EA,	0x00000000},		/*00000000,11EA,Cutoff,invert=0*/
		{ 0x11EB,	0x00000000},		/*00000000,11EB,Cutoff,invert=0*/
		{ 0x11F0,	0x3F800000},		/*3F800000,11F0,Through,0dB,fs/1,invert=0*/
		{ 0x11F1,	0x00000000},		/*00000000,11F1,Through,0dB,fs/1,invert=0*/
		{ 0x11F2,	0x00000000},		/*00000000,11F2,Through,0dB,fs/1,invert=0*/
		{ 0x11F3,	0x00000000},		/*00000000,11F3,Through,0dB,fs/1,invert=0*/
		{ 0x11F4,	0x00000000},		/*00000000,11F4,Through,0dB,fs/1,invert=0*/
		{ 0x11F5,	0x3F800000},		/*3F800000,11F5,Through,0dB,fs/1,invert=0*/
		{ 0x11F6,	0x00000000},		/*00000000,11F6,Through,0dB,fs/1,invert=0*/
		{ 0x11F7,	0x00000000},		/*00000000,11F7,Through,0dB,fs/1,invert=0*/
		{ 0x11F8,	0x00000000},		/*00000000,11F8,Through,0dB,fs/1,invert=0*/
		{ 0x11F9,	0x00000000},		/*00000000,11F9,Through,0dB,fs/1,invert=0*/
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
		{ 0xFFFF,	0xFFFFFFFF }
	} ;
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
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1003,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1004,Cutoff,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1005,Cutoff,fs/1,invert=0*/
	0x10, 0x06, 	/* 0x1006 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1006,Cutoff,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1007,Cutoff,invert=0*/
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
	 0x3D, 0xA2, 0xAD, 0x80, 	 /*3DA2AD80,1010*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1011,Free,fs/1,invert=0*/
	 0x3F, 0x7F, 0xFE, 0x00, 	 /*3F7FFE00,1012,Free,fs/1,invert=0*/
	0x10, 0x13, 	/* 0x1013 */
	 0x40, 0x2C, 0x58, 0x40, 	 /*402C5840,1013,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
	 0xC0, 0x2A, 0x0D, 0x00, 	 /*C02A0D00,1014,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
	 0x3F, 0x65, 0xF2, 0x40, 	 /*3F65F240,1015,HBF,50Hz,400Hz,9dB,fs/1,invert=0*/
	0x10, 0x16, 	/* 0x1016 */
	 0x3F, 0x53, 0x0B, 0x40, 	 /*3F530B40,1016,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
	 0xBF, 0x52, 0xFF, 0x00, 	 /*BF52FF00,1017,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0xF4, 0x00, 	 /*3F7FF400,1018,LBF,0.69Hz,0.837Hz,0dB,fs/1,invert=0*/
	0x10, 0x19, 	/* 0x1019 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1019,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,101A,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,101B,Through,0dB,fs/1,invert=0*/
	0x10, 0x1C, 	/* 0x101C */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,101C,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,101D,Cutoff,invert=0*/
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
	 0x3C, 0xA1, 0x75, 0xC0, 	 /*3CA175C0,1030,LPF,150Hz,0dB,fs/1,invert=0*/
	 0x3C, 0xA1, 0x75, 0xC0, 	 /*3CA175C0,1031,LPF,150Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x75, 0xE8, 0xC0, 	 /*3F75E8C0,1032,LPF,150Hz,0dB,fs/1,invert=0*/
	0x10, 0x33, 	/* 0x1033 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1033,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1034,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1035,Through,0dB,fs/1,invert=0*/
	0x10, 0x36, 	/* 0x1036 */
	 0x4F, 0x7F, 0xFF, 0xC0, 	 /*4F7FFFC0,1036,Free,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1037,Free,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1038,Free,fs/1,invert=0*/
	0x10, 0x39, 	/* 0x1039 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1039,Free,fs/1,invert=0*/
	 0x30, 0x80, 0x00, 0x00, 	 /*30800000,103A,Free,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,103B,Free,fs/1,invert=0*/
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
	 0x3B, 0xDA, 0x25, 0x80, 	 /*3BDA2580,10A1,LPF,50Hz,0dB,fs/1,invert=0*/
	 0x3B, 0xDA, 0x25, 0x80, 	 /*3BDA2580,10A2,LPF,50Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7C, 0x97, 0x80, 	 /*3F7C9780,10A3,LPF,50Hz,0dB,fs/1,invert=0*/
	0x10, 0xA4, 	/* 0x10A4 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10A4,Free,fs/1,invert=0*/
	 0x3A, 0x03, 0x12, 0x40, 	 /*3A031240,10A5,Free,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10A6,Free,fs/1,invert=0*/
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
	 0x3F, 0x70, 0x80, 0x00, 	 /*3F708000,10C0,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
	 0xBF, 0x6C, 0x58, 0x80, 	 /*BF6C5880,10C1,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x5C, 0xD8, 0xC0, 	 /*3F5CD8C0,10C2,HBF,65Hz,550Hz,0dB,fs/1,invert=0*/
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
	 0x3D, 0xF7, 0xD4, 0x00, 	 /*3DF7D400,10CC,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
	 0xBD, 0xF5, 0xA5, 0x40, 	 /*BDF5A540,10CD,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
	 0x3F, 0x7F, 0x50, 0x80, 	 /*3F7F5080,10CE,LBF,10Hz,33Hz,-8dB,fs/1,invert=0*/
	0x10, 0xD0, 	/* 0x10D0 */
	 0x3F, 0xFF, 0x64, 0xC0, 	 /*3FFF64C0,10D0,6dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10D1,Cutoff,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10D2,0dB,invert=0*/
	0x10, 0xD3, 	/* 0x10D3 */
	 0x3E, 0xCB, 0xD4, 0xC0, 	 /*3ECBD4C0,10D3,-8dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10D4,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10D5,0dB,invert=0*/
	0x10, 0xD7, 	/* 0x10D7 */
	 0x40, 0xB3, 0xCE, 0x80, 	 /*40B3CE80,10D7,LPF,2500Hz,27dB,fs/1,invert=0*/
	 0x40, 0xB3, 0xCE, 0x80, 	 /*40B3CE80,10D8,LPF,2500Hz,27dB,fs/1,invert=0*/
	 0x3E, 0xFE, 0xFC, 0x80, 	 /*3EFEFC80,10D9,LPF,2500Hz,27dB,fs/1,invert=0*/
	0x10, 0xDA, 	/* 0x10DA */
	 0x3F, 0x79, 0x22, 0x80, 	 /*3F792280,10DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	 0xBF, 0xEB, 0xE2, 0x80, 	 /*BFEBE280,10DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	 0x3F, 0xEB, 0xE2, 0x80, 	 /*3FEBE280,10DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	0x10, 0xDD, 	/* 0x10DD */
	 0x3F, 0x6B, 0x57, 0x00, 	 /*3F6B5700,10DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	 0xBF, 0x64, 0x79, 0xC0, 	 /*BF6479C0,10DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
	0x10, 0xE0, 	/* 0x10E0 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,10E0,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E1,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E2,Through,0dB,fs/1,invert=0*/
	0x10, 0xE3, 	/* 0x10E3 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E3,Through,0dB,fs/1,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,10E4,Through,0dB,fs/1,invert=0*/
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
