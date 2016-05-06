/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#include <linux/init.h>
#include <linux/io.h>
#define		K7_OISINI
#include "imx278_lgit_ois.h"
#include "imx278_lgit_ois_data.h"
#include "../mini_isp_ois_interface.h"


//#include <core/math.h>

/*
    all interface:
        success return 0
        fail return -1
*/
#define RamWrite32A(a,b)   do{ \
    if(mini_isp_ois_RamWrite32A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define RamRead32A(a,b)          do{ \
    if(mini_isp_ois_RamRead32A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define RamWriteA(a,b)           do{ \
    if(mini_isp_ois_RamWrite16A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define RamReadA(a,b)            do{ \
    if(mini_isp_ois_RamRead16A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define RegWriteA(a,b)           do{ \
    if(mini_isp_ois_RegWrite8A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define RegReadA(a,b)            do{ \
    if(mini_isp_ois_RegRead8A(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define WitTim              mini_isp_ois_WitTim

#define CntWrtRam(a,b)            do{ \
    if(mini_isp_ois_RamWrite16Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define CntWrtRag(a,b)           do{ \
    if(mini_isp_ois_RegWrite8Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define CntWrtRam32(a,b)         do{ \
    if(mini_isp_ois_RamWrite32Burst(a,b) != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
    } \
}while(0)

#define CHECK_RETURN(a) do{ \
    if(a != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0);


//**************************
//	Global Variable
//**************************
//unsigned char spigyrocheck=0x00;

#define	OIS_FW_POLLING_PASS		0
#define	OIS_FW_POLLING_FAIL		-1
#define	OIS_FW_POLLING_VERSION_FAIL		-2
#define	CLRGYR_POLLING_LIMIT_A	6
#define	ACCWIT_POLLING_LIMIT_A	6
#define	INIDGY_POLLING_LIMIT_A	12
#define INIDGY_POLLING_LIMIT_B	12

#define FILREGTAB	7
#define	XY_SIMU_SET
#ifdef	XY_SIMU_SET
	#define FILRAMTAB	200
#else	//XY_SIMU_SET
	#define FILRAMTAB	363
#endif	//XY_SIMU_SET

#ifdef	CATCHMODE

/*Filter Calculator Version 4.02*/
/*the time and date : 2015/4/24 18:23:14*/
/*FC filename : LC898122_TDK95_V0009_d_1div*/
/*fs,23438Hz*/
/*LSI No.,LC898122*/
/*Comment,*/

#ifdef	INI_SHORT1
/* 8bit */
K7_OISINI__ const unsigned char CsFilRegDat_D0D0[] = { 
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

K7_OISINI__ const unsigned char CsFilReg_D0D0[] = {
	 3, 4, 3, 4, 0xFF 
}; 

/* 32bit */
K7_OISINI__ const unsigned char CsFilRamDat_D0D0[] = { 
	0x10, 0x00, 	/* 0x1000 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1000,0dB,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1001,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1002,Cutoff,invert=0*/
	0x10, 0x03, 	/* 0x1003 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1003,0dB,invert=0*/
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1004,Free,fs/1,invert=0*/
	 0x3A, 0x03, 0x12, 0x40, 	 /*3A031240,1005,Free,fs/1,invert=0*/
	0x10, 0x06, 	/* 0x1006 */
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1006,Free,fs/1,invert=0*/
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
//	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1022,0dB,invert=0*/
	 0x3F, 0xEC, 0x5C, 0x40, 	 /*3FEC5C40,1022,5.3273dB,invert=0*/
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
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1043,Free,fs/1,invert=0*/
	 0x39, 0x83, 0x12, 0x40, 	 /*39831240,1044,Free,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1045,Free,fs/1,invert=0*/
	0x10, 0x46, 	/* 0x1046 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1046,Free,fs/1,invert=0*/
	 0x3A, 0x83, 0x12, 0x40, 	 /*3A831240,1047,Free,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,1048,Free,fs/1,invert=0*/
	0x10, 0x49, 	/* 0x1049 */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,1049,Free,fs/1,invert=0*/
	 0x3A, 0xC4, 0x9B, 0x80, 	 /*3AC49B80,104A,Free,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,104B,Free,fs/1,invert=0*/
	0x10, 0x4C, 	/* 0x104C */
	 0x00, 0x00, 0x00, 0x00, 	 /*00000000,104C,Free,fs/1,invert=0*/
	 0x3B, 0x03, 0x12, 0x40, 	 /*3B031240,104D,Free,fs/1,invert=0*/
	 0x3F, 0x80, 0x00, 0x00, 	 /*3F800000,104E,Free,fs/1,invert=0*/
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
	 0x3B, 0xDA, 0x25, 0x80, 	 /*3BDA2580,10A4,LPF,50Hz,0dB,fs/1,invert=0*/
	 0x3B, 0xDA, 0x25, 0x80, 	 /*3BDA2580,10A5,LPF,50Hz,0dB,fs/1,invert=0*/
	 0x3F, 0x7C, 0x97, 0x80, 	 /*3F7C9780,10A6,LPF,50Hz,0dB,fs/1,invert=0*/
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

K7_OISINI__ const unsigned char CsFilRam_D0D0[] = {
	 14, 14, 14, 14, 6, 6, 14, 14, 14, 14, 14, 14, 14, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 14, 14, 14, 14, 14, 6, 14, 14, 6, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14, 10, 14, 14, 14, 6, 14, 14, 14, 6, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 10, 14, 6, 0xFF 
}; 

#else	//INI_SHORT1

/* 8bit */
K7_OISINI__ const struct STFILREG	CsFilReg_D0D0[FILREGTAB]	= {
	
		{ 0x0111,	0x00},		/*00,0111*/
		{ 0x0113,	0x00},		/*00,0113*/
		{ 0x0114,	0x00},		/*00,0114*/
		{ 0x0172,	0x00},		/*00,0172*/
		{ 0x01E3,	0x00},		/*00,01E3*/
		{ 0x01E4,	0x00},		/*00,01E4*/
		{ 0xFFFF,	0xFF }
	} ;

/* 32bit */
K7_OISINI__ const struct STFILRAM	CsFilRam_D0D0[FILRAMTAB]	= {
	
		{ 0x1000,	0x3F800000},		/*3F800000,1000,0dB,invert=0*/
		{ 0x1001,	0x3F800000},		/*3F800000,1001,0dB,invert=0*/
		{ 0x1002,	0x00000000},		/*00000000,1002,Cutoff,invert=0*/
		{ 0x1003,	0x3F800000},		/*3F800000,1003,0dB,invert=0*/
		{ 0x1004,	0x00000000},		/*00000000,1004,Free,fs/1,invert=0*/
		{ 0x1005,	0x3A031240},		/*3A031240,1005,Free,fs/1,invert=0*/
		{ 0x1006,	0x3F800000},		/*3F800000,1006,Free,fs/1,invert=0*/
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
		//{ 0x1022,	0x3F800000},		/*3F800000,1022,0dB,invert=0*/
		{ 0x1022,	0x3FEC5C40},		/*3FEC5C40,1022,5.3273dB,invert=0*/
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
		{ 0x1043,	0x00000000},		/*00000000,1043,Free,fs/1,invert=0*/
		{ 0x1044,	0x39831240},		/*39831240,1044,Free,fs/1,invert=0*/
		{ 0x1045,	0x3F800000},		/*3F800000,1045,Free,fs/1,invert=0*/
		{ 0x1046,	0x00000000},		/*00000000,1046,Free,fs/1,invert=0*/
		{ 0x1047,	0x3A831240},		/*3A831240,1047,Free,fs/1,invert=0*/
		{ 0x1048,	0x3F800000},		/*3F800000,1048,Free,fs/1,invert=0*/
		{ 0x1049,	0x00000000},		/*00000000,1049,Free,fs/1,invert=0*/
		{ 0x104A,	0x3AC49B80},		/*3AC49B80,104A,Free,fs/1,invert=0*/
		{ 0x104B,	0x3F800000},		/*3F800000,104B,Free,fs/1,invert=0*/
		{ 0x104C,	0x00000000},		/*00000000,104C,Free,fs/1,invert=0*/
		{ 0x104D,	0x3B031240},		/*3B031240,104D,Free,fs/1,invert=0*/
		{ 0x104E,	0x3F800000},		/*3F800000,104E,Free,fs/1,invert=0*/
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
		{ 0x10A4,	0x3BDA2580},		/*3BDA2580,10A4,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10A5,	0x3BDA2580},		/*3BDA2580,10A5,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x10A6,	0x3F7C9780},		/*3F7C9780,10A6,LPF,50Hz,0dB,fs/1,invert=0*/
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
#ifndef	XY_SIMU_SET
		{ 0x1100,	0x3F800000},		/*3F800000,1100,0dB,invert=0*/
		{ 0x1101,	0x3F800000},		/*3F800000,1101,0dB,invert=0*/
		{ 0x1102,	0x00000000},		/*00000000,1102,Cutoff,invert=0*/
		{ 0x1103,	0x3F800000},		/*3F800000,1103,0dB,invert=0*/
		{ 0x1104,	0x00000000},		/*00000000,1104,Free,fs/1,invert=0*/
		{ 0x1105,	0x3A031240},		/*3A031240,1105,Free,fs/1,invert=0*/
		{ 0x1106,	0x3F800000},		/*3F800000,1106,Free,fs/1,invert=0*/
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
		{ 0x1143,	0x00000000},		/*00000000,1143,Free,fs/1,invert=0*/
		{ 0x1144,	0x39831240},		/*39831240,1144,Free,fs/1,invert=0*/
		{ 0x1145,	0x3F800000},		/*3F800000,1145,Free,fs/1,invert=0*/
		{ 0x1146,	0x00000000},		/*00000000,1146,Free,fs/1,invert=0*/
		{ 0x1147,	0x3A831240},		/*3A831240,1147,Free,fs/1,invert=0*/
		{ 0x1148,	0x3F800000},		/*3F800000,1148,Free,fs/1,invert=0*/
		{ 0x1149,	0x00000000},		/*00000000,1149,Free,fs/1,invert=0*/
		{ 0x114A,	0x3AC49B80},		/*3AC49B80,114A,Free,fs/1,invert=0*/
		{ 0x114B,	0x3F800000},		/*3F800000,114B,Free,fs/1,invert=0*/
		{ 0x114C,	0x00000000},		/*00000000,114C,Free,fs/1,invert=0*/
		{ 0x114D,	0x3B031240},		/*3B031240,114D,Free,fs/1,invert=0*/
		{ 0x114E,	0x3F800000},		/*3F800000,114E,Free,fs/1,invert=0*/
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
		{ 0x11A4,	0x3BDA2580},		/*3BDA2580,11A4,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11A5,	0x3BDA2580},		/*3BDA2580,11A5,LPF,50Hz,0dB,fs/1,invert=0*/
		{ 0x11A6,	0x3F7C9780},		/*3F7C9780,11A6,LPF,50Hz,0dB,fs/1,invert=0*/
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
		{ 0x11C0,	0x3F696080},		/*3F696080,11C0,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x11C1,	0xBF6509C0},		/*BF6509C0,11C1,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x11C2,	0x3F4E6A40},		/*3F4E6A40,11C2,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x11C3,	0x3F800000},		/*3F800000,11C3,Through,0dB,fs/1,invert=0*/
		{ 0x11C4,	0x00000000},		/*00000000,11C4,Through,0dB,fs/1,invert=0*/
		{ 0x11C5,	0x00000000},		/*00000000,11C5,Through,0dB,fs/1,invert=0*/
		{ 0x11C6,	0x3D506F00},		/*3D506F00,11C6,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C7,	0x3D506F00},		/*3D506F00,11C7,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C8,	0x3F65F240},		/*3F65F240,11C8,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C9,	0x3C62BC00},		/*3C62BC00,11C9,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CA,	0x3C62BC00},		/*3C62BC00,11CA,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CB,	0x3F7FE940},		/*3F7FE940,11CB,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CC,	0x3DB5DCC0},		/*3DB5DCC0,11CC,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x11CD,	0xBDB41D80},		/*BDB41D80,11CD,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x11CE,	0x3F7F7380},		/*3F7F7380,11CE,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x11D0,	0x3FFF64C0},		/*3FFF64C0,11D0,6dB,invert=0*/
		{ 0x11D1,	0x00000000},		/*00000000,11D1,Cutoff,invert=0*/
		{ 0x11D2,	0x3F800000},		/*3F800000,11D2,0dB,invert=0*/
		{ 0x11D3,	0x3F004DC0},		/*3F004DC0,11D3,-6dB,invert=0*/
		{ 0x11D4,	0x3F800000},		/*3F800000,11D4,0dB,invert=0*/
		{ 0x11D5,	0x3F800000},		/*3F800000,11D5,0dB,invert=0*/
		{ 0x11D7,	0x417D9540},		/*417D9540,11D7,Through,24dB,fs/1,invert=0*/
		{ 0x11D8,	0x00000000},		/*00000000,11D8,Through,24dB,fs/1,invert=0*/
		{ 0x11D9,	0x00000000},		/*00000000,11D9,Through,24dB,fs/1,invert=0*/
		{ 0x11DA,	0x3F792280},		/*3F792280,11DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DB,	0xBFEBE280},		/*BFEBE280,11DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DC,	0x3FEBE280},		/*3FEBE280,11DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DD,	0x3F6B5700},		/*3F6B5700,11DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DE,	0xBF6479C0},		/*BF6479C0,11DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11E0,	0x3E8081C0},		/*3E8081C0,11E0,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E1,	0x3E8081C0},		/*3E8081C0,11E1,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E2,	0x3EFEFC80},		/*3EFEFC80,11E2,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E3,	0x00000000},		/*00000000,11E3,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E4,	0x00000000},		/*00000000,11E4,LPF,2500Hz,0dB,fs/1,invert=0*/
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
#endif	//XY_SIMU_SET
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
	
} ;

#endif	//INI_SHORT1

#else	//CATCHMODE

/*Filter Calculator Version 4.02*/
/*the time and date : 2014/10/22 18:23:00*/
/*FC filename : LC898122_FIL_HUAWEI_TDK_V0002*/
/*fs,23438Hz*/
/*LSI No.,LC898122*/
/*Comment,*/

#ifdef	INI_SHORT1
/* 8bit */
K7_OISINI__ const unsigned char CsFilRegDat_D0D0[] = { 
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

K7_OISINI__ const unsigned char CsFilReg_D0D0[] = {
	 3, 4, 3, 4, 0xFF 
}; 

/* 32bit */
K7_OISINI__ const unsigned char CsFilRamDat_D0D0[] = { 
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

K7_OISINI__ const unsigned char CsFilRam_D0D0[] = {
	 14, 14, 14, 14, 6, 6, 14, 14, 14, 14, 14, 14, 14, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 14, 14, 14, 14, 14, 6, 14, 14, 6, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14, 10, 14, 14, 14, 6, 14, 14, 14, 6, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 10, 14, 6, 0xFF 
}; 

#else	//INI_SHORT1

/* 8bit */
K7_OISINI__ const struct STFILREG	CsFilReg_D0D0[FILREGTAB]	= {
	
		{ 0x0111,	0x00},		/*00,0111*/
		{ 0x0113,	0x00},		/*00,0113*/
		{ 0x0114,	0x00},		/*00,0114*/
		{ 0x0172,	0x00},		/*00,0172*/
		{ 0x01E3,	0x00},		/*00,01E3*/
		{ 0x01E4,	0x00},		/*00,01E4*/
		{ 0xFFFF,	0xFF }
	
} ;

/* 32bit */
K7_OISINI__ const struct STFILRAM	CsFilRam_D0D0[FILRAMTAB]	= {
	
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
#ifndef	XY_SIMU_SET
		{ 0x1100,	0x3F800000},		/*3F800000,1100,0dB,invert=0*/
		{ 0x1101,	0x3F800000},		/*3F800000,1101,0dB,invert=0*/
		{ 0x1102,	0x00000000},		/*00000000,1102,Cutoff,invert=0*/
		{ 0x1103,	0x3EBFED40},		/*3EBFED40,1103,-8.5227dB,invert=0*/
		{ 0x1104,	0x3860DE00},		/*3860DE00,1104,LPF,0.4Hz,0dB,fs/1,invert=0*/
		{ 0x1105,	0x3860DE00},		/*3860DE00,1105,LPF,0.4Hz,0dB,fs/1,invert=0*/
		{ 0x1106,	0x3F7FF900},		/*3F7FF900,1106,LPF,0.4Hz,0dB,fs/1,invert=0*/
		{ 0x1107,	0x3F800000},		/*3F800000,1107,0dB,invert=0*/
		{ 0x1108,	0xBF800000},		/*BF800000,1108,0dB,invert=1*/
		{ 0x1109,	0x3F800000},		/*3F800000,1109,0dB,invert=0*/
		{ 0x110A,	0x3F800000},		/*3F800000,110A,0dB,invert=0*/
		{ 0x110B,	0x3F800000},		/*3F800000,110B,0dB,invert=0*/
		{ 0x110C,	0x3F800000},		/*3F800000,110C,0dB,invert=0*/
		{ 0x110E,	0x3F800000},		/*3F800000,110E,0dB,invert=0*/
		{ 0x1110,	0x3DA2ADC0},		/*3DA2ADC0,1110*/
		{ 0x1111,	0x00000000},		/*00000000,1111,Free,fs/1,invert=0*/
		{ 0x1112,	0x3F7FFD80},		/*3F7FFD80,1112,Free,fs/1,invert=0*/
		{ 0x1113,	0x3F76D280},		/*3F76D280,1113,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
		{ 0x1114,	0xBF723C00},		/*BF723C00,1114,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
		{ 0x1115,	0x3F690E80},		/*3F690E80,1115,HBF,70Hz,350Hz,0dB,fs/1,invert=0*/
		{ 0x1116,	0x40FE2B00},		/*40FE2B00,1116,HPF,0.5Hz,18dB,fs/1,invert=0*/
		{ 0x1117,	0xC0FE2B00},		/*C0FE2B00,1117,HPF,0.5Hz,18dB,fs/1,invert=0*/
		{ 0x1118,	0x3F7FF740},		/*3F7FF740,1118,HPF,0.5Hz,18dB,fs/1,invert=0*/
		{ 0x1119,	0x3ED1D880},		/*3ED1D880,1119,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
		{ 0x111A,	0xBED1CFC0},		/*BED1CFC0,111A,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
		{ 0x111B,	0x3F7FFB80},		/*3F7FFB80,111B,LBF,0.25Hz,0.61Hz,0dB,fs/1,invert=0*/
		{ 0x111C,	0x00000000},		/*00000000,111C,Cutoff,invert=0*/
		{ 0x111D,	0x3F800000},		/*3F800000,111D,0dB,invert=0*/
		{ 0x111E,	0x3F800000},		/*3F800000,111E,0dB,invert=0*/
		{ 0x1120,	0x3F800000},		/*3F800000,1120,0dB,invert=0*/
		{ 0x1121,	0x3F800000},		/*3F800000,1121,0dB,invert=0*/
		{ 0x1122,	0x3F800000},		/*3F800000,1122,0dB,invert=0*/
		{ 0x1123,	0x3F800000},		/*3F800000,1123,Through,0dB,fs/1,invert=0*/
		{ 0x1124,	0x00000000},		/*00000000,1124,Through,0dB,fs/1,invert=0*/
		{ 0x1125,	0x00000000},		/*00000000,1125,Through,0dB,fs/1,invert=0*/
		{ 0x1126,	0x00000000},		/*00000000,1126,Through,0dB,fs/1,invert=0*/
		{ 0x1127,	0x00000000},		/*00000000,1127,Through,0dB,fs/1,invert=0*/
		{ 0x1130,	0x3F800000},		/*3F800000,1130,Through,0dB,fs/1,invert=0*/
		{ 0x1131,	0x00000000},		/*00000000,1131,Through,0dB,fs/1,invert=0*/
		{ 0x1132,	0x00000000},		/*00000000,1132,Through,0dB,fs/1,invert=0*/
		{ 0x1133,	0x3F800000},		/*3F800000,1133,Through,0dB,fs/1,invert=0*/
		{ 0x1134,	0x00000000},		/*00000000,1134,Through,0dB,fs/1,invert=0*/
		{ 0x1135,	0x00000000},		/*00000000,1135,Through,0dB,fs/1,invert=0*/
		{ 0x1136,	0x3F800000},		/*3F800000,1136,Through,0dB,fs/1,invert=0*/
		{ 0x1137,	0x00000000},		/*00000000,1137,Through,0dB,fs/1,invert=0*/
		{ 0x1138,	0x00000000},		/*00000000,1138,Through,0dB,fs/1,invert=0*/
		{ 0x1139,	0x3F800000},		/*3F800000,1139,Through,0dB,fs/1,invert=0*/
		{ 0x113A,	0x00000000},		/*00000000,113A,Through,0dB,fs/1,invert=0*/
		{ 0x113B,	0x00000000},		/*00000000,113B,Through,0dB,fs/1,invert=0*/
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
		{ 0x11A1,	0x3C58B440},		/*3C58B440,11A1,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x11A2,	0x3C58B440},		/*3C58B440,11A2,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x11A3,	0x3F793A40},		/*3F793A40,11A3,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x11A4,	0x3C58B440},		/*3C58B440,11A4,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x11A5,	0x3C58B440},		/*3C58B440,11A5,LPF,100Hz,0dB,fs/1,invert=0*/
		{ 0x11A6,	0x3F793A40},		/*3F793A40,11A6,LPF,100Hz,0dB,fs/1,invert=0*/
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
		{ 0x11C0,	0x3F696080},		/*3F696080,11C0,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x11C1,	0xBF6509C0},		/*BF6509C0,11C1,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x11C2,	0x3F4E6A40},		/*3F4E6A40,11C2,HBF,70Hz,800Hz,0dB,fs/1,invert=0*/
		{ 0x11C3,	0x3F800000},		/*3F800000,11C3,Through,0dB,fs/1,invert=0*/
		{ 0x11C4,	0x00000000},		/*00000000,11C4,Through,0dB,fs/1,invert=0*/
		{ 0x11C5,	0x00000000},		/*00000000,11C5,Through,0dB,fs/1,invert=0*/
		{ 0x11C6,	0x3D506F00},		/*3D506F00,11C6,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C7,	0x3D506F00},		/*3D506F00,11C7,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C8,	0x3F65F240},		/*3F65F240,11C8,LPF,400Hz,0dB,fs/1,invert=0*/
		{ 0x11C9,	0x3C62BC00},		/*3C62BC00,11C9,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CA,	0x3C62BC00},		/*3C62BC00,11CA,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CB,	0x3F7FE940},		/*3F7FE940,11CB,LPF,1.3Hz,38dB,fs/1,invert=0*/
		{ 0x11CC,	0x3DB5DCC0},		/*3DB5DCC0,11CC,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x11CD,	0xBDB41D80},		/*BDB41D80,11CD,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x11CE,	0x3F7F7380},		/*3F7F7380,11CE,LBF,8Hz,36Hz,-8dB,fs/1,invert=0*/
		{ 0x11D0,	0x3FFF64C0},		/*3FFF64C0,11D0,6dB,invert=0*/
		{ 0x11D1,	0x00000000},		/*00000000,11D1,Cutoff,invert=0*/
		{ 0x11D2,	0x3F800000},		/*3F800000,11D2,0dB,invert=0*/
		{ 0x11D3,	0x3F004DC0},		/*3F004DC0,11D3,-6dB,invert=0*/
		{ 0x11D4,	0x3F800000},		/*3F800000,11D4,0dB,invert=0*/
		{ 0x11D5,	0x3F800000},		/*3F800000,11D5,0dB,invert=0*/
		{ 0x11D7,	0x417D9540},		/*417D9540,11D7,Through,24dB,fs/1,invert=0*/
		{ 0x11D8,	0x00000000},		/*00000000,11D8,Through,24dB,fs/1,invert=0*/
		{ 0x11D9,	0x00000000},		/*00000000,11D9,Through,24dB,fs/1,invert=0*/
		{ 0x11DA,	0x3F792280},		/*3F792280,11DA,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DB,	0xBFEBE280},		/*BFEBE280,11DB,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DC,	0x3FEBE280},		/*3FEBE280,11DC,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DD,	0x3F6B5700},		/*3F6B5700,11DD,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11DE,	0xBF6479C0},		/*BF6479C0,11DE,PKF,860Hz,-6dB,5,fs/1,invert=0*/
		{ 0x11E0,	0x3E8081C0},		/*3E8081C0,11E0,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E1,	0x3E8081C0},		/*3E8081C0,11E1,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E2,	0x3EFEFC80},		/*3EFEFC80,11E2,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E3,	0x00000000},		/*00000000,11E3,LPF,2500Hz,0dB,fs/1,invert=0*/
		{ 0x11E4,	0x00000000},		/*00000000,11E4,LPF,2500Hz,0dB,fs/1,invert=0*/
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
#endif	//XY_SIMU_SET
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
	
} ;
#endif	//INI_SHORT1

#endif	//CATCHMODE


// DI Coefficient Setting Value
#define		COEFTBL	7
const unsigned long	ClDiCof[ COEFTBL ]	= {
	0x3F7FFD80,		/* 0 */
	0x3F7FFD80,		/* 1 */
	0x3F7FFD80,		/* 2 */
	0x3F7FFD80,		/* 3 */
	0x3F7FFD80,		/* 4 */
	0x3F7FFD80,		/* 5 */
	0x3F7FFD80		/* 6 */
} ;

//**************************
//	Global Variable			
//**************************
int OnsemiI2CCheck(void)
{
	unsigned char UcLsiVer;
	RegReadA( CVER, &UcLsiVer );		// 0x27E
	return (UcLsiVer == 0xA1) ? 1 : 0;	//In the case of using LC898122A
	//return (UcLsiVer == 0x93) ? 1 : 0;	//In the case of using LC898122A
}
#if 0//we no use
//********************************************************************************
// Function Name 	: IniSet
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
int	IniSet( void )
{
	if(OnsemiI2CCheck() == 0) return OIS_FW_POLLING_FAIL;
	
	//RegWriteA( SOFTRES2, 0x30 );
	RegWriteA( SOFTRES1, 0x30 );
	WitTim(5);
	// Clock Setting
	IniClk() ;
	// AF Initial Setting 
	IniAf()	;

	// Read E2PROM Data
	E2pDat() ;
	// Get Version Info.
	if( VerInf() != OIS_FW_POLLING_PASS ) return OIS_FW_POLLING_VERSION_FAIL;

#if 1
	// I/O Port Initial Setting
	IniIop() ;

#ifdef	INI_SHORT3
#else	//INI_SHORT3
	// Monitor & Other Initial Setting
	IniMon() ;
#endif	//INI_SHORT3

	// Servo Initial Setting
	IniSrv() ; //Hall Limiter Update AT High Version Info.
	// Gyro Initial Setting
	IniGyr() ;
	// Filter Initial Setting
	if( IniFil() != OIS_FW_POLLING_PASS ) return OIS_FW_POLLING_FAIL ;
	// DigitalGyro Initial Setting
	if( IniDgy() != OIS_FW_POLLING_PASS ) return OIS_FW_POLLING_FAIL ;
	// Adjust Fix Value Setting
	IniAdj() ;
#endif
	//RamAccFixMod( ON ) ;
	
	return OIS_FW_POLLING_PASS;
}

//********************************************************************************
// Function Name 	: E2pDat
// Return Value		: NON
// Argument Value	: NON
// Explanation		: E2PROM Calibration Data Read Function
// History			: First edition 						2013.06.21 Y.Kim
//********************************************************************************
void	E2pDat( void )
{
	unsigned char	UcGvcFlg ;

	MemClr( ( unsigned char * )&StCalDat, sizeof( stCalDat ) ) ;

	E2pRed( (unsigned short)0x093B	, 1, ( unsigned char * )&UcGvcFlg ) ;	//GYRO OFFSET Mobile Flag
	
	E2pRed( (unsigned short)HALL_BIAS_X,			2,	( unsigned char * )&StCalDat.StHalAdj.UsHlxGan ) ; WitTim(5);
	E2pRed( (unsigned short)HALL_BIAS_Y,			2,	( unsigned char * )&StCalDat.StHalAdj.UsHlyGan ) ; WitTim(5);

	E2pRed( (unsigned short)HALL_OFFSET_X,			2,	( unsigned char * )&StCalDat.StHalAdj.UsHlxOff ) ; WitTim(5);
	E2pRed( (unsigned short)HALL_OFFSET_Y,			2,	( unsigned char * )&StCalDat.StHalAdj.UsHlyOff ) ; WitTim(5);

	E2pRed( (unsigned short)LOOP_GAIN_X,			2,	( unsigned char * )&StCalDat.StLopGan.UsLxgVal ) ; WitTim(5);
	E2pRed( (unsigned short)LOOP_GAIN_Y,			2,	( unsigned char * )&StCalDat.StLopGan.UsLygVal ) ; WitTim(5);

	E2pRed( (unsigned short)LENS_CENTER_FINAL_X,	2,	( unsigned char * )&StCalDat.StLenCen.UsLsxVal ) ; WitTim(5);
	E2pRed( (unsigned short)LENS_CENTER_FINAL_Y,	2,	( unsigned char * )&StCalDat.StLenCen.UsLsyVal ) ; WitTim(5);

// 	E2pRed( (unsigned short)LENS_CENTER_FINAL_X,	2,	( unsigned char * )&StCalDat.StLenCen.UsLsxVal ) ; WitTim(5);
// 	E2pRed( (unsigned short)LENS_CENTER_FINAL_Y,	2,	( unsigned char * )&StCalDat.StLenCen.UsLsyVal ) ; WitTim(5);
	
	if( 0xE7 == UcGvcFlg ){
		E2pRed( (unsigned short)0x0937,					2,	( unsigned char * )&StCalDat.StGvcOff.UsGxoVal ) ; WitTim(5);	//GYRO OFFSET Mobile
		E2pRed( (unsigned short)0x0939,					2,	( unsigned char * )&StCalDat.StGvcOff.UsGyoVal ) ; WitTim(5);	//GYRO OFFSET Mobile
	}else{
		E2pRed( (unsigned short)GYRO_AD_OFFSET_X,		2,	( unsigned char * )&StCalDat.StGvcOff.UsGxoVal ) ; WitTim(5);
		E2pRed( (unsigned short)GYRO_AD_OFFSET_Y,		2,	( unsigned char * )&StCalDat.StGvcOff.UsGyoVal ) ; WitTim(5);
	}

	E2pRed( (unsigned short)OSC_CLK_VAL,			1,	( unsigned char * )&StCalDat.UcOscVal ) ; WitTim(5);

	E2pRed( (unsigned short)ADJ_HALL_FLAG,			2,	( unsigned char * )&StCalDat.UsAdjHallF ) ; WitTim(5);
	E2pRed( (unsigned short)ADJ_GYRO_FLAG,			2,	( unsigned char * )&StCalDat.UsAdjGyroF ) ; WitTim(5);
	E2pRed( (unsigned short)ADJ_LENS_FLAG,			2,	( unsigned char * )&StCalDat.UsAdjLensF ) ; WitTim(5);

	E2pRed( (unsigned short)GYRO_GAIN_X,			4,	( unsigned char * )&StCalDat.UlGxgVal ) ; WitTim(5);
	E2pRed( (unsigned short)GYRO_GAIN_Y,			4,	( unsigned char * )&StCalDat.UlGygVal ) ; WitTim(5);

	E2pRed( (unsigned short)FW_VERSION_INFO,		2,	( unsigned char * )&StCalDat.UsVerDat ) ; WitTim(5);
	
	//Hall Limiter Add by Bamtol.Lee at 2015.02.02
	E2pRed( (unsigned short)HALL_LIMIT_X,			4,	( unsigned char * )&StCalDat.UlHlxLmt ) ; WitTim(5);
	E2pRed( (unsigned short)HALL_LIMIT_Y,			4,	( unsigned char * )&StCalDat.UlHlyLmt ) ; WitTim(5);
	//END at 2015.02.02 02

	return;
}
#endif
//********************************************************************************
// Function Name 	: VerInf
// Return Value		: Vesion check result
// Argument Value	: NON
// Explanation		: F/W Version Check
// History			: First edition 						2013.03.21 Y.Kim
//********************************************************************************
int	VerInf( void )
{
	//CDBG("%s : %x, %x \n",__func__, (unsigned char)(StCalDat.UsVerDat >> 8, (unsigned char)(StCalDat.UsVerDat));
	UcVerHig = (unsigned char)(StCalDat.UsVerDat >> 8 ) ;		// System Version
	UcVerLow = (unsigned char)(StCalDat.UsVerDat)  ;			// Filter Version
	
	if( UcVerHig == 0xD0 ){							// 0xA1 1st
		UcVerHig = 0x00 ;							// Matching module version to System
	//Hall Limiter Add by Bamtol.Lee at 2015.02.02
	}else if ( UcVerHig == 0xD1 ){
		UcVerHig = 0x01 ;		
	//END at 2015.02.02 02
	}else{
		return OIS_FW_POLLING_VERSION_FAIL;
	}
	
	/* Matching Version -> Filter */
	if( UcVerLow == 0xD0 ){							// 0xA1 1st
		UcVerLow = 0x00 ;
	} else{
		return OIS_FW_POLLING_VERSION_FAIL;			// Matching module version to Filter
	};
	
	return OIS_FW_POLLING_PASS;
}

//********************************************************************************
// Function Name 	: IniClk
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniClk( void )
{
	RegReadA( CVER ,	&UcCvrCod );		// 0x027E

	/*Clock Enables*/
	RegWriteA( CLKON,		0x1F ) ;			// 0x020B
}

//********************************************************************************
// Function Name 	: IniAf
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Open AF Initial Setting
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniAf( void )
{
	unsigned char	UcStbb0 ;
	
	AfDrvSw( OFF ) ;								/* AF Drvier Block Ena=0 */
	RegWriteA( DRVFCAF	, 0x20 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
	RegWriteA( DRVFC3AF	, 0x00 );					// 0x0083	DGAINDAF	Gain 0
	RegWriteA( DRVFC4AF	, 0x80 );					// 0x0084	DOFSTDAF
	RegWriteA( PWMAAF,    0x00 ) ;					// 0x0090	AF PWM standby
	RegWriteA( AFFC,   0x80 ) ;						// 0x0088	OpenAF/-/-
	
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	RegWriteA( DRVFC2AF,    0x00 ) ;				// 0x0082	AF slope0
	RegWriteA( DRVCH3SEL,   0x00 ) ;				// 0x0085	AF H bridge control
	#endif	//INI_SHORT3
	
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
	AfVcmMod( AFMODE_FAST );					//
	RamWriteA( LTHDH,		LTHDHL_AF ) ;		// 0x0306 - 0x0307 (Register continuos write) 2014.05.19
	AfVcmCod( 0x0000 );							//

	UcStbb0 |= 0x80 ;
	RegWriteA( STBB0, UcStbb0 ) ;			// 0x0250	
	RegWriteA( STBB1	, 0x05 ) ;			// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]

	AfDrvSw( ON ) ;								/* AF Driver Block Ena=1 */
}

//********************************************************************************
// Function Name 	: AfDrvSw
// Return Value		: NON
// Argument Value	: 0:OFF  1:ON
// Explanation		: AF Driver Mode setting function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	AfDrvSw( unsigned char UcDrvSw )
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
}

//********************************************************************************
// Function Name 	: AfVcmMod
// Return Value		: NON
// Argument Value	: AF Code
// Explanation		: VCM AF Code setting function
// History			: First edition 						2013.12.23 YS.Kim
//********************************************************************************
void	AfVcmMod( unsigned char UcModVal)
{
	RegWriteA( TCODEH, UcModVal );	// Mode Setting
}

//********************************************************************************
// Function Name 	: AfVcmCod
// Return Value		: NON
// Argument Value	: AF Code
// Explanation		: VCM AF Code setting function
// History			: First edition 						2013.12.23 YS.Kim
//********************************************************************************
#define	MAX_ACTCODE 1023	//10bit
void	AfVcmCod( unsigned short UsCodVal)
{
	unsigned char	ucFlg = 0x00;
	unsigned char	UcCnt = 0x00;
	
	if(UsCodVal<0) UsCodVal=0; 
	else if(UsCodVal>MAX_ACTCODE) UsCodVal=MAX_ACTCODE;
	
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
}

//********************************************************************************
// Function Name 	: IniIop
// Return Value		: NON
// Argument Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniIop( void )
{
	/*select IOP signal*/
	RegWriteA( IOP1SEL,		0x00 ); 	// 0x0231	IOP1 : DGDATAIN (ATT:0236h[0]=1)
}

//********************************************************************************
// Function Name 	: IniDgy
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
int	IniDgy( void )
{
 	unsigned char	UcGrini ;
 
	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
	RegWriteA( SPIM 	, 0x01 );					// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
													// 		DGSPI4	0: 3-wire SPI, 1: 4-wire SPI
	/*Set to Command Mode*/
	RegWriteA( GRSEL	, 0x01 );					// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA( GRINI	, 0x80 );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

	RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA( GRINI, (unsigned char)( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
#if 0
	RegWriteA( GRADR0,	0x6B ) ;					// 0x0283	Set CLKSEL
	RegWriteA( GSETDT,	0x01 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x6A ) ;					// 0x0283	Set I2C_DIS
	RegWriteA( GSETDT,	0x10 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x1B ) ;					// 0x0283	Set GYRO_CONFIG
	RegWriteA( GSETDT,	( FS_SEL << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/
#else
	//Invenses recommend setting 140307
// 	RegWriteA( GRADR0,	0x6B ) ;					// 0x0283	Set DEVICE_RESET
// 	RegWriteA( GSETDT,	0x80 ) ;					// 0x028A	Set Write Data
// 	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
// 	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/
// 	WitTim(35ms); //need 35ms wait after DEVICE_RESET

	RegWriteA( GRADR0,	0x6B ) ;					// 0x0283	Set CLKSEL
	RegWriteA( GSETDT,	0x01 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x1B ) ;					// 0x0283	Set FS_SEL
	RegWriteA( GSETDT,	( FS_SEL << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x1A ) ;					// 0x0283	Set DLPF_CFG
	RegWriteA( GSETDT,	0x00 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x6A ) ;					// 0x0283	Set I2C_IF_DIS, SIG_COND_RESET
	RegWriteA( GSETDT,	0x11 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	if( AccWit( 0x10 ) == OIS_FW_POLLING_FAIL ){ return OIS_FW_POLLING_FAIL; }		/* Digital Gyro busy wait 				*/
#endif
	RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA( GRINI, (unsigned char)( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

	RegWriteA( RDSEL,	0x7C ) ;					// 0x028B	RDSEL(Data1 and 2 for continuos mode)
	
	GyOutSignal() ;

	return OIS_FW_POLLING_PASS;
}

//********************************************************************************
// Function Name 	: IniMon
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniMon( void )
{
	RegWriteA( PWMMONA, 0x00 ) ;				// 0x0030	0:off
	
	RegWriteA( MONSELA, 0x5C ) ;				// 0x0270	DLYMON1
	RegWriteA( MONSELB, 0x5D ) ;				// 0x0271	DLYMON2
	RegWriteA( MONSELC, 0x00 ) ;				// 0x0272	
	RegWriteA( MONSELD, 0x00 ) ;				// 0x0273	

	// Monitor Circuit
	RegWriteA( WC_PINMON1,	0x00 ) ;			// 0x01C0	Filter Monitor
	RegWriteA( WC_PINMON2,	0x00 ) ;			// 0x01C1	
	RegWriteA( WC_PINMON3,	0x00 ) ;			// 0x01C2	
	RegWriteA( WC_PINMON4,	0x00 ) ;			// 0x01C3	
	/* Delay Monitor */
	RegWriteA( WC_DLYMON11,	0x04 ) ;			// 0x01C5	DlyMonAdd1[10:8]
	RegWriteA( WC_DLYMON10,	0x40 ) ;			// 0x01C4	DlyMonAdd1[ 7:0]
	RegWriteA( WC_DLYMON21,	0x04 ) ;			// 0x01C7	DlyMonAdd2[10:8]
	RegWriteA( WC_DLYMON20,	0xC0 ) ;			// 0x01C6	DlyMonAdd2[ 7:0]
	RegWriteA( WC_DLYMON31,	0x00 ) ;			// 0x01C9	DlyMonAdd3[10:8]
	RegWriteA( WC_DLYMON30,	0x00 ) ;			// 0x01C8	DlyMonAdd3[ 7:0]
	RegWriteA( WC_DLYMON41,	0x00 ) ;			// 0x01CB	DlyMonAdd4[10:8]
	RegWriteA( WC_DLYMON40,	0x00 ) ;			// 0x01CA	DlyMonAdd4[ 7:0]

/* Monitor */
	RegWriteA( PWMMONA, 0x80 ) ;				// 0x0030	1:on 
//	RegWriteA( IOP0SEL,		0x01 ); 			// 0x0230	IOP0 : MONA

}

//********************************************************************************
// Function Name 	: IniSrv
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniSrv( void )
{
	unsigned char	UcStbb0 ;

	UcPwmMod = INIT_PWMMODE ;						// Driver output mode

	RegWriteA( WC_EQON,		0x00 ) ;				// 0x0101		Filter Calcu
	RegWriteA( WC_RAMINITON,0x00 ) ;				// 0x0102		
	ClrGyr( 0x0000 , CLR_ALL_RAM );					// All Clear

	RegWriteA( WH_EQSWX,	0x02 ) ;				// 0x0170		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	RegWriteA( WH_EQSWY,	0x02 ) ;				// 0x0171		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	RamAccFixMod( OFF ) ;							// 32bit Float mode
	
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	/* Monitor Gain */
	RamWrite32A( dm1g, 0x3F800000 ) ;				// 0x109A
	RamWrite32A( dm2g, 0x3F800000 ) ;				// 0x109B
	RamWrite32A( dm3g, 0x3F800000 ) ;				// 0x119A
	RamWrite32A( dm4g, 0x3F800000 ) ;				// 0x119B
	#endif	//INI_SHORT3


	

	//END at 2015.02.02 02
	/* Emergency Stop */
	RegWriteA( WH_EMGSTPON,	0x00 ) ;				// 0x0178		Emergency Stop OFF
	RegWriteA( WH_EMGSTPTMR,0xFF ) ;				// 0x017A		255*(16/23.4375kHz)=174ms
	
	RamWrite32A( sxemglev,   0x3F800000 ) ;			// 0x10EC		Hall X Emergency threshold
	RamWrite32A( syemglev,   0x3F800000 ) ;			// 0x11EC		Hall Y Emergency threshold
	
	/* Hall Servo smoothing */
	RegWriteA( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF
	RegWriteA( WH_SMTSRVSMP,0x06 ) ;				// 0x017D		2.7ms=2^06/23.4375kHz
#ifdef	CATCHMODE
	RegWriteA( WH_SMTTMR,	0x0F ) ;				// 0x017E		10ms=(15+1)*16/23.4375kHz
#else	//CATCHMODE
	RegWriteA( WH_SMTTMR,	0x01 ) ;				// 0x017E		1.3ms=(1+1)*16/23.4375kHz
#endif	//CATCHMODE
	
	RamWrite32A( sxsmtav,   0xBC800000 ) ;			// 0x10ED		1/64 X smoothing ave coefficient
	RamWrite32A( sysmtav,   0xBC800000 ) ;			// 0x11ED		1/64 Y smoothing ave coefficient
	RamWrite32A( sxsmtstp,  0x3AE90466 ) ;			// 0x10EE		0.001778 X smoothing offset
	RamWrite32A( sysmtstp,  0x3AE90466 ) ;			// 0x11EE		0.001778 Y smoothing offset
	
	/* High-dimensional correction  */
	RegWriteA( WH_HOFCON,	0x11 ) ;				// 0x0174		OUT 3x3
	
	/* (0.425X^3+0.55X)*(0.425X^3+0.55X) 10.2ohm*/		//20141014 Komori
	/* Front */
	RamWrite32A( sxiexp3,   0x3ED9999A ) ;			// 0x10BA		
	RamWrite32A( sxiexp2,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A( sxiexp1,   0x3F0CCCCD ) ;			// 0x10BC		
	RamWrite32A( sxiexp0,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A( sxiexp,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A( syiexp3,   0x3ED9999A ) ;			// 0x11BA		
	RamWrite32A( syiexp2,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A( syiexp1,   0x3F0CCCCD ) ;			// 0x11BC		
	RamWrite32A( syiexp0,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A( syiexp,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A( sxoexp3,   0x3ED9999A ) ;			// 0x10FA		
	RamWrite32A( sxoexp2,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A( sxoexp1,   0x3F0CCCCD ) ;			// 0x10FC		
	RamWrite32A( sxoexp0,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A( sxoexp,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A( syoexp3,   0x3ED9999A ) ;			// 0x11FA		
	RamWrite32A( syoexp2,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A( syoexp1,   0x3F0CCCCD ) ;			// 0x11FC		
	RamWrite32A( syoexp0,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A( syoexp,    0x3F800000 ) ;			// 0x11FE
	
#ifdef	CATCHMODE
	RegWriteA( WC_DPI1ADD0,		0x3B ) ;				// 0x01B0		Data Pass
	RegWriteA( WC_DPI1ADD1,		0x00 ) ;				// 0x01B1		0x143B(GXK2Z2) --> 0x1405(GXI2Z1)
	RegWriteA( WC_DPI2ADD0,		0xBB ) ;				// 0x01B2		
	RegWriteA( WC_DPI2ADD1,		0x00 ) ;				// 0x01B3		0x14BB(GYK2Z2) --> 0x1485(GYI2Z1)
	RegWriteA( WC_DPI3ADD0,		0x38 ) ;				// 0x01B4		
	RegWriteA( WC_DPI3ADD1,		0x00 ) ;				// 0x01B5		0x1438(GXK1Z2) --> 0x143A(GXK2Z1)
	RegWriteA( WC_DPI4ADD0,		0xB8 ) ;				// 0x01B6		
	RegWriteA( WC_DPI4ADD1,		0x00 ) ;				// 0x01B7		0x14B8(GYK1Z2) --> 0x14BA(GYK2Z1)
	
	RegWriteA( WC_DPO1ADD0,		0x05 ) ;				// 0x01B8		Data Pass
	RegWriteA( WC_DPO1ADD1,		0x00 ) ;				// 0x01B9		
	RegWriteA( WC_DPO2ADD0,		0x85 ) ;				// 0x01BA		
	RegWriteA( WC_DPO2ADD1,		0x00 ) ;				// 0x01BB		
	RegWriteA( WC_DPO3ADD0,		0x3A ) ;				// 0x01BC		
	RegWriteA( WC_DPO3ADD1,		0x00 ) ;				// 0x01BD		
	RegWriteA( WC_DPO4ADD0,		0xBA ) ;				// 0x01BE		
	RegWriteA( WC_DPO4ADD1,		0x00 ) ;				// 0x01BF		
	
	RegWriteA( WC_DPON,			0x0F ) ;				// 0x0105		Data pass ON
#endif	//CATCHMODE
	
	/* Ram Access */
	RamAccFixMod( OFF ) ;							// 32bit float mode

	// PWM Signal Generate
	DrvSw( OFF ) ;									/* 0x0070	Driver Block Ena=0 */
	RegWriteA( DRVFC2	, 0x90 );					// 0x0002	Slope 3, Dead Time = 30ns
	RegWriteA( DRVSELX	, 0xFF );					// 0x0003	PWM X drv max current  DRVSELX[7:0]
	RegWriteA( DRVSELY	, 0xFF );					// 0x0004	PWM Y drv max current  DRVSELY[7:0]

#ifdef	PWM_BREAK
	RegWriteA( PWMFC,   0x3D ) ;					// 0x0011	VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
#else
	RegWriteA( PWMFC,   0x21 ) ;					// 0x0011	VREF, PWMCLK/256, MODE1, 12Bit Accuracy
#endif

	RegWriteA( PWMA,    0x00 ) ;					// 0x0010	PWM X/Y standby
	RegWriteA( PWMDLYX,  0x04 ) ;					// 0x0012	X Phase Delay Setting
	RegWriteA( PWMDLYY,  0x04 ) ;					// 0x0013	Y Phase Delay Setting

	RegWriteA( PWMPERIODX,	0x00 ) ;			// 0x0018		PWM Carrier Freq
	RegWriteA( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
	RegWriteA( PWMPERIODY,	0x00 ) ;			// 0x001A		PWM Carrier Freq
	RegWriteA( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
	
	/* Linear PWM circuit setting */
	RegWriteA( CVA		, 0xC0 );					// 0x0020	Linear PWM mode enable
	RegWriteA( CVFC2 	, 0x80 );					// 0x0022

	RegReadA( STBB0 	, &UcStbb0 );				// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x80 ;
	RegWriteA( STBB0, UcStbb0 ) ;					// 0x0250	OIS standby
	
}



//********************************************************************************
// Function Name 	: IniGyr
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	IniGyr( void )
{
	
#ifdef	CATCHMODE
	/* CPU control */
	RegWriteA( WC_CPUOPEON , 0x11 );	// 0x0103	 	CPU control
	RegWriteA( WC_CPUOPE1ADD , 0x05 );	// 0x018A	 	0x1405(GXI2Z1), 0x1485(GYI2Z1)
	RegWriteA( WC_CPUOPE2ADD , 0x3A );	// 0x018B	 	0x143A(GXK2Z1), 0x14BA(GYK2Z1)
	RegWriteA( WG_EQSW	, 0x43 );		// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
#else	//CATCHMODE
	/*Gyro Filter Setting*/
	RegWriteA( WG_EQSW	, 0x03 );						// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
#endif	//CATCHMODE
	
	/*Gyro Filter Down Sampling*/
	
	RegWriteA( WG_SHTON	, 0x10 );						// 0x0107		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
														//				CmShtOpe[1:0] 00: Shtter OFF, 01: Shutter ON, 1x:External Shutter
	RegWriteA( WG_SHTMOD , 0x06 );						// 0x0116	 	Shutter Hold mode

	// Limiter
	RamWrite32A( gxlmt1H, GYRLMT1H ) ;					// 0x1028
	RamWrite32A( gylmt1H, GYRLMT1H ) ;					// 0x1128

	RamWrite32A( Sttx12aM, 	GYRA12_MID );				// 0x104F
	//RamWrite32A( Sttx12aH, 	GYRA12_HGH );				// 0x105F
	RamWrite32A( Sttx12bM, 	GYRB12_MID );				// 0x106F
	RamWrite32A( Sttx12bH, 	GYRB12_HGH );				// 0x107F
	RamWrite32A( Sttx34aM, 	GYRA34_MID );				// 0x108F
//	RamWrite32A( Sttx34aH, 	GYRA34_HGH );				// 0x109F
	RamWrite32A( Sttx34bM, 	GYRB34_MID );				// 0x10AF
	RamWrite32A( Sttx34bH, 	GYRB34_HGH );				// 0x10BF
	RamWrite32A( Stty12aM, 	GYRA12_MID );				// 0x114F
	//RamWrite32A( Stty12aH, 	GYRA12_HGH );				// 0x115F
	RamWrite32A( Stty12bM, 	GYRB12_MID );				// 0x116F
	RamWrite32A( Stty12bH, 	GYRB12_HGH );				// 0x117F
	RamWrite32A( Stty34aM, 	GYRA34_MID );				// 0x118F
//	RamWrite32A( Stty34aH, 	GYRA34_HGH );				// 0x119F
	RamWrite32A( Stty34bM, 	GYRB34_MID );				// 0x11AF
	RamWrite32A( Stty34bH, 	GYRB34_HGH );				// 0x11BF

#ifdef	CATCHMODE
  #ifdef	CORRECT_1DEG
	SelectPtRange( ON ) ;
  #else
	SelectPtRange( OFF ) ;
  #endif
	/* Pan/Tilt parameter */
	RegWriteA( WG_PANADDA, 		0x12 );		// 0x0130	GXH2Z2/GYH2Z2 Select
	RegWriteA( WG_PANADDB, 		0x3B );		// 0x0131	GXK2Z2/GYK2Z2 Select
#else	//CATCHMODE
	SelectPtRange( OFF ) ;
	
	/* Pan/Tilt parameter */
	RegWriteA( WG_PANADDA, 		0x12 );					// 0x0130	GXH1Z2/GYH1Z2 Select
	RegWriteA( WG_PANADDB, 		0x09 );					// 0x0131	GXIZ/GYIZ Select
#endif	//CATCHMODE
	
	 //Threshold
	RamWrite32A( SttxHis, 	0x00000000 );				// 0x1226
	RamWrite32A( SttxaL, 	0x00000000 );				// 0x109D
	RamWrite32A( SttxbL, 	0x00000000 );				// 0x109E
	RamWrite32A( SttyaL, 	0x00000000 );				// 0x119D
	RamWrite32A( SttybL, 	0x00000000 );				// 0x119E
	
	// Pan level
	RegWriteA( WG_PANLEVABS, 		0x00 );				// 0x0133
	
	// Average parameter are set IniAdj

#ifdef	CATCHMODE
	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA( WG_PANSTT21JUG0, 	0x00 );		// 0x0140
	RegWriteA( WG_PANSTT21JUG1, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA( WG_PANSTT31JUG0, 	0x00 );		// 0x0142
	RegWriteA( WG_PANSTT31JUG1, 	0x00 );		// 0x0143
	// State 4 -> 1
	RegWriteA( WG_PANSTT41JUG0, 	0x77 );		// 0x0144
	RegWriteA( WG_PANSTT41JUG1, 	0x00 );		// 0x0145
	// State 1 -> 2
	RegWriteA( WG_PANSTT12JUG0, 	0x00 );		// 0x0146
	RegWriteA( WG_PANSTT12JUG1, 	0x00 );		// 0x0147
	// State 1 -> 3
	RegWriteA( WG_PANSTT13JUG0, 	0x00 );		// 0x0148
	RegWriteA( WG_PANSTT13JUG1, 	0x07 );		// 0x0149
	// State 2 -> 3
	RegWriteA( WG_PANSTT23JUG0, 	0x00 );		// 0x014A
	RegWriteA( WG_PANSTT23JUG1, 	0x00 );		// 0x014B
	// State 4 -> 3
	RegWriteA( WG_PANSTT43JUG0, 	0x00 );		// 0x014C
	RegWriteA( WG_PANSTT43JUG1, 	0x00 );		// 0x014D
	// State 3 -> 4
	RegWriteA( WG_PANSTT34JUG0, 	0x77 );		// 0x014E
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
	RegWriteA( WG_PANTRSON0, 		0x13 );		// 0x0132	USE iSTP/I12
	
	// State Setting
	IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA( WG_PANSTTSETILHLD,	0x00 );		// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA( WG_PANSTT2TMR0,	0x01 );		// 0x013C
	RegWriteA( WG_PANSTT2TMR1,	0x00 );		// 0x013D	
	RegWriteA( WG_PANSTT4TMR0,	0x00 );		// 0x013E
	RegWriteA( WG_PANSTT4TMR1,	0x09 );		// 0x013F	
	
	RegWriteA( WG_PANSTTXXXTH,	0x00 );		// 0x015A

  #if 1
	AutoGainContIni() ;
	/* exe function */
	AutoGainControlSw( ON ) ;							/* Auto Gain Control Mode ON  */
  #else
	StartUpGainContIni();
  #endif

#else	//CATCHMODE

	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA( WG_PANSTT21JUG0, 	0x00 );				// 0x0140
	RegWriteA( WG_PANSTT21JUG1, 	0x00 );				// 0x0141
	// State 3 -> 1
	RegWriteA( WG_PANSTT31JUG0, 	0x01 );				// 0x0142
	RegWriteA( WG_PANSTT31JUG1, 	0x00 );				// 0x0143
	// State 4 -> 1
	RegWriteA( WG_PANSTT41JUG0, 	0x00 );				// 0x0144
	RegWriteA( WG_PANSTT41JUG1, 	0x00 );				// 0x0145
	// State 1 -> 2
	RegWriteA( WG_PANSTT12JUG0, 	0x00 );				// 0x0146
	RegWriteA( WG_PANSTT12JUG1, 	0x00 );				// 0x0147
	// State 1 -> 3
	RegWriteA( WG_PANSTT13JUG0, 	0x00 );				// 0x0148
	RegWriteA( WG_PANSTT13JUG1, 	0x07 );				// 0x0149
	// State 2 -> 3
	RegWriteA( WG_PANSTT23JUG0, 	0x00 );				// 0x014A
	RegWriteA( WG_PANSTT23JUG1, 	0x00 );				// 0x014B
	// State 4 -> 3
	RegWriteA( WG_PANSTT43JUG0, 	0x00 );				// 0x014C
	RegWriteA( WG_PANSTT43JUG1, 	0x00 );				// 0x014D
	// State 3 -> 4
	RegWriteA( WG_PANSTT34JUG0, 	0x00 );				// 0x014E
	RegWriteA( WG_PANSTT34JUG1, 	0x00 );				// 0x014F
	// State 2 -> 4
	RegWriteA( WG_PANSTT24JUG0, 	0x00 );				// 0x0150
	RegWriteA( WG_PANSTT24JUG1, 	0x00 );				// 0x0151
	// State 4 -> 2
	RegWriteA( WG_PANSTT42JUG0, 	0x00 );				// 0x0152
	RegWriteA( WG_PANSTT42JUG1, 	0x00 );				// 0x0153
	
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	// State Timer
	RegWriteA( WG_PANSTT1LEVTMR, 	0x00 );				// 0x015B
	RegWriteA( WG_PANSTT2LEVTMR, 	0x00 );				// 0x015C
	RegWriteA( WG_PANSTT3LEVTMR, 	0x00 );				// 0x015D
	RegWriteA( WG_PANSTT4LEVTMR, 	0x00 );				// 0x015E
	#endif	//INI_SHORT3
	
	// Control filter
	//RegWriteA( WG_PANTRSON0, 		0x11 );				// 0x0132	USE I12/iSTP/Gain-Filter
	RegWriteA( WG_PANTRSON0, 		0x91 );				// 0x0132	USE I12/iSTP/Gain-Filter, USE Linear
	
	RegWriteA( WG_PANSTTSETGYRO, 	0x00 );				// 0x0154
	RegWriteA( WG_PANSTTSETGAIN, 	0x10 );				// 0x0155
	RegWriteA( WG_PANSTTSETISTP, 	0x10 );				// 0x0156
	RegWriteA( WG_PANSTTSETIFTR,	0x10 );				// 0x0157
	RegWriteA( WG_PANSTTSETLFTR,	0x00 );				// 0x0158
	
	#ifdef	INI_SHORT2
	#else	//INI_SHORT2
	// State Setting
	IniPtMovMod( OFF ) ;								// Pan/Tilt setting (Still)
	#endif	//INI_SHORT2
	
	// Hold
	RegWriteA( WG_PANSTTSETILHLD,	0x00 );				// 0x015F
	
	// State2,4 Step Time Setting
	RegWriteA( WG_PANSTT2TMR0,	0xEA );					// 0x013C	9.983787013ms
	RegWriteA( WG_PANSTT2TMR1,	0x00 );					// 0x013D
	RegWriteA( WG_PANSTT4TMR0,	0x92 );					// 0x013E	49.91893506ms
	RegWriteA( WG_PANSTT4TMR1,	0x04 );					// 0x013F
	
	RegWriteA( WG_PANSTTXXXTH,	0x0F );					// 0x015A

  #ifdef	GAIN_CONT
	AutoGainContIni() ;
	/* exe function */
  	AutoGainControlSw( ON ) ;							/* Auto Gain Control Mode OFF */
  #endif	//GAIN_CONT
	
#endif	//CATCHMODE
}

//********************************************************************************
// Function Name 	: IniPtAve
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Pan/Tilt Average parameter setting function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtAve( void )
{
#ifdef	CATCHMODE
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	RegWriteA( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA( WG_PANSTT2DWNSMP0, 0x00 );		// 0x0136
	RegWriteA( WG_PANSTT2DWNSMP1, 0x00 );		// 0x0137
	RegWriteA( WG_PANSTT3DWNSMP0, 0x00 );		// 0x0138
	RegWriteA( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B
	#endif	//INI_SHORT3
	
	RamWrite32A( st1mean, 0x3f800000 );		// 0x1235
	RamWrite32A( st2mean, 0x3f800000 );		// 0x1236
	RamWrite32A( st3mean, 0x3f800000 );		// 0x1237
	RamWrite32A( st4mean, 0x3f800000 );		// 0x1238
#else	//CATCHMODE
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	RegWriteA( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA( WG_PANSTT2DWNSMP0, 0x00 );		// 0x0136
	RegWriteA( WG_PANSTT2DWNSMP1, 0x00 );		// 0x0137
	RegWriteA( WG_PANSTT3DWNSMP0, 0x00 );		// 0x0138
	RegWriteA( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B
	#endif	//INI_SHORT3
	
	RamWrite32A( st1mean, 0x3f800000 );			// 0x1235
	RamWrite32A( st2mean, 0x3f800000 );			// 0x1236
	RamWrite32A( st3mean, 0x3f800000 );			// 0x1237
	RamWrite32A( st4mean, 0x3f800000 );			// 0x1238
#endif	//CATCHMODE
}

//********************************************************************************
// Function Name 	: IniPtMovMod
// Return Value		: NON
// Argument Value	: OFF:Still  ON:Movie
// Explanation		: Pan/Tilt parameter setting by mode function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtMovMod( unsigned char UcPtMod )
{
#ifdef	CATCHMODE
	switch ( UcPtMod ) {
		case OFF :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x00 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x90 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158

			break ;
		case ON :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x00 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x90 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158
			break ;
	}
#else	//CATCHMODE
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
#endif	//CATCHMODE
}

//********************************************************************************
// Function Name 	: SelectPtRange
// Retun Value		: NON
// Argment Value	: OFF:Narrow  ON:Wide
// Explanation		: Pan/Tilt parameter Range function
// History			: First edition 						2014.04.08 Y.Shigeoka
//********************************************************************************
void	SelectPtRange( unsigned char UcSelRange )
{
	switch ( UcSelRange ) {
		case OFF :
			RamWrite32A( gxlmt3HS0, GYRLMT3_S1 ) ;		// 0x1029
			RamWrite32A( gylmt3HS0, GYRLMT3_S1 ) ;		// 0x1129
			
			RamWrite32A( gxlmt3HS1, GYRLMT3_S2 ) ;		// 0x102A
			RamWrite32A( gylmt3HS1, GYRLMT3_S2 ) ;		// 0x112A

			RamWrite32A( gylmt4HS0, GYRLMT4_S1 ) ;		//0x112B	Y axis Limiter4 High Threshold0
			RamWrite32A( gxlmt4HS0, GYRLMT4_S1 ) ;		//0x102B	X axis Limiter4 High Threshold0
			
			RamWrite32A( gxlmt4HS1, GYRLMT4_S2 ) ;		//0x102C	X axis Limiter4 High Threshold1
			RamWrite32A( gylmt4HS1, GYRLMT4_S2 ) ;		//0x112C	Y axis Limiter4 High Threshold1
		
			RamWrite32A( Sttx12aH, 	GYRA12_HGH );		// 0x105F
			RamWrite32A( Stty12aH, 	GYRA12_HGH );		// 0x115F

			RamWrite32A( Sttx34aH, 	GYRA34_HGH );		// 0x109F
			RamWrite32A( Stty34aH, 	GYRA34_HGH );		// 0x119F

			break ;
		
#ifdef	CATCHMODE
		case ON :
			RamWrite32A( gxlmt3HS0, GYRLMT3_S1_W ) ;		// 0x1029
			RamWrite32A( gylmt3HS0, GYRLMT3_S1_W ) ;		// 0x1129
			
			RamWrite32A( gxlmt3HS1, GYRLMT3_S2_W ) ;		// 0x102A
			RamWrite32A( gylmt3HS1, GYRLMT3_S2_W ) ;		// 0x112A

			RamWrite32A( gylmt4HS0, GYRLMT4_S1_W ) ;		//0x112B	Y axis Limiter4 High Threshold0
			RamWrite32A( gxlmt4HS0, GYRLMT4_S1_W ) ;		//0x102B	X axis Limiter4 High Threshold0
			
			RamWrite32A( gxlmt4HS1, GYRLMT4_S2_W ) ;		//0x102C	X axis Limiter4 High Threshold1
			RamWrite32A( gylmt4HS1, GYRLMT4_S2_W ) ;		//0x112C	Y axis Limiter4 High Threshold1
		
			RamWrite32A( Sttx12aH, 	GYRA12_HGH_W );			// 0x105F
			RamWrite32A( Stty12aH, 	GYRA12_HGH_W );			// 0x115F

			RamWrite32A( Sttx34aH, 	GYRA34_HGH_W );			// 0x109F
			RamWrite32A( Stty34aH, 	GYRA34_HGH_W );			// 0x119F

			break ;
#endif // CATCHMODE
	}
}

//********************************************************************************
// Function Name 	: SelectIstpMod
// Retun Value		: NON
// Argment Value	: OFF:Narrow  ON:Wide
// Explanation		: Pan/Tilt parameter Range function
// History			: First edition 						2014.04.08 Y.Shigeoka
//********************************************************************************
void	SelectIstpMod( unsigned char UcSelRange )
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
#endif	//CATCHMODE
	}
}

#ifdef	CATCHMODE
//********************************************************************************
// Function Name 	: SetDCoffsetContValue
// Retun Value		: NON
// Argment Value	: Off: Normal On : Speed Up
// Explanation		: DC offset value controller
// History			: First edition 						2015.03.03 K.abe
//********************************************************************************
void	SetDCoffsetContValue( unsigned char UcSelRange )
{
	switch ( UcSelRange ) {
		case OFF :
			RamWrite32A( gxib, 0x3A03126F) ;		// 0x1005	0.0005
			RamWrite32A( gyib, 0x3A03126F) ;		// 0x1105
			break;
		case ON :
			RamWrite32A( gxib, 0x3B031240 ) ;		// 0x1005	0.002
			RamWrite32A( gyib, 0x3B031240 ) ;		// 0x1105
			break;
	}
}
#endif // CATCHMODE  

#if 0// we no use
//********************************************************************************
// Function Name 	: IniFil
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
int	IniFil( void )
{
#ifdef	INI_SHORT1
	unsigned char	UcAryId ;
	unsigned short	UsDatId, UsDatNum ;
	
	unsigned char	*pFilRegDat;
	unsigned char	*pFilReg;
	unsigned char	*pFilRamDat;
	unsigned char	*pFilRam;
	
	if ( UcVerLow == 0x00 ){
		pFilRegDat	= (unsigned char *)CsFilRegDat_D0D0;
		pFilReg		= (unsigned char *)CsFilReg_D0D0;
		pFilRamDat	= (unsigned char *)CsFilRamDat_D0D0;
		pFilRam		= (unsigned char *)CsFilRam_D0D0;
	}
	else{
		return OIS_FW_POLLING_FAIL;
	}
	
	RegWriteA( WC_RAMACCXY, 0x01 ) ;			// 0x018D	Filter copy on
	
	// Filter Registor Parameter Setting
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( pFilReg[ UcAryId ] != 0xFF )
	{
		UsDatNum	= pFilReg[ UcAryId ];
		CntWrt( ( unsigned char * )&pFilRegDat[ UsDatId ], UsDatNum ) ;
		UcAryId++ ;
		UsDatId	+= UsDatNum ;
	}
	// Filter X-axis Ram Parameter Setting	
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( pFilRam[ UcAryId ] != 0xFF )
	{
		UsDatNum	= pFilRam[ UcAryId ];
		CntWrt( ( unsigned char * )&pFilRamDat[ UsDatId ], UsDatNum ) ;
		UsDatId	+= UsDatNum ;
		UcAryId++ ;
	}
	
	RegWriteA( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Simultaneously Setting Off
	
	return OIS_FW_POLLING_PASS ;
#else	//INI_SHORT1
	unsigned short		UsAryId ;
	struct STFILREG		*pFilReg;
	struct STFILRAM		*pFilRam;
	
	if ( UcVerLow == 0x00 ){
		pFilReg = (struct STFILREG *)CsFilReg_D0D0;
		pFilRam = (struct STFILRAM *)CsFilRam_D0D0;
	}
	else{
		return OIS_FW_POLLING_FAIL;
	}
	
	RegWriteA( WC_RAMACCXY, 0x01 ) ;			// 0x018D	Simultaneously Setting On
	
	// Filter Register Parameter Setting
	UsAryId	= 0 ;
	while( pFilReg[ UsAryId ].UsRegAdd != 0xFFFF )
	{
		RegWriteA( pFilReg[ UsAryId ].UsRegAdd, pFilReg[ UsAryId ].UcRegDat ) ;
		UsAryId++ ;
		if( UsAryId > FILREGTAB ){ return OIS_FW_POLLING_FAIL ; }
	}
	
	// Filter Ram Parameter Setting
	UsAryId	= 0 ;
	while( pFilRam[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWrite32A( pFilRam[ UsAryId ].UsRamAdd, pFilRam[ UsAryId ].UlRamDat ) ;
		UsAryId++ ;
		if( UsAryId > FILRAMTAB ){ return OIS_FW_POLLING_FAIL ; }
	}
	
	RegWriteA( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Simultaneously Setting Off
	
	return OIS_FW_POLLING_PASS ;
#endif	//INI_SHORT1
}

//********************************************************************************
// Function Name 	: IniAdj
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniAdj( void )
{
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	RegWriteA( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Filter copy off
	#endif	//INI_SHORT3
	
#ifdef	CATCHMODE
 #ifdef	CORRECT_1DEG
	SelectIstpMod( ON ) ;
 #else
	SelectIstpMod( OFF ) ;
 #endif
 	SetDCoffsetContValue( ON ) ;
#else
	SelectIstpMod( OFF ) ;
#endif // CATCHMODE
	
	IniPtAve( ) ;								// Average setting
	
	/* OIS */
	RegWriteA( CMSDAC0, BIAS_CUR_OIS ) ;		// 0x0251	Hall DAC Current
	RegWriteA( OPGSEL0, AMP_GAIN_X ) ;			// 0x0253	Hall amp Gain X
	RegWriteA( OPGSEL1, AMP_GAIN_Y ) ;			// 0x0254	Hall amp Gain Y

	/* AF */
	RegWriteA( CMSDAC1, BIAS_CUR_AF ) ;			// 0x0252	Hall Dac current
	RegWriteA( OPGSEL2, AMP_GAIN_AF ) ;			// 0x0255	Hall amp Gain AF
	
	/* OSC Clock value */
	if( ((unsigned char)StCalDat.UcOscVal == 0x00) || ((unsigned char)StCalDat.UcOscVal == 0xFF) ){
		RegWriteA( OSCSET, OSC_INI ) ;				// 0x0257	OSC ini
	}else{
		RegWriteA( OSCSET, StCalDat.UcOscVal ) ;		// 0x0257
	}
	
	/* adjusted value */
	/* Gyro X axis Offset */
	if( ( StCalDat.StGvcOff.UsGxoVal == 0x0000 ) || ( StCalDat.StGvcOff.UsGxoVal == 0xFFFF )){
		RegWriteA( IZAH,	DGYRO_OFST_XH ) ;		// 0x02A0		Set Offset High byte
		RegWriteA( IZAL,	DGYRO_OFST_XL ) ;		// 0x02A1		Set Offset Low byte
	}else{
		RegWriteA( IZAH, (unsigned char)(StCalDat.StGvcOff.UsGxoVal >> 8) ) ;	// 0x02A0		Set Offset High byte
		RegWriteA( IZAL, (unsigned char)(StCalDat.StGvcOff.UsGxoVal) ) ;		// 0x02A1		Set Offset Low byte
	}
	/* Gyro Y axis Offset */
	if( ( StCalDat.StGvcOff.UsGyoVal == 0x0000 ) || ( StCalDat.StGvcOff.UsGyoVal == 0xFFFF )){
		RegWriteA( IZBH,	DGYRO_OFST_YH ) ;		// 0x02A2		Set Offset High byte
		RegWriteA( IZBL,	DGYRO_OFST_YL ) ;		// 0x02A3		Set Offset Low byte
	}else{
		RegWriteA( IZBH, (unsigned char)(StCalDat.StGvcOff.UsGyoVal >> 8) ) ;	// 0x02A2		Set Offset High byte
		RegWriteA( IZBL, (unsigned char)(StCalDat.StGvcOff.UsGyoVal) ) ;		// 0x02A3		Set Offset Low byte
	}
	
	/* Ram Access */
	RamAccFixMod( ON ) ;						// 16bit Fix mode
	
	/* OIS adjusted parameter */
	/* Hall X axis Bias,Offset,Lens center */
	if( (StCalDat.UsAdjHallF == 0x0000 ) || (StCalDat.UsAdjHallF == 0xFFFF ) || (StCalDat.UsAdjHallF & ( EXE_HXADJ - EXE_END )) ){
		RamWriteA( DAXHLO,		DAHLXO_INI ) ;				// 0x1479
		RamWriteA( DAXHLB,		DAHLXB_INI ) ;				// 0x147A
	}else{
		RamWriteA( DAXHLO, StCalDat.StHalAdj.UsHlxOff ) ;	// 0x1479
		RamWriteA( DAXHLB, StCalDat.StHalAdj.UsHlxGan ) ;	// 0x147A
	}

	/* Hall Y axis Bias,Offset,Lens center */
	if( (StCalDat.UsAdjHallF == 0x0000 ) || (StCalDat.UsAdjHallF == 0xFFFF ) || (StCalDat.UsAdjHallF & ( EXE_HYADJ - EXE_END )) ){
		RamWriteA( DAYHLO,		DAHLYO_INI ) ;				// 0x14F9
		RamWriteA( DAYHLB,		DAHLYB_INI ) ;				// 0x14FA
	}else{
		RamWriteA( DAYHLO, StCalDat.StHalAdj.UsHlyOff ) ;	// 0x14F9
		RamWriteA( DAYHLB, StCalDat.StHalAdj.UsHlyGan ) ;	// 0x14FA
	}

	/* Hall X axis Loop Gain */
	if( (StCalDat.UsAdjHallF == 0x0000 ) || (StCalDat.UsAdjHallF == 0xFFFF ) || (StCalDat.UsAdjHallF & ( EXE_LXADJ - EXE_END )) ){
		RamWriteA( sxg,			SXGAIN_INI ) ;			// 0x10D3
	}else{
		RamWriteA( sxg, StCalDat.StLopGan.UsLxgVal ) ;	// 0x10D3
	}

	/* Hall Y axis Loop Gain */
	if( (StCalDat.UsAdjHallF == 0x0000 ) || (StCalDat.UsAdjHallF == 0xFFFF ) || (StCalDat.UsAdjHallF & ( EXE_LYADJ - EXE_END )) ){
		RamWriteA( syg,			SYGAIN_INI ) ;			// 0x11D3
	}else{
		RamWriteA( syg, StCalDat.StLopGan.UsLygVal ) ;	// 0x11D3
	}
	
	/* Ram Access */
	RamAccFixMod( OFF ) ;						// 32bit Float mode

	/* Gyro X axis Gain */
	if( ( StCalDat.UlGxgVal == 0x00000000 ) || ( StCalDat.UlGxgVal == 0xFFFFFFFF ) ){
		RamWrite32A( gxzoom, GXGAIN_INI ) ;				// 0x1020 Gyro X axis Gain adjusted value
	}else{
		RamWrite32A( gxzoom, StCalDat.UlGxgVal ) ;		// 0x1020 Gyro X axis Gain adjusted value
	}

	/* Gyro Y axis Gain */
	if( ( StCalDat.UlGygVal == 0x00000000 ) || ( StCalDat.UlGygVal == 0xFFFFFFFF ) ){
		RamWrite32A( gyzoom, GYGAIN_INI ) ;				// 0x1120 Gyro Y axis Gain adjusted value
	}else{
		RamWrite32A( gyzoom, StCalDat.UlGygVal ) ;		// 0x1120 Gyro Y axis Gain adjusted value
	}

	RamAccFixMod( ON ) ;						// 16bit Fix mode
	if( (StCalDat.UsAdjHallF == 0x0000 ) || (StCalDat.UsAdjLensF == 0xFFFF ) || (StCalDat.UsAdjLensF & ( EXE_CXADJ - EXE_END )) ){
		RamWriteA( OFF0Z,	HXOFF0Z_INI ) ;				// 0x1450
	}else{
		RamWriteA( OFF0Z, StCalDat.StLenCen.UsLsxVal ) ;	// 0x1450
	}

	if( (StCalDat.UsAdjHallF == 0x0000 ) || (StCalDat.UsAdjLensF == 0xFFFF ) || (StCalDat.UsAdjLensF & ( EXE_CYADJ - EXE_END )) ){
		RamWriteA( OFF1Z,	HYOFF1Z_INI ) ;				// 0x14D0
	}else{
		RamWriteA( OFF1Z, StCalDat.StLenCen.UsLsyVal ) ;	// 0x14D0
	}
	RamAccFixMod( OFF ) ;						// 32bit Float mode
	
	/* Hall output limiter */
	//Hall Limiter Add by Bamtol.Lee at 2015.02.02
	if(UcVerHig == 0x01)
	{
		if( ( StCalDat.UlHlxLmt == 0x00000000 ) || ( StCalDat.UlHlxLmt == 0xFFFFFFFF ) ){
			RamWrite32A( sxlmta1, 0x3F800000 ) ;				// 0x1020 Gyro X axis Gain adjusted value
		}else{
			RamWrite32A( sxlmta1, StCalDat.UlHlxLmt ) ;		// 0x1020 Gyro X axis Gain adjusted value
		}

		if( ( StCalDat.UlHlyLmt == 0x00000000 ) || ( StCalDat.UlHlyLmt == 0xFFFFFFFF ) ){
			RamWrite32A( sylmta1, 0x3F800000 ) ;				// 0x1020 Gyro X axis Gain adjusted value
		}else{
			RamWrite32A( sylmta1, StCalDat.UlHlyLmt ) ;		// 0x1020 Gyro X axis Gain adjusted value
		}

	}else{

		RamWrite32A( sxlmta1,   0x3F800000 ) ;			// 0x10E6		Hall X output Limit
		RamWrite32A( sylmta1,   0x3F800000 ) ;			// 0x11E6		Hall Y output Limit
	}
	//END at 2015.02.02 02
	
	
	RamWrite32A( sxq, SXQ_INI ) ;				// 0x10E5	X axis output direction initial value
	RamWrite32A( syq, SYQ_INI ) ;				// 0x11E5	Y axis output direction initial value
	
#ifdef	CATCHMODE
	RamWrite32A( gx45g, G_45G_INI ) ;			// 0x1000
	RamWrite32A( gy45g, G_45G_INI ) ;			// 0x1100
	ClrGyr( 0x00FF , CLR_FRAM1 );		// Gyro Delay RAM Clear
#endif	//CATCHMODE
	
	RegWriteA( PWMA 	, 0xC0 );				// 0x0010		PWM enable
	
	RegWriteA( STBB0 	, 0xDF );				// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA( WC_EQSW	, 0x02 ) ;				// 0x01E0
	RegWriteA( WC_MESLOOP1	, 0x02 ) ;			// 0x0193
	RegWriteA( WC_MESLOOP0	, 0x00 ) ;			// 0x0192
	RegWriteA( WC_AMJLOOP1	, 0x02 ) ;			// 0x01A3
	RegWriteA( WC_AMJLOOP0	, 0x00 ) ;			// 0x01A2
	
#ifdef	CATCHMODE
	SetPanTiltMode( OFF ) ;					/* Pan/Tilt OFF */

	SetH1cMod( ACTMODE ) ;					/* Lvl Change Active mode */
	
	DrvSw( ON ) ;							/* 0x0001		Driver Mode setting */
	
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

	#ifdef	INI_SHORT2
	#else	//INI_SHORT2
	SetPanTiltMode( OFF ) ;					/* Pan/Tilt OFF */
	#endif	//INI_SHORT2
	
	#ifdef	INI_SHORT2
	#else	//INI_SHORT2
	SetGcf( 0 ) ;							/* DI initial value */
	#endif	//INI_SHORT2
	
 	#ifdef	H1COEF_CHANGER
	SetH1cMod( ACTMODE ) ;					/* Lvl Change Active mode */
//	SetH1cMod( MOVMODE ) ;					/* Lvl Change Active mode */
	#endif	//H1COEF_CHANGER
	
	DrvSw( ON ) ;							/* 0x0001		Driver Mode setting */
	
	RegWriteA( WC_EQON, 0x01 ) ;				// 0x0101	Filter ON
	
	RegWriteA( WG_NPANST12BTMR, 0x01 ) ;		// 0x0167
	SetPanTiltMode( ON ) ;						// Pan/Tilt
	RegWriteA( WG_PANSTT6, 0x44 ) ;				// 0x010A
	RegWriteA( WG_PANSTT6, 0x11 ) ;				// 0x010A
#endif	//CATCHMODE
}
#endif
//********************************************************************************
// Function Name 	: DrvSw
// Return Value		: NON
// Argument Value	: 0:OFF  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
void	DrvSw( unsigned char UcDrvSw )
{
	if( UcDrvSw == ON )
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
		} else {
#ifdef	PWM_BREAK
			RegWriteA( DRVFC	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA( DRVFC	, 0xC0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE1
#endif
		}
	}
	else
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0x30 );				// 0x0001	Drvier Block Ena=0
		} else {
#ifdef	PWM_BREAK
			RegWriteA( DRVFC	, 0x00 );				// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA( DRVFC	, 0x00 );				// 0x0001	Drvier Block Ena=0
#endif
		}
	}
}

//********************************************************************************
// Function Name 	: RamAccFixMod
// Return Value		: NON
// Argument Value	: 0:OFF  1:ON
// Explanation		: Ram Access Fix Mode setting function
// History			: First edition 						2013.05.21 Y.Shigeoka
//********************************************************************************
void	RamAccFixMod( unsigned char UcAccMod )
{
	switch ( UcAccMod ) {
	case OFF :
		RegWriteA( WC_RAMACCMOD,	0x00 ) ;		// 0x018C		GRAM Access Float32bit
		break ;
	case ON :
		RegWriteA( WC_RAMACCMOD,	0x31 ) ;		// 0x018C		GRAM Access Fix32bit
		break ;
	}
}





//********************************************************************************
// Function Name 	: RtnCen
// Return Value		: Command Status
// Argument Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
unsigned char	RtnCen( unsigned char	UcCmdPar )
{
	unsigned char	UcCmdSts ;
	
	UcCmdSts	= EXE_END ;
	
	GyrCon( OFF ) ;											// Gyro OFF
	
	if( !UcCmdPar ) {										// X,Y Centering
		
		StbOnn() ;											// Slope Mode
		
	} else if( UcCmdPar == 0x01 ) {							// X Centering Only
		
		SrvCon( X_DIR, ON ) ;								// X only Servo ON
		SrvCon( Y_DIR, OFF ) ;
	} else if( UcCmdPar == 0x02 ) {							// Y Centering Only
		
		SrvCon( X_DIR, OFF ) ;								// Y only Servo ON
		SrvCon( Y_DIR, ON ) ;
	}
	
	return( UcCmdSts ) ;
}

//********************************************************************************
// Function Name 	: SrvCon
// Return Value		: NON
// Argument Value	: X or Y Select, Servo ON/OFF
// Explanation		: Servo ON,OFF Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	SrvCon( unsigned char	UcDirSel, unsigned char	UcSwcCon )
{
	unsigned char	UcAccMod ;
	
	RegReadA( WC_RAMACCMOD, &UcAccMod ) ;

	if( UcSwcCon ) {
		if( !UcDirSel ) {								// X Direction
			RegWriteA( WH_EQSWX , 0x03 ) ;				// 0x0170
			if( UcAccMod == 0x31 ){ 
				RamWriteA( sxggf, 0x0000 ) ;			// 0x10B5
			}else{
				RamWrite32A( sxggf, 0x00000000 ) ;		// 0x10B5
			}
		} else {										// Y Direction
			RegWriteA( WH_EQSWY , 0x03 ) ;				// 0x0171
			if( UcAccMod == 0x31 ){ 
				RamWriteA( syggf, 0x0000 ) ;			// 0x11B5
			}else{
				RamWrite32A( syggf, 0x00000000 ) ;		// 0x11B5
			}
		}
	} else {
		if( !UcDirSel ) {								// X Direction
			RegWriteA( WH_EQSWX , 0x02 ) ;				// 0x0170
			if( UcAccMod == 0x31 ){ 
				RamWriteA( SXLMT, 0x0000 ) ;			// 0x1477
			}else{
				RamWrite32A( SXLMT, 0x00000000 ) ;		// 0x1477
			}
		} else {										// Y Direction
			RegWriteA( WH_EQSWY , 0x02 ) ;				// 0x0171
			if( UcAccMod == 0x31 ){ 
				RamWriteA( SYLMT, 0x0000 ) ;			// 0x14F7
			}else{
				RamWrite32A( SYLMT, 0x00000000 ) ;		// 0x14F7
			}
		}
	}
}

//********************************************************************************
// Function Name 	: StbOnn
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Stabilizer For Servo On Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void StbOnn( void )
{
	unsigned char	UcRegValx,UcRegValy;					// Register value 
	unsigned char	UcRegIni ;
	unsigned char	UcCnt ;
	
	RegReadA( WH_EQSWX , &UcRegValx ) ;			// 0x0170
	RegReadA( WH_EQSWY , &UcRegValy ) ;			// 0x0171
	
	if( (( UcRegValx & 0x01 ) != 0x01 ) && (( UcRegValy & 0x01 ) != 0x01 ))
	{

		RegWriteA( WH_SMTSRVON,	0x01 ) ;				// 0x017C		Smooth Servo ON
		
		SrvCon( X_DIR, ON ) ;
		SrvCon( Y_DIR, ON ) ;
		
		UcCnt = 0;
 		UcRegIni = 0x11;
 		while( (UcRegIni & 0x77) != 0x66 )
 		{
			RegReadA( RH_SMTSRVSTT,	&UcRegIni ) ;		// 0x01F8		Smooth Servo phase read
//			UcRegIni = 0x11;
			WitTim( 1 );
			UcCnt = UcCnt + 1;
			if(UcCnt > 60){
				break;
			}
 		}

//		WitTim( 100 ) ;
	
		RegWriteA( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF
		
	}
	else
	{
		SrvCon( X_DIR, ON ) ;
		SrvCon( Y_DIR, ON ) ;

	}
}

//********************************************************************************
// Function Name 	: GyrCon
// Return Value		: NON
// Argument Value	: Gyro Filter ON or OFF
// Explanation		: Gyro Filter Control Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	GyrCon( unsigned char	UcGyrCon )
{
	unsigned char	UcAccMod ;

	RegReadA( WC_RAMACCMOD, &UcAccMod ) ;

	// Return HPF Setting
	RegWriteA( WG_SHTON, 0x00 ) ;									// 0x0107
	
	if( UcGyrCon == ON ) {												// Gyro ON
		
		ClrGyr( 0x000E , CLR_FRAM1 );			// Gyro Delay RAM Clear
		
		if( UcAccMod == 0x31 ){
			RamWriteA( sxggf, 0x7FFF ) ;		// 0x10B5
			RamWriteA( syggf, 0x7FFF ) ;		// 0x11B5
			
#ifdef	CATCHMODE
#else	//CATCHMODE
			RamAccFixMod( OFF ) ;
			RamWrite32A( gxib_1, 0xBF800000 ) ;		// Down
			RamWrite32A( gyib_1, 0xBF800000 ) ;		// Down
//			RamWrite32A( gxib_1, 0xB261CF49 ) ;		// Down
//			RamWrite32A( gyib_1, 0xB261CF49 ) ;		// Down
			IniPtMovMod( OFF );						//2014.05.19
			RamAccFixMod( ON ) ;
#endif	//CATCHMODE
		}else{
			RamWrite32A( sxggf, 0x3F800000 ) ;	// 0x10B5
			RamWrite32A( syggf, 0x3F800000 ) ;	// 0x11B5
			
#ifdef	CATCHMODE
#else	//CATCHMODE
			RamWrite32A( gxib_1, 0xBF800000 ) ;		// Down
			RamWrite32A( gyib_1, 0xBF800000 ) ;		// Down
//			RamWrite32A( gxib_1, 0xB261CF49 ) ;		// Down
//			RamWrite32A( gyib_1, 0xB261CF49 ) ;		// Down
			IniPtMovMod( OFF );						//2014.05.19
#endif	//CATCHMODE
		}
#ifdef	CATCHMODE
#else	//CATCHMODE
		RegWriteA( WG_PANSTT6, 0x00 ) ;				// 0x010A
#endif	//CATCHMODE
		//SetPanTiltMode(OFF) ;						// Pantilt OFF
	} else if( UcGyrCon == SPC ) {					// Gyro ON for LINE
		if( UcAccMod == 0x31 ){
			RamWriteA( sxggf, 0x7FFF ) ;		// 0x10B5
			RamWriteA( syggf, 0x7FFF ) ;		// 0x11B5
		}else{
			RamWrite32A( sxggf, 0x3F800000 ) ;	// 0x10B5
			RamWrite32A( syggf, 0x3F800000 ) ;	// 0x11B5
		}

	} else {															// Gyro OFF
		if( UcAccMod == 0x31 ){
			RamWriteA( sxggf, 0x0000 ) ;		// 0x10B5
			RamWriteA( syggf, 0x0000 ) ;		// 0x11B5
		}else{
			RamWrite32A( sxggf, 0x00000000 ) ;	// 0x10B5
			RamWrite32A( syggf, 0x00000000 ) ;	// 0x11B5
		}

	}
}

//********************************************************************************
// Function Name 	: SetPanTiltMode
// Return Value		: NON
// Argument Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	SetPanTiltMode( unsigned char UcPnTmod )
{
	SetDCoffsetContValue( OFF );
	
	switch ( UcPnTmod ) {
	case OFF :
		RegWriteA( WG_PANON, 0x00 ) ;			// 0x0109	X,Y Pan/Tilt Function OFF
		break ;
	case ON :
		RegWriteA( WG_PANON, 0x01 ) ;			// 0x0109	X,Y Pan/Tilt Function ON
		break ;
	}
	
}

//********************************************************************************
// Function Name 	: OisEna
// Return Value		: NON
// Argument Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	OisEna( void )
{
	// Servo ON
	//SrvCon( X_DIR, ON ) ;
	//SrvCon( Y_DIR, ON ) ;
	
	GyrCon( ON ) ;
}


//********************************************************************************
// Function Name 	: OisEnaLin
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function for Line adjustment
// History			: First edition 						2013.09.05 Y.Shigeoka
//********************************************************************************
void	OisEnaLin( void )
{
	// Servo ON
	//SrvCon( X_DIR, ON ) ;
	//SrvCon( Y_DIR, ON ) ;

	GyrCon( SPC ) ;
}


//********************************************************************************
// Function Name 	: OisOff
// Retun Value		: 
// Argment Value	: 
// Explanation		: 
// History			: 
//********************************************************************************
void	OisOff( void ) //Ois Off
{
 	GyrCon( OFF ) ;

  	return;
}

//********************************************************************************
// Function Name 	: ClrGyr
// Retun Value		: NON
// Argment Value	: UsClrFil - Select filter to clear.  If 0x0000, clears entire filter
//					  UcClrMod - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
int	ClrGyr( unsigned short UsClrFil , unsigned char UcClrMod )
{
	unsigned char	UcRamClr;
	
	/*Select Filter to clear*/
	RegWriteA( WC_RAMDLYMOD1,	(unsigned char)(UsClrFil >> 8)) ;		// 0x018F		FRAM Initialize Hbyte
	RegWriteA( WC_RAMDLYMOD0,	(unsigned char)UsClrFil ) ;				// 0x018E		FRAM Initialize Lbyte
	
	/*Enable Clear*/
	RegWriteA( WC_RAMINITON	, UcClrMod ) ;	// 0x0102	[ - | - | - | - ][ - | - | Ram Clr | Coef Clr ]
	
	/*Check RAM Clear complete*/
	do{
		RegReadA( WC_RAMINITON, &UcRamClr );
		UcRamClr &= UcClrMod;
	}while( UcRamClr != 0x00 );

	return 0;
}

void	MemClr( unsigned char	*NcTgtPtr, unsigned short	UsClrSiz )
{
	unsigned short	UsClrIdx ;
	
	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}

	return;
}

//********************************************************************************
// Function Name 	: AccWit
// Return Value		: NON
// Argument Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************

int	AccWit( unsigned char UcTrgDat )
{
	unsigned char	UcFlgVal ;
	unsigned char	UcCntPla ;
	UcFlgVal	= 1 ;
	UcCntPla	= 0 ;
	
	do{
		RegReadA( GRACC, &UcFlgVal ) ;
		UcFlgVal	&= UcTrgDat ;
		UcCntPla++ ;
	} while( UcFlgVal && ( UcCntPla < ACCWIT_POLLING_LIMIT_A ) ) ;
	if( UcCntPla == ACCWIT_POLLING_LIMIT_A ) { return OIS_FW_POLLING_FAIL; }
	
	return OIS_FW_POLLING_PASS ;
}


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
void	AutoGainContIni( void )
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
	
	RegWriteA( WG_LEVADD, (unsigned char)MONADR );		// 0x0120	Input signal
	RegWriteA( WG_LEVTMR, 		TIMEBSE );				// 0x0123	Base Time
	RegWriteA( WG_LEVTMRLOW, 	TIMELOW );				// 0x0121	X Low Time
	RegWriteA( WG_LEVTMRHGH, 	TIMEHGH );				// 0x0122	X Hgh Time
	RegWriteA( WG_ADJGANADD, (unsigned char)GANADR );		// 0x0128	control address
	#ifdef	INI_SHORT3
	#else	//INI_SHORT3
	RegWriteA( WG_ADJGANGO, 		0x00 );				// 0x0108	manual off
	#endif	//INI_SHORT3
}

//********************************************************************************
// Function Name 	: AutoGainControlSw
// Retun Value		: NON
// Argment Value	: 0 :OFF  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
void	AutoGainControlSw( unsigned char UcModeSw )
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
	
}

#ifdef	CATCHMODE
//********************************************************************************
// Function Name 	: StartUpGainContIni
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Start UP Gain Control initial function
// History			: First edition 						2014.09.16 Y.Shigeoka
//********************************************************************************
  #define	ST_TRI_LEVEL		0x3A031280		/* 0.0005 */
  #define	ST_TIMELOW			0x00			/* */
  #define	ST_TIMEHGH			0x00			/* */
  #define	ST_TIMEBSE			0x00			/* */
  #define	ST_MONADR			GXXFZ
  #define	ST_GANADR			pxmbb
  #define	ST_XMINGAIN		0x3A031240		/* 0.0005 Target gain 0x10A5*/
  #define	ST_XMAXGAIN		0x3C031280		/* 0.0080 Initial gain*/
  #define	ST_YMINGAIN		0x3A031240
  #define	ST_YMAXGAIN		0x3C031280
  #define	ST_XSTEPUP			0x3F800000		/* 1	 */
  #define	ST_XSTEPDN			0xB3E50F84		/* -0.0000001 	 */
  #define	ST_YSTEPUP			0x3F800000		/* 1	 */
  #define	ST_YSTEPDN			0xB3E50F84		/* -0.05 	 */
	
void	StartUpGainContIni( void )
{
	RamWrite32A( gxlevlow, ST_TRI_LEVEL );					// 0x10AE	Low Th
	RamWrite32A( gylevlow, ST_TRI_LEVEL );					// 0x11AE	Low Th
	RamWrite32A( gxadjmin, ST_XMINGAIN );					// 0x1094	Low gain
	RamWrite32A( gxadjmax, ST_XMAXGAIN );					// 0x1095	Hgh gain
	RamWrite32A( gxadjdn, ST_XSTEPDN );					// 0x1096	-step
	RamWrite32A( gxadjup, ST_XSTEPUP );					// 0x1097	+step
	RamWrite32A( gyadjmin, ST_YMINGAIN );					// 0x1194	Low gain
	RamWrite32A( gyadjmax, ST_YMAXGAIN );					// 0x1195	Hgh gain
	RamWrite32A( gyadjdn, ST_YSTEPDN );					// 0x1196	-step
	RamWrite32A( gyadjup, ST_YSTEPUP );					// 0x1197	+step
	
	RegWriteA( WG_LEVADD, (unsigned char)ST_MONADR );		// 0x0120	Input signal
	RegWriteA( WG_LEVTMR, 		ST_TIMEBSE );				// 0x0123	Base Time
	RegWriteA( WG_LEVTMRLOW, 	ST_TIMELOW );				// 0x0121	X Low Time
	RegWriteA( WG_LEVTMRHGH, 	ST_TIMEHGH );				// 0x0122	X Hgh Time
	RegWriteA( WG_ADJGANADD, (unsigned char)ST_GANADR );		// 0x0128	control address
	RegWriteA( WG_ADJGANGO, 		0x00 );					// 0x0108	manual off
}

//********************************************************************************
// Function Name 	: InitGainControl
// Retun Value		: NON
// Argment Value	: OFF,  ON
// Explanation		: Gain Control function
// History			: First edition 						2014.09.16 Y.Shigeoka
//********************************************************************************
unsigned char	InitGainControl( unsigned char uc_mode )
{
	unsigned char	uc_rtval;
	
	uc_rtval = 0x00 ;
	
	switch( uc_mode) {
	case	0x00 :
		RamWrite32A( gx2x4xb, 0x00000000 ) ;		// 0x1021 
		RamWrite32A( gy2x4xb, 0x00000000 ) ;		// 0x1121 
		
		RegWriteA( WG_ADJGANGO, 		0x22 );					// 0x0108	manual on to go to max
		uc_rtval = 0x22 ;
		while( uc_rtval ){
			
			RegReadA( WG_ADJGANGO, 		&uc_rtval );			// 0x0108	status read
		} ;
		
	case	0x01 :
		RamWrite32A( gx2x4xb, 0x00000000 ) ;		// 0x1021 
		RamWrite32A( gy2x4xb, 0x00000000 ) ;		// 0x1121 
	
		RegWriteA( WG_ADJGANGO, 		0x11 );					// 0x0108	manual on to go to min(initial)
		break;
		
	case	0x02 :
		RegReadA( WG_ADJGANGO, 		&uc_rtval );			// 0x0108	status read
		break;
		
	case	0x03 :
		
		ClrGyr( 0x000E , CLR_FRAM1 );		// Gyro Delay RAM Clear
		RamWrite32A( gx2x4xb, 0x3F800000 ) ;		// 0x1021 
		RamWrite32A( gy2x4xb, 0x3F800000 ) ;		// 0x1121 
		
		AutoGainContIni() ;
		AutoGainControlSw( ON ) ;								/* Auto Gain Control Mode ON  */
		break;
	}
	
	return( uc_rtval ) ;
}
#endif 	//CATCHMOD

#endif	//GAIN_CONT


//********************************************************************************
// Function Name 	: S2cPro
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: S2 Command Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	S2cPro( unsigned char uc_mode )
{
	if( uc_mode == 1 )
	{
#ifdef	H1COEF_CHANGER
		SetH1cMod( S2MODE ) ;							/* cancel Lvl change */
#endif	//H1COEF_CHANGER

#ifndef	CATCHMODE
		// HPFÅ®Through Setting
		RegWriteA( WG_SHTON, 0x11 ) ;							// 0x0107
#endif	//CATCHMODE

		RamWrite32A( gxh1c, DIFIL_S2 );							// 0x1012
		RamWrite32A( gyh1c, DIFIL_S2 );							// 0x1112
	}
	else
	{
		RamWrite32A( gxh1c, UlH1Coefval );							// 0x1012
		RamWrite32A( gyh1c, UlH1Coefval );							// 0x1112

#ifndef CATCHMODE
		// HPFÅ®Through Setting
		RegWriteA( WG_SHTON, 0x00 ) ;							// 0x0107
#endif	//CATCHMODE

#ifdef	H1COEF_CHANGER
		SetH1cMod( UcH1LvlMod ) ;							/* Re-setting */
#endif	//H1COEF_CHANGER
	}
	
}

//********************************************************************************
// Function Name 	: SetGcf
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set DI filter coefficient Function
// History			: First edition 						2013.03.22 Y.Shigeoka
//********************************************************************************
void	SetGcf( unsigned char	UcSetNum )
{
	
	/* Zoom Step */
	if(UcSetNum > (COEFTBL - 1))
		UcSetNum = (COEFTBL -1) ;			/* Set Maximum to COEFTBL-1 */

	UlH1Coefval	= ClDiCof[ UcSetNum ] ;
		
	// Zoom Value Setting
	RamWrite32A( gxh1c, UlH1Coefval ) ;		/* 0x1012 */
	RamWrite32A( gyh1c, UlH1Coefval ) ;		/* 0x1112 */

#ifdef H1COEF_CHANGER
		SetH1cMod( UcSetNum ) ;							/* Re-setting */
#endif

}

#ifdef	H1COEF_CHANGER
//********************************************************************************
// Function Name 	: SetH1cMod
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set H1C coefficient Level chang Function
// History			: First edition 						2013.04.18 Y.Shigeoka
//********************************************************************************
void	SetH1cMod( unsigned char	UcSetNum )
{
	
	switch( UcSetNum ){
#ifdef	CATCHMODE

	case ( ACTMODE ):				// initial 
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
		
		/* enable setting */
			
		UcH1LvlMod = UcSetNum ;
		
		// Limit value Value Setting
//TRACE("C CNTROL  ---------------------------->\n" );
 #ifdef	CORRECT_1DEG
		RamWrite32A( gxlmt6L, MINLMT_W ) ;		/* 0x102D L-Limit */
		RamWrite32A( gxlmt6H, MAXLMT_W ) ;		/* 0x102E H-Limit */

		RamWrite32A( gylmt6L, MINLMT_W ) ;		/* 0x112D L-Limit */
		RamWrite32A( gylmt6H, MAXLMT_W ) ;		/* 0x112E H-Limit */

		RamWrite32A( gxmg, 		CHGCOEF_W ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 		CHGCOEF_W ) ;		/* 0x11AA Change coefficient gain */
 #else	//CORRECT_1DEG
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */

		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */

		RamWrite32A( gxmg, 		CHGCOEF ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 		CHGCOEF ) ;		/* 0x11AA Change coefficient gain */
 #endif	//CORRECT_1DEG
		RamWrite32A( gxhc_tmp, 	DIFIL_S2 ) ;	/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, 	DIFIL_S2 ) ;	/* 0x110E Base Coef */
//TRACE("C CNTROL  <----------------------------\n" );
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( S2MODE ):				// cancel lvl change mode 
		RegWriteA( WG_HCHR, 0x10 ) ;			// 0x011B	GmHChrOn[1]=0 Sw OFF
		break ;
		
	case( MOVMODE ):			// Movie mode 
		IniPtMovMod( ON ) ;						// Pan/Tilt setting (Movie)
		SelectPtRange( OFF ) ;					// Range narrow
		SelectIstpMod( OFF ) ;					// Range narrow
		
//TRACE("C CNTROL  ---------------------------->\n" );
		RamWrite32A( gxlmt6L, MINLMT_MOV ) ;	/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT_MOV ) ;	/* 0x112D L-Limit */

		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, CHGCOEF_MOV ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF_MOV ) ;		/* 0x11AA Change coefficient gain */
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
//TRACE("C CNTROL  <----------------------------\n" );
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( MOVMODE_W ):			// Movie mode (wide)
		IniPtMovMod( ON ) ;							// Pan/Tilt setting (Movie)
		SelectPtRange( ON ) ;					// Range wide
		SelectIstpMod( ON ) ;					// Range wide
		
//TRACE("C CNTROL  ---------------------------->\n" );
		RamWrite32A( gxlmt6L, MINLMT_MOV_W ) ;	/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT_MOV_W ) ;	/* 0x112D L-Limit */

		RamWrite32A( gxlmt6H, MAXLMT_W ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT_W ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, CHGCOEF_MOV_W ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF_MOV_W ) ;		/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
//TRACE("C CNTROL  <----------------------------\n" );
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( STILLMODE ):				// Still mode 
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
		SelectPtRange( OFF ) ;					// Range narrow
		SelectIstpMod( OFF ) ;					// Range narrow
		
		UcH1LvlMod = UcSetNum ;
			
//TRACE("C CNTROL  ---------------------------->\n" );
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, 	CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 	CHGCOEF ) ;			/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
//TRACE("C CNTROL  <----------------------------\n" );
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	case( STILLMODE_W ):			// Still mode (Wide)
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
		SelectPtRange( ON ) ;					// Range wide
		SelectIstpMod( ON ) ;					// Range wide
		
		UcH1LvlMod = UcSetNum ;
			
//TRACE("C CNTROL  ---------------------------->\n" );
		RamWrite32A( gxlmt6L, MINLMT_W ) ;		/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT_W ) ;		/* 0x112D L-Limit */
		
		RamWrite32A( gxlmt6H, MAXLMT_W ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT_W ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, 	CHGCOEF_W ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 	CHGCOEF_W ) ;			/* 0x11AA Change coefficient gain */
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
//TRACE("C CNTROL  <----------------------------\n" );
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	default :
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
		SelectPtRange( OFF ) ;					// Range narrow
		SelectIstpMod( OFF ) ;					// Range narrow
		
		UcH1LvlMod = UcSetNum ;
			
//TRACE("C CNTROL  ---------------------------->\n" );
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, 	CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, 	CHGCOEF ) ;			/* 0x11AA Change coefficient gain */
//TRACE("C CNTROL  <----------------------------\n" );
			
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;

#else	//CATCHMODE

	case ( ACTMODE ):				// initial 
	#ifdef	INI_SHORT2
	#else	//INI_SHORT2
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
		
		/* enable setting */
		/* Zoom Step */
		UlH1Coefval	= ClDiCof[ 0 ] ;
			
		UcH1LvlMod = 0 ;
	#endif	//INI_SHORT2

		// Limit value Value Setting
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */

		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */

		RamWrite32A( gxmg, CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF ) ;			/* 0x11AA Change coefficient gain */

	#ifdef	INI_SHORT2
		RamWrite32A( gxhc_tmp, DIFIL_S2 ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, DIFIL_S2 ) ;		/* 0x110E Base Coef */
		
		RamWrite32A( gxh1c, DIFIL_S2 ) ;		/* 0x1012 Base Coef */
		RamWrite32A( gyh1c, DIFIL_S2 ) ;		/* 0x1112 Base Coef */
	#else	//INI_SHORT2
		RamWrite32A( gxhc_tmp, UlH1Coefval ) ;	/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, UlH1Coefval ) ;	/* 0x110E Base Coef */
	#endif	//INI_SHORT2
		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;

	case( S2MODE ):				// cancel lvl change mode 
		RegWriteA( WG_HCHR, 0x10 ) ;			// 0x011B	GmHChrOn[1]=0 Sw OFF
		break ;

	case( MOVMODE ):			// Movie mode 
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)	2014.05.19
		
		RamWrite32A( gxlmt6L, MINLMT_MOV ) ;	/* 0x102D L-Limit */
		RamWrite32A( gxlmt6H, MAXLMT_MOV ) ;	/* 0x102E H-Limit */
		
		RamWrite32A( gylmt6L, MINLMT_MOV ) ;	/* 0x112D L-Limit */
		RamWrite32A( gylmt6H, MAXLMT_MOV ) ;	/* 0x112E H-Limit */

		RamWrite32A( gxmg, CHGCOEF_MOV ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF_MOV ) ;		/* 0x11AA Change coefficient gain */

	#ifdef	INI_SHORT2
	#else	//INI_SHORT2
		RamWrite32A( gxhc_tmp, UlH1Coefval ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, UlH1Coefval ) ;		/* 0x110E Base Coef */
	#endif	//INI_SHORT2

		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;
		
	default :
		IniPtMovMod( OFF ) ;							// Pan/Tilt setting (Still)
		
		UcH1LvlMod = UcSetNum ;
			
		// Limit value Value Setting
		RamWrite32A( gxlmt6L, MINLMT ) ;		/* 0x102D L-Limit */
		RamWrite32A( gxlmt6H, MAXLMT ) ;		/* 0x102E H-Limit */
		
		RamWrite32A( gylmt6L, MINLMT ) ;		/* 0x112D L-Limit */
		RamWrite32A( gylmt6H, MAXLMT ) ;		/* 0x112E H-Limit */
		
		RamWrite32A( gxmg, CHGCOEF ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A( gymg, CHGCOEF ) ;			/* 0x11AA Change coefficient gain */

	#ifdef	INI_SHORT2
	#else	//INI_SHORT2
		RamWrite32A( gxhc_tmp, UlH1Coefval ) ;		/* 0x100E Base Coef */
		RamWrite32A( gyhc_tmp, UlH1Coefval ) ;		/* 0x110E Base Coef */
	#endif	//INI_SHORT2

		RegWriteA( WG_HCHR, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON
		break ;

#endif	//CATCHMODE

	}
}

#endif	//H1COEF_CHANGER

#ifdef	STANDBY_MODE
//********************************************************************************
// Function Name 	: SetStandby
// Retun Value		: NON
// Argment Value	: 0:Standby ON 1:Standby OFF 2:Standby2 ON 3:Standby2 OFF 
//					: 4:Standby3 ON 5:Standby3 OFF
// Explanation		: Set Standby
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	SetStandby( unsigned char UcContMode )
{
	unsigned char	UcStbb0 , UcClkon ;
	
	switch(UcContMode)
	{
	case STB1_ON:
		RegWriteA( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
		RegWriteA( STBB0 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA( STBB1 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( PWMA 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( CVA,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw( OFF ) ;						/* Driver OFF */
		AfDrvSw( OFF ) ;					/* AF Driver OFF */
		RegWriteA( PWMMONA, 0x00 ) ;		// 0x0030	Monitor Standby
//		RegWriteA( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		break ;
	case STB1_OFF:
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
//		RegWriteA( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA( PWMMONA, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw( ON ) ;						/* Driver Mode setting */
		AfDrvSw( ON ) ;						/* AF Driver Mode setting */
		RegWriteA( CVA		, 0xC0 );		// 0x0020	Linear PWM mode enable
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( PWMA		, 0xC0 );		// 0x0010	PWM enable
		RegWriteA( STBB1	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( STBB0	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		break ;
	case STB2_ON:
		RegWriteA( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
		RegWriteA( STBB0 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA( STBB1 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( PWMA 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( CVA,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw( OFF ) ;						/* Drvier Block Ena=0 */
		AfDrvSw( OFF ) ;					/* AF Drvier Block Ena=0 */
		RegWriteA( PWMMONA, 0x00 ) ;		// 0x0030	Monitor Standby
//		RegWriteA( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		RegWriteA( CLKON, 0x00 ) ;			/* 0x020B	Servo & PWM Clock OFF + D-Gyro I/F OFF	*/
		break ;
	case STB2_OFF:
		RegWriteA( CLKON,	0x1F ) ;		// 0x020B	[ - | - | CmOpafClkOn | CmAfpwmClkOn | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
//		RegWriteA( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA( PWMMONA, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw( ON ) ;						/* Driver Mode setting */
		AfDrvSw( ON ) ;						/* AF Driver Mode setting */
		RegWriteA( CVA, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( PWMA	, 	0xC0 );			// 0x0010	PWM enable
		RegWriteA( STBB1	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( STBB0	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		break ;
	case STB3_ON:
		RegWriteA( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
		RegWriteA( STBB0 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA( STBB1 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( PWMA 	, 0x00 );			// 0x0010		PWM Standby
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( CVA,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw( OFF ) ;						/* Drvier Block Ena=0 */
		AfDrvSw( OFF ) ;					/* AF Drvier Block Ena=0 */
		RegWriteA( PWMMONA, 0x00 ) ;		// 0x0030	Monitor Standby
//		RegWriteA( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		RegWriteA( CLKON, 0x00 ) ;			/* 0x020B	Servo & PWM Clock OFF + D-Gyro I/F OFF	*/
		RegWriteA( I2CSEL, 0x01 ) ;			/* 0x0248	I2C Noise Cancel circuit OFF	*/
		RegWriteA( OSCSTOP, 0x02 ) ;		// 0x0256	Source Clock Input OFF
		break ;
	case STB3_OFF:
		RegWriteA( OSCSTOP, 0x00 ) ;		// 0x0256	Source Clock Input ON
		RegWriteA( I2CSEL, 0x00 ) ;			/* 0x0248	I2C Noise Cancel circuit ON	*/
		RegWriteA( CLKON,	0x1F ) ;		// 0x020B	[ - | - | - | - | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
//		RegWriteA( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA( PWMMONA, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw( ON ) ;						/* Driver Mode setting */
		AfDrvSw( ON ) ;						/* AF Driver Mode setting */
		RegWriteA( CVA, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA( PWMAAF,	0x00 );			// 0x0090		AF PWM Standby
		RegWriteA( PWMA	, 	0xC0 );			// 0x0010	PWM enable
		RegWriteA( STBB1	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( STBB0	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		break ;
		
	case STB4_ON:
		RegWriteA( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
		RegWriteA( STBB0 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA( STBB1 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( PWMA 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( CVA,  	0x00 ) ;		/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw( OFF ) ;						/* Drvier Block Ena=0 */
		AfDrvSw( OFF ) ;					/* AF Drvier Block Ena=0 */
		RegWriteA( PWMMONA, 0x00 ) ;		// 0x0030	Monitor Standby
//		RegWriteA( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		GyOutSignalCont( ) ;				/* Gyro Continuos mode */
		RegWriteA( CLKON, 0x04 ) ;			/* 0x020B	Servo & PWM Clock OFF + D-Gyro I/F ON	*/
		break ;
	case STB4_OFF:
		RegWriteA( CLKON,	0x1F ) ;		// 0x020B	[ - | - | - | - | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep( OFF ) ;				/* Gyro OIS mode */
//		RegWriteA( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA( PWMMONA, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw( ON ) ;						/* Driver Mode setting */
		AfDrvSw( ON ) ;						/* AF Driver Mode setting */
		RegWriteA( CVA, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA( PWMAAF, 	0x00 );			// 0x0090		AF PWM Standby
		RegWriteA( PWMA	, 	0xC0 );			// 0x0010	PWM enable
		RegWriteA( STBB1	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( STBB0	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		break ;
		
		/************** special mode ************/
	case STB2_OISON:
		RegReadA( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0 &= 0x80 ;
		RegWriteA( STBB0 	, UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA( PWMA 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA( CVA,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw( OFF ) ;						/* Drvier Block Ena=0 */
		RegWriteA( PWMMONA, 0x00 ) ;		// 0x0030	Monitor Standby
//		RegWriteA( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		RegReadA( CLKON, &UcClkon ) ;		/* 0x020B	PWM Clock OFF + D-Gyro I/F OFF	SRVCLK can't OFF */
		UcClkon &= 0x1A ;
		RegWriteA( CLKON, UcClkon ) ;		/* 0x020B	PWM Clock OFF + D-Gyro I/F OFF	SRVCLK can't OFF */
		break ;
	case STB2_OISOFF:
		RegReadA( CLKON, &UcClkon ) ;		/* 0x020B	PWM Clock OFF + D-Gyro I/F ON  */
		UcClkon |= 0x05 ;
		RegWriteA( CLKON,	UcClkon ) ;		// 0x020B	[ - | - | CmOpafClkOn | CmAfpwmClkOn | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
//		RegWriteA( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA( PWMMONA, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw( ON ) ;						/* Driver Mode setting */
		RegWriteA( CVA, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA( PWMA	, 	0xC0 );			// 0x0010	PWM enable
		RegReadA( STBB0	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0 |= 0x5F ;
		RegWriteA( STBB0	, UcStbb0 );	// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		break ;
		
	case STB2_AFON:
		RegWriteA( DRVFCAF	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
		RegReadA( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0 &= 0x7F ;
		RegWriteA( STBB0 	, UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA( STBB1 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		AfDrvSw( OFF ) ;					/* AF Drvier Block Ena=0 */
		RegWriteA( PWMMONA, 0x00 ) ;		// 0x0030	Monitor Standby
		RegReadA( CLKON, &UcClkon ) ;		/* 0x020B	OPAF Clock OFF + AFPWM OFF	SRVCLK can't OFF	*/
		UcClkon &= 0x07 ;
		RegWriteA( CLKON, UcClkon ) ;		/* 0x020B	OPAF Clock OFF + AFPWM OFF	SRVCLK can't OFF	*/
		break ;
	case STB2_AFOFF:
		RegReadA( CLKON, &UcClkon ) ;		/* 0x020B	OPAF Clock ON + AFPWM ON  */
		UcClkon |= 0x18 ;
		RegWriteA( CLKON,	UcClkon ) ;		// 0x020B	[ - | - | CmOpafClkOn | CmAfpwmClkOn | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		AfDrvSw( ON ) ;						/* AF Driver Mode setting */
		RegWriteA( PWMAAF 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA( STBB1	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegReadA( STBB0	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0 |= 0x80 ;
		RegWriteA( STBB0	, UcStbb0 );	// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		break ;
		/************** special mode ************/
	}
}

//********************************************************************************
// Function Name 	: SelectGySleep
// Retun Value		: NON
// Argment Value	: mode	
// Explanation		: Select Gyro mode Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SelectGySleep( unsigned char UcSelMode )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcRamIni ;
	unsigned char	UcGrini ;

	if(UcSelMode == ON)
	{
		RegWriteA( WC_EQON, 0x00 ) ;		// 0x0101	Equalizer OFF
		RegWriteA( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/

		RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		RegWriteA( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni |= 0x40 ;					/* Set Sleep bit */
//  #ifdef GYROSTBY 2014.05.19
		UcRamIni &= ~0x01 ;					/* Clear PLL bit(internal oscillator */
//  #endif 2014.05.19
		
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	UcRamIni ) ;	/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/

  #ifdef GYROSTBY
		RegWriteA( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	0x07 ) ;		/* 0x028A	Set Write Data(STBY ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
	}
	else
	{
  #ifdef GYROSTBY
		RegWriteA( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	0x00 ) ;		/* 0x028A	Set Write Data(STBY OFF)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni &= ~0x40 ;					/* Clear Sleep bit */
//  #ifdef GYROSTBY 2014.05.19
		UcRamIni |=  0x01 ;					/* Set PLL bit */
//  #endif 2014.05.19
		
		RegWriteA( GSETDT,	UcRamIni ) ;	// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		
		RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		RegWriteA( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		
		GyOutSignal( ) ;					/* Select Gyro output signal 			*/
		
		WitTim( 50 ) ;						// 50ms wait
		
		RegWriteA( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON

		ClrGyr( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #else									/* Panasonic */
	
//	unsigned char	UcRamIni ;


	if(UcSelMode == ON)
	{
		RegWriteA( WC_EQON, 0x00 ) ;		// 0x0101	GYRO Equalizer OFF
		RegWriteA( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/
		RegWriteA( GRADR0,	0x4C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	0x02 ) ;		/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
	}
	else
	{
		RegWriteA( GRADR0,	0x4C ) ;		// 0x0283	Set Write Command
		RegWriteA( GSETDT,	0x00 ) ;		// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		GyOutSignal( ) ;					/* Select Gyro output signal 			*/
		
		WitTim( 50 ) ;						// 50ms wait
		
		RegWriteA( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON
		ClrGyr( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #endif
}

//********************************************************************************
// Function Name 	: GyOutSignal
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	GyOutSignal( void )
{

	RegWriteA( GRADR0,	GYROX_INI ) ;			// 0x0283	Set Gyro XOUT H~L
	RegWriteA( GRADR1,	GYROY_INI ) ;			// 0x0284	Set Gyro YOUT H~L
	
	/*Start OIS Reading*/
	RegWriteA( GRSEL	, 0x02 );				// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

//********************************************************************************
// Function Name 	: GyOutSignalCont
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Continuosl Function
// History			: First edition 						2013.06.06 Y.Shigeoka
//********************************************************************************
void	GyOutSignalCont( void )
{

	/*Start OIS Reading*/
	RegWriteA( GRSEL	, 0x04 );				// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

#endif	//STANDBY_MODE

#if 0//we no use
#define		GYROFF_HIGH_MOBILE		0x1482 //30 DPS
#define		GYROFF_LOW_MOBILE		0xEB7E //-30 DPS
//********************************************************************************
// Function Name 	: GenMesMobile
// Retun Value		: A/D Convert Result
// Argment Value	: Measure Filter Input Signal Ram Address
// Explanation		: General Measure Function
// History			: First edition 						2013.01.10 Y.Shigeoka
//********************************************************************************
short	GenMesMobile( unsigned short	UsRamAdd, unsigned char	UcMesMod )
{
	short	SsMesRlt ;
	unsigned char	UcMesFin;

	RegWriteA( WC_MES1ADD0, (unsigned char)UsRamAdd ) ;							// 0x0194
	RegWriteA( WC_MES1ADD1, (unsigned char)(( UsRamAdd >> 8 ) & 0x0001 ) ) ;	// 0x0195
	RamWrite32A( MSABS1AV, 0x00000000 ) ;				// 0x1041	Clear
	
	if( !UcMesMod ) {
		RegWriteA( WC_MESLOOP1, 0x04 ) ;				// 0x0193
		RegWriteA( WC_MESLOOP0, 0x00 ) ;				// 0x0192	1024 Times Measure
		RamWrite32A( msmean	, 0x3A7FFFF7 );				// 0x1230	1/CmMesLoop[15:0]
	} else {
		RegWriteA( WC_MESLOOP1, 0x01 ) ;				// 0x0193
		RegWriteA( WC_MESLOOP0, 0x00 ) ;				// 0x0192	1 Times Measure
		RamWrite32A( msmean	, 0x3F800000 );				// 0x1230	1/CmMesLoop[15:0]
	}

	RegWriteA( WC_MESABS, 0x00 ) ;						// 0x0198	none ABS
	//BsyWit( WC_MESMODE, 0x01 ) ;						// 0x0190	normal Measure
	RegWriteA( WC_MESMODE, 0x01 ) ;						// 0x0190	normal Measure
	WitTim( 100 );										// Wait 1024 Times Measure Time
	RegReadA( WC_MESMODE, &UcMesFin ) ;					// 0x0190	normal Measure
	if (0x00 == UcMesFin){
		WitTim( 100 );
	}


	RamAccFixMod( ON ) ;							// Fix mode
	
	RamReadA( MSABS1AV, ( unsigned short * )&SsMesRlt ) ;	// 0x1041

	RamAccFixMod( OFF ) ;							// Float mode
	
	return( SsMesRlt ) ;
}

//********************************************************************************
// Function Name 	: TneGvcMobile
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Gyro VC offset
// History			: First edition 						2013.01.15  Y.Shigeoka
//********************************************************************************
int	TneGvcMobile( void )
{
	int  SiRsltSts;
	unsigned short	UsGxoVal, UsGyoVal;
	unsigned char	UcGvcFlg = 0xFF;

	// A/D Offset Clear
	RegWriteA( IZAH,	0x00 ) ;	// 0x02A0		Set Offset High byte
	RegWriteA( IZAL,	0x00 ) ;	// 0x02A1		Set Offset Low byte
	RegWriteA( IZBH,	0x00 ) ;	// 0x02A2		Set Offset High byte
	RegWriteA( IZBL,	0x00 ) ;	// 0x02A3		Set Offset Low byte
	
	//MesFil( THROUGH ) ;				// Set Measure filter
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

	//////////
	// X
	//////////
	RegWriteA( WC_MES1ADD0, 0x00 ) ;		// 0x0194
	RegWriteA( WC_MES1ADD1, 0x00 ) ;		// 0x0195
	ClrGyr( 0x1000 , CLR_FRAM1 );						// Measure Filter RAM Clear
	UsGxoVal = (unsigned short)GenMesMobile( AD2Z, 0 );	// GYRMON1(0x1110) <- GXADZ(0x144A)
	RegWriteA( IZAH, (unsigned char)(UsGxoVal >> 8) ) ;	// 0x02A0		Set Offset High byte
	RegWriteA( IZAL, (unsigned char)(UsGxoVal) ) ;		// 0x02A1		Set Offset Low byte

	//////////
	// Y
	//////////
	RegWriteA( WC_MES1ADD0, 0x00 ) ;		// 0x0194
	RegWriteA( WC_MES1ADD1, 0x00 ) ;		// 0x0195
	ClrGyr( 0x1000 , CLR_FRAM1 );						// Measure Filter RAM Clear
	UsGyoVal = (unsigned short)GenMesMobile( AD3Z, 0 );	// GYRMON2(0x1111) <- GYADZ(0x14CA)
	RegWriteA( IZBH, (unsigned char)(UsGyoVal >> 8) ) ;	// 0x02A2		Set Offset High byte
	RegWriteA( IZBL, (unsigned char)(UsGyoVal) ) ;		// 0x02A3		Set Offset Low byte
	
	SiRsltSts = EXE_END ;						/* Clear Status */

	if(( (signed short)UsGxoVal < (signed short)GYROFF_LOW_MOBILE ) || ( (signed short)UsGxoVal > (signed short)GYROFF_HIGH_MOBILE ))
	{
		SiRsltSts |= EXE_GXADJ ;
	}
	
	if(( (signed short)UsGyoVal < (signed short)GYROFF_LOW_MOBILE ) || ( (signed short)UsGyoVal > (signed short)GYROFF_HIGH_MOBILE ))
	{
		SiRsltSts |= EXE_GYADJ ;
	}
	
	E2pWrt( (unsigned short)0x0937	, 2, ( unsigned char * )&UsGxoVal ) ;WitTim( 10 );		//GYRO OFFSET Mobile X
	E2pWrt( (unsigned short)0x0939	, 2, ( unsigned char * )&UsGyoVal ) ;WitTim( 10 );		//GYRO OFFSET Mobile Y
		
	if( EXE_END == SiRsltSts ){
		UcGvcFlg  = 0xE7 ;
		E2pWrt( (unsigned short)0x093B	, 1, ( unsigned char * )&UcGvcFlg ) ;WitTim( 10 );	//GYRO OFFSET Mobile Flag
		SiRsltSts = 0;																		//Success
	}else{
		UcGvcFlg = 0xF0 ;
		E2pWrt( (unsigned short)0x093B	, 1, ( unsigned char * )&UcGvcFlg ) ;WitTim( 10 );	//GYRO OFFSET Mobile Flag
		SiRsltSts = -1;																		//Fail
	}

	return( SiRsltSts );
}


#endif

#if 0//we no use
//********************************************************************************
// Function Name 	: GyroEllipseLmt
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Gyro circle limit
// History			: First edition 						
//********************************************************************************
#define	LimitCoef	(float)0.68449		/* 2 * ÉŒ * 32.8lsb/Åã/s * 4Hz * 1deg * 26.7556dB * 85um/68um / 7fffh(4HzéûÇÃêœï™â”èägainÇ≈ä∑éZ) */
void GyroEllipseLmt( unsigned char ucmode )
{
    UnFltVal	UnAdjLmtx, UnAdjLmty, UnH1Lmtval ;
	UnFltVal	UnAdjLmt2x, UnAdjLmt2y ;			// gxlens, gylens

	UnH1Lmtval.SfFltVal = LimitCoef;

	RamRead32A( gxzoom    , &UnAdjLmtx.UlLngVal );
	RamRead32A( gyzoom    , &UnAdjLmty.UlLngVal );
	
	UnAdjLmtx.SfFltVal *= UnH1Lmtval.SfFltVal;
	UnAdjLmty.SfFltVal *= UnH1Lmtval.SfFltVal;
	
////////// gxlens, gylens ////////// ////////// //////////
	RamRead32A( gxlens    , &UnAdjLmt2x.UlLngVal );
	RamRead32A( gylens    , &UnAdjLmt2y.UlLngVal );
	
	UnAdjLmtx.SfFltVal *= UnAdjLmt2x.SfFltVal;
	UnAdjLmty.SfFltVal *= UnAdjLmt2x.SfFltVal;
////////// gxlens, gylens ////////// ////////// //////////
	
	if(UnAdjLmtx.SfFltVal < 0 )		UnAdjLmtx.SfFltVal = -UnAdjLmtx.SfFltVal;
	if(UnAdjLmty.SfFltVal < 0 )		UnAdjLmty.SfFltVal = -UnAdjLmty.SfFltVal;
	
	RamWrite32A( gxlmt3HS0 , UnAdjLmtx.UlLngVal );
	RamWrite32A( gylmt3HS0 , UnAdjLmty.UlLngVal );
	
	if(!ucmode){
		RegWriteA( WG_LMT3MOD , 0x00 );
	}else{
		
		RegWriteA( WG_LMT3MOD , 0x01 );
	}
}
#endif
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
 
 // Dead Lock Check
#define READ_COUNT_NUM	3
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
 
//==============================================================================
// ois_init.c Code END
//==============================================================================

int imx278_lgit_ois_init(void)
{
#if 0
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
#endif
    return 0;
}

int imx278_lgit_ois_initSpecial(void)
{
#if 0
     return IniSetAf();
#else
    return 0;
#endif
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
        imx278_lgit_ois_change_mode(OIS_CAPTURE);
        OisEna();
    }
    else
    {
    	imx278_lgit_ois_change_mode(OIS_PREVIEW);
        RtnCen(0x00);
    }

    return 0;
}
int imx278_lgit_ois_turn_onoff_lin(uint32_t on)
{
    if(on)
    {
        imx278_lgit_ois_change_mode(OIS_CAPTURE);
        OisEna();
    }
    else
    {
        imx278_lgit_ois_change_mode(OIS_PREVIEW);
        RtnCen(0x00);
    }
    return 0;
}

void imx278_lgit_InitOISData(void *oisdata, uint32_t oislenth)
 {
 #if 0
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
#endif
 }
 //for test interface
int imx278_lgit_ois_AF_SetPos(short data)
{
    return AF_SetPos(data);
}
int imx278_lgit_ois_RamAccFixMod(uint8_t data)
{
     RamAccFixMod(data);
    return 0;
}
int imx278_lgit_ois_RtnCen(uint8_t data)
{
     RtnCen(data);
	 return 0;
}
int imx278_lgit_ois_SetPanTiltMode(uint8_t data)
{
     SetPanTiltMode(data);
	return 0;
}
int imx278_lgit_ois_RunGea(void)
{
#if 0
    return RunGea();
#else
return 0;
#endif
}
int imx278_lgit_ois_RunHea(void)
{
#if 0
    return RunHea();
#else
return 0;
#endif
}
int imx278_lgit_ois_OisEna(void)
{
     OisEna();
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

    SrvCon( X_DIR, UcStbX ) ;
    SrvCon( Y_DIR, UcStbY ) ;

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

int imx278_lgit_ois_MagnetismRead(int32_t *otpcenX, int32_t *otpcenY, int32_t *srvoffX, int32_t *srvoffY)
{
    RamRead32A(OFF0Z ,otpcenX);
    RamRead32A(OFF1Z ,otpcenY);
    RamRead32A(AD0OFFZ, srvoffX);
    RamRead32A(AD1OFFZ, srvoffY);
    return 0;
}

