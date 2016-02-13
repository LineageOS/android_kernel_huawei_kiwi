/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#ifndef _IMX278_SUNNY_OIS_HEAD
#define _IMX278_SUNNY_OIS_HEAD

#include "OisDef.h"

void imx278_sunny_InitOISData(void *oisdata, uint32_t oislenth);
int imx278_sunny_ois_init(void);
int imx278_sunny_ois_initSpecial(void);
int imx278_sunny_ois_turn_onoff(uint32_t on);
int imx278_sunny_ois_turn_onoff_lin(uint32_t on);
//for test interface
int imx278_sunny_ois_AF_SetPos(short);
int imx278_sunny_ois_RamAccFixMod(uint8_t);
int imx278_sunny_ois_RtnCen(uint8_t);
int imx278_sunny_ois_SetPanTiltMode(uint8_t);
int imx278_sunny_ois_RunGea(void);
int imx278_sunny_ois_RunHea(void);
int imx278_sunny_ois_OisEna(void);
int imx278_sunny_ois_MagnetismRead(int32_t *otpcenX, int32_t *otpcenY, int32_t *srvoffX, int32_t *srvoffY);
int imx278_sunny_ois_StbOnnN( uint8_t UcStbY , uint8_t UcStbX );
int imx278_sunny_ois_setGyroGain(int32_t xgain, int32_t ygain);
int imx278_sunny_ois_getGyroGain(int32_t* xgain, int32_t* ygain);
#endif
