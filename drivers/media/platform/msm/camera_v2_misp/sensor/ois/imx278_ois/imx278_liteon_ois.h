/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#ifndef _IMX278_LITEON_OIS_HEAD
#define _IMX278_LITEON_OIS_HEAD

#include "OisDef.h"

void imx278_liteon_InitOISData(void *oisdata, uint32_t oislenth);
int imx278_liteon_ois_init(void);
int imx278_liteon_ois_initSpecial(void);
int imx278_liteon_ois_turn_onoff(uint32_t on);
int imx278_liteon_ois_turn_onoff_lin(uint32_t on);
//for test interface
int imx278_liteon_ois_AF_SetPos(short);
int imx278_liteon_ois_RamAccFixMod(uint8_t);
int imx278_liteon_ois_RtnCen(uint8_t);
int imx278_liteon_ois_SetPanTiltMode(uint8_t);
int imx278_liteon_ois_RunGea(void);
int imx278_liteon_ois_RunHea(void);
int imx278_liteon_ois_OisEna(void);
int imx278_liteon_ois_setGyroGain(int32_t new_xgain, int32_t new_ygain);
int imx278_liteon_ois_getGyroGain(int32_t* xgain, int32_t* ygain);
int imx278_liteon_ois_MagnetismRead(int32_t *otpcenX, int32_t *otpcenY, int32_t *srvoffX, int32_t *srvoffY);
int imx278_liteon_ois_StbOnnN( uint8_t UcStbY , uint8_t UcStbX );
#endif
