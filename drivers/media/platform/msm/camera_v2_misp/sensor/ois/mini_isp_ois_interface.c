/*add camera ois driver*/
/*adjust back camera resolution and optimize ois driver*/
#include "../mini_isp/camera_agent.h"
#include "mini_isp_ois_interface.h"

#include "./imx278_ois/imx278_sunny_ois.h"
#include "./imx278_ois/imx278_liteon_ois.h"
#include "./imx278_ois/imx278_lgit_ois.h"


#include "msm_ois.h"
#include <media/msm_cam_sensor.h>

#define CHECK_RETURN(a) do \
{ \
    if(a != 0) \
    { \
        pr_err("%s:%d fail! \n",__func__,__LINE__); \
        return -1; \
    } \
}while(0)


/*#define MSM_OIS_INTERFACE_DEBUG*/
#undef OIS_CDBG
#ifdef MSM_OIS_INTERFACE_DEBUG
#define OIS_CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define OIS_CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#ifndef ON
 #define    ON    0x01
 #define    OFF    0x00
#endif
//delete ois otp

struct msm_imx278_ois_fn_t {
	int (*initialize) (void);
    int (*initializeSpecial) (void);
	void (*init_ois_otp)(void *, uint32_t);
	int (*turn_onoff) (uint32_t);
    int (*turn_onoff_lin) (uint32_t);

    //for test interface
    int (*AF_SetPos)(short);
    int (*RamAccFixMod)(uint8_t);
    int (*RtnCen)(uint8_t);
    int (*SetPanTiltMode)(uint8_t);
    int (*RunGea)(void);
    int (*RunHea)(void);
    int (*OisEna)(void);
    int (*MagnetismRead)(int32_t *, int32_t *, int32_t *, int32_t *);
    int (*StbOnnN)( uint8_t UcStbY , uint8_t UcStbX );
    int (*setGyroGain)(int32_t, int32_t);
    int (*getGyroGain)(int32_t*, int32_t*);
};

static struct msm_imx278_ois_fn_t imx278_sunny_ois_func = {
	.initialize = imx278_sunny_ois_init,
	.initializeSpecial = imx278_sunny_ois_initSpecial, //af
	.init_ois_otp = imx278_sunny_InitOISData,
	.turn_onoff = imx278_sunny_ois_turn_onoff,
    .turn_onoff_lin = imx278_sunny_ois_turn_onoff_lin,

    //for test interface
	.AF_SetPos = imx278_sunny_ois_AF_SetPos,
	.RamAccFixMod = imx278_sunny_ois_RamAccFixMod,
	.RtnCen = imx278_sunny_ois_RtnCen,
	.SetPanTiltMode = imx278_sunny_ois_SetPanTiltMode,
	.RunGea = imx278_sunny_ois_RunGea,
	.RunHea = imx278_sunny_ois_RunHea,
	.OisEna = imx278_sunny_ois_OisEna,
	.MagnetismRead = imx278_sunny_ois_MagnetismRead,
	.StbOnnN = imx278_sunny_ois_StbOnnN,
	//for adjust gyro gain
	.setGyroGain = imx278_sunny_ois_setGyroGain,
	.getGyroGain = imx278_sunny_ois_getGyroGain,
};
static struct msm_imx278_ois_fn_t imx278_liteon_ois_func = {
	.initialize = imx278_liteon_ois_init,
	.initializeSpecial = imx278_liteon_ois_initSpecial, //af
	.init_ois_otp = imx278_liteon_InitOISData,
	.turn_onoff = imx278_liteon_ois_turn_onoff,
    .turn_onoff_lin = imx278_liteon_ois_turn_onoff_lin,

    //for test interface
	.AF_SetPos = imx278_liteon_ois_AF_SetPos,
	.RamAccFixMod = imx278_liteon_ois_RamAccFixMod,
	.RtnCen = imx278_liteon_ois_RtnCen,
	.SetPanTiltMode = imx278_liteon_ois_SetPanTiltMode,
	.RunGea = imx278_liteon_ois_RunGea,
	.RunHea = imx278_liteon_ois_RunHea,
	.OisEna = imx278_liteon_ois_OisEna,
	.MagnetismRead = imx278_liteon_ois_MagnetismRead,
	.StbOnnN = imx278_liteon_ois_StbOnnN,
	//for adjust gyro gain
	.setGyroGain = imx278_liteon_ois_setGyroGain,
	.getGyroGain = imx278_liteon_ois_getGyroGain,
};
static struct msm_imx278_ois_fn_t imx278_lgit_ois_func = {
	.initialize = imx278_lgit_ois_init,
	.initializeSpecial = imx278_lgit_ois_initSpecial, //af
	.init_ois_otp = imx278_lgit_InitOISData,
	.turn_onoff = imx278_lgit_ois_turn_onoff,
    .turn_onoff_lin = imx278_lgit_ois_turn_onoff_lin,

    //for test interface
	.AF_SetPos = imx278_lgit_ois_AF_SetPos,
	.RamAccFixMod = imx278_lgit_ois_RamAccFixMod,
	.RtnCen = imx278_lgit_ois_RtnCen,
	.SetPanTiltMode = imx278_lgit_ois_SetPanTiltMode,
	.RunGea = imx278_lgit_ois_RunGea,
	.RunHea = imx278_lgit_ois_RunHea,
	.OisEna = imx278_lgit_ois_OisEna,
	.MagnetismRead = imx278_lgit_ois_MagnetismRead,
	.StbOnnN = imx278_lgit_ois_StbOnnN,
};

static int exit_flag = 0;
void mini_isp_init_exit_flag(int flag)
{
    exit_flag = flag;
}

static struct msm_imx278_ois_fn_t* check_module_id(int32_t moduleid)
{
    struct msm_imx278_ois_fn_t *ois_func = NULL;

    switch(moduleid)
    {
        case SUNNY_MODULE_ID:
            ois_func = &imx278_sunny_ois_func;
            break;
        case LITEON_MODULE_ID:
            ois_func = &imx278_liteon_ois_func;
            break;
        case LGIT_MODULE_ID:
            ois_func = &imx278_lgit_ois_func;

        default:
            break;
    };

    return ois_func;
}

//delete ois otp
int32_t mini_isp_ois_init(int32_t module_id)
{
    int32_t rc = -EINVAL;
    struct msm_imx278_ois_fn_t *ois_func = check_module_id(module_id);

    if(ois_func && ois_func->initialize)
    {
        rc = ois_func->initialize();

        if(rc !=0 )
        {
            pr_err("%s: fail! \n", __func__);
        }
    }

    return rc;
}
int32_t mini_isp_ois_setGyroGain(int32_t module_id, int32_t new_xgain, int32_t new_ygain)
{
    int32_t rc = 0;
    struct msm_imx278_ois_fn_t *ois_func = check_module_id(module_id);

    if(ois_func && ois_func->setGyroGain)
    {
        rc = ois_func->setGyroGain(new_xgain,new_ygain);
    }

    return rc;
}

int32_t mini_isp_ois_getGyroGain(int32_t module_id, int32_t* xgain, int32_t* ygain)
{
    int32_t rc = 0;
    struct msm_imx278_ois_fn_t *ois_func = check_module_id(module_id);

    if(ois_func && ois_func->getGyroGain)
    {
        rc = ois_func->getGyroGain(xgain,ygain);
    }

    return rc;
}

int32_t mini_isp_ois_turn_on(int32_t module_id, uint32_t onoff, int32_t turn_on_type)
{
    int32_t rc = -EINVAL;
    struct msm_imx278_ois_fn_t *ois_func = check_module_id(module_id);

    if(ois_func && ois_func->turn_onoff)
    {
        if(turn_on_type)
        {
             pr_err("ois turn_onoff_lin in mmitest mode\n");
             rc = ois_func->turn_onoff_lin(onoff);
        }
        else
        {
             pr_err("ois turn_onoff in normal mode\n");
             rc = ois_func->turn_onoff(onoff);
        }
        
        if(rc !=0 )
        {
            pr_err("%s: fail! \n", __func__);
        }
    }
    
    return rc;
}
#define OIS_RAM_X               0x1450
#define OIS_RAM_Y               0x14D0

int mini_isp_ois_start_running_test(int32_t module_id)
{
    int rc = 0;
    unsigned short Xbak, Ybak;
    struct msm_imx278_ois_fn_t *ois_func = check_module_id(module_id);

    if(!ois_func)
    {
        return -1;
    }

    pr_info("%s: enter \n",__func__);

    //disable ois
    rc = ois_func->RtnCen(0x00);
    CHECK_RETURN(rc);

    /* Aging Test Start */
    rc = ois_func->RamAccFixMod(1);
    CHECK_RETURN(rc);
    
    rc = mini_isp_ois_RamRead16A(OIS_RAM_X, &Xbak);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamRead16A(OIS_RAM_Y, &Ybak);
    CHECK_RETURN(rc);

    //position 1
    rc = ois_func->AF_SetPos(1023);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(50);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, 0x999);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, 0x666);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(500);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, Xbak);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, Ybak);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(450);

    //postion 2
    rc = ois_func->AF_SetPos(1023);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(50);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, 0x666);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, 0x999);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(500);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, Xbak);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, Ybak);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(450);

    //postion 3
    rc = ois_func->AF_SetPos(1023);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(50);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, 0x999);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, 0x999);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(500);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, Xbak);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, Ybak);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(450);

    //postion 4
    rc = ois_func->AF_SetPos(1023);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(50);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, 0x666);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, 0x999);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(500);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_X, Xbak);
    CHECK_RETURN(rc);
    rc = mini_isp_ois_RamWrite16A(OIS_RAM_Y, Ybak);
    CHECK_RETURN(rc);
    mini_isp_ois_WitTim(450);
    rc = ois_func->RamAccFixMod(0);
    CHECK_RETURN(rc);

    return 0;
}
/*delete mini_isp_ois_start_mmi_test function, no use*/

int32_t mini_isp_ois_start_mag_test(int32_t module_id, void *userdata)
{
    struct msm_ois_cfg_data *cdata = (struct msm_ois_cfg_data *)userdata;
    struct msm_imx278_ois_fn_t *ois_func = check_module_id(module_id);
    struct	msm_ois_mag_info_t maginfo = {0};
    void *ptr_dest = NULL;
    int rc = 0;

    pr_info("Enter: %s \n", __func__);
    if(!cdata || !ois_func)
    {
        return -1;
    }

    ois_func->StbOnnN(OFF, OFF);
    mini_isp_ois_WitTim(300);
    ois_func->MagnetismRead(&(maginfo.otpcenX), &(maginfo.otpcenY), &(maginfo.srvoffX), &(maginfo.srvoffY));

    pr_info("%s Enter: %8x %8x %8x %8x \n", __func__,maginfo.otpcenX, maginfo.otpcenY, maginfo.srvoffX, maginfo.srvoffY);

    ptr_dest = (void *) cdata->cfg.mag_info;

    rc  = copy_to_user((void __user *)ptr_dest, &maginfo, sizeof(maginfo));
    if(rc)
    {
        pr_err("%s cannot copy error rc=%d \n", __func__, rc);
    }

    pr_info("%s mag_info: %8x %8x %8x %8x \n", __func__,cdata->cfg.mag_info->otpcenX,
        cdata->cfg.mag_info->otpcenY, cdata->cfg.mag_info->srvoffX, cdata->cfg.mag_info->srvoffY);

    ois_func->RtnCen(0x00);
    mini_isp_ois_WitTim(20);
    //Pan/Tilt ON
    ois_func->SetPanTiltMode(1);
    mini_isp_ois_WitTim(10);
    /*move to msm_ois_mag_test*/

    return 0;
}

void mini_isp_ois_WitTim(uint16_t delay)
{
    usleep(delay * 1000); //delay ms
}

static uint16_t ois_cal_checksum(uint8_t *buf, uint32_t size)
{
    uint16_t index;
    uint32_t sum = 0;
    uint16_t sumvalue;

    for(index = 0; index < size; index ++) {
        if(0 == (index % 2))
            sum += buf[index];
        else
            sum += (buf[index] << 8);
    }
    sumvalue = (uint16_t)(65536 - (sum & 0x0000FFFF));
    return sumvalue;
}

#ifdef MSM_OIS_INTERFACE_DEBUG
static void mini_isp_ois_dump_data(uint8_t *data, uint32_t length)
{
    int32_t i = 0;
    for(i = 0; i<length; i++)
    {
        pr_err("data[%d] = %d \n",i,(int32_t)data[i]);
    }
}
#endif

//delete ois otp
int mini_isp_ois_RamWrite32A(uint32_t addr, uint32_t data)
{ 
    uint16_t length = 6;    
    int errorcode = 0;
    uint16_t checksum = 0;
    uint32_t cmdLen = 8, block = 8192, len = 11;
    uint8_t in_buf[8] = {0};  
    uint8_t out[11] = {0}; //bluk data size+ 2 bytes ISPCMD_CKSUMBYTES

    //fill in buf, length is cmdlen
    memcpy(&(in_buf[0]), &len, sizeof(len));    
    memcpy(&(in_buf[4]), &block, sizeof(block));

    //fill bulk buffer: 1byte ctrl + 16bit length + 16bit addr + 32bit data
    out[0] = 0x17; //pair mode, 16-bit address, 32-bit data, write  
    out[1] = length & 0xff;
    out[2] = (length >> 8) & 0xff; //The data length from 1st Address to the end
    out[3] = (addr >> 8) & 0xff;
    out[4] = addr & 0xff;
    out[5] = (data >> 24) & 0xff;
    out[6] = (data >> 16) & 0xff;
    out[7] = (data >> 8) & 0xff;
    out[8] = data & 0xff;
    checksum = ois_cal_checksum(out, len-2);
    out[9] = checksum & 0xff;
    out[10] = (checksum >> 8) & 0xff;
   
    // send by spi
    errorcode = misp_exec_write_block(ISPCMD_BULK_WRITE_OIS_DATA,in_buf,cmdLen,out,len);
    if(errorcode)
    {
        pr_err("%s: fail addr=0x%x, data=0x%x\n",__func__,addr,data);
    }
    else
    {
        OIS_CDBG("%s: Out addr = 0x%x success \n",__func__,addr);
    }

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RamRead32A(uint16_t addr, void* data)
{
    uint32_t OpCode = ISPCMD_BULK_READ_OIS_DATA;
    uint8_t ctrl_byte = 0x16;    //pair mode, 16-bit address, 32-bit data, read    
    uint16_t length = 6;    
    uint8_t out[6] = {0}; //recv 32bit data
    uint8_t in_buf[11] = {0};
    uint8_t t_buf[4]={0};
    int errorcode = 0;
    uint32_t cmdLen = 11, block = 8192;

    //fill buffer: 1byte ctrl + 16bit length + 16bit addr + 32bit data
    in_buf[0] = ctrl_byte;
    in_buf[1] = length & 0xff;
    in_buf[2] = (length >> 8) & 0xff;
    in_buf[3] = (addr >> 8) & 0xff;
    in_buf[4] = addr & 0xff;
    in_buf[5] = 0;
    in_buf[6] = 0;
    memcpy(&(in_buf[7]), &block, sizeof(block));
    
    errorcode = misp_exec_bidir_cmd(OpCode,in_buf,cmdLen,1,out,length);
    if(errorcode)
    {
        pr_err("%s: fail addr=0x%x\n",__func__,addr);
    }
    else
    {
        OIS_CDBG("%s: Out addr = 0x%x success \n",__func__,addr);
    }  
    
    t_buf[0] = out[3];    
    t_buf[1] = out[2];    
    t_buf[2] = out[1];    
    t_buf[3] = out[0];

    memcpy(data, t_buf, 4);

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RamWrite16A(uint16_t addr, uint16_t data)
{ 
    uint16_t length = 4;     
    int errorcode = 0;
    uint16_t checksum = 0;
    uint32_t cmdLen = 8, block = 8192, len = 9;
    uint8_t in_buf[8] = {0};  
    uint8_t out[9] = {0}; //bluk data size+ 2 bytes ISPCMD_CKSUMBYTES
    
    //fill in buf, length is cmdlen
    memcpy(&(in_buf[0]), &len, sizeof(len));    
    memcpy(&(in_buf[4]), &block, sizeof(block));
    
    //fill bulk buffer: 1byte ctrl + 16bit length + 16bit addr + 16bit data
    out[0] = 0x13;    //pair mode, 16-bit address, 16-bit data, write  
    out[1] = length & 0xff;
    out[2] = (length >> 8) & 0xff; //The data length from 1st Address to the end
    out[3] = (addr >> 8) & 0xff;
    out[4] = addr & 0xff;
    out[5] = (data >> 8) & 0xff;
    out[6] = data & 0xff;
    checksum = ois_cal_checksum(out, len-2);
    out[7] = checksum & 0xff;
    out[8] = (checksum >> 8) & 0xff;
   
    // send by spi
    errorcode = misp_exec_write_block(ISPCMD_BULK_WRITE_OIS_DATA,in_buf,cmdLen,out,len);
    if(errorcode)
    {
        pr_err("%s: fail addr=0x%x, data=0x%x\n",__func__,addr,data);
    }
    else
    {
        OIS_CDBG("%s: Out addr = 0x%x success \n",__func__,addr);
    }

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RamRead16A(uint16_t addr, uint16_t *data)
{
    uint32_t OpCode = ISPCMD_BULK_READ_OIS_DATA;
    uint8_t ctrl_byte = 0x12;    //pair mode, 16-bit address, 16-bit data, read
    uint16_t length = 4;//2byte addr + 2byte data
    uint8_t out[4] = {0}; //recv 16bit data
    uint8_t in_buf[11] = {0};
    int errorcode = 0;
    uint32_t cmdLen = 11, block = 8192;

    //fill buffer: 1byte ctrl + 16bit length + 16bit addr + 16bit data
    in_buf[0] = ctrl_byte;
    in_buf[1] = length & 0xff;
    in_buf[2] = (length >> 8) & 0xff;
    in_buf[3] = (addr >> 8) & 0xff;
    in_buf[4] = addr & 0xff;
    in_buf[5] = 0;
    in_buf[6] = 0;
    memcpy(&(in_buf[7]), &block, sizeof(block));
    
    errorcode = misp_exec_bidir_cmd(OpCode,in_buf,cmdLen,1,out,length);
    if(errorcode)
    {
        pr_err("%s: fail addr=0x%x\n",__func__,addr);
    }
    else
    {
        OIS_CDBG("%s: Out addr = 0x%x success \n",__func__,addr);
    } 
       
    *data = (out[0] << 8) | out[1];

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RegWrite8A(uint16_t addr, uint16_t data)
{
    uint16_t length = 3;    
    int errorcode = 0;
    uint16_t checksum = 0;
    uint32_t cmdLen = 8, block = 8192, len = 8;
    uint8_t out[8] = {0}; //bluk data size+ 2 bytes ISPCMD_CKSUMBYTES
    uint8_t in_buf[8] = {0};
    //uint32_t in_buf_u[2] = {len,block};
    
    memcpy(&(in_buf[0]), &len, sizeof(len));    
    memcpy(&(in_buf[4]), &block, sizeof(block));

    //fill bulk buffer: 1byte ctrl + 16bit length + 16bit addr + 8bit data
    out[0] = 0x11; //pair mode, 16-bit address, 8-bit data, write  
    out[1] = length & 0xff;
    out[2] = (length >> 8) & 0xff; //The data length from 1st Address to the end
    out[3] = (addr >> 8) & 0xff;
    out[4] = addr & 0xff;
    out[5] = data & 0xff;
    checksum = ois_cal_checksum(out, len-2);
    out[6] = checksum & 0xff;
    out[7] = (checksum >> 8) & 0xff;
   
    // send by spi
    errorcode = misp_exec_write_block(ISPCMD_BULK_WRITE_OIS_DATA,in_buf,cmdLen,out,len);
    if(errorcode)
    {
        pr_err("%s: fail addr=0x%x, data=0x%x\n",__func__,addr,data);
    }
    else
    {
        OIS_CDBG("%s: Out addr = 0x%x success \n",__func__,addr);
    }

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RegRead8A(uint16_t addr, uint8_t *data)
{
    uint32_t OpCode = ISPCMD_BULK_READ_OIS_DATA;
    uint8_t ctrl_byte = 0x10;    //pair mode, 16-bit address, 8-bit data, read
    uint16_t length = 3;//2byte addr + 1byte data
    uint8_t out[3] = {0}; //recv 8bit data
    uint8_t in_buf[11] = {0};
    uint32_t cmdLen = 11, block = 8192;
    int errorcode = 0;

    //fill buffer: 1byte ctrl + 16bit length + 16bit addr + 8bit data
    in_buf[0] = ctrl_byte;
    in_buf[1] = length & 0xff;
    in_buf[2] = (length >> 8) & 0xff;
    in_buf[3] = (addr >> 8) & 0xff;
    in_buf[4] = addr & 0xff;
    in_buf[5] = 0;
    in_buf[6] = 0;
    memcpy(&(in_buf[7]), &block, sizeof(block));
    
    errorcode = misp_exec_bidir_cmd(OpCode,in_buf,cmdLen,1,out,length);
    if(errorcode)
    {
        pr_err("%s: fail addr=0x%x\n",__func__,addr);
    }
    else
    {
        OIS_CDBG("%s: Out addr = 0x%x data = 0x%x success \n",__func__,addr,(int)out[0]);
    } 
       
    *data = out[0];

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RamWrite16Burst(uint8_t data[], uint16_t length)
{
    uint8_t *out = NULL;  //bluk data size+ 2 bytes ISPCMD_CKSUMBYTES
    int errorcode = 0;
    int32_t i = 0;
    uint16_t checksum = 0;
    uint32_t cmdLen = 8, block = 8192, len = 5 + length;
    uint8_t in_buf[8] = {0};

    out = (uint8_t *)kmalloc(len,GFP_KERNEL);
    if(!out)
    {
        pr_err("%s: kmalloc %d bytes fail \n",__func__,len);
        return -1;
    }

    //fill in buf, length is cmdlen
    memcpy(&(in_buf[0]), &len, sizeof(len));    
    memcpy(&(in_buf[4]), &block, sizeof(block));

    //fill bulk buffer: 1byte ctrl + 16bit length + 16bit addr + 16bit data
    out[0] = 0x93;    //burst mode, 16-bit address, 16-bit data, write 
    out[1] = length & 0xff;
    out[2] = (length >> 8) & 0xff; //The data length from 1st Address to the end
    for(i = 0; i < length; i++ ) 
    {        
        out[3 + i] = data[i];    
    }
    
    checksum = ois_cal_checksum(out, len - 2);    
    out[3 + i] = checksum & 0xff;    
    out[4 + i] = (checksum >> 8) & 0xff;
   
    // send by spi
    errorcode = misp_exec_write_block(ISPCMD_BULK_WRITE_OIS_DATA,in_buf,cmdLen,out,len);
    if(errorcode)
    {
        pr_err("%s: fail \n",__func__);
#ifdef MSM_OIS_INTERFACE_DEBUG
        mini_isp_ois_dump_data(data,length);
#endif
    }
    else
    {
        OIS_CDBG("%s: Out length = %d success \n",__func__,length);
    } 

    kfree(out);
    out = NULL;

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RegWrite8Burst(uint8_t data[], uint16_t length)
{     
    uint8_t *out = NULL;
    int errorcode = 0;
    int32_t i = 0;
    uint16_t checksum = 0;
    uint32_t cmdLen = 8, block = 8192, len = 5 + length;
    uint8_t in_buf[8] = {0};

    out = (uint8_t *)kmalloc(len,GFP_KERNEL);
    if(!out)
    {
        pr_err("%s: kmalloc %d bytes fail \n",__func__,len);
        return -1;
    }

    //fill in buf, length is cmdlen
    memcpy(&(in_buf[0]), &len, sizeof(len));    
    memcpy(&(in_buf[4]), &block, sizeof(block));

    //fill bulk buffer: 1byte ctrl + 16bit length + 16bit addr + 8bit data
    out[0] = 0x91; //burst mode, 16-bit address, 8-bit data, write
    out[1] = length & 0xff;
    out[2] = (length >> 8) & 0xff; //The data length from 1st Address to the end
    for(i = 0; i < length; i++ ) 
    {        
        out[3 + i] = data[i];    
    }
    
    checksum = ois_cal_checksum(out, len - 2);    
    out[3 + i] = checksum & 0xff;    
    out[4 + i] = (checksum >> 8) & 0xff;
   
    // send by spi
    errorcode = misp_exec_write_block(ISPCMD_BULK_WRITE_OIS_DATA,in_buf,cmdLen,out,len);
    if(errorcode)
    {
        pr_err("%s: fail \n",__func__);
#ifdef MSM_OIS_INTERFACE_DEBUG
        mini_isp_ois_dump_data(data,length);
#endif
    }
    else
    {
        OIS_CDBG("%s: Out length = %d success \n",__func__,length);
    } 

    kfree(out);
    out = NULL;

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}

int mini_isp_ois_RamWrite32Burst(uint8_t data[], uint16_t length)
{      
    uint8_t *out = NULL;
    int errorcode = 0;
    int32_t i = 0;
    uint16_t checksum = 0;
    uint32_t cmdLen = 8, block = 8192, len = 5 + length;
    uint8_t in_buf[8] = {0};

    out = (uint8_t *)kmalloc(len,GFP_KERNEL);
    if(!out)
    {
        pr_err("%s: kmalloc %d bytes fail \n",__func__,len);
        return -1;
    }

    //fill in buf, length is cmdlen
    memcpy(&(in_buf[0]), &len, sizeof(len));    
    memcpy(&(in_buf[4]), &block, sizeof(block));

    //fill bulk buffer: 1byte ctrl + 16bit length + 32bit addr + 32bit data
    out[0] = 0x97;    //burst mode, 16-bit address, 32-bit data, write
    out[1] = length & 0xff;
    out[2] = (length >> 8) & 0xff; //The data length from 1st Address to the end
    for(i = 0; i < length; i++ ) 
    {        
        out[3 + i] = data[i];
    }

    checksum = ois_cal_checksum(out, len - 2);    
    out[3 + i] = checksum & 0xff;    
    out[4 + i] = (checksum >> 8) & 0xff;
   
    // send by spi
    errorcode = misp_exec_write_block(ISPCMD_BULK_WRITE_OIS_DATA,in_buf,cmdLen,out,len);
    if(errorcode)
    {
        pr_err("%s: fail \n",__func__);
#ifdef MSM_OIS_INTERFACE_DEBUG
        mini_isp_ois_dump_data(data,length);
#endif
    }
    else
    {
        OIS_CDBG("%s: Out length = %d success \n",__func__,length);
    } 

    kfree(out);
    out = NULL;

    if(exit_flag)
    {
        pr_info("%s:%d exit_flag is true to force end init\n",__func__,__LINE__);
        errorcode = -1;
    }

    return errorcode;
}




