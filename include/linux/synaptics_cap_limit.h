/*Add for huawei TP*/
/*
 * Copyright (c) 2014 Huawei Device Company
 *
 * This file provide capacitor limit for synaptics touch IC.
 * 
 * 2014-01-26:Add "synaptics_cap_limit.h" by sunlibin
 *
 */
#ifndef __SYNAPTICS_CAP_LIMIT__
#define __SYNAPTICS_CAP_LIMIT__

/*Add synaptics capacitor test function */
#define MMITEST

#ifdef MMITEST
/*Default config*/
#define TOUCH_RX_DEFAULT 28
#define TOUCH_TX_DEFAULT 15
/*Config for 5 inch TP*/
#define TOUCH_TX_5INCH	13
#define TOUCH_RX_5INCH	24
/*Config for 5.5 inch TP*/
#define TOUCH_TX_5p5INCH	15
#define TOUCH_RX_5p5INCH	28

/*Default config*/
/*To define the default capacitance threshold value */
#define MAX_CAP_LIMIT 9999
#define MIN_CAP_LIMIT -9999
#define RX_DIAG_ULIMIT 1150
#define RX_DIAG_LLIMIT 850
#define RX_OTHER_ULIMIT 100
/*To delete the following variables and arrays, because the synaptics capacitance limit has been moved to the dts*/


#endif/*MMITEST*/

#endif

