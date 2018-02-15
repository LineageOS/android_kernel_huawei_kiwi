/******************************************************************************
 * @file result.c
 *
 * result.c
 *
 * @version 0.0.1
 * @authors btok
 *
 *****************************************************************************//*
 * Copyright (2014), Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, associated documentation and materials ("Software") is owned
 * by Cypress Semiconductor Corporation ("Cypress") and is protected by and
 * subject to worldwide patent protection (United States and foreign), United
 * States copyright laws and international treaty provisions. Therefore, unless
 * otherwise specified in a separate license agreement between you and Cypress,
 * this Software must be treated like any other copyrighted material.
 * Reproduction, modification, translation, compilation, or representation of
 * this Software in any other form (e.g., paper, magnetic, optical, silicon) is
 * prohibited without Cypress's express written permission.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Cypress reserves the right to make changes to the Software without notice.
 * Cypress does not assume any liability arising out of the application or use
 * of Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use as critical components in any products
 * where a malfunction or failure may reasonably be expected to result in
 * significant injury or death ("High Risk Product"). By including Cypress's
 * product in a High Risk Product, the manufacturer of such system or
 * application assumes all risk of such use and in doing so indemnifies Cypress
 * against all liability.
 *
 * Use of this Software may be limited by and subject to the applicable Cypress
 * software license agreement.
 *****************************************************************************/

//#include <stdio.h>
//#include <stdint.h>
//#include <stdlib.h>
//#include <time.h>
//#include <errno.h>

#include "result.h"
#include "version.h"
#include "debug.h"
#include "../cyttsp5_regs.h"
#include "../cyttsp5_core.h"

#include <linux/fs.h>


#define SENSOR_CM_VALIDATION        "Sensor Cm Validation"
#define SELFCAP_CALIBRATION_CHECK    "Self-cap Calibration Check"

#define PASS    "PASS"
#define FAIL    "FAIL"

#define PRINT_RESULT(index ,value, file) \
do {\
    if(value == 0){\
        seq_printf(file, "%dF-", index);\
    }else{\
        seq_printf(file, "%dP-", index);\
    }\
}while(0)

static int print_cm_info(struct seq_file *file, struct configuration *configuration,
        struct result *result)
{
    int ret = 0;
    int i, j;

    if ( !result || ! configuration) {
        ret = -EINVAL;
        tp_log_err("%s, param invalid\n", __func__);
        goto exit;
    }
    /*print cm_sensor_data*/
    if(result->cm_sensor_raw_data) {
        seq_printf(file, "cm_sensor_raw_data:\n");
        for(i = 0 ; i < result->tx_num; i++) {
            for(j = 0 ; j < result->rx_num; j++) {
                seq_printf(file, "%6d", result->cm_sensor_raw_data[i*result->rx_num + j]);
            }
            seq_printf(file, "\n");
        }
    }

    if(result->cm_sensor_data) {
        seq_printf(file, "cm_sensor_data:\n");
        for(i = 0 ; i < result->tx_num; i++) {
            for(j = 0 ; j < result->rx_num; j++) {
                seq_printf(file, "%6d", result->cm_sensor_data[i*result->rx_num + j]);
            }
            seq_printf(file, "\n");
        }
    }

    if(result->cm_gradient_col) {
        seq_printf(file, "cm_gradient_col:\n");
        for (i = 0; i < configuration->cm_gradient_check_col_size; i++)
           seq_printf(file, "%6d", result->cm_gradient_col[i].gradient_val);
        seq_printf(file, "\n");
    }

    if(result->cm_gradient_row) {
        seq_printf(file, "cm_gradient_row:\n");
        for (i = 0; i < configuration->cm_gradient_check_row_size; i++)
           seq_printf(file, "%6d", result->cm_gradient_row[i].gradient_val);
        seq_printf(file, "\n");
    }
exit:
    return ret;
}

static int print_cp_info(struct seq_file *file, struct configuration *configuration,
        struct result *result)
{
    int ret = 0;
    int i;

    if ( !result || ! configuration) {
        ret = -EINVAL;
        tp_log_err("%s, param invalid\n", __func__);
        goto exit;
    }
    /*print cp_sensor_data*/
    if(result->cp_sensor_rx_raw_data) {
        seq_printf(file, "cp_sensor_rx_raw_data:\n");
        for(i = 0 ; i < result->rx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_rx_raw_data[i]);
        seq_printf(file, "\n");
    }

    if(result->cp_sensor_rx_data) {
        seq_printf(file, "cp_sensor_rx_data:\n");
        for(i = 0 ; i < result->rx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_rx_data[i]);
        seq_printf(file, "\n");
    }

    if(result->cp_sensor_tx_raw_data) {
        seq_printf(file, "cp_sensor_tx_raw_data:\n");
        for(i = 0 ; i < result->tx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_tx_raw_data[i]);
        seq_printf(file, "\n");
    }

    if(result->cp_sensor_tx_data) {
        seq_printf(file, "cp_sensor_tx_data:\n");
        for(i = 0 ; i < result->tx_num; i++)
            seq_printf(file, "%6d", result->cp_sensor_tx_data[i]);
        seq_printf(file, "\n");
    }
exit:
    return ret;
}

int result_save(struct seq_file *file, struct configuration *configuration,
        struct result *result)
{
    int ret = 0;
    if(! configuration || !result) {
        tp_log_err("%s, param invalid\n",__func__);
        ret = -EINVAL;
        goto exit;
    }
    //printk("result:");
    seq_printf(file, "result:");
    PRINT_RESULT(0, result->test_summary, file);
    if(result->cm_test_run) {
        PRINT_RESULT(1, result->cm_test_pass, file);
        PRINT_RESULT(2, result->cm_sensor_validation_pass, file);
        PRINT_RESULT(3, result->cm_sensor_col_delta_pass, file);
        PRINT_RESULT(4, result->cm_sensor_row_delta_pass, file );
    }
    if(result->cp_test_run) {
        PRINT_RESULT(5, result->cp_test_pass, file );
        PRINT_RESULT(6, result->cp_rx_validation_pass, file);
        PRINT_RESULT(7, result->cp_tx_validation_pass, file);
    }
    seq_printf(file, "\n");
    seq_printf(file ,"rx_num:%d, tx_num:%d\n", result->rx_num, result->tx_num);
    seq_printf(file ,"button_num:%d, firmware_version:0x%x\n", result->button_num, result->config_ver);

    if(result->cm_test_run) {
        print_cm_info(file, configuration, result);
    }
    if(result->cp_test_run) {
        print_cp_info(file, configuration, result);
    }

exit:
    return ret;
}
