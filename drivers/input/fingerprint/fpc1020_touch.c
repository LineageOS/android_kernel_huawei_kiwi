/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

//#define DEBUG
//#define FPC1020_NAV_DBG
//#define FPC1020_NAV_TOUCH_DBG
//#define FPC1020_NAV_DPAD_DBG
//#define DEBUG_TIME
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/time.h>
#include "fpc1020_common.h"
#include "fpc1020_input.h"
#include "fpc1020_capture.h"
#include "fpc1020_debug.h"
#include "fpc1020_navlib.h"
/* Pointers into huge-buffer for image data */
#ifndef FPC1020_NAV_HEAP_BUF
static u8* p_prev_img = 0;
static u8* p_curr_img = 0;
#endif
/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
#ifdef CONFIG_INPUT_FPC1020_NAV
static int fpc1020_write_nav_setup(fpc1020_data_t* fpc1020);

static int capture_nav_image(fpc1020_data_t* fpc1020);

/* -------------------------------------------------------------------- */
/* driver constants							*/
/* -------------------------------------------------------------------- */
#define FPC1020_KEY_FINGER_PRESENT	KEY_F18	/* 188*/
/* make the val set in fpc1020_navlib_initialize function */
//#define FPC1020_INPUT_POLL_INTERVAL (20000u)

#define FPC1020_INPUT_POLL_INTERVAL	(30000u)
#define FPC1020_CLICK_INTERVAL		(10000u)
#define THRESHOLD_DURATION_CLICK	300/*click threshold*/
#define THRESHOLD_DURATION_HOLD		500/*long press threshold*/
#define THRESHOLD_DURATION_DTAP		500
#define THRESHOLD_DURATION_TAP		1500

#define FLOAT_MAX 100

#define DEVICE_WIDTH 1080
#define DEVICE_HEIGHT 1920

enum
{
    FNGR_ST_NONE = 0,
    FNGR_ST_DETECTED,
    FNGR_ST_LOST,
    FNGR_ST_TAP,
    FNGR_ST_HOLD,
    FNGR_ST_MOVING,
    FNGR_ST_L_HOLD,
    FNGR_ST_DOUBLE_TAP,
};

enum
{
    FPC1020_INPUTMODE_TRACKPAD	= 0,	//trackpad(navi) event report
    FPC1020_INPUTMODE_MOUSE		= 1,	//mouse event report
    FPC1020_INPUTMODE_TOUCH		= 2,	//touch event report
    FPC1020_INPUTMODE_DPAD		= 3,	//dpad event report
};

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */

int fpc1020_capture_nav_wait_finger_down(fpc1020_data_t* fpc1020)
{
    int error = 0;

    error = fpc1020_nav_wait_finger_present(fpc1020);


    fpc1020_read_irq(fpc1020, true);

    return (error >= 0) ? 0 : error;
}

void init_enhanced_navi_setting(fpc1020_data_t* fpc1020)
{
    fpc_log_info("ENTER!\n");

    switch (fpc1020->nav.input_mode)
    {
        case FPC1020_INPUTMODE_TRACKPAD:
            fpc1020->nav.p_sensitivity_key = 160; // 25;
            fpc1020->nav.p_sensitivity_ptr = 180;
            fpc1020->nav.p_multiplier_x = 120; // 75;
            fpc1020->nav.p_multiplier_y = 120; // 95
            fpc1020->nav.multiplier_key_accel = 1;
            fpc1020->nav.multiplier_ptr_accel = 2;
            fpc1020->nav.threshold_key_accel = 70;
            fpc1020->nav.threshold_ptr_accel = 10;
            fpc1020->nav.threshold_ptr_start = 5;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.nav_finger_up_threshold = 3;
            break;
        case FPC1020_INPUTMODE_DPAD:
            fpc1020->nav.p_sensitivity_key = 200;
            fpc1020->nav.p_sensitivity_ptr = 180;
            fpc1020->nav.p_multiplier_x = 400;
            fpc1020->nav.p_multiplier_y = 600;
            fpc1020->nav.multiplier_key_accel = 1;
            fpc1020->nav.multiplier_ptr_accel = 2;
            fpc1020->nav.threshold_key_accel = 40;
            fpc1020->nav.threshold_ptr_accel = 20;
            fpc1020->nav.threshold_ptr_start = 5;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.nav_finger_up_threshold = 1;
            fpc1020->nav.threshold_key_start = 8;
            fpc1020->nav.threshold_key_start_up = 80;
            fpc1020->nav.threshold_key_start_down = 80;
            fpc1020->nav.threshold_key_start_left = 120;
            fpc1020->nav.threshold_key_start_right = 120;
            fpc1020->nav.threshold_key_offset = 8;
            fpc1020->nav.duration_key_clear = 2000;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.threshold_key_dispatch = 4;
            break;
        case FPC1020_INPUTMODE_TOUCH:
            fpc1020->nav.p_sensitivity_key = 160;//180;
            fpc1020->nav.p_sensitivity_ptr = 180;
            fpc1020->nav.p_multiplier_x = 700;//110;
            fpc1020->nav.p_multiplier_y = 2000 ; // 110;
            fpc1020->nav.multiplier_key_accel = 1;// 1;
            fpc1020->nav.multiplier_ptr_accel = 2;
            fpc1020->nav.threshold_key_accel = 40;
            fpc1020->nav.threshold_ptr_accel = 20;
            fpc1020->nav.threshold_ptr_start = 5;
            fpc1020->nav.duration_ptr_clear = 100;
            fpc1020->nav.nav_finger_up_threshold = 1;
            break;
        default:
            break;
    }
}

/*--------------------------------------------------------*/
static void dispatch_dpad_event(fpc1020_data_t* fpc1020, int x, int y, int finger_status)
{
    int abs_x, abs_y;
    int sign_x, sign_y;
    int key_code;
    int reset = 0;
    static int active = 0;
    static int issued_key = 0;
    static unsigned long prev_time = 0;
    unsigned long current_time = jiffies * 1000 / HZ;
    key_code = -1;
    switch (finger_status)
    {
        case FNGR_ST_DETECTED:
            fpc1020->nav.sum_x = 0;
            fpc1020->nav.sum_y = 0;
        break;

        case FNGR_ST_LOST:
            if (fpc1020->nav.move_down_cnt == 1)
             {
                if (fpc1020->nav.tap_status != FNGR_ST_TAP)
                {
                    fpc1020->nav.tap_status = FNGR_ST_TAP;
                    fpc1020->nav.tap_start = current_time;
                }
                else if (fpc1020->nav.tap_status == FNGR_ST_TAP
                && current_time - fpc1020->nav.tap_start <= THRESHOLD_DURATION_DTAP)
                {
                    fpc1020->nav.report_key = KEY_DELETE;
                    queue_work(fpc1020->fpc_nav_workqueue, &fpc1020->fpc_nav_work);
                    fpc1020->nav.tap_status = -1;
                    fpc_log_info("[DPAD] report m_double click\n");
                }
             }
            fpc1020->nav.sum_x = 0;
            fpc1020->nav.sum_y = 0;
            fpc1020->nav.move_up_cnt = 0;
            fpc1020->nav.move_down_cnt = 0;
            fpc1020->nav.throw_event = 0;
            fpc1020->nav.throw_event_move = 0;
            break;

        case  FNGR_ST_TAP:
              if (fpc1020->nav.throw_event == 0){
                  fpc1020->nav.report_key = KEY_EXIT;
                  queue_work(fpc1020->fpc_nav_workqueue, &fpc1020->fpc_nav_work);
                  fpc_log_info("[DPAD] report single click\n");
              }
            break;

        case FNGR_ST_DOUBLE_TAP:
             if (fpc1020->nav.throw_event == 0){
                 fpc1020->nav.report_key = KEY_DELETE;
                 queue_work(fpc1020->fpc_nav_workqueue, &fpc1020->fpc_nav_work);
                 fpc1020->nav.tap_status = -1;
                 fpc_log_info("[DPAD] report double click\n");
             }
            break;

        case FNGR_ST_HOLD:
             if (fpc1020->nav.throw_event_move == 0)
             {
                fpc1020->nav.report_key = KEY_ENTER;
                queue_work(fpc1020->fpc_nav_workqueue, &fpc1020->fpc_nav_work);
                mdelay(30);
                fpc1020->nav.throw_event = 1;
                fpc_log_info("[DPAD] report long press\n");
              }
            fpc1020->nav.tap_status = -1;
            break;

        case FNGR_ST_MOVING:
            fpc_log_nav("[DPAD] raw axis x=%d, y=%d\n", x, y);
            // Handle Acceleration
            sign_x = x > 0 ? 1 : -1;
            sign_y = y > 0 ? 1 : -1;
            abs_x = x * sign_x;
            abs_y = y * sign_y;
            if (prev_time != 0 && (current_time - prev_time > fpc1020->nav.duration_key_clear))
            {
                reset = 1;
            }
            prev_time = current_time;
            //spin_lock_irqsave(&easy_wake_guesure_lock, irq_flags);
            abs_x = x > 0 ? x : -x;
            abs_y = y > 0 ? y : -y;
            if (abs_x > fpc1020->nav.threshold_key_accel)
                x = (fpc1020->nav.threshold_key_accel
                     + (abs_x - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_x;
            if (abs_y > fpc1020->nav.threshold_key_accel)
                y = (fpc1020->nav.threshold_key_accel
                     + (abs_y - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_y;
            // Correct axis factor
            x = x * fpc1020->nav.p_multiplier_x / FLOAT_MAX;
            y = y * fpc1020->nav.p_multiplier_y / FLOAT_MAX;
            // Adjust Sensitivity
            fpc_log_nav("[DPAD] multiplied axis x=%d, y=%d\n", x, y);
            fpc1020->nav.sum_x += x;
            fpc1020->nav.sum_y += y;
            if (abs_x > abs_y)
            {
                //fpc1020->nav.sum_x += x;
                if (issued_key)
                {
                    if (x < 0 && issued_key != KEY_LEFT)
                    {
                        reset = 1;
                    }
                    if ( x > 0 && issued_key != KEY_RIGHT )
                    {
                        reset = 1;
                    }
                }
            }
            else
            {
                //fpc1020->nav.sum_y += y;
                if (issued_key)
                {
                    if (y < 0 && issued_key != KEY_UP)
                    {
                        reset = 1;
                    }
                    if (y > 0 && issued_key != KEY_DOWN)
                    {
                        reset = 1;
                    }
                }
            }
            if (reset)
            {
                //fpc1020->nav.sum_x = fpc1020->nav.sum_y = 0;
                active = 0;
                issued_key = 0;
            }
            fpc_log_nav("[DPAD] active=%d, sum_x=%d, sum_y=%d, key_start=%d, key_offset=%d\n",
                    active, fpc1020->nav.sum_x, fpc1020->nav.sum_y, fpc1020->nav.threshold_key_start, fpc1020->nav.threshold_key_offset);
            //if (!active) {
            if(fpc1020->nav.throw_event == 0)
            {
                if (abs_x > abs_y && fpc1020->nav.sum_x > fpc1020->nav.threshold_key_start_right
                    && (fpc1020->nav.sum_y < fpc1020->nav.threshold_key_offset
                        && fpc1020->nav.sum_y > -fpc1020->nav.threshold_key_offset))
                {
                    //key_code = KEY_RIGHT;
                    fpc1020->nav.sum_x -= fpc1020->nav.threshold_key_start;
                    fpc1020->nav.sum_y = 0;
                }
                else if (abs_x > abs_y && fpc1020->nav.sum_x < -fpc1020->nav.threshold_key_start_left
                         && (fpc1020->nav.sum_y < fpc1020->nav.threshold_key_offset
                             && fpc1020->nav.sum_y > -fpc1020->nav.threshold_key_offset))
                {
                    //key_code = KEY_LEFT;
                    fpc1020->nav.sum_x += fpc1020->nav.threshold_key_start;
                    fpc1020->nav.sum_y = 0;
                }
                else if (abs_x < abs_y && fpc1020->nav.sum_y < -fpc1020->nav.threshold_key_start_up)
                {
                    fpc1020->nav.move_up_cnt++;
                    key_code = KEY_UP;
                    //fpc1020->nav.sum_y += fpc1020->nav.threshold_key_start;
                    fpc1020->nav.sum_x = 0;
                }
                else if (abs_x < abs_y && fpc1020->nav.sum_y > fpc1020->nav.threshold_key_start_down)
                {
                    fpc1020->nav.move_down_cnt++;
                    key_code = KEY_DOWN;
                    //fpc1020->nav.sum_y -= fpc1020->nav.threshold_key_start;
                    fpc1020->nav.sum_x = 0;
                }
                else
                {
                    key_code = 0;
                }
                if (key_code)
                {
                    active = 1;
                    {
                        if (fpc1020->nav.move_up_cnt > 0 && key_code == KEY_UP)
                        {
                            fpc1020->nav.report_key = KEY_UP;
                            queue_work(fpc1020->fpc_nav_workqueue, &fpc1020->fpc_nav_work);
                            fpc1020->nav.throw_event = 1;
                            fpc1020->nav.throw_event_move = 1;
                            fpc_log_info("[DPAD] report key up\n");
                        }
                        if (fpc1020->nav.move_down_cnt > 1 && key_code == KEY_DOWN)
                        {
                            fpc1020->nav.report_key = KEY_DOWN;
                            queue_work(fpc1020->fpc_nav_workqueue, &fpc1020->fpc_nav_work);
                            fpc1020->nav.throw_event = 1;
                            fpc1020->nav.throw_event_move = 1;
                            fpc_log_info("[DPAD] report key down\n");
                        }
                        switch (key_code)
                        {
                            case KEY_UP:
                                fpc_log_nav("[DPAD] KEY UP !\n");
                                break;
                            case KEY_DOWN:
                                fpc_log_nav("[DPAD] KEY DOWN !\n");
                                break;
                            case KEY_LEFT:
                                fpc_log_nav("[DPAD] KEY LEFT !\n");
                                break;
                            case KEY_RIGHT:
                                fpc_log_nav("[DPAD] KEY RIGHT !\n");
                                break;
                            default:
                                break;
                        }
                    }
                    issued_key = key_code;
                }
                else
                {
                    return;
                }
            }
    }
}

static void dispatch_trackpad_event(fpc1020_data_t* fpc1020, int x, int y, int finger_status)
{
    int abs_x, abs_y;
    int sign_x, sign_y;

    if (finger_status == FNGR_ST_TAP)
    {
        input_report_key(fpc1020->input_dev, KEY_ENTER, 1);
        input_sync(fpc1020->input_dev);
        input_report_key(fpc1020->input_dev, KEY_ENTER, 0);
        input_sync(fpc1020->input_dev);
        return;
    }

    sign_x = x > 0 ? 1 : -1;
    sign_y = y > 0 ? 1 : -1;
    abs_x = x * sign_x;
    abs_y = y * sign_y;

    abs_x = x > 0 ? x : -x;
    abs_y = y > 0 ? y : -y;

    if (abs_x > fpc1020->nav.threshold_key_accel)
        x = (fpc1020->nav.threshold_key_accel
             + (abs_x - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_x;
    if (abs_y > fpc1020->nav.threshold_key_accel)
        y = (fpc1020->nav.threshold_key_accel
             + (abs_y - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_y;

    // Correct axis factor
    x = x * fpc1020->nav.p_multiplier_x / FLOAT_MAX;
    y = y * fpc1020->nav.p_multiplier_y / FLOAT_MAX;

    // Adjust Sensitivity
    x = x * fpc1020->nav.p_sensitivity_key / FLOAT_MAX;
    y = y * fpc1020->nav.p_sensitivity_key / FLOAT_MAX;

    input_report_rel(fpc1020->input_dev, REL_X, x);
    input_report_rel(fpc1020->input_dev, REL_Y, y);

    input_sync(fpc1020->input_dev);
}


/* -------------------------------------------------------------------- */
static void dispatch_touch_event(fpc1020_data_t* fpc1020, int x, int y, int finger_status)
{
    int sign_x, sign_y;
    int abs_x, abs_y;

    switch (finger_status)
    {
        case FNGR_ST_DETECTED:
        case FNGR_ST_TAP:
            break;

        case FNGR_ST_LOST:
            fpc_log_nav("[TOUCH] finger lost, nav_sum_x=%d, nav_sum_y=%d, filer=%d\n",
                    fpc1020->nav.nav_sum_x, fpc1020->nav.nav_sum_y, fpc1020->nav.throw_event);

            if (fpc1020->nav.nav_sum_y > 17)
            { fpc1020->nav.move_direction = SLIDE_DOWN; }
            else if (fpc1020->nav.nav_sum_y < -1)
            { fpc1020->nav.move_direction = SLIDE_UP; }
            if (fpc1020->nav.throw_event != 1)
            {
                if (fpc1020->nav.move_direction == SLIDE_DOWN)/*move down*/
                {
                    input_report_key(fpc1020->touch_pad_dev, KEY_DOWN, 1);
                    input_sync(fpc1020->touch_pad_dev);
                    input_report_key(fpc1020->touch_pad_dev, KEY_DOWN, 0);
                    input_sync(fpc1020->touch_pad_dev);
                    fpc1020->nav.filter_key = 1;
                    fpc_log_info("[TOUCH] report key down\n");
                }
                else if (fpc1020->nav.move_direction == SLIDE_UP)
                {
                    input_report_key(fpc1020->touch_pad_dev, KEY_UP, 1);
                    input_sync(fpc1020->touch_pad_dev);
                    input_report_key(fpc1020->touch_pad_dev, KEY_UP, 0);
                    input_sync(fpc1020->touch_pad_dev);
                    fpc1020->nav.filter_key = 1;
                    fpc_log_info("[TOUCH] report key up\n");
                }
            }
            fpc1020->nav.throw_event = 0;
            fpc1020->nav.move_direction = 0; /*1: slide down, 1: slide up, 0: init*/
            fpc1020->nav.move_up_cnt = 0;
            fpc1020->nav.move_down_cnt = 0;
            fpc1020->nav.nav_sum_y = 0;
            fpc1020->nav.nav_sum_x = 0;
            break;
        case FNGR_ST_DOUBLE_TAP:
            input_report_key(fpc1020->touch_pad_dev, KEY_DELETE, 1);
            input_sync(fpc1020->touch_pad_dev);
            input_report_key(fpc1020->touch_pad_dev, KEY_DELETE, 0);
            input_sync(fpc1020->touch_pad_dev);
            fpc1020->nav.throw_event = 1;
            mdelay(50);
            fpc_log_info("[TOUCH] report double click\n");
            break;

        case FNGR_ST_HOLD:
            input_report_key(fpc1020->touch_pad_dev, KEY_ENTER, 1);
            input_sync(fpc1020->touch_pad_dev);
            input_report_key(fpc1020->touch_pad_dev, KEY_ENTER, 0);
            input_sync(fpc1020->touch_pad_dev);
            fpc1020->nav.throw_event = 1;
            mdelay(50);
            fpc_log_info("[TOUCH] report long press\n");
            break;

        case FNGR_ST_MOVING:
            sign_x = x > 0 ? 1 : -1;
            sign_y = y > 0 ? 1 : -1; //reverse direction
            abs_x = x > 0 ? x : -x;
            abs_y = y > 0 ? y : -y;
            if (abs_x > fpc1020->nav.threshold_key_accel)
                x = (fpc1020->nav.threshold_key_accel
                     + ( abs_x - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_x;
            if (abs_y > fpc1020->nav.threshold_key_accel)
                y = (fpc1020->nav.threshold_key_accel
                     + (abs_y - fpc1020->nav.threshold_key_accel) * fpc1020->nav.multiplier_key_accel) * sign_y;
            x = x * fpc1020->nav.p_multiplier_x / FLOAT_MAX;
            y = y * fpc1020->nav.p_multiplier_y / FLOAT_MAX;
            x = x * fpc1020->nav.p_sensitivity_key / FLOAT_MAX;
            y = y * fpc1020->nav.p_sensitivity_key / FLOAT_MAX;

            fpc_log_nav("[TOUCH] touch moving: x=%d y=%d sum_x=%d sum_y=%d\n",
                    x, y, fpc1020->nav.nav_sum_x, fpc1020->nav.nav_sum_y);

            fpc1020->nav.nav_sum_x += x;
            fpc1020->nav.nav_sum_y += y;
            break;

        default:
            fpc_log_info("[TOUCH] undefined gesture events\n");
            break;
    }
}


/* -------------------------------------------------------------------- */
static void process_navi_event(fpc1020_data_t* fpc1020, int dx, int dy, int finger_status)
{
    const int THRESHOLD_RANGE_TAP = 500000;
    const int THRESHOLD_RANGE_MIN_TAP = 4;
    const int THRESHOLD_RANGE_DOUBLE_TAP = 1600;
    //const unsigned long THRESHOLD_DURATION_TAP = 3000;//350;
    int filtered_finger_status = finger_status;
    static int deviation_x = 0;
    static int deviation_y = 0;
    int deviation;
    static unsigned long tick_down = 0;
    unsigned long tick_curr = jiffies * 1000 / HZ;
    unsigned long duration = 0;

    if (finger_status == FNGR_ST_DETECTED)
    {
        tick_down = tick_curr;
    }

    if (tick_down > 0)
    {
        duration = tick_curr - tick_down;
        /*workaround to thraw the events*/
        if (duration<=70&&finger_status!=FNGR_ST_LOST&&dx!=0&&dy!=0) {
            dx = 0;
            dy = 0;
        }
        /*workaround end*/
        deviation_x += dx;
        deviation_y += dy;
        deviation = deviation_x * deviation_x + deviation_y * deviation_y;
        fpc_log_nav("deviation=%d, duration=%lu, tap_status=%d, throw_event=%d, tick_consume=%lu\n",
            deviation, duration, fpc1020->nav.tap_status, fpc1020->nav.throw_event, tick_curr - fpc1020->nav.tap_start);
        if (deviation > THRESHOLD_RANGE_TAP)
        {
            deviation_x = 0;
            deviation_y = 0;
            tick_down = 0;
            fpc1020->nav.tap_status = -1;
            fpc_log_nav("throw the events\n");
            if (duration > THRESHOLD_DURATION_TAP)
            {
                filtered_finger_status = FNGR_ST_HOLD;// FNGR_ST_L_HOLD;
                fpc_log_nav("prepare long press because of outside\n");
            }
        }
        else
        {
            if (duration < THRESHOLD_DURATION_TAP)
            {
                if (finger_status == FNGR_ST_LOST && fpc1020->nav.detect_zones > 1)
                {
                    if (fpc1020->nav.tap_status == FNGR_ST_TAP
                        && tick_curr - fpc1020->nav.tap_start <= THRESHOLD_DURATION_DTAP
                        && ((deviation <= THRESHOLD_RANGE_DOUBLE_TAP )||((dy == fpc1020->nav.move_pre_y)&& (dx==fpc1020->nav.move_pre_x))))
                    {
                        fpc1020->nav.tap_status = FNGR_ST_DOUBLE_TAP;
                        filtered_finger_status = FNGR_ST_DOUBLE_TAP;
                        //fpc1020->nav.detect_zones = 0;
                        fpc_log_nav("prepare report double click\n");
                    }
                    else if ((deviation <= THRESHOLD_RANGE_DOUBLE_TAP || ((dy == fpc1020->nav.move_pre_y)&& (dx==fpc1020->nav.move_pre_x)))&&
                        duration < THRESHOLD_DURATION_CLICK)

                    {
                        filtered_finger_status = FNGR_ST_TAP;
                        fpc1020->nav.tap_status = FNGR_ST_TAP;
                        fpc1020->nav.tap_start = tick_curr;
                        fpc_log_nav("prepare report single click\n");
                    }
                    else
                    {
                        filtered_finger_status = FNGR_ST_LOST;
                        fpc1020->nav.throw_event = 0;
                        fpc_log_nav("still report finger lost\n");
                    }
                    //tick_down = 0;
                    deviation_x = 0;
                    deviation_y = 0;
                }
                else if ((deviation <= THRESHOLD_RANGE_MIN_TAP ||
                   ((dy == fpc1020->nav.move_pre_y)&& (dx==fpc1020->nav.move_pre_x)))
                && duration > THRESHOLD_DURATION_HOLD)
               {
                //if (deviation < THRESHOLD_RANGE_MIN_TAP)
                filtered_finger_status = FNGR_ST_HOLD;// FNGR_ST_L_HOLD;
                fpc1020->nav.tap_status = -1;
                tick_down = 0;
                deviation_x = 0;
                deviation_y = 0;
                fpc_log_nav("prepare report long press\n");
               }
            }
            else if ((deviation <= THRESHOLD_RANGE_MIN_TAP ||
                ((dy == fpc1020->nav.move_pre_y)&& (dx==fpc1020->nav.move_pre_x)))
            && duration > THRESHOLD_DURATION_HOLD)
            {
                //if (deviation < THRESHOLD_RANGE_MIN_TAP)
                filtered_finger_status = FNGR_ST_HOLD;// FNGR_ST_L_HOLD;
                fpc1020->nav.tap_status = -1;
                tick_down = 0;
                deviation_x = 0;
                deviation_y = 0;
                fpc_log_nav("prepare report long press\n");
            }
            fpc1020->nav.move_pre_x = dx;
            fpc1020->nav.move_pre_y = dy;
        }

    }

    switch (fpc1020->nav.input_mode)
    {
        case FPC1020_INPUTMODE_TRACKPAD :
            dispatch_trackpad_event(fpc1020, dy, dx, filtered_finger_status);
            break;
        case FPC1020_INPUTMODE_DPAD:
            dispatch_dpad_event(fpc1020, dy, dx, filtered_finger_status);
            break;
        case FPC1020_INPUTMODE_TOUCH:
            dispatch_touch_event(fpc1020, dy, dx, filtered_finger_status);
            break;
        default:
            fpc_log_info("undefined input mode\n");
            break;
    }
}
#ifdef FPC1020_NAV_WQ
static void fpc_nav_work_func(struct work_struct *work)
{
        fpc1020_data_t *fpc1020 =container_of(work, fpc1020_data_t, fpc_nav_work);
        input_report_key(fpc1020->touch_pad_dev, fpc1020->nav.report_key, 1);
        input_sync(fpc1020->touch_pad_dev);
        input_report_key(fpc1020->touch_pad_dev, fpc1020->nav.report_key, 0);
        input_sync(fpc1020->touch_pad_dev);
}
#endif

/* -------------------------------------------------------------------- */
int  fpc1020_input_init(fpc1020_data_t* fpc1020)
{
    int error = 0;
    nav_setup_t* p_nav_setup = NULL;

    fpc_log_info("ENTER!\n");

    if ((fpc1020->chip.type != FPC1020_CHIP_1020A)
        && (fpc1020->chip.type != FPC1020_CHIP_1021A)
        && (fpc1020->chip.type != FPC1020_CHIP_1021B)
        && (fpc1020->chip.type != FPC1020_CHIP_1021F)
        && (fpc1020->chip.type != FPC1020_CHIP_1150A)
        && (fpc1020->chip.type != FPC1020_CHIP_1140B)
        && (fpc1020->chip.type != FPC1020_CHIP_1140C))
        {
        fpc_log_err("chip not supported (%s)\n", fpc1020_hw_id_text(fpc1020));
        return -EINVAL;
    }

    fpc1020->input_dev = input_allocate_device();

    if (!fpc1020->input_dev)
    {
        fpc_log_err("[TOUCH] input_allocate_device failed\n");
        error  = -ENOMEM;
    }

    if (!error)
    {
        fpc1020->input_dev->name = FPC1020_TOUCH_PAD_DEV_NAME;

        /* Set event bits according to what events we are generating */
        set_bit(EV_KEY, fpc1020->input_dev->evbit);
        set_bit(EV_REL, fpc1020->input_dev->evbit);
        input_set_capability(fpc1020->input_dev, EV_REL, REL_X);
        input_set_capability(fpc1020->input_dev, EV_REL, REL_Y);
        //input_set_capability(fpc1020->input_dev, EV_KEY, BTN_MOUSE);
        input_set_capability(fpc1020->input_dev, EV_KEY, KEY_ENTER);
#if defined (SUPPORT_DOUBLE_TAP)
        input_set_capability(fpc1020->input_dev, EV_KEY, KEY_VOLUMEUP);
        input_set_capability(fpc1020->input_dev, EV_KEY, KEY_VOLUMEDOWN);
#endif
        input_set_capability(fpc1020->input_dev, EV_KEY, KEY_DELETE);
        input_set_capability(fpc1020->input_dev, EV_KEY, KEY_SLEEP);

        //set_bit(FPC1020_KEY_FINGER_PRESENT, fpc1020->input_dev->keybit);

        /* Register the input device */
        error = input_register_device(fpc1020->input_dev);

        if (error)
        {
            fpc_log_err("[TOUCH] input_register_device failed\n");
            input_free_device(fpc1020->input_dev);
            fpc1020->input_dev = NULL;
        }
    }

    fpc1020->touch_pad_dev = input_allocate_device();

    if (!fpc1020->touch_pad_dev)
    {
        fpc_log_err("[DPAD] input_allocate_device failed\n");
        error  = -ENOMEM;
    }

    if (!error)
    {
        fpc1020->touch_pad_dev->name = FPC1020_INPUT_NAME;

        /* Set event bits according to what events we are generating */
        set_bit(EV_KEY, fpc1020->touch_pad_dev->evbit);
        set_bit(EV_ABS, fpc1020->touch_pad_dev->evbit);
        set_bit(BTN_TOUCH, fpc1020->touch_pad_dev->keybit);
        set_bit(ABS_X, fpc1020->touch_pad_dev->absbit);
        set_bit(ABS_Y, fpc1020->touch_pad_dev->absbit);
        set_bit(ABS_Z, fpc1020->touch_pad_dev->absbit);
        /*dpad*/
        /*double tap demo*/
        set_bit(KEY_UP, fpc1020->touch_pad_dev->keybit);
        set_bit(KEY_RIGHT, fpc1020->touch_pad_dev->keybit);
        set_bit(KEY_LEFT, fpc1020->touch_pad_dev->keybit);
        set_bit(KEY_DOWN, fpc1020->touch_pad_dev->keybit);
        set_bit(KEY_BACK, fpc1020->touch_pad_dev->keybit);
        set_bit(KEY_EXIT, fpc1020->touch_pad_dev->keybit);
        set_bit(KEY_DELETE, fpc1020->touch_pad_dev->keybit);
        input_set_capability(fpc1020->touch_pad_dev, EV_KEY, KEY_ENTER);
        input_set_capability(fpc1020->touch_pad_dev, EV_KEY, KEY_BACK);
        input_set_capability(fpc1020->touch_pad_dev, EV_KEY, KEY_EXIT);
        input_set_capability(fpc1020->touch_pad_dev, EV_KEY, KEY_DELETE);
        input_set_abs_params(fpc1020->touch_pad_dev, ABS_X, 0, DEVICE_WIDTH, 0, 0);
        input_set_abs_params(fpc1020->touch_pad_dev, ABS_Y, 0, DEVICE_HEIGHT, 0, 0);

        /* Register the input device */
        error = input_register_device(fpc1020->touch_pad_dev);

        if (error)
        {
            fpc_log_err("[DPAD] input_register_device failed\n");
            input_free_device(fpc1020->touch_pad_dev);
            fpc1020->touch_pad_dev = NULL;
        }
    }

    error = fpc1020_navlib_initialize(fpc1020->chip.type);
    if(error)
    {
        input_free_device(fpc1020->input_dev);
        input_free_device(fpc1020->touch_pad_dev);
        fpc1020->input_dev = NULL;
        fpc1020->touch_pad_dev = NULL;
        fpc_log_err("fpc1020_navlib_initialize failed\n");
        return -EINVAL;
    }

    p_nav_setup = fpc1020_navlib_get_nav_setup();

#ifndef FPC1020_NAV_HEAP_BUF
    if (!error)
    {
        if (get_nav_img_capture_size() * 2 > fpc1020->huge_buffer_size)
        {
            fpc_log_err("Huge buffer not large enough for navigation\n");
            error = -ENOMEM;
        }
        else
        {
            p_prev_img = fpc1020->huge_buffer;
            p_curr_img = fpc1020->huge_buffer + get_nav_img_capture_size();
        }
    }
#endif
    if (!error)
    {
        /* sub area setup */
        fpc1020->nav.image_nav_row_start = ((fpc1020->chip.pixel_rows - p_nav_setup->height) / 2);
        fpc1020->nav.image_nav_row_count = p_nav_setup->height;
        fpc1020->nav.image_nav_col_start = ((fpc1020->chip.pixel_columns - p_nav_setup->width) / 2) / fpc1020->chip.adc_group_size;
        fpc1020->nav.image_nav_col_groups = (p_nav_setup->width + fpc1020->chip.adc_group_size - 1) / fpc1020->chip.adc_group_size;

        fpc1020->nav.input_mode = FPC1020_INPUTMODE_DPAD;
        //fpc1020->nav.input_mode = FPC1020_INPUTMODE_TOUCH;
        init_enhanced_navi_setting(fpc1020);
        fpc1020->nav.throw_event = 0;
    }
#ifdef FPC1020_NAV_WQ
        fpc1020->fpc_nav_workqueue = create_singlethread_workqueue("fpc_nav_wq");
        INIT_WORK(&fpc1020->fpc_nav_work, fpc_nav_work_func);
#endif
    return error;
}


/* -------------------------------------------------------------------- */
void  fpc1020_input_destroy(fpc1020_data_t* fpc1020)
{
    fpc_log_info("ENTER!\n");

    if (fpc1020->input_dev != NULL)
    { input_free_device(fpc1020->input_dev); }
    if (fpc1020->touch_pad_dev != NULL)
    { input_free_device(fpc1020->touch_pad_dev); }
}

/* -------------------------------------------------------------------- */
void fpc1020_input_enable(fpc1020_data_t* fpc1020, bool enabled)
{

    fpc1020->nav.enabled = enabled;

    return ;
}

int fpc1020_subzrea_zones_sum(int zones)
{
    u8 count = 0;
    u16 mask = FPC1020_FINGER_DETECT_ZONE_MASK;
    if (zones < 0)
        return zones;
    else {
        zones &= mask;
        while (zones && mask) {
            count += (zones & 1) ? 1 : 0;
            zones >>= 1;
            mask >>= 1;
        }
    }
    return (int)count;
}
/* -------------------------------------------------------------------- */
int fpc1020_input_task(fpc1020_data_t* fpc1020)
{
    u8 zones;
    u32 abs_x;
    u32 abs_y;
    ktime_t capture_start_time;
    bool isReverse = false;
    int dx = 0;
    int dy = 0;
    int sumX = 0;
    int sumY = 0;
    int error = 0;
    unsigned char* prevBuffer = NULL;
    unsigned char* curBuffer = NULL;
    unsigned long diffTime = 0;
    unsigned long click_diffTime = 0;
    nav_setup_t* p_nav_setup = NULL;

    fpc_log_info("ENTER!\n");
    p_nav_setup = fpc1020_navlib_get_nav_setup();
    error = fpc1020_write_nav_setup(fpc1020);
    if (fpc1020->diag.result && (fpc1020->finger_status == FPC1020_FINGER_DOWN))
    {
        error = fpc1020_capture_wait_finger_up(fpc1020);
        if (0 == error)
            fpc1020->finger_status = FPC1020_FINGER_UP;
        fpc_log_info("finger_status = %d, error = %d\n", fpc1020->finger_status, error);
    }
    while (!fpc1020->worker.stop_request &&
           fpc1020->nav.enabled && (error >= 0))
    {

        error = fpc1020_capture_nav_wait_finger_down(fpc1020);
        if (error < 0)
        { break; }
        error = fpc1020_write_nav_setup(fpc1020);
        if (error < 0)
        { break; }
        error = fpc1020_check_finger_present_raw(fpc1020);
        process_navi_event(fpc1020, 0, 0, FNGR_ST_DETECTED);
        fpc1020->nav.click_start = jiffies;
        fpc1020->down = true;
        zones = fpc1020_subzrea_zones_sum(error);
        fpc1020->nav.detect_zones = zones;
        fpc_log_nav("report touch zones = %d\n", fpc1020->nav.detect_zones);
        capture_start_time = ktime_get();
#ifdef FPC1020_NAV_HEAP_BUF
        error = capture_nav_image(fpc1020);
#else
        error = fpc1020_capture_buffer(fpc1020, p_prev_img, 0, get_nav_img_capture_size());
#endif /*FPC1020_NAV_HEAP_BUF*/

        fpc_log_nav("[TIME] init capture time: %lld ns\n",
                (ktime_get().tv64 - capture_start_time.tv64));

        if (error < 0)
        { break; }
#ifdef FPC1020_NAV_HEAP_BUF
        memcpy(fpc1020->prev_img_buf,
               fpc1020->huge_buffer, get_nav_img_capture_size());
#endif

        while (!fpc1020->worker.stop_request && (error >= 0))
        {
            error = fpc1020_check_finger_present_sum(fpc1020);
            if (error < fpc1020->setup.capture_finger_up_threshold + 1)
            {
                fpc_log_nav("prepare report finger up\n");

#if 1
                click_diffTime = abs(jiffies - fpc1020->nav.click_start);
                click_diffTime = click_diffTime * 1000000 / HZ;
                if (click_diffTime <= FPC1020_CLICK_INTERVAL&&fpc1020->nav.detect_zones == 0)
                {
                    if (fpc1020->nav.tap_status == FNGR_ST_TAP &&
                    ((jiffies * 1000 / HZ)-fpc1020->nav.tap_start <= THRESHOLD_DURATION_DTAP))
                    {
                        fpc_log_nav("prepare fast report#1 double click\n");
                        process_navi_event(fpc1020, 0, 0, FNGR_ST_DOUBLE_TAP);
                    }
                    else
                    {
                        fpc_log_nav("prepare fast report#1 single click\n");
                        fpc1020->nav.tap_status = FNGR_ST_TAP;
                        fpc1020->nav.tap_start = jiffies * 1000 / HZ;
                    }
                }
#endif
                process_navi_event(fpc1020, 0, 0, FNGR_ST_LOST);
                fpc1020->down = false;
                fpc1020->nav.throw_event = 0;
                fpc1020->nav.report_keys = 0;
                sumX = 0;
                sumY = 0;
                fpc1020->nav.time = 0;
                isReverse = false;
                break;
            }
#ifdef FPC1020_NAV_HEAP_BUF
            if (isReverse)
            {
                prevBuffer = fpc1020->cur_img_buf;
                curBuffer = fpc1020->prev_img_buf;
            }
            else
            {
                prevBuffer = fpc1020->prev_img_buf;
                curBuffer = fpc1020->cur_img_buf;
            }
#else
            u8* p_tmp_curr_img = p_curr_img;
            p_curr_img = p_prev_img;
            p_prev_img = p_tmp_curr_img;
#endif
            capture_start_time = ktime_get();
#ifdef FPC1020_NAV_HEAP_BUF
            error = capture_nav_image(fpc1020);
#else
            error = fpc1020_capture_buffer(fpc1020, p_curr_img, 0, get_nav_img_capture_size());
#endif
            fpc_log_nav("[TIME] capture time: %lld ns\n",
                    (ktime_get().tv64 - capture_start_time.tv64));
            if (error < 0)
            { break; }
#ifdef FPC1020_NAV_HEAP_BUF
            memcpy(curBuffer,
                   fpc1020->huge_buffer,
                   get_nav_img_capture_size());
#endif

            error = fpc1020_check_finger_present_sum(fpc1020);
            if (error < fpc1020->setup.capture_finger_up_threshold + 1)
            {
                fpc_log_nav("prepare report finger up\n");

#if 1
                click_diffTime = abs(jiffies - fpc1020->nav.click_start);
                click_diffTime = click_diffTime * 1000000 / HZ;
                if (click_diffTime <= FPC1020_CLICK_INTERVAL && fpc1020->nav.detect_zones == 0)
                {
                    if (fpc1020->nav.tap_status == FNGR_ST_TAP &&
                    ((jiffies * 1000 / HZ)-fpc1020->nav.tap_start <= THRESHOLD_DURATION_DTAP))
                    {
                        fpc_log_nav("prepare fast report#2 double click\n");
                        process_navi_event(fpc1020, 0, 0, FNGR_ST_DOUBLE_TAP);
                    }
                    else
                    {
                        fpc_log_nav("prepare fast report#2 single click\n");
                        fpc1020->nav.tap_status = FNGR_ST_TAP;
                        fpc1020->nav.tap_start = jiffies * 1000 / HZ;
                        }
                    }
#endif
                process_navi_event(fpc1020, 0, 0, FNGR_ST_LOST);
                fpc1020->down = false;
                fpc1020->nav.throw_event = 0;
                fpc1020->nav.report_keys = 0;
                sumX = 0;
                sumY = 0;
                fpc1020->nav.time = 0;
                isReverse = false;
                break;
            }
#ifdef FPC1020_NAV_HEAP_BUF
            isReverse = !isReverse;
#endif
            capture_start_time = ktime_get();
#ifdef FPC1020_NAV_HEAP_BUF
            get_movement(prevBuffer, curBuffer, &dx, &dy);
#else
            get_movement(p_prev_img, p_curr_img, &dx, &dy);
#endif
            fpc_log_nav("[TIME] get_movement time: %lld ns\n",
                    (ktime_get().tv64 - capture_start_time.tv64));
            /*workaround to filter data*/
            abs_x = dx > 0 ? dx : -dx;
            abs_y = dy > 0 ? dy : -dy;
            if(abs_x >=28 || abs_y>=28)
            {
                dx = 0;
                dy = 0;
            }
            sumX += dx;
            sumY += dy;
            fpc_log_nav("get_movement dx=%d, dy=%d, sumX=%d, sumY=%d\n", dx, dy, sumX, sumY);
            diffTime = abs(jiffies - fpc1020->nav.time);
            if (diffTime > 0)
            {
                diffTime = diffTime * 1000000 / HZ;
                if (diffTime >= p_nav_setup->time)
                {
                    fpc_log_nav("move poll\n");
                    process_navi_event(fpc1020, sumX, sumY, FNGR_ST_MOVING);
                    sumX = 0;
                    sumY = 0;
                    fpc1020->nav.time = jiffies;
                }
            }

        }
    }
    fpc_log_debug("stop_request = %d, nav.enabled = %d, error = %d\n",
        fpc1020->worker.stop_request, fpc1020->nav.enabled, error);

    if (error < 0)
    {
        fpc_log_err("%s error = %d\n",
            (error == -EINTR) ? "Interrupted!" : "FAILED!", error);
    }
    if (fp_CAPTURE != atomic_read(&fpc1020->taskstate))
    {
        atomic_set(&fpc1020->taskstate, fp_UNINIT);
        fpc_log_info("fpc1020_input_task set taskstate=%d\n",atomic_read(&fpc1020->taskstate));
    }
    fpc1020->nav.enabled = false;
    return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_write_nav_setup(fpc1020_data_t* fpc1020)
{
    //const int mux = 2;
    int error = 0;
    u16 temp_u16;
    u32 temp_u32;
    u8 temp_u8;
    fpc1020_reg_access_t reg;
    nav_setup_t* p_nav_setup = NULL;

    fpc_log_debug("ENTER!\n");
    p_nav_setup = fpc1020_navlib_get_nav_setup();
    error = fpc1020_wake_up(fpc1020);
    if (error)
    { goto out; }

    error = fpc1020_write_sensor_setup(fpc1020);
    if (error)
    { goto out; }
#if 0
    temp_u16 = fpc1020->setup.adc_shift[mux];
    temp_u16 <<= 8;
    temp_u16 |= fpc1020->setup.adc_gain[mux];
#else
    temp_u16 = 8;
    temp_u16 <<= 8;
    temp_u16 |= 5;
#endif
    FPC1020_MK_REG_WRITE(reg, FPC102X_REG_ADC_SHIFT_GAIN, &temp_u16);
    error = fpc1020_reg_access(fpc1020, &reg);
    if (error)
    { goto out; }

    //temp_u16 = fpc1020->setup.pxl_ctrl[mux];
    temp_u16 = 0x0f1f;
    FPC1020_MK_REG_WRITE(reg, FPC102X_REG_PXL_CTRL, &temp_u16);
    error = fpc1020_reg_access(fpc1020, &reg);
    if (error)
    { goto out; }
    error = fpc1020_capture_set_crop(
                        fpc1020,
                        p_nav_setup->crop_x / fpc1020->chip.adc_group_size,
                        p_nav_setup->crop_w / fpc1020->chip.adc_group_size,
                        p_nav_setup->crop_y,
                        p_nav_setup->crop_h);
    if (error)
    { goto out; }

    // Setup skipping of rows
    switch (p_nav_setup->row_skip)
    {
        case 1:
            temp_u32 = 0;
            break;
        case 2:
            temp_u32 = 1;
            break;
        case 4:
            temp_u32 = 2;
            break;
        case 8:
            temp_u32 = 3;
            break;
        default:
            error = -EINVAL;
            break;
    }
    if (error) {
        fpc_log_err("row_skip is invaild, error = %d\n", error);
        goto out;
    }

    temp_u32 <<= 8;

    // Setup masking of columns
    switch (p_nav_setup->col_mask)
    {
        case 1:
            temp_u32 |= 0xff;
            break;
        case 2:
            temp_u32 |= 0xcc;
            break;
        case 4:
            temp_u32 |= 0x88;
            break;
        case 8:
            temp_u32 |= 0x80;
            break;
        default:
            error = -EINVAL;
            break;
    }
    if (error) {
        fpc_log_err("col_mask is invaild, error = %d\n", error);
        goto out;
    }

    temp_u32 <<= 8;
    temp_u32 |= 0; // No multisampling
    FPC1020_MK_REG_WRITE(reg, FPC102X_REG_IMG_SMPL_SETUP, &temp_u32);
    error = fpc1020_reg_access(fpc1020, &reg);
    if (error)
    { goto out; }

    temp_u8 = 0x08;
    FPC1020_MK_REG_WRITE(reg, FPC1020_REG_FNGR_DET_THRES, &temp_u8);
    error = fpc1020_reg_access(fpc1020, &reg);

    fpc_log_nav("col_start=%d, col_groups=%d, row_start=%d, row_count=%d\n",
            fpc1020->nav.image_nav_col_start, fpc1020->nav.image_nav_col_groups,
            fpc1020->nav.image_nav_row_start, fpc1020->nav.image_nav_row_count);

out:
    return error;
}


/* -------------------------------------------------------------------- */

/* -------------------------------------------------------------------- */
void fpc1020_rotate_image(fpc1020_data_t* fpc1020,
                          int sensor_width,
                          int sensor_height)
{
    u8* rotateBuffer;
    u8* curr_buffer;
    int dx = 0;
    int dy = 0;
    u32 image_size_bytes = sensor_width*sensor_height;

    curr_buffer = fpc1020->huge_buffer;
    rotateBuffer = kzalloc(image_size_bytes * sizeof(u8), GFP_KERNEL);
    for (dx = 0; dx < sensor_width; dx++)
    {
        for (dy = 0; dy < sensor_height; dy++)
        {
            rotateBuffer[dx + (dy * sensor_width)] = curr_buffer[(sensor_width - dx - 1) * sensor_height + dy];
        }
    }
    memcpy(curr_buffer, &rotateBuffer[0], image_size_bytes);
    kfree(rotateBuffer);
}
/*
static int fpc1020_check_image(fpc1020_data_t* fpc1020,
                          int sensor_width,
                          int sensor_height)
{
    u8* curr_buffer;
    u32 count;
    int dx;
    int dy;

    curr_buffer = fpc1020->huge_buffer;
    count = 0;
    for (dx = 0; dx < sensor_width; dx++)
    {
        for (dy = 0; dy < sensor_height; dy++)
        {
            if (curr_buffer[dx + (dy * sensor_width)] == 255)
            count ++;
        }
    }
    return count;
}
*/
/* -------------------------------------------------------------------- */
static int capture_nav_image(fpc1020_data_t* fpc1020)
{
    int error = 0;

    error = fpc1020_capture_buffer(fpc1020,
                                   fpc1020->huge_buffer,
                                   0,
                                   get_nav_img_capture_size());

    return error;
}

#endif
/* -------------------------------------------------------------------- */
