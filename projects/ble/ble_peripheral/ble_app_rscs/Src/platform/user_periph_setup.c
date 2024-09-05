/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_periph_setup.h"
#include "board_SK.h"
#include "gr_includes.h"
#include "app_assert.h"
#include "app_log.h"
#include "app_scheduler.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "flash_scatter_config.h"
#include "app_rtc.h"

/*
 * DEFINENS
 *****************************************************************************************
 */
#define APP_SCHEDULER_QUEUE_SIZE            16                /**< Size of app scheduler queue. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x0e, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern bool g_sensor_calibration_done_flag;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
#if APP_LOG_STORE_ENABLE
static void get_timestamp(app_log_store_time_t *p_time)
{
    app_rtc_time_t time;
    app_rtc_get_time(&time);
    p_time->year=time.year;
    p_time->month=time.mon;
    p_time->day=time.date;
    p_time->hour=time.hour;
    p_time->min=time.min;
    p_time->sec=time.sec;
    p_time->msec=time.ms;
}

static void app_rtc(void)
{
    app_rtc_time_t time;
    app_rtc_init(NULL);
    time.year = 23;
    time.mon  = 2;
    time.date = 13;
    time.hour = 8;
    time.min  = 0;
    time.sec  = 0;
    time.week = 0;
    time.ms   = 0;
    app_rtc_init_time(&time);
}

static void log_store_init(void)
{
    app_log_store_info_t store_info;
    app_log_store_op_t   op_func;

    store_info.nv_tag   = 0x40ff;
    store_info.db_addr  = FLASH_START_ADDR + 0x60000;
    store_info.db_size  = 0x20000;
    store_info.blk_size = 0x1000;

    op_func.flash_init  = hal_flash_init;
    op_func.flash_erase = hal_flash_erase;
    op_func.flash_write = hal_flash_write;
    op_func.flash_read  = hal_flash_read;
    op_func.time_get    = get_timestamp;
    op_func.sem_give    = NULL;
    op_func.sem_take    = NULL;

    app_rtc();
    app_log_store_init(&store_info, &op_func);
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    if (BSP_KEY_OK_ID == key_id && APP_KEY_SINGLE_CLICK == key_click_type)
    {
        g_sensor_calibration_done_flag = true;
    }
}

void app_periph_init(void)
{
    app_scheduler_init(APP_SCHEDULER_QUEUE_SIZE);
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
#if APP_LOG_STORE_ENABLE
    log_store_init();
#endif
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}
