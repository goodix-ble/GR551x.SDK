/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "watcher.h"
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"
#include "pmu_calibration.h"
#include "app_rtc.h"
#include "dfu_port.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_TASK_STACK_SIZE             ( 128 )//unit : word
#define LOG_STORE_DUMP_TASK_STACK_SIZE  ( 512 )//unit : word

#ifdef SOC_GR5515
#define DFU_TASK_STACK_SIZE             ( 1024 * 2 )//unit : word
#else
#define DFU_TASK_STACK_SIZE             ( 256 )//unit : word
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);
calendar_time_t g_calendar_time;
char *const     weeday_str[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void print_test_task(void *p_arg)
{
    while (1)
    {
        app_rtc_get_time(&g_calendar_time);
        APP_LOG_INFO("TickCount: %d, Time: %02d/%02d %02d:%02d:%02d.\r\n",
                      xTaskGetTickCount(),
                      g_calendar_time.mon, g_calendar_time.date,
                      g_calendar_time.hour, g_calendar_time.min, g_calendar_time.sec);
        app_log_flush();
        vTaskDelay(1000);
    }
}

static void dfu_schedule_task(void *p_arg)
{
    while (1)
    {
        dfu_schedule();
        vTaskDelay(100);
    }
}

#if APP_LOG_STORE_ENABLE
static void log_store_dump_task(void *p_arg)
{
    while (1)
    {
        app_log_store_schedule();
    }
}
#endif

static void app_calendar_init(void)
{
    g_calendar_time.year = 21;
    g_calendar_time.mon  = 12;
    g_calendar_time.date = 1;
    g_calendar_time.hour = 1;
    g_calendar_time.min  = 00;
    g_calendar_time.sec  = 00;
    app_rtc_init(NULL);
    app_rtc_init_time(&g_calendar_time);
}

/**
 *****************************************************************************************
 * @brief To create two task, the one is ble-schedule, another is watcher task
 *****************************************************************************************
 */
static void vStartTasks(void *arg)
{
    app_calendar_init();
    xTaskCreate(print_test_task, "print_task", APP_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(dfu_schedule_task, "dfu_schedule_task", DFU_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
#if APP_LOG_STORE_ENABLE
    xTaskCreate(log_store_dump_task, "log_store_dump_task", LOG_STORE_DUMP_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);
#endif
    vTaskDelete(NULL);
}

/**
 *****************************************************************************************
 * @brief main function
 *****************************************************************************************
 */
int main(void)
{
    app_periph_init();                                              /*<init user periph .*/
    ble_stack_init(ble_evt_handler, &heaps_table);                  /*< init ble stack*/

    xTaskCreate(vStartTasks, "create_task", 512, NULL, 0, NULL);    /*< create some demo tasks via freertos */
    vTaskStartScheduler();                                          /*< freertos run all tasks*/
    for (;;);                                                       /*< Never perform here */
}
