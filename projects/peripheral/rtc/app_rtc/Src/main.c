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
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "app_rtc.h"
#include "board_SK.h"
#include "grx_hal.h"
#include "grx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
char *const weeday_str[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static uint32_t interval = 0;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
void app_rtc_evt_handler(app_rtc_evt_t * p_evt)
{
    app_rtc_time_t time;

    if (p_evt->type == APP_RTC_EVT_DATE_ALARM)
    {
        printf("Date alarm.\r\n");
    }
    if (p_evt->type == APP_RTC_EVT_TICK_ALARM)
    {
        interval++;
        if ((interval % 100) == 0)
        {
            app_rtc_get_time(&time);
            printf("Tick alarm, %02d.%02d.%02d %s %02d:%02d:%02d:%03d, %d.\r\n",
            time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec, time.ms, interval);
        }
    }
}

void app_rtc(void)
{
    app_rtc_time_t time;
    app_rtc_alarm_t alarm;

    app_rtc_init(app_rtc_evt_handler);

    time.year = 19;
    time.mon  = 5;
    time.date = 20;
    time.hour = 8;
    time.min  = 0;
    time.sec  = 0;
    time.week = 0;
    time.ms   = 0;
    app_rtc_init_time(&time);

#if (CFG_LPCLK_INTERNAL_EN == 0) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    for (uint32_t i = 0; i < 5; i++)
    {
        delay_ms(1000);
        app_rtc_get_time(&time);
        printf("App current time: %02d.%02d.%02d %s %02d:%02d:%02d:%03d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec, time.ms);
    }

    alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    alarm.alarm_date_week_mask = 0xFF;
    alarm.hour = 8;
    alarm.min  = 1;
    printf("Set a date alarm every day at 8.01 am.\r\n");
    app_rtc_setup_alarm(&alarm);
    for (uint32_t i = 0; i < 12; i++)
    {
        delay_ms(1000);
        app_rtc_get_time(&time);
        printf("APP current time: %02d.%02d.%02d %s %02d:%02d:%02d:%03d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec, time.ms);
    }

    printf("Set an tick alarm every 10 ms.\r\n");
    app_rtc_setup_tick(10);
#else
    while(1)
    {
        delay_ms(1000);
        app_rtc_get_time(&time);
        printf("App current time: %02d.%02d.%02d %s %02d:%02d:%02d:%03d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec, time.ms);
    }
#endif
}

int main(void)
{
    /* Initial printf mode and UART */
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 RTC APP example.                   *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will print current time for every 100  *\r\n");
    printf("* tick interrupts.                                   *\r\n");
    printf("******************************************************\r\n");

    app_rtc();

    while(1);
}
