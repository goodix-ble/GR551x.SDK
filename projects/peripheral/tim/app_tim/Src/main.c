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
#include "app_tim.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
app_tim_params_t p_params_tim0;
app_tim_params_t p_params_tim1;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint32_t g_tim0_cnt = 0;
volatile uint32_t g_tim1_cnt = 0;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_tim0_event_handler(app_tim_evt_t *p_evt)
{
    if (*p_evt == APP_TIM_EVT_DONE)
    {
         printf("\r\nThis is %dth call TIMER0.\r\n", g_tim0_cnt++);
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if(*p_evt == APP_TIM_EVT_CHANNEL0)
    {
        uint32_t cur_val = app_tim_get_channel0_val(APP_TIM_ID_0);
        printf("\r\nTIMER0_CHANNEL0 EVENT = %d.\r\n", cur_val);
    }
    if(*p_evt == APP_TIM_EVT_CHANNEL1)
    {
        uint32_t cur_val = app_tim_get_channel1_val(APP_TIM_ID_0);
        printf("\r\nTIMER0_CHANNEL1 EVENT = %d.\r\n", cur_val);
    }
    if(*p_evt == APP_TIM_EVT_CHANNEL2)
    {
        uint32_t cur_val = app_tim_get_channel2_val(APP_TIM_ID_0);
        printf("\r\nTIMER0_CHANNEL2 EVENT = %d.\r\n", cur_val);
    }
    if(*p_evt == APP_TIM_EVT_CHANNEL3)
    {
        uint32_t cur_val = app_tim_get_channel3_val(APP_TIM_ID_0);
        printf("\r\nTIMER0_CHANNEL3 EVENT = %d.\r\n", cur_val);
    }
#endif
}

void app_tim1_event_handler(app_tim_evt_t *p_evt)
{
    if (*p_evt == APP_TIM_EVT_DONE)
    {
         g_tim1_cnt++;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if(*p_evt == APP_TIM_EVT_CHANNEL0)
    {
        uint32_t cur_val = app_tim_get_channel0_val(APP_TIM_ID_1);
        printf("\r\nTIMER1_CHANNEL0 EVENT = %d.\r\n", cur_val);
    }
    if(*p_evt == APP_TIM_EVT_CHANNEL1)
    {
        uint32_t cur_val = app_tim_get_channel1_val(APP_TIM_ID_1);
        printf("\r\nTIMER1_CHANNEL1 EVENT = %d.\r\n", cur_val);
    }
    if(*p_evt == APP_TIM_EVT_CHANNEL2)
    {
        uint32_t cur_val = app_tim_get_channel2_val(APP_TIM_ID_1);
        printf("\r\nTIMER1_CHANNEL2 EVENT = %d.\r\n", cur_val);
    }
    if(*p_evt == APP_TIM_EVT_CHANNEL3)
    {
        uint32_t cur_val = app_tim_get_channel3_val(APP_TIM_ID_1);
        printf("\r\nTIMER1_CHANNEL3 EVENT = %d.\r\n", cur_val);
    }
#endif
}

void tim_interrupt(void)
{
    p_params_tim0.id = APP_TIM_ID_0;
    p_params_tim0.init.auto_reload = SystemCoreClock - 1;
    p_params_tim1.id = APP_TIM_ID_1;
    p_params_tim1.init.auto_reload = SystemCoreClock / 100 - 1;

    app_tim_init(&p_params_tim0, app_tim0_event_handler);
    app_tim_init(&p_params_tim1, app_tim1_event_handler);

    app_tim_start(APP_TIM_ID_0);
    app_tim_start(APP_TIM_ID_1);

    while(g_tim1_cnt < 1000);

    app_tim_stop(APP_TIM_ID_0);
    app_tim_stop(APP_TIM_ID_1);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
void tim_gpio_init(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    // Note：User need to configure the capture_pin of APP_TIM as an input mode.
    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.pin  = APP_IO_PIN_6;
    io_init.mux  = APP_IO_MUX;
    app_io_init(APP_IO_TYPE_MSIO, &io_init);

    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = APP_IO_PIN_7;
    io_init.mux  = APP_IO_MUX;
    app_io_init(APP_IO_TYPE_MSIO, &io_init);
}

void tim_io_capture(void)
{
    p_params_tim0.id = APP_TIM_ID_0;
    p_params_tim0.init.auto_reload = SystemCoreClock - 1;
    // Note：User need to configure the capture_pin of APP_TIM as an input mode.
    p_params_tim0.init.capture_channel0.capture_pin = HAL_TIMER_CAPTURE_MSIO_PIN_6;
    p_params_tim0.init.capture_channel0.edge_capture = HAL_TIMER_CAPTURE_BOTH;

    app_tim_init(&p_params_tim0, app_tim0_event_handler);

    printf("\r\nTIMER0 IO capture start\r\n");
    app_tim_start(APP_TIM_ID_0);

    for (int i = 0;i < 10;i++)
    {
        app_io_toggle_pin(APP_IO_TYPE_MSIO, APP_IO_PIN_7);
        delay_ms(500);
    }

    app_tim_stop(APP_TIM_ID_0);
}
#endif

int main(void)
{
    board_init();

    printf("\r\n");
    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 Timer App example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* In this sample code, timer0 will interrupt every   *\r\n");
    printf("* 1 second, and the UART will print every 1 second.  *\r\n");
    printf("* Timer1 will interrupt every 10ms, and program will *\r\n");
    printf("* stop after 10s.                                    *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    tim_interrupt();

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    tim_gpio_init();
    tim_io_capture();
#endif

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
