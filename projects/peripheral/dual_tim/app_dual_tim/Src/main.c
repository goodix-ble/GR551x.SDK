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
#include "app_dual_tim.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DEFAULT_AUTO_RELOAD  63999999
#define TIMER_MS(X) (SystemCoreClock / 1000 *(X) - 1)
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#define CMP_TIMER_MS(X) (SystemCoreClock / 1000 *(X))
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#define DUAL_TIM0_CHANNAL_A_PARAM { {APP_IO_TYPE_MSIO, APP_IO_PIN_5}, \
                                    {HAL_DUAL_TIMER_CHANNEL_A, CMP_TIMER_MS(600), CMP_TIMER_MS(400), DUAL_TIMER_IO_INIT_RESET, \
                                     DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, \
                                     ENABLE, ENABLE, ENABLE, ENABLE, ENABLE} }
#define DUAL_TIM0_CHANNAL_B_PARAM { {APP_IO_TYPE_MSIO, APP_IO_PIN_6}, \
                                    {HAL_DUAL_TIMER_CHANNEL_B, CMP_TIMER_MS(600), CMP_TIMER_MS(400), DUAL_TIMER_IO_INIT_RESET, \
                                     DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, \
                                     ENABLE, ENABLE, ENABLE, ENABLE, ENABLE} }
#define DUAL_TIM0_CHANNAL_C_PARAM { {APP_IO_TYPE_MSIO, APP_IO_PIN_7}, \
                                    {HAL_DUAL_TIMER_CHANNEL_C, CMP_TIMER_MS(600), CMP_TIMER_MS(400), DUAL_TIMER_IO_INIT_RESET, \
                                     DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, DUAL_TIMER_IO_ACTION_TOGGLE, \
                                     ENABLE, ENABLE, ENABLE, ENABLE, ENABLE} }
#endif

app_dual_tim_params_t p_params_tim0 = {
    .id = APP_DUAL_TIM_ID_0,
    .init = {
        .prescaler    = DUAL_TIMER_PRESCALER_DIV0,
        .counter_mode = DUAL_TIMER_COUNTERMODE_LOOP,
        .auto_reload  = DEFAULT_AUTO_RELOAD,
    },
};

app_dual_tim_params_t p_params_tim1 = {
    .id = APP_DUAL_TIM_ID_1,
    .init = {
        .prescaler    = DUAL_TIMER_PRESCALER_DIV0,
        .counter_mode = DUAL_TIMER_COUNTERMODE_ONESHOT,
        .auto_reload  = DEFAULT_AUTO_RELOAD,
    },
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint32_t g_tim0_cnt = 0;
volatile uint32_t g_tim1_cnt = 0;
volatile uint8_t g_tim1_done_flag = 0;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_dual_tim0_event_handler(app_dual_tim_evt_t *p_evt)
{
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
         printf("\r\nThis is %dth call DUAL TIMER0.\r\n", g_tim0_cnt++);
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if (*p_evt == APP_DUAL_TIM_EVT_ACT_START)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_ACT_START callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_ACT_STOP)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_ACT_STOP callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOA_ACT_C1)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOA_ACT_C1 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOA_ACT_C2)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOA_ACT_C2 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_ACT_PERIOD)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_ACT_PERIOD callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOB_ACT_C1)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOB_ACT_C1 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOB_ACT_C2)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOB_ACT_C2 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOC_ACT_C1)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOC_ACT_C1 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOC_ACT_C2)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOC_ACT_C2 callback.\r\n");
    }
#endif
}

void app_dual_tim1_event_handler(app_dual_tim_evt_t *p_evt)
{
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
         g_tim1_done_flag = 1;
         printf("\r\n  This is %dth call DUAL TIMER1.\r\n", g_tim1_cnt++);
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if (*p_evt == APP_DUAL_TIM_EVT_ACT_START)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_ACT_START callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_ACT_STOP)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_ACT_STOP callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOA_ACT_C1)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOA_ACT_C1 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOA_ACT_C2)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOA_ACT_C2 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_ACT_PERIOD)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_ACT_PERIOD callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOB_ACT_C1)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOB_ACT_C1 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOB_ACT_C2)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOB_ACT_C2 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOC_ACT_C1)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOC_ACT_C1 callback.\r\n");
    }
    if (*p_evt == APP_DUAL_TIM_EVT_IOC_ACT_C2)
    {
         printf("\r\nThis is APP_DUAL_TIM_EVT_IOC_ACT_C2 callback.\r\n");
    }
#endif
}

void dual_timer_interrupt(void)
{
    uint32_t tim1_interrupt_time= 1000; // uinit: ms
    p_params_tim0.init.auto_reload = TIMER_MS(1000);
    p_params_tim1.init.auto_reload = TIMER_MS(1000);
    app_dual_tim_init(&p_params_tim0, app_dual_tim0_event_handler);
    app_dual_tim_init(&p_params_tim1, app_dual_tim1_event_handler);

    printf("\r\nDUAL TIMER0 and DUAL TIMER1 start\r\n");
    app_dual_tim_start(APP_DUAL_TIM_ID_0);
    app_dual_tim_start(APP_DUAL_TIM_ID_1);
    // Set DUAL TIMER0 next Timing time to 5000ms, and does not affect the last timing time
    app_dual_tim_set_background_reload(APP_DUAL_TIM_ID_0, TIMER_MS(5000));

    while(g_tim1_cnt < 5)
    {
        if(g_tim1_done_flag)
        {
            g_tim1_done_flag = 0;
            p_params_tim1.init.counter_mode = DUAL_TIMER_COUNTERMODE_ONESHOT;
            tim1_interrupt_time += 1000;
            p_params_tim1.init.auto_reload =  TIMER_MS(tim1_interrupt_time);
            app_dual_tim_set_params(&p_params_tim1, APP_DUAL_TIM_ID_1);
            app_dual_tim_start(APP_DUAL_TIM_ID_1);
        }
    }

    app_dual_tim_stop(APP_DUAL_TIM_ID_0);
    app_dual_tim_stop(APP_DUAL_TIM_ID_1);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
void dual_timer_io_ctrl(void)
{
    g_tim0_cnt = 0;
    p_params_tim0.init.auto_reload = TIMER_MS(1000);
    app_dual_tim_init(&p_params_tim0, app_dual_tim0_event_handler);

    app_dual_tim_io_crtl_params_t io_crtl_cha_params = DUAL_TIM0_CHANNAL_A_PARAM;
    app_dual_tim_io_crtl_params_t io_crtl_chb_params = DUAL_TIM0_CHANNAL_B_PARAM;
    app_dual_tim_io_crtl_params_t io_crtl_chc_params = DUAL_TIM0_CHANNAL_C_PARAM;

    app_dual_tim_io_crtl_config(APP_DUAL_TIM_ID_0, &io_crtl_cha_params);
    app_dual_tim_io_crtl_config(APP_DUAL_TIM_ID_0, &io_crtl_chb_params);
    app_dual_tim_io_crtl_config(APP_DUAL_TIM_ID_0, &io_crtl_chc_params);

    printf("\r\nDUAL TIMER0 IO control start\r\n");
    app_dual_tim_start(APP_DUAL_TIM_ID_0);

    while (g_tim0_cnt < 10);

    app_dual_tim_stop(APP_DUAL_TIM_ID_0);
}
#endif

int main(void)
{
    board_init();

    printf("\r\n");
    printf("\r\n");
    printf("********************************************************\r\n");
    printf("*             Dual_Timer App example.                  *\r\n");
    printf("*                                                      *\r\n");
    printf("* In this sample code, dual_timer0 will interrupt      *\r\n");
    printf("* every 5 second, and the UART will print every        *\r\n");
    printf("* interrupt. Dual_timer1 is in ONESHOT mode, and the   *\r\n");
    printf("* interrupt time add 1 second after interrupt          *\r\n");
    printf("* program will stop after 15s.                         *\r\n");
    printf("********************************************************\r\n");
    printf("\r\n");

    dual_timer_interrupt();

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    dual_timer_io_ctrl();
#endif

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
