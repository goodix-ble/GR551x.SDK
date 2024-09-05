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
#include "app_pwm.h"
#include "board_SK.h"
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#include "app_pwm_dma.h"
#endif

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
app_pwm_params_t pwm_params = {
    .id = APP_PWM0_MODULE,
    .pin_cfg = {
        .channel_a = {
            .type   = APP_PWM0_GPIO_TYPE,
        #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
            .mux    = APP_PWM0_PIN_MUX_A,
        #else
            .mux    = APP_PWM0_GPIO_MUX,
        #endif
            .pin    = APP_PWM0_CHANNEL_A,
            .pull   = APP_IO_NOPULL,
            .enable = APP_PWM_PIN_ENABLE,
        },
        .channel_b = {
            .type   = APP_PWM0_GPIO_TYPE,
        #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
            .mux    = APP_PWM0_PIN_MUX_B,
        #else
            .mux    = APP_PWM0_GPIO_MUX,
        #endif
            .pin    = APP_PWM0_CHANNEL_B,
            .pull   = APP_IO_NOPULL,
            .enable = APP_PWM_PIN_ENABLE,
        },
        .channel_c = {
            .type   = APP_PWM0_GPIO_TYPE,
        #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
            .mux    = APP_PWM0_PIN_MUX_C,
        #else
            .mux    = APP_PWM0_GPIO_MUX,
        #endif
            .pin    = APP_PWM0_CHANNEL_C,
            .pull   = APP_IO_NOPULL,
            .enable = APP_PWM_PIN_ENABLE,
        },
    },
    .active_channel = APP_PWM_ACTIVE_CHANNEL_ALL,
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    .use_mode = {
        .pwm_dma_instance = DMA0,
        .pwm_dma_channel = DMA_Channel0,
    },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    .init = {
        .mode = PWM_FLICKER_MODE,
        .prd_cycles = 0,
        .none_coding_mode_cfg = {
            .align = PWM_ALIGNED_EDGE,
            .freq = 100,
            .bperiod = 500,
            .hperiod = 200,
            .bstoplvl = PWM_STOP_LVL_LOW,
            .channel_a = {
                .duty = 30,
                .drive_polarity = PWM_DRIVEPOLARITY_POSITIVE,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
                .fstoplvl = PWM_STOP_LVL_LOW,
#endif
            },
            .channel_b = {
                .duty = 50,
                .drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
                .fstoplvl = PWM_STOP_LVL_HIGH,
#endif
            },
            .channel_c = {
                .duty = 80,
                .drive_polarity = PWM_DRIVEPOLARITY_POSITIVE,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
                .fstoplvl = PWM_STOP_LVL_LOW,
#endif
            },
        },
        .coding_mode_cfg = {
            .period = 64,
            .waiting_time = 64000,
            .data_width_valid = 0x1F,
            .coding_channel_select = PWM_CODING_CHANNEL_ALL,
            .channel_a = {
                .comp0 = 16,
                .comp1 = 32,
                .drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE,
                .waiting_time_lvl = PWM_WAITING_TIME_LVL_LOW,
            },
            .channel_b = {
                .comp0 = 32,
                .comp1 = 48,
                .drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE,
                .waiting_time_lvl = PWM_WAITING_TIME_LVL_LOW,
            },
            .channel_c = {
                .comp0 = 12,
                .comp1 = 56,
                .drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE,
                .waiting_time_lvl = PWM_WAITING_TIME_LVL_LOW,
            },
        },
    },
#else
    .init = {
        .mode     = PWM_MODE_FLICKER,
        .align    = PWM_ALIGNED_EDGE,
        .freq     = 100,
        .bperiod  = 500,
        .hperiod  = 200,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .bstoplvl = PWM_STOP_LVL_LOW,
#endif
        .channel_a = {
            .duty = 30,
            .drive_polarity = PWM_DRIVEPOLARITY_POSITIVE,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .fstoplvl = PWM_STOP_LVL_LOW,
#endif
        },
        .channel_b = {
            .duty = 50,
            .drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .fstoplvl = PWM_STOP_LVL_HIGH,
#endif
        },
        .channel_c = {
            .duty = 80,
            .drive_polarity = PWM_DRIVEPOLARITY_POSITIVE,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .fstoplvl = PWM_STOP_LVL_LOW,
#endif
        },
    },
#endif
};

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
uint32_t pwm_test_data[4] = {
    0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A, 0,
};
volatile uint32_t g_done_cnt = 0;

void app_pwm_event_handler(app_pwm_evt_t *p_evt)
{
    if (p_evt->type == APP_PWM_CHANNEL_A_ERROR)
    {
        printf("\r\nThis is APP_PWM_CHANNEL_A_ERROR callback.\r\n");
    }
    if (p_evt->type == APP_PWM_CHANNEL_B_ERROR)
    {
        printf("\r\nThis is APP_PWM_CHANNEL_B_ERROR callback.\r\n");
    }
    if (p_evt->type == APP_PWM_CHANNEL_C_ERROR)
    {
        printf("\r\nThis is APP_PWM_CHANNEL_C_ERROR callback.\r\n");
    }
    if (p_evt->type == APP_PWM_CODING_DONE)
    {
        g_done_cnt = 1;
        printf("\r\nThis is APP_PWM_CODING_DONE callback.\r\n");
    }
    if (p_evt->type == APP_PWM_CODING_LOAD)
    {
        printf("\r\nThis is APP_PWM_CODING_LOAD callback.\r\n");
    }
}
#endif

void app_pwm_test(void)
{
    uint16_t ret = APP_DRV_SUCCESS;
    app_pwm_channel_init_t channel_cfg = {0};

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    ret = app_pwm_init(&pwm_params, NULL);
#else
    ret = app_pwm_init(&pwm_params);
#endif
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nPWM initial failed! Please check the input paraments.\r\n");
    }

    app_pwm_start(APP_PWM0_MODULE);

    delay_ms(2000);

    app_pwm_update_freq(APP_PWM0_MODULE, 1000);

    channel_cfg.duty = 20;
    channel_cfg.drive_polarity = PWM_DRIVEPOLARITY_POSITIVE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    channel_cfg.fstoplvl = PWM_STOP_LVL_LOW;
#endif
    app_pwm_config_channel(APP_PWM0_MODULE, APP_PWM_ACTIVE_CHANNEL_B, &channel_cfg);
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    app_pwm_inactive_channel(APP_PWM0_MODULE, APP_PWM_ACTIVE_CHANNEL_A);
#endif

    delay_ms(2000);

    app_pwm_stop(APP_PWM0_MODULE);

    delay_ms(2000);

    channel_cfg.duty = 60;
    channel_cfg.drive_polarity = PWM_DRIVEPOLARITY_POSITIVE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    channel_cfg.fstoplvl = PWM_STOP_LVL_HIGH;
#endif
    app_pwm_config_channel(APP_PWM0_MODULE, APP_PWM_ACTIVE_CHANNEL_ALL, &channel_cfg);

    app_pwm_start(APP_PWM0_MODULE);

    delay_ms(2000);

    app_pwm_stop(APP_PWM0_MODULE);
    app_pwm_deinit(APP_PWM0_MODULE);
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
void app_pwm_coding(void)
{
    uint16_t ret = APP_DRV_SUCCESS;
    pwm_params.init.mode = PWM_CODING_MODE;
    pwm_params.init.prd_cycles = 32;
    ret = app_pwm_init(&pwm_params, app_pwm_event_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nPWM initial failed! Please check the input paraments.\r\n");
    }

    g_done_cnt = 0;
    app_pwm_start_coding_in_three_channels(APP_PWM_ID_0, 0x5A5A5A5A, 0x5A5A5A5A, 0x5A5A5A5A);
    while(g_done_cnt == 0);

    app_pwm_deinit(APP_PWM0_MODULE);
}

void app_pwm_coding_dma(void)
{
    uint16_t ret = APP_DRV_SUCCESS;
    pwm_params.init.mode = PWM_CODING_MODE;
    pwm_params.init.prd_cycles = 32;

    /* Please initialize DMA in the following order. */
    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_pwm_init(&pwm_params, app_pwm_event_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nPWM initial failed! Please check the input paraments.\r\n");
    }
    ret = app_pwm_dma_init(&pwm_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nPWM DMA initial failed! Please check the input paraments.\r\n");
    }

    g_done_cnt = 0;
    app_pwm_start_coding_with_dma(APP_PWM_ID_0, pwm_test_data, sizeof(pwm_test_data));
    while(g_done_cnt == 0);

    /* Please deinitialize DMA in the following order. */
    app_pwm_dma_deinit(APP_PWM0_MODULE);
    app_pwm_deinit(APP_PWM0_MODULE);
}
#endif

int main(void)
{
    board_init();

    printf("\r\n");
    printf("***********************************************************\r\n");
    printf("*                  PWM_Output example.                    *\r\n");
    printf("*                                                         *\r\n");
    printf("*                     PWM_a(MSIOA*)                       *\r\n");
    printf("*                     PWM_b(MSIOA*)                       *\r\n");
    printf("*                     PWM_c(MSIOA*)                       *\r\n");
    printf("*                                                         *\r\n");
    printf("* This sample code will show PWM output.                  *\r\n");
    printf("* You can use logic analyzer to get PWM_a/b/c wave        *\r\n");
    printf("* on MSIOA*/MSIOA*/MSIOA*                                 *\r\n");
    printf("***********************************************************\r\n");

    app_pwm_test();

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    app_pwm_coding();
    app_pwm_coding_dma();
#endif

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
