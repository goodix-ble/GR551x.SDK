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
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"
#include "app_error.h"
#include "app_io.h"
#include "app_gpiote.h"
#include "app_dual_tim.h"
#include "ble_lcp.h"

#define CRC_INIT          0x555555
#define ACCESS_ADDRESS    0x6730363F
#define FREQ_MHZ          2474
#define CHANNEL_IDX       34

#define DUAL_TIM0_PARAM           { APP_DUAL_TIM_ID_0, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_ONESHOT, 64000000 - 1 }}

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

app_gpiote_param_t s_app_pulse_io_cfg[] =
{
    {APP_IO_TYPE_NORMAL,   APP_IO_PIN_30,    APP_IO_MODE_OUTPUT,  APP_IO_NOPULL, NULL},
};

void app_gpio_output_cfg(void)
{
    app_gpiote_init(s_app_pulse_io_cfg, sizeof(s_app_pulse_io_cfg) / sizeof (app_gpiote_param_t));
}

app_dual_tim_params_t params_tim0 = DUAL_TIM0_PARAM;

SECTION_RAM_CODE uint16_t gdx_lcp_data_rx_handler_callback(uint8_t header, uint8_t length, uint8_t *p_payload)
{
    uint16_t status = 0;
    uint8_t header_ack, length_ack, payload_ack;

    // toggle for retore pulse
    app_io_toggle_pin(APP_IO_TYPE_NORMAL, APP_IO_PIN_30);
    gdx_lcp_rx_stop();
    header_ack = 0x00;
    length_ack = 0x1;
    payload_ack = 0x24;
    gdx_lcp_data_tx(header_ack, length_ack, &payload_ack);
    gdx_lcp_rx_start();

    return status;
}

void app_lcp_tim0_event_handler(app_dual_tim_evt_t *p_evt)
{
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
        //TODO: add timeout handler
    }
}

static void app_lcp_timer_init(void)
{
    params_tim0.init.auto_reload = SystemCoreClock - 1;

    app_dual_tim_init(&params_tim0, app_lcp_tim0_event_handler);
}

void lcp_init(void)
{
    sdk_err_t error_code;
    gdx_lcp_config_t lcp_config;

    lcp_config.mode = LCP_RX;
    lcp_config.txpwr_dbm = 0;
    lcp_config.freq = FREQ_MHZ;
    lcp_config.ch_idx = CHANNEL_IDX;
    lcp_config.access_address = ACCESS_ADDRESS;
    lcp_config.crc_init = CRC_INIT;
    lcp_config.rx_handler_cb = &gdx_lcp_data_rx_handler_callback;
    error_code = gdx_lcp_init(&lcp_config);
    APP_ERROR_CHECK(error_code);
    app_lcp_timer_init();
    APP_LOG_INFO("Light Communication Protocol Client Example Started.");
}

void lcp_deinit(void)
{
    sdk_err_t error_code;

    error_code = gdx_lcp_deinit();
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Light Communication Protocol Client Example Stop.");
}

int main (void)
{
    // Initialize user peripherals.
    app_periph_init();

    // Initialize ble stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    lcp_init();
    app_gpio_output_cfg();
    gdx_lcp_rx_start();

    // loop
    while (1)
    {
//        app_log_flush();
//        pwr_mgmt_schedule();
    }
}
