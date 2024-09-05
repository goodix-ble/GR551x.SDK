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
#include "pmu_calibration.h"

#define CRC_INIT          0x555555
#define ACCESS_ADDRESS    0x71764129
#define FREQ_MHZ          2402
#define CHANNEL_IDX       0


#define DUAL_TIM0_PARAM           { APP_DUAL_TIM_ID_0, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_ONESHOT, 64000000 - 1 }}

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

app_gpiote_param_t s_app_pulse_io_cfg[] =
{
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    {APP_IO_TYPE_AON,   APP_IO_PIN_3,    APP_IO_MODE_OUTPUT,  APP_IO_NOPULL, NULL},
#else
    {APP_IO_TYPE_NORMAL,   APP_IO_PIN_30,    APP_IO_MODE_OUTPUT,  APP_IO_NOPULL, NULL},
#endif
};
uint8_t rx_flag=0;
void app_gpio_output_cfg(void)
{
    app_gpiote_init(s_app_pulse_io_cfg, sizeof(s_app_pulse_io_cfg) / sizeof (app_gpiote_param_t));
}

app_dual_tim_params_t params_tim0 = DUAL_TIM0_PARAM;

uint32_t ptx_tx_cnt = 0;
uint16_t p_rx_cnt = 0;
uint8_t whiten_channel_idx[40]=
{
22,23,24,25,26,27,28,29,30,31,
32,34,35,36,37,38,39,40,41,42,
43,44,45,46,47,48,49,50,51,52,
53,54,55,56,57,58,59,21,33,60,
};

extern uint32_t lld_lcp_get_whiten_seed_by_channel(uint8_t  ch_idx);

void test_lcp_send(uint16_t cnt)
{
    //lcp_init();
    uint8_t payload[5] = {0x01, 0x02, 0x03, 0x4, 0x05};
//    uint8_t header, length;
    //app_lcp_timer_start(625);
    gdx_lcp_rx_stop();
    payload[0] = cnt&0xFF;
    payload[1] = (cnt>>8)&0xFF;
    gdx_lcp_data_tx(0x20, 0x5, &payload[0]);
}
void gdx_lcp_data_tx_handler_callback()
{
    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    gdx_lcp_channel_set((FREQ_MHZ + (p_rx_cnt+1)%2));
    gdx_lcp_whitening_seed_set(lld_lcp_get_whiten_seed_by_channel(whiten_channel_idx[(p_rx_cnt+1)%2] ));
    gdx_lcp_rx_start();
    #endif
}

SECTION_RAM_CODE void gdx_lcp_data_rx_done_callback(uint8_t type)
{

}

SECTION_RAM_CODE uint16_t gdx_lcp_data_rx_handler_callback(uint8_t header, uint8_t length, uint8_t *p_payload)
{
    uint16_t status = 0;
    uint16_t *cnt = (uint16_t*)p_payload;

    // toggle for retore pulse
    //app_io_toggle_pin(APP_IO_TYPE_AON, s_app_pulse_io_cfg[0].pin);
    gdx_lcp_rx_stop();
    p_rx_cnt = *cnt;
    test_lcp_send(*cnt);
    #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    gdx_lcp_channel_set(FREQ_MHZ+((*cnt+1)%2),(*cnt+1)%2);
    gdx_lcp_rx_start();
    #endif
    printf("recv cnt =%d\r\n",*cnt);

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

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

void lcp_init(void)
{
    sdk_err_t error_code;
    gdx_lcp_config_t lcp_config;

    lcp_config.trx_mode = LCP_TRX_MODE_SW_RX;
    lcp_config.txpwr_dbm = 0;
    lcp_config.freq_mhz = FREQ_MHZ;
    lcp_config.access_address = ACCESS_ADDRESS;
    lcp_config.crc_init = CRC_INIT;
    lcp_config.rx_window_size_us = 0;
    lcp_config.rate= LCP_RATE_1MBPS;
    lcp_config.whiten_en = true;
    lcp_config.tx_done_cb = gdx_lcp_data_tx_handler_callback;
    lcp_config.rx_done_cb = gdx_lcp_data_rx_done_callback;
    lcp_config.rx_handler_cb = gdx_lcp_data_rx_handler_callback;
    lcp_config.b_disable_rx_oneshot_mode = true;

    lcp_config.trx_timer_period_us = 0;
    lcp_config.trx_timer_trigger_trx_time_us = 0;


    error_code = gdx_lcp_init(&lcp_config);
    gdx_lcp_whitening_seed_set(lld_lcp_get_whiten_seed_by_channel(whiten_channel_idx[CHANNEL_IDX] ));
    APP_ERROR_CHECK(error_code);
    app_lcp_timer_init();
    // Stop pmu calibration timer before lcp start, avoid affecting gpio irq response
    system_pmu_calibration_stop();
    APP_LOG_INFO("PRX  Light Communication Protocol Example Started.");
}


#else
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
    lcp_config.rx_handler_cb = gdx_lcp_data_rx_handler_callback;
    gdx_lcp_init(&lcp_config);
    APP_ERROR_CHECK(error_code);
    app_lcp_timer_init();
    APP_LOG_INFO("PRX Light Communication Protocol Client Example Started.");
}
#endif

void lcp_deinit(void)
{
    gdx_lcp_deinit();
    APP_LOG_INFO("PRX Light Communication Protocol Client Example Stop.");
}

volatile uint32_t wanggang10 = 0;

int main (void)
{
    // Initialize user peripherals.
    app_periph_init();

    // Initialize ble stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    lcp_init();
    //app_gpio_output_cfg();
    gdx_lcp_rx_start();

    while (1)
    {

    }
}
