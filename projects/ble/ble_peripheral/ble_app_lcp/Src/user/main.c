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
#include "app_io.h"
#include "app_error.h"
#include "app_gpiote.h"
#include "ble_lcp.h"
#include "app_dual_tim.h"
#include "pmu_calibration.h"

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
#include "app_tim.h"
#define REG_BLE_RD(addr)         (*(volatile uint32_t *)(addr))
typedef struct
{
    uint32_t                pin_time;
    uint32_t                send_time;
    uint16_t                count;
    uint8_t                 mode;
} app_io_toggle_time_t;
#endif
#define TIMER_CNT_US      (1000*1)
#define CRC_INIT          0x555555
#define ACCESS_ADDRESS    0x71764129
#define FREQ_MHZ          2402
#define CHANNEL_IDX       0

#define DUAL_TIM0_PARAM           { APP_DUAL_TIM_ID_0, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_ONESHOT, 64000000 - 1 }}
#define TIMER_SYSCNT_TO_US(X)     ((SystemCoreClock / 1000000) *(X) - 1)

/*
 * FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_lcp_timer_start(uint32_t timeout_ms);
void app_lcp_timer1_start(uint32_t timeout_us);
static void app_gpiote_event_handler(app_io_evt_t *p_evt);
extern uint32_t lld_lcp_get_whiten_seed_by_channel(uint8_t  ch_idx);
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

app_gpiote_param_t s_app_pulse_io_cfg[] =
{
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    {APP_IO_TYPE_AON,   APP_IO_PIN_3,    APP_IO_MODE_IT_BOTH_EDGE,  APP_IO_PULLUP, app_gpiote_event_handler},
    {APP_IO_TYPE_AON,   APP_IO_PIN_4,    APP_IO_MODE_OUTPUT,  APP_IO_PULLUP, NULL},
#else
    {APP_IO_TYPE_NORMAL,   APP_IO_PIN_26,    APP_IO_MODE_IT_RISING,  APP_IO_PULLUP, app_gpiote_event_handler},
#endif
};
app_io_init_t g_io_init;
app_dual_tim_params_t params_tim0 = DUAL_TIM0_PARAM;
static uint16_t send_cnt=0;
uint8_t whiten_channel_idx[40]=
{
22,23,24,25,26,27,28,29,30,31,
32,34,35,36,37,38,39,40,41,42,
43,44,45,46,47,48,49,50,51,52,
53,54,55,56,57,58,59,21,33,60,
};

SECTION_RAM_CODE void app_gpio_detection_edge_change(void)
{
    if (s_app_pulse_io_cfg[0].mode == APP_IO_MODE_IT_RISING)
    {
        s_app_pulse_io_cfg[0].mode = APP_IO_MODE_IT_FALLING;
    }
    else if (s_app_pulse_io_cfg[0].mode == APP_IO_MODE_IT_FALLING)
    {
        s_app_pulse_io_cfg[0].mode = APP_IO_MODE_IT_RISING;
    }

    g_io_init.mode = s_app_pulse_io_cfg[0].mode;
    app_io_init(s_app_pulse_io_cfg[0].type, &g_io_init);
}


SECTION_RAM_CODE static void app_gpiote_event_handler(app_io_evt_t *p_evt)
{
    if (NULL == p_evt)
        return;

    if (p_evt->pin & (s_app_pulse_io_cfg[0].pin))
    {
        send_cnt++;
        uint8_t header, length;
        uint8_t payload[5] = {0x01, 0x02, 0x03, 0x4, 0x05};
        gdx_lcp_rx_stop();
        header = 0x00;
        length = 0x5;
        payload[0] = send_cnt&0xFF;
        payload[1] = (send_cnt>>8)&0xFF;
        #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
        gdx_lcp_channel_set(FREQ_MHZ+(send_cnt%2),send_cnt%2);
        gdx_lcp_data_tx(header, length, payload);
        gdx_lcp_rx_start();
        #else
        gdx_lcp_channel_set((FREQ_MHZ + send_cnt%2));
        gdx_lcp_whitening_seed_set(lld_lcp_get_whiten_seed_by_channel(whiten_channel_idx[send_cnt%2] ));
        gdx_lcp_data_tx(header, length, payload);
        app_lcp_timer_start(TIMER_CNT_US);
        #endif
         


        app_io_toggle_pin(APP_IO_TYPE_AON, s_app_pulse_io_cfg[1].pin);
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
        //The gpio is in single edge detection mode, change the detection edge
        //between rising and falling dynamically after responsing one gpio irq
        app_gpio_detection_edge_change();
#endif
    }
}

void app_gpio_input_cfg(void)
{
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    app_io_pin_state_t   pin_state;

    app_gpiote_init(s_app_pulse_io_cfg, sizeof(s_app_pulse_io_cfg) / sizeof (app_gpiote_param_t));
    pin_state = app_io_read_pin(s_app_pulse_io_cfg[0].type, s_app_pulse_io_cfg[0].pin);
    if (APP_IO_PIN_RESET == pin_state)
    {
        s_app_pulse_io_cfg[0].mode = APP_IO_MODE_IT_RISING;
    }
    else
    {
        s_app_pulse_io_cfg[0].mode = APP_IO_MODE_IT_FALLING;
    }

    g_io_init.pin  = s_app_pulse_io_cfg[0].pin;
    g_io_init.mode = s_app_pulse_io_cfg[0].mode;
    g_io_init.pull = s_app_pulse_io_cfg[0].pull;
    g_io_init.mux  = APP_IO_MUX_7;
    app_io_init(s_app_pulse_io_cfg[0].type, &g_io_init);
#else
    g_io_init.pin  = s_app_pulse_io_cfg[0].pin;
    g_io_init.mode = s_app_pulse_io_cfg[0].mode;
    g_io_init.pull = s_app_pulse_io_cfg[0].pull;
    g_io_init.mux  = APP_IO_MUX_0;
    app_io_init(s_app_pulse_io_cfg[0].type, &g_io_init);
    
    g_io_init.pin  = s_app_pulse_io_cfg[1].pin;
    g_io_init.mode = s_app_pulse_io_cfg[1].mode;
    g_io_init.pull = s_app_pulse_io_cfg[1].pull;
    g_io_init.mux  = APP_IO_MUX_0;
    app_io_init(s_app_pulse_io_cfg[1].type, &g_io_init);

    app_gpiote_init(s_app_pulse_io_cfg, sizeof(s_app_pulse_io_cfg) / sizeof (app_gpiote_param_t));
#endif

    NVIC_SetPriority(AON_EXT_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
}

void app_lcp_tim0_event_handler(app_dual_tim_evt_t *p_evt)
{

}

static void app_lcp_timer_init(void)
{
    params_tim0.init.auto_reload = SystemCoreClock - 1;
    app_dual_tim_init(&params_tim0, app_lcp_tim0_event_handler);
}

SECTION_RAM_CODE void app_lcp_timer_start(uint32_t timeout_us)
{
    app_dual_tim_params_t p_params_tim0 = DUAL_TIM0_PARAM;

    p_params_tim0.init.auto_reload = TIMER_SYSCNT_TO_US(timeout_us);
    app_dual_tim_set_params(&p_params_tim0, APP_DUAL_TIM_ID_0);
    app_dual_tim_start(APP_DUAL_TIM_ID_0);
}

void app_lcp_timer_stop(void)
{
    app_dual_tim_stop(APP_DUAL_TIM_ID_0);
}

void app_lcp_timer_deinit(void)
{
    app_dual_tim_deinit(APP_DUAL_TIM_ID_0);
}
//static uint16_t recv_cnt=0;
SECTION_RAM_CODE uint16_t gdx_lcp_data_rx_handler_callback(uint8_t header, uint8_t length, uint8_t *p_payload)
{
    uint16_t status = 0;

    uint16_t *cnt = (uint16_t*)p_payload;
//    recv_cnt= *cnt;
    printf("recv cnt =%d\r\n",*cnt);

    return status;
}


void gdx_lcp_data_tx_handler_callback()
{

}
SECTION_RAM_CODE void gdx_lcp_data_rx_done_callback(uint8_t type)
{

}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)




void lcp_init(void)
{
    sdk_err_t error_code;
    gdx_lcp_config_t lcp_config;

    lcp_config.trx_mode = LCP_TRX_MODE_TIMER_TX;
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

    lcp_config.trx_timer_period_us = TIMER_CNT_US;
    lcp_config.trx_timer_trigger_trx_time_us = TIMER_CNT_US;

    
    error_code = gdx_lcp_init(&lcp_config);
    gdx_lcp_whitening_seed_set(lld_lcp_get_whiten_seed_by_channel(whiten_channel_idx[CHANNEL_IDX] ));
    //gdx_lcp_t2r_turn_around_time_adjust(50);
    APP_ERROR_CHECK(error_code);
    app_lcp_timer_init();
    // Stop pmu calibration timer before lcp start, avoid affecting gpio irq response
    system_pmu_calibration_stop();
    APP_LOG_INFO("PTX  Light Communication Protocol Example Started.");
    extern void lld_lcp_timer_binding(bool b_dual_timer, uint8_t timer_id);
    gdx_lcp_timer_binding(true, APP_DUAL_TIM_ID_0);
    //gdx_lcp_timer_binding(true, 0);

    //app_lcp_timer_start(625);
}

#else
void lcp_init(void)
{
    sdk_err_t error_code;
    gdx_lcp_config_t lcp_config;

    lcp_config.mode = LCP_TX;
    lcp_config.txpwr_dbm = 0;
    lcp_config.freq = FREQ_MHZ;
    lcp_config.ch_idx = CHANNEL_IDX;
    lcp_config.access_address = ACCESS_ADDRESS;
    lcp_config.crc_init = CRC_INIT;
    lcp_config.rx_handler_cb = &gdx_lcp_data_rx_handler_callback;
    error_code = gdx_lcp_init(&lcp_config);
    APP_ERROR_CHECK(error_code);
    app_lcp_timer_init();
    // Stop pmu calibration timer before lcp start, avoid affecting gpio irq response
    system_pmu_calibration_stop();
    APP_LOG_INFO("Light Communication Protocol Example Started.");
}
#endif

void lcp_deinit(void)
{
    gdx_lcp_deinit();
    // Resume pmu calibration after lcp stop
    system_pmu_calibration_init(30000);
    app_lcp_timer_deinit();
    APP_LOG_INFO("Light Communication Protocol Example Stop.");
}

int main (void)
{
    // Initialize user peripherals.
    app_periph_init();
    app_lcp_timer_init();
    // Initialize ble stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    lcp_init();
    app_gpio_input_cfg();

    // loop
    while (1)
    {

    }
}
