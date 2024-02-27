/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "grx_sys.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "ble_lcp.h"
#include "app_dual_tim.h"
#include "app_timer.h"
#include "app_idt.h"
#include "ble_ism.h"
#include "app_io.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */

#define DUAL_TIM0_PARAM           { APP_DUAL_TIM_ID_0, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_ONESHOT, SystemCoreClock - 1 }}
#define US_SYSCNT_TO_TIMER(X)     ((SystemCoreClock / 1000000) *(X) - 1)
#define IDT_MAX_LENGTH            251

/*
 * FUNCTION DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
typedef struct
{
    uint8_t length;
    uint8_t data[IDT_MAX_LENGTH];
} idt_packet_t;

static idt_packet_t s_idt_packet;

static app_timer_id_t s_idt_init_delay_timer;

static bool s_idt_prepared = false;
static idt_cb_func_t *sp_idt_cb_func = NULL;

static uint32_t s_rtt_tx_time = 0;
static uint32_t s_rtt_rx_time = 0;

static uint8_t  s_ack_data = 0xA5;

static uint32_t s_access_addr = 0;
static uint32_t s_channel_idx = 0;

static uint8_t s_random_nums[10];
static uint8_t s_random_idx = 0;

SECTION_RAM_CODE static void app_idt_direct_timer_stop(void)
{
    app_dual_tim_stop(APP_DUAL_TIM_ID_0);
}

SECTION_RAM_CODE static void app_idt_direct_timer_start(uint32_t timeout_us)
{
    app_dual_tim_params_t p_params_tim0 = DUAL_TIM0_PARAM;

    p_params_tim0.init.auto_reload = US_SYSCNT_TO_TIMER(timeout_us);
    app_dual_tim_set_params(&p_params_tim0, APP_DUAL_TIM_ID_0);
    app_dual_tim_start(APP_DUAL_TIM_ID_0);
}

SECTION_RAM_CODE static void direct_data_transport_transmit_cfm_cb(void)
{
    uint32_t time = 0;
    
    app_io_toggle_pin(APP_IO_TYPE_NORMAL, APP_IO_PIN_29);
    s_idt_prepared = true;
    
    s_rtt_rx_time = SysTick->VAL;
    app_idt_direct_timer_stop();
    
    if (NULL != sp_idt_cb_func && NULL != sp_idt_cb_func->transmit_cfm_cb)
    {
        sp_idt_cb_func->transmit_cfm_cb();
    }

    if (s_rtt_rx_time < s_rtt_tx_time)
    {
        time = s_rtt_tx_time - s_rtt_rx_time;
    }
    else
    {
        time = (s_rtt_tx_time + 0xFFFFFF) - s_rtt_rx_time;
    }

    APP_LOG_INFO("RTT Time = %d us", (time/64));
}

SECTION_RAM_CODE static void app_idt_transmit_timeout_handler(app_dual_tim_evt_t *p_evt)
{
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
        app_idt_direct_timer_start(10 * s_random_nums[s_random_idx]);
        APP_LOG_INFO("Retry, delay = %d", s_random_nums[s_random_idx]);

        ISM_direct_transmit(s_idt_packet.length, s_idt_packet.data, direct_data_transport_transmit_cfm_cb);
        
        s_random_idx++;
        if (s_random_idx >= 10)
        {
            s_random_idx = 0;
        }
    }
}

SECTION_RAM_CODE static void direct_data_transport_receive_cb(uint8_t packet_length, uint8_t *packet_data)
{
    if (packet_data[0] != 0xA5)
    {
        ISM_direct_transmit(1, &s_ack_data, direct_data_transport_transmit_cfm_cb);
        if (NULL != sp_idt_cb_func && NULL != sp_idt_cb_func->receive_cb)
        {
            sp_idt_cb_func->receive_cb(packet_data, packet_length);
        }
    }
}

static void idt_init_delay_handler(void *p_arg)
{
    APP_LOG_INFO("ISM system prepared");

    s_idt_prepared = true;
}

static bool rand_num_generator(uint8_t *p_buf, uint8_t len)
{    
    uint8_t i = 0;    
    uint32_t random = 0;    
    hal_status_t status = HAL_OK;    
    rng_handle_t rng_handle = {0};        
    rng_handle.p_instance = RNG;    
    rng_handle.init.seed_mode  = RNG_SEED_FR0_S0;    
    rng_handle.init.lfsr_mode  = RNG_LFSR_MODE_128BIT;    
    rng_handle.init.out_mode   = RNG_OUTPUT_FR0_S0;    
    rng_handle.init.post_mode  = RNG_POST_PRO_NOT;        
    hal_rng_deinit(&rng_handle);    hal_rng_init(&rng_handle);        
    for(i = 0; i < len; i++)    
    {           
        status = hal_rng_generate_random_number(&rng_handle, NULL, &random);
        if(HAL_OK != status)        
        {            
            return false;        
        }
        *p_buf++ = (uint8_t)(random & 0xFF);    
    }    
    return true;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

SECTION_RAM_CODE void idt_data_send(uint16_t length, uint8_t *p_data)
{
    app_io_toggle_pin(APP_IO_TYPE_NORMAL, APP_IO_PIN_29);
    s_rtt_tx_time = SysTick->VAL;
    
    s_idt_prepared = false;

    s_idt_packet.length = length;
    memcpy(s_idt_packet.data, p_data, length);
    ISM_direct_transmit(length, p_data, direct_data_transport_transmit_cfm_cb);

    app_idt_direct_timer_start(10 * s_random_nums[s_random_idx]);
}

void app_idt_init(idt_info_t *p_idt_info, idt_cb_func_t *p_idt_callbacks)
{
    s_access_addr = p_idt_info->access_addr;
    s_channel_idx = p_idt_info->channel_idx;
    sp_idt_cb_func = p_idt_callbacks;
    
    app_timer_create(&s_idt_init_delay_timer, ATIMER_ONE_SHOT, idt_init_delay_handler);
    app_timer_start(s_idt_init_delay_timer, p_idt_info->delay_time, NULL);
    
    
    APP_LOG_INFO("ISM direct transport start");

    ISM_direct_init(s_channel_idx, s_access_addr);
    ISM_direct_receive(direct_data_transport_receive_cb);
    
    rand_num_generator(s_random_nums, 10);
    
    hal_deinit();
    SysTick->LOAD  = 0xFFFFFF;      /* set reload register */
    SysTick->VAL   = 0UL;           /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_ENABLE_Msk; 
    
    app_dual_tim_params_t p_params_tim0 = DUAL_TIM0_PARAM;
    app_dual_tim_init(&p_params_tim0, app_idt_transmit_timeout_handler);
}

bool idt_status_check(void)
{
    return s_idt_prepared;
}


