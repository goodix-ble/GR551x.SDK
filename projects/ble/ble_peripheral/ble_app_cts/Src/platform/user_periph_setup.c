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
#include "cts.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "board_SK.h"
#include "app_assert.h"
#include "app_log.h"
#include "cts_c.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_RX_BUFFER_SIZE         244

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x05, 0x00, 0xcf, 0x3e, 0xcb, 0xea};
static uint8_t        s_sec_change = 0;
static uint8_t        s_uart_rx_buffer[UART_RX_BUFFER_SIZE];
/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt)
{
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        cts_c_data_parse(s_uart_rx_buffer, p_evt->data.size);
        app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    }
}

void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    cts_adj_info_t  cts_adj_info;

    if (BSP_KEY_UP_ID == key_id && APP_KEY_SINGLE_CLICK == key_click_type)
    {
        APP_LOG_DEBUG("Change Current Time value manually.");
        cts_adj_info.adjust_reason                  = CTS_AR_MAUAL_TIME_UPDATE;
        cts_adj_info.day_date_time.day_of_week      = CTS_WEEK_THURSDAT;
        cts_adj_info.day_date_time.date_time.year   = 2019;
        cts_adj_info.day_date_time.date_time.month  = 2;
        cts_adj_info.day_date_time.date_time.day    = 28;
        cts_adj_info.day_date_time.date_time.hour   = 22;
        cts_adj_info.day_date_time.date_time.min    = 22;
        cts_adj_info.day_date_time.date_time.sec    = s_sec_change;
        cts_adj_info.day_date_time.fractions_256    = 0;
        cts_cur_time_adjust(&cts_adj_info);
        s_sec_change += 10;
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}


