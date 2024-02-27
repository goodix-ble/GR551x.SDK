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
#include "gr_includes.h"
#include "app_assert.h"
#include "app_log.h"
#include "cts_c.h"
#include "cts.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "app_pwr_mgmt.h"
#include "board_SK.h"
#include "uds.h"
#include "wss_db.h"

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
static const uint8_t        s_bd_addr[SYS_BD_ADDR_LEN] = {0x1F, 0x00, 0xCF, 0x3E, 0xCB, 0xEA};
static uint8_t              s_uart_rx_buffer[UART_RX_BUFFER_SIZE];

extern wss_db_user_buf_t    s_wss_db_user_buf;
extern wss_rec_cmd_queue_t  s_wss_db_rec_cmd_queue;

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
    if (BSP_KEY_UP_ID == key_id)
    {
        uint8_t cur_user_id;
#if defined(PTS_AUTO_TEST)
        cur_user_id = 0x00;
#else
        cur_user_id = uds_get_cur_user_index(0);
#endif
        s_wss_db_user_buf.user_id = cur_user_id;
        s_wss_db_user_buf.user_data[cur_user_id].db_change_incr_val++;
        s_wss_db_user_buf.user_data[cur_user_id].age++;
        wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
        
        uds_db_change_incr_val_send(0, cur_user_id);
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}


