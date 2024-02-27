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
#include "hal_flash.h"
#include "custom_config.h"
#include "app_pwr_mgmt.h"
#include "user_keyboard.h"

/*
 * DEFINENS
 *****************************************************************************************
 */

#define UART_RX_BUFFER_SIZE      10

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t          s_bd_addr[SYS_BD_ADDR_LEN] = {0x1A, 0x00, 0xCF, 0x3E, 0xCB, 0xEA};
static uint8_t                s_uart_rx_buffer[UART_RX_BUFFER_SIZE];
bool                          s_bond_erase_enable  = false;
static uart_rec_handler_reg_t s_uart_rec_handler_reg;

/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    bool                  send_data_flag = true;
    keyboard_media_data_t media_ctrl_data;
    memset(&media_ctrl_data, HID_KEYBOARD_RESERVED, sizeof(keyboard_media_data_t));

    if (BSP_KEY_UP_ID == key_id)
    {
        if (key_click_type == APP_KEY_SINGLE_CLICK)
        {
            media_ctrl_data.volume_up = 1;
        }
        else if (key_click_type == APP_KEY_DOUBLE_CLICK)
        {
            media_ctrl_data.previous_track = 1;
        }
        else if (key_click_type == APP_KEY_LONG_CLICK)
        {
            media_ctrl_data.play_pause = 1;
        }
    }
    else if (BSP_KEY_DOWN_ID == key_id)
    {
        if (key_click_type == APP_KEY_SINGLE_CLICK)
        {
            media_ctrl_data.volume_down = 1;
        }
        else if (key_click_type == APP_KEY_DOUBLE_CLICK)
        {
            media_ctrl_data.next_track = 1;
        }
        else if (key_click_type == APP_KEY_LONG_CLICK)
        {
        }
    }
    else
    {
        send_data_flag = false;
    }

    if (send_data_flag)
    {
        user_keyboard_media_send_data(0, &media_ctrl_data);
        memset(&media_ctrl_data, HID_KEYBOARD_RESERVED, sizeof(keyboard_media_data_t));
        user_keyboard_media_send_data(0, &media_ctrl_data);
    }
}

void app_uart_evt_handler(app_uart_evt_t *p_evt)
{
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        if(s_uart_rec_handler_reg != NULL)
        {
            s_uart_rec_handler_reg(p_evt);
        }

        app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_bond_state_set(void)
{
    app_io_pin_state_t pin_state = APP_IO_PIN_SET;
    app_io_init_t      io_init   = APP_IO_DEFAULT_CONFIG;

    io_init.pull = APP_IO_PULLUP;
    io_init.pin  = APP_KEY_UP_PIN;
    io_init.mux  = APP_KEY_UP_MUX;
    app_io_init(APP_KEY_UP_IO_TYPE, &io_init);

    pin_state = app_io_read_pin(APP_KEY_UP_IO_TYPE, APP_KEY_UP_PIN);

    if (APP_IO_PIN_RESET == pin_state)
    {
        s_bond_erase_enable = true;
    }
    else
    {
        s_bond_erase_enable = false;
    }
    app_io_deinit(APP_KEY_UP_IO_TYPE, APP_KEY_UP_PIN);
}

bool ble_bond_state_get(void)
{
    return s_bond_erase_enable;
}

uint8_t *uart_rec(void)
{
    return s_uart_rx_buffer;
}

void app_periph_init(uart_rec_handler_reg_t p_uart_handler)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    s_uart_rec_handler_reg = p_uart_handler;
    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    pwr_mgmt_mode_set(PMR_MGMT_IDLE_MODE);
}



