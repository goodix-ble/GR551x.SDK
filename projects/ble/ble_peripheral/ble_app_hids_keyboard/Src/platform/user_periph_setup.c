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
#include "board_SK.h"
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
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x1A, 0x00, 0xCF, 0x3E, 0xCB, 0xEA};
static uint8_t        s_uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t               now_connect_indx     = 0;
bool                  start_pair_flag      = false;
bool                  pair_successful_flag = false;
bool                  s_bond_erase_enable  = false;

//~!@#$%^&*()_+{}|:"<>?
//`-=[]\;',./
static const uint8_t symbol_ascii_tab[] =
{
    0x7E, 0x21, 0x40, 0x23, 0x24, 0x25, 0x5E, 0x26, 0x2A, 0x28, 0x29, 0x5F, 0x2B, 0x7B, 0x7D, 0x7C, 0x3A,
    0x22, 0x3C, 0x3E, 0x3F, 0x60, 0x2D, 0x3D, 0x5B, 0x5D, 0x5C, 0x3B, 0x27, 0x2C, 0x2E, 0x2F, 0x20, 0x7F,
};

static const uint8_t symbol_hid_tab[] =
{
    HID_KEYBOARD_GRV_ACCENT,HID_KEYBOARD_1,HID_KEYBOARD_2,HID_KEYBOARD_3,HID_KEYBOARD_4,HID_KEYBOARD_5,\
    HID_KEYBOARD_6,HID_KEYBOARD_7,HID_KEYBOARD_8,HID_KEYBOARD_9,HID_KEYBOARD_0,HID_KEYBOARD_MINUS,\
    HID_KEYBOARD_EQUAL,HID_KEYBOARD_LEFT_BRKT,HID_KEYBOARD_RIGHT_BRKT,HID_KEYBOARD_BACK_SLASH,HID_KEYBOARD_SEMI_COLON,\
    HID_KEYBOARD_SGL_QUOTE,HID_KEYBOARD_COMMA,HID_KEYBOARD_DOT, HID_KEYBOARD_FWD_SLASH,\

    HID_KEYBOARD_GRV_ACCENT,HID_KEYBOARD_MINUS,HID_KEYBOARD_EQUAL,HID_KEYBOARD_LEFT_BRKT,HID_KEYBOARD_RIGHT_BRKT,\
    HID_KEYBOARD_BACK_SLASH,HID_KEYBOARD_SEMI_COLON,HID_KEYBOARD_SGL_QUOTE,HID_KEYBOARD_COMMA,\
    HID_KEYBOARD_DOT,HID_KEYBOARD_FWD_SLASH,HID_KEYBOARD_SPACEBAR,HID_KEYBOARD_DELETE
};

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
    else if (BSP_KEY_OK_ID == key_id)
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
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t          pair_code = 0x00;

    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        if (start_pair_flag)// start pairing
        {
            pair_code += (s_uart_rx_buffer[0] - 0x30) * 100000;
            pair_code += (s_uart_rx_buffer[1] - 0x30) * 10000;
            pair_code += (s_uart_rx_buffer[2] - 0x30) * 1000;
            pair_code += (s_uart_rx_buffer[3] - 0x30) * 100;
            pair_code += (s_uart_rx_buffer[4] - 0x30) * 10;
            pair_code += (s_uart_rx_buffer[5] - 0x30);
            APP_LOG_INFO("Received Pass code: %d", pair_code);

            memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept   = true;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((pair_code & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((pair_code & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((pair_code & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((pair_code & 0xFF000000) >> 24);
            ble_sec_enc_cfm(now_connect_indx, &cfm_enc);

            start_pair_flag = false;
        }
        if (pair_successful_flag)//paired successful
        {
            keyboard_keys_data_t data;
            memset(&data, HID_KEYBOARD_RESERVED, sizeof(keyboard_keys_data_t));

            if (s_uart_rx_buffer[0] >=0x30 && s_uart_rx_buffer[0]<= 0x39)// 0-9
            {
                if (s_uart_rx_buffer[0] ==0x30)
                {
                    data.key_code[0] = HID_KEYBOARD_0;
                }
                else
                {
                    data.key_code[0] = (s_uart_rx_buffer[0]-0x31) + HID_KEYBOARD_1;
                }
            }
            else if (s_uart_rx_buffer[0] >=0x41 && s_uart_rx_buffer[0]<= 0x59)// A-Z
            {
                data.key_code[0] = (s_uart_rx_buffer[0]-0x41) + HID_KEYBOARD_A;
                data.left_shift = 0x01;
            }
            else if (s_uart_rx_buffer[0] >=0x61 && s_uart_rx_buffer[0]<= 0x7a)// a-z
            {
                data.key_code[0] = (s_uart_rx_buffer[0]-0x61) + HID_KEYBOARD_A;
            }
            else if (s_uart_rx_buffer[0] == 0x0D)//enter key
            {
                data.key_code[0] = HID_KEYBOARD_RETURN;
            }
            else
            {
                for (uint8_t i=0; i < sizeof(symbol_ascii_tab); i++)
                {
                    if (s_uart_rx_buffer[0] == symbol_ascii_tab[i])
                    {
                        data.key_code[0] = symbol_hid_tab[i];
                        if (i < 21)
                        {
                            data.left_shift = 0x01;
                        }
                        break;
                    }
                }
            }
            user_keyboard_keys_send_data(0, &data);
            memset(&data, HID_KEYBOARD_RESERVED, sizeof(keyboard_keys_data_t));
            user_keyboard_keys_send_data(0, &data);
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

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    pwr_mgmt_mode_set(PMR_MGMT_IDLE_MODE);
}


