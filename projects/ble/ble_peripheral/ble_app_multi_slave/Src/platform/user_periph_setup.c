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

#include "gr_includes.h"
#include "board_SK.h"

#include "keyboard_slave.h"

/*
 * DEFINENS
 *****************************************************************************************
 */
#define UART_RX_BUFFER_SIZE      10

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_uart_rx_buffer[UART_RX_BUFFER_SIZE];

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
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt)
{
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        keyboard_keys_data_t data = {0};

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
        keyboard_slave_keys_send_data(&data);
        memset(&data, HID_KEYBOARD_RESERVED, sizeof(keyboard_keys_data_t));
        keyboard_slave_keys_send_data(&data);
        app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    }
}

void app_periph_init(void)
{
    board_init();
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);

    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
}


