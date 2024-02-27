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
#include "user_mouse.h"
#include "uart_simu_key_init.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define XY_MOVEMENT_SPEED                  50
#define WHEEL_MOVEMENT_SPEED               5

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x07, 0x00, 0xcf, 0x3e, 0xcb, 0xea};
bool                  s_bond_erase_enable = false;

static void key_continue_press_task(uint8_t key_id)
{
    bool         send_data_flag = true;
    mouse_data_t data;
    memset(&data, 0x00, sizeof(mouse_data_t));

    switch (key_id)
    {
        case VIR_KEY_UP_ID:
            data.y_delta = -XY_MOVEMENT_SPEED;
            break;

        case VIR_KEY_DOWN_ID:
            data.y_delta = XY_MOVEMENT_SPEED;
            break;

        case VIR_KEY_LEFT_ID:
             data.x_delta = -XY_MOVEMENT_SPEED;
            break;

        case VIR_KEY_RIGHT_ID:
            data.x_delta = XY_MOVEMENT_SPEED;
            break;
        default:
            send_data_flag = false;
            break;
    }

    if (send_data_flag)
    {
        user_mouse_data_send(0, &data);
    }
}

static void key_single_press_task(uint8_t key_id)
{
    bool         send_data_flag    = true;
    bool         button_press_flag = true;
    mouse_data_t data;
    
    memset(&data, 0x00, sizeof(mouse_data_t));
    switch (key_id)
    {
        case VIR_KEY_UP_ID:
            button_press_flag = false;
            data.wheel_delta = WHEEL_MOVEMENT_SPEED;
            break;

        case VIR_KEY_DOWN_ID:
            button_press_flag = false;
            data.wheel_delta = -WHEEL_MOVEMENT_SPEED;
            break;

        case VIR_KEY_LEFT_ID:
             data.left_button_press = true;
            break;

        case VIR_KEY_OK_ID:
            data.middle_button_press = true;
            break;

        case VIR_KEY_RIGHT_ID:
            data.right_button_press = true;
            break;
        default:
            send_data_flag = false;
            break;
    }

    if (send_data_flag)
    {
        user_mouse_data_send(0, &data);
        if (button_press_flag)//release button
        {
            memset(&data, 0x00, sizeof(mouse_data_t));
            user_mouse_data_send(0, &data);
        }

    }
}

static void key_double_press_task(uint8_t key_id)
{
    bool         send_data_flag = true;
    media_data_t data;
    memset(&data, 0x00, sizeof(media_data_t));

    switch (key_id)
    {
        case VIR_KEY_UP_ID:
            data.volume_up = 0x01;
            break;

        case VIR_KEY_DOWN_ID:
            data.volume_down = 0x01;
            break;

        case VIR_KEY_OK_ID:
            data.play_pause = 0x01;
            break;

        case VIR_KEY_RIGHT_ID:
            data.next_track = 0x01;
            break;

        case VIR_KEY_LEFT_ID:
            data.previous_track = 0x01;
            break;

        default:
            send_data_flag = false;
            break;
    }

    if (send_data_flag)
    {
        user_mouse_media_send(0, &data);
        memset(&data, 0x00, sizeof(media_data_t));
        user_mouse_media_send(0, &data);
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

void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    if (key_click_type == APP_KEY_SINGLE_CLICK)
    {
        key_single_press_task(key_id);
    }
    else if (key_click_type == APP_KEY_CONTINUE_CLICK)
    {
        key_continue_press_task(key_id);
    }
    else if (key_click_type == APP_KEY_DOUBLE_CLICK)
    {
        key_double_press_task(key_id);
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    uart_simu_key_init();
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}

