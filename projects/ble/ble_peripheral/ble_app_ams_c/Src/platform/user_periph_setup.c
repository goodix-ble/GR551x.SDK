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
#include "ams_c.h"
#include "custom_config.h"
#include "uart_simu_key_init.h"
#include "app_assert.h"
#include "app_log.h"
#include "app_error.h"
#include "board_SK.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**< Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x18, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void cmd_process(ams_c_cmd_id_t cmd_id)
{
    if (ams_c_cmd_enable_check(cmd_id))
    {
        sdk_err_t error_code;
        error_code = ams_c_cmd_send(0, cmd_id);
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        APP_LOG_INFO("The command is not available.");
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    ams_c_cmd_id_t cmd_id;

    if (VIR_KEY_OK_ID == key_id )
    {
        if (APP_KEY_SINGLE_CLICK == key_click_type)
        {
            APP_LOG_INFO("Media play.");
            cmd_id = AMS_CMD_ID_PLAY;
            cmd_process(cmd_id);
        }
        else if (APP_KEY_DOUBLE_CLICK == key_click_type)
        {
            APP_LOG_INFO("Media pause.");
            cmd_id = AMS_CMD_ID_PAUSE;
            cmd_process(cmd_id);
        }
        else if (APP_KEY_LONG_CLICK == key_click_type)
        {
            APP_LOG_INFO("Toggle play pause.");
            cmd_id = AMS_CMD_ID_TOGGLE_PLAY_PAUSE;
            cmd_process(cmd_id);
        }
    }
    else if (VIR_KEY_RIGHT_ID == key_id)
    {
        APP_LOG_INFO("Next track.");
        cmd_id = AMS_CMD_ID_NEXT_TRACK;
        cmd_process(cmd_id);
    }
    else if (VIR_KEY_LEFT_ID == key_id)
    {
        APP_LOG_INFO("Previous track.");
        cmd_id = AMS_CMD_ID_PREVIOUS_TRACK;
        cmd_process(cmd_id);
    }
    else if (VIR_KEY_UP_ID == key_id)
    {
        APP_LOG_INFO("Volume up.");
        cmd_id = AMS_CMD_ID_VOLUME_UP;
        cmd_process(cmd_id);
    }
    else if (VIR_KEY_DOWN_ID == key_id)
    {
        APP_LOG_INFO("Volume down.");
        cmd_id = AMS_CMD_ID_VOLUME_DOWN;
        cmd_process(cmd_id);
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    uart_simu_key_init();
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}
