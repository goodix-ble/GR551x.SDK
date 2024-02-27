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
#include "pass.h"
#include "board_SK.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "app_log.h"
#include "app_assert.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x03, 0x01, 0xcf, 0x3e, 0xcb, 0xea};
static uint8_t       s_alert_status_change_count;     /**< For PTS use. */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    if (BSP_KEY_OK_ID == key_id)
    {
        if (APP_KEY_SINGLE_CLICK == key_click_type)
        {
            uint8_t current_ringer_state = pass_ringer_setting_get();

            if (PASS_RINGER_SET_NORMAL == current_ringer_state)
            {
                pass_ringer_setting_set(0, PASS_RINGER_SET_SILENT);
                APP_LOG_DEBUG("Already set Silent Mode.");
            }

            if (PASS_RINGER_SET_SILENT == current_ringer_state)
            {
                pass_ringer_setting_set(0, PASS_RINGER_SET_NORMAL);
                APP_LOG_DEBUG("Already set Normal Mode.");
            }
        }
        else if (APP_KEY_DOUBLE_CLICK == key_click_type)
        {
            switch (s_alert_status_change_count)
            {
                case 0:
                    pass_alert_status_set(0, PASS_NO_STATE_ACTIVE);
                    APP_LOG_DEBUG("Already set no alerts.");
                    break;

                case 1:
                    pass_alert_status_set(0, PASS_RINGER_ACTIVE);
                    APP_LOG_DEBUG("Already set Ringer State is active.");
                    break;

                case 2:
                    pass_alert_status_set(0, PASS_DISPLAY_ALERT_ACTIVE);
                    APP_LOG_DEBUG("Already set Display State is active.");
                    break;

                case 3:
                    pass_alert_status_set(0, PASS_VIBRATE_ACTIVE);
                    APP_LOG_DEBUG("Already set Vibrating State is active.");
                    break;

                default:
                    break;
            }

            if (4 == ++s_alert_status_change_count)
            {
                s_alert_status_change_count = 0;
            }
        }
    }

}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}


