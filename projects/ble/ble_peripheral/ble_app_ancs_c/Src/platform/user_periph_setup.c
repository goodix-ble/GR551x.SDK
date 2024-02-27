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
#include "board_SK.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "uart_simu_key_init.h"
#include "ancs_protocol.h"
#include "app_assert.h"
#include "app_log.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**< Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x00, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    uint16_t uid;
    if (key_click_type == APP_KEY_SINGLE_CLICK)
    {
        if (VIR_KEY_OK_ID == key_id)
        {
                pwr_mgmt_mode_set(PMR_MGMT_IDLE_MODE);
                uid = ancs_get_uid();
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_TITLE);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_SUBTITLE);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_MESSAGE);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_DATE);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL);
                ancs_notify_attr_get(uid, ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL);
            }
            else if (VIR_KEY_LEFT_ID  == key_id)
            {
                APP_LOG_INFO("pressed key left");
                uid = ancs_get_uid();
                if (uid > 0)
                {
                    ancs_action_perform(uid, ACTION_ID_NEGATIVE);
                }
            }
            else if (VIR_KEY_RIGHT_ID == key_id)
            {
                APP_LOG_INFO("pressed key right");
                uid = ancs_get_uid();
                ancs_action_perform(uid, ACTION_ID_POSITIVE);
            }
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    uart_simu_key_init();
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}



