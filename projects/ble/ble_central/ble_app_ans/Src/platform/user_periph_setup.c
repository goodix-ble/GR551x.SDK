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
#include "user_app.h"
#include "ans.h"
#include "board_SK.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "app_assert.h"
#include "app_log.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern uint8_t          g_unread_alert_num[ANS_CAT_ID_NB];
extern uint8_t          g_new_alert_num[ANS_CAT_ID_NB];
extern new_alert_info_t g_new_alert_record[ANS_CAT_ID_NB];

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x00, 0x01, 0xcf, 0x3e, 0xcb, 0xea};


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    ans_new_alert_t    new_alert;
    ans_unread_alert_t unread_alert;

    if (BSP_KEY_OK_ID == key_id)
    {
        if (APP_KEY_SINGLE_CLICK == key_click_type)
        {
            APP_LOG_DEBUG("One Email received.");

            g_new_alert_num[ANS_CAT_ID_EMAIL]++;
            g_unread_alert_num[ANS_CAT_ID_EMAIL]++;
            g_new_alert_record[ANS_CAT_ID_EMAIL].length = 4;
            memcpy(g_new_alert_record[ANS_CAT_ID_EMAIL].str_info, "Tony", 4);

            new_alert.cat_id     = ANS_CAT_ID_EMAIL;
            new_alert.alert_num  = g_new_alert_num[ANS_CAT_ID_EMAIL];
            new_alert.length     = 4;
            memcpy(new_alert.str_info, "Tony", 4);
            ans_new_alert_send(0, &new_alert);

            unread_alert.cat_id     = ANS_CAT_ID_EMAIL;
            unread_alert.unread_num = g_unread_alert_num[ANS_CAT_ID_EMAIL];
            ans_unread_alert_send(0, &unread_alert);
        }
        else if (APP_KEY_DOUBLE_CLICK == key_click_type)
        {
            APP_LOG_DEBUG("One Missed call.");

            g_new_alert_num[ANS_CAT_ID_MISSED_CALL]++;
            g_unread_alert_num[ANS_CAT_ID_MISSED_CALL]++;
            g_new_alert_record[ANS_CAT_ID_MISSED_CALL].length = 3;
            memcpy(g_new_alert_record[ANS_CAT_ID_MISSED_CALL].str_info, "Tom", 3);

            new_alert.cat_id     = ANS_CAT_ID_MISSED_CALL;
            new_alert.alert_num  = g_new_alert_num[ANS_CAT_ID_MISSED_CALL];
            new_alert.length     = 3;
            memcpy(new_alert.str_info, "Tom", 3);
            ans_new_alert_send(0, &new_alert);

            unread_alert.cat_id     = ANS_CAT_ID_MISSED_CALL;
            unread_alert.unread_num = g_unread_alert_num[ANS_CAT_ID_MISSED_CALL];
            ans_unread_alert_send(0, &unread_alert);
        }
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}
