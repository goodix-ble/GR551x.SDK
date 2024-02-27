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
#include "gr_includes.h"
#include "hal_flash.h"
#include "board_SK.h"
#include "app_gpiote.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t      s_bd_addr[SYS_BD_ADDR_LEN] = {0x0c, 0x00, 0xcf, 0x3e, 0xcb, 0xea};
static app_gpiote_param_t s_gpiote_param;
static bool first_wkup = true;
/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void wkup_key_handler(app_io_evt_t *p_evt)
{
    if (p_evt->pin == APP_KEY_OK_PIN && \
        p_evt->type == APP_KEY_OK_IO_TYPE && !first_wkup)
    {
        gap_advertising_start();
    }
    else
    {
        first_wkup = false;
    }
}

static void wkup_key_init(void)
{
    s_gpiote_param.type          = APP_KEY_OK_IO_TYPE;
    s_gpiote_param.mode          = APP_KEY_TRIGGER_MODE;
    s_gpiote_param.pin           = APP_KEY_OK_PIN;
    s_gpiote_param.pull          = APP_IO_PULLUP;
    s_gpiote_param.io_evt_cb     = wkup_key_handler;

    app_gpiote_init(&s_gpiote_param, 1);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    wkup_key_init();
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}

bool is_enter_ultra_deep_sleep(void)
{
    if (APP_IO_PIN_RESET != app_io_read_pin(APP_IO_TYPE_AON, APP_KEY_OK_PIN))
    {
        return true;
    }

    return false;
}

