/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include "user_periph_setup.h"

#include "heartrate_slave.h"
#include "keyboard_slave.h"

#include "app_log.h"
#include "app_error.h"



#define BLE_GAP_DEVICE_NAME         "Goodix_Multi_Slave"

/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static bool s_ble_stack_init = false;

static void ble_evt_handler(const ble_evt_t *p_evt)
{
    if (BLE_COMMON_EVT_STACK_INIT == p_evt->evt_id)
    {
        sdk_err_t       ret;
        sdk_version_t   version;

        sys_sdk_verison_get(&version);
        APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)", version.major, version.minor, version.build, version.commit_id);
        APP_LOG_INFO("Goodix Multi Slave application example started.");

        ble_gap_pair_enable(true);
        ble_gap_privacy_params_set(900, true);
        ret = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)BLE_GAP_DEVICE_NAME, strlen(BLE_GAP_DEVICE_NAME));
        APP_ERROR_CHECK(ret);

        s_ble_stack_init = true;
        return;
    }

    if (BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ == p_evt->evt_id)
    {
        ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
        return;
    }

    heartrate_slave_ble_evt_handler(p_evt);
    keyboard_slave_ble_evt_handler(p_evt);
}

int main (void)
{
    app_periph_init();

    // Initialize BLE Stack.
    ble_stack_init(ble_evt_handler, &heaps_table);
    while(!s_ble_stack_init);

    heartrate_slave_init();
    keyboard_slave_init();

    // loop
    while (1)
    {
        app_log_flush();
        pwr_mgmt_schedule();
    }
}
