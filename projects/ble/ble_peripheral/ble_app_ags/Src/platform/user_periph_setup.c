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
#include "app_error.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "app_pwr_mgmt.h"
#include "board_SK.h"
#include "user_app.h"
#include "user_gadget.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define ACTIVE_WORD_LED BSP_LED_NUM_0

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x21, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 *****************************************************************************************
 */
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
/**
 *****************************************************************************************
 * @brief Speechdata event handler.
 *
 * @param[in] p_name: Pointer to the state name of Speechdata directive.
 * @param[in] p_data: Pointer to the data.
 *****************************************************************************************
 */
static void speechdata_evt_handler(alexaGadgetSpeechData_SpeechmarksDirectivePayloadProto *p_data)
{
}
#endif

#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
/**
 *****************************************************************************************
 * @brief State update event handler.
 *
 * @param[in] p_name: Pointer to the state name of StateUpdate directive.
 * @param[in] p_data: Pointer to the data.
 *****************************************************************************************
 */
static void state_update_evt_handler(const char *p_name, const char *p_data)
{
    if (strcmp(p_name, "wakeword") == 0 && (strcmp(p_data, "active") == 0))
    {
        APP_LOG_DEBUG("Wakeword is active.");
        bsp_led_open(ACTIVE_WORD_LED);
    }
    else if (strcmp(p_name, "wakeword") == 0 && (strcmp(p_data, "cleared") == 0))
    {
        APP_LOG_DEBUG("Wakeword is cleared.");
        bsp_led_close(ACTIVE_WORD_LED);
    }
}
#endif

#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
/**
 *****************************************************************************************
 * @brief Custom event handler.
 *
 * @param[in] p_name: Pointer to the name.
 * @param[in] p_data: Pointer to the data.
 * @param[in] len:    The length of the data.
 *****************************************************************************************
 */
static void custom_evt_handler(const char *p_name, const char *p_data, uint16_t len)
{
    /* Process the data accroding to the directive name. */
}

/**
 *****************************************************************************************
 * @brief Send custom event.
 *
 * @param[in] conn_idx: The connection index.
 *****************************************************************************************
 */
static void custom_test_event_send(uint8_t conn_idx)
{
    /* examples:
       sdk_err_t  error_code;
       char *p_name      = "TestName";
       char *p_payload   = "{\"finished\":\"yes\", \"remainingBatteryPercent\" : 80}";
       error_code = user_gadget_custom_event_send(conn_idx, (uint8_t *)p_name, (uint8_t *)p_payload);
       APP_ERROR_CHECK(error_code); */
}
#endif

/**
 *****************************************************************************************
 * @brief User gadget event handler.
 *
 * @param[in] p_evt: Pointer to the user gadget event.
 *****************************************************************************************
 */
static void user_gadget_event_process(user_gadget_evt_t *p_evt)
{
    switch (p_evt->user_gadget_evt_type)
    {
        case USER_GADGET_EVT_HANDSHAKE_DONE:
            APP_LOG_DEBUG("Handshake procedure is completed.");
            break;

        case USER_GADGET_EVT_SET_ALERT:
            APP_LOG_DEBUG("Set Alert.");
            break;

        case USER_GADGET_EVT_DELETE_ALERT:
            APP_LOG_DEBUG("Delete Alert.");
            break;

        case USER_GADGET_EVT_SET_INDICATOR:
            APP_LOG_DEBUG("Set Indicator.");
            break;

        case USER_GADGET_EVT_CLEAR_INDICATOR:
            APP_LOG_DEBUG("Clear Indicator.");
            break;

        case USER_GADGET_EVT_TEMPO:
            APP_LOG_DEBUG("Tempo.");
            break;

        case USER_GADGET_EVT_SPEECHMARKS:
            APP_LOG_DEBUG("Speechmarks.");
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
            speechdata_evt_handler(p_evt->data.p_speechmarks_drt_payload);        
#endif
            break;

        case USER_GADGET_EVT_STATE_UPDATE:
            APP_LOG_DEBUG("State Update.");
#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
            for (uint8_t i = 0; i < p_evt->data.p_state_update_drt_payload->states_count; ++i) 
            {
                APP_LOG_DEBUG("name:%s, value:%s\n", p_evt->data.p_state_update_drt_payload->states[i].name,
                                                     p_evt->data.p_state_update_drt_payload->states[i].value);
                state_update_evt_handler(p_evt->data.p_state_update_drt_payload->states[i].name,
                                         p_evt->data.p_state_update_drt_payload->states[i].value);
            }
#endif
        break;

    case USER_GADGET_EVT_CUSTOM:
        APP_LOG_DEBUG("Custom Event.");
#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
        custom_evt_handler(p_evt->data.CustomDirectivePayloadProto.name, 
                           p_evt->data.CustomDirectivePayloadProto.payload,
                           p_evt->data.CustomDirectivePayloadProto.size);
#endif
        break;

    default:
        APP_LOG_DEBUG("Unknown event: %d\n", p_evt->user_gadget_evt_type);
        break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    sdk_err_t error_code;

    if (BSP_KEY_OK_ID == key_id)
    {
        if (APP_KEY_SINGLE_CLICK == key_click_type)
        {
            error_code = ble_gap_adv_stop(0);
            
            if (error_code)
            {
                error_code = ble_gap_bond_devs_clear();
                APP_ERROR_CHECK(error_code);
                
                error_code = ble_gap_disconnect(0);
                if (error_code)
                {
                    app_adv_start();
                }
            }
        }
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    board_init();
    user_gadget_evt_handler_init(user_gadget_event_process);
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
}


