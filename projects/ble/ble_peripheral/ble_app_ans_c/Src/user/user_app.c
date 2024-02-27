/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "user_app.h"
#include "ans_c.h"
#include "utility.h"
#include "grx_sys.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_ANS_C"       /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                32                  /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                48                  /**< The advertising max interval (in units of 0.625 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                     /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;                /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                              /**< Advertising data. */
{
    0x0d,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'A', 'N', 'S', '_', 'C',

    // Device Service UUID
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_ALERT_NTF),
    HI_U16(BLE_ATT_SVC_ALERT_NTF),

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier:0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02,
    0x03,
};

static char *s_cat_id_str[AANS_C_CAT_ID_NB] = {"Simple.",
                                               "Email",
                                               "News",
                                               "Call",
                                               "Missed call",
                                               "SMS",
                                               "Voice mail",
                                               "Schedule",
                                               "High Prioritized Alert",
                                               "Instant Messaging",
                                              };

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t error_code;

    ble_gap_pair_enable(true);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL_MAX;
    s_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL_MIN;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 *@brief Print Supported New Alert Catrgory information.
 *****************************************************************************************
 */
static void sup_new_alert_cat_info_print(uint16_t cat_ids)
{
    if (0 == cat_ids)
    {
        APP_LOG_INFO("Supported New Alert Category List: NULL.");
    }
    else
    {
        APP_LOG_INFO("Supported New Alert Category List as follow:.");

        for (uint8_t i = 0; i < AANS_C_CAT_ID_NB; i++)
        {
            if (cat_ids & (1 << i))
            {
                APP_LOG_INFO("%s Supported.", s_cat_id_str[i]);
            }
        }
    }
}

/**
 *****************************************************************************************
 *@brief Print Supported Unread Alert Catrgory information.
 *****************************************************************************************
 */
static void sup_unread_alert_cat_info_print(uint16_t cat_ids)
{

    if (0 == cat_ids)
    {
        APP_LOG_INFO("Supported Unread Alert Category List: NULL.");
    }
    else
    {
        APP_LOG_INFO("Supported Unread Alert Category List as follow:.");

        for (uint8_t i = 0; i < AANS_C_CAT_ID_NB; i++)
        {
            if (cat_ids & (1 << i))
            {
                APP_LOG_INFO("%s Supported.", s_cat_id_str[i]);
            }
        }
    }
}

/**
 *****************************************************************************************
 *@brief Process Alsert Notification Service Client event.
 *****************************************************************************************
 */
static void ans_c_evt_process(ans_c_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case ANS_C_EVT_DISCOVERY_COMPLETE:
            ans_c_new_alert_notify_set(p_evt->conn_idx, true);
            break;

        case ANS_C_EVT_NEW_ALERT_NTF_SET_SUCCESS:
            ans_c_unread_alert_notify_set(p_evt->conn_idx, true);
            break;

        case ANS_C_EVT_UNREAD_ALERT_STA_NTF_SET_SUCCESS:
            ans_c_sup_new_alert_cat_read(p_evt->conn_idx);
            break;

        case ANS_C_EVT_SUP_NEW_ALERT_CAT_RECEIV:
            sup_new_alert_cat_info_print(p_evt->value.sup_new_alert_cat_ids);
            ans_c_sup_unread_alert_cat_read(p_evt->conn_idx);
            break;

        case ANS_C_EVT_SUP_UNREAD_ALERT_CAT_REC:
            sup_unread_alert_cat_info_print(p_evt->value.sup_new_alert_cat_ids);
            break;

        case  ANS_C_EVT_NEW_ALERT_RECEIVE:
            printf("[%02d]New %s ", p_evt->value.new_alert.alert_num,
                   s_cat_id_str[p_evt->value.new_alert.cat_id]);

            if (0 != p_evt->value.new_alert.alert_num)
            {
                printf("Last from %s", p_evt->value.new_alert.str_info);
            }

            printf("\r\n");
            break;

        case ANS_C_EVT_UNREAD_ALERT_RECEIVE:
            APP_LOG_INFO("[%02d]Unread %s.", p_evt->value.unread_alert.unread_num,
                         s_cat_id_str[p_evt->value.unread_alert.cat_id]);
            break;

        case ANS_C_EVT_CTRL_POINT_SET_SUCCESS:
            APP_LOG_INFO("Control Point has been sent.");
            break;

        default:
            break;
    }
}

static void app_connected_handler(uint8_t conn_idx, uint8_t status, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;
    
    if (BLE_SUCCESS == status)
    {
        APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                     p_param->peer_addr.addr[5],
                     p_param->peer_addr.addr[4],
                     p_param->peer_addr.addr[3],
                     p_param->peer_addr.addr[2],
                     p_param->peer_addr.addr[1],
                     p_param->peer_addr.addr[0]);
    }

    error_code = ans_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            if (p_evt->evt_status)
            {
                APP_LOG_DEBUG("Adverting started failed(0X%02X).", p_evt->evt_status);
            }
            break;
        
        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.params.adv_stop.reason == BLE_GAP_STOPPED_REASON_TIMEOUT)
            {
                APP_LOG_DEBUG("Advertising timeout.");
            }
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, p_evt->evt_status, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;
    }
}

void ble_app_init(void)
{
    sdk_err_t         error_code;
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
    APP_LOG_INFO("Alert Notification Service Client example started.");

    error_code = ans_client_init(ans_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

