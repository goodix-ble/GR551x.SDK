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
#include "grx_sys.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_Tem"     /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL          32               /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_FAST_MAX_INTERVAL          48               /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL          160              /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL          400              /**< The slow advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0                /**< The advertising timeout in units of seconds. */
#define MIN_CONN_INTERVAL                  320              /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  520              /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0                /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   400              /**< Connection supervisory timeout (4 seconds). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                     /**< Advertising data. */
{
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    0x01,
    0x01,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02, 0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                 /**< Scan responce data. */
{
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'T', 'e', 'm',
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(false);

    s_gap_adv_param.adv_intv_max  = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min  = APP_ADV_FAST_MIN_INTERVAL;
    s_gap_adv_param.adv_mode      = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map      = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode     = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol    = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    uint8_t dev_name[32];
    uint16_t dev_name_len = 32;
    error_code = ble_gap_device_name_get(dev_name, &dev_name_len);
    APP_ERROR_CHECK(error_code);

    if (!strcmp((const char *)dev_name, BLE_GAP_DEVNAME_DEFAULT))
    {
        // Set the default Device Name.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, (uint8_t const *)DEVICE_NAME, strlen(DEVICE_NAME));
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        // Set the Device Name is writable from the peer.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, NULL, 0);
        APP_ERROR_CHECK(error_code);
    }

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration     = 0;
    s_gap_adv_time_param.max_adv_evt  = 0;
}


/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    error_code = ble_gap_adv_start(conn_idx, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

static void app_connected_handler(const ble_gap_evt_connected_t *p_param)
{
    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);
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

        case BLE_GAPM_EVT_CH_MAP_SET:
            break;
        
        case BLE_GAPM_EVT_WHITELIST_SET:
            break;

        case BLE_GAPM_EVT_PER_ADV_LIST_SET:
            break;

        case BLE_GAPM_EVT_PRIVACY_MODE_SET:
            break;

        case BLE_GAPM_EVT_LEPSM_REGISTER:
            break;

        case BLE_GAPM_EVT_LEPSM_UNREGISTER:
            break;

        case BLE_GAPM_EVT_SCAN_REQUEST:
            break;

        case BLE_GAPM_EVT_ADV_DATA_UPDATE:
            break;

        case BLE_GAPM_EVT_SCAN_START:
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            break;

        case BLE_GAPM_EVT_SYNC_ESTABLISH:
            break;

        case BLE_GAPM_EVT_SYNC_STOP:
            break;

        case BLE_GAPM_EVT_SYNC_LOST:
            break;

        case BLE_GAPM_EVT_READ_RSLV_ADDR:
            break;

        case BLE_GAPC_EVT_PHY_UPDATED:
            break;

        case BLE_GAPM_EVT_DEV_INFO_GOT:
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(&(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            break;

        case BLE_GAPC_EVT_CONNECT_CANCLE:
            break;

        case BLE_GAPC_EVT_AUTO_CONN_TIMEOUT:
            break;

        case BLE_GAPC_EVT_PEER_NAME_GOT:
            break;

        case BLE_GAPC_EVT_CONN_INFO_GOT:
            break;

        case BLE_GAPC_EVT_PEER_INFO_GOT:
            break;

        case BLE_GAPC_EVT_DATA_LENGTH_UPDATED:
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            break;

        case BLE_GATT_COMMON_EVT_PRF_REGISTER:
            break;

        case BLE_GATTS_EVT_READ_REQUEST:
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            break;

        case BLE_GATTS_EVT_PREP_WRITE_REQUEST:
            break;

        case BLE_GATTS_EVT_NTF_IND:
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            break;

        case BLE_GATTC_EVT_SRVC_BROWSE:
            break;

        case BLE_GATTC_EVT_PRIMARY_SRVC_DISC:
            break;

        case BLE_GATTC_EVT_INCLUDE_SRVC_DISC:
            break;

        case BLE_GATTC_EVT_CHAR_DISC:
            break;

        case BLE_GATTC_EVT_CHAR_DESC_DISC:
            break;

        case BLE_GATTC_EVT_READ_RSP:
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            break;

        case BLE_GATTC_EVT_NTF_IND:
            if (BLE_GATT_INDICATION == p_evt->evt.gattc_evt.params.ntf_ind.type)
            {
                ble_gattc_indicate_cfm(p_evt->evt.gattc_evt.index, p_evt->evt.gattc_evt.params.ntf_ind.handle);
            }
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            break;

        case BLE_SEC_EVT_KEY_PRESS_NTF:
            break;

        case BLE_SEC_EVT_KEY_MISSING:
            break;

        case BLE_L2CAP_EVT_CONN_REQ:
            break;

        case BLE_L2CAP_EVT_CONN_IND:
            break;

        case BLE_L2CAP_EVT_ADD_CREDITS_IND:
            break;

        case BLE_L2CAP_EVT_DISCONNECTED:
            break;

        case BLE_L2CAP_EVT_SDU_RECV:
            break;

        case BLE_L2CAP_EVT_SDU_SEND:
            break;

        case BLE_L2CAP_EVT_ADD_CREDITS_CPLT:
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
    APP_LOG_INFO("Template application example started.");

    services_init();
    gap_params_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

