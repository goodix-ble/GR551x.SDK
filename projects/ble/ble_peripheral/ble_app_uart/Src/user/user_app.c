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
#include "transport_scheduler.h"
#include "user_periph_setup.h"
#include "gus.h"
#include "grx_sys.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"


/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                     "Goodix_UART"   /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL       32              /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_FAST_MAX_INTERVAL       48              /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL       160             /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL       400             /**< The slow advertising max interval (in units of 0.625 ms). */
#define MAX_MTU_DEFUALT                 247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                 247              /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT             2              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT            251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT             2120            /**< Defualt maximum packet transmission time. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x11, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    GUS_SERVICE_UUID,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7, 
    0x04,
    // Goodix specific adv data
    0x02,0x03,
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x0c, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'U', 'A', 'R', 'T',
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

    s_gap_adv_param.adv_intv_max = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_FAST_MIN_INTERVAL;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    error_code = ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}

/**
 *****************************************************************************************
 * @brief Function for process gus service event
 *
 * @param[in] p_evt: Pointer to gus event stucture.
 *****************************************************************************************
 */
static void gus_service_process_event(gus_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case GUS_EVT_TX_PORT_OPENED:
            transport_flag_set(GUS_TX_NTF_ENABLE, true);
            break;

        case GUS_EVT_TX_PORT_CLOSED:
            transport_flag_set(GUS_TX_NTF_ENABLE, false);
            break;

        case GUS_EVT_RX_DATA_RECEIVED:
            ble_to_uart_push(p_evt->p_data, p_evt->length);
            break;

        case GUS_EVT_TX_DATA_SENT:
            transport_flag_set(BLE_TX_CPLT, true);
            transport_ble_continue_send();
            break;

        case GUS_EVT_FLOW_CTRL_ENABLE:
            transport_flag_set(BLE_FLOW_CTRL_ENABLE, true);
            break;

        case GUS_EVT_FLOW_CTRL_DISABLE:
            transport_flag_set(BLE_FLOW_CTRL_ENABLE, false);
            break;

        case GUS_EVT_TX_FLOW_OFF:
            transport_flag_set(BLE_TX_FLOW_ON, false);
            break;

        case GUS_EVT_TX_FLOW_ON:
            transport_flag_set(BLE_TX_FLOW_ON, true);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Function for initializing services
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t   error_code;
    gus_init_t gus_init;

    gus_init.evt_handler = gus_service_process_event;

    error_code = gus_service_init(&gus_init);
    APP_ERROR_CHECK(error_code);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    transport_ble_init();
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
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
 ****************************************************************************************
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

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(&(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            update_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
            break;
    }
}

void ble_app_init(void)
{
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t     version;
    sdk_err_t         error_code;

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
    APP_LOG_INFO("Goodix UART example started.");

    transport_ble_init();
    transport_uart_init();
    services_init();
    gap_params_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}


