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
#include "ble_scanner.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "gus.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   160             /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     80              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   0               /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               6               /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               24              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

#define MAX_MTU_DEFUALT                     247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     247             /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 2              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */

#define UART_UUID_LEN                       16              /**< Defualt UART UUID length. */
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const char     s_target_name[] = "Goodix_UART";                                          /**< Target name string. */
static const uint8_t  s_target_addr[SYS_BD_ADDR_LEN] = {0x13, 0x00, 0xcf, 0x3e, 0xcb, 0xea};    /**< Target board address. */
static const uint8_t  s_target_uuid[UART_UUID_LEN] = {GUS_SERVICE_UUID};                        /**< Target uuid string. */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Handle ble scanner module event.
 *****************************************************************************************
 */
static void ble_scanner_evt_handler(ble_scanner_evt_t *p_evt)
{
    sdk_err_t    error_code;

    switch (p_evt->evt_type)
    {
        case BLE_SCANNER_EVT_CONNECTED:
            error_code = ble_gattc_mtu_exchange(p_evt->param.conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case BLE_SCANNER_EVT_FILTER_MATCH:
            APP_LOG_INFO("Found target device.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    error_code = ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}

/**
 *****************************************************************************************
 *@brief Initialize the BLE scan module.
 *****************************************************************************************
 */
static void gap_scan_init(void)
{
    sdk_err_t                 error_code;
    ble_scanner_init_t        scan_init;
    ble_scanner_filter_data_t filter_data;

    memset(&scan_init, 0x00, sizeof(ble_scanner_init_t));
    memset(&filter_data, 0x00, sizeof(ble_scanner_filter_data_t));
    
    scan_init.scan_param.scan_type      = BLE_GAP_SCAN_ACTIVE;
    scan_init.scan_param.scan_mode      = BLE_GAP_SCAN_GEN_DISC_MODE;
    scan_init.scan_param.scan_dup_filt  = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    scan_init.scan_param.use_whitelist  = false;
    scan_init.scan_param.interval       = APP_SCAN_INTERVAL;
    scan_init.scan_param.window         = APP_SCAN_WINDOW;
    scan_init.scan_param.timeout        = APP_SCAN_DURATION;
    scan_init.conn_param.type           = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    scan_init.conn_param.interval_min   = APP_CONN_INTERVAL_MIN;
    scan_init.conn_param.interval_max   = APP_CONN_INTERVAL_MAX;
    scan_init.conn_param.slave_latency  = APP_CONN_SLAVE_LATENCY;
    scan_init.conn_param.sup_timeout    = APP_CONN_SUP_TIMEOUT;
    scan_init.connect_auto              = true;
    scan_init.err_handler               = NULL;
    scan_init.evt_handler               = ble_scanner_evt_handler;

    filter_data.svr_uuid.length         = UART_UUID_LEN;
    filter_data.svr_uuid.p_data         = s_target_uuid;
    filter_data.dev_name.length         = strlen(s_target_name);
    filter_data.dev_name.p_data         = (uint8_t *)s_target_name;
    filter_data.target_addr.addr_type   = BLE_GAP_ADDR_TYPE_PUBLIC;
    memcpy(filter_data.target_addr.gap_addr.addr, s_target_addr, SYS_BD_ADDR_LEN);

    ble_scanner_filter_set(BLE_SCANNER_UUID_FILTER | BLE_SCANNER_ADDR_FILTER, &filter_data);

    error_code = ble_scanner_init(&scan_init);
    APP_ERROR_CHECK(error_code);

    ble_scanner_filter_enable(BLE_SCANNER_FILTER_ALL_MATCH);

    error_code = ble_scanner_start();
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Handle event of Goodix UART Client.
 *****************************************************************************************
 */
static void gus_c_evt_process(gus_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch (p_evt->evt_type)
    {
        case GUS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Goodix Uart Service discovery completely.");
            error_code = gus_c_tx_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            transport_flag_set(BLE_TX_FLOW_ON, true);
            break;

        case GUS_C_EVT_TX_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled TX Notification.");
            error_code = gus_c_flow_ctrl_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case GUS_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled Flow Control Notification.");
            transport_flag_set(BLE_FLOW_CTRL_ENABLE, true);
            break;

        case GUS_C_EVT_PEER_DATA_RECEIVE:
            ble_to_uart_push(p_evt->p_data, p_evt->length);
            break;

        case GUS_C_EVT_TX_FLOW_OFF:
            transport_flag_set(BLE_TX_FLOW_ON, false);
            break;

        case GUS_C_EVT_TX_FLOW_ON:
            transport_flag_set(BLE_TX_FLOW_ON, true);
            break;

        case GUS_C_EVT_TX_CPLT:
            transport_flag_set(BLE_TX_CPLT, true);
            transport_ble_continue_send();
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Deal MTU exchange task.
 *****************************************************************************************
 */
static void app_mtu_exchange_handler(uint8_t conn_idx)
{
    sdk_err_t   error_code;

    error_code = gus_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal device disconnect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] disconnect_reason: disconnect reason.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    sdk_err_t   error_code;

    transport_ble_init();

    error_code = ble_scanner_start();
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    ble_scanner_evt_on_ble_capture(p_evt);
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_SCAN_START:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                APP_LOG_DEBUG("Scan started failed(0X%02X).", p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            if (BLE_GAP_STOPPED_REASON_TIMEOUT == p_evt->evt.gapm_evt.params.scan_stop.reason)
            {
                APP_LOG_DEBUG("Scan Timeout.");
            }
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            break;

        case BLE_GAPC_EVT_CONNECTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Connected.");
            }
            break; 

        case BLE_GAPC_EVT_DISCONNECTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Disconnected (0x%02X).", p_evt->evt.gapc_evt.params.disconnected.reason);
                app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            }
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("MTU exchanged.");
                update_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
                app_mtu_exchange_handler(p_evt->evt.gapc_evt.index);
            }
            break;
    }
}

void ble_app_init(void)
{
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t version;
    sdk_err_t    error_code;

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
    APP_LOG_INFO("Goodix UART Service Client example started.");

    transport_ble_init();
    transport_uart_init();

    error_code = gus_client_init(gus_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();
    gap_scan_init();
}

