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
#include "at_cmd_handler.h"
#include "user_periph_setup.h"
#include "transport_scheduler.h"
#include "app_log.h"
#include "at_cmd_utils.h"
#include "utility.h"
#include "ring_buffer.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                         "Goodix_UART_AT"   /**< Name of device which will be included in the advertising data. */
#define APP_ADV_INTERVAL_MIN                160                /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                160                /**< The advertising max interval (in units of 0.625 ms). */
#define APP_SCAN_INTERVAL                   15                 /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15                 /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   0                  /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               36                 /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               36                 /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0                  /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400                /**< Connection supervisory timeout(in unit of 10 ms). */
#define APP_CONN_TIMEOUT                    400                /**< Connection timeout(in unit of 10 ms). */

#define MAX_MTU_DEFUALT                     247                /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     247                 /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 1                  /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251                /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120               /**< Defualt maximum packet transmission time. */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
ble_gap_adv_param_t      g_gap_adv_param;        /**< Advertising parameters for legay advertising. */
ble_gap_adv_time_param_t g_gap_adv_time_param;
ble_gap_scan_param_t     g_gap_scan_param;       /**< Scanning parameters. */
ble_gap_init_param_t     g_gap_connect_param;
bool                     g_master_set_cccd_flag;
static bool is_connected;
extern ring_buffer_t     s_uart_to_ble_buffer;
extern ring_buffer_t     s_ble_to_uart_buffer;

uint8_t g_adv_data_set[28] =
{
    0x11,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    GUS_SERVICE_UUID,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02,
    0x03,
};

uint8_t g_adv_rsp_data_set[28] =            /**< Scan responce data. */
{
    0x0f,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'U', 'A', 'R', 'T', '_', 'A', 'T'
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    ble_gap_pair_enable(false);
    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);

    // Default GAP Advertising Parameter Init
    g_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL_MAX;
    g_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL_MIN;
    g_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    g_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    g_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    g_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &g_gap_adv_param);
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, g_adv_data_set, sizeof(g_adv_data_set));
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, g_adv_rsp_data_set, sizeof(g_adv_rsp_data_set));

    g_gap_adv_time_param.duration    = 0;
    g_gap_adv_time_param.max_adv_evt = 0;

    // Default GAP Scan Parameter Init
    g_gap_scan_param.scan_type     = BLE_GAP_SCAN_ACTIVE;
    g_gap_scan_param.scan_mode     = BLE_GAP_SCAN_OBSERVER_MODE;
    g_gap_scan_param.scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    g_gap_scan_param.use_whitelist = false;
    g_gap_scan_param.interval      = APP_SCAN_INTERVAL;
    g_gap_scan_param.window        = APP_SCAN_WINDOW;
    g_gap_scan_param.timeout       = APP_SCAN_DURATION;

    // Default GAP Connect Parameter Init
    g_gap_connect_param.type          = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    g_gap_connect_param.interval_min  = APP_CONN_INTERVAL_MIN;
    g_gap_connect_param.interval_max  = APP_CONN_INTERVAL_MAX;
    g_gap_connect_param.slave_latency = APP_CONN_SLAVE_LATENCY;
    g_gap_connect_param.sup_timeout   = APP_CONN_SUP_TIMEOUT;
    g_gap_connect_param.conn_timeout  = APP_CONN_TIMEOUT;
}


/**
 *****************************************************************************************
 * @brief Process gus event.
 *
 * @param[in] p_evt: Pointer to gus even strcture.
 *****************************************************************************************
 */
static void gus_service_process_event(gus_evt_t *p_evt)
{
    uint8_t ble_rx_data[AT_CMD_BUFFER_SIZE_MAX];

    switch (p_evt->evt_type)
    {
        case GUS_EVT_TX_PORT_OPENED:
            transport_flag_set(GUS_TX_NTF_ENABLE, true);
            break;

        case GUS_EVT_TX_PORT_CLOSED:
            transport_flag_set(GUS_TX_NTF_ENABLE, false);
            break;

        case GUS_EVT_RX_DATA_RECEIVED:
            if (0 == memcmp(p_evt->p_data, "AT:", 3))
            {
                memcpy(ble_rx_data, p_evt->p_data, p_evt->length);

                if ((0x0d != p_evt->p_data[p_evt->length - 2]) ||\
                    (0x0a != p_evt->p_data[p_evt->length - 1]))
                {
                    ble_rx_data[p_evt->length]     = 0x0d;
                    ble_rx_data[p_evt->length + 1] = 0x0a;
                }

                at_cmd_parse(AT_CMD_SRC_BLE, ble_rx_data, p_evt->length + 2);
            }
            else
            {
                ble_to_uart_buff_data_push(p_evt->p_data, p_evt->length);
            }

            break;

        case GUS_EVT_TX_DATA_SENT:
            transport_flag_set(BLE_TX_CPLT, true);
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
 * @brief Process gus client event.
 *
 * @param[in] p_evt: Pointer to gus client even strcture.
 *****************************************************************************************
 */
static void gus_client_process_event(gus_c_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case GUS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Goodix Uart Service discovery completely.");
            g_master_set_cccd_flag = true;
            gus_c_tx_notify_set(p_evt->conn_idx, g_master_set_cccd_flag);
            break;

        case GUS_C_EVT_TX_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled TX Notification.");
            transport_flag_set(GUS_TX_NTF_ENABLE, g_master_set_cccd_flag);
            gus_c_flow_ctrl_notify_set(p_evt->conn_idx, true);
            break;

        case GUS_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled Flow Control Notification.");
            transport_flag_set(BLE_FLOW_CTRL_ENABLE, true);
            break;

        case GUS_C_EVT_PEER_DATA_RECEIVE:
            ble_to_uart_buff_data_push(p_evt->p_data, p_evt->length);
            break;

        case GUS_C_EVT_TX_FLOW_OFF:
            transport_flag_set(BLE_TX_FLOW_ON, false);
            break;

        case GUS_C_EVT_TX_FLOW_ON:
            transport_flag_set(BLE_TX_FLOW_ON, true);
            break;

        case GUS_C_EVT_TX_CPLT:
            transport_flag_set(BLE_TX_CPLT, true);
            break;

        default:
            break;
    }
}

static void services_init(void)
{
    gus_init_t gus_init;
    gus_init.evt_handler = gus_service_process_event;
    gus_service_init(&gus_init);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ********************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    AT_CMD_RSP_DEF(cmd_rsp);

    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                uart_at_dev_state_set(ADVERTISING);
            }
            else
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }
            at_cmd_execute_cplt(&cmd_rsp);
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                if (!is_connected)
                {
                    uart_at_dev_state_set(STANDBY);
                }
            }
            else
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }
            at_cmd_execute_cplt(&cmd_rsp);
            break;
            
        case BLE_GAPM_EVT_SCAN_START:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                uart_at_dev_state_set(SCANNING);
            }
            else
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }
            at_cmd_execute_cplt(&cmd_rsp);
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                uart_at_dev_state_set(STANDBY);
            }
            else
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }

            at_cmd_execute_cplt(&cmd_rsp);
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            uart_at_adv_report_task(p_evt->evt.gapm_evt.params.adv_report.data,
                                    p_evt->evt.gapm_evt.params.adv_report.length,
                                    &(p_evt->evt.gapm_evt.params.adv_report.broadcaster_addr));
            break;

        case BLE_GAPC_EVT_CONNECTED:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }
            else
            {
                uart_at_conn_task(p_evt->evt.gapc_evt.params.connected.ll_role);
                uart_at_dev_state_set(CONNECTED);
                is_connected = true;

                if (BLE_GAP_LL_ROLE_MASTER == p_evt->evt.gapc_evt.params.connected.ll_role)
                {
                    gus_c_disc_srvc_start(p_evt->evt.gapc_evt.index);
                }
                APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                                p_evt->evt.gapc_evt.params.connected.peer_addr.addr[5],
                                p_evt->evt.gapc_evt.params.connected.peer_addr.addr[4],
                                p_evt->evt.gapc_evt.params.connected.peer_addr.addr[3],
                                p_evt->evt.gapc_evt.params.connected.peer_addr.addr[2],
                                p_evt->evt.gapc_evt.params.connected.peer_addr.addr[1],
                                p_evt->evt.gapc_evt.params.connected.peer_addr.addr[0]);
            }
            at_cmd_execute_cplt(&cmd_rsp);
            GLOBAL_EXCEPTION_DISABLE();
            memset(s_uart_to_ble_buffer.p_buffer, 0, sizeof(s_uart_to_ble_buffer.buffer_size));
            memset(s_ble_to_uart_buffer.p_buffer, 0, sizeof(s_ble_to_uart_buffer.buffer_size));
            s_uart_to_ble_buffer.write_index = 0;
            s_uart_to_ble_buffer.read_index  = 0;
            s_ble_to_uart_buffer.write_index = 0;
            s_ble_to_uart_buffer.read_index  = 0;
            transport_flag_set(BLE_TX_CPLT, true);
            transport_flag_set(GUS_TX_NTF_ENABLE, false);
            transport_flag_set(BLE_FLOW_CTRL_ENABLE, false);
            transport_flag_set(BLE_TX_FLOW_ON, true);
            transport_flag_set(BLE_RX_FLOW_ON, true);
            GLOBAL_EXCEPTION_ENABLE();
            break; 

        case BLE_GAPC_EVT_DISCONNECTED:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }
            else
            {
                uart_at_dev_state_set(STANDBY);
                APP_LOG_INFO("Disconnected (0x%02X)", p_evt->evt.gapc_evt.params.disconnected.reason);
            }
            at_cmd_execute_cplt(&cmd_rsp);
            break;

        case BLE_GAPC_EVT_CONNECT_CANCLE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
                uart_at_dev_state_set(STANDBY);
            }
            else
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }

            at_cmd_execute_cplt(&cmd_rsp);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;
        
        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                update_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
                cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "MTU:%d", p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
            }
            else
            {
                cmd_rsp.error_code = at_cmd_ble_err_convert(p_evt->evt_status);
            }

            at_cmd_execute_cplt(&cmd_rsp);
            break;
    }
}

void ble_app_init(void)
{
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    ble_gap_addr_get(&bd_addr);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
    APP_LOG_INFO("Goodix UART(AT) example start.");

    gap_params_init();
    transport_ble_init();
    transport_uart_init();
    services_init();
    gus_client_init(gus_client_process_event);
    uart_at_init(BLE_GAP_ROLE_PERIPHERAL);

    ble_gap_adv_start(0, &g_gap_adv_time_param);
}

