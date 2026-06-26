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
#include "board_SK.h"
#include "otas_c.h"
#include "app_timer.h"
#include "app_log.h"
#include "grx_sys.h"
#include "dfu_master.h"
#include "custom_config.h"
#include "user_periph_setup.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   15              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   2000            /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               12              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               12              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

#define MAX_MTU_DEFAULT                     247             /**< Default length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFAULT                     23              /**< Default length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFAULT                 10              /**< Default length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFAULT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFAULT                 2120            /**< Default maximum packet transmission time. */


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_scan_param_t s_scan_param;
static ble_gap_init_param_t s_conn_param;
static const uint8_t s_target_addr0[SYS_BD_ADDR_LEN] = {0x20, 0xaa, 0xcf, 0x3e, 0xcb, 0xea}; // APP
static const uint8_t s_target_addr1[SYS_BD_ADDR_LEN] = {0x21, 0xaa, 0xcf, 0x3e, 0xcb, 0xea}; // APP_Bootloader
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    s_scan_param.scan_type     = BLE_GAP_SCAN_ACTIVE;
    s_scan_param.scan_mode     = BLE_GAP_SCAN_OBSERVER_MODE;
    s_scan_param.scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    s_scan_param.use_whitelist = false;
    s_scan_param.interval      = APP_SCAN_INTERVAL;
    s_scan_param.window        = APP_SCAN_WINDOW;
    s_scan_param.timeout       = APP_SCAN_DURATION;
    s_scan_param.timeout       = APP_SCAN_DURATION;

    ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &s_scan_param);

    s_conn_param.type                = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    s_conn_param.interval_min        = APP_CONN_INTERVAL_MIN;
    s_conn_param.interval_max        = APP_CONN_INTERVAL_MAX;
    s_conn_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    s_conn_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    s_conn_param.conn_timeout        = DFU_ACK_WAIT_TIMEOUT/10;

    ble_gap_l2cap_params_set(MAX_MTU_DEFAULT, MAX_MPS_DEFAULT, MAX_NB_LECB_DEFAULT);
    ble_gap_data_length_set(MAX_TX_OCTET_DEFAULT, MAX_TX_TIME_DEFAULT);
    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}


/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static bool user_otas_uuid_find(const uint8_t *p_data, const uint16_t length)
{
    uint16_t current_pos = 0;

    if (NULL == p_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t filed_type  = 0;
        uint8_t data_length = 0;
        uint8_t fragment_length = p_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        filed_type  = p_data[current_pos++];

        if ((BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID == filed_type) || \
                (BLE_GAP_AD_TYPE_MORE_128_BIT_UUID == filed_type))
        {
            uint8_t parase_uuid[16] = {0};
            uint8_t target_uuid[16] = OTAS_SVC_UUID;
            uint8_t counter_128_bit_uuid =  data_length / 16;

            for (uint8_t i = 0; i < counter_128_bit_uuid; i++)
            {
                memcpy(parase_uuid, &p_data[current_pos + (16 * i)], 16);

                if (0 == memcmp(target_uuid, parase_uuid, 16))
                {
                    return true;
                }
            }

            return false;
        }

        current_pos += data_length;
    }

    return false;
}

static void otas_c_evt_process(otas_c_evt_t *p_evt)
{
#if DFU_BLE_ENABLE
    switch (p_evt->evt_type)
    {
        case OTAS_C_EVT_DISCOVERY_COMPLETE:
            otas_c_tx_notify_set(p_evt->conn_idx, true);
            break;

        case OTAS_C_EVT_TX_NTF_SET_SUCCESS:
            ble_gap_phy_update(p_evt->conn_idx,
                               BLE_GAP_PHY_LE_2MBPS,
                               BLE_GAP_PHY_LE_2MBPS,
                               0);
            user_master_status_set(MASTER_BLE_CONNECTED);
            user_master_status_set(MASTER_FAST_DFU_MODE_SET);
            break;

        case OTAS_C_EVT_TX_CPLT:
            if (dfu_m_fast_dfu_mode_get() == FAST_DFU_MODE_DISABLE)
            {
            #if DFU_ASYNC_TX_ENABLE
                dfu_m_send_data_cmpl_process();
            #endif
            }
            else if (dfu_m_get_program_size() != 0)
            {
                dfu_m_fast_send_data_cmpl_process();
            }
            break;

        case OTAS_C_EVT_PEER_DATA_RECEIVE:
            dfu_m_cmd_parse(p_evt->p_data, p_evt->length);
            break;

        default:
            break;
    }
#endif
}

/**
 *****************************************************************************************
 *@brief Function for deal disconnect.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t status, const uint8_t disconnect_reason)
{
    APP_LOG_INFO("Disconnected (0x%02X).", disconnect_reason);
    uart_console_reset();
}

static void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    if (user_otas_uuid_find(p_data, length))
    {
        if ((memcmp(s_target_addr0, p_bdaddr->gap_addr.addr, SYS_BD_ADDR_LEN) == 0) ||
            (memcmp(s_target_addr1, p_bdaddr->gap_addr.addr, SYS_BD_ADDR_LEN) == 0))
        {
            APP_LOG_DEBUG("Scanned address %02X:%02X:%02X:%02X:%02X:%02X",
               p_bdaddr->gap_addr.addr[5],
               p_bdaddr->gap_addr.addr[4],
               p_bdaddr->gap_addr.addr[3],
               p_bdaddr->gap_addr.addr[2],
               p_bdaddr->gap_addr.addr[1],
               p_bdaddr->gap_addr.addr[0]);
            memcpy(&s_conn_param.peer_addr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
            ble_gap_scan_stop();
        }
    }
}

/**
 *****************************************************************************************
 * @brief Scan stop handler.
 *****************************************************************************************
 */
static void app_scan_stop_handler(ble_gap_stopped_reason_t reason)
{
    if (BLE_GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        APP_LOG_DEBUG("SCAN TIMEOUT");
        uart_console_reset();
    }
    else
    {
        ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &s_conn_param);
    }
}

/**
 *****************************************************************************************
 *@brief Function for deal device connect.
 *****************************************************************************************
 */
static void app_connected_handler(uint8_t status, uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    if (BLE_SUCCESS == status)
    {
        ble_gattc_mtu_exchange(conn_idx);
    }
    else
    {
        APP_LOG_DEBUG("Connection error(0X%02X)", status);
        uart_console_reset();
    }
}

static void app_mtu_exchange_handler(uint8_t conn_idx)
{
    otas_c_disc_srvc_start(conn_idx);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_SCAN_START:
            if (p_evt->evt_status)
            {
                APP_LOG_DEBUG("Scan started failed(0X%02X)", p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            app_scan_stop_handler(p_evt->evt.gapm_evt.params.scan_stop.reason);
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            app_adv_report_handler(p_evt->evt.gapm_evt.params.adv_report.data, p_evt->evt.gapm_evt.params.adv_report.length, &p_evt->evt.gapm_evt.params.adv_report.broadcaster_addr);
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt_status, p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.connected);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt_status, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                app_mtu_exchange_handler(p_evt->evt.gapc_evt.index);
            }
            break;

        default:
            break;
    }
}

void ble_data_send(uint8_t *p_data, uint16_t length)
{
    otas_c_tx_data_send(0, p_data, length);
}

void app_start_scan(void)
{
    APP_LOG_DEBUG("Target address %02X:%02X:%02X:%02X:%02X:%02X or %02X:%02X:%02X:%02X:%02X:%02X",
                 s_target_addr0[5],
                 s_target_addr0[4],
                 s_target_addr0[3],
                 s_target_addr0[2],
                 s_target_addr0[1],
                 s_target_addr0[0],

                 s_target_addr1[5],
                 s_target_addr1[4],
                 s_target_addr1[3],
                 s_target_addr1[2],
                 s_target_addr1[1],
                 s_target_addr1[0]);
    ble_gap_scan_start();
}

void ble_app_init(void)
{
    otas_client_init(otas_c_evt_process);
    gap_params_init();
}
