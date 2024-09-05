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

#define MAX_MTU_DEFUALT                     247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     23              /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */

#define DFU_ACK_WAIT_TIMEOUT                4000            /**< DFU master wait ACK timeout(in unit of 1 ms). */

uint16_t fast_dfu_program_one_size = 0;

extern uint8_t ble_send_cplt_flag;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_scan_param_t s_scan_param;
static ble_gap_init_param_t s_conn_param;
static app_timer_id_t       s_ack_wait_timer_id;
static uint16_t             s_master_last_count;
static uint16_t             s_master_curr_count;
static const uint8_t s_target_addr[SYS_BD_ADDR_LEN] = {0x20, 0xaa, 0xcf, 0x3e, 0xcb, 0xea};

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

    ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
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
    extern uint8_t fast_dfu_mode;
    extern uint32_t program_size;

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
            if (fast_dfu_mode == FAST_DFU_MODE_DISABLE)
            {
                dfu_m_send_data_cmpl_process();
            }
            else if (fast_dfu_mode == FAST_DFU_MODE_ENABLE && program_size != 0)
            {
                ble_send_cplt_flag = 1;
            }
            break;

        case OTAS_C_EVT_PEER_DATA_RECEIVE:
            dfu_m_cmd_prase(p_evt->p_data, p_evt->length);
            break;

        default:
            break;
    }
}

static void ack_wait_timeout_handler(void* p_arg)
{
    if (s_master_last_count < s_master_curr_count)
    {
        s_master_last_count = s_master_curr_count;
    }
    else
    {
        s_master_last_count = 0;
        s_master_curr_count = 0;
        app_timer_stop(s_ack_wait_timer_id);
        dfu_m_parse_state_reset();
    }
}

static void master_timer_init(void)
{
    app_timer_create(&s_ack_wait_timer_id, ATIMER_REPEAT, ack_wait_timeout_handler);
}

/**
 *****************************************************************************************
 *@brief Function for deal disconnect.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{

}

static void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    if (user_otas_uuid_find(p_data, length) && memcmp(s_target_addr, p_bdaddr->gap_addr.addr, SYS_BD_ADDR_LEN) == 0)
    {
        memcpy(&s_conn_param.peer_addr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
        ble_gap_scan_stop();
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

    }
}

static void app_mtu_exchange_handler(uint8_t conn_idx)
{
    otas_c_disc_srvc_start(conn_idx);
}

static void fast_dfu_program_flash_size_set(uint8_t mtu_size)
{
    fast_dfu_program_one_size = mtu_size;
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void master_timer_start(void)
{
    app_timer_start(s_ack_wait_timer_id, DFU_ACK_WAIT_TIMEOUT, NULL);
}

void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_SCAN_START:
            if (p_evt->evt_status)
            {
                APP_LOG_DEBUG("Scan started failed(0X%02X).", p_evt->evt_status);
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
                fast_dfu_program_flash_size_set(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
            }
            break;
    }
} 

void master_timer_stop(void)
{
    app_timer_stop(s_ack_wait_timer_id);
}

void ble_data_send(uint8_t *p_data, uint16_t length)
{
    otas_c_tx_data_send(0, p_data, length);
}

void app_start_scan(void)
{
    ble_gap_scan_start();
}
void app_dfu_rev_cmd_cb(void)
{
    s_master_curr_count++;
}

void ble_app_init(void)
{
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    otas_client_init(otas_c_evt_process);
    gap_params_init();
    master_timer_init();
    user_master_status_set(MASTER_IDLE);
}
