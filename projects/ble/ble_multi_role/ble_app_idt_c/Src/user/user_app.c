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
#include "user_periph_setup.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "app_idt.h"
#include "app_dual_tim.h"
#include "app_io.h"
/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   15              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   1000               /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               6              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               6              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

#define MAX_MTU_DEFUALT                     247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     247              /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */

#define ISM_SEND_INTERVAL               1000

#define ACCESS_ADDRESS    0x6730363F
#define CHANNEL_IDX       34

#define MS_SYSCNT_TO_TIMER(X)     ((SystemCoreClock / 1000) *(X) - 1)
#define DUAL_TIM1_PARAM           { APP_DUAL_TIM_ID_1, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_LOOP, MS_SYSCNT_TO_TIMER(1000) }}


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static idt_info_t s_idt_info = 
{
    .access_addr = ACCESS_ADDRESS,
    .channel_idx = CHANNEL_IDX,
    .cmd_id = 0,
    .delay_time = 2000
};
static bool s_idt_enbale = false;

static uint32_t s_all_tx_num = 0;
static uint8_t  s_tx_data[5] = { 0 };
 
static ble_gap_bdaddr_t  s_target_bdaddr;                       /**< Target board address. */


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
    sdk_err_t   error_code;
    ble_gap_scan_param_t scan_param;

    scan_param.scan_type     = BLE_GAP_SCAN_ACTIVE;
    scan_param.scan_mode     = BLE_GAP_SCAN_OBSERVER_MODE;
    scan_param.scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    scan_param.use_whitelist = false;
    scan_param.interval      = APP_SCAN_INTERVAL;
    scan_param.window        = APP_SCAN_WINDOW;
    scan_param.timeout       = APP_SCAN_DURATION;

    error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &scan_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}


/**
 *****************************************************************************************
 * @brief Find IDTS uuid from advertising data.
 *
 * @param[in] p_data:   Pointer to advertising report data.
 * @param[in] length:   Length of advertising report data.
 *
 * @return Operation result.
 *****************************************************************************************
 */
static bool user_idts_uuid_find(const uint8_t *p_data, const uint16_t length)
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
            uint8_t target_uuid[16] = IDTS_SVC_UUID;
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

static void idts_c_evt_process(idts_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch (p_evt->evt_type)
    {
        case IDTS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Goodix Uart Service discovery completely.");
            error_code = idts_c_tx_indicate_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);

            break;

        case IDTS_C_EVT_TX_IND_SET_SUCCESS:
            APP_LOG_INFO("Enabled TX Notification.");
            idts_c_tx_data_send(p_evt->conn_idx, (uint8_t *)(&s_idt_info), sizeof(idt_info_t));

            break;

        case IDTS_C_EVT_PEER_DATA_RECEIVE:
            if (!memcmp(&s_idt_info, p_evt->p_data, sizeof(idt_info_t)))
            {
                s_idt_info.cmd_id = 1;
                idts_c_tx_data_send(p_evt->conn_idx, (uint8_t *)(&s_idt_info), sizeof(idt_info_t));

                s_idt_enbale = true;

            }

            break;

        case IDTS_C_EVT_TX_CPLT:
            APP_LOG_INFO("Tx complete.");

            break;

        default:
            break;
    }
}

SECTION_RAM_CODE static void dul_tim_idt_send_handler(app_dual_tim_evt_t *p_evt)
{
    app_io_toggle_pin(APP_IO_TYPE_NORMAL, APP_IO_PIN_28);
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
        
        if (idt_status_check())
        {
            s_tx_data[0] = 0;
            s_tx_data[1] = ((s_all_tx_num >> 24) & 0xFF);
            s_tx_data[2] = ((s_all_tx_num >> 16) & 0xFF);
            s_tx_data[3] = ((s_all_tx_num >> 8) & 0xFF);
            s_tx_data[4] = ((s_all_tx_num >> 0) & 0xFF);
            s_all_tx_num++;

            idt_data_send(5, s_tx_data);
        }
    }
}

static void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t   error_code;

    if (user_idts_uuid_find(p_data, length))
    {
        memcpy(&s_target_bdaddr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
        error_code = ble_gap_scan_stop();
        APP_ERROR_CHECK(error_code);
    }
}

static void app_scan_stop_handler(void)
{
    sdk_err_t        error_code;
    ble_gap_init_param_t gap_connect_param;

    gap_connect_param.type                = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    gap_connect_param.interval_min        = APP_CONN_INTERVAL_MIN;
    gap_connect_param.interval_max        = APP_CONN_INTERVAL_MAX;
    gap_connect_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    gap_connect_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    gap_connect_param.peer_addr.gap_addr  = s_target_bdaddr.gap_addr;
    gap_connect_param.peer_addr.addr_type = s_target_bdaddr.addr_type;

    error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
    APP_ERROR_CHECK(error_code);
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t   error_code;

    error_code =idts_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}


static void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    if (false == s_idt_enbale)
    {
        sdk_err_t   error_code;

        error_code = ble_gap_scan_start();
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        app_idt_init(&s_idt_info, NULL);

        app_dual_tim_params_t p_params_tim1 = DUAL_TIM1_PARAM;
        app_dual_tim_init(&p_params_tim1, dul_tim_idt_send_handler);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void idt_send_start(void)
{
    app_dual_tim_start(APP_DUAL_TIM_ID_1);
}

void idt_send_stop(void)
{
    app_dual_tim_stop(APP_DUAL_TIM_ID_1);
}

void ble_evt_handler(const ble_evt_t *p_evt)
{
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
            else
            {
                app_scan_stop_handler();
            }
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            app_adv_report_handler(p_evt->evt.gapm_evt.params.adv_report.data, 
                                   p_evt->evt.gapm_evt.params.adv_report.length, 
                                   &p_evt->evt.gapm_evt.params.adv_report.broadcaster_addr);
            break;

        case BLE_GAPC_EVT_CONNECTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Connected.");
                app_connected_handler(p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.connected);
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
            }
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
    APP_LOG_INFO("Goodix ISM direct transport client started.");

    error_code = idts_client_init(idts_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}

