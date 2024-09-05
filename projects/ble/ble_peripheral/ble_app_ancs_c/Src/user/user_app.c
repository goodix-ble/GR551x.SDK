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
#include "utility.h"
#include "ancs_c.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                    "Goodix_ANCS_C" /**< Device Name which will be set in GAP. */
#define APP_ADV_MIN_INTERVAL           32              /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL           48              /**< The advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS     0               /**< The advertising timeout in units of seconds. */
#define MIN_CONN_INTERVAL              6               /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL              12              /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                  0               /**< Slave latency. */
#define CONN_SUP_TIMEOUT               400             /**< Connection supervisory timeout (4 seconds). */
#define ANCS_DISC_DELAY_TIME           1000            /**< Wait time before ANCS service discovery (1 seconds). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;
static ble_gap_adv_time_param_t s_gap_adv_time_param;

static const uint8_t s_adv_data_set[] =
{
    0x11,
    BLE_GAP_AD_TYPE_RQRD_128_BIT_SVC_UUID,
    0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f,
    0xa4, 0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4,
    0x05, 0x79,//ANCS SERVICE UUID

    0x3,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_UNKNOWN),
    HI_U16(BLE_APPEARANCE_UNKNOWN),

};

static const uint8_t s_adv_rsp_data_set[] =
{
    0xE,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'A', 'N', 'C', 'S', '_', 'C',
};

static app_timer_id_t s_delay_timer_id;

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
    sdk_err_t error_code;

    ble_gap_pair_enable(true);

    error_code = ble_gap_privacy_params_set(150, true);
    APP_ERROR_CHECK(error_code);

    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL3,
        .io_cap    = BLE_SEC_IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
    };
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 *@brief Process ANCS Client event.
 *****************************************************************************************
 */
static void ancs_c_evt_process(ancs_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch(p_evt->evt_type)
    {
        case BLE_ANCS_C_EVT_DISCOVERY_CPLT:
            error_code = ancs_c_ntf_source_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case BLE_ANCS_C_EVT_NTF_SOURCE_NTF_ENABLED:
            APP_LOG_INFO("ANCS notification source notification enabled.");
            error_code = ancs_c_data_source_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case BLE_ANCS_C_EVT_DATA_SOURCE_NTF_ENABLED:
            APP_LOG_INFO("ANCS data source notification enabled.");
            break;

        case BLE_ANCS_C_EVT_NTF_SOURCE_RECEIVE:
            break;

        case BLE_ANCS_C_EVT_DATA_SOURCE_RECEIVE:
            break;

        case BLE_ANCS_C_EVT_WRITE_OP_ERR:
            APP_LOG_INFO("Write error.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t error_code;
    error_code = ancs_c_client_init(ancs_c_evt_process);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Handler the delay timer timeout.
 *****************************************************************************************
 */
static void delay_timer_handler(void* p_arg)
{
    ancs_c_discovery_service(0);
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t error_code;
    error_code = app_timer_create(&s_delay_timer_id, ATIMER_ONE_SHOT, delay_timer_handler);
    APP_ERROR_CHECK(error_code);
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t tk;

    if (NULL == p_enc_req)
    {
        return;
    }
    memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));

    switch (p_enc_req->req_type)
    {
        // user need to decide whether to accept the pair request
        case BLE_SEC_PAIR_REQ:
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept = true;
            break;

        // user need to input the password
        case BLE_SEC_TK_REQ:
            APP_LOG_INFO("Please Input pin code: 123456.");
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept = true;
            tk = 123456;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;

    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);

    error_code = ble_sec_enc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

static void ancs_c_disc_delay_timer_start(void)
{
    app_timer_start(s_delay_timer_id, ANCS_DISC_DELAY_TIME, NULL);
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
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                ancs_c_disc_delay_timer_start();
            }
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
    APP_LOG_INFO("Apple Notification Center Client example started.");

    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}
