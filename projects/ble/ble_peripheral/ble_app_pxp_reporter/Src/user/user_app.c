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
#include "lls.h"
#include "ias.h"
#include "tps.h"
#include "grx_sys.h"
#include "utility.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"


/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                        "Goodix_PXP"     /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL          32               /**< The fast advertising min interval (in units of 0.625 ms).*/
#define APP_ADV_FAST_MAX_INTERVAL          48               /**< The fast advertising max interval (in units of 0.625 ms).*/
#define APP_ADV_SLOW_MIN_INTERVAL          1600             /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL          4000             /**< The slow advertising max interval (in units of 0.625 ms). */
#define FAST_ADV_DURATION                  3000             /**< Advertising duration for fast connection (in unit of 10 ms). */
#define INIT_TX_POWER_LEVEL                0                /**< Initial Tx power level, unit dBm, rang [-100, 20] */

enum
{
    APP_ADV_TYPE_NONE,
    APP_ADV_TYPE_FAST,
    APP_ADV_TYPE_SLOW,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */
static uint8_t              s_app_adv_type;                 /**< The type of current advertising. */

static const uint8_t s_adv_data_set[] =                     /**< Advertising data. */
{
    0x0B,
    BLE_GAP_AD_TYPE_SHORTENED_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'P', 'X', 'P',

    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_LINK_LOSS),
    HI_U16(BLE_ATT_SVC_LINK_LOSS),
    LO_U16(BLE_ATT_SVC_IMMEDIATE_ALERT),
    HI_U16(BLE_ATT_SVC_IMMEDIATE_ALERT),
    LO_U16(BLE_ATT_SVC_TX_POWER),
    HI_U16(BLE_ATT_SVC_TX_POWER),

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02, 0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                     /**< Scan response data. */
{
    0x11,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'D', 'X', '_', 'P', 'X', 'P', '_', 'R', 'e', 'p', 'o', 'r', 't', 'e', 'r',
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void lls_process_event(lls_evt_t *p_lls_evt);
static void fast_advertising_start(void);

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

    ble_gap_pair_enable(true);
#ifndef PTS_AUTO_TEST
    error_code = ble_gap_privacy_params_set(900, true);
    APP_ERROR_CHECK(error_code);
#endif

    // Set the default security parameters.
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY,
    };

    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    s_gap_adv_param.max_tx_pwr   = 0;

    s_gap_adv_time_param.max_adv_evt = 0;

    uint8_t  dev_name[32];
    uint16_t dev_name_len = 32;

    error_code = ble_gap_device_name_get(dev_name, &dev_name_len);
    APP_ERROR_CHECK(error_code);

    if (!strcmp((const char *)dev_name, BLE_GAP_DEVNAME_DEFAULT))
    {
        // Set the default Device Name.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_SEC_CON, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        // Set the Device Name is writable from the peer.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_SEC_CON, NULL, 0);
        APP_ERROR_CHECK(error_code);
    }

    s_app_adv_type = APP_ADV_TYPE_NONE;
}

static void lls_process_event(lls_evt_t *p_lls_evt)
{
    if (LLS_EVT_LINK_LOSS_ALERT == p_lls_evt->evt_type)
    {
        switch (p_lls_evt->alert_level)
        {
            case LLS_ALERT_LEVEL_NO_ALERT:
                APP_LOG_DEBUG("Proximity Reporter: LLS NO ALERT.");
                break;

            case LLS_ALERT_LEVEL_MILD_ALERT:
                APP_LOG_DEBUG("Proximity Reporter: LLS MILD ALERT.");
                break;

            case LLS_ALERT_LEVEL_HIGH_ALERT:
                APP_LOG_DEBUG("Proximity Reporter: LLS HIGH ALERT.");
                break;

            default:
                APP_LOG_DEBUG("Proximity Reporter: LLS Undefined ALERT.");
                break;
        }
    }
}

static void ias_process_event(ias_evt_t *p_ias_evt)
{
    if (IAS_EVT_ALERT_LEVEL_UPDATED == p_ias_evt->evt_type)
    {
        switch (p_ias_evt->alert_level)
        {
            case IAS_ALERT_NONE:
                APP_LOG_DEBUG("Proximity Reporter: IAS NO ALERT.");
                break;

            case IAS_ALERT_MILD:
                APP_LOG_DEBUG("Proximity Reporter: IAS MILD ALERT.");
                break;

            case IAS_ALERT_HIGH:
                APP_LOG_DEBUG("Proximity Reporter: IAS HIGH ALERT.");
                break;

            default:
                APP_LOG_DEBUG("Proximity Reporter: IAS Undefined ALERT.");
                break;
        }
    }
}

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t  error_code;
    ias_init_t ias_env_init;
    lls_init_t lls_env_init;
    tps_init_t tps_env_init;

    lls_env_init.initial_alert_level = LLS_ALERT_LEVEL_MILD_ALERT;
    lls_env_init.evt_handler         = lls_process_event;
    error_code                       = lls_service_init(&lls_env_init);
    APP_ERROR_CHECK(error_code);

    ias_env_init.evt_handler = ias_process_event;
    error_code               = ias_service_init(&ias_env_init);
    APP_ERROR_CHECK(error_code);

    tps_env_init.initial_tx_power_level = INIT_TX_POWER_LEVEL;
    error_code                          = tps_service_init(&tps_env_init);
    APP_ERROR_CHECK(error_code);
}

static void fast_advertising_start(void)
{
    sdk_err_t error_code;

    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_LIM_DISCOVERABLE;
    s_gap_adv_param.adv_intv_max = APP_ADV_FAST_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_FAST_MIN_INTERVAL;
    s_gap_adv_param.max_tx_pwr   = 0;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = FAST_ADV_DURATION;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_FAST;
    APP_LOG_DEBUG("Starting fast advertising.");
}

static void slow_advertising_start(void)
{
    sdk_err_t error_code;

    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.adv_intv_min = APP_ADV_SLOW_MIN_INTERVAL;
    s_gap_adv_param.adv_intv_max = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.max_tx_pwr   = 0;

    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set,  sizeof(s_adv_data_set));
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = 0;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_SLOW;
    APP_LOG_DEBUG("Starting slow advertising.");
}

static void app_adv_stopped_handler(uint8_t status, ble_gap_stopped_reason_t reason)
{
    if (status)
    {
        APP_LOG_DEBUG("Adverting started failed(0X%02X).", status);
    }

    if (BLE_GAP_STOPPED_REASON_TIMEOUT == reason && APP_ADV_TYPE_FAST == s_app_adv_type)
    {
        slow_advertising_start();
    }
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t          tk;

    if (NULL == p_enc_req)
    {
        return;
    }

    memset((uint8_t *)&cfm_enc, 0, sizeof(cfm_enc));

    switch (p_enc_req->req_type)
    {
        // User needs to decide whether to accept the pair request.
        case BLE_SEC_PAIR_REQ:
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept   = true;
            break;

        // User needs to input the password.
        case BLE_SEC_TK_REQ:
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept   = true;

            tk = 123456;    // 0x0001E240

            memset(cfm_enc.data.tk.key, 0, sizeof(cfm_enc.data.tk.key));
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

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    APP_LOG_INFO("Disconnected (0x%02X).", reason);
    fast_advertising_start();
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    s_app_adv_type = APP_ADV_TYPE_NONE;

    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);

#ifndef PTS_AUTO_TEST
    sdk_err_t error_code;
    error_code = ble_gap_tx_power_set(BLE_GAP_ACTIVITY_ROLE_CON, conn_idx, INIT_TX_POWER_LEVEL);
    APP_ERROR_CHECK(error_code);
#endif
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
            app_adv_stopped_handler(p_evt->evt_status, p_evt->evt.gapm_evt.params.adv_stop.reason);
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
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;
        
        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (SDK_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Link has been successfully encrypted");
            }
            break;
    }
}

void ble_app_init(void)
{
    sdk_err_t         error_code;
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t    version;

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
    APP_LOG_INFO("Proximity Reporter example started.");

    services_init();
    gap_params_init();
    fast_advertising_start();
}

