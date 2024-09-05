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
#include "user_mouse.h"
#include "hids.h"
#include "bas.h"
#include "dis.h"
#include "grx_sys.h"
#include "sensorsim.h"
#include "app_timer.h"
#include "app_log.h"
#include "utility.h"
#include "app_error.h"
#include "ble_advertising.h"
#include "ble_connect.h"
#include "ble_error.h"
#ifdef PTS_AUTO_TEST
#include "board_SK.h"
#endif

/*
 * DEFINES
 *****************************************************************************************
 */

/**@brief Gapm config data. */
#define DEVICE_NAME                           "Goodix_Mouse"   /**< Device Name which will be set in GAP. */
#define APP_ADV_DIRECTED_LOW_DUTY_INTERVAL    32               /**< The low-duty direct advertising interval (in units of 0.625 ms). */
#define APP_ADV_DIRECTED_LOW_DUTY_TIMEOUT     500              /**< The low-duty direct advertising timeout (in units of 10 ms). */

#if defined(SWIFT_PAIR_SUPPORTED)
#define APP_ADV_FAST_INTERVAL                 48               /**< The fast advertising interval (in units of 0.625 ms). */
#define APP_ADV_FAST_TIMEOUT                  3000             /**< The fast advertising timeout (in units of 10 ms). */
#define APP_ADV_SLOW_INTERVAL                 160              /**< The slow advertising max interval (in units of 0.625 ms). */
#else
#define APP_ADV_FAST_INTERVAL                 80               /**< The fast advertising interval (in units of 0.625 ms). */
#define APP_ADV_FAST_TIMEOUT                  1000             /**< The fast advertising timeout (in units of 10 ms). */
#define APP_ADV_SLOW_INTERVAL                 320              /**< The slow advertising max interval (in units of 0.625 ms). */
#endif

#define APP_ADV_SLOW_TIMEOUT                  0                /**< The slow advertising timeout (in units of 10 ms). */
#define APP_PREF_CONN_INTERVAL_MAX            160              /**< Minimum prefer connection interval (in units of 1.25 ms). */
#define APP_PREF_CONN_INTERVAL_MIN            160              /**< Maximum prefer connection interval (in units of 1.25 ms). */
#define APP_PREF_CONN_SLAVE_LATENCY           0                /**< Prefer connection slave latency. */
#define APP_PREF_CONN_TIMEOUT                 400              /**< Prefer connection timeout. (in units of 10 ms). */
#define APP_CONN_PARAM_UPDATE_TIMES           3                /**< The times of attempts to update conn parameter. */
#define APP_FIRST_CONN_PARAM_UPDATE_DELAY     10000            /**< The waiting time before initiating connection parameter update
                                                                    for the first time. (in units of 10 ms). */
#define APP_SECOND_CONN_PARAM_UPDATE_DELAY    30000            /**< The waiting time before initiating connection parameter update
                                                                    for the second time. (in units of 10 ms). */
#define PRIVACY_RENEW_DURATION                150              /**< 150 seconds */

/**< macros for simulating hardware. */
#define BATTERY_LEVEL_MIN                     81
#define BATTERY_LEVEL_MAX                     100
#define BATTERY_LEVEL_INCREAMENT              1
#define HW_SIM_UPDATE_INTERVAL                2000

/**< macros for Microsoft Swift Pair Feature. */
#if defined(SWIFT_PAIR_SUPPORTED)
#define MICROSOFT_VENDOR_ID                   0x0006           /**< Microsoft Vendor ID.*/
#define MICROSOFT_BEACON_ID                   0x03             /**< Microsoft Beacon ID, used to indicate that Swift Pair feature is supported. */
#define MICROSOFT_BEACON_SUB_SCENARIO         0x00             /**< Microsoft Beacon Sub Scenario, used to indicate how the peripheral will pair using Swift Pair feature. */
#define RESERVED_RSSI_BYTE                    0x80             /**< Reserved RSSI byte, used to maintain forwards and backwards compatibility. */
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if defined(SWIFT_PAIR_SUPPORTED)
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0D,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'M', 'o', 'u', 's', 'e',
    
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_MOUSE),
    HI_U16(BLE_APPEARANCE_HID_MOUSE),
    
    0x06,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    LO_U16(MICROSOFT_VENDOR_ID),
    HI_U16(MICROSOFT_VENDOR_ID),
    MICROSOFT_BEACON_ID,
    MICROSOFT_BEACON_SUB_SCENARIO,
    RESERVED_RSSI_BYTE,
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};

static const uint8_t s_new_adv_data_set[] =
{
    0x0D,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'M', 'o', 'u', 's', 'e'
};

static const uint8_t s_new_adv_rsp_data_set[] =
{
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_MOUSE),
    HI_U16(BLE_APPEARANCE_HID_MOUSE),

    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};
#else
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0D,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'M', 'o', 'u', 's', 'e'
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_MOUSE),
    HI_U16(BLE_APPEARANCE_HID_MOUSE),

    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};
#endif

static ble_gap_bdaddr_t s_conn_bdaddr;

static app_timer_id_t    s_hw_simulator_timer_id;
static sensorsim_cfg_t   s_battery_sim_cfg;
static sensorsim_state_t s_battery_sim_state;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize battery simulator module.
 *****************************************************************************************
 */
static void hw_simulator_init(void)
{
    s_battery_sim_cfg.min          = BATTERY_LEVEL_MIN;
    s_battery_sim_cfg.max          = BATTERY_LEVEL_MAX;
    s_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREAMENT;
    s_battery_sim_cfg.start_at_max = true;
    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);
}

/**
 *****************************************************************************************
 * @brief Battery timeout handler.
 *****************************************************************************************
 */
static void hw_simulator_timer_handler(void *p_arg)
{
    uint8_t battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state,
                                                       &s_battery_sim_cfg);
    bas_batt_lvl_update(0, 0, battery_level);

#ifdef PTS_AUTO_TEST
    uint8_t key_id;
    if (battery_level > 90)
    {
        key_id = BSP_KEY_DOWN_ID;
    }
    else
    {
        key_id = BSP_KEY_UP_ID;
    }
    app_key_evt_handler(key_id, APP_KEY_CONTINUE_CLICK);
#endif
}

/**
 *****************************************************************************************
 * @brief Set peer address for dirct advertising.
 *****************************************************************************************
 */
static void direct_adv_addr_set(void)
{
    sdk_err_t            error_code;
    ble_gap_white_list_t whitelist;

    error_code = ble_gap_whitelist_get(&whitelist);
    APP_ERROR_CHECK(error_code);

    if (whitelist.num)
    {
        ble_advertising_dir_addr_fill(&(whitelist.items[whitelist.num - 1]));
    }
}

/**
 *****************************************************************************************
 * @brief Error handler for ble_advertising module.
 *****************************************************************************************
 */
static void ble_adv_err_handler(uint8_t err_code)
{
    APP_LOG_ERROR("ble_adv_err_evt_handler: 0x%02x.", err_code);
}

/**
 *****************************************************************************************
 * @brief Event handler for ble_advertising module.
 *****************************************************************************************
 */
static void ble_adv_evt_handler(ble_adv_evt_type_t evt)
{
    switch (evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            APP_LOG_INFO("BLE_ADV_EVT_DIRECTED_HIGH_DUTY.");
            break;
        case BLE_ADV_EVT_DIRECTED_LOW_DUTY:
            APP_LOG_INFO("BLE_ADV_EVT_DIRECTED_LOW_DUTY.");
            break;
        case BLE_ADV_EVT_FAST:
#if defined(SWIFT_PAIR_SUPPORTED)
            ble_advertising_adv_data_update(s_new_adv_data_set, sizeof(s_new_adv_data_set), s_new_adv_rsp_data_set, sizeof(s_new_adv_rsp_data_set));
#endif
            APP_LOG_INFO("BLE_ADV_EVT_FAST.");
            break;
        case BLE_ADV_EVT_SLOW:
            APP_LOG_INFO("BLE_ADV_EVT_SLOW.");
            break;
        case BLE_ADV_EVT_DIR_ADDR_REQUEST:
            APP_LOG_INFO("BLE_ADV_EVT_DIR_ADDR_REQUEST.");
            direct_adv_addr_set();
            break;
        case BLE_ADV_EVT_INVALID:
            APP_LOG_INFO("Invalid Adv event type.");
            break;      
        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Initialize BLE security configration.
 *****************************************************************************************
 */
static void app_sec_init(bool erase_bond)
{
    sdk_err_t    error_code;

    ble_gap_pair_enable(true);
    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME, 
                                         strlen(DEVICE_NAME));

#ifdef PTS_AUTO_TEST
    /* If enable privacy mode, IUT refuses the connection request with Public
     * Address of PTS Dongle in the next test case. */
    error_code = ble_gap_privacy_params_set(900, false);
    APP_ERROR_CHECK(error_code);
#else
    error_code = ble_gap_privacy_params_set(PRIVACY_RENEW_DURATION, true);
    APP_ERROR_CHECK(error_code);
#endif

    if (erase_bond)
    {
        error_code = ble_gap_bond_devs_clear();
        APP_ERROR_CHECK(error_code);

        error_code = ble_gap_whitelist_clear();
        APP_ERROR_CHECK(error_code);

        APP_LOG_DEBUG("Bonding and Whitelist are cleared");
    }

    //set the default security parameters.
#if defined(SWIFT_PAIR_SUPPORTED)
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
    };
#else
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL3,
        .io_cap    = BLE_SEC_IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ALL,
        .rkey_dist = BLE_SEC_KDIST_ALL,
    };
#endif
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Initialize ble_advertising module.
 *****************************************************************************************
 */
static void app_adv_init(void)
{
    ble_adv_init_t   ble_adv_init;

    memset(&ble_adv_init, 0, sizeof(ble_adv_init));

    ble_adv_init.err_handler = ble_adv_err_handler;
    ble_adv_init.evt_handler = ble_adv_evt_handler;
    ble_adv_init.adv_mode_cfg.multi_link_enabled = false;
    ble_adv_init.adv_mode_cfg.adv_on_disconnect_enabled = true;
    ble_adv_init.adv_mode_cfg.adv_directed_high_duty_enabled = true;
    ble_adv_init.adv_mode_cfg.adv_directed_low_duty_enabled = true;
    ble_adv_init.adv_mode_cfg.adv_fast_enabled = true;
    ble_adv_init.adv_mode_cfg.adv_slow_enabled = true;
    ble_adv_init.adv_mode_cfg.adv_extended_enabled = false;
    ble_adv_init.adv_mode_cfg.adv_directed_low_duty_interval = APP_ADV_DIRECTED_LOW_DUTY_INTERVAL;
    ble_adv_init.adv_mode_cfg.adv_directed_low_duty_timeout = APP_ADV_DIRECTED_LOW_DUTY_TIMEOUT;
    ble_adv_init.adv_mode_cfg.adv_fast_interval = APP_ADV_FAST_INTERVAL;
    ble_adv_init.adv_mode_cfg.adv_fast_timeout  = APP_ADV_FAST_TIMEOUT;
    ble_adv_init.adv_mode_cfg.adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    ble_adv_init.adv_mode_cfg.adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT;
    ble_adv_init.adv_data.p_data = s_adv_data_set;
    ble_adv_init.adv_data.length = sizeof(s_adv_data_set);
    ble_adv_init.srp_data.p_data = s_adv_rsp_data_set;
    ble_adv_init.srp_data.length = sizeof(s_adv_rsp_data_set);

    ble_advertising_init(&ble_adv_init);
}


/**
 *****************************************************************************************
 * @brief Error handler for ble_connect module.
 *****************************************************************************************
 */
static void ble_conn_err_handler(uint8_t err_code)
{
    APP_LOG_ERROR("ble_conn_err_handler:0x%02x", err_code);
}

/**
 *****************************************************************************************
 * @brief Event handler for ble_connect module.
 *****************************************************************************************
 */
static void ble_conn_evt_handler(ble_conn_evt_t *p_evt)
{
    uint8_t connect_cnts = ble_connect_established_cnt_get();
    ble_gap_bdaddr_t *p_peer_bdaddr = &p_evt->param.p_link_info->peer_addr;

    switch (p_evt->evt_type)
    {
        case BLE_CONN_EVT_CONNECTED:
            memcpy(&s_conn_bdaddr.gap_addr, p_peer_bdaddr, sizeof(ble_gap_bdaddr_t));

            APP_LOG_INFO("BLE_CONN_EVT_CONNECTED. Connection counts = %d", connect_cnts);
            APP_LOG_INFO("Peer:%02X:%02X:%02X:%02X:%02X:%02X.",
                        p_evt->param.p_link_info->peer_addr.gap_addr.addr[5],
                        p_evt->param.p_link_info->peer_addr.gap_addr.addr[4],
                        p_evt->param.p_link_info->peer_addr.gap_addr.addr[3],
                        p_evt->param.p_link_info->peer_addr.gap_addr.addr[2],
                        p_evt->param.p_link_info->peer_addr.gap_addr.addr[1],
                        p_evt->param.p_link_info->peer_addr.gap_addr.addr[0]);

#if defined(SWIFT_PAIR_SUPPORTED)
            ble_sec_enc_start(0);
#endif

            break;

        case BLE_CONN_EVT_DISCONNECTED:
            APP_LOG_INFO("BLE_CONN_EVT_DISCONNECTED: 0x%02x. Connection counts = %d", p_evt->param.disconn_reason, connect_cnts);
            break;

        case BLE_CONN_EVT_PATAM_UPDATED:
            APP_LOG_INFO("BLE_CONN_EVT_PATAM_UPDATED.");
            APP_LOG_INFO("Connection update completed, intvl %0.2fms, ltcy %d, to %dms",
                        p_evt->param.p_update_param->conn_interval * 1.25,
                        p_evt->param.p_update_param->slave_latency,
                        p_evt->param.p_update_param->sup_timeout * 10);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Initialize ble_connect module.
 *****************************************************************************************
 */
static void app_conn_init(void)
{
    sdk_err_t            err_code;
    ble_conn_init_t      conn_init;
    ble_gap_conn_param_t gap_conn_param =
    {
        .interval_min  = APP_PREF_CONN_INTERVAL_MIN,
        .interval_max  = APP_PREF_CONN_INTERVAL_MAX,
        .slave_latency = APP_PREF_CONN_SLAVE_LATENCY,
        .sup_timeout   = APP_PREF_CONN_TIMEOUT
    };

    memset(&conn_init, 0, sizeof(conn_init));

    conn_init.conn_param_manage_enable      = true;
    conn_init.p_conn_param                  = &gap_conn_param;
    conn_init.first_conn_param_update_delay = APP_FIRST_CONN_PARAM_UPDATE_DELAY;
    conn_init.next_conn_param_update_delay  = APP_SECOND_CONN_PARAM_UPDATE_DELAY;
    conn_init.max_conn_param_update_count   = APP_CONN_PARAM_UPDATE_TIMES;
    conn_init.disconnect_on_fail            = true;
    conn_init.evt_handler                   = ble_conn_evt_handler;
    conn_init.err_handler                   = ble_conn_err_handler;

    err_code = ble_connect_init(&conn_init);
    APP_ERROR_CHECK(err_code);
}

/**
 *****************************************************************************************
 * @brief Start advertising.
 *****************************************************************************************
 */
static void app_adv_start(void)
{
    sdk_err_t error_code;

    error_code = ble_advertising_start(BLE_ADV_MODE_DIRECTED_HIGH_DUTY);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Print device mac address.
 *****************************************************************************************
 */
static void dev_mac_info_print(void)
{
    sdk_err_t         error_code;
    ble_gap_bdaddr_t  bd_addr;

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
}

/**
 *****************************************************************************************
 * @brief Initialize APP timers.
 *****************************************************************************************
 */
static void timer_init(void)
{
    sdk_err_t error_code;
    error_code = app_timer_create(&s_hw_simulator_timer_id, ATIMER_REPEAT,
                                  hw_simulator_timer_handler);
    APP_ERROR_CHECK(error_code);
    error_code = app_timer_start(s_hw_simulator_timer_id,
                                 HW_SIM_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Initialize services.
 *****************************************************************************************
 */
static void services_init(void)
{
    user_mouse_service_init();
}

static void app_paring_succeed_handler(void)
{
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
#if !defined(SWIFT_PAIR_SUPPORTED)
    uint32_t      tk;
#endif

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
            cfm_enc.accept   = true;
            break;

#if !defined(SWIFT_PAIR_SUPPORTED)
        // user need to input the password
        case BLE_SEC_TK_REQ:
            APP_LOG_INFO("Please Input pin code: 123456.");
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept   = true;
            tk = 123456;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;
#endif

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    if (!p_evt->evt_status)
    {
        ble_advertising_evt_on_ble_capture(p_evt);
        ble_connect_evt_on_ble_capture(p_evt);
    }

    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("Link has been successfully encrypted.");
                app_paring_succeed_handler();
            }
            else
            {
                APP_LOG_INFO("Pairing failed for error 0x%x.", p_evt->evt_status);
            }
            break;
    }
}

void ble_app_init(void)
{
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                version.major, version.minor, version.build, version.commit_id);
    APP_LOG_INFO("HID Mouse example started.");
    hw_simulator_init();
    timer_init();
    dev_mac_info_print();
    services_init();
    app_sec_init(ble_bond_state_get());
    app_conn_init();
    app_adv_init();
    app_adv_start();
}

