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
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define BEACON_CFG

/**@brief Gapm config data. */
#define APP_ADV_INTERVAL                160                            /**< In <iBeacon Spec>, using a fixed 100ms advertising interval (in units of 0.625 ms). */
#define DEFAULE_ADV_INDEX               0                              /**< The value is ZERO for legacy advertising. */
#define BEACON_UPDATE_INTERVAL          5000                           /**< The beacon minor major update interval(in unit of 1 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                       /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;                  /**< Advertising time parameter. */

static app_timer_id_t           s_beacon_update_timer_id;              /**< Beacon update timer id. */
static uint16_t                 s_beacon_update_count;                 /**< Beacon update count. */

static bool                     s_major_update_flag;                   /**< Flag indicate major value can update. */
static bool                     s_minor_update_flag;                   /**< Flag indicate minor value can update. */

static uint8_t s_adv_data_set[] =                                      /**< Advertising data. */
{
#ifdef BEACON_CFG
    // Manufacturer specific adv data type
    0x1A,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Company Identifier
    0x4c, 0x00,
    // Beacon Data Type
    0x02,
    // Data Length
    0x15,
    // UUID - Variable based on different use cases/applications
    0x01, 0x12, 0x23, 0x34,
    0x45, 0x56, 0x67, 0x78,
    0x89, 0x9a, 0xab, 0xbc,
    0xcd, 0xde, 0xef, 0xf0,
    // Major value for identifying Beacons
    0x00,
    0x01,
    // Minor value for identifying Beacons
    0x00,
    0x01,
    // The Beacon's measured RSSI at meter distance in dBm
    0xc3
#else
    // Manufacturer specific adv data type
    0x1A,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    0xF7, 0x04,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    1, 9, 8, 4,
    0xFF
#endif
};

#ifndef BEACON_CFG
static const uint8_t s_adv_rsp_data_set[] =                          /**< Scan responce data. */
{
    0x13,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'B','r','o','a','d','c','a', 's', 't', 'e', 'r',
};
#endif

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

    ble_gap_pair_enable(false);

    s_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
#ifdef BEACON_CFG
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_BROADCASTER;
#else
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_SCAN_IND;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
#endif

    error_code = ble_gap_adv_param_set(DEFAULE_ADV_INDEX, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(DEFAULE_ADV_INDEX,
                                      BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set,
                                      sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);
#ifndef BEACON_CFG
    error_code = ble_gap_adv_data_set(DEFAULE_ADV_INDEX,
                                      BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set,
                                      sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 * @brief The timeout handler of beacon update timer.
 *****************************************************************************************
 */
static void beacon_timer_event_process(void *p_arg)
{
    sdk_err_t error_code;

    error_code = ble_gap_adv_stop(DEFAULE_ADV_INDEX);
    APP_ERROR_CHECK(error_code);

    s_beacon_update_count++;
    if (s_beacon_update_count % 2 == 0)
    {
        s_major_update_flag = true;
    }
    else
    {
        s_minor_update_flag = true;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void update_major_minor_value(void)
{
    sdk_err_t error_code;

    if (true == s_major_update_flag)
    {
        s_major_update_flag = false;
        s_adv_data_set[22] = HI_U16(s_beacon_update_count);
        s_adv_data_set[23] = LO_U16(s_beacon_update_count);
    }

    if (true == s_minor_update_flag)
    {
        s_minor_update_flag = false;
        s_adv_data_set[24] = HI_U16(s_beacon_update_count);
        s_adv_data_set[25] = LO_U16(s_beacon_update_count);
    }

    error_code = ble_gap_adv_data_set(DEFAULE_ADV_INDEX,
                                      BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set,
                                      sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

#ifndef BEACON_CFG
    error_code = ble_gap_adv_data_set(DEFAULE_ADV_INDEX,
                                      BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set,
                                      sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
#endif
    error_code = ble_gap_adv_param_set(DEFAULE_ADV_INDEX, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(DEFAULE_ADV_INDEX, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

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
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                update_major_minor_value();
            }
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
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
#ifdef BEACON_CFG
    APP_LOG_INFO("Beacon example started.");
#else
    APP_LOG_INFO("Broadcaster example started.");
#endif

    gap_params_init();

    error_code = app_timer_create(&s_beacon_update_timer_id, ATIMER_REPEAT, beacon_timer_event_process);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_start(s_beacon_update_timer_id, BEACON_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(DEFAULE_ADV_INDEX, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

