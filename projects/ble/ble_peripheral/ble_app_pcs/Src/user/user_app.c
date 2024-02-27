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
#include "pcs.h"
#include "gr_includes.h"
#include "board_SK.h"
#include "app_timer.h"
#include "app_log.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DEVICE_NAME                     "Goodix_Power"  /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL                1600            /**< The advertising interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT                 3000            /**< Advertising timeout (in units of 10ms). */
#define APP_PCS_NTF_NB                  100             /**< Number of app pcs notify. */
#define DEFAULT_MTU_SIZE                247             /**< Default mtu size. */
#define MAX_NB_LECB_DEFUALT             10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT            251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT             2120            /**< Defualt maximum packet transmission time. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool g_is_user_set_op;                                  /**< Flag for user set operation. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static bool                     s_is_notify_enable;              /**< Notificaiton is enabled or not. */
static uint32_t                 s_notify_counter;                /**< Counter of notificaiton. */
static ble_gap_adv_param_t      s_gap_adv_param;                 /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;            /**< Advertising time parameter. */
static adv_data_set_t           s_adv_data_set;                  /**< Advertising data set. */

// 3 byte for adv type
static uint8_t              s_adv_data_10b[7] =
{
    0x06,
    BLE_GAP_AD_TYPE_SHORTENED_NAME,
    'P', 'o', 'w', 'e', 'r'
};

// 3 byte for adv type
static uint8_t              s_adv_data_17b[14] =
{
    0x0d,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'P', 'o', 'w', 'e', 'r',
};

// 3 byte for adv type
static uint8_t              s_adv_data_24b[21] =
{
    0x14,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    0xF7,
    0x04,
    0x02,
    0x03,
};

// 3 byte for adv type
static uint8_t              s_adv_data_31b[28] =
{
    0x0d,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'P', 'o', 'w', 'e', 'r',

    0x0d,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    0xF7,
    0x04,
    0x02,
    0x03,
};

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
    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);

    memcpy(s_adv_data_set.adv_data, s_adv_data_31b, 28);
    s_adv_data_set.length = 28;

    s_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    ble_gap_l2cap_params_set(DEFAULT_MTU_SIZE, DEFAULT_MTU_SIZE, 1);
}

/**
 *****************************************************************************************
 * @brief Notify data.
 *****************************************************************************************
 */
static void pcs_tx_data_notify(void)
{
    uint8_t notify_data[PCS_MAX_DATA_LEN] = {0};

    notify_data[0] = LO_UINT32_T(s_notify_counter);
    notify_data[1] = L2_UINT32_T(s_notify_counter);
    notify_data[2] = L3_UINT32_T(s_notify_counter);
    notify_data[3] = HI_UINT32_T(s_notify_counter);

    pcs_tx_data_send(0, notify_data, PCS_MAX_DATA_LEN);
}

/**
 *****************************************************************************************
 * @brief Parse setting parameters.
 *****************************************************************************************
 */
void pcs_param_parse(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    uint8_t                     response[2];
    uint8_t                     tx_phys;
    uint8_t                     rx_phys;
    uint8_t                     phy_opt;
    int8_t                      tx_power_set;
    sdk_err_t                   error_code;
    ble_gap_conn_update_param_t gap_conn_param;

    switch (p_data[0])
    {
        case PCS_SETTING_TYPE_ADV_INTERVAL:
            s_gap_adv_param.adv_intv_max = BUILD_U16(p_data[1], p_data[2]);
            s_gap_adv_param.adv_intv_min = BUILD_U16(p_data[1], p_data[2]);
            response[0] = PCS_SETTING_TYPE_ADV_INTERVAL;
            response[1] = PCS_SET_PARAM_SUCCESS;
            pcs_setting_reply(0, response, 2);
            break;

        case PCS_SETTING_TYPE_CONN_PARAM:
            gap_conn_param.interval_min  = BUILD_U16(p_data[1], p_data[2]);
            gap_conn_param.interval_max  = BUILD_U16(p_data[3], p_data[4]);
            gap_conn_param.slave_latency = BUILD_U16(p_data[5], p_data[6]);
            gap_conn_param.sup_timeout   = BUILD_U16(p_data[7], p_data[8]);

            if (SDK_SUCCESS == ble_gap_conn_param_update(conn_idx, &gap_conn_param))
            {
                g_is_user_set_op = true;
            }
            break;

        case PCS_SETTING_TYPE_PHY:
            tx_phys = p_data[1];
            rx_phys = p_data[2];
            phy_opt = p_data[3];

            if (SDK_SUCCESS == ble_gap_phy_update(0, tx_phys, rx_phys, phy_opt))
            {
                g_is_user_set_op = true;
            }
            break;

        case PCS_SETTING_TYPE_ADV_DATA:
            response[0] = PCS_SETTING_TYPE_ADV_DATA;
            response[1] = PCS_SET_PARAM_SUCCESS;

            if (PCS_SET_ADV_DATA_3B == p_data[1])
            {
                s_adv_data_set.length = 0;     // 3 byte for adv type
            }
            else if (PCS_SET_ADV_DATA_10B == p_data[1])
            {
                memcpy(s_adv_data_set.adv_data, s_adv_data_10b, 7);
                s_adv_data_set.length   = 7;     // 3 byte for adv type
            }
            else if (PCS_SET_ADV_DATA_17B == p_data[1])
            {
                memcpy(s_adv_data_set.adv_data, s_adv_data_17b, 14);
                s_adv_data_set.length   = 14;     // 3 byte for adv type
            }
            else if (PCS_SET_ADV_DATA_24B == p_data[1])
            {
                memcpy(s_adv_data_set.adv_data, s_adv_data_24b, 21);
                s_adv_data_set.length   = 21;     // 3 byte for adv type
            }
            else if (PCS_SET_ADV_DATA_31B == p_data[1])
            {
                memcpy(s_adv_data_set.adv_data, s_adv_data_31b, 28);
                s_adv_data_set.length   = 28;     // 3 byte for adv type
            }
            else
            {
                response[1] = PCS_SET_PARAM_FAIL;
            }
            pcs_setting_reply(0, response, 2);
            break;

        case PCS_SETTING_TYPE_TX_POWER:
            if (0x01 == p_data[1])
            {
                tx_power_set = 0 - p_data[2];
            }
            else if (0x00 == p_data[1])
            {
                tx_power_set = p_data[2];
            }
            s_gap_adv_param.max_tx_pwr = tx_power_set;

            error_code = ble_gap_tx_power_set(BLE_GAP_ACTIVITY_ROLE_CON, conn_idx, tx_power_set);

            response[0] = PCS_SETTING_TYPE_TX_POWER;
            response[1] = SDK_SUCCESS == error_code ? PCS_SET_PARAM_SUCCESS : PCS_SET_PARAM_FAIL;

            pcs_setting_reply(0, response, 2);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process PCS service event
 *
 * @param[in] p_evt: Pointer to PCS event stucture.
 *****************************************************************************************
 */
static void pcs_service_event_process(pcs_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case PCS_EVT_TX_ENABLE:
            s_is_notify_enable = true;
            pcs_tx_data_notify();
            break;

        case PCS_EVT_TX_DISABLE:
            s_is_notify_enable = false;
            break;

        case PCS_EVT_TX_DATA_SENT:
            if (s_is_notify_enable)
            {
                s_notify_counter++;
                pcs_tx_data_notify();
            }
            break;

        case PCS_EVT_PARAM_SET:
            pcs_param_parse(p_evt->conn_idx, p_evt->p_data, p_evt->length);
            break;

        case PCS_EVT_DISCONNECTED:
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
    pcs_init_t pcs_init;

    pcs_init.evt_handler = pcs_service_event_process;
    pcs_service_init(&pcs_init);
}

static void app_gap_phy_update_handler(uint8_t conn_idx, uint8_t status)
{
    uint8_t response[2];

    if (g_is_user_set_op)
    {
        g_is_user_set_op = false;
        response[0] = PCS_SETTING_TYPE_PHY;
        response[1] = status == BLE_SUCCESS ? PCS_SET_PARAM_SUCCESS : PCS_SET_PARAM_FAIL;
        pcs_setting_reply(0, response, 2);
    }
}

static void app_gap_connection_update_handler(uint8_t conn_idx, uint8_t status, const ble_gap_evt_conn_param_updated_t *p_conn_param_updated)
{
    uint8_t response[7];

    if (g_is_user_set_op)
    {
        if (BLE_SUCCESS != status)
        {
            response[0] = PCS_SETTING_TYPE_CONN_PARAM;
            response[1] = PCS_SET_PARAM_FAIL;
            pcs_setting_reply(0, response, 2);
        }
        else
        {
            response[0] = PCS_SETTING_TYPE_CONN_PARAM;
            response[1] = LO_U16(p_conn_param_updated->conn_interval);
            response[2] = HI_U16(p_conn_param_updated->conn_interval);
            response[3] = LO_U16(p_conn_param_updated->slave_latency);
            response[4] = HI_U16(p_conn_param_updated->slave_latency);
            response[5] = LO_U16(p_conn_param_updated->sup_timeout);
            response[6] = HI_U16(p_conn_param_updated->sup_timeout);
            pcs_setting_reply(0, response, 7);
        }

        g_is_user_set_op = false;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            break;
            
        case BLE_GAPC_EVT_PHY_UPDATED:
            app_gap_phy_update_handler(p_evt->evt.gapc_evt.index, p_evt->evt_status);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            app_gap_connection_update_handler(p_evt->evt.gapc_evt.index, p_evt->evt_status, &(p_evt->evt.gapc_evt.params.conn_param_updated));
            break;
    }
}

void ble_app_init(void)
{
    services_init();
    gap_params_init();
    gap_advertising_start();
}

void gap_advertising_start(void)
{
    ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set.adv_data, s_adv_data_set.length);

    s_gap_adv_time_param.duration    = APP_ADV_TIMEOUT;
    s_gap_adv_time_param.max_adv_evt = 0;

    ble_gap_adv_start(0, &s_gap_adv_time_param);
}

