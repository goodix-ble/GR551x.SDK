/**
 *****************************************************************************************
 *
 * @file throughput.c
 *
 * @brief The implementation of throughput functions.
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
#include "throughput.h"
#include "user_app.h"
#include "utility.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_log.h"
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define MTU_MAX_SIZE       512

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
app_timer_id_t g_throughput_timer_id;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t  s_tx_data[MTU_MAX_SIZE] = {0x00};
static uint16_t s_mtu                   = 23;
static uint32_t s_all_send_packets;
static uint32_t s_all_send_bytes;
static uint32_t s_all_get_bytes;
static uint32_t s_last_all_get_bytes;
static uint32_t s_last_all_send_bytes;
static uint32_t s_throughput_1s_counter;
static bool     s_start_send_flag;
static bool     s_start_update_ci_flag;
static bool     s_start_update_pdu_flag;
static bool     s_start_update_phy_flag;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void thrpt_data_send(uint32_t index)
{
    sdk_err_t    error_code;
    s_tx_data[0] = LO_UINT32_T(index);
    s_tx_data[1] = L2_UINT32_T(index);
    s_tx_data[2] = L3_UINT32_T(index);
    s_tx_data[3] = HI_UINT32_T(index);
    s_tx_data[4] = 0x57;
    s_tx_data[5] = 0x75;
    error_code = ths_data_send(0, s_tx_data, s_mtu - 3);
    APP_ERROR_CHECK(error_code);
}

static void thrpt_param_set(uint8_t param[])
{
    ble_gap_conn_update_param_t gap_conn_param;
    uint16_t     tx_octects;
    uint16_t     tx_time;
    uint8_t      tx_phys, rx_phys, phy_opt;
    uint8_t      response[3];
    int8_t       tx_power;
    sdk_err_t    error_code;

    switch (param[0])
    {
        case THS_SETTINGS_TYPE_CI:
            gap_conn_param.interval_min  = BUILD_U16(param[1], param[2]);
            gap_conn_param.interval_max  = BUILD_U16(param[3], param[4]);
            gap_conn_param.slave_latency = BUILD_U16(param[5], param[6]);
            gap_conn_param.sup_timeout   = BUILD_U16(param[7], param[8]);
            error_code = ble_gap_conn_param_update(0, &gap_conn_param);
            APP_ERROR_CHECK(error_code);
            s_start_update_ci_flag = true;
            break;

        case THS_SETTINGS_TYPE_MTU:
            response[0] = THS_SETTINGS_TYPE_MTU;
            response[1] = LO_U16(s_mtu);
            response[2] = HI_U16(s_mtu);
            ths_settings_notify(0, response, 3);
            break;

        case THS_SETTINGS_TYPE_PDU:
            tx_octects = BUILD_U16(param[1], param[2]);
            tx_time    = BUILD_U16(param[3], param[4]);
            error_code = ble_gap_data_length_update(0, tx_octects, tx_time);
            APP_ERROR_CHECK(error_code);
            s_start_update_pdu_flag = true;
            break;

        case THS_SETTINGS_TYPE_PHY:
            tx_phys = param[1];
            rx_phys = param[2];
            phy_opt = param[3];
            error_code = ble_gap_phy_update(0, tx_phys, rx_phys, phy_opt);
            APP_ERROR_CHECK(error_code);
            s_start_update_phy_flag = true;
            break;

        case THS_SETTINGS_TYPE_TRANS_MODE:
            response[0] = THS_SETTINGS_TYPE_TRANS_MODE;
            error_code = ths_settings_notify(0, response, 1);
            APP_ERROR_CHECK(error_code);
            break;

        case THS_SETTINGS_TYPE_TX_POWER:
            if (0x01 == param[1])
            {
                tx_power = 0 - param[2];
            }
            else if (0x00 == param[1])
            {
                tx_power = param[2];
            }
            else
            {
                break;
            }
            error_code = ble_gap_tx_power_set(BLE_GAP_ACTIVITY_ROLE_CON, 0, tx_power);
            APP_ERROR_CHECK(error_code);
            response[0] = THS_SETTINGS_TYPE_TX_POWER;
            error_code = ths_settings_notify(0, response, 1);
            APP_ERROR_CHECK(error_code);
            break;

        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void thrpt_counter_handler(void *p_arg)
{
    uint32_t instant_ths = 0;
    uint32_t average_ths = 0;

    ble_gap_conn_info_get(0, BLE_GAP_GET_CON_RSSI);

    if ((s_last_all_get_bytes + s_last_all_send_bytes) <= (s_all_get_bytes + s_all_send_bytes))
    {
        s_throughput_1s_counter++;
        instant_ths = ((s_all_get_bytes + s_all_send_bytes) - (s_last_all_get_bytes + s_last_all_send_bytes)) / 125;
        average_ths =  (s_all_get_bytes + s_all_send_bytes) / s_throughput_1s_counter / 125;
        UNUSED_VARIABLE(instant_ths);
        UNUSED_VARIABLE(average_ths);
        APP_LOG_INFO("The instant throughput:  %dkbs", instant_ths);
        APP_LOG_INFO("The average throughput:  %dkbs(%ds)\r\n",average_ths, s_throughput_1s_counter);
    }
    else
    {
        s_all_get_bytes         = 0;
        s_all_send_bytes        = 0;
        s_throughput_1s_counter = 0;
    }

    s_last_all_get_bytes  = s_all_get_bytes;
    s_last_all_send_bytes = s_all_send_bytes;
}

void thrpt_event_process(ths_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case THS_EVT_DATA_RECEIVED:
            s_all_get_bytes += p_evt->length;
            break;

        case THS_EVT_SETTINGS_CHANGED:
            thrpt_param_set(p_evt->p_data);
            break;

        case THS_EVT_TOGGLE_SET:
            if (THS_TOGGLE_STATE_ON == p_evt->p_data[0])
            {
                if (THS_MASTER_WRITE_MODE == ths_transport_mode_get())
                {
                    s_start_send_flag = false;
                    s_all_get_bytes = 0;
                }
                else
                {
                    s_start_send_flag  = true;
                    s_all_get_bytes  = 0;
                    s_all_send_packets = 0;
                    s_all_send_bytes   = 0;
                    thrpt_data_send(s_all_send_packets);
                }
                app_timer_start(g_throughput_timer_id, 1000, NULL);
            }
            else if (THS_TOGGLE_STATE_OFF == p_evt->p_data[0])
            {
                s_start_send_flag = false;
                app_timer_stop(g_throughput_timer_id);
            }

            break;

        case THS_EVT_DATA_SENT:
            if (s_start_send_flag)
            {
                s_all_send_bytes += s_mtu;
                s_all_send_packets++;
                thrpt_data_send(s_all_send_packets);
            }

            break;

        default:
            break;
    }
}

bool thrpt_is_update_started(thrpt_param_type_t param_type)
{
    bool started = false;

    switch (param_type)
    {
        case THRPT_PARAM_CI:
            started = s_start_update_ci_flag;
            break;

        case THRPT_PARAM_PDU:
            started = s_start_update_pdu_flag;
            break;

        case THRPT_PARAM_PHY:
            started = s_start_update_phy_flag;
            break;

        default:
            break;
    }

    return started;
}

void thrpt_update_stat_clean(thrpt_param_type_t param_type)
{
    switch (param_type)
    {
        case THRPT_PARAM_CI:
            s_start_update_ci_flag  = false;
            break;

        case THRPT_PARAM_PDU:
            s_start_update_pdu_flag = false;
            break;

        case THRPT_PARAM_PHY:
            s_start_update_phy_flag = false;
            break;

        default:
            break;
    }
}

void thrpt_update_mtu(uint16_t new_mtu)
{
    s_mtu = new_mtu;
}

