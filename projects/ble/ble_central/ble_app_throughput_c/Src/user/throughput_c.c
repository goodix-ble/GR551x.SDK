/**
 *******************************************************************************
 *
 * @file throughput_c.c
 *
 * @brief The implementation of throughput client functions.
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
#include "throughput_c.h"
#include "ths_c.h"
#include "user_periph_setup.h"
#include "app_timer.h"
#include "grx_sys.h"
#include "utility.h"
#include <string.h>
#include "user_app.h"
#include "app_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define MTU_MAX_SIZE         512

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
app_timer_id_t g_throughput_timer_id;
extern uint8_t g_control_mode;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint16_t        s_mtu;
static int8_t          s_rssi;
static uint8_t         s_tx_data[MTU_MAX_SIZE];
static uint32_t        s_throughput_1s_counter;
static uint32_t        s_all_get_packets;
static uint32_t        s_all_get_bytes;
static uint32_t        s_all_send_packets;
static uint32_t        s_all_send_bytes;
static uint32_t        s_last_all_get_bytes;
static uint32_t        s_last_all_send_bytes;
static bool            s_is_testing;
bool                   s_is_enable_toggle;
ths_c_transport_mode_t s_trans_mode;
static uint8_t         s_cur_trans_mode;


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Sen data to peer.
 *
 * @param[in] index: Index of packet send.
 *****************************************************************************************
 */
static void thrpt_c_data_send(uint32_t index)
{
    s_tx_data[0] = LO_UINT32_T(index);
    s_tx_data[1] = L2_UINT32_T(index);
    s_tx_data[2] = L3_UINT32_T(index);
    s_tx_data[3] = HI_UINT32_T(index);
    s_tx_data[4] = 0x57;
    s_tx_data[5] = 0x75;

    ths_c_tx_data_send(0, s_tx_data, s_mtu - 3);

    s_all_send_bytes += (s_mtu - 3);
    s_all_send_packets++;
}

/**
 *****************************************************************************************
 * @brief Send connect set parameter.
 *
 * @param[in] p_data: Pointer to connect set parameter data.
 * @param[in] length: Length of connect set parameter data.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool ci_param_send(uint8_t *p_data, uint16_t length)
{
    uint8_t  ci_param_check_idx = 0;
    uint16_t data_length        = 0;
    uint16_t ci_param_check[4]  = {0};
    uint8_t  ci_param_send_data[9]   = {0};

    while (data_length < length)
    {
        if (p_data[data_length] <= '9' && p_data[data_length] >= '0')
        {
            ci_param_check[ci_param_check_idx] = ci_param_check[ci_param_check_idx] * 10 + (p_data[data_length] - '0');
        }
        else if (':' == p_data[data_length])
        {
            ci_param_check_idx++;
        }
        else
        {
            APP_LOG_INFO("Invalid parameter.");
            return false;
        }

        data_length++;
    }

    ci_param_send_data[0] = 0x00;
    ci_param_send_data[1] = LO_U16(ci_param_check[0]);
    ci_param_send_data[2] = HI_U16(ci_param_check[0]);
    ci_param_send_data[3] = LO_U16(ci_param_check[1]);
    ci_param_send_data[4] = HI_U16(ci_param_check[1]);
    ci_param_send_data[5] = LO_U16(ci_param_check[2]);
    ci_param_send_data[6] = HI_U16(ci_param_check[2]);
    ci_param_send_data[7] = LO_U16(ci_param_check[3]);
    ci_param_send_data[8] = HI_U16(ci_param_check[3]);


    ths_c_comm_param_send(0, ci_param_send_data, 9);
    return true;
}

/**
 *****************************************************************************************
 * @brief Send mtu set parameter.
 *
 * @param[in] p_data: Pointer to mtu set parameter data.
 * @param[in] length: Length of mtu set parameter data.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool mtu_param_send(uint8_t *p_data, uint16_t length)
{
    uint16_t mtu_set     = 0;
    uint16_t data_length = 0;

    while (data_length < length)
    {
        if (p_data[data_length] <= '9' && p_data[data_length] >= '0')
        {
            mtu_set = mtu_set * 10 + (p_data[data_length] - '0');
        }
        else
        {
            APP_LOG_INFO("Invalid set parameter.");
            return false;
        }

        data_length++;
    }


    if (mtu_set < 23 || mtu_set > 512)
    {
        APP_LOG_INFO("Invalid set parameter.");
        return false;
    }

    app_mtu_exchange(mtu_set);

    return true;
}

/**
 *****************************************************************************************
 * @brief Send pdu set parameter.
 *
 * @param[in] p_data: Pointer to pdu set parameter data.
 * @param[in] length: Length of pdu set parameter data.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool pdu_param_send(uint8_t *p_data, uint16_t length)
{
    uint16_t data_length         = 0;
    uint8_t  pdu_param_check_idx = 0;
    uint16_t pdu_param_check[2]  = {0};
    uint8_t  pdu_param_send_data[5]   = {0};

    while (data_length < length)
    {
        if (p_data[data_length] <= '9' && p_data[data_length] >= '0')
        {
            pdu_param_check[pdu_param_check_idx] = pdu_param_check[pdu_param_check_idx] * 10 + (p_data[data_length] - '0');
        }
        else if (':' == p_data[data_length])
        {
            pdu_param_check_idx++;
        }
        else
        {
            APP_LOG_INFO("Invalid set parameter.");
            return false;
        }

        data_length++;
    }

    pdu_param_send_data[0] = 0x02;
    pdu_param_send_data[1] = LO_U16(pdu_param_check[0]);
    pdu_param_send_data[2] = HI_U16(pdu_param_check[0]);
    pdu_param_send_data[3] = LO_U16(pdu_param_check[1]);
    pdu_param_send_data[4] = HI_U16(pdu_param_check[1]);

    ths_c_comm_param_send(0, pdu_param_send_data, 5);
    return true;
}

/**
 *****************************************************************************************
 * @brief Send phy set parameter.
 *
 * @param[in] p_data: Pointer to phy set parameter data.
 * @param[in] length: Length of phy set parameter.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool phy_param_send(uint8_t *p_data, uint16_t length)
{
    uint16_t data_length         = 0;
    uint8_t  phy_param_check_idx = 0;
    uint16_t phy_param_check[3]  = {0};
    uint8_t  phy_param_send_data[4]   = {0};

    while (data_length < length)
    {
        if (p_data[data_length] <= '9' && p_data[data_length] >= '0')
        {
            phy_param_check[phy_param_check_idx] = phy_param_check[phy_param_check_idx] * 10 + (p_data[data_length] - '0');
        }
        else if (':' == p_data[data_length])
        {
            phy_param_check_idx++;
        }
        else
        {
            APP_LOG_INFO("Invalid set parameter.");
            return false;
        }

        data_length++;
    }

    phy_param_send_data[0] = 0x03;
    phy_param_send_data[1] = phy_param_check[0];
    phy_param_send_data[2] = phy_param_check[1];
    phy_param_send_data[3] = phy_param_check[2];

    ble_gap_phy_update(0, phy_param_send_data[1], phy_param_send_data[2], phy_param_send_data[3]);

    ths_c_comm_param_send(0, phy_param_send_data, 4);
    return true;
}

/**
 *****************************************************************************************
 * @brief Send tx power set parameter.
 *
 * @param[in] p_data: Pointer to tx power set parameter data.
 * @param[in] length: Length of tx power set parameter.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool tx_pwr_param_send(uint8_t *p_data, uint16_t length)
{
    uint16_t data_length           = 0;
    uint8_t  tx_pwr_set            = {0};
    uint8_t  tx_pwr_param_send_data[3]  = {0};

    if (p_data[data_length] == '-')
    {
        data_length++;
        tx_pwr_param_send_data[1] = 0x01;
    }
    else
    {
        tx_pwr_param_send_data[1] = 0x00;
    }

    while (data_length < length)
    {
        if (p_data[data_length] <= '9' && p_data[data_length] >= '0')
        {
            tx_pwr_set = tx_pwr_set * 10 + (p_data[data_length] - '0');
        }
        else
        {
            APP_LOG_INFO("Invalid set parameter.");
            return false;
        }

        data_length++;
    }

    if (tx_pwr_param_send_data[1] == 0x01)
    {
        ble_gap_tx_power_set(BLE_GAP_ACTIVITY_ROLE_CON, 0, 0 - tx_pwr_set);
    }
    else
    {
        ble_gap_tx_power_set(BLE_GAP_ACTIVITY_ROLE_CON, 0, tx_pwr_set);
    }

    tx_pwr_param_send_data[0] = THS_C_SETTINGS_TYPE_TX_POWER;
    tx_pwr_param_send_data[2] = tx_pwr_set;

    ths_c_comm_param_send(0, tx_pwr_param_send_data, 3);

    return true;
}

/**
 *****************************************************************************************
 * @brief Send transport mode set parameter.
 *
 * @param[in] p_data: Pointer to transport mode set parameter data.
 * @param[in] length: Length of transport mode set parameter data.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool transport_mode_param_send(uint8_t *p_data, uint16_t length)
{
    uint8_t        transport_mode_param_send_data[2];

    transport_mode_param_send_data[0] = 0x04;
    transport_mode_param_send_data[1] = p_data[0] - '0';
    s_trans_mode                 = (ths_c_transport_mode_t)(p_data[0] - '0');

    if (s_cur_trans_mode == s_trans_mode)
    {
        APP_LOG_INFO("The transport mode is not changed.");
        return false;
    }
    else if (s_is_testing)
    {
        APP_LOG_INFO("The transport mode can not be changed.");
        return false;
    }
    else
    {
        s_cur_trans_mode = s_trans_mode;

        if ((s_trans_mode != THS_C_SLAVE_NOTIFY_MODE) && \
            (s_trans_mode != THS_C_MASTER_WRITE_MODE) && \
            (s_trans_mode != THS_C_DOUBLE_MODE))
        {
            APP_LOG_INFO("Invalid set parameter.");
            return false;
        }
    }


    ths_c_comm_param_send(0, transport_mode_param_send_data, 2);
    return true;
}

/**
 *****************************************************************************************
 * @brief Send toggle set parameter.
 *
 * @param[in] p_data: Pointer to toggle set parameter data.
 * @param[in] length: Length of toggle set parameter data.
 *
 * @return The result of send.
 *****************************************************************************************
 */
static bool toggle_mode_param_send(uint8_t *p_data, uint16_t length)
{
    static uint8_t current_toggle_value = 0xff;

    if (current_toggle_value == p_data[0])
    {
        APP_LOG_INFO("The toggle value is not changed.");
        return false;
    }
    else
    {
        current_toggle_value = p_data[0];

        if ('0' == p_data[0])
        {
            s_is_enable_toggle = false;
        }
        else if ('1' == p_data[0])
        {
            s_is_enable_toggle = true;
        }
        else
        {
            APP_LOG_INFO("Invalid parameter.");
            return false;
        }
    }

    ths_c_toggle_set(0, s_is_enable_toggle);
    return true;
}

/**
 *****************************************************************************************
 * @brief Print response information of parameter set for connection and transport mode.
 *
 * @param[in] p_data: Pointer to response data.
 * @param[in] length: Length of response data.
 *****************************************************************************************
 */
static void print_response_from_peer(uint8_t *p_data, uint16_t length)
{
    if (NULL == p_data)
    {
        return;
    }

    uint16_t interval;
    uint16_t latency;
    uint16_t time_out;

    switch (p_data[0])
    {
        case THS_C_SETTINGS_TYPE_CI:
            interval = BUILD_U16(p_data[2], p_data[3]);
            latency  = BUILD_U16(p_data[4], p_data[5]);
            time_out = BUILD_U16(p_data[6], p_data[7]);
            UNUSED_VARIABLE(interval);
            UNUSED_VARIABLE(latency);
            UNUSED_VARIABLE(time_out);
            APP_LOG_INFO("Connection parameter setted.Interval:%d(%0.2fms); Latency:%d; Time out:%d(%dms).",
                         interval,
                         interval * 1.25,
                         latency,
                         time_out,
                         time_out * 10);
            break;

        case THS_C_SETTINGS_TYPE_PDU:
            if (BLE_SUCCESS == p_data[1])
            {
                APP_LOG_INFO("PDU parameter set completely.");
            }
            else
            {
                APP_LOG_INFO("PDU parameter set failed, error code:0X%02x.", p_data[1]);
            }
            break;

        case THS_C_SETTINGS_TYPE_PHY:
            if (BLE_SUCCESS == p_data[1])
            {
                APP_LOG_INFO("PHY parameter set completely.");
            }
            else
            {
                APP_LOG_INFO("PHY parameter set failed, error code:0X%02x.", p_data[1]);
            }

            break;

        case THS_C_SETTINGS_TYPE_TRANS_MODE:
            APP_LOG_INFO("Transport mode parameter set completely.");
            break;

        case THS_C_SETTINGS_TYPE_TX_POWER:
            if (BLE_SUCCESS == p_data[1])
            {
                APP_LOG_INFO("TX power set completely.");
            }
            else
            {
                APP_LOG_INFO("TX power set fail.");
            }
            break;

        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void thrpt_c_init(void)
{
    s_mtu                   = 23;
    s_all_get_packets       = 0;
    s_all_get_bytes         = 0;
    s_all_send_packets      = 0;
    s_all_send_bytes        = 0;
    s_throughput_1s_counter = 0;
    s_is_testing            = false;
    s_trans_mode            = THS_C_SLAVE_NOTIFY_MODE;
    s_cur_trans_mode        = 0xff;
}

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
        UNUSED_VARIABLE(s_rssi);
        APP_LOG_INFO("The RSSI:                %ddBm", s_rssi);
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

void thrpt_c_event_process(ths_c_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case THS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Throughput Service discovery completely.");
            ths_c_tx_notify_set(p_evt->conn_idx, true);
            break;

        case THS_C_EVT_TX_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled TX Notification.");
            ths_c_setting_notify_set(p_evt->conn_idx, true);
            break;

        case THS_C_EVT_SETTING_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled SETTING Notification.");
            break;

        case THS_C_EVT_SETTING_RSP_RECEIVE:
            print_response_from_peer(p_evt->p_data, p_evt->length);
            break;

        case THS_C_EVT_TOGGLE_SET_SUCCESS:
            if (s_is_enable_toggle)
            {
                s_is_testing = true;
                APP_LOG_INFO("Throughput test start.");

                if (THS_C_MASTER_WRITE_MODE == s_trans_mode || THS_C_DOUBLE_MODE == s_trans_mode)
                {
                    thrpt_c_data_send(s_all_send_packets);
                }

                s_all_get_packets       = 0;
                s_all_get_bytes         = 0;
                s_all_send_packets      = 0;
                s_all_send_bytes        = 0;
                s_throughput_1s_counter = 0;
                app_timer_start(g_throughput_timer_id, 1000, NULL);
            }
            else
            {
                s_is_testing = false;
                app_timer_stop(g_throughput_timer_id);
                APP_LOG_INFO("Throughput test stop.");
            }
            break;

        case THS_C_EVT_TX_SUCCESS:
            if (s_is_testing)
            {
                thrpt_c_data_send(s_all_send_packets);
            }
            break;

        case THS_C_EVT_THRP_DATA_RECEIVE:
            if (s_is_testing)
            {
                s_all_get_bytes += p_evt->length;
                s_all_get_packets++;
            }

            break;

        default:
            break;
    }
}

void thrpt_c_data_parse(uint8_t *p_data, uint16_t length)
{
    if (p_data[length - 1] != '\n' || p_data[length - 2] != '\r')
    {
        APP_LOG_INFO("Invalid input.");
        return;
    }

    if (0 == memcmp(p_data, "SCAN", 4))
    {
        g_control_mode = 1;
        app_start_scan();
        return;
    }

    if (0 == memcmp(p_data, "TOGGLE_SET:", 11))
    {
        toggle_mode_param_send(&p_data[11], length - 13);
        return;
    }

    if (s_is_testing)
    {
        APP_LOG_INFO("Please stop throughput firstly. then set parameters.");
        return;
    }

    if (0 == memcmp(p_data, "CI:", 3))
    {
        ci_param_send(&p_data[3], length - 5);
    }
    else if (0 == memcmp(p_data, "MTU:", 4))
    {
        mtu_param_send(&p_data[4], length - 6);
    }
    else if (0 == memcmp(p_data, "PDU:", 4))
    {
        pdu_param_send(&p_data[4], length - 6);
    }
    else if (0 == memcmp(p_data, "PHY:", 4))
    {
        phy_param_send(&p_data[4], length - 6);
    }
    else if (0 == memcmp(p_data, "TRANS_MODE:", 11))
    {
        transport_mode_param_send(&p_data[11], length - 13);
    }
    else if (0 == memcmp(p_data, "TX_PWR:", 7))
    {
        tx_pwr_param_send(&p_data[7], length - 9);
    }
    else
    {
        APP_LOG_INFO("Invalid Input.");
    }
}

void thrpt_c_mtu_update(uint16_t mtu)
{
    s_mtu = mtu;
}

void thrpt_c_rssi_update(int8_t rssi)
{
    s_rssi = rssi;
}
