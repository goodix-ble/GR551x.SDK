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
#include "transport_scheduler.h"
#include "user_periph_setup.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   160              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     160              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   10000           /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               15              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               15              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */
#define UPDATE_APP_CONN_INTERVAL_MIN        59               /**< Update minimal connection interval(in units of 1.25 ms). */
#define UPDATE_APP_CONN_INTERVAL_MAX        59              /**< Update maximal connection interval(in units of 1.25 ms). */
#define MAX_NB_LECB_DEFUALT                 2              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_bdaddr_t         s_target_bdaddr;                                                       /**< Target board address. */
static const uint8_t            peer_bd_addr[SYS_BD_ADDR_LEN] = {0x03, 0xae, 0xdf, 0x3e, 0xcb, 0xea};  /**< Peer device address. */
static ble_gap_init_param_t     gap_connect_param;
static uint8_t                  adv_header = 0;
static uint32_t                 adv_crc    = 0;
static uint16_t                 slave_receive_packet_num = 0;
static uint8_t                  adv_data[247] = {0};

uint8_t            rx_buffer[CFG_BOND_DEVS][247];               /**< Buffer used to receiving data. */

/**< security parameters. */
static ble_sec_param_t s_sec_param =
{
    .level     = BLE_SEC_MODE1_LEVEL1,
    .io_cap    = BLE_SEC_IO_KEYBOARD_ONLY,
    .oob       = false,
    .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM,
    .key_size  = 16,
    .ikey_dist = BLE_SEC_KDIST_ENCKEY,
    .rkey_dist = BLE_SEC_KDIST_ENCKEY,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern bd_info s_bd_info;
 
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

    ble_gap_pair_enable(true);
    ble_sec_params_set(&s_sec_param);
    ble_gap_privacy_params_set(150, true);
    
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

static void receieve_packet_check_init(void)
{
    adv_header = 0xA0;
    adv_crc    = 0xA00A0A;
    
    uint8_t tmp[10] = "Goodix_BLE";
    uint8_t *buffer = adv_data;
    uint8_t len = 10;
    memcpy(buffer, tmp, 7);
    buffer += 7;
    for(int i = 0; i < 23; i++)
    {
        memcpy(buffer, tmp, len);
        buffer += len;
    }
    memcpy(buffer, tmp, 4);
}

static void rx_packet_right_rate(uint16_t length, uint8_t *p_data)
{
    if (memcmp(&(p_data[0]), &adv_header, 1) == 0 &&
        memcmp(&(p_data[3]), adv_data, 241) == 0 &&
        memcmp(&(p_data[244]), &adv_crc, 3) == 0)
    {
        slave_receive_packet_num++;
        memset(p_data, 0, 247);
    }
}

/**
 *****************************************************************************************
 * @brief Find MLMR uuid from advertising data.
 *
 * @param[in] p_data:   Pointer to advertising report data.
 * @param[in] length:   Length of advertising report data.
 *
 * @return Operation result.
 *****************************************************************************************
 */
static bool user_mlmr_uuid_find(const uint8_t *p_data, const uint16_t length)
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
            uint8_t target_uuid[16] = MLMR_C_SVC_UUID;
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

static void mlmr_c_evt_process(mlmr_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch (p_evt->evt_type)
    {
        case MLMR_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Goodix Uart Service discovery completely.");
            error_code = mlmr_c_tx_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case MLMR_C_EVT_TX_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled TX Notification.");
            error_code = ble_gattc_mtu_exchange(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case MLMR_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled Flow Control Notification.");
            break;

        case MLMR_C_EVT_PEER_DATA_RECEIVE:
            APP_LOG_INFO("The master successfully accepts peer data.");
            mlmr_c_tx_data_send(0,p_evt->p_data, p_evt->length);
            rx_packet_right_rate(p_evt->length,p_evt->p_data);
            break;

        case MLMR_C_EVT_TX_CPLT:
            transport_flag_set(BLE_TX_CPLT, true);
            transport_ble_continue_send();
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Deal receive advertising report task.
 *
 * @param[in] p_data:   Pointer to advertising report data.
 * @param[in] length:   Length of advertising report data.
 * @param[in] p_bdaddr: Pointer of broadcast address with broadcast type.
 *****************************************************************************************
 */
static void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t   error_code;

    if (user_mlmr_uuid_find(p_data, length))
    {
        memcpy(&s_target_bdaddr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
        if(memcmp(s_target_bdaddr.gap_addr.addr, peer_bd_addr, BLE_GAP_ADDR_LEN) == 0 )
        {
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Deal device stop scan task.
 *****************************************************************************************
 */
static void app_scan_stop_handler(void)
{
    sdk_err_t        error_code;

    gap_connect_param.type                = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    gap_connect_param.interval_min        = APP_CONN_INTERVAL_MIN;
    gap_connect_param.interval_max        = APP_CONN_INTERVAL_MAX;
    gap_connect_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    gap_connect_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    gap_connect_param.peer_addr.gap_addr  = s_target_bdaddr.gap_addr;
    gap_connect_param.peer_addr.addr_type = s_target_bdaddr.addr_type;
    gap_connect_param.conn_timeout        = 0;

    error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal device connect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] p_param:  Pointer of connection complete event.
 *****************************************************************************************
 */
static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t   error_code;

    error_code = ble_sec_enc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
    error_code = mlmr_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal device disconnect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] disconnect_reason: disconnect reason.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    sdk_err_t   error_code;

    transport_ble_init();
    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal MTU stop exchange task.
 *****************************************************************************************
 */
static void app_mtu_exchange_handler(uint8_t conn_idx)
{
    sdk_err_t   error_code;
    ble_gap_conn_update_param_t gap_conn_param;
    
    gap_conn_param.interval_min  = UPDATE_APP_CONN_INTERVAL_MIN;
    gap_conn_param.interval_max  = UPDATE_APP_CONN_INTERVAL_MAX;
    gap_conn_param.slave_latency = APP_CONN_SLAVE_LATENCY;
    gap_conn_param.sup_timeout   = APP_CONN_SUP_TIMEOUT;
    gap_conn_param.ce_len        = 2;
    
    error_code = ble_gap_conn_param_update(conn_idx, &gap_conn_param);
    APP_ERROR_CHECK(error_code);

}

/**
 *****************************************************************************************
 * @brief Deal link encrypte request event task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] p_enc_req: Pointer of link encrypte request data.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    APP_LOG_DEBUG("Receiving encryption request");

    uint32_t tk = s_bd_info.tk;
    ble_sec_cfm_enc_t cfm_enc;

    memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));

    if (NULL == p_enc_req)
    {
        return;
    }

    switch (p_enc_req->req_type)
    {
        case BLE_SEC_PAIR_REQ:
        {
            APP_LOG_DEBUG("pair req");
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept = true;
            break;
        }

        case BLE_SEC_TK_REQ:
        {
            APP_LOG_DEBUG("tk req");
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept = true;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;
        }

        case BLE_SEC_OOB_REQ:
        {
            APP_LOG_DEBUG("oob req");
            break;
        }

        case BLE_SEC_NC_REQ:
        {
            APP_LOG_DEBUG("nc req");
            uint32_t num = *(uint32_t *)(p_enc_req->data.nc_data.value);
            APP_LOG_DEBUG("num=%d", num);
            cfm_enc.req_type = BLE_SEC_NC_REQ;
            cfm_enc.accept = true;
            break;
        }

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
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

        case BLE_GAPM_EVT_SCAN_START:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                APP_LOG_DEBUG("Scan started failed(0X%02X).", p_evt->evt_status);
            }
            else
            {
                APP_LOG_INFO("Scan started success");
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
            app_adv_report_handler(p_evt->evt.gapm_evt.params.adv_report.data, p_evt->evt.gapm_evt.params.adv_report.length, &p_evt->evt.gapm_evt.params.adv_report.broadcaster_addr);
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
        /******************************************************************************************************************************************************************************/
        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("MTU exchanged.");
                update_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
                app_mtu_exchange_handler(p_evt->evt.gapc_evt.index);
            } 
            break;
        /******************************************************************************************************************************************************************************/  
        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.gapc_evt.index, &p_evt->evt.sec_evt.params.enc_req);
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            APP_LOG_DEBUG("Pair complete, result = 0x%02x", p_evt->evt_status);
            break;
    }
}

void ble_app_init(void)
{
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t version;
    sdk_err_t    error_code;

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
    APP_LOG_INFO("Goodix multi role peripheral client example started.");

    transport_ble_init();
    transport_uart_init();
    error_code = mlmr_client_init(mlmr_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();
    receieve_packet_check_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}
