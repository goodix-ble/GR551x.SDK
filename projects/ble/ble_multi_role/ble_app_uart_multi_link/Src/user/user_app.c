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
#include "app_timer.h"
#include "app_io.h"
#include "grx_hal.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "grx_hal.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                         "Multi_role"      /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                 64               /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                 64               /**< The advertising max interval (in units of 0.625 ms). */
#define APP_SCAN_INTERVAL                    88               /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                      88               /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                    10000            /**< Duration of the scanning(in units of 10 ms). */
#define APP_FIRST_CONN_INTERVAL_MIN          70                /**< Minimal connection interval(in units of 1.25 ms). */
#define APP_FIRST_CONN_INTERVAL_MAX          70                /**< Maximal connection interval(in units of 1.25 ms). */
#define APP_SECOND_CONN_INTERVAL_MIN         80                /**< Minimal connection interval(in units of 1.25 ms). */
#define APP_SECOND_CONN_INTERVAL_MAX         80                /**< Maximal connection interval(in units of 1.25 ms). */
#define APP_THIRD_CONN_INTERVAL_MIN          90               /**< Minimal connection interval(in units of 1.25 ms). */
#define APP_THIRD_CONN_INTERVAL_MAX          90               /**< Maximal connection interval(in units of 1.25 ms). */
#define RECONN_INTERVAL_MIN                  8                /**< Minimal connection interval(in units of 1.25 ms). */
#define RECONN_INTERVAL_MAX                  8                /**< Minimal connection interval(in units of 1.25 ms). */
#define APP_FIRST_UPDATE_CONN_INTERVAL_MIN   51               /**< Update minimal connection interval(in units of 1.25 ms). */
#define APP_FIRST_UPDATE_CONN_INTERVAL_MAX   51               /**< Update maximal connection interval(in units of 1.25 ms). */
#define APP_SECOND_UPDATE_CONN_INTERVAL_MIN  53               /**< Update minimal connection interval(in units of 1.25 ms). */
#define APP_SECOND_UPDATE_CONN_INTERVAL_MAX  53               /**< Update maximal connection interval(in units of 1.25 ms). */
#define APP_THIRD_UPDATE_CONN_INTERVAL_MIN   55               /**< Update minimal connection interval(in units of 1.25 ms). */
#define APP_THIRD_UPDATE_CONN_INTERVAL_MAX   55               /**< Update maximal connection interval(in units of 1.25 ms). */
#define APP_CONN_SLAVE_LATENCY               0                /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                 200              /**< Connection supervisory timeout (in unit of 10 ms). */
#define MAX_NB_LECB_DEFUALT                  2               /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                 251              /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                  2120             /**< Defualt maximum packet transmission time. */

/**@brief user parameters. */
#define MASTER_CONN_LINK_MAX            3                /**< Maximum number of connect links as master. */
#define SLAVE_CONN_LINK_MAX             2                /**< Maximum number of connect links as slave. */
#define SLAVE_SEND_DATA_INTERVAL        1000             /**< The interval of as slave send data to peer. */
#define SEND_PACKET_NUM_INTERVAL        60000

/**@brief ble status. */
#define BLE_STATE_IDLE                  0x00             /**< BLE is idle status. */
#define BLE_STATE_CONNECTING            0x01             /**< BLE is connecting status. */
#define BLE_STATE_CONNECTED             0x02             /**< BLE is connected status. */

#define BLE_TRANS_DATA_SIZE             241             /**< The size of data transfer between devices. */
#define ALL_CONNECT_NUM                 4            /**< The value when all devices are successfully connected. */
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */
static ble_gap_init_param_t     gap_connect_param;

static uint32_t  central_received_packet = 0;
static uint8_t   update_ci_num           = 0;
static uint8_t   ble_status              = BLE_STATE_IDLE;

static uint32_t                slave_tk;                                        /**< TK, device as the slave. */
static uint32_t                master_tk;                                       /**< TK, device as the master. */
static ble_gap_ll_role_type_t  role[5];                                         /**< Address, device as slave. */

static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x11, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    MLMR_SERVICE_UUID,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7, 
    0x04,
    // Goodix specific adv data
    0x02,0x03,
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x0b, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'M', 'u', 'l', 't', 'i', '_', 'r', 'o', 'l', 'e',
};

static app_timer_id_t   s_slave_timer_id;
static app_timer_id_t   s_send_packet_num_id;

/**< Address information of the peer device. */
static  ble_gap_bdaddr_t  s_target_bdaddr;                                                        /**< Target board address. */
uint8_t first_slave_target_addr[SYS_BD_ADDR_LEN]  = {0x06, 0xae, 0xdf, 0x3e, 0xcb, 0xea};  /**< First peer device address. */
uint8_t second_slave_target_addr[SYS_BD_ADDR_LEN] = {0x05, 0xae, 0xdf, 0x3e, 0xcb, 0xea};  /**< Second peer device address. */
uint8_t third_slave_target_addr[SYS_BD_ADDR_LEN]  = {0x04, 0xae, 0xdf, 0x3e, 0xcb, 0xea};  /**< Third peer device address. */
uint8_t first_slave_conn_index  = 0xff;
uint8_t second_slave_conn_index = 0xff;
uint8_t third_slave_conn_index  = 0xff;

static const uint8_t first_master_target_addr[SYS_BD_ADDR_LEN] = {0x08, 0xae, 0xdf, 0x3e, 0xcb, 0xea};     /**< First peer device address. */
static const uint8_t second_master_target_addr[SYS_BD_ADDR_LEN] = {0x07, 0xae, 0xdf, 0x3e, 0xcb, 0xea};    /**<Second peer device address. */

/**< Connection information of the peer device. */
static bool     notify_enable_flag[5] = {0};                            /**< True if notify enable. */
static uint8_t  s_role[5];                                              /**< The role of peer device. */
static uint8_t  s_master_conn_num = 0;                                  /**< Number of connect as master . */
static uint8_t  s_slave_conn_num  = 0;                                  /**< Number of connect as slave. */

/**< Calculate the correct rate of received packet. */
static uint8_t       adv_header;
static uint16_t      adv_length ;
static uint32_t      adv_crc ;
static uint8_t       adv_data[BLE_TRANS_DATA_SIZE] = {0};

uint8_t  rx_buffer[CFG_BOND_DEVS][BLE_TRANS_DATA_SIZE+6];                                  /**< Buffer used to receiving data. */

/**< security parameters. */
static ble_sec_param_t s_sec_param =
{
    .level    = BLE_SEC_MODE1_LEVEL1,
    .io_cap   = BLE_SEC_IO_KEYBOARD_ONLY,
    .oob      = false,
    .auth     = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM,
    .key_size = 16,
    .ikey_dist = BLE_SEC_KDIST_ENCKEY,
    .rkey_dist = BLE_SEC_KDIST_ENCKEY,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Handle the master timer timeout.
 *
 * @details This function will be called each time the master timer expires.
 *****************************************************************************************
 */

static void slave_timeout_handler(void* p_arg)
{
    sdk_err_t error_code;
    uint8_t data[BLE_TRANS_DATA_SIZE+6];

    uint8_t *buffer = data;
    uint16_t length = BLE_TRANS_DATA_SIZE+6;
    uint16_t adv_data_len = BLE_TRANS_DATA_SIZE;
    memcpy(buffer, &adv_header, 1);
    buffer += 1;
    memcpy(buffer, &adv_length, 2);
    buffer += 2;
    memcpy(buffer, adv_data, adv_data_len);
    buffer += adv_data_len;
    memcpy(buffer, &adv_crc, 3);
    
    for(uint8_t i = 0; i < 5;i++)
    {
        if(s_role[i] == BLE_GAP_LL_ROLE_SLAVE && notify_enable_flag[i] == true)
        {
            error_code = mlmr_tx_data_send(i, data, length);
            APP_ERROR_CHECK(error_code);
        }
        
        if(s_role[i] == BLE_GAP_LL_ROLE_MASTER && notify_enable_flag[i] == true)
        {
            error_code = mlmr_c_tx_data_send(i, data, length);
            APP_ERROR_CHECK(error_code);
        }
    }
}

static void send_packet_num_timeout_handler(void* p_arg)
{
    float rx_right_rate = 0;
    rx_right_rate = (float)central_received_packet / 3;
    APP_LOG_INFO("The correct rate of central receive packets within 1 minute is %.2f%%.",rx_right_rate);
    central_received_packet = 0;
}

static void rx_packet_right_rate(uint16_t length, uint8_t *p_data)
{
    if (memcmp(&(p_data[0]), &adv_header, 1) == 0 &&
        memcmp(&(p_data[3]), adv_data, BLE_TRANS_DATA_SIZE) == 0 &&
        memcmp(&(p_data[BLE_TRANS_DATA_SIZE+3]), &adv_crc, 3) == 0)
    {
        central_received_packet++;
        memset(p_data, 0, BLE_TRANS_DATA_SIZE+6);
    }
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t error_code;

    error_code = app_timer_create(&s_slave_timer_id, ATIMER_REPEAT, slave_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_send_packet_num_id, ATIMER_REPEAT, send_packet_num_timeout_handler);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_adv_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(true);
    ble_sec_params_set(&s_sec_param);
    ble_gap_privacy_params_set(150, true);
    
    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL_MAX;
    s_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL_MIN;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    error_code = ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}

static void receieve_packet_check_init(void)
{
    adv_header = 0xA0;
    adv_length = BLE_TRANS_DATA_SIZE+6;
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

static void gap_scan_params_init(void)
{
    sdk_err_t        error_code;
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
            uint8_t target_uuid[16] = {MLMR_SERVICE_UUID};
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

/**
 *****************************************************************************************
 * @brief Function for process mlmr service event
 *
 * @param[in] p_evt: Pointer to mlmr event stucture.
 *****************************************************************************************
 */
static void mlmr_service_process_event(mlmr_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case MLMR_EVT_TX_PORT_OPENED:
            transport_flag_set(BLE_TX_NTF_ENABLE, true);
            if(notify_enable_flag[p_evt->conn_idx] == false)
            {
                notify_enable_flag[p_evt->conn_idx] = true;
            }
            break;

        case MLMR_EVT_TX_PORT_CLOSED:
            notify_enable_flag[p_evt->conn_idx] = false;
            transport_flag_set(BLE_TX_NTF_ENABLE, false);
            break;

        case MLMR_EVT_RX_DATA_RECEIVED:
            rx_packet_right_rate(p_evt->length,p_evt->p_data);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Function for process mlmr_c service event
 *
 * @param[in] p_evt: Pointer to mlmr_c event stucture.
 *****************************************************************************************
 */
static void mlmr_client_process_event(mlmr_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch (p_evt->evt_type)
    {
        case MLMR_C_EVT_DISCOVERY_COMPLETE:
            error_code = mlmr_c_tx_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case MLMR_C_EVT_TX_NTF_SET_SUCCESS:
            if(notify_enable_flag[p_evt->conn_idx] == false)
            {
                notify_enable_flag[p_evt->conn_idx] = true;
            }

            error_code = ble_gattc_mtu_exchange(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case MLMR_C_EVT_PEER_DATA_RECEIVE:
            rx_packet_right_rate(p_evt->length,p_evt->p_data);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Function for initializing services
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t   error_code;
    mlmr_init_t mlmr_init;

    mlmr_init.evt_handler = mlmr_service_process_event;

    error_code = mlmr_service_init(&mlmr_init);
    APP_ERROR_CHECK(error_code);
}

static void app_client_connected_handler(uint8_t conn_idx)
{
    sdk_err_t   error_code;
    s_role[conn_idx] = BLE_GAP_LL_ROLE_MASTER;
    s_master_conn_num++;

    ble_status = BLE_STATE_CONNECTED;
    if(s_master_conn_num < MASTER_CONN_LINK_MAX)
    {
        ble_gap_scan_start();
    }

    if(conn_idx < (MASTER_CONN_LINK_MAX + SLAVE_CONN_LINK_MAX))
    {
        error_code = mlmr_c_disc_srvc_start(conn_idx);
        APP_ERROR_CHECK(error_code);
        error_code = ble_sec_enc_start(conn_idx);
        APP_ERROR_CHECK(error_code);
    }
}

static void app_service_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;
    s_role[conn_idx] = BLE_GAP_LL_ROLE_SLAVE;
    s_slave_conn_num++;
    
    if(s_slave_conn_num < SLAVE_CONN_LINK_MAX)
    {
        error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
        APP_ERROR_CHECK(error_code);
    }
}

static void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t   error_code;
    
    if(user_mlmr_uuid_find(p_data,length))
    {
        memcpy(&s_target_bdaddr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
        
        if(((memcmp(s_target_bdaddr.gap_addr.addr, first_slave_target_addr, BLE_GAP_ADDR_LEN) == 0) && (first_slave_conn_index == 0xff)) ||
           ((memcmp(s_target_bdaddr.gap_addr.addr, second_slave_target_addr, BLE_GAP_ADDR_LEN) == 0) && (second_slave_conn_index == 0xff)) ||
           ((memcmp(s_target_bdaddr.gap_addr.addr, third_slave_target_addr, BLE_GAP_ADDR_LEN) == 0) && (third_slave_conn_index == 0xff)))
        {
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
        }
    }
}

static void app_scan_stop_handler(void)
{
    sdk_err_t        error_code;

    gap_connect_param.type                = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    gap_connect_param.interval_min        = APP_FIRST_CONN_INTERVAL_MIN;
    gap_connect_param.interval_max        = APP_FIRST_CONN_INTERVAL_MAX;
    gap_connect_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    gap_connect_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    gap_connect_param.peer_addr.gap_addr  = s_target_bdaddr.gap_addr;
    gap_connect_param.peer_addr.addr_type = s_target_bdaddr.addr_type;
    gap_connect_param.conn_timeout        = 0;
    
    if(s_master_conn_num == 0)
    {
        error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
        APP_ERROR_CHECK(error_code);
        ble_status = BLE_STATE_CONNECTING;
    }
    if(s_master_conn_num == 1)
    {
        gap_connect_param.interval_min        = APP_SECOND_CONN_INTERVAL_MIN;
        gap_connect_param.interval_max        = APP_SECOND_CONN_INTERVAL_MAX;
        error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
        APP_ERROR_CHECK(error_code);
        ble_status = BLE_STATE_CONNECTING;
    }
    if(s_master_conn_num == 2)
    {
        gap_connect_param.interval_min        = APP_THIRD_CONN_INTERVAL_MIN;
        gap_connect_param.interval_max        = APP_THIRD_CONN_INTERVAL_MAX;
        error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
        APP_ERROR_CHECK(error_code);
        ble_status = BLE_STATE_CONNECTING;
    }
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    if(memcmp(p_param->peer_addr.addr, first_slave_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_param->ll_role == BLE_GAP_LL_ROLE_MASTER)
        {
            role[conn_idx] = BLE_GAP_LL_ROLE_MASTER;
            master_tk = 123456;
        }
        first_slave_conn_index = conn_idx;
    }
    
    if(memcmp(p_param->peer_addr.addr,second_slave_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_param->ll_role == BLE_GAP_LL_ROLE_MASTER)
        {
            role[conn_idx] = BLE_GAP_LL_ROLE_MASTER;
            master_tk = 234567;
        }
        second_slave_conn_index = conn_idx;
    }

    if(memcmp(p_param->peer_addr.addr, third_slave_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_param->ll_role == BLE_GAP_LL_ROLE_MASTER)
        {
            role[conn_idx] = BLE_GAP_LL_ROLE_MASTER;
            master_tk = 345678;
        }
        third_slave_conn_index = conn_idx;
    }

    if(memcmp(p_param->peer_addr.addr, first_master_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_param->ll_role == BLE_GAP_LL_ROLE_SLAVE)
        {
            role[conn_idx] = BLE_GAP_LL_ROLE_SLAVE;
            slave_tk = 456789;
        }
    }

    if(memcmp(p_param->peer_addr.addr, second_master_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_param->ll_role == BLE_GAP_LL_ROLE_SLAVE)
        {
            role[conn_idx] = BLE_GAP_LL_ROLE_SLAVE;
            slave_tk = 567890;
        }
    }

    if(p_param->ll_role == BLE_GAP_LL_ROLE_SLAVE)
    {
        app_service_connected_handler(conn_idx, p_param);
    }
    
    if(p_param->ll_role == BLE_GAP_LL_ROLE_MASTER)
    {
        app_client_connected_handler(conn_idx);
    }
}

static void app_mtu_exchange_handler(uint8_t conn_idx)
{
    sdk_err_t   error_code;
    ble_gap_conn_update_param_t gap_conn_param;
    if (s_role[conn_idx] == BLE_GAP_LL_ROLE_MASTER)
    {
        update_ci_num++;
        if ((update_ci_num % 3) == 0)
        {
            gap_conn_param.interval_min  = APP_FIRST_UPDATE_CONN_INTERVAL_MIN;
            gap_conn_param.interval_max  = APP_FIRST_UPDATE_CONN_INTERVAL_MAX;
        }
        if ((update_ci_num % 3) == 1)
        {
            gap_conn_param.interval_min  = APP_SECOND_UPDATE_CONN_INTERVAL_MIN;
            gap_conn_param.interval_max  = APP_SECOND_UPDATE_CONN_INTERVAL_MAX;
        }
        if ((update_ci_num % 3) == 2)
        {
            gap_conn_param.interval_min  = APP_THIRD_UPDATE_CONN_INTERVAL_MIN;
            gap_conn_param.interval_max  = APP_THIRD_UPDATE_CONN_INTERVAL_MAX;
        }

        gap_conn_param.slave_latency = APP_CONN_SLAVE_LATENCY;
        gap_conn_param.sup_timeout   = APP_CONN_SUP_TIMEOUT;
        gap_conn_param.ce_len        = 2;
        
        error_code = ble_gap_conn_param_update(conn_idx, &gap_conn_param);
        APP_ERROR_CHECK(error_code);
    }
}

static void app_service_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t   error_code;

    notify_enable_flag[conn_idx] = false;
    s_slave_conn_num--;
    if(s_slave_conn_num == SLAVE_CONN_LINK_MAX - 1)
    {
        error_code = ble_gap_adv_start(0,&s_gap_adv_time_param);
        APP_ERROR_CHECK(error_code);
    }
}

static void app_client_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    notify_enable_flag[conn_idx] = false;
    s_master_conn_num--;

    if(ble_status != BLE_STATE_CONNECTING)
    {
        ble_gap_scan_start();
    }
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    APP_LOG_DEBUG("Receiving encryption request");

    uint32_t tk;

    if(role[conn_idx] == BLE_GAP_LL_ROLE_MASTER)
    {
        tk = master_tk;
    }
    else
    {
        tk = slave_tk;
    }

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
 ****************************************************************************************
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
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break; 

        case BLE_GAPC_EVT_DISCONNECTED:
            APP_LOG_INFO("Disconnected (0x%02X).", p_evt->evt.gapc_evt.params.disconnected.reason);

            if(role[p_evt->evt.gapc_evt.index] == BLE_GAP_LL_ROLE_MASTER)
            {
                app_client_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);

                if(first_slave_conn_index == p_evt->evt.gapc_evt.index)
                {
                    first_slave_conn_index = 0xff;
                }
                if(second_slave_conn_index == p_evt->evt.gapc_evt.index)
                {
                    second_slave_conn_index = 0xff;
                }
                if(third_slave_conn_index == p_evt->evt.gapc_evt.index)
                {
                    third_slave_conn_index = 0xff;
                }
            }
            else
            {
                app_service_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            }
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;
        
        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
             APP_LOG_INFO("Update connection parameter complete, conn idx is %d.", p_evt->evt.gapc_evt.index);
             if(p_evt->evt.gapc_evt.index == ALL_CONNECT_NUM)
             {
                app_timer_start(s_slave_timer_id, SLAVE_SEND_DATA_INTERVAL, NULL);
                app_timer_start(s_send_packet_num_id,SEND_PACKET_NUM_INTERVAL, NULL);
             }
        break;
        
        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;
            
        case BLE_SEC_EVT_LINK_ENCRYPTED:
            APP_LOG_DEBUG("Pair complete, result = 0x%02x", p_evt->evt_status);
            break;
        
        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if(BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("MTU exchanged, the value is %d.", p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
                update_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
                app_mtu_exchange_handler(p_evt->evt.gatt_common_evt.index);
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
    APP_LOG_INFO("Goodix multi role example started.");

    transport_ble_init();
    transport_uart_init();
    services_init();
    error_code = mlmr_client_init(mlmr_client_process_event);
    APP_ERROR_CHECK(error_code);
    gap_adv_params_init();
    gap_scan_params_init();
    app_timer_init();
    receieve_packet_check_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(0,&s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}
