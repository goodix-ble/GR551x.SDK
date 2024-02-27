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
#include "app_log.h"
#include "app_error.h"
#include "user_config.h"
#include "ags.h"
#include "dis.h"
#include "user_gadget.h"
#include "ble_gattc.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_ADV_MIN_INTERVAL               32           /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL               48           /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0            /**< The advertising timeout in units of seconds. */

#define MIN_CONN_INTERVAL                  320          /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  520          /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   1000         /**< Connection supervisory timeout (4 seconds). */
#define DEFAULT_MTU_SIZE                   247          /**< Default mtu size. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */
static uint16_t                 s_mtu_exchanged;

static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'A', 'G', 'S',
};

static const uint8_t s_paring_rsp_data_set[] =
{
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID, 
    0x03, 0xFE,
    
    0x17,
    BLE_GAP_AD_TYPE_SERVICE_16_BIT_DATA,
    0x03, 0xFE,
    0xF7, 0x04,
    0x00,
    0xFF,
    0x00,
    AGS_PAIR_ADV_FLAG,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t s_reconnect_rsp_data_set[] = 
{
    0x1B,
    BLE_GAP_AD_TYPE_SERVICE_16_BIT_DATA,
    0x03, 0xFE,
    0xF7, 0x04,
    0x00,
    0xFF,
    0x00,
    AGS_RECONNECT_ADV_FLAG,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static dis_sys_id_t s_devinfo_system_id =                        /**< Device system id. */
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},           /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}                        /**< DUMMY Organisation Unique ID (OUI),
                                                                      You shall use the OUI of your company. */
};

static char s_devinfo_model_number[]  = "ag_sensor_01";          /**< Device model number string. */
static char s_devinfo_serial_number[] = "0001";                  /**< Device serial number string. */
static char s_devinfo_firmware_rev[]  = "1.0";                   /**< Device firmware revision string. */
static char s_devinfo_hardware_rev[]  = "1.0";                   /**< Device hardware revision string. */
static char s_devinfo_software_rev[]  = "0.80";                  /**< Device software revision string. */
static char s_devinfo_mfr_name[]      = "Goodix";                /**< Device manufacture name string. */

static char s_devinfo_cert[] =                                   /**< Device regulatory certification data. */
{
    DIS_11073_BODY_EXP,                                          /**< authoritative body type. */
    0x00,                                                        /**< authoritative body structure type. */
    'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'   /**< authoritative body data. */
};

static dis_pnp_id_t s_devinfo_pnp_id =                           /**< Device PnP id. */
{
    .vendor_id_source = 1,                                       /**< Vendor ID source (1=Bluetooth SIG). */
    .vendor_id        = 0x04F7,                                  /**< Vendor ID. */
    .product_id       = 0x0000,                                  /**< Product ID (vendor-specific). */
    .product_version  = 0x0110                                   /**< Product version (JJ.M.N). */
};

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

    error_code = ble_gap_privacy_params_set(900, true);
    APP_ERROR_CHECK(error_code);
    
    // Set the default security parameters.
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ALL,
        .rkey_dist = BLE_SEC_KDIST_ALL,
    };
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max  = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min  = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode      = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map      = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode     = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol    = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
   
    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_GEN_RSLV, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    uint8_t  dev_name[32];
    uint16_t dev_name_len = 32;
    error_code = ble_gap_device_name_get(dev_name, &dev_name_len);
    APP_ERROR_CHECK(error_code);

    if (!strcmp((const char *)dev_name, BLE_GAP_DEVNAME_DEFAULT))
    {
        // Set the default Device Name.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        // Set the Device Name is writable from the peer.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, NULL, 0);
        APP_ERROR_CHECK(error_code);
    }

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration     = 0;
    s_gap_adv_time_param.max_adv_evt  = 0;
    ble_gap_l2cap_params_set(DEFAULT_MTU_SIZE, DEFAULT_MTU_SIZE, 1);
}

/**
 *****************************************************************************************
 * @brief Alexa Gadget event handler.
 *
 * @param[in] p_evt: Pointer to the ags event.
 *****************************************************************************************
 */
static void ags_event_process(ags_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case AGS_EVT_ECHO_RX_DATA_SENT:
            user_gadget_protobuf_owner_release(p_evt->conn_idx);
            break;

        case AGS_EVT_ECHO_TX_DATA_RECEIVED:
            break;
        
        case AGS_EVT_ECHO_RX_NOTI_ENABLE:
            if (s_mtu_exchanged)
            {
                user_gadget_protocol_version_send(p_evt->conn_idx);
            }
            break;
        
        case AGS_EVT_ECHO_RX_NOTI_DISABLE:
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
    sdk_err_t   error_code;
    dis_init_t  dis_env_init;
    ags_init_t  ags_init;

    /*------------------------------------------------------------------*/
    dis_env_init.char_mask                   = DIS_CHAR_FULL;
    dis_env_init.manufact_name_str.p_str     = s_devinfo_mfr_name;
    dis_env_init.manufact_name_str.length    = strlen(s_devinfo_mfr_name);
    dis_env_init.model_num_str.p_str         = s_devinfo_model_number;
    dis_env_init.model_num_str.length        = strlen(s_devinfo_model_number);
    dis_env_init.serial_num_str.p_str        = s_devinfo_serial_number;
    dis_env_init.serial_num_str.length       = strlen(s_devinfo_serial_number);
    dis_env_init.hw_rev_str.p_str            = s_devinfo_hardware_rev;
    dis_env_init.hw_rev_str.length           = strlen(s_devinfo_hardware_rev);
    dis_env_init.fw_rev_str.p_str            = s_devinfo_firmware_rev;
    dis_env_init.fw_rev_str.length           = strlen(s_devinfo_firmware_rev);
    dis_env_init.sw_rev_str.p_str            = s_devinfo_software_rev;
    dis_env_init.sw_rev_str.length           = strlen(s_devinfo_software_rev);
    dis_env_init.p_sys_id                    = &s_devinfo_system_id;
    dis_env_init.reg_cert_data_list.p_list   = s_devinfo_cert;
    dis_env_init.reg_cert_data_list.list_len = strlen(s_devinfo_cert);
    dis_env_init.p_pnp_id                    = &s_devinfo_pnp_id;
    error_code = dis_service_init(&dis_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    ags_init.char_mask                       = AGS_CHAR_FULL;
    ags_init.ags_alexa_stream_cb             = user_gadget_alexa_stream_cb;
    ags_init.ags_control_stream_cb           = user_gadget_control_stream_cb;
    ags_init.evt_handler                     = ags_event_process;
    error_code = ags_service_init(&ags_init);
    APP_ERROR_CHECK(error_code);
}

static void app_adv_stopped_handler(ble_gap_stopped_reason_t reason)
{
    sdk_err_t error_code;
    if (BLE_GAP_STOPPED_REASON_ON_USER == reason)
    {
        error_code = ble_gap_bond_devs_clear();
        APP_ERROR_CHECK(error_code);

        error_code = ble_gap_disconnect(0);
        if (error_code)
        {
            app_adv_start();
        }
    }
    else if (BLE_GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        APP_LOG_INFO("Advertising timeout.");
    }
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{   
    ble_sec_cfm_enc_t cfm_enc;

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

        default:
            APP_LOG_DEBUG("Unsupported pairing method.");
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_adv_start(void)
{
    uint16_t                ret_val;
    sdk_err_t               error_code;
    ble_gap_bond_dev_list_t bond_dev_list;

    ret_val = ble_gap_bond_devs_get(&bond_dev_list);
    APP_LOG_DEBUG("Bond dev nums: %x.", bond_dev_list.num);

    if (!ret_val)
    {
        if (bond_dev_list.num <= 0)
        {
            APP_LOG_DEBUG("Paring with Echo.");
            error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_GEN_RSLV, &s_gap_adv_param);
            APP_ERROR_CHECK(error_code);
            error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
            APP_ERROR_CHECK(error_code);
            error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_paring_rsp_data_set, sizeof(s_paring_rsp_data_set));
            APP_ERROR_CHECK(error_code);
        }
        else
        {
            APP_LOG_DEBUG("Reconnecting to peer.");
            error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_GEN_RSLV, &s_gap_adv_param);
            APP_ERROR_CHECK(error_code);
            error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
            APP_ERROR_CHECK(error_code);
            error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_reconnect_rsp_data_set, sizeof(s_reconnect_rsp_data_set));
            APP_ERROR_CHECK(error_code);
        }

        error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        APP_LOG_DEBUG("Getting bond device info failed: %x.", ret_val);
    }
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
                app_adv_stopped_handler(p_evt->evt.gapm_evt.params.adv_stop.reason);
            }
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.connected);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            APP_LOG_INFO("Disconnected (0x%02X).", p_evt->evt.gapc_evt.params.disconnected.reason);
            user_gadget_protobuf_owner_release(p_evt->evt.gapc_evt.index);
            app_adv_start();
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                s_mtu_exchanged = 1;
            }
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
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
    APP_LOG_INFO("Alexa Gadget example started.");

    services_init();
    gap_params_init();

    app_adv_start();
    APP_ERROR_CHECK(error_code);
}

