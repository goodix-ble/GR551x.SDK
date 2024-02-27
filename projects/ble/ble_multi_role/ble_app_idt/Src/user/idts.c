/**
 *****************************************************************************************
 *
 * @file idts.c
 *
 * @brief Goodix UART Service Implementation.
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
******************************************************************************************
*/
#include "idts.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The UUIDs of IDTS characteristics. */
#define IDTS_SERVER_TX_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x02, 0xA0, 0xED, 0xA6}
#define IDTS_SERVER_RX_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x03, 0xA0, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Goodix UART Service Attributes Indexes. */
enum idts_attr_idx_t
{
    IDTS_IDX_SVC,
    
    IDTS_IDX_TX_CHAR,
    IDTS_IDX_TX_VAL,
    IDTS_IDX_TX_CFG,

    IDTS_IDX_RX_CHAR,
    IDTS_IDX_RX_VAL,

    IDTS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Goodix UART Service environment variable. */
struct idts_env_t
{
    idts_init_t             idts_init;                               /**< Goodix UART Service initialization variables. */
    uint16_t                start_hdl;                               /**< Start handle of services */
    uint16_t                tx_ind_cfg[IDTS_CONNECTION_MAX];         /**< TX Characteristic Notification configuration of the peers. */
    ble_gatts_create_db_t   idts_gatts_db;                           /**< IDT Service attributs database. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct idts_env_t s_idts_env;
static const uint16_t    s_char_mask = 0xFFFF;
static const uint8_t     s_idts_svc_uuid[] = {IDTS_SERVICE_UUID};

/**@brief Full IDTS Database Description which is used to add attributes into the ATT database. */
static const ble_gatts_attm_desc_128_t idts_attr_tab[IDTS_IDX_NB] =
{
    // IDTS service
    [IDTS_IDX_SVC]            = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // IDTS TX Characteristic Declaration
    [IDTS_IDX_TX_CHAR]        = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // IDTS TX Characteristic Value
    [IDTS_IDX_TX_VAL]         = {IDTS_SERVER_TX_UUID, 
                                BLE_GATTS_INDICATE_PERM_UNSEC,
                                (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                IDTS_MAX_DATA_LEN},
    // IDTS TX Characteristic - Client Characteristic Configuration Descriptor
    [IDTS_IDX_TX_CFG]         = {ATT_128_CLIENT_CHAR_CFG,
                                BLE_GATTS_READ_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM_UNSEC,
                                0,
                                0},

    // IDTS RX Characteristic Declaration
    [IDTS_IDX_RX_CHAR]        = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // IDTS RX Characteristic Value
    [IDTS_IDX_RX_VAL]         = {IDTS_SERVER_RX_UUID,
                                BLE_GATTS_WRITE_REQ_PERM_UNSEC | BLE_GATTS_WRITE_CMD_PERM_UNSEC,
                                (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                                IDTS_MAX_DATA_LEN},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void idts_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t cfm;
    uint16_t             handle    = p_param->handle;
    uint8_t              tab_index = 0;

    tab_index  = prf_find_idx_by_handle(handle, s_idts_env.start_hdl, IDTS_IDX_NB, (uint8_t *)&s_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case IDTS_IDX_TX_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_idts_env.tx_ind_cfg[conn_idx];
            cfm.status = BLE_SUCCESS;
            break;

        default:
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Point to the parameters of the write request.
 *****************************************************************************************
 */
static void idts_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint8_t               handle    = p_param->handle;
    uint8_t               tab_index = 0;
    uint16_t              cccd_value;
    idts_evt_t            event;
    ble_gatts_write_cfm_t cfm;

    tab_index      = prf_find_idx_by_handle(handle,s_idts_env.start_hdl, IDTS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    
    switch (tab_index)
    {
        case IDTS_IDX_RX_VAL:
            event.evt_type       = IDTS_EVT_RX_DATA_RECEIVED;
            event.p_data = (uint8_t *)p_param->value;
            event.length = p_param->length;
//            s_idts_env.access_addr = *(uint32_t *)(p_param->value);
//            s_idts_env.channel_idx = p_param->value[4];
            break;

        case IDTS_IDX_TX_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = (PRF_CLI_START_IND == cccd_value) ? IDTS_EVT_TX_PORT_OPENED : IDTS_EVT_TX_PORT_CLOSED;
            s_idts_env.tx_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && IDTS_EVT_INVALID != event.evt_type && s_idts_env.idts_init.evt_handler)
    {
        s_idts_env.idts_init.evt_handler(&event);
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the cccd recover request.
 *
 * @param[in]: conn_idx:   Connection index
 * @param[in]: handle:     The handle of cccd attribute.
 * @param[in]: cccd_value: The value of cccd attribute.
 *****************************************************************************************
 */
static void idts_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t           tab_index = 0;
    idts_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index      = prf_find_idx_by_handle(handle,s_idts_env.start_hdl, IDTS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    event.evt_type = IDTS_EVT_INVALID;

    switch (tab_index)
    {
        case IDTS_IDX_TX_CFG:
            event.evt_type = (PRF_CLI_START_IND == cccd_value) ? IDTS_EVT_TX_PORT_OPENED : IDTS_EVT_TX_PORT_CLOSED;
            s_idts_env.tx_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (IDTS_EVT_INVALID != event.evt_type && s_idts_env.idts_init.evt_handler)
    {
        s_idts_env.idts_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_ntf_ind:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void idts_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (NULL != s_idts_env.idts_init.evt_handler)
    {
        idts_evt_t event;
        event.conn_idx = conn_idx;

        if (BLE_SUCCESS == status && BLE_GATT_INDICATION == p_ntf_ind->type)
        {
            event.evt_type = IDTS_EVT_TX_DATA_SENT;
            s_idts_env.idts_init.evt_handler(&event);
        }
    }
}

static void idts_ble_evt_handler(const ble_evt_t *p_evt)
{	
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            idts_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            idts_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            idts_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            idts_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;	
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t idts_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t            error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_IND == s_idts_env.tx_ind_cfg[conn_idx])
    {
        // Fill in the parameter structure
        send_cmd.type   = BLE_GATT_INDICATION;
        send_cmd.handle = prf_find_handle_by_idx(IDTS_IDX_TX_VAL, s_idts_env.start_hdl, (uint8_t *)&s_char_mask);

        // Pack measured value in database
        send_cmd.length = length;
        send_cmd.value  = p_data;

        // Send notification to peer device
        error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }

    return error_code;
}


sdk_err_t idts_service_init(idts_init_t *p_idts_init)
{
    if (NULL == p_idts_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_idts_env.idts_init, p_idts_init, sizeof(idts_init_t));
    
    s_idts_env.start_hdl  = PRF_INVALID_HANDLE;

    s_idts_env.idts_gatts_db.shdl                  = &s_idts_env.start_hdl;
    s_idts_env.idts_gatts_db.uuid                  = s_idts_svc_uuid;
    s_idts_env.idts_gatts_db.attr_tab_cfg          = (uint8_t *)&s_char_mask;
    s_idts_env.idts_gatts_db.max_nb_attr           = IDTS_IDX_NB;
    s_idts_env.idts_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_idts_env.idts_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_idts_env.idts_gatts_db.attr_tab.attr_tab_128 = idts_attr_tab;

    return ble_gatts_prf_add(&s_idts_env.idts_gatts_db, idts_ble_evt_handler);
}
