/**
 *****************************************************************************************
 *
 * @file ags.c
 *
 * @brief Alexa Gadget Profile implementation.
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
#include "ags.h"
#include "user_config.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The UUIDs of AGS characteristics. 
 *        The Tx channel transmits data from an Echo device to a gadget.
 *        The Rx channel reansmits data from a gadget to an Echo device.
 */
#define AGS_TX_UUID {0x76, 0x30, 0xF8, 0xDD, 0x90, 0xA3, 0x61, 0xAC, 0xA7, 0x43, 0x05, 0x30, 0x77, 0xB1, 0x4E, 0xF0}
#define AGS_RX_UUID {0x0B, 0x42, 0x82, 0x1F, 0x64, 0x72, 0x2F, 0x8A, 0xB4, 0x4B, 0x79, 0x18, 0x5B, 0xA0, 0xEE, 0x2B}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)


/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Alexa Gadget Service Attributes Indexes. */
enum
{
    AGS_IDX_SVC,

    AGS_IDX_TX_CHAR,
    AGS_IDX_TX_VAL,

    AGS_IDX_RX_CHAR,
    AGS_IDX_RX_VAL,
    AGS_IDX_RX_NTF_CFG,

    AGS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Alexa Gadget Service environment variable. */
struct ags_env_t
{
    ags_init_t              ags_init;                               /**< Alexa Gadget Service initialization variables. */
    uint16_t                start_hdl;                              /**< Alexa Gadget Service start handle. */
    ags_stream_env_t        ags_stream_env[3];                      /**< The environment variable of gadget stream. */
    uint16_t                rx_ntf_cfg[AGS_CONNECTION_MAX];         /**< The configuration of Rx Notification which is configured by the peer devices. */
    ble_gatts_create_db_t   ags_gatts_db;                           /**< Alexa Gadget Service attributs database. */
};

/**@brief Alexa Gadget stream payload. */
typedef struct 
{
    uint8_t          p_data[USER_GADGET_TRANSACTION_BUF_SIZE];   /**< Gadget stream payload. */
    uint16_t         length;                                     /**< Length of data. */
    uint16_t         offset;                                     /**< Offset of data. */
} ags_stream_payload_t;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static sdk_err_t ags_echo_rx_val_chunk(uint8_t conn_idx);
static sdk_err_t ags_stream_ack_send(uint8_t conn_idx, uint8_t stream_env_id, bool ack_flag);
static void      ags_echo_tx_val_decode(uint8_t conn_idx, const uint8_t *buf, uint16_t length);
static void      ags_encode_ack(ags_ack_packet_t *ack_packet, uint8_t stream_id, uint8_t trxn_id, bool ack_flag);
static void      ags_encode_header(void *header, uint8_t stream_id, uint8_t trxn_type, uint8_t payload_length, uint16_t trxn_length);
static void      ags_reset_stream(uint8_t stream_env_id, uint16_t total_length);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct ags_env_t     s_ags_env;
static uint8_t              s_gadget_tx_buffer[244];
static uint8_t              s_trxn_counter;
static uint8_t              s_tx_sequ_num;
static ags_stream_payload_t s_ags_stream_payload;
static const uint8_t        s_ags_svc_uuid[] = {AGS_SERVICE_UUID};

/**@brief Full AGS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_128_t ags_attr_tab[AGS_IDX_NB]  =
{
    // Alexa Gadget Service Declaration
    [AGS_IDX_SVC]        = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // AGS Tx Characteristic - Declaration
    [AGS_IDX_TX_CHAR]    = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // AGS Tx Characteristic - Value
    [AGS_IDX_TX_VAL]     = {AGS_TX_UUID,
                            BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_CMD_PERM(BLE_GATTS_UNAUTH),
                            (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                            AGS_TX_VAL_LEN_MAX},
    
    // AGS Tx Characteristic - Declaration
    [AGS_IDX_RX_CHAR]    = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // AGS Tx Characteristic - Value
    [AGS_IDX_RX_VAL]     = {AGS_RX_UUID, 
                            BLE_GATTS_NOTIFY_PERM(BLE_GATTS_UNAUTH), 
                            (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)), 
                            AGS_RX_VAL_LEN_MAX},
    // AGS Tx Characteristic - Client Characteristic Configuration Descriptor
    [AGS_IDX_RX_NTF_CFG] = {ATT_128_CLIENT_CHAR_CFG,
                            BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                            0,
                            0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  The parameters of the read request.
 *****************************************************************************************
 */
static void ags_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t               handle    = p_param->handle;
    uint8_t               tab_index = prf_find_idx_by_handle(handle, 
                                                             s_ags_env.start_hdl,
                                                             AGS_IDX_NB,
                                                             (uint8_t *)&s_ags_env.ags_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {        
        case AGS_IDX_RX_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_ags_env.rx_ntf_cfg[conn_idx];
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
 * @param[in]: conn_idx: Connection index
 * @param[in]: p_param:  The parameters of the write request.
 *****************************************************************************************
 */
static void ags_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t              handle      = p_param->handle;
    uint16_t              tab_index   = 0;
    uint16_t              cccd_value  = 0;
    ags_evt_t             event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_ags_env.start_hdl,
                                        AGS_IDX_NB,
                                        (uint8_t *)&s_ags_env.ags_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = AGS_EVT_INVALID;
    event.conn_idx = conn_idx;
 
    switch (tab_index)
    {
        case AGS_IDX_TX_VAL:
            ags_echo_tx_val_decode(conn_idx, p_param->value, p_param->length);
            event.evt_type = AGS_EVT_ECHO_TX_DATA_RECEIVED;
            break;
        
        case AGS_IDX_RX_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ?\
                               AGS_EVT_ECHO_RX_NOTI_ENABLE :\
                               AGS_EVT_ECHO_RX_NOTI_DISABLE);
            s_ags_env.rx_ntf_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && AGS_EVT_INVALID != event.evt_type && s_ags_env.ags_init.evt_handler)
    {
        s_ags_env.ags_init.evt_handler(&event);
    }
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
static void ags_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t  tab_index  = 0;
    ags_evt_t event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index = prf_find_idx_by_handle(handle,
                                       s_ags_env.start_hdl,
                                       AGS_IDX_NB,
                                       (uint8_t *)&s_ags_env.ags_init.char_mask);

    event.evt_type = AGS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case AGS_IDX_RX_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ?\
                               AGS_EVT_ECHO_RX_NOTI_ENABLE :\
                               AGS_EVT_ECHO_RX_NOTI_DISABLE);
            s_ags_env.rx_ntf_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (AGS_EVT_INVALID != event.evt_type && s_ags_env.ags_init.evt_handler)
    {
        s_ags_env.ags_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] status:   The status of the complete event.  
 * @param[in] p_ntf_id: Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void ags_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (s_ags_env.ags_init.evt_handler && SDK_SUCCESS == status)
    {
        if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
        {
            ags_echo_rx_val_chunk(conn_idx);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Reset the gadget stream environment variables.
 *
 * @param[in] stream_env_id: The environment ID of gadget stream which is active.
 * @param[in] total_length:  The total payload length of this stream.
 *****************************************************************************************
 */
static inline void ags_reset_stream(uint8_t stream_env_id, uint16_t total_length)
{
    s_ags_env.ags_stream_env[stream_env_id].is_active       = true;
    s_ags_env.ags_stream_env[stream_env_id].ack_flag        = false;
    s_ags_env.ags_stream_env[stream_env_id].trxn_id         = 0;
    s_ags_env.ags_stream_env[stream_env_id].total_length    = total_length;
    s_ags_env.ags_stream_env[stream_env_id].received_length = 0;
}

/**
 *****************************************************************************************
 * @brief Decode the data from Echo device.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_data:     Pointer to the data which to be decoded.
 * @param[in] length:     Data length.
 *****************************************************************************************
 */
static void ags_echo_tx_val_decode(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    sdk_err_t error_code;
    uint8_t   stream_env_id;
    uint16_t  payload_length;
    uint16_t  payload_offset;

    if (NULL == p_data || (length < sizeof(ags_header_base_t)))
    {
        APP_LOG_DEBUG("Unexcepted length: %d", length);
        return;
    }

    ags_header_base_t *p_header_base = (ags_header_base_t *)p_data;

    for (uint8_t i=0; i < ARRAY_SIZE(s_ags_env.ags_stream_env); i++)
    {
        if (s_ags_env.ags_stream_env[i].stream_id == p_header_base->stream_id)
        {
            stream_env_id = i;
        }
    }

    if (s_ags_env.ags_stream_env[stream_env_id].is_active)
    {
        if (p_header_base->length_ext)
        {
            ags_header_subs_ext_t *p_header_subs_ext = (ags_header_subs_ext_t *)p_data;
            
            payload_length = be16toh(p_header_subs_ext->payload_length);
            payload_offset = sizeof(ags_header_subs_ext_t);
        }
        else
        {
            ags_header_subs_t *p_header_subs = (ags_header_subs_t *)p_data;
            
            payload_length = p_header_subs->payload_length;
            payload_offset = sizeof(ags_header_subs_t);
        }
    }
    else
    {
        uint16_t total_length;

        if (p_header_base->length_ext)
        {
            ags_header_first_ext_t *p_header_first_ext = (ags_header_first_ext_t *)p_data;
            
            total_length = be16toh(p_header_first_ext->total_trxn_length);
            payload_length = be16toh(p_header_first_ext->payload_length);
            payload_offset = sizeof(ags_header_first_ext_t);
        }
        else
        {
            ags_header_first_t *p_header_first = (ags_header_first_t *)p_data;
            
            total_length = be16toh(p_header_first->total_trxn_length);
            payload_length = p_header_first->payload_length;
            payload_offset = sizeof(ags_header_first_t);
        }

        ags_reset_stream(stream_env_id, total_length);
    }

    if (p_header_base->ack_flag)
    {
        s_ags_env.ags_stream_env[stream_env_id].ack_flag = p_header_base->ack_flag;
        s_ags_env.ags_stream_env[stream_env_id].trxn_id  = p_header_base->trxn_id;
    }

    s_ags_env.ags_stream_env[stream_env_id].received_length += payload_length;

    if (s_ags_env.ags_stream_env[stream_env_id].received_length == s_ags_env.ags_stream_env[stream_env_id].total_length)
    {
        s_ags_env.ags_stream_env[stream_env_id].is_active = 0;
    }
    else if (s_ags_env.ags_stream_env[stream_env_id].received_length > s_ags_env.ags_stream_env[stream_env_id].total_length)
    {
        s_ags_env.ags_stream_env[stream_env_id].is_active = 0;
        APP_LOG_DEBUG("Stream Overrun: %d %d.", s_ags_env.ags_stream_env[stream_env_id].received_length, 
                                                s_ags_env.ags_stream_env[stream_env_id].total_length);
    }

    bool ack_flag = false;

    switch (p_header_base->stream_id)
    {
        case AGS_CONTROL_STREAM_ID:
            ack_flag = s_ags_env.ags_init.ags_control_stream_cb(conn_idx, &((uint8_t *)p_data)[payload_offset], payload_length, \
                                                                s_ags_env.ags_stream_env[stream_env_id].is_active);
            break;

        case AGS_ALEXA_STREAM_ID:
            ack_flag = s_ags_env.ags_init.ags_alexa_stream_cb(conn_idx, &((uint8_t *)p_data)[payload_offset], payload_length, \
                                                              s_ags_env.ags_stream_env[stream_env_id].is_active);
            break;

        default:
            APP_LOG_DEBUG("Unknown Stream ID: %d", p_header_base->stream_id);
            break;
    }

    error_code = ags_stream_ack_send(conn_idx, stream_env_id, ack_flag);
    APP_ERROR_CHECK(error_code);
    return;
}

/**
 *****************************************************************************************
 * @brief Send ACK packet.
 *        If the Echo device request the ACK packet, the gadget must send the Echo device 
 *        an ACK packet in response to the last packet of the transaction.
 * @param[in] conn_idx:       Connection index.
 * @param[in] stream_env_id:  The environment ID of gadget stream which is active.
 * @param[in] ack_flag:       The ack flag of the ACK packet.
 *
 * @return the result of sending ACK packet.
 *****************************************************************************************
 */
static sdk_err_t ags_stream_ack_send(uint8_t conn_idx, uint8_t stream_env_id, bool ack_flag)
{
    sdk_err_t error_code = SDK_SUCCESS;
    ags_evt_t ags_evt;

    if (!s_ags_env.ags_stream_env[stream_env_id].is_active && (s_ags_env.ags_stream_env[stream_env_id].ack_flag))
    {
        ags_ack_packet_t ack_packet;
        
        ags_encode_ack(&ack_packet, s_ags_env.ags_stream_env[stream_env_id].stream_id, \
                       s_ags_env.ags_stream_env[stream_env_id].trxn_id, ack_flag);
        
        ble_gatts_noti_ind_t ags_rx_val_noti;
        
        ags_rx_val_noti.type   = BLE_GATT_NOTIFICATION;
        ags_rx_val_noti.handle = prf_find_handle_by_idx(AGS_IDX_RX_VAL,
                                                        s_ags_env.start_hdl,
                                                        (uint8_t *)&s_ags_env.ags_init.char_mask);
        ags_rx_val_noti.length = sizeof(ack_packet);
        ags_rx_val_noti.value  = (uint8_t *)&ack_packet;
        
        error_code = ble_gatts_noti_ind(conn_idx, &ags_rx_val_noti);
        
        ags_evt.evt_type = AGS_EVT_ECHO_RX_DATA_SENT;
        ags_evt.conn_idx = conn_idx;
        ags_evt.p_data   = (uint8_t *)&ack_packet;
        ags_evt.length   = sizeof(ack_packet);
        s_ags_env.ags_init.evt_handler(&ags_evt);
    }
    return error_code;
}

/**
 *****************************************************************************************
 * @brief Send the data to RX Characteristic chunk by chunk.
 *
 * @param[in] conn_idx:   Connection index.
 *
 * @return the result of sending operation.
 *****************************************************************************************
 */
static sdk_err_t ags_echo_rx_val_chunk(uint8_t conn_idx)
{
    sdk_err_t error_code;
    ags_evt_t ags_evt;

    if (s_ags_stream_payload.length)
    {
        uint8_t          header_size = sizeof(ags_header_subs_t);
        uint16_t         max_packet_size;
        uint16_t         payload_size;
        uint8_t          stream_id;
        uint8_t          trxn_type;
        ble_gatts_noti_ind_t ags_rx_val_noti;
        
        ble_gatt_mtu_get(conn_idx, &max_packet_size);
        max_packet_size -= 3;
        
        if (max_packet_size < (header_size + s_ags_stream_payload.length))
        {
            payload_size = max_packet_size - header_size;
            s_ags_stream_payload.length -= payload_size;
        }
        else
        {
            payload_size = s_ags_stream_payload.length;
            s_ags_stream_payload.length = 0;
        }
        
        // Get stream ID from prededing transmit.
        stream_id = ((ags_header_base_t *)s_gadget_tx_buffer)->stream_id;
        if (s_ags_stream_payload.length)
        {
            trxn_type = AGS_TRANSACTION_TYPE_CONT;
        }
        else
        {
            trxn_type = AGS_TRANSACTION_TYPE_LAST;
        }
        
        ags_encode_header(&s_gadget_tx_buffer[0], stream_id, trxn_type, payload_size, 0);
        memcpy(&s_gadget_tx_buffer[header_size], &s_ags_stream_payload.p_data[s_ags_stream_payload.offset], payload_size);
        
        s_ags_stream_payload.offset += payload_size;
        
        ags_rx_val_noti.type   = BLE_GATT_NOTIFICATION;
        ags_rx_val_noti.handle = prf_find_handle_by_idx(AGS_IDX_RX_VAL,
                                                        s_ags_env.start_hdl,
                                                        (uint8_t *)&s_ags_env.ags_init.char_mask);
        ags_rx_val_noti.length = header_size + payload_size;
        ags_rx_val_noti.value  = s_gadget_tx_buffer;
        
        error_code = ble_gatts_noti_ind(conn_idx, &ags_rx_val_noti);
    }
    else
    {
        memset(&s_ags_stream_payload, 0, sizeof(ags_stream_payload_t));
        s_ags_stream_payload.length = 0;
        s_ags_stream_payload.offset = 0;
        
        ags_evt.evt_type = AGS_EVT_ECHO_RX_DATA_SENT;
        ags_evt.conn_idx = conn_idx;
        ags_evt.p_data   = s_ags_stream_payload.p_data;
        s_ags_env.ags_init.evt_handler(&ags_evt);
        
        error_code = SDK_SUCCESS;
    }
    return error_code;
}

/**
 *****************************************************************************************
 * @brief Enocde the ACK packet header.
 *
 * @param[in] p_ack_packet: Pointer to the ACK packet.
 * @param[in] stream_id:    The stream ID of the packet that requested the acknowledgement.
 * @param[in] trxn_id:      The tracsaction ID of the packet that requested the acknowledgement.
 * @param[in] ack_flag:     The ack flag of the ACK packet.
 *****************************************************************************************
 */
static void ags_encode_ack(ags_ack_packet_t *p_ack_packet, uint8_t stream_id, uint8_t trxn_id, bool ack_flag)
{
    p_ack_packet->header_base.stream_id  = stream_id;
    p_ack_packet->header_base.trxn_id    = trxn_id;
    p_ack_packet->header_base.sequ_num   = 0;  // This can be any value.
    p_ack_packet->header_base.trxn_type  = AGS_TRANSACTION_TYPE_CTRL;
    p_ack_packet->header_base.ack_flag   = ack_flag ? AGS_ACK_ACK : AGS_ACK_NACK;
    p_ack_packet->header_base.length_ext = AGS_LEN_EXT_NO_EXT;
    p_ack_packet->reserved_1             = AGS_HEADER_ACK_RES_1;
    p_ack_packet->payload_length         = AGS_HEADER_ACK_PAYLOAD_LEN;
    p_ack_packet->reserved_2             = AGS_HEADER_ACK_RES_2; 
    p_ack_packet->result_code            = ack_flag ? AGS_RES_CODE_SUCCESS : AGS_RES_CODE_UNSUPPORTED;
}

/**
 *****************************************************************************************
 * @brief Enocde the header of other type packet.
 *
 * @param[in] p_header:       Pointer to the header.
 * @param[in] stream_id:      The stream ID, @ref ags_header_stream_id_t.
 * @param[in] trxn_type:      The transaction type, @ref ags_header_trxn_type_t.
 * @param[in] payload_length: Payload length.
 * @param[in] trxn_length:    Total transaction length.
 *****************************************************************************************
 */
static void ags_encode_header(void *p_header, uint8_t stream_id, uint8_t trxn_type, uint8_t payload_length, uint16_t trxn_length)
{
    ags_header_base_t *p_header_base = p_header;
    
    p_header_base->stream_id  = stream_id;
    p_header_base->sequ_num   = s_tx_sequ_num++;
    p_header_base->trxn_type  = trxn_type;
    p_header_base->ack_flag   = AGS_ACK_NACK;
    p_header_base->length_ext = AGS_LEN_EXT_NO_EXT;
    
    if (AGS_TRANSACTION_TYPE_FIRST == trxn_type)
    {
        ags_header_first_t *p_header_first = p_header;
        
        s_trxn_counter++;
        p_header_base->trxn_id         = s_trxn_counter;
        p_header_first->reserved       = AGS_HEADER_FIRST_RES;
        p_header_first->payload_length = payload_length;
        
        htobe16(&(p_header_first->total_trxn_length), trxn_length);
    }
    else
    {
        ags_header_subs_t *p_header_subs = p_header;
        
        p_header_base->trxn_id = s_trxn_counter; 
        p_header_subs->payload_length = payload_length;
    }
}

static void ags_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            ags_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            ags_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            ags_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            ags_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
sdk_err_t ags_stream_send(uint8_t conn_idx, uint8_t stream_id, void *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    uint16_t         max_packet_size;
    uint16_t         payload_size;
    uint16_t         header_size;
    ble_gatts_noti_ind_t ags_rx_val_noti;
    
    memset(&s_ags_stream_payload, 0, sizeof(s_ags_stream_payload));
    s_tx_sequ_num = 0;
    s_ags_stream_payload.length = 0;
    
    ble_gatt_mtu_get(conn_idx, &max_packet_size);
    max_packet_size -= 3;

    memcpy(s_ags_stream_payload.p_data, (uint8_t *)p_data, length);
    
    header_size = sizeof(ags_header_first_t);
    
    if (max_packet_size < (header_size + length))
    {
        s_ags_stream_payload.length = header_size + length - max_packet_size;
    }
    else
    {
        s_ags_stream_payload.length = 0;
    }
    
    payload_size = length - s_ags_stream_payload.length;
    s_ags_stream_payload.offset = payload_size;
    
    ags_encode_header(&s_gadget_tx_buffer[0], stream_id, AGS_TRANSACTION_TYPE_FIRST, payload_size, length);
    memcpy(&s_gadget_tx_buffer[header_size], p_data, payload_size);
    
    ags_rx_val_noti.type   = BLE_GATT_NOTIFICATION;
    ags_rx_val_noti.handle = prf_find_handle_by_idx(AGS_IDX_RX_VAL,
                                                    s_ags_env.start_hdl,
                                                    (uint8_t *)&s_ags_env.ags_init.char_mask);
    ags_rx_val_noti.length = header_size + payload_size;
    ags_rx_val_noti.value  = s_gadget_tx_buffer;
    
    error_code = ble_gatts_noti_ind(conn_idx, &ags_rx_val_noti);
    return error_code;
}

sdk_err_t ags_non_stream_send(uint8_t conn_idx, void *p_data, uint16_t length)
{
    sdk_err_t            error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t ags_rx_val_noti;
    ags_evt_t            ags_evt;

    ags_rx_val_noti.type   = BLE_GATT_NOTIFICATION;
    ags_rx_val_noti.handle = prf_find_handle_by_idx(AGS_IDX_RX_VAL,
                                                    s_ags_env.start_hdl,
                                                    (uint8_t *)&s_ags_env.ags_init.char_mask);
    ags_rx_val_noti.length = length;
    ags_rx_val_noti.value  = (uint8_t *)p_data;   
    
    error_code = ble_gatts_noti_ind(conn_idx, &ags_rx_val_noti);
    
    ags_evt.evt_type = AGS_EVT_ECHO_RX_DATA_SENT;
    ags_evt.conn_idx = conn_idx;
    ags_evt.p_data   = (uint8_t *)p_data; 
    ags_evt.length   = length;
    s_ags_env.ags_init.evt_handler(&ags_evt);

    return error_code;
}

sdk_err_t ags_service_init(ags_init_t *p_ags_init)
{
    if (NULL == p_ags_init)
    {
        return SDK_ERR_POINTER_NULL;
    }
    
    s_ags_env.ags_stream_env[0].stream_id = AGS_CONTROL_STREAM_ID;
    s_ags_env.ags_stream_env[0].is_active = false;
    s_ags_env.ags_stream_env[1].stream_id = AGS_ALEXA_STREAM_ID;
    s_ags_env.ags_stream_env[1].is_active = false;
    s_ags_env.ags_stream_env[2].stream_id = AGS_OTA_STREAM_ID;
    s_ags_env.ags_stream_env[2].is_active = false;

    memcpy(&s_ags_env.ags_init, p_ags_init, sizeof(ags_init_t));

    s_ags_env.ags_gatts_db.shdl                  = &s_ags_env.start_hdl;
    s_ags_env.ags_gatts_db.uuid                  = s_ags_svc_uuid;
    s_ags_env.ags_gatts_db.attr_tab_cfg          = (uint8_t *)&(s_ags_env.ags_init.char_mask);
    s_ags_env.ags_gatts_db.max_nb_attr           = AGS_IDX_NB;
    s_ags_env.ags_gatts_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128); 
    s_ags_env.ags_gatts_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_ags_env.ags_gatts_db.attr_tab.attr_tab_128 = ags_attr_tab;

    return ble_gatts_prf_add(&s_ags_env.ags_gatts_db, ags_ble_evt_handler);
}
