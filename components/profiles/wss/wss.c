/**
 ****************************************************************************************
 *
 * @file wss.c
 *
 * @brief Weight Scale Service implementation.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */
#include "wss.h"
#include "wss_db.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_log.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief Weight Scale Service Attributes Indexes. */
enum
{
    // Weight Scale Service
    WSS_IDX_SVC,
    
    WSS_IDX_INCL_SVC,
    
    // WSS Scale Feature Characteristics
    WSS_IDX_WS_FEAT_CHAR,
    WSS_IDX_WS_FEAT_VAL,
    
    // Weight Measurement Characteristic
    WSS_IDX_WEIGHT_MEAS_CHAR,
    WSS_IDX_WEIGHT_MEAS_VAL,
    WSS_IDX_WEIGHT_MEAS_IND_CFG,

    WSS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Weight Scale measurement data stream. */
typedef struct 
{
    uint8_t          *p_data;           /**< Pointer to data. */
    uint16_t         length;            /**< Length of data. */
    uint16_t         offset;            /**< Offset of data. */
} wss_meas_data_stream_t;

/**@brief Weight Scale Service environment variable. */
struct wss_env_t
{
    wss_init_t               wss_init;                          /**< Weight Scale Service initialization variables. */
    uint16_t                 start_hdl;                         /**< Weight Scale Service start handle. */
    uint8_t                  cur_user_index;                    /**< Current User Index. */
    uint16_t                 meas_ind_cfg[WSS_CONNECTION_MAX];  /**< The configuration of Weight Scale Indication which is configured by the peer devices. */
    uint16_t                 *p_incl_srvc_start_handle;
    ble_gatts_create_db_t    wss_gatts_db;                      /**< Weight Scale Service attributs database. */ 
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static uint16_t    wss_meas_value_encoded(wss_meas_val_t *p_wss_meas_val, uint8_t index);
static uint16_t    wss_indicate_meas_value_chunk(uint8_t conn_idx);
static uint8_t     local_buff[WSS_MEAS_VAL_LEN_MAX * WSS_CACHE_MEAS_NUM_MAX];

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct wss_env_t        s_wss_env;
static wss_meas_data_stream_t  s_wss_meas_val;
static uint16_t                s_packet_length;
static const uint8_t           s_wss_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_WEIGHT_SCALE);

/**@brief Full WSS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t wss_attr_tab[WSS_IDX_NB] =
{
    // WSS Service Declaration
    [WSS_IDX_SVC]                 = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    //Include Service Declaration
    [WSS_IDX_INCL_SVC]            = {BLE_ATT_DECL_INCLUDE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Weight Scale Feature Characteristic - Declaration
    [WSS_IDX_WS_FEAT_CHAR]        = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Weight Scale Feature Characteristic - Value
    [WSS_IDX_WS_FEAT_VAL]         = {BLE_ATT_CHAR_WEIGHT_SCALE_FEATURE,
                                     BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH),
                                     BLE_GATTS_ATT_VAL_LOC_USER,
                                     WSS_FEAT_VAL_LEN_MAX},

    // Weight Measurement Characteristic - Declaration
    [WSS_IDX_WEIGHT_MEAS_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Weight Measurement Characteristic Declaration - value
    [WSS_IDX_WEIGHT_MEAS_VAL]     = {BLE_ATT_CHAR_WEIGHT_MEASUREMENT,
                                     BLE_GATTS_INDICATE_PERM(BLE_GATTS_UNAUTH),
                                     BLE_GATTS_ATT_VAL_LOC_USER,
                                     WSS_MEAS_VAL_LEN_MAX},
    // Weight Measurement Declaration - Client Characteristic Configuration Descriptor
    [WSS_IDX_WEIGHT_MEAS_IND_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
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
static void wss_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t               handle    = p_param->handle;
    uint8_t               tab_index = prf_find_idx_by_handle(handle,
                                                             s_wss_env.start_hdl,
                                                             WSS_IDX_NB,
                                                            (uint8_t *)&s_wss_env.wss_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case WSS_IDX_WS_FEAT_VAL:
        {
            cfm.length = sizeof(uint32_t);
            cfm.value  = (uint8_t *)&s_wss_env.wss_init.feature;
            break;
        }

        case WSS_IDX_WEIGHT_MEAS_IND_CFG:
        {
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_wss_env.meas_ind_cfg[conn_idx];
            break;
        }

        default:
        {
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
        }
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
static void   wss_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t              handle      = p_param->handle;
    uint16_t              tab_index   = 0;
    uint16_t              cccd_value  = 0;
    wss_evt_t             event;
    ble_gatts_write_cfm_t cfm;

    tab_index      = prf_find_idx_by_handle(handle,
                                            s_wss_env.start_hdl,
                                            WSS_IDX_NB,
                                            (uint8_t *)&s_wss_env.wss_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = WSS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {       
        case WSS_IDX_WEIGHT_MEAS_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              WSS_EVT_MEAS_INDICATION_ENABLE : \
                              WSS_EVT_MEAS_INDICATION_DISABLE);
            s_wss_env.meas_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && WSS_EVT_INVALID != event.evt_type && s_wss_env.wss_init.evt_handler)
    {
        s_wss_env.wss_init.evt_handler(&event);
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
static void wss_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index   = 0;
    wss_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_wss_env.start_hdl,
                                        WSS_IDX_NB,
                                        (uint8_t *)&s_wss_env.wss_init.char_mask);

    event.evt_type = WSS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case WSS_IDX_WEIGHT_MEAS_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              WSS_EVT_MEAS_INDICATION_ENABLE : \
                              WSS_EVT_MEAS_INDICATION_DISABLE);
            s_wss_env.meas_ind_cfg[conn_idx] = cccd_value;
            break;
                        
        default:
            break;
    }

    if (WSS_EVT_INVALID != event.evt_type && s_wss_env.wss_init.evt_handler)
    {
        s_wss_env.wss_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] status:     Complete event status.
 * @param[in] p_ntf_ind:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void wss_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (s_wss_env.wss_init.evt_handler && SDK_SUCCESS == status && s_wss_env.meas_ind_cfg[conn_idx])
    {
        if (BLE_GATT_INDICATION == p_ntf_ind->type)
        {
            wss_indicate_meas_value_chunk(conn_idx);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Handle Weight Measurement data indication.
 *
 * @param[in] conn_idx: Connection index.
 *
 * @return Result of handle.
 *****************************************************************************************
 */
static uint16_t wss_indicate_meas_value_chunk(uint8_t conn_idx)
{
    uint16_t             chunk_len;
    ble_gatts_noti_ind_t wss_meas_ind;
    sdk_err_t            error_code;
    
    chunk_len = s_wss_meas_val.length - s_wss_meas_val.offset;
    chunk_len = chunk_len > s_packet_length ? s_packet_length : chunk_len;
    
    if (0 == chunk_len)
    {
        s_wss_meas_val.p_data = NULL;
        s_wss_meas_val.length = 0;
        s_wss_meas_val.offset = 0;
        
        return SDK_SUCCESS;
    }

    wss_meas_ind.type   = BLE_GATT_INDICATION;
    wss_meas_ind.handle = prf_find_handle_by_idx(WSS_IDX_WEIGHT_MEAS_VAL,
                                                 s_wss_env.start_hdl,
                                                 (uint8_t *)&s_wss_env.wss_init.char_mask);
    wss_meas_ind.length = chunk_len;
    wss_meas_ind.value  = (uint8_t *)s_wss_meas_val.p_data + s_wss_meas_val.offset;

    error_code = ble_gatts_noti_ind(conn_idx, &wss_meas_ind);
    if (SDK_SUCCESS == error_code)
    {
        s_wss_meas_val.offset += chunk_len;
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Encoding a Weight Scale Measurement.
 *
 * @param[in]  p_wss_meas_val: Pointer to WS measurement value to be encoded.
 * @param[in]  cache_num:      The number of measurment caches.
 *
 * @return Length of encoded data.
 *****************************************************************************************
 */
static uint16_t wss_meas_value_encoded(wss_meas_val_t *p_wss_meas_val, uint8_t cache_num)
{
    uint8_t  flags  = 0;
    uint16_t length = 0;
    double   height_in_m;
    double   weight_in_kg;
    
    memset(local_buff, 0, sizeof(local_buff));
    s_wss_meas_val.length = 0;
    
    for (uint8_t i = 0; i < cache_num; i++)
    {
        length++;
        if (WSS_MEAS_UNSUCCESS != p_wss_meas_val->weight)
        {
            // Weight Field
            local_buff[length++] = LO_U16(p_wss_meas_val[i].weight);
            local_buff[length++] = HI_U16(p_wss_meas_val[i].weight);
        }
        else
        {
            // Weight Field
            local_buff[length++] = LO_U16(WSS_MEAS_UNSUCCESS);
            local_buff[length++] = HI_U16(WSS_MEAS_UNSUCCESS);
        }
        
        // Time Stamp Field
        if (s_wss_env.wss_init.feature & WSS_FEAT_TIME_STAMP && (s_wss_env.wss_init.time_stamp_present))
        {
            local_buff[length++] = LO_U16(p_wss_meas_val[i].time_stamp.year);
            local_buff[length++] = HI_U16(p_wss_meas_val[i].time_stamp.year);
            local_buff[length++] = p_wss_meas_val[i].time_stamp.month;
            local_buff[length++] = p_wss_meas_val[i].time_stamp.day;
            local_buff[length++] = p_wss_meas_val[i].time_stamp.hour;
            local_buff[length++] = p_wss_meas_val[i].time_stamp.min;
            local_buff[length++] = p_wss_meas_val[i].time_stamp.sec;
            flags |= WSS_MEAS_FLAG_DATE_TIME_PRESENT;
        }

        // User ID Field
        if (s_wss_env.wss_init.feature & WSS_FEAT_MULTI_USER && (s_wss_env.wss_init.multi_user_present))
        {
            local_buff[length++] = p_wss_meas_val[i].user_id;
            flags |= WSS_MEAS_FLAG_USER_ID_PRESENT;
        }

        if (WSS_MEAS_UNSUCCESS != p_wss_meas_val->weight)
        {
            // BMI and Height Fields
            if (WSS_FEAT_BMI & s_wss_env.wss_init.feature && (s_wss_env.wss_init.bmi_present))
            {
                uint16_t bmi;
                
                s_packet_length = WSS_MEAS_VAL_LEN_MAX;
                
                if (WSS_UNIT_SI == s_wss_env.wss_init.wss_unit)
                {
                    if ((WSS_FEAT_MASS_RES_5G & s_wss_env.wss_init.feature) && \
                        (WSS_FEAT_HEIGHT_RES_1MM & s_wss_env.wss_init.feature) && \
                        (WSS_HEIGHT_RES_1MM == s_wss_env.wss_init.wss_height_res) && \
                        (WSS_MASS_RES_5G == s_wss_env.wss_init.wss_mass_res))
                    {
                        weight_in_kg = p_wss_meas_val[i].weight * 0.005;
                        height_in_m  = p_wss_meas_val->height * 0.01;
                        bmi = (weight_in_kg/(height_in_m * height_in_m)) * 10;
                    }
                }
                else
                {
                    if ((WSS_FEAT_MASS_RES_5G & s_wss_env.wss_init.feature) && \
                        (WSS_FEAT_HEIGHT_RES_1MM & s_wss_env.wss_init.feature) && \
                        (WSS_HEIGHT_RES_1MM == s_wss_env.wss_init.wss_height_res) && \
                        (WSS_MASS_RES_5G == s_wss_env.wss_init.wss_mass_res))
                    {
                        weight_in_kg = p_wss_meas_val[i].weight * 0.01 * 0.4535924; // turn lb to kg.
                        height_in_m  = p_wss_meas_val->height * 0.1 * 0.0254; // turn in to m.
                        bmi = (weight_in_kg/(height_in_m * height_in_m)) * 10;
                    }                    
                }
                local_buff[length++] = LO_U16(bmi);
                local_buff[length++] = HI_U16(bmi);
                local_buff[length++] = LO_U16(p_wss_meas_val->height * 10);
                local_buff[length++] = HI_U16(p_wss_meas_val->height * 10);

                flags |= WSS_MEAS_FLAG_BMI_HEIGHT_PRESENT;
            }
            else if (WSS_FEAT_BMI & s_wss_env.wss_init.feature && (!s_wss_env.wss_init.bmi_present))
            {
                s_packet_length = WSS_MEAS_VAL_LEN_MAX - 4;
            }
        }
        else
        {
            s_packet_length = WSS_MEAS_VAL_LEN_MAX - 4;
        }
        
        if (WSS_UNIT_SI == s_wss_env.wss_init.wss_unit)
        {
            flags |= WSS_MEAS_FLAG_UNIT_SI;
        }
        else
        {
            flags |= WSS_MEAS_FLAG_UNIT_IMPERIAL;
        }
        
        local_buff[s_wss_meas_val.length] = flags;
        s_wss_meas_val.length = length;
    }
    
    s_wss_meas_val.offset = 0;
    s_wss_meas_val.p_data = &local_buff[0];
    
    return length;
}

static void wss_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            wss_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;
        
        case BLE_GATTS_EVT_WRITE_REQUEST:
            wss_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        
        case BLE_GATTS_EVT_NTF_IND:
            wss_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            wss_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t wss_measurement_send(uint8_t conn_idx, wss_meas_val_t *p_wss_meas_val, uint8_t cache_num)
{
    if (NULL == p_wss_meas_val || 25 < cache_num)
    {
        return SDK_ERR_INVALID_PARAM;
    }
    
    sdk_err_t error_code = SDK_ERR_IND_DISABLED;
    
    if (PRF_CLI_START_IND == s_wss_env.meas_ind_cfg[conn_idx])
    {
        wss_meas_value_encoded(p_wss_meas_val, cache_num);
        error_code = wss_indicate_meas_value_chunk(conn_idx);
    }

    return error_code;
}

sdk_err_t wss_service_init(wss_init_t *p_wss_init, uint16_t *p_incl_srvc_start_handle)
{
    if (NULL == p_wss_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_wss_env, 0, sizeof(s_wss_env));
    memcpy(&s_wss_env.wss_init, p_wss_init, sizeof(wss_init_t));
    s_wss_env.p_incl_srvc_start_handle = p_incl_srvc_start_handle;
    
    s_wss_env.start_hdl                         = PRF_INVALID_HANDLE;
    s_wss_env.wss_gatts_db.shdl                 = &s_wss_env.start_hdl;
    s_wss_env.wss_gatts_db.uuid                 = s_wss_svc_uuid;
    s_wss_env.wss_gatts_db.attr_tab_cfg         = (uint8_t *)&(s_wss_env.wss_init.char_mask);
    s_wss_env.wss_gatts_db.max_nb_attr          = WSS_IDX_NB;
    s_wss_env.wss_gatts_db.srvc_perm            = 0;
    s_wss_env.wss_gatts_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_wss_env.wss_gatts_db.attr_tab.attr_tab_16 = wss_attr_tab;
    s_wss_env.wss_gatts_db.inc_srvc_num         = 1;
    s_wss_env.wss_gatts_db.inc_srvc_handle[0]   = p_incl_srvc_start_handle;

    return ble_gatts_prf_add(&s_wss_env.wss_gatts_db, wss_ble_evt_handler);
}


