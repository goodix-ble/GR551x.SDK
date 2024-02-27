/**
 ****************************************************************************************
 *
 * @file gls.c
 *
 * @brief Glucose Profile Sensor implementation.
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
#include "gls.h"
#include "gls_db.h"
#include "gls_racp.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_log.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief Glucose Service Attributes Indexes. */
enum
{
    // Glucose Service
    GLS_IDX_SVC,

    // Glucose Measurement
    GLS_IDX_MEAS_CHAR,
    GLS_IDX_MEAS_VAL,
    GLS_IDX_MEAS_NTF_CFG,

    // Glucose Measurement Context
    GLS_IDX_MEAS_CTX_CHAR,
    GLS_IDX_MEAS_CTX_VAL,
    GLS_IDX_MEAS_CTX_NTF_CFG,

    // Glucose Feature
    GLS_IDX_FEATURE_CHAR,
    GLS_IDX_FEATURE_VAL,

    // Record Access Control Point
    GLS_IDX_REC_ACCESS_CTRL_CHAR,
    GLS_IDX_REC_ACCESS_CTRL_VAL,
    GLS_IDX_REC_ACCESS_CTRL_IND_CFG,

    GLS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Glucose Service environment variable. */
struct gls_env_t
{
    gls_init_t              gls_init;                               /**< Glucose Service initialization variables. */
    uint16_t                start_hdl;                              /**< Glucose Service start handle. */
    uint16_t                next_seq_num;                           /**< Sequence number of the next database record. */
    uint8_t                 proc_record_idx;                        /**< Current record index. */
    uint16_t                proc_record_seq_num;                    /**< Sequence number of current request. */
    uint16_t                proc_records_reported;                  /**< Number of reported records. */
    bool                    is_record_continue_send;                /**< State for continue send record. */
    bool                    racp_in_progress;                       /**< A previously triggered Control Point operation is still in progress. */
    uint8_t                 ntf_mask;                               /**< Mask for measurement notify or measurement context notify. */
    gls_racp_req_t          racp_req;                               /**< Buffer saved current RACP request decode result. */
    uint16_t                meas_ntf_cfg[GLS_CONNECTION_MAX];       /**< The configuration of Glucose Measurement Notification which is configured by the peer devices. */
    uint16_t                meas_ctx_ntf_cfg[GLS_CONNECTION_MAX];   /**< The configuration of Glucose Measurement Context Notification which is configured by the peer devices. */
    uint16_t                racp_ind_cfg[GLS_CONNECTION_MAX];       /**< The configuration of Record Access Control Point Indication which is configured by the peer devices. */
    ble_gatts_create_db_t   gls_gatts_db;                            /**< Glucose Service attributs database. */
};

/*
* LOCAL FUNCTION DECLARATION
****************************************************************************************
*/
static void        gls_receive_racp_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length);
static bool        gls_are_meas_racp_cccd_configured(uint8_t conn_idx);
static sdk_err_t   gls_meas_val_send(uint8_t conn_idx, gls_rec_t *p_rec);

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static struct gls_env_t s_gls_env;
static const uint8_t    s_gls_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_GLUCOSE);

/**@brief Full GLS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t gls_attr_tab[GLS_IDX_NB] =
{
    // Glucose Service Declaration
    [GLS_IDX_SVC]          = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Glucose Measurement Characteristic Declaration
    [GLS_IDX_MEAS_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Glucose Measurement Characteristic Value
    [GLS_IDX_MEAS_VAL]     = {BLE_ATT_CHAR_GLUCOSE_MEAS, BLE_GATTS_NOTIFY_PERM(BLE_GATTS_AUTH),
                              BLE_GATTS_ATT_VAL_LOC_USER, GLS_MEAS_VAL_LEN_MAX},
    // Glucose Measurement Characteristic - Client Characteristic Configuration Descriptor
    [GLS_IDX_MEAS_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                              BLE_GATTS_READ_PERM(BLE_GATTS_AUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH),
                              0, 0},

    // Glucose Measurement Context Characteristic Declaration
    [GLS_IDX_MEAS_CTX_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Glucose Measurement Context Characteristic Value
    [GLS_IDX_MEAS_CTX_VAL]     = {BLE_ATT_CHAR_GLUCOSE_MEAS_CTX, BLE_GATTS_NOTIFY_PERM(BLE_GATTS_AUTH),
                                  BLE_GATTS_ATT_VAL_LOC_USER, GLS_MEAS_CTX_LEN_MAX},
    // Glucose Measurement Context Characteristic - Client Characteristic Configuration Descriptor
    [GLS_IDX_MEAS_CTX_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                  BLE_GATTS_READ_PERM(BLE_GATTS_AUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH),
                                  0, 0},

    // Glucose Features Characteristic Declaration
    [GLS_IDX_FEATURE_CHAR]     = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Glucose Features Characteristic Value
#if defined(PTS_AUTO_TEST)
    [GLS_IDX_FEATURE_VAL]      = {BLE_ATT_CHAR_GLUCOSE_FEATURE, BLE_GATTS_READ_PERM_UNSEC,
                                  BLE_GATTS_ATT_VAL_LOC_USER, sizeof(uint16_t)},
#else
    [GLS_IDX_FEATURE_VAL]      = {BLE_ATT_CHAR_GLUCOSE_FEATURE, BLE_GATTS_READ_PERM(BLE_GATTS_AUTH),
                                  BLE_GATTS_ATT_VAL_LOC_USER, sizeof(uint16_t)},
#endif

    // Record Access Control Point characteristic Declaration
    [GLS_IDX_REC_ACCESS_CTRL_CHAR]     = {BLE_ATT_DECL_CHARACTERISTIC,  BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Record Access Control Point characteristic Value
    [GLS_IDX_REC_ACCESS_CTRL_VAL]      = {BLE_ATT_CHAR_REC_ACCESS_CTRL_PT,
                                          BLE_GATTS_INDICATE_PERM_UNSEC | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH),
                                          BLE_GATTS_ATT_VAL_LOC_USER,
                                          GLS_REC_ACCESS_CTRL_LEN_MAX},
    // Record Access Control Point characteristic - Client Characteristic Configuration Descriptor
    [GLS_IDX_REC_ACCESS_CTRL_IND_CFG]  = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                          BLE_GATTS_READ_PERM(BLE_GATTS_AUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH),
                                          0, 0},

};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  The parameters of the read request.
 *****************************************************************************************
 */
static void gls_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = prf_find_idx_by_handle(handle,
                                  s_gls_env.start_hdl,
                                  GLS_IDX_NB,
                                  (uint8_t *)&s_gls_env.gls_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case GLS_IDX_MEAS_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_gls_env.meas_ntf_cfg[conn_idx];
            break;

        case GLS_IDX_MEAS_CTX_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_gls_env.meas_ctx_ntf_cfg[conn_idx];
            break;

        case GLS_IDX_FEATURE_VAL:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_gls_env.gls_init.feature;
            break;

        case GLS_IDX_REC_ACCESS_CTRL_IND_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_gls_env.racp_ind_cfg[conn_idx];
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
static void gls_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t          handle      = p_param->handle;
    uint16_t          tab_index   = 0;
    uint16_t          cccd_value  = 0;
    bool              racp_evt    = false;
    gls_evt_t         event;
    ble_gatts_write_cfm_t cfm;

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_gls_env.start_hdl,
                                        GLS_IDX_NB,
                                        (uint8_t *)&s_gls_env.gls_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = GLS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case GLS_IDX_MEAS_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              GLS_EVT_MEAS_NOTIFICATION_ENABLED : \
                              GLS_EVT_MEAS_NOTIFICATION_DISABLED);
            s_gls_env.meas_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GLS_IDX_MEAS_CTX_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              GLS_EVT_CTX_NOTIFICATION_ENABLED : \
                              GLS_EVT_CTX_NOTIFICATION_DISABLED);
            s_gls_env.meas_ctx_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GLS_IDX_REC_ACCESS_CTRL_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              GLS_EVT_CTRL_INDICATION_ENABLED : \
                              GLS_EVT_CTRL_INDICATION_DISABLED);
            s_gls_env.racp_ind_cfg[conn_idx] = cccd_value;
            break;

        case GLS_IDX_REC_ACCESS_CTRL_VAL:
            if (!gls_are_meas_racp_cccd_configured(conn_idx))
            {
                cfm.status = GLS_ERROR_CCCD_INVALID;
            }
            else if (s_gls_env.racp_in_progress && GLS_RACP_OP_ABORT_OP != p_param->value[0])
            {
                cfm.status = GLS_ERROR_PROC_IN_PROCESS;
            }
            else
            {
                racp_evt = true;
            }
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (racp_evt)
    {
        gls_receive_racp_handler(conn_idx, p_param->value, p_param->length);
    }
    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && GLS_EVT_INVALID != event.evt_type && s_gls_env.gls_init.evt_handler)
    {
        s_gls_env.gls_init.evt_handler(&event);
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
static void gls_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index   = 0;
    gls_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_gls_env.start_hdl,
                                        GLS_IDX_NB,
                                        (uint8_t *)&s_gls_env.gls_init.char_mask);

    event.evt_type = GLS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case GLS_IDX_MEAS_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              GLS_EVT_MEAS_NOTIFICATION_ENABLED : \
                              GLS_EVT_MEAS_NOTIFICATION_DISABLED);
            s_gls_env.meas_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GLS_IDX_MEAS_CTX_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              GLS_EVT_CTX_NOTIFICATION_ENABLED : \
                              GLS_EVT_CTX_NOTIFICATION_DISABLED);
            s_gls_env.meas_ctx_ntf_cfg[conn_idx] = cccd_value;
            break;

        case GLS_IDX_REC_ACCESS_CTRL_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              GLS_EVT_CTRL_INDICATION_ENABLED : \
                              GLS_EVT_CTRL_INDICATION_DISABLED);
            s_gls_env.racp_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (GLS_EVT_INVALID != event.evt_type && s_gls_env.gls_init.evt_handler)
    {
        s_gls_env.gls_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the disconnection event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] reason:   Reason of disconnection.
 *****************************************************************************************
 */
static void gls_disconnect_evt_handler(uint8_t conn_idx, uint8_t reason)
{
    s_gls_env.racp_in_progress = false;
}

/**
 *****************************************************************************************
 * @brief Check glucose measurement notification and record access control point indication are both configured.
 *
 * @param[in] conn_idx: Connection index.
 *
 * @return Result of checking.
 *****************************************************************************************
 */
static bool gls_are_meas_racp_cccd_configured(uint8_t conn_idx)
{
    if ((PRF_CLI_STOP_NTFIND == s_gls_env.meas_ntf_cfg[conn_idx]) || \
            (PRF_CLI_STOP_NTFIND == s_gls_env.racp_ind_cfg[conn_idx]))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 *****************************************************************************************
 * @brief Set the next sequence number by reading the last record in the data base.
 *
 * @return If set successfully or not.
 *****************************************************************************************
 */
static bool gls_next_sequence_num_set(void)
{
    uint16_t  records_num;
    gls_rec_t gls_res;

    records_num = gls_db_records_num_get();

    if (0 < records_num)
    {
        if (gls_db_record_get(records_num - 1, &gls_res))
        {
            s_gls_env.next_seq_num = gls_res.meas_val.sequence_number + 1;
        }
        else
        {
            return false;
        }
    }
    else
    {
        s_gls_env.next_seq_num = 0;
    }

    return true;
}

/**
 *****************************************************************************************
 * @brief Informing that the REPORT RECORDS procedure is completed.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void gls_report_records_completed(uint8_t conn_idx)
{
    gls_racp_rsp_t  racp_rsp;
    uint8_t         encoded_racp_rsp[GLS_REC_ACCESS_CTRL_LEN_MAX];
    uint16_t        encode_length;

    memset(&racp_rsp, 0, sizeof(gls_racp_rsp_t));

    racp_rsp.op_code                 = GLS_RACP_OP_RSP_CODE;
    racp_rsp.operand.rsp.op_code_req = GLS_RACP_OP_REP_STRD_RECS;

    if (s_gls_env.proc_records_reported)
    {
        racp_rsp.operand.rsp.status = GLS_RACP_RSP_SUCCESS;
    }
    else
    {
        racp_rsp.operand.rsp.status = GLS_RACP_RSP_NO_RECS_FOUND;
    }

    encode_length = gls_racp_rsp_encode(&racp_rsp, encoded_racp_rsp);
    gls_racp_rsp_send(conn_idx, encoded_racp_rsp, encode_length);
}

/**
 *****************************************************************************************
 * @brief Report all records handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_all_records_report(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    uint16_t  records_num;
    gls_rec_t gls_res;

    records_num = gls_db_records_num_get();

    if (s_gls_env.proc_record_idx >= records_num)
    {
        s_gls_env.is_record_continue_send = false;
        s_gls_env.racp_in_progress        = false;
        gls_report_records_completed(conn_idx);
    }
    else if (gls_db_record_get(s_gls_env.proc_record_idx, &gls_res))
    {
        s_gls_env.is_record_continue_send = true;
        gls_meas_val_send(conn_idx, &gls_res);
    }
}

/**
 *****************************************************************************************
 * @brief Report less or equal setting value records handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_less_or_equal_records_report(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    uint16_t  records_num;
    gls_rec_t gls_res;

    records_num = gls_db_records_num_get();

    while (s_gls_env.proc_record_idx < records_num)
    {
        if (gls_db_record_get(s_gls_env.proc_record_idx, &gls_res))
        {
            if (s_gls_env.proc_record_seq_num >= gls_res.meas_val.sequence_number)
            {
                s_gls_env.is_record_continue_send = true;
                gls_meas_val_send(conn_idx, &gls_res);
                return;
            }

            s_gls_env.proc_record_idx++;
        }
        else
        {
            break;
        }
    };

    s_gls_env.is_record_continue_send = false;
    s_gls_env.racp_in_progress        = false;
    gls_report_records_completed(conn_idx);
}

/**
 *****************************************************************************************
 * @brief Report greater or equal setting value records handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_greater_or_equal_records_report(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    uint16_t  records_num;
    gls_rec_t gls_res;

    records_num = gls_db_records_num_get();

    while (s_gls_env.proc_record_idx < records_num)
    {
        if (gls_db_record_get(s_gls_env.proc_record_idx, &gls_res))
        {
            if (s_gls_env.proc_record_seq_num <= gls_res.meas_val.sequence_number)
            {
                s_gls_env.is_record_continue_send = true;
                gls_meas_val_send(conn_idx, &gls_res);
                return;
            }

            s_gls_env.proc_record_idx++;
        }
        else
        {
            break;
        }
    };

    s_gls_env.is_record_continue_send = false;
    s_gls_env.racp_in_progress        = false;
    gls_report_records_completed(conn_idx);
}

/**
 *****************************************************************************************
 * @brief Report within range of setting value records handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_within_range_of_records_report(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    uint16_t  records_num;
    gls_rec_t gls_res;

    records_num = gls_db_records_num_get();

    while (s_gls_env.proc_record_idx < records_num)
    {
        if (gls_db_record_get(s_gls_env.proc_record_idx, &gls_res))
        {
            if (GLS_RACP_FILTER_SEQ_NUMBER == p_racp_req->filter.racp_filter_type)
            {
                if ((gls_res.meas_val.sequence_number >= s_gls_env.racp_req.filter.val.seq_num.min) && \
                        (gls_res.meas_val.sequence_number <= s_gls_env.racp_req.filter.val.seq_num.max))
                {
                    s_gls_env.is_record_continue_send = true;
                    gls_meas_val_send(conn_idx, &gls_res);
                    return;
                }
            }
            else if (GLS_RACP_FILTER_USER_FACING_TIME == p_racp_req->filter.racp_filter_type)
            {
                if ( !(-1 == gls_racp_user_time_compare(&gls_res.meas_val.base_time, &s_gls_env.racp_req.filter.val.time.min)) && \
                        !(-1 == gls_racp_user_time_compare(&s_gls_env.racp_req.filter.val.time.max, &gls_res.meas_val.base_time)))
                {
                    s_gls_env.is_record_continue_send = true;
                    gls_meas_val_send(conn_idx, &gls_res);
                    break;
                }
            }
            s_gls_env.proc_record_idx++;
        }
        else
        {
            break;
        }
    };

    s_gls_env.is_record_continue_send = false;
    s_gls_env.racp_in_progress        = false;
    gls_report_records_completed(conn_idx);
}

/**
 *****************************************************************************************
 * @brief Report first or last records handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_first_or_last_records_report(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    uint16_t  records_num;
    gls_rec_t gls_res;
    bool      is_get_rec;

    records_num = gls_db_records_num_get();

    if (0 < s_gls_env.proc_records_reported || records_num == 0)
    {
        s_gls_env.is_record_continue_send = false;
        s_gls_env.racp_in_progress        = false;
        gls_report_records_completed(conn_idx);
    }
    else
    {
        s_gls_env.is_record_continue_send = true;
        is_get_rec                        = false;

        if (GLS_RACP_OPERATOR_FIRST_REC == p_racp_req->filter.racp_operator)
        {
            is_get_rec = gls_db_record_get(0, &gls_res);
        }
        else if (GLS_RACP_OPERATOR_LAST_REC == p_racp_req->filter.racp_operator)
        {
            is_get_rec = gls_db_record_get(records_num - 1, &gls_res);
        }

        if (is_get_rec)
        {
            gls_meas_val_send(conn_idx, &gls_res);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Report records request handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_report_records_req_handler(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    switch (p_racp_req->filter.racp_operator)
    {
        case GLS_RACP_OPERATOR_ALL_RECS:
            gls_all_records_report(conn_idx, p_racp_req);
            break;

        case GLS_RACP_OPERATOR_LE_OR_EQ:
            s_gls_env.proc_record_seq_num = p_racp_req->filter.val.seq_num.max;
            gls_less_or_equal_records_report(conn_idx, p_racp_req);
            break;

        case GLS_RACP_OPERATOR_GT_OR_EQ:
            s_gls_env.proc_record_seq_num = p_racp_req->filter.val.seq_num.min;
            gls_greater_or_equal_records_report(conn_idx, p_racp_req);
            break;

        case GLS_RACP_OPERATOR_WITHIN_RANGE_OF:
            gls_within_range_of_records_report(conn_idx, p_racp_req);
            break;

        case GLS_RACP_OPERATOR_FIRST_REC:
        case GLS_RACP_OPERATOR_LAST_REC:
            gls_first_or_last_records_report(conn_idx, p_racp_req);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Report number of records request handler.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_racp_rsp: Pointer to buffer saved response.
 *****************************************************************************************
 */
static void gls_report_records_num_req_handler(uint8_t conn_idx, gls_racp_req_t *p_racp_req)
{
    gls_racp_rsp_t  racp_rsp;
    uint8_t         encoded_racp_rsp[GLS_REC_ACCESS_CTRL_LEN_MAX];
    uint16_t        encode_length;
    uint16_t        req_num;

    req_num = gls_db_filter_records_num_get(&p_racp_req->filter);

    racp_rsp.op_code               = GLS_RACP_OP_NB_OF_STRD_RECS_RSP;
    racp_rsp.racp_operator         = GLS_RACP_OPERATOR_NULL;
    racp_rsp.operand.num_of_record = req_num;

    encode_length = gls_racp_rsp_encode(&racp_rsp, encoded_racp_rsp);
    gls_racp_rsp_send(conn_idx, encoded_racp_rsp, encode_length);
}

/**
 *****************************************************************************************
 * @brief Abort operation request handler.
 *
 * @param[in] conn_idx:   Connection index.
 *****************************************************************************************
 */
static void gls_abort_operation_handler(uint8_t conn_idx)
{
    gls_racp_rsp_t      racp_rsp;
    uint8_t             encoded_racp_rsp[GLS_REC_ACCESS_CTRL_LEN_MAX];
    uint16_t            encode_length;

    memset(&racp_rsp, 0, sizeof(gls_racp_rsp_t));

    if (s_gls_env.racp_in_progress)
    {
        s_gls_env.racp_in_progress = false;
    }

    racp_rsp.operand.rsp.status = GLS_RACP_RSP_SUCCESS;

    racp_rsp.op_code                 = GLS_RACP_OP_RSP_CODE;
    racp_rsp.racp_operator           = GLS_RACP_OPERATOR_NULL;
    racp_rsp.operand.rsp.op_code_req = GLS_RACP_OP_ABORT_OP;
    encode_length = gls_racp_rsp_encode(&racp_rsp, encoded_racp_rsp);
    gls_racp_rsp_send(conn_idx, encoded_racp_rsp, encode_length);
}

/**
 *****************************************************************************************
 * @brief Record Access Control Point receive handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void gls_receive_racp_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    gls_racp_rsp_t      racp_rsp;
    gls_racp_operand_t  status;
    uint8_t             encoded_racp_rsp[GLS_REC_ACCESS_CTRL_LEN_MAX];
    uint16_t            encode_length;

    memset(&s_gls_env.racp_req, 0, sizeof(gls_racp_req_t));
    memset(&racp_rsp, 0, sizeof(gls_racp_rsp_t));

    if (GLS_REC_ACCESS_CTRL_LEN_MIN <= length)
    {
        status = gls_racp_req_decode(p_data, length, &s_gls_env.racp_req);

        if ((GLS_RACP_RSP_VALID_DECODE == status) && \
                (GLS_RACP_OP_REP_STRD_RECS <= s_gls_env.racp_req.op_code) && \
                (GLS_RACP_OP_REP_NB_OF_STRD_RECS >= s_gls_env.racp_req.op_code))
        {
            if (GLS_RACP_OP_REP_STRD_RECS == s_gls_env.racp_req.op_code)
            {
                s_gls_env.racp_in_progress      = true;
                s_gls_env.proc_records_reported = 0;
                s_gls_env.proc_record_idx       = 0;
                gls_report_records_req_handler(conn_idx, &s_gls_env.racp_req);
            }
            else if (GLS_RACP_OP_REP_NB_OF_STRD_RECS == s_gls_env.racp_req.op_code)
            {
                s_gls_env.racp_in_progress = true;
                gls_report_records_num_req_handler(conn_idx, &s_gls_env.racp_req);
            }
            else if (GLS_RACP_OP_ABORT_OP == s_gls_env.racp_req.op_code)
            {
                gls_abort_operation_handler(conn_idx);
            }
        }

        else
        {
            if (GLS_RACP_RSP_VALID_DECODE != status)
            {
                racp_rsp.operand.rsp.status = status;
            }

            if ((GLS_RACP_OP_REP_STRD_RECS > s_gls_env.racp_req.op_code) || \
                    (GLS_RACP_OP_REP_NB_OF_STRD_RECS < s_gls_env.racp_req.op_code))
            {
                racp_rsp.operand.rsp.status = GLS_RACP_RSP_OP_CODE_NOT_SUP;
            }

            racp_rsp.op_code                 = GLS_RACP_OP_RSP_CODE;
            racp_rsp.operand.rsp.op_code_req = s_gls_env.racp_req.op_code;
            encode_length = gls_racp_rsp_encode(&racp_rsp, encoded_racp_rsp);
            gls_racp_rsp_send(conn_idx, encoded_racp_rsp, encode_length);
        }
    }

}

/**
 *****************************************************************************************
 * @brief Encode a Glucose measurement.
 *
 * @param[in]  p_meas:           Pointer to GLS measurement value to be encoded.
 * @param[out] p_encoded_buffer: Buffer where the encoded data will be written.
 *
 * @return Length of encoded data.
 *****************************************************************************************
 */
static uint8_t gls_meas_value_encode(const gls_meas_val_t *p_meas, uint8_t *p_encoded_buffer)
{
    uint8_t length = 0;

    p_encoded_buffer[length++] = p_meas->flags;

    p_encoded_buffer[length++] = LO_U16(p_meas->sequence_number);
    p_encoded_buffer[length++] = HI_U16(p_meas->sequence_number);

    p_encoded_buffer[length++] = LO_U16(p_meas->base_time.year);
    p_encoded_buffer[length++] = HI_U16(p_meas->base_time.year);
    p_encoded_buffer[length++] = p_meas->base_time.month;
    p_encoded_buffer[length++] = p_meas->base_time.day;
    p_encoded_buffer[length++] = p_meas->base_time.hour;
    p_encoded_buffer[length++] = p_meas->base_time.min;
    p_encoded_buffer[length++] = p_meas->base_time.sec;

    if (p_meas->flags & GLS_MEAS_FLAG_TIME_OFFSET)
    {
        p_encoded_buffer[length++] = LO_U16(p_meas->time_offset);
        p_encoded_buffer[length++] = HI_U16(p_meas->time_offset);
    }

    if (p_meas->flags & GLS_MEAS_FLAG_CONC_TYPE_LOC)
    {
        uint16_t encoded_concentration;

        encoded_concentration = ((p_meas->glucose_concentration.exponent << 12) & 0xF000) |
                                ((p_meas->glucose_concentration.mantissa <<  0) & 0x0FFF);

        p_encoded_buffer[length++] = (uint8_t)(encoded_concentration);
        p_encoded_buffer[length++] = (uint8_t)(encoded_concentration >> 8);
        p_encoded_buffer[length++] = (p_meas->sample_location << 4) | (p_meas->type & 0x0F);
    }

    if (p_meas->flags & GLS_MEAS_FLAG_SENSOR_STATUS)
    {
        p_encoded_buffer[length++] = LO_U16(p_meas->sensor_status_annunciation);
        p_encoded_buffer[length++] = HI_U16(p_meas->sensor_status_annunciation);
    }

    return length;
}

/**
 *****************************************************************************************
 * @brief Send a glucose measurement/context.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] p_rec: Pointer to measurement to be sent.
 *
 * @return Result of notify value
 *****************************************************************************************
 */
static sdk_err_t gls_meas_val_send(uint8_t conn_idx, gls_rec_t *p_rec)
{
    sdk_err_t        error_code = BLE_SUCCESS;
    uint8_t          encoded_glc_meas[GLS_MEAS_VAL_LEN_MAX];
    uint16_t         length;
    ble_gatts_noti_ind_t gls_ntf;

    length  = gls_meas_value_encode(&p_rec->meas_val, encoded_glc_meas);

    memset(&gls_ntf, 0, sizeof (ble_gatts_noti_ind_t));

    if (PRF_CLI_START_NTF == s_gls_env.meas_ntf_cfg[conn_idx])
    {
        gls_ntf.type   = BLE_GATT_NOTIFICATION;
        gls_ntf.handle = prf_find_handle_by_idx(GLS_IDX_MEAS_VAL,
                                                s_gls_env.start_hdl,
                                                (uint8_t *)&s_gls_env.gls_init.char_mask);
        gls_ntf.length = length;
        gls_ntf.value  = encoded_glc_meas;
        error_code     = ble_gatts_noti_ind(conn_idx, &gls_ntf);

        if (BLE_SUCCESS == error_code)
        {
            s_gls_env.ntf_mask = GLS_NTF_OF_MEAS;
        }
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void gls_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (SDK_SUCCESS == status)
    {
        if (BLE_GATT_INDICATION == p_ntf_ind->type)
        {
            s_gls_env.racp_in_progress = false;
        }
        else if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
        {
            if (s_gls_env.ntf_mask & GLS_NTF_OF_MEAS)
            {
                s_gls_env.ntf_mask = GLS_NTF_OF_NULL;

                if (s_gls_env.is_record_continue_send)
                {
                    s_gls_env.proc_record_idx++;
                    s_gls_env.proc_records_reported++;
                    gls_report_records_req_handler(conn_idx, &s_gls_env.racp_req);
                }
                else
                {
                    s_gls_env.racp_in_progress = false;
                }
            }
        }
    }
}

static void gls_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            gls_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            gls_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            gls_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            gls_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            gls_disconnect_evt_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
bool gls_new_meas_record(gls_rec_t *p_rec)
{
    p_rec->meas_val.sequence_number = s_gls_env.next_seq_num++;
    return gls_db_record_add(p_rec);
}

sdk_err_t   gls_racp_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    ble_gatts_noti_ind_t racp_rsp;

    if (PRF_CLI_START_IND == s_gls_env.racp_ind_cfg[conn_idx])
    {
        racp_rsp.type     = BLE_GATT_INDICATION;
        racp_rsp.handle   = prf_find_handle_by_idx(GLS_IDX_REC_ACCESS_CTRL_VAL,
                            s_gls_env.start_hdl,
                            (uint8_t *)&s_gls_env.gls_init.char_mask);
        racp_rsp.length = length;
        racp_rsp.value  = p_data;
        error_code      = ble_gatts_noti_ind(conn_idx, &racp_rsp);
    }

    return error_code;
}

sdk_err_t gls_service_init(gls_init_t *p_gls_init)
{
    if (NULL == p_gls_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_gls_env, 0, sizeof(s_gls_env));
    memcpy(&s_gls_env.gls_init, p_gls_init, sizeof(gls_init_t));
    gls_next_sequence_num_set();
    gls_db_init();

    s_gls_env.start_hdl  = PRF_INVALID_HANDLE;

    s_gls_env.gls_gatts_db.shdl                 = &s_gls_env.start_hdl;
    s_gls_env.gls_gatts_db.uuid                 = s_gls_svc_uuid;
    s_gls_env.gls_gatts_db.attr_tab_cfg         = (uint8_t *)&(s_gls_env.gls_init.char_mask);
    s_gls_env.gls_gatts_db.max_nb_attr          = GLS_IDX_NB;
    s_gls_env.gls_gatts_db.srvc_perm            = 0; 
    s_gls_env.gls_gatts_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_gls_env.gls_gatts_db.attr_tab.attr_tab_16 = gls_attr_tab;

    return ble_gatts_prf_add(&s_gls_env.gls_gatts_db, gls_ble_evt_handler);
}
