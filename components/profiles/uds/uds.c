/**
 ****************************************************************************************
 *
 * @file uds.c
 *
 * @brief User Data Service implementation.
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
#include "uds.h"
#include "wss_db.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_log.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief User Data Service Attributes Indexes. */
enum
{
    // User Data Service
    UDS_IDX_SVC,

    // Database Change Increment Characteristic
    UDS_IDX_DB_CHANGE_INCR_CHAR,
    UDS_IDX_DB_CHANGE_INCR_VAL,
    UDS_IDX_DB_CHANGE_INCR_NTF_CFG,

    // User Index Characteristic
    UDS_IDX_USER_INDEX_CHAR,
    UDS_IDX_USER_INDEX_VAL,

    // User Control Point Characteristic
    UDS_IDX_CTRL_POINT_CHAR,
    UDS_IDX_CTRL_POINT_VAL,
    UDS_IDX_CTRL_POINT_IND_CFG,

    // Age Characteristic
    UDS_IDX_AGE_CHAR,
    UDS_IDX_AGE_VAL,

    // Date of Birth Characteristics
    UDS_IDX_DATE_OF_BIRTH_CHAR,
    UDS_IDX_DATE_OF_BIRTH_VAL,

    // First Name Characteristic
    UDS_IDX_FIRST_NAME_CHAR,
    UDS_IDX_FIRST_NAME_VAL,

    // Height Characteristic
    UDS_IDX_HEIGHT_CHAR,
    UDS_IDX_HEIGHT_VAL,

    // Gender Characteristic
    UDS_IDX_GENDER_CHAR,
    UDS_IDX_GENDER_VAL,

    // Registered User Characteristic
    UDS_IDX_REGIST_USER_CHAR,
    UDS_IDX_REGIST_USER_VAL,
    UDS_IDX_REGIST_USER_IND_CFG,

    UDS_IDX_NB
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief User Data Service environment variable. */
struct uds_env_t
{
    uds_init_t               uds_init;                                    /**< User Data Service initialization variables. */
    uint16_t                 start_hdl;                                   /**< User Data Service start handle. */
    bool                     ucp_in_progress;                             /**< A previously triggered Control Point operation is still in progress. */
    uint8_t                  segm_head_roll_num;                          /**< Rolling Segment Number of Segmentation Header. */
    uint8_t                  consent_try_num;                             /**< The number of consent tries. */
    uint16_t                 db_change_incr_ntf_cfg[UDS_CONNECTION_MAX];  /**< The configuration of Current Time Notification which is configured by the peer devices. */
    uint16_t                 ctrl_point_ind_cfg[UDS_CONNECTION_MAX];      /**< The configuration of SC Control Point Notification which is configured by the peer devices. */
    uint16_t                 regist_user_ind_cfg[UDS_CONNECTION_MAX];     /**< The configuration of SC Control Point Notification which is configured by the peer devices. */
    ble_gatts_create_db_t    uds_gatts_db;                                /**< User Data Service attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static sdk_err_t   uds_age_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt);
static sdk_err_t   uds_date_of_birth_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt);
static sdk_err_t   uds_first_name_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt);
static sdk_err_t   uds_height_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt);
static sdk_err_t   uds_gender_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt);
static sdk_err_t   uds_db_change_incr_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt);

static void        uds_receive_ucp_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length);
static void        uds_regist_user_value_encode(uint8_t conn_idx);
static sdk_err_t   uds_indicate_user_char_val_chunk(uint8_t conn_idx);


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct uds_env_t            s_uds_env;
static uds_regi_user_data_stream_t s_regi_user_char_val_stream;
static uint8_t                     local_buf[UDS_REGI_USER_VAL_LEN_MAX * 5 * 3];
static uint16_t                    local_segm_length[10];
static const uint8_t               s_uds_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_USER_DATA);
extern wss_db_user_buf_t           s_wss_db_user_buf;
extern wss_rec_cmd_queue_t         s_wss_db_rec_cmd_queue;

/**@brief Full UDS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_t uds_attr_tab[UDS_IDX_NB] =
{
    // User Data Service Declaration
    [UDS_IDX_SVC]                    = {BLE_ATT_DECL_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // Database Change Increment Characteristic - Declaration
    [UDS_IDX_DB_CHANGE_INCR_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Database Change Increment Characteristic - Value
    [UDS_IDX_DB_CHANGE_INCR_VAL]     = {BLE_ATT_CHAR_DATABASE_CHANGE_INCREMENT,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_NOTIFY_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_DB_CHANGE_INCR_VAL_LEN_MAX},
    // Database Change Increment Characteristic - Client Characteristic Configuration Descriptor
    [UDS_IDX_DB_CHANGE_INCR_NTF_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, 
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        0,
                                        0},

    // User Index Characteristic - Declaration
    [UDS_IDX_USER_INDEX_CHAR]        = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // User Index Characteristic - Value
    [UDS_IDX_USER_INDEX_VAL]         = {BLE_ATT_CHAR_USER_INDEX,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_USER_INDEX_VAL_LEN_MAX},

    // User Control Point Characteristic - Declaration
    [UDS_IDX_CTRL_POINT_CHAR]        = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // User Control Point Characteristic - Value
    [UDS_IDX_CTRL_POINT_VAL]         = {BLE_ATT_CHAR_USER_CONTROL_POINT,
                                        BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_INDICATE_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_CTRL_PT_VAL_LEN_MAX},
    // User Control Point Characteristic - Client Characteristic Configuration Descriptor
    [UDS_IDX_CTRL_POINT_IND_CFG]     = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        0,
                                        0},
    
    // Age Characteristic - Declaration
    [UDS_IDX_AGE_CHAR]               = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Age Characteristic - Value
    [UDS_IDX_AGE_VAL]                = {BLE_ATT_CHAR_AGE,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_AGE_VAL_LEN_MAX},

    // Date of Birth Characteristic - Declaration
    [UDS_IDX_DATE_OF_BIRTH_CHAR]     = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Date of Birth Characteristic - Value
    [UDS_IDX_DATE_OF_BIRTH_VAL]      = {BLE_ATT_CHAR_DATE_OF_BIRTH,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_DATE_OF_BIRTH_VAL_LEN_MAX},
    
    // First Name Characteristic - Declaration
    [UDS_IDX_FIRST_NAME_CHAR]        = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // First Name Characteristic - Value
    [UDS_IDX_FIRST_NAME_VAL]         = {BLE_ATT_CHAR_FIRST_NAME,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_FIRST_NAME_VAL_LEN_MAX},

    // Height Characteristic - Declaration
    [UDS_IDX_HEIGHT_CHAR]            = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Height Characteristic - Value
    [UDS_IDX_HEIGHT_VAL]             = {BLE_ATT_CHAR_HEIGHT,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_HEIGHT_VAL_LEN_MAX},
    
    // Gender Characteristic - Declaration
    [UDS_IDX_GENDER_CHAR]            = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Gender Characteristic - Value
    [UDS_IDX_GENDER_VAL]             = {BLE_ATT_CHAR_GENDER,
                                        BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_GENDER_VAL_LEN_MAX},
    
    // Registered User Characteristic Declaration
    [UDS_IDX_REGIST_USER_CHAR]       = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // Registered User Characteristic Declaration value
    [UDS_IDX_REGIST_USER_VAL]        = {BLE_ATT_CHAR_REGISTERED_USER,
                                        BLE_GATTS_INDICATE_PERM(BLE_GATTS_UNAUTH),
                                        BLE_GATTS_ATT_VAL_LOC_USER,
                                        UDS_REGI_USER_VAL_LEN_MAX},
    // Registered User Characteristic Declaration  - Client Characteristic Configuration Descriptor
    [UDS_IDX_REGIST_USER_IND_CFG]    = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
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
 * @brief Handles reception of the disconnection event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] reason:   Reason of disconnection.
 *****************************************************************************************
 */
static void uds_disconnect_evt_handler(uint8_t conn_idx, uint8_t reason)
{
    s_uds_env.ucp_in_progress = false;
}

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void uds_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t  cfm;
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = prf_find_idx_by_handle(handle,
                                                         s_uds_env.start_hdl,
                                                         UDS_IDX_NB,
                                                         (uint8_t *)&s_uds_env.uds_init.char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    uint32_t db_change_incr_val;

    switch (tab_index)
    {
        case UDS_IDX_AGE_VAL:
        {
            if (s_uds_env.uds_init.user_index == UDS_UNKNOWN_USER)
            {
                cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            else
            {   
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)&s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].age;
            }
            break;
        }

        case UDS_IDX_DATE_OF_BIRTH_VAL:
        {
            if (s_uds_env.uds_init.user_index == UDS_UNKNOWN_USER)
            {
                cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            else
            {
                cfm.length = sizeof(birth_date_t);
                cfm.value  = (uint8_t *)&s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].date_of_birth;
            }
            break;
        }

        case UDS_IDX_FIRST_NAME_VAL:
        {
            if (s_uds_env.uds_init.user_index == UDS_UNKNOWN_USER)
            {
                cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            else
            {
                cfm.length = s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].name_length;
                cfm.value  = (uint8_t *)&s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].first_name;
            }
            break;
        }

        case UDS_IDX_HEIGHT_VAL:
        {
            if (s_uds_env.uds_init.user_index == UDS_UNKNOWN_USER)
            {
                cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            else
            {
                cfm.length = sizeof(uint16_t);
                cfm.value  = (uint8_t *)&s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].height;
            }
            break;
        }

        case UDS_IDX_GENDER_VAL:
        {
            if (s_uds_env.uds_init.user_index == UDS_UNKNOWN_USER)
            {
                cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            else
            {
                cfm.length = sizeof(uint8_t);
                cfm.value  = (uint8_t *)&s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].gender;
            }
            break;
        }

        case UDS_IDX_DB_CHANGE_INCR_VAL:
        {      
            if (UDS_UNKNOWN_USER == s_uds_env.uds_init.user_index)
            {
                db_change_incr_val   = UDS_DB_CHANGE_INCR_DEFAULT_VAL;

                cfm.length           = sizeof(uint32_t);
                cfm.value            = (uint8_t *)&db_change_incr_val;
            }
            else
            {
                cfm.length = sizeof(uint32_t);
                cfm.value  = (uint8_t *)&(s_wss_db_user_buf.user_data[s_uds_env.uds_init.user_index].db_change_incr_val);
            }
            break;
        }

        case UDS_IDX_DB_CHANGE_INCR_NTF_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_uds_env.db_change_incr_ntf_cfg[conn_idx];
            break;

        case UDS_IDX_USER_INDEX_VAL:
        {
            cfm.length = sizeof(uint8_t);
            cfm.value  = (uint8_t *)&s_uds_env.uds_init.user_index;
            break;
        }

        case UDS_IDX_REGIST_USER_IND_CFG:
        {
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_uds_env.regist_user_ind_cfg[conn_idx];
            break;
        }

        case UDS_IDX_CTRL_POINT_IND_CFG:
        {
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_uds_env.ctrl_point_ind_cfg[conn_idx];
            break;
        }

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
 * @param[in]: p_param:  Pointer to the parameters of the write request.
 *****************************************************************************************
 */
static void uds_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t          handle      = p_param->handle;
    uint16_t          tab_index   = 0;
    uint16_t          cccd_value  = 0;
    bool              ucp_evt     = false;
    uds_evt_t         event;
    ble_gatts_write_cfm_t cfm;

    tab_index      = prf_find_idx_by_handle(handle,
                                            s_uds_env.start_hdl,
                                            UDS_IDX_NB,
                                            (uint8_t *)&s_uds_env.uds_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = UDS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case UDS_IDX_AGE_VAL:
            if (UDS_UNKNOWN_USER == s_uds_env.uds_init.user_index)
            {
                cfm.status = UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            
            uds_age_write_handler(p_param->value, p_param->length, &cfm, &event);
            break;

        case UDS_IDX_DATE_OF_BIRTH_VAL:
            if (UDS_UNKNOWN_USER  == s_uds_env.uds_init.user_index)
            {
                cfm.status = UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            
            uds_date_of_birth_write_handler(p_param->value, p_param->length, &cfm, &event);
            break;

        case UDS_IDX_FIRST_NAME_VAL:
            if (UDS_UNKNOWN_USER  == s_uds_env.uds_init.user_index)
            {
                cfm.status = UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            event.evt_type = UDS_EVT_FIRST_NAME_SET_BY_PEER;
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            
            uds_first_name_write_handler(p_param->value, p_param->length, &cfm, &event);
            break;

        case UDS_IDX_HEIGHT_VAL:
            if (UDS_UNKNOWN_USER  == s_uds_env.uds_init.user_index)
            {
                cfm.status = UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            
            uds_height_write_handler(p_param->value, p_param->length, &cfm, &event);
            break;

        case UDS_IDX_GENDER_VAL:
            if (UDS_UNKNOWN_USER  == s_uds_env.uds_init.user_index)
            {
                cfm.status = UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            
            uds_gender_write_handler(p_param->value, p_param->length, &cfm, &event);
            break;

        case UDS_IDX_DB_CHANGE_INCR_VAL:
            if (UDS_UNKNOWN_USER  == s_uds_env.uds_init.user_index)
            {
                cfm.status = UDS_ERROR_UD_ACCESS_NOT_PERMIT;
            }
            event.p_data   = p_param->value;
            event.length   = p_param->length;
            
            uds_db_change_incr_write_handler(p_param->value, p_param->length, &cfm, &event);
            break;

        case UDS_IDX_DB_CHANGE_INCR_NTF_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_ENABLE : \
                              UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_DISABLE);
            s_uds_env.db_change_incr_ntf_cfg[conn_idx] = cccd_value;
            break;

        case UDS_IDX_REGIST_USER_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              UDS_EVT_REGIST_USER_INDICATION_ENABLE : \
                              UDS_EVT_REGIST_USER_INDICATION_DISABLE);
            s_uds_env.regist_user_ind_cfg[conn_idx] = cccd_value;
            break;

        case UDS_IDX_CTRL_POINT_VAL:
            if (PRF_CLI_START_IND != s_uds_env.ctrl_point_ind_cfg[conn_idx])
            {
                cfm.status = UDS_ERROR_CCCD_INVALID;
                break;
            }
            else if (s_uds_env.ucp_in_progress)
            {
                cfm.status = UDS_ERROR_PROC_IN_PROGRESS;
            }
            else if (PRF_CLI_START_IND == s_uds_env.ctrl_point_ind_cfg[conn_idx])
            {
                if (UDS_CTRL_PT_OP_LIST_ALL_USERS == p_param->value[0] && \
                    (PRF_CLI_START_IND != s_uds_env.regist_user_ind_cfg[conn_idx]))
                {
                    cfm.status = UDS_ERROR_CCCD_INVALID;
                }
                else
                {
                    ucp_evt = true;
                    s_uds_env.ucp_in_progress = true;
                }
            }
            break;

        case UDS_IDX_CTRL_POINT_IND_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              UDS_EVT_CTRL_POINT_INDICATION_ENABLE : \
                              UDS_EVT_CTRL_POINT_INDICATION_DISABLE);
            s_uds_env.ctrl_point_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (ucp_evt)
    {
        uds_receive_ucp_handler(conn_idx, p_param->value, p_param->length);
    }
    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && UDS_EVT_INVALID != event.evt_type && s_uds_env.uds_init.evt_handler)
    {
        s_uds_env.uds_init.evt_handler(&event);
    }

}

/**
 *****************************************************************************************
 * @brief Handles Prepare write value callback function.
 *
 * @param[in]: conn_idx:      Connection index
 * @param[in]: p_prep_req:    Pointer to the handle of cccd attribute.
 *****************************************************************************************
 */
static void uds_gatts_prep_write_evt_handler(uint8_t conn_idx, const ble_gatts_evt_prep_write_t *p_prep_req)
{
    ble_gatts_prep_write_cfm_t cfm;

    cfm.handle = p_prep_req->handle;
    cfm.length = UDS_REGI_USER_VAL_LEN_MAX;
    cfm.status = BLE_SUCCESS;

    APP_LOG_DEBUG("Prepare write.");
    ble_gatts_prepare_write_cfm(conn_idx, &cfm);
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
static void uds_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t          tab_index   = 0;
    uds_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_uds_env.start_hdl,
                                        UDS_IDX_NB,
                                        (uint8_t *)&s_uds_env.uds_init.char_mask);

    event.evt_type = UDS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case UDS_IDX_DB_CHANGE_INCR_NTF_CFG:
            event.evt_type = ((PRF_CLI_START_NTF == cccd_value) ? \
                              UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_ENABLE : \
                              UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_DISABLE);
            s_uds_env.db_change_incr_ntf_cfg[conn_idx] = cccd_value;
            break;

        case UDS_IDX_CTRL_POINT_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              UDS_EVT_CTRL_POINT_INDICATION_ENABLE : \
                              UDS_EVT_CTRL_POINT_INDICATION_DISABLE);
            s_uds_env.ctrl_point_ind_cfg[conn_idx] = cccd_value;
            break;

        case UDS_IDX_REGIST_USER_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ? \
                              UDS_EVT_REGIST_USER_INDICATION_ENABLE : \
                              UDS_EVT_REGIST_USER_INDICATION_DISABLE);
            s_uds_env.regist_user_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (UDS_EVT_INVALID != event.evt_type && s_uds_env.uds_init.evt_handler)
    {
        s_uds_env.uds_init.evt_handler(&event);
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
static void uds_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    uds_evt_t event;

    event.evt_type = UDS_EVT_INVALID;
    event.conn_idx = conn_idx;

    uint8_t tab_index;
    tab_index = prf_find_idx_by_handle(p_ntf_ind->handle, 
                                       s_uds_env.start_hdl,
                                       UDS_IDX_NB,
                                       (uint8_t *)&s_uds_env.uds_init.char_mask);

    if (BLE_GAP_ERR_TIMEOUT == status)
    {
        ble_gap_disconnect(conn_idx);
    }

    if (s_uds_env.uds_init.evt_handler && SDK_SUCCESS == status)
    {
        if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
        {
            event.evt_type = UDS_EVT_DB_CHANGE_INCR_SEND_CPLT;
            s_uds_env.uds_init.evt_handler(&event);
        }
        else if (BLE_GATT_INDICATION == p_ntf_ind->type && (UDS_IDX_REGIST_USER_VAL == tab_index))
        {
            uds_indicate_user_char_val_chunk(conn_idx);
        }
        else if (BLE_GATT_INDICATION == p_ntf_ind->type && (UDS_IDX_CTRL_POINT_VAL == tab_index))
        {
            event.evt_type = UDS_EVT_CTRL_POINT_RSP_CPLT;
            s_uds_env.ucp_in_progress = false;
            s_uds_env.uds_init.evt_handler(&event);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Handle Age Characteristic write event.
 *
 * @param[in]  p_data: Pointer to the age characteristic value.
 * @param[in]  length: The length of data.
 * @param[in]  p_cfm:  Pointer of the write confirmation.
 *****************************************************************************************
 */
static sdk_err_t uds_age_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt)
{
    sdk_err_t error_code;

    if (length != 1 || p_data[0] > 120)
    {
        p_cfm->status = BLE_GATT_ERR_WRITE;
        error_code = BLE_GATT_ERR_WRITE;
    }
    else
    {
        p_evt->evt_type = UDS_EVT_AGE_SET_BY_PEER;
        
        uint8_t age                                  = p_data[0];
        uint8_t cur_user_id                          = s_uds_env.uds_init.user_index;
        
        s_wss_db_user_buf.user_id                    = cur_user_id;
        s_wss_db_user_buf.user_data[cur_user_id].age = age;
        wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

        error_code = BLE_SUCCESS;
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle Date of Birth Characteristic write event.
 *
 * @param[in]  p_data: Pointer to Date of Birth Characteristic value.
 * @param[in]  length: The length of data.
 * @param[in]  p_cfm:  Pointer of the write confirmation.
 *****************************************************************************************
 */
static sdk_err_t uds_date_of_birth_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt)
{
    sdk_err_t error_code;
    
    if (length != 4)
    {
        p_cfm->status = BLE_GATT_ERR_WRITE;
        error_code    = BLE_GATT_ERR_WRITE;
    }
    else
    {
        birth_date_t date_of_birth;
        date_of_birth.year  = BUILD_U16(p_data[0], p_data[1]);
        date_of_birth.month = p_data[2];
        date_of_birth.day   = p_data[3];
        
        if (date_of_birth.month > 12 || (date_of_birth.day > 31) ||\
            ((date_of_birth.year < 1582 || date_of_birth.year > 9999) && (date_of_birth.year != 0)))
        {
            p_cfm->status = BLE_GATT_ERR_WRITE;
            error_code    = BLE_GATT_ERR_WRITE;
        }
        else
        {
            if ((((date_of_birth.month == 1) || (date_of_birth.month == 3) || (date_of_birth.month == 5) ||
                (date_of_birth.month == 7) || (date_of_birth.month == 8) || (date_of_birth.month == 10) || (date_of_birth.month == 12))
                && (date_of_birth.day > 31)) ||
                (((date_of_birth.month == 4) || (date_of_birth.month == 6) || (date_of_birth.month == 9) || (date_of_birth.month == 11))
                && (date_of_birth.day > 30)))
            {
                p_cfm->status = BLE_GATT_ERR_WRITE;
                error_code    = BLE_GATT_ERR_WRITE;
            }
            else if (date_of_birth.month == 2)
            {
                if (((date_of_birth.year % 4 == 0) && (date_of_birth.year % 100 != 0)) || (date_of_birth.year % 400 ==0)) // leap year
                {
                    if (date_of_birth.day > 29)
                    {
                        p_cfm->status = BLE_GATT_ERR_WRITE;
                        error_code    = BLE_GATT_ERR_WRITE;
                    }
                    else
                    {
                        p_evt->evt_type = UDS_EVT_DATE_OF_BIRTH_SET_BY_PEER;
                        uint8_t cur_user_id                                    = s_uds_env.uds_init.user_index;

                        s_wss_db_user_buf.user_id                              = cur_user_id;
                        s_wss_db_user_buf.user_data[cur_user_id].date_of_birth = date_of_birth;
                        wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
                        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

                        error_code = BLE_SUCCESS;
                    }
                }
                else // average year
                {
                    if (date_of_birth.day > 28)
                    {
                        p_cfm->status = BLE_GATT_ERR_WRITE;
                        error_code    = BLE_GATT_ERR_WRITE;
                    }
                    else
                    {
                        p_evt->evt_type = UDS_EVT_DATE_OF_BIRTH_SET_BY_PEER;
                        uint8_t cur_user_id                                    = s_uds_env.uds_init.user_index;

                        s_wss_db_user_buf.user_id                              = cur_user_id;
                        s_wss_db_user_buf.user_data[cur_user_id].date_of_birth = date_of_birth;
                        wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
                        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

                        error_code = BLE_SUCCESS;
                    }
                }
            }
            else
            {
                p_evt->evt_type = UDS_EVT_DATE_OF_BIRTH_SET_BY_PEER;
                uint8_t cur_user_id                                    = s_uds_env.uds_init.user_index;

                s_wss_db_user_buf.user_id                              = cur_user_id;
                s_wss_db_user_buf.user_data[cur_user_id].date_of_birth = date_of_birth;
                wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
                wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
                
                error_code = BLE_SUCCESS;
            }
        }
    }
    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle First Name Characteristic write event.
 *
 * @param[in]  p_data:  Pointer to First Name Characteristic value.
 * @param[in]  length:  The length of data.
 * @param[in]  p_cfm:   Pointer of the write confirmation.
 *****************************************************************************************
 */
static sdk_err_t uds_first_name_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt)
{
    sdk_err_t error_code;
    uint8_t   cur_user_id                                = s_uds_env.uds_init.user_index;

    s_wss_db_user_buf.user_id                            = cur_user_id;
    s_wss_db_user_buf.user_data[cur_user_id].name_length = length;
    memcpy(&s_wss_db_user_buf.user_data[cur_user_id].first_name, p_data, length);
    wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
    wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
    
    error_code = BLE_SUCCESS;
    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle Height Characteristic write event.
 *
 * @param[in]  p_data: Pointer to Height Characteristic value.
 * @param[in]  length: The length of data.
 * @param[in]  p_cfm:  Pointer of the write confirmation.
 *****************************************************************************************
 */
static sdk_err_t uds_height_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt)
{
    sdk_err_t error_code;

    if (length != 2)
    {
        p_cfm->status = BLE_GATT_ERR_WRITE;
        error_code = BLE_GATT_ERR_WRITE;
    }
    else
    {
        uint16_t height = BUILD_U16(p_data[0], p_data[1]);
        if (height < 40 || 250 < height)
        {
            p_cfm->status = BLE_GATT_ERR_WRITE;
            error_code    = BLE_GATT_ERR_WRITE;
        }
        else
        {
            p_evt->evt_type = UDS_EVT_HEIGHT_SET_BY_PEER;
            uint8_t cur_user_id                             = s_uds_env.uds_init.user_index;

            s_wss_db_user_buf.user_id                       = cur_user_id;
            s_wss_db_user_buf.user_data[cur_user_id].height = height;
            wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_WRITE_REC;
            wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

            error_code = BLE_SUCCESS;
        }
    }
    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle Gender Characteristic write event.
 *
 * @param[in]  p_data: Pointer to Gender Characteristic value.
 * @param[in]  length: The length of data.
 * @param[in]  p_cfm:  Pointer of the write confirmation.
 *****************************************************************************************
 */
static sdk_err_t uds_gender_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt)
{
    sdk_err_t error_code;

    if (length != 1 || (p_data[0] > 2))
    {
        p_cfm->status = BLE_GATT_ERR_WRITE;
        error_code    = BLE_GATT_ERR_WRITE;
    }
    else
    {
        p_evt->evt_type = UDS_EVT_GENDER_SET_BY_PEER;
        uint8_t               gender                    = p_data[0];
        uint8_t               cur_user_id               = s_uds_env.uds_init.user_index;

        s_wss_db_user_buf.user_id                       = cur_user_id;    
        s_wss_db_user_buf.user_data[cur_user_id].gender = gender;
        wss_db_rec_cmd_type_t ucp_cmd                   = UCP_CMD_TYPE_WRITE_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
        
        error_code = BLE_SUCCESS;
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle Database Change Increment value write event.
 *
 * @param[in]  p_data: Pointer to Database Change Increment Characteristic value.
 * @param[in]  length: Length of Database Change Increment Characteristic value.
 * @param[in]  p_cfm:  Pointer of the write confirmation.
 *****************************************************************************************
 */
static sdk_err_t uds_db_change_incr_write_handler(const uint8_t *p_data, uint16_t length, ble_gatts_write_cfm_t *p_cfm, uds_evt_t *p_evt)
{
    sdk_err_t error_code;

    if (length != UDS_DB_CHANGE_INCR_VAL_LEN_MAX)
    {
        p_cfm->status = BLE_GATT_ERR_WRITE;
        error_code    = BLE_GATT_ERR_WRITE;
    }
    else
    {
        p_evt->evt_type = UDS_EVT_DB_CHANGE_INCR_SET_BY_PEER;
        uint32_t              db_change_incr_val                    = BUILD_U32(p_data[0], p_data[1], p_data[2], p_data[3]);
        uint8_t               cur_user_id                           = s_uds_env.uds_init.user_index;

        s_wss_db_user_buf.user_id                                   = cur_user_id;
        s_wss_db_user_buf.user_data[cur_user_id].db_change_incr_val = db_change_incr_val;
        wss_db_rec_cmd_type_t ucp_cmd                               = UCP_CMD_TYPE_WRITE_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

        error_code = BLE_SUCCESS;
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief User Control Point Register New User handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void uds_op_regist_new_user_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uds_evt_t   event;
    uint8_t     rsp[UDS_CTRL_PT_RSP_LEN_MAX];

    rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = UDS_CTRL_PT_OP_REGIST_NEW_USER;
    rsp[2] = UDS_CTRL_PT_RSP_SUCCESS;

    uint16_t consent_code = BUILD_U16(p_data[0], p_data[1]);

    if ((sizeof(uint16_t) != length) || (UDS_CONSENT_CODE_VAL_MAX < consent_code))
    {
        rsp[2] = UDS_CTRL_PT_RSP_INVALID_PARAM;
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN);
    }
    else
    {
        uint8_t new_user_id;      
        new_user_id = wss_db_user_id_get();

        if (UDS_UNKNOWN_USER == new_user_id)
        {
            new_user_id = 0x01;
        }

        uint8_t first_name[]                                         = {'d', 'e', 'f', 'a', 'u', 'l', 't'};

        s_wss_db_user_buf.user_id                                    = new_user_id;
        s_wss_db_user_buf.user_data[new_user_id].consent_code        = consent_code;
        s_wss_db_user_buf.user_data[new_user_id].db_change_incr_val  = UDS_DB_CHANGE_INCR_DEFAULT_VAL;
        s_wss_db_user_buf.user_data[new_user_id].age                 = 18;
        s_wss_db_user_buf.user_data[new_user_id].height              = 165;  // The unit is cm.
        s_wss_db_user_buf.user_data[new_user_id].gender              = 0x00;
        s_wss_db_user_buf.user_data[new_user_id].date_of_birth.year  = 2000;
        s_wss_db_user_buf.user_data[new_user_id].date_of_birth.month = 01;
        s_wss_db_user_buf.user_data[new_user_id].date_of_birth.day   = 01;
        s_wss_db_user_buf.user_data[new_user_id].name_length         = sizeof(first_name);
        memcpy(s_wss_db_user_buf.user_data[new_user_id].first_name, &first_name[0], sizeof(first_name));
        wss_db_rec_cmd_type_t ucp_cmd                                = UCP_CMD_TYPE_ADD_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

        event.conn_idx = conn_idx;
        event.evt_type = UDS_EVT_REGIST_NEW_USER;
        event.p_data   = &new_user_id;
        event.length   = length;
        s_uds_env.uds_init.evt_handler(&event);
        s_uds_env.uds_init.user_index = new_user_id;

        rsp[3] = new_user_id;
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN + 1);
    }
}

/**
 *****************************************************************************************
 * @brief User Control Point Consent handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void uds_op_consent_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uds_evt_t  event;
    uint8_t    cur_user_id = p_data[0];
    uint8_t    rsp[UDS_CTRL_PT_RSP_LEN_MIN];

    rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = UDS_CTRL_PT_OP_CONSENT;
    rsp[2] = UDS_CTRL_PT_RSP_SUCCESS;

    uint16_t consent_code = BUILD_U16(p_data[1], p_data[2]);

    if (UDS_CONSENT_TRY_NUM_MAX <= s_uds_env.consent_try_num)
    {
#if defined(PTS_AUTO_TEST)
        uint8_t cur_user_id                                   = s_uds_env.uds_init.user_index;

        s_wss_db_user_buf.user_id                             = cur_user_id;
        s_wss_db_user_buf.user_data[cur_user_id].consent_code = 0xFFFF;
        wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_CLEAR_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
#endif
        rsp[2] = UDS_CTRL_PT_RSP_FAILED;
    }
    else if (((sizeof(uint16_t)+sizeof(uint8_t)) != length) || \
             (UDS_UNKNOWN_USER < cur_user_id) || \
             (UDS_UNKNOWN_USER > cur_user_id && (cur_user_id >= WSS_DB_RECORDS_MAX)) || \
             (UDS_CONSENT_CODE_VAL_MAX <= consent_code))
    {
        rsp[2] = UDS_CTRL_PT_RSP_INVALID_PARAM;
    }
    else
    {
        if (consent_code != s_wss_db_user_buf.user_data[cur_user_id].consent_code)
        {
            rsp[2] = UDS_CTRL_PT_RSP_USER_NOT_AUTH;
            s_uds_env.consent_try_num++;

#if defined(PTS_AUTO_TEST)
            uint8_t cur_user_id                                   = s_uds_env.uds_init.user_index;

            s_wss_db_user_buf.user_id                             = cur_user_id;
            s_wss_db_user_buf.user_data[cur_user_id].consent_code = 0xFFFF;
            wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_CLEAR_REC;
            wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
#endif
        }
        else
        {
            event.conn_idx                = conn_idx;
            event.evt_type                = UDS_EVT_USER_GRANT_ACCESS;
            event.p_data                  = p_data;
            event.length                  = length;
            event.uds_chars_val.height    = s_wss_db_user_buf.user_data[cur_user_id].height;
            s_uds_env.uds_init.evt_handler(&event);
            
            s_uds_env.uds_init.user_index = cur_user_id;
            s_uds_env.consent_try_num     = 0;
        }
    }

    uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN);
}

/**
 *****************************************************************************************
 * @brief User Control Point Delete User Data handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void uds_op_del_user_data_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uds_evt_t  event;
    uint8_t    rsp[UDS_CTRL_PT_RSP_LEN_MAX];

    rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = UDS_CTRL_PT_OP_DEL_USER_DATA;
    rsp[2] = UDS_CTRL_PT_RSP_SUCCESS;

    if (UDS_UNKNOWN_USER == s_uds_env.uds_init.user_index)
    {
        rsp[2] = UDS_CTRL_PT_RSP_USER_NOT_AUTH;
    }
    else if (length > 0)
    {
        rsp[2] = UDS_CTRL_PT_RSP_INVALID_PARAM;
    }
    else
    {
        uint8_t cur_user_id                                   = s_uds_env.uds_init.user_index;

        s_wss_db_user_buf.user_id                             = cur_user_id;
        s_wss_db_user_buf.user_data[cur_user_id].consent_code = 0xFFFF;
        wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_DEL_REC;
        wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
        
        event.conn_idx                = conn_idx;
        event.evt_type                = UDS_EVT_DEL_USER_DATA;
        event.p_data                  = p_data;
        event.length                  = length;
        s_uds_env.uds_init.evt_handler(&event);
        s_uds_env.uds_init.user_index = UDS_UNKNOWN_USER;
    }

    uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN);
}

/**
 *****************************************************************************************
 * @brief User Control Point List All Users handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void uds_op_list_all_users_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uds_evt_t  event;
    uint8_t    rsp[UDS_CTRL_PT_RSP_LEN_MAX];

    rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = UDS_CTRL_PT_OP_LIST_ALL_USERS;
    rsp[2] = UDS_CTRL_PT_RSP_SUCCESS;

    uint16_t user_num = wss_db_records_num_get();
    
    if (length > 0)
    {
        rsp[2] = UDS_CTRL_PT_RSP_INVALID_PARAM;
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN);
    }
    else if (UDS_UNKNOWN_USER > user_num)
    {
        event.conn_idx = conn_idx;
        event.evt_type = UDS_EVT_CTRL_POINT_SET_BY_PEER;
        event.p_data   = p_data;
        event.length   = length;
        s_uds_env.uds_init.evt_handler(&event);

        if (PRF_CLI_START_IND == s_uds_env.regist_user_ind_cfg[conn_idx])
        {
            uds_regist_user_value_encode(0);
            uds_indicate_user_char_val_chunk(conn_idx);
        }
        else
        {
            rsp[3] = user_num;
            uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN + 1);
        }
    }
    else if (0 == user_num)
    {
        rsp[3] = user_num;
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN + 1);
    }
}

/**
 *****************************************************************************************
 * @brief User Control Point Delete User(s) handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void uds_op_del_users_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uds_evt_t  event;
    uint8_t    cur_user_id = p_data[0];
    uint8_t    rsp[UDS_CTRL_PT_RSP_LEN_MIN+1];

    rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = UDS_CTRL_PT_OP_DEL_USERS;
    rsp[2] = UDS_CTRL_PT_RSP_SUCCESS;
    rsp[3] = cur_user_id;

    if (sizeof(uint8_t) != length || UDS_UNKNOWN_USER < cur_user_id || \
        (WSS_DB_RECORDS_MAX <= cur_user_id && cur_user_id < UDS_UNKNOWN_USER))
    {
        rsp[2] = UDS_CTRL_PT_RSP_INVALID_PARAM;
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN);
    }
    else
    {
        if (UDS_UNKNOWN_USER == cur_user_id)
        {
            wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_CLEAR_REC;
            wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
            {
                s_uds_env.uds_init.user_index = UDS_UNKNOWN_USER;
                
                for (uint8_t i=0; i<WSS_DB_RECORDS_MAX; i++)
                {
                    s_wss_db_user_buf.user_data[i].consent_code = 0xFFFF;
                }
            }
        }
        else
        {
            if (wss_db_user_id_valid_confirm(cur_user_id))
            {
                s_wss_db_user_buf.user_id                             = cur_user_id;
                s_wss_db_user_buf.user_data[cur_user_id].consent_code = 0xFFFF;
                wss_db_rec_cmd_type_t ucp_cmd = UCP_CMD_TYPE_DEL_REC;
                if (!wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd))
                {
                    rsp[2] = UDS_CTRL_PT_RSP_FAILED;
                }
            }
            else
            {
                rsp[2] = UDS_CTRL_PT_RSP_INVALID_PARAM;
            }
        }
        
        event.conn_idx = conn_idx;
        event.evt_type = UDS_EVT_DEL_USERS;
        event.p_data   = p_data;
        event.length   = length;
        
        if (s_uds_env.uds_init.user_index == cur_user_id)
        {
            s_uds_env.uds_init.user_index = UDS_UNKNOWN_USER;
        }

        s_uds_env.uds_init.evt_handler(&event);
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN+1);
    }
}

/**
 *****************************************************************************************
 * @brief User Control Point receive handler.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *****************************************************************************************
 */
static void uds_receive_ucp_handler(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    uint8_t   rsp[UDS_CTRL_PT_RSP_LEN_MAX];

    switch(p_data[0])
    {
        case UDS_CTRL_PT_OP_REGIST_NEW_USER:
            uds_op_regist_new_user_handler(conn_idx, &p_data[1], length - 1);
            break;

        case UDS_CTRL_PT_OP_CONSENT:
            uds_op_consent_handler(conn_idx, &p_data[1], length - 1);
            break;

        case UDS_CTRL_PT_OP_DEL_USER_DATA:
            uds_op_del_user_data_handler(conn_idx, &p_data[1], length - 1);
            break;

        case UDS_CTRL_PT_OP_LIST_ALL_USERS:
            uds_op_list_all_users_handler(conn_idx, &p_data[1], length - 1);
            break;
        
        case UDS_CTRL_PT_OP_DEL_USERS:
            uds_op_del_users_handler(conn_idx, &p_data[1], length - 1);
            break;

        default:
            rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = p_data[0];
            rsp[2] = UDS_CTRL_PT_RSP_NOT_SUP;
            uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN);
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Indicate the Registered User Characteristic value.
 *
 * @param[in] conn_idx:  Connection index.
 *****************************************************************************************
 */
static sdk_err_t uds_indicate_user_char_val_chunk(uint8_t conn_idx)
{
    uint8_t          chunk_len;
    ble_gatts_noti_ind_t uds_ind;
    sdk_err_t        error_code;

    chunk_len = s_regi_user_char_val_stream.p_segm_length[s_regi_user_char_val_stream.segm_num - s_regi_user_char_val_stream.segm_offset];
    
    if (0 == s_regi_user_char_val_stream.segm_offset)
    {
        s_regi_user_char_val_stream.p_data        = NULL;
        s_regi_user_char_val_stream.offset        = 0;
        s_regi_user_char_val_stream.length        = 0;
        s_regi_user_char_val_stream.p_segm_length = NULL;
        s_regi_user_char_val_stream.segm_num      = 0;
        s_regi_user_char_val_stream.segm_offset   = 0;

        uds_evt_t event;
        event.evt_type = UDS_EVT_REGIST_USER_RSP_CPLT;
        s_uds_env.uds_init.evt_handler(&event);

        // After indicating all users, then indicate the number of users.
        uint8_t   rsp[UDS_CTRL_PT_RSP_LEN_MIN+1];
        uint16_t  user_num = wss_db_records_num_get();

        rsp[0] = UDS_CTRL_PT_OP_RSP_CODE;
        rsp[1] = UDS_CTRL_PT_OP_LIST_ALL_USERS;
        rsp[2] = UDS_CTRL_PT_RSP_SUCCESS;
        rsp[3] = user_num;
        uds_ctrl_pt_rsp_send(conn_idx, rsp, UDS_CTRL_PT_RSP_LEN_MIN+1);

        return SDK_SUCCESS;
    }

    uds_ind.type   = BLE_GATT_INDICATION;
    uds_ind.handle = prf_find_handle_by_idx(UDS_IDX_REGIST_USER_VAL,
                                            s_uds_env.start_hdl,
                                            (uint8_t *)&s_uds_env.uds_init.char_mask);
    uds_ind.length = chunk_len;
    uds_ind.value  = (uint8_t *)s_regi_user_char_val_stream.p_data + s_regi_user_char_val_stream.offset;
      
    error_code     = ble_gatts_noti_ind(conn_idx, &uds_ind);

    if(SDK_SUCCESS == error_code)
    {
        s_regi_user_char_val_stream.offset += chunk_len;
        s_regi_user_char_val_stream.segm_offset--;
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Encode the Registered User Characteristic value.
 *
 * @param[in] conn_idx:  Connection index.
 *****************************************************************************************
 */
static void uds_regist_user_value_encode(uint8_t conn_idx)
{
    memset(&s_regi_user_char_val_stream, 0, sizeof(s_regi_user_char_val_stream));

    s_uds_env.segm_head_roll_num = 0;

    memset(&local_buf, 0, sizeof(local_buf));
    memset(&local_segm_length, 0, sizeof(local_segm_length));

    uint16_t  length         = 0;
    uint16_t  chunk_len      = 0;
    uint16_t  segm_total_num = 0;

    for (uint8_t i = 0; i < WSS_DB_RECORDS_MAX; i++)
    {
        if (wss_db_user_id_valid_confirm(i))
        {
            wss_user_data_t user_data;
            memcpy(&user_data, &(s_wss_db_user_buf.user_data[i]), sizeof(wss_user_data_t));
            
            uint16_t        segm_num_per_user = 0;
            uint16_t        name_offset_per_user = 0;

            do
            {
                chunk_len = user_data.name_length - name_offset_per_user;
                
                if (UDS_REGI_USER_DATA_LEN_MAX - 2 >= chunk_len && 0 == segm_num_per_user)
                {
                    uint8_t segm_header = 0;

                    uint8_t regi_user_data_flags = 0;
                    uint8_t regi_user_index;

                    segm_header |= (s_uds_env.segm_head_roll_num << 2);
                    segm_header |= UDS_ONLY_REGI_USER_SEGM;  // the only packet.
                    
                    if (s_uds_env.uds_init.uds_regi_user_data_flag.regi_user_name_present)
                    {
                        regi_user_data_flags |= UDS_REGI_USER_NAME_PRESENT;
                    }

                    if (s_uds_env.uds_init.uds_regi_user_data_flag.user_name_truncated)
                    {
                        regi_user_data_flags |= UDS_USER_NAME_TRUNCATED;
                    }

                    regi_user_index = i;

                    local_buf[length++] = segm_header;
                    local_buf[length++] = regi_user_data_flags;
                    local_buf[length++] = regi_user_index;

                    memcpy(&local_buf[length], user_data.first_name, user_data.name_length);
                    length += user_data.name_length;

                    local_segm_length[segm_total_num++] = user_data.name_length + 3;
                    s_regi_user_char_val_stream.length += (user_data.name_length + 3);
                    name_offset_per_user += user_data.name_length;

                }
                else if (UDS_REGI_USER_DATA_LEN_MAX >= chunk_len && 0 != segm_num_per_user)
                {
                    uint8_t segm_header = 0;
                    segm_header |= (s_uds_env.segm_head_roll_num << 2);
                    segm_header |= UDS_LAST_REGI_USER_SEGM;  // the last packet.

                    local_buf[length++] = segm_header;

                    memcpy(&local_buf[length], user_data.first_name + name_offset_per_user, chunk_len);
                    length += chunk_len;

                    local_segm_length[segm_total_num++] = chunk_len + 1;
                    s_regi_user_char_val_stream.length += (chunk_len + 1);
                    name_offset_per_user += chunk_len;       
                }
                else if (UDS_REGI_USER_DATA_LEN_MAX < chunk_len)
                {
                    if (0 == segm_num_per_user)
                    {
                        uint8_t segm_header = 0;
                        uint8_t regi_user_data_flags = 0;
                        uint8_t regi_user_index;

                        segm_header |= (s_uds_env.segm_head_roll_num << 2);
                        segm_header |= UDS_FIRST_REGI_USER_SEGM;  // the first packet.

                        if (s_uds_env.uds_init.uds_regi_user_data_flag.regi_user_name_present)
                        {
                            regi_user_data_flags |= UDS_REGI_USER_NAME_PRESENT;
                        }

                        if (s_uds_env.uds_init.uds_regi_user_data_flag.user_name_truncated)
                        {
                            regi_user_data_flags |= UDS_USER_NAME_TRUNCATED;
                        }

                        regi_user_index = i;

                        local_buf[length++] = segm_header;
                        local_buf[length++] = regi_user_data_flags;
                        local_buf[length++] = regi_user_index;

                        memcpy(&local_buf[length], user_data.first_name, UDS_REGI_USER_DATA_LEN_MAX - 2);
                        length += UDS_REGI_USER_DATA_LEN_MAX - 2;

                        local_segm_length[segm_total_num++] = UDS_REGI_USER_VAL_LEN_MAX;
                        s_regi_user_char_val_stream.length += UDS_REGI_USER_VAL_LEN_MAX;
                        name_offset_per_user += UDS_REGI_USER_DATA_LEN_MAX - 2;
                    }
                    else 
                    {
                        uint8_t segm_header = 0;
                        segm_header |= (s_uds_env.segm_head_roll_num << 2);
                        segm_header |= UDS_MIDDLE_REGI_USER_SEGM;  // the middle packet.

                        local_buf[length++] = segm_header;
                        memcpy(&local_buf[length], user_data.first_name  + name_offset_per_user, UDS_REGI_USER_DATA_LEN_MAX - 2);
                        length += UDS_REGI_USER_DATA_LEN_MAX - 2;

                        local_segm_length[segm_total_num++] = UDS_REGI_USER_VAL_LEN_MAX;
                        s_regi_user_char_val_stream.length += UDS_REGI_USER_VAL_LEN_MAX;
                        name_offset_per_user += UDS_REGI_USER_DATA_LEN_MAX - 2;                         
                    }
                }
                segm_num_per_user++;
                if (63 == s_uds_env.segm_head_roll_num)
                {
                    s_uds_env.segm_head_roll_num = 0;
                }
                else
                {
                    s_uds_env.segm_head_roll_num++;
                }
            }
            while (name_offset_per_user != user_data.name_length);        
        }
    }

    s_regi_user_char_val_stream.segm_offset   = segm_total_num;
    s_regi_user_char_val_stream.segm_num      = segm_total_num;
    s_regi_user_char_val_stream.p_segm_length = local_segm_length;
    s_regi_user_char_val_stream.p_data        = local_buf;
    s_regi_user_char_val_stream.offset        = 0;
}

static void uds_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            uds_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            uds_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;
        
        case BLE_GATTS_EVT_PREP_WRITE_REQUEST:
            uds_gatts_prep_write_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.prep_wr_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            uds_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            uds_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            uds_disconnect_evt_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void uds_set_cur_user_index(uint8_t conn_idx, uint8_t user_index)
{
    s_uds_env.uds_init.user_index = user_index;
}

uint8_t uds_get_cur_user_index(uint8_t conn_idx)
{
    return s_uds_env.uds_init.user_index;
}

sdk_err_t uds_db_change_incr_val_send(uint8_t conn_idx, uint8_t user_index)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    uint16_t         length;
    ble_gatts_noti_ind_t uds_ntf;

    if (UDS_UNKNOWN_USER != user_index)
    {
        length = sizeof(s_wss_db_user_buf.user_data[user_index].db_change_incr_val);
        if (PRF_CLI_START_NTF == s_uds_env.db_change_incr_ntf_cfg[conn_idx])
        {
            uds_ntf.type   = BLE_GATT_NOTIFICATION;
            uds_ntf.handle = prf_find_handle_by_idx(UDS_IDX_DB_CHANGE_INCR_VAL,
                                                    s_uds_env.start_hdl,
                                                    (uint8_t *)&s_uds_env.uds_init.char_mask);
            uds_ntf.length = length;
            uds_ntf.value  = (uint8_t *)&(s_wss_db_user_buf.user_data[user_index].db_change_incr_val);
            error_code     = ble_gatts_noti_ind(conn_idx, &uds_ntf);
        }
    }
    return error_code;
}

sdk_err_t uds_ctrl_pt_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;
    ble_gatts_noti_ind_t ctrl_pt_rsp;

    if (PRF_CLI_START_IND == s_uds_env.ctrl_point_ind_cfg[conn_idx])
    {
        ctrl_pt_rsp.type   = BLE_GATT_INDICATION;
        ctrl_pt_rsp.handle = prf_find_handle_by_idx(UDS_IDX_CTRL_POINT_VAL,
                                                    s_uds_env.start_hdl,
                                                    (uint8_t *)&s_uds_env.uds_init.char_mask);
        ctrl_pt_rsp.length = length;
        ctrl_pt_rsp.value  = p_data;
        error_code         = ble_gatts_noti_ind(conn_idx, &ctrl_pt_rsp);
    }

    return error_code;
}

sdk_err_t uds_service_init(uds_init_t *p_uds_init)
{
    if (NULL == p_uds_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_uds_env, 0, sizeof(s_uds_env));
    memcpy(&s_uds_env.uds_init, p_uds_init, sizeof(uds_init_t));

    s_uds_env.start_hdl  = PRF_INVALID_HANDLE;

    s_uds_env.uds_gatts_db.shdl                 = &s_uds_env.start_hdl;
    s_uds_env.uds_gatts_db.uuid                 = s_uds_svc_uuid;
    s_uds_env.uds_gatts_db.attr_tab_cfg         = (uint8_t *)&(s_uds_env.uds_init.char_mask);
    s_uds_env.uds_gatts_db.max_nb_attr          = UDS_IDX_NB;
    s_uds_env.uds_gatts_db.srvc_perm            = 0; 
    s_uds_env.uds_gatts_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_uds_env.uds_gatts_db.attr_tab.attr_tab_16 = uds_attr_tab;

    return ble_gatts_prf_add(&s_uds_env.uds_gatts_db, uds_ble_evt_handler);
}

#if defined(PTS_AUTO_TEST)
void uds_regist_new_user(uint8_t conn_idx)
{
    uint8_t new_user_id;      
    new_user_id = wss_db_user_id_get();

    if (UDS_UNKNOWN_USER == new_user_id)
    {
        new_user_id = 0x01;
    }

    uint8_t first_name[]                                         = {'d', 'e', 'f', 'a', 'u', 'l', 't'};

    s_wss_db_user_buf.user_id                                    = new_user_id;
    s_wss_db_user_buf.user_data[new_user_id].consent_code        = 0x0001;
    s_wss_db_user_buf.user_data[new_user_id].db_change_incr_val  = UDS_DB_CHANGE_INCR_DEFAULT_VAL;
    s_wss_db_user_buf.user_data[new_user_id].age                 = 18;
    s_wss_db_user_buf.user_data[new_user_id].height              = 165;  // The unit is cm.
    s_wss_db_user_buf.user_data[new_user_id].gender              = 0x00;
    s_wss_db_user_buf.user_data[new_user_id].date_of_birth.year  = 2000;
    s_wss_db_user_buf.user_data[new_user_id].date_of_birth.month = 01;
    s_wss_db_user_buf.user_data[new_user_id].date_of_birth.day   = 01;
    s_wss_db_user_buf.user_data[new_user_id].name_length         = sizeof(first_name);
    memcpy(s_wss_db_user_buf.user_data[new_user_id].first_name, &first_name[0], sizeof(first_name));
    wss_db_rec_cmd_type_t ucp_cmd                                = UCP_CMD_TYPE_ADD_REC;
    wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);

    s_uds_env.uds_init.user_index = new_user_id;
}

void uds_del_users(uint8_t conn_idx)
{
    s_wss_db_user_buf.user_id                             = 0x00;
    wss_db_rec_cmd_type_t ucp_cmd                         = UCP_CMD_TYPE_DEL_REC;
    wss_db_ucp_cmd_queue_elem_push(&s_wss_db_rec_cmd_queue, ucp_cmd);
}
#endif


