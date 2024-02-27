/**
 ****************************************************************************************
 *
 * @file uds.h
 *
 * @brief User Data Service API.
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_UDS User Data Service (UDS)
 * @{
 * @brief User Data Service module.
 *
 * @details The User Data Service (UDS) exposes user-related data in a sports and fitness, home, or healthcare
 *          environment. This allows remote access and update of user data by a client as well as the
 *          synchronization of user data between a server and a client.
 *
 *          After \ref uds_init_t variable is intialized, the application must call \ref uds_service_init()
 *          to add User Data Service and UDS Characteristic, Database Change Increment, User Index, User
 *          Control Point and Registered User characteristics to the BLE Stack database according to 
  *         \ref uds_init_t.char_mask.
 */

#ifndef __UDS_H__
#define __UDS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup UDS_MACRO Defines
 * @{
 */
#define UDS_CONNECTION_MAX                10                           /**< Maximum number of UDS connections. */
#define UDS_AGE_VAL_LEN_MAX               2                            /**< Maximum length of Age Characteristic value. */
#define UDS_DATE_OF_BIRTH_VAL_LEN_MAX     4                            /**< Maximum length of Date of Birth Characteristic value. */
#define UDS_FIRST_NAME_VAL_LEN_MAX        50                           /**< Maximum length of First Name Characteristic value. */
#define UDS_HEIGHT_VAL_LEN_MAX            2                            /**< Maximum length of Height Characteristic value. */
#define UDS_GENDER_VAL_LEN_MAX            2                            /**< Maximum length of Gender Characteristic value. */
#define UDS_DB_CHANGE_INCR_VAL_LEN_MAX    4                            /**< Maximum length of Database Change Increment value. */
#define UDS_USER_INDEX_VAL_LEN_MAX        1                            /**< Maximum length of User Index value. */
#define UDS_CTRL_PT_RSP_LEN_MIN           3                            /**< Mimimum length of User Control Point response value. */
#define UDS_CTRL_PT_RSP_LEN_MAX           20                           /**< Maximum length of User Control Point response value. */
#define UDS_CTRL_PT_VAL_LEN_MAX           19                           /**< Maximum length of User Control Point value. */
#define UDS_REGI_USER_DATA_LEN_MAX        19                           /**< Maximum length of Registered User Data value. */
#define UDS_REGI_USER_VAL_LEN_MAX         20                           /**< Maximum length of Registered User Characteristic value. */

#define UDS_ERROR_UD_ACCESS_NOT_PERMIT    0x80                         /**< Error code: The user data access is not permitted. */
#define UDS_ERROR_PROC_IN_PROGRESS        0xFE                         /**< Error code: A previously triggered User Control Point operation is still in progress. */
#define UDS_ERROR_CCCD_INVALID            0xFD                         /**< Error code: The Client Characteristic Configuration descriptor is not configured. */

#define UDS_DB_CHANGE_INCR_DEFAULT_VAL    0x00                         /**< Database Change Increment default value. */
#define UDS_UNKNOWN_USER                  0xFF                         /**< Unknown User. */
#define UDS_CONSENT_CODE_VAL_MAX          0x270F                       /**< Maximum value of Consent Code. */
#define UDS_CONSENT_TRY_NUM_MAX           0x04                         /**< Maximum number of consent tries. */

#define UDS_MIDDLE_REGI_USER_SEGM         0x00                         /**< The middle segment of Registered User Characterristic value. */
#define UDS_FIRST_REGI_USER_SEGM          0x01                         /**< The first segment of Registered User Characterristic value. */
#define UDS_LAST_REGI_USER_SEGM           0x02                         /**< The last segment of Registered User Characterristic value. */
#define UDS_ONLY_REGI_USER_SEGM           0x03                         /**< The only segment of Registered User Characterristic value. */

#define UDS_MEAS_FLAG_DEFAULT             0x3E                          /**< The default flag of UDS MEAS */

/**
 * @defgroup UDS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref uds_init_t.char_mask.
 */
#define UDS_CHAR_MANDATORY              0x000001FF     /**< Bit mask for mandatory characteristic in UDS. */
#define UDS_CHAR_AGE_SUP                0x00000600     /**< Bit mask for Age characteristic that is optional. */
#define UDS_CHAR_DATE_OF_BIRTH_SUP      0x00001800     /**< Bit mask for date of birth characteristic that is optional. */
#define UDS_CHAR_FIRST_NAME_SUP         0x00006000     /**< Bit mask for first name characteristic that is optional. */
#define UDS_CHAR_HEIGHT_SUP             0x00018000     /**< Bit mask for height characteristic that is optional. */
#define UDS_CHAR_GENDER_SUP             0x00060000     /**< Bit mask for gender characteristic that is optional. */
#define UDS_CHAR_REGIST_USER_SUP        0x00380000     /**< Bit mask for Registered User characteristic that is optional. */
#define UDS_CHAR_FULL                   0x003FFFFF     /**< Bit mask of the full characteristic. */
/** @} */
/** @} */

/**
 * @defgroup UDS_ENUM Enumerations
 * @{
 */
/**@brief User Data Service Control Point Operation Code.*/
typedef enum
{
    UDS_CTRL_PT_OP_RESERVED,         /**< Reserved for future use. */
    UDS_CTRL_PT_OP_REGIST_NEW_USER,  /**< Register New User Operation Code.*/
    UDS_CTRL_PT_OP_CONSENT,          /**< Consent Operation Code.*/
    UDS_CTRL_PT_OP_DEL_USER_DATA,    /**< Delete User Data Operation Code.*/
    UDS_CTRL_PT_OP_LIST_ALL_USERS,   /**< List All Users Operation Code.*/
    UDS_CTRL_PT_OP_DEL_USERS,        /**< Delete User(s) Operation code. */
    UDS_CTRL_PT_OP_RSP_CODE = 0x20,  /**< Response code. */
} uds_ctrl_pt_op_code_t;

/**@brief User Data Service Control Point Response value.*/
typedef enum
{
    UDS_CTRL_PT_RSP_RESERVED,        /**< Reserved value. */
    UDS_CTRL_PT_RSP_SUCCESS,         /**< Operation Success. */
    UDS_CTRL_PT_RSP_NOT_SUP,         /**< Operation Code Not Supported. */
    UDS_CTRL_PT_RSP_INVALID_PARAM,   /**< Invalid Parameter. */
    UDS_CTRL_PT_RSP_FAILED,          /**< Operation Failed. */
    UDS_CTRL_PT_RSP_USER_NOT_AUTH,   /**< User Not Authorized. */
} uds_ctrl_pt_rsp_t;

/**@brief User Data Service event type. */
typedef enum
{
    UDS_EVT_INVALID = 0x00,                       /**< Invalid event. */
    UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_ENABLE,   /**< Database Change Increment Notification is enabled. */
    UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_DISABLE,  /**< Database Change Increment Notification is disabled. */
    UDS_EVT_CTRL_POINT_INDICATION_ENABLE,         /**< Indicate that User Control Point indication has been enabled. */
    UDS_EVT_CTRL_POINT_INDICATION_DISABLE,        /**< Indicate that User Control Point indication has been disabled. */
    UDS_EVT_REGIST_USER_INDICATION_ENABLE,        /**< Indicate that Registered User indication has been enabled. */
    UDS_EVT_REGIST_USER_INDICATION_DISABLE,       /**< Indicate that Registered User indication has been disabled. */
    UDS_EVT_DB_CHANGE_INCR_SEND_CPLT,             /**< Indicate that Database Change Increment has been notified. */
    UDS_EVT_CTRL_POINT_RSP_CPLT,                  /**< Indicate that User Control Point response has been indicated. */
    UDS_EVT_REGIST_USER_RSP_CPLT,                 /**< Indicate that Registered User response has been indicated. */
    UDS_EVT_AGE_SET_BY_PEER,                      /**< Age has been set by peer. */
    UDS_EVT_DATE_OF_BIRTH_SET_BY_PEER,            /**< Date of Birth has been set by peer. */
    UDS_EVT_FIRST_NAME_SET_BY_PEER,               /**< First Name has been set by peer. */
    UDS_EVT_HEIGHT_SET_BY_PEER,                   /**< Height has been set by peer. */
    UDS_EVT_GENDER_SET_BY_PEER,                   /**< Gender has been set by peer. */
    UDS_EVT_DB_CHANGE_INCR_SET_BY_PEER,           /**< Database Change Increment has been set by peer. */
    UDS_EVT_CTRL_POINT_SET_BY_PEER,               /**< User Control Point has been set by peer. */
    UDS_EVT_USER_GRANT_ACCESS,                    /**< Indicate that User is waiting to be granted access. */
    UDS_EVT_REGIST_NEW_USER,                      /**< Indicate that User is waiting to be granted access. */
    UDS_EVT_DEL_USER_DATA,
    UDS_EVT_DEL_USERS,    
} uds_evt_type_t;

/**@brief UDS Characteristics Flags. */
enum uds_chars_flag_bits
{
    UDS_CHARS_WEIGHT_PRESENT     = 0x01,     /**< Flag bit for Weight Present. */
    UDS_CHARS_AGE_PRESENT        = 0x02,     /**< Flag bit for Age Present. */
    UDS_CHARS_BIRTH_DATE_PRESENT = 0x04,     /**< Flag bit for Date of Birth Present. */
    UDS_CHARS_FIRST_NAME_PRESENT = 0x08,     /**< Flag bit for First Name Present. */
    UDS_CHARS_HEIGHT_PRESENT     = 0x10,     /**< Flag bit for Height Present. */
    UDS_CHARS_GENDER_PRESENT     = 0X20,     /**< Flag bit for Gender Present. */
};

/**@brief Registered User Data Flags. */
enum uds_regi_user_data_flag_bits
{
    UDS_REGI_USER_NAME_PRESENT = 0x01,       /**< Flag bit for Registered User Name Present. */
    UDS_USER_NAME_TRUNCATED    = 0x02,       /**< Flag bit for User Name is truncated. */
};
/** @} */

/**
 * @defgroup UDS_STRUCT Structures
 * @{
 */
/**@brief UDS Characteristic - birthdate's structure. */
typedef struct
{
    uint16_t year;     /**< year time element of birth date. */
    uint8_t  month;    /**< month time element of birth date. */
    uint8_t  day;      /**< day time element of birth date. */
} birth_date_t;

/**@brief UDS Characteristics Flags structure. */
typedef struct
{
    uint8_t weight_present     :1;    /**< Weight Present. */
    uint8_t age_present        :1;    /**< Age Present. */
    uint8_t birth_date_present :1;    /**< Date of Birth Present. */  
    uint8_t first_name_present :1;    /**< First Name Present. */
    uint8_t height_present     :1;    /**< Height Present. */
    uint8_t gender_present     :1;    /**< Gender Present. */
} uds_chars_flag_t;

/**@brief UDS Characteristics value structure. */
typedef struct
{
    uint16_t     weight;              /**< User's weight. */
	uint8_t      age;                 /**< User's age. */
	birth_date_t date_of_birth;       /**< User's birth date. */
	uint16_t     height;              /**< User's height. */
	uint8_t      gender;              /**< User's gender. */
    uint8_t     *p_first_name;        /**< User's first name. */
    uint16_t     name_length;         /**< Length of User's first name. */
} uds_chars_val_t;

/**@brief Registered User Data Flags structure. */
typedef struct
{
    uint8_t regi_user_name_present :1;    /**< Registered User Name Present. */
    uint8_t user_name_truncated    :1;    /**< User Name is truncated. */
    uint8_t reserved               :6;    /**< Reserved for Future Use. */ 
} uds_regi_user_data_flag_t;

/**@brief User Data Service event. */
typedef struct
{
    uint8_t                  conn_idx;            /**< The index of the connection. */
    uds_evt_type_t           evt_type;            /**< The UDS event type. */
    const uint8_t            *p_data;             /**< Pointer to event data. */
    uint16_t                 length;              /**< Length of event data. */
    uint16_t                 consent_code;        /**< Consent Code data. */
    uds_chars_val_t          uds_chars_val;       /**< UDS Characteristics value set by peer. */
    uint32_t                 db_change_incr_val;  /**< Database Change Increment value set by peer. */
} uds_evt_t;
/** @} */

/**
 * @defgroup UDS_TYPEDEF Typedefs
 * @{
 */
/**@brief User Data Service event handler type. */
typedef void (*uds_evt_handler_t)(uds_evt_t *p_evt);
/** @} */

/**
 * @defgroup UDS_STRUCT Structures
 * @{
 */
/**@brief UDS Registered User Data stream. */
typedef struct 
{
    uint8_t          *p_data;         /**< Pointer to Registered User Data stream. */
    uint16_t         length;          /**< Length of Registered User Data stream. */
    uint16_t         offset;          /**< Offset of Registered User Data stream. */
    uint16_t         *p_segm_length;  /**< Pointer to each segmentation length of Registered User Data stream. */
    uint16_t         segm_num;        /**< Registered User Data segmentations' number. */
    uint16_t         segm_offset;     /**< Offset of Registered User Data segmentations' number. */
} uds_regi_user_data_stream_t;

/**@brief User Data Service init structure. This contains all option and data needed for initialization of the service. */
typedef struct
{
    uds_evt_handler_t         evt_handler;              /**< User Data Service event handler. */
    uint32_t                  char_mask;                /**< Initialize the mask of supported characteristics, and configured with \ref UDS_CHAR_MASK. */
    uint8_t                   user_index;               /**< Initialize the user index. */
    uint8_t                   db_change_incr_val;       /**< Initialize the Database Change Increment value. */
    uds_regi_user_data_flag_t uds_regi_user_data_flag;  /**< Initialize the Registered User Data Flags structure. */
    uds_chars_flag_t          uds_chars_flag;           /**< Initialize the UDS Characteristics Flags structure. */
} uds_init_t;
/** @} */

/**
 * @defgroup UDS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a User Data Service instance and add in the DB.
 *
 * @param[in] p_uds_init: Pointer to UDS Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t uds_service_init(uds_init_t *p_uds_init);

/**
 *****************************************************************************************
 * @brief Send Registered User Characteristic value.
 *
 * @param[in] conn_idx:   Connnection index.
 *
 * @return Result of sending Registered User Characteristic value.
 *****************************************************************************************
 */
sdk_err_t uds_regi_user_val_send(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Send Database Change Increment Characteristic value if UDS Characteristics value has been changed.
 *
 * @param[in] conn_idx:   Connnection index.
 * @param[in] user_idx:   Current user index.
 *
 * @return Result of sending Database Change Increment Characteristic value.
 *****************************************************************************************
 */
sdk_err_t uds_db_change_incr_val_send(uint8_t conn_idx, uint8_t user_idx);

/**
 *****************************************************************************************
 * @brief Send User Control Point response if it's indication has been enabled.
 *
 * @param[in] conn_idx:   Connnection index.
 * @param[in] p_data:     Pointer to data.
 * @param[in] length:     Length of data.
 *
 * @return Result of indicate value.
 *****************************************************************************************
 */
sdk_err_t uds_ctrl_pt_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Get current user index.
 *
 * @param[in] conn_idx:   Connnection index.
 *
 * @return Result of current user index.
 *****************************************************************************************
 */
uint8_t uds_get_cur_user_index(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Set current user index.
 *
 * @param[in] conn_idx:   Connnection index.
 * @param[in] user_index: current user index.
 *****************************************************************************************
 */
void uds_set_cur_user_index(uint8_t conn_idx, uint8_t user_index);

#if defined(PTS_AUTO_TEST)
void uds_regist_new_user(uint8_t conn_idx);
#endif

/** @} */

#endif
/** @} */
/** @} */

