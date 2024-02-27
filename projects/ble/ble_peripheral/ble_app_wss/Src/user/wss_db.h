/**
 ****************************************************************************************
 *
 * @file wss_db.h
 *
 * @brief Weight Scale Service Database API.
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
 * @defgroup BLE_SDK_WSS_DB Weight Scale Service Database (WSS_DB)
 * @{
 * @brief Weight Scale Service Database module.
 *
 * @details This module implements at database of stored user data, application can add, delete,
 *          get and clear user data records in database after database
 *          environment variables are initialized.
 */

#ifndef __WSS_DB_H__
#define __WSS_DB_H__

#include "uds.h"
#include "bcs.h"
#include "wss.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup WSS_DB_MAROC Defines
 * @{
 */
#define WSS_DB_RECORDS_MAX      2      /**< Maximum number of recorded weight scale data stored. */
#define WSS_DB_REC_CMD_MAX      10
#define WSS_DB_BCS_MEAS_TYPE    0x00   /**< Type of Body Composition measurement data. */
#define WSS_DB_WSS_MEAS_TYPE    0x01   /**< Type of Weight Scale measurement data. */
#define WSS_DB_ENV_TAG          0xC020 /**< The nvds tag of saving database enviorenment params. */
/** @} */

/**
 * @defgroup UDS_ENUM Enumerations
 * @{
 */
/**@brief User Data Service Control Point Command Type.*/
typedef enum
{
    UCP_CMD_TYPE_READ_REC,   /**< Register New User Operation Code.*/
    UCP_CMD_TYPE_WRITE_REC,  /**< Consent Operation Code.*/
    UCP_CMD_TYPE_ADD_REC,    /**< Delete User Data Operation Code.*/
    UCP_CMD_TYPE_DEL_REC,    /**< Delete User Data Operation Code.*/
    UCP_CMD_TYPE_CLEAR_REC,  /**< List All Users Operation Code.*/
} wss_db_rec_cmd_type_t;
/** @} */

/**
 * @defgroup WSS_STRUCT Structures
 * @{
 */
 /**@brief Weight Scale measurement value structure */
typedef union
{
    bcs_meas_val_t bcs_meas_val;                  /**< Body Composition measurement value. */
    wss_meas_val_t wss_meas_val;                  /**< Weight Scale measurement value. */
} meas_val_t;

/**@brief Weight Scale Database measurement fifo structure */
typedef struct
{
    meas_val_t meas_val[WSS_CACHE_MEAS_NUM_MAX];  /**< Measurement value. */
    uint16_t   front;                             /**< Fifo front. */
    uint16_t   tail;                              /**< Fifo tail. */
} wss_db_meas_fifo_t;

/**@brief Weight Scale record structure */
typedef struct
{
    wss_db_meas_fifo_t     wss_db_meas_fifo[2];   /**< User's measurement data fifo. */
} wss_rec_t;

typedef struct
{
    wss_db_rec_cmd_type_t rec_cmd[10];                       /**< Measurement value. */
    uint16_t              front;                             /**< Fifo front. */
    uint16_t              tail;                              /**< Fifo tail. */
} wss_rec_cmd_queue_t;

/**@brief Weight Scale record structure */
typedef struct
{
    uint16_t     consent_code;
    uint32_t     db_change_incr_val;
    uint16_t     weight;              /**< User's weight. */
	uint8_t      age;                 /**< User's age. */
	birth_date_t date_of_birth;       /**< User's birth date. */
	uint16_t     height;              /**< User's height. */
	uint8_t      gender;              /**< User's gender. */
    uint8_t      first_name[50];     /**< User's first name. */
    uint16_t     name_length;         /**< Length of User's first name. */    
} wss_user_data_t;

/**@brief Weight Scale Service Database environment variable. */
typedef struct
{
    wss_user_data_t user_data[WSS_DB_RECORDS_MAX];          /**< User data. */
    uint8_t         user_id;
} wss_db_user_buf_t;
/** @} */

/**
 * @defgroup WSS_DB_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize the user data record database.
 *****************************************************************************************
 */
void wss_db_init(void);

/**
 *****************************************************************************************
 * @brief Get the number of records in the database.
 *
 * @return Number of records in the database.
 *****************************************************************************************
 */
uint16_t wss_db_records_num_get(void);

/**
 *****************************************************************************************
 * @brief Get the ucp command from ucp command queue.
 *
 * @param[in]   p_ucp_cmd_queue: Pointer to the ucp command queue.
 * @param[in]   ucp_cmd:         An ucp command.
 *
 * @return If get command successfully or not.
 *****************************************************************************************
 */
bool wss_db_ucp_cmd_queue_elem_push(wss_rec_cmd_queue_t *p_ucp_cmd_queue, wss_db_rec_cmd_type_t ucp_cmd);

/**
 *****************************************************************************************
 * @brief Set the measurement fifo value of a user.
 *
 * @param[in]   rec_idx:    Index of the record to change.
 * @param[in]   p_meas_val: Pointer to measurement fifo value.
 * @param[in]   meas_type:  Type of measurement fifo value. 
 *
 * @return If set value successfully or not.
 *****************************************************************************************
 */
bool wss_db_record_meas_fifo_set(uint8_t rec_idx, meas_val_t *p_meas_val, uint8_t meas_type);

/**
 *****************************************************************************************
 * @brief Get the measurement fifo value of a user.
 *
 * @param[in]   rec_idx:    Index of the record to obtain.
 * @param[in]   p_meas_val: Pointer to measurement fifo value.
 * @param[in]   meas_type:  Type of measurement fifo value. 
 *
 * @return If get value successfully or not.
 *****************************************************************************************
 */
uint8_t wss_db_record_meas_fifo_get(uint8_t rec_idx, meas_val_t *p_meas_val, uint8_t meas_type);

/**
 *****************************************************************************************
 * @brief Get the new user id.
 *
 * @return If get new user id successfully or not.
 *****************************************************************************************
 */
uint16_t wss_db_user_id_get(void);

/**
 *****************************************************************************************
 * @brief Update user data Scheduling Function..
 *****************************************************************************************
 */
void user_data_update_schedule(void);

/**
 *****************************************************************************************
 * @brief Confirm the validation of the user id.
 *
 * @param[in]   user_id:    The user id.
 *
 * @return If the user id is valid or not.
 *****************************************************************************************
 */
bool wss_db_user_id_valid_confirm(uint8_t user_id);
/** @} */

#endif
/** @} */
/** @} */
