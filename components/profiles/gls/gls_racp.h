/**
 ****************************************************************************************
 *
 * @file gls_racp.h
 *
 * @brief Glucose Service Record Access Control Point API.
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
 * @defgroup BLE_SDK_GLS_RACP Glucose Service Record Access Control Point (GLS_RACP)
 * @{
 * @brief Glucose Service Record Access Control Point module.
 *
 * @details This module implements Glucose Service Record Access Control Point (RACP), includeding
 *          parsing, executing and responsing RACP from Client.
 */

#ifndef __GLS_RACP_H__
#define __GLS_RACP_H__

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ble_prf_types.h"
#include "gr_includes.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup GLS_RACP_MACRO Defines
 * @{
 */
#define GLS_RACP_FILTER_SEQ_NUM_LEN       2       /**< Length of filter sequence number. */
#define GLS_RACP_FILTER_USER_TIME_LEN     7       /**< Length of filter user facing time. */
/** @} */

/**
 * @defgroup GLS_RACP_ENUM Enumerations
 * @{
 */
/**@brief Glucose Recoerd Access Control Point Operation Codes. */
typedef enum
{
    GLS_RACP_OP_RESERVED,               /**< Reserved for future use. */
    GLS_RACP_OP_REP_STRD_RECS,          /**< Report stored records (Operator: Value from Operator Table). */
    GLS_RACP_OP_DEL_STRD_RECS,          /**< Delete stored records (Operator: Value from Operator Table). */
    GLS_RACP_OP_ABORT_OP,               /**< Abort operation (Operator: Null 'value of 0x00 from Operator Table'). */
    GLS_RACP_OP_REP_NB_OF_STRD_RECS,    /**< Report number of stored records (Operator: Value from Operator Table). */
    GLS_RACP_OP_NB_OF_STRD_RECS_RSP,    /**< Number of stored records response (Operator: Null 'value of 0x00 from Operator Table'). */
    GLS_RACP_OP_RSP_CODE,               /**< Response Code (Operator: Null 'value of 0x00 from Operator Table'). */
} gls_racp_op_code_t;

/**@brief Glucose Recoerd Access Control Point Operator. */
typedef enum
{
    GLS_RACP_OPERATOR_NULL,             /**< NULL. */
    GLS_RACP_OPERATOR_ALL_RECS,         /**< All records. */
    GLS_RACP_OPERATOR_LE_OR_EQ,         /**< Less than or equal to. */
    GLS_RACP_OPERATOR_GT_OR_EQ,         /**< Greater than or equal to. */
    GLS_RACP_OPERATOR_WITHIN_RANGE_OF,  /**< Within rang of (inclusive). */
    GLS_RACP_OPERATOR_FIRST_REC,        /**< First record(i.e. oldest record). */
    GLS_RACP_OPERATOR_LAST_REC,         /**< Last record (i.e. most recent record). */
} gls_racp_operator_t;

/**@brief Glucose Recoerd Access Control Point Response codes. */
typedef enum
{
    GLS_RACP_RSP_RESERVED,                  /**< Reserved for future use. */
    GLS_RACP_RSP_SUCCESS,                   /**< Normal response for successful operation. */
    GLS_RACP_RSP_OP_CODE_NOT_SUP,           /**< Normal response if unsupported Op Code is received. */
    GLS_RACP_RSP_INVALID_OPERATOR,          /**< Normal response if Operator received does not meet the requirements of the service (e.g. Null was expected). */
    GLS_RACP_RSP_OPERATOR_NOT_SUP,          /**< Normal response if unsupported Operator is received. */
    GLS_RACP_RSP_INVALID_OPERAND,           /**< Normal response if Operand received does not meet the requirements of the service. */
    GLS_RACP_RSP_NO_RECS_FOUND,             /**< Normal response if request to report stored records or request to delete stored records resulted in no records meeting criteria. */
    GLS_RACP_RSP_ABORT_UNSUCCESSFUL,        /**< Normal response if request for Abort cannot be completed. */
    GLS_RACP_RSP_PROCEDURE_NOT_COMPLETED,   /**< Normal response if unable to complete a procedure for any reason. */
    GLS_RACP_RSP_OPERAND_NOT_SUP,           /**< Normal response if unsupported Operand is received. */
    GLS_RACP_RSP_VALID_DECODE = 0xff        /**< User-defined response if RACP request is valid for GLS. */
} gls_racp_operand_t;

/**@brief Glucose Recoerd Access Control Point filter types. */
typedef enum
{
    GLS_RACP_FILTER_RESERVED,           /**< Reserved for future use. */
    GLS_RACP_FILTER_SEQ_NUMBER,         /**< Filter data using Sequence Number criteria. */
    GLS_RACP_FILTER_USER_FACING_TIME,   /**< Filter data using User Facing Time criteria. */
} gls_racp_filter_type_t;
/** @} */

/**
 * @defgroup GLS_RACP_STRUCT Structures
 * @{
 */
/**@brief Glucose Recoerd Access Control Point filter value. */
typedef struct
{
    gls_racp_operator_t    racp_operator;           /**< Glucose Recoerd Access Control Point Operator. */
    gls_racp_filter_type_t racp_filter_type;        /**< Glucose Recoerd Access Control Point filter types. */
    union
    {
        struct
        {
            uint16_t min;                            /**< Min sequence number. */
            uint16_t max;                            /**< Max sequence number. */
        } seq_num;                                   /**< Sequence number filtering (racp_filter_type = GLS_RACP_FILTER_SEQ_NUMBER). */
        struct
        {
            prf_date_time_t min;                     /**< Min base time. */
            prf_date_time_t max;                     /**< Max base time. */
        } time;                                      /**< User facing time filtering (filter_type = GLS_RACP_FILTER_USER_FACING_TIME). */
    } val;                                           /**< Filter union. */
} gls_racp_filter_t;

/**@brief Glucose Recoerd Access Control Point request value. */
typedef struct
{
    gls_racp_op_code_t   op_code;            /**< Glucose Recoerd Access Control Point Operation Code. */
    gls_racp_filter_t    filter;             /**< Glucose Recoerd Access Control Point Operation Code. */
} gls_racp_req_t;

/**@brief Glucose Recoerd Access Control Point response value. */
typedef struct
{
    gls_racp_op_code_t   op_code;            /**< Glucose Recoerd Access Control Point Operation Code. */
    gls_racp_operator_t  racp_operator;      /**< Glucose Recoerd Access Control Point Operator. */
    union
    {
        uint16_t num_of_record;              /**< Number of record (if op_code = GLS_RACP_OP_REP_NB_OF_STRD_RECS). */
        struct
        {
            gls_racp_op_code_t op_code_req;             /**< Request Op Code. */
            gls_racp_operand_t status;                  /**< Command Status (if op_code = GLS_RACP_OP_RSP_CODE). */
        } rsp;                               /**< Response. */
    } operand;                               /**< Response operand. */
} gls_racp_rsp_t;
/** @} */

/**
 * @defgroup GLS_RACP_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Decode record access control point request.
 *
 * @param[in]  p_data:     Pointer to data.
 * @param[in]  length:     Length of data.
 * @param[out] p_racp_req: Potnter to buffer saved decode result.
 *
 * @return Record access control point request decode result.
 *****************************************************************************************
 */
gls_racp_operand_t gls_racp_req_decode(const uint8_t *p_data, uint16_t length, gls_racp_req_t *p_racp_req);

/**
 *****************************************************************************************
 * @brief Encode record access control point response.
 *
 * @param[in]  p_racp_rsp:       Pointer to buffer saved response.
 * @param[out] p_encoded_buffer: Pointer to buffer where the encoded data will be written.
 *
 * @return Length of encode data.
 *****************************************************************************************
 */
uint16_t gls_racp_rsp_encode(gls_racp_rsp_t *p_racp_rsp, uint8_t *p_encoded_buffer);

/**
 *****************************************************************************************
 * @brief Compared two date time.
 *
 * @param[in]  p_compared_date_time: Pointer to compared date time.
 * @param[out] p_base_date_time:     Pointer to base date time.
 *
 * @return Result of compared.
 *          1: compared_date_time > base_date_time.
 *         -1: compared_date_time < base_date_time
 *          0: compared_date_time = base_date_time
 *****************************************************************************************
 */
int8_t gls_racp_user_time_compare(prf_date_time_t *p_compared_date_time, prf_date_time_t *p_base_date_time);
/** @} */

#endif
/** @} */
/** @} */

