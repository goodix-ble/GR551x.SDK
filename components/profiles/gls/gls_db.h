/**
 ****************************************************************************************
 *
 * @file gls_db.h
 *
 * @brief Glucose Service Database API.
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
 * @defgroup BLE_SDK_GLS_DB Glucose Service Database (GLS_DB)
 * @{
 * @brief Glucose Service Database module.
 *
 * @details This module implements at database of stored glucose measurement values, application
 *          can add, delete, get and clear glucose measurement records in database after database
 *          environment variables are initialized.
 */

#ifndef __GLS_DB_H__
#define __GLS_DB_H__

#include "gls.h"
#include "gls_racp.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup GLS_DB_MAROC Defines
 * @{
 */
#define GLS_DB_RECORDS_MAX      20      /**< Maximum glucose measurement values records stored. */
/** @} */


/**
 * @defgroup GLS_DB_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize the glucose record database.
 *****************************************************************************************
 */
void gls_db_init(void);

/**
 *****************************************************************************************
 * @brief Add a record at the end of the database.
 *
 * @details This call adds a record as the last record in the database.
 *
 * @param[in] p_rec: Pointer to record to add to database.
 *
 * @return If add successfully or not.
 *****************************************************************************************
 */
bool gls_db_record_add(gls_rec_t *p_rec);

/**
 *****************************************************************************************
 * @brief Delete a database entry.
 *
 * @details This call deletes a record from the database.
 *
 * @param[in] rec_idx: Index of record to delete.
 *
 * @return If delete successfully or not.
 *****************************************************************************************
 */
bool gls_db_record_delete(uint8_t rec_idx);

/**
 *****************************************************************************************
 * @brief Get the number of records in the database.
 *
 * @return Number of records in the database.
 *****************************************************************************************
 */
uint16_t gls_db_records_num_get(void);

/**
 *****************************************************************************************
 * @brief Get the number of records satisfying the filtering condition.
 *
 * @return Number of records filtered.
 *****************************************************************************************
 */
uint16_t gls_db_filter_records_num_get(gls_racp_filter_t *p_filter);

/**
 *****************************************************************************************
 * @brief Get a record from the database.
 *
 * @details This call returns a specified record from the database.
 *
 * @param[in]   rec_idx: Index of the record to retrieve.
 * @param[out]  p_rec:    Pointer to record structure where retrieved record is copied to.
 *
 * @return If get successfully or not.
 *****************************************************************************************
 */
bool gls_db_record_get(uint8_t rec_idx, gls_rec_t *p_rec);

/**
 *****************************************************************************************
 * @brief Clear database.
 *****************************************************************************************
 */
void gls_db_record_clear(void);
/** @} */

#endif
/** @} */
/** @} */
