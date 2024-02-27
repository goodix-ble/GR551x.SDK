/**
 ****************************************************************************************
 *
 * @file gls_db.c
 *
 * @brief Glucose Service Database implementation.
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
#include "gls_db.h"

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Glucose Service record structure. */
struct gls_db_single_rec_t
{
    bool        is_recorded;
    gls_rec_t   record;
};

/**@brief Glucose Service Database environment variable. */
struct gls_db_env_t
{
    struct gls_db_single_rec_t  database[GLS_DB_RECORDS_MAX];  /**< Glucose Service measurement values records database. */
    uint8_t                     rec_index[GLS_DB_RECORDS_MAX]; /**< Glucose Service measurement values record indexs. */
    uint16_t                    num_records;                   /**< Number of  measurement values records in database. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static struct gls_db_env_t s_gls_db_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Get number of records which are less or equal to setting value.
 *
 * @param[in] p_filter: Pointer to filter parameters.
 *
 * @return Number of records.
 *****************************************************************************************
 */
uint16_t gls_db_le_or_eq_record_num_get(gls_racp_filter_t *p_filter)
{
    gls_rec_t gls_res;
    uint16_t  num_get = 0;

    for (uint8_t i = 0; i < s_gls_db_env.num_records; i++)
    {
        if (gls_db_record_get(i, &gls_res))
        {
            if (GLS_RACP_FILTER_SEQ_NUMBER == p_filter->racp_filter_type)
            {
                if (p_filter->val.seq_num.max >= gls_res.meas_val.sequence_number)
                {
                    num_get++;
                }
            }
            else if (GLS_RACP_FILTER_USER_FACING_TIME == p_filter->racp_filter_type)
            {
                if (-1 != gls_racp_user_time_compare(&p_filter->val.time.max,
                                                     &gls_res.meas_val.base_time))
                {
                    num_get++;
                }
            }
        }
        else
        {
            break;
        }
    }

    return num_get;
}

/**
 *****************************************************************************************
 * @brief Get number of records which are greater or equal to setting value.
 *
 * @param[in] p_filter: Pointer to filter parameters.
 *
 * @return Number of records.
 *****************************************************************************************
 */
uint16_t gls_db_gt_or_eq_record_num_get(gls_racp_filter_t *p_filter)
{
    gls_rec_t gls_res;
    uint16_t  num_get = 0;

    for (uint8_t i = 0; i < s_gls_db_env.num_records; i++)
    {
        if (gls_db_record_get(i, &gls_res))
        {
            if (GLS_RACP_FILTER_SEQ_NUMBER == p_filter->racp_filter_type)
            {
                if (p_filter->val.seq_num.min <= gls_res.meas_val.sequence_number)
                {
                    num_get++;
                }
            }
            else if (GLS_RACP_FILTER_USER_FACING_TIME == p_filter->racp_filter_type)
            {
                if (-1 != gls_racp_user_time_compare(&gls_res.meas_val.base_time,
                                                     &p_filter->val.time.min))
                {
                    num_get++;
                }
            }
        }
        else
        {
            break;
        }
    }

    return num_get;
}

/**
 *****************************************************************************************
 * @brief Get number of records which are within range of setting value.
 *
 * @param[in] p_filter: Pointer to filter parameters.
 *
 * @return Number of records.
 *****************************************************************************************
 */
uint16_t gls_db_within_range_record_num_get(gls_racp_filter_t *p_filter)
{
    gls_rec_t gls_res;
    uint16_t  num_get = 0;


    for (uint8_t i = 0; i < s_gls_db_env.num_records; i++)
    {
        if (gls_db_record_get(i, &gls_res))
        {
            if (GLS_RACP_FILTER_SEQ_NUMBER == p_filter->racp_filter_type)
            {
                if ((gls_res.meas_val.sequence_number >= p_filter->val.seq_num.min) && \
                        (gls_res.meas_val.sequence_number <= p_filter->val.seq_num.max))
                {
                    num_get++;
                }
            }
            else if (GLS_RACP_FILTER_USER_FACING_TIME == p_filter->racp_filter_type)
            {
                if ((-1 != gls_racp_user_time_compare(&gls_res.meas_val.base_time, &p_filter->val.time.min)) && \
                        (-1 != gls_racp_user_time_compare(&p_filter->val.time.max, &gls_res.meas_val.base_time)))
                {
                    num_get++;
                }
            }
        }
        else
        {
            break;
        }
    }

    return num_get;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void gls_db_init(void)
{
    for (uint8_t i = 0; i < GLS_DB_RECORDS_MAX; i++)
    {
        s_gls_db_env.database[i].is_recorded = false;
        s_gls_db_env.rec_index[i]            = 0xFF;
    }

    s_gls_db_env.num_records = 0;
}

bool gls_db_record_add(gls_rec_t *p_rec)
{
    uint8_t i = 0;

    if (GLS_DB_RECORDS_MAX <= s_gls_db_env.num_records)
    {
        return false;
    }

    for (i = 0; i < GLS_DB_RECORDS_MAX; i++)
    {
        if (!s_gls_db_env.database[i].is_recorded)
        {
            s_gls_db_env.database[i].is_recorded = true;
            s_gls_db_env.database[i].record      = *p_rec;
            s_gls_db_env.rec_index[s_gls_db_env.num_records] = i;
            s_gls_db_env.num_records++;
            return true;
        }
    }

    return false;
}

bool gls_db_record_delete(uint8_t rec_idx)
{
    if (rec_idx >= s_gls_db_env.num_records)
    {
        return false;
    }

    s_gls_db_env.database[s_gls_db_env.rec_index[rec_idx]].is_recorded = false;

    s_gls_db_env.num_records--;

    for (uint8_t i = rec_idx; i < s_gls_db_env.num_records; i++)
    {
        s_gls_db_env.rec_index[i] = s_gls_db_env.rec_index[i + 1];
    }

    return true;
}

uint16_t gls_db_records_num_get(void)
{
    return s_gls_db_env.num_records;
}

uint16_t gls_db_filter_records_num_get(gls_racp_filter_t *p_filter)
{
    uint16_t  num_get = 0;

    switch (p_filter->racp_operator)
    {
        case GLS_RACP_OPERATOR_ALL_RECS:
            num_get = s_gls_db_env.num_records;
            break;

        case GLS_RACP_OPERATOR_LE_OR_EQ:
            num_get = gls_db_le_or_eq_record_num_get(p_filter);
            break;

        case GLS_RACP_OPERATOR_GT_OR_EQ:
            num_get = gls_db_gt_or_eq_record_num_get(p_filter);
            break;

        case GLS_RACP_OPERATOR_WITHIN_RANGE_OF:
            num_get = gls_db_within_range_record_num_get(p_filter);
            break;

        case GLS_RACP_OPERATOR_FIRST_REC:
        case GLS_RACP_OPERATOR_LAST_REC:
            if (0 != s_gls_db_env.num_records)
            {
                num_get = 1;
            }
            break;

        default:
            break;
    }

    return num_get;
}

bool gls_db_record_get(uint8_t rec_idx, gls_rec_t *p_rec)
{
    if (rec_idx >= s_gls_db_env.num_records)
    {
        return false;
    }

    *p_rec = s_gls_db_env.database[s_gls_db_env.rec_index[rec_idx]].record;

    return true;
}

void gls_db_record_clear(void)
{
    for (uint8_t i = 0; i < GLS_DB_RECORDS_MAX; i++)
    {
        s_gls_db_env.database[i].is_recorded = false;
        s_gls_db_env.rec_index[i]            = 0xFF;
    }

    s_gls_db_env.num_records = 0;
}

