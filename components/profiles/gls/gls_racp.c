/**
 ****************************************************************************************
 *
 * @file gls_racp.c
 *
 * @brief Glucose Record Access Control Point implementation.
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
#include "gls_racp.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
int8_t gls_racp_user_time_compare(prf_date_time_t *p_compared_date_time, prf_date_time_t *p_base_date_time)
{
    if (p_compared_date_time->year < p_base_date_time->year)
    {
        return -1;
    }
    else if (p_compared_date_time->year > p_base_date_time->year)
    {
        return 1;
    }

    if (p_compared_date_time->month < p_base_date_time->month)
    {
        return -1;
    }
    else if (p_compared_date_time->month > p_base_date_time->month)
    {
        return 1;
    }

    if (p_compared_date_time->day < p_base_date_time->day)
    {
        return -1;
    }
    else if (p_compared_date_time->day > p_base_date_time->day)
    {
        return 1;
    }

    if (p_compared_date_time->hour < p_base_date_time->hour)
    {
        return -1;
    }
    else if (p_compared_date_time->hour > p_base_date_time->hour)
    {
        return 1;
    }

    if (p_compared_date_time->min < p_base_date_time->min)
    {
        return -1;
    }
    else if (p_compared_date_time->min > p_base_date_time->min)
    {
        return 1;
    }

    if (p_compared_date_time->sec < p_base_date_time->sec)
    {
        return -1;
    }
    else if (p_compared_date_time->sec > p_base_date_time->sec)
    {
        return 1;
    }

    return 0;
}

gls_racp_operand_t gls_racp_req_decode(const uint8_t *p_data, uint16_t length, gls_racp_req_t *p_racp_req)
{
    uint8_t index = 0;

    p_racp_req->op_code              = (gls_racp_op_code_t)p_data[index++];
    p_racp_req->filter.racp_operator = (gls_racp_operator_t)p_data[index++];

    if (GLS_RACP_OP_REP_STRD_RECS == p_racp_req->op_code ||
        GLS_RACP_OP_DEL_STRD_RECS == p_racp_req->op_code ||
        GLS_RACP_OP_REP_NB_OF_STRD_RECS == p_racp_req->op_code)
    {
        if(GLS_RACP_OPERATOR_ALL_RECS > p_racp_req->filter.racp_operator)
        {
            return GLS_RACP_RSP_INVALID_OPERATOR;
        }
    }
    else
    {
        if(GLS_RACP_OPERATOR_ALL_RECS <= p_racp_req->filter.racp_operator)
        {
            return GLS_RACP_RSP_INVALID_OPERATOR;
        }
    }

    if (GLS_RACP_OPERATOR_LAST_REC < p_racp_req->filter.racp_operator)
    {
        return GLS_RACP_RSP_OPERATOR_NOT_SUP;
    }

    if ((GLS_RACP_OPERATOR_LE_OR_EQ <= p_racp_req->filter.racp_operator) && \
            ((GLS_RACP_OPERATOR_WITHIN_RANGE_OF) >= p_racp_req->filter.racp_operator))
    {
        if (length <= index)
        {
            return GLS_RACP_RSP_INVALID_OPERAND;
        }
        else
        {
            p_racp_req->filter.racp_filter_type = (gls_racp_filter_type_t)p_data[index++];

            if (GLS_RACP_FILTER_SEQ_NUMBER == p_racp_req->filter.racp_filter_type)
            {
                if (GLS_RACP_OPERATOR_LE_OR_EQ == p_racp_req->filter.racp_operator)
                {
                    if (GLS_RACP_FILTER_SEQ_NUM_LEN != (length - index))
                    {
                        return GLS_RACP_RSP_INVALID_OPERAND;
                    }
                    else
                    {
                        p_racp_req->filter.val.seq_num.max = BUILD_U16(p_data[index], p_data[index + 1]);
                    }
                }
                else if (GLS_RACP_OPERATOR_GT_OR_EQ == p_racp_req->filter.racp_operator)
                {
                    if (GLS_RACP_FILTER_SEQ_NUM_LEN != (length - index))
                    {
                        return GLS_RACP_RSP_INVALID_OPERAND;
                    }
                    else
                    {
                        p_racp_req->filter.val.seq_num.min = BUILD_U16(p_data[index], p_data[index + 1]);
                    }
                }
                else if (GLS_RACP_OPERATOR_WITHIN_RANGE_OF == p_racp_req->filter.racp_operator)
                {
                    if (GLS_RACP_FILTER_SEQ_NUM_LEN * 2 != (length - index))
                    {
                        return GLS_RACP_RSP_INVALID_OPERAND;
                    }
                    else
                    {
                        p_racp_req->filter.val.seq_num.min = BUILD_U16(p_data[index], p_data[index + 1]);
                        p_racp_req->filter.val.seq_num.max = BUILD_U16(p_data[index + 2], p_data[index + 3]);

                        if (p_racp_req->filter.val.seq_num.min > p_racp_req->filter.val.seq_num.max)
                        {
                            return GLS_RACP_RSP_INVALID_OPERAND;
                        }
                    }
                }

            }
            else if (GLS_RACP_FILTER_USER_FACING_TIME == p_racp_req->filter.racp_filter_type)
            {
                if (GLS_RACP_OPERATOR_LE_OR_EQ == p_racp_req->filter.racp_operator)
                {
                    if (GLS_RACP_FILTER_USER_TIME_LEN != (length - index))
                    {
                        return GLS_RACP_RSP_INVALID_OPERAND;
                    }
                    else
                    {
                        prf_unpack_date_time(&p_data[index], &p_racp_req->filter.val.time.max);
                    }
                }
                else if (GLS_RACP_OPERATOR_GT_OR_EQ == p_racp_req->filter.racp_operator)
                {
                    if (GLS_RACP_FILTER_USER_TIME_LEN != (length - index))
                    {
                        return GLS_RACP_RSP_INVALID_OPERAND;
                    }
                    else
                    {
                        prf_unpack_date_time(&p_data[index], &p_racp_req->filter.val.time.min);
                    }
                }
                else if (GLS_RACP_OPERATOR_WITHIN_RANGE_OF == p_racp_req->filter.racp_operator)
                {
                    if (GLS_RACP_FILTER_USER_TIME_LEN * 2 != (length - index))
                    {
                        return GLS_RACP_RSP_INVALID_OPERAND;
                    }
                    else
                    {
                        index += prf_unpack_date_time(&p_data[index], &p_racp_req->filter.val.time.min);
                        prf_unpack_date_time(&p_data[index], &p_racp_req->filter.val.time.max);

                        if (-1 != gls_racp_user_time_compare(&p_racp_req->filter.val.time.max, &p_racp_req->filter.val.time.min))
                        {
                            return GLS_RACP_RSP_INVALID_OPERAND;
                        }
                    }
                }
            }
            else
            {
                return GLS_RACP_RSP_OPERAND_NOT_SUP;
            }
        }
    }
    else if (0 != (length - index))
    {
        return GLS_RACP_RSP_INVALID_OPERAND;
    }

    return GLS_RACP_RSP_VALID_DECODE;
}

uint16_t gls_racp_rsp_encode(gls_racp_rsp_t *p_racp_rsp, uint8_t *p_encoded_buffer)
{
    uint16_t length = 0;

    p_encoded_buffer[length++] = p_racp_rsp->op_code;
    p_encoded_buffer[length++] = GLS_RACP_OPERATOR_NULL;

    if (GLS_RACP_OP_REP_NB_OF_STRD_RECS == p_racp_rsp->op_code)
    {
        p_encoded_buffer[length++] = LO_U16(p_racp_rsp->operand.num_of_record);
        p_encoded_buffer[length++] = HI_U16(p_racp_rsp->operand.num_of_record);
    }
    else
    {
        p_encoded_buffer[length++] = p_racp_rsp->operand.rsp.op_code_req;
        p_encoded_buffer[length++] = p_racp_rsp->operand.rsp.status;
    }

    return length;
}

