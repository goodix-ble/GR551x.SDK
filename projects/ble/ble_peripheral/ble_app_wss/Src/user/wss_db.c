/**
 ****************************************************************************************
 *
 * @file wss_db.c
 *
 * @brief Weight Scale Service Database implementation.
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
#include "wss_db.h"
#include "app_log.h"
#include "grx_sys.h"
#include "app_io.h"

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Weight Scale Service Database environment variable. */
typedef struct
{
    uint16_t                    tag[WSS_DB_RECORDS_MAX];                /**< NVDS Tag of each record. */
    bool                        is_recorded[WSS_DB_RECORDS_MAX];        /**< Is a record used. */
    wss_db_user_buf_t           user_buf;                               /**< User data. */
    uint16_t                    num_records;                            /**< Number of user data values records in database. */
} wss_db_env_t;

/*
* LOCAL FUNCTION DECLARATION
*****************************************************************************************
*/
static bool wss_db_fifo_is_empty(wss_db_meas_fifo_t *p_db_meas_fifo);
static bool wss_db_fifo_is_full(wss_db_meas_fifo_t *p_db_meas_fifo);
static bool wss_db_fifo_elem_push(wss_db_meas_fifo_t *p_db_meas_fifo, meas_val_t *p_meas_val);
static bool wss_db_fifo_elem_pop(wss_db_meas_fifo_t *p_db_meas_fifo, meas_val_t *p_meas_val);

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static wss_db_env_t  s_wss_db_env;
wss_db_user_buf_t    s_wss_db_user_buf;
wss_rec_cmd_queue_t  s_wss_db_rec_cmd_queue;

/**
 *****************************************************************************************
 * @brief Determine if an ucp command queue is empty.
 *
 * @param[in] p_ucp_cmd_queue: Pointer to an ucp command queue.
 *
 * @return If this ucp command queue is empty.
 *****************************************************************************************
 */
static bool wss_db_ucp_cmd_queue_is_empty(wss_rec_cmd_queue_t *p_ucp_cmd_queue)
{
    if (p_ucp_cmd_queue->front == p_ucp_cmd_queue->tail)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 *****************************************************************************************
 * @brief Determine if an ucp command queue is full.
 *
 * @param[in] p_ucp_cmd_queue: Pointer to an ucp command queue.
 *
 * @return If this usp command queue is full.
 *****************************************************************************************
 */
static bool wss_db_ucp_cmd_queue_is_full(wss_rec_cmd_queue_t *p_ucp_cmd_queue)
{
    if ((p_ucp_cmd_queue->tail + 1) % WSS_DB_REC_CMD_MAX == p_ucp_cmd_queue->front)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 *****************************************************************************************
 * @brief Save an ucp command to queue.
 *
 * @param[in] p_ucp_cmd_queue: Pointer to ucp command queue.
 * @param[in] ucp_cmd:         Ucp command.
 *
 * @return If save an ucp command is successful.
 *****************************************************************************************
 */
bool wss_db_ucp_cmd_queue_elem_push(wss_rec_cmd_queue_t *p_ucp_cmd_queue, wss_db_rec_cmd_type_t ucp_cmd)
{
    if (wss_db_ucp_cmd_queue_is_full(p_ucp_cmd_queue))
    {
        return false;
    }

    p_ucp_cmd_queue->rec_cmd[p_ucp_cmd_queue->tail] = ucp_cmd;
    p_ucp_cmd_queue->tail = (p_ucp_cmd_queue->tail + 1) % WSS_DB_REC_CMD_MAX;

    return true;
}

/**
 *****************************************************************************************
 * @brief Get an ucp command from queue.
 *
 * @param[in] p_ucp_cmd_queue: Pointer to an ucp command queue.
 * @param[in] p_ucp_cmd:       Pointer to an ucp command.
 *
 * @return If get an ucp command from queue is successful.
 *****************************************************************************************
 */
static bool wss_db_ucp_cmd_queue_elem_pop(wss_rec_cmd_queue_t *p_ucp_cmd_queue, wss_db_rec_cmd_type_t * p_ucp_cmd)
{
    if (wss_db_ucp_cmd_queue_is_empty(p_ucp_cmd_queue))
    {
        return false;
    }

    *p_ucp_cmd = p_ucp_cmd_queue->rec_cmd[p_ucp_cmd_queue->front];
    p_ucp_cmd_queue->front = (p_ucp_cmd_queue->front + 1) % WSS_DB_REC_CMD_MAX;

    return true;
}

/**
 *****************************************************************************************
 * @brief Determine if measurement fifo is empty.
 *
 * @param[in] p_db_meas_fifo: Pointer to a measurement fifo.
 *
 * @return If measurement fifo is empty.
 *****************************************************************************************
 */
static bool wss_db_fifo_is_empty(wss_db_meas_fifo_t *p_db_meas_fifo)
{
    if (p_db_meas_fifo->front == p_db_meas_fifo->tail)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 *****************************************************************************************
 * @brief Determine if measurement fifo is full.
 *
 * @param[in] p_db_meas_fifo: Pointer to a measurement fifo.
 *
 * @return If measurement fifo is full.
 *****************************************************************************************
 */
static bool wss_db_fifo_is_full(wss_db_meas_fifo_t *p_db_meas_fifo)
{
    if ((p_db_meas_fifo->tail + 1) % WSS_CACHE_MEAS_NUM_MAX == p_db_meas_fifo->front)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 *****************************************************************************************
 * @brief Get a measurement fifo value.
 *
 * @param[in] p_db_meas_fifo: Pointer to a measurement fifo.
 * @param[in] p_meas_val:     Pointer to a measurement value to be added.
 *
 * @return If set a measurement fifo value is successful.
 *****************************************************************************************
 */
static bool wss_db_fifo_elem_push(wss_db_meas_fifo_t *p_db_meas_fifo, meas_val_t *p_meas_val)
{
    if (wss_db_fifo_is_full(p_db_meas_fifo))
    {
        APP_LOG_DEBUG("Database is full.");
        p_db_meas_fifo->front = (p_db_meas_fifo->front + 1) % WSS_CACHE_MEAS_NUM_MAX;
    }

    memcpy(&p_db_meas_fifo->meas_val[p_db_meas_fifo->tail], p_meas_val, sizeof(meas_val_t));
    p_db_meas_fifo->tail = (p_db_meas_fifo->tail + 1) % WSS_CACHE_MEAS_NUM_MAX;

    return true;
}

/**
 *****************************************************************************************
 * @brief Get a measurement fifo value.
 *
 * @param[in] p_db_meas_fifo: Pointer to a measurement fifo.
 * @param[in] p_meas_val:     Pointer to a measurement value to be filled.
 *
 * @return If get a measurement fifo value is successful.
 *****************************************************************************************
 */
static bool wss_db_fifo_elem_pop(wss_db_meas_fifo_t *p_db_meas_fifo, meas_val_t *p_meas_val)
{
    if (wss_db_fifo_is_empty(p_db_meas_fifo))
    {
        return false;
    }

    *p_meas_val           = p_db_meas_fifo->meas_val[p_db_meas_fifo->front];
    p_db_meas_fifo->front = (p_db_meas_fifo->front + 1) % WSS_CACHE_MEAS_NUM_MAX;

    return true;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void wss_db_init(void)
{
    uint8_t   status;
    wss_rec_t wss_rec;

    memset(&s_wss_db_env, 0, sizeof(wss_db_env_t));
    memset(&wss_rec, 0, sizeof(wss_rec));

    uint16_t temp_len = sizeof(s_wss_db_env);
    status = nvds_get(WSS_DB_ENV_TAG, &temp_len, (uint8_t *)&s_wss_db_env);
    
    if (status) // No record
    {
        /* Initialized the database. */
        uint16_t  tag = WSS_DB_ENV_TAG + 1;
        
        for (uint8_t i = 0; i < WSS_DB_RECORDS_MAX; i++)
        {
            memset(&(s_wss_db_env.user_buf.user_data[i]), 0, sizeof(wss_user_data_t));
            s_wss_db_env.user_buf.user_data[i].consent_code = 0xFFFF;
            s_wss_db_env.tag[i]                             = tag++;
            s_wss_db_env.is_recorded[i]                     = false;
            
            nvds_put(s_wss_db_env.tag[i], sizeof(wss_rec), (const uint8_t *)&wss_rec);
        }
        s_wss_db_env.num_records = 0;

        status = nvds_put(WSS_DB_ENV_TAG, sizeof(wss_db_env_t), (const uint8_t *)&s_wss_db_env);
    }
    else
    {
        memcpy(&s_wss_db_user_buf, &(s_wss_db_env.user_buf), sizeof(wss_db_user_buf_t));

        for (uint8_t i = 0; i < WSS_DB_RECORDS_MAX; i++)
        {   
            status = nvds_get(s_wss_db_env.tag[i], &temp_len, (uint8_t *)&wss_rec);
            if (status)
            {
                nvds_del(s_wss_db_env.tag[i]);
                nvds_put(s_wss_db_env.tag[i], sizeof(wss_rec), (const uint8_t *)&wss_rec);
            }
        }
    }
}

bool wss_db_user_id_valid_confirm(uint8_t user_id)
{
    bool ret = true;

    if (user_id >= WSS_DB_RECORDS_MAX)
    {
        return false;
    }
    
    if (!s_wss_db_env.is_recorded[user_id])
    {
        ret = false;
    }
    
    return ret;
}

uint16_t wss_db_user_id_get()
{
    uint16_t new_user_id = UDS_UNKNOWN_USER;
    for (uint8_t i = 0; i < WSS_DB_RECORDS_MAX; i++)
    {
        if (!s_wss_db_env.is_recorded[i])
        {
            new_user_id = i;
            break;
        }
    }
    return new_user_id;
}

bool wss_db_record_add(uint8_t user_id)
{
    uint8_t status;
    bool    ret = true;

    if (WSS_DB_RECORDS_MAX <= s_wss_db_env.num_records && (s_wss_db_env.is_recorded[user_id]))
    {
        return false;
    }

    s_wss_db_env.is_recorded[user_id] = true;
    s_wss_db_env.num_records++;
        
    status = nvds_put(WSS_DB_ENV_TAG, sizeof(s_wss_db_env), (const uint8_t *)&s_wss_db_env);
    if (status)
    {
        ret = false;
        APP_LOG_DEBUG("Add new record failed! status:%d.", status);
    }

    return ret;
}

static bool wss_db_user_data_get(uint8_t user_id)
{
    uint8_t   status;
    uint16_t  temp_len = sizeof(s_wss_db_env);
    bool      ret      = true;     

    if (!s_wss_db_env.is_recorded[user_id])
    {
        return false;
    }
        
    status = nvds_get(WSS_DB_ENV_TAG, &temp_len, (uint8_t *)&s_wss_db_env);
    if (status)
    {
        APP_LOG_DEBUG("Require the record failed! status:%d.", status);
        ret = false;
    }

    return ret;
}

static bool wss_db_user_data_set(uint8_t user_id)
{
    uint8_t   status;
    bool      ret = true;     

    if (!s_wss_db_env.is_recorded[user_id])
    {
        return false;
    }

    status = nvds_put(WSS_DB_ENV_TAG, sizeof(wss_db_env_t), (const uint8_t *)&s_wss_db_env);
    if (status)
    {
        APP_LOG_DEBUG("Write the record failed! status:%d.", status);
        ret = false;
    }

    return ret;
}

bool wss_db_record_meas_fifo_set(uint8_t user_id, meas_val_t *p_meas_val, uint8_t meas_type)
{
    uint8_t   status;
    wss_rec_t wss_rec;
    bool      ret      = true;
    uint16_t  temp_len = sizeof(wss_rec_t);

    if (!s_wss_db_env.is_recorded[user_id])
    {
        return false;
    }

    status = nvds_get(s_wss_db_env.tag[user_id], &temp_len, (uint8_t *)&wss_rec);
    if (!status)
    {
        uint8_t put_status;

        if (WSS_DB_WSS_MEAS_TYPE == meas_type)
        {
            if (wss_db_fifo_is_full(&wss_rec.wss_db_meas_fifo[0]))
            {
                meas_val_t meas_val;
                wss_db_fifo_elem_pop(&wss_rec.wss_db_meas_fifo[0], &meas_val);
            }

            if (wss_db_fifo_elem_push(&wss_rec.wss_db_meas_fifo[0], p_meas_val))
            {
                put_status = nvds_put(s_wss_db_env.tag[user_id], sizeof(wss_rec), (const uint8_t *)&wss_rec);
                if (put_status)
                {
                    APP_LOG_DEBUG("Update the wss measurement data failed! status: %d.", put_status);
                    ret = false;
                }
            }
        }
        else if (WSS_DB_BCS_MEAS_TYPE == meas_type)
        {
            if (wss_db_fifo_is_full(&wss_rec.wss_db_meas_fifo[1]))
            {
                meas_val_t meas_val;
                wss_db_fifo_elem_pop(&wss_rec.wss_db_meas_fifo[1], &meas_val);
            }

            if (wss_db_fifo_elem_push(&wss_rec.wss_db_meas_fifo[1], p_meas_val))
            {
                put_status = nvds_put(s_wss_db_env.tag[user_id], sizeof(wss_rec), (const uint8_t *)&wss_rec);
                if (put_status)
                {
                    APP_LOG_DEBUG("Update the bcs measurement data failed! status: %d.", put_status);
                    ret = false;
                }
            }
        }
    }
    else
    {
        ret = false;
        APP_LOG_DEBUG("Require the measurement data before set failed! status: %d.", status);
    }
    return ret;
}

uint8_t wss_db_record_meas_fifo_get(uint8_t user_id, meas_val_t *p_meas_val, uint8_t meas_type)
{
    uint8_t   cache_num = 0;
    wss_rec_t wss_rec;
    uint8_t   status;
    uint16_t  temp_len = sizeof(wss_rec);
    
    status = nvds_get(s_wss_db_env.tag[user_id], &temp_len, (uint8_t *)&wss_rec);
    if (!status)
    {
        uint8_t put_status;
        if (WSS_DB_WSS_MEAS_TYPE == meas_type)
        {
            if (wss_db_fifo_is_empty(&wss_rec.wss_db_meas_fifo[0]))
            {
                APP_LOG_DEBUG("No enough wss data.");
            }
            else
            {
                while (wss_db_fifo_elem_pop(&wss_rec.wss_db_meas_fifo[0], p_meas_val++))
                {
                    put_status = nvds_put(s_wss_db_env.tag[user_id], sizeof(wss_rec), (const uint8_t *)&wss_rec);
                    if (put_status)
                    {
                        APP_LOG_DEBUG("Update the wss measurement data failed! status: %d.", put_status);
                    }
                    cache_num++;
                }
            }
        }
        else if (WSS_DB_BCS_MEAS_TYPE == meas_type)
        {
            if (wss_db_fifo_is_empty(&wss_rec.wss_db_meas_fifo[1]))
            {
                APP_LOG_DEBUG("No enough bcs data.");
            }
            else
            {
                while (wss_db_fifo_elem_pop(&wss_rec.wss_db_meas_fifo[1], p_meas_val++))
                {
                    put_status = nvds_put(s_wss_db_env.tag[user_id], sizeof(wss_rec), (const uint8_t *)&wss_rec);
                    if (put_status)
                    {
                        APP_LOG_DEBUG("Update the bcs measurement data failed! status: %d.", put_status);
                    }
                    cache_num++;
                }
            }
        }
    }
    else
    {
        APP_LOG_DEBUG("Require the measurement data failed! status: %d.", status);
    }
    return cache_num;
}

uint16_t wss_db_records_num_get(void)
{
    return s_wss_db_env.num_records;
}

static bool wss_db_record_delete(uint8_t user_id)
{
    uint8_t   status;
    bool      ret = true;

    if (!s_wss_db_env.is_recorded[user_id])
    {
        return false;
    }

    memset(&(s_wss_db_env.user_buf.user_data[user_id]), 0, sizeof(wss_user_data_t));
    s_wss_db_env.is_recorded[user_id]                     = false;
    s_wss_db_env.num_records --;
    
    status = nvds_put(WSS_DB_ENV_TAG, sizeof(wss_db_env_t), (const uint8_t *)&s_wss_db_env);
    if (status)
    {
        APP_LOG_ERROR("Delete the record failed!");
        ret = false;
    }

    return ret;
}

bool wss_db_record_clear(void)
{
    bool    ret = true;
    uint8_t status;

    for (uint8_t i = 0; i < WSS_DB_RECORDS_MAX; i++)
    {
        s_wss_db_env.is_recorded[i] = false;
        memset(&(s_wss_db_env.user_buf.user_data[i]), 0, sizeof(wss_user_data_t));
    }
    
    s_wss_db_env.num_records = 0;

    status = nvds_put(WSS_DB_ENV_TAG, sizeof(s_wss_db_env), (const uint8_t *)&s_wss_db_env);
    if (status)
    {
        ret = false;
        APP_LOG_ERROR("Clear database failed.");
    }

    return ret;
}

void wss_db_env_save()
{
    uint8_t status;

    status = nvds_put(WSS_DB_ENV_TAG, sizeof(wss_db_env_t), (const uint8_t *)&s_wss_db_env);
    if (status)
    {
        APP_LOG_DEBUG("Save the database enviorment params failed.");
    }
}

void user_data_update_schedule()
{
    while (!wss_db_ucp_cmd_queue_is_empty(&s_wss_db_rec_cmd_queue))
    {
        memcpy(&s_wss_db_env.user_buf, &s_wss_db_user_buf, sizeof(wss_db_user_buf_t));

        wss_db_rec_cmd_type_t ucp_cmd;
        wss_db_ucp_cmd_queue_elem_pop(&s_wss_db_rec_cmd_queue, &ucp_cmd);
        
        switch (ucp_cmd)
        {
            case UCP_CMD_TYPE_READ_REC:
                wss_db_user_data_get(s_wss_db_user_buf.user_id);
                break;
            
            case UCP_CMD_TYPE_WRITE_REC:
                wss_db_user_data_set(s_wss_db_user_buf.user_id);
                break;
            
            case UCP_CMD_TYPE_ADD_REC:
                wss_db_record_add(s_wss_db_user_buf.user_id);
                break;
            
            case UCP_CMD_TYPE_DEL_REC:
                wss_db_record_delete(s_wss_db_user_buf.user_id);
                break;
            
            case UCP_CMD_TYPE_CLEAR_REC:
                wss_db_record_clear();
                break;
                
            default:
                break;
        }
        wss_db_env_save();
    }
}
