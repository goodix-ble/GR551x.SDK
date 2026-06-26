/**
 ****************************************************************************************
 *
 * @file app_fds.h
 *
 * @brief App flash data storage.h
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

#ifndef __APP_FDS_H__
#define __APP_FDS_H__

#include "lfs.h"

/**
 * @defgroup APP_FDS_MAROC Defines
 * @{
 */
#define APP_FDS_KEY_STRING_TYPE     0    /**< 1: String type, 0: Int type. */

/**
 * @defgroup APP_FDS_ERROR_CODE Possible error codes
 * @{
 */
#define APP_FDS_ERR_OK             0   /**< No error. */
#define APP_FDS_ERR_NO_ENTRY      -2   /**< No entry. */
#define APP_FDS_ERR_IO            -5   /**< Error during device operation. */
#define APP_FDS_ERR_NO_MEM        -12  /**< No more memory available. */
#define APP_FDS_ERR_EXIST         -17  /**< Entry already exists . */
#define APP_FDS_ERR_INVAL         -22  /**< Invalid parameter. */
#define APP_FDS_ERR_VALUE_BIG     -27  /**< Value length too large. */
#define APP_FDS_ERR_NO_SPACE      -28  /**< No space left on device. */
#define APP_FDS_ERR_CORRUPT       -84  /**< Corrupted. */
/** @} */


/** @} */


/**
 * @defgroup APP_FDS_TYPEDEF Typedefs
 * @{
 */
/**@brief Key-value fds memory read function type.*/
typedef int (*app_fds_read_t)(uint32_t addr, uint8_t *p_buffer, uint32_t size);

/**@brief Key-value fds memory write function type.*/
typedef int (*app_fds_write_t)(uint32_t addr, uint8_t *p_buffer, uint32_t size);

/**@brief Key-value fds memory erase function type.*/
typedef int (*app_fds_erase_t)(uint32_t addr,uint32_t size);

/**@brief Key-value fds memory option sync function type.*/
typedef int (*app_fds_sync_t)(void);

/**@brief Key-value fds lock function type.*/
typedef int (*app_fds_lock_t)(void);

/**@brief Key-value fds unlock function type.*/
typedef int (*app_fds_unlock_t)(void);

/**@brief Key-value fds traverse callback type.*/
typedef void (*app_fds_traverse_cb_t)(
#if  APP_FDS_KEY_STRING_TYPE
                                           char *p_key,
#else
                                           uint32_t key,
#endif
                                           uint32_t length,
                                           bool* is_continue);

/** @} */

/**
 * @defgroup APP_FDS_STRUCT Structures
 * @{
 */

/**@brief Key-value fds config. */
typedef struct
{
    app_fds_read_t  fds_read;         /**< Key-value fds read function. */
    app_fds_write_t fds_write;        /**< Key-value fds write function. */
    app_fds_erase_t fds_erase;        /**< Key-value fds erase function. */
    app_fds_sync_t  fds_sync;         /**< Key-value fds op sync function. */
    app_fds_lock_t  fds_lock;         /**< Key-value fds lock function. */
    app_fds_lock_t  fds_unlock;       /**< Key-value fds unlock function. */
    uint32_t        fds_start_addr;   /**< Key-value fds start address. */
    uint32_t        fds_4k_block_cnt; /**< Key-value fds block count. */
} app_fds_config_t;
/** @} */

/**
 * @defgroup APP_FDS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Key-value fds module init.
 *
 * @param[in] fds_start_addr:     FDS Flash start address.
 * @param[in] fds_4k_block_cnt:   FDS Flash 4k block number.
 *
 * @return Result of operation.
 *****************************************************************************************
 */
int app_fds_init(uint32_t fds_start_addr, uint32_t fds_4k_block_cnt);

/**
 *****************************************************************************************
 * @brief Key-value fds write.
 *
 * @param[in] p_key/key:  Value key.
 * @param[in] p_value:    Pointer to value.
 * @param[in] length:     Length of value.
 *
 * @return Number of bytes write, or a negative error code on failure.
 *****************************************************************************************
 */
int app_fds_value_write(
#if  APP_FDS_KEY_STRING_TYPE
                              char *p_key,
#else
                              uint32_t key,
#endif
                              const void *p_value,
                              uint32_t length);

/**
 *****************************************************************************************
 * @brief Key-value fds read.
 *
 * @param[in] p_key/key:  Value key.
 * @param[in] p_value:    Pointer to buffer.
 * @param[in] length:     Length of buffer.
 *
 * @return Number of bytes read, or a negative error code on failure.
 *****************************************************************************************
 */
int app_fds_value_read(
#if  APP_FDS_KEY_STRING_TYPE
                             char *p_key,
#else
                             uint32_t key,
#endif
                             void *p_buffer,
                             uint32_t length);

/**
 *****************************************************************************************
 * @brief Key-value fds delete.
 *
 * @param[in] p_key/key:  Value key.
 *
 * @return Result of operation.
 *****************************************************************************************
 */
int app_fds_value_delete(
#if  APP_FDS_KEY_STRING_TYPE
                               char *p_key);
#else
                               uint32_t key);
#endif

/**
 *****************************************************************************************
 * @brief Key-value fds traverse.
 *
 * @param[in] p_instance:  Pointer to key-value fds instance, @note use deafult instance.
 * @param[in] traverse_cb: Callback of traverse.
 *
 * @return Result of operation.
 *****************************************************************************************
 */
int app_fds_traverse(app_fds_traverse_cb_t traverse_cb);

/** @} */

#endif
/** @} */
/** @} */

