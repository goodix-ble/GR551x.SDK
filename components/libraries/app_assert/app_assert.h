/**
 ****************************************************************************************
 *
 * @file app_assert.h
 *
 * @brief App Assert API
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

#ifndef __APP_ASSERT_H__
#define __APP_ASSERT_H__

#include "grx_sys.h"
#include <stdint.h>

/**
 * @defgroup APP_ASSERT_MAROC Defines
 * @{
 */
/**@brief Macro for calling error handler function if assert check failed. */
#define APP_ASSERT_CHECK(EXPR)                              \
    do                                                      \
    {                                                       \
        if (!(EXPR))                                        \
        {                                                   \
            app_assert_handler(#EXPR, __FILE__, __LINE__);  \
        }                                                   \
    } while(0)

/**@brief APP_ASSERT info length defines. */
#define APP_ASSERT_FILE_NAME_LEN             64             /**< Length of file name. Unit: byte. */
#define APP_ASSERT_FILE_LINE_LEN             4              /**< Length of file line. Unit: byte. */
#define APP_ASSERT_PARAM0_LEN                4              /**< Length of param0. Unit: byte. */
#define APP_ASSERT_PARAM1_LEN                4              /**< Length of param1. Unit: byte. */

/** @} */

/**
 * @defgroup APP_ASSERT_ENUM Enums
 * @{
 */
/**
  * @brief APP_ASSERT_ERROR error code.
  */
typedef enum
{
    APP_ASSERT_SUCCESS                   = 0x0,
    APP_ASSERT_NO_INFO                   = 0x1,
    APP_ASSERT_INSUFFICIENT_BUFFER_SPACE = 0x2
} app_assert_error_t;
/** @} */

/**
 * @defgroup APP_ASSERT_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Init user assert callbacks.
 *****************************************************************************************
 */
void app_assert_init(void);

/**
 *****************************************************************************************
 * @brief App assert handler.
 *
 * @param[in] expr:  Pxpression.
 * @param[in] file:  File name.
 * @param[in] line:  Line number.
 *****************************************************************************************
 */
void app_assert_handler(const char *expr, const char *file, int line);

/**
 *****************************************************************************************
 * @brief App assert warning callback.
 *
 * @param[in] param0:  Parameter0.
 * @param[in] param1:  Parameter1.
 * @param[in] file:    File name.
 * @param[in] line:    Line number.
 *****************************************************************************************
 */
void app_assert_warn_cb(int param0, int param1, const char *file, int line);

/**
 *****************************************************************************************
 * @brief App assert parameter callback.
 *
 * @param[in] param0:  Parameter0.
 * @param[in] param1:  Parameter1.
 * @param[in] file:    File name.
 * @param[in] line:    Line number.
 *****************************************************************************************
 */
void app_assert_param_cb(int param0, int param1, const char *file, int line);

/**
 *****************************************************************************************
 * @brief App assert error callback.
 *
 * @param[in] expr:   Pxpression.
 * @param[in] file:   File name.
 * @param[in] line:   Line number.
 *****************************************************************************************
 */
void app_assert_err_cb(const char *expr, const char *file, int line);

/**
 *****************************************************************************************
 * @brief Get assert information.
 *
 * @param[in] p_info:     A buffer that points to the storage of p_info.
 *                        Layout: APP_ASSERT_FILE_NAME_LEN bytes file_name + APP_ASSERT_FILE_LINE_LEN byte file_line + APP_ASSERT_PARAM0_LEN byte param0 + APP_ASSERT_PARAM1_LEN byte param1.
 * @param[in] info_len:   buffer length.
 *                        Note: info_len >= (APP_ASSERT_FILE_NAME_LEN + APP_ASSERT_FILE_LINE_LEN + APP_ASSERT_PARAM0_LEN + APP_ASSERT_PARAM1_LEN).
 *
 * @return Result of get assert info. @ref APP_ASSERT_ERROR.
 *****************************************************************************************
 */
app_assert_error_t app_assert_get_info(uint8_t *p_info, uint8_t info_len);

/** @} */

#endif 



