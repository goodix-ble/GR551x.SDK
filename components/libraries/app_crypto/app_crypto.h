/**
 ****************************************************************************************
 *
 * @file app_crypto.h
 *
 * @brief App Crypto API
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


#ifndef __APP_CRYPTO_H__
#define __APP_CRYPTO_H__

#include "app_crypto_ecc.h"
#include "app_crypto_sha256.h"
#include "app_crypto_hmac.h"
#include "app_crypto_aes128.h"

/**
 * @defgroup APP_CRYPTO_MAROC Defines
 * @{
 */
/**
 * @defgroup APP_CRYPTO_RET_CODE Return Code of APIs
 * @{
 * @brief Return Code of APIs.
 */
#define AC_SUCCESS                   0
#define AC_ERR_INVALID_PARAM        -1
#define AC_ERR_INVALID_TYPE         -2
#define AC_ERR_AUTH_FAIL            -3
#define AC_ERR_INTERNAL             -99
/** @} */

/** @} */

/**
 * @defgroup APP_CRYPTO_TYPEDEF Typedefs
 * @{
 */
/**@brief  Random Number Generate Function Type. */
typedef int (*ac_rng_func_t)(uint8_t *dest, uint32_t size);
/** @} */


 /**
 * @defgroup APP_CRYPTO_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Init generate random number function into app crypto module.
 *
 * @param[in] rng_func: Pointer to rng function.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
int ac_rng_func_init(ac_rng_func_t rng_func);
/** @} */
#endif
