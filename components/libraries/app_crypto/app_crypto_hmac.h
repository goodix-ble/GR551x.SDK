/**
 ****************************************************************************************
 *
 * @file app_crypto_hmac.h
 *
 * @brief App Crypto HMAC API
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
#ifndef __APP_CRYPTO_HMAC_H__
#define __APP_CRYPTO_HMAC_H__

#include <stdint.h>

//MAC_SHA256

 /**
 * @defgroup APP_CRYPTO_HMAC_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief MAC_SHA256 initialization procedure.
 *
 * @param[in]  key:   Pointer to key.
 * @param[in]  size:  Size of key.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
int ac_hmac_init(const uint8_t *key, uint32_t size);

/**
 *****************************************************************************************
 * @brief MAC_SHA256 update procedure.
 *
 * @param[in]  p_data:  Pointer to data.
 * @param[in]  length:  Length of data.
 *
 * @return Result of update.
 *****************************************************************************************
 */
int ac_hmac_update(const uint8_t *p_data, uint32_t length);

/**
 *****************************************************************************************
 * @brief MAC_SHA256 finish procedure
 *
 * @param[out]  tag:     Pointer to tag.
 * @param[in]   length:  Length of tag.
 * 
 * @return Result of finish.
 *****************************************************************************************
 */
int ac_hmac_finish(uint8_t *tag, uint8_t length);
/** @} */

#endif
