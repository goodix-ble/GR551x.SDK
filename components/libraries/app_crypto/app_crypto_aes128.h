/**
 ****************************************************************************************
 *
 * @file app_crypto_aes.h
 *
 * @brief App Crypto AES API
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
#ifndef __APP_CRYPTO_AES_H__
#define __APP_CRYPTO_AES_H__

#include <stdint.h>

/**
 * @defgroup APP_CRYPTO_AES_MAROC Defines
 * @{
 */
#define AC_AES_BLOCK_SIZE       16
/** @} */


 /**
 * @defgroup APP_CRYPTO_AES_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Set AES-128 encryption key.
 *
 * @param[in]  key:  Pointer to AES key.
 *
 * @return Result of set.
 *****************************************************************************************
 */
int ac_aes128_key_set(const uint8_t *key);

/**
 *****************************************************************************************
 * @brief AES-128 ECB Encryption procedure.
 *
 * @param[in]  in:     Pointer to  plaintext to encrypt.
 * @param[in]  inlen:  Length of plaintext to encrypt.
 * @param[out] out:    Pointer to receive ciphertext buffer.
 * @param[in]  outlen: Length of receive ciphertext buffer.
 *
 * @return Result of size.
 *****************************************************************************************
 */
int ac_aes128_ecb_encrypt(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen);

/**
 *****************************************************************************************
 * @brief AES-128 ECB Decryption procedure.
 *
 * @param[in]  in:     Pointer to  ciphertext to decrypt.
 * @param[in]  inlen:  Length of ciphertext to decrypt.
 * @param[out] out:    Pointer to receive plaintext buffer.
 * @param[in]  outlen: Length of receive plaintext buffer.
 * 
 * @return Result of size.
 *****************************************************************************************
 */
int ac_aes128_ecb_decrypt(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen);

/**
 *****************************************************************************************
 * @brief AES-128 CBC Encryption procedure.
 *
 * @param[in]  in:     Pointer to  plaintext to encrypt.
 * @param[in]  inlen:  Length of plaintext to encrypt.
 * @param[out] out:    Pointer to receive ciphertext buffer.
 * @param[in]  outlen: Length of receive ciphertext buffer.
 * @param[in]  iv:     Pointer of IV for the this encrypt.
 *
 * @return Result of size.
 *****************************************************************************************
 */
int ac_aes128_cbc_encrypt(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen, const uint8_t *iv);

/**
 *****************************************************************************************
 * @brief AES-128 CBC Decryption procedure.
 *
 * @param[in]  in:     Pointer to  plaintext to decrypt.
 * @param[in]  inlen:  Length of plaintext to decrypt.
 * @param[out] out:    Pointer to receive ciphertext buffer.
 * @param[in]  outlen: Length of receive ciphertext buffer.
 * @param[in]  iv:     Pointer of IV for the this decrypt.
 *
 * @return Result of size.
 *****************************************************************************************
 */
int ac_aes128_cbc_decrypt(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen, const uint8_t *iv);

/**
 *****************************************************************************************
 * @brief AES-128 CTR procedure.
 *
 * @param[in]  in:     Pointer to ciphertext (plaintext).
 * @param[in]  inlen:  Length of ciphertext (plaintext).
 * @param[out] out:    Pointer to receive plaintext (ciphertext) buffer.
 * @param[in]  outlen: Length of receive plaintext (ciphertext) buffer.
 * @param[in]  ctr:    Pointer of current counter value.
 *
 * @return Result of size.
 *****************************************************************************************
 */
int ac_aes128_ctr(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen, uint8_t *ctr);
/** @} */

#endif
