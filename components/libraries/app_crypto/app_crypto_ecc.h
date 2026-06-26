/**
 ****************************************************************************************
 *
 * @file app_crypto_ecc.h
 *
 * @brief App Crypto ECC API
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
#ifndef __APP_CRYPTO_ECC_H__
#define __APP_CRYPTO_ECC_H__

#include <stdint.h>

/**
 * @defgroup APP_CRYPTO_ECC_MAROC Defines
 * @{
 */
#define AC_ECC_SECP160R1_PRI_KEY_SIZE       21
#define AC_ECC_SECP192R1_PRI_KEY_SIZE       24
#define AC_ECC_SECP224R1_PRI_KEY_SIZE       28
#define AC_ECC_SECP256R1_PRI_KEY_SIZE       32
#define AC_ECC_SECP256K1_PRI_KEY_SIZE       32

#define AC_ECC_SECP160R1_PUB_KEY_SIZE       40
#define AC_ECC_SECP192R1_PUB_KEY_SIZE       48
#define AC_ECC_SECP224R1_PUB_KEY_SIZE       56
#define AC_ECC_SECP256R1_PUB_KEY_SIZE       64
#define AC_ECC_SECP256K1_PUB_KEY_SIZE       64
/** @} */

/**
 * @defgroup APP_CRYPTO_ECC_ENUM Enumerations
 * @{
 */
/**@brief App Crypto ECC Curve Type. */
typedef enum
{
    AC_ECC_SECP160R1,
    AC_ECC_SECP192R1,
    AC_ECC_SECP224R1,
    AC_ECC_SECP256R1,
    AC_ECC_SECP256K1,
} ac_ecc_type_t;
/** @} */


 /**
 * @defgroup APP_CRYPTO_ECC_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Generate a public/private key pair.
 *
 * @param[in]  curve_type:  ECC curve_type.
 * @param[out] public_key:  Pointer to public key buffer, must be at least 2 * the curve size (in bytes) long.
 * @param[out] private_key: Pointer to private key buffer, same as the curve size, except for secp160r1 that must be 21 bytes.
 *
 * @return Result of make.
 *****************************************************************************************
 */
int ac_ecc_make_key(ac_ecc_type_t curve_type, uint8_t *public_key, uint8_t *private_key);

/**
 *****************************************************************************************
 * @brief Get size of private key.
 *
 * @param[in]  curve_type:  ECC curve_type.
 *
 * @return Result of size.
 *****************************************************************************************
 */
int ac_ecc_private_key_size_get(ac_ecc_type_t curve_type);

/**
 *****************************************************************************************
 * @brief Get size of public key.
 *
 * @param[in]  curve_type:  ECC curve_type.
 *
 * @return Result of size.
 *****************************************************************************************
 */
int ac_ecc_public_key_size_get(ac_ecc_type_t curve_type);

/**
 *****************************************************************************************
 * @brief Compress a public key.
 *
 * @param[in]  curve_type:  ECC curve_type.
 * @param[in]  public_key:  Pointer to public key.
 * @param[out] compressed:  Pointer to compressed public key, Must be at least (curve size + 1) bytes long.
 *
 * @return Result of compress.
 *****************************************************************************************
 */
int ac_ecc_public_key_compress(ac_ecc_type_t curve_type, uint8_t *public_key, uint8_t *compressed);

/**
 *****************************************************************************************
 * @brief Decompress a compressed public key.
 *
 * @param[in]  curve_type:  ECC curve_type.
 * @param[in]  compressed:  Pointer to compressed public key.
 * @param[out] public_key:  Pointer to decompressed public key.
 *
 * @return Result of decompress.
 *****************************************************************************************
 */
int ac_ecc_public_key_decompress(ac_ecc_type_t curve_type, uint8_t *compressed, uint8_t *public_key);

/**
 *****************************************************************************************
 * @brief Compute the corresponding public key for a private key.
 *
 * @param[in] curve_type:   ECC curve_type.
 * @param[in] private_key:  Pointer to private key.
 * @param[out] public_key:  Pointer to private key.
 *
 * @return Result of check.
 *****************************************************************************************
 */
int ac_ecc_public_key_compute(ac_ecc_type_t curve_type, const uint8_t *private_key, uint8_t *public_key);

/**
 *****************************************************************************************
 * @brief Check one public.
 *
 * @param[in] curve_type:  ECC curve_type.
 * @param[in] public_key:  Pointer to public key.
 *
 * @return Result of check.
 *****************************************************************************************
 */
int ac_ecc_public_key_check(ac_ecc_type_t curve_type, uint8_t *public_key);

/**
 *****************************************************************************************
 * @brief Generate an ECDSA signature for a given hash value.
 *
 * @param[in] curve_type:   ECC curve_type.
 * @param[in] private_key:  Pointer to private key.
 * @param[in] message_hash: Pointer to hash of the message to sign.
 * @param[in] hash_size:    Size of hash of the message to sign.
 * @param[out] signature:   Pointer to signature value.
 *
 * @return Result of sign.
 *****************************************************************************************
 */
int ac_ecc_ecdsa_sign(ac_ecc_type_t curve_type, const uint8_t *private_key, const uint8_t *message_hash, uint16_t hash_size, uint8_t *signature);

/**
 *****************************************************************************************
 * @brief Verify an ECDSA signature.
 *
 * @param[in] curve_type:   ECC curve_type.
 * @param[in] public_key:   Pointer to public key.
 * @param[in] message_hash: Pointer to hash of the message to sign.
 * @param[in] hash_size:    Size of hash of the message to sign.
 * @param[in] signature:    Pointer to signature value.
 *
 * @return Result of verify.
 *****************************************************************************************
 */
int ac_ecc_ecdsa_verify(ac_ecc_type_t curve_type, const uint8_t *public_key, const uint8_t *message_hash, uint16_t hash_size, uint8_t *signature);

/**
 *****************************************************************************************
 * @brief Compute shared secret.
 *
 * @param[in]  curve_type:  ECC curve_type.
 * @param[in]  public_key:  Pointer to remote public key.
 * @param[in]  private_key: Pointer to self private key.
 * @param[out] secret:      Shared secret value.
 *
 * @return Result of compute.
 *****************************************************************************************
 */
int ac_ecc_ecdh_shared_secret_compute(ac_ecc_type_t curve_type, uint8_t *public_key, uint8_t *private_key, uint8_t *secret);

/**
 *****************************************************************************************
 * @brief Multiplies a point by a scalar.
 *
 * @param[in]  curve_type:  ECC curve_type.
 * @param[out] result:      Pointer to result.
 * @param[in]  point:       Pointer to point.
 * @param[in] scalar:       Pointer to scalar.
 *
 * @return Result of compute.
 *****************************************************************************************
 */   
int ac_ecc_point_mult( ac_ecc_type_t curve_type, uint32_t *result, const uint32_t *point, const uint32_t *scalar);

/**
 *****************************************************************************************
 * @brief Computes result = product % mod, where product is 2N words long. Currently only designed to work for mod == curve->p or curve_n.
 *
 * @param[in]  curve_type:  ECC curve_type.
 * @param[out] result:      Pointer to result.
 * @param[in]  product:     Pointer to product.
 * @param[in]  mod:         Pointer to mod.
 * @param[in]  num_words:   Number of words.
 *
 * @return Result of compute.
 *****************************************************************************************
 */   
int ac_ecc_mmod(uint32_t *result, uint32_t *product, const uint32_t *mod, uint8_t num_words);

/**
 *****************************************************************************************
 * @brief Converts an integer in native format to big-endian bytes.
 *
 * @param[out] bytes:       Output data.
 * @param[in]  num_bytes:   Num bytes.
 * @param[in]  native:      native format data.
 *****************************************************************************************
 */
void ac_ecc_native_to_bytes(uint8_t *bytes, int num_bytes, const uint32_t *native);

/**
 *****************************************************************************************
 * @brief Converts an integer in native format to big-endian bytes.
 *
 * @param[out] native:      native format data.
 * @param[in]  bytes:       Input data.
 * @param[in]  num_bytes:   Num bytes.
 *****************************************************************************************
 */
void ac_ecc_bytes_to_native(uint32_t *native, const uint8_t *bytes, int num_bytes);

/** @} */

#endif
