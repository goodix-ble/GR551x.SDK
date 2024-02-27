/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2022 GOODIX
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
 *****************************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "app_io.h"
#include "app_gpiote.h"
#include "grx_hal.h"
#include "board_SK.h"

#include "mbedtls/cipher.h"
#include "mbedtls/aes.h"
#include "mbedtls/gcm.h"

/*
 * VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/* Plain Data */
uint8_t plain_data[64] = {
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
};

/* Private Key */
uint8_t key_128[16] = {
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
};

uint8_t key_192[24] = {
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
};

uint8_t key_256[32] = {
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
};

/* Intialization Vector */
uint8_t iv[16] = {
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
};

uint8_t encrypt_buf[128] = {0};
uint8_t decrypt_buf[128] = {0};

/*
 * DEFINES
 *****************************************************************************************
 */
/* testing for multiple blocks */
//#define CBC_MULT_BLOCK
//#define CTR_MULT_BLOCK
//#define CFB_MULT_BLOCK
//#define OFB_MULT_BLOCK

/* testing for CBC padding mode (only enable one of them) */
#define CBC_PADDING_NONE
//#define CBC_PADDING_PKCS7
//#define CBC_PADDING_ZEROS

/* testing for key size (only enable one of them) */
#define KEY_SIZE_128
//#define KEY_SIZE_192
//#define KEY_SIZE_256

/* AES GCM authenticated TAG length */
#define GCM_TAG_LEN 16

/* testing for AES GCM add data */
//#define GCM_ADD_DATA

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void mbedtls_aes_ecb_enc_test(void)
{
    int ret;
    int i;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_ECB);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_ECB);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_ENCRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. update cipher */
#ifdef HAS_AES_DRIVER
    ret = mbedtls_cipher_update(&ctx, plain_data, 64, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, plain_data, 16, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+16, 16, encrypt_buf+16, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+32, 16, encrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+48, 16, encrypt_buf+48, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;

    /* 7. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, encrypt_buf+olen, &len);
    if (ret != 0) {
        goto exit;
    }
    olen += len;
#endif

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    for(i=0;i<olen;i++)
    {
     printf("%02X ",encrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 8. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_ecb_dec_test(void)
{
    int ret;
    int i;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_ECB);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_ECB);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_DECRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. update cipher */
#ifdef HAS_AES_DRIVER
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 64, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 16, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+16, 16, decrypt_buf+16, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+32, 16, decrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+48, 16, decrypt_buf+48, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 7. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, decrypt_buf+olen, &len);
    if (ret != 0) {
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    for(i=0;i<olen;i++)
    {
     printf("%02X ",decrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 8. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_cbc_enc_test(void)
{
    int ret;
    int i;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CBC);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_CBC);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CBC);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
#ifdef CBC_PADDING_NONE
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );
#endif
#ifdef CBC_PADDING_PKCS7
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_PKCS7 );
#endif
#ifdef CBC_PADDING_ZEROS
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_ZEROS );
#endif

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_ENCRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set iv */
    ret = mbedtls_cipher_set_iv(&ctx, iv, sizeof(iv));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef CBC_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, plain_data, 32, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+32, 32, encrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, plain_data, 64, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, encrypt_buf+olen, &len);
    if (ret != 0) {
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    for(i=0;i<olen;i++)
    {
     printf("%02X ",encrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_cbc_dec_test(void)
{
    int ret;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;
    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CBC);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_CBC);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CBC);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_DECRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set iv */
    ret = mbedtls_cipher_set_iv(&ctx, iv, sizeof(iv));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef CBC_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 32, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+32, 32, decrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 64, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
//    ret = mbedtls_cipher_finish(&ctx, decrypt_buf+olen, &len);
//    if (ret != 0) {
//        printf("mbedtls_cipher_finish error\r\n");
//        goto exit;
//    }
//    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    int i;
    for(i=0;i<olen;i++)
    {
     printf("%02X ",decrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_ctr_enc_test(void)
{
    int ret;
    int i;
    size_t len;
    int olen = 0;
    uint8_t nonce_counter[16] = {0};
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CTR);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_CTR);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CTR);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_ENCRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set nonce_counter */
    ret = mbedtls_cipher_set_iv(&ctx, nonce_counter, sizeof(nonce_counter));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef CTR_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, plain_data, 32, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+32, 32, encrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, plain_data, 64, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, encrypt_buf+olen, &len);
    if (ret != 0) {
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    for(i=0;i<olen;i++)
    {
     printf("%02X ",encrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_ctr_dec_test(void)
{
    int ret;
    size_t len;
    int olen = 0;
    uint8_t nonce_counter[16] = {0};
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;
    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CTR);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_CTR);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CTR);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_DECRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set nonce_counter */
    ret = mbedtls_cipher_set_iv(&ctx, nonce_counter, sizeof(nonce_counter));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef CTR_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 32, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+32, 32, decrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 64, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, decrypt_buf+olen, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_finish error\r\n");
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    int i;
    for(i=0;i<olen;i++)
    {
     printf("%02X ",decrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_cfb128_enc_test(void)
{
    int ret;
    int i;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CFB128);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_CFB128);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CFB128);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_ENCRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set iv */
    ret = mbedtls_cipher_set_iv(&ctx, iv, sizeof(iv));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef CFB_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, plain_data, 32, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+32, 32, encrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, plain_data, 64, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, encrypt_buf+olen, &len);
    if (ret != 0) {
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    for(i=0;i<olen;i++)
    {
     printf("%02X ",encrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_cfb128_dec_test(void)
{
    int ret;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;
    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CFB128);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_CFB128);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CFB128);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_DECRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set iv */
    ret = mbedtls_cipher_set_iv(&ctx, iv, sizeof(iv));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef CFB_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 32, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+32, 32, decrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 64, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, decrypt_buf+olen, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_finish error\r\n");
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    int i;
    for(i=0;i<olen;i++)
    {
     printf("%02X ",decrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_ofb_enc_test(void)
{
    int ret;
    int i;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_OFB);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_OFB);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_OFB);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_ENCRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set iv */
    ret = mbedtls_cipher_set_iv(&ctx, iv, sizeof(iv));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef OFB_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, plain_data, 32, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, plain_data+32, 32, encrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, plain_data, 64, encrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, encrypt_buf+olen, &len);
    if (ret != 0) {
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    for(i=0;i<olen;i++)
    {
     printf("%02X ",encrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_ofb_dec_test(void)
{
    int ret;
    size_t len;
    int olen = 0;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;
    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_OFB);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_OFB);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_OFB);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set padding mode */
    mbedtls_cipher_set_padding_mode( &ctx, MBEDTLS_PADDING_NONE );

    /* 5. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_DECRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 6. set iv */
    ret = mbedtls_cipher_set_iv(&ctx, iv, sizeof(iv));
    if (ret != 0) {
        goto exit;
    }

    /* 7. update cipher */
#ifdef OFB_MULT_BLOCK
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 32, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
    ret = mbedtls_cipher_update(&ctx, encrypt_buf+32, 32, decrypt_buf+32, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#else
    ret = mbedtls_cipher_update(&ctx, encrypt_buf, 64, decrypt_buf, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_update error\r\n");
        goto exit;
    }
    olen += len;
#endif

    /* 8. finish cipher */
    ret = mbedtls_cipher_finish(&ctx, decrypt_buf+olen, &len);
    if (ret != 0) {
        printf("mbedtls_cipher_finish error\r\n");
        goto exit;
    }
    olen += len;

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    printf("olen = %d\r\n", olen);
    int i;
    for(i=0;i<olen;i++)
    {
     printf("%02X ",decrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 9. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_gcm_enc_test(void)
{
    int ret;
    int i;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;
#ifdef GCM_ADD_DATA
    uint8_t add[32] = {
        'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
        'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    };
#endif

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_GCM);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_GCM);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_GCM);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_ENCRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_ENCRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 5. gcm encryption */
#ifdef GCM_ADD_DATA
    ret = mbedtls_gcm_crypt_and_tag( ctx.cipher_ctx, MBEDTLS_GCM_ENCRYPT, 64, iv, sizeof(iv), add, sizeof(add), plain_data, encrypt_buf, GCM_TAG_LEN, encrypt_buf+64 );
#else
    ret = mbedtls_gcm_crypt_and_tag( ctx.cipher_ctx, MBEDTLS_GCM_ENCRYPT, 64, iv, sizeof(iv), NULL, 0, plain_data, encrypt_buf, GCM_TAG_LEN, encrypt_buf+64 );
#endif
    if (ret != 0) {
        printf("mbedtls_gcm_crypt_and_tag error\r\n");
        goto exit;
    }

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    for(i=0;i<(64+GCM_TAG_LEN);i++)
    {
     printf("%02X ",encrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 6. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

void mbedtls_aes_gcm_dec_test(void)
{
    int ret;
    mbedtls_cipher_context_t ctx;
    const mbedtls_cipher_info_t *info;
#ifdef GCM_ADD_DATA
    uint8_t add[32] = {
        'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
        'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    };
#endif

    /* 1. init cipher structuer */
    mbedtls_cipher_init(&ctx);

    /* 2. get info structuer from type */
#ifdef KEY_SIZE_128
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_GCM);
#endif
#ifdef KEY_SIZE_192
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_192_GCM);
#endif
#ifdef KEY_SIZE_256
    info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_GCM);
#endif

    /* 3. setup cipher structuer */
    ret = mbedtls_cipher_setup(&ctx, info);
    if (ret != 0) {
        printf("mbedtls_cipher_setup error\r\n");
        goto exit;
    }

    /* 4. set key */
#ifdef KEY_SIZE_128
    ret = mbedtls_cipher_setkey(&ctx, key_128, sizeof(key_128) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_192
    ret = mbedtls_cipher_setkey(&ctx, key_192, sizeof(key_192) * 8, MBEDTLS_DECRYPT);
#endif
#ifdef KEY_SIZE_256
    ret = mbedtls_cipher_setkey(&ctx, key_256, sizeof(key_256) * 8, MBEDTLS_DECRYPT);
#endif
    if (ret != 0) {
        printf("mbedtls_cipher_setkey error\r\n");
        goto exit;
    }

    /* 5. gcm encryption */
#ifdef GCM_ADD_DATA
    ret = mbedtls_gcm_auth_decrypt( ctx.cipher_ctx, 64, iv, sizeof(iv), add, sizeof(add), encrypt_buf+64, GCM_TAG_LEN, encrypt_buf, decrypt_buf );
#else
    ret = mbedtls_gcm_auth_decrypt( ctx.cipher_ctx, 64, iv, sizeof(iv), NULL, 0, encrypt_buf+64, GCM_TAG_LEN, encrypt_buf, decrypt_buf );
#endif
    if (ret != 0) {
        printf("mbedtls_gcm_auth_decrypt error\r\n");
        goto exit;
    }

    /* show */
    printf("cipher name:%s block size is:%d\r\n", mbedtls_cipher_get_name(&ctx), mbedtls_cipher_get_block_size(&ctx));
    int i;
    for(i=0;i<64;i++)
    {
     printf("%02X ",decrypt_buf[i]);
    }
    printf("\r\n");

exit:
    /* 6. free cipher structure */
    mbedtls_cipher_free(&ctx);
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                Crypto AES example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how AES work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    mbedtls_aes_ecb_enc_test();
    mbedtls_aes_ecb_dec_test();
    mbedtls_aes_cbc_enc_test();
    mbedtls_aes_cbc_dec_test();
    mbedtls_aes_ctr_enc_test();
    mbedtls_aes_ctr_dec_test();
    mbedtls_aes_cfb128_enc_test();
    mbedtls_aes_cfb128_dec_test();
    mbedtls_aes_ofb_enc_test();
    mbedtls_aes_ofb_dec_test();
    mbedtls_aes_gcm_enc_test();
    mbedtls_aes_gcm_dec_test();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
