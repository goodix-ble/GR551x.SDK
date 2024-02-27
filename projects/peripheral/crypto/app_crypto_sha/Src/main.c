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

#include "mbedtls/md.h"

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

uint8_t hmac_key[32] = {
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
    'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D',
};

/*
 * DEFINES
 *****************************************************************************************
 */
/* testing for SHA multiple blocks */
//#define SHA_MULT_BLOCK

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void mbedtls_sha_test(void)
{
    int ret = 0;
    int i;
    uint8_t outbuf[32] = {0};
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t *info;

    /* 1. init md structuer */
    mbedtls_md_init(&ctx);

    /* 2. get info structuer from type */
    info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    /* 3. setup md info structure */
    ret = mbedtls_md_setup(&ctx, info, 0);
    if (ret != 0) {
        printf("mbedtls_md_setup error\r\n");
        goto exit;
    }

    /* 4. start */
    ret = mbedtls_md_starts(&ctx);
     if (ret != 0) {
         printf("mbedtls_md_starts error\r\n");
        goto exit;
    }

    /* 5. update */
#ifdef SHA_MULT_BLOCK
    ret = mbedtls_md_update(&ctx, plain_data, 32);
    if (ret != 0) {
        printf("mbedtls_md_update error\r\n");
        goto exit;
    }
    ret = mbedtls_md_update(&ctx, plain_data+32, 32);
    if (ret != 0) {
        printf("mbedtls_md_update error\r\n");
        goto exit;
    }
#else
    ret = mbedtls_md_update(&ctx, plain_data, sizeof(plain_data));
    if (ret != 0) {
        printf("mbedtls_md_update error\r\n");
        goto exit;
    }
#endif

    /* 6. finish */
    ret = mbedtls_md_finish(&ctx, outbuf);
    if (ret != 0) {
        printf("mbedtls_md_finish error\r\n");
        goto exit;
    }

    /* show */
    printf("SHA-256 (len = %d):\r\n", 32);
    for(i=0;i<32;i++)
    {
        printf("%02X ", outbuf[i]);
    }
    printf("\r\n");
exit:
    /* 7. free */
    mbedtls_md_free(&ctx);
}

void mbedtls_sha_test_add(void)
{
    int ret = 0;
    int i;
    uint8_t outbuf[32] = {0};
    const mbedtls_md_info_t *info;
    info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    ret = mbedtls_md( info, plain_data, sizeof(plain_data), outbuf );
    if (ret != 0)
    {
        printf("mbedtls_md error\r\n");
        return;
    }

    printf("SHA-256 (len = %d):\r\n", 32);
    for(i=0;i<32;i++)
    {
        printf("%02X ", outbuf[i]);
    }
    printf("\r\n");
}

void mbedtls_hmac_test(void)
{
    int ret = 0;
    int i;
    uint8_t outbuf[32] = {0};
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t *info;

    /* 1. init md structuer */
    mbedtls_md_init(&ctx);

    /* 2. get info structuer from type */
    info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    /* 3. setup md info structure */
    ret = mbedtls_md_setup(&ctx, info, 1);
    if (ret != 0) {
        printf("mbedtls_md_setup error\r\n");
        goto exit;
    }

    /* 4. start */
    ret = mbedtls_md_hmac_starts(&ctx, hmac_key, sizeof(hmac_key));
     if (ret != 0) {
         printf("mbedtls_md_hmac_starts error\r\n");
        goto exit;
    }

    /* 5. update */
#ifdef SHA_MULT_BLOCK
    ret = mbedtls_md_hmac_update(&ctx, plain_data, 32);
    if (ret != 0) {
        printf("mbedtls_md_hmac_update error\r\n");
        goto exit;
    }
    ret = mbedtls_md_hmac_update(&ctx, plain_data+32, 32);
    if (ret != 0) {
        printf("mbedtls_md_hmac_update error\r\n");
        goto exit;
    }
#else
    ret = mbedtls_md_hmac_update(&ctx, plain_data, sizeof(plain_data));
    if (ret != 0) {
        printf("mbedtls_md_hmac_update error\r\n");
        goto exit;
    }
#endif

    /* 6. finish */
    ret = mbedtls_md_hmac_finish(&ctx, outbuf);
    if (ret != 0) {
        printf("mbedtls_md_hmac_finish error\r\n");
        goto exit;
    }

    /* show */
    printf("HMAC-SHA256 (len = %d):\r\n", 32);
    for(i=0;i<32;i++)
    {
        printf("%02X ", outbuf[i]);
    }
    printf("\r\n");
exit:
    /* 7. free */
    mbedtls_md_free(&ctx);
}

void mbedtls_hmac_test_add(void)
{
    int ret = 0;
    int i;
    uint8_t outbuf[32] = {0};
    const mbedtls_md_info_t *info;
    info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    ret = mbedtls_md_hmac( info, hmac_key, sizeof(hmac_key), plain_data, sizeof(plain_data), outbuf );
    if (ret != 0)
    {
        printf("mbedtls_md_hmac error\r\n");
        return;
    }

    printf("HMAC-SHA256 (len = %d):\r\n", 32);
    for(i=0;i<32;i++)
    {
        printf("%02X ", outbuf[i]);
    }
    printf("\r\n");
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                Crypto SHA example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how SHA work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    mbedtls_sha_test();
    mbedtls_sha_test_add();
    mbedtls_hmac_test();
    mbedtls_hmac_test_add();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
