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

#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/platform.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecdh.h"

/*
 * VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static int entropy_source(void *data, uint8_t *output, size_t len, size_t *olen)
{
    uint32_t rng32;
    rng_handle_t g_rng_handle;

    g_rng_handle.p_instance = RNG;
    g_rng_handle.init.seed_mode  = RNG_SEED_FR0_S0;
    g_rng_handle.init.lfsr_mode  = RNG_LFSR_MODE_59BIT;
    g_rng_handle.init.out_mode   = RNG_OUTPUT_FR0_S0;
    g_rng_handle.init.post_mode  = RNG_POST_PRO_NOT;
    hal_rng_deinit(&g_rng_handle);
    hal_rng_init(&g_rng_handle);

    hal_rng_generate_random_number(&g_rng_handle, NULL, &rng32);

    if (len > sizeof(rng32)) {
        len = sizeof(rng32);
    }
    memcpy(output, &rng32, len);
    *olen = len;

    hal_rng_deinit(&g_rng_handle);
    return 0;
}

void mbedtls_ecdh_test(void)
{
    int ret;
    const char *pers = "ecdh_test";
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ecp_point client_pub, server_pub;
    mbedtls_ecp_group grp;
    mbedtls_mpi client_secret, server_secret;
    mbedtls_mpi client_pri, server_pri;

    /* 1. init structure */
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_mpi_init(&client_secret);
    mbedtls_mpi_init(&server_secret);
    mbedtls_mpi_init(&client_pri);
    mbedtls_mpi_init(&server_pri);
    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&client_pub);
    mbedtls_ecp_point_init(&server_pub);

    /* 2. update seed with we own interface ported */
    printf( "\r\nSeeding the random number generator..." );

    mbedtls_entropy_add_source(&entropy, entropy_source, NULL,
                               MBEDTLS_ENTROPY_MAX_GATHER,
                               MBEDTLS_ENTROPY_SOURCE_STRONG);
    ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen(pers));
    if(ret != 0) {
        printf( "failed! mbedtls_ctr_drbg_seed returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( " ok\r\n" );

    /* 3. select ecp group SECP256R1 */
    printf("\r\nSelect ecp group SECP256R1...");

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if(ret != 0) {
        printf( "failed! mbedtls_ecp_group_load returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }

    printf("ok\r\n");

    /* 4. Client generate public parameter */
    printf("\r\nClient Generate public parameter...");

    ret = mbedtls_ecdh_gen_public(&grp, &client_pri, &client_pub, mbedtls_ctr_drbg_random, &ctr_drbg);
    if(ret != 0) {
        printf( "failed! mbedtls_ecdh_gen_public returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( " ok\r\n" );

    /* 5. Client generate public parameter */
    printf("\r\nServer Generate public parameter...");

    ret = mbedtls_ecdh_gen_public(&grp, &server_pri, &server_pub, mbedtls_ctr_drbg_random, &ctr_drbg);
    if(ret != 0) {
        printf( "failed! mbedtls_ecdh_gen_public returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( " ok\r\n" );

    /* 6. Calc shared secret */
    printf("\r\nClient Calc shared secret...");

    ret = mbedtls_ecdh_compute_shared(&grp, &client_secret, &server_pub, &client_pri, mbedtls_ctr_drbg_random, &ctr_drbg);
    if(ret != 0) {
        printf( "failed! mbedtls_ecdh_compute_shared returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( " ok\r\n" );

    /* 7. Server Calc shared secret */
    printf("\r\nServer Calc shared secret...");

    ret = mbedtls_ecdh_compute_shared(&grp, &server_secret, &client_pub, &server_pri, mbedtls_ctr_drbg_random, &ctr_drbg);
    if(ret != 0) {
        printf( "failed! mbedtls_ecdh_compute_shared returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( " ok\r\n" );

    /* 8. mpi compare */
    ret = mbedtls_mpi_cmp_mpi(&server_secret, &client_secret);
    if(ret == 0)
    {
        printf("\r\nECDH test pass\r\n");
    }
    else
    {
        printf("\r\nECDH test fail\r\n");
    }

exit:
    /* 9. release structure */
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
    mbedtls_mpi_free(&client_secret);
    mbedtls_mpi_free(&server_secret);
    mbedtls_mpi_free(&client_pri);
    mbedtls_mpi_free(&server_pri);
    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&client_pub);
    mbedtls_ecp_point_free(&server_pub);
    return;
}

void mbedtls_ecdsa_test(void)
{
    int ret = 0;
    int i;
    uint32_t slen = 0;
    uint8_t hash[32] = {
        0xf3, 0xae, 0xfe, 0x62, 0x96, 0x5a, 0x91, 0x90,
        0x36, 0x10, 0xf0, 0xe2, 0x3c, 0xc8, 0xa6, 0x9d,
        0x5b, 0x87, 0xce, 0xa6, 0xd2, 0x8e, 0x75, 0x48,
        0x9b, 0x0d, 0x2c, 0xa0, 0x2e, 0xd7, 0x99, 0x3c,
    };
    uint8_t private_key[32] = {
        0xe8, 0xdd, 0x64, 0xcb, 0xa5, 0x16, 0xeb, 0x6b,
        0x09, 0xb0, 0x3d, 0x70, 0x5f, 0x9f, 0x1f, 0xfb,
        0xaf, 0xea, 0xb6, 0xab, 0x24, 0xef, 0xee, 0x75,
        0x60, 0xa0, 0x11, 0x7d, 0xc9, 0x6d, 0x09, 0x78,
    };
    uint8_t public_key[65] = {
        0x04,
        0x1f, 0x59, 0xb0, 0x65, 0x04, 0x0e, 0x63, 0xdc,
        0x02, 0x4c, 0x6a, 0x19, 0x26, 0x1b, 0xc3, 0x9e,
        0x5b, 0xdc, 0x62, 0xb3, 0x25, 0xea, 0x15, 0x5a,
        0x54, 0xd5, 0x71, 0xfc, 0x2d, 0x15, 0x4b, 0x9b,
        0x1b, 0x86, 0x38, 0x99, 0x31, 0xe3, 0xe7, 0x98,
        0x5e, 0x32, 0x16, 0x57, 0x97, 0x0f, 0xab, 0x7f,
        0x12, 0x21, 0x70, 0x86, 0x61, 0x8a, 0x48, 0xd2,
        0x24, 0xd5, 0x73, 0x94, 0x82, 0x74, 0xc4, 0x72,
    };
    uint8_t signature[MBEDTLS_ECDSA_MAX_LEN] = {0};
    mbedtls_ecdsa_context ctx;
    mbedtls_mpi r, s;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    const char *pers = "ecdsa_test";

    /* 1. init structure */
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    mbedtls_ecdsa_init(&ctx);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);

    /* 2. update seed with we own interface ported */
    mbedtls_entropy_add_source(&entropy, entropy_source, NULL,
                               MBEDTLS_ENTROPY_MAX_GATHER,
                               MBEDTLS_ENTROPY_SOURCE_STRONG);
    ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen(pers));
    if(ret != 0) {
        printf( " failed\n  ! mbedtls_ctr_drbg_seed returned %d(-0x%04x)\n", ret, -ret);
        goto exit;
    }

    /* 3. select ecp group SECP256R1 */
    ret = mbedtls_ecp_group_load( &ctx.grp, MBEDTLS_ECP_DP_SECP256R1 );
    if(ret != 0)
    {
        printf( " failed\n  ! mbedtls_ecp_group_load returned %d\r\n", ret );
        goto exit;
    }

    /* 4. load keypair */
    ret = mbedtls_mpi_read_binary( &ctx.d, private_key, sizeof private_key );
    if( ret != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_mpi_read_binary returned %d\n", ret );
        goto exit;
    }
    ret = mbedtls_ecp_point_read_binary( &ctx.grp, &ctx.Q, public_key, sizeof public_key );
    if( ret != 0 )
    {
        printf( " failed\n  ! mbedtls_ecp_point_read_binary returned %d\r\n", ret );
        goto exit;
    }

    /* 4. sign */
    ret = mbedtls_ecdsa_write_signature( &ctx, MBEDTLS_MD_NONE, hash, sizeof hash, signature, &slen, mbedtls_ctr_drbg_random, &ctr_drbg );
    if(ret != 0)
    {
        printf( " failed\n  ! mbedtls_ecdsa_write_signature returned %d\r\n", ret );
        goto exit;
    }

    /* 4. show */
    printf("\r\nECDSA signature in ASN.1:\r\n");
    for(i=0;i<slen;i++)
    {
     printf("%02X ",signature[i]);
    }
    printf("\r\n");

    /* 5. verify */
    ret = mbedtls_ecdsa_read_signature( &ctx, hash, sizeof hash, signature, slen );
    if(ret != 0)
    {
        printf( " failed\n  ! mbedtls_ecdsa_read_signature returned %d\r\n", ret );
        goto exit;
    }
    printf("\r\nECDSA test pass\r\n");

exit:
    /* 6. release structure */
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
    mbedtls_ecdsa_free(&ctx);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
    return;
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                Crypto ECC example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how ECC work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    mbedtls_ecdh_test();
    mbedtls_ecdsa_test();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
