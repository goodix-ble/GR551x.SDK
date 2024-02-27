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
#include "mbedtls/rsa.h"
#include "mbedtls/pk.h"

/*
 * VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t hash[32] = {
    0xe8, 0xbc, 0xcc, 0x22, 0x4b, 0x5f, 0xda, 0x5a,
    0x8d, 0x5f, 0x81, 0x97, 0x1f, 0x12, 0x4b, 0xb1,
    0x8b, 0x36, 0x91, 0xdf, 0xec, 0x68, 0xe6, 0x22,
    0x41, 0xf5, 0x24, 0x6b, 0x00, 0x2a, 0xd8, 0xd5,
};

uint8_t N_buf[256] = {
    0xAB, 0xAC, 0x48, 0x22, 0xC6, 0xC1, 0x0F, 0x41,
    0xF0, 0xF6, 0x4D, 0x99, 0xC1, 0xB5, 0x3E, 0x6F,
    0xBB, 0x39, 0xF8, 0x05, 0xF6, 0x90, 0x46, 0x59,
    0xE5, 0x4B, 0x99, 0x6D, 0x18, 0xFB, 0x43, 0x50,
    0x3A, 0xBD, 0xC4, 0xD9, 0x0F, 0x2E, 0x43, 0x5C,
    0xBF, 0x23, 0xE6, 0xDC, 0xE9, 0x47, 0x96, 0xB2,
    0xB5, 0xAF, 0x39, 0x1E, 0xCF, 0x28, 0xAB, 0x50,
    0xD5, 0x73, 0xCA, 0x9B, 0xC6, 0xEA, 0x7C, 0x84,
    0x51, 0x77, 0x1B, 0xA3, 0x1B, 0xF7, 0xD4, 0xE9,
    0x78, 0x8D, 0xC7, 0xDE, 0x9B, 0x4A, 0x6E, 0x62,
    0x9D, 0x42, 0x9F, 0xEE, 0x37, 0x6B, 0x08, 0x3E,
    0xFB, 0x11, 0x12, 0x50, 0xFC, 0xAB, 0x89, 0xC1,
    0x78, 0xDC, 0xD4, 0x56, 0xD0, 0x15, 0x95, 0x33,
    0x96, 0xB2, 0x5A, 0x76, 0xFE, 0x91, 0xCE, 0xFA,
    0xDF, 0x8E, 0xEA, 0x7D, 0x3E, 0x3D, 0x28, 0x9E,
    0x29, 0x75, 0x74, 0xD2, 0x7B, 0xFD, 0x5F, 0xCD,
    0x1E, 0xE0, 0xA2, 0x6E, 0xCA, 0x8C, 0x9E, 0x18,
    0x21, 0x26, 0x03, 0x90, 0x88, 0x2E, 0xEC, 0xDB,
    0xC9, 0xC6, 0x99, 0x9F, 0x13, 0x88, 0xE4, 0x19,
    0x96, 0xB9, 0x52, 0x1B, 0x4C, 0x61, 0xB1, 0x45,
    0xE0, 0xB2, 0x20, 0xB3, 0x11, 0xA2, 0x0B, 0xE0,
    0x7E, 0xF7, 0x31, 0xE1, 0x94, 0xB0, 0xB7, 0x7A,
    0x25, 0xA0, 0xA3, 0x7B, 0xDC, 0xA7, 0x5A, 0xE2,
    0x91, 0xEE, 0x72, 0x85, 0xAE, 0xF2, 0x4B, 0x79,
    0x58, 0x01, 0x1E, 0x26, 0xD8, 0x1E, 0x09, 0xB0,
    0xE2, 0x04, 0xCA, 0xC4, 0xE3, 0x9A, 0xB2, 0x4E,
    0x0C, 0x2B, 0xC8, 0xA5, 0xF4, 0xED, 0x53, 0x47,
    0x19, 0x83, 0xCF, 0xDE, 0xA0, 0xFD, 0x09, 0x34,
    0x26, 0x98, 0x7F, 0xF5, 0x68, 0x98, 0x80, 0xD9,
    0x76, 0xD8, 0xB2, 0xB8, 0xB9, 0xDA, 0x47, 0xDC,
    0x57, 0x94, 0x6C, 0xE2, 0x5F, 0x38, 0x4E, 0x4B,
    0xCF, 0x8E, 0x65, 0xE3, 0x2C, 0x15, 0x5D, 0xA7,
};

uint8_t P_buf[128] = {
    0xE6, 0x71, 0xD6, 0xB3, 0xA1, 0xDF, 0xBC, 0x94,
    0xEF, 0x46, 0xBE, 0xA6, 0xEB, 0x3D, 0xFE, 0xC2,
    0x84, 0xCD, 0xF6, 0x0D, 0x85, 0x8F, 0x0F, 0x2A,
    0xCA, 0xA6, 0x88, 0x9F, 0x46, 0x52, 0x6D, 0x11,
    0x94, 0xA0, 0xC0, 0xFC, 0x31, 0x50, 0x9A, 0x31,
    0xB6, 0x01, 0x0F, 0x08, 0x12, 0x00, 0x20, 0xCA,
    0x26, 0x07, 0x45, 0x90, 0xD7, 0xE0, 0x1F, 0x59,
    0x5D, 0x2E, 0x0A, 0x03, 0xB4, 0xB5, 0xAA, 0xAF,
    0x12, 0xBB, 0x92, 0x73, 0xA4, 0xDD, 0x4C, 0x29,
    0x4B, 0x41, 0x7B, 0x06, 0x3A, 0x45, 0xB2, 0x9B,
    0xDB, 0xE8, 0xA6, 0x97, 0xB6, 0xDB, 0x65, 0xE2,
    0x54, 0xFC, 0xC1, 0xDB, 0x7B, 0x63, 0x46, 0x16,
    0x3C, 0xAF, 0xB6, 0xE9, 0xAB, 0xD5, 0x09, 0xF0,
    0x76, 0x4D, 0x9A, 0x83, 0x86, 0x43, 0xF0, 0xEF,
    0x73, 0x22, 0xC4, 0xEE, 0x38, 0xE0, 0x00, 0xEE,
    0x7D, 0x8F, 0x5D, 0xB9, 0x55, 0xAE, 0x2F, 0xE5,
};

uint8_t Q_buf[128] = {
    0xBE, 0xB5, 0xF4, 0xBA, 0xCC, 0xB1, 0x56, 0x19,
    0xF8, 0x1A, 0x7D, 0xF0, 0x57, 0x38, 0xD7, 0x01,
    0x58, 0x72, 0xCB, 0x77, 0x25, 0x43, 0x7C, 0xCD,
    0x6F, 0x57, 0xD8, 0xCA, 0x7B, 0xC1, 0x54, 0xCC,
    0x4B, 0x2D, 0x8E, 0x65, 0x9D, 0x8D, 0xA4, 0x7C,
    0x86, 0x94, 0xAD, 0xCF, 0xE5, 0x69, 0xA8, 0xA3,
    0x04, 0x37, 0xCF, 0xC8, 0xCD, 0xA8, 0xA0, 0xB0,
    0x4F, 0xA0, 0xE6, 0xC4, 0xFF, 0x3F, 0xCC, 0xDC,
    0x50, 0x9C, 0xCB, 0x97, 0xF5, 0x5C, 0x69, 0xE2,
    0x75, 0xB2, 0x19, 0xD2, 0xC7, 0xE0, 0x4E, 0xBF,
    0xBB, 0x6D, 0xBB, 0x78, 0xB5, 0x12, 0x0B, 0x6D,
    0x2F, 0x8E, 0x0C, 0x3D, 0xD2, 0xBC, 0x6C, 0x5D,
    0x4E, 0xB6, 0xEF, 0x42, 0xB9, 0x24, 0xA4, 0x94,
    0xE5, 0x8D, 0x4C, 0x3F, 0x3F, 0xB5, 0x38, 0xE7,
    0x6D, 0x21, 0x0C, 0x21, 0xB7, 0x3B, 0x58, 0x11,
    0xC3, 0xC3, 0x2A, 0xAA, 0x7E, 0x93, 0x06, 0x9B,
};

uint8_t D_buf[256] = {
    0x2A, 0x61, 0x18, 0xDB, 0xB0, 0xEA, 0x06, 0xEC,
    0xB5, 0xE1, 0xEF, 0xCD, 0x35, 0xB3, 0x1A, 0xEB,
    0x35, 0x32, 0x7D, 0xC6, 0x67, 0x14, 0x3D, 0xB1,
    0xC2, 0x77, 0x93, 0xB4, 0x09, 0x77, 0x39, 0xAA,
    0x4C, 0x1D, 0xFD, 0xC2, 0xC1, 0xCB, 0x1F, 0x68,
    0xFD, 0x6C, 0x8C, 0xF8, 0xDB, 0x03, 0xC7, 0xB1,
    0x6D, 0x45, 0x88, 0xD9, 0xD0, 0xB5, 0x0E, 0xF9,
    0xA0, 0xFF, 0xF9, 0x33, 0xD4, 0x7A, 0x9D, 0x6B,
    0x82, 0xBA, 0xDF, 0x11, 0x38, 0x7D, 0xC9, 0x4B,
    0x2B, 0x6E, 0x00, 0xB9, 0xB4, 0xE7, 0x3E, 0x71,
    0xCC, 0xB5, 0x9E, 0x4A, 0x8D, 0xD6, 0xB7, 0xCD,
    0xE8, 0x67, 0xD1, 0xF8, 0x39, 0x2F, 0xD5, 0x8D,
    0x73, 0xDB, 0x3B, 0xF6, 0x04, 0x5F, 0x57, 0x9A,
    0x49, 0x28, 0xD5, 0x49, 0x4B, 0xD5, 0xCD, 0xDD,
    0x43, 0xA4, 0x01, 0xC2, 0x12, 0x43, 0xC1, 0xCB,
    0xAD, 0x9B, 0x67, 0xCF, 0xF0, 0x38, 0xDF, 0xD5,
    0x98, 0xC2, 0x67, 0x1D, 0x19, 0x17, 0xF3, 0x55,
    0xF2, 0x23, 0x60, 0x25, 0x38, 0x2F, 0x56, 0x50,
    0x1B, 0x1C, 0xFD, 0x78, 0xA3, 0x81, 0xE4, 0x26,
    0xB4, 0x40, 0x17, 0xDC, 0x14, 0x32, 0xE2, 0x17,
    0x70, 0x24, 0xCD, 0x4E, 0xFC, 0xA6, 0xDE, 0x78,
    0x7A, 0x47, 0x99, 0xBF, 0x5A, 0x5A, 0xA0, 0xB8,
    0x86, 0xB4, 0x12, 0x04, 0xE3, 0x4B, 0x84, 0x59,
    0x67, 0xD2, 0x8B, 0xD2, 0x0D, 0xB9, 0xAD, 0x4D,
    0xB0, 0x58, 0x9A, 0x37, 0xFF, 0x62, 0xE6, 0xBD,
    0x3C, 0x79, 0x05, 0xB1, 0x29, 0xD2, 0x87, 0x3E,
    0x0B, 0x56, 0x85, 0x01, 0x04, 0x35, 0x7A, 0xE0,
    0x86, 0x1F, 0xA4, 0x6E, 0xC4, 0x62, 0x8C, 0xA0,
    0x76, 0x8F, 0x6B, 0x5E, 0x82, 0xAC, 0xC9, 0xEA,
    0x66, 0x51, 0x61, 0xF0, 0xE1, 0x18, 0x21, 0x33,
    0xA3, 0x3D, 0x3C, 0x82, 0x49, 0xD8, 0xC1, 0xBB,
    0x39, 0xAD, 0x10, 0xA4, 0x77, 0x97, 0x91, 0x65,
};

uint8_t E_buf[3] = { 0x01, 0x00, 0x01 };

/*
 * DEFINES
 *****************************************************************************************
 */
/* testing for using random key */
//#define RSA_USE_RANDOM_KEY

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

#ifdef RSA_USE_RANDOM_KEY
static void dump_rsa_key(mbedtls_rsa_context *ctx)
{
    int i;
    uint8_t buf[512];

    printf("\r\n  +++++++++++++++++ rsa keypair +++++++++++++++++\r\n");

    mbedtls_mpi_write_binary( &ctx->N, buf, mbedtls_mpi_size(&ctx->N) );
    printf("N:\r\n");
    for(i=0;i<mbedtls_mpi_size(&ctx->N);i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
    mbedtls_mpi_write_binary( &ctx->E, buf, mbedtls_mpi_size(&ctx->E) );
    printf("E:\r\n");
    for(i=0;i<mbedtls_mpi_size(&ctx->E);i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
    mbedtls_mpi_write_binary( &ctx->D, buf, mbedtls_mpi_size(&ctx->D) );
    printf("D:\r\n");
    for(i=0;i<mbedtls_mpi_size(&ctx->D);i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
    mbedtls_mpi_write_binary( &ctx->P, buf, mbedtls_mpi_size(&ctx->P) );
    printf("P:\r\n");
    for(i=0;i<mbedtls_mpi_size(&ctx->P);i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
    mbedtls_mpi_write_binary( &ctx->Q, buf, mbedtls_mpi_size(&ctx->Q) );
    printf("Q:\r\n");
    for(i=0;i<mbedtls_mpi_size(&ctx->Q);i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");

    printf("\r\n  +++++++++++++++++ rsa keypair +++++++++++++++++\r\n");
}
#endif

#ifndef RSA_USE_RANDOM_KEY
static int set_rsa_key(mbedtls_rsa_context *ctx)
{
    int ret = 0;
    mbedtls_mpi N;
    mbedtls_mpi P;
    mbedtls_mpi Q;
    mbedtls_mpi D;
    mbedtls_mpi E;

    mbedtls_mpi_init(&N);
    mbedtls_mpi_init(&P);
    mbedtls_mpi_init(&Q);
    mbedtls_mpi_init(&D);
    mbedtls_mpi_init(&E);

    mbedtls_mpi_read_binary(&N, N_buf, sizeof N_buf);
    mbedtls_mpi_read_binary(&P, P_buf, sizeof P_buf);
    mbedtls_mpi_read_binary(&Q, Q_buf, sizeof Q_buf);
    mbedtls_mpi_read_binary(&D, D_buf, sizeof D_buf);
    mbedtls_mpi_read_binary(&E, E_buf, sizeof E_buf);

    ret= mbedtls_rsa_import( ctx, &N, &P, &Q, &D, &E);
    if(ret != 0) {
        goto exit;
    }
    ret = mbedtls_rsa_complete( ctx );
    if(ret != 0) {
        goto exit;
    }

exit:
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&P);
    mbedtls_mpi_free(&Q);
    mbedtls_mpi_free(&D);
    mbedtls_mpi_free(&E);
    return ret;
}
#endif

void mbedtls_rsa_rsassa_pkcs1_v15_test(void)
{
    int ret;
    int i;
    uint8_t sig[256] = {0};
    uint32_t sig_len;
    const char *pers = "rsa_test";
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_pk_context ctx;

    /* 1. init structure */
    mbedtls_pk_init( &ctx );
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_add_source(&entropy, entropy_source, NULL,
                               MBEDTLS_ENTROPY_MAX_GATHER,
                               MBEDTLS_ENTROPY_SOURCE_STRONG);
    const mbedtls_pk_info_t *info = mbedtls_pk_info_from_type( MBEDTLS_PK_RSA );
    mbedtls_pk_setup( &ctx, info );
    mbedtls_rsa_init(mbedtls_pk_rsa(ctx), MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_NONE);

    /* 2. update seed with we own interface ported */
    printf( "\r\nSeeding the random number generator..." );
    ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen(pers));
    if(ret != 0) {
        printf("failed! mbedtls_ctr_drbg_seed returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( "ok\r\n" );

    /* 3. generate RSA keypair */
    printf( "\r\nGenerate RSA keypair..." );

#ifdef RSA_USE_RANDOM_KEY
    ret = mbedtls_rsa_gen_key(mbedtls_pk_rsa(ctx), mbedtls_ctr_drbg_random, &ctr_drbg, 2048, 65537);
    if(ret != 0) {
        printf("failed! mbedtls_rsa_gen_key returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
#else
    ret = set_rsa_key(mbedtls_pk_rsa(ctx));
    if(ret != 0) {
        printf("failed! set_rsa_key returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
#endif
    printf( "ok\r\n" );

#ifdef RSA_USE_RANDOM_KEY
    /* show */
    dump_rsa_key(mbedtls_pk_rsa(ctx));
#endif

    /* 4. sign */
    printf( "\r\nRSA rsassa pkcs1 v15 sign..." );

    ret = mbedtls_pk_sign( &ctx, MBEDTLS_MD_NONE, hash, 32, sig, &sig_len, mbedtls_ctr_drbg_random, &ctr_drbg );
    if(ret != 0) {
        printf("failed! mbedtls_rsa_pkcs1_sign returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( "sign ok\r\n" );

    /* show */
    printf("\r\nRSA signature:\r\n");
    for(i=0;i<256;i++)
    {
        printf("%02X ", sig[i]);
    }
    printf("\r\n");

    /* 5. verify */
    printf( "\r\nRSA rsassa pkcs1 v15 verify..." );
    ret = mbedtls_pk_verify( &ctx, MBEDTLS_MD_NONE, hash, 32, sig, sig_len);
    if(ret != 0) {
        printf("failed! mbedtls_rsa_pkcs1_verify returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( "verify ok\r\n" );

exit:
    /* 6. release structure */
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
    mbedtls_pk_free( &ctx );
    return;
}

void mbedtls_rsa_rsassa_pss_test(void)
{
    int ret;
    int i;
    uint8_t sig[256] = {0};
    uint32_t sig_len;
    const char *pers = "rsa_test";
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_pk_context ctx;

    /* 1. init structure */
    mbedtls_pk_init( &ctx );
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_add_source(&entropy, entropy_source, NULL,
                               MBEDTLS_ENTROPY_MAX_GATHER,
                               MBEDTLS_ENTROPY_SOURCE_STRONG);
    const mbedtls_pk_info_t *info = mbedtls_pk_info_from_type( MBEDTLS_PK_RSA );
    mbedtls_pk_setup( &ctx, info );
    mbedtls_rsa_init(mbedtls_pk_rsa(ctx), MBEDTLS_RSA_PKCS_V21, MBEDTLS_MD_SHA256);

    /* 2. update seed with we own interface ported */
    printf( "\r\nSeeding the random number generator..." );
    ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen(pers));
    if(ret != 0) {
        printf("failed! mbedtls_ctr_drbg_seed returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( "ok\r\n" );

    /* 3. generate RSA keypair */
    printf( "\r\nGenerate RSA keypair..." );

#ifdef RSA_USE_RANDOM_KEY
    ret = mbedtls_rsa_gen_key(mbedtls_pk_rsa(ctx), mbedtls_ctr_drbg_random, &ctr_drbg, 2048, 65537);
    if(ret != 0) {
        printf("failed! mbedtls_rsa_gen_key returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
#else
    ret = set_rsa_key(mbedtls_pk_rsa(ctx));
    if(ret != 0) {
        printf("failed! set_rsa_key returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
#endif
    printf( "ok\r\n" );

#ifdef RSA_USE_RANDOM_KEY
    /* show */
    dump_rsa_key(mbedtls_pk_rsa(ctx));
#endif

    /* 4. sign */
    printf( "\r\nRSA rsassa pss sign..." );

    ret = mbedtls_pk_sign( &ctx, MBEDTLS_MD_NONE, hash, 32, sig, &sig_len, mbedtls_ctr_drbg_random, &ctr_drbg );
    if(ret != 0) {
        printf("failed! mbedtls_rsa_pkcs1_sign returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( "sign ok\r\n" );

    /* show */
    printf("\r\nRSA signature:\r\n");
    for(i=0;i<256;i++)
    {
        printf("%02X ", sig[i]);
    }
    printf("\r\n");

    /* 5. verify */
    printf( "\r\nRSA rsassa pss verify..." );
    ret = mbedtls_pk_verify( &ctx, MBEDTLS_MD_NONE, hash, 32, sig, sig_len);
    if(ret != 0) {
        printf("failed! mbedtls_rsa_pkcs1_verify returned %d(-0x%04x)\r\n", ret, -ret);
        goto exit;
    }
    printf( "verify ok\r\n" );

exit:
    /* 6. release structure */
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
    mbedtls_pk_free( &ctx );
    return;
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                Crypto RSA example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how RSA work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    /* RSASSA-PKCS1-v1_5 */
    mbedtls_rsa_rsassa_pkcs1_v15_test();

    /* RSASSA-PSS */
    mbedtls_rsa_rsassa_pss_test();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
