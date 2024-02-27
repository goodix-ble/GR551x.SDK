/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "app_rng.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
app_rng_params_t rng_polling_params = {
   .use_type = APP_RNG_TYPE_POLLING,
   .init = {
       .seed_mode = RNG_SEED_USER,
       .lfsr_mode = RNG_LFSR_MODE_59BIT,
       .out_mode  = RNG_OUTPUT_LFSR,
       .post_mode = RNG_POST_PRO_NOT,
   },
};

app_rng_params_t rng_interrupt_params = {
    .use_type = APP_RNG_TYPE_INTERRUPT,
    .init = {
        .seed_mode = RNG_SEED_USER,
        .lfsr_mode = RNG_LFSR_MODE_59BIT,
        .out_mode  = RNG_OUTPUT_LFSR,
        .post_mode = RNG_POST_PRO_NOT,
    },
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint32_t g_flag_it = 0;
volatile uint32_t g_data = 0;
uint16_t g_random_seed[8] = {0x1234, 0x5678, 0x90AB, 0xCDEF, 0x1468, 0x2345, 0x5329, 0x2411};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_rng_event_handler(app_rng_evt_t *p_evt)
{
    if (p_evt->type == APP_RNG_EVT_DONE)
    {
        g_flag_it = 1;
        g_data = p_evt->random_data;
    }
}

void rng_polling(void)
{
    uint32_t random_number = 0;

    app_rng_init(&rng_polling_params, NULL);
    printf("\r\nuse the seed for user to generate random number(Polling)\r\n");
    app_rng_gen_sync(g_random_seed, &random_number);
    printf("random numbers is %08x\r\n", random_number);
    app_rng_deinit();

    /* Use FR0 to generate true random number */
    rng_polling_params.init.seed_mode = RNG_SEED_FR0_S0;
    rng_polling_params.init.out_mode = RNG_OUTPUT_FR0_S0;

    app_rng_init(&rng_polling_params, NULL);
    printf("\r\nuse FR0 to generate random number(Polling)\r\n");
    app_rng_gen_sync(NULL, &random_number);
    printf("random numbers is %08x\r\n", random_number);
    app_rng_deinit();
}

void rng_interrupt(void)
{
    app_rng_init(&rng_interrupt_params, app_rng_event_handler);
    printf("\r\nuse the seed for user to generate random number(Interrupt)\r\n");
    g_flag_it = 0;
    app_rng_gen_async(g_random_seed);
    while(g_flag_it == 0);
    printf("random numbers is %08x\r\n", g_data);
    app_rng_deinit();

    /* Use FR0 to generate true random number */
    rng_interrupt_params.init.seed_mode = RNG_SEED_FR0_S0;
    rng_interrupt_params.init.out_mode = RNG_OUTPUT_FR0_S0;

    app_rng_init(&rng_interrupt_params, app_rng_event_handler);
    printf("\r\nuse FR0 to generate random number(Interrupt)\r\n");
    g_flag_it = 0;
    app_rng_gen_async(NULL);
    while(g_flag_it == 0);
    printf("random numbers is %08x\r\n", g_data);
    app_rng_deinit();
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("*****************************************************************\r\n");
    printf("*                       RNG App example.                        *\r\n");
    printf("*                                                               *\r\n");
    printf("* RNG generates random numbers in Polling mode(User Seed/FR0).  *\r\n");
    printf("* RNG generates random numbers in Interrupt mode(User Seed/FR0) *\r\n");
    printf("* This sample code will print random number on terminal.        *\r\n");
    printf("*****************************************************************\r\n");
    printf("\r\n");

    /* RNG generates random numbers in Polling mode(User Seed/FR0) */
    rng_polling();

    /* RNG generates random numbers in Interrupt mode(User Seed/FR0) */
    rng_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
