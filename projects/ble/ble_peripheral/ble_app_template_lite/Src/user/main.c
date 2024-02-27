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
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
 /**@brief Stack global variables for Bluetooth protocol stack. */
 
#define PRF_BUF_LITE_SIZE (92*CFG_MAX_PRFS)
#define BOND_BUF_LITE_SIZE (8*CFG_MAX_BOND_DEVS)
#define CONN_BUF_LITE_SIZE (372*CFG_MAX_CONNECTIONS)

#define ENV_HEAP_LITE_SIZE       (366 + CFG_MAX_CONNECTIONS * 788)
#define ATT_DB_HEAP_LITE_SIZE    (768 + 12)
#define KE_MSG_HEAP_LITE_SIZE    (3355 + CFG_MAX_CONNECTIONS * 520)
#define NON_RET_HEAP_LITE_SIZE   (82 * 2 + 12)

uint8_t prf_buf[PRF_BUF_LITE_SIZE]   __attribute__((aligned (32))) = {0};
uint8_t bond_buf[BOND_BUF_LITE_SIZE] __attribute__((aligned (32))) = {0};
uint8_t conn_buf[CONN_BUF_LITE_SIZE] __attribute__((aligned (32))) = {0};
uint8_t env_heap_buf[ENV_HEAP_LITE_SIZE] __attribute__((aligned (32)))= {0};
uint8_t att_db_heap_buf[ATT_DB_HEAP_LITE_SIZE] __attribute__((aligned (32)))= {0};
uint8_t ke_msg_heap_buf[KE_MSG_HEAP_LITE_SIZE] __attribute__((aligned (32))) = {0};
uint8_t non_ret_heap_buf[NON_RET_HEAP_LITE_SIZE]__attribute__((aligned (32))); 

int main (void)
{
    // Initialize user peripherals.
    app_periph_init();
    
    stack_heaps_table_t heaps_table = { (uint32_t *)env_heap_buf,\
                                        (uint32_t *)att_db_heap_buf,\
                                        (uint32_t *)ke_msg_heap_buf,\
                                        (uint32_t *)non_ret_heap_buf,\
                                        ENV_HEAP_LITE_SIZE,\
                                        ATT_DB_HEAP_LITE_SIZE,\
                                        KE_MSG_HEAP_LITE_SIZE,\
                                        NON_RET_HEAP_LITE_SIZE,\
                                        (uint8_t *)prf_buf,\
                                        PRF_BUF_LITE_SIZE,\
                                        (uint8_t *)bond_buf,\
                                        BOND_BUF_LITE_SIZE,\
                                        (uint8_t *)conn_buf,\
                                        CONN_BUF_LITE_SIZE};

    // Initialize ble stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    // loop
    while (1)
    {
        pwr_mgmt_schedule();
    }
}
