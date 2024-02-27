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
#include "app_dma.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DMA_DATA_LEN                        (4092)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_src_data[DMA_DATA_LEN]  = {0};
uint8_t g_dst_data[DMA_DATA_LEN]  = {0};
volatile bool g_tfr_flag = false;
volatile bool g_blk_flag = false;
volatile bool g_err_flag = false;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static dma_id_t s_dma_channel_id = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void app_dma_callback(app_dma_evt_type_t type)
{
    switch (type)
    {
        case APP_DMA_EVT_ERROR:
            g_err_flag = true;
            break;
        case APP_DMA_EVT_TFR:
            g_tfr_flag = true;
            break;
        case APP_DMA_EVT_BLK:
            g_blk_flag = true;
            break;
        default:
            break;
    }
    return;
}

static void dma_test_data_init(void)
{
    uint32_t index;

    g_tfr_flag = 0;
    g_blk_flag = 0;
    g_err_flag = 0;

    for(index = 0; index < DMA_DATA_LEN; index++)
    {
        g_src_data[index] = index;
        g_dst_data[index] = 0;
    }
    return;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_dma0_memory_to_memory(void)
{
    uint32_t delay_count = 0;
    app_dma_params_t dma_params = { 0 };

    dma_test_data_init();

    dma_params.p_instance               = DMA0;
    dma_params.channel_number           = DMA_Channel0;
    dma_params.init.direction           = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment       = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment       = DMA_DST_INCREMENT;
    dma_params.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    dma_params.init.mode                = DMA_NORMAL;
#endif
    dma_params.init.priority            = DMA_PRIORITY_LOW;

    s_dma_channel_id = app_dma_init(&dma_params, app_dma_callback);
    if(-1 == s_dma_channel_id)
    {
        printf("app_dma0_init fail\r\n");
        return;
    }
    g_tfr_flag = false;
    g_blk_flag = false;
    g_err_flag = false;
    app_dma_start(s_dma_channel_id, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    while((g_tfr_flag != true) && (g_blk_flag != true) && (g_err_flag != true) && (delay_count++ < 500))
    {
        delay_ms(10);
    }
    if(memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("app_dma0_memory_to_memory: fail\r\n");
    }
    else
    {
        printf("app_dma0_memory_to_memory: success\r\n");
    }
    app_dma_deinit(s_dma_channel_id);
    return;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
void app_dma1_memory_to_memory(void)
{
    uint32_t delay_count = 0;
    app_dma_params_t dma_params = {DMA1, DMA_Channel0, 0};

    dma_test_data_init();

    dma_params.p_instance               = DMA1;
    dma_params.channel_number           = DMA_Channel0;
    dma_params.init.direction           = DMA_MEMORY_TO_MEMORY;
    dma_params.init.src_increment       = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment       = DMA_DST_INCREMENT;
    dma_params.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    dma_params.init.mode                = DMA_NORMAL;
    dma_params.init.priority            = DMA_PRIORITY_LOW;

    s_dma_channel_id = app_dma_init(&dma_params, app_dma_callback);
    if(-1 == s_dma_channel_id)
    {
        printf("app_dma1_init fail\r\n");
        return;
    }
    g_tfr_flag = false;
    g_blk_flag = false;
    g_err_flag = false;
    app_dma_start(s_dma_channel_id, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    while((g_tfr_flag != true) && (g_blk_flag != true) && (g_err_flag != true) && (delay_count++ < 500))
    {
        delay_ms(10);
    }
    if(memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("app_dma1_memory_to_memory: fail\r\n");
    }
    else
    {
        printf("app_dma1_memory_to_memory: success\r\n");
    }
    app_dma_deinit(s_dma_channel_id);
    return;
}
#endif

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   DMA example.                     *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how DMA transfer data   *\r\n");
    printf("* from memory to memory.                             *\r\n");
    printf("******************************************************\r\n");

    app_dma0_memory_to_memory();

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    app_dma1_memory_to_memory();
#endif

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
