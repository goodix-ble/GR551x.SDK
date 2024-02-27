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
#include "grx_hal.h"
#include "app_i2s.h"
#include "app_i2s_dma.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef HAL_I2S_MODULE_ENABLED

#define TEST_LENGTH                512

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;
volatile uint8_t g_slave_tdone  = 0;
volatile uint8_t g_slave_rdone  = 0;

app_i2s_params_t i2sm_params = {
    .id = APP_I2S_ID_MASTER,
    .pin_cfg = {
        .ws = {
            .type = APP_I2S_MASTER_WS_IO_TYPE,
            .mux  = APP_I2S_MASTER_WS_PINMUX,
            .pin  = APP_I2S_MASTER_WS_PIN,
            .pull = APP_IO_NOPULL,
        },
        .sdo = {
            .type = APP_I2S_MASTER_SDO_IO_TYPE,
            .mux  = APP_I2S_MASTER_SDO_PINMUX,
            .pin  = APP_I2S_MASTER_SDO_PIN,
            .pull = APP_IO_NOPULL,
        },
        .sdi = {
            .type = APP_I2S_MASTER_SDI_IO_TYPE,
            .mux  = APP_I2S_MASTER_SDI_PINMUX,
            .pin  = APP_I2S_MASTER_SDI_PIN,
            .pull = APP_IO_NOPULL,
        },
        .sclk = {
            .type = APP_I2S_MASTER_SCLK_IO_TYPE,
            .mux  = APP_I2S_MASTER_SCLK_PINMUX,
            .pin  = APP_I2S_MASTER_SCLK_PIN,
            .pull = APP_IO_NOPULL,
        },
    },
    .dma_cfg = {
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .tx_dma_instance = DMA1,
        .rx_dma_instance = DMA1,
#else
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
#endif
        .tx_dma_channel  = DMA_Channel0,
        .rx_dma_channel  = DMA_Channel1,
    },
    .init = {
        .data_size    = I2S_DATASIZE_32BIT,
        .clock_source = I2S_CLOCK_SRC_96M,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .ws_cycles   = I2S_WS_CYCLES_32,
#endif
        .audio_freq = 48000,
    },
};

app_i2s_params_t i2ss_params = {
    .id = APP_I2S_ID_SLAVE,
    .pin_cfg = {
        .ws = {
            .type = APP_I2S_SLAVE_WS_IO_TYPE,
            .mux  = APP_I2S_SLAVE_WS_PINMUX,
            .pin  = APP_I2S_SLAVE_WS_PIN,
            .pull = APP_IO_NOPULL,
        },
        .sdo = {
            .type = APP_I2S_SLAVE_SDO_IO_TYPE,
            .mux  = APP_I2S_SLAVE_SDO_PINMUX,
            .pin  = APP_I2S_SLAVE_SDO_PIN,
            .pull = APP_IO_NOPULL,
        },
        .sdi = {
            .type = APP_I2S_SLAVE_SDI_IO_TYPE,
            .mux  = APP_I2S_SLAVE_SDI_PINMUX,
            .pin  = APP_I2S_SLAVE_SDI_PIN,
            .pull = APP_IO_NOPULL,
        },
        .sclk = {
            .type = APP_I2S_SLAVE_SCLK_IO_TYPE,
            .mux  = APP_I2S_SLAVE_SCLK_PINMUX,
            .pin  = APP_I2S_SLAVE_SCLK_PIN,
            .pull = APP_IO_NOPULL,
        },
    },
    .dma_cfg = {
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .tx_dma_instance = DMA1,
        .rx_dma_instance = DMA1,
#else
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
#endif
        .tx_dma_channel  = DMA_Channel2,
        .rx_dma_channel  = DMA_Channel3,
    },
    .init = {
        .data_size    = I2S_DATASIZE_32BIT,
        .clock_source = I2S_CLOCK_SRC_96M,
 #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .ws_cycles   = I2S_WS_CYCLES_32,
 #endif
        .audio_freq = 48000,
    },
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void print_buf_16bits(uint16_t *buf, uint32_t nbytes)
{
    for (uint32_t i = 0; i < (nbytes >> 1); i++)
    {
        printf("0x%04X ", buf[i]);
        if ((i & 0x7) == 0x7)
            printf("\r\n");
    }
    printf("\r\n");
}

void print_buf_32bits(uint32_t *buf, uint32_t nbytes)
{
    for (uint32_t i = 0; i < (nbytes >> 2); i++)
    {
        printf("0x%08X ", buf[i]);
        if ((i & 0x3) == 0x3)
            printf("\r\n");
    }
    printf("\r\n");
}

void print_buf(const char *tag, void *buf, uint32_t nbytes, uint8_t type_size)
{
    printf("%s\r\n", tag);
    if (type_size <= 2)
        print_buf_16bits((uint16_t *)buf, nbytes);
    else
        print_buf_32bits((uint32_t *)buf, nbytes);
}

void app_i2sm_callback(app_i2s_evt_t *p_evt)
{
    if (p_evt->type == APP_I2S_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_I2S_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if (p_evt->type == APP_I2S_EVT_ERROR)
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

void app_i2ss_callback(app_i2s_evt_t *p_evt)
{
    if (p_evt->type == APP_I2S_EVT_TX_CPLT)
    {
        g_slave_tdone = 1;
    }
    if (p_evt->type == APP_I2S_EVT_RX_DATA)
    {
        g_slave_rdone = 1;
    }
    if (p_evt->type == APP_I2S_EVT_ERROR)
    {
        g_slave_tdone = 1;
        g_slave_rdone = 1;
    }
}

void i2s_interrupt(void)
{
    uint16_t ret = 0;
    uint8_t  type_szie = 2;

    uint16_t wdata[TEST_LENGTH] = {0};
    uint16_t rdata[TEST_LENGTH] = {0};

    for(uint32_t i = 0; i < (sizeof(wdata) >> 1); i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    ret = app_i2s_init(&i2sm_params, app_i2sm_callback);
    if (ret != 0)
    {
        printf("\r\nI2SM initial failed! Please check the input paraments.\r\n");
        return;
    }

    ret = app_i2s_init(&i2ss_params, app_i2ss_callback);
    if (ret != 0)
    {
        printf("\r\nI2SM initial failed! Please check the input paraments.\r\n");
        return;
    }
    if (I2S_DATASIZE_16BIT < i2sm_params.init.data_size)
        type_szie = 4;

    printf("\r\nI2SM interrupt write start.\r\n");
    app_i2s_flush_tx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_rx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_tx_fifo(APP_I2S_ID_SLAVE);
    app_i2s_flush_rx_fifo(APP_I2S_ID_SLAVE);
    g_master_tdone = 0;
    g_slave_rdone = 0;
    app_i2s_receive_async(APP_I2S_ID_SLAVE, rdata, sizeof(rdata) >> 2);
    app_i2s_transmit_async(APP_I2S_ID_MASTER, wdata, sizeof(wdata) >> 2);
    while((g_master_tdone == 0) || (g_slave_rdone == 0));
    delay_ms(2);
    app_i2s_disable_clock(APP_I2S_ID_MASTER);
    print_buf("Please check I2SS Received: ", (void *)rdata, sizeof(rdata), type_szie);
    delay_ms(1000);
    printf("\r\nI2SM interrupt read start.\r\n");
    app_i2s_flush_tx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_rx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_tx_fifo(APP_I2S_ID_SLAVE);
    app_i2s_flush_rx_fifo(APP_I2S_ID_SLAVE);
    g_master_rdone = 0;
    g_slave_tdone = 0;
    memset(rdata, 0, sizeof(rdata));
    app_i2s_transmit_async(APP_I2S_ID_SLAVE, wdata, sizeof(wdata) >> 2);
    app_i2s_receive_async(APP_I2S_ID_MASTER, rdata, sizeof(rdata) >> 2);
    while((g_master_rdone == 0) || (g_slave_tdone == 0));
    delay_ms(2);
    app_i2s_disable_clock(APP_I2S_ID_MASTER);
    print_buf("Please check I2SM Received: ", (void *)rdata, sizeof(rdata), type_szie);
}

void i2s_dma(void)
{
    uint16_t ret = 0;
    uint8_t  type_szie = 2;

    uint16_t wdata[TEST_LENGTH] = {0};
    uint16_t rdata[TEST_LENGTH] = {0};

    ret = app_i2s_init(&i2sm_params, app_i2sm_callback);
    if (ret != 0)
    {
        printf("\r\nI2SM initial failed! Please check the input paraments.\r\n");
        return;
    }

    ret = app_i2s_init(&i2ss_params, app_i2ss_callback);
    if (ret != 0)
    {
        printf("\r\nI2SM initial failed! Please check the input paraments.\r\n");
        return;
    }
    for(uint32_t i = 0; i < (sizeof(wdata) >> 1); i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    ret = app_i2s_dma_init(&i2sm_params);
    if (ret != 0)
    {
        printf("\r\nI2SM initial failed! Please check the input paraments.\r\n");
        return;
    }
    ret = app_i2s_dma_init(&i2ss_params);
    if (ret != 0)
    {
        printf("\r\nI2SM initial failed! Please check the input paraments.\r\n");
        return;
    }

    if (I2S_DATASIZE_16BIT < i2sm_params.init.data_size)
        type_szie = 4;

    printf("\r\nI2SM dma write start.\r\n");
    app_i2s_flush_tx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_rx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_tx_fifo(APP_I2S_ID_SLAVE);
    app_i2s_flush_rx_fifo(APP_I2S_ID_SLAVE);
    g_master_tdone = 0;
    g_slave_rdone = 0;
    app_i2s_dma_receive_async(APP_I2S_ID_SLAVE, rdata, sizeof(rdata) >> 2);
    app_i2s_dma_transmit_async(APP_I2S_ID_MASTER, wdata, sizeof(wdata) >> 2);
    while((g_master_tdone == 0) || (g_slave_rdone == 0));
    delay_ms(2);
    app_i2s_disable_clock(APP_I2S_ID_MASTER);
    print_buf("Please check I2SS Received: ", (void *)rdata, sizeof(rdata), type_szie);
    delay_ms(1000);
    printf("\r\nI2SM dma read start.\r\n");
    app_i2s_flush_tx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_rx_fifo(APP_I2S_ID_MASTER);
    app_i2s_flush_tx_fifo(APP_I2S_ID_SLAVE);
    app_i2s_flush_rx_fifo(APP_I2S_ID_SLAVE);
    g_master_rdone = 0;
    g_slave_tdone = 0;
    memset(rdata, 0, sizeof(rdata));
    app_i2s_dma_transmit_async(APP_I2S_ID_SLAVE, wdata, sizeof(wdata) >> 2);
    app_i2s_dma_receive_async(APP_I2S_ID_MASTER, rdata, sizeof(rdata) >> 2);
    while((g_master_rdone == 0) || (g_slave_tdone == 0));
    delay_ms(2);
    app_i2s_disable_clock(APP_I2S_ID_MASTER);
    print_buf("Please check I2SM Received: ", (void *)rdata, sizeof(rdata), type_szie);
}

#endif  /* HAL_I2S_MODULE_ENABLED */

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*             I2S_MASTER_APP  example.               *\r\n");
    printf("*                                                    *\r\n");
    printf("*          Config: 32-bits data, 48KHz               *\r\n");
    printf("*                                                    *\r\n");
    printf("*               I2S_M   <------->  I2S_S             *\r\n");
    printf("*           WS (MASTER) -------->   WS (SLAVE)       *\r\n");
    printf("*           SCL(MASTER) -------->   SCL(SLAVE)       *\r\n");
    printf("*           SDO(MASTER) -------->   SDI(SLAVE)       *\r\n");
    printf("*           SDI(MASTER) <--------   SDO(SLAVE)       *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2S_M and slave device.             *\r\n");
    printf("* This smaple will show I2S master send data to i2s  *\r\n");
    printf("* slave device.                                      *\r\n");
    printf("******************************************************\r\n");

#ifdef HAL_I2S_MODULE_ENABLED
    i2s_interrupt();
    i2s_dma();
#endif  /* HAL_I2S_MODULE_ENABLED */

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
