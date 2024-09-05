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
#include "app_io.h"
#include "app_i2c.h"
#include "app_i2c_dma.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SLAVE_DEV_ADDR                 0x55
#define SLAVE_I2C_ID                   APP_I2C_SLAVE_ID

app_i2c_params_t slave_params = {
    .id = SLAVE_I2C_ID,
    .role = APP_I2C_ROLE_SLAVE,
    .pin_cfg = {
        .scl = {
            .type = APP_I2C_SLAVE_SCL_IO_TYPE,
            .mux  = APP_I2C_SLAVE_SCL_PINMUX,
            .pin  = APP_I2C_SLAVE_SCL_PIN,
            .pull = APP_IO_PULLUP,
        },
        .sda = {
            .type = APP_I2C_SLAVE_SDA_IO_TYPE,
            .mux  = APP_I2C_SLAVE_SDA_PINMUX,
            .pin  = APP_I2C_SLAVE_SDA_PIN,
            .pull = APP_IO_PULLUP,
        },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel  = DMA_Channel2,
        .rx_dma_channel  = DMA_Channel3,
    },
    .init = {
        .speed = I2C_SPEED_100K,
        .own_address = SLAVE_DEV_ADDR,
        .addressing_mode = I2C_ADDRESSINGMODE_7BIT,
        .general_call_mode = I2C_GENERALCALL_DISABLE,
    },
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_slave_tdone = 0;
volatile uint8_t g_slave_rdone = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static void app_i2c_slave_evt_handler(app_i2c_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_I2C_EVT_ERROR:
            g_slave_tdone = 1;
            g_slave_rdone = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:
            g_slave_tdone = 1;
            break;

        case APP_I2C_EVT_RX_DATA:
            g_slave_rdone = 1;
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_i2c_slave_interrupt(void)
{
    uint32_t i;
    uint16_t ret = APP_DRV_SUCCESS;
    uint8_t  wdata[256] = {0};
    uint8_t  rdata[256] = {0};

    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_i2c_init(&slave_params, app_i2c_slave_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nI2C slave initial failed! Please check the input parameters.\r\n");
        return;
    }

    for(i = 0; i < 256; i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    printf("I2C slave receive start.\r\n");
    g_slave_rdone = 0;
    ret = app_i2c_receive_async(SLAVE_I2C_ID, 0, rdata, 256);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("I2C slave receive failed.\r\n");
        return;
    }
    while(g_slave_rdone == 0);

    printf("I2C slave received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }

    memset(rdata, 0, sizeof(rdata));
    printf("I2C slave send start.\r\n");
    g_slave_tdone = 0;
    ret = app_i2c_transmit_async(SLAVE_I2C_ID, 0, wdata, 256);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("I2C slave send failed.\r\n");
        return;
    }
    while(g_slave_tdone == 0);

    app_i2c_deinit(SLAVE_I2C_ID);
}

void app_i2c_slave_dma(void)
{
    uint32_t i;
    uint16_t ret = APP_DRV_SUCCESS;
    uint8_t  wdata[256] = {0};
    uint8_t  rdata[256] = {0};

    /* Please initialize DMA in the following order. */
    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_i2c_init(&slave_params, app_i2c_slave_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nI2C slave initial failed! Please check the input parameters.\r\n");
        return;
    }

    ret = app_i2c_dma_init(&slave_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nI2C slave initial dma failed! Please check the input parameters.\r\n");
        return;
    }

    for(i = 0; i < 256; i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    printf("I2C slave receive start.\r\n");
    g_slave_rdone = 0;
    ret = app_i2c_dma_receive_async(SLAVE_I2C_ID, 0, rdata, 256);

    if (ret != APP_DRV_SUCCESS)
    {
        printf("I2C slave receive failed.\r\n");
        return;
    }
    while(g_slave_rdone == 0);

    printf("I2C slave received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }

    memset(rdata, 0, sizeof(rdata));
    printf("I2C slave send start.\r\n");

    g_slave_tdone = 0;
    ret = app_i2c_dma_transmit_async(SLAVE_I2C_ID, 0, wdata, 256);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("I2C slave send failed.\r\n");
        return;
    }
    while(g_slave_tdone == 0);

    /* Please deinitialize DMA in the following order. */
    app_i2c_dma_deinit(SLAVE_I2C_ID);
    app_i2c_deinit(SLAVE_I2C_ID);
}


int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              I2C_SLAVE APP example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: Fast Speed, 7-bits Addr, 8-bits data       *\r\n");
    printf("* Note: You need connect pull-up resistor on SCL/SDA *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will show I2C master device send data  *\r\n");
    printf("* to slave and receive data from it.                 *\r\n");
    printf("******************************************************\r\n");

    printf("\r\napp i2c slave interrupt test.\r\n");
    app_i2c_slave_interrupt();

    printf("\r\napp i2c slave dma test.\r\n");
    app_i2c_slave_dma();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

