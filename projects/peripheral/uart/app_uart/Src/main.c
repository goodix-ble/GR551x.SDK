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
#include "app_uart.h"
#include "app_uart_dma.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_DATA_LEN                       (512)
#define UART_ID                             APP_UART_ID

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_tx_buffer[UART_DATA_LEN] = {0};
uint8_t g_rx_buffer[UART_DATA_LEN] = {0};
uint8_t g_ring_buffer[UART_DATA_LEN + 1] = {0};
uint8_t g_message_0[] = "APP UART example.\r\nPlease input characters(<126) and end with newline.\r\n";
uint8_t g_message_1[] = "Input:\r\n";
volatile uint16_t rlen = 0;
volatile uint8_t g_tdone = 0;
volatile uint8_t g_rdone = 0;

app_uart_params_t uart_params = {
    .id      = UART_ID,
    .pin_cfg = {
           .tx = {
              .type = APP_UART_TX_IO_TYPE,
              .mux  = APP_UART_TX_PINMUX,
              .pin  = APP_UART_TX_PIN,
              .pull = APP_IO_PULLUP,
           },
           .rx = {
              .type = APP_UART_RX_IO_TYPE,
              .mux  = APP_UART_RX_PINMUX,
              .pin  = APP_UART_RX_PIN,
              .pull = APP_IO_PULLUP,
           },
    },

    .init = {
        .baud_rate = 115200,
        .data_bits = UART_DATABITS_8,
        .stop_bits = UART_STOPBITS_1,
        .parity    = UART_PARITY_NONE,
        .hw_flow_ctrl    = UART_HWCONTROL_NONE,
        .rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE,
    },
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_uart_callback(app_uart_evt_t *p_evt)
{
    if (p_evt->type == APP_UART_EVT_TX_CPLT)
    {
        g_tdone = 1;
    }
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        g_rdone = 1;
        rlen = p_evt->data.size;
        memcpy(g_tx_buffer, g_rx_buffer, rlen);
    }
    if (p_evt->type == APP_UART_EVT_ERROR)
    {
        g_tdone = 1;
        g_rdone = 1;
    }
}

void app_uart_demo(void)
{
    uint16_t ret = 0;
    app_uart_tx_buf_t uart_buffer = {0};

    uart_buffer.tx_buf = g_ring_buffer;
    uart_buffer.tx_buf_size = sizeof(g_ring_buffer);
    ret = app_uart_init(&uart_params, app_uart_callback, &uart_buffer);
    if (ret != APP_DRV_SUCCESS)
    {
        return;
    }
    app_uart_transmit_sync(UART_ID, g_message_0, sizeof(g_message_0), 5000);
    app_uart_transmit_sync(UART_ID, g_message_1, sizeof(g_message_1), 5000);

    while (1)
    {
        g_rdone = 0;
        app_uart_receive_async(UART_ID, g_rx_buffer, sizeof(g_rx_buffer));
        while (g_rdone == 0);

        g_tdone = 0;
        app_uart_transmit_async(UART_ID, g_tx_buffer, rlen);
        while (g_tdone == 0);
    }
}

int main(void)
{
    app_uart_demo();
}
