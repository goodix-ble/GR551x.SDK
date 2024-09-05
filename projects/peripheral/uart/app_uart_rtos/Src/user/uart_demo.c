/**
 *****************************************************************************************
 *
 * @file test_uart.c
 *
 * @brief UART test demo based on RTOS.
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
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "app_uart.h"
#include "board_SK.h"

 /*
 * DEFINES
 *****************************************************************************************
 */
#define APP_TASK_STACK_SIZE          512
#define UART_DATA_LEN                4095
#define UART_ID                      APP_UART1_ID

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;
static SemaphoreHandle_t rx_sync_sem = NULL;
static uint8_t g_tx_buffer[UART_DATA_LEN]   = {0};
static uint8_t g_rx_buffer[UART_DATA_LEN]   = {0};
static uint8_t g_ring_buffer[UART_DATA_LEN] = {0};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
app_uart_params_t uart_params = {
    .id      = UART_ID,
    .pin_cfg = {
           .tx = {
              .type = APP_UART1_TX_IO_TYPE,
              .mux  = APP_UART1_TX_PINMUX,
              .pin  = APP_UART1_TX_PIN,
              .pull = APP_IO_PULLUP,
           },
           .rx = {
              .type = APP_UART1_RX_IO_TYPE,
              .mux  = APP_UART1_RX_PINMUX,
              .pin  = APP_UART1_RX_PIN,
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
static void app_uart_callback(app_uart_evt_t *p_evt)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        xSemaphoreGiveFromISR(rx_sync_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    if (p_evt->type == APP_UART_EVT_ERROR)
    {

    }
}

static void init_write_buffer(void)
{
    int i = 0;
    for (i = 0; i < UART_DATA_LEN; i++)
    {
        g_tx_buffer[i] = i;
    }
}

void uart_demo_init(void)
{
    app_drv_err_t ret = 0;
    app_uart_tx_buf_t uart_buffer = {0};

    uart_buffer.tx_buf = g_ring_buffer;
    uart_buffer.tx_buf_size = sizeof(g_ring_buffer);

    rx_sync_sem = xSemaphoreCreateBinary();
    if (rx_sync_sem == NULL)
    {
        printf("Create uart rx sync semaphore failed.\r\n");
        return;
    }
    ret = app_uart_init(&uart_params, app_uart_callback, &uart_buffer);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("UART initialization failed.\r\n");
    }
}

void uart_demo_process(void)
{
    uint32_t count = 0;
    uint32_t ret;

    init_write_buffer();

    while (1)
    {
        memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
        app_uart_receive_async(UART_ID, g_rx_buffer, UART_DATA_LEN);
        app_uart_transmit_sync(UART_ID, g_tx_buffer, UART_DATA_LEN, 1000);
        ret = xSemaphoreTake(rx_sync_sem,  pdMS_TO_TICKS(300));
        if (ret != pdTRUE)
        {
            printf("Take the uart rx semaphore failed. ret=0x%x\r\n", ret);
            continue;
        }

        if (memcmp(g_rx_buffer, g_tx_buffer, UART_DATA_LEN) == 0)
        {
            if ((count % 5) == 0)
            {
                printf("UART receive data verify success.\r\n");
            }
        }
        else
        {
             printf("UART receive data failed!\r\n");
        }

        count++; 
        vTaskDelay(10);
    }
}

static void uart_task(void *arg)
{
    uart_demo_init();
    uart_demo_process();
}

void uart_demo_task(void)
{
    printf("APP UART RTOS example.\r\n");
    xTaskCreate(uart_task, "uart_task", APP_TASK_STACK_SIZE, NULL,  configMAX_PRIORITIES - 1, &my_task_handle);
}
