/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
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
#include "user_periph_setup.h"
#include "at_cmd.h"
#include "at_cmd_utils.h"
#include "transport_scheduler.h"
#include "board_SK.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "app_uart.h"
#include "app_uart_dma.h"
#include "app_log.h"
#include "app_assert.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_TX_BUFFER_SIZE      0x2000
#define UART_RX_BUFFER_SIZE      244

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t     s_bd_addr[SYS_BD_ADDR_LEN] = {0x01, 0x02, 0xcf, 0x3e, 0xcb, 0xea};
static app_uart_tx_buf_t s_uart_buffer;
static app_uart_params_t s_uart_param;

static uint8_t s_uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t s_uart_rx_buffer[UART_RX_BUFFER_SIZE];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void log_flush(void)
{
    app_uart_flush(APP_UART_ID);
}

static void app_log_assert_init(void)
{
    app_log_init_t log_init;

    log_init.filter.level                 = APP_LOG_LVL_DEBUG;
    log_init.fmt_set[APP_LOG_LVL_ERROR]   = APP_LOG_FMT_ALL & (~APP_LOG_FMT_TAG);
    log_init.fmt_set[APP_LOG_LVL_WARNING] = APP_LOG_FMT_LVL;
    log_init.fmt_set[APP_LOG_LVL_INFO]    = APP_LOG_FMT_LVL;
    log_init.fmt_set[APP_LOG_LVL_DEBUG]   = APP_LOG_FMT_LVL;

    app_log_init(&log_init, uart_tx_data_send,  log_flush);
    app_assert_init();
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt)
{
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        if (0 == memcmp(s_uart_rx_buffer, "AT:", 3))
        {
            at_cmd_parse(AT_CMD_SRC_UART, s_uart_rx_buffer, p_evt->data.size);
        }
        else
        {
            uart_to_ble_buff_data_push(s_uart_rx_buffer, p_evt->data.size);
        }

        app_uart_dma_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
    }
    else if (APP_UART_EVT_TX_CPLT == p_evt->type)
    {
        update_ble_flow_ctrl_state();
    }
}

void uart_init(uint32_t baud_rate)
{
    s_uart_buffer.tx_buf       = s_uart_tx_buffer;
    s_uart_buffer.tx_buf_size  = UART_TX_BUFFER_SIZE;

    s_uart_param.id                   = APP_UART_ID;
    s_uart_param.init.baud_rate       = baud_rate;
    s_uart_param.init.data_bits       = UART_DATABITS_8;
    s_uart_param.init.stop_bits       = UART_STOPBITS_1;
    s_uart_param.init.parity          = UART_PARITY_NONE;
    s_uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    s_uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    s_uart_param.pin_cfg.rx.type      = APP_UART_RX_IO_TYPE;
    s_uart_param.pin_cfg.rx.pin       = APP_UART_RX_PIN;
    s_uart_param.pin_cfg.rx.mux       = APP_UART_RX_PINMUX;
    s_uart_param.pin_cfg.rx.pull      = APP_UART_RX_PULL;
    s_uart_param.pin_cfg.tx.type      = APP_UART_TX_IO_TYPE;
    s_uart_param.pin_cfg.tx.pin       = APP_UART_TX_PIN;
    s_uart_param.pin_cfg.tx.mux       = APP_UART_TX_PINMUX;
    s_uart_param.pin_cfg.tx.pull      = APP_UART_TX_PULL;
    s_uart_param.dma_cfg.tx_dma_instance = DMA0;
    s_uart_param.dma_cfg.rx_dma_instance = DMA0;
    s_uart_param.dma_cfg.tx_dma_channel = DMA_Channel2;
    s_uart_param.dma_cfg.rx_dma_channel = DMA_Channel3;

    app_uart_dma_deinit(APP_UART_ID);
    app_uart_deinit(APP_UART_ID);

    APP_LOG_INFO("The baud rate update value=%d.",s_uart_param.init.baud_rate);
    app_uart_init(&s_uart_param, app_uart_evt_handler, &s_uart_buffer);
    app_uart_dma_init(&s_uart_param);
    app_uart_dma_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
}

uint32_t app_uart_baud_get(void)
{
    return s_uart_param.init.baud_rate;
}

void uart_tx_data_send(uint8_t *p_data, uint16_t length)
{
    app_uart_dma_transmit_async(APP_UART_ID, p_data, length);
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    app_log_assert_init();
    uart_init(APP_UART_BAUDRATE);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}

