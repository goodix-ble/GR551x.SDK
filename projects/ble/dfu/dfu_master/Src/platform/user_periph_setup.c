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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "user_periph_setup.h"
#include "user_app.h"
#include "grx_sys.h"
#include "custom_config.h"
#include "grx_hal.h"
#include "board_SK.h"
#include "dfu_master.h"
#include "user_dfu_master.h"
#include "app_uart.h"
#include "app_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_CONSOLE_RX_BUF_SIZE    64
// Systick generates an interrupt once per SYSTICK_IRQ_PERIOD
#define SYSTICK_IRQ_PERIOD          10  //Unit: ms

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_master_sts = MASTER_DFU_FW_INFO_SET;
static uint8_t s_uart_rx_cmd[UART_CONSOLE_RX_BUF_SIZE];
static uint32_t s_uart_console_rx_size = 0;
static size_t s_systick_irq_count = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
#if DFU_UART_ENABLE
static void user_uart_dfu_start(void)
{
    dfu_m_start();
    user_master_status_set(MASTER_UART_UPDATING);
}
#endif

#if DFU_BLE_ENABLE
static void user_ble_dfu_start(void)
{
    dfu_m_start();
    user_master_status_set(MASTER_BLE_UPDATING);
}
#endif

static void uart_console_get_dfu_fw_info(uint8_t *input, uint16_t size)
{
    char *token;
    char str_copy[UART_CONSOLE_RX_BUF_SIZE];
    strncpy(str_copy, (char *)input, sizeof(str_copy) - 1);
    str_copy[sizeof(str_copy) - 1] = '\0';
    uint32_t fw_addr = 0;
    uint32_t fw_max_size = 0;
    token = strtok(str_copy, ",");
    if (token != NULL)
    {
        fw_addr = (uint32_t)strtol(token, NULL, 16);
        set_fw_save_addr(fw_addr);
    }

    token = strtok(NULL, ",");
    if (token != NULL)
    {
        fw_max_size = (uint32_t)strtol(token, NULL, 16);
        set_fw_max_size(fw_max_size);
    }

    APP_LOG_DEBUG("FW save address: 0x%08x, max size 0x%x", fw_addr, fw_max_size);
    if (user_get_img_info())
    {
        user_master_status_set(MASTER_SELECT_UART_OR_BLE);
    }
    else
    {
        user_master_status_set(MASTER_DFU_FW_INFO_SET);
    }
}

static void user_master_select_uart_or_ble(uint8_t select)
{
    if (select == UART_UPGRADE_MODE)
    {
    #if DFU_UART_ENABLE
        #if DFU_BLE_ENABLE
        dfu_m_fast_dfu_mode_set(FAST_DFU_MODE_DISABLE);
        #endif
        user_dfu_m_init(UART_UPGRADE_MODE, DFU_SEND_SIZE_MAX);
        user_master_status_set(MASTER_UART_DFU_START);
        user_uart_dfu_start();
    #else
        APP_LOG_WARNING("NOT SUPPORT UART MODE");
    #endif
    }
    else if (select == BLE_UPGRADE_MODE)
    {
    #if DFU_BLE_ENABLE
        user_dfu_m_init(BLE_UPGRADE_MODE, DFU_SEND_SIZE_MAX);
        user_master_status_set(MASTER_BLE_DFU_SCAN);
        app_start_scan();
    #else
        APP_LOG_WARNING("NOT SUPPORT BLE MODE");
        uart_console_reset();
    #endif
    }
}

#if DFU_BLE_ENABLE
static void user_fast_dfu_mode_set(uint8_t select)
{
    if (select == 1)
    {
        dfu_m_fast_dfu_mode_set(FAST_DFU_MODE_DISABLE);
    }
    else if (select == 2)
    {
        dfu_m_fast_dfu_mode_set(FAST_DFU_MODE_ENABLE);
    }

    user_master_status_set(MASTER_BLE_DFU_START);
}
#endif

static void uart_console_handler(uint8_t* data, uint16_t len)
{
    if (len)
    {
        uint8_t select = data[0] - '0';
        switch (s_master_sts)
        {
            case MASTER_DFU_FW_INFO_SET:
                uart_console_get_dfu_fw_info(data, len);
                break;
            case MASTER_SELECT_UART_OR_BLE:
                user_master_select_uart_or_ble(select);
                break;
        #if DFU_BLE_ENABLE
            case MASTER_FAST_DFU_MODE_SET:
                user_fast_dfu_mode_set(select);
                break;
        #endif

            default:
                APP_LOG_WARNING("%s:Master Status(0x%x) Error", __FUNCTION__, s_master_sts);
                break;
        }
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void user_master_status_set(uint8_t status)
{
    s_master_sts = status;
    switch (status)
    {
        case MASTER_SELECT_UART_OR_BLE:
            APP_LOG_DEBUG("Select an Upgrade Mode and Start:\n1.UART\n2.BLE");
            break;

        case MASTER_BLE_CONNECTED:
        #if DFU_BLE_ENABLE
            APP_LOG_DEBUG("BLE Device Connected");
            otas_c_ctrl_data_send(0, OTAS_C_CTRL_PT_OP_DFU_ENTER);
        #endif
            break;

        case MASTER_BLE_DFU_START:
        #if DFU_BLE_ENABLE
            user_ble_dfu_start();
        #endif
            break;

        case MASTER_BLE_DFU_SCAN:
            APP_LOG_DEBUG("BLE Start Scanning...");
            break;

        case MASTER_DFU_FW_INFO_SET:
            APP_LOG_DEBUG("Input the FW save address and maximum size, e.g:\r\n"
                          "0x00240000,0x30000");
            break;

        case MASTER_FAST_DFU_MODE_SET:
            APP_LOG_DEBUG("Select Fast DFU or Normal DFU:\n1.Normal DFU\n2.Fast DFU");
            break;

        default:
            break;
    }
}

uint8_t user_master_status_get(void)
{
    return s_master_sts;
}

// uart console irq callback
void app_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            break;

        case APP_UART_EVT_RX_DATA:
            s_uart_console_rx_size = p_evt->data.size;
            app_uart_receive_async(APP_UART_ID, s_uart_rx_cmd, sizeof(s_uart_rx_cmd));
            break;

        case APP_UART_EVT_ERROR:
            app_uart_receive_async(APP_UART_ID, s_uart_rx_cmd, sizeof(s_uart_rx_cmd));
            break;

        default:
            break;
    }
}

void uart_console_init(void)
{
    bsp_log_init();
    app_uart_receive_async(APP_UART_ID, s_uart_rx_cmd, sizeof(s_uart_rx_cmd));
}

void uart_console_reset(void)
{
    user_master_status_set(MASTER_DFU_FW_INFO_SET);
    dfu_m_parse_state_reset();
}

void uart_console_schedule(void)
{
    if (s_uart_console_rx_size)
    {
        uart_console_handler(s_uart_rx_cmd, s_uart_console_rx_size);
        s_uart_console_rx_size = 0;
        memset(s_uart_rx_cmd, 0, sizeof(s_uart_rx_cmd));
    }
}

void systick_init(void)
{
    SystemCoreUpdateClock();
    // Generates an interrupt once per SYSTICK_IRQ_PERIOD
    SysTick_Config(SystemCoreClock/(1000/SYSTICK_IRQ_PERIOD));
    hal_nvic_enable_irq(SysTick_IRQn);
}

uint32_t systick_get_time(void)
{
    return s_systick_irq_count*(SYSTICK_IRQ_PERIOD);
}

void SysTick_Handler(void)
{
    s_systick_irq_count++;
}
