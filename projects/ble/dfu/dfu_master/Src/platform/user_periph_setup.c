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
#include "user_app.h"
#include "grx_sys.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "grx_hal.h"
#include "board_SK.h"
#include "dfu_master.h"
#include "user_dfu_m_cfg.h"
#include "app_uart.h"
#include "app_log.h"
#include "flash_scatter_config.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define FW_IMG_PATTERN                 0x4744
#define FW_MAX_IMG_CNT                 10
#define FW_MAX_COMMENTS_CNT            12

#define FW_FIRST_BOOT_INFO_ADDR        FLASH_START_ADDR                 //boot info size 32 Bytes
#define FW_IMG_INFO_ADDR               (FW_FIRST_BOOT_INFO_ADDR + 0x40) //400 Bytes
#define FW_IMG_INFO_SIZE               400
#define DFU_UART_TX_BUFF_SIZE          0x400                            //<Size of app uart tx buffer
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uart_handle_t g_uart_handle;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_uart_rx_data[DFU_DATA_SEND_SIZE];
static uint8_t s_dfu_uart_rx_data[DFU_DATA_SEND_SIZE];
static uint8_t s_master_sts = MASTER_IDLE;
static uint8_t        s_all_img_count   = 0;
static dfu_img_info_t s_fw_img_info[FW_MAX_IMG_CNT];
static uint8_t s_dfu_uart_tx_buffer[DFU_UART_TX_BUFF_SIZE];
static app_uart_params_t uart_param;
void user_prompt_message_output(void);

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void dfu_app_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch(p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            dfu_m_send_data_cmpl_process();
            break;

        case APP_UART_EVT_RX_DATA:
            dfu_m_cmd_prase(s_dfu_uart_rx_data, p_evt->data.size);
            app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_data, DFU_DATA_SEND_SIZE);
            break;

        case APP_UART_EVT_ERROR:
            break;

        default:
            break;
    }
}

void dfu_uart_init(void)
{
    app_uart_tx_buf_t uart_buffer;

    uart_buffer.tx_buf       = s_dfu_uart_tx_buffer;
    uart_buffer.tx_buf_size  = DFU_UART_TX_BUFF_SIZE;

    uart_param.id                   = APP_UART1_ID;
    uart_param.init.baud_rate       = APP_UART_BAUDRATE;
    uart_param.init.data_bits       = UART_DATABITS_8;
    uart_param.init.stop_bits       = UART_STOPBITS_1;
    uart_param.init.parity          = UART_PARITY_NONE;
    uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    uart_param.pin_cfg.rx.type      = APP_UART1_RX_IO_TYPE;
    uart_param.pin_cfg.rx.pin       = APP_UART1_RX_PIN;
    uart_param.pin_cfg.rx.mux       = APP_UART1_RX_PINMUX;
    uart_param.pin_cfg.rx.pull      = APP_UART_RX_PULL;
    uart_param.pin_cfg.tx.type      = APP_UART1_TX_IO_TYPE;
    uart_param.pin_cfg.tx.pin       = APP_UART1_TX_PIN;
    uart_param.pin_cfg.tx.mux       = APP_UART1_TX_PINMUX;
    uart_param.pin_cfg.tx.pull      = APP_UART_TX_PULL;

    app_uart_init(&uart_param, dfu_app_uart_evt_handler, &uart_buffer);

    app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_data, DFU_DATA_SEND_SIZE);
}

static uint8_t user_fw_img_info_get(dfu_img_info_t s_fw_img_info[])
{
    uint8_t i;
    uint8_t img_count = 0;
    uint8_t once_img_size = 40;
    uint8_t read_buffer[FW_IMG_INFO_SIZE];

#ifndef SOC_GR533X
    bool flash_security_status = false;
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(true);
    }
#endif

    hal_flash_read(FW_IMG_INFO_ADDR, read_buffer, FW_IMG_INFO_SIZE);//read decoded data

#ifndef SOC_GR533X
    if(sys_security)
    {
        hal_flash_set_security(flash_security_status);
    }
#endif

    for(i=0; i<FW_MAX_IMG_CNT; i++)
    {
        if(((read_buffer[i * once_img_size + 1] << 8) | read_buffer[i * once_img_size]) == FW_IMG_PATTERN)
        {
            memcpy((uint8_t*)&s_fw_img_info[img_count], &read_buffer[i*once_img_size], once_img_size);

            if(s_fw_img_info[img_count].boot_info.load_addr != APP_CODE_LOAD_ADDR)
            {
                img_count++;
            }
        }
    }

    return img_count;
}

void user_master_status_set(uint8_t status)
{
    s_master_sts = status;
    user_prompt_message_output();
}

uint8_t user_master_status_get(void)
{
    return s_master_sts;
}

void user_master_idle(uint8_t select)
{
    extern uint8_t fast_dfu_mode;

    if(select ==  1)
    {
        fast_dfu_mode = FAST_DFU_MODE_DISABLE;
        user_dfu_m_init(DFU_MODE_UART, DFU_DATA_SEND_SIZE);
        dfu_m_get_info();
        user_master_status_set(MASTER_UART_SELECT_IMG);
    }
    else if(select == 2)
    {
        user_dfu_m_init(DFU_MODE_BLE, DFU_DATA_SEND_SIZE);
        user_master_status_set(MASTER_BLE_SELECT_DEVICE);
        app_start_scan();
    }
}

void user_uart_select_img(uint8_t select)
{
    if (s_all_img_count)
    {
        user_dfu_m_start(&s_fw_img_info[select]);
        user_master_status_set(MASTER_UART_UPDATING);
    }
}

void user_ble_select_device(uint8_t select)
{

}

void user_ble_select_img(uint8_t select)
{
    if (s_all_img_count)
    {
        user_dfu_m_start(&s_fw_img_info[select]);
        user_master_status_set(MASTER_BLE_UPDATING);
    }
}

void user_fast_dfu_mode_set(uint8_t select)
{
    extern uint8_t fast_dfu_mode;

    if (select == 1)
    {
        fast_dfu_mode = FAST_DFU_MODE_DISABLE;
    }
    else if (select == 2)
    {
        fast_dfu_mode = FAST_DFU_MODE_ENABLE;
    }

    user_master_status_set(MASTER_BLE_SELECT_IMG);
}

void uart_cmd_handler(uint8_t* data, uint16_t len)
{
    if(len == 1)
    {
        uint8_t select = data[0] - 0x30;

        switch(s_master_sts)
        {
            case MASTER_IDLE:
                user_master_idle(select);
                break;

            case MASTER_UART_SELECT_IMG:
                user_uart_select_img(select);
                break;

            case MASTER_BLE_SELECT_IMG:
                user_ble_select_img(select);
                break;

            case MASTER_FAST_DFU_MODE_SET:
                user_fast_dfu_mode_set(select);
                break;

            default:
                APP_LOG_WARNING("%s:Master Status(0x%x) Error", __FUNCTION__, s_master_sts);
                break;
        }
    }
}

void user_prompt_message_output(void)
{
    switch(s_master_sts)
    {
        case MASTER_IDLE:
            APP_LOG_DEBUG("Select an Upgrade Mode:\n1.UART\n2.BLE");
            break;

        case MASTER_BLE_CONNECTED:
            APP_LOG_DEBUG("BLE Device Connected");
            otas_c_ctrl_data_send(0, 0x474f4f44);
            dfu_m_get_info();
            break;

        case MASTER_BLE_SELECT_DEVICE:
            APP_LOG_DEBUG("BLE Start Scanning...");
            break;

        case MASTER_UART_SELECT_IMG:
        case MASTER_BLE_SELECT_IMG:
           {
                APP_LOG_DEBUG("Select an Firmware:");
                for(int i = 0; i < s_all_img_count; i++)
                {
                    APP_LOG_DEBUG("%d.%s", i, (char*)&s_fw_img_info[i].comments);
                }
            }
            break;

        case MASTER_FAST_DFU_MODE_SET:
            APP_LOG_DEBUG("Select Fast DFU or Normal DFU:\n1.Normal DFU\n2.Fast DFU");
            break;

        default:
            break;
    }
}

void app_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch(p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            break;

        case APP_UART_EVT_RX_DATA:
            uart_cmd_handler(s_uart_rx_data, p_evt->data.size);
            dfu_m_cmd_prase(s_uart_rx_data, p_evt->data.size);
            app_uart_receive_async(APP_UART_ID, s_uart_rx_data, DFU_DATA_SEND_SIZE);
            break;

        case APP_UART_EVT_ERROR:
            break;

        default:
            break;
    }
}

void app_periph_init(void)
{
    board_init();
    dfu_uart_init();
    user_dfu_m_init(DFU_MODE_UART, DFU_DATA_SEND_SIZE);
    app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_data, DFU_DATA_SEND_SIZE);
    app_uart_receive_async(APP_UART_ID, s_uart_rx_data, DFU_DATA_SEND_SIZE);
    s_all_img_count = user_fw_img_info_get(s_fw_img_info);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}

void uart_data_send(uint8_t *p_data, uint16_t length)
{
    app_uart_transmit_async(APP_UART_ID, p_data, length);
}

void dfu_uart_data_send(uint8_t *p_data, uint16_t length)
{
    app_uart_transmit_async(APP_UART1_ID, p_data, length);
}
