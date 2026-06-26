/**
 ****************************************************************************************
 *
 * @file user_dfu_master.c
 *
 * @brief User DFU master implementation.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2024-2025 GOODIX
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
 ****************************************************************************************
 */
#include "dfu_master.h"
#include "user_dfu_master.h"
#include "user_periph_setup.h"
#include "user_app.h"
#include "hal_flash.h"
#include <string.h>
#include "app_log.h"
#include "app_assert.h"
#include "app_uart.h"
#include "board_SK.h"

#define APP_INFO_PATTERN               0x47525858
#define IMG_INFO_PATTERN               0x4744
#define BUILD_IN_APP_INFO_OFFSET       0x200
#define IMG_INFO_ALIGN_BYTE            16
#define FLASH_PAGE_LEN                 256
#define DFU_FRAME_HEADER_L             0x44U
#define DFU_FRAME_HEADER_H             0x47U
#define DFU_FRAME_DATA_LEN_POS         5U
#define DFU_FRAME_OTHER_LEN            8U

#define DFU_UART_TX_BUFF_SIZE          (DFU_TX_FRAME_MAX)     //<Size of dfu uart tx buffer
#define DFU_UART_RX_BUFF_SIZE          (DFU_RX_FRAME_MAX * 2) //<Size of dfu uart rx buffer

#define FLOOR_ALIGN(addr, size)        ((addr) - ((addr) % (size)))

/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */
static void user_dfu_m_get_img_info(dfu_img_info_t *p_img_info);
static void user_dfu_m_get_img_data(uint32_t addr, uint8_t *p_data, uint16_t length);
static void user_dfu_m_event_handler(dfu_m_event_t event, uint8_t progress);
static uint32_t user_dfu_m_get_time(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static dfu_img_info_t s_image_info;  // The new firmware image information
static uint32_t s_fw_save_addr;      // The new firmware save address on the master
static uint32_t s_fw_max_size;       // The new firmware max size

#if DFU_UART_ENABLE
static app_uart_params_t s_dfu_uart_param;
static uint8_t s_dfu_uart_tx_buffer[DFU_UART_TX_BUFF_SIZE];
static uint8_t s_dfu_uart_rx_data[DFU_UART_RX_BUFF_SIZE];
#endif

static dfu_m_func_cfg_t s_dfu_m_func_cfg =
{
    .dfu_m_get_img_info  = user_dfu_m_get_img_info,
    .dfu_m_get_img_data  = user_dfu_m_get_img_data,
    .dfu_m_event_handler = user_dfu_m_event_handler,
    .dfu_m_get_time      = user_dfu_m_get_time,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static bool find_img_info(uint8_t *p_data, uint32_t len, dfu_img_info_t *p_img_info)
{
    dfu_img_info_t *p_temp_img_info;
    for (uint32_t i = 0; i <= (len - sizeof(dfu_img_info_t)); i += IMG_INFO_ALIGN_BYTE)
    {
        //lint -e9087 -e740 [required] Casting is safe
        p_temp_img_info = (dfu_img_info_t *)&p_data[i];
        if ((IMG_INFO_PATTERN == p_temp_img_info->pattern) &&
            (p_temp_img_info->boot_info.load_addr >= DFU_SLAVE_FLASH_START_ADDR) &&
            (p_temp_img_info->boot_info.load_addr == p_temp_img_info->boot_info.run_addr))
        {
            memcpy(p_img_info, p_temp_img_info, sizeof(dfu_img_info_t));
            return true;
        }
    }
    return false;
}

bool user_get_img_info(void)
{
    dfu_img_info_t *p_img_info = &s_image_info;
    uint32_t fw_data[(FLASH_PAGE_LEN + sizeof(dfu_img_info_t)) / sizeof(uint32_t)];
    memset(fw_data, 0x00, sizeof(fw_data));

    uint32_t addr = DFU_FW_SAVE_MASTER_ADDR + DFU_FW_SIZE_MAX - sizeof(fw_data);
    addr = FLOOR_ALIGN(addr, IMG_INFO_ALIGN_BYTE);
    while (addr > DFU_FW_SAVE_MASTER_ADDR)
    {
        memset(p_img_info, 0x00, sizeof(dfu_img_info_t));
        memset(fw_data, 0x00, sizeof(fw_data));
        user_dfu_m_get_img_data(addr, (uint8_t *)fw_data, sizeof(fw_data));
        if (find_img_info((uint8_t *)fw_data, sizeof(fw_data), p_img_info))
        {
        #if APP_LOG_ENABLE
            APP_LOG_DEBUG("Get image info OK:");
            APP_LOG_DEBUG("Load Address = 0x%08x", p_img_info->boot_info.load_addr);
            APP_LOG_DEBUG("Run Address  = 0x%08x", p_img_info->boot_info.run_addr);
            APP_LOG_DEBUG("Bin Size     = 0x%08x", p_img_info->boot_info.bin_size);
            APP_LOG_DEBUG("CheckSum     = 0x%08x", p_img_info->boot_info.check_sum);
            char comments[sizeof(((dfu_img_info_t *)0)->comments) + 1U];
            memcpy(comments, p_img_info->comments, sizeof(((dfu_img_info_t *)0)->comments));
            comments[sizeof(comments) - 1U] = '\0';
            APP_LOG_DEBUG("Comments     = %s", comments);
        #endif /* APP_LOG_ENABLE */
            return true;
        }
        else
        {
            addr -= sizeof(fw_data) - sizeof(dfu_img_info_t);
        }
    }
    APP_LOG_DEBUG("Get image info error");
    return false;
}

static void user_dfu_m_get_img_info(dfu_img_info_t *img_info)
{
    memcpy(img_info, &s_image_info, sizeof(dfu_img_info_t));
}

static void user_dfu_m_get_img_data(uint32_t addr, uint8_t *p_data, uint16_t length)
{
#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526)
    bool flash_security_status = false;
    uint32_t sys_security = sys_security_enable_status_check();
    if (sys_security)
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(false);
    }
#endif

    hal_flash_read(addr, p_data, length);

#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526)
    if (sys_security)
    {
        hal_flash_set_security(flash_security_status);
    }
#endif
}

static void user_dfu_m_event_handler(dfu_m_event_t event, uint8_t progress)
{
    switch (event)
    {
        case PRO_START_SUCCESS:
            APP_LOG_DEBUG("Upgrade Start");
            break;

        case PRO_FLASH_SUCCESS:
            APP_LOG_DEBUG("Upgrade Progress %d%%", progress);
            break;

        case PRO_END_SUCCESS:
            APP_LOG_DEBUG("Upgrade Success");
            user_master_status_set(MASTER_DFU_FW_INFO_SET);
            break;

        case ERASE_START_SUCCESS:
            APP_LOG_DEBUG("Erase Flash Start");
            break;

        case ERASE_SUCCESS:
            APP_LOG_DEBUG("Erase Flash Success");
            break;

        case FAST_DFU_PRO_FLASH_SUCCESS:
            APP_LOG_DEBUG("Upgrade Progress %d%%", progress);
            break;

        case ERASE_END_SUCCESS:
            APP_LOG_DEBUG("Erase Flash End");
            break;

        case DFU_ACK_TIMEOUT:
            APP_LOG_DEBUG("DFU TIMEOUT");
            dfu_m_parse_state_reset();
            user_master_status_set(MASTER_DFU_FW_INFO_SET);
            break;

        case PRO_END_FAIL:
            APP_LOG_DEBUG("Program end fail");
            dfu_m_parse_state_reset();
            user_master_status_set(MASTER_DFU_FW_INFO_SET);
            break;

        default:
            APP_LOG_ERROR("dfu_m_event_handler error event[0x%x]", event);
            break;
    }
}

static uint32_t user_dfu_m_get_time(void)
{
    return systick_get_time();
}

#if DFU_UART_ENABLE
static bool is_dfu_frame(const uint8_t *p_data)
{
    // Just check frame header
    if ((DFU_FRAME_HEADER_L == p_data[0]) && (DFU_FRAME_HEADER_H == p_data[1]))
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void dfu_app_uart_evt_handler(app_uart_evt_t * p_evt)
{
    static uint32_t rx_size = 0;
    static uint32_t frame_len = 0;
    static bool is_new_frame = true;

    switch (p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
        #if DFU_ASYNC_TX_ENABLE
            dfu_m_send_data_cmpl_process();
        #endif
            break;

        case APP_UART_EVT_RX_DATA:
            if ((is_new_frame) && (rx_size > DFU_FRAME_DATA_LEN_POS))
            {
                if (is_dfu_frame(s_dfu_uart_rx_data))
                {
                    const uint32_t data_len =(s_dfu_uart_rx_data[DFU_FRAME_DATA_LEN_POS  - 1U]) + (s_dfu_uart_rx_data[DFU_FRAME_DATA_LEN_POS] << 8U);
                    frame_len = data_len + DFU_FRAME_OTHER_LEN;
                    is_new_frame = false;
                }
                else
                {
                    // Not DFU frame, ignore the received data
                    rx_size = 0;
                }
            }

            if ((rx_size == frame_len) && frame_len)
            {
                dfu_m_cmd_parse(s_dfu_uart_rx_data, p_evt->data.size);
                memset(s_dfu_uart_rx_data, 0, rx_size);
                is_new_frame = true;
                rx_size = 0;
                frame_len = 0;
            }

            dfu_m_cmd_parse(s_dfu_uart_rx_data, p_evt->data.size);
            app_uart_receive_async(APP_UART1_ID, &s_dfu_uart_rx_data[rx_size], sizeof(s_dfu_uart_rx_data) - rx_size);
            break;

        case APP_UART_EVT_ERROR:
            // RX error, process received data, and start receive new frame.
            dfu_m_cmd_parse(s_dfu_uart_rx_data, rx_size);
            is_new_frame = true;
            rx_size = 0;
            frame_len = 0;
            app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_data, sizeof(s_dfu_uart_rx_data));
            break;

        default:
            break;
    }
}

static void dfu_uart_init(void)
{
    app_uart_tx_buf_t uart_buffer;

    uart_buffer.tx_buf       = s_dfu_uart_tx_buffer;
    uart_buffer.tx_buf_size  = DFU_UART_TX_BUFF_SIZE;

    s_dfu_uart_param.id                   = APP_UART1_ID;
    s_dfu_uart_param.init.baud_rate       = APP_UART1_BAUDRATE;
    s_dfu_uart_param.init.data_bits       = UART_DATABITS_8;
    s_dfu_uart_param.init.stop_bits       = UART_STOPBITS_1;
    s_dfu_uart_param.init.parity          = UART_PARITY_NONE;
    s_dfu_uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    s_dfu_uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    s_dfu_uart_param.pin_cfg.rx.type      = APP_UART1_RX_IO_TYPE;
    s_dfu_uart_param.pin_cfg.rx.pin       = APP_UART1_RX_PIN;
    s_dfu_uart_param.pin_cfg.rx.mux       = APP_UART1_RX_PINMUX;
    s_dfu_uart_param.pin_cfg.rx.pull      = APP_UART1_RX_PULL;
    s_dfu_uart_param.pin_cfg.tx.type      = APP_UART1_TX_IO_TYPE;
    s_dfu_uart_param.pin_cfg.tx.pin       = APP_UART1_TX_PIN;
    s_dfu_uart_param.pin_cfg.tx.mux       = APP_UART1_TX_PINMUX;
    s_dfu_uart_param.pin_cfg.tx.pull      = APP_UART1_TX_PULL;

    app_uart_init(&s_dfu_uart_param, dfu_app_uart_evt_handler, &uart_buffer);

    app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_data, sizeof(s_dfu_uart_rx_data));
    memset(s_dfu_uart_rx_data, 0, sizeof(s_dfu_uart_rx_data));
}

static void dfu_uart_data_send(uint8_t *p_data, uint16_t length)
{
#if DFU_ASYNC_TX_ENABLE
    app_uart_transmit_async(APP_UART1_ID, p_data, length);
#else
    app_uart_transmit_sync(APP_UART1_ID, p_data, length, 1000);
#endif
}
#endif /* DFU_UART_ENABLE */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void user_dfu_m_init(uint8_t dfu_mode, uint16_t once_send_size)
{
#if DFU_UART_ENABLE
    if (UART_UPGRADE_MODE == dfu_mode)
    {
        static bool dfu_uart_init_flag = false;
        if (!dfu_uart_init_flag)
        {
            dfu_uart_init_flag = true;
            dfu_uart_init();
        }
        s_dfu_m_func_cfg.dfu_m_send_data = dfu_uart_data_send;
    }
#endif

#if DFU_BLE_ENABLE
    if (BLE_UPGRADE_MODE == dfu_mode)
    {
        static bool dfu_ble_init_flag = false;
        if (!dfu_ble_init_flag)
        {
            dfu_ble_init_flag = true;
            // Initialize BLE Stack.
            ble_stack_init(ble_evt_handler, &heaps_table);
        }
        s_dfu_m_func_cfg.dfu_m_send_data = ble_data_send;
    }
#endif

    dfu_m_init(&s_dfu_m_func_cfg, once_send_size);
}

void set_fw_save_addr(uint32_t addr)
{
    s_fw_save_addr = addr;
}

uint32_t get_fw_save_addr(void)
{
    return s_fw_save_addr;
}

uint32_t get_fw_max_size(void)
{
    return s_fw_max_size;
}

void set_fw_max_size(uint32_t size)
{
    s_fw_max_size = size;
}
