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
#include "gr_includes.h"
#include "custom_config.h"
#include "bootloader_config.h"
#include "otas.h"
#include "app_log.h"
#include "dfu_port.h"
#include "user_periph_setup.h"
#include "board_SK.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if BOOTLOADER_DFU_UART_ENABLE
#define DFU_FRAME_HEADER_L         0x44U
#define DFU_FRAME_HEADER_H         0x47U
#define DFU_FRAME_DATA_LEN_POS     5U
#define DFU_FRAME_OTHER_LEN        8U
#define DFU_UART_RX_BUFF_SIZE      0x800U
#define DFU_UART_TX_BUFF_SIZE      0x800U

#ifndef DFU_UART_ISP_TIMEOUT
#define DFU_UART_ISP_TIMEOUT       20  // Timeout for RX ISP commands or TX response. Unit: ms
#endif

static app_uart_params_t s_dfu_uart_param;
static uint8_t s_dfu_uart_rx_buffer[DFU_UART_RX_BUFF_SIZE];
static uint8_t s_dfu_uart_tx_buffer[DFU_UART_TX_BUFF_SIZE];
#endif

// Systick generates an interrupt once per SYSTICK_IRQ_PERIOD
#define SYSTICK_IRQ_PERIOD          100  //unit: ms
static size_t s_systick_irq_count = 0;
static size_t s_bootloader_timeout_start_time = 0;

#if BOOTLOADER_WDT_ENABLE
static aon_wdt_handle_t     bootloader_wdt_handle_0;
#if defined(HAL_AON_WDT_INSTANCE_MAX) && (HAL_AON_WDT_INSTANCE_MAX > 1)
static aon_wdt_handle_t     bootloader_wdt_handle_1;
#endif
#endif

#if BOOTLOADER_DFU_ENABLE
static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback    = dfu_programing_callback,
    .dfu_program_end_callback   = dfu_program_end_callback,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("Start DFU OTA.");
}

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("DFU OTA %d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("DFU OTA complete.");
}
#endif

/**
 *****************************************************************************************
 * @brief Initialize watch dog.
 *****************************************************************************************
 */
#if BOOTLOADER_WDT_ENABLE
static void bootloader_wdt_init(void)
{
    extern uint32_t SystemSlowClock;
#if defined(HAL_AON_WDT_INSTANCE_MAX) && (HAL_AON_WDT_INSTANCE_MAX > 1)
    bootloader_wdt_handle_0.p_instance = AON_WDT0;
    bootloader_wdt_handle_0.init.prescaler = AON_WDT_CLK_DIV16;
#endif
    bootloader_wdt_handle_0.init.counter = 20000;
    bootloader_wdt_handle_0.init.alarm_counter = 0;

#if defined(SOC_GR5515)
    bootloader_wdt_handle_0.init.counter = 20 * 32768 - 1;
#endif
#if defined(SOC_GR5526) || defined(SOC_GR5X25) || defined(SOC_GR533X) || defined(SOC_GR5405)
    bootloader_wdt_handle_0.SystemCoreLowClock = &SystemSlowClock;
#endif

    hal_aon_wdt_init(&bootloader_wdt_handle_0);

#if defined(HAL_AON_WDT_INSTANCE_MAX) && (HAL_AON_WDT_INSTANCE_MAX > 1)
    bootloader_wdt_handle_1.p_instance = AON_WDT1;
    bootloader_wdt_handle_1.init.prescaler = AON_WDT_CLK_DIV16;
    bootloader_wdt_handle_1.init.counter = 20000;
    bootloader_wdt_handle_1.init.alarm_counter = 0;
    hal_aon_wdt_init(&bootloader_wdt_handle_1);
#endif
}
#endif

// systick will be disabled in sys_firmware_jump()
static void systick_init(void)
{
    SystemCoreUpdateClock();
    // Generates an interrupt once per SYSTICK_IRQ_PERIOD
    SysTick_Config(SystemCoreClock/(1000/SYSTICK_IRQ_PERIOD));
    hal_nvic_enable_irq(SysTick_IRQn);
}

static void systick_deinit(void)
{
    SysTick->CTRL = 0;
}

#if BOOTLOADER_DFU_UART_ENABLE
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

static void dfu_uart_evt_handler(app_uart_evt_t * p_evt)
{
    static uint32_t rx_size = 0;
    static uint32_t frame_len = 0;
    static bool is_new_frame = true;

    bootloader_timeout_refresh();

    switch (p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            break;

        case APP_UART_EVT_RX_DATA:
            rx_size += p_evt->data.size;

            if ((is_new_frame) && (rx_size > DFU_FRAME_DATA_LEN_POS))
            {
                if (is_dfu_frame(s_dfu_uart_rx_buffer))
                {
                    const uint32_t data_len =(s_dfu_uart_rx_buffer[DFU_FRAME_DATA_LEN_POS  - 1U]) + (s_dfu_uart_rx_buffer[DFU_FRAME_DATA_LEN_POS] << 8U);
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
                dfu_uart_receive_data_process(s_dfu_uart_rx_buffer, rx_size);
                memset(s_dfu_uart_rx_buffer, 0, rx_size);
                is_new_frame = true;
                rx_size = 0;
                frame_len = 0;
            }

            app_uart_receive_async(APP_UART1_ID, &s_dfu_uart_rx_buffer[rx_size], DFU_UART_RX_BUFF_SIZE - rx_size);
            break;

        case APP_UART_EVT_ERROR:
            // RX error, process received data, and start receive new frame.
            dfu_uart_receive_data_process(s_dfu_uart_rx_buffer, rx_size);
            is_new_frame = true;
            rx_size = 0;
            frame_len = 0;
            app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, DFU_UART_RX_BUFF_SIZE);
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

    app_uart_init(&s_dfu_uart_param, dfu_uart_evt_handler, &uart_buffer);
}

static void dfu_uart_deinit(void)
{
    app_uart_deinit(APP_UART1_ID);
}

static void uart_send_data(uint8_t *data, uint16_t size)
{
    app_uart_transmit_sync(APP_UART1_ID, data, size, 1000);
}

bool bootloader_uart_isp_check(void)
{
    bool ret = false;
    const uint8_t isp_cmd[]     = {0x44, 0x47, 0x00, 0x00, 0x06, 0x00, 0x47, 0x52, 0x35, 0x35, 0x31, 0x78, 0xB2, 0x01};
    const uint8_t isp_cmd_rsp[] = {0x44, 0x47, 0x00, 0x00, 0x07, 0x00, 0x01, 0x47, 0x52, 0x35, 0x35, 0x31, 0x78, 0xB4, 0x01};
    uint8_t rx_buf[sizeof(isp_cmd)*2];
    memset(rx_buf, 0, sizeof(rx_buf));
    app_uart_receive_sync(APP_UART1_ID, rx_buf, sizeof(rx_buf), DFU_UART_ISP_TIMEOUT);
    for (uint32_t i = 0; i<= (sizeof(rx_buf) - sizeof(isp_cmd)); i++)
    {
        if (0 == memcmp(&rx_buf[i], isp_cmd, sizeof(isp_cmd)))
        {
            // RX ISP command
            ret = true;
            break;
        }
    }

    app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, sizeof(s_dfu_uart_rx_buffer));
    if (ret)
    {
        // TX ISP command response
        app_uart_transmit_sync(APP_UART1_ID, (uint8_t*)isp_cmd_rsp, sizeof(isp_cmd_rsp), DFU_UART_ISP_TIMEOUT);
    }
    return ret;
}
#endif /* BOOTLOADER_DFU_UART_ENABLE */

void app_periph_init(void)
{
#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526)
    // Turn on the clock of encryption module.
    app_boot_turn_on_encrypt_clock();
#endif
#if defined(SOC_GR5405)
    // Use HFXO as system clock source. Do not auto switch between HFXO and HFRC.
    hal_clock_request_xo_osc(XO_REQUEST_DEVICE_NUM_USER);
#endif

#if BOOTLOADER_DFU_UART_ENABLE
    dfu_uart_init();
#endif
#if BOOTLOADER_WDT_ENABLE
    bootloader_wdt_init();
#endif

    systick_init();

#if APP_LOG_ENABLE
    bsp_log_init();
#endif

    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);

#if BOOTLOADER_DFU_ENABLE
#if BOOTLOADER_DFU_UART_ENABLE
    dfu_port_init(uart_send_data, DFU_FW_SAVE_ADDR, &dfu_pro_call);
#else
    dfu_port_init(NULL, DFU_FW_SAVE_ADDR, &dfu_pro_call);
#endif
#endif
}

void app_periph_deinit(void)
{
    systick_deinit();

#if BOOTLOADER_DFU_UART_ENABLE
    dfu_uart_deinit();
#endif

#if BOOTLOADER_DFU_BLE_ENABLE
    // BLE Stack deinit by SDK
#endif
}

void bootloader_wdt_refresh(void)
{
#if BOOTLOADER_WDT_ENABLE
    hal_aon_wdt_refresh(&bootloader_wdt_handle_0);
#if defined(HAL_AON_WDT_INSTANCE_MAX) && (HAL_AON_WDT_INSTANCE_MAX > 1)
    hal_aon_wdt_refresh(&bootloader_wdt_handle_1);
#endif
#endif
}

void cortex_backtrace_fault_handler(void)
{
    hal_nvic_system_reset();
    while(1);
}

void SysTick_Handler(void)
{
    s_systick_irq_count++;
}

size_t bootloader_get_time(void)
{
    return s_systick_irq_count*(SYSTICK_IRQ_PERIOD);
}

void bootloader_timeout_refresh(void)
{
    s_bootloader_timeout_start_time = bootloader_get_time();
}

size_t bootloader_timeout_get_start_time(void)
{
    return s_bootloader_timeout_start_time;
}
