/**
  ****************************************************************************************
  * @file    hci_uart.c
  * @author  BLE SDK Team
  * @brief   H4TL UART driver.
  ****************************************************************************************
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
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>        // standard definition
#include "board_SK.h"
#include "custom_config.h"
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "app_uart.h"
#include "hci_uart.h"      // uart definition
/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */

static hci_uart_call_t hci_uart_api =
{
#if (HCI_UART_GRP_ID == 0)
    hci_uart0_init,
    hci_uart0_flow_on,
    hci_uart0_flow_off,
    hci_uart0_finish_transfers,
    hci_uart0_read,
    hci_uart0_write,
#elif (HCI_UART_GRP_ID == 1)
    hci_uart1_init,
    hci_uart1_flow_on,
    hci_uart1_flow_off,
    hci_uart1_finish_transfers,
    hci_uart1_read,
    hci_uart1_write,
#endif
};

 /* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
typedef struct
{
    /// call back function pointer
    void (*callback) (void*, uint8_t);
    /// dummy data pointer returned to callback when operation is over.
    void *p_dummy;
} uart_txrxchannel_t;

/// UART environment structure
typedef struct
{
    /// tx channel
    uart_txrxchannel_t tx;
    /// rx channel
    uart_txrxchannel_t rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
} uart_env_tag_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// uart environment structure
static uart_env_tag_t uart_env;
static app_uart_params_t s_params;
static app_uart_tx_buf_t s_hci_uart_buffer;
static uint8_t s_hci_buffer[256] = {0};
/*
 * PRIVATED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void hci_uart_callback(app_uart_id_t id, app_uart_evt_t *p_evt)
{
    void *data = NULL;
    void (*callback) (void*, uint8_t) = NULL;

    if (APP_UART_EVT_TX_CPLT == p_evt->type)
    {
        // Retrieve callback pointer
        callback = uart_env.tx.callback;
        data     = uart_env.tx.p_dummy;

        if(callback != NULL)
        {
            // Clear callback pointer
            uart_env.tx.callback = NULL;
            uart_env.tx.p_dummy  = NULL;

            // Call handler
            callback(data, 0);
        }
    }
    else if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        // Retrieve callback pointer
        callback = uart_env.rx.callback;
        data     = uart_env.rx.p_dummy;

        if(callback != NULL)
        {
            // Clear callback pointer
            uart_env.rx.callback = NULL;
            uart_env.rx.p_dummy  = NULL;
            
            // Call handler
            callback(data, 0);
        }
    }
    else if (APP_UART_EVT_ERROR == p_evt->type)
    {
        uint8_t tmp;
        uart_handle_t *p_uart = app_uart_get_handle(id);

        /* dummy read to clear RLS interrupt */
        tmp = ll_uart_receive_data8(p_uart->p_instance);
        (void) tmp;

        /* Flush RX_FIFO */
        ll_uart_flush_rx_fifo(p_uart->p_instance);

        /* If Overrun error occurs. */
        if(p_evt->data.error_code & HAL_UART_ERROR_OE)
        {
            p_evt->type = APP_UART_EVT_RX_DATA;
            hci_uart_callback(id, p_evt);
        }
    }
}

static void hci_uart0_callback(app_uart_evt_t *p_evt)
{
    hci_uart_callback(APP_UART_ID_0, p_evt);
}

static void hci_uart1_callback(app_uart_evt_t *p_evt)
{
    hci_uart_callback(APP_UART_ID_1, p_evt);
}

static void hci_uart_init(app_uart_id_t id, void (*callback)(app_uart_evt_t*))
{
    s_params.id = id;

    s_params.pin_cfg.tx.type = APP_HCI_UART_TRN_PORT;
    s_params.pin_cfg.tx.mux = APP_HCI_UART_TX_PINMUX;
    s_params.pin_cfg.tx.pin = APP_HCI_UART_TX_PIN;
    s_params.pin_cfg.rx.type = APP_HCI_UART_TRN_PORT;
    s_params.pin_cfg.rx.mux = APP_HCI_UART_RX_PINMUX;
    s_params.pin_cfg.rx.pin = APP_HCI_UART_RX_PIN;
    #if HCI_UART_FLOW_ON == 1
    params.pin_cfg.cts.type = APP_HCI_UART_FLOW_PORT;
    params.pin_cfg.cts.mux = APP_HCI_UART_CTS_PINMUX;
    params.pin_cfg.cts.pin = APP_HCI_UART_CTS_PIN;
    params.pin_cfg.rts.type = APP_HCI_UART_FLOW_PORT;
    params.pin_cfg.rts.mux = APP_HCI_UART_RTS_PINMUX;
    params.pin_cfg.rts.pin = APP_HCI_UART_RTS_PIN;
    #endif //HCI_UART_FLOW_ON

    s_params.init.baud_rate = APP_HCI_UART_BAUDRATE;
    s_params.init.data_bits       = UART_DATABITS_8;
    s_params.init.stop_bits       = UART_STOPBITS_1;
    s_params.init.parity          = UART_PARITY_NONE;
    #if HCI_UART_FLOW_ON == 1
    params.init.hw_flow_ctrl    = UART_HWCONTROL_RTS_CTS;
    #else
    s_params.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    #endif //HCI_UART_FLOW_ON
    s_params.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE;
    app_uart_init(&s_params, callback, &s_hci_uart_buffer);
    return;
}

static void hci_uart_write(app_uart_id_t id, uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    if(NULL == uart_env.tx.callback)
    {
        uart_env.tx.callback = callback;
        uart_env.tx.p_dummy  = p_dummy;
    }
    app_uart_transmit_async(id, p_buffer, (uint16_t)size);
    return;
}

static void hci_uart_read(app_uart_id_t id, uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    if(NULL == uart_env.rx.callback)
    {
        uart_env.rx.callback = callback;
        uart_env.rx.p_dummy  = p_dummy;
    }

    app_uart_receive_async(id, p_buffer, (uint16_t)size);
    return;
}

static void hci_uart_flow_on(app_uart_id_t id)
{
    //Define HCI_UART_FLOW_ON to Open UART Flow Control
    #if HCI_UART_FLOW_ON == 1  //To define macro this way for auto test script to locate and modify, don't change this line
    uart_handle_t *p_uart = app_uart_get_handle(id);
    /* Disable flow control default for .bit version < R02_27a_B0228a_k7 because CTS issue */
    ll_uart_set_hw_flow_ctrl(p_uart->p_instance, LL_UART_HWCONTROL_RTS_CTS);
    #endif //HCI_UART_FLOW_ON
    return;
}

static bool hci_uart_flow_off(app_uart_id_t id)
{
    bool flow_off = true;
    uart_handle_t *p_uart = app_uart_get_handle(id);
    hal_uart_state_t state = hal_uart_get_state(p_uart);

    do {
        /* Check if sleep is allowed by Host and if no transmission is ongoing */
        if ((state == HAL_UART_STATE_BUSY_TX) || (state == HAL_UART_STATE_BUSY_RX))
        {
            flow_off = false;
            break;
        }

        /* Force RTS to 'flow off' */
        ll_uart_set_hw_flow_ctrl(p_uart->p_instance, LL_UART_HWCONTROL_NONE);

        state = hal_uart_get_state(p_uart);
        /* Check if data has been received during wait time */
        if (state == HAL_UART_STATE_BUSY_RX)
        {
            /* Re-enable UART flow */
            hci_uart_flow_on(id);

            flow_off = false;
        }
    } while(0);
    return flow_off;
}

static void hci_uart_finish_transfers(app_uart_id_t id)
{
    return;
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void hci_uart0_init(void)
{
    hci_uart_init(APP_UART_ID_0, hci_uart0_callback);
}

void hci_uart1_init(void)
{
    hci_uart_init(APP_UART_ID_1, hci_uart1_callback);
}

void hci_uart0_flow_on(void)
{
    hci_uart_flow_on(APP_UART_ID_0);
}

void hci_uart1_flow_on(void)
{
    hci_uart_flow_on(APP_UART_ID_1);
}

bool hci_uart0_flow_off(void)
{
    return hci_uart_flow_off(APP_UART_ID_0);
}

bool hci_uart1_flow_off(void)
{
    return hci_uart_flow_off(APP_UART_ID_1);
}

void hci_uart0_finish_transfers(void)
{
    hci_uart_finish_transfers(APP_UART_ID_0);
}

void hci_uart1_finish_transfers(void)
{
    hci_uart_finish_transfers(APP_UART_ID_1);
}

void hci_uart0_read(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy)
{
    hci_uart_read(APP_UART_ID_0, p_buffer, size, callback, p_dummy);
}

void hci_uart1_read(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy)
{
    hci_uart_read(APP_UART_ID_1, p_buffer, size, callback, p_dummy);
}

void hci_uart0_write(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy)
{
    hci_uart_write(APP_UART_ID_0, p_buffer, size, callback, p_dummy);
}

void hci_uart1_write(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy)
{
    hci_uart_write(APP_UART_ID_1, p_buffer, size, callback, p_dummy);
}

void ble_hci_uart_init(void)
{
    s_hci_uart_buffer.tx_buf = s_hci_buffer;
    s_hci_uart_buffer.tx_buf_size = sizeof(s_hci_buffer);
    ble_hci_uart_register(0, &hci_uart_api);
}

void ble_hci_stack_init_handle(void)
{

}

