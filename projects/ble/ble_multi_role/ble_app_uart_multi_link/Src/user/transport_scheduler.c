/**
 *****************************************************************************************
 *
 * @file transport_scheduler.c
 *
 * @brief Transport schedule function Implementation.
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
******************************************************************************************
*/
#include "transport_scheduler.h"
#include "user_periph_setup.h"
#include "ring_buffer.h"
#include "mlmr.h"
#include <stdbool.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define RING_BUFFER_SIZE     5120
#define UART_ONCE_SEND_SIZE  244

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint16_t      s_mtu_size = 23;
static bool          s_transport_flag[FLAGS_NB];
static uint8_t       s_uart_to_ble_buff[RING_BUFFER_SIZE];
static uint8_t       s_ble_to_uart_buff[RING_BUFFER_SIZE];
static uint8_t       s_uart_tx_data[UART_ONCE_SEND_SIZE];
static uint8_t       s_ble_tx_data[244];
static ring_buffer_t s_uart_rx_ring_buffer;
static ring_buffer_t s_ble_rx_ring_buffer;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Transport data from ble to uart.
 *****************************************************************************************
 */
static void transport_uart_data_send(void)
{

    uint16_t read_len;
    uint16_t items_avail;

    items_avail = ring_buffer_items_count_get(&s_ble_rx_ring_buffer);

    if (items_avail > 0)
    {
        read_len = ring_buffer_read(&s_ble_rx_ring_buffer, s_uart_tx_data, UART_ONCE_SEND_SIZE);
        uart_tx_data_send(s_uart_tx_data, read_len);
    }
}

/**
 *****************************************************************************************
 * @brief Transport data from uart to ble.
 *****************************************************************************************
 */
static void transport_ble_data_send(void)
{
    uint16_t read_len;
    uint16_t items_avail;

    items_avail = ring_buffer_items_count_get(&s_uart_rx_ring_buffer);

    if (items_avail > 0)
    {
        read_len = ring_buffer_read(&s_uart_rx_ring_buffer, s_ble_tx_data, s_mtu_size - 3);
        transport_flag_set(BLE_TX_CPLT, false);
        mlmr_tx_data_send(0, s_ble_tx_data, read_len);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void transport_schedule(void)
{
    transport_uart_data_send();

    if (transport_flag_cfm(BLE_TX_NTF_ENABLE) && transport_flag_cfm(BLE_TX_CPLT) && \
         transport_flag_cfm(BLE_SCHEDULE_ON))
    {
        transport_ble_data_send();
    }
}

void transport_ble_init(void)
{
    ring_buffer_init(&s_uart_rx_ring_buffer, s_uart_to_ble_buff, RING_BUFFER_SIZE);
    transport_flag_set(BLE_TX_CPLT, true);
    transport_flag_set(BLE_TX_NTF_ENABLE, false);
    transport_flag_set(BLE_SCHEDULE_ON, true);
}

void transport_uart_init(void)
{
    ring_buffer_init(&s_ble_rx_ring_buffer, s_ble_to_uart_buff, RING_BUFFER_SIZE);
    transport_flag_set(UART_TX_CPLT, true);
}

void transport_ble_continue_send(void)
{
    uint16_t read_len;
    uint16_t items_avail;

    transport_flag_set(BLE_SCHEDULE_ON, true);

    items_avail = ring_buffer_items_count_get(&s_uart_rx_ring_buffer);

    if (items_avail > 0)
    {
        read_len = ring_buffer_read(&s_uart_rx_ring_buffer, s_ble_tx_data, s_mtu_size - 3);
        transport_flag_set(BLE_TX_CPLT, false);
        transport_flag_set(BLE_SCHEDULE_ON, false);
        mlmr_tx_data_send(0, s_ble_tx_data, read_len);
    }
}

void uart_to_ble_push(uint8_t const *p_data, uint16_t length)
{
    ring_buffer_write(&s_uart_rx_ring_buffer, p_data, length);
}

void ble_to_uart_push(uint8_t const *p_data, uint16_t length)
{ 
    ring_buffer_write(&s_ble_rx_ring_buffer, p_data, length);
    ring_buffer_is_reach_left_threshold(&s_ble_rx_ring_buffer, s_mtu_size);
}

void transport_flag_set(transport_flag_t flag, bool en_dis)
{
    s_transport_flag[flag] = en_dis;
}

bool transport_flag_cfm(transport_flag_t flag)
{
    return  s_transport_flag[flag];
}

void update_mtu_size(uint16_t new_mtu)
{
    s_mtu_size = new_mtu;
}
