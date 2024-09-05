/**
 ****************************************************************************************
 *
 * @file hci_adapter.c
 *
 * @brief HCI adapter function Implementation.
 *
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
 *****************************************************************************************
 */

#include "vhci.h"
#include "gr55xx_hal.h"
#include "string.h"

#define RX_CHANNEL_LOCK()    LOCAL_INT_DISABLE(BLE_IRQn)
#define RX_CHANNEL_UNLOCK()  LOCAL_INT_RESTORE()


/*
 * STRUCT DEFINE
 *******************************************************************************
 */
typedef struct
{
    uint8_t                *p_channel;
    uint16_t                cache_size;
    uint16_t                write_index;
    uint16_t                read_index;
} ble_hci_rx_channel_mgr_t;

typedef struct
{
    void     (*callback) (void*, uint8_t);
    void     *p_dummy;
    uint8_t  *p_buffer;
    uint32_t  size;
    bool      rx_wait;
} ble_hci_controller_rd_mgr_t;

/*
 * LOCAL FUNCTION DECLARATION
 *******************************************************************************
 */
static void hci_uart0_init(void);
static void hci_uart0_flow_on(void); 
static bool hci_uart0_flow_off(void);
static void hci_uart0_finish_transfers(void);
static void hci_uart0_read(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy);
static void hci_uart0_write(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy);

/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */
static ble_hci_rx_channel_mgr_t    s_hci_ad_rx_channel_mgr;
static ble_hci_controller_rd_mgr_t s_hci_ad_controller_rd_mgr;
static ble_hci_host_recv_cb_t      s_host_recv_cb;

static hci_uart_call_t hci_uart_api =
{
    hci_uart0_init,
    hci_uart0_flow_on,
    hci_uart0_flow_off,
    hci_uart0_finish_transfers,
    hci_uart0_read,
    hci_uart0_write,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void hci_ad_rx_channel_init(ble_hci_rx_channel_t *p_rx_channel)
{
    s_hci_ad_rx_channel_mgr.p_channel   = p_rx_channel->p_channel;
    s_hci_ad_rx_channel_mgr.cache_size  = p_rx_channel->cache_size;
    s_hci_ad_rx_channel_mgr.read_index  = 0;
    s_hci_ad_rx_channel_mgr.write_index = 0;
}

static uint16_t hci_ad_rx_channel_items_count_get(void)
{
    if (s_hci_ad_rx_channel_mgr.read_index <= s_hci_ad_rx_channel_mgr.write_index)
    {
        return s_hci_ad_rx_channel_mgr.write_index - s_hci_ad_rx_channel_mgr.read_index;
    }
    else
    {
        return s_hci_ad_rx_channel_mgr.cache_size - s_hci_ad_rx_channel_mgr.read_index + s_hci_ad_rx_channel_mgr.write_index;
    }
}

static uint16_t hci_ad_rx_channel_read(uint8_t *p_rd_data, uint16_t length)
{
    RX_CHANNEL_LOCK();
    uint16_t items_avail = 0;
    uint16_t over_flow   = 0;
    uint16_t wr_idx      = s_hci_ad_rx_channel_mgr.write_index;
    uint16_t rd_idx      = s_hci_ad_rx_channel_mgr.read_index;
    uint16_t buffer_size = s_hci_ad_rx_channel_mgr.cache_size;
    uint8_t  *p_buffer   = s_hci_ad_rx_channel_mgr.p_channel;

    if (wr_idx >= rd_idx)
    {
        items_avail = wr_idx - rd_idx;
        length = (length > items_avail ? items_avail : length);
    }
    else
    {
        items_avail = buffer_size - rd_idx + wr_idx;
        length = (length > items_avail ? items_avail : length);

        if (rd_idx + length >= buffer_size)
        {
            over_flow = length + rd_idx - buffer_size;
        }
    }

    memcpy(p_rd_data, p_buffer + rd_idx, length - over_flow);
    memcpy(p_rd_data + length - over_flow, p_buffer, over_flow);
    rd_idx += length;

    if (rd_idx >= buffer_size && rd_idx > wr_idx)
    {
        rd_idx -= buffer_size;
    }

    s_hci_ad_rx_channel_mgr.read_index = rd_idx;

    RX_CHANNEL_UNLOCK();

    return length;
}

static void hci_ad_host_tx_data_handle(void)
{
    if (s_hci_ad_controller_rd_mgr.rx_wait)
    {
        if (s_hci_ad_controller_rd_mgr.size <= hci_ad_rx_channel_items_count_get())
        {
            void *data = NULL;
            void (*callback) (void*, uint8_t) = NULL;

            hci_ad_rx_channel_read(s_hci_ad_controller_rd_mgr.p_buffer, s_hci_ad_controller_rd_mgr.size);

            callback = s_hci_ad_controller_rd_mgr.callback;
            data     = s_hci_ad_controller_rd_mgr.p_dummy;

            s_hci_ad_controller_rd_mgr.callback = NULL;
            s_hci_ad_controller_rd_mgr.p_dummy  = NULL;
            s_hci_ad_controller_rd_mgr.rx_wait = false;

            if(callback != NULL)
            {
                callback(data, 0);
            }
        }
    }
}

static void hci_uart0_init(void)
{
    return;
}

static void hci_uart0_flow_on(void)
{
    return;
}

static bool hci_uart0_flow_off(void)
{
    return false;
}

static void hci_uart0_finish_transfers(void)
{
    return;
}

static void hci_uart0_read(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy)
{
    if(NULL == s_hci_ad_controller_rd_mgr.callback)
    {
        s_hci_ad_controller_rd_mgr.callback = callback;
        s_hci_ad_controller_rd_mgr.p_dummy  = p_dummy;
        s_hci_ad_controller_rd_mgr.p_buffer = p_buffer;
        s_hci_ad_controller_rd_mgr.size     = size;
        s_hci_ad_controller_rd_mgr.rx_wait  = true;
    }

    hci_ad_host_tx_data_handle();
}

static void hci_uart0_write(uint8_t *p_buffer, uint32_t size, void (*callback)(void*, uint8_t), void *p_dummy)
{
    if (s_host_recv_cb)
    {
        s_host_recv_cb(p_buffer, size);
    }

    if(callback != NULL)
    {
        callback(p_dummy, 0);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t ble_hci_init(ble_hci_rx_channel_t *p_rx_channel, ble_hci_host_recv_cb_t host_recv_cb)
{
    if (NULL == p_rx_channel ||
        NULL == p_rx_channel->p_channel ||
        0    == p_rx_channel->cache_size ||
        NULL == host_recv_cb)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    s_host_recv_cb = host_recv_cb;

    hci_ad_rx_channel_init(p_rx_channel);

    ble_hci_uart_register(0, &hci_uart_api);

    return SDK_SUCCESS;
}

sdk_err_t ble_hci_host_packet_send(uint8_t *p_data, uint16_t length)
{
    if (NULL == p_data || 0 == length)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    RX_CHANNEL_LOCK();

    uint16_t over_flow   = 0;
    uint16_t wr_idx      = s_hci_ad_rx_channel_mgr.write_index;
    uint16_t rd_idx      = s_hci_ad_rx_channel_mgr.read_index;
    uint16_t buffer_size = s_hci_ad_rx_channel_mgr.cache_size;
    uint8_t  *p_buffer   = s_hci_ad_rx_channel_mgr.p_channel;

    if (ble_hci_rx_channel_surplus_space_get() < length)
    {

        return SDK_ERR_NO_RESOURCES;
    }

    if ((rd_idx <= wr_idx) && (wr_idx + length >= buffer_size))
    {
        over_flow = wr_idx + length - buffer_size;
    }

    memcpy(p_buffer + wr_idx, p_data, length - over_flow);
    memcpy(p_buffer, p_data + length - over_flow, over_flow);
    wr_idx += length;

    if (wr_idx >= buffer_size)
    {
        wr_idx -= buffer_size;
    }

    s_hci_ad_rx_channel_mgr.write_index = wr_idx;

    RX_CHANNEL_UNLOCK();

    hci_ad_host_tx_data_handle();

    return SDK_SUCCESS;
}

uint16_t ble_hci_rx_channel_surplus_space_get(void)
{
    uint16_t surplus_space;
    uint16_t wr_idx = s_hci_ad_rx_channel_mgr.write_index;
    uint16_t rd_idx = s_hci_ad_rx_channel_mgr.read_index;

    RX_CHANNEL_LOCK();

    if (rd_idx > wr_idx)
    {
        surplus_space = rd_idx - wr_idx - 1;
    }
    else
    {
        surplus_space = s_hci_ad_rx_channel_mgr.cache_size - wr_idx + rd_idx - 1;
    }

    RX_CHANNEL_UNLOCK();

    return surplus_space;
}
