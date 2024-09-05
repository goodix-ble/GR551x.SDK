/**
 ****************************************************************************************
 *
 * @file hci_adapter.c
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
 *****************************************************************************************
 */

#ifndef __VHCI_H__
#define __VHCI_H__

#include "ble.h"
#include "vhci.h"

typedef struct
{
    uint8_t     *p_channel;    /**< Pointer to buffer for controller receive cache. */
    uint16_t     cache_size;   /**< Size of the cache buffer. */
} ble_hci_rx_channel_t;

/**@brief Receive controller pachet callback type. */
typedef void (*ble_hci_host_recv_cb_t)(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Initialize ble hci adapter module.
 *
 * @param[in] p_rx_channel: Pointer to hci adapter rx channel
 * @param[in] host_recv_cb: Callback to receive controller packet.
 *
 * @return Result of initializing.
 *****************************************************************************************
 */
sdk_err_t ble_hci_init(ble_hci_rx_channel_t *p_rx_channel, ble_hci_host_recv_cb_t host_recv_cb);

/**
 *****************************************************************************************
 * @brief BLE HCI adapter host send packet.
 *
 * @param[in] p_data: Pointer to packet data.
 * @param[in] length: Length of packet data.
 *
 * @return Result of send.
 *****************************************************************************************
 */
sdk_err_t ble_hci_host_packet_send(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Get surplus space of controller receive channel.
 *
 * @return Result of get.
 *****************************************************************************************
 */
uint16_t ble_hci_rx_channel_surplus_space_get(void);

#endif


