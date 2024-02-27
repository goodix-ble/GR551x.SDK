/**
 ****************************************************************************************
 *
 * @file ble_ism.h
 *
 * @brief BLE ISM Direct transport API.
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
 
  /**
 * @addtogroup BLE
 * @{
 */

/**
 ****************************************************************************************
 * @brief Callback of ISM direct packet TX is done.
 ****************************************************************************************
 */
void (*transmit_cfm_cb)(void);

/**
 ****************************************************************************************
 * @brief Callback of ISM direct packet RX is done.
 *
 * @param[in] packet_length     Length of the received packet data
 * @param[in] packet_data       Pointer to the packet data received through ISM direct.
 ****************************************************************************************
 */
void (*receive_cb)(uint8_t packet_length, uint8_t *packet_data);

/**
 ****************************************************************************************
 * @brief Initialize the data transmission function of ISM direct.
 *
 * @param[in] ch_num            Transmission frequency number(0-159 @2360Mhz-2520Mhz, Step 1MHz)
 * @param[in] access_address    Transmission access address
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 ****************************************************************************************
 */
uint8_t ISM_direct_init(uint8_t ch_num, uint32_t access_address);

/**
 ****************************************************************************************
 * @brief Receive a data packet through ISM direct.
 *
 * @param[in] ISM_direct_receive_cb Pointer to the function to call when ISM direct packet RX is done
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 ****************************************************************************************
 */
uint8_t ISM_direct_receive(void (*ISM_direct_receive_cb)(uint8_t packet_length, uint8_t *packet_data));

/**
 ****************************************************************************************
 * @brief Transmit a data packet through ISM direct.
 *
 * @param[in] packet_length     Length of packet data to be sent
 * @param[in] packet_data       Pointer to the packet data through ISM direct to be sent
 * @param[in] transmit_cfm_cb   Pointer to the function to call when ISM direct packet TX is done
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 ****************************************************************************************
 */
uint8_t ISM_direct_transmit(uint8_t packet_length, uint8_t *packet_data, void (*transmit_cfm_cb)(void));

/** @} */


