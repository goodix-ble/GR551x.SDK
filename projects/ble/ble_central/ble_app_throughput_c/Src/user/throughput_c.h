/**
 *****************************************************************************************
 *
 * @file throughput_c.h
 *
 * @brief Throughput Client APIs
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

#ifndef _THROUGHPUT_C_H__
#define _THROUGHPUT_C_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "ths_c.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize throughtput client transport flags and variables.
 *****************************************************************************************
 */
void thrpt_c_init(void);


/**
 *****************************************************************************************
 * @brief Handler for throughput timeout.
 *****************************************************************************************
 */
void thrpt_counter_handler(void *p_arg);

/**
 *****************************************************************************************
 * @brief The event handler for Throughput Service Client.
 *
 * @param[in] p_evt:  Pointer to the event of Throughput Service Client.
 *****************************************************************************************
 */
void thrpt_c_event_process(ths_c_evt_t *p_evt);

/**
 *****************************************************************************************
 * @brief Parse data from uart.
 *
 * @param[in] p_data: Pointer to data need to be parsed.
 * @param[in] length: Length of data need to be parsed.
 *****************************************************************************************
 */
void thrpt_c_data_parse(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Update the value of current mtu.
 *
 * @param[in] mtu: New value of current mtu.
 *****************************************************************************************
 */
void thrpt_c_mtu_update(uint16_t mtu);

/**
 *****************************************************************************************
 * @brief Update the value of current rssi.
 *
 * @param[in] mtu: New value of current rssi.
 *****************************************************************************************
 */
void thrpt_c_rssi_update(int8_t rssi);
#endif
