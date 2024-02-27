/**
 *****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Header file - User Periph Init
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

#ifndef __USER_PERIPH_SETUP_H__
#define __USER_PERIPH_SETUP_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize User Periph (GPIO SPI IIC ...).
 *****************************************************************************************
 */
void app_periph_init(void);

/**
 *****************************************************************************************
 * @brief Send data via uart.
 *
 * @param[in] p_data: Pointer to data need to be sent.
 * @param[in] length: Length of data need to be sent.
 *****************************************************************************************
 */
void uart_tx_data_send(uint8_t *p_data, uint16_t length);

#endif
