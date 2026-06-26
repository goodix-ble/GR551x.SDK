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

#include <stdint.h>

/*
 * DEFINES
 *****************************************************************************************
 */
// Master status
#define MASTER_IDLE                 0x00
#define MASTER_UART_UPDATING        0x02
#define MASTER_UART_UPDATED         0x03
#define MASTER_BLE_SELECT_DEVICE    0x04
#define MASTER_BLE_CONNECTED        0x05
#define MASTER_BLE_UPDATING         0x07
#define MASTER_BLE_UPDATED          0x08

#define MASTER_FAST_DFU_MODE_SET    0x10
#define MASTER_DFU_START            0x11
#define MASTER_UART_DFU_START       0x12
#define MASTER_BLE_DFU_START        0x13
#define MASTER_DFU_FW_INFO_SET      0x14
#define MASTER_SELECT_UART_OR_BLE   0x15
#define MASTER_BLE_DFU_SCAN         0x16

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
 * @brief Set master status.
 *
 * @param[in]  status: master status.
 *****************************************************************************************
 */
void user_master_status_set(uint8_t status);

/**
 *****************************************************************************************
 * @brief Get master status.
 *
 * @retval Master status.
 *****************************************************************************************
 */
uint8_t user_master_status_get(void);

/**
 *****************************************************************************************
 * @brief UART console schedule.
 *
 *****************************************************************************************
 */
void uart_console_schedule(void);

/**
 *****************************************************************************************
 * @brief Reset UART console.
 *
 *****************************************************************************************
 */
void uart_console_reset(void);

/**
 *****************************************************************************************
 * @brief  Initialize UART console
 *
 *****************************************************************************************
 */
void uart_console_init(void);

/**
 *****************************************************************************************
 * @brief  Initialize systick
 *
 *****************************************************************************************
 */
void systick_init(void);

/**
 *****************************************************************************************
 * @brief Get systick time.
 *
 * @retval Time in milliseconds.
 *****************************************************************************************
 */
uint32_t systick_get_time(void);

#endif
