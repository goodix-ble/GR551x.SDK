/**
 *****************************************************************************************
 *
 * @file user_dfu_master.h
 *
 * @brief User DFU master header file
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2024 GOODIX
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

#ifndef __USER_DFU_MASTER_H__
#define __USER_DFU_MASTER_H__
#include <stdbool.h>
#include <stdint.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_UPGRADE_MODE                        1
#define BLE_UPGRADE_MODE                         2

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief User dfu master init.
 *****************************************************************************************
 */
void user_dfu_m_init(uint8_t dfu_mode, uint16_t once_send_size);

/**
 *****************************************************************************************
 * @brief Set firmware save address.
 * @param[in]  addr: Firmware save address in master.
 *****************************************************************************************
 */
void set_fw_save_addr(uint32_t addr);

/**
 *****************************************************************************************
 * @brief Set firmware maximum size.
 * @param[in]  size: Firmware maximum size.
 *****************************************************************************************
 */
void set_fw_max_size(uint32_t size);

/**
 *****************************************************************************************
 * @brief Get firmware save address in master.
 * @retval Firmware save address in master.
 *****************************************************************************************
 */
uint32_t get_fw_save_addr(void);

/**
 *****************************************************************************************
 * @brief Get firmware maximum size.
 * @retval Firmware maximum size.
 *****************************************************************************************
 */
uint32_t get_fw_max_size(void);

/**
 *****************************************************************************************
 * @brief Get image information.
 * @retval Operation result. true - success; false - fail.
 *****************************************************************************************
 */
bool user_get_img_info(void);

#endif  // __USER_DFU_MASTER_H__
