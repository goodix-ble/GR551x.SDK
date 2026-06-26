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
 * @brief Deinitialize User Periph.
 *****************************************************************************************
 */
void app_periph_deinit(void);

/**
 *****************************************************************************************
 * @brief Feed watch dog.
 *****************************************************************************************
 */
void bootloader_wdt_refresh(void);

/**
 *****************************************************************************************
 * @brief Get bootloader running time. Unit: milliseconds.
 *****************************************************************************************
 */
size_t bootloader_get_time(void);

/**
 *****************************************************************************************
 * @brief Refresh timeout time.
 * If an external command is received refresh start time.
 *****************************************************************************************
 */
void bootloader_timeout_refresh(void);

/**
 *****************************************************************************************
 * @brief Get bootloader timeout start time. Unit: milliseconds.
 *****************************************************************************************
 */
size_t bootloader_timeout_get_start_time(void);

/**
 ****************************************************************************************
 * @brief  Check if the ISP command is received.
 * Note: After receiving the ISP command, it will enter the non copy upgrade mode(single bank).
 * @return true: received ISP command.
 *         false: not received ISP command.
 ****************************************************************************************
 */
bool bootloader_uart_isp_check(void);

#endif
