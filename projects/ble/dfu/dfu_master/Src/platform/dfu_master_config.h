/**
 *****************************************************************************************
 *
 * @file dfu_master_config.h
 *
 * @brief DFU master configuration file
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

#ifndef __DFU_MASTER_CONFIG_H__
#define __DFU_MASTER_CONFIG_H__

// Note: When users porting to new platforms, flash_scatter_config.h and user_dfu_master.h is not necessary
#include "flash_scatter_config.h"
#include "user_dfu_master.h"

/*
 * DEFINES
 *****************************************************************************************
 */
// Whether to enable debug log, 0: Disable, 1: Enable
#define DFU_MASTER_DEBUG                         0
#if DFU_MASTER_DEBUG
#include "app_log.h"
#define DFU_MASTER_LOG(...)                      APP_LOG_DEBUG(__VA_ARGS__)
#else
#define DFU_MASTER_LOG(...)
#endif

// NOTE: search image info based on DFU_FW_SAVE_MASTER_ADDR and DFU_FW_SIZE_MAX. When porting, fixed addresses and size can be used.
// The new firmware save address on the master
#define DFU_FW_SAVE_MASTER_ADDR                  get_fw_save_addr()
// The new firmware max size
#define DFU_FW_SIZE_MAX                          get_fw_max_size()

// The flash start address of slave(GR5xx). When porting, fixed addresses can be used. GR551x: 0x01000000, other: 0x00200000
#define DFU_SLAVE_FLASH_START_ADDR               FLASH_START_ADDR

// Whether to enable the dfu port ble, 0: Disable, 1: Enable
#define DFU_BLE_ENABLE                           1

// Whether to enable the dfu port uart, 0: Disable, 1: Enable
#define DFU_UART_ENABLE                          1

// DFU flash bank mode, DFU_MODE_NON_COPY_UPGRADE or DFU_MODE_COPY_UPGRADE
#define DFU_BANK_MODE                            DFU_MODE_NON_COPY_UPGRADE

// Select the method to get the firmware save address on the slave
#define GET_SAVE_ADDR_BY_NEW_FW                  0 // By load address of new firmware
#define GET_SAVE_ADDR_BY_SLAVE                   1 // By slave firmware info(Copy_load_addr)
#define GET_SAVE_ADDR_BY_USER                    2 // By user specified address(DFU_FW_SAVE_ADDR)
#define DFU_GET_FW_SAVE_ADDR_METHOD              GET_SAVE_ADDR_BY_SLAVE
#if DFU_GET_FW_SAVE_ADDR_METHOD == GET_SAVE_ADDR_BY_USER
// The new firmware save address on the slave
#define DFU_FW_SAVE_ADDR                         (FLASH_START_ADDR + 0x40000U)
#endif

// Whether to enable the dfu asynchronous transmission mode, 0: Disable, 1: Enable
#define DFU_ASYNC_TX_ENABLE                      1

// Flash program length for one frame: 1 to 1024 bytes, recommended value is a multiple of flash page size
#define DFU_ONCE_PROGRAM_LEN                     256U

// Note: The physical transmission are determined by the UART or BLE
// In asynchronous transmission mode, the maximum length of a single logical transmission.
// If a frame length is greater than this, the frame will be split into multiple transmissions.
// In BLE mode, DFU_SEND_SIZE_MAX is limited by MTU
// In UART mode, DFU_SEND_SIZE_MAX is limited by UART TX buffer
#define DFU_SEND_SIZE_MAX                        244U

// DFU master wait ACK timeout(in unit of 1 ms)
#define DFU_ACK_WAIT_TIMEOUT                     4000U

// Note: After setting the DFU mode, the slave may need to be reset to enter the bootloader
// DFU slave chip reset time(in unit of 1 ms)
#define DFU_SLAVE_RESET_TIME                     2000U

#if DFU_BLE_ENABLE && !DFU_ASYNC_TX_ENABLE
#error "BLE is asynchronous transmission"
#endif

#endif /* __DFU_MASTER_CONFIG_H__ */
