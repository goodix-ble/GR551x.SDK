/**
 ****************************************************************************************
 *
 * @file bootloader_config.h
 *
 * @brief bootloader configuration file.
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

#ifndef _BOOTLOADER_CONFIG_H_
#define _BOOTLOADER_CONFIG_H_

#include <stdint.h>
#include "grx_sys.h"

// <o> Whether to enable port firmware verify and jump strategy
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_BOOT_PORT_ENABLE             0

// <o> Whether to enable the dfu port ble
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_DFU_BLE_ENABLE               0

// <o> Whether to enable the dfu port uart
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_DFU_UART_ENABLE              1

// <o> Whether to enable the dfu
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_DFU_ENABLE                   (BOOTLOADER_DFU_UART_ENABLE || BOOTLOADER_DFU_BLE_ENABLE)

// <o> Whether to enable the watchdog
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_WDT_ENABLE                   1

// <o> Whether to enable the signature verification function
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_SIGN_ENABLE                  0

// Note: Signature verification will increase startup time.
//       In order to speed up startup time, signature verification can only be performed after the firmware is updated.
//       Factory burned firmware does not verify signature.
// <o> Whether to enable signature verification only after firmware update
// <0=> Disable
// <1=> Enable
#if BOOTLOADER_SIGN_ENABLE
#define BOOTLOADER_SIGN_VERIFY_AFTER_DFU        1
#else
#define BOOTLOADER_SIGN_VERIFY_AFTER_DFU        0
#endif

// Note: If the APP is searched by name, the first 12 bytes of the APP firmware name must match APP_FW_COMMENTS.
//       If the APP is searched by load address, the load address of the APP firmware is fixed to APP_FW_LOAD_ADDR.
//       If both BOOTLOADER_BY_NAME_ENABLE and BOOTLOADER_BY_ADDR_ENABLE are enabled, name search will be prioritized
// <o> Whether to enable searching for APP FW by APP_FW_COMMENTS
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_BY_NAME_ENABLE               1

// <o> Whether to enable searching for APP FW by APP_FW_LOAD_ADDR
// <0=> Disable
// <1=> Enable
#define BOOTLOADER_BY_ADDR_ENABLE               0

// Application firmware comments definition
#define APP_FW_COMMENTS                         "ble_app_temp"

// <o> Application firmware load address
#define APP_FW_LOAD_ADDR                        (FLASH_START_ADDR + 0x40000)

// Firmware saves the address during DFU, and this address will be sent to the master(Firmware Info Get CMD), which can use it or specify another address.
#define DFU_FW_SAVE_ADDR                        (FLASH_START_ADDR + 0x40000)

// <o> Bootloader receive external command timeout, in seconds. If timeout occurs, the DFU_INFO is erased and the chip is reset.
// <i> Default: 60 seconds
#define BOOTLOADER_TIMEOUT                      (60U)

//Hash value of the signed public key
#define BOOTLOADER_PUBLIC_KEY_HASH              0x77,0x20,0x28,0x95,0xB2,0xE1,0xD5,0x5F,0xAD,0xAA,0x81,0x7C,0xA6,0x7E,0xE3,0x95

#if !BOOTLOADER_BY_NAME_ENABLE && !BOOTLOADER_BY_ADDR_ENABLE
#error "Must select a jump method"
#endif

#if !BOOTLOADER_SIGN_ENABLE && BOOTLOADER_SIGN_VERIFY_AFTER_DFU
#error "BOOTLOADER_SIGN_VERIFY_AFTER_DFU need to be 0"
#endif

#endif
