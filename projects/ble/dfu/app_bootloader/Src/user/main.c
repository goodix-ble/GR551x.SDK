/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr_includes.h"
#include "user_periph_setup.h"
#include "bootloader_boot.h"
#include "bootloader_config.h"
#include "dfu_port.h"

#ifndef BOOTLOADER_ENABLE
#error "Not define BOOTLOADER_ENABLE in app bootloader, please define it in this project"
#endif

/* WDT_RUN_ENABLE = 1 means that the watchdog is enabled until user takes control of the watchdog. */
#ifdef WDT_RUN_ENABLE
#if !WDT_RUN_ENABLE
#warning "Bootloader enable WDT_RUN_ENABLE is recommended"
#endif
#endif

/* BOOT_LONG_TIME = 1 means that Boot startup delay which is not recommended to enable in app bootloader  */
#ifdef BOOT_LONG_TIME
#if BOOT_LONG_TIME
#warning "Bootloader enable BOOT_LONG_TIME is not recommended"
#endif
#endif

#if APP_CODE_LOAD_ADDR < (DFU_INFO_START_ADDR + DFU_FLASH_SECTOR_SIZE)
#error "Bootloader Firmware overlaps with DFU_INFO"
#endif

int main (void)
{
    app_periph_init();
    bootloader_dfu_task();
    bootloader_verify_task();
    bootloader_jump_task();

    while (1)
    {
#if BOOTLOADER_WDT_ENABLE
        bootloader_wdt_refresh();
#endif
#if BOOTLOADER_DFU_ENABLE
        dfu_schedule();
#endif
        bootloader_timeout_task();
    }
}
