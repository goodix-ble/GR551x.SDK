/**
 ****************************************************************************************
 *
 * @file app_fds_port.c
 *
 * @brief App flash data storage port.c
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


#include "app_fds.h"
#include "grx_sys.h"
#include "hal_flash.h"


static uint32_t __l_irq_rest;

static int app_fds_lock(void)
{
    __l_irq_rest = __get_BASEPRI();

    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) + (1 << (NVIC_GetPriorityGrouping() + 1)));

    return APP_FDS_ERR_OK;
}

static int app_fds_unlock(void)
{
    __set_BASEPRI(__l_irq_rest);

    return APP_FDS_ERR_OK;
}

static int app_lfs_block_read(uint32_t addr, uint8_t *p_buffer, uint32_t size)
{
    return size == hal_flash_read(addr, (uint8_t*)p_buffer, size) ? APP_FDS_ERR_OK : APP_FDS_ERR_IO;
}

static int app_lfs_block_prog(uint32_t addr, uint8_t *p_buffer, uint32_t size)
{
    return size == hal_flash_write(addr, (uint8_t*)p_buffer, size) ? APP_FDS_ERR_OK : APP_FDS_ERR_IO;
}

static int app_lfs_block_erase(uint32_t addr,uint32_t size)
{
    return hal_flash_erase (addr, size) ? APP_FDS_ERR_OK : APP_FDS_ERR_IO;
}

static int app_lfs_device_sync(void)
{
    return APP_FDS_ERR_OK;
}

app_fds_config_t s_app_fds_config = 
{
    .fds_lock       = app_fds_lock,
    .fds_unlock     = app_fds_unlock,
    .fds_read       = app_lfs_block_read,
    .fds_write      = app_lfs_block_prog,
    .fds_erase      = app_lfs_block_erase,
    .fds_sync       = app_lfs_device_sync,
};




