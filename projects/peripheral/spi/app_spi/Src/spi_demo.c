/**
 *****************************************************************************************
 *
 * @file spi_demo.c
 *
 * @brief SPI demo
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
#include <stdint.h>
#include <stdbool.h>
#include "app_log.h"
#include "app_io.h"
#include "app_spi.h"
#include "spi_flash.h"
#include "spi_demo_cfg.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define FLASH_CLOCK_PRESCALER           64u
#define FLASH_ACCESS_TYPE               APP_SPI_TYPE_DMA

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t __attribute__((aligned(32))) write_buffer[FLASH_OPERATION_LENGTH] = {0};
static uint8_t __attribute__((aligned(32))) read_buffer[FLASH_OPERATION_LENGTH] = {0};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void  print_verify_data(uint32_t addr, uint8_t *p_data, uint32_t nbytes)
{
    uint32_t i = 0;

    printf("--------------------------\r\n");
    for (i = 0; i < nbytes; i++)
    {
        if (!(i & 0x7))
        {
            printf("\r\n0x%08X\t", addr + i);
        }
        printf("0x%02X ", p_data[i]);
    }
    printf("--------------------------\r\n");
}

static int data_cmp(uint32_t addr, uint8_t *p_result, uint8_t *p_except, uint32_t nbytes)
{
    if (memcmp(p_result, p_except, nbytes) != 0)
    {
        printf("\r\nData verify failed!\r\n");
        print_verify_data(addr, p_result, nbytes);
        return -1;
    }
    return 0;
}

static void init_write_buffer(void)
{
    int i = 0;
    for (i = 0; i < FLASH_OPERATION_LENGTH; i++)
    {
        write_buffer[i] = i;
    }
}

static int spi_flash_erase(uint32_t addr)
{
    int i = 0;
    spi_flash_sector_erase(addr);

    memset(read_buffer, 0, FLASH_OPERATION_LENGTH);
    spi_flash_read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    for (i = 0; i < FLASH_OPERATION_LENGTH; i++)
    {
        if (read_buffer[i] != 0xFF)
        {
            return -1;
        }
    }
    return 0;
}

static void spi_flash_program(uint32_t addr, uint8_t *p_wbuf, uint32_t nbytes)
{
    int page_num = 0;
    int i        = 0;
    uint32_t remain_bytes = nbytes;
    uint32_t w_bytes  = 0;

    page_num = nbytes / 256;
    if (nbytes % 256)
    {
        page_num += 1;
    }

    for (i = 0; i < page_num; i++)
    {
        w_bytes = (remain_bytes >= 256) ? 256 : remain_bytes;
        spi_flash_page_program(addr + i * 256, &p_wbuf[i * 256], w_bytes);
        remain_bytes -= w_bytes;
    }
}


static void spi_master_init(void)
{
    uint32_t flash_id = 0;

    spi_flash_init(FLASH_CLOCK_PRESCALER);

    flash_id = spi_flash_read_id();
    printf("Flash id is:0x%x\r\n", flash_id);
}

static void spi_master_process(void)
{
    uint32_t test_sec = 0;
    int ret = 0;

    init_write_buffer();
    ret = spi_flash_erase(FLASH_PROGRAM_START_ADDR);
    if (ret != 0)
    {
        printf("Flash erase failed!\r\n");
        while(1);
    }

    spi_flash_program(FLASH_PROGRAM_START_ADDR, write_buffer, FLASH_OPERATION_LENGTH);

    while(1)
    {
        memset(read_buffer, 0x00, sizeof(read_buffer));
        spi_flash_fast_read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);

        ret = data_cmp(FLASH_PROGRAM_START_ADDR, read_buffer, write_buffer, FLASH_OPERATION_LENGTH);
        if (ret == 0)
        {
            printf("[Time:%02d:%02d] SPI flash read data verify success.\r\n", test_sec / 60, test_sec % 60);
        }
        else
        {
            printf("SPI flash read data verify failed!\r\n");
            while(1);
        }

        test_sec++;

        if (test_sec > SPI_DEMO_MINS * 60)
        {
            printf("SPI flash read data pass.\r\n");
            while(1);
        }

        delay_ms(1000);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void spi_demo_task(void)
{
    spi_master_init();
    spi_master_process();
}
