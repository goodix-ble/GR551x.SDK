/**
  ****************************************************************************************
  * @file    gr55xx_spi_flash.h
  * @author  BLE Driver Team
  * @brief   Header file containing functions prototypes of spi flash library.
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
  ****************************************************************************************
  */
#ifndef __GR55XX_SPI_FLASH_H__
#define __GR55XX_SPI_FLASH_H__

#include <stdbool.h>
#include "grx_hal.h"
#include "app_io.h"
#include "app_qspi.h"
#include "app_qspi_dma.h"
#include "app_spi.h"
#include "app_spi_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Flash operation instruction macro definition
  * @{
  */

#define SPI_FLASH_CMD_WRSR              0x01
#define SPI_FLASH_CMD_WRSR1             0x31
#define SPI_FLASH_CMD_RDSR              0x05

#define SPI_FLASH_CMD_WREN              0x06
#define SPI_FLASH_CMD_WRDI              0x04

#define SPI_FLASH_CMD_READ              0x03
#define SPI_FLASH_CMD_FREAD             0x0B
#define SPI_FLASH_CMD_DOFR              0x3B
#define SPI_FLASH_CMD_DIOFR             0xBB
#define SPI_FLASH_CMD_QOFR              0x6B
#define SPI_FLASH_CMD_QIOFR             0xEB
#define SPI_FLASH_CMD_READ_RESET        0xFF
#define SPI_FLASH_CMD_DPP               0xA2
#define SPI_FLASH_CMD_DREAD             0x3B
#define SPI_FLASH_CMD_PP                0x02
#define SPI_FLASH_CMD_SE                0x20
#define SPI_FLASH_CMD_BE_32             0x52
#define SPI_FLASH_CMD_BE_64             0xD8
#define SPI_FLASH_CMD_CE                0xC7
#define SPI_FLASH_CMD_PES               0x75
#define SPI_FLASH_CMD_PER               0x7A

#define SPI_FLASH_CMD_RDI               0xAB
#define SPI_FLASH_CMD_REMS              0x90
#define SPI_FLASH_CMD_RDID              0x9F

#define SPI_FLASH_CMD_RSTEN             0x66
#define SPI_FLASH_CMD_RST               0x99
#define SPI_FLASH_CMD_DP                0xB9
#define SPI_FLASH_CMD_RDP               0xAB

#define SPI_FLASH_CMD_SFUD              0x5A

#define DUMMY_BYTE                      0xFF

#define SPI_FLASH_PAGE_SIZE             0x00100
#define SPI_FLASH_SECTOR_SIZE           0x01000
#define SPI_FLASH_BLOCK_SIZE            0x10000
#define SPI_FLASH_ADDRESS_MAX           0xFFFFF

/** @} */

/**
  * @addtogroup Spi Flash IO configuration Structures
  * @{
  */
typedef enum
{
    FLASH_SPIM_ID,                   /**< SPI master module.     */
    FLASH_QSPI_ID0,                  /**< QSPI master module 0.  */
    FLASH_QSPI_ID1,                  /**< QSPI master module 1.  */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    FLASH_QSPI_ID2,                  /**< QSPI master module 2.  */
#endif
    FLASH_SPI_ID_MAX,                /**< Only for check parameter, not used as input parameters. */
} spi_type_t;

typedef struct _spi_io
{
    app_io_type_t gpio;
    uint32_t      pin;
    app_io_mux_t  mux;
} spi_io_t;

typedef struct _flash_io
{
    spi_io_t spi_cs;
    spi_io_t spi_clk;
    union
    {
        spi_io_t spim_mosi;
        spi_io_t qspi_io0;
    } spi_io0;
    union
    {
        spi_io_t spim_miso;
        spi_io_t qspi_io1;
    } spi_io1;
    spi_io_t qspi_io2;
    spi_io_t qspi_io3;
} flash_io_t;

typedef struct _flash_init
{
    spi_type_t spi_type;
    flash_io_t flash_io;
    bool        is_dual_line;
    bool        is_high_freq;
} flash_init_t;

typedef struct flash_control
{
    uint8_t         qspi_tmt_done;
    uint8_t         qspi_rcv_done;
    app_qspi_id_t   qspi_id;
    uint8_t         spi_tmt_done;
    uint8_t         spi_rcv_done;
    uint8_t         spi_tx_rx_done;
    app_spi_id_t    spi_id;
} qspi_control_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_SPI_FLASH_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the SPI FLASH DRIVER according to the specified parameters
 *         in the spi_flash_io_t.
 *
 * @param[in]  p_params: Pointer to spi_flash_io_t parameter.
 *
 ****************************************************************************************
 */
void spi_flash_init(flash_init_t *p_flash_init);

/**
 *******************************************************************************
 * @brief Write flash Memory.
 *
 * @param[in]       address: start address in flash to write data to.
 * @param[in,out]   buffer: buffer of data to write.
 * @param[in]       nbytes: number of bytes to write.
 *
 * @return          number of bytes written
 *******************************************************************************
 */
uint32_t spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes);

/**
 *******************************************************************************
 * @brief Read flash Memory.
 *
 * @param[in]       address: start address in flash to read data.
 * @param[in,out]   buffer: buffer to read data to.
 * @param[in]       nbytes: number of bytes to read.
 *
 * @return          number of bytes read
 *******************************************************************************
 */
uint32_t spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes);

/**
 *******************************************************************************
 * @brief Erase flash region.
 *
 * @note All sectors that have address in range of [addr, addr+len]
 *       will be erased. If addr is not sector aligned, preceding data
 *       on the sector that addr belongs to will also be erased.
 *       If (addr + size) is not sector aligned, the whole sector
 *       will also be erased.
 *
 * @param[in] address: start address in flash to write data to.
 * @param[in] size: number of bytes to write.
 *
 * @retval true: If successful.
 * @retval false: If failure.
 *******************************************************************************
 */
bool spi_flash_sector_erase(uint32_t address, uint32_t size);

/**
 *******************************************************************************
 * @brief Erase flash chip.
 *
 * @retval true: If successful.
 * @retval false: If failure.
 *******************************************************************************
 */
bool spi_flash_chip_erase(void);

/**
 *******************************************************************************
 * @brief Reset flash chip.
 *
 *******************************************************************************
 */
void spi_flash_chip_reset(void);

/**
 *******************************************************************************
 * @brief Get flash chip id.
 *
 * @retval Flash chip id.
 *******************************************************************************
 */
uint32_t spi_flash_device_id(void);

/**
 *******************************************************************************
 * @brief Get Flash information.
 *
 * @param[in,out] id: Pointer to flash id.
 * @param[in,out] size: Pointer to flash size, Unit:Byte.
 *
 *******************************************************************************
 */
void spi_flash_device_info(uint32_t *id, uint32_t *size);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // __GR55XX_SPI_FLASH_H__
