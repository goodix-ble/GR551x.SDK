#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include <stdint.h>
#include "app_spi.h"
#include "app_spi_dma.h"

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

#define SPI_FLASH_CMD_PP                0x02
#define SPI_FLASH_CMD_QPP               0x32
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

#define DUMMY_BYTE                      0xFF

void spi_flash_init(uint32_t clock_prescaler);
void spi_flash_deinit(void);
uint32_t spi_flash_read_id(void);
void spi_flash_sector_erase(uint32_t dst);
void spi_flash_chip_erase(void);
void spi_flash_page_program(uint32_t dst, uint8_t *data, uint32_t nbytes);
void spi_flash_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes);
void spi_flash_fast_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes);
void spi_flash_reset(void);
void spi_flash_power_down(void);
void spi_flash_wakeup(void);

#endif /* __SPI_FLASH_H */
