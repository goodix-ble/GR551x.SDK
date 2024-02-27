/**
 ******************************************************************************
 *
 * @file gr55xx_nvds.h
 *
 * @brief NVDS API
 *
 ******************************************************************************
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

/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
  @addtogroup NVDS Non-Volatile Data Storage
  @{
  @brief Definitions and prototypes for the NVDS interface.
 */

#ifndef __GR55XX_NVDS_H__
#define __GR55XX_NVDS_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup NVDS_DEFINES Defines
 * @{ */
#define NV_TAGCAT_APP       0x4000  /**< NVDS tag mask for user application. */

/**@brief Get NVDS tag for user application.
 * The values of Tag 0x0000 and 0xFFFF are invalid. idx should not be used
 * as the parameter of NVDS APIs directly. The range of idx is 0x0001~0x3FFF.
 */
#define NV_TAG_APP(idx)     (NV_TAGCAT_APP | ((idx) & 0x3FFF))
/** @} */

/**@addtogroup NVDS_ENUMERATIONS Enumerations
 * @{ */
/**@brief NVDS Returned Status. */
enum NVDS_STATUS
{
    NVDS_SUCCESS,                /**< NVDS succeeds. */
    NVDS_FAIL,                   /**< NVDS failed. */
    NVDS_TAG_NOT_EXISTED,        /**< NVDS tag does not exist. */
    NVDS_SPACE_NOT_ENOUGH,       /**< NVDS space is not enough. */
    NVDS_LENGTH_OUT_OF_RANGE,    /**< NVDS length out of range. */
    NVDS_INVALID_PARA,           /**< NVDS invalid params. */
    NVDS_INVALID_START_ADDR,     /**< NVDS invalid start address. */
    NVDS_INVALID_SECTORS,        /**< NVDS invalid sector. */
    NVDS_COMPACT_FAILED,         /**< NVDS failed to compact sectors. */
    NVDS_STORAGE_ACCESS_FAILED , /**< NVDS failed to access storage. */
    NVDS_GC_COMPLETE,            /**< NVDS garbage collection complete. */
    NVDS_NOT_INIT,               /**< NVDS not initialize. */
    NVDS_POINTER_NULL            /**< NVDS or driver function repalce error: NULL. */
};
/** @} */

/**@addtogroup NVDS_STRUCTURES Structures
 * @{ */
/**@brief NVDS Item tag. */
typedef uint16_t NvdsTag_t;
/** @} */

/** @addtogroup NVDS_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Initialize the sectors for NVDS.
 *
 * @note NVDS locates in the last sector of Flash.
 *
 * @param[in] start_addr: Start address of NVDS area. If the value equals zero,
                          NVDS area will locate in the last sector(s) in flash
                          memory. If the value does not equal zero, it must be
                          sector-aligned.
 * @param[in] sectors:    The number of sectors.
 *
 * @return ::NVDS_SUCCESS if successful.
 * @return error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_init(uint32_t start_addr, uint8_t sectors);

/** @addtogroup NVDS_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief De-initialize the sectors for NVDS.
 *
 * @note nvds_deinit will erase the flash sectors.
 *
 * @param[in] start_addr: Start address of NVDS area. If the value does not equal zero, 
                          it must be sector-aligned. If the value equals zero, 
                          NVDS area will locate in the last sector(s) in flash memory.
 * @param[in] sectors:    The number of sectors.
 *
 * @return Result of nvds deinit.
 ****************************************************************************************
 */
uint8_t nvds_deinit(uint32_t start_addr, uint8_t sectors);

/**
 ****************************************************************************************
 * @brief Read data from NVDS.
 *
 * @param[in]     tag:   Valid NVDS item tag.
 * @param[in,out] p_len: Pointer to the length of data.
 * @param[out]    p_buf: Data is read into the buffer.
 *
 * @return ::NVDS_SUCCESS if successful.
 * @return error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_get(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Write data to NVDS. If the tag does not exist, create one.
 *
 * @param[in] tag:   Valid NVDS item tag.
 * @param[in] len:   Length of data to be written, the max length of data is 1024 bytes.
 * @param[in] p_buf: Data to be written.
 *
 * @return ::NVDS_SUCCESS: if successful.
 * @return error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_put(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Delete a tag in NVDS
 *
 * @param[in] tag: The tag to be deleted.
 *
 * @return NVDS_SUCCESS: If the deletion is successful.
 * @return Error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_del(NvdsTag_t tag);

/**
 ****************************************************************************************
 * @brief Get the length of a tag in NVDS
 *
 * @param[in] tag: The tag to get the length.
 *
 * @return length: if tag exists.
 * @return 0: if tag does not exist.
 ****************************************************************************************
 */
uint16_t nvds_tag_length(NvdsTag_t tag);


/** @addtogroup LOCAL_FLASH_FUNCTIONS Local Flash Functions
 * @{ */
/**@brief Flash operation API based on hal flash.*/
/**
 ****************************************************************************************
 * @brief Erase flash chip.
 *
 * @return true if successful.
 * @return false if not successful.
 ****************************************************************************************
 */
bool local_hal_flash_erase_chip(void);

/**
 ****************************************************************************************
 * @brief Erase flash region.
 *
 * @note All sectors that have address in range of [addr, addr+len]
 *       will be erased. If addr is not sector aligned, preceding data
 *       on the sector that addr belongs to will also be erased.
 *       If (addr + size) is not sector aligned, the whole sector
 *       will also be erased.
 *
 * @param[in] addr    start address in flash to write data to.
 * @param[in] size    number of bytes to write.
 *
 * @return true if successful.
 * @return false if not successful.
 ****************************************************************************************
 */
bool local_hal_flash_erase(const uint32_t addr, const uint32_t size);

/**
 *******************************************************************************
 * @brief Write flash Memory.
 *
 * @param[in]       addr    start address in flash to write data to.
 * @param[in,out]   buf     buffer of data to write.
 * @param[in]       size    number of bytes to write.
 *
 * @return          number of bytes written
 *******************************************************************************
 */
uint32_t local_hal_flash_write(const uint32_t addr, const uint8_t *buf, const uint32_t size);

/**
 *******************************************************************************
 * @brief Read flash Memory.
 *
 * @param[in]       addr    start address in flash to read data.
 * @param[in,out]   buf     buffer to read data to.
 * @param[in]       size    number of bytes to read.
 *
 * @return          number of bytes read
 *******************************************************************************
 */
uint32_t local_hal_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size);

/**
 *******************************************************************************
 * @brief Get Flash information.
 *
 * @param[in,out] id Pointer to flash id.
 * @param[in,out] size Pointer to flash size.
 *
 *******************************************************************************
 */
void local_hal_flash_get_info(uint32_t *id, uint32_t *size);

/**
 *******************************************************************************
 * @brief Get encrypted and decrypted status in write-read operations.
 *
 * @return true             Enable encrypted and decrypted.
 * @return false            Disable encrypted and decrypted.
 *******************************************************************************
 */
bool local_hal_flash_get_security(void);

/**
 *******************************************************************************
 * @brief Enable encrypted and decrypted in write-read operations.
 *
 * @param[in]       enable  control encrypted and decrypte.
 *
 *******************************************************************************
 */
void local_hal_flash_set_security(bool enable);

/**
 *******************************************************************************
 * @brief Write flash Memory reliably. 
 *
 * @note It's possible that the data was not written into Flash Memory
 *       successfully. This function reads the data from Flash Memory to check
 *       the reliability of programming Flash Memory.
 * @param[in]       addr    start address in flash to write data to.
 * @param[in,out]   buf     buffer of data to write.
 * @param[in]       size    number of bytes to write.
 *
 * @return          number of bytes written
 *******************************************************************************
 */
uint32_t local_hal_flash_write_r(const uint32_t addr, const uint8_t *buf, const uint32_t size);

/** @} */

/** @} */

#endif

/** @} */
/** @} */

