/**
 *****************************************************************************************
 *
 * @file dfu_master.h
 *
 * @brief  DFU master API.
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
#ifndef __DFU_MASTER_H__
#define __DFU_MASTER_H__

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "dfu_master_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup DFU_MASTER_MAROC Defines
 * @{
 */
#define DFU_VERSION                          0x02U   /**< The DFU Version. */

#define FAST_DFU_MODE_ENABLE                 0x02U   /**< Fast DFU Mode Enable. */
#define FAST_DFU_MODE_DISABLE                0x00U   /**< Fast DFU Mode Disable. */
#define DFU_MODE_COPY_UPGRADE                1U      /**< Copy DFU mode (Double bank, Background). */
#define DFU_MODE_NON_COPY_UPGRADE            2U      /**< Non-Copy DFU mode (Single bank, Non-background). */

#define DFU_FW_ENC_OR_SIGN_PATTERN           0xDEADBEEFU  /**< FW encryption or signature pattern. */
#define DFU_FW_SIGN_PATTERN                  0x4E474953U  /**< FW sign pattern("SIGN"). */

// DFU master tx frame max length
#define DFU_TX_FRAME_MAX                     (DFU_ONCE_PROGRAM_LEN + 15U)
// DFU master rx frame max length
#define DFU_RX_FRAME_MAX                     64U

/**
 * @defgroup DFU_MASTER_ENUM Enumerations
 * @{
 */
/**@brief DFU master event type definition. */
typedef enum
{
    FRAME_CHECK_ERROR = 0,              /**<0x00 Frame check error event. */
    IMG_INFO_CHECK_FAIL,                /**<0x01 FW info check event. */
    IMG_INFO_LOAD_ADDR_ERROR,           /**<0x02 img info load addr error. */
    GET_INFO_FAIL,                      /**<0x03 GET info error event. */
    PRO_START_ERROR,                    /**<0x04 FW program start error event. */
    PRO_START_SUCCESS,                  /**<0x05 FW program start success event. */
    PRO_FLASH_SUCCESS,                  /**<0x06 FW program success event. */
    PRO_FLASH_FAIL,                     /**<0x07 FW program fail event. */
    PRO_END_SUCCESS,                    /**<0x08 FW program end success event. */
    PRO_END_FAIL,                       /**<0x09 FW program end fail event. */
    ERASE_START_SUCCESS,                /**<0x0A Erase start success. */
    ERASE_SUCCESS,                      /**<0x0B Erase success. */
    ERASE_END_SUCCESS,                  /**<0x0C Erase end success. */
    ERASE_REGION_NOT_ALIGNED,           /**<0x0D Erase regions not aligned. */
    ERASE_REGION_OVERLAP,               /**<0x0E Erase regions overlap. */
    ERASE_FLASH_FAIL,                   /**<0x0F Erase flash fail. */
    ERASE_REGION_NOT_EXIST,             /**<0x10 Erase region not exist. */
    FAST_DFU_PRO_FLASH_SUCCESS,         /**<0x11 fast dfu program flash success. */
    FAST_DFU_FLASH_FAIL,                /**<0x12 FW write flash error. */
    DFU_FW_SAVE_ADDR_CONFLICT,          /**<0x13 DFU address conflict. */
    DFU_ACK_TIMEOUT                     /**<0x14 ACK timeout. */
}dfu_m_event_t;
/** @} */

/**
 * @defgroup DFU_MASTER_STRUCT Structures
 * @{
 */
/**@brief Boot information definition. */
typedef struct
{
    uint32_t bin_size;
    uint32_t check_sum;
    uint32_t load_addr;
    uint32_t run_addr;
    uint32_t xqspi_xip_cmd;
    uint32_t xqspi_speed:4;           /*!< bit: 0..3  clock speed */
    uint32_t code_copy_mode:1;        /*!< bit: 4 code copy mode */
    uint32_t system_clk:3;            /*!< bit: 5..7 system clock */
    uint32_t check_image:1;           /*!< bit: 8 check image */
    uint32_t boot_delay:1;            /*!< bit: 9 boot delay time */
    uint32_t signature_algorithm:2;   /*!< bit: 10..11 signature algorithm */
    uint32_t reserved:20;             /*!< bit: 20 reserved */
} boot_info_t;

/**@brief IMG information definition. */
typedef struct
{
    uint16_t        pattern;           /**< IMG info pattern. */
    uint16_t        version;           /**< IMG version. */
    boot_info_t     boot_info;         /**< IMG boot info. */
    uint8_t         comments[12];      /**< IMG comments. */
}dfu_img_info_t;

/**@brief DFU master used function config definition. */
typedef struct
{
    void (*dfu_m_get_img_info)(dfu_img_info_t *img_info);                      /**< Get information about the firmware to be updated. */
    void (*dfu_m_get_img_data)(uint32_t addr, uint8_t *data, uint16_t len);    /**< Get data about the firmware to be updated. */
    void (*dfu_m_send_data)(uint8_t *data, uint16_t len);                      /**< Send data to peer device. */
    void (*dfu_m_event_handler)(dfu_m_event_t event, uint8_t progress);        /**< Send event to app. */
    uint32_t (*dfu_m_get_time)(void);                                          /**< Get system current time, in ms. */
}dfu_m_func_cfg_t;
/** @} */

/**
 * @defgroup DFU_MASTER_TYPEDEF Typedefs
 * @{
 */
/**@brief DFU Master Receive CMD Callback type. */
typedef void (*dfu_m_rev_cmd_cb_t)(void);
/** @} */

/**
 * @defgroup DFU_MASTER_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Function for initializing the DFU master.
 *
 * @note When APP wants to add DFU master feature, all functions in @ref dfu_m_func_cfg_t should be registered.
 *
 * @param[in]  dfu_m_func_cfg: DFU master used functions.
 * @param[in]  once_send_size: DFU master once send size.
 *****************************************************************************************
 */
void dfu_m_init(dfu_m_func_cfg_t *dfu_m_func_cfg, uint16_t once_send_size);

/**
 *****************************************************************************************
 * @brief Start DFU process.
 *
 *****************************************************************************************
 */
void dfu_m_start(void);

/**
 *****************************************************************************************
 * @brief Function for reset the DFU cmd parse state.
 * @note This function should be called when restart DFU.
 *****************************************************************************************
 */
void dfu_m_parse_state_reset(void);

/**
 *****************************************************************************************
 * @brief Function for checking DFU master cmd.
 * @note This function should be called in loop.
 *
 * @param[in] rev_cmd_cb: Receive CMD callback.
 *****************************************************************************************
 */
void dfu_m_schedule(dfu_m_rev_cmd_cb_t rev_cmd_cb);

/**
 *****************************************************************************************
 * @brief Function for parse received data.
 *
 * @param[in]  data: Received data.
 * @param[in]  len: Data length.
 *****************************************************************************************
 */
void dfu_m_cmd_parse(const uint8_t* data, uint16_t len);

#if DFU_ASYNC_TX_ENABLE
/**
 *****************************************************************************************
 * @brief When asynchronous transmission mode is enabled, this function should be called when data sent completely
 *****************************************************************************************
 */
void dfu_m_send_data_cmpl_process(void);

#if DFU_BLE_ENABLE
/**
 *****************************************************************************************
 * @brief Set the enabled status of fast dfu mode.
 * @note Only valid in BLE mode.
 *
 * @param[in] setting: FAST_DFU_MODE_ENABLE or FAST_DFU_MODE_DISABLE.
 *****************************************************************************************
 */
void dfu_m_fast_dfu_mode_set(uint8_t setting);

/**
 *****************************************************************************************
 * @brief Get the enabled status of fast dfu mode.
  * @note Only valid in BLE mode.
 *
 * @retval FAST_DFU_MODE_ENABLE or FAST_DFU_MODE_DISABLE.
 *****************************************************************************************
 */
uint8_t dfu_m_fast_dfu_mode_get(void);

/**
 *****************************************************************************************
 * @brief When fast transmission mode is enabled, this function should be called when data sent completely
 * @note Only valid in BLE mode.
 *****************************************************************************************
 */
void dfu_m_fast_send_data_cmpl_process(void);
#endif /* DFU_BLE_ENABLE */
#endif /* DFU_ASYNC_TX_ENABLE */

/**
 *****************************************************************************************
 * @brief Get the flash programmed size.
 *
 * @retval Flash programmed size.
 *****************************************************************************************
 */
uint32_t dfu_m_get_program_size(void);

/** @} */
#endif
