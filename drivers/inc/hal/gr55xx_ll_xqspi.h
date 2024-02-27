/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_xqspi.h
 * @author  BLE SDK Team
 * @brief   Header file containing functions prototypes of XQSPI LL library.
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_XQSPI XQSPI
  * @brief XQSPI LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_XQSPI_H__
#define __GR55xx_LL_XQSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (XQSPI)

/** @defgroup LL_XQSPI_DRIVER_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup XQSPI_LL_ES_INIT XQSPI Exported init structure
  * @{
  */

/**
  * @brief  XQSPI init structures definition
  */
typedef struct _ll_xqspi_init_t
{
    uint32_t mode;                      /**< Specifies the work mode, XIP mode or QSPI mode.
                                             This parameter can be a value of @ref XQSPI_LL_EC_MODE.*/

    uint32_t cache_mode;                /**< Specifies the cache mode in XIP mode.
                                             This parameter can be a value of @ref XQSPI_LL_EC_CACHE_MODE.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_enable_cache().*/

    uint32_t read_cmd;                  /**< Specifies the XQSPI read command in XIP mode.
                                             This parameter can be a value of @ref XQSPI_LL_EC_XIP_READ_CMD.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_set_xip_cmd().*/

    uint32_t data_size;                 /**< Specifies the XQSPI data width, only in QSPI mode.
                                             This parameter can be a value of @ref XQSPI_LL_EC_QSPI_DATASIZE.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_set_qspi_datasize().*/

    uint32_t data_order;                /**< Specifies the XQSPI data order, MSB oe LSB, only in QSPI mode.
                                             This parameter can be a value of @ref XQSPI_LL_EC_QSPI_DATAORDER.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_set_qspi_data_order().*/

    uint32_t clock_polarity;            /**< Specifies the serial clock steady state.
                                             This parameter can be a value of @ref XQSPI_LL_EC_QSPI_POLARITY in XIP mode or @ref XQSPI_LL_EC_QSPI_POLARITY in QSPI mode.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_set_xip_cpol() or @ref ll_xqspi_set_qspi_cpol().*/

    uint32_t clock_phase;               /**< Specifies the clock active edge for the bit capture.
                                             This parameter can be a value of @ref XQSPI_LL_EC_QSPI_PHASE in XIP mode or @ref XQSPI_LL_EC_QSPI_PHASE in QSPI mode.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_set_xip_cpha() or @ref ll_xqspi_set_qspi_cpha().*/

    uint32_t baud_rate;                 /**< Specifies the BaudRate  be used to configure the transmit and receive SCK clock.
                                             This parameter can be a value of @ref XQSPI_LL_EC_QSPI_BAUD_REAT.

                                             This feature can be modified afterwards using unitary function @ref ll_xqspi_set_qspi_speed().*/

} ll_xqspi_init_t;

/** @} */

/** @} */

/**
  * @defgroup  XQSPI_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup XQSPI_LL_Exported_Constants XQSPI Exported Constants
  * @{
  */

/** @defgroup XQSPI_LL_EC_MODE XQSPI work mode
  * @{
  */
#define LL_XQSPI_MODE_XIP                   0   /**< XIP mode   */
#define LL_XQSPI_MODE_QSPI                  1   /**< QSPI mode  */
/** @} */

/** @defgroup XQSPI_LL_EC_XIP_READ_CMD XIP read command
  * @{
  */
#define LL_XQSPI_XIP_CMD_READ               0x03    /**< Read mode                  */
#define LL_XQSPI_XIP_CMD_FAST_READ          0x0B    /**< Fast Read mode             */
#define LL_XQSPI_XIP_CMD_DUAL_OUT_READ      0x3B    /**< Dual-Out Fast Read mode     */
#define LL_XQSPI_XIP_CMD_DUAL_IO_READ       0xBB    /**< Dual-IO Fast Read mode      */
#define LL_XQSPI_XIP_CMD_QUAD_OUT_READ      0x6B    /**< Quad-Out Fast Read mode     */
#define LL_XQSPI_XIP_CMD_QUAD_IO_READ       0xEB    /**< Quad-IO Fast Read mode      */
/** @} */

/** @defgroup XQSPI_LL_EC_XIP_SS Slave select
  * @{
  */
#define LL_XQSPI_XIP_SS0                    (1UL << XQSPI_XIP_CFG_SS_Pos)   /**< Slave select 0 */
#define LL_XQSPI_XIP_SS1                    (2UL << XQSPI_XIP_CFG_SS_Pos)   /**< Slave select 1 */
#define LL_XQSPI_XIP_SS2                    (4UL << XQSPI_XIP_CFG_SS_Pos)   /**< Slave select 2 */
#define LL_XQSPI_XIP_SS3                    (8UL << XQSPI_XIP_CFG_SS_Pos)   /**< Slave select 3 */
/** @} */

/** @defgroup XQSPI_LL_EC_XIP_ADDR_MODE Address bytes in command
  * @{
  */
#define LL_XQSPI_XIP_ADDR_3BYTES            0x00000000UL                /**< Address command is 3 bytes  */
#define LL_XQSPI_XIP_ADDR_4BYTES            XQSPI_XIP_CFG_ADDR4         /**< Address command is 4 bytes  */
/** @} */

/** @defgroup XQSPI_LL_EC_XIP_ENDIAN Read data endian mode
  * @{
  */
#define LL_XQSPI_XIP_ENDIAN_BIG             0x00000000UL                /**< Read data in big endian    */
#define LL_XQSPI_XIP_ENDIAN_LITTLE          XQSPI_XIP_CFG_LE32          /**< Read data in little endian */
/** @} */

/** @defgroup XQSPI_LL_EC_CACHE_MODE XIP cache mode
  * @{
  */
#define LL_XQSPI_CACHE_DIS                  0   /**< Cache OFF */
#define LL_XQSPI_CACHE_EN                   1   /**< Cache ON  */
/** @} */

/** @defgroup XQSPI_LL_EC_CACHE_FIFO_MODE Cache FIFO mode
  * @{
  */
#define LL_XQSPI_CACHE_FIFO_NORMAL          0x00000000UL                /**< FIFO in normal mode */
#define LL_XQSPI_CACHE_FIFO_CLEAR           XQSPI_CACHE_CTRL0_FIFO      /**< FIFO in clear mode  */
/** @} */

/** @defgroup XQSPI_LL_EC_CACHE_HITMISS_COUNTER_MODE Cache hit/miss counters mode
  * @{
  */
#define LL_XQSPI_CACHE_HITMISS_NORMAL       0x00000000UL                /**< Hit/Miss counters in normal mode */
#define LL_XQSPI_CACHE_HITMISS_CLEAR        XQSPI_CACHE_CTRL0_HITMISS   /**< Hit/Miss counters in clear mode  */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_FLAG QSPI Flags Defines
  * @brief    Flags defines which can be used with LL_XQSPI_ReadReg function
  * @{
  */
#define LL_XQSPI_QSPI_STAT_RFTF             XQSPI_QSPI_STAT_RXWMARK     /**< Rx FIFO watermark flag         */
#define LL_XQSPI_QSPI_STAT_RFF              XQSPI_QSPI_STAT_RXFULL      /**< Rx FIFO full flag              */
#define LL_XQSPI_QSPI_STAT_RFE              XQSPI_QSPI_STAT_RXEMPTY     /**< Rx FIFO empty flag             */
#define LL_XQSPI_QSPI_STAT_TFTF             XQSPI_QSPI_STAT_TXWMARK     /**< Tx FIFO watermark flag         */
#define LL_XQSPI_QSPI_STAT_TFF              XQSPI_QSPI_STAT_TXFULL      /**< Tx FIFO full flag              */
#define LL_XQSPI_QSPI_STAT_TFE              XQSPI_QSPI_STAT_TXEMPTY     /**< Tx FIFO empty flag             */
#define LL_XQSPI_QSPI_STAT_BUSY             XQSPI_QSPI_STAT_XFERIP      /**< Busy flag                      */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_IT QSPI interrupt Defines
  * @brief    Interrupt defines which can be used with LL_XQSPI_ReadReg and  LL_XQSPI_WriteReg functions
  * @{
  */
#define LL_XQSPI_QSPI_IM_DONE               XQSPI_QSPI_XFER_DPULSE_Msk  /**< Transmite Done Interrupt enable            */
#define LL_XQSPI_QSPI_IM_RFF                XQSPI_QSPI_RX_FPULSE_Msk    /**< Receive FIFO Full Interrupt enable         */
#define LL_XQSPI_QSPI_IM_RFTF               XQSPI_QSPI_RX_WPULSE_Msk    /**< Receive FIFO Watermark Interrupt enable    */
#define LL_XQSPI_QSPI_IM_TFTF               XQSPI_QSPI_TX_WPULSE_Msk    /**< Transmit FIFO Watermark Interrupt enable   */
#define LL_XQSPI_QSPI_IM_TFE                XQSPI_QSPI_TX_EPULSE_Msk    /**< Transmit FIFO Empty Interrupt enable       */

#define LL_XQSPI_QSPI_IS_DONE               XQSPI_QSPI_XFER_DPULSE_Msk  /**< Transmite Done Interrupt flag              */
#define LL_XQSPI_QSPI_IS_RFF                XQSPI_QSPI_RX_FPULSE_Msk    /**< Receive FIFO Full Interrupt flag           */
#define LL_XQSPI_QSPI_IS_RFTF               XQSPI_QSPI_RX_WPULSE_Msk    /**< Receive FIFO Watermark Interrupt flag      */
#define LL_XQSPI_QSPI_IS_TFTF               XQSPI_QSPI_TX_WPULSE_Msk    /**< Transmit FIFO Watermark Interrupt flag     */
#define LL_XQSPI_QSPI_IS_TFE                XQSPI_QSPI_TX_EPULSE_Msk    /**< Transmit FIFO Empty Interrupt flag         */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_FIFO_WATERMARK QSPI FIFO Watermark
  * @{
  */
#define LL_XQSPI_QSPI_FIFO_WATERMARK_1_8    0UL                       /**< FIFO depth/8      */
#define LL_XQSPI_QSPI_FIFO_WATERMARK_1_4    1UL                       /**< FIFO depth/4      */
#define LL_XQSPI_QSPI_FIFO_WATERMARK_1_2    2UL                       /**< FIFO depth/2      */
#define LL_XQSPI_QSPI_FIFO_WATERMARK_3_4    3UL                       /**< FIFO depth*3/4    */
#define LL_XQSPI_QSPI_FIFO_DEPTH            16UL                      /**< FIFO full depth   */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_FRAMEFORMAT QSPI Frame Format
  * @{
  */
#define LL_XQSPI_QSPI_FRF_SPI               0x00000000UL                            /**< SPI frame format for transfer      */
#define LL_XQSPI_QSPI_FRF_DUALSPI           (2UL << XQSPI_QSPI_AUXCTRL_QMODE_Pos)   /**< Dual-SPI frame format for transfer */
#define LL_XQSPI_QSPI_FRF_QUADSPI           (3UL << XQSPI_QSPI_AUXCTRL_QMODE_Pos)   /**< Quad-SPI frame format for transfer */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_DATAORDER QSPI Data Order
  * @{
  */
#define LL_XQSPI_QSPI_LSB                   0x00000000UL                /**< LSB first for transfer */
#define LL_XQSPI_QSPI_MSB                   XQSPI_QSPI_CTRL_MSB1ST      /**< MSB first for transfer */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_DATASIZE QSPI Datawidth
  * @{
  */
#define LL_XQSPI_QSPI_DATASIZE_4BIT         0x00000000UL                            /**< Data length for XQSPI transfer:  4 bits */
#define LL_XQSPI_QSPI_DATASIZE_8BIT         (1UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer:  8 bits */
#define LL_XQSPI_QSPI_DATASIZE_12BIT        (2UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer: 12 bits */
#define LL_XQSPI_QSPI_DATASIZE_16BIT        (3UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer: 16 bits */
#define LL_XQSPI_QSPI_DATASIZE_20BIT        (4UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer: 20 bits */
#define LL_XQSPI_QSPI_DATASIZE_24BIT        (5UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer: 24 bits */
#define LL_XQSPI_QSPI_DATASIZE_28BIT        (6UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer: 28 bits */
#define LL_XQSPI_QSPI_DATASIZE_32BIT        (7UL << XQSPI_QSPI_AUXCTRL_BITSIZE_Pos) /**< Data length for XQSPI transfer: 32 bits */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_PHASE QSPI Clock Phase
  * @{
  */
#define LL_XQSPI_SCPHA_1EDGE                0   /**< First clock transition is the first data capture edge  */
#define LL_XQSPI_SCPHA_2EDGE                1   /**< Second clock transition is the first data capture edge */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_POLARITY QSPI Clock Polarity
  * @{
  */
#define LL_XQSPI_SCPOL_LOW                  0   /**< Clock to 0 when idle */
#define LL_XQSPI_SCPOL_HIGH                 1   /**< Clock to 1 when idle */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_BAUD_REAT QSPI Buad Rate
  * @{
  */
#define LL_XQSPI_BAUD_RATE_64M              0x00000000UL                                /**< Clock to 64MHz */
#define LL_XQSPI_BAUD_RATE_48M              (1UL << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos)   /**< Clock to 48MHz */
#define LL_XQSPI_BAUD_RATE_32M              (2UL << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos)   /**< Clock to 32MHz */
#define LL_XQSPI_BAUD_RATE_24M              (3UL << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos)   /**< Clock to 24MHz */
#define LL_XQSPI_BAUD_RATE_16M              (4UL << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos)   /**< Clock to 16MHz */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_PRESENT QSPI Present Bypass
  * @{
  */
#define LL_XQSPI_ENABLE_PRESENT             0   /**< Enable Present Bypass  */
#define LL_XQSPI_DISABLE_PRESENT            1   /**< Disable Present Bypass */
/** @} */

/** @defgroup XQSPI_LL_EC_QSPI_FLASH_WRITE QSPI Flash write bits
  * @{
  */
#define LL_XQSPI_FLASH_WRITE_128BIT         0   /**< 128bits flash write  */
#define LL_XQSPI_FLASH_WRITE_32BIT          1   /**< 32bits  flash write  */
/** @} */

/** @defgroup XQSPI_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL XQSPI InitStrcut default configuartion
  */
#define LL_XQSPI_DEFAULT_CONFIG                                 \
{                                                               \
    .mode               = LL_XQSPI_MODE_QSPI,                   \
    .cache_mode         = LL_XQSPI_CACHE_EN,                    \
    .read_cmd           = LL_XQSPI_XIP_CMD_READ,                \
    .data_size          = LL_XQSPI_QSPI_DATASIZE_8BIT,          \
    .data_order         = LL_XQSPI_QSPI_MSB,                    \
    .clock_polarity     = LL_XQSPI_SCPOL_HIGH,                  \
    .clock_phase        = LL_XQSPI_SCPHA_2EDGE,                 \
    .baud_rate          = LL_XQSPI_BAUD_RATE_16M,               \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup XQSPI_LL_Exported_Macros XQSPI Exported Macros
  * @{
  */

/** @defgroup XQSPI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in XQSPI register
  * @param  __instance__ XQSPI instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_XQSPI_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in XQSPI register
  * @param  __instance__ XQSPI instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_XQSPI_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup XQSPI_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup XQSPI_LL_XQSPI_Configuration Cache driver functions
  * @{
  */

/**
  * @brief  Enable cache function
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_cache(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_DIS);
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/**
  * @brief  Disable cache function
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_cache(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_DIS);
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/**
  * @brief  Check if cache function is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_cache(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_DIS) != (XQSPI_CACHE_CTRL0_DIS));
}

/**
  * @brief  Enable tag memory flush
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | TAG                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_cache_flush(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_FLUSH);
}

/**
  * @brief  Disable tag memory flush
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | TAG                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_cache_flush(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_FLUSH);
}

/**
  * @brief  Check if tag memory flush is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | TAG                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_cache_flush(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_FLUSH) == (XQSPI_CACHE_CTRL0_FLUSH));
}

/**
  * @brief  Set FIFO mode
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | FIFO                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_CACHE_FIFO_NORMAL
  *         @arg @ref LL_XQSPI_CACHE_FIFO_CLEAR
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_cache_fifo(xqspi_regs_t *XQSPIx, uint32_t mode)
{
    MODIFY_REG(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_FIFO, mode);
}

/**
  * @brief  Get FIFO mode
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | FIFO                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_CACHE_FIFO_NORMAL
  *         @arg @ref LL_XQSPI_CACHE_FIFO_CLEAR
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_cache_fifo(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_FIFO));
}

/**
  * @brief  Set HIT/MISS mode
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | HITMISS                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_CACHE_HITMISS_NORMAL
  *         @arg @ref LL_XQSPI_CACHE_HITMISS_CLEAR
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_cache_hitmiss(xqspi_regs_t *XQSPIx, uint32_t mode)
{
    MODIFY_REG(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_HITMISS, mode);
}

/**
  * @brief  Get HIT/MISS mode
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | HITMISS                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_CACHE_HITMISS_NORMAL
  *         @arg @ref LL_XQSPI_CACHE_HITMISS_CLEAR
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_cache_hitmiss(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_HITMISS));
}

/**
  * @brief  Set debugbus configurations signals
  * @note   These bits should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | DBGBUS_SEL                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  sel This parameter can between: 0 ~ 0x7
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_cache_dbgbus(xqspi_regs_t *XQSPIx, uint32_t sel)
{
    MODIFY_REG(XQSPIx->CACHE.CTRL1, XQSPI_CACHE_CTRL1_DBGBUS_SEL, sel << XQSPI_CACHE_CTRL1_DBGBUS_SEL_Pos);
}

/**
  * @brief  Get debugbus configurations signals
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | DBGBUS_SEL                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 0x7
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_cache_dbgbus(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->CACHE.CTRL1, XQSPI_CACHE_CTRL1_DBGBUS_SEL) >> XQSPI_CACHE_CTRL1_DBGBUS_SEL_Pos);
}

/**
  * @brief  Enable debug bus mux
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | DBGMUX_EN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_cache_dbgmux(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->CACHE.CTRL1, XQSPI_CACHE_CTRL1_DBGMUX_EN);
}

/**
  * @brief  Disable debug bus mux
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | DBGMUX_EN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_cache_dbgmux(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->CACHE.CTRL1, XQSPI_CACHE_CTRL1_DBGMUX_EN);
}

/**
  * @brief  Check if debug bus mux is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | DBGMUX_EN                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_cache_dbgmux(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->CACHE.CTRL1, XQSPI_CACHE_CTRL1_DBGMUX_EN) != (XQSPI_CACHE_CTRL1_DBGMUX_EN));
}

/**
  * @brief  Get hit counter
  * @note   This bit only be read.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | HIT_COUNT            | HITCOUNT                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 0xFFFFFFFF
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_cache_hitcount(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_REG(XQSPIx->CACHE.HIT_COUNT));
}

/**
  * @brief  Get miss counter
  * @note   This bit only be read.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MISS_COUNT           | MISSCOUNT                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 0xFFFFFFFF
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_cache_misscount(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_REG(XQSPIx->CACHE.MISS_COUNT));
}

/**
  * @brief  Get cache status
  * @note   This bit only be read.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT                 | STAT                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 1
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_cache_flag(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->CACHE.STAT, XQSPI_CACHE_STAT));
}

/** @} */

/** @defgroup XQSPI_LL_XIP_Configuration XIP LL driver functions
  * @{
  */

/**
  * @brief  Set read command
  * @note   These bits should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | CFG_CMD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cmd This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_XIP_CMD_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_FAST_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_DUAL_OUT_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_DUAL_IO_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_QUAD_OUT_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_QUAD_IO_READ
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_cmd(xqspi_regs_t *XQSPIx, uint32_t cmd)
{
    MODIFY_REG(XQSPIx->XIP.CTRL0, XQSPI_XIP_CFG_CMD, cmd);
}

/**
  * @brief  Get read command
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | CFG_CMD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_XIP_CMD_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_FAST_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_DUAL_OUT_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_DUAL_IO_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_QUAD_OUT_READ
  *         @arg @ref LL_XQSPI_XIP_CMD_QUAD_IO_READ
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_cmd(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL0, XQSPI_XIP_CFG_CMD));
}

/**
  * @brief  Enable high performance mode
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_HPEN                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_xip_hp(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_HPEN);
}

/**
  * @brief  Disable high performance mode
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_HPEN                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_xip_hp(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_HPEN);
}

/**
  * @brief  Check if high performance mode is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_HPEN                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_xip_hp(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_HPEN) == (XQSPI_XIP_CFG_HPEN));
}

/**
  * @brief  Set slave select
  * @note   These bits should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_SS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  ss This parameter can be one or more of the following values:
  *         @arg @ref LL_XQSPI_XIP_SS0
  *         @arg @ref LL_XQSPI_XIP_SS1
  *         @arg @ref LL_XQSPI_XIP_SS2
  *         @arg @ref LL_XQSPI_XIP_SS3
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_ss(xqspi_regs_t *XQSPIx, uint32_t ss)
{
    MODIFY_REG(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_SS, ss);
}

/**
  * @brief  Get slave select
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_SS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_XIP_SS0
  *         @arg @ref LL_XQSPI_XIP_SS1
  *         @arg @ref LL_XQSPI_XIP_SS2
  *         @arg @ref LL_XQSPI_XIP_SS3
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_ss(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_SS));
}

/**
  * @brief  Set clock phase
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_CPHA                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cpha This parameter can be one or more of the following values:
  *         @arg @ref LL_XQSPI_SCPHA_1EDGE
  *         @arg @ref LL_XQSPI_SCPHA_2EDGE
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_cpha(xqspi_regs_t *XQSPIx, uint32_t cpha)
{
    MODIFY_REG(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_CPHA, cpha << XQSPI_XIP_CFG_CPHA_Pos);
}

/**
  * @brief  Get clock phase
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_CPHA                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_SCPHA_1EDGE
  *         @arg @ref LL_XQSPI_SCPHA_2EDGE
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_cpha(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_CPHA) >> XQSPI_XIP_CFG_CPHA_Pos);
}

/**
  * @brief  Set clock polarity
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_CPOL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cpol This parameter can be one or more of the following values:
  *         @arg @ref LL_XQSPI_SCPOL_LOW
  *         @arg @ref LL_XQSPI_SCPOL_HIGH
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_cpol(xqspi_regs_t *XQSPIx, uint32_t cpol)
{
    MODIFY_REG(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_CPOL, cpol << XQSPI_XIP_CFG_CPOL_Pos);
}

/**
  * @brief  Get clock polarity
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_CPOL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_SCPOL_LOW
  *         @arg @ref LL_XQSPI_SCPOL_HIGH
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_cpol(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_CPOL) >> XQSPI_XIP_CFG_CPOL_Pos);
}

/**
  * @brief  Set address bytes in command
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_ADDR4                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  size This parameter can be one or more of the following values:
  *         @arg @ref LL_XQSPI_XIP_ADDR_3BYTES
  *         @arg @ref LL_XQSPI_XIP_ADDR_4BYTES
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_addr_size(xqspi_regs_t *XQSPIx, uint32_t size)
{
    MODIFY_REG(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_ADDR4, size);
}

/**
  * @brief  Get address bytes in command
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_ADDR4                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_XIP_ADDR_3BYTES
  *         @arg @ref LL_XQSPI_XIP_ADDR_4BYTES
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_addr_size(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_ADDR4));
}

/**
  * @brief  Set endian in reading data
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_LE32                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  endian This parameter can be one or more of the following values:
  *         @arg @ref LL_XQSPI_XIP_ENDIAN_BIG
  *         @arg @ref LL_XQSPI_XIP_ENDIAN_LITTLE
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_endian(xqspi_regs_t *XQSPIx, uint32_t endian)
{
    MODIFY_REG(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_LE32, endian);
}

/**
  * @brief  Get endian in reading data
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL1                | CFG_LE32                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_XIP_ENDIAN_BIG
  *         @arg @ref LL_XQSPI_XIP_ENDIAN_LITTLE
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_endian(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL1, XQSPI_XIP_CFG_LE32));
}

/**
  * @brief  Set high performance command
  * @note   These bits should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL2                | CFG_HPMODE                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cmd    This value is specified by different QSPI FLASH memory vendor to enter into its status register
  *                to activate HP mode in dual I/O and Quad I/O access. This parameter can between: 0 ~ 0xFF.
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_hp_cmd(xqspi_regs_t *XQSPIx, uint32_t cmd)
{
    MODIFY_REG(XQSPIx->XIP.CTRL2, XQSPI_XIP_CFG_HPMODE, cmd << XQSPI_XIP_CFG_HPMODE_Pos);
}

/**
  * @brief  Get high performance command
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL2                | CFG_HPMODE                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 0xFF.
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_hp_cmd(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL2, XQSPI_XIP_CFG_HPMODE) >> XQSPI_XIP_CFG_HPMODE_Pos);
}

/**
  * @brief  Set dummy cycles in command
  * @note   These bits should not be changed when XIP is ongoing.
  *         - Fast Read Dual I/O: dummycycles = 4 * cycles + 4
  *         - Fast Read Quad I/O: dummycycles = 2 * cycles + 2
  *         - Fast Read Dual Out: dummycycles = 8 * cycles
  *         - Fast Read Quad Out: dummycycles = 8 * cycles
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL2                | CFG_DUMMYCYCLES                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cycles This parameter can between: 0 ~ 0xF.
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_dummycycles(xqspi_regs_t *XQSPIx, uint32_t cycles)
{
    MODIFY_REG(XQSPIx->XIP.CTRL2, XQSPI_XIP_CFG_DUMMYCYCLES, cycles << XQSPI_XIP_CFG_DUMMYCYCLES_Pos);
}

/**
  * @brief  Get dummy cycles in command
  * @note   - Fast Read Dual I/O: dummycycles = 4 * cycles + 4
  *         - Fast Read Quad I/O: dummycycles = 2 * cycles + 2
  *         - Fast Read Dual Out: dummycycles = 8 * cycles
  *         - Fast Read Quad Out: dummycycles = 8 * cycles
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL2                | CFG_DUMMYCYCLES                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 0xF.
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_dummycycles(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL2, XQSPI_XIP_CFG_DUMMYCYCLES));
}

/**
  * @brief  Set dummy cycles in high performance end
  * @note   These bits should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL2                | CFG_ENDDUMMY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cycles This parameter can between: 0 ~ 3.
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_xip_dummy_hp(xqspi_regs_t *XQSPIx, uint32_t cycles)
{
    MODIFY_REG(XQSPIx->XIP.CTRL2, XQSPI_XIP_CFG_ENDDUMMY, cycles << XQSPI_XIP_CFG_ENDDUMMY_Pos);
}

/**
  * @brief  Get dummy cycles in high performance end
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL2                | CFG_ENDDUMMY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 3.
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_dummy_hp(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.CTRL2, XQSPI_XIP_CFG_ENDDUMMY) >> XQSPI_XIP_CFG_ENDDUMMY_Pos);
}

/**
  * @brief  Enable XIP mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL3                | EN_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_xip(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->XIP.CTRL3, XQSPI_XIP_EN_REQ);
}

/**
  * @brief  Disable XIP mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL3                | EN_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_xip(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->XIP.CTRL3, XQSPI_XIP_EN_REQ);
}

/**
  * @brief  Check if XIP mode is enabled
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL3                | EN_REQ                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_xip(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->XIP.CTRL3, XQSPI_XIP_EN_REQ) == (XQSPI_XIP_EN_REQ));
}

/**
  * @brief  Get XIP status
  * @note   This bit is read-only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT                 | EN_OUT                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 1
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_xip_flag(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.STAT, XQSPI_XIP_EN_OUT));
}

/**
  * @brief  Check if XIP interrupt is enabled
  * @note   This bit is read-only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | INT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 1
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_xip_it(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.INTEN, XQSPI_XIP_INT_EN));
}

/**
  * @brief  Get XIP interrupt flag
  * @note   This bit is read-only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | INT_STAT                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 1
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_flag_xip_it(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.INTSTAT, XQSPI_XIP_INT_STAT));
}

/**
  * @brief  Get XIP interrupt request
  * @note   This bit is read-only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTREQ               | INT_REQ                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 1
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_req_xip_it(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->XIP.INTREQ, XQSPI_XIP_INT_REQ));
}

/**
  * @brief  Set XIP interrupt enable
  * @note   This bit is write-only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSET               | INT_SET                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_xip_it(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->XIP.INTSET, XQSPI_XIP_INT_SET);
}

/**
  * @brief  Set XIP interrupt disable
  * @note   This bit is write-only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTCLR               | INT_CLR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_xip_it(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->XIP.INTCLR, XQSPI_XIP_INT_CLR);
}

/** @} */

/** @defgroup XQSPI_LL_QSPI_Configuration QSPI driver functions
  * @{
  */

/**
  * @brief  Write 8-bit in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | TX_DATA              | DATA                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  tx_data This parameter can between: 0x00 ~ 0xFF
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_qspi_transmit_data8(xqspi_regs_t *XQSPIx, uint8_t tx_data)
{
    *((__IOM uint8_t *)&XQSPIx->QSPI.TX_DATA) = tx_data;
}

/**
  * @brief  Write 16-bit in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | TX_DATA              | DATA                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  tx_data This parameter can between: 0x00 ~ 0xFFFF
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_qspi_transmit_data16(xqspi_regs_t *XQSPIx, uint16_t tx_data)
{
    *((__IOM uint16_t *)&XQSPIx->QSPI.TX_DATA) = tx_data;
}

/**
  * @brief  Write 32-bit in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | TX_DATA              | DATA                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  tx_data This parameter can between: 0x00 ~ 0xFFFFFFFF
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_qspi_transmit_data32(xqspi_regs_t *XQSPIx, uint32_t tx_data)
{
    *((__IOM uint32_t *)&XQSPIx->QSPI.TX_DATA) = tx_data;
}

/**
  * @brief  Read 8 bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | RX_DATA              | DATA                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value between: 0x00 ~ 0xFF
  */
SECTION_RAM_CODE __STATIC_INLINE uint8_t ll_xqspi_qspi_receive_data8(xqspi_regs_t *XQSPIx)
{
    return (uint8_t)(READ_REG(XQSPIx->QSPI.RX_DATA));
}

/**
  * @brief  Read 16 bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | RX_DATA              | DATA                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value between: 0x00 ~ 0xFFFF
  */
SECTION_RAM_CODE __STATIC_INLINE uint16_t ll_xqspi_qspi_receive_data16(xqspi_regs_t *XQSPIx)
{
    return (uint16_t)(READ_REG(XQSPIx->QSPI.RX_DATA));
}

/**
  * @brief  Read 32 bits in the data register
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | RX_DATA              | DATA                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value between: 0x00 ~ 0xFFFFFFFF
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_qspi_receive_data32(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_REG(XQSPIx->QSPI.RX_DATA));
}

/**
  * @brief  Set TX FIFO threshold level
  * @note   FIFO maximum depth is 16 units.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | TXWMARK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_8
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_4
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_2
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_3_4
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_tft(xqspi_regs_t *XQSPIx, uint32_t threshold)
{
    MODIFY_REG(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_TXWMARK, threshold << XQSPI_QSPI_CTRL_TXWMARK_Pos);
}

/**
  * @brief  Get TX FIFO threshold level
  * @note   FIFO maximum depth is 16 units.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | TXWMARK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_8
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_4
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_2
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_3_4
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_tft(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_TXWMARK) >> XQSPI_QSPI_CTRL_TXWMARK_Pos);
}

/**
  * @brief  Set RX FIFO threshold level
  * @note   FIFO maximum depth is 16 units.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | RXWMARK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  threshold This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_8
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_4
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_2
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_3_4
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_rft(xqspi_regs_t *XQSPIx, uint32_t threshold)
{
    MODIFY_REG(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_RXWMARK, threshold << XQSPI_QSPI_CTRL_RXWMARK_Pos);
}

/**
  * @brief  Get RX FIFO threshold level
  * @note   FIFO maximum depth is 16 units.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | RXWMARK                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_8
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_4
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_1_2
  *         @arg @ref LL_XQSPI_QSPI_FIFO_WATERMARK_3_4
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_rft(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_RXWMARK) >> XQSPI_QSPI_CTRL_RXWMARK_Pos);
}

/**
  * @brief  Enable dummy cycles
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | MWAITEN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi_dummy(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_MWAITEN);
}

/**
  * @brief  Disable dummy cycles
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | MWAITEN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi_dummy(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_MWAITEN);
}

/**
  * @brief  Check if dummy cycles is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | MWAITEN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_qspi_dummy(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_MWAITEN) == (XQSPI_QSPI_CTRL_MWAITEN));
}

/**
  * @brief  Enable DMA mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | DMA                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi_dma(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_DMA);
}

/**
  * @brief  Disable DMA mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | DMA                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi_dma(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_DMA);
}

/**
  * @brief  Check if DMA mode is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | DMA                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_qspi_dma(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_DMA) == (XQSPI_QSPI_CTRL_DMA));
}

/**
  * @brief  Set clock polarity
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CPOL                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cpol This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_SCPOL_LOW
  *         @arg @ref LL_XQSPI_SCPOL_HIGH
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_cpol(xqspi_regs_t *XQSPIx, uint32_t cpol)
{
    MODIFY_REG(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CPOL, cpol << XQSPI_QSPI_CTRL_CPOL_Pos);
}

/**
  * @brief  Get clock polarity
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CPOL                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_SCPOL_LOW
  *         @arg @ref LL_XQSPI_SCPOL_HIGH
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_cpol(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CPOL) >> XQSPI_QSPI_CTRL_CPOL_Pos);
}

/**
  * @brief  Set clock phase
  * @note   This bit should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CPHA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  cpha This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_SCPHA_1EDGE
  *         @arg @ref LL_XQSPI_SCPHA_2EDGE
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_cpha(xqspi_regs_t *XQSPIx, uint32_t cpha)
{
    MODIFY_REG(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CPHA, cpha << XQSPI_QSPI_CTRL_CPHA_Pos);
}

/**
  * @brief  Get clock phase
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CPHA                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_SCPHA_1EDGE
  *         @arg @ref LL_XQSPI_SCPHA_2EDGE
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_cpha(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CPHA) >> XQSPI_QSPI_CTRL_CPHA_Pos);
}

/**
  * @brief  Set serial data order
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | MSB1ST                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  order This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_LSB
  *         @arg @ref LL_XQSPI_QSPI_MSB
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_data_order(xqspi_regs_t *XQSPIx, uint32_t order)
{
    MODIFY_REG(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_MSB1ST, order);
}

/**
  * @brief  Get serial data order
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | MSB1ST                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_LSB
  *         @arg @ref LL_XQSPI_QSPI_MSB
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_data_order(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_MSB1ST));
}

/**
  * @brief  Enable continuous transfer mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CONTXFER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi_contxfer(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CONTXFER);
}

/**
  * @brief  Disable continuous transfer mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CONTXFER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi_contxfer(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CONTXFER);
}

/**
  * @brief  Check if continuous transfer mode is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | CONTXFER                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_qspi_contxfer(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_CONTXFER) == (XQSPI_QSPI_CTRL_CONTXFER));
}

/**
  * @brief  Enable continuous transfer extend mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | CONTXFERX                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi_contxfer_extend(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_CONTXFERX);
}

/**
  * @brief  Disable continuous transfer extend mode
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | CONTXFERX                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi_contxfer_extend(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_CONTXFERX);
}

/**
  * @brief  Check if continuous transfer extend mode is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | CONTXFERX                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_qspi_contxfer_extend(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_CONTXFERX) == (XQSPI_QSPI_AUXCTRL_CONTXFERX));
}

/**
  * @brief  Set data size
  * @note   These bits should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | BITSIZE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  szie This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_4BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_8BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_12BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_16BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_20BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_24BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_28BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_32BIT
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_datasize(xqspi_regs_t *XQSPIx, uint32_t szie)
{
    MODIFY_REG(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_BITSIZE, szie);
}

/**
  * @brief  Get data size
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | BITSIZE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_4BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_8BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_12BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_16BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_20BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_24BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_28BIT
  *         @arg @ref LL_XQSPI_QSPI_DATASIZE_32BIT
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_datasize(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_BITSIZE));
}

/**
  * @brief  Enable inhibt data input to RX FIFO
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | INHIBITDIN                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_inhibt_rx(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_INHIBITDIN);
}

/**
  * @brief  Disable inhibt data input to RX FIFO
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | INHIBITDIN                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_inhibt_rx(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_INHIBITDIN);
}

/**
  * @brief  Check if inhibt data input to RX FIFO is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | INHIBITDIN                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_inhibt_rx(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_INHIBITDIN) == XQSPI_QSPI_AUXCTRL_INHIBITDIN);
}

/**
  * @brief  Enable inhibt data output to TX FIFO
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | INHIBITDOUT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_inhibt_tx(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_INHIBITDOUT);
}

/**
  * @brief  Disable inhibt data output to TX FIFO
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | INHIBITDOUT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_inhibt_tx(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_INHIBITDOUT);
}

/**
  * @brief  Check if inhibt data input to TX FIFO is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | INHIBITDOUT                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_inhibt_tx(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_INHIBITDOUT) == XQSPI_QSPI_AUXCTRL_INHIBITDOUT);
}

/**
  * @brief  Set frame format
  * @note   These bits should not be changed when communication is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | QMODE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  format This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_FRF_SPI
  *         @arg @ref LL_XQSPI_QSPI_FRF_DUALSPI
  *         @arg @ref LL_XQSPI_QSPI_FRF_QUADSPI
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_frf(xqspi_regs_t *XQSPIx, uint32_t format)
{
    MODIFY_REG(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_QMODE, format);
}

/**
  * @brief  Get frame format
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AUX_CTRL             | QMODE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one even value:
  *         @arg @ref LL_XQSPI_QSPI_FRF_SPI
  *         @arg @ref LL_XQSPI_QSPI_FRF_DUALSPI
  *         @arg @ref LL_XQSPI_QSPI_FRF_QUADSPI
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_frf(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_QMODE));
}

/**
  * @brief  Get QSPI status
  *
  *  \rst
  *  +----------------------+------------------------------------------------------+
  *  | Register             | BitsName                                             |
  *  +======================+======================================================+
  *  | STATUS               | RXFULL RXWMARK RXEMPTY TXFULL TXWMARK TXEMPTY XFERIP |
  *  +----------------------+------------------------------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one or combination of the following values:
  *         @arg @ref LL_XQSPI_QSPI_STAT_RFTF
  *         @arg @ref LL_XQSPI_QSPI_STAT_RFF
  *         @arg @ref LL_XQSPI_QSPI_STAT_RFE
  *         @arg @ref LL_XQSPI_QSPI_STAT_TFTF
  *         @arg @ref LL_XQSPI_QSPI_STAT_TFF
  *         @arg @ref LL_XQSPI_QSPI_STAT_TFE
  *         @arg @ref LL_XQSPI_QSPI_STAT_BUSY
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_status(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_REG(XQSPIx->QSPI.STAT));
}

/**
  * @brief  Check active flag
  *
  *  \rst
  *  +----------------------+------------------------------------------------------+
  *  | Register             | BitsName                                             |
  *  +======================+======================================================+
  *  | STATUS               | RXFULL RXWMARK RXEMPTY TXFULL TXWMARK TXEMPTY XFERIP |
  *  +----------------------+------------------------------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_STAT_RFTF
  *         @arg @ref LL_XQSPI_QSPI_STAT_RFF
  *         @arg @ref LL_XQSPI_QSPI_STAT_RFE
  *         @arg @ref LL_XQSPI_QSPI_STAT_TFTF
  *         @arg @ref LL_XQSPI_QSPI_STAT_TFF
  *         @arg @ref LL_XQSPI_QSPI_STAT_TFE
  *         @arg @ref LL_XQSPI_QSPI_STAT_BUSY
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_active_qspi_flag(xqspi_regs_t *XQSPIx, uint32_t flag)
{
    return (READ_BITS(XQSPIx->QSPI.STAT, flag) == (flag));
}

/**
  * @brief  Enable slave select output
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLAVE_SEL            | OUT3 OUT2 OUT1 OUT0               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  ssout This parameter can between: 0 ~ 0xFF
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi_ssout(xqspi_regs_t *XQSPIx, uint32_t ssout)
{
    SET_BITS(XQSPIx->QSPI.SLAVE_SEL, ssout);
}

/**
  * @brief  Disable slave select output
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLAVE_SEL            | OUT3 OUT2 OUT1 OUT0               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  ssout This parameter can between: 0 ~ 0xFF
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi_ssout(xqspi_regs_t *XQSPIx, uint32_t ssout)
{
    CLEAR_BITS(XQSPIx->QSPI.SLAVE_SEL, ssout);
}

/**
  * @brief  Set slave select output polarity
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLAVE_SEL_POL        | POL3 POL2 POL1 POL0               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  sspol This parameter can between: 0 ~ 0xFF
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_sspol(xqspi_regs_t *XQSPIx, uint32_t sspol)
{
    SET_BITS(XQSPIx->QSPI.SLAVE_SEL_POL, sspol);
}

/**
  * @brief  Get slave select output polarity
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLAVE_SEL_POL        | POL3 POL2 POL1 POL0               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 0xFF
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_sspol(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_REG(XQSPIx->QSPI.SLAVE_SEL_POL));
}

/**
  * @brief  Get FIFO Transmission Level
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TX_FIFO_LVL          | TXFIFOLVL                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 16
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_tx_fifo_level(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.TX_FIFO_LVL, XQSPI_QSPI_TXFIFOLVL));
}

/**
  * @brief  Get FIFO reception Level
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RX_FIFO_LVL          | RXFIFOLVL                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 16
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_rx_fifo_level(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.RX_FIFO_LVL, XQSPI_QSPI_RXFIFOLVL));
}

/**
  * @brief  Enable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | INT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_IM_DONE
  *         @arg @ref LL_XQSPI_QSPI_IM_RFF
  *         @arg @ref LL_XQSPI_QSPI_IM_RFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFE
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi_it(xqspi_regs_t *XQSPIx, uint32_t mask)
{
    SET_BITS(XQSPIx->QSPI.INTEN, mask);
}

/**
  * @brief  Disable interrupt
  * @note   This bit controls the generation of an interrupt when an event occurs.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | INT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_IM_DONE
  *         @arg @ref LL_XQSPI_QSPI_IM_RFF
  *         @arg @ref LL_XQSPI_QSPI_IM_RFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFE
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi_it(xqspi_regs_t *XQSPIx, uint32_t mask)
{
    CLEAR_BITS(XQSPIx->QSPI.INTEN, mask);
}

/**
  * @brief  Check if interrupt is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | INT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  mask This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_IM_DONE
  *         @arg @ref LL_XQSPI_QSPI_IM_RFF
  *         @arg @ref LL_XQSPI_QSPI_IM_RFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFE
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_qspi_it(xqspi_regs_t *XQSPIx, uint32_t mask)
{
    return (READ_BITS(XQSPIx->QSPI.INTEN, mask) == (mask));
}

/**
  * @brief  Get XQSPI interrupt flags
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | INT_STAT                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one or combination of the following values:
  *         @arg @ref LL_XQSPI_QSPI_IS_DONE
  *         @arg @ref LL_XQSPI_QSPI_IS_RFF
  *         @arg @ref LL_XQSPI_QSPI_IS_RFTF
  *         @arg @ref LL_XQSPI_QSPI_IS_TFTF
  *         @arg @ref LL_XQSPI_QSPI_IS_TFE
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_it_flag(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_REG(XQSPIx->QSPI.INTSTAT));
}

/**
  * @brief  Check interrupt flag
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | XFER_DPULSE                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  INTSTAT | RX_FPULSE
  *  INTSTAT | RX_WPULSE
  *  INTSTAT | TX_WPULSE
  *  INTSTAT | TX_EPULSE
  *
  * @param  XQSPIx XQSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_IS_DONE
  *         @arg @ref LL_XQSPI_QSPI_IS_RFF
  *         @arg @ref LL_XQSPI_QSPI_IS_RFTF
  *         @arg @ref LL_XQSPI_QSPI_IS_TFTF
  *         @arg @ref LL_XQSPI_QSPI_IS_TFE
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_qspi_it_flag(xqspi_regs_t *XQSPIx, uint32_t flag)
{
    return (READ_BITS(XQSPIx->QSPI.INTSTAT, flag) == (flag));
}

/**
  * @brief  Clear interrupt flag
  * @note   Clearing interrupt flag is done by writting INTCLR register
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTCLR               | INT_CLR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  flag This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_QSPI_IM_DONE
  *         @arg @ref LL_XQSPI_QSPI_IM_RFF
  *         @arg @ref LL_XQSPI_QSPI_IM_RFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFTF
  *         @arg @ref LL_XQSPI_QSPI_IM_TFE
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_clear_qspi_flag(xqspi_regs_t *XQSPIx, uint32_t flag)
{
    WRITE_REG(XQSPIx->QSPI.INTCLR, flag);
}

/**
  * @brief  Set master inter-transfer delay
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSTR_IT_DELAY        | MWAIT                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  wait This parameter can between: 0 ~ 255
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_wait(xqspi_regs_t *XQSPIx, uint32_t wait)
{
    MODIFY_REG(XQSPIx->QSPI.MSTR_IT_DELAY, XQSPI_QSPI_MWAIT_MWAIT, wait << XQSPI_QSPI_MWAIT_MWAIT_Pos);
}

/**
  * @brief  Get master inter-transfer delay
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSTR_IT_DELAY        | MWAIT                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can between: 0 ~ 255
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_wait(xqspi_regs_t *XQSPIx)
{
    return (uint32_t)(READ_BITS(XQSPIx->QSPI.MSTR_IT_DELAY, XQSPI_QSPI_MWAIT_MWAIT) >> XQSPI_QSPI_MWAIT_MWAIT_Pos);
}

/**
  * @brief  Enable QSPI
  * @note   This bit should not be enable when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SPIEN                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_qspi(xqspi_regs_t *XQSPIx)
{
    SET_BITS(XQSPIx->QSPI.SPIEN, XQSPI_QSPI_EN_EN);
}

/**
  * @brief  Disable QSPI
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SPIEN                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_qspi(xqspi_regs_t *XQSPIx)
{
    CLEAR_BITS(XQSPIx->QSPI.SPIEN, XQSPI_QSPI_EN_EN);
}

/**
  * @brief  Check if QSPI is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SPIEN                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enabled_qspi(xqspi_regs_t *XQSPIx)
{
    return (READ_BITS(XQSPIx->QSPI.SPIEN, XQSPI_QSPI_EN_EN) == (XQSPI_QSPI_EN_EN));
}

/**
  * @brief  Set QSPI Flash write bits
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | FLASH_WRITE          | FLASH_WRITE                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  bits This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_FLASH_WRITE_128BIT
  *         @arg @ref LL_XQSPI_FLASH_WRITE_32BIT
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_flash_write(xqspi_regs_t *XQSPIx, uint32_t bits)
{
    WRITE_REG(XQSPIx->QSPI.FLASH_WRITE, bits);
}

/**
  * @brief  Get QSPI Flash write bits
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | FLASH_WRITE          | FLASH_WRITE                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_FLASH_WRITE_128BIT
  *         @arg @ref LL_XQSPI_FLASH_WRITE_32BIT
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_flash_write(xqspi_regs_t *XQSPIx)
{
    //GR5515_C and future version.
    return READ_REG(XQSPIx->QSPI.FLASH_WRITE);
}

/**
  * @brief  Set QSPI Present Bypass
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | BYPASS               | BYPASS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @param  bypass This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_ENABLE_PRESENT
  *         @arg @ref LL_XQSPI_DISABLE_PRESENT
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_present_bypass(xqspi_regs_t *XQSPIx, uint32_t bypass)
{
    WRITE_REG(XQSPIx->QSPI.BYPASS, bypass);
}

/**
  * @brief  Get QSPI Present Bypass
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | BYPASS               | BYPASS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  XQSPIx XQSPI instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_ENABLE_PRESENT
  *         @arg @ref LL_XQSPI_DISABLE_PRESENT
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_present_bypass(xqspi_regs_t *XQSPIx)
{
    return READ_REG(XQSPIx->QSPI.BYPASS);
}

/**
  * @brief  Enable exflash power
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | EFLASH_PAD_EN                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_exflash_power(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_EFLASH_PAD_EN);
}

/**
  * @brief  Disable exflash power
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | EFLASH_PAD_EN                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_exflash_power(void)
{
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_EFLASH_PAD_EN);
}

/**
  * @brief  Check if exflash power is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | EFLASH_PAD_EN                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enable_exflash_power(void)
{
    return (READ_BITS(AON->PWR_RET01, AON_PWR_REG01_EFLASH_PAD_EN) == (AON_PWR_REG01_EFLASH_PAD_EN));
}

/**
  * @brief  Set XQSPI serial clock
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | XF_SCK_CLK_SEL                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  speed This parameter can be one of the following values:
  *         @arg @ref LL_XQSPI_BAUD_RATE_64M
  *         @arg @ref LL_XQSPI_BAUD_RATE_48M
  *         @arg @ref LL_XQSPI_BAUD_RATE_32M
  *         @arg @ref LL_XQSPI_BAUD_RATE_24M
  *         @arg @ref LL_XQSPI_BAUD_RATE_16M
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_set_qspi_speed(uint32_t speed)
{
    MODIFY_REG(AON->PWR_RET01, AON_PWR_REG01_XF_SCK_CLK_SEL, speed);
}

/**
  * @brief  Get XQSPI serial clock
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | XF_SCK_CLK_SEL                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_XQSPI_BAUD_RATE_64M
  *         @arg @ref LL_XQSPI_BAUD_RATE_48M
  *         @arg @ref LL_XQSPI_BAUD_RATE_32M
  *         @arg @ref LL_XQSPI_BAUD_RATE_24M
  *         @arg @ref LL_XQSPI_BAUD_RATE_16M
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_get_qspi_speed(void)
{
    return (uint32_t)(READ_BITS(AON->PWR_RET01, AON_PWR_REG01_XF_SCK_CLK_SEL));
}

/**
  * @brief  Enable cache data retention.
  * @note   This bit should not be changed when XIP is ongoing..
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | XF_TAG_RET                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_enable_cache_retention(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_XF_TAG_RET);
}

/**
  * @brief  Disable cache data retention.
  * @note   This bit should not be changed when XIP is ongoing.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | XF_TAG_RET                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_xqspi_disable_cache_retention(void)
{
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_XF_TAG_RET);
}

/**
  * @brief  Check if tag memory retention is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | XF_TAG_RET                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_xqspi_is_enable_cache_retention(void)
{
    return (READ_BITS(AON->PWR_RET01, AON_PWR_REG01_XF_TAG_RET) == (AON_PWR_REG01_XF_TAG_RET));
}



/** @} */

/** @defgroup XQSPI_LL_Init XQSPI Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize XQSPI registers (Registers restored to their default values).
  * @param  XQSPIx XQSPI instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: XQSPI registers are de-initialized
  *          - ERROR: XQSPI registers are not de-initialized
  */
error_status_t ll_xqspi_deinit(xqspi_regs_t *XQSPIx);

/**
  * @brief  Initialize XQSPI registers according to the specified
  *         parameters in default.
  * @param  XQSPIx XQSPI instance
  * @param  p_xqspi_init Pointer to a ll_xqspi_init_t structure that contains the configuration
  *                         information for the specified XQPSI peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: XQSPI registers are initialized according to default
  *          - ERROR: Problem occurred during XQSPI Registers initialization
  */
error_status_t ll_xqspi_init(xqspi_regs_t *XQSPIx, ll_xqspi_init_t *p_xqspi_init);

/**
  * @brief Set each field of a @ref ll_xqspi_init_t type structure to default value.
  * @param p_xqspi_init  Pointer to a @ref ll_xqspi_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_xqspi_struct_init(ll_xqspi_init_t *p_xqspi_init);

/** @} */

/** @} */

#endif /* XQSPI */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_XQSPI_H__ */

/** @} */

/** @} */

/** @} */
