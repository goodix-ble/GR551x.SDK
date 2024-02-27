/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DMA LL library.
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

/** @defgroup LL_DMA DMA
  * @brief DMA LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_DMA_H__
#define __GR55xx_LL_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (DMA)

/** @defgroup DMA_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup DMA_LL_ES_INIT DMA Exported init structures
  * @{
  */

/**
  * @brief LL DMA init Structure definition
  */
typedef struct _ll_dma_init
{
    uint32_t src_address;            /**< Specifies the Source base address for DMA transfer.

                                         This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

    uint32_t dst_address;            /**< Specifies the Destination base address for DMA transfer.

                                         This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

    uint32_t direction;             /**< Specifies if the data will be transferred from memory to peripheral,
                                         from memory to memory or from peripheral to memory or form peripheral to peripheral.
                                         This parameter can be a value of @ref DMA_LL_EC_DIRECTION

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_data_transfer_direction(). */

    uint32_t  mode;                 /**< Specifies the Single block or Multi-block operation mode.
                                         This parameter can be a value of @ref DMA_LL_EC_MODE
                                         @note: The circular buffer mode cannot be used if the memory to memory
                                                data transfer direction is configured on the selected Channel

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_mode(). */

    uint32_t src_increment_mode;    /**< Specifies whether the Source address is incremented or decrement or not.
                                         This parameter can be a value of @ref DMA_LL_EC_SOURCE

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_source_increment_mode(). */

    uint32_t dst_increment_mode;    /**< Specifies whether the Destination address is incremented or decrement or not.
                                         This parameter can be a value of @ref DMA_LL_EC_DESTINATION

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_destination_increment_mode(). */

    uint32_t src_data_width;        /**< Specifies the Souce transfer width alignment(byte, half word, word).
                                         This parameter can be a value of @ref DMA_LL_EC_SDATAALIGN

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_source_width(). */

    uint32_t dst_data_width;        /**< Specifies the Destination transfer width alignment(byte, half word, word).
                                         This parameter can be a value of @ref DMA_LL_EC_DDATAALIGN

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_destination_width(). */

    uint32_t block_size;            /**< Specifies the number of data to transfer, in data unit.
                                         The data unit is equal to the source buffer configuration set in src_data_width parameters.
                                         This parameter must be a value between Min_Data = 0 and Max_Data = 0x1FF

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_block_size(). */

    uint32_t src_peripheral;        /**< Specifies the Source peripheral type.
                                         This parameter can be a value of @ref DMA_LL_EC_PERIPH

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_source_peripheral(). */

    uint32_t dst_peripheral;        /**< Specifies the Destination peripheral type.
                                         This parameter can be a value of @ref DMA_LL_EC_PERIPH

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_destination_peripheral(). */

    uint32_t priority;              /**< Specifies the channel priority level.
                                         This parameter can be a value of @ref DMA_LL_EC_PRIORITY

                                         This feature can be modified afterwards using unitary function @ref ll_dma_set_channel_priority_level(). */

} ll_dma_init_t;

/** @} */

/** @} */

/**
  * @defgroup  DMA_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DMA_LL_Exported_Constants DMA Exported Constants
  * @{
  */

/** @defgroup DMA_LL_EC_CHANNEL CHANNEL
  * @{
  */
#define LL_DMA_CHANNEL_0                      ((uint32_t)0x00000000U) /**< DMA Channel 0 */
#define LL_DMA_CHANNEL_1                      ((uint32_t)0x00000001U) /**< DMA Channel 1 */
#define LL_DMA_CHANNEL_2                      ((uint32_t)0x00000002U) /**< DMA Channel 2 */
#define LL_DMA_CHANNEL_3                      ((uint32_t)0x00000003U) /**< DMA Channel 3 */
#define LL_DMA_CHANNEL_4                      ((uint32_t)0x00000004U) /**< DMA Channel 4 */
#define LL_DMA_CHANNEL_5                      ((uint32_t)0x00000005U) /**< DMA Channel 5 */
#define LL_DMA_CHANNEL_6                      ((uint32_t)0x00000006U) /**< DMA Channel 6 */
#define LL_DMA_CHANNEL_7                      ((uint32_t)0x00000007U) /**< DMA Channel 7 */
#define LL_DMA_CHANNEL_ALL                    ((uint32_t)0xFFFF0000U) /**< DMA Channel all (used only for function @ref ll_dma_deinit(). */
/** @} */

/** @defgroup DMA_LL_EC_DIRECTION Transfer Direction
  * @{
  */
#define LL_DMA_DIRECTION_MEMORY_TO_MEMORY     DMA_CTLL_TT_FC_M2M  /**< Memory to memory direction     */
#define LL_DMA_DIRECTION_MEMORY_TO_PERIPH     DMA_CTLL_TT_FC_M2P  /**< Memory to peripheral direction */
#define LL_DMA_DIRECTION_PERIPH_TO_MEMORY     DMA_CTLL_TT_FC_P2M  /**< Peripheral to memory direction */
#define LL_DMA_DIRECTION_PERIPH_TO_PERIPH     DMA_CTLL_TT_FC_P2P  /**< Peripheral to Peripheral direction */
/** @} */


/** @defgroup DMA_LL_EC_MODE Transfer mode
  * @{
  */
#define LL_DMA_MODE_SINGLE_BLOCK              ((uint32_t)0x00000000U)                      /**< Single block */
#define LL_DMA_MODE_MULTI_BLOCK_SRC_RELOAD    DMA_CFGL_RELOAD_SRC                          /**< Multi-block: src address reload, dst address contiguous */
#define LL_DMA_MODE_MULTI_BLOCK_DST_RELOAD    DMA_CFGL_RELOAD_DST                          /**< Multi-block: src address contiguous, dst address reload */
#define LL_DMA_MODE_MULTI_BLOCK_ALL_RELOAD    (DMA_CFGL_RELOAD_SRC | DMA_CFGL_RELOAD_DST)  /**< Multi-block: src address reload, dst address reload */
/** @} */

/** @defgroup DMA_LL_EC_SOURCE Source increment mode
  * @{
  */
#define LL_DMA_SRC_INCREMENT          DMA_CTLL_SINC_INC    /**< Source Address increment */
#define LL_DMA_SRC_DECREMENT          DMA_CTLL_SINC_DEC    /**< Source Address decrement */
#define LL_DMA_SRC_NO_CHANGE          DMA_CTLL_SINC_NO     /**< Source Address no change */
/** @} */

/** @defgroup DMA_LL_EC_DESTINATION Destination increment mode
  * @{
  */
#define LL_DMA_DST_INCREMENT          DMA_CTLL_DINC_INC   /**< Destination Address increment */
#define LL_DMA_DST_DECREMENT          DMA_CTLL_DINC_DEC   /**< Destination Address decrement */
#define LL_DMA_DST_NO_CHANGE          DMA_CTLL_DINC_NO    /**< Destination Address no change */
/** @} */

/** @defgroup DMA_LL_EC_SRC_BURST Source burst transaction length
  * @{
  */
#define LL_DMA_SRC_BURST_LENGTH_1     DMA_CTLL_SRC_MSIZE_1   /**< Source Burst length: 1 word */
#define LL_DMA_SRC_BURST_LENGTH_4     DMA_CTLL_SRC_MSIZE_4   /**< Source Burst length: 4 words */
#define LL_DMA_SRC_BURST_LENGTH_8     DMA_CTLL_SRC_MSIZE_8   /**< Source Burst length: 8 words */
#define LL_DMA_SRC_BURST_LENGTH_16    DMA_CTLL_SRC_MSIZE_16  /**< Source Burst length: 16 words */
#define LL_DMA_SRC_BURST_LENGTH_32    DMA_CTLL_SRC_MSIZE_32  /**< Source Burst length: 32 words */
#define LL_DMA_SRC_BURST_LENGTH_64    DMA_CTLL_SRC_MSIZE_64  /**< Source Burst length: 64 words */
/** @} */

/** @defgroup DMA_LL_EC_DST_BURST Destination burst transaction length
  * @{
  */
#define LL_DMA_DST_BURST_LENGTH_1     DMA_CTLL_DST_MSIZE_1   /**< Destination Burst length: 1 word */
#define LL_DMA_DST_BURST_LENGTH_4     DMA_CTLL_DST_MSIZE_4   /**< Destination Burst length: 4 words */
#define LL_DMA_DST_BURST_LENGTH_8     DMA_CTLL_DST_MSIZE_8   /**< Destination Burst length: 8 words */
#define LL_DMA_DST_BURST_LENGTH_16    DMA_CTLL_DST_MSIZE_16  /**< Destination Burst length: 16 words */
#define LL_DMA_DST_BURST_LENGTH_32    DMA_CTLL_DST_MSIZE_32  /**< Destination Burst length: 32 words */
#define LL_DMA_DST_BURST_LENGTH_64    DMA_CTLL_DST_MSIZE_64  /**< Destination Burst length: 64 words */
/** @} */

/** @defgroup DMA_LL_EC_SDATAALIGN Source data alignment
  * @{
  */
#define LL_DMA_SDATAALIGN_BYTE        DMA_CTLL_SRC_TR_WIDTH_8    /**< Source data alignment : Byte     */
#define LL_DMA_SDATAALIGN_HALFWORD    DMA_CTLL_SRC_TR_WIDTH_16   /**< Source data alignment : HalfWord */
#define LL_DMA_SDATAALIGN_WORD        DMA_CTLL_SRC_TR_WIDTH_32   /**< Source data alignment : Word     */
/** @} */

/** @defgroup DMA_LL_EC_DDATAALIGN Destination data alignment
  * @{
  */
#define LL_DMA_DDATAALIGN_BYTE        DMA_CTLL_DST_TR_WIDTH_8    /**< Destination data alignment : Byte     */
#define LL_DMA_DDATAALIGN_HALFWORD    DMA_CTLL_DST_TR_WIDTH_16   /**< Destination data alignment : HalfWord */
#define LL_DMA_DDATAALIGN_WORD        DMA_CTLL_DST_TR_WIDTH_32   /**< Destination data alignment : Word     */
/** @} */

/** @defgroup DMA_LL_EC_PRIORITY Transfer Priority level
  * @{
  */
#define LL_DMA_PRIORITY_0             DMA_CFGL_CH_PRIOR_0    /**< Priority level : 0 */
#define LL_DMA_PRIORITY_1             DMA_CFGL_CH_PRIOR_1    /**< Priority level : 1 */
#define LL_DMA_PRIORITY_2             DMA_CFGL_CH_PRIOR_2    /**< Priority level : 2 */
#define LL_DMA_PRIORITY_3             DMA_CFGL_CH_PRIOR_3    /**< Priority level : 3 */
#define LL_DMA_PRIORITY_4             DMA_CFGL_CH_PRIOR_4    /**< Priority level : 4 */
#define LL_DMA_PRIORITY_5             DMA_CFGL_CH_PRIOR_5    /**< Priority level : 5 */
#define LL_DMA_PRIORITY_6             DMA_CFGL_CH_PRIOR_6    /**< Priority level : 6 */
#define LL_DMA_PRIORITY_7             DMA_CFGL_CH_PRIOR_7    /**< Priority level : 7 */
/** @} */

/** @defgroup DMA_LL_EC_SHANDSHAKING Source handshake interface
  * @{
  */
#define LL_DMA_SHANDSHAKING_HW        ((uint32_t)0x00000000U) /**< Source: hardware handshake */
#define LL_DMA_SHANDSHAKING_SW        DMA_CFGL_HS_SEL_SRC     /**< Source: software handshake */
/** @} */

/** @defgroup DMA_LL_EC_DHANDSHAKING Destination handshake interface
  * @{
  */
#define LL_DMA_DHANDSHAKING_HW        ((uint32_t)0x00000000U) /**< Destination: hardware handshake */
#define LL_DMA_DHANDSHAKING_SW        DMA_CFGL_HS_SEL_DST     /**< Destination: software handshake */
/** @} */

/** @defgroup DMA_LL_EC_PERIPH DMA Peripheral type
  * @{
  */
#define LL_DMA_PERIPH_SPIM_TX         ((uint32_t)0x00000000U) /**< DMA Peripheral type is SPIM TX   */
#define LL_DMA_PERIPH_SPIM_RX         ((uint32_t)0x00000001U) /**< DMA Peripheral type is SPIM RX   */
#define LL_DMA_PERIPH_SPIS_TX         ((uint32_t)0x00000002U) /**< DMA Peripheral type is SPIS TX   */
#define LL_DMA_PERIPH_SPIS_RX         ((uint32_t)0x00000003U) /**< DMA Peripheral type is SPIS RX   */
#define LL_DMA_PERIPH_QSPI0_TX        ((uint32_t)0x00000004U) /**< DMA Peripheral type is QSPI0 TX  */
#define LL_DMA_PERIPH_QSPI0_RX        ((uint32_t)0x00000005U) /**< DMA Peripheral type is QSPI0 RX  */
#define LL_DMA_PERIPH_I2C0_TX         ((uint32_t)0x00000006U) /**< DMA Peripheral type is I2C0 TX   */
#define LL_DMA_PERIPH_I2C0_RX         ((uint32_t)0x00000007U) /**< DMA Peripheral type is I2C0 RX   */
#define LL_DMA_PERIPH_I2C1_TX         ((uint32_t)0x00000008U) /**< DMA Peripheral type is I2C1 TX   */
#define LL_DMA_PERIPH_I2C1_RX         ((uint32_t)0x00000009U) /**< DMA Peripheral type is I2C1 RX   */
#define LL_DMA_PERIPH_I2S_S_TX        ((uint32_t)0x00000008U) /**< DMA Peripheral type is I2S_S TX  */
#define LL_DMA_PERIPH_I2S_S_RX        ((uint32_t)0x00000009U) /**< DMA Peripheral type is I2S_S RX  */
#define LL_DMA_PERIPH_UART0_TX        ((uint32_t)0x0000000AU) /**< DMA Peripheral type is UART0 TX  */
#define LL_DMA_PERIPH_UART0_RX        ((uint32_t)0x0000000BU) /**< DMA Peripheral type is UART0 RX  */
#define LL_DMA_PERIPH_QSPI1_TX        ((uint32_t)0x0000000CU) /**< DMA peripheral type is QSPI1 TX  */
#define LL_DMA_PERIPH_QSPI1_RX        ((uint32_t)0x0000000DU) /**< DMA peripheral type is QSPI1 RX  */
#define LL_DMA_PERIPH_I2S_M_TX        ((uint32_t)0x0000000CU) /**< DMA Peripheral type is I2S_M TX  */
#define LL_DMA_PERIPH_I2S_M_RX        ((uint32_t)0x0000000DU) /**< DMA Peripheral type is I2S_M RX  */
#define LL_DMA_PERIPH_SNSADC          ((uint32_t)0x0000000EU) /**< DMA peripheral type is SNSADC    */
#define LL_DMA_PERIPH_MEM             ((uint32_t)0x0000000FU) /**< DMA peripheral type is Memory    */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DMA_LL_Exported_Macros DMA Exported Macros
  * @{
  */

/** @defgroup DMA_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in DMA register
  * @param  __instance__ DMA instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_DMA_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__.__REG__, (__VALUE__))

/**
  * @brief  Read a value in DMA register
  * @param  __instance__ DMA instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_DMA_ReadReg(__instance__, __REG__) READ_REG(__instance__.__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup DMA_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup DMA_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable DMA Module.
  * @note This function is used to enable the DMA Module, which must be done before any
  *       channel activity can begin.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_REG              | CFG_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->MISCELLANEOU.CFG, DMA_MODULE_CFG_EN);
}

/**
  * @brief  Disable DMA Module.
  * @note If the ll_dma_disable() function is called while any dma channel is still active,
  *       the ll_dma_is_enable() function still return 1 to indicate that there are channels
  *       still active until hardware has terminated all cativity on all channels, at which
  *       point the ll_dma_is_enable() function returns 0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_REG              | CFG_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->MISCELLANEOU.CFG, 0);
}

/**
  * @brief  Check if DMA Module is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_REG              | CFG_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enable(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->MISCELLANEOU.CFG, DMA_MODULE_CFG_EN) == DMA_MODULE_CFG_EN);
}

/**
  * @brief  Enable DMA channel.
  * @note When the DMA Module is disabled, then call this function to DMA_CFG_REG register
  *       is ignored and call ll_dma_disable_channel() function will always returns 0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CH_EN_REG            | CH_EN_WE&CH_EN                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_channel(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->MISCELLANEOU.CH_EN, (1 << (channel + DMA_CH_WE_EN_Pos)) + (1 << channel));
}

/**
  * @brief  Disable DMA channel.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CH_EN_REG            | CH_EN_WE&CH_EN                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_channel(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->MISCELLANEOU.CH_EN, (1 << (channel + DMA_CH_WE_EN_Pos)));
}

/**
  * @brief  Check if DMA channel is enabled or disabled.
  * @note Software can therefore poll this function to determine when channel is free
  *       for a new DMA transfer.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CH_EN_REG            | CH_EN_WE&CH_EN                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enabled_channel(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->MISCELLANEOU.CH_EN, (1 << channel)) ? 1 : 0;
}

/**
  * @brief  Suspend a DMA channel transfer.
  * @note   Suspends all DMA data transfers from the source until the ll_dma_resume_channel()
  *         function is called. The function may be called after enabling the DMA channel.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFGL                 | CH_SUSP                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_suspend_channel(dma_regs_t *DMAx, uint32_t channel)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_CH_SUSP, DMA_CFGL_CH_SUSP);
}

/**
  * @brief  Resume a DMA channel.
  * @note The function may be called after enabling the DMA channel.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFGL                 | CH_SUSP                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_resume_channel(dma_regs_t *DMAx, uint32_t channel)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_CH_SUSP, 0);
}

/**
  * @brief  Check if DMA channel is suspended or resumed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFGL                 | CH_SUSP                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_suspended(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_CH_SUSP) == DMA_CFGL_CH_SUSP);
}

/**
  * @brief  Check if DMA channel FIFO is empty.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFGL                 | FIFO_EMPTY                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_empty_fifo(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_FIFO_EMPTY) == DMA_CFGL_FIFO_EMPTY);
}

/**
  * @brief  Configure all parameters link to DMA transfer.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CCR                  | DIR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  CCR | MEM2MEM
  *  CCR | CIRC
  *  CCR | PINC
  *  CCR | MINC
  *  CCR | PSIZE
  *  CCR | MSIZE
  *  CCR | PL
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_DMA_MODE_SINGLE_BLOCK or @ref LL_DMA_MODE_MULTI_BLOCK_SRC_RELOAD or @ref LL_DMA_MODE_MULTI_BLOCK_DST_RELOAD or @ref LL_DMA_MODE_MULTI_BLOCK_ALL_RELOAD
  *         @arg @ref LL_DMA_SRC_INCREMENT or @ref LL_DMA_SRC_DECREMENT or @ref LL_DMA_SRC_NO_CHANGE
  *         @arg @ref LL_DMA_DST_INCREMENT or @ref LL_DMA_DST_DECREMENT or @ref LL_DMA_DST_NO_CHANGE
  *         @arg @ref LL_DMA_SDATAALIGN_BYTE or @ref LL_DMA_SDATAALIGN_HALFWORD or @ref LL_DMA_SDATAALIGN_WORD
  *         @arg @ref LL_DMA_DDATAALIGN_BYTE or @ref LL_DMA_DDATAALIGN_HALFWORD or @ref LL_DMA_DDATAALIGN_WORD
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_1 or @ref LL_DMA_SRC_BURST_LENGTH_4 or @ref LL_DMA_SRC_BURST_LENGTH_8
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_1 or @ref LL_DMA_DST_BURST_LENGTH_4 or @ref LL_DMA_DST_BURST_LENGTH_8
  * @retval None
  */
__STATIC_INLINE void ll_dma_config_transfer(dma_regs_t *DMAx, uint32_t channel, uint32_t configuration)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DST_TR_WIDTH | DMA_CTLL_SRC_TR_WIDTH |\
               DMA_CTLL_DINC | DMA_CTLL_SINC | DMA_CTLL_DST_MSIZE | DMA_CTLL_SRC_MSIZE | DMA_CTLL_TT_FC,
               configuration);
}

/**
  * @brief  Set Data transfer direction (read from peripheral or from memory).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | TT_FC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_PERIPH
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_data_transfer_direction(dma_regs_t *DMAx, uint32_t channel, uint32_t direction)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_TT_FC, direction);
}

/**
  * @brief  Get Data transfer direction (read from peripheral or from memory).
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | TT_FC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_PERIPH
  */
__STATIC_INLINE uint32_t ll_dma_get_data_transfer_direction(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_TT_FC);
}

/**
  * @brief  Set DMA mode Single block or Multi block.
  * @note The circular buffer mode cannot be used if the memory-to-memory
  * data transfer is configured on the selected Channel.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_LO               | RELOAD_DST                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_MODE_SINGLE_BLOCK
  *         @arg @ref LL_DMA_MODE_MULTI_BLOCK_SRC_RELOAD
  *         @arg @ref LL_DMA_MODE_MULTI_BLOCK_DST_RELOAD
  *         @arg @ref LL_DMA_MODE_MULTI_BLOCK_ALL_RELOAD
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_mode(dma_regs_t *DMAx, uint32_t channel, uint32_t mode)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_RELOAD_DST | DMA_CFGL_RELOAD_SRC, mode);
}


/**
  * @brief  Get DMA mode circular or normal.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_LO               | RELOAD_DST                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_MODE_SINGLE_BLOCK
  *         @arg @ref LL_DMA_MODE_MULTI_BLOCK_SRC_RELOAD
  *         @arg @ref LL_DMA_MODE_MULTI_BLOCK_DST_RELOAD
  *         @arg @ref LL_DMA_MODE_MULTI_BLOCK_ALL_RELOAD
  */
__STATIC_INLINE uint32_t ll_dma_get_mode(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_RELOAD_DST | DMA_CFGL_RELOAD_SRC);
}

/**
  * @brief  Set Source increment mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | SINC                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  src_increment_mode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_SRC_INCREMENT
  *         @arg @ref LL_DMA_SRC_DECREMENT
  *         @arg @ref LL_DMA_SRC_NO_CHANGE
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_source_increment_mode(dma_regs_t *DMAx, uint32_t channel, uint32_t src_increment_mode)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_SINC, src_increment_mode);
}

/**
  * @brief  Get Source increment mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | SINC                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_SRC_INCREMENT
  *         @arg @ref LL_DMA_SRC_DECREMENT
  *         @arg @ref LL_DMA_SRC_NO_CHANGE
  */
__STATIC_INLINE uint32_t ll_dma_get_source_increment_mode(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_SINC);
}

/**
  * @brief  Set Destination increment mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | DINC                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  dst_increment_mode This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DST_INCREMENT
  *         @arg @ref LL_DMA_DST_DECREMENT
  *         @arg @ref LL_DMA_DST_NO_CHANGE
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_destination_increment_mode(dma_regs_t *DMAx, uint32_t channel, uint32_t dst_increment_mode)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DINC, dst_increment_mode);
}

/**
  * @brief  Get Destination increment mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | DINC                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_DST_INCREMENT
  *         @arg @ref LL_DMA_DST_DECREMENT
  *         @arg @ref LL_DMA_DST_NO_CHANGE
  */
__STATIC_INLINE uint32_t ll_dma_get_destination_increment_mode(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DINC);
}

/**
  * @brief  Set Source transfer width.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | SRC_TR_WIDTH                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  src_width This parameter can be one of the following values:
  *         @arg @ref LL_DMA_SDATAALIGN_BYTE
  *         @arg @ref LL_DMA_SDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_SDATAALIGN_WORD
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_source_width(dma_regs_t *DMAx, uint32_t channel, uint32_t src_width)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_SRC_TR_WIDTH, src_width);
}

/**
  * @brief  Get Source transfer width.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | SRC_TR_WIDTH                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_SDATAALIGN_BYTE
  *         @arg @ref LL_DMA_SDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_SDATAALIGN_WORD
  */
__STATIC_INLINE uint32_t ll_dma_get_source_width(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_SRC_TR_WIDTH);
}

/**
  * @brief  Set Destination transfer width.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | DST_TR_WIDTH                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  dst_width This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DDATAALIGN_BYTE
  *         @arg @ref LL_DMA_DDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_DDATAALIGN_WORD
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_destination_width(dma_regs_t *DMAx, uint32_t channel, uint32_t dst_width)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DST_TR_WIDTH, dst_width);
}

/**
  * @brief  Get Destination transfer width.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | DST_TR_WIDTH                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_DDATAALIGN_BYTE
  *         @arg @ref LL_DMA_DDATAALIGN_HALFWORD
  *         @arg @ref LL_DMA_DDATAALIGN_WORD
  */
__STATIC_INLINE uint32_t ll_dma_get_destination_width(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DST_TR_WIDTH);
}

/**
  * @brief  Set Source Burst Transaction Length.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | SRC_MSIZE                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  burst_length This parameter can be one of the following values:
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_1
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_4
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_8
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_source_burst_length(dma_regs_t *DMAx, uint32_t channel, uint32_t burst_length)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_SRC_MSIZE, burst_length);
}

/**
  * @brief  Get Burst Transaction Length.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | SRC_MSIZE                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_1
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_4
  *         @arg @ref LL_DMA_SRC_BURST_LENGTH_8
  */
__STATIC_INLINE uint32_t ll_dma_get_source_burst_length(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_SRC_MSIZE);
}

/**
  * @brief  Set Destination Burst Transaction Length.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | DST_MSIZE                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  burst_length This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_1
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_4
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_8
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_destination_burst_length(dma_regs_t *DMAx, uint32_t channel, uint32_t burst_length)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DST_MSIZE, burst_length);
}

/**
  * @brief  Get Destination Burst Transaction Length.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_LO               | DST_MSIZE                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_1
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_4
  *         @arg @ref LL_DMA_DST_BURST_LENGTH_8
  */
__STATIC_INLINE uint32_t ll_dma_get_destination_burst_length(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_DST_MSIZE);
}

/**
  * @brief  Set Channel priority level.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_LO               | CH_PRIOR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  priority This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PRIORITY_0
  *         @arg @ref LL_DMA_PRIORITY_1
  *         @arg @ref LL_DMA_PRIORITY_2
  *         @arg @ref LL_DMA_PRIORITY_3
  *         @arg @ref LL_DMA_PRIORITY_4
  *         @arg @ref LL_DMA_PRIORITY_5
  *         @arg @ref LL_DMA_PRIORITY_6
  *         @arg @ref LL_DMA_PRIORITY_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_channel_priority_level(dma_regs_t *DMAx, uint32_t channel, uint32_t priority)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_CH_PRIOR, priority);
}

/**
  * @brief  Get Channel priority level.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_LO               | CH_PRIOR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PRIORITY_0
  *         @arg @ref LL_DMA_PRIORITY_1
  *         @arg @ref LL_DMA_PRIORITY_2
  *         @arg @ref LL_DMA_PRIORITY_3
  *         @arg @ref LL_DMA_PRIORITY_4
  *         @arg @ref LL_DMA_PRIORITY_5
  *         @arg @ref LL_DMA_PRIORITY_6
  *         @arg @ref LL_DMA_PRIORITY_7
  */
__STATIC_INLINE uint32_t ll_dma_get_channel_priority_level(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_CH_PRIOR);
}

/**
  * @brief  Set the block size of a transfer.
  * @note   This action has no effect if channel is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_HI               | BLOCK_TS                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  block_size Between Min_Data = 0 and Max_Data = 0xFFF
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_block_size(dma_regs_t *DMAx, uint32_t channel, uint32_t block_size)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_HI, DMA_CTLH_BLOCK_TS, block_size);
}

/**
  * @brief  Get the block size of a transfer.
  * @note   Once the channel is enabled, the return value indicate the
  *         remaining bytes to be transmitted.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTL_HI               | BLOCK_TS                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Between Min_Data = 0 and Max_Data = 0xFFF
  */
__STATIC_INLINE uint32_t ll_dma_get_block_size(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_HI, DMA_CTLH_BLOCK_TS);
}

/**
  * @brief  Configure the Source and Destination addresses.
  * @note   Each IP using DMA provides an API to get directly the register adress (LL_PPP_DMA_GetRegAddr)
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SAR                  | SAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  DAR | DAR
  *  CTL_LO | TT_FC
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  src_address Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  * @param  dst_address Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  * @param  direction This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref LL_DMA_DIRECTION_PERIPH_TO_PERIPH
  * @retval None
  */
__STATIC_INLINE void ll_dma_config_address(dma_regs_t *DMAx,
                                           uint32_t    channel,
                                           uint32_t    src_address,
                                           uint32_t    dst_address,
                                           uint32_t    direction)
{
    WRITE_REG(DMAx->CHANNEL[channel].SAR, src_address);
    WRITE_REG(DMAx->CHANNEL[channel].DAR, dst_address);
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_TT_FC, direction);
}

/**
  * @brief  Set the Source address.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SAR                  | SAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  address Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_source_address(dma_regs_t *DMAx, uint32_t channel, uint32_t address)
{
    WRITE_REG(DMAx->CHANNEL[channel].SAR, address);
}

/**
  * @brief  Set the Destination address.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DAR                  | DAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  address Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_destination_address(dma_regs_t *DMAx, uint32_t channel, uint32_t address)
{
    WRITE_REG(DMAx->CHANNEL[channel].DAR, address);
}

/**
  * @brief  Get Source address.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SAR                  | SAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_dma_get_source_address(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_REG(DMAx->CHANNEL[channel].SAR);
}

/**
  * @brief  Get Destination address.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DAR                  | DAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_dma_get_destination_address(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_REG(DMAx->CHANNEL[channel].DAR);
}

/**
  * @brief  Set the Memory to Memory Source address.
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SAR                  | SAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  CTL_LO | TT_FC
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  address Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_m2m_src_address(dma_regs_t *DMAx, uint32_t channel, uint32_t address)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_TT_FC, 0);
    WRITE_REG(DMAx->CHANNEL[channel].SAR, address);
}

/**
  * @brief  Set the Memory to Memory Destination address.
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DAR                  | DAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  CTL_LO | TT_FC
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  address Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_m2m_dst_address(dma_regs_t *DMAx, uint32_t channel, uint32_t address)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_TT_FC, 0);
    WRITE_REG(DMAx->CHANNEL[channel].DAR, address);
}

/**
  * @brief  Get the Memory to Memory Source address.
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SAR                  | SAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_dma_get_m2m_src_address(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_REG(DMAx->CHANNEL[channel].SAR);
}

/**
  * @brief  Get the Memory to Memory Destination address.
  * @note   Interface used for direction LL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | DAR                  | DAR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Between Min_Data = 0 and Max_Data = 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_dma_get_m2m_dst_address(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_REG(DMAx->CHANNEL[channel].DAR);
}

/**
  * @brief  Set source peripheral for DMA instance on Channel x.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_HI               | SRC_PER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  peripheral This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PERIPH_SPIM_TX
  *         @arg @ref LL_DMA_PERIPH_SPIM_RX
  *         @arg @ref LL_DMA_PERIPH_SPIS_TX
  *         @arg @ref LL_DMA_PERIPH_SPIS_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C0_TX
  *         @arg @ref LL_DMA_PERIPH_I2C0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C1_TX
  *         @arg @ref LL_DMA_PERIPH_I2C1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_RX
  *         @arg @ref LL_DMA_PERIPH_UART0_TX
  *         @arg @ref LL_DMA_PERIPH_UART0_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_RX
  *         @arg @ref LL_DMA_PERIPH_SNSADC
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_source_peripheral(dma_regs_t *DMAx, uint32_t channel, uint32_t peripheral)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_HI, DMA_CFGH_SRC_PER, (peripheral << DMA_CFGH_SRC_PER_Pos));
}

/**
  * @brief  Get source peripheral for DMA instance on Channel x.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_HI               | SRC_PER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PERIPH_SPIM_TX
  *         @arg @ref LL_DMA_PERIPH_SPIM_RX
  *         @arg @ref LL_DMA_PERIPH_SPIS_TX
  *         @arg @ref LL_DMA_PERIPH_SPIS_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C0_TX
  *         @arg @ref LL_DMA_PERIPH_I2C0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C1_TX
  *         @arg @ref LL_DMA_PERIPH_I2C1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_RX
  *         @arg @ref LL_DMA_PERIPH_UART0_TX
  *         @arg @ref LL_DMA_PERIPH_UART0_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_RX
  *         @arg @ref LL_DMA_PERIPH_SNSADC
  */
__STATIC_INLINE uint32_t ll_dma_get_source_peripheral(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_HI, DMA_CFGH_SRC_PER) >> DMA_CFGH_SRC_PER_Pos;
}

/**
  * @brief  Set destination peripheral for DMA instance on Channel x.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_HI               | DST_PER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  peripheral This parameter can be one of the following values:
  *         @arg @ref LL_DMA_PERIPH_SPIM_TX
  *         @arg @ref LL_DMA_PERIPH_SPIM_RX
  *         @arg @ref LL_DMA_PERIPH_SPIS_TX
  *         @arg @ref LL_DMA_PERIPH_SPIS_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C0_TX
  *         @arg @ref LL_DMA_PERIPH_I2C0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C1_TX
  *         @arg @ref LL_DMA_PERIPH_I2C1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_RX
  *         @arg @ref LL_DMA_PERIPH_UART0_TX
  *         @arg @ref LL_DMA_PERIPH_UART0_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_RX
  *         @arg @ref LL_DMA_PERIPH_SNSADC
  * @retval None
  */
__STATIC_INLINE void ll_dma_set_destination_peripheral(dma_regs_t *DMAx, uint32_t channel, uint32_t peripheral)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_HI, DMA_CFGH_DST_PER, (peripheral << DMA_CFGH_DST_PER_Pos));
}

/**
  * @brief  Get destination peripheral for DMA instance on Channel x.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_HI               | DST_PER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA_PERIPH_SPIM_TX
  *         @arg @ref LL_DMA_PERIPH_SPIM_RX
  *         @arg @ref LL_DMA_PERIPH_SPIS_TX
  *         @arg @ref LL_DMA_PERIPH_SPIS_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C0_TX
  *         @arg @ref LL_DMA_PERIPH_I2C0_RX
  *         @arg @ref LL_DMA_PERIPH_I2C1_TX
  *         @arg @ref LL_DMA_PERIPH_I2C1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_S_RX
  *         @arg @ref LL_DMA_PERIPH_UART0_TX
  *         @arg @ref LL_DMA_PERIPH_UART0_RX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_TX
  *         @arg @ref LL_DMA_PERIPH_QSPI1_RX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_TX
  *         @arg @ref LL_DMA_PERIPH_I2S_M_RX
  *         @arg @ref LL_DMA_PERIPH_SNSADC
  */
__STATIC_INLINE uint32_t ll_dma_get_destination_peripheral(dma_regs_t *DMAx, uint32_t channel)
{
    return READ_BITS(DMAx->CHANNEL[channel].CTL_HI, DMA_CFGH_DST_PER) >> DMA_CFGH_DST_PER_Pos;
}

/**
  * @brief  Set source and destination source handshaking interface.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CFG_HI               | DST_PER                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  src_handshaking This parameter can be one of the following values:
  *         @arg @ref LL_DMA_SHANDSHAKING_HW
  *         @arg @ref LL_DMA_SHANDSHAKING_HW
  * @param  dst_handshaking This parameter can be one of the following values:
  *         @arg @ref LL_DMA_DHANDSHAKING_HW
  *         @arg @ref LL_DMA_DHANDSHAKING_HW
  * @retval None
  */
__STATIC_INLINE void ll_dma_select_handshaking(dma_regs_t *DMAx, uint32_t channel, uint32_t src_handshaking, uint32_t dst_handshaking)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CFG_LO, DMA_CFGL_HS_SEL_SRC | DMA_CFGL_HS_SEL_DST,
               src_handshaking | dst_handshaking);
}

/**
  * @brief  Source Single Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SGL_REQ_SRC          | REQ_SRC_WE&REQ_SRC                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  REQ_SRC | SRC_WE&SRC
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_src_single_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.SGL_RQ_SRC, (1 << (channel + DMA_SGL_REQ_SRC_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.REQ_SRC, (1 << (channel + DMA_REQ_SRC_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Source Burst Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | REQ_SRC              | SRC_WE&SRC                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_src_burst_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.REQ_SRC, (1 << (channel + DMA_REQ_SRC_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Source Last Single Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SGL_REQ_SRC          | REQ_SRC_WE&REQ_SRC                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  LST_SRC | LST_SRC_WE&LST_SRC
  *  REQ_SRC | SRC_WE&SRC
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_src_last_single_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.SGL_RQ_SRC, (1 << (channel + DMA_SGL_REQ_SRC_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.LST_SRC, (1 << (channel + DMA_LST_SRC_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.REQ_SRC, (1 << (channel + DMA_REQ_SRC_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Source Last Burst Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | LST_SRC              | LST_SRC_WE&LST_SRC                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  REQ_SRC | SRC_WE&SRC
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_src_last_burst_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.LST_SRC, (1 << (channel + DMA_LST_SRC_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.REQ_SRC, (1 << (channel + DMA_REQ_SRC_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Destination Single Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SGL_REQ_DST          | REQ_DST_WE&REQ_DST                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  REQ_DST | DST_WE&DST
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_dst_single_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.SGL_RQ_DST, (1 << (channel + DMA_SGL_REQ_DST_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.REQ_DST, (1 << (channel + DMA_REQ_DST_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Destination Burst Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | REQ_DST              | DST_WE&DST                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_dst_burst_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.REQ_DST, (1 << (channel + DMA_REQ_DST_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Destination Last Single Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SGL_REQ_DST          | REQ_DST_WE&REQ_DST                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  LST_DST | LST_DST_WE&LST_DST
  *  REQ_DST | DST_WE&DST
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_dst_last_single_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.SGL_RQ_DST, (1 << (channel + DMA_SGL_REQ_DST_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.LST_DST, (1 << (channel + DMA_LST_DST_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.REQ_DST, (1 << (channel + DMA_REQ_DST_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Destination Last Burst Transaction Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | LST_DST              | LST_DST_WE&LST_DST                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  REQ_DST | DST_WE&DST
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_req_dst_last_burst_transaction(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->HANDSHAKE.LST_DST, (1 << (channel + DMA_LST_DST_WE_Pos)) + (1 << channel));
    WRITE_REG(DMAx->HANDSHAKE.REQ_DST, (1 << (channel + DMA_REQ_DST_WE_Pos)) + (1 << channel));
}

/** @} */

/** @defgroup DMA_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get DMA Module global transfer complete interrupt status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS_INT           | TFR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_gtfr(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_EVT, DMA_STAT_INT_TFR) == DMA_STAT_INT_TFR);
}

/**
  * @brief  Get DMA Module global block complete interrupt status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS_INT           | BLOCK                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_gblk(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_EVT, DMA_STAT_INT_BLK) == DMA_STAT_INT_BLK);
}

/**
  * @brief  Get DMA Module global source transaction complete interrupt status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS_INT           | SRCT                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_gsrct(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_EVT, DMA_STAT_INT_SRC) == DMA_STAT_INT_SRC);
}

/**
  * @brief  Get DMA Module global destination transaction complete interrupt status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS_INT           | DSTT                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_gdstt(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_EVT, DMA_STAT_INT_DST) == DMA_STAT_INT_DST);
}

/**
  * @brief  Get DMA Module global error interrupt status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS_INT           | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_gerr(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_EVT, DMA_STAT_INT_ERR) == DMA_STAT_INT_ERR);
}

/**
  * @brief  Indicate the Raw Status of IntTfr Interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_TFR              | RAW                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_rtfr(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.RAW_CH_EVT[0], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the Raw Status of IntBlock Interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_BLK              | RAW                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_rblk(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.RAW_CH_EVT[2], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the Raw Status of IntSrcTran Interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_SRC_TRN          | RAW                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_rsrct(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.RAW_CH_EVT[4], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the Raw Status of IntDstTran Interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_DST_TRN          | RAW                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_rdstt(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.RAW_CH_EVT[6], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the Raw Status of IntErr Interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RAW_ERR              | RAW                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_rerr(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.RAW_CH_EVT[8], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the status of DMA Channel transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the status of Channel 0 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr0(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 0)) == (1 << 0));
}

/**
  * @brief  Indicate the status of Channel 1 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr1(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 1)) == (1 << 1));
}

/**
  * @brief  Indicate the status of Channel 2 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr2(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 2)) == (1 << 2));
}

/**
  * @brief  Indicate the status of Channel 3 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr3(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 3)) == (1 << 3));
}

/**
  * @brief  Indicate the status of Channel 4 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr4(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 4)) == (1 << 4));
}

/**
  * @brief  Indicate the status of Channel 5 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr5(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 5)) == (1 << 5));
}

/**
  * @brief  Indicate the status of Channel 6 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr6(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 6)) == (1 << 6));
}

/**
  * @brief  Indicate the status of Channel 7 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_TFR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_tfr7(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[0], (1 << 7)) == (1 << 7));
}

/**
  * @brief  Indicate the status of DMA Channel block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the status of Channel 0 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk0(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 0)) == (1 << 0));
}

/**
  * @brief  Indicate the status of Channel 1 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk1(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 1)) == (1 << 1));
}

/**
  * @brief  Indicate the status of Channel 2 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk2(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 2)) == (1 << 2));
}

/**
  * @brief  Indicate the status of Channel 3 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk3(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 3)) == (1 << 3));
}

/**
  * @brief  Indicate the status of Channel 4 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk4(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 4)) == (1 << 4));
}

/**
  * @brief  Indicate the status of Channel 5 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk5(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 5)) == (1 << 5));
}

/**
  * @brief  Indicate the status of Channel 6 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk6(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 6)) == (1 << 6));
}

/**
  * @brief  Indicate the status of Channel 7 block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_BLK             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_blk7(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[2], (1 << 7)) == (1 << 7));
}

/**
  * @brief  Indicate the status of DMA Channel source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the status of Channel 0 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct0(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 0)) == (1 << 0));
}

/**
  * @brief  Indicate the status of Channel 1 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct1(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 1)) == (1 << 1));
}

/**
  * @brief  Indicate the status of Channel 2 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct2(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 2)) == (1 << 2));
}

/**
  * @brief  Indicate the status of Channel 3 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct3(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 3)) == (1 << 3));
}

/**
  * @brief  Indicate the status of Channel 4 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct4(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 4)) == (1 << 4));
}

/**
  * @brief  Indicate the status of Channel 5 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct5(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 5)) == (1 << 5));
}

/**
  * @brief  Indicate the status of Channel 6 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct6(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 6)) == (1 << 6));
}

/**
  * @brief  Indicate the status of Channel 7 source transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_SRC_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_srct7(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[4], (1 << 7)) == (1 << 7));
}

/**
  * @brief  Indicate the status of DMA Channel destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the status of Channel 0 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt0(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 0)) == (1 << 0));
}

/**
  * @brief  Indicate the status of Channel 1 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt1(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 1)) == (1 << 1));
}

/**
  * @brief  Indicate the status of Channel 2 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt2(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 2)) == (1 << 2));
}

/**
  * @brief  Indicate the status of Channel 3 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt3(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 3)) == (1 << 3));
}

/**
  * @brief  Indicate the status of Channel 4 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt4(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 4)) == (1 << 4));
}

/**
  * @brief  Indicate the status of Channel 5 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt5(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 5)) == (1 << 5));
}

/**
  * @brief  Indicate the status of Channel 6 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt6(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 6)) == (1 << 6));
}

/**
  * @brief  Indicate the status of Channel 7 destination transaction complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_DST_TRN         | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_dstt7(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[6], (1 << 7)) == (1 << 7));
}

/**
  * @brief Indicate the status of DMA Channel error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Indicate the status of Channel 0 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err0(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 0)) == (1 << 0));
}

/**
  * @brief  Indicate the status of Channel 1 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err1(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 1)) == (1 << 1));
}

/**
  * @brief  Indicate the status of Channel 2 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err2(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 2)) == (1 << 2));
}

/**
  * @brief  Indicate the status of Channel 3 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err3(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 3)) == (1 << 3));
}

/**
  * @brief  Indicate the status of Channel 4 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err4(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 4)) == (1 << 4));
}

/**
  * @brief  Indicate the status of Channel 5 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err5(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 5)) == (1 << 5));
}

/**
  * @brief  Indicate the status of Channel 6 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err6(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 6)) == (1 << 6));
}

/**
  * @brief  Indicate the status of Channel 7 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STAT_ERR             | STATUS                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_active_flag_err7(dma_regs_t *DMAx)
{
    return (READ_BITS(DMAx->EVENT.STATUS_CH_EVT[8], (1 << 7)) == (1 << 7));
}

/**
  * @brief  Clear DMA Channel transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << channel));
}

/**
  * @brief  Clear Channel 0 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr0(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 0));
}

/**
  * @brief  Clear Channel 1 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr1(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 1));
}

/**
  * @brief  Clear Channel 2 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr2(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 2));
}

/**
  * @brief  Clear Channel 3 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr3(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 3));
}

/**
  * @brief  Clear Channel 4 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr4(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 4));
}

/**
  * @brief  Clear Channel 5 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr5(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 5));
}

/**
  * @brief  Clear Channel 6 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr6(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 6));
}

/**
  * @brief  Clear Channel 7 transfer complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_TFR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_tfr7(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[0], (1 << 7));
}

/**
  * @brief  Clear DMA Channel block complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << channel));
}

/**
  * @brief  Clear Channel 0 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk0(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 0));
}

/**
  * @brief  Clear Channel 1 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk1(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 1));
}

/**
  * @brief  Clear Channel 2 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk2(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 2));
}

/**
  * @brief  Clear Channel 3 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk3(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 3));
}

/**
  * @brief  Clear Channel 4 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk4(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 4));
}

/**
  * @brief  Clear Channel 5 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk5(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 5));
}

/**
  * @brief  Clear Channel 6 Block Cmplete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk6(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 6));
}

/**
  * @brief  Clear Channel 7 Block Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_BLK              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_blk7(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[2], (1 << 7));
}

/**
  * @brief  Clear DMA Channel source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << channel));
}

/**
  * @brief  Clear Channel 0 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct0(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 0));
}

/**
  * @brief  Clear Channel 1 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct1(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 1));
}

/**
  * @brief  Clear Channel 2 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct2(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 2));
}

/**
  * @brief  Clear Channel 3 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct3(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 3));
}

/**
  * @brief  Clear Channel 4 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct4(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 4));
}

/**
  * @brief  Clear Channel 5 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct5(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 5));
}

/**
  * @brief  Clear Channel 6 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct6(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 6));
}

/**
  * @brief  Clear Channel 7 source transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_SRC_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_srct7(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[4], (1 << 7));
}

/**
  * @brief  Clear DMA Channel destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << channel));
}

/**
  * @brief  Clear Channel 0 destination transaction Complete status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt0(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 0));
}

/**
  * @brief  Clear Channel 1 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt1(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 1));
}

/**
  * @brief  Clear Channel 2 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt2(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 2));
}

/**
  * @brief  Clear Channel 3 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt3(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 3));
}

/**
  * @brief  Clear Channel 4 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt4(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 4));
}

/**
  * @brief  Clear Channel 5 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt5(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 5));
}

/**
  * @brief  Clear Channel 6 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt6(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 6));
}

/**
  * @brief  Clear Channel 7 destination transaction Complete flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_DST_TRN          | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_dstt7(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[6], (1 << 7));
}

/**
  * @brief  Clear DMA Channel error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << channel));
}

/**
  * @brief  Clear Channel 0 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err0(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 0));
}

/**
  * @brief  Clear Channel 1 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err1(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 1));
}

/**
  * @brief  Clear Channel 2 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err2(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 2));
}

/**
  * @brief  Clear Channel 3 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err3(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 3));
}

/**
  * @brief  Clear Channel 4 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err4(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 4));
}

/**
  * @brief  Clear Channel 5 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err5(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 5));
}

/**
  * @brief  Clear Channel 6 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err6(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 6));
}

/**
  * @brief  Clear Channel 7 error flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CLR_ERR              | CLEAR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @retval None.
  */
__STATIC_INLINE void ll_dma_clear_flag_err7(dma_regs_t *DMAx)
{
    WRITE_REG(DMAx->EVENT.CLEAR_CH_EVT[8], (1 << 7));
}

/** @} */

/** @defgroup DMA_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable Transfer Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_TFR             | TFR_WE&TFR                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_it_tfr(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[0], (1 << (channel + DMA_MASK_TFR_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Enable Block Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_BLK             | BLK_WE&BLK                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_it_blk(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[2], (1 << (channel + DMA_MASK_BLK_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Enable source transaction Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_SRC_TRN         | SRC_TRN_WE&SRC_TRN                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_it_srct(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[4], (1 << (channel + DMA_MASK_SRC_TRN_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Enable destination transaction Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_DST_TRN         | DST_TRN_WE&DST_TRN                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_it_dstt(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[6], (1 << (channel + DMA_MASK_DST_TRN_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Enable error interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_ERR             | ERR_WE&ERR                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_it_err(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[8], (1 << (channel + DMA_MASK_ERR_WE_Pos)) + (1 << channel));
}

/**
  * @brief  Disable Transfer Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_TFR             | TFR_WE&TFR                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_it_tfr(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[0], (1 << (channel + DMA_MASK_TFR_WE_Pos)));
}

/**
  * @brief  Disable Block Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_BLK             | BLK_WE&BLK                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_it_blk(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[2], (1 << (channel + DMA_MASK_BLK_WE_Pos)));
}

/**
  * @brief  Disable source transaction Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_SRC_TRN         | SRC_TRN_WE&SRC_TRN                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_it_srct(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[4], (1 << (channel + DMA_MASK_SRC_TRN_WE_Pos)));
}

/**
  * @brief  Disable destination transaction Complete interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_DST_TRN         | DST_TRN_WE&DST_TRN                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_it_dstt(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[6], (1 << (channel + DMA_MASK_DST_TRN_WE_Pos)));
}

/**
  * @brief  Disable error interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_ERR             | ERR_WE&ERR                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_it_err(dma_regs_t *DMAx, uint32_t channel)
{
    WRITE_REG(DMAx->EVENT.MASK_CH_EVT[8], (1 << (channel + DMA_MASK_ERR_WE_Pos)));
}

/**
  * @brief  Check if DMA Transfer interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_TFR             | TFR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enable_it_tfr(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.MASK_CH_EVT[0], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Check if DMA block interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_BLK             | BLK_WE&BLK                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enable_it_blk(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.MASK_CH_EVT[2], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Check if DMA source transaction interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_SRC_TRN         | SRC_TRN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enable_it_srct(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.MASK_CH_EVT[4], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Check if DMA destination transaction interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_DST_TRN         | DST_TRN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enable_it_dstt(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.MASK_CH_EVT[6], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Check if DMA error interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MASK_ERR             | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_dma_is_enable_it_err(dma_regs_t *DMAx, uint32_t channel)
{
    return (READ_BITS(DMAx->EVENT.MASK_CH_EVT[8], (1 << channel)) == (1 << channel));
}

/**
  * @brief  Enable DMA channel interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTLL                 | INI_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_enable_it(dma_regs_t *DMAx, uint32_t channel)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_INI_EN, DMA_CTLL_INI_EN);
}

/**
  * @brief  Disable DMA channel interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTLL                 | INI_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  DMAx DMA instance.
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void ll_dma_disable_it(dma_regs_t *DMAx, uint32_t channel)
{
    MODIFY_REG(DMAx->CHANNEL[channel].CTL_LO, DMA_CTLL_INI_EN, 0);
}

/** @} */

/** @defgroup DMA_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize the DMA registers to their default reset values.
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: DMA registers are de-initialized
  *          - ERROR: DMA registers are not de-initialized
  */
error_status_t ll_dma_deinit(dma_regs_t *DMAx, uint32_t channel);

/**
  * @brief  Initialize the DMA registers according to the specified parameters in p_dma_init.
  * @param  DMAx DMAx instance
  * @param  channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6
  *         @arg @ref LL_DMA_CHANNEL_7
  * @param  p_dma_init pointer to a @ref ll_dma_init_t structure.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: DMA registers are initialized
  *          - ERROR: Not applicable
  */
error_status_t ll_dma_init(dma_regs_t *DMAx, uint32_t channel, ll_dma_init_t *p_dma_init);

/**
  * @brief Set each field of a @ref ll_dma_init_t type structure to default value.
  * @param p_dma_init  Pointer to a @ref ll_dma_init_t structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void ll_dma_struct_init(ll_dma_init_t *p_dma_init);

/** @} */

/** @} */

#endif /* DMA */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_DMA_H__ */

/** @} */

/** @} */

/** @} */
