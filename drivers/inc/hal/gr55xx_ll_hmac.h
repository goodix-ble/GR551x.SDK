/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_hmac.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of HMAC LL library.
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

/** @defgroup LL_HMAC HMAC
  * @brief HMAC LL module driver.
  * @{
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_HMAC_H__
#define __GR55XX_LL_HMAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (HMAC)

/** @defgroup HMAC_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup HMAC_LL_ES_INIT HMAC Exported Init structures
  * @{
  */

/**
  * @brief LL HMAC Init Structure definition
  */
typedef struct _ll_hmac_init_t
{
    uint32_t *p_key;         /**< Key         */

    uint32_t *p_hash;        /**< HASH value  */

} ll_hmac_init_t;

/** @} */

/** @} */

/**
  * @defgroup  HMAC_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup HMAC_LL_Exported_Constants HMAC Exported Constants
  * @{
  */

/** @defgroup HMAC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags definitions which can be used with LL_HMAC_ReadReg function
  * @{
  */
#define LL_HMAC_FLAG_DATAREADY_SHA                           HMAC_STATUS_DATAREADY_SHA      /**< HMAC data ready(SHA mode)   */
#define LL_HMAC_FLAG_DATAREADY_HMAC                          HMAC_STATUS_DATAREADY_HMAC     /**< HMAC data ready(HAMC mode)  */
#define LL_HMAC_FLAG_DMA_MESSAGEDONE                         HMAC_STATUS_MESSAGEDONE_DMA    /**< HMAC dma message done       */
#define LL_HMAC_FLAG_DMA_DONE                                HMAC_STATUS_TRANSDONE_DMA      /**< HMAC dma transfer done      */
#define LL_HMAC_FLAG_DMA_ERR                                 HMAC_STATUS_TRANSERR_DMA       /**< HMAC dma transfer error     */
#define LL_HMAC_FLAG_KEY_VALID                               HMAC_STATUS_KEYVALID           /**< HMAC has fetched key        */
/** @} */

/** @defgroup HMAC_LL_EC_HASH_MODE Hash Mode
  * @{
  */
#define LL_HMAC_HASH_STANDARD                                0x00000000U                            /**< Standard Mode */
#define LL_HMAC_HASH_USER                                    (1UL << HMAC_CONFIG_ENABLE_USERHASH)   /**< User Mode     */
/** @} */

/** @defgroup HMAC_LL_EC_CALCULATE_TYPE Calculate Type
  * @{
  */
#define LL_HMAC_CALCULATETYPE_HMAC                           0x00000000U                            /**< HMAC mode */
#define LL_HMAC_CALCULATETYPE_SHA                            (1UL << HMAC_CONFIG_CALCTYPE_Pos)      /**< SHA  moe  */
/** @} */

/** @defgroup HMAC_LL_EC_KEY_TYPE Key Type
  * @{
  */
#define LL_HMAC_KEYTYPE_MCU                                  0x00000000U                            /**< MCU        */
#define LL_HMAC_KEYTYPE_AHB                                  (1UL << HMAC_CONFIG_KEYTYPE_Pos)       /**< AHB master */
#define LL_HMAC_KEYTYPE_KRAM                                 (2UL << HMAC_CONFIG_KEYTYPE_Pos)       /**< Key Port   */
/** @} */

/** @defgroup HMAC_LL_EC_TRANSFER_SIZE Transfer Size
  * @{
  */
#define LL_HMAC_DMA_TRANSIZE_MIN                             (1)     /**< Min size = 1 block      */
#define LL_HMAC_DMA_TRANSIZE_MAX                             (512)   /**< Min size = 512 blocks   */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HMAC_LL_Exported_Macros HMAC Exported Macros
  * @{
  */

/** @defgroup HMAC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in HMAC register
  * @param  __INSTANCE__ HMAC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_HMAC_WriteReg(__INSTANCE__, __REG__, __VALUE__)   WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in HMAC register
  * @param  __INSTANCE__ HMAC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_HMAC_ReadReg(__INSTANCE__, __REG__)               READ_REG(__INSTANCE__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup HMAC_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup HMAC_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable HMAC.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_ENABLE);
}

/**
  * @brief  Disable HMAC.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CTRL, HMAC_CTRL_ENABLE);
}

/**
  * @brief  Indicate whether the HMAC is enabled.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CTRL, HMAC_CTRL_ENABLE) == (HMAC_CTRL_ENABLE));
}

/**
  * @brief  Enable HMAC DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | START_DMA                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_dma_start(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_START_DMA);
}

/**
  * @brief  Disable HMAC DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | START_DMA                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_dma_start(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CTRL, HMAC_CTRL_START_DMA);
}

/**
  * @brief  Indicate whether the HMAC DMA mode is enabled.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | START_DMA                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_dma_start(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CTRL, HMAC_CTRL_START_DMA) == (HMAC_CTRL_START_DMA));
}

/**
  * @brief  Enable fetch key through AHB/key port.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | ENABLE_RKEY                       |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_read_key(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_ENABLE_RKEY);
}

/**
  * @brief  Enable last block transfer in MCU/DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | LASTTRANSFER                      |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_last_transfer(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CTRL, HMAC_CTRL_LASTTRANSFER);
}

/**
  * @brief  Enable user HASH.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | ENABLE_USERHASH                   |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_user_hash(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CONFIG, HMAC_CONFIG_ENABLE_USERHASH);
}

/**
  * @brief  Disable user HASH.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | ENABLE_USERHASH                   |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_user_hash(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CONFIG, HMAC_CONFIG_ENABLE_USERHASH);
}

/**
  * @brief  Indicate whether the user HASH is enabled.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | ENABLE_USERHASH                   |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_user_hash(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CONFIG, HMAC_CONFIG_ENABLE_USERHASH) == (HMAC_CONFIG_ENABLE_USERHASH));
}

/**
  * @brief  Enable HMAC in little endian.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | ENDIAN                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_little_endian(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CONFIG, HMAC_CONFIG_ENDIAN);
}

/**
  * @brief  Disable HMAC in little endian.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | ENDIAN                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_little_endian(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CONFIG, HMAC_CONFIG_ENDIAN);
}

/**
  * @brief  Indicate whether the HMAC is in little endian.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | ENDIAN                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_little_endian(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CONFIG, HMAC_CONFIG_ENDIAN) == (HMAC_CONFIG_ENDIAN));
}

/**
  * @brief  Set ways to obtain HMAC key.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | KEYTYPE                           |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  type This parameter can be one of the following values:
  *         @arg @ref LL_HMAC_KEYTYPE_MCU
  *         @arg @ref LL_HMAC_KEYTYPE_AHB
  *         @arg @ref LL_HMAC_KEYTYPE_KRAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key_type(hmac_regs_t *HMACx, uint32_t type)
{
    MODIFY_REG(HMACx->CONFIG, HMAC_CONFIG_KEYTYPE, type);
}

/**
  * @brief  Get ways to obtain HMAC key.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | KEYTYPE                           |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_HMAC_KEYTYPE_MCU
  *         @arg @ref LL_HMAC_KEYTYPE_AHB
  *         @arg @ref LL_HMAC_KEYTYPE_KRAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_key_type(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CONFIG, HMAC_CONFIG_KEYTYPE));
}

/**
  * @brief  Enable SHA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | CALCTYPE                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_sha(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CONFIG, HMAC_CONFIG_CALCTYPE);
}

/**
  * @brief  Disable SHA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | CALCTYPE                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_sha(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CONFIG, HMAC_CONFIG_CALCTYPE);
}

/**
  * @brief  Indicate whether the SHA mode is enabled.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | CALCTYPE                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_sha(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CONFIG, HMAC_CONFIG_CALCTYPE) == (HMAC_CONFIG_CALCTYPE));
}

/**
  * @brief  Enable private mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | PRIVATE                           |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_private(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->CONFIG, HMAC_CONFIG_PRIVATE);
}

/**
  * @brief  Disable private mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | PRIVATE                           |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_private(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->CONFIG, HMAC_CONFIG_PRIVATE);
}

/**
  * @brief  Indicate whether the private mode is enabled.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG               | PRIVATE                           |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_private(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->CONFIG, HMAC_CONFIG_PRIVATE) == (HMAC_CONFIG_PRIVATE));
}

/** @} */

/** @defgroup HMAC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable the done interrupt for HMAC.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTERRUPT            | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_enable_it_done(hmac_regs_t *HMACx)
{
    SET_BITS(HMACx->INTERRUPT, HMAC_INTERRUPT_ENABLE);
}

/**
  * @brief  Disable the done interrupt for HMAC.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTERRUPT            | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_disable_it_done(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->INTERRUPT, HMAC_INTERRUPT_ENABLE);
}

/**
  * @brief  Indicate whether Done Interrupt is enabled.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTERRUPT            | ENABLE                            |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_enabled_it_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->INTERRUPT, HMAC_INTERRUPT_ENABLE) == (HMAC_INTERRUPT_ENABLE));
}

/** @} */

/** @defgroup HMAC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Indicate whether SHA Ready flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS               | DATAREADY_SHA                     |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_sha_ready(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STATUS, HMAC_STATUS_DATAREADY_SHA) == HMAC_STATUS_DATAREADY_SHA);
}

/**
  * @brief  Indicate whether HMAC Ready flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS               | DATAREADY_HMAC                    |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_hmac_ready(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STATUS, HMAC_STATUS_DATAREADY_HMAC) == HMAC_STATUS_DATAREADY_HMAC);
}

/**
  * @brief  Indicate whether DMA Transmit Message Done flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS               | MESSAGEDONE_DMA                   |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_dma_message_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STATUS, HMAC_STATUS_MESSAGEDONE_DMA) == HMAC_STATUS_MESSAGEDONE_DMA);
}

/**
  * @brief  Indicate whether DMA Transfer Done flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS               | TRANSDONE_DMA                     |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_dma_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STATUS, HMAC_STATUS_TRANSDONE_DMA) == HMAC_STATUS_TRANSDONE_DMA);
}

/**
  * @brief  Indicate whether DMA Transfer Error flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS               | TRANSERR_DMA                      |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_dma_error(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STATUS, HMAC_STATUS_TRANSERR_DMA) == HMAC_STATUS_TRANSERR_DMA);
}

/**
  * @brief  Indicate whether Key Valid flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | STATUS               | KEYVALID                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_key_valid(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->STATUS, HMAC_STATUS_KEYVALID) == HMAC_STATUS_KEYVALID);
}

/**
  * @brief  Indicate whether Done interrupt flag is set.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTERRUPT            | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_hmac_is_action_flag_it_done(hmac_regs_t *HMACx)
{
    return (READ_BITS(HMACx->INTERRUPT, HMAC_INTERRUPT_DONE) == HMAC_INTERRUPT_DONE);
}

/**
  * @brief  Clear Done interrupt flag.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTERRUPT            | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval None
  */
__STATIC_INLINE void ll_hmac_clear_flag_it_done(hmac_regs_t *HMACx)
{
    CLEAR_BITS(HMACx->INTERRUPT, HMAC_INTERRUPT_DONE);
}

/** @} */

/** @defgroup HMAC_LL_EF_DMA_Management DMA_Management
  * @{
  */

/**
  * @brief  Set HMAC transfer blocks in DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TRANSIZE             | TRANSIZE                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  block This parameter can be one of the following values: 1 ~ 512
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_dma_transfer_block(hmac_regs_t *HMACx, uint32_t block)
{
    MODIFY_REG(HMACx->TRAN_SIZE, HMAC_TRANSIZE, (block << 6) - 1);
}

/**
  * @brief  Get HMAC transfer blocks in DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TRANSIZE             | TRANSIZE                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval Return value is between: 1 ~ 512
  */
__STATIC_INLINE uint32_t ll_hmac_get_dma_transfer_block(hmac_regs_t *HMACx)
{
    return ((READ_BITS(HMACx->TRAN_SIZE, HMAC_TRANSIZE) + 1) >> 6);
}

/**
  * @brief  Set HMAC read address of RAM in DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RSTART_ADDR          | RSTART_ADDR                       |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_dma_read_address(hmac_regs_t *HMACx, uint32_t address)
{
    WRITE_REG(HMACx->RSTART_ADDR, address);
}

/**
  * @brief  Get HMAC read address of RAM in DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RSTART_ADDR          | RSTART_ADDR                       |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval Return value is the address in RAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_dma_read_address(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->RSTART_ADDR));
}

/**
  * @brief  Set HMAC write address of RAM in DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | WSTART_ADDR          | WSTART_ADDR                       |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_dma_write_address(hmac_regs_t *HMACx, uint32_t address)
{
    WRITE_REG(HMACx->WSTART_ADDR, address);
}

/**
  * @brief  Get HMAC write address of RAM in DMA mode.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | WSTART_ADDR          | WSTART_ADDR                       |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval Return value is the address in RAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_dma_write_address(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->WSTART_ADDR));
}

/** @} */

/** @defgroup HMAC_LL_EF_Data_Management Data_Management
  * @{
  */

/**
  * @brief  Set user HASH[255:224].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[0]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_255_224(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[0], hash);
}

/**
  * @brief  Set user HASH[223:192].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[1]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_223_192(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[1], hash);
}

/**
  * @brief  Set user HASH[191:160].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[2]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_191_160(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[2], hash);
}

/**
  * @brief  Set user HASH[159:128].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[3]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_159_128(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[3], hash);
}

/**
  * @brief  Set user HASH[127:96].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[4]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_127_96(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[4], hash);
}

/**
  * @brief  Set user HASH[95:64].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[5]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_95_64(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[5], hash);
}

/**
  * @brief  Set user HASH[63:32].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[6]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_63_32(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[6], hash);
}

/**
  * @brief  Set user HASH[31:0].

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | USER_HASH[7]         | USER_HASH                         |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  hash This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_user_hash_31_0(hmac_regs_t *HMACx, uint32_t hash)
{
    WRITE_REG(HMACx->USER_HASH[7], hash);
}

/**
  * @brief  Get abstract from HMAC.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | FIFO_OUT             | FIFO_OUT                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval Abstract
  */
__STATIC_INLINE uint32_t ll_hmac_get_data(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->FIFO_OUT));
}

/**
  * @brief  Send data to calculate.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | FIFO_MESSAGE         | FIFO_MESSAGE                      |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  data This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_data(hmac_regs_t *HMACx, uint32_t data)
{
    WRITE_REG(HMACx->MESSAGE_FIFO, data);
}

/**
  * @brief  Set HMAC key0.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[0]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key0(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[0], key);
}

/**
  * @brief  Set HMAC key1.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[1]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key1(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[1], key);
}

/**
  * @brief  Set HMAC key2.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[2]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key2(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[2], key);
}

/**
  * @brief  Set HMAC key3.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[3]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key3(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[3], key);
}

/**
  * @brief  Set HMAC key4.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[4]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key4(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[4], key);
}

/**
  * @brief  Set HMAC key5.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[5]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key5(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[5], key);
}

/**
  * @brief  Set HMAC key6.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[6]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key6(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[6], key);
}

/**
  * @brief  Set HMAC key7.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY[7]               | KEY                               |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  key This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key7(hmac_regs_t *HMACx, uint32_t key)
{
    WRITE_REG(HMACx->KEY[7], key);
}

/**
  * @brief  Set HMAC key address in memory.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY_ADDR             | KEY_ADDR                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  address This parameter can be one of the address in RAM
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key_address(hmac_regs_t *HMACx, uint32_t address)
{
    WRITE_REG(HMACx->KEY_ADDR, address);
}

/**
  * @brief  Get HMAC key address in memory.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KEY_ADDR             | KEY_ADDR                          |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @retval Return value is the address in RAM
  */
__STATIC_INLINE uint32_t ll_hmac_get_key_address(hmac_regs_t *HMACx)
{
    return (READ_REG(HMACx->KEY_ADDR));
}

/**
  * @brief  Set HMAC fetch key port mask.

  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | KPORT_MASK           | KPORT_MASK                        |
  *  +----------------------+-----------------------------------+
  * \endrst

  * @param  HMACx HMAC instance
  * @param  mask This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_hmac_set_key_port_mask(hmac_regs_t *HMACx, uint32_t mask)
{
    WRITE_REG(HMACx->KPORT_MASK, mask);
}

/** @} */

/** @defgroup HMAC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize HMAC registers (Registers restored to their default values).
  * @param  HMACx       HMAC Instance
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: HMAC registers are de-initialized
  *          - ERROR: HMAC registers are not de-initialized
  */
error_status_t ll_hmac_deinit(hmac_regs_t *HMACx);

/**
  * @brief  Initialize HMAC registers according to the specified
  *         parameters in p_hmac_init.
  * @param  HMACx       HMAC Instance
  * @param  p_hmac_init   Pointer to a ll_hmac_init_t structure that contains the configuration
  *                     information for the specified HMAC peripheral.
  * @retval An error_status_t  enumeration value:
  *          - SUCCESS: HMAC registers are initialized according to p_hmac_init content
  *          - ERROR: Problem occurred during HMAC Registers initialization
  */
error_status_t ll_hmac_init(hmac_regs_t *HMACx, ll_hmac_init_t *p_hmac_init);

/**
  * @brief Set each field of a @ref ll_hmac_init_t type structure to default value.
  * @param p_hmac_init    Pointer to a @ref ll_hmac_init_t structure
  *                     whose fields will be set to default values.
  * @retval None
  */
void ll_hmac_struct_init(ll_hmac_init_t *p_hmac_init);

/** @} */

/** @} */

#endif /* HMAC */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_HMAC_H__ */

/** @} */

/** @} */

/** @} */
