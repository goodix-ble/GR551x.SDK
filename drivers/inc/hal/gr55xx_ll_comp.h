/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_comp.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of COMP LL library.
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

/** @defgroup LL_COMP COMP
  * @brief COMP LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_COMP_H__
#define __GR55XX_LL_COMP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON)

/** @defgroup COMP_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup COMP_LL_ES_INIT COMP Exported init structures
  * @{
  */

/**
  * @brief LL COMP init Structure definition
  */
typedef struct _ll_comp_init
{
    uint32_t input_source;  /**< Specifies the input source for the comparator.
                                 This parameter can be any value of @ref COMP_LL_EC_INPUT_SRC.

                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_input_src(). */

    uint32_t ref_source;    /**< Specifies the reference source for the comparator.
                                 This parameter can be any value of @ref COMP_LL_EC_INPUT_SRC.

                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_ref_src(). */
    uint32_t ref_value;     /*!< Specifies the value of the COMP buffered reference.
                                 If ref_source select to LL_COMP_REF_SRC_VBAT, this parameter can be a value between: 0 ~ 7.
                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_vbatt_lvl().

                                 If ref_source select to LL_COMP_REF_SRC_VREF, this parameter can be a value between: 0 ~ 63.
                                 This parameter can be modified afterwards using unitary function @ref ll_comp_set_vref_lvl(). */
} ll_comp_init_t;

/** @} */

/** @} */

/**
  * @defgroup  COMP_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_LL_EC_INPUT_SRC COMP INPUT SOURCE
  * @{
  */
#define LL_COMP_INPUT_SRC_IO0         (0UL << AON_RF_REG_10_CHANNEL_SEL_P_Pos)    /**< Set MSIO_0 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO1         (1UL << AON_RF_REG_10_CHANNEL_SEL_P_Pos)    /**< Set MSIO_1 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO2         (2UL << AON_RF_REG_10_CHANNEL_SEL_P_Pos)    /**< Set MSIO_2 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO3         (3UL << AON_RF_REG_10_CHANNEL_SEL_P_Pos)    /**< Set MSIO_3 as inputs for the comparator */
#define LL_COMP_INPUT_SRC_IO4         (4UL << AON_RF_REG_10_CHANNEL_SEL_P_Pos)    /**< Set MSIO_4 as inputs for the comparator */
/** @} */

/** @defgroup COMP_LL_EC_REF_SRC COMP REF SOURCE
  * @{
  */
#define LL_COMP_REF_SRC_IO0           (0UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set MSIO_0 as references for the comparator */
#define LL_COMP_REF_SRC_IO1           (1UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set MSIO_1 as references for the comparator */
#define LL_COMP_REF_SRC_IO2           (2UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set MSIO_2 as references for the comparator */
#define LL_COMP_REF_SRC_IO3           (3UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set MSIO_3 as references for the comparator */
#define LL_COMP_REF_SRC_IO4           (4UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set MSIO_4 as references for the comparator */
#define LL_COMP_REF_SRC_VBAT          (6UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set VBATT as references for the comparator  */
#define LL_COMP_REF_SRC_VREF          (7UL << AON_RF_REG_10_CHANNEL_SEL_N_Pos)    /**< Set VREF as references for the comparator   */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP_LL_Exported_Macros COMP Exported Macros
  * @{
  */

/** @defgroup COMP_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in COMP register
  * @param  __instance__ COMP instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_COMP_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG((__instance__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in COMP register
  * @param  __instance__ COMP instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_COMP_ReadReg(__instance__, __REG__) READ_REG((__instance__)->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup COMP_LL_Private_Macros COMP Private Macros
  * @{
  */

/** @defgroup COMP_LL_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief Default configuartion for initializing structure
  */
#define LL_COMP_DEFAULT_CONFIG                     \
{                                                  \
    .channel_p  = LL_COMP_CHANNEL_IO0,             \
    .channel_n  = LL_COMP_CHANNEL_IO1,             \
}
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup COMP_LL_EF_Configuration Basic Configuration
  * @{
  */

/**
  * @brief  Enable COMP module.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RF_REG_10            | COMP_EN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_enable(void)
{
	SET_BITS(AON->PWR_RET01, AON_PWR_REG01_WAKE_UP_SEL_MSIO_COMP);
	SET_BITS(AON->RF_REG_10, AON_RF_REG_10_WAKE_COMP_EN_Msk);
}

/**
  * @brief  Disable COMP module.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RF_REG_10            | COMP_EN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_comp_disable(void)
{
    CLEAR_BITS(AON->RF_REG_10, AON_RF_REG_10_WAKE_COMP_EN_Msk);
	CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_WAKE_UP_SEL_MSIO_COMP);
}

/**
  * @brief  Set channel of COMP input source.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RF_REG_10            | AON_RF_REG_10_CHANNEL_SEL_P       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_COMP_INPUT_SRC_IO0
  *         @arg @ref LL_COMP_INPUT_SRC_IO1
  *         @arg @ref LL_COMP_INPUT_SRC_IO2
  *         @arg @ref LL_COMP_INPUT_SRC_IO3
  *         @arg @ref LL_COMP_INPUT_SRC_IO4
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_input_src(uint32_t source)
{
    MODIFY_REG(AON->RF_REG_10, AON_RF_REG_10_CHANNEL_SEL_P_Msk, source);
}

/**
  * @brief  Set channel of COMP reference source.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RF_REG_10            | AON_RF_REG_10_CHANNEL_SEL_N       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_COMP_REF_SRC_IO0
  *         @arg @ref LL_COMP_REF_SRC_IO1
  *         @arg @ref LL_COMP_REF_SRC_IO2
  *         @arg @ref LL_COMP_REF_SRC_IO3
  *         @arg @ref LL_COMP_REF_SRC_IO4
  *         @arg @ref LL_COMP_REF_SRC_VBAT
  *         @arg @ref LL_COMP_REF_SRC_VREF
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_ref_src(uint32_t source)
{
    MODIFY_REG(AON->RF_REG_10, AON_RF_REG_10_CHANNEL_SEL_N_Msk, source);
}

/**
  * @brief  Set VBATT control level.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RF_REG_10            | BATT_LVL_CTRL_LV                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  level This parameter can be a value between: 0 ~ 7
  *         Vbatt_ref = ((level+1)/10) * VBATT
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_vbatt_lvl(uint32_t level)
{
    MODIFY_REG(AON->RF_REG_10, AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Msk, level << AON_RF_REG_10_COMP_BATT_LVL_CTRL_LV_Pos);
}

/**
  * @brief  Set VREF control level.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | RF_REG_10            | COMP_REF_CTRL_LV                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  level This parameter can be a value between: 0 ~ 63
  *         Vref = 30mv * level
  * @retval None
  */
__STATIC_INLINE void ll_comp_set_vref_lvl(uint32_t level)
{
    MODIFY_REG(AON->RF_REG_10, AON_RF_REG_10_COMP_REF_CTRL_LV_Msk, level << AON_RF_REG_10_COMP_REF_CTRL_LV_Pos);
}

/**
  * @brief  Indicate if the COMP Interrupt Flag is set or not.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | MSIO_COMP                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or o).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_comp_is_active_flag_it(void)
{
    return (READ_BITS(AON->SLP_EVENT, AON_SLP_EVENT_PMU_MSIO_COMP) == AON_SLP_EVENT_PMU_MSIO_COMP);
}

/**
  * @brief Clear Interrupt Status flag for COMP.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | MSIO_COMP                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None.
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_comp_clear_flag_it(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->SLP_EVENT, AON_SLP_EVENT_PMU_MSIO_COMP);
    GLOBAL_EXCEPTION_ENABLE();
}

/** @} */

/** @defgroup COMP_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize COMP registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: COMP registers are de-initialized
  *          - ERROR: COMP registers are not de-initialized
  */
error_status_t ll_comp_deinit(void);

/**
  * @brief  Initialize COMP registers according to the specified.
  *         parameters in p_comp_init.
  * @param  p_comp_init Pointer to a ll_comp_init_t structure that contains the configuration
  *                             information for the specified COMP peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: COMP registers are initialized according to p_comp_init content
  *          - ERROR: Problem occurred during COMP Registers initialization
  */
error_status_t ll_comp_init(ll_comp_init_t *p_comp_init);

/**
  * @brief Set each field of a @ref ll_comp_init_t type structure to default value.
  * @param p_comp_init  Pointer to a @ref ll_comp_init_t structure
  *                             whose fields will be set to default values.
  * @retval None
  */
void ll_comp_struct_init(ll_comp_init_t *p_comp_init);

/** @} */

/** @} */

#endif /* AON */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_COMP_H__ */

/** @} */

/** @} */

/** @} */
