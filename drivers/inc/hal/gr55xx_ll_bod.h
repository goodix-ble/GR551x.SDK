/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_bod.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CALENDAR LL library.
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

/** @defgroup LL_BOD BOD
  * @brief BOD LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_BOD_H_
#define __GR55XX_LL_BOD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

/** @defgroup BOD_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup BOD_LL_ES_INIT BOD Exported init structures
  * @{
  */

/**
  * @brief LL BOD init Structure definition
  */
typedef struct _ll_bod_init
{
    uint8_t bod_en;      /**< Specifies the bod enable.

                                 This parament can be modified afterwards using unitary function @ref ll_bod_enable() and  ll_bod_disable(). */

    uint8_t bod2_en;     /**< Specifies the bod2 enable.

                                 This parament can be modified afterwards using unitary function @ref ll_bod2_enable() and  ll_bod2_disable().. */

    uint8_t bod2_lvl;    /**< Specifies the bod2 level.

                                 This parament can be modified afterwards using unitary function @ref ll_bod2_lvl_ctrl_lv_set(). */

    uint8_t bod_static_en;    /**< Specifies the bod static enbale.
                                 This parameter can be a value of @ref LL_BOD_STATIC_ENABLE.
                                 This parament can be modified afterwards using unitary function @ref ll_bod_static_lv_enable() and ll_bod_static_lv_disable(). */
} ll_bod_init_t;

/** @} */

/** @} */

/**
  * @defgroup  BOD_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup BOD_LL_Exported_Constants BOD Exported Constants
  * @{
  */

/** @defgroup BOD_LL_ENABLE BOD ENABLE
  * @{
  */
#define LL_BOD_ENABLE                0x1   /**< BOD enable  */
#define LL_BOD_DISABLE               0x0   /**< BOD disable  */
/** @} */

/** @defgroup BOD2_LL_ENABLE BOD2 ENABLE
  * @{
  */
#define LL_BOD2_ENABLE               0x1   /**< BOD2 enable  */
#define LL_BOD2_DISABLE              0x0   /**< BOD2 disable  */
/** @} */

/** @defgroup BOD_LL_STATIC_ENABLE BOD STATIC ENABLE
  * @{
  */
#define LL_BOD_STATIC_ENABLE        (0x1)   /**< BOD STATIC enable  */
#define LL_BOD_STATIC_DISABLE       (0x0)   /**< BOD STATIC disable  */
/** @} */

/** @defgroup BOD2_LL_LEVEL BOD2 LVEVL
  * @{
  */
#define LL_BOD2_LEVEL_0              0x0   /**< BOD2 Level 0  */
#define LL_BOD2_LEVEL_1              0x1   /**< BOD2 Level 1  */
#define LL_BOD2_LEVEL_2              0x2   /**< BOD2 Level 2  */
#define LL_BOD2_LEVEL_3              0x3   /**< BOD2 Level 3  */
#define LL_BOD2_LEVEL_4              0x4   /**< BOD2 Level 4  */
#define LL_BOD2_LEVEL_5              0x5   /**< BOD2 Level 5  */
#define LL_BOD2_LEVEL_6              0x6   /**< BOD2 Level 6  */
#define LL_BOD2_LEVEL_7              0x7   /**< BOD2 Level 7  */
/** @} */

/** @} */

/** @} */

/** @defgroup BOD_LL_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
  * @brief  Enable the bod
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod_en_lv
  *
  */
__STATIC_INLINE void ll_bod_enable(void)
{
    SET_BITS(AON->RF_REG_3, AON_RF_REG_3_BOD_EN);
}

/**
  * @brief  Disable the bod
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod_en_lv
  *
  */
__STATIC_INLINE void ll_bod_disable(void)
{
    CLEAR_BITS(AON->RF_REG_3, AON_RF_REG_3_BOD_EN);
}

/**
  * @brief  Enable the bod2
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod2_en_lv
  *
  */
__STATIC_INLINE void ll_bod2_enable(void)
{
    SET_BITS(AON->RF_REG_3, AON_RF_REG_3_BOD2_EN);
}

/**
  * @brief  Disable the bod2
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod2_en_lv
  *
  */
__STATIC_INLINE void ll_bod2_disable(void)
{
    CLEAR_BITS(AON->RF_REG_3, AON_RF_REG_3_BOD2_EN);
}

/**
  * @brief  Set bod control level
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod_lvl_ctrl_lv_3_0
  * @param  lvl_ctrl_lv: 0x0 ~ 0x7
  */
__STATIC_INLINE void ll_bod2_lvl_ctrl_lv_set(uint8_t lvl_ctrl_lv)
{
    MODIFY_REG(AON->RF_REG_3, AON_RF_REG_3_BOD_LVL_CTRL_LV, (lvl_ctrl_lv << AON_RF_REG_3_BOD_LVL_CTRL_LV_Pos));
}

/**
  * @brief  enable bod static lv
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod_static_lv
  */
__STATIC_INLINE void ll_bod_static_lv_enable(void)
{
    SET_BITS(AON->RF_REG_3, AON_RF_REG_3_BOD_STATIC_LV_EN);
}

/**
  * @brief  disable bod static lv
  *
  *  Register|BitsName
  *  --------|--------
  *  RF_REG_3 | bod_static_lv
  */
__STATIC_INLINE void ll_bod_static_lv_disable(void)
{
    CLEAR_BITS(AON->RF_REG_3, AON_RF_REG_3_BOD_STATIC_LV_EN);
}

/**
  * @brief  enable BOD FEDGE Event.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_IRQ | PMU_BOD_FEDGE
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_bod_enable_fedge(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_WAKE_UP_SEL_PMU_BOD_FEDGE);
}

/**
  * @brief  disable BOD FEDGE Event.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_IRQ | PMU_BOD_FEDGE
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE void ll_bod_disable_fedge(void)
{
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_WAKE_UP_SEL_PMU_BOD_FEDGE);
}

/**
  * @brief  Indicate if the BOD REDGE Event Flag is set or not.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_IRQ | PMU_BOD_REDGE
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_bod_is_active_flag_fedge(void)
{
    return (uint32_t)(READ_BITS(AON->SLP_EVENT, AON_SLP_EVENT_PMU_BOD_FEDGE) == AON_SLP_EVENT_PMU_BOD_FEDGE);
}

/**
  * @brief  Clear Interrupt Status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  AON_IRQ| PMU_BOD_REDGE
  *
  * @retval None
  */
__STATIC_INLINE void ll_bod_clear_flag_fedge(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    WRITE_REG(AON->SLP_EVENT, ~AON_SLP_EVENT_PMU_BOD_FEDGE);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  De-initialize the BOD registers to their default reset values.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PDM registers are de-initialized
  *          - ERROR: PDM registers are not de-initialized
  */
error_status_t ll_bod_deinit(void);

/**
  * @brief  Initialize the BOD registers according to the specified parameters.
  * @param  p_bod_init pointer to a @ref ll_bod_init_t structure.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: BOD registers are initialized
  *          - ERROR: Not applicable
  */
error_status_t ll_bod_init(ll_bod_init_t *p_bod_init);

/**
  * @brief Set BOD initial structure to default value.
  * @param p_bod_init  Pointer to a @ref ll_bod_init_t structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void ll_bod_struct_init(ll_bod_init_t *p_bod_init);
/** @} */

#endif

/** @} */

/** @} */

/** @} */
