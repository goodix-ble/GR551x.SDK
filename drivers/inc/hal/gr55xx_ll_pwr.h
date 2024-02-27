/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_pwr.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWR LL library.
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

/** @defgroup LL_PWR PWR
  * @brief PWR LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_LL_PWR_H__
#define __GR55xx_LL_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON)

/**
  * @defgroup  PWR_LL_MACRO Defines
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup PWR_LL_Private_Constants PWR Private Constants
  * @{
  */

/** @defgroup PWR_LL_PC_EXT_WAKEUP_CTL_LSB  External Wakeup Control Low Significant Bit Defines
  * @{
  */
#define LL_PWR_EXTWKUP_TYPE_LSB             (0x01U <<  AON_EXT_WKUP_CTL_TYPE_Pos)   /**< External wakeup level type */
#define LL_PWR_EXTWKUP_INVERT_LSB           (0x01U <<  AON_EXT_WKUP_CTL_INVERT_Pos) /**< External wakeup level invert */
#define LL_PWR_EXTWKUP_SRC_EN_LSB           (0x01U <<  AON_EXT_WKUP_CTL_SRC_EN_Pos) /**< External wakeup source enable */
/** @} */

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PWR_LL_Private_Macro PWR Private Macros
  * @{
  */

/** @defgroup PWR_LL_PM_EXT_WAKEUP_CTL_LSB  External Wakeup Control Low Significant Bit Defines
  * @{
  */

/**
  * @brief PWR_LL_PM_GET_MEM_PWR_MSK PWR Get Memory Power Value Mask
  */
#define __LL_PWR_GET_MEM_PWR_MASK(__POWER__)   (((__POWER__) == LL_PWR_MEM_POWER_OFF) ? 0x0U : \
                                                (((__POWER__) == LL_PWR_MEM_POWER_FULL) ? 0xAAAAAAAAU : 0xFFFFFFFFU))

/** @} */

/** @} */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_LL_EC_WAKEUP_COND  Wakeup Condition
  * @{
  */
#define LL_PWR_WKUP_COND_EXT                AON_PWR_REG01_WAKE_UP_SEL_EXTWKUP       /**< External wakeup: AON_GPIO   */
#define LL_PWR_WKUP_COND_TIMER              AON_PWR_REG01_WAKE_UP_SEL_TIMER         /**< AON Timer wakeup            */
#define LL_PWR_WKUP_COND_BLE                AON_PWR_REG01_WAKE_UP_SEL_BLE           /**< BLE wakeup                  */
#define LL_PWR_WKUP_COND_CALENDAR           AON_PWR_REG01_WAKE_UP_SEL_CALENDAR      /**< Calendar wakeup             */
#define LL_PWR_WKUP_COND_BOD_FEDGE          AON_PWR_REG01_WAKE_UP_SEL_PMU_BOD_FEDGE /**< PMU Bod falling edge wakeup */
#define LL_PWR_WKUP_COND_MSIO_COMP          AON_PWR_REG01_WAKE_UP_SEL_MSIO_COMP     /**< Msio comparator wakeup      */
#define LL_PWR_WKUP_COND_ALL                AON_PWR_REG01_WAKE_UP_SEL               /**< All wakeup sources mask     */
/** @} */


/** @defgroup PWR_LL_EC_WAKEUP_EVT Wakeup Event
 *  @note     Only available on GR5515_C and later version
  * @{
  */
#define LL_PWR_WKUP_EVENT_BLE               AON_SLP_EVENT_SMCOSCEN              /**< BLE Timer wakeup event             */
#define LL_PWR_WKUP_EVENT_TIMER             AON_SLP_EVENT_TIMER                 /**< AON Timer wakeup event             */
#define LL_PWR_WKUP_EVENT_EXT               AON_SLP_EVENT_EXTWKUP               /**< External wakeup event: AON_GPIO    */
#define LL_PWR_WKUP_EVENT_BOD_FEDGE         AON_SLP_EVENT_PMU_BOD_FEDGE         /**< PMU Bod wakeup event               */
#define LL_PWR_WKUP_EVENT_MSIO_COMP         AON_SLP_EVENT_PMU_MSIO_COMP         /**< Msio comparator wakeup event       */
#define LL_PWR_WKUP_EVENT_WDT               AON_SLP_EVENT_WDT_REBOOT            /**< AON WDT wakeup event               */
#define LL_PWR_WKUP_EVENT_CALENDAR          AON_SLP_EVENT_CALENDAR_TIMER_ALARM  /**< Calendar wakeup event              */
#define LL_PWR_WKUP_EVENT_ALL               (AON_SLP_EVENT_SMCOSCEN      | \
                                             AON_SLP_EVENT_TIMER         | \
                                             AON_SLP_EVENT_EXTWKUP       | \
                                             AON_SLP_EVENT_PMU_BOD_FEDGE | \
                                             AON_SLP_EVENT_PMU_MSIO_COMP | \
                                             AON_SLP_EVENT_WDT_REBOOT    | \
                                             AON_SLP_EVENT_CALENDAR_TIMER_ALARM) /**< All event mask  */
/** @} */

/** @defgroup PWR_LL_EC_EXTWAKEUP_PIN  External Wakeup Pins
  * @{
  */
#define LL_PWR_EXTWKUP_PIN0                 (0x00000001U)   /**< WKUP pin 0 : AON_GPIO_PIN0 */
#define LL_PWR_EXTWKUP_PIN1                 (0x00000002U)   /**< WKUP pin 1 : AON_GPIO_PIN1 */
#define LL_PWR_EXTWKUP_PIN2                 (0x00000004U)   /**< WKUP pin 2 : AON_GPIO_PIN2 */
#define LL_PWR_EXTWKUP_PIN3                 (0x00000008U)   /**< WKUP pin 3 : AON_GPIO_PIN3 */
#define LL_PWR_EXTWKUP_PIN4                 (0x00000010U)   /**< WKUP pin 4 : AON_GPIO_PIN4 */
#define LL_PWR_EXTWKUP_PIN5                 (0x00000020U)   /**< WKUP pin 5 : AON_GPIO_PIN5 */
#define LL_PWR_EXTWKUP_PIN6                 (0x00000040U)   /**< WKUP pin 6 : AON_GPIO_PIN6 */
#define LL_PWR_EXTWKUP_PIN7                 (0x00000080U)   /**< WKUP pin 7 : AON_GPIO_PIN7 */
#define LL_PWR_EXTWKUP_PIN_ALL              (0x000000FFU)   /**< WKUP pin all : AON_GPIO_PIN0 ~ AON_GPIO_PIN7 */
/** @} */

/** @defgroup PWR_LL_EC_EXTWAKEUP_TYPE  External Wakeup Type
  * @{
  */
#define LL_PWR_EXTWKUP_TYPE_LOW            (LL_PWR_EXTWKUP_INVERT_LSB | LL_PWR_EXTWKUP_TYPE_LSB | LL_PWR_EXTWKUP_SRC_EN_LSB)  /**< Low level wakeup */
#define LL_PWR_EXTWKUP_TYPE_HIGH           (LL_PWR_EXTWKUP_TYPE_LSB | LL_PWR_EXTWKUP_SRC_EN_LSB)      /**< High level wakeup */
#define LL_PWR_EXTWKUP_TYPE_RISING         (0x00000000U)                                                    /**< Rising edge wakeup */
#define LL_PWR_EXTWKUP_TYPE_FALLING        (LL_PWR_EXTWKUP_INVERT_LSB | LL_PWR_EXTWKUP_SRC_EN_LSB)    /**< Falling edge wakeup */
/** @} */

/** @defgroup PWR_LL_EC_PSC_CMD  Power State Control Commands
 * @{
 */
#define LL_PWR_CMD_LOOPBACK                 AON_PSC_CMD_OPC_OPCODE_LOOPBACK         /**< Reserved command 0                         */
#define LL_PWR_CMD_EF_DIR_ON                AON_PSC_CMD_OPC_OPCODE_EF_DIR_ON        /**< Reserved command 1                         */
#define LL_PWR_CMD_32_TIMER_LD              AON_PSC_CMD_OPC_OPCODE_32_TIMER_LD      /**< Load sleep timer command                   */
#define LL_PWR_CMD_DEEP_SLEEP               AON_PSC_CMD_OPC_OPCODE_DEEP_SLEEP       /**< Enter Deep Sleep Mode command              */
#define LL_PWR_CMD_EF_DIR_OFF               AON_PSC_CMD_OPC_OPCODE_EF_DIR_OFF       /**< Reserved command 2                         */
#define LL_PWR_CMD_EXT_CLK                  AON_PSC_CMD_OPC_OPCODE_EXT_CLK          /**< Select external clock (xo_32KHz) command   */
#define LL_PWR_CMD_RNG_CLK                  AON_PSC_CMD_OPC_OPCODE_RNG_CLK          /**< Select RING OSC clock command              */
#define LL_PWR_CMD_RTC_CLK                  AON_PSC_CMD_OPC_OPCODE_RTC_CLK          /**< Select RTC clock command                   */
#define LL_PWR_CMD_RNG2_CLK                 AON_PSC_CMD_OPC_OPCODE_RNG2_CLK         /**< Select RING OSC clock command              */
#define LL_PWR_CMD_LD_MEM_SLP_CFG           AON_PSC_CMD_OPC_OPCODE_LD_MEM_SLP_CFG   /**< Load memory sleep settings command         */
#define LL_PWR_CMD_LD_MEM_WKUP_CFG          AON_PSC_CMD_OPC_OPCODE_LD_MEM_WKUP_CFG  /**< Load memory wakeup settings command        */
#define LL_PWR_CMD_DPAD_LE_HI               AON_PSC_CMD_OPC_OPCODE_DPAD_LE_HI       /**< Force dpad_le high                         */
#define LL_PWR_CMD_DPAD_LE_LO               AON_PSC_CMD_OPC_OPCODE_DPAD_LE_LO       /**< Force dpad_le low                          */
#define LL_PWR_CMD_SLP_TIMER_MODE_NORMAL    AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_0 /**< Enable sleep timer mode 0 command          */
#define LL_PWR_CMD_SLP_TIMER_MODE_SINGLE    AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_1 /**< Enable sleep timer mode 1 command          */
#define LL_PWR_CMD_SLP_TIMER_MODE_RELOAD    AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_2 /**< Enable sleep timer mode 2 command          */
#define LL_PWR_CMD_SLP_TIMER_MODE_DISABLE   AON_PSC_CMD_OPC_OPCODE_SLP_TIMER_MODE_3 /**< Enable sleep timer mode 3 command          */
/** @} */


/** @} */

/** @defgroup PWR_LL_EC_DPAD_VALUE  Dpad LE State
 * @{
 */
#define LL_PWR_DPAD_LE_OFF                  (0x00000000U)   /**< Dpad LE LOW */
#define LL_PWR_DPAD_LE_ON                   (0x00000001U)   /**< Dpad LE High  */
/** @} */

/** @defgroup PWR_LL_EC_TIMER_READ_SEL  Timer Read Select
 *  @note     Only available on GR5515_C and later version
 * @{
 */
#define LL_PWR_TIMER_READ_SEL_CAL_TIMER     AON_PAD_CTL1_TIMER_READ_SEL_CAL_TIMER   /**< Calendar timer     */
#define LL_PWR_TIMER_READ_SEL_AON_WDT       AON_PAD_CTL1_TIMER_READ_SEL_AON_WDT     /**< AON watchdog timer */
#define LL_PWR_TIMER_READ_SEL_SLP_TIMER     AON_PAD_CTL1_TIMER_READ_SEL_SLP_TIMER   /**< Sleep timer        */
#define LL_PWR_TIMER_READ_SEL_CAL_ALARM     AON_PAD_CTL1_TIMER_READ_SEL_CAL_ALARM   /**< Calendar alarm     */
/** @} */

/** @} */


/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWR_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup PWR_LL_EM_WRITE_READ Common write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PWR register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_PWR_WriteReg(__REG__, __VALUE__) WRITE_REG(AON->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PWR register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_PWR_ReadReg(__REG__) READ_REG(AON->__REG__)
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Functions PWR Exported Functions
  * @{
  */

/** @defgroup PWR_LL_EF_Low_Power_Mode_Configuration Low power mode configuration
  * @{
  */

/**
  * @brief  Set the DeepSleep WakeUp Condition
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WAKE_UP_SEL                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  condition This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WKUP_COND_EXT
  *         @arg @ref LL_PWR_WKUP_COND_TIMER
  *         @arg @ref LL_PWR_WKUP_COND_BLE
  *         @arg @ref LL_PWR_WKUP_COND_CALENDAR
  *         @arg @ref LL_PWR_WKUP_COND_BOD_FEDGE
  *         @arg @ref LL_PWR_WKUP_COND_MSIO_COMP
  *         @arg @ref LL_PWR_WKUP_COND_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_wakeup_condition(uint32_t condition)
{
    MODIFY_REG(AON->PWR_RET01, AON_PWR_REG01_WAKE_UP_SEL, condition);
}

/**
  * @brief  Get the Selected DeepSleep WakeUp Condition
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WAKE_UP_SEL                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WKUP_COND_EXT
  *         @arg @ref LL_PWR_WKUP_COND_TIMER
  *         @arg @ref LL_PWR_WKUP_COND_BLE
  *         @arg @ref LL_PWR_WKUP_COND_CALENDAR
  *         @arg @ref LL_PWR_WKUP_COND_BOD_FEDGE
  *         @arg @ref LL_PWR_WKUP_COND_MSIO_COMP
  *         @arg @ref LL_PWR_WKUP_COND_ALL
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_wakeup_condition(void)
{
    return ((uint32_t)READ_BITS(AON->PWR_RET01, AON_PWR_REG01_WAKE_UP_SEL));
}

/**
  * @brief  Get the Event that triggered the DeepSleep WakeUp.
 *  @note     Only available on GR5515_C and later version
 *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | SMCOSCEN_EVENT                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  SLP_EVENT | TIMER_EVENT
  *  SLP_EVENT | EXT_WKUP_EVENT
  *  SLP_EVENT | WATCHDOG_EVENT
  *
  * @retval Returned value can be combination of the following values:
  *         @arg @ref LL_PWR_WKUP_EVENT_BLE
  *         @arg @ref LL_PWR_WKUP_EVENT_TIMER
  *         @arg @ref LL_PWR_WKUP_EVENT_EXT
  *         @arg @ref LL_PWR_WKUP_EVENT_BOD_FEDGE
  *         @arg @ref LL_PWR_WKUP_EVENT_MSIO_COMP
  *         @arg @ref LL_PWR_WKUP_EVENT_WDT
  *         @arg @ref LL_PWR_WKUP_EVENT_CALENDAR
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_wakeup_event(void)
{
    return ((uint32_t)READ_BITS(AON->SLP_EVENT, LL_PWR_WKUP_EVENT_ALL));
}

/**
  * @brief  Enable the External WakeUp PINx functionality
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | MASK                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_ext_wakeup_pin(uint32_t wakeup_pin)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->EXT_WKUP_CTL, wakeup_pin);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable the External WakeUp PINx functionality
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | MASK                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_ext_wakeup_pin(uint32_t wakeup_pin)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->EXT_WKUP_CTL, wakeup_pin);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Check if the External WakeUp PINx functionality is enabled
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | MASK                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_ext_wakeup_pin(uint32_t wakeup_pin)
{
    return (READ_BITS(AON->EXT_WKUP_CTL, wakeup_pin) == wakeup_pin);
}

/**
  * @brief  Set the WakeUp Type of External WakeUp PINx.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | INVERT                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  EXT_WKUP_CTL | TYPE
  *
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  * @param  wakeup_type This parameter can be one of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_LOW
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_HIGH
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_RISING
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_FALLING
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_ext_wakeup_type(uint32_t wakeup_pin, uint32_t wakeup_type)
{
    uint32_t invert =  ((wakeup_type & LL_PWR_EXTWKUP_INVERT_LSB) == LL_PWR_EXTWKUP_INVERT_LSB) ? (wakeup_pin << AON_EXT_WKUP_CTL_INVERT_Pos) : 0;
    uint32_t type =  ((wakeup_type & LL_PWR_EXTWKUP_TYPE_LSB) == LL_PWR_EXTWKUP_TYPE_LSB) ? (wakeup_pin << AON_EXT_WKUP_CTL_TYPE_Pos) : 0;
    GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(AON->EXT_WKUP_CTL, (wakeup_pin << AON_EXT_WKUP_CTL_INVERT_Pos) | (wakeup_pin << AON_EXT_WKUP_CTL_TYPE_Pos), invert | type);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Get the WakeUp Type of External WakeUp PINx.
  * @note   Warning: only one pin can be passed as parameter.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | INVERT                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  EXT_WKUP_CTL | TYPE
  *
  * @param  wakeup_pin This parameter can be one of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_LOW
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_HIGH
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_RISING
  *         @arg @ref LL_PWR_EXTWKUP_TYPE_FALLING
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_ext_wakeup_type(uint32_t wakeup_pin)
{
    return ((uint32_t)(READ_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_INVERT | AON_EXT_WKUP_CTL_TYPE) >> POSITION_VAL(wakeup_pin)));
}

/**
  * @brief  Set the 32 bits AON Sleep Timer Value to WakeUp the MCU from DeepSleep Mode.
  * @note   After the value was set, use @arg @ref LL_PWR_CMD_32_TIMER_LD command to
  *         load the configuration into Power State Controller.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TIMER_VALUE          | PWR_CTL_TIMER_32B                 |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  value  32 bits count value loaded into the t32bit_timer
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_sleep_timer_value(uint32_t value)
{
    WRITE_REG(AON->TIMER_VALUE, value);
}

/**
  * @brief  Get the 32 bit AON Sleep Timer Value to WakeUp the MCU from DeepSleep Mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TIMER_VALUE          | PWR_CTL_TIMER_32B                 |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval 32 bit AON Timer Count Value
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_sleep_timer_value(void)
{
    return READ_REG(AON->TIMER_VALUE);
}

/**
  * @brief  Enable the SMC WakeUp Request.
  * @note   Once this is set up, MCU will wake up SMC, and this bit need to be cleared by MCU.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | SMC_WAKEUP_REQ                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_smc_wakeup_req(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_SMC_WAKEUP_REQ);
}

/**
  * @brief  Disable the SMC WakeUp Request.
  * @note   This function is used to clear SMC WakeUp Request.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | SMC_WAKEUP_REQ                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_smc_wakeup_req(void)
{
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_SMC_WAKEUP_REQ);
}

/**
  * @brief  Check if the SMC WakeUp Request was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | SMC_WAKEUP_REQ                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_smc_wakeup_req(void)
{
    return (READ_BITS(AON->PWR_RET01, AON_PWR_REG01_SMC_WAKEUP_REQ) == AON_PWR_REG01_SMC_WAKEUP_REQ);
}

/**
  * @brief  Set the DPAD LE value during sleep and after wake up.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MEM_N_SLP_CTL        | DPAD_LE_SLP_VAL                   |
  *  +----------------------+-----------------------------------+
  *  | MEM_N_SLP_CTL        | DPAD_LE_WKUP_VAL                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  *
  * @param  sleep  This parameter can be one of the following values:
  *         @arg @ref LL_PWR_DPAD_LE_OFF
  *         @arg @ref LL_PWR_DPAD_LE_ON
  * @param  wakeup This parameter can be one of the following values:
  *         @arg @ref LL_PWR_DPAD_LE_OFF
  *         @arg @ref LL_PWR_DPAD_LE_ON
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_dpad_le_value(uint32_t sleep, uint32_t wakeup)
{
    MODIFY_REG(AON->MEM_N_SLP_CTL, AON_MEM_CTL_DPAD_LE_SLP_VAL, (sleep << AON_MEM_CTL_DPAD_LE_SLP_VAL_Pos));
    MODIFY_REG(AON->MEM_N_SLP_CTL, AON_MEM_CTL_DPAD_LE_WKUP_VAL, (wakeup << AON_MEM_CTL_DPAD_LE_WKUP_VAL_Pos));
}

/**
  * @brief  Request to excute the Power State Controller Command.
  * @note   The PSC command can only be excuted when Power State Controller is not in busy state.
  *         Use @ref ll_pwr_is_active_flag_psc_cmd_busy() to check the busy status, and make sure
  *         the last command has been finished.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PSC_CMD_OPC          | OPCODE                            |
  *  +----------------------+-----------------------------------+
  *  | PSC_CMD              | MCU_PWR_REQ                       |
  *  +----------------------+-----------------------------------+
  * \endrst

  *
  * @param  command This parameter can be one of the following values:
  *         @arg @ref LL_PWR_CMD_LOOPBACK
  *         @arg @ref LL_PWR_CMD_EF_DIR_ON
  *         @arg @ref LL_PWR_CMD_32_TIMER_LD
  *         @arg @ref LL_PWR_CMD_DEEP_SLEEP
  *         @arg @ref LL_PWR_CMD_EF_DIR_OFF
  *         @arg @ref LL_PWR_CMD_EXT_CLK
  *         @arg @ref LL_PWR_CMD_RNG_CLK
  *         @arg @ref LL_PWR_CMD_RTC_CLK
  *         @arg @ref LL_PWR_CMD_LD_MEM_SLP_CFG
  *         @arg @ref LL_PWR_CMD_LD_MEM_WKUP_CFG
  *         @arg @ref LL_PWR_CMD_DPAD_LE_HI (*)
  *         @arg @ref LL_PWR_CMD_DPAD_LE_LO (*)
  *         @arg @ref LL_PWR_CMD_SLP_TIMER_MODE_NORMAL (*)
  *         @arg @ref LL_PWR_CMD_SLP_TIMER_MODE_SINGLE (*)
  *         @arg @ref LL_PWR_CMD_SLP_TIMER_MODE_RELOAD (*)
  *         @arg @ref LL_PWR_CMD_SLP_TIMER_MODE_DISABLE (*)
  *
  *         (*) Not available in A0 and B0
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_req_excute_psc_command(uint32_t command)
{
    WRITE_REG(AON->PSC_CMD_OPC, (uint8_t)command);
    SET_BITS(AON->PSC_CMD, AON_PSC_CMD_MCU_PWR_REQ);
}

/** @} */

/** @addtogroup PWR_LL_EF_Communication_Configuration BLE Communication timer and core configuration function
  * @{
  */

/**
  * @brief  Enable the Communication Timer Reset.
  * @note   Comm timer can be reset when all ble connection were disconnected and
  *         MCU was ready to enter into deepsleep mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | COMM_TIMER_RST_N                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_timer_reset(void)
{
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_COMM_TIMER_RST_N);
}

/**
  * @brief  Disable the Communication Timer Reset, and set Communication Timer to running state.
  * @note   After powered up, Comm Timer need to enter into running mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | COMM_TIMER_RST_N                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_comm_timer_reset(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_COMM_TIMER_RST_N);
}

/**
  * @brief  Check if the Communication Timer Reset was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | COMM_TIMER_RST_N                  |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_comm_timer_reset(void)
{
    return ((uint32_t)(READ_BITS(AON->PWR_RET01, AON_PWR_REG01_COMM_TIMER_RST_N) == 0x0U));
}

/**
  * @brief  Enable the Communication Core Reset.
  * @note   Comm Core can be reset when all ble connection were disconnected and
  *         MCU was ready to enter into deepsleep mode, and When COMM_CORE_RST_N
  *         is 0, the ble is held in reset.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | COMM_CORE_RST_N                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_core_reset(void)
{
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_COMM_CORE_RST_N);
}

/**
  * @brief  Disable the Communication Core Reset, and set Communication Core to running state.
  * @note   After powered up, Comm Core need to enter into running mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | COMM_CORE_RST_N                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_comm_core_reset(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_COMM_CORE_RST_N);
}

/**
  * @brief  Check if the Communication Core Reset was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PWR_RET01            | COMM_CORE_RST_N                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_comm_core_reset(void)
{
    return ((uint32_t)(READ_BITS(AON->PWR_RET01, AON_PWR_REG01_COMM_CORE_RST_N) == 0x0U));
}

/**
  * @brief Enable the Communication Timer Power, the Communication Timer will be Powered Up.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ISO_EN_PD_COMM_TIMER              |
  *  +----------------------+-----------------------------------+
  *  | CALENDAR_TIMER_CTL   | PWR_EN_PD_COMM_TIMER              |
  *  +----------------------+-----------------------------------+
  * \endrst

  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_timer_power(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_ISO_EN_PD_COMM_TIMER);
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_TIMER);
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_ISO_EN_PD_COMM_TIMER);
}

/**
  * @brief  Disable the Communication Timer Power, the Communication Timer will be Powered Down.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ISO_EN_PD_COMM_TIMER              |
  *  +----------------------+-----------------------------------+
  *  | CALENDAR_TIMER_CTL   | PWR_EN_PD_COMM_TIMER              |
  *  +----------------------+-----------------------------------+
  * \endrst

  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_comm_timer_power(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_TIMER);
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_ISO_EN_PD_COMM_TIMER);
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_TIMER);
}

/**
  * @brief Check if the Communication Timer Power was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ISO_EN_PD_COMM_TIMER              |
  *  +----------------------+-----------------------------------+
  *  | CALENDAR_TIMER_CTL   | PWR_EN_PD_COMM_TIMER              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_comm_timer_power(void)
{
    return ((uint32_t)(READ_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_TIMER) == AON_PWR_REG01_PWR_EN_PD_COMM_TIMER));
}

/**
  * @brief Enable the Communication Core Power, the Communication Core will be Powered Up.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ISO_EN_PD_COMM_CORE               |
  *  +----------------------+-----------------------------------+
  *  | CALENDAR_TIMER_CTL   | PWR_EN_PD_COMM_CORE               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_core_power(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_CORE);
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_ISO_EN_PD_COMM_CORE);
}

/**
  * @brief  Disable the Communication Core Power, the Communication Core will be Powered Down.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ISO_EN_PD_COMM_CORE               |
  *  +----------------------+-----------------------------------+
  *  | CALENDAR_TIMER_CTL   | PWR_EN_PD_COMM_CORE               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_comm_core_power(void)
{
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_CORE);
    SET_BITS(AON->PWR_RET01, AON_PWR_REG01_ISO_EN_PD_COMM_CORE);
    CLEAR_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_CORE);
}

/**
  * @brief Check if the Communication Core Power was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ISO_EN_PD_COMM_CORE               |
  *  +----------------------+-----------------------------------+
  *  | CALENDAR_TIMER_CTL   | PWR_EN_PD_COMM_CORE               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_comm_core_power(void)
{
    return ((uint32_t)(READ_BITS(AON->PWR_RET01, AON_PWR_REG01_PWR_EN_PD_COMM_CORE) == AON_PWR_REG01_PWR_EN_PD_COMM_CORE));
}

/**
  * @brief  Select which timer value to read
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PAD_CTL1             | TIMER_READ_SEL                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  select This parameter can be one of the following values:
  *         @arg @ref LL_PWR_TIMER_READ_SEL_CAL_TIMER
  *         @arg @ref LL_PWR_TIMER_READ_SEL_AON_WDT
  *         @arg @ref LL_PWR_TIMER_READ_SEL_SLP_TIMER
  *         @arg @ref LL_PWR_TIMER_READ_SEL_CAL_ALARM
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_timer_read_select(uint32_t select)
{
    GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(AON->AON_PAD_CTL1, AON_PAD_CTL1_TIMER_READ_SEL, select);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Get which timer value was selected to read.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PAD_CTL1             | TIMER_READ_SEL                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_TIMER_READ_SEL_CAL_TIMER
  *         @arg @ref LL_PWR_TIMER_READ_SEL_AON_WDT
  *         @arg @ref LL_PWR_TIMER_READ_SEL_SLP_TIMER
  *         @arg @ref LL_PWR_TIMER_READ_SEL_CAL_ALARM
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_timer_read_select(void)
{
    return ((uint32_t)READ_BITS(AON->AON_PAD_CTL1, AON_PAD_CTL1_TIMER_READ_SEL));
}

/**
  * @brief  Get current timer value based on the selection.
  * @note   Please read multiple times until get a stable value.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PAD_CTL1             | TIMER_READ_SEL                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_TIMER_READ_SEL_CAL_TIMER
  *         @arg @ref LL_PWR_TIMER_READ_SEL_AON_WDT
  *         @arg @ref LL_PWR_TIMER_READ_SEL_SLP_TIMER
  *         @arg @ref LL_PWR_TIMER_READ_SEL_CAL_ALARM
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_timer_read_value(void)
{
    return ((uint32_t)READ_REG(AON->TIMER_VAL));
}

/**
  * @brief Enable high frequency crystal oscillator sleep mode, and diable OSC.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_OSC_SLEEP_EN      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_osc_sleep(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN);
    GLOBAL_EXCEPTION_ENABLE();
}


/**
  * @brief Disable high frequency crystal oscillator sleep mode.
  * @note  Switch OSC from sleep mode into normal active mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_OSC_SLEEP_EN      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_osc_sleep(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Check if the OSC sleep mode was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_OSC_SLEEP_EN      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_osc_sleep(void)
{
    return ((uint32_t)(READ_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN) == AON_COMM_DEEPSLCNTL_OSC_SLEEP_EN));
}

/**
  * @brief Enable Radio sleep mode, and disable Radio module.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_RADIO_SLEEP_EN    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_radio_sleep(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Disable Radio sleep mode.
  * @note  Switch Radio from sleep mode into normal active mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_RADIO_SLEEP_EN    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_radio_sleep(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Check if the Radio sleep mode was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_RADIO_SLEEP_EN    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_radio_sleep(void)
{
    return ((uint32_t)(READ_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN) == AON_COMM_DEEPSLCNTL_RADIO_SLEEP_EN));
}

/**
  * @brief Enable Communication Core Deep Sleep Mode.
  * @note  This bit is reset on DEEP_SLEEP_STAT falling edge.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_DEEP_SLEEP_ON     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_core_deep_sleep(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Disable Communication Core Deep Sleep Mode.
  * @note  Switch Communication Core from sleep mode into normal active mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_DEEP_SLEEP_ON     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_comm_core_deep_sleep(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Check if the Communication Core Deep Sleep Mode was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_DEEP_SLEEP_ON     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_comm_core_deep_sleep(void)
{
    return ((uint32_t)(READ_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON) == AON_COMM_DEEPSLCNTL_DEEP_SLEEP_ON));
}

/**
  * @brief Enable Wake Up Request from Software.
  * @note  Applies when system is in Deep Sleep Mode. It wakes up the Communication Core
  *        when written with a 1. No action happens if it is written with 0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_soft_wakeup_req(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Check if the Wake Up Request was enabled or disabled.
  * @note  Resets at 0 means request action is performed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_soft_wakeup_req(void)
{
    return ((uint32_t)(READ_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ) == AON_COMM_DEEPSLCNTL_SOFT_WAKEUP_REQ));
}

/**
  * @brief Enable Communication Core external wakeup.
  * @note  After this configuration, Communication Core can be woken up by external wake-up
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_EXTWKUPDSB        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_enable_comm_core_ext_wakeup(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_EXTWKUPDSB);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Disable Communication Core external wakeup.
  * @note  After this configuration, Communication Core cannot be woken up by external wake-up
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_EXTWKUPDSB        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_comm_core_ext_wakeup(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_EXTWKUPDSB);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief Check if the Communication Core external wakeup was enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_EXTWKUPDSB        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_enabled_comm_core_ext_wakeup(void)
{
    return ((uint32_t)(READ_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_EXTWKUPDSB) == 0x0U));
}

/**
  * @brief  Set the time in low_power_clk clock cycles to spend in Deep Sleep Mode before waking-up the device.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | COMM_TMR_DEEPSLWKUP  | AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME|
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  time  32 bit clock cycles loaded into the AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_comm_core_wakeup_time(uint32_t time)
{
    WRITE_REG(AON->PWR_RET28, time);
}

/**
  * @brief  Get the time in low_power_clk clock cycles to spend in Deep Sleep Mode before waking-up the device.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | COMM_TMR_DEEPSLWKUP  | AON_COMM_TMR_DEEPSLWKUP_DEEPSLTIME|
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Clock cycles to spend in Deep Sleep Mode before waking-up the device
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_comm_wakeup_time(void)
{
    return ((uint32_t)READ_REG(AON->PWR_RET28));
}


/**
  * @brief  Get the actual duration of the last deep sleep phase measured in low_power_clk clock cycle.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | COMM_TMR_DEEPSLPSTAT | DEEPSLDUR                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Sleep duration
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_comm_sleep_duration(void)
{
    return ((uint32_t)READ_REG(MCU_SUB->COMM_TMR_DEEPSLPSTAT));
}

/**
  * @brief  Set the wakeup timing in low_power_clk clock cycles to spend when waking-up the device.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | COMM_TMR_ENBPRESET   | TWEXT                             |
  *  +----------------------+-----------------------------------+
  *  | COMM_TMR_ENBPRESET   | TWOSC                             |
  *  +----------------------+-----------------------------------+
  *  | COMM_TMR_ENBPRESET   | TWRM                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  twext   Time in low power oscillator cycles allowed for stabilization of the high frequency
  *                 oscillator following an external wake–up request (signal wakeup_req).
  * @param  twosc   Time in low power oscillator cycles allowed for stabilization of the high frequency
  *                 oscillator when the deep–sleep mode has been left due to sleep–timer expiry.
  * @param  twrm    Time in low power oscillator cycles allowed for the radio module to leave low–power mode.
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_set_comm_wakeup_timing(uint32_t twext, uint32_t twosc, uint32_t twrm)
{
    WRITE_REG(AON->PWR_RET29, (twext << AON_COMM_TMR_ENBPRESET_TWEXT_Pos) |
                              (twosc << AON_COMM_TMR_ENBPRESET_TWOSC_Pos) |
                              (twrm  << AON_COMM_TMR_ENBPRESET_TWRM_Pos));
}


/**
  * @brief  Read the wakeup timing in low_power_clk clock cycles to spend when waking-up the device.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | COMM_TMR_ENBPRESET   | TWEXT                             |
  *  +----------------------+-----------------------------------+
  *  | COMM_TMR_ENBPRESET   | TWOSC                             |
  *  +----------------------+-----------------------------------+
  *  | COMM_TMR_ENBPRESET   | TWRM                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  *
  * @retval COMM_TMR_ENBPRESET Register value
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_read_comm_wakeup_timing(void)
{
    return ((uint32_t)READ_REG(AON->PWR_RET29));
}

/**
  * @brief  Read the Twosc of the wakeup timing in low_power_clk clock cycles to spend when waking-up the device.
  *
  * @retval TWOSC value
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_read_comm_wakeup_timing_twosc(void)
{
    return ((((uint32_t)READ_REG(AON->PWR_RET29) & AON_COMM_TMR_ENBPRESET_TWOSC_Msk)) >> AON_COMM_TMR_ENBPRESET_TWOSC_Pos);
}


/** @} */

/** @defgroup PWR_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Get the External Wake Up Status.
  * @note   0 means not waked up and 1 means waked up.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | EXT_WKUP_STATUS                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_get_ext_wakeup_status(void)
{
    return ((uint32_t)(READ_BITS(AON->SLP_EVENT, AON_SLP_EVENT_EXT_WKUP_STATUS) >> AON_SLP_EVENT_EXT_WKUP_STATUS_Pos) & \
            (uint32_t)(READ_BITS(AON->EXT_WKUP_CTL, LL_PWR_EXTWKUP_PIN_ALL)));
}

/**
  * @brief  Clear the External Wake Up Status.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | EXT_WKUP_STATUS                   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  wakeup_pin This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_EXTWKUP_PIN0
  *         @arg @ref LL_PWR_EXTWKUP_PIN1
  *         @arg @ref LL_PWR_EXTWKUP_PIN2
  *         @arg @ref LL_PWR_EXTWKUP_PIN3
  *         @arg @ref LL_PWR_EXTWKUP_PIN4
  *         @arg @ref LL_PWR_EXTWKUP_PIN5
  *         @arg @ref LL_PWR_EXTWKUP_PIN_ALL
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_clear_ext_wakeup_status(uint32_t wakeup_pin)
{
    GLOBAL_EXCEPTION_DISABLE();
    WRITE_REG(AON->SLP_EVENT, ~(wakeup_pin << AON_SLP_EVENT_EXT_WKUP_STATUS_Pos));
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Clear the Event that triggered the DeepSleep WakeUp.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | SMCOSCEN_EVENT                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  SLP_EVENT | TIMER_EVENT
  *  SLP_EVENT | EXT_WKUP_EVENT
  *  SLP_EVENT | WATCHDOG_EVENT
  *
  * @param  event This parameter can be a combination of the following values:
  *         @arg @ref LL_PWR_WKUP_EVENT_BLE
  *         @arg @ref LL_PWR_WKUP_EVENT_TIMER
  *         @arg @ref LL_PWR_WKUP_EVENT_EXT
  *         @arg @ref LL_PWR_WKUP_EVENT_BOD_FEDGE
  *         @arg @ref LL_PWR_WKUP_EVENT_MSIO_COMP
  *         @arg @ref LL_PWR_WKUP_EVENT_WDT
  *         @arg @ref LL_PWR_WKUP_EVENT_CALENDAR
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_clear_wakeup_event(uint32_t event)
{
    WRITE_REG(AON->SLP_EVENT, ~(event & LL_PWR_WKUP_EVENT_ALL));
}

/**
  * @brief  Indicate if the Power State Controller is in busy state.
  * @note   This is bit set 1 when the PSC_CMD_REQ[0] is set to 1, and will remain 1 until
  *         the PSC_CMD_OPC has been transferred to the PSC.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PSC_CMD              | MCU_PWR_BUSY                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_active_flag_psc_cmd_busy(void)
{
    return (READ_BITS(AON->PSC_CMD, AON_PSC_CMD_MCU_PWR_BUSY) == AON_PSC_CMD_MCU_PWR_BUSY);
}

/**
  * @brief  Indicate if the Communication Core is in Deep Sleep Mode.
  * @note   When Communication Core is in Deep Sleep Mode, only low_power_clk is running.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MSIO_PAD_CFG_1       | COMM_DEEPSLCNTL_DEEP_SLEEP_STAT   |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
SECTION_RAM_CODE __STATIC_INLINE uint32_t ll_pwr_is_active_flag_comm_deep_sleep_stat(void)
{
    return (READ_BITS(AON->MSIO_PAD_CFG_1, AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT) == AON_COMM_DEEPSLCNTL_DEEP_SLEEP_STAT);
}

/**
  * @brief  Disable cache function
  * @note  The cache should be closed before chip go to deepsleep.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL0                | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
SECTION_RAM_CODE __STATIC_INLINE void ll_pwr_disable_cache_module(void)
{
    SET_BITS(XQSPI->CACHE.CTRL0, XQSPI_CACHE_CTRL0_DIS);
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/** @} */

/** @} */
/** @} */

#endif /* defined(AON) */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_LL_PWR_H__ */

/** @} */

/** @} */

/** @} */
