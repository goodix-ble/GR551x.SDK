/**
 ****************************************************************************************
 *
 * @file gr55xx_hal_pwr.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWR HAL library.
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

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_PWR PWR
  * @brief PWR HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_PWR_H__
#define __GR55xx_HAL_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_pwr.h"
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup HAL_PWR_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_PWR_SLEEP_ELAPSED_HANDLER HAL PWR sleep elapsed handler define
  * @{
  */
  
/**
  * @brief   PWR Sleep Timer Elapsed callback
  */

typedef void (*pwr_slp_elapsed_handler_t)(void);

/** @} */


/** @defgroup HAL_PWR_CALLBACK_HANDLER PWR callback handle
  * @{
  */

/**
  * @brief PWR callback handle Structure definition
  */
typedef struct _hal_pwr_handler
{
    pwr_slp_elapsed_handler_t       pwr_slp_elapsed_hander;     /**< PWR sleep timer elapsed callback */
} hal_pwr_handler_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_PWR_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_WakeUp_Pins PWR WakeUp Pins
  * @{
  */
#define PWR_EXTWKUP_PIN0                LL_PWR_EXTWKUP_PIN0             /**< External wakeup pin 0 */
#define PWR_EXTWKUP_PIN1                LL_PWR_EXTWKUP_PIN1             /**< External wakeup pin 1 */
#define PWR_EXTWKUP_PIN2                LL_PWR_EXTWKUP_PIN2             /**< External wakeup pin 2 */
#define PWR_EXTWKUP_PIN3                LL_PWR_EXTWKUP_PIN3             /**< External wakeup pin 3 */
#define PWR_EXTWKUP_PIN4                LL_PWR_EXTWKUP_PIN4             /**< External wakeup pin 4 */
#define PWR_EXTWKUP_PIN5                LL_PWR_EXTWKUP_PIN5             /**< External wakeup pin 5 */
#define PWR_EXTWKUP_PIN6                LL_PWR_EXTWKUP_PIN6             /**< External wakeup pin 6 */
#define PWR_EXTWKUP_PIN7                LL_PWR_EXTWKUP_PIN7             /**< External wakeup pin 7 */
#define PWR_EXTWKUP_PIN_ALL             LL_PWR_EXTWKUP_PIN_ALL          /**< External wakeup pin 0 ~ 7 */
/** @} */

/** @defgroup PWR_WakeUp_Conditions  PWR Wakeup Condition
  * @{
  */
#define PWR_WKUP_COND_EXT               LL_PWR_WKUP_COND_EXT             /**< External wakeup: AON_GPIO   */
#define PWR_WKUP_COND_TIMER             LL_PWR_WKUP_COND_TIMER           /**< AON Timer wakeup            */
#define PWR_WKUP_COND_BLE               LL_PWR_WKUP_COND_BLE             /**< BLE wakeup                  */
#define PWR_WKUP_COND_CALENDAR          LL_PWR_WKUP_COND_CALENDAR        /**< Calendar wakeup             */
#define PWR_WKUP_COND_BOD_FEDGE         LL_PWR_WKUP_COND_BOD_FEDGE       /**< PMU Bod falling edge wakeup */
#define PWR_WKUP_COND_MSIO_COMP         LL_PWR_WKUP_COND_MSIO_COMP       /**< Msio comparator wakeup      */
#define PWR_WKUP_COND_ALL               LL_PWR_WKUP_COND_ALL             /**< All wakeup sources mask     */

/** @} */

/** @defgroup PWR_External_WakeUp_Type  PWR External Wakeup Type
  * @{
  */
#define PWR_EXTWKUP_TYPE_LOW            LL_PWR_EXTWKUP_TYPE_LOW         /**< Low level wakeup */
#define PWR_EXTWKUP_TYPE_HIGH           LL_PWR_EXTWKUP_TYPE_HIGH        /**< High level wakeup */
#define PWR_EXTWKUP_TYPE_RISING         LL_PWR_EXTWKUP_TYPE_RISING      /**< Rising edge wakeup */
#define PWR_EXTWKUP_TYPE_FALLING        LL_PWR_EXTWKUP_TYPE_FALLING     /**< Falling edge wakeup */
/** @} */

/** @defgroup PWR_Sleep_Timer_Mode  PWR Sleep Timer Mode
 * @{
 */
#define PWR_SLP_TIMER_MODE_NORMAL       0x0U    /**< Start counting after sleeping and disabled when waked up */
#define PWR_SLP_TIMER_MODE_SINGLE       0x1U    /**< Single mode(keep counting until finished) */
#define PWR_SLP_TIMER_MODE_RELOAD       0x2U    /**< Auto reload  */
#define PWR_SLP_TIMER_MODE_DISABLE      0x3U    /**< Disabled (used for reset mode) */
/** @} */

/** @defgroup PWR_Timer_Type  PWR Timer Type
 *  @note     Only available on GR5515_C and later versions.
 *  @{
 */
#define PWR_TIMER_TYPE_CAL_TIMER        LL_PWR_TIMER_READ_SEL_CAL_TIMER  /**< Calendar timer     */
#define PWR_TIMER_TYPE_AON_WDT          LL_PWR_TIMER_READ_SEL_AON_WDT    /**< AON watchdog timer */
#define PWR_TIMER_TYPE_SLP_TIMER        LL_PWR_TIMER_READ_SEL_SLP_TIMER  /**< Sleep timer        */
#define PWR_TIMER_TYPE_CAL_ALARM        LL_PWR_TIMER_READ_SEL_CAL_ALARM  /**< Calendar timer     */
/** @} */


/** @defgroup PWR_Memory_Power_State  Memory Power State
 * @{
 */
#define PWR_MEM_POWER_OFF               LL_PWR_MEM_POWER_OFF            /**< Power off */
#define PWR_MEM_POWER_FULL              LL_PWR_MEM_POWER_FULL           /**< Full power */
#define PWR_MEM_POWER_RETENTION         LL_PWR_MEM_POWER_RETENTION      /**< Power retention, low valtage mode */
/** @} */

/** @defgroup PWR_Communication_Power_State  Communication Power State
 * @{
 */
#define PWR_COMM_TIMER_POWER_DOWN       0x0U  /**< Power down communication timer */
#define PWR_COMM_TIMER_POWER_UP         0x1U  /**< Power on  communication timer */
#define PWR_COMM_CORE_POWER_DOWN        0x0U  /**< Power down communication core */
#define PWR_COMM_CORE_POWER_UP          0x1U  /**< Power on  communication core */
/** @} */

/** @defgroup PWR_Communication_Mode  Communication Mode
 * @{
 */
#define PWR_COMM_TIMER_MODE_RESET       0x0U  /**< Communication timer in reset mode */
#define PWR_COMM_TIMER_MODE_RUNNING     0x1U  /**< Communication timer in running mode */
#define PWR_COMM_CORE_MODE_RESET        0x0U  /**< Communication core in reset mode */
#define PWR_COMM_CORE_MODE_RUNNING      0x1U  /**< Communication core in running mode */
/** @} */

/** @defgroup PWR_Timeout_definition PWR Timeout definition
 * @{
  */
#define HAL_PWR_TIMEOUT_DEFAULT_VALUE ((uint32_t)0x000FFFFF)         /**< 0xFFFFF counts */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup  PWR_Private_Macros   PWR Private Macros
  * @{
  */

/**
  * @brief Check if PWR wakeup condition is valid.
  * @param __COND__ PWR wakeup condition.
  * @retval SET (__COND__ is valid) or RESET (__COND__ is invalid)
  */
#define IS_PWR_WAKEUP_CONDITION(__COND__)       ((((__COND__) & PWR_WKUP_COND_ALL) != 0x00U) &&\
                                                 (((__COND__) & ~PWR_WKUP_COND_ALL) == 0x00U))

/**
  * @brief Check if PWR external wakeup pin is valid.
  * @param __PIN__ PWR external wakeup pin.
  * @retval SET (__PIN__ is valid) or RESET (__PIN__ is invalid)
  */
#define IS_PWR_EXT_WAKEUP_PIN(__PIN__)          ((((__PIN__) & PWR_EXTWKUP_PIN_ALL) != 0x00U) &&\
                                                 (((__PIN__) & ~PWR_EXTWKUP_PIN_ALL) == 0x00U))

/**
  * @brief Check if PWR sleep timer mode is valid.
  * @param __MODE__ PWR sleep timer mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PWR_SLP_TIMER_MODE(__MODE__)         (((__MODE__) == PWR_SLP_TIMER_MODE_NORMAL) || \
                                                 ((__MODE__) == PWR_SLP_TIMER_MODE_SINGLE) || \
                                                 ((__MODE__) == PWR_SLP_TIMER_MODE_RELOAD) || \
                                                 ((__MODE__) == PWR_SLP_TIMER_MODE_DISABLE))

/**
  * @brief Check if PWR external wakeup type is valid.
  * @param __TYPE__ PWR external wakeup type.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_PWR_EXTWKUP_TYPE(__TYPE__)           (((__TYPE__) == PWR_EXTWKUP_TYPE_LOW)    || \
                                                 ((__TYPE__) == PWR_EXTWKUP_TYPE_HIGH)   || \
                                                 ((__TYPE__) == PWR_EXTWKUP_TYPE_RISING) || \
                                                 ((__TYPE__) == PWR_EXTWKUP_TYPE_FALLING))

/**
  * @brief Check if PWR memory block is valid.
  * @param __BLOCK__ PWR memory block.
  * @retval SET (__BLOCK__ is valid) or RESET (__BLOCK__ is invalid)
  */
#define IS_PWR_MEM_BLOCK(__BLOCK__)             ((((__BLOCK__) & PWR_MEM_ALL) != 0x00U) &&\
                                                 (((__BLOCK__) & ~PWR_MEM_ALL) == 0x00U))

/**
  * @brief Check if PWR memory power state is valid.
  * @param __STATE__ PWR memory power state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is invalid)
  */
#define IS_PWR_MEM_POWER_STAT(__STATE__)        (((__STATE__) == PWR_MEM_POWER_OFF)  || \
                                                 ((__STATE__) == PWR_MEM_POWER_FULL) || \
                                                 ((__STATE__) == PWR_MEM_POWER_RETENTION))

/**
  * @brief Check if PWR BLE communication timer power state is valid.
  * @param __STATE__ PWR BLE communication timer power state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is invalid)
  */
#define IS_PWR_COMM_TIMER_POWER_STAT(__STATE__) (((__STATE__) == PWR_COMM_TIMER_POWER_DOWN) || \
                                                 ((__STATE__) == PWR_COMM_TIMER_POWER_UP))

/**
  * @brief Check if PWR BLE communication core power state is valid.
  * @param __STATE__ PWR BLE communication core power state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is invalid)
  */
#define IS_PWR_COMM_CORE_POWER_STAT(__STATE__)  (((__STATE__) == PWR_COMM_CORE_POWER_DOWN) || \
                                                 ((__STATE__) == PWR_COMM_CORE_POWER_UP))

/**
  * @brief Check if PWR BLE communication timer mode is valid.
  * @param __MODE__ PWR BLE communication timer mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PWR_COMM_TIMER_MODE(__MODE__)        (((__MODE__) == PWR_COMM_TIMER_MODE_RESET) || \
                                                 ((__MODE__) == PWR_COMM_TIMER_MODE_RUNNING))

/**
  * @brief Check if PWR BLE communication core mode is valid.
  * @param __MODE__ PWR BLE communication core mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_PWR_COMM_CORE_MODE(__MODE__)         (((__MODE__) == PWR_COMM_CORE_MODE_RESET) || \
                                                 ((__MODE__) == PWR_COMM_CORE_MODE_RUNNING))

/**
  * @brief Check if PWR sleep timer type is valid.
  * @param __TYPE__ PWR sleep timer type.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_PWR_PWR_TIMER_TYPE(__TYPE__)         (((__TYPE__) == PWR_TIMER_TYPE_CAL_TIMER) || \
                                                 ((__TYPE__) == PWR_TIMER_TYPE_AON_WDT)   || \
                                                 ((__TYPE__) == PWR_TIMER_TYPE_SLP_TIMER) || \
                                                 ((__TYPE__) == PWR_TIMER_TYPE_CAL_ALARM))

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_PWR_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup PWR_Exported_Functions_Group1 Low Power mode configuration functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Set the DeepSleep WakeUp Condition
 * @param[in]  condition: This parameter can be a combination of the following values:
 *         @arg @ref PWR_WKUP_COND_EXT
 *         @arg @ref PWR_WKUP_COND_TIMER
 *         @arg @ref PWR_WKUP_COND_BLE
 *         @arg @ref PWR_WKUP_COND_CALENDAR
 *         @arg @ref PWR_WKUP_COND_BOD_FEDGE
 *         @arg @ref PWR_WKUP_COND_MSIO_COMP
 *         @arg @ref PWR_WKUP_COND_ALL
 * @note   When @ref PWR_WKUP_COND_EXT is set, use @ref hal_pwr_config_ext_wakeup() to configure wakeup pins and pin trigger type.
 *         When @ref PWR_WKUP_COND_TIMER is set, use @ref hal_pwr_config_timer_wakeup() to configure the time count to wakeup.
 *         When @ref PWR_WKUP_COND_ALL is set, use @ref hal_pwr_config_ext_wakeup() and @ref hal_pwr_config_timer_wakeup() to configure
 *         AON timer and External AON GPIO.
 ****************************************************************************************
 */
void hal_pwr_set_wakeup_condition(uint32_t condition);

/**
 ****************************************************************************************
 * @brief  Configure the AON Sleep Timer mode and count used to wakeup MCU.
 * @param[in]  timer_mode: Specifies the sleep timer mode.
 *         This parameter can be a combination of the following values:
 *         @arg @ref PWR_SLP_TIMER_MODE_NORMAL
 *         @arg @ref PWR_SLP_TIMER_MODE_SINGLE
 *         @arg @ref PWR_SLP_TIMER_MODE_RELOAD
 *         @arg @ref PWR_SLP_TIMER_MODE_DISABLE
 * @param[in]  load_count: Count value of the AON Sleep Timer.
 * @note   The sleep clock of AON Timer is 32 KHz.
 ****************************************************************************************
 */
void hal_pwr_config_timer_wakeup(uint8_t timer_mode, uint32_t load_count);

/**
 ****************************************************************************************
 * @brief  Configure the External AON GPIO pins and pin trigger type that is used to wakeup MCU.
 * @param[in]  ext_wakeup_pinx: This parameter can be a combination of the following values:
 *         @arg @ref PWR_EXTWKUP_PIN0
 *         @arg @ref PWR_EXTWKUP_PIN1
 *         @arg @ref PWR_EXTWKUP_PIN2
 *         @arg @ref PWR_EXTWKUP_PIN3
 *         @arg @ref PWR_EXTWKUP_PIN4
 *         @arg @ref PWR_EXTWKUP_PIN5
 *         @arg @ref PWR_EXTWKUP_PIN_ALL
 * @param[in]  ext_wakeup_type: This parameter can be a combination of the following values:
 *         @arg @ref PWR_EXTWKUP_TYPE_LOW
 *         @arg @ref PWR_EXTWKUP_TYPE_HIGH
 *         @arg @ref PWR_EXTWKUP_TYPE_RISING
 *         @arg @ref PWR_EXTWKUP_TYPE_FALLING
 * @note   When the level of any selected GPIO pin changes in accordance with the set
 *         trigger type, MCU will be waked up from DeepSleep mode.
 ****************************************************************************************
 */
void hal_pwr_config_ext_wakeup(uint32_t ext_wakeup_pinx, uint32_t ext_wakeup_type);

/**
 ****************************************************************************************
 * @brief  Disable the interrupt wake-up function of the specified AON GPIO pin.
 * @param[in]  disable_wakeup_pinx: This parameter can be a combination of the following values:
 *         @arg @ref PWR_EXTWKUP_PIN0
 *         @arg @ref PWR_EXTWKUP_PIN1
 *         @arg @ref PWR_EXTWKUP_PIN2
 *         @arg @ref PWR_EXTWKUP_PIN3
 *         @arg @ref PWR_EXTWKUP_PIN4
 *         @arg @ref PWR_EXTWKUP_PIN5
 *         @arg @ref PWR_EXTWKUP_PIN_ALL
 ****************************************************************************************
 */
void hal_pwr_disable_ext_wakeup(uint32_t disable_wakeup_pinx);

/**
 ****************************************************************************************
 * @brief Enters DeepSleep mode.
 * @note  In DeepSleep mode, all I/O pins keep the same state as in Run mode.
 ****************************************************************************************
*/
void hal_pwr_enter_chip_deepsleep(void);

/** @} */

/** @addtogroup PWR_Exported_Functions_Group2 BLE Communication timer and core configuration function
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Set the power state of communication timer and communication core in running mode.
 * @param[in]  timer_power_state: This parameter can be one of the following values:
 *         @arg @ref PWR_COMM_TIMER_POWER_UP
 *         @arg @ref PWR_COMM_TIMER_POWER_DOWN
 * @param[in]  core_power_state: This parameter can be one of the following values:
 *         @arg @ref PWR_COMM_CORE_POWER_UP
 *         @arg @ref PWR_COMM_CORE_POWER_DOWN
 ****************************************************************************************
 */
void hal_pwr_set_comm_power(uint32_t timer_power_state, uint32_t core_power_state);

/**
 ****************************************************************************************
 * @brief  Set the work mode of communication timer and communication core.
 * @param[in]  timer_mode: This parameter can be one of the following values:
 *         @arg @ref PWR_COMM_TIMER_MODE_RESET
 *         @arg @ref PWR_COMM_TIMER_MODE_RUNNING
 * @param[in]  core_mode: This parameter can be one of the following values:
 *         @arg @ref PWR_COMM_CORE_MODE_RESET
 *         @arg @ref PWR_COMM_CORE_MODE_RUNNING
 ****************************************************************************************
 */
void hal_pwr_set_comm_mode(uint32_t timer_mode, uint32_t core_mode);

/**
 ****************************************************************************************
 * @brief  Get the current value of specified timer.
 * @note   Only available on GR5515_C and later versions.
 * @param[in]  timer_type: This parameter can be one of the following values:
 *         @arg @ref PWR_TIMER_TYPE_CAL_TIMER
 *         @arg @ref PWR_TIMER_TYPE_AON_WDT
 *         @arg @ref PWR_TIMER_TYPE_SLP_TIMER
 *         @arg @ref PWR_TIMER_TYPE_CAL_ALARM
 * @param[out] p_value: Pointer to an integer storing current value
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_pwr_get_timer_current_value(uint32_t timer_type, uint32_t *p_value);

/** @} */

/** @addtogroup PWR_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Handle PWR Sleep Timer interrupt request.
 * @note   Only available on GR5515_C and later versions.
 ****************************************************************************************
 */
void hal_pwr_sleep_timer_irq_handler(void);


/**
 ****************************************************************************************
 * @brief  PWR Sleep Timer Elapsed callback.
 * @note   Only available on GR5515_C and later versions.
 *         This function should not be modified. When the callback is needed,
 *         the hal_pwr_sleep_timer_elapsed_callback can be implemented in the user file.
 ****************************************************************************************
 */
void hal_pwr_sleep_timer_elapsed_callback(void);



/** @} */

/** @} */

#ifdef __cplusplus
}
#endif


#endif /* __GR55xx_HAL_PWR_H__ */

/** @} */

/** @} */

/** @} */
