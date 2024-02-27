/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal.h
 * @author  BLE Driver Team
 * @brief   This file contains all the functions prototypes for the HAL
 *          module driver.
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

/** @defgroup HAL_HAL HAL
  * @brief HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_H__
#define __GR55xx_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"
#include "gr55xx_hal_conf.h"
#include "gr55xx_delay.h"

/** @addtogroup HAL_HAL_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_HAL_Callback Callback
  * @{
  */

/**
  * @brief HAL_HAL Callback function definition
  */

typedef struct _hal_callback
{
    void (*msp_init)(void);         /**< HAL init MSP callback                  */
    void (*msp_deinit)(void);       /**< HAL de-init MSP callback               */
} hal_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_MACRO Defines
  * @{
  */

/* Private macros ------------------------------------------------------------*/
/* Exported macros ------------------------------------------------------------*/
/** @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */

/** @brief compare if a > b
 *  @sa CO_MAX
 */
#define CO_MAX(a,b) ((a) > (b) ? (a) : (b))

/** @brief Disable BLE_IRQn and BLESLP_IRQn.
 *  @sa BLE_INT_DISABLE
 */
#define BLE_INT_DISABLE()                                                                            \
do {                                                                                                 \
    volatile uint32_t __ble_l_irq_rest = __get_PRIMASK();                                            \
    volatile bool __ble_int_status = NVIC_GetEnableIRQ(BLE_IRQn) || NVIC_GetEnableIRQ(BLESLP_IRQn);  \
    __set_PRIMASK(1);                                                                                \
    if (__ble_int_status)                                                                            \
    {                                                                                                \
        NVIC_DisableIRQ(BLE_IRQn);                                                                   \
        NVIC_DisableIRQ(BLESLP_IRQn);                                                                \
    }                                                                                                \
    __set_PRIMASK(__ble_l_irq_rest);

/** @brief Restore BLE_IRQn and BLESLP_IRQn.
 *  @sa BLE_INT_RESTORE
 */
#define BLE_INT_RESTORE()                                                                            \
    __ble_l_irq_rest = __get_PRIMASK();                                                              \
    __set_PRIMASK(1);                                                                                \
    if (__ble_int_status)                                                                            \
    {                                                                                                \
        NVIC_EnableIRQ(BLE_IRQn);                                                                    \
        NVIC_EnableIRQ(BLESLP_IRQn);                                                                 \
    }                                                                                                \
    __set_PRIMASK(__ble_l_irq_rest);                                                                 \
} while(0)

/** @brief Disable interrupts globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE macro
 * since this last one will close the brace that the current macro opens. This
 * means that both macros must be located at the same scope level.
 */
#define GLOBAL_INT_DISABLE()                                                 \
do {                                                                         \
    volatile uint32_t __nvic_iser0 = 0xFFFFFFFF;                             \
    volatile uint32_t __nvic_iser1 = 0xFFFFFFFF;                             \
    volatile uint32_t __ret_pri = __get_PRIMASK();                           \
    __set_PRIMASK(1);                                                        \
    if( (NVIC->ICER[0] != 0xFFFFFFFF) ||  (NVIC->ICER[1] != 0xFFFFFFFF) )    \
    {                                                                        \
        __nvic_iser0 = NVIC->ISER[0];                                        \
        __nvic_iser1 = NVIC->ISER[1];                                        \
        NVIC->ICER[0] = 0xFFFFFFFF;                                          \
        NVIC->ICER[1] = 0xFFFFFFFF;                                          \
    }                                                                        \
    __set_PRIMASK(__ret_pri);                                                \
    __DSB();                                                                 \
    __ISB();                                                                 \

/** @brief Restore external interrupts(Exception Type: 16~255) from the previous disable.
 *  @sa GLOBAL_INT_RESTORE
 */
#define GLOBAL_INT_RESTORE()                                                 \
    __ret_pri = __get_PRIMASK();                                             \
    __set_PRIMASK(1);                                                        \
    if( (__nvic_iser0 != 0xFFFFFFFF) || (__nvic_iser1 != 0xFFFFFFFF) )       \
    {                                                                        \
        NVIC->ISER[0] = __nvic_iser0;                                        \
        NVIC->ISER[1] = __nvic_iser1;                                        \
    }                                                                        \
    __set_PRIMASK(__ret_pri);                                                \
} while(0)

/** @brief Disable external interrupts with a priority lower than IRQn_Type in the system.
 * This macro must be used in conjunction with the @ref LOCAL_INT_RESTORE macro
 * since this last one will close the brace that the current macro opens. This
 * means that both macros must be located at the same scope level.
 */
#define LOCAL_INT_DISABLE(IRQn_Type)                         \
do {                                                         \
    uint32_t __l_irq_rest = __get_BASEPRI();                 \
    __set_BASEPRI(NVIC_GetPriority(IRQn_Type) +              \
                 (1 << (NVIC_GetPriorityGrouping() + 1)));   \

/** @brief Restore external interrupts(apart from the BLE) from the previous disable.
 *  @sa EXP_BLE_INT_RESTORE
 */
#define LOCAL_INT_RESTORE()                                  \
    __set_BASEPRI(__l_irq_rest);                             \
} while(0)


/** @brief Check if the program is running on the FPGA platform.
 */
#define CHECK_IS_ON_FPGA()                (AON->FPGA_CTRL & AON_REG_FPGA_CTRL_EXIST)

#define SYSTICK_RELOAD_VALUE              (SysTick->LOAD)  /**< SysTick Reload value. */
#define SYSTICK_CURRENT_VALUE             (SysTick->VAL)   /**< SysTick Current value. */

/** @} */

/** @} */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_HAL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup HAL_Exported_Functions_Group1 Initialization and De-initialization Functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the Flash interface, the NVIC allocation and initial clock
          configuration. It also initializes the source of time base when timeout
          is needed.
      (+) De-initialize common part of the HAL.
      (+) Configure The time base source to have 1ms time base with a dedicated
          Tick interrupt priority.
        (++) SysTick timer is used by default as source of time base, but user can
             eventually implement his or her proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept as 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
        (++) Time base configuration function (hal_init_tick()) is called automatically
             at the beginning of the program after reset by hal_init().
        (++) Source of time base is configured  to generate interrupts at regular
             time intervals. Care must be taken if hal_delay() is called from a
             peripheral ISR process, the Tick interrupt line must have higher priority
            (numerically lower) than the peripheral interrupt. Otherwise the caller
            ISR process will be blocked.
       (++) Functions affecting time base configurations are declared as __Weak
            to make  override possible  in case of other  implementations in user file.

@endverbatim
 * @{
 */

/**
 ****************************************************************************************
 * @brief  This function configures time base source, NVIC and Low level hardware.
 *
 * @note   This function is called at the beginning of program after reset and before
 *         the clock configuration.
 *         The SysTick configuration is based on AHB clock and the NVIC configuration
 *         is set to Priority group 4.
 *         When the time base configuration is done, time base tick starts incrementing.
 *         In the default implementation, SysTick is used as source of time base.
 *         The tick variable is incremented each 1ms in its ISR.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_init(void);

/**
 ****************************************************************************************
 * @brief  This function de-initializes common part of the HAL and stops the source
 *         of time base.
 *
 * @note   This function is optional.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_deinit(void);

/**
 ****************************************************************************************
 * @brief  Initialize the MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_msp_init could be implemented in the user file.
 ****************************************************************************************
 */
void hal_msp_init(void);

/**
 ****************************************************************************************
 * @brief  De-initialize the MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_msp_deinit could be implemented in the user file.
 ****************************************************************************************
 */
void hal_msp_deinit(void);

/**
 ****************************************************************************************
 * @brief  This function configures the source of the time base.
 *
 * @param[in]  tick_priority: Tick interrupt priority.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_init_tick (uint32_t tick_priority);

/** @} */

/** @addtogroup HAL_Exported_Functions_Group2 HAL Control functions
 *  @brief    HAL Control functions
 *
@verbatim
 ===============================================================================
                      ##### HAL Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Suspend the time base source interrupt
      (+) Resume the time base source interrupt
      (+) Get the HAL API driver version

@endverbatim
 * @{
 */

/**
 ****************************************************************************************
 * @brief  Suspend Tick increment.
 *
 * @note   In the default implementation , SysTick timer is the source of time base. It is
 *         used to generate interrupts at regular time intervals. Once hal_suspend_tick()
 *         is called, the SysTick interrupt will be disabled so Tick increment
 *         is suspended.
 *         This function is declared as __WEAK to be overwritten in case of other
 *         implementations  in user file.
 ****************************************************************************************
 */
void hal_suspend_tick(void);

/**
 ****************************************************************************************
 * @brief  Resume Tick increment.
 *
 * @note   In the default implementation , SysTick timer is the source of time base. It is
 *         used to generate interrupts at regular time intervals. Once hal_resume_tick()
 *         is called, the SysTick interrupt will be enabled so Tick increment
 *         is resumed.
 *         The function is declared as __WEAK to be overwritten in case of other
 *         implementations  in user file.
 ****************************************************************************************
 */
void hal_resume_tick(void);

/**
 ****************************************************************************************
 * @brief  This function returns the HAL revision
 *
 * @return version: 0xXYZR (8 bits for each decimal, R for RC)
 ****************************************************************************************
 */
uint32_t hal_get_hal_version(void);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_H__ */

/** @} */

/** @} */

/** @} */
