/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_sleep_timer.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of sleep timer HAL library.
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

/** @defgroup HAL_SLEEP_TIMER SLEEP_TIMER
  * @brief SLEEP TIMER HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_SLEEP_TIMER_H__
#define __GR55xx_HAL_SLEEP_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_SLEEP_TIMER_FUNCTIONS Functions
  * @{
  */

/**
****************************************************************************************
* @brief  Configure the AON Sleep Timer mode, count and start used to wakeup MCU.
* @param[in]  mode: Specifies the sleep timer mode.
*             This parameter can be a combination of the following values:
*             @arg @ref PWR_SLP_TIMER_MODE_NORMAL
*             @arg @ref PWR_SLP_TIMER_MODE_SINGLE
*             @arg @ref PWR_SLP_TIMER_MODE_RELOAD
* @param[in]  value: Count value of the AON Sleep Timer.
* @retval ::HAL_OK: Operation is OK.
* @retval ::HAL_ERROR: Operation is ERROR.
****************************************************************************************
*/
hal_status_t hal_sleep_timer_config_and_start(uint8_t mode, uint32_t value);

/**
****************************************************************************************
* @brief  stop Sleep Timer
****************************************************************************************
*/
void hal_sleep_timer_stop(void);

/**
****************************************************************************************
* @brief  Get the current value of sleep timer
* @retval the current value of sleep timer
****************************************************************************************
*/
uint32_t hal_sleep_timer_get_current_value(void);

/**
****************************************************************************************
* @brief  Get clock frequency of sleep timer
* @retval clock frequency of sleep timer
****************************************************************************************
*/
uint32_t hal_sleep_timer_get_clock_freq(void);

/**
****************************************************************************************
* @brief  get sleep timer is running or not
* @retval runing state of sleep timer (1 or 0).
****************************************************************************************
*/
uint8_t hal_sleep_timer_status_get(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_SLEEP_TIMER_H__ */

/** @} */

/** @} */

/** @} */
