/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_aon_wdt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AON WDT LL library.
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

/** @defgroup LL_AON_WDT AON_WDT
  * @brief AON_WDT LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_AON_WDT_H__
#define __GR55XX_LL_AON_WDT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (AON)

/* Exported functions --------------------------------------------------------*/
/** @defgroup AON_WDT_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup AON_WDT_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable AON watchdog counter and interrupt event.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_enable(void)
{
    SET_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_EN);
}

/**
  * @brief  Disable AON watchdog counter and interrupt event.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_disable(void)
{
    CLEAR_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_EN);
}

/**
  * @brief  Check if the AON_WDT peripheral is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_EN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_enabled(void)
{
    return (READ_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_EN) == (AON_EXT_WKUP_CTL_WDT_EN));
}

/**
  * @brief  Specify the AON WDT down-counter reload value.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | TIMER_VALUE          | TIMER_VALUE                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  counter Value for reload down-counter which should ranging between 0 ~ 0xFFFF_FFFF
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_set_reload_counter(uint32_t counter)
{
    WRITE_REG(AON->TIMER_VALUE, counter);
}

/**
  * @brief  Reloads AON WDT counter.
  * @note   The value in TIMER_VALUE register will be reloaded into AON WDT down-counter
  *         after enable this bit, so ll_aon_wdt_set_reload_counter() should be called before
  *         every reload.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_RELOAD                        |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_reload_counter(void)
{
    SET_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_RELOAD);
}

/**
  * @brief  Read the AON WDT counter current value.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AON_PAD_CTL1         | AON_WDT_TIMER                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  TIMER_VAL    | TIMER_VAL_READ
  *
  * @retval Value for current counter which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_counter(void)
{
    MODIFY_REG(AON->AON_PAD_CTL1, AON_PAD_CTL1_TIMER_READ_SEL, AON_PAD_CTL1_TIMER_READ_SEL_AON_WDT);
    return (uint32_t)READ_REG(AON->TIMER_VAL);
}

/**
  * @brief  Specify the AON_WDT down-counter alarm value
  * @note   AON watchdog will generate an interrupt when it counts down to the
  *         alarm value to alram that it is almost expired.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_ALARM                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  counter Value between Min_Data=0 and Max_Data=0x1F
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_set_alarm_counter(uint32_t counter)
{
    MODIFY_REG(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_ALARM, (counter << AON_EXT_WKUP_CTL_WDT_ALARM_Pos) & AON_EXT_WKUP_CTL_WDT_ALARM);
}

/**
  * @brief  Get the AON_WDT down-counter alarm value
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_ALARM                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Value between Min_Data=0 and Max_Data=0x1F
  */
__STATIC_INLINE uint32_t ll_aon_wdt_get_alarm_counter(void)
{
    return (uint32_t)(READ_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_ALARM) >> AON_EXT_WKUP_CTL_WDT_ALARM_Pos);
}

/** @} */

/** @defgroup AON_WDT_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate if the AON Watchdog Running Flag is set or not.
  * @note   This bit can be used to check if AON Watchdog is in running state.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | EXT_WKUP_CTL         | WDT_RUNNING                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_active_flag_running(void)
{
    return (uint32_t)(READ_BITS(AON->EXT_WKUP_CTL, AON_EXT_WKUP_CTL_WDT_RUNNING) == (AON_EXT_WKUP_CTL_WDT_RUNNING));
}

/**
  * @brief  Indicate if the AON WDT Reboot Event Flag is set or not.
  * @note   This bit is set by hardware when the counter has reached alarm value.
  *         It can be cleared by writing 0 to this bit.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | SLP_EVENT_WDT                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_aon_wdt_is_active_flag_reboot(void)
{
    return (uint32_t)(READ_BITS(AON->SLP_EVENT, AON_SLP_EVENT_WDT_REBOOT) == AON_SLP_EVENT_WDT_REBOOT);
}

/**
  * @brief  Clear Interrupt Status flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | SLP_EVENT_WDT                     |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_aon_wdt_clear_flag_reboot(void)
{
    WRITE_REG(AON->SLP_EVENT, ~AON_SLP_EVENT_WDT_REBOOT);
}

/** @} */

/** @} */

#endif /* AON_WDT */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_AON_WDT_H__ */

/** @} */

/** @} */

/** @} */
