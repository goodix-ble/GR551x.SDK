/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_calendar.h
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

/** @defgroup LL_CALENDAR CALENDAR
  * @brief CALENDAR LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_CALENDAR_H__
#define __GR55XX_LL_CALENDAR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON)

/**
  * @defgroup  CALENDAR_LL_MACRO Defines
  * @{
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup CALENDAR_LL_Exported_Constants CALENDAR Exported Constants
  * @{
  */

/** @defgroup CALENDAR_LL_EC_CLOCK_DIV Clock divider
  * @{
  */
#define LL_CALENDAR_DIV_NONE            ((uint32_t)0x00U)                                       /**< Select RTC clock     */
#define LL_CALENDAR_DIV_32              ((uint32_t)0x01U << AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos) /**< Select 1/32 divider  */
#define LL_CALENDAR_DIV_64              ((uint32_t)0x02U << AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos) /**< Select 1/64 divider  */
#define LL_CALENDAR_DIV_128             ((uint32_t)0x03U << AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos) /**< Select 1/128 divider */
#define LL_CALENDAR_DIV_256             ((uint32_t)0x04U << AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos) /**< Select 1/256 divider */
#define LL_CALENDAR_NO_CLOCK            ((uint32_t)0x05U << AON_CALENDAR_TIMER_CTL_CLK_SEL_Pos) /**< Select no clock      */
/** @} */

/** @} */
/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup CALENDAR_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup CALENDAR_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable calendar counter.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_enable(void)
{
    SET_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_EN);
}

/**
  * @brief  Disable calendar counter.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_disable(void)
{
    CLEAR_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_EN);
}

/**
  * @brief  Check if the CALENDAR peripheral is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_enabled(void)
{
    return (READ_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_EN) == (AON_CALENDAR_TIMER_CTL_EN));
}

/**
  * @brief  Reloads CALENDAR counter.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | VAL_LOAD                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  TIMER_VALUE | TIMER_VALUE
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_counter(uint32_t counter)
{
    WRITE_REG(AON->TIMER_VALUE, counter);
    SET_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_VAL_LOAD);
}

/**
  * @brief  Reloads CALENDAR alarm.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ALARM_VAL_LOAD                    |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  TIMER_VALUE | TIMER_VALUE
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_reload_alarm(uint32_t alarm)
{
    WRITE_REG(AON->TIMER_VALUE, alarm);
    SET_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_ALARM_VAL_LOAD);
}

/**
  * @brief  Read the CALENDAR counter current value.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AON_PAD_CTL1         | CAL_TIMER                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  TIMER_VAL    | TIMER_VAL_READ
  *
  * @retval Value for current counter which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_calendar_get_counter(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(AON->AON_PAD_CTL1, AON_PAD_CTL1_TIMER_READ_SEL, AON_PAD_CTL1_TIMER_READ_SEL_CAL_TIMER);
    GLOBAL_EXCEPTION_ENABLE();
    return (uint32_t)READ_REG(AON->TIMER_VAL);
}

/**
  * @brief  Read the CALENDAR counter alarm value.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AON_PAD_CTL1         | CAL_ALARM                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *  TIMER_VAL    | TIMER_VAL_READ
  *
  * @retval Value for current alarm which should ranging between 0 ~ 0xFFFF_FFFF
  */
__STATIC_INLINE uint32_t ll_calendar_get_alarm(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(AON->AON_PAD_CTL1, AON_PAD_CTL1_TIMER_READ_SEL, AON_PAD_CTL1_TIMER_READ_SEL_CAL_ALARM);
    GLOBAL_EXCEPTION_ENABLE();
    return (uint32_t)READ_REG(AON->TIMER_VAL);
}

/**
  * @brief  Get the CALENDAR wrap-around value.
  * @note   The value should be read multiple times until get the same value in at least two reads.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | WRAP_CNT                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval Value between Min_Data=0 and Max_Data=0xF
  */
__STATIC_INLINE uint32_t ll_calendar_get_wrapcnt(void)
{
    return (uint32_t)(READ_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_WRAP_CNT) >> AON_CALENDAR_TIMER_CTL_WRAP_CNT_Pos);
}

/**
  * @brief  Select the CALENDAR clock divider.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | CLK_SEL                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  div This parameter can be one of the following values:
  *         @arg @ref LL_CALENDAR_DIV_NONE
  *         @arg @ref LL_CALENDAR_DIV_32
  *         @arg @ref LL_CALENDAR_DIV_64
  *         @arg @ref LL_CALENDAR_DIV_128
  *         @arg @ref LL_CALENDAR_DIV_256
  *         @arg @ref LL_CALENDAR_NO_CLOCK
  * @retval None
  */
__STATIC_INLINE void ll_calendar_set_clock_div(uint32_t div)
{
    MODIFY_REG(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_CLK_SEL, div);
}

/**
  * @brief  Enable calendar alarm interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ALARM_INT_EN                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_enable_alarm(void)
{
    SET_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_ALARM_INT_EN);
}

/**
  * @brief  Disable calendar alarm interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ALARM_INT_EN                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_disable_alarm(void)
{
    CLEAR_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_ALARM_INT_EN);
}

/**
  * @brief  Check if the CALENDAR alarm interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | ALARM_INT_EN                      |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_it_is_enabled_alarm(void)
{
    return (READ_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_ALARM_INT_EN) == (AON_CALENDAR_TIMER_CTL_ALARM_INT_EN));
}

/**
  * @brief  Enable calendar wrap interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | WRAP_INT_EN                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_enable_wrap(void)
{
    SET_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_WRAP_INT_EN);
}

/**
  * @brief  Disable calendar warp interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | WRAP_INT_EN                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_it_disable_wrap(void)
{
    CLEAR_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_WRAP_INT_EN);
}

/**
  * @brief  Check if the CALENDAR wrap interrupt is enabled or disabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CALENDAR_TIMER_CTL   | WRAP_INT_EN                       |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_it_is_enabled_wrap(void)
{
    return (READ_BITS(AON->CALENDAR_TIMER_CTL, AON_CALENDAR_TIMER_CTL_WRAP_INT_EN) == (AON_CALENDAR_TIMER_CTL_WRAP_INT_EN));
}

/** @} */

/** @defgroup CALENDAR_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate if the CALENDAR alarm event flag is set or not.
  * @note   This bit is set by hardware when the counter has reached alarm value.
  *         It can be cleared by writing 0 to this bit.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | CALENDAR_TIMER_ALARM              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_active_flag_alarm(void)
{
    return (uint32_t)(READ_BITS(AON->SLP_EVENT, AON_SLP_EVENT_CALENDAR_TIMER_ALARM) == AON_SLP_EVENT_CALENDAR_TIMER_ALARM);
}

/**
  * @brief  Indicate if the CALENDAR wrap event flag is set or not.
  * @note   This bit is set by hardware when the counter has overflow.
  *         It can be cleared by writing 0 to this bit.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | CALENDAR_TIMER_WRAP               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_calendar_is_active_flag_wrap(void)
{
    return (uint32_t)(READ_BITS(AON->SLP_EVENT, AON_SLP_EVENT_CALENDAR_TIMER_WRAP) == AON_SLP_EVENT_CALENDAR_TIMER_WRAP);
}

/**
  * @brief  Clear calendar alarm interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | CALENDAR_TIMER_ALARM              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_clear_flag_alarm(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    WRITE_REG(AON->SLP_EVENT, ~AON_SLP_EVENT_CALENDAR_TIMER_ALARM);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Clear calendar wrap interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SLP_EVENT            | CALENDAR_TIMER_WRAP               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_calendar_clear_flag_wrap(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    WRITE_REG(AON->SLP_EVENT, ~AON_SLP_EVENT_CALENDAR_TIMER_WRAP);
    GLOBAL_EXCEPTION_ENABLE();
}

/** @} */

/** @} */

#endif /* CALENDAR */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_CALENDAR_H__ */

/** @} */

/** @} */

/** @} */
