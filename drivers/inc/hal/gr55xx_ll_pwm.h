/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_pwm.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PWM LL library.
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

/** @defgroup LL_PWM PWM
  * @brief PWM LL module driver.
  * @{
  */

#ifndef __GR55XX_LL_PWM_H__
#define __GR55XX_LL_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gr55xx.h"

#if defined (PWM0) || defined (PWM1)

/** @defgroup PWM_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PWM_LL_ES_INIT PWM Exported init structures
  * @{
  */

/**
  * @brief  LL PWM Output Channel init Structure definition.
  */
typedef struct _ll_pwm_channel_init_t
{
    uint8_t duty;               /**< Specifies the duty in PWM output mode.
                                     This parameter must be a number ranges between Min_Data=0 and Max_Data=100.

                                     This feature can be modified afterwards using unitary function ll_pwm_set_compare_xn()
                                     where X can be (A, B, C) and n can be (0, 1).*/

    uint8_t drive_polarity;      /**< Specifies the drive polarity in PWM output mode.
                                     This parameter can be a value of @ref PWM_LL_EC_DRIVEPOLARITY.

                                     This feature can be modified afterwards using unitary function ll_pwm_enable_positive_drive_channel_x()
                                     and ll_pwm_disable_positive_drive_channel_x() where X can be (A, B, C).*/

} ll_pwm_channel_init_t;

/**
  * @brief  LL PWM  init Structure definition.
  */
typedef struct _ll_pwm_init_t
{
    uint32_t mode;              /**< Specifies the PWM output mode.
                                     This parameter can be a value of @ref PWM_LL_EC_MODE.

                                     This feature can be modified afterwards using unitary function @ref ll_pwm_set_mode().*/

    uint32_t align;             /**< Specifies the PWM alignment pulses.
                                     This parameter can be a value of @ref PWM_LL_EC_ALIGN.*/

    uint32_t prescaler;         /**< Specifies the prescaler value which will be used configure PWM output frequency.
                                     This parameter must be a number ranges between Min_Data = 0 and Max_Data = 0xFFFFFFFF.
                                     This parameter should be larger than 128.

                                     This feature can be modified afterwards using unitary function @ref ll_pwm_set_prescaler().*/

    uint32_t bprescaler;        /**< Specifies the required prescaler that the duty changes from 0% to 100% in breath mode.
                                     This parameter must be a number ranges between Min_Data=0 and Max_Data=0xFFFFFFFF.
                                     This parameter is recommended to be larger than 128*prescaler to guarantee an ideal breath effect.

                                     This feature can be modified afterwards using unitary function @ref ll_pwm_set_breath_prescaler().*/

    uint32_t hprescaler;        /**< Specifies the required prescaler in breath hold state.
                                     This parameter must be a number ranges between Min_Data=0 and Max_Data=0xFFFFFF.

                                     This feature can be modified afterwards using unitary function @ref ll_pwm_set_hold_prescaler().*/

    ll_pwm_channel_init_t channel_a;    /**< Specifies the configuration of channelA.
                                                 This parameter can be a value of @ref ll_pwm_channel_init_t.*/

    ll_pwm_channel_init_t channel_b;    /**< Specifies the configuration of channelB.
                                                 This parameter can be a value of @ref ll_pwm_channel_init_t.*/

    ll_pwm_channel_init_t channel_c;    /**< Specifies the configuration of channelC.
                                                 This parameter can be a value of @ref ll_pwm_channel_init_t.*/

} ll_pwm_init_t;

/** @} */

/** @} */

/**
  * @defgroup  PWM_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWM_LL_Exported_Constants PWM Exported Constants
  * @{
  */

/** @defgroup PWM_LL_EC_MODE PWM mode
  * @{
  */
#define LL_PWM_FLICKER_MODE                 (0x00000000U)       /**< PWM flicker mode */
#define LL_PWM_BREATH_MODE                  PWM_MODE_BREATHEN   /**< PWM breath mode  */
/** @} */

/** @defgroup PWM_LL_EC_ALIGN PWM alignment pulses
  * @{
  */
#define LL_PWM_EDGE_ALIGNED                 (0x00000000U)       /**< PWM edge-aligned */
#define LL_PWM_CENTER_ALIGNED               (0x00000001U)       /**< PWM center-aligned  */
/** @} */

/** @defgroup PWM_LL_EC_DRIVEPOLARITY PWM drive polarity
  * @{
  */
#define LL_PWM_DRIVEPOLARITY_NEGATIVE       (0x00000000U)       /**< PWM led-negative-drive mode */
#define LL_PWM_DRIVEPOLARITY_POSITIVE       (0x00000001U)       /**< PWM led-positive-drive mode */
/** @} */

/** @defgroup PWM_LL_EC_ACTIONEVENT PWM action event
  * @{
  */
#define LL_PWM_ACTIONEVENT_NONE             (0x00000000U)       /**< No action event      */
#define LL_PWM_ACTIONEVENT_CLEAR            (0x00000001U)       /**< Action event CLEAR   */
#define LL_PWM_ACTIONEVENT_SET              (0x00000002U)       /**< Action event SET     */
#define LL_PWM_ACTIONEVENT_TOGGLE           (0x00000003U)       /**< Action event TOGGLE  */
/** @} */

/** @defgroup PWM_LL_EC_PERIOD_UNIT PWM period unit default configuretion
  * @{
  */
#define LL_PWM_PRESCALER_UNIT               (128)               /**< The unit of prescaler is 128 */
#define LL_PWM_BREATH_PRESCALER_UNIT        (128)               /**< The unit of breath prescaler is 128 */
#define LL_PWM_HOLD_PRESCALER_UNIT          (10)                /**< The unit of hold prescaler is 10 */
/** @} */

/** @defgroup PWM_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL PWM Channel InitStrcut default configuartion
  */
#define LL_PWM_CHANNEL_DEFAULT_CONFIG                   \
{                                                       \
    .duty          = 50,                                \
    .drive_polarity = LL_PWM_DRIVEPOLARITY_POSITIVE,    \
}

/**
  * @brief LL PWM InitStrcut default configuartion
  */
#define LL_PWM_DEFAULT_CONFIG                       \
{                                                   \
    .mode       = LL_PWM_FLICKER_MODE,              \
    .align      = LL_PWM_EDGE_ALIGNED,              \
    .prescaler  = 10 * LL_PWM_PRESCALER_UNIT,       \
    .bprescaler = 10 * LL_PWM_BREATH_PRESCALER_UNIT * 10 * LL_PWM_PRESCALER_UNIT,  \
    .hprescaler = 10 * LL_PWM_HOLD_PRESCALER_UNIT * 10 * LL_PWM_PRESCALER_UNIT,    \
    .channel_a  = LL_PWM_CHANNEL_DEFAULT_CONFIG,    \
    .channel_b  = LL_PWM_CHANNEL_DEFAULT_CONFIG,    \
    .channel_c  = LL_PWM_CHANNEL_DEFAULT_CONFIG,    \
}

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWM_LL_Exported_Macros PWM Exported Macros
  * @{
  */

/** @defgroup PWM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PWM register
  * @param  __instance__ PWM instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_PWM_WriteReg(__instance__, __REG__, __VALUE__)  WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PWM register
  * @param  __instance__ PWM instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_PWM_ReadReg(__instance__, __REG__)              READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PWM_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup PWM_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable PWM.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->MODE, PWM_MODE_EN);
}

/**
  * @brief  Disable PWM.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->MODE, PWM_MODE_EN);
}

/**
  * @brief  Indicate whether the PWM is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->MODE, PWM_MODE_EN) == (PWM_MODE_EN));
}

/**
  * @brief  Enable PWM pause.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | PAUSE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_pause(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->MODE, PWM_MODE_PAUSE);
}

/**
  * @brief  Disable PWM pause.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | PAUSE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_pause(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->MODE, PWM_MODE_PAUSE);
}

/**
  * @brief  Indicate whether the PWM pause is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | PAUSE                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_pause(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->MODE, PWM_MODE_PAUSE) == (PWM_MODE_PAUSE));
}

/**
  * @brief  Set PWM mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | BREATHEN                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_PWM_FLICKER_MODE
  *         @arg @ref LL_PWM_BREATH_MODE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_mode(pwm_regs_t *PWMx, uint32_t mode)
{
    MODIFY_REG(PWMx->MODE, PWM_MODE_BREATHEN, mode);
}

/**
  * @brief  Get PWM mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | BREATHEN                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_FLICKER_MODE
  *         @arg @ref LL_PWM_BREATH_MODE
  */
__STATIC_INLINE uint32_t ll_pwm_get_mode(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->MODE, PWM_MODE_BREATHEN));
}

/**
  * @brief  Enable positive drive mode in channelA.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENA                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_positive_drive_channel_a(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->MODE, PWM_MODE_DPENA);
}

/**
  * @brief  Disable positive drive mode in channelA.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENA                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_positive_drive_channel_a(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->MODE, PWM_MODE_DPENA);
}

/**
  * @brief  Indicate whether the positive drive mode in channelA is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENA                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_positive_drive_channel_a(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->MODE, PWM_MODE_DPENA) == (PWM_MODE_DPENA));
}

/**
  * @brief  Enable positive drive mode in channelB.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENB                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_positive_drive_channel_b(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->MODE, PWM_MODE_DPENB);
}

/**
  * @brief  Disable positive drive mode in channelB.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENB                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_positive_drive_channel_b(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->MODE, PWM_MODE_DPENB);
}

/**
  * @brief  Indicate whether the positive drive mode in channelB is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENB                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_positive_drive_channel_b(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->MODE, PWM_MODE_DPENB) == (PWM_MODE_DPENB));
}

/**
  * @brief  Enable positive drive mode in channelC.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_positive_drive_channel_c(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->MODE, PWM_MODE_DPENC);
}

/**
  * @brief  Disable positive drive mode in channelC.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_positive_drive_channel_c(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->MODE, PWM_MODE_DPENC);
}

/**
  * @brief  Indicate whether the positive drive mode in channelC is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | MODE                 | DPENC                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_positive_drive_channel_c(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->MODE, PWM_MODE_DPENC) == (PWM_MODE_DPENC));
}

/**
  * @brief  Check update active flag
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SAG                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_active_flag_update_all(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SAG) == (PWM_UPDATE_SAG));
}

/**
  * @brief  Enable update all parameters.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SA                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_all(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SA);
}

/**
  * @brief  Disable update all parameters.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SA                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_all(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SA);
}

/**
  * @brief  Indicate whether the update all parameters is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SA                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_all(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SA) == (PWM_UPDATE_SA));
}

/**
  * @brief  Enable update period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSPRD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_period(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSPRD);
}

/**
  * @brief  Disable update period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSPRD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_period(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSPRD);
}

/**
  * @brief  Indicate whether the update period is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSPRD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_period(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSPRD) == (PWM_UPDATE_SSPRD));
}

/**
  * @brief  Enable update compareA0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPA0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_compare_a0(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPA0);
}

/**
  * @brief  Disable update compareA0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPA0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_compare_a0(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPA0);
}

/**
  * @brief  Indicate whether the update compareA0 is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPA0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_compare_a0(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPA0) == (PWM_UPDATE_SSCMPA0));
}

/**
  * @brief  Enable update compareA1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPA1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_compare_a1(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPA1);
}

/**
  * @brief  Disable update compareA1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPA1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_compare_a1(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPA1);
}

/**
  * @brief  Indicate whether the update compareA1 is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPA1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_compare_a1(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPA1) == (PWM_UPDATE_SSCMPA1));
}

/**
  * @brief  Enable update compareB0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPB0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_compare_b0(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPB0);
}

/**
  * @brief  Disable update compareB0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPB0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_compare_b0(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPB0);
}

/**
  * @brief  Indicate whether the update compareB0 is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPB0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_compare_b0(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPB0) == (PWM_UPDATE_SSCMPB0));
}

/**
  * @brief  Enable update compareB1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPB1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_compare_b1(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPB1);
}

/**
  * @brief  Disable update compareB1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPB1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_compare_b1(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPB1);
}

/**
  * @brief  Indicate whether the update compareB1 is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPB1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_compare_b1(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPB1) == (PWM_UPDATE_SSCMPB1));
}

/**
  * @brief  Enable update compareC0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPC0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_compare_c0(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPC0);
}

/**
  * @brief  Disable update compareC0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPC0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_compare_c0(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPC0);
}

/**
  * @brief  Indicate whether the update compareC0 is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPC0                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_compare_c0(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPC0) == (PWM_UPDATE_SSCMPC0));
}

/**
  * @brief  Enable update compareC1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPC1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_compare_c1(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPC1);
}

/**
  * @brief  Disable update compareC1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPC1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_compare_c1(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPC1);
}

/**
  * @brief  Indicate whether the update compareC1 is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSCMPC1                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_compare_c1(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSCMPC1) == (PWM_UPDATE_SSCMPC1));
}

/**
  * @brief  Enable update pause.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSPAUSE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_pause(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSPAUSE);
}

/**
  * @brief  Disable update pause.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSPAUSE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_pause(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSPAUSE);
}

/**
  * @brief  Indicate whether the update pause is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSPAUSE                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_pause(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSPAUSE) == (PWM_UPDATE_SSPAUSE));
}

/**
  * @brief  Enable update breath period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSBRPRD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_breath_period(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSBRPRD);
}

/**
  * @brief  Disable update breath period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSBRPRD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_breath_period(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSBRPRD);
}

/**
  * @brief  Indicate whether the update breath period is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSBRPRD                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_breath_period(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSBRPRD) == (PWM_UPDATE_SSBRPRD));
}

/**
  * @brief  Enable update hold period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSHOLD                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_hold_period(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSHOLD);
}

/**
  * @brief  Disable update hold period.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSHOLD                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_hold_period(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSHOLD);
}

/**
  * @brief  Indicate whether the update hold period is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSHOLD                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_hold_period(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSHOLD) == (PWM_UPDATE_SSHOLD));
}

/**
  * @brief  Enable update active event.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSAQCTRL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_enable_update_active_event(pwm_regs_t *PWMx)
{
    SET_BITS(PWMx->UPDATE, PWM_UPDATE_SSAQCTRL);
}

/**
  * @brief  Disable update active event.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSAQCTRL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval None
  */
__STATIC_INLINE void ll_pwm_disable_update_active_event(pwm_regs_t *PWMx)
{
    CLEAR_BITS(PWMx->UPDATE, PWM_UPDATE_SSAQCTRL);
}

/**
  * @brief  Indicate whether the update active event is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | UPDATE               | SSAQCTRL                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pwm_is_enabled_update_active_event(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->UPDATE, PWM_UPDATE_SSAQCTRL) == (PWM_UPDATE_SSAQCTRL));
}

/**
  * @brief  Set the PWM prescaler.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PRD                  | PRD                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  prescaler This parameter ranges between Min_Data=1 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_prescaler(pwm_regs_t *PWMx, uint32_t prescaler)
{
    WRITE_REG(PWMx->PRD, prescaler);
}

/**
  * @brief  Get the PWM prescaler.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | PRD                  | PRD                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=1 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_prescaler(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->PRD));
}

/**
  * @brief  Set the PWM compare counter A0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPA0                | CMPA0                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  compare This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_compare_a0(pwm_regs_t *PWMx, uint32_t compare)
{
    WRITE_REG(PWMx->CMPA0, compare);
}

/**
  * @brief  Get the PWM compare counter A0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPA0                | CMPA0                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_compare_a0(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->CMPA0));
}

/**
  * @brief  Set the PWM compare counter A1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPA1                | CMPA1                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  compare This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_compare_a1(pwm_regs_t *PWMx, uint32_t compare)
{
    WRITE_REG(PWMx->CMPA1, compare);
}

/**
  * @brief  Get the PWM compare counter A1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPA1                | CMPA1                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_compare_a1(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->CMPA1));
}

/**
  * @brief  Set the PWM compare counter B0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPB0                | CMPB0                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  compare This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_compare_b0(pwm_regs_t *PWMx, uint32_t compare)
{
    WRITE_REG(PWMx->CMPB0, compare);
}

/**
  * @brief  Get the PWM compare counter B0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPB0                | CMPB0                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_compare_b0(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->CMPB0));
}

/**
  * @brief  Set the PWM compare counter B1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPB1                | CMPB1                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  compare This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_compare_b1(pwm_regs_t *PWMx, uint32_t compare)
{
    WRITE_REG(PWMx->CMPB1, compare);
}

/**
  * @brief  Get the PWM compare counter B1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPB1                | CMPB1                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_compare_b1(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->CMPB1));
}

/**
  * @brief  Set the PWM compare counter C0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPC0                | CMPC0                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  compare This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_compare_c0(pwm_regs_t *PWMx, uint32_t compare)
{
    WRITE_REG(PWMx->CMPC0, compare);
}

/**
  * @brief  Get the PWM compare counter C0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPC0                | CMPC0                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_compare_c0(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->CMPC0));
}

/**
  * @brief  Set the PWM compare counter C1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPC1                | CMPC1                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  compare This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_compare_c1(pwm_regs_t *PWMx, uint32_t compare)
{
    WRITE_REG(PWMx->CMPC1, compare);
}

/**
  * @brief  Get the PWM compare counter C1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CMPC1                | CMPC1                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_compare_c1(pwm_regs_t *PWMx)
{
    return (READ_REG(PWMx->CMPC1));
}

/**
  * @brief  Set the channel A0 action event when PWM counter value reaches compare counter A0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | A0                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  action_event This parameter can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_action_event_cmp_a0(pwm_regs_t *PWMx, uint32_t action_event)
{
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_A0, action_event << PWM_AQCTRL_A0_Pos);
}

/**
  * @brief  Get the channel A0 action event when PWM counter value reaches compare counter A0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | A0                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  */
__STATIC_INLINE uint32_t ll_pwm_get_action_event_cmp_a0(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->AQCTRL, PWM_AQCTRL_A0) >> PWM_AQCTRL_A0_Pos);
}

/**
  * @brief  Set the channel A1 action event when PWM counter value reaches compare counter A1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | A1                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  action_event This parameter can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_action_event_cmp_a1(pwm_regs_t *PWMx, uint32_t action_event)
{
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_A1, action_event << PWM_AQCTRL_A1_Pos);
}

/**
  * @brief  Get the channel A1 action event when PWM counter value reaches compare counter A1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | A1                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  */
__STATIC_INLINE uint32_t ll_pwm_get_action_event_cmp_a1(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->AQCTRL, PWM_AQCTRL_A1) >> PWM_AQCTRL_A1_Pos);
}

/**
  * @brief  Set the channel B0 action event when PWM counter value reaches compare counter B0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | B0                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  action_event This parameter can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_action_event_cmp_b0(pwm_regs_t *PWMx, uint32_t action_event)
{
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_B0, action_event << PWM_AQCTRL_B0_Pos);
}

/**
  * @brief  Get the channel B0 action event when PWM counter value reaches compare counter B0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | B0                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  */
__STATIC_INLINE uint32_t ll_pwm_get_action_event_cmp_b0(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->AQCTRL, PWM_AQCTRL_B0) >> PWM_AQCTRL_B0_Pos);
}

/**
  * @brief  Set the channel B1 action event when PWM counter value reaches compare counter B1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | B1                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  action_event This parameter can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_action_event_cmp_b1(pwm_regs_t *PWMx, uint32_t action_event)
{
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_B1, action_event << PWM_AQCTRL_B1_Pos);
}

/**
  * @brief  Get the channel B1 action event when PWM counter value reaches compare counter B1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | B1                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  */
__STATIC_INLINE uint32_t ll_pwm_get_action_event_cmp_b1(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->AQCTRL, PWM_AQCTRL_B1) >> PWM_AQCTRL_B1_Pos);
}

/**
  * @brief  Set the channel C0 action event when PWM counter value reaches compare counter C0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | C0                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  action_event This parameter can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_action_event_cmp_c0(pwm_regs_t *PWMx, uint32_t action_event)
{
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_C0, action_event << PWM_AQCTRL_C0_Pos);
}

/**
  * @brief  Get the channel C0 action event when PWM counter value reaches compare counter C0.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | C0                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  */
__STATIC_INLINE uint32_t ll_pwm_get_action_event_cmp_c0(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->AQCTRL, PWM_AQCTRL_C0) >> PWM_AQCTRL_C0_Pos);
}

/**
  * @brief  Set the channel C1 action event when PWM counter value reaches compare counter C1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | C1                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  action_event This parameter can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_action_event_cmp_c1(pwm_regs_t *PWMx, uint32_t action_event)
{
    MODIFY_REG(PWMx->AQCTRL, PWM_AQCTRL_C1, action_event << PWM_AQCTRL_C1_Pos);
}

/**
  * @brief  Get the channel C1 action event when PWM counter value reaches compare counter C1.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | AQCTRL               | C1                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PWM_ACTIONEVENT_NONE
  *         @arg @ref LL_PWM_ACTIONEVENT_CLEAR
  *         @arg @ref LL_PWM_ACTIONEVENT_SET
  *         @arg @ref LL_PWM_ACTIONEVENT_TOGGLE
  */
__STATIC_INLINE uint32_t ll_pwm_get_action_event_cmp_c1(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->AQCTRL, PWM_AQCTRL_C1) >> PWM_AQCTRL_C1_Pos);
}

/**
  * @brief  Set the breath prescaler in breath mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | BRPRD                | BRPRD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  bprescaler This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_breath_prescaler(pwm_regs_t *PWMx, uint32_t bprescaler)
{
    MODIFY_REG(PWMx->BRPRD, PWM_BRPRD_BRPRD, bprescaler);
}

/**
  * @brief  Get the breath prescaler in breath mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | BRPRD                | BRPRD                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_breath_prescaler(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->BRPRD, PWM_BRPRD_BRPRD));
}

/**
  * @brief  Set the hold prescaler in breath mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | HOLD                 | HOLD                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @param  hprescaler This parameter ranges between Min_Data=0 and Max_Data=0xFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pwm_set_hold_prescaler(pwm_regs_t *PWMx, uint32_t hprescaler)
{
    MODIFY_REG(PWMx->HOLD, PWM_HOLD_HOLD, hprescaler);
}

/**
  * @brief  Get the hold prescaler in breath mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | HOLD                 | HOLD                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PWMx PWM instance
  * @retval Return value ranges between Min_Data=0 and Max_Data=0xFFFFFF
  */
__STATIC_INLINE uint32_t ll_pwm_get_hold_prescaler(pwm_regs_t *PWMx)
{
    return (READ_BITS(PWMx->HOLD, PWM_HOLD_HOLD));
}

/** @} */

/** @defgroup PWM_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize PWM registers (Registers restored to their default values).
  * @param  PWMx PWM instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PWM registers are de-initialized
  *          - ERROR: PWM registers are not de-initialized
  */
error_status_t ll_pwm_deinit(pwm_regs_t *PWMx);

/**
  * @brief  Initialize PWM registers according to the specified
  *         parameters in PWM_InitStruct.
  * @param  PWMx PWM instance
  * @param  p_pwm_init Pointer to a ll_pwm_init_t structure that contains the configuration
  *                         information for the specified PWM peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PWM registers are initialized according to p_pwm_init content
  *          - ERROR: Problem occurred during PWM Registers initialization
  */
error_status_t ll_pwm_init(pwm_regs_t *PWMx, ll_pwm_init_t *p_pwm_init);

/**
  * @brief Set each field of a @ref ll_pwm_init_t type structure to default value.
  * @param p_pwm_init  Pointer to a @ref ll_pwm_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_pwm_struct_init(ll_pwm_init_t *p_pwm_init);

/** @} */

/** @} */

#endif /* PWM0 || PWM1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_PWM_H__ */

/** @} */

/** @} */

/** @} */
