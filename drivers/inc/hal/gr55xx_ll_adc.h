/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_adc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of ADC LL library.
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

/** @defgroup LL_ADC ADC
  * @brief ADC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_ADC_H__
#define __GR55XX_LL_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(AON)

/** @defgroup LL_ADC_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup LL_ADC_ES_INIT ADC Exported init structures
  * @{
  */

/**
  * @brief LL ADC init Structure definition
  */
typedef struct _ll_adc_init
{
    uint32_t channel_p;     /**< Specifies the input source to ADC channel P.
                                 This parameter can be any value of @ref LL_ADC_EC_INPUT_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_channelp(). */

    uint32_t channel_n;     /**< Specifies the input source to ADC channel N.
                                 This parameter can be any value of @ref LL_ADC_EC_INPUT_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_channeln(). */

    uint32_t input_mode;    /**< Specifies the operation mode for the ADC sample.
                                 This parameter can be a value of @ref LL_ADC_EC_INPUT_MODE.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_input_mode(). */

    uint32_t ref_source;    /**< Specifies the source of the ADC reference.
                                 This parameter can be a value of @ref LL_ADC_EC_REFERENCE_SRC.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_ref().*/

    uint32_t ref_value;     /*!< Specifies the value of the ADC buffered reference.
                                 This parameter can be a value of @ref LL_ADC_EC_REFERENCE.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_ref_value().*/

    uint32_t clock;         /**< Specifies the clock of ADC.
                                 This parameter can be a value of @ref LL_ADC_EC_CLK.

                                 This parament can be modified afterwards using unitary function @ref ll_adc_set_clock().*/

} ll_adc_init_t;

/** @} */

/** @} */

/**
  * @defgroup  LL_ADC_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LL_ADC_Exported_Constants ADC Exported Constants
  * @{
  */

/** @defgroup LL_ADC_EC_CLK ADC CLOCK
  * @{
  */
#define LL_ADC_CLK_16               (0x00000000UL)                                /**< 16 MHz    */
#define LL_ADC_CLK_8                (1UL << AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos)   /**< 8 MHz     */
#define LL_ADC_CLK_4                (2UL << AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos)   /**< 4 MHz     */
#define LL_ADC_CLK_2                (3UL << AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos)   /**< 2 MHz     */
#define LL_ADC_CLK_1P6              (4UL << AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos)   /**< 1.6 MHz   */
#define LL_ADC_CLK_1                (5UL << AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos)   /**< 1 MHz     */
/** @} */

/** @defgroup LL_ADC_EC_REFERENCE ADC Buffered Internal Reference Value
  * @{
  */
#define LL_ADC_REF_VALUE_0P8        (0x3UL << AON_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 0.85 V */
#define LL_ADC_REF_VALUE_1P2        (0x7UL << AON_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 1.28 V */
#define LL_ADC_REF_VALUE_1P6        (0xAUL << AON_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 1.60 V */
//#define LL_ADC_REF_VALUE_2P0        (0xFUL << AON_SNSADC_CFG_REF_VALUE_Pos)   /**< Reference = 2.00 V */
/** @} */

/** @defgroup LL_ADC_EC_INPUT_MODE ADC Input Mode
  * @{
  */
#define LL_ADC_INPUT_SINGLE         (1UL << AON_SNSADC_CFG_SINGLE_EN_Pos)     /**< Single ended mode */
#define LL_ADC_INPUT_DIFFERENTIAL   (0x00000000UL)                            /**< Differential mode */
/** @} */

/** @defgroup LL_ADC_EC_INPUT_SRC ADC Input Source
  * @{
  */
#define LL_ADC_INPUT_SRC_IO0        (0UL)  /**< Select MSIO0 as input       */
#define LL_ADC_INPUT_SRC_IO1        (1UL)  /**< Select MSIO1 as input       */
#define LL_ADC_INPUT_SRC_IO2        (2UL)  /**< Select MSIO2 as input       */
#define LL_ADC_INPUT_SRC_IO3        (3UL)  /**< Select MSIO3 as input       */
#define LL_ADC_INPUT_SRC_IO4        (4UL)  /**< Select MSIO4 as input       */
#define LL_ADC_INPUT_SRC_TMP        (5UL)  /**< Select temperature as input */
#define LL_ADC_INPUT_SRC_BAT        (6UL)  /**< Select Vbattery as input    */
#define LL_ADC_INPUT_SRC_REF        (7UL)  /**< Select reference as input   */

/** @} */

/** @defgroup LL_ADC_EC_REFERENCE_SRC ADC Reference Source
  * @{
  */
#define LL_ADC_REF_SRC_BUF_INT      (0x00000000UL)                            /**< Select buffered internal reference as reference   */
#define LL_ADC_REF_SRC_IO0          (3UL << AON_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO0 as reference                         */
#define LL_ADC_REF_SRC_IO1          (4UL << AON_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO1 as reference                         */
#define LL_ADC_REF_SRC_IO2          (5UL << AON_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO2 as reference                         */
#define LL_ADC_REF_SRC_IO3          (6UL << AON_SNSADC_CFG_REF_SEL_Pos)       /**< Select MSIO3 as reference                         */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LL_ADC_Exported_Macros ADC Exported Macros
  * @{
  */

/** @defgroup LL_ADC_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in ADC register
  * @param  __instance__ ADC instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_ADC_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG((__instance__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in ADC register
  * @param  __instance__ ADC instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_ADC_ReadReg(__instance__, __REG__) READ_REG((__instance__)->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup LL_ADC_Private_Macros ADC Private Macros
  * @{
  */

/** @defgroup LL_ADC_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief LL ADC InitStrcut default configuartion
  */
#define LL_ADC_DEFAULT_CONFIG                      \
{                                                  \
    .channel_p  = LL_ADC_INPUT_SRC_IO0,            \
    .channel_n  = LL_ADC_INPUT_SRC_IO1,            \
    .input_mode = LL_ADC_INPUT_DIFFERENTIAL,       \
    .ref_source = LL_ADC_REF_SRC_BUF_INT,          \
    .ref_value  = LL_ADC_REF_VALUE_1P5,            \
    .clock      = LL_ADC_CLK_16                    \
}
/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup LL_ADC_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup LL_ADC_EF_Configuration Basic Configuration
  * @{
  */

/**
  * @brief  Enable ADC module.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable(void)
{
    SET_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_EN_Msk);
}

/**
  * @brief  Disable ADC module.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable(void)
{
    CLEAR_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_EN_Msk);
}

/**
  * @brief  Check if ADC module is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled(void)
{
    return (READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_EN_Msk) == (AON_SNSADC_CFG_EN_Msk));
}

/**
  * @brief  Enable ADC clock.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | MSIO_PAD_CFG_1       | ADC_CLK_EN                  |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_clock(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(AON->MSIO_PAD_CFG_1, AON_MSIO_PAD_CFG_1_ADC_CLK_EN);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable ADC clock.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | MSIO_PAD_CFG_1       | ADC_CLK_EN                  |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_clock(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(AON->MSIO_PAD_CFG_1, AON_MSIO_PAD_CFG_1_ADC_CLK_EN);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Check if ADC clock is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | MSIO_PAD_CFG_1       | ADC_CLK_EN                  |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_clock(void)
{
    return (READ_BITS(AON->MSIO_PAD_CFG_1, AON_MSIO_PAD_CFG_1_ADC_CLK_EN) == (AON_MSIO_PAD_CFG_1_ADC_CLK_EN));
}

/**
  * @brief  Set ADC clock source.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | MSIO_PAD_CFG_1       | ADC_CLK_SEL                 |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  clk This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CLK_16
  *         @arg @ref LL_ADC_CLK_8
  *         @arg @ref LL_ADC_CLK_4
  *         @arg @ref LL_ADC_CLK_2
  *         @arg @ref LL_ADC_CLK_1P6
  *         @arg @ref LL_ADC_CLK_1
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_clock(uint32_t clk)
{
    GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(AON->MSIO_PAD_CFG_1, AON_MSIO_PAD_CFG_1_ADC_CLK_SEL, clk);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Return source for ADC clock.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | MSIO_PAD_CFG_1       | ADC_CLK_SEL                 |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CLK_16
  *         @arg @ref LL_ADC_CLK_8
  *         @arg @ref LL_ADC_CLK_4
  *         @arg @ref LL_ADC_CLK_2
  *         @arg @ref LL_ADC_CLK_1P6
  *         @arg @ref LL_ADC_CLK_1
  */
__STATIC_INLINE uint32_t ll_adc_get_clock(void)
{
    return (uint32_t)(READ_BITS(AON->MSIO_PAD_CFG_1, AON_MSIO_PAD_CFG_1_ADC_CLK_SEL) >> AON_MSIO_PAD_CFG_1_ADC_CLK_SEL_Pos);
}

/**
  * @brief  Set ADC bias reference.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG1                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  value This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REF_VALUE_0P8
  *         @arg @ref LL_ADC_REF_VALUE_1P2
  *         @arg @ref LL_ADC_REF_VALUE_1P6
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref_value(uint32_t value)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_REF_VALUE_Msk, value);
}

/**
  * @brief  Return ADC bias reference.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG1                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REF_VALUE_0P8
  *         @arg @ref LL_ADC_REF_VALUE_1P2
  *         @arg @ref LL_ADC_REF_VALUE_1P6
  */
__STATIC_INLINE uint32_t ll_adc_get_ref_value(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_REF_VALUE_Msk) >> AON_SNSADC_CFG_REF_VALUE_Pos);
}

/**
  * @brief  Enable temperature sensor.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_temp(void)
{
    SET_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_TEMP_EN_Msk);
}

/**
  * @brief  Disable temperature sensor.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_temp(void)
{
    CLEAR_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_TEMP_EN_Msk);
}

/**
  * @brief  Check if temperature sensor is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_temp(void)
{
    return (READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_TEMP_EN_Msk) == (AON_SNSADC_CFG_TEMP_EN_Msk));
}

/**
  * @brief  Enable Vbattery sensor.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_vbat(void)
{
    SET_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_VBAT_EN_Msk);
}

/**
  * @brief  Disable Vbattery sensor.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_vbat(void)
{
    CLEAR_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_VBAT_EN_Msk);
}

/**
  * @brief  Check if Vbattery sensor is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_vbat(void)
{
    return (READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_VBAT_EN_Msk) == (AON_SNSADC_CFG_VBAT_EN_Msk));
}

/**
  * @brief  Set ADC input mode.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SINGLE
  *         @arg @ref LL_ADC_INPUT_DIFFERENTIAL
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_input_mode(uint32_t mode)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_SINGLE_EN_Msk, mode);
}

/**
  * @brief  Return ADC input mode.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SINGLE
  *         @arg @ref LL_ADC_INPUT_DIFFERENTIAL
  */
__STATIC_INLINE uint32_t ll_adc_get_input_mode(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_SINGLE_EN_Msk) >> AON_SNSADC_CFG_SINGLE_EN_Pos);
}

/**
  * @brief  Enable offset calibration.
  * @note   Enable offset calibration, used to swap inputs of comparator for offset
  *         calibration.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_ofs_cal(void)
{
    SET_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_OFS_CAL_EN_Msk);
}

/**
  * @brief  Disable offset calibration.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_ofs_cal(void)
{
    CLEAR_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_OFS_CAL_EN_Msk);
}

/**
  * @brief  Check if offset calibration is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_ofs_cal(void)
{
    return (READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_OFS_CAL_EN_Msk) == (AON_SNSADC_CFG_OFS_CAL_EN_Msk));
}

/**
  * @brief  Set dynamic rang of ADC.
  * @note   When higher input signal frequencies close to Nyquist rate, you should set 1.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  rang This parameter can be a value between: 1 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_dynamic_rang(uint32_t rang)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_DYMAMIC_Msk, (rang & 0x7) << AON_SNSADC_CFG_DYMAMIC_Pos);
}

/**
  * @brief  Return ADC dynamic rang.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG2                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be a value between: 1 ~ 7
  */
__STATIC_INLINE uint32_t ll_adc_get_dynamic_rang(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_DYMAMIC_Msk) >> AON_SNSADC_CFG_DYMAMIC_Pos);
}

/**
  * @brief  Set source of ADC input channelP.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG3                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_channelp(uint32_t source)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_CHN_P_Msk, source << AON_SNSADC_CFG_CHN_P_Pos);
}

/**
  * @brief  Return source of ADC input channelP.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG3                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  */
__STATIC_INLINE uint32_t ll_adc_get_channelp(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_CHN_P_Msk) >> AON_SNSADC_CFG_CHN_P_Pos);
}

/**
  * @brief  Set source of ADC input channelN.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG3                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_channeln(uint32_t source)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_CHN_N_Msk, source << AON_SNSADC_CFG_CHN_N_Pos);
}

/**
  * @brief  Return source of ADC input channelN.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG3                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_INPUT_SRC_IO0
  *         @arg @ref LL_ADC_INPUT_SRC_IO1
  *         @arg @ref LL_ADC_INPUT_SRC_IO2
  *         @arg @ref LL_ADC_INPUT_SRC_IO3
  *         @arg @ref LL_ADC_INPUT_SRC_IO4
  *         @arg @ref LL_ADC_INPUT_SRC_TMP
  *         @arg @ref LL_ADC_INPUT_SRC_BAT
  */
__STATIC_INLINE uint32_t ll_adc_get_channeln(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_CHN_N_Msk) >> AON_SNSADC_CFG_CHN_N_Pos);
}

/**
  * @brief  Enable ADC MAS_RST.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_enable_mas_rst(void)
{
    SET_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_MAS_RST_Msk);
}

/**
  * @brief  Disable ADC MAS_RST.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval None
  */
__STATIC_INLINE void ll_adc_disable_mas_rst(void)
{
    CLEAR_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_MAS_RST_Msk);
}

/**
  * @brief  Check if ADC MAS_RST is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_enabled_mas_rst(void)
{
    return (READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_MAS_RST_Msk) == (AON_SNSADC_CFG_MAS_RST_Msk));
}

/**
  * @brief  Set source of ADC reference.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REF_SRC_BUF_INT
  *         @arg @ref LL_ADC_REF_SRC_IO0
  *         @arg @ref LL_ADC_REF_SRC_IO1
  *         @arg @ref LL_ADC_REF_SRC_IO2
  *         @arg @ref LL_ADC_REF_SRC_IO3
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref(uint32_t source)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_REF_SEL_Msk, source);
}

/**
  * @brief  Return source of ADC reference.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REF_SRC_BUF_INT
  *         @arg @ref LL_ADC_REF_SRC_IO0
  *         @arg @ref LL_ADC_REF_SRC_IO1
  *         @arg @ref LL_ADC_REF_SRC_IO2
  *         @arg @ref LL_ADC_REF_SRC_IO3
  */
__STATIC_INLINE uint32_t ll_adc_get_ref(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_REF_SEL_Msk) >> AON_SNSADC_CFG_REF_SEL_Pos);
}

/**
  * @brief  Set current of ADC reference circuit.
  * @note   When samples at 100kbps, you should set 0.
  *         When samples at 1mbps, you should set 7.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  source This parameter can be a value between: 0 ~ 7
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_ref_current(uint32_t source)
{
    MODIFY_REG(AON->SNSADC_CFG, AON_SNSADC_CFG_REF_HP_Msk, (source & 0x7) << AON_SNSADC_CFG_REF_HP_Pos);
}

/**
  * @brief  Return current of ADC reference circuit.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SNSADC_CFG           | REG4                        |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be a value between: 0 ~ 7
  */
__STATIC_INLINE uint32_t ll_adc_get_ref_current(void)
{
    return (uint32_t)(READ_BITS(AON->SNSADC_CFG, AON_SNSADC_CFG_REF_HP_Msk) >> AON_SNSADC_CFG_REF_HP_Pos);
}

/** @} */

/** @defgroup LL_ADC_EF_FIFO_Access FIFO Access
  * @{
  */

/**
  * @brief  Return samples value of ADC by reading FIFO.
  * @note   There are two value in the register, both of them is 16bits.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SENSE_ADC_FIFO       | SENSE_ADC_FIFO              |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Smaples value of input
  */
__STATIC_INLINE uint32_t ll_adc_read_fifo(void)
{
    return (uint32_t)(READ_REG(MCU_SUB->SENSE_ADC_FIFO));
}

/**
  * @brief  Set threshold of ADC FIFO.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SENSE_FF_THRESH      | SENSE_FF_THRESH             |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @param  thresh This parameter can be a value between: 0 ~ 64
  * @retval None
  */
__STATIC_INLINE void ll_adc_set_thresh(uint32_t thresh)
{
    MODIFY_REG(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_THRESH, (thresh & 0x3F) << MCU_SUB_SNSADC_FF_THRESH_Pos);
}

/**
  * @brief  Return threshold of ADC FIFO.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SENSE_FF_THRESH      | SENSE_FF_THRESH             |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be a value between: 0 ~ 64
  */
__STATIC_INLINE uint32_t ll_adc_get_thresh(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_FF_THRESH, MCU_SUB_SNSADC_FF_THRESH) >> MCU_SUB_SNSADC_FF_THRESH_Pos);
}

/**
  * @brief  Check if ADC FIFO is not empty.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SENSE_ADC_STAT       | VAL                         |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_adc_is_fifo_notempty(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_VAL) == MCU_SUB_SNSADC_STAT_VAL);
}

/**
  * @brief  Return count of ADC FIFO.
  *
  *  \rst
  *  +----------------------+-----------------------------+
  *  | Register             | BitsName                    |
  *  +======================+=============================+
  *  | SENSE_ADC_STAT       | FF_COUNT                    |
  *  +----------------------+-----------------------------+
  * \endrst
  *
  * @retval Returned value can be a value between: 0 ~ 64
  */
__STATIC_INLINE uint32_t ll_adc_get_fifo_count(void)
{
    return (uint32_t)(READ_BITS(MCU_SUB->SENSE_ADC_STAT, MCU_SUB_SNSADC_STAT_FF_COUNT) >> MCU_SUB_SNSADC_STAT_FF_COUNT_Pos);
}

/** @} */

/** @defgroup LL_ADC_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize ADC registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR: ADC registers are not de-initialized
  */
error_status_t ll_adc_deinit(void);

/**
  * @brief  Initialize ADC registers according to the specified.
  *         parameters in p_adc_init.
  * @param  p_adc_init Pointer to a ll_adc_init_t structure that contains the configuration
  *                             information for the specified ADC peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: ADC registers are initialized according to p_adc_init content
  *          - ERROR: Problem occurred during ADC Registers initialization
  */
error_status_t ll_adc_init(ll_adc_init_t *p_adc_init);

/**
  * @brief Set each field of a @ref ll_adc_init_t type structure to default value.
  * @param p_adc_init  Pointer to a @ref ll_adc_init_t structure
  *                             whose fields will be set to default values.
  * @retval None
  */
void ll_adc_struct_init(ll_adc_init_t *p_adc_init);

/** @} */

/** @} */

#endif /* AON */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_ADC_H__ */

/** @} */

/** @} */

/** @} */
