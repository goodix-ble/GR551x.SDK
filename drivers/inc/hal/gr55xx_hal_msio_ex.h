/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_msio_ex.h
 * @author  BLE Driver Team
 * @brief   Header file containing extended macro of MSIO HAL library.
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

/** @defgroup HAL_MSIOEx MSIOEx
  * @brief MSIOEx HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_MSIO_EX_H__
#define __GR55xx_HAL_MSIO_EX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_msio.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @defgroup  HAL_MSIOEX_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup MSIOEx_Exported_Constants MSIOEx Exported Constants
  * @{
  */

/** @defgroup MSIOEx_Mux_Mode MSIOEx Mux Mode definition
  * @{
  */
#define MSIO_MUX_0                      LL_MSIO_MUX_0   /**< MSIO mux mode 0 */
#define MSIO_MUX_1                      LL_MSIO_MUX_1   /**< MSIO mux mode 1 */
#define MSIO_MUX_2                      LL_MSIO_MUX_2   /**< MSIO mux mode 2 */
#define MSIO_MUX_3                      LL_MSIO_MUX_3   /**< MSIO mux mode 3 */
#define MSIO_MUX_4                      LL_MSIO_MUX_4   /**< MSIO mux mode 4 */
#define MSIO_MUX_5                      LL_MSIO_MUX_5   /**< MSIO mux mode 5 */
#define MSIO_MUX_6                      LL_MSIO_MUX_6   /**< MSIO mux mode 6 */
#define MSIO_MUX_7                      LL_MSIO_MUX_7   /**< MSIO mux mode 7 */
/** @} */

/** @defgroup MSIOEx_Mux_Function_Selection MSIOEx Mux function selection
  * @{
  */

#if defined (GR551xx)
/*---------------------------------- GR551xx ------------------------------*/

/** @defgroup MSIOEx_Common_Selection MSIO PIN common MUX selection(Available for all MSIO pins)
  * @{
  */

#define MSIO_PIN_MUX_GPIO               MSIO_MUX_7  /**< MSIO PIN x Mux Select GPIO */

/** @} */

/** @defgroup MSIOEx_PIN0_Mux_Selection MSIO_PIN0 MUX selection
  * @{
  */
#define MSIO_PIN0_MUX_PWM0_A         MSIO_MUX_0  /**< MSIO_PIN0 Mux Select PWM0_A */
#define MSIO_PIN0_MUX_UART0_TX       MSIO_MUX_1  /**< MSIO_PIN0 Mux Select UART0_TX */
#define MSIO_PIN0_MUX_UART1_TX       MSIO_MUX_2  /**< MSIO_PIN0 Mux Select UART1_TX */
#define MSIO_PIN0_MUX_I2C0_SCL       MSIO_MUX_3  /**< MSIO_PIN0 Mux Select I2C0_SCL */
#define MSIO_PIN0_MUX_I2C1_SCL       MSIO_MUX_4  /**< MSIO_PIN0 Mux Select I2C1_SCL */
/** @} */

/** @defgroup MSIOEx_PIN1_Mux_Selection MSIO_PIN1 MUX selection
  * @{
  */
#define MSIO_PIN1_MUX_PWM0_B         MSIO_MUX_0  /**< MSIO_PIN1 Mux Select PWM0_B */
#define MSIO_PIN1_MUX_UART0_RX       MSIO_MUX_1  /**< MSIO_PIN1 Mux Select UART0_RX */
#define MSIO_PIN1_MUX_UART1_RX       MSIO_MUX_2  /**< MSIO_PIN1 Mux Select UART1_RX */
#define MSIO_PIN1_MUX_I2C0_SDA       MSIO_MUX_3  /**< MSIO_PIN1 Mux Select I2C0_SDA */
#define MSIO_PIN1_MUX_I2C1_SDA       MSIO_MUX_4  /**< MSIO_PIN1 Mux Select I2C1_SDA */
/** @} */

/** @defgroup MSIOEx_PIN2_Mux_Selection MSIO_PIN2 MUX selection
  * @{
  */
#define MSIO_PIN2_MUX_PWM0_C         MSIO_MUX_0  /**< MSIO_PIN2 Mux Select PWM0_C */
/** @} */

/** @defgroup MSIOEx_PIN3_Mux_Selection MSIO_PIN3 MUX selection
  * @{
  */
#define MSIO_PIN3_MUX_PWM1_A         MSIO_MUX_0  /**< MSIO_PIN3 Mux Select PWM1_A */
#define MSIO_PIN3_MUX_UART0_RTS      MSIO_MUX_1  /**< MSIO_PIN3 Mux Select UART0_RTS */
#define MSIO_PIN3_MUX_UART1_RTS      MSIO_MUX_2  /**< MSIO_PIN3 Mux Select UART1_RTS */
#define MSIO_PIN3_MUX_I2C0_SCL       MSIO_MUX_3  /**< MSIO_PIN3 Mux Select I2C0_SCL */
#define MSIO_PIN3_MUX_I2C1_SCL       MSIO_MUX_4  /**< MSIO_PIN3 Mux Select I2C1_SCL */
/** @} */

/** @defgroup MSIOEx_PIN4_Mux_Selection MSIO_PIN4 MUX selection
  * @{
  */
#define MSIO_PIN4_MUX_PWM1_B         MSIO_MUX_0  /**< MSIO_PIN4 Mux Select PWM1_B */
#define MSIO_PIN4_MUX_UART0_CTS      MSIO_MUX_1  /**< MSIO_PIN4 Mux Select UART0_CTS */
#define MSIO_PIN4_MUX_UART1_CTS      MSIO_MUX_2  /**< MSIO_PIN4 Mux Select UART1_CTS */
#define MSIO_PIN4_MUX_I2C0_SDA       MSIO_MUX_3  /**< MSIO_PIN4 Mux Select I2C0_SDA */
#define MSIO_PIN4_MUX_I2C1_SDA       MSIO_MUX_4  /**< MSIO_PIN4 Mux Select I2C1_SDA */
/** @} */

/**
  * @brief Check if MSIO mux mode is valid.
  * @param __MUX__ MSIO mux mode.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_MSIO_MUX(__MUX__)        (((__MUX__) <= MSIO_MUX_7))

/*------------------------------------------------------------------------------------------*/
#endif /* GR551xx */

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_MSIO_EX_H__ */

/** @} */

/** @} */

/** @} */

