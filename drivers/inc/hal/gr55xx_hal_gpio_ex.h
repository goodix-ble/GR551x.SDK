/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_gpio_ex.h
 * @author  BLE Driver Team
 * @brief   Header file containing extended macro of GPIO HAL library.
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

/** @defgroup HAL_GPIOEx GPIOEx
  * @brief GPIOEx HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_GPIO_EX_H__
#define __GR55xx_HAL_GPIO_EX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_gpio.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @defgroup  HAL_GPIOEX_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Constants GPIOEx Exported Constants
  * @{
  */

/** @defgroup GPIOEx_Mux_Mode GPIOEx Mux Mode definition
  * @{
  */
#define GPIO_MUX_0                          LL_GPIO_MUX_0   /**< GPIO Mux mode 0 */
#define GPIO_MUX_1                          LL_GPIO_MUX_1   /**< GPIO Mux mode 1 */
#define GPIO_MUX_2                          LL_GPIO_MUX_2   /**< GPIO Mux mode 2 */
#define GPIO_MUX_3                          LL_GPIO_MUX_3   /**< GPIO Mux mode 3 */
#define GPIO_MUX_4                          LL_GPIO_MUX_4   /**< GPIO Mux mode 4 */
#define GPIO_MUX_5                          LL_GPIO_MUX_5   /**< GPIO Mux mode 5 */
#define GPIO_MUX_6                          LL_GPIO_MUX_6   /**< GPIO Mux mode 6 */
#define GPIO_MUX_7                          LL_GPIO_MUX_7   /**< GPIO Mux mode 7 */
#define GPIO_MUX_8                          LL_GPIO_MUX_8   /**< GPIO Mux mode 8 */
/** @} */

/** @defgroup GPIOEx_Mux_Function_Selection GPIOEx Mux function selection
  * @{
  */

#if defined (GR551xx)
/*---------------------------------- GR551xx ------------------------------*/

/** @defgroup GPIOEx_Common_Selection GPIO PIN common MUX selection(Available for all GPIO pins)
  * @{
  */
#define GPIO_PIN_MUX_TESTBUS                GPIO_MUX_8  /**< GPIO PIN x Mux Select TESTBUS */

#define GPIO_PIN_MUX_GPIO                   GPIO_MUX_7  /**< GPIO PIN x Mux Select GPIO */

/** @} */

/** @defgroup GPIOEx_GPIO0_PIN0_Mux_Selection GPIO0_PIN0 MUX selection
  * @{
  */
#define GPIO0_PIN0_MUX_SWD_CLK              GPIO_MUX_0  /**< GPIO0_PIN0 Mux Select SWD_CLK */
#define GPIO0_PIN0_MUX_I2C0_SCL             GPIO_MUX_1  /**< GPIO0_PIN0 Mux Select I2C0_SCL */
#define GPIO0_PIN0_MUX_I2C1_SCL             GPIO_MUX_2  /**< GPIO0_PIN0 Mux Select I2C1_SCL */
#define GPIO0_PIN0_MUX_UART1_RTS            GPIO_MUX_3  /**< GPIO0_PIN0 Mux Select UART1_RTS */
#define GPIO0_PIN0_MUX_UART0_TX             GPIO_MUX_4  /**< GPIO0_PIN0 Mux Select UART0_TX */
#define GPIO0_PIN0_MUX_UART1_TX             GPIO_MUX_5  /**< GPIO0_PIN0 Mux Select UART1_TX */
#define GPIO0_PIN0_MUX_UART0_RTS            GPIO_MUX_6  /**< GPIO0_PIN0 Mux Select UART0_RTS */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN1_Mux_Selection GPIO0_PIN1 MUX selection
  * @{
  */
#define GPIO0_PIN1_MUX_SWD_IO               GPIO_MUX_0  /**< GPIO0_PIN1 Mux Select SWD_IO */
#define GPIO0_PIN1_MUX_I2C0_SDA             GPIO_MUX_1  /**< GPIO0_PIN1 Mux Select I2C0_SDA */
#define GPIO0_PIN1_MUX_I2C1_SDA             GPIO_MUX_2  /**< GPIO0_PIN1 Mux Select I2C1_SDA */
#define GPIO0_PIN1_MUX_UART1_CTS            GPIO_MUX_3  /**< GPIO0_PIN1 Mux Select UART1_CTS */
#define GPIO0_PIN1_MUX_UART0_RX             GPIO_MUX_4  /**< GPIO0_PIN1 Mux Select UART0_RX */
#define GPIO0_PIN1_MUX_UART1_RX             GPIO_MUX_5  /**< GPIO0_PIN1 Mux Select UART1_RX */
#define GPIO0_PIN1_MUX_UART0_CTS            GPIO_MUX_6  /**< GPIO0_PIN1 Mux Select UART0_CTS */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN2_Mux_Selection GPIO0_PIN2 MUX selection
  * @{
  */
#define GPIO0_PIN2_MUX_UART0_CTS            GPIO_MUX_0  /**< GPIO0_PIN2 Mux Select UART0_CTS */
#define GPIO0_PIN2_MUX_SIM_PRESENCE         GPIO_MUX_1  /**< GPIO0_PIN2 Mux Select SIM_PRESENCE */
#define GPIO0_PIN2_MUX_SWV                  GPIO_MUX_2  /**< GPIO0_PIN2 Mux Select SWV */
#define GPIO0_PIN2_MUX_SPIS_CS_N            GPIO_MUX_3  /**< GPIO0_PIN2 Mux Select SPIS_CS_N */
#define GPIO0_PIN2_MUX_I2C0_SDA             GPIO_MUX_4  /**< GPIO0_PIN2 Mux Select I2C0_SDA */
#define GPIO0_PIN2_MUX_PWM0_A               GPIO_MUX_5  /**< GPIO0_PIN2 Mux Select PWM0_A */
#define GPIO0_PIN2_MUX_FERP_TRIG            GPIO_MUX_6  /**< GPIO0_PIN2 Mux Select FERP_TRIG */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN3_Mux_Selection GPIO0_PIN3 MUX selection
  * @{
  */
#define GPIO0_PIN3_MUX_UART0_TX             GPIO_MUX_0  /**< GPIO0_PIN3 Mux Select UART0_TX */
#define GPIO0_PIN3_MUX_SIM_RST_N            GPIO_MUX_1  /**< GPIO0_PIN3 Mux Select SIM_RST_N */
#define GPIO0_PIN3_MUX_SPIM_CLK             GPIO_MUX_2  /**< GPIO0_PIN3 Mux Select SPIM_CLK */
#define GPIO0_PIN3_MUX_SPIS_CLK             GPIO_MUX_3  /**< GPIO0_PIN3 Mux Select SPIS_CLK */
#define GPIO0_PIN3_MUX_SPIM_CS1             GPIO_MUX_4  /**< GPIO0_PIN3 Mux Select SPIM_CS1 */
#define GPIO0_PIN3_MUX_PWM0_B               GPIO_MUX_5  /**< GPIO0_PIN3 Mux Select PWM0_B */
#define GPIO0_PIN3_MUX_COEX_BLE_TX          GPIO_MUX_6  /**< GPIO0_PIN3 Mux Select COEX_BLE_TX */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN4_Mux_Selection GPIO0_PIN4 MUX selection
  * @{
  */
#define GPIO0_PIN4_MUX_UART0_RX             GPIO_MUX_0  /**< GPIO0_PIN4 Mux Select UART0_RX */
#define GPIO0_PIN4_MUX_SIM_IO               GPIO_MUX_1  /**< GPIO0_PIN4 Mux Select SIM_IO */
#define GPIO0_PIN4_MUX_SPIM_MOSI            GPIO_MUX_2  /**< GPIO0_PIN4 Mux Select SPIM_MOSI */
#define GPIO0_PIN4_MUX_SPIS_MISO            GPIO_MUX_3  /**< GPIO0_PIN4 Mux Select SPIS_MISO */
#define GPIO0_PIN4_MUX_SPIM_CS0             GPIO_MUX_4  /**< GPIO0_PIN4 Mux Select SPIM_CS0 */
#define GPIO0_PIN4_MUX_PWM0_C               GPIO_MUX_5  /**< GPIO0_PIN3 Mux Select PWM0_C */
#define GPIO0_PIN4_MUX_COEX_BLE_RX          GPIO_MUX_6  /**< GPIO0_PIN4 Mux Select COEX_BLE_RX */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN5_Mux_Selection GPIO0_PIN5 MUX selection
  * @{
  */
#define GPIO0_PIN5_MUX_UART0_RTS            GPIO_MUX_0  /**< GPIO0_PIN5 Mux Select UART0_RTS */
#define GPIO0_PIN5_MUX_SIM_CLK              GPIO_MUX_1  /**< GPIO0_PIN5 Mux Select SIM_CLK */
#define GPIO0_PIN5_MUX_SPIM_MISO            GPIO_MUX_2  /**< GPIO0_PIN5 Mux Select SPIM_MISO */
#define GPIO0_PIN5_MUX_SPIS_MOSI            GPIO_MUX_3  /**< GPIO0_PIN5 Mux Select SPIS_MOSI */
//#define GPIO0_PIN5_MUX_SPIM_MISO            GPIO_MUX_4  /**< GPIO0_PIN5 Mux Select SPIM_MISO */
#define GPIO0_PIN5_MUX_I2C0_SCL             GPIO_MUX_5  /**< GPIO0_PIN5 Mux Select I2C0_SCL */
#define GPIO0_PIN5_MUX_COEX_WLAN_TX         GPIO_MUX_6  /**< GPIO0_PIN5 Mux Select COEX_WLAN_TX */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN6_Mux_Selection GPIO0_PIN6 MUX selection
  * @{
  */
#define GPIO0_PIN6_MUX_I2SM_WS              GPIO_MUX_0  /**< GPIO0_PIN6 Mux Select I2S_WS */
#define GPIO0_PIN6_MUX_I2SS_WS              GPIO_MUX_1  /**< GPIO0_PIN6 Mux Select I2S_S_WS */
#define GPIO0_PIN6_MUX_SPIM_CS0             GPIO_MUX_2  /**< GPIO0_PIN6 Mux Select SPIM_CS0 */
#define GPIO0_PIN6_MUX_UART1_RX             GPIO_MUX_3  /**< GPIO0_PIN6 Mux Select UART1_RX */
#define GPIO0_PIN6_MUX_SPIM_MOSI            GPIO_MUX_4  /**< GPIO0_PIN6 Mux Select SPIM_MOSI */
#define GPIO0_PIN6_MUX_I2C0_SDA             GPIO_MUX_5  /**< GPIO0_PIN6 Mux Select I2C0_SDA */
#define GPIO0_PIN6_MUX_COEX_WLAN_RX         GPIO_MUX_6  /**< GPIO0_PIN6 Mux Select COEX_WLAN_RX */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN7_Mux_Selection GPIO0_PIN7 MUX selection
  * @{
  */
#define GPIO0_PIN7_MUX_I2SM_TX_SDO          GPIO_MUX_0  /**< GPIO0_PIN7 Mux Select I2SM_TX_SDO */
#define GPIO0_PIN7_MUX_I2SS_TX_SDO          GPIO_MUX_1  /**< GPIO0_PIN7 Mux Select I2SS_TX_SDO */
#define GPIO0_PIN7_MUX_SPIM_CS1             GPIO_MUX_2  /**< GPIO0_PIN7 Mux Select SPIM_CS1 */
#define GPIO0_PIN7_MUX_UART1_TX             GPIO_MUX_3  /**< GPIO0_PIN7 Mux Select UART1_TX */
#define GPIO0_PIN7_MUX_SPIM_CLK             GPIO_MUX_4  /**< GPIO0_PIN7 Mux Select SPIM_CLK */
#define GPIO0_PIN7_MUX_PWM1_A               GPIO_MUX_5  /**< GPIO0_PIN7 Mux Select PWM1_A */
#define GPIO0_PIN7_MUX_COEX_BLE_PROC        GPIO_MUX_6  /**< GPIO0_PIN7 Mux Select COEX_BLE_PROC */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN8_Mux_Selection GPIO0_PIN8 MUX selection
  * @{
  */
#define GPIO0_PIN8_MUX_XQSPIM_IO_0          GPIO_MUX_0  /**< GPIO0_PIN8 Mux Select XQSPIM_IO_0 */
#define GPIO0_PIN8_MUX_I2C1_SDA             GPIO_MUX_1  /**< GPIO0_PIN8 Mux Select I2C1_SDA */
#define GPIO0_PIN8_MUX_QSPIM1_IO_0          GPIO_MUX_2  /**< GPIO0_PIN8 Mux Select QSPIM1_IO_0 */
#define GPIO0_PIN8_MUX_UART1_RX             GPIO_MUX_3  /**< GPIO0_PIN8 Mux Select UART1_RX */
#define GPIO0_PIN8_MUX_PWM1_B               GPIO_MUX_5  /**< GPIO0_PIN8 Mux Select PWM1_B */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN9_Mux_Selection GPIO0_PIN9 MUX selection
  * @{
  */
#define GPIO0_PIN9_MUX_XQSPIM_CLK           GPIO_MUX_0  /**< GPIO0_PIN9 Mux Select XQSPIM_CLK */
#define GPIO0_PIN9_MUX_I2C1_SCL             GPIO_MUX_1  /**< GPIO0_PIN9 Mux Select I2C1_SCL */
#define GPIO0_PIN9_MUX_QSPIM1_CLK           GPIO_MUX_2  /**< GPIO0_PIN9 Mux Select QSPIM1_CLK */
#define GPIO0_PIN9_MUX_UART1_TX             GPIO_MUX_3  /**< GPIO0_PIN9 Mux Select UART1_TX */
#define GPIO0_PIN9_MUX_PWM1_C               GPIO_MUX_5  /**< GPIO0_PIN9 Mux Select PWM1_C */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN10_Mux_Selection GPIO0_PIN10 MUX selection
  * @{
  */
#define GPIO0_PIN10_MUX_I2SM_RX_SDI         GPIO_MUX_0  /**< GPIO0_PIN10 Mux Select I2SM_RX_SDI */
#define GPIO0_PIN10_MUX_I2SS_RX_SDI         GPIO_MUX_1  /**< GPIO0_PIN10 Mux Select I2SS_RX_SDI */
#define GPIO0_PIN10_MUX_UART0_TX            GPIO_MUX_2  /**< GPIO0_PIN10 Mux Select UART0_TX */
#define GPIO0_PIN10_MUX_I2C0_SCL            GPIO_MUX_4  /**< GPIO0_PIN10 Mux Select I2C0_SCL */
#define GPIO0_PIN10_MUX_PWM1_B              GPIO_MUX_5  /**< GPIO0_PIN10 Mux Select PWM1_B */
#define GPIO0_PIN10_MUX_COEX_BLE_TX         GPIO_MUX_6  /**< GPIO0_PIN10 Mux Select COEX_BLE_TX */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN11_Mux_Selection GPIO0_PIN11 MUX selection
  * @{
  */
#define GPIO0_PIN11_MUX_I2SM_SCLK           GPIO_MUX_0  /**< GPIO0_PIN10 Mux Select I2SM_SCLK */
#define GPIO0_PIN11_MUX_I2SS_SCLK           GPIO_MUX_1  /**< GPIO0_PIN10 Mux Select I2SS_SCLK */
#define GPIO0_PIN11_MUX_UART0_RX            GPIO_MUX_2  /**< GPIO0_PIN10 Mux Select UART0_RX */
#define GPIO0_PIN11_MUX_I2C0_SDA            GPIO_MUX_4  /**< GPIO0_PIN10 Mux Select I2C0_SDA */
#define GPIO0_PIN11_MUX_PWM1_C              GPIO_MUX_5  /**< GPIO0_PIN10 Mux Select PWM1_C */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN12_Mux_Selection GPIO0_PIN12 MUX selection
  * @{
  */
#define GPIO0_PIN12_MUX_XQSPIM_IO_3         GPIO_MUX_0  /**< GPIO0_PIN12 Mux Select XQSPIM_IO_3 */
#define GPIO0_PIN12_MUX_SPIM_CLK            GPIO_MUX_1  /**< GPIO0_PIN12 Mux Select SPIM_CLK */
#define GPIO0_PIN12_MUX_QSPIM1_IO3          GPIO_MUX_2  /**< GPIO0_PIN12 Mux Select QSPIM1_IO3 */
#define GPIO0_PIN12_MUX_SIM_PRESENCE        GPIO_MUX_3  /**< GPIO0_PIN12 Mux Select SIM_PRESENCE */
#define GPIO0_PIN12_MUX_I2SM_WS             GPIO_MUX_4  /**< GPIO0_PIN12 Mux Select I2SM_WS */
#define GPIO0_PIN12_MUX_I2SS_WS             GPIO_MUX_5  /**< GPIO0_PIN12 Mux Select I2SS_WS */
#define GPIO0_PIN12_MUX_SPIS_CS             GPIO_MUX_6  /**< GPIO0_PIN12 Mux Select I2SS_WS */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN13_Mux_Selection GPIO0_PIN13 MUX selection
  * @{
  */
#define GPIO0_PIN13_MUX_XQSPIM_IO_2         GPIO_MUX_0  /**< GPIO0_PIN13 Mux Select XQSPIM_IO_2 */
#define GPIO0_PIN13_MUX_SPIM_MOSI           GPIO_MUX_1  /**< GPIO0_PIN13 Mux Select SPIM_MOSI */
#define GPIO0_PIN13_MUX_QSPIM1_IO_2         GPIO_MUX_2  /**< GPIO0_PIN13 Mux Select QSPIM1_IO_2 */
#define GPIO0_PIN13_MUX_SIM_RST_N           GPIO_MUX_3  /**< GPIO0_PIN13 Mux Select SIM_RST_N */
#define GPIO0_PIN13_MUX_I2SM_TX_SDO         GPIO_MUX_4  /**< GPIO0_PIN13 Mux Select I2SM_TX_SDO */
#define GPIO0_PIN13_MUX_I2SS_TX_SDO         GPIO_MUX_5  /**< GPIO0_PIN13 Mux Select I2SS_TX_SDO */
#define GPIO0_PIN13_MUX_SPIS_CLK            GPIO_MUX_6  /**< GPIO0_PIN13 Mux Select SPIS_CLK */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN14_Mux_Selection GPIO0_PIN14 MUX selection
  * @{
  */
#define GPIO0_PIN14_MUX_XQSPIM_IO_1         GPIO_MUX_0  /**< GPIO0_PIN14 Mux Select XQSPIM_IO_1 */
#define GPIO0_PIN14_MUX_SPIM_MISO           GPIO_MUX_1  /**< GPIO0_PIN14 Mux Select SPIM_MISO */
#define GPIO0_PIN14_MUX_QSPIM1_IO1          GPIO_MUX_2  /**< GPIO0_PIN14 Mux Select QSPIM1_IO1 */
#define GPIO0_PIN14_MUX_SIM_IO              GPIO_MUX_3  /**< GPIO0_PIN14 Mux Select SIM_IO */
#define GPIO0_PIN14_MUX_I2SM_RX_SDI         GPIO_MUX_4  /**< GPIO0_PIN14 Mux Select I2SM_RX_SDI */
#define GPIO0_PIN14_MUX_I2SS_RX_SDI         GPIO_MUX_5  /**< GPIO0_PIN14 Mux Select I2SS_RX_SDI */
#define GPIO0_PIN14_MUX_SPIS_MISO           GPIO_MUX_6  /**< GPIO0_PIN14 Mux Select SPIS_MISO */
/** @} */

/** @defgroup GPIOEx_GPIO0_PIN15_Mux_Selection GPIO0_PIN15 MUX selection
  * @{
  */
#define GPIO0_PIN15_MUX_XQSPIM_CS_N         GPIO_MUX_0  /**< GPIO0_PIN15 Mux Select QSPIM_CS_N */
#define GPIO0_PIN15_MUX_SPIM_CS0            GPIO_MUX_1  /**< GPIO0_PIN15 Mux Select SPIM_CS0 */
#define GPIO0_PIN15_MUX_QSPIM1_CS_N         GPIO_MUX_2  /**< GPIO0_PIN15 Mux Select QSPIM1_CS_N */
#define GPIO0_PIN15_MUX_SIM_CLK             GPIO_MUX_3  /**< GPIO0_PIN15 Mux Select SIM_CLK */
#define GPIO0_PIN15_MUX_I2SM_SCLK           GPIO_MUX_4  /**< GPIO0_PIN15 Mux Select I2SM_SCLK */
#define GPIO0_PIN15_MUX_I2SS_SCLK           GPIO_MUX_5  /**< GPIO0_PIN15 Mux Select I2SS_SCLK */
#define GPIO0_PIN15_MUX_SPIS_MOSI           GPIO_MUX_6  /**< GPIO0_PIN15 Mux Select SPIS_MOSI */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN0_Mux_Selection GPIO1_PIN0 MUX selection
  * @{
  */
#define GPIO1_PIN0_MUX_SPIM_MISO            GPIO_MUX_0  /**< GPIO1_PIN0 Mux Select SPIM_MISO */
#define GPIO1_PIN0_MUX_SPIS_MOSI            GPIO_MUX_1  /**< GPIO1_PIN0 Mux Select SPIS_MOSI */
#define GPIO1_PIN0_MUX_SIM_IO               GPIO_MUX_2  /**< GPIO1_PIN0 Mux Select SIM_IO */
#define GPIO1_PIN0_MUX_I2SM_RX_SDI          GPIO_MUX_3  /**< GPIO1_PIN0 Mux Select I2SM_RX_SDI */
#define GPIO1_PIN0_MUX_I2SS_RX_SDI          GPIO_MUX_4  /**< GPIO1_PIN0 Mux Select I2SS_RX_SDI */
#define GPIO1_PIN0_MUX_QSPIM0_IO_1          GPIO_MUX_5  /**< GPIO1_PIN0 Mux Select QSPIM0_IO_1 */
#define GPIO1_PIN0_MUX_ISO_SYNC             GPIO_MUX_6  /**< GPIO1_PIN0 Mux Select ISO_SYNC */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN1_Mux_Selection GPIO1_PIN1 MUX selection
  * @{
  */
#define GPIO1_PIN1_MUX_SPIM_CS0             GPIO_MUX_0  /**< GPIO1_PIN1 Mux Select SPIM_CS0 */
#define GPIO1_PIN1_MUX_SPIS_CS              GPIO_MUX_1  /**< GPIO1_PIN1 Mux Select SPIS_CS */
#define GPIO1_PIN1_MUX_SIM_CLK              GPIO_MUX_2  /**< GPIO1_PIN1 Mux Select SIM_CLK */
#define GPIO1_PIN1_MUX_I2SM_SCLK            GPIO_MUX_3  /**< GPIO1_PIN1 Mux Select I2SM_SCLK */
#define GPIO1_PIN1_MUX_I2SS_SCLK            GPIO_MUX_4  /**< GPIO1_PIN1 Mux Select I2SS_SCLK */
#define GPIO1_PIN1_MUX_QSPIM0_IO_2          GPIO_MUX_5  /**< GPIO1_PIN1 Mux Select QSPIM0_IO_2 */
#define GPIO1_PIN1_MUX_COEX_BLE_RX          GPIO_MUX_6  /**< GPIO1_PIN1 Mux Select COEX_BLE_RX */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN2_Mux_Selection GPIO1_PIN2 MUX selection
  * @{
  */
#define GPIO1_PIN2_MUX_QSPIM0_CS_N          GPIO_MUX_0  /**< GPIO1_PIN2 Mux Select QSPIM0_CS_N */
#define GPIO1_PIN2_MUX_XQSPIM_CS_N          GPIO_MUX_1  /**< GPIO1_PIN2 Mux Select XQSPIM_CS_N */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN3_Mux_Selection GPIO1_PIN3 MUX selection
  * @{
  */
#define GPIO1_PIN3_MUX_QSPIM0_IO_3          GPIO_MUX_0  /**< GPIO1_PIN3 Mux Select QSPIM0_IO_3 */
#define GPIO1_PIN3_MUX_XQSPIM_IO_3          GPIO_MUX_1  /**< GPIO1_PIN3 Mux Select XQSPIM_IO_3 */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN4_Mux_Selection GPIO1_PIN4 MUX selection
  * @{
  */
#define GPIO1_PIN4_MUX_QSPIM0_CLK           GPIO_MUX_0  /**< GPIO1_PIN4 Mux Select QSPIM0_CLK */
#define GPIO1_PIN4_MUX_XQSPIM_CLK           GPIO_MUX_1  /**< GPIO1_PIN4 Mux Select XQSPIM_CLK */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN5_Mux_Selection GPIO1_PIN5 MUX selection
  * @{
  */
#define GPIO1_PIN5_MUX_QSPIM0_IO_2          GPIO_MUX_0  /**< GPIO1_PIN5 Mux Select QSPIM0_IO_2 */
#define GPIO1_PIN5_MUX_XQSPIM_IO_2          GPIO_MUX_1  /**< GPIO1_PIN5 Mux Select XQSPIM_IO_2 */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN6_Mux_Selection GPIO1_PIN6 MUX selection
  * @{
  */
#define GPIO1_PIN6_MUX_QSPIM0_IO_1          GPIO_MUX_0  /**< GPIO1_PIN6 Mux Select QSPIM0_IO_1 */
#define GPIO1_PIN6_MUX_XQSPIM_IO_1          GPIO_MUX_1  /**< GPIO1_PIN6 Mux Select XQSPIM_IO_1 */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN7_Mux_Selection GPIO1_PIN7 MUX selection
  * @{
  */
#define GPIO1_PIN7_MUX_QSPIM0_IO_0          GPIO_MUX_0  /**< GPIO1_PIN7 Mux Select QSPIM0_IO_0 */
#define GPIO1_PIN7_MUX_XQSPIM_IO_0          GPIO_MUX_1  /**< GPIO1_PIN7 Mux Select XQSPIM_IO_0 */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN8_Mux_Selection GPIO1_PIN8 MUX selection
  * @{
  */
#define GPIO1_PIN8_MUX_SPIM_CLK             GPIO_MUX_0  /**< GPIO1_PIN8 Mux Select SPIM_CLK */
#define GPIO1_PIN8_MUX_SPIS_CLK             GPIO_MUX_1  /**< GPIO1_PIN8 Mux Select SPIS_CLK */
#define GPIO1_PIN8_MUX_SIM_PRESENCE         GPIO_MUX_2  /**< GPIO1_PIN8 Mux Select SIM_PRESENCE */
#define GPIO1_PIN8_MUX_I2SM_WS              GPIO_MUX_3  /**< GPIO1_PIN8 Mux Select I2SM_WS */
#define GPIO1_PIN8_MUX_I2SS_WS              GPIO_MUX_4  /**< GPIO1_PIN8 Mux Select I2SS_WS */
#define GPIO1_PIN8_MUX_QSPIM0_CLK           GPIO_MUX_5  /**< GPIO1_PIN8 Mux Select QSPIM0_CLK */
#define GPIO1_PIN8_MUX_COEX_WLAN_TX         GPIO_MUX_6  /**< GPIO1_PIN8 Mux Select COEX_WLAN_TX */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN9_Mux_Selection GPIO1_PIN9 MUX selection
  * @{
  */
#define GPIO1_PIN9_MUX_SPIM_MOSI            GPIO_MUX_0  /**< GPIO1_PIN9 Mux Select SPIM_MOSI */
#define GPIO1_PIN9_MUX_SPIS_MISO            GPIO_MUX_1  /**< GPIO1_PIN9 Mux Select SPIS_MISO */
#define GPIO1_PIN9_MUX_SIM_RST_N            GPIO_MUX_2  /**< GPIO1_PIN9 Mux Select SIM_RST_N */
#define GPIO1_PIN9_MUX_I2SM_TX_SD0          GPIO_MUX_3  /**< GPIO1_PIN9 Mux Select I2SM_TX_SD0 */
#define GPIO1_PIN9_MUX_I2SS_TX_SD0          GPIO_MUX_4  /**< GPIO1_PIN9 Mux Select I2SS_TX_SD0 */
#define GPIO1_PIN9_MUX_QSPIM0_IO_0          GPIO_MUX_5  /**< GPIO1_PIN9 Mux Select QSPIM0_IO_0 */
#define GPIO1_PIN9_MUX_COEX_BLE_PROC        GPIO_MUX_6  /**< GPIO1_PIN9 Mux Select COEX_BLE_PROC */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN10_Mux_Selection GPIO1_PIN10 MUX selection
  * @{
  */
#define GPIO1_PIN10_MUX_I2C1_SDA            GPIO_MUX_0  /**< GPIO1_PIN10 Mux Select I2C1_SDA */
#define GPIO1_PIN10_MUX_UART1_RX            GPIO_MUX_1  /**< GPIO1_PIN10 Mux Select UART1_RX */
#define GPIO1_PIN10_MUX_I2C0_SDA            GPIO_MUX_2  /**< GPIO1_PIN10 Mux Select I2C0_SDA */
#define GPIO1_PIN10_MUX_PWM0_C              GPIO_MUX_3  /**< GPIO1_PIN10 Mux Select PWM0_C */
#define GPIO1_PIN10_MUX_PWM1_C                GPIO_MUX_4  /**< GPIO1_PIN10 Mux Select PWM5 */
#define GPIO1_PIN10_MUX_UART0_RX            GPIO_MUX_5  /**< GPIO1_PIN10 Mux Select UART0_RX */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN11_Mux_Selection GPIO1_PIN11 MUX selection
  * @{
  */
#define GPIO1_PIN11_MUX_UART1_RTS           GPIO_MUX_1  /**< GPIO1_PIN11 Mux Select UART1_RTS */
#define GPIO1_PIN11_MUX_UART0_RTS           GPIO_MUX_5  /**< GPIO1_PIN11 Mux Select UART0_RTS */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN12_Mux_Selection GPIO1_PIN12 MUX selection
  * @{
  */
#define GPIO1_PIN12_MUX_UART1_CTS           GPIO_MUX_1  /**< GPIO1_PIN12 Mux Select UART1_CTS */
#define GPIO1_PIN12_MUX_UART0_CTS           GPIO_MUX_5  /**< GPIO1_PIN12 Mux Select UART0_CTS */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN13_Mux_Selection GPIO1_PIN13 MUX selection
  * @{
  */

/** @} */

/** @defgroup GPIOEx_GPIO1_PIN14_Mux_Selection GPIO1_PIN14 MUX selection
  * @{
  */
#define GPIO1_PIN14_MUX_I2C1_SCL            GPIO_MUX_0  /**< GPIO1_PIN14 Mux Select I2C1_SCL */
#define GPIO1_PIN14_MUX_UART1_TX            GPIO_MUX_1  /**< GPIO1_PIN14 Mux Select UART1_TX */
#define GPIO1_PIN14_MUX_I2C0_SCL            GPIO_MUX_2  /**< GPIO1_PIN14 Mux Select I2C0_SCL */
#define GPIO1_PIN14_MUX_PWM0_B              GPIO_MUX_3  /**< GPIO1_PIN14 Mux Select PWM0_B */
#define GPIO1_PIN14_MUX_PWM1_B              GPIO_MUX_4  /**< GPIO1_PIN14 Mux Select PWM1_B */
#define GPIO1_PIN14_MUX_UART0_TX            GPIO_MUX_5  /**< GPIO1_PIN14 Mux Select UART0_TX */
#define GPIO1_PIN14_MUX_COEX_BLE_TX         GPIO_MUX_6  /**< GPIO1_PIN14 Mux Select COEX_BLE_TX */
/** @} */

/** @defgroup GPIOEx_GPIO1_PIN15_Mux_Selection GPIO1_PIN15 MUX selection
  * @{
  */
#define GPIO1_PIN15_MUX_SPIM_CS1            GPIO_MUX_0  /**< GPIO1_PIN15 Mux Select SPIM_CS1 */
#define GPIO1_PIN15_MUX_PWM0_A              GPIO_MUX_3  /**< GPIO1_PIN15 Mux Select PWM0_A */
#define GPIO1_PIN15_MUX_PWM1_A              GPIO_MUX_4  /**< GPIO1_PIN15 Mux Select PWM1_A */
#define GPIO1_PIN15_MUX_QSPIM0_IO_3         GPIO_MUX_5  /**< GPIO1_PIN15 Mux Select QSPIM0_IO_3 */
#define GPIO1_PIN15_MUX_COEX_WLAN_TX        GPIO_MUX_6  /**< GPIO1_PIN15 Mux Select COEX_WLAN_TX */
/** @} */

/**
  * @brief Check if GPIO Mux mode is valid.
  * @param __MUX__ GPIO mux mode.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_GPIO_MUX(__MUX__)        (((__MUX__) <= GPIO_MUX_8))

/*------------------------------------------------------------------------------------------*/
#endif /* GR551xx */

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Macros GPIOEx Exported Macros
  * @{
  */

/** @defgroup GPIOEx_Get_Port_Index GPIOEx_Get Port Index
* @{
  */
#if defined(GR551xx)

/**
  * @brief Get GPIO Port Index.
  * @param __GPIOx__ GPIO instance.
  * @retval Port Index.
  */
#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIO0))? 0U : 1U)

#endif /* GR551xx */
/** @} */

/** @defgroup GPIOEx_Get_Port_IRQNum GPIOEx_Get Port IRQ number
* @{
  */
#if defined(GR551xx)

/**
  * @brief Get GPIO Port IRQ number.
  * @param __GPIOx__ GPIO instance.
  * @retval Port IRQ number.
  */
#define GPIO_GET_IRQNUM(__GPIOx__)    (((__GPIOx__) == (GPIO0))? EXT0_IRQn : EXT1_IRQn)

#endif /* GR551xx */

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_GPIO_EX_H__ */

/** @} */

/** @} */

/** @} */
