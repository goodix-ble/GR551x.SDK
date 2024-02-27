/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_cgc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of CGC LL library.
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

/** @defgroup LL_CGC CGC
  * @brief CGC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_CGC_H__
#define __GR55XX_LL_CGC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined(MCU_SUB)

/** @defgroup CGC_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup CGC_LL_ES_INIT CGC Exported init structures
  * @{
  */

/**
  * @brief LL CGC init Structure definition
  */
typedef struct _ll_cgc_init_t
{
    uint32_t wfi_clk0;          /**< Specifies the block that automatically closes the clock.
                                    This parameter can be a combination of @ref CGC_LL_EC_WFI_CLK0. */

    uint32_t wfi_clk1;          /**< Specifies the block that automatically closes the clock.
                                    This parameter can be a combination of @ref CGC_LL_EC_WFI_CLK1. */

    uint32_t wfi_clk2;          /**< Specifies the block that automatically closes the clock.
                                    This parameter can be a combination of @ref CGC_LL_EC_WFI_CLK2. */

    uint32_t force_clk0;        /**< Specifies the blocks for forced turn off clock.
                                    This parameter can be a combination of @ref CGC_LL_EC_FRC_CLK0. */

    uint32_t force_clk1;        /**< Specifies the blocks for forced turn off clock.
                                    This parameter can be a combination of @ref CGC_LL_EC_FRC_CLK1. */

    uint32_t force_clk2;        /**< Specifies the blocks for forced turn off clock.
                                    This parameter can be a combination of @ref CGC_LL_EC_FRC_CLK2. */         
} ll_cgc_init_t;

/** @} */

/** @} */

/**
  * @defgroup  CGC_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CGC_LL_Exported_Constants CGC Exported Constants
  * @{
  */

/** @defgroup CGC_LL_EC_WFI_CLK0 Block0 Clock During WFI 
  * @{
  */
#define LL_CGC_WFI_SECU_HCLK                    MCU_SUB_WFI_SECU_HCLK         /**< Hclk for all security blocks */
#define LL_CGC_WFI_SIM_HCLK                     MCU_SUB_WFI_SIM_HCLK          /**< Hclk for sim card interface  */
#define LL_CGC_WFI_HTB_HCLK                     MCU_SUB_WFI_HTB_HCLK          /**< Hclk for hopping table       */
#define LL_CGC_WFI_PWM_HCLK                     MCU_SUB_WFI_PWM_HCLK          /**< Hclk for PWM                 */
#define LL_CGC_WFI_ROM_HCLK                     MCU_SUB_WFI_ROM_HCLK          /**< Hclk for ROM                 */
#define LL_CGC_WFI_SNSADC_HCLK                  MCU_SUB_WFI_SNSADC_HCLK       /**< Hclk for sense ADC           */
#define LL_CGC_WFI_GPIO_HCLK                    MCU_SUB_WFI_GPIO_HCLK         /**< Hclk for GPIOs               */
#define LL_CGC_WFI_DMA_HCLK                     MCU_SUB_WFI_DMA_HCLK          /**< Hclk for DMA engine          */
#define LL_CGC_WFI_BLE_BRG_HCLK                 MCU_SUB_WFI_BLE_BRG_HCLK      /**< Hclk for BLE MCU bridge      */
#define LL_CGC_WFI_APB_SUB_HCLK                 MCU_SUB_WFI_APB_SUB_HCLK      /**< Hclk for APB subsystem       */
#define LL_CGC_WFI_SERIAL_HCLK                  MCU_SUB_WFI_SERIAL_HCLK       /**< Hclk for serial blocks       */
#define LL_CGC_WFI_I2S_S_HCLK                   MCU_SUB_WFI_I2S_S_HCLK        /**< Hclk for I2S slave           */

#define LL_CGC_WFI_ALL_HCLK0                    ((uint32_t)0x00000FFFU)       /**< All clock group 0            */
/** @} */

/** @defgroup CGC_LL_EC_WFI_CLK1 Block1 Clock During WFI 
  * @{
  */
#define LL_CGC_WFI_AON_MCUSUB_HCLK              MCU_SUB_WFI_AON_MCUSUB_HCLK   /**< Hclk for Always-on register  */
#define LL_CGC_WFI_XF_XQSPI_HCLK                MCU_SUB_WFI_XF_XQSPI_HCLK     /**< Hclk for cache top           */
#define LL_CGC_WFI_SRAM_HCLK                    MCU_SUB_WFI_SRAM_HCLK         /**< Hclk for SRAMs               */

#define LL_CGC_WFI_ALL_HCLK1                    ((uint32_t)0x00000007U)       /**< All clock group 1            */
/** @} */

/** @defgroup CGC_LL_EC_WFI_CLK2 Block2 Clock During WFI 
  * @{
  */
#define LL_CGC_WFI_SECU_DIV4_PCLK               MCU_SUB_WFI_SECU_DIV4_PCLK    /**< Div4 clk for security blocks */
#define LL_CGC_WFI_XQSPI_DIV4_PCLK              MCU_SUB_WFI_XQSPI_DIV4_PCLK   /**< Div4 clk for xf qspi         */

#define LL_CGC_WFI_ALL_HCLK2                    ((uint32_t)0x05000000U)       /**< All clock group 2            */
/** @} */


/** @defgroup CGC_LL_EC_FRC_CLK0 Force Clock OFF
  * @{
  */
#define LL_CGC_FRC_SECU_HCLK                    MCU_SUB_FORCE_SECU_HCLK       /**< Hclk for all security blocks */
#define LL_CGC_FRC_SIM_HCLK                     MCU_SUB_FORCE_SIM_HCLK        /**< Hclk for sim card interface  */
#define LL_CGC_FRC_HTB_HCLK                     MCU_SUB_FORCE_HTB_HCLK        /**< Hclk for hopping table       */
#define LL_CGC_FRC_PWM_HCLK                     MCU_SUB_FORCE_PWM_HCLK        /**< Hclk for PWM                 */
#define LL_CGC_FRC_ROM_HCLK                     MCU_SUB_FORCE_ROM_HCLK        /**< Hclk for ROM                 */
#define LL_CGC_FRC_SNSADC_HCLK                  MCU_SUB_FORCE_SNSADC_HCLK     /**< Hclk for sense ADC           */
#define LL_CGC_FRC_GPIO_HCLK                    MCU_SUB_FORCE_GPIO_HCLK       /**< Hclk for GPIOs               */
#define LL_CGC_FRC_DMA_HCLK                     MCU_SUB_FORCE_DMA_HCLK        /**< Hclk for DMA engine          */
#define LL_CGC_FRC_BLE_BRG_HCLK                 MCU_SUB_FORCE_BLE_BRG_HCLK    /**< Hclk for BLE MCU bridge      */
#define LL_CGC_FRC_APB_SUB_HCLK                 MCU_SUB_FORCE_APB_SUB_HCLK    /**< Hclk for APB subsystem       */
#define LL_CGC_FRC_SERIAL_HCLK                  MCU_SUB_FORCE_SERIAL_HCLK     /**< Hclk for serial blocks       */
#define LL_CGC_FRC_I2S_S_HCLK                   MCU_SUB_FORCE_I2S_S_HCLK      /**< Hclk for I2S slave           */

#define LL_CGC_FRC_ALL_HCLK0                    ((uint32_t)0x00000FFFU)       /**< All clock group 0            */
/** @} */

/** @defgroup CGC_LL_EC_FRC_CLK1 Force Clock OFF
  * @{
  */
#define LL_CGC_FRC_AON_MCUSUB_HCLK              MCU_SUB_FORCE_AON_MCUSUB_HCLK /**< Hclk for Always-on register  */
#define LL_CGC_FRC_XF_XQSPI_HCLK                MCU_SUB_FORCE_XF_XQSPI_HCLK   /**< Hclk for cache top           */
#define LL_CGC_FRC_SRAM_HCLK                    MCU_SUB_FORCE_SRAM_HCLK       /**< Hclk for SRAMs               */

#define LL_CGC_FRC_ALL_HCLK1                    ((uint32_t)0x00070000U)       /**< All clock group 1            */
/** @} */

/** @defgroup CGC_LL_EC_FRC_CLK2 Force Clock OFF
  * @{
  */
#define LL_CGC_FRC_UART0_HCLK                   MCU_SUB_FORCE_UART0_HCLK      /**< Hclk for uart0               */
#define LL_CGC_FRC_UART1_HCLK                   MCU_SUB_FORCE_UART1_HCLK      /**< Hclk for uart1               */
#define LL_CGC_FRC_I2C0_HCLK                    MCU_SUB_FORCE_I2C0_HCLK       /**< Hclk for i2c0                */
#define LL_CGC_FRC_I2C1_HCLK                    MCU_SUB_FORCE_I2C1_HCLK       /**< Hclk for i2c1                */
#define LL_CGC_FRC_SPIM_HCLK                    MCU_SUB_FORCE_SPIM_HCLK       /**< Hclk for spim                */
#define LL_CGC_FRC_SPIS_HCLK                    MCU_SUB_FORCE_SPIS_HCLK       /**< Hclk for spis                */
#define LL_CGC_FRC_QSPI0_HCLK                   MCU_SUB_FORCE_QSPI0_HCLK      /**< Hclk for qspi0               */
#define LL_CGC_FRC_QSPI1_HCLK                   MCU_SUB_FORCE_QSPI1_HCLK      /**< Hclk for qspi1               */
#define LL_CGC_FRC_I2S_HCLK                     MCU_SUB_FORCE_I2S_HCLK        /**< Hclk for i2s                 */
#define LL_CGC_FRC_SECU_DIV4_PCLK               MCU_SUB_FORCE_SECU_DIV4_PCLK  /**< Div4 clk for security blocks */
#define LL_CGC_FRC_XQSPI_DIV4_PCLK              MCU_SUB_FORCE_XQSPI_DIV4_PCLK /**< Div4 clk for xf qspi         */

#define LL_CGC_FRC_SERIALS_HCLK2                ((uint32_t)0x0001FF00U)       /**< Hclk for serial blocks       */
#define LL_CGC_FRC_ALL_HCLK2                    ((uint32_t)0x0A01FF00U)       /**< All clock group 2            */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CGC_LL_Exported_Macros CGC Exported Macros
  * @{
  */

/** @defgroup CGC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in CGC register
  * @param  __instance__ CGC instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_CGC_WriteReg(__instance__, __REG__, __VALUE__) WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in CGC register
  * @param  __instance__ CGC instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_CGC_ReadReg(__instance__, __REG__) READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup CGC_LL_Private_Macros CGC Private Macros
  * @{
  */

/** @defgroup CGC_LL_EC_DEFAULT_CONFIG InitStruct default configuartion
  * @{
  */

/**
  * @brief LL CGC InitStrcut default configuartion
  */
#define LL_CGC_DEFAULT_CONFIG                   \
{                                               \
    .wfi_clk0        = ~LL_CGC_WFI_ALL_HCLK0,   \
    .wfi_clk1        = ~LL_CGC_WFI_ALL_HCLK1,   \
    .wfi_clk2        = ~LL_CGC_WFI_ALL_HCLK2,   \
    .force_clk0      = ~LL_CGC_FRC_ALL_HCLK0,   \
    .force_clk1      = ~LL_CGC_FRC_ALL_HCLK1,   \
    .force_clk2      = ~LL_CGC_FRC_ALL_HCLK2,   \
}
/** @} */

/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @defgroup CGC_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup CGC_LL_EF_CLK_Configuration Clock Configuration
  * @{
  */

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: Security/SIM/HTB/PWM/
  *        ROM/SNSADC/GPIO/DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *  CG_CTRL_0 | SIM_HCLK
  *  CG_CTRL_0 | HTB_HCLK
  *  CG_CTRL_0 | PWM_HCLK
  *  CG_CTRL_0 | ROM_HCLK
  *  CG_CTRL_0 | SNSADC_HCLK
  *  CG_CTRL_0 | GPIO_HCLK
  *  CG_CTRL_0 | DMA_HCLK
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *  CG_CTRL_0 | APB_SUB_HCLK
  *  CG_CTRL_0 | SERIAL_HCLK
  *  CG_CTRL_0 | I2S_S_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_HCLK
  *         @arg @ref LL_CGC_WFI_SIM_HCLK
  *         @arg @ref LL_CGC_WFI_HTB_HCLK
  *         @arg @ref LL_CGC_WFI_PWM_HCLK
  *         @arg @ref LL_CGC_WFI_ROM_HCLK
  *         @arg @ref LL_CGC_WFI_SNSADC_HCLK
  *         @arg @ref LL_CGC_WFI_GPIO_HCLK
  *         @arg @ref LL_CGC_WFI_DMA_HCLK
  *         @arg @ref LL_CGC_WFI_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_WFI_APB_SUB_HCLK
  *         @arg @ref LL_CGC_WFI_SERIAL_HCLK
  *         @arg @ref LL_CGC_WFI_I2S_S_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_0(uint32_t clk_mask)
{
    WRITE_REG(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], clk_mask);
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: Security/SIM/HTB/PWM/
  *        ROM/SNSADC/GPIO/DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *  CG_CTRL_0 | SIM_HCLK
  *  CG_CTRL_0 | HTB_HCLK
  *  CG_CTRL_0 | PWM_HCLK
  *  CG_CTRL_0 | ROM_HCLK
  *  CG_CTRL_0 | SNSADC_HCLK
  *  CG_CTRL_0 | GPIO_HCLK
  *  CG_CTRL_0 | DMA_HCLK
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *  CG_CTRL_0 | APB_SUB_HCLK
  *  CG_CTRL_0 | SERIAL_HCLK
  *  CG_CTRL_0 | I2S_S_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_HCLK
  *         @arg @ref LL_CGC_WFI_SIM_HCLK
  *         @arg @ref LL_CGC_WFI_HTB_HCLK
  *         @arg @ref LL_CGC_WFI_PWM_HCLK
  *         @arg @ref LL_CGC_WFI_ROM_HCLK
  *         @arg @ref LL_CGC_WFI_SNSADC_HCLK
  *         @arg @ref LL_CGC_WFI_GPIO_HCLK
  *         @arg @ref LL_CGC_WFI_DMA_HCLK
  *         @arg @ref LL_CGC_WFI_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_WFI_APB_SUB_HCLK
  *         @arg @ref LL_CGC_WFI_SERIAL_HCLK
  *         @arg @ref LL_CGC_WFI_I2S_S_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_0(void)
{
    return READ_REG(MCU_SUB->MCU_SUBSYS_CG_CTRL[0]);
}


/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_WFI_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_WFI_SRAM_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_1(uint32_t clk_mask)
{
	GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_MSK_HCLK_1, clk_mask);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_WFI_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_WFI_SRAM_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_1(void)
{
    return READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_MSK_HCLK_1);
}

/**
  * @brief Some peripherals automatic turn off clock during WFI. (Include: SECU_DIV4/XQSPI_DIV4)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_WFI_XQSPI_DIV4_PCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_wfi_off_hclk_2(uint32_t clk_mask)
{
    GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_MSK_HCLK_2, clk_mask);
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Return to clock blocks that is turned off during WFI.(Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_WFI_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_WFI_XQSPI_DIV4_PCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_wfi_off_hclk_2(void)
{
    return READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_MSK_HCLK_2);
}

/**
  * @brief Some peripherals force turn off clock. (Include: Security/SIM/HTB/PWM/ROM/SNSADC/GPIO/
  *        DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *  CG_CTRL_1 | SIM_HCLK
  *  CG_CTRL_1 | HTB_HCLK
  *  CG_CTRL_1 | PWM_HCLK
  *  CG_CTRL_1 | ROM_HCLK
  *  CG_CTRL_1 | SNSADC_HCLK
  *  CG_CTRL_1 | GPIO_HCLK
  *  CG_CTRL_1 | DMA_HCLK
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *  CG_CTRL_1 | APB_SUB_HCLK
  *  CG_CTRL_1 | SERIAL_HCLK
  *  CG_CTRL_1 | I2S_S_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_SECU_HCLK
  *         @arg @ref LL_CGC_FRC_SIM_HCLK
  *         @arg @ref LL_CGC_FRC_HTB_HCLK
  *         @arg @ref LL_CGC_FRC_PWM_HCLK
  *         @arg @ref LL_CGC_FRC_ROM_HCLK
  *         @arg @ref LL_CGC_FRC_SNSADC_HCLK
  *         @arg @ref LL_CGC_FRC_GPIO_HCLK
  *         @arg @ref LL_CGC_FRC_DMA_HCLK
  *         @arg @ref LL_CGC_FRC_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_FRC_APB_SUB_HCLK
  *         @arg @ref LL_CGC_FRC_SERIAL_HCLK
  *         @arg @ref LL_CGC_FRC_I2S_S_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_0(uint32_t clk_mask)
{
    WRITE_REG(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], clk_mask);
}

/**
  * @brief  Return to clock blocks that was forcibly closed.(Include: Security/SIM/HTB/PWM/
  *        ROM/SNSADC/GPIO/DMA/BLE_BRG/APB_SUB/SERIAL/I2S)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *  CG_CTRL_1 | SIM_HCLK
  *  CG_CTRL_1 | HTB_HCLK
  *  CG_CTRL_1 | PWM_HCLK
  *  CG_CTRL_1 | ROM_HCLK
  *  CG_CTRL_1 | SNSADC_HCLK
  *  CG_CTRL_1 | GPIO_HCLK
  *  CG_CTRL_1 | DMA_HCLK
  *  CG_CTRL_1 | BLE_BRG_HCLK
  *  CG_CTRL_1 | APB_SUB_HCLK
  *  CG_CTRL_1 | SERIAL_HCLK
  *  CG_CTRL_1 | I2S_S_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_SECU_HCLK
  *         @arg @ref LL_CGC_FRC_SIM_HCLK
  *         @arg @ref LL_CGC_FRC_HTB_HCLK
  *         @arg @ref LL_CGC_FRC_PWM_HCLK
  *         @arg @ref LL_CGC_FRC_ROM_HCLK
  *         @arg @ref LL_CGC_FRC_SNSADC_HCLK
  *         @arg @ref LL_CGC_FRC_GPIO_HCLK
  *         @arg @ref LL_CGC_FRC_DMA_HCLK
  *         @arg @ref LL_CGC_FRC_BLE_BRG_HCLK
  *         @arg @ref LL_CGC_FRC_APB_SUB_HCLK
  *         @arg @ref LL_CGC_FRC_SERIAL_HCLK
  *         @arg @ref LL_CGC_FRC_I2S_S_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_0(void)
{
    return READ_REG(MCU_SUB->MCU_SUBSYS_CG_CTRL[1]);
}


/**
  * @brief Some peripherals force turn off clock. (Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_FRC_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_FRC_SRAM_HCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_1(uint32_t clk_mask)
{
	GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_MSK_HCLK_1, clk_mask);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Return to clock blocks that was forcibly closed.(Include: AON_MCUSUB/XF_XQSPI/SRAM)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_AON_MCUSUB_HCLK
  *         @arg @ref LL_CGC_FRC_XF_XQSPI_HCLK
  *         @arg @ref LL_CGC_FRC_SRAM_HCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_1(void)
{
    return READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_MSK_HCLK_1);
}

/**
  * @brief Some peripherals force turn off clock. (Include: UART0_HCLK/UART1_HCLK/I2C0_HCLK/
  *        I2C1_HCLK/SPIM_HCLK/SPIS_HCLK/QSPI0_HCLK/QSPI1_HCLK/I2S_HCLK/SECU_DIV4_PCLK/XQSPI_DIV4_PCLK)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK
  *  PERIPH_GC | UART1_HCLK
  *  PERIPH_GC | I2C0_HCLK
  *  PERIPH_GC | I2C1_HCLK
  *  PERIPH_GC | SPIM_HCLK
  *  PERIPH_GC | SPIS_HCLK
  *  PERIPH_GC | QSPI0_HCLK
  *  PERIPH_GC | QSPI1_HCLK
  *  PERIPH_GC | I2S_HCLK
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @param  clk_mask This parameter can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_UART0_HCLK
  *         @arg @ref LL_CGC_FRC_UART1_HCLK
  *         @arg @ref LL_CGC_FRC_I2C0_HCLK
  *         @arg @ref LL_CGC_FRC_I2C1_HCLK
  *         @arg @ref LL_CGC_FRC_SPIM_HCLK
  *         @arg @ref LL_CGC_FRC_SPIS_HCLK
  *         @arg @ref LL_CGC_FRC_QSPI0_HCLK
  *         @arg @ref LL_CGC_FRC_QSPI1_HCLK
  *         @arg @ref LL_CGC_FRC_I2S_HCLK
  *         @arg @ref LL_CGC_FRC_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_FRC_XQSPI_DIV4_PCLK
  * @retval None
  */
__STATIC_INLINE void ll_cgc_set_force_off_hclk_2(uint32_t clk_mask)
{
	GLOBAL_EXCEPTION_DISABLE();
    MODIFY_REG(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_MSK_HCLK_2, clk_mask);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Return to clock blocks that was forcibly closed.(Include: UART0_HCLK/UART1_HCLK/I2C0_HCLK/
  *        I2C1_HCLK/SPIM_HCLK/SPIS_HCLK/QSPI0_HCLK/QSPI1_HCLK/I2S_HCLK/SECU_DIV4_PCLK/XQSPI_DIV4_PCLK)
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK
  *  PERIPH_GC | UART1_HCLK
  *  PERIPH_GC | I2C0_HCLK
  *  PERIPH_GC | I2C1_HCLK
  *  PERIPH_GC | SPIM_HCLK
  *  PERIPH_GC | SPIS_HCLK
  *  PERIPH_GC | QSPI0_HCLK
  *  PERIPH_GC | QSPI1_HCLK
  *  PERIPH_GC | I2S_HCLK
  *  PERIPH_GC | SECU_DIV4_PCLK
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_CGC_FRC_UART0_HCLK
  *         @arg @ref LL_CGC_FRC_UART1_HCLK
  *         @arg @ref LL_CGC_FRC_I2C0_HCLK
  *         @arg @ref LL_CGC_FRC_I2C1_HCLK
  *         @arg @ref LL_CGC_FRC_SPIM_HCLK
  *         @arg @ref LL_CGC_FRC_SPIS_HCLK
  *         @arg @ref LL_CGC_FRC_QSPI0_HCLK
  *         @arg @ref LL_CGC_FRC_QSPI1_HCLK
  *         @arg @ref LL_CGC_FRC_I2S_HCLK
  *         @arg @ref LL_CGC_FRC_SECU_DIV4_PCLK
  *         @arg @ref LL_CGC_FRC_XQSPI_DIV4_PCLK
  */
__STATIC_INLINE uint32_t ll_cgc_get_force_off_hclk_2(void)
{
    return READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_MSK_HCLK_2);
}

/**
  * @brief  Enable security blocks(including AES, PKC, Present, HMAC) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_secu_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SECU_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable security blocks(including AES, PKC, Present, HMAC) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_secu_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SECU_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the security blocks(including AES, PKC, Present, HMAC) automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SECU_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_secu_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SECU_HCLK) == (MCU_SUB_WFI_SECU_HCLK));
}

/**
  * @brief  Enable SIM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_sim_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SIM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable SIM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_sim_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SIM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the SIM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SIM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_sim_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SIM_HCLK) == (MCU_SUB_WFI_SIM_HCLK));
}

/**
  * @brief  Enable Hopping Table automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_htb_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_HTB_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable Hopping Table automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_htb_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_HTB_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the Hopping Table automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | HTB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_htb_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_HTB_HCLK) == (MCU_SUB_WFI_HTB_HCLK));
}

/**
  * @brief  Enable PWM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | PWM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_pwm_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_PWM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable PWM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | PWM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_pwm_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_PWM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the PWM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | PWM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_pwm_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_PWM_HCLK) == (MCU_SUB_WFI_PWM_HCLK));
}

/**
  * @brief  Enable ROM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_rom_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_ROM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable ROM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_rom_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_ROM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the ROM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | ROM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_rom_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_ROM_HCLK) == (MCU_SUB_WFI_ROM_HCLK));
}

/**
  * @brief  Enable SNSADC automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SNSADC_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_snsadc_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SNSADC_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable SNSADC automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SNSADC_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_snsadc_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SNSADC_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the SNSADC automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SNSADC_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_snsadc_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SNSADC_HCLK) == (MCU_SUB_WFI_SNSADC_HCLK));
}

/**
  * @brief  Enable GPIO automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | GPIO_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_gpio_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_GPIO_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable GPIO automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | GPIO_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_gpio_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_GPIO_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the GPIO automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | GPIO_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_gpio_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_GPIO_HCLK) == (MCU_SUB_WFI_GPIO_HCLK));
}

/**
  * @brief  Enable DMA automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | DMA_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_dma_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_DMA_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable DMA automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | DMA_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_dma_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_DMA_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the DMA automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | DMA_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_dma_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_DMA_HCLK) == (MCU_SUB_WFI_DMA_HCLK));
}

/**
  * @brief  Enable BLE Bridge automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_ble_brg_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_BLE_BRG_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable BLE Bridge automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_ble_brg_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_BLE_BRG_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the BLE Bridge automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | BLE_BRG_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_ble_brg_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_BLE_BRG_HCLK) == (MCU_SUB_WFI_BLE_BRG_HCLK));
}

/**
  * @brief  Enable APB Subsystem automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | APB_SUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_apb_sub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_APB_SUB_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable APB Subsystem automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | APB_SUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_apb_sub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_APB_SUB_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the APB Subsystem automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | APB_SUB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_apb_sub_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_APB_SUB_HCLK) == (MCU_SUB_WFI_APB_SUB_HCLK));
}

/**
  * @brief  Enable serial blocks(including I2C, UART, QSPI, I2S, SPI) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_serial_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SERIAL_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable serial blocks(including I2C, UART, QSPI, I2S, SPI) automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_serial_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SERIAL_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the serial blocks(including I2C, UART, QSPI, I2S, SPI) automatic turn off 
  *         clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | SERIAL_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_serial_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_SERIAL_HCLK) == (MCU_SUB_WFI_SERIAL_HCLK));
}

/**
  * @brief  Enable I2S slave automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | I2S_S_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_i2s_s_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_I2S_S_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable I2S slave automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | I2S_S_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_i2s_s_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_I2S_S_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the I2S slave automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_0 | I2S_S_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_i2s_s_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[0], MCU_SUB_WFI_I2S_S_HCLK) == (MCU_SUB_WFI_I2S_S_HCLK));
}

/**
  * @brief  Enable AON_MUCSUB automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_aon_mcusub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_AON_MCUSUB_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable AON_MUCSUB automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_aon_mcusub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_AON_MCUSUB_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the AON_MUCSUB automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_aon_mcusub_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_AON_MCUSUB_HCLK) == (MCU_SUB_WFI_AON_MCUSUB_HCLK));
}

/**
  * @brief  Enable XQSPI automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_xqspi_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_XF_XQSPI_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable XQSPI automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_xqspi_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_XF_XQSPI_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the XQSPI automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_xqspi_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_XF_XQSPI_HCLK) == (MCU_SUB_WFI_XF_XQSPI_HCLK));
}

/**
  * @brief  Enable SRAM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_sram_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_SRAM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable SRAM automatic turn off clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_sram_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();
    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_SRAM_HCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the SRAM automatic turn off clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_sram_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_WFI_SRAM_HCLK) == (MCU_SUB_WFI_SRAM_HCLK));
}

/**
  * @brief  Enable security blocks automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_secu_div4_hclk(void)
{	
	GLOBAL_EXCEPTION_DISABLE();
    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_SECU_DIV4_PCLK);
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable security blocks automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_secu_div4_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_SECU_DIV4_PCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the security blocks automatic turn off div4
  *         clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SECU_DIV4_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_secu_div4_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_SECU_DIV4_PCLK) == (MCU_SUB_WFI_SECU_DIV4_PCLK));
}

/**
  * @brief  Enable XQSPI automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_wfi_off_xqspi_div4_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_XQSPI_DIV4_PCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disable XQSPI automatic turn off div4 clock during WFI
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_wfi_off_xqspi_div4_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_XQSPI_DIV4_PCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the XQSPI automatic turn off div4 clock is enabled.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | XQSPI_DIV4_PCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_wfi_off_xqspi_div4_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_WFI_XQSPI_DIV4_PCLK) == (MCU_SUB_WFI_XQSPI_DIV4_PCLK));
}

/**
  * @brief  Enabling force to turn off the clock for security blocks(including AES, PKC, Present, HMAC).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_secu_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SECU_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for security blocks(including AES, PKC, Present, HMAC).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_secu_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SECU_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for security blocks(including AES, PKC, Present, HMAC) is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SECU_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_secu_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SECU_HCLK) == (MCU_SUB_FORCE_SECU_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for SIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_sim_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SIM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for SIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SIM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_sim_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SIM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for SIM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SIM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_sim_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SIM_HCLK) == (MCU_SUB_FORCE_SIM_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for Hopping Table.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_htb_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_HTB_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for Hopping Table.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | HTB_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_htb_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_HTB_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for Hopping Table is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | HTB_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_htb_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_HTB_HCLK) == (MCU_SUB_FORCE_HTB_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for PWM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | PWM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_pwm_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_PWM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for PWM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | PWM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_pwm_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_PWM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for PWM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | PWM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_pwm_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_PWM_HCLK) == (MCU_SUB_FORCE_PWM_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for ROM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_rom_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_ROM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for ROM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | ROM_HCLK
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_rom_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_ROM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for ROM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | ROM_HCLK
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_rom_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_ROM_HCLK) == (MCU_SUB_FORCE_ROM_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for SNSADC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SNSADC_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_snsadc_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SNSADC_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for SNSADC.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SNSADC_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_snsadc_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SNSADC_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for SNSADC is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SNSADC_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_snsadc_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SNSADC_HCLK) == (MCU_SUB_FORCE_SNSADC_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for GPIO.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | GPIO_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_gpio_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_GPIO_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for GPIO.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | GPIO_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_gpio_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_GPIO_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for GPIO is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | GPIO_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_gpio_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_GPIO_HCLK) == (MCU_SUB_FORCE_GPIO_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for DMA.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | DMA_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_dma_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_DMA_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for DMA.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | DMA_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_dma_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_DMA_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for DMA is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | DMA_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_dma_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_DMA_HCLK) == (MCU_SUB_FORCE_DMA_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for BLE Bridge.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | BLE_BRG_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_ble_brg_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_BLE_BRG_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for BLE Bridge.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | BLE_BRG_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_ble_brg_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_BLE_BRG_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for BLE Bridge is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | BLE_BRG_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_ble_brg_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_BLE_BRG_HCLK) == (MCU_SUB_FORCE_BLE_BRG_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for APB Subsystem.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | APB_SUB_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_apb_sub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_APB_SUB_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for APB Subsystem.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | APB_SUB_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_apb_sub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_APB_SUB_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for APB Subsystem is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | APB_SUB_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_apb_sub_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_APB_SUB_HCLK) == (MCU_SUB_FORCE_APB_SUB_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for serial blocks(including I2C, UART, QSPI, I2S, SPI).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SERIAL_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_serial_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SERIAL_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for serial blocks(including I2C, UART, QSPI, I2S, SPI).
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SERIAL_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_serial_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SERIAL_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for serial blocks(including I2C, UART, QSPI, I2S, SPI) is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | SERIAL_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_serial_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_SERIAL_HCLK) == (MCU_SUB_FORCE_SERIAL_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for I2S slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | I2S_S_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2s_s_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_I2S_S_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for I2S slave.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | I2S_S_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2s_s_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_I2S_S_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for I2S slave is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_1 | I2S_S_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2s_s_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[1], MCU_SUB_FORCE_I2S_S_HCLK) == (MCU_SUB_FORCE_I2S_S_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for AON_MUCSUB.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_aon_mcusub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_AON_MCUSUB_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for AON_MUCSUB.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_aon_mcusub_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_AON_MCUSUB_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for AON_MUCSUB is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | AON_MCUSUB_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_aon_mcusub_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_AON_MCUSUB_HCLK) == (MCU_SUB_FORCE_AON_MCUSUB_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for XQSPI.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_xqspi_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_XF_XQSPI_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for XQSPI.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_xqspi_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_XF_XQSPI_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for XQSPI is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | XF_XQSPI_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_xqspi_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_XF_XQSPI_HCLK) == (MCU_SUB_FORCE_XF_XQSPI_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for SRAM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_sram_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_SRAM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for SRAM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_sram_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_SRAM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for SRAM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  CG_CTRL_2 | SRAM_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_sram_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_SUBSYS_CG_CTRL[2], MCU_SUB_FORCE_SRAM_HCLK) == (MCU_SUB_FORCE_SRAM_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for UART0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart0_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_UART0_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for UART0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart0_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_UART0_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for UART0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART0_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart0_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_UART0_HCLK) == (MCU_SUB_FORCE_UART0_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for UART1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART1_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_uart1_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_UART1_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for UART1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART1_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_uart1_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_UART1_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for UART1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | UART1_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_uart1_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_UART1_HCLK) == (MCU_SUB_FORCE_UART1_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for I2C0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C0_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c0_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2C0_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for I2C0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C0_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c0_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2C0_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for I2C0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C0_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c0_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2C0_HCLK) == (MCU_SUB_FORCE_I2C0_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for I2C1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C1_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2c1_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2C1_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for I2C1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C1_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2c1_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2C1_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for I2C1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2C1_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2c1_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2C1_HCLK) == (MCU_SUB_FORCE_I2C1_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for SPIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIM_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_spim_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SPIM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for SPIM.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIM_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_spim_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SPIM_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for SPIM is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIM_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_spim_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SPIM_HCLK) == (MCU_SUB_FORCE_SPIM_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for SPIS.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIS_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_spis_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SPIS_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for SPIS.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIS_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_spis_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SPIS_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for SPIS is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | SPIS_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_spis_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SPIS_HCLK) == (MCU_SUB_FORCE_SPIS_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for QSPI0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI0_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_qspi0_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_QSPI0_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for QSPI0.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI0_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_qspi0_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_QSPI0_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for QSPI0 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI0_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_qspi0_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_QSPI0_HCLK) == (MCU_SUB_FORCE_QSPI0_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for QSPI1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI1_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_qspi1_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_QSPI1_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for QSPI1.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI1_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_qspi1_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_QSPI1_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for QSPI1 is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | QSPI1_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_qspi1_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_QSPI1_HCLK) == (MCU_SUB_FORCE_QSPI1_HCLK));
}

/**
  * @brief  Enabling force to turn off the clock for I2S master.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_i2s_m_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2S_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the clock for I2S master.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_i2s_m_hclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2S_HCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the clock for I2S master is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_i2s_m_hclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_I2S_HCLK) == (MCU_SUB_FORCE_I2S_HCLK));
}

/**
  * @brief  Enabling force to turn off the div4 clock for security blocks.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_enable_force_off_secu_div4_pclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    SET_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SECU_DIV4_PCLK);

	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Disabling force to turn off the div4 clock for security blocks.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK 
  *
  * @retval None
  */
__STATIC_INLINE void ll_cgc_disable_force_off_secu_div4_pclk(void)
{
	GLOBAL_EXCEPTION_DISABLE();

    CLEAR_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SECU_DIV4_PCLK);
	
	GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @brief  Indicate whether the div4 clock for security blocks is forced to close.
  *
  *  Register  | BitsName
  *  ----------|--------
  *  PERIPH_GC | I2S_HCLK 
  *
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_cgc_is_enabled_force_off_secu_div4_pclk(void)
{
    return (READ_BITS(MCU_SUB->MCU_PERIPH_CG, MCU_SUB_FORCE_SECU_DIV4_PCLK) == (MCU_SUB_FORCE_SECU_DIV4_PCLK));
}


/** @} */

/** @defgroup CGC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize CGC registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: CGC registers are de-initialized
  *          - ERROR: CGC registers are not de-initialized
  */
error_status_t ll_cgc_deinit(void);

/**
  * @brief  Initialize CGC registers according to the specified.
  *         parameters in p_cgc_init.
  * @param  p_cgc_init Pointer to a ll_cgc_init_t structure that contains the configuration
  *                             information for the specified CGC register.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: CGC registers are initialized according to p_cgc_init content
  *          - ERROR: Problem occurred during CGC Registers initialization
  */
error_status_t ll_cgc_init(ll_cgc_init_t *p_cgc_init);

/**
  * @brief Set each field of a @ref ll_cgc_init_t type structure to default value.
  * @param p_cgc_init  Pointer to a @ref ll_cgc_init_t structure
  *                             whose fields will be set to default values.
  * @retval None
  */
void ll_cgc_struct_init(ll_cgc_init_t *p_cgc_init);

/** @} */

/** @} */


#endif /* CGC */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_CGC_H__ */

/** @} */

/** @} */

/** @} */
