/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_pkc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of PKC LL library.
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

/** @defgroup LL_PKC PKC
  * @brief PKC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_PKC_H__
#define __GR55XX_LL_PKC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (PKC)

/** @addtogroup PKC_LL_MACRO
  * @{
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PKC_LL_Private_Macro PKC Private Macros
  * @{
  */
#define ECC_U32_LENGTH              (8)     /**< ECC Array Length */
#define RSA_U32_LENGTH              (64)    /**< RSA Array Length */

/** @} */

/** @} */

/** @defgroup PKC_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PKC_LL_ES_INIT PKC Exported Init structures
  * @{
  */

/**
  * @brief LL PKC ECC Point Structure definition
  */
typedef struct _ll_ecc_point
{
    uint32_t X[ECC_U32_LENGTH];     /**< Specifies the point in x-axis */

    uint32_t Y[ECC_U32_LENGTH];     /**< Specifies the point in y-axis */

} ll_ecc_point_t;

/**
  * @brief LL PKC ECC P-256 Elliptic Curve Init Structure definition
  */
typedef struct _ll_ecc_curve_init
{
    uint32_t A[ECC_U32_LENGTH];             /**< Operand A array */
    uint32_t B[ECC_U32_LENGTH];             /**< Operand B array */

    uint32_t P[ECC_U32_LENGTH];             /**< Prime number P array */
    uint32_t PRSquare[ECC_U32_LENGTH];      /**< R^2 mod P, where R = 2^256 */
    uint32_t ConstP;                        /**< Montgomery multiplication constant in prime field P, ConstP = 1 */

    uint32_t N[ECC_U32_LENGTH];             /**< Prime number N array */
    uint32_t NRSquare[ECC_U32_LENGTH];      /**< R^2 mod N, where R = 2^256 */
    uint32_t ConstN;                        /**< Montgomery multiplication constant in prime field N, ConstN = 0xee00bc4f */

    uint32_t H;                             /**< H */

    ll_ecc_point_t G;                       /**< ECC Point G */

} ll_ecc_curve_init_t;

/**
  * @brief LL PKC Init Structure definition
  */
typedef struct _ll_pkc_init
{
    ll_ecc_curve_init_t *p_ecc_curve;     /**< Specifies the pointer to elliptic curve description */

    uint32_t data_bits;                 /**< Specifies the Data size: 256 ~ 2048bits        */

} ll_pkc_init_t;

/** @} */

/** @} */

/**
  * @defgroup  PKC_LL_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PKC_LL_Exported_Constants PKC Exported Constants
  * @{
  */

/** @defgroup PKC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_PKC_ReadReg function
  * @{
  */
#define LL_PKC_WORKSTAT_BUSY                                PKC_WORKSTAT_BUSY   /**< Busy flag */
/** @} */

/** @defgroup PKC_LL_EC_IT IT Defines
  * @brief    Interrupt defines which can be used with LL_PKC_ReadReg and LL_PKC_WriteReg functions
  * @{
  */
#define LL_PKC_INTEN_DONE                                   PKC_INTEN_DONE      /**< Operation Done Interrupt source              */
#define LL_PKC_INTEN_ERR                                    PKC_INTEN_ERR       /**< Operation Error Interrupt source             */
#define LL_PKC_INTEN_BAOVF                                  PKC_INTEN_BAOVF     /**< Big Integer Result Overflow Interrupt source */
/** @} */

/** @defgroup PKC_LL_EC_BITS_LENGTH Bits Length
  * @{
  */
#define LL_PKC_BITS_LENGTH_MIN                              (256U)   /**< Bits length min value  */
#define LL_PKC_BITS_LENGTH_MAX                              (2048U)  /**< Bits length max value  */
#define LL_PKC_BIGMULTI_BITS_LENGTH_MAX                     (1024U)  /**< Big number multiplication bits Length max value */
/** @} */

/** @defgroup PKC_LL_EC_OPERATION_MODE Operation Mode
  * @{
  */
#define LL_PKC_operation_mode_MULTIPLY                       (0x00000000U)                     /**< Multiplication operation mode              */
#define LL_PKC_operation_mode_INVERTION                      (1UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Inversion operation mode                   */
#define LL_PKC_operation_mode_ADD                            (2UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Addition operation mode                    */
#define LL_PKC_operation_mode_SUB                            (3UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Subtraction operation mode                 */
#define LL_PKC_operation_mode_COMPARE                        (4UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Comparison operation mode                  */
#define LL_PKC_operation_mode_LEFTSHIFT                      (5UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Left Shift operation mode                  */
#define LL_PKC_operation_mode_BIGINTEGERMULTIPLY             (6UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Big Number Multiplication operation mode   */
#define LL_PKC_operation_mode_BIGINTEGERADD                  (7UL << PKC_SW_CTRL_OPMODE_Pos)   /**< Big Number Addition operation mode         */
/** @} */

/** @defgroup PKC_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */

/**
  * @brief LL PKC ECC Curve default configuretion.
  */
#define LL_ECC_CURVE_DEFAULT_CONFIG                          LL_ECC_CURVE_SECP256R1_CONFIG

/**
  * @brief LL PKC ECC Curve SECP256R1 configuretion.
  */
#define LL_ECC_CURVE_SECP256R1_CONFIG                                                                             \
{                                                                                                                 \
    .A        = {0xFFFFFFFC, 0x00000004, 0x00000000, 0x00000000, 0x00000003, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFC}, \
    .B        = {0xDC30061D, 0x04874834, 0xE5A220AB, 0xF7212ED6, 0xACF005CD, 0x78843090, 0xD89CDF62, 0x29C4BDDF}, \
    .P        = {0xFFFFFFFF, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}, \
    .PRSquare = {0x00000004, 0xFFFFFFFD, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFB, 0xFFFFFFFF, 0x00000000, 0x00000003}, \
    .ConstP   =  1,                                                                                               \
    .N        = {0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xBCE6FAAD, 0xA7179E84, 0xF3B9CAC2, 0xFC632551}, \
    .NRSquare = {0x66E12D94, 0xF3D95620, 0x2845B239, 0x2B6BEC59, 0x4699799C, 0x49BD6FA6, 0x83244C95, 0xBE79EEA2}, \
    .ConstN   =  0xEE00BC4F,                                                                                      \
    .H        =  1,                                                                                               \
    .G.X      = {0x6B17D1F2, 0xE12C4247, 0xF8BCE6E5, 0x63A440F2, 0x77037D81, 0x2DEB33A0, 0xF4A13945, 0xD898C296}, \
    .G.Y      = {0x4FE342E2, 0xFE1A7F9B, 0x8EE7EB4A, 0x7C0F9E16, 0x2BCE3357, 0x6B315ECE, 0xCBB64068, 0x37BF51F5}, \
}

/**
  * @brief LL PKC ECC Curve SECP256K1 configuretion.
  */
#define LL_ECC_CURVE_SECP256K1_CONFIG                                                                             \
{                                                                                                                 \
    .A        = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000}, \
    .B        = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000007, 0x00001AB7}, \
    .P        = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFC2F}, \
    .PRSquare = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x000007A2, 0x000E90A1}, \
    .ConstP   =  0XD2253531,                                                                                      \
    .N        = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xBAAEDCE6, 0xAF48A03B, 0xBFD25E8C, 0xD0364141}, \
    .NRSquare = {0x9D671CD5, 0x81C69BC5, 0xE697F5E4, 0x5BCD07C6, 0x741496C2, 0x0E7CF878, 0x896CF214, 0x67D7D140}, \
    .ConstN   =  0X5588B13F,                                                                                      \
    .H        =  1,                                                                                               \
    .G.X      = {0x79BE667E, 0xF9DCBBAC, 0x55A06295, 0xCE870B07, 0x029BFCDB, 0x2DCE28D9, 0x59F2815B, 0x16F81798}, \
    .G.Y      = {0x483ADA77, 0x26A3C465, 0x5DA4FBFC, 0x0E1108A8, 0xFD17B448, 0xA6855419, 0x9C47D08F, 0xFB10D4B8}, \
}

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PKC_LL_Exported_Macros PKC Exported Macros
  * @{
  */

/** @defgroup PKC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PKC register
  * @param  __INSTANCE__ PKC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_PKC_WriteReg(__INSTANCE__, __REG__, __VALUE__)   WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PKC register
  * @param  __INSTANCE__ PKC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_PKC_ReadReg(__INSTANCE__, __REG__)               READ_REG(__INSTANCE__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PKC_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup PKC_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable pkc.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->CTRL, PKC_CTRL_EN);
}

/**
  * @brief  Disable pkc.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->CTRL, PKC_CTRL_EN);
}

/**
  * @brief  Indicate whether the pkc is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | EN                                |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CTRL, PKC_CTRL_EN) == (PKC_CTRL_EN));
}

/**
  * @brief  Enable pkc start in hardware mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | START                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable_hardware_start(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->CTRL, PKC_CTRL_START);
}

/**
  * @brief  Disable pkc start in hardware mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | START                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable_hardware_start(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->CTRL, PKC_CTRL_START);
}

/**
  * @brief  Indicate whether the pkc start in hardware mode is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | START                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled_hardware_start(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CTRL, PKC_CTRL_START) == (PKC_CTRL_START));
}

/**
  * @brief  Enable pkc software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | SWCTRL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable_software(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->CTRL, PKC_CTRL_SWCTRL);
}

/**
  * @brief  Disable pkc software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | SWCTRL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable_software(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->CTRL, PKC_CTRL_SWCTRL);
}

/**
  * @brief  Indicate whether the pkc software mode is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | SWCTRL                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled_software(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CTRL, PKC_CTRL_SWCTRL) == (PKC_CTRL_SWCTRL));
}

/**
  * @brief  Enable pkc reset.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | SWRST                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable_reset(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->CTRL, PKC_CTRL_SWRST);
}

/**
  * @brief  Disable pkc reset.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | SWRST                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable_reset(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->CTRL, PKC_CTRL_SWRST);
}

/**
  * @brief  Indicate whether the pkc reset is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CTRL                 | SWRST                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled_reset(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CTRL, PKC_CTRL_SWRST) == (PKC_CTRL_SWRST));
}

/**
  * @brief  Set PKC parameter k pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG0              | KPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_k_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG0, PKC_CONFIG0_KPTR, pointer << PKC_CONFIG0_KPTR_Pos);
}

/**
  * @brief  Get PKC parameter k pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG0              | KPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_k_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG0, PKC_CONFIG0_KPTR) >> PKC_CONFIG0_KPTR_Pos);
}

/**
  * @brief  Set PKC parameter r pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG0              | RPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG0, PKC_CONFIG0_RPTR, pointer << PKC_CONFIG0_RPTR_Pos);
}

/**
  * @brief  Get PKC parameter r pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG0              | RPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG0, PKC_CONFIG0_RPTR) >> PKC_CONFIG0_RPTR_Pos);
}

/**
  * @brief  Set PKC parameter p pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG1              | PPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_p_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG1, PKC_CONFIG1_PPTR, pointer << PKC_CONFIG1_PPTR_Pos);
}

/**
  * @brief  Get PKC parameter p pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG1              | PPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_p_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG1, PKC_CONFIG1_PPTR) >> PKC_CONFIG1_PPTR_Pos);
}

/**
  * @brief  Set PKC parameter R^2 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG1              | RSQPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_rsq_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG1, PKC_CONFIG1_RSQPTR, pointer << PKC_CONFIG1_RSQPTR_Pos);
}

/**
  * @brief  Get PKC parameter R^2 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG1              | RSQPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_rsq_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG1, PKC_CONFIG1_RSQPTR) >> PKC_CONFIG1_RSQPTR_Pos);
}

/**
  * @brief  Set PKC parameter Gx pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG2              | GXPTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_gx_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG2, PKC_CONFIG2_GXPTR, pointer << PKC_CONFIG2_GXPTR_Pos);
}

/**
  * @brief  Get PKC parameter Gx pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG2              | GXPTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_gx_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG2, PKC_CONFIG2_GXPTR) >> PKC_CONFIG2_GXPTR_Pos);
}

/**
  * @brief  Set PKC parameter Gy pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG2              | GYPTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_gy_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG2, PKC_CONFIG2_GYPTR, pointer << PKC_CONFIG2_GYPTR_Pos);
}

/**
  * @brief  Get PKC parameter Gy pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG2              | GYPTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_gy_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG2, PKC_CONFIG2_GYPTR) >> PKC_CONFIG2_GYPTR_Pos);
}

/**
  * @brief  Set PKC parameter Gz pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG3              | GZPTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_gz_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG3, PKC_CONFIG3_GZPTR, pointer << PKC_CONFIG3_GZPTR_Pos);
}

/**
  * @brief  Get PKC parameter Gz pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG3              | GZPTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_gz_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG3, PKC_CONFIG3_GZPTR) >> PKC_CONFIG3_GZPTR_Pos);
}

/**
  * @brief  Set PKC parameter R0x pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG3              | R0XPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r0x_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG3, PKC_CONFIG3_R0XPTR, pointer << PKC_CONFIG3_R0XPTR_Pos);
}

/**
  * @brief  Get PKC parameter R0x pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG3              | R0XPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r0x_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG3, PKC_CONFIG3_R0XPTR) >> PKC_CONFIG3_R0XPTR_Pos);
}

/**
  * @brief  Set PKC parameter R0y pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG4              | R0YPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r0y_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG4, PKC_CONFIG4_R0YPTR, pointer << PKC_CONFIG4_R0YPTR_Pos);
}

/**
  * @brief  Get PKC parameter R0y pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG4              | R0YPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r0y_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG4, PKC_CONFIG4_R0YPTR) >> PKC_CONFIG4_R0YPTR_Pos);
}

/**
  * @brief  Set PKC parameter R0z pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG4              | R0ZPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r0z_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG4, PKC_CONFIG4_R0ZPTR, pointer << PKC_CONFIG4_R0ZPTR_Pos);
}

/**
  * @brief  Get PKC parameter R0z pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG4              | R0ZPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r0z_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG4, PKC_CONFIG4_R0ZPTR) >> PKC_CONFIG4_R0ZPTR_Pos);
}

/**
  * @brief  Set PKC parameter R1x pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG5              | R1XPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r1x_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG5, PKC_CONFIG5_R1XPTR, pointer << PKC_CONFIG5_R1XPTR_Pos);
}

/**
  * @brief  Get PKC parameter R1x pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG5              | R1XPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r1x_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG5, PKC_CONFIG5_R1XPTR) >> PKC_CONFIG5_R1XPTR_Pos);
}

/**
  * @brief  Set PKC parameter R1y pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG5              | R1YPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r1y_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG5, PKC_CONFIG5_R1YPTR, pointer << PKC_CONFIG5_R1YPTR_Pos);
}

/**
  * @brief  Get PKC parameter R1y pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG5              | R1YPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r1y_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG5, PKC_CONFIG5_R1YPTR) >> PKC_CONFIG5_R1YPTR_Pos);
}

/**
  * @brief  Set PKC parameter R1z pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG6              | R1ZPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_r1z_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG6, PKC_CONFIG6_R1ZPTR, pointer << PKC_CONFIG6_R1ZPTR_Pos);
}

/**
  * @brief  Get PKC parameter R1z pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG6              | R1ZPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_r1z_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG6, PKC_CONFIG6_R1ZPTR) >> PKC_CONFIG6_R1ZPTR_Pos);
}

/**
  * @brief  Set PKC parameter Tmp1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG6              | TMP1PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmp1_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG6, PKC_CONFIG6_TMP1PTR, pointer << PKC_CONFIG6_TMP1PTR_Pos);
}

/**
  * @brief  Get PKC parameter Tmp1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG6              | TMP1PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmp1_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG6, PKC_CONFIG6_TMP1PTR) >> PKC_CONFIG6_TMP1PTR_Pos);
}

/**
  * @brief  Set PKC parameter Tmp2 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG7              | TMP2PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmp2_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG7, PKC_CONFIG7_TMP2PTR, pointer << PKC_CONFIG7_TMP2PTR_Pos);
}

/**
  * @brief  Get PKC parameter Tmp2 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG7              | TMP2PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmp2_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG7, PKC_CONFIG7_TMP2PTR) >> PKC_CONFIG7_TMP2PTR_Pos);
}

/**
  * @brief  Set PKC parameter Tmp3 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG7              | TMP3PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmp3_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG7, PKC_CONFIG7_TMP3PTR, pointer << PKC_CONFIG7_TMP3PTR_Pos);
}

/**
  * @brief  Get PKC parameter Tmp3 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG7              | TMP3PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmp3_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG7, PKC_CONFIG7_TMP3PTR) >> PKC_CONFIG7_TMP3PTR_Pos);
}

/**
  * @brief  Set PKC parameter Tmp4 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG8              | TMP4PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmp4_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG8, PKC_CONFIG8_TMP4PTR, pointer << PKC_CONFIG8_TMP4PTR_Pos);
}

/**
  * @brief  Get PKC parameter Tmp4 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG8              | TMP4PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmp4_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG8, PKC_CONFIG8_TMP4PTR) >> PKC_CONFIG8_TMP4PTR_Pos);
}

/**
  * @brief  Set PKC parameter Tmp5 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG8              | TMP5PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmp5_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG8, PKC_CONFIG8_TMP5PTR, pointer << PKC_CONFIG8_TMP5PTR_Pos);
}

/**
  * @brief  Get PKC parameter Tmp5 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG8              | TMP5PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmp5_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG8, PKC_CONFIG8_TMP5PTR) >> PKC_CONFIG8_TMP5PTR_Pos);
}

/**
  * @brief  Set PKC parameter Tmp6 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG9              | TMP6PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmp6_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG9, PKC_CONFIG9_TMP6PTR, pointer << PKC_CONFIG9_TMP6PTR_Pos);
}

/**
  * @brief  Get PKC parameter Tmp6 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG9              | TMP6PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmp6_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG9, PKC_CONFIG9_TMP6PTR) >> PKC_CONFIG9_TMP6PTR_Pos);
}

/**
  * @brief  Set PKC parameter Constant1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG9              | CONST1PTR                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_constant1_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG9, PKC_CONFIG9_CONST1PTR, pointer << PKC_CONFIG9_CONST1PTR_Pos);
}

/**
  * @brief  Get PKC parameter Constant1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG9              | CONST1PTR                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_constant1_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG9, PKC_CONFIG9_CONST1PTR) >> PKC_CONFIG9_CONST1PTR_Pos);
}

/**
  * @brief  Set PKC parameter X1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG10             | X1PTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_x1_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG10, PKC_CONFIG10_X1PTR, pointer << PKC_CONFIG10_X1PTR_Pos);
}

/**
  * @brief  Get PKC parameter X1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG10             | X1PTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_x1_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG10, PKC_CONFIG10_X1PTR) >> PKC_CONFIG10_X1PTR_Pos);
}

/**
  * @brief  Set PKC parameter X2 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG10             | X2PTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_x2_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG10, PKC_CONFIG10_X2PTR, pointer << PKC_CONFIG10_X2PTR_Pos);
}

/**
  * @brief  Get PKC parameter X2 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG10             | X2PTR                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_x2_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG10, PKC_CONFIG10_X2PTR) >> PKC_CONFIG10_X2PTR_Pos);
}

/**
  * @brief  Set PKC parameter MITmp pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG11             | MITMPPTR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mitmp_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG11, PKC_CONFIG11_MITMPPTR, pointer << PKC_CONFIG11_MITMPPTR_Pos);
}

/**
  * @brief  Get PKC parameter MITmp pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG11             | MITMPPTR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mitmp_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG11, PKC_CONFIG11_MITMPPTR) >> PKC_CONFIG11_MITMPPTR_Pos);
}

/**
  * @brief  Set PKC parameter TmpK pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG11             | TMPKPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_tmpk_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG11, PKC_CONFIG11_TMPKPTR, pointer << PKC_CONFIG11_TMPKPTR_Pos);
}

/**
  * @brief  Get PKC parameter TmpK pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG11             | TMPKPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_tmpk_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG11, PKC_CONFIG11_TMPKPTR) >> PKC_CONFIG11_TMPKPTR_Pos);
}

/**
  * @brief  Set ECC parameter A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG12             | APTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_ecc_a_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG12, PKC_CONFIG12_APTR, pointer << PKC_CONFIG12_APTR_Pos);
}

/**
  * @brief  Get ECC parameter A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG12             | APTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_ecc_a_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG12, PKC_CONFIG12_APTR) >> PKC_CONFIG12_APTR_Pos);
}

/**
  * @brief  Set ECC parameter B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG12             | BPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_ecc_b_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->CONFIG12, PKC_CONFIG12_BPTR, pointer << PKC_CONFIG12_BPTR_Pos);
}

/**
  * @brief  Get ECC parameter B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG12             | BPTR                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_ecc_b_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->CONFIG12, PKC_CONFIG12_BPTR) >> PKC_CONFIG12_BPTR_Pos);
}

/**
  * @brief  Set constant value for montgomery multiply in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG13             | CONSTP                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  ConstP This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_constp(pkc_regs_t *PKCx, uint32_t ConstP)
{
    WRITE_REG(PKCx->CONFIG13, ConstP);
}

/**
  * @brief  Get constant value for montgomery multiply in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | CONFIG13             | CONSTP                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_constp(pkc_regs_t *PKCx)
{
    return (READ_REG(PKCx->CONFIG13));
}

/**
  * @brief  Enable pkc start in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | OPSTART                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable_software_start(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_OPSTART);
}

/**
  * @brief  Disable pkc start in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | OPSTART                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable_software_start(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_OPSTART);
}

/**
  * @brief  Indicate whether the pkc start in software mode is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | OPSTART                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled_software_start(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_OPSTART) == (PKC_SW_CTRL_OPSTART));
}

/**
  * @brief  Set operation mode in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | OPMODE                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  operation_mode This parameter can be one of the following values:
  *         @arg @ref LL_PKC_operation_mode_MULTIPLY
  *         @arg @ref LL_PKC_operation_mode_INVERTION
  *         @arg @ref LL_PKC_operation_mode_ADD
  *         @arg @ref LL_PKC_operation_mode_SUB
  *         @arg @ref LL_PKC_operation_mode_COMPARE
  *         @arg @ref LL_PKC_operation_mode_LEFTSHIFT
  *         @arg @ref LL_PKC_operation_mode_BIGINTEGERMULTIPLY
  *         @arg @ref LL_PKC_operation_mode_BIGINTEGERADD
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_operation_mode(pkc_regs_t *PKCx, uint32_t operation_mode)
{
    MODIFY_REG(PKCx->SW_CTRL, PKC_SW_CTRL_OPMODE, operation_mode);
}

/**
  * @brief  Get operation mode in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | OPMODE                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value can be one of the following values:
  *         @arg @ref LL_PKC_operation_mode_MULTIPLY
  *         @arg @ref LL_PKC_operation_mode_INVERTION
  *         @arg @ref LL_PKC_operation_mode_ADD
  *         @arg @ref LL_PKC_operation_mode_SUB
  *         @arg @ref LL_PKC_operation_mode_COMPARE
  *         @arg @ref LL_PKC_operation_mode_LEFTSHIFT
  *         @arg @ref LL_PKC_operation_mode_BIGINTEGERMULTIPLY
  *         @arg @ref LL_PKC_operation_mode_BIGINTEGERADD
  */
__STATIC_INLINE uint32_t ll_pkc_get_operation_mode(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_OPMODE));
}

/**
  * @brief  Enable Dummy Multi in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | STARTDM                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable_dummy_multi(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_STARTDM);
}

/**
  * @brief  Disable Dummy Multi in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | STARTDM                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable_dummy_multi(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_STARTDM);
}

/**
  * @brief  Indicate whether the Dummy Multi in software mode is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | STARTDM                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled_dummy_multi(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_STARTDM) == (PKC_SW_CTRL_STARTDM));
}

/**
  * @brief  Enable Random Clock Gating in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | RANDEN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_enable_random_clock_gating(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_RANDEN);
}

/**
  * @brief  Disable Random Clock Gating in software mode.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | RANDEN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval None
  */
__STATIC_INLINE void ll_pkc_disable_random_clock_gating(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_RANDEN);
}

/**
  * @brief  Indicate whether the Random Clock Gating in software mode is enabled.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CTRL              | RANDEN                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enabled_random_clock_gating(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CTRL, PKC_SW_CTRL_RANDEN) == (PKC_SW_CTRL_RANDEN));
}

/**
  * @brief  Set modular multiplication parameter A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG0           | MMAPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mm_a_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG0, PKC_SW_CONFIG0_MMAPTR, pointer << PKC_SW_CONFIG0_MMAPTR_Pos);
}

/**
  * @brief  Get modular multiplication parameter A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG0           | MMAPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mm_a_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG0, PKC_SW_CONFIG0_MMAPTR) >> PKC_SW_CONFIG0_MMAPTR_Pos);
}

/**
  * @brief  Set modular multiplication parameter B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG0           | MMBPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mm_b_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG0, PKC_SW_CONFIG0_MMBPTR, pointer << PKC_SW_CONFIG0_MMBPTR_Pos);
}

/**
  * @brief  Get modular multiplication parameter B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG0           | MMBPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mm_b_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG0, PKC_SW_CONFIG0_MMBPTR) >> PKC_SW_CONFIG0_MMBPTR_Pos);
}

/**
  * @brief  Set modular multiplication parameter P pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG1           | MMPPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mm_p_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG1, PKC_SW_CONFIG1_MMPPTR, pointer << PKC_SW_CONFIG1_MMPPTR_Pos);
}

/**
  * @brief  Get modular multiplication parameter P pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG1           | MMPPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mm_p_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG1, PKC_SW_CONFIG1_MMPPTR) >> PKC_SW_CONFIG1_MMPPTR_Pos);
}

/**
  * @brief  Set modular multiplication parameter C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG1           | MMCPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mm_c_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG1, PKC_SW_CONFIG1_MMCPTR, pointer << PKC_SW_CONFIG1_MMCPTR_Pos);
}

/**
  * @brief  Get modular multiplication parameter C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG1           | MMCPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mm_c_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG1, PKC_SW_CONFIG1_MMCPTR) >> PKC_SW_CONFIG1_MMCPTR_Pos);
}

/**
  * @brief  Set modular add/sub parameter A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG2           | MASAPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mas_a_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG2, PKC_SW_CONFIG2_MASAPTR, pointer << PKC_SW_CONFIG2_MASAPTR_Pos);
}

/**
  * @brief  Get modular add/sub parameter A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG2           | MASAPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mas_a_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG2, PKC_SW_CONFIG2_MASAPTR) >> PKC_SW_CONFIG2_MASAPTR_Pos);
}

/**
  * @brief  Set modular add/sub parameter B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG2           | MASBPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mas_b_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG2, PKC_SW_CONFIG2_MASBPTR, pointer << PKC_SW_CONFIG2_MASBPTR_Pos);
}

/**
  * @brief  Get modular add/sub parameter B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG2           | MASBPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mas_b_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG2, PKC_SW_CONFIG2_MASBPTR) >> PKC_SW_CONFIG2_MASBPTR_Pos);
}

/**
  * @brief  Set modular add/sub parameter P pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG3           | MASPPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mas_p_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG3, PKC_SW_CONFIG3_MASPPTR, pointer << PKC_SW_CONFIG3_MASPPTR_Pos);
}

/**
  * @brief  Get modular add/sub parameter P pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG3           | MASPPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mas_p_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG3, PKC_SW_CONFIG3_MASPPTR) >> PKC_SW_CONFIG3_MASPPTR_Pos);
}

/**
  * @brief  Set modular add/sub parameter C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG3           | MASCPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mas_c_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG3, PKC_SW_CONFIG3_MASCPTR, pointer << PKC_SW_CONFIG3_MASCPTR_Pos);
}

/**
  * @brief  Get modular add/sub parameter C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG3           | MASCPTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mas_c_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG3, PKC_SW_CONFIG3_MASCPTR) >> PKC_SW_CONFIG3_MASCPTR_Pos);
}

/**
  * @brief  Set modular invertion parameter U pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG4           | MIUPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mi_u_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG4, PKC_SW_CONFIG4_MIUPTR, pointer << PKC_SW_CONFIG4_MIUPTR_Pos);
}

/**
  * @brief  Get modular invertion parameter U pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG4           | MIUPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mi_u_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG4, PKC_SW_CONFIG4_MIUPTR) >> PKC_SW_CONFIG4_MIUPTR_Pos);
}

/**
  * @brief  Set modular invertion parameter V pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG4           | MIVPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mi_v_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG4, PKC_SW_CONFIG4_MIVPTR, pointer << PKC_SW_CONFIG4_MIVPTR_Pos);
}

/**
  * @brief  Get modular invertion parameter V pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG4           | MIVPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mi_v_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG4, PKC_SW_CONFIG4_MIVPTR) >> PKC_SW_CONFIG4_MIVPTR_Pos);
}

/**
  * @brief  Set modular invertion parameter X1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG5           | MIX1PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mi_x1_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG5, PKC_SW_CONFIG5_MIX1PTR, pointer << PKC_SW_CONFIG5_MIX1PTR_Pos);
}

/**
  * @brief  Get modular invertion parameter X1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG5           | MIX1PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mi_x1_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG5, PKC_SW_CONFIG5_MIX1PTR) >> PKC_SW_CONFIG5_MIX1PTR_Pos);
}

/**
  * @brief  Set modular invertion parameter X1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG5           | MIX2PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_mi_x2_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG5, PKC_SW_CONFIG5_MIX2PTR, pointer << PKC_SW_CONFIG5_MIX2PTR_Pos);
}

/**
  * @brief  Get modular invertion parameter X1 pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG5           | MIX2PTR                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_mi_x2_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG5, PKC_SW_CONFIG5_MIX2PTR) >> PKC_SW_CONFIG5_MIX2PTR_Pos);
}

/**
  * @brief  Set modular invertion parameter Tmp pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG6           | MITMPPTR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter is the offset in pkc sram, and the value can between: 0 ~ 0x200
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_swmi_tmp_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG6, PKC_SW_CONFIG6_MITMPPTR, pointer << PKC_SW_CONFIG6_MITMPPTR_Pos);
}

/**
  * @brief  Get modular invertion parameter Tmp pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG6           | MITMPPTR                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x200
  */
__STATIC_INLINE uint32_t ll_pkc_get_swmi_tmp_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG6, PKC_SW_CONFIG6_MITMPPTR) >> PKC_SW_CONFIG6_MITMPPTR_Pos);
}

/**
  * @brief  Set operation word length-bits.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG7           | WORDLEN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  WordLength This parameter can be one of the following values: 256 ~ 2048
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_operation_word_length(pkc_regs_t *PKCx, uint32_t WordLength)
{
    MODIFY_REG(PKCx->SW_CONFIG7, PKC_SW_CONFIG7_WORDLEN, (WordLength >> 5) - 1);
}

/**
  * @brief  Get operation word length-bits.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG7           | WORDLEN                           |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 256 ~ 2048
  */
__STATIC_INLINE uint32_t ll_pkc_get_operation_word_length(pkc_regs_t *PKCx)
{
    return ((READ_BITS(PKCx->SW_CONFIG7, PKC_SW_CONFIG7_WORDLEN) + 1) << 5);
}

/**
  * @brief  Get K output in invertion operation.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG8           | MIKOUT                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FFF
  */
__STATIC_INLINE uint32_t ll_pkc_get_mik_output(pkc_regs_t *PKCx)
{
    return (READ_REG(PKCx->SW_CONFIG8) & PKC_SW_CONFIG8_MIKOUT_Msk);
}

/**
  * @brief  Set dummy multiply seed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG9           | DMRNGSEED                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  seed This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_dummy_multiply_seed(pkc_regs_t *PKCx, uint32_t seed)
{
    WRITE_REG(PKCx->SW_CONFIG9, seed);
}

/**
  * @brief  Get dummy multiply seed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG9           | DMRNGSEED                         |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pkc_get_dummy_multiply_seed(pkc_regs_t *PKCx)
{
    return (READ_REG(PKCx->SW_CONFIG9));
}

/**
  * @brief  Set big integer operand A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG10          | BMAPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter can be one of the following values: 0 ~ 0x1FF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_bm_a_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG10, PKC_SW_CONFIG10_BMAPTR, pointer << PKC_SW_CONFIG10_BMAPTR_Pos);
}

/**
  * @brief  Get big integer operand A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG10          | BMAPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FF
  */
__STATIC_INLINE uint32_t ll_pkc_get_bm_a_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG10, PKC_SW_CONFIG10_BMAPTR) >> PKC_SW_CONFIG10_BMAPTR_Pos);
}

/**
  * @brief  Set big integer operand B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG10          | BMBPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter can be one of the following values: 0 ~ 0x1FF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_bm_b_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG10, PKC_SW_CONFIG10_BMBPTR, pointer << PKC_SW_CONFIG10_BMBPTR_Pos);
}

/**
  * @brief  Get big integer operand B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG10          | BMBPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FF
  */
__STATIC_INLINE uint32_t ll_pkc_get_bm_b_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG10, PKC_SW_CONFIG10_BMBPTR) >> PKC_SW_CONFIG10_BMBPTR_Pos);
}

/**
  * @brief  Set big integer result C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG11          | BMCPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter can be one of the following values: 0 ~ 0x1FF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_bm_c_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG11, PKC_SW_CONFIG11_BMCPTR, pointer << PKC_SW_CONFIG11_BMCPTR_Pos);
}

/**
  * @brief  Get big integer result C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG11          | BMCPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FF
  */
__STATIC_INLINE uint32_t ll_pkc_get_bm_c_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG11, PKC_SW_CONFIG11_BMCPTR) >> PKC_SW_CONFIG11_BMCPTR_Pos);
}

/**
  * @brief  Set big integer operand A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG11          | BAAPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter can be one of the following values: 0 ~ 0x1FF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_ba_a_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG11, PKC_SW_CONFIG11_BAAPTR, pointer << PKC_SW_CONFIG11_BAAPTR_Pos);
}

/**
  * @brief  Get big integer operand A pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG11          | BAAPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FF
  */
__STATIC_INLINE uint32_t ll_pkc_get_ba_a_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG11, PKC_SW_CONFIG11_BAAPTR) >> PKC_SW_CONFIG11_BAAPTR_Pos);
}

/**
  * @brief  Set big integer operand B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG12          | BABPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter can be one of the following values: 0 ~ 0x1FF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_ba_b_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG12, PKC_SW_CONFIG12_BABPTR, pointer << PKC_SW_CONFIG12_BABPTR_Pos);
}

/**
  * @brief  Get big integer operand B pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG12          | BABPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FF
  */
__STATIC_INLINE uint32_t ll_pkc_get_ba_b_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG12, PKC_SW_CONFIG12_BABPTR) >> PKC_SW_CONFIG12_BABPTR_Pos);
}

/**
  * @brief  Set big integer result C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG12          | BACPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  pointer This parameter can be one of the following values: 0 ~ 0x1FF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_ba_c_pointer(pkc_regs_t *PKCx, uint32_t pointer)
{
    MODIFY_REG(PKCx->SW_CONFIG12, PKC_SW_CONFIG12_BACPTR, pointer << PKC_SW_CONFIG12_BACPTR_Pos);
}

/**
  * @brief  Get big integer result C pointer in pkc sram.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG12          | BACPTR                            |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0x1FF
  */
__STATIC_INLINE uint32_t ll_pkc_get_ba_c_pointer(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->SW_CONFIG12, PKC_SW_CONFIG12_BACPTR) >> PKC_SW_CONFIG12_BACPTR_Pos);
}

/**
  * @brief  Set random clock gating seed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG13          | RANDSEED                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @param  seed This parameter can be one of the following values: 0 ~ 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_pkc_set_random_clock_gating_seed(pkc_regs_t *PKCx, uint32_t seed)
{
    WRITE_REG(PKCx->SW_CONFIG13, seed);
}

/**
  * @brief  Get random clock gating seed.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | SW_CONFIG13          | RANDSEED                          |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval Return value is between: 0 ~ 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t ll_pkc_get_random_clock_gating_seed(pkc_regs_t *PKCx)
{
    return (READ_REG(PKCx->SW_CONFIG13));
}

/** @} */

/** @defgroup PKC_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable the operation done interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_enable_it_done(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->INTEN, PKC_INTEN_DONE);
}

/**
  * @brief  Enable the operation error interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_enable_it_err(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->INTEN, PKC_INTEN_ERR);
}

/**
  * @brief  Enable the big integer overflow interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | BAOVF                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_enable_it_big_add_overflow(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->INTEN, PKC_INTEN_BAOVF);
}

/**
  * @brief  Disable the operation done interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_disable_it_done(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->INTEN, PKC_INTEN_DONE);
}

/**
  * @brief  Disable the operation error interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_disable_it_err(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->INTEN, PKC_INTEN_ERR);
}

/**
  * @brief  Disable the big integer overflow interrupt.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | BAOVF                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_disable_it_big_add_overflow(pkc_regs_t *PKCx)
{
    CLEAR_BITS(PKCx->INTEN, PKC_INTEN_BAOVF);
}

/**
  * @brief  Indicate whether the operation done interrupt is enable.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enable_it_done(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->INTEN, PKC_INTEN_DONE) == PKC_INTEN_DONE);
}

/**
  * @brief  Indicate whether the operation error interrupt is enable.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enable_it_err(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->INTEN, PKC_INTEN_ERR) == PKC_INTEN_ERR);
}

/**
  * @brief  Indicate whether the big integer overflow interrupt is enable.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTEN                | BAOVF                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_enable_it_big_add_overflow(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->INTEN, PKC_INTEN_BAOVF) == PKC_INTEN_BAOVF);
}

/** @} */

/** @defgroup PKC_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Indicate whether the operation done interrupt is pending.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_action_flag_it_done(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->INTSTAT, PKC_INTSTAT_DONE) == PKC_INTSTAT_DONE);
}

/**
  * @brief  Indicate whether the operation error interrupt is pending.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_action_flag_it_err(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->INTSTAT, PKC_INTSTAT_ERR) == PKC_INTSTAT_ERR);
}

/**
  * @brief  Indicate whether the big integer overflow interrupt is pending.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | BAOVF                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_action_flag_it_big_add_overflow(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->INTSTAT, PKC_INTSTAT_BAOVF) == PKC_INTSTAT_BAOVF);
}

/**
  * @brief  Clear the operation done interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | DONE                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_clear_flag_it_done(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->INTSTAT, PKC_INTSTAT_DONE);
}

/**
  * @brief  Clear the operation error interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | ERR                               |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_clear_flag_it_err(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->INTSTAT, PKC_INTSTAT_ERR);
}

/**
  * @brief  Clear the big integer overflow interrupt flag.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | INTSTAT              | BAOVF                             |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval none.
  */
__STATIC_INLINE void ll_pkc_clear_flag_it_big_add_overflow(pkc_regs_t *PKCx)
{
    SET_BITS(PKCx->INTSTAT, PKC_INTSTAT_BAOVF);
}

/**
  * @brief  Indicate whether the busy flag is set.
  *
  *  \rst
  *  +----------------------+-----------------------------------+
  *  | Register             | BitsName                          |
  *  +======================+===================================+
  *  | WORKSTAT             | BUSY                              |
  *  +----------------------+-----------------------------------+
  * \endrst
  *
  * @param  PKCx PKC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_pkc_is_action_flag_busy(pkc_regs_t *PKCx)
{
    return (READ_BITS(PKCx->WORKSTAT, PKC_WORKSTAT_BUSY) == PKC_WORKSTAT_BUSY);
}

/** @} */

/** @defgroup PKC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize PKC registers (Registers restored to their default values).
  * @param  PKCx PKC Instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PKC registers are de-initialized
  *          - ERROR: PKC registers are not de-initialized
  */
error_status_t ll_pkc_deinit(pkc_regs_t *PKCx);

/**
  * @brief  Initialize PKC registers according to the specified
  *         parameters in p_pkc_init.
  * @param  PKCx PKC Instance
  * @param  p_pkc_init pointer to a ll_pkc_init_t structure that contains the configuration
  *                         information for the specified PKC peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: PKC registers are initialized according to p_pkc_init content
  *          - ERROR: Problem occurred during PKC Registers initialization
  */
error_status_t ll_pkc_init(pkc_regs_t *PKCx, ll_pkc_init_t *p_pkc_init);

/**
  * @brief Set each field of a @ref ll_pkc_init_t type structure to default value.
  * @param p_pkc_init  pointer to a @ref ll_pkc_init_t structure
  *                         whose fields will be set to default values.
  * @retval None
  */
void ll_pkc_struct_init(ll_pkc_init_t *p_pkc_init);

/** @} */

/** @} */

#endif /* PKC */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_PKC_H__ */

/** @} */

/** @} */

/** @} */
