/**************************************************************************//**
 * @file     gr55xx.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device GR55xx
 * @version  V1.00
 * @date     12. June 2018
 ******************************************************************************/
/*
 * Copyright (c) 2016-2018, Shenzhen Huiding Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup GR55xx
  * @{
  */

#ifndef __GR55xx_H__
#define __GR55xx_H__
#ifndef CFG_LAYER_TAG_SDK
#ifndef CFG_LAYER_TAG_ROM
#include "custom_config.h"
#endif
#endif

#if defined (__GNUC__)
#define __ramfunc __attribute__((noinline)) \
                  __attribute__((long_call, section(".ramfunc")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief GR55 Family
  */
#if !defined (GR55)
#define GR55
#endif /* GR55 */

/* Uncomment the line below according to the target GR55 device used in your
   application
  */
#if !defined (GR551xx)
    #define GR551xx
#endif

/** @} */

/** @addtogroup Device_Included
  * @{
  */
#if defined(GR551xx)
    #include "gr551xx.h"

    #define GR55XX_RAM_ADDRESS             0x30000000
    #define GR55XX_FLASH_ADDRESS           0x01000000
    #define GR55XX_ALIAS_ADDRESS           0x00800000
#else
    #error "Please select first the target GR55xx device used in your application (in gr551xx.h file)"
#endif

/** @} */

/** @addtogroup Exported_types
  * @{
  */

typedef enum
{
    RESET = 0,
    SET = !RESET
} flag_status_t, it_status_t;

typedef enum
{
    DISABLE = 0,
    ENABLE = !DISABLE
} functional_state_t;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
    ERROR = 0,
    SUCCESS = !ERROR
} error_status_t;

/** @} */

/** @addtogroup Exported_macros
  * @{
  */
#ifndef SET_BITS
#define SET_BITS(REG, BIT)     ((REG) |= (BIT))
#endif

#ifndef CLEAR_BITS
#define CLEAR_BITS(REG, BIT)   ((REG) &= ~(BIT))
#endif

#ifndef READ_BITS
#define READ_BITS(REG, BIT)    ((REG) & (BIT))
#endif

#ifndef CLEAR_REG
#define CLEAR_REG(REG)        ((REG) = (0x0))
#endif

#ifndef WRITE_REG
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#endif

#ifndef READ_REG
#define READ_REG(REG)         ((REG))
#endif

#ifndef MODIFY_REG
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#endif

#ifndef POSITION_VAL
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))
#endif

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#ifndef SECTION_RAM_CODE
#if defined(CFG_LAYER_TAG_ROM)
    #define SECTION_RAM_CODE
#else
#if defined ( __ICCARM__ ) || defined (__GNUC__)
    #define SECTION_RAM_CODE __ramfunc
#else
    #define SECTION_RAM_CODE __attribute__((section("RAM_CODE")))   /**< To prevent doxygen from misidentifying the function name */
#endif
#endif
#endif

#ifndef C_CONSTRUCTOR
#if defined ( __ICCARM__ )
#define C_CONSTRUCTOR
#else
#define C_CONSTRUCTOR __attribute__((constructor))
#endif
#endif

#ifndef TINY_RAM_SECTION
    #define TINY_RAM_SECTION SECTION_RAM_CODE
#endif

#ifndef RESERVE_RAM_SECTION
#if defined ( __CC_ARM )
#define RESERVE_RAM_SECTION __attribute__((section("RAM_RESERVE"),zero_init))   /**< To prevent doxygen from misidentifying the function name */
#endif
#endif

/** @brief Disable interrupts globally in the system(apart from the NMI).
 *  This macro must be used in conjunction with the @ref GLOBAL_EXCEPTION_ENABLE macro
 *  since this last one will close the brace that the current macro opens.  This means
 *  that both macros must be located at the same scope level.
 */
#define GLOBAL_EXCEPTION_DISABLE()                         \
do {                                                       \
    uint32_t __l_irq_rest = __get_PRIMASK();               \
    __set_PRIMASK(1)


/** @brief Restore interrupts from the previous global disable(apart from the NMI).
 *  @sa GLOBAL_EXCEPTION_ENABLE
 */
#define GLOBAL_EXCEPTION_ENABLE()                          \
    __set_PRIMASK(__l_irq_rest);                           \
} while(0)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_H__ */

/** @} */

/** @} */
