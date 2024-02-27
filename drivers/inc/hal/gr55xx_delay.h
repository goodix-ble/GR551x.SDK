/**
 ****************************************************************************************
 *
 * @file gr55xx_delay.h
 * @author  BLE Driver Team
 * @brief PERIPHERAL API DELAY DRIVER
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

#ifndef __GR55xx_DELAY_H__
#define __GR55xx_DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gr55xx.h"

/** @addtogroup PERIPHERAL Peripheral Driver
 * @{
 */

/** @addtogroup HAL_DRIVER HAL Driver
 * @{
 */

/** @defgroup HAL_DELAY DELAY
  * @brief Delay HAL module driver.
  * @{
  */
/** @addtogroup HAL_DELAY_DEFINES Defines
  * @{
  */
#define DELAY_US_DWT /**< dealy us dwt define */

#if defined ( __CC_ARM )

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE                   static __forceinline     /**< Static inline define */
#endif

#elif defined ( __GNUC__ )

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline /**< Static inline define */
#endif

#else

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE                   __STATIC_INLINE          /**< Static inline define */
#endif

#endif
/** @} */

#ifndef DELAY_US_DWT
/** @addtogroup HAL_DELAY_TYPEDEFS Typedefs
  * @{
  */
/**
  * @brief Pointer to a function for delaying execution
  */
typedef void (* delay_func_t)(uint32_t);
/** @} */
#endif

#ifdef DELAY_US_DWT
/** @addtogroup HAL_DELAY_FUNCTIONS Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Enable the DWT.
 * @param _demcr_initial:    demcr initial. 
 * @param _dwt_ctrl_initial: dwt ctrl initial.
 ****************************************************************************************
 */
void hal_dwt_enable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial);

/**
 ****************************************************************************************
 * @brief Disable the DWT.
 * @param _demcr_initial:    demcr initial. 
 * @param _dwt_ctrl_initial: dwt ctrl initial.
 ****************************************************************************************
 */
void hal_dwt_disable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial);
/** @} */

/** @addtogroup HAL_DELAY_DEFINES Defines
 * @{
 */
/**
 * @brief  Timeout module init. This macro must be used in
 *         conjunction with the @ref HAL_TIMEOUT_DEINIT macro
 */
#define HAL_TIMEOUT_INIT()                                               \
    uint32_t _demcr_initial = CoreDebug->DEMCR;                          \
    uint32_t _dwt_ctrl_initial = DWT->CTRL;                              \
do {                                                                     \
    hal_dwt_enable(_demcr_initial, _dwt_ctrl_initial);                   \
} while (0)

/**
  * @brief  Timeout module deinit. This macro must be used in
  *         conjunction with the @ref HAL_TIMEOUT_INIT macro
  */
#define HAL_TIMEOUT_DEINIT()                                             \
do {                                                                     \
    hal_dwt_disable(_demcr_initial, _dwt_ctrl_initial);                  \
} while(0)
/** @} */

/** @addtogroup HAL_DELAY_FUNCTIONS Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Function for delaying execution for number of us.
 * @note GR55xxx is based on ARM Cortex-M4, and this fuction is based on Data Watchpoint and Trace (DWT) unit so delay is precise.
 * @param number_of_us: The maximum delay time is about 67 seconds in 64M system clock. 
 *                      The faster the system clock, the shorter the maximum delay time.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void delay_us(uint32_t number_of_us)
{
    const uint8_t clocks[] = {64, 48, 16, 24, 16, 32};
    uint32_t cycles = number_of_us * (clocks[AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL]);

    if (number_of_us == 0)
    {
        return;
    }

    HAL_TIMEOUT_INIT();

    // Get start value of the cycle counter.
    uint32_t cyccnt_initial = DWT->CYCCNT;

    // Wait time end
    while ((DWT->CYCCNT - cyccnt_initial) < cycles)
    {}

    HAL_TIMEOUT_DEINIT();
}
/** @} */
#endif

#ifndef DELAY_US_DWT

#if defined ( __CC_ARM )

#pragma push
#pragma O2
/** @addtogroup HAL_DELAY_FUNCTIONS Functions
 * @{
 */
 /**
 ****************************************************************************************
 * @brief Function for delaying execution for number of us.
 * @note GR55xxx is based on ARM Cortex-M4, and this fuction is based on Data Watchpoint and Trace (DWT) unit so delay is precise.
 * @param number_of_us: The maximum delay time is about 67 seconds in 64M system clock. 
 *                      The faster the system clock, the shorter the maximum delay time.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void delay_us(uint32_t number_of_us)
{
    uint32_t pc = (unsigned int)__current_pc();
    uint8_t clocks[] = {64, 48, 16, 24, 16, 32};

    if (number_of_us == 0)
    {
        return;
    }

    static const uint16_t delay_ramcode[] = {
        0x3809, // SUBS r0, #9
        0xd8fd, // BHI .-2
        0x4770  // BX LR
        };
    // Set LSB to 1 to execute code in Thumb mode.
    const delay_func_t delay_ram_cycles = (delay_func_t)((((uint32_t)delay_ramcode) | 1));

    static const uint16_t delay_flashcode[] = {
        0x3803, // SUBS r0, #3
        0xd8fd, // BHI .-2
        0x4770  // BX LR
        };
    // Set LSB to 1 to execute code in Thumb mode.
    const delay_func_t delay_flash_cycles = (delay_func_t)((((uint32_t)delay_flashcode) | 1));
        
    static const uint16_t delay_aliascode[] = {
        0x3803, // SUBS r0, #3
        0xd8fd, // BHI .-2
        0x4770  // BX LR
        };
    // Set LSB to 1 to execute code in Thumb mode.
    const delay_func_t delay_alias_cycles = (delay_func_t)((((uint32_t)delay_aliascode) | 1));

    uint32_t cycles = number_of_us * (clocks[AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL]);

    if(pc & GR55XX_RAM_ADDRESS)
        delay_ram_cycles(cycles);
    else if(pc & GR55XX_FLASH_ADDRESS)
        delay_flash_cycles(cycles);
    else if(pc & GR55XX_ALIAS_ADDRESS)
        delay_alias_cycles(cycles);
    else
    {
        cycles = cycles / 4;
        __asm
        {
        loop:
            NOP
            SUBS cycles, #1
            BNE loop
        }
    }
}
#pragma pop

#elif defined ( _WIN32 ) || defined ( __unix ) || defined ( __APPLE__ )

#ifndef CUSTOM_DELAY_US
/**
 ****************************************************************************************
 * @brief Function for delaying execution for number of us.
 * @param number_of_us: The maximum delay time is about 67 seconds in 64M system clock. 
 *                      The faster the system clock, the shorter the maximum delay time.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void delay_us(uint32_t number_of_us)
{
}
#endif

#elif defined ( __GNUC__ ) || ( __ICCARM__ )
/**
 ****************************************************************************************
 * @brief Function for delaying execution for number of us.
 * @param number_of_us: The maximum delay time is about 67 seconds in 64M system clock. 
 *                      The faster the system clock, the shorter the maximum delay time.
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void delay_us(uint32_t number_of_us)
{
    uint8_t clocks[] = {64, 48, 16, 24, 16, 32};

    if (number_of_us)
    {
        uint32_t cycles;
        cycles = number_of_us*(clocks[AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL])/6;
        __asm__ volatile ("1:\n"
                          "NOP\n"
                          "NOP\n"
                          "NOP\n"
                          "SUBS %[cycles], %[cycles], #1\n"
                          "BNE.N 1b\n"
                           : [cycles] "=r" (cycles)
                           : "[cycles]" "r"  (cycles)
                          );
    }
}
/** @} */
#endif

#endif

/** @addtogroup HAL_DELAY_FUNCTIONS Functions
 * @{
 */
/**
 * @brief Function for delaying execution for number of milliseconds.
 *
 * @note GR55xx is based on ARM Cortex-M4, and this fuction is based on Data Watchpoint and Trace (DWT) unit so delay is precise.
 *
 * @note Function internally calls @ref delay_us so the maximum delay is the
 * same as in case of @ref delay_us.
 *
 * @param number_of_ms: The maximum delay time is about 67 seconds in 64M system clock.
 *                      The faster the system clock, the shorter the maximum delay time.
 *
 */
__STATIC_FORCEINLINE void delay_ms(uint32_t number_of_ms)
{
    delay_us(1000 * number_of_ms);
    return;
}
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_DELAY_H__ */
/** @} */
/** @} */
/** @} */

