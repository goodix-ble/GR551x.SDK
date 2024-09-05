/**
 ******************************************************************************
 *
 * @file platform_sdk.h
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
 *****************************************************************************************
 */


/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
  @addtogroup Plat_SDK Platform SDK
  @{
  @brief Definitions and prototypes for the Platform SDK
 */

#ifndef _PLATFORM_SDK_H
#define _PLATFORM_SDK_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "system_gr55xx.h"
#include "gr55xx_hal_def.h"

/**@addtogroup PlAT_SDK_ENUM Enumerations
 * @{ */

/**@brief system clock and run mode. */
typedef enum
{
   XIP_64M = 0,            /**< XIP 64M. */
   XIP_48M,                /**< XIP 48M. */
   XIP_XO16M,              /**< XIP XO 16M. */
   XIP_24M,                /**< XIP 24M. */
   XIP_16M,                /**< XIP 16M. */
   XIP_32M,                /**< XIP 32M. */
   MIRROR_64M,             /**< MIRROR 64M. */
   MIRROR_48M,             /**< MIRROR 48M. */
   MIRROR_XO16M,           /**< MIRROR X) 16M. */
   MIRROR_24M,             /**< MIRROR 24M. */
   MIRROR_16M,             /**< MIRROR 16M. */
   MIRROR_32M,             /**< MIRROR 32M. */
} run_mode_t;

/**@brief sdk clock type. */
typedef enum
{
    RNG_OSC_CLK = 0,       /**< RNG OSC CLOCK. */
    RTC_OSC_CLK,           /**< RTC OSC CLOCK. */
    RNG_OSC_CLK2,          /**< RNG OSC CLOCK2. */
} sdk_clock_type_t;


/**@brief memory power setting mode. */
typedef enum
{
   MEM_POWER_FULL_MODE = 0,   /**< Full mode. */
   MEM_POWER_AUTO_MODE,       /**< Auto mode. */
} mem_power_t;
 /** @} */

/** @addtogroup PLAT_SDK_FUNCTIONS Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief   platform sdk init function.
 ****************************************************************************************
 */
void platform_sdk_init(void);

/**
 ****************************************************************************************
 * @brief  Set the memory power management mode, which can be automatic mode or full power on mode.
 * @param[in] mem_pwr_mode : MEM_POWER_FULL_MODE or MEM_POWER_AUTO_MODE.
 * @retval : void
 ****************************************************************************************
 */
void mem_pwr_mgmt_mode_set(mem_power_t mem_pwr_mode);

/**
 ****************************************************************************************
 * @brief  Control the memory power supply by specifying start address and length.
 * @param[in] start_addr : the start address of memory that user want to config
 * @param[in] size       : the size of memory that user want to config
 ****************************************************************************************
 */
void mem_pwr_mgmt_mode_set_from(uint32_t start_addr, uint32_t size);

/**
 ****************************************************************************************
 * @brief  update the counter A and counter B.
 * @param[in] cnt_a :  DCDC Stable Time.
 * @param[in] cnt_b :  Oscillator Stable Time.
 ****************************************************************************************
 */
void system_lp_counter_set(uint8_t cnt_a, uint8_t cnt_b);

/**
 ****************************************************************************************
 * @brief  Set Time to wakeup oscillator before BLE Activity.
 * @param[in] run_mode :  run mode.
 * @param[in] osc_us   :  Time Reserved for wakeup oscillator(unit: us).
 ****************************************************************************************
 */
void ble_wakeup_osc_time_set(run_mode_t run_mode, uint16_t osc_us);

/**
 ****************************************************************************************
 * @brief  Get Time to wakeup oscillator before BLE Activity.
 * @param[in] run_mode :  run mode.
 * @return Time Reserved for wakeup oscillator(unit: us).
 ****************************************************************************************
 */
uint16_t ble_wakeup_osc_time_get(run_mode_t run_mode);

/**
 ****************************************************************************************
 * @brief  Get NVDS Start Address.
 *
 * @return The NVDS Start Address.
 ****************************************************************************************
 */
uint32_t nvds_get_start_addr(void);

/**
 ****************************************************************************************
 * @brief  Set BLE Program Delay.
 * @param[in] run_mode :  run mode.
 * @param[in] hslot    :  program delay in half slot(unit: 312.5us).
 ****************************************************************************************
 */
void ble_program_delay_set(run_mode_t run_mode, uint8_t hslot);

/**
 ****************************************************************************************
 * @brief  Set BLE Sleep Algorithm Duration.
 * @param[in] run_mode :  run mode.
 * @param[in] dur_hus  :  BLE Sleep Algorithm Duration(unit: 0.5us).
 ****************************************************************************************
 */
void ble_sleep_algo_dur_set(run_mode_t run_mode, uint16_t dur_hus);

/**
 ****************************************************************************************
 * @brief  Set Maximum BLE Pushing Frame Time.
 * @param[in] run_mode :  run mode.
 * @param[in] max_push_hus :  maximum pushing frame time(unit: hus).
 ****************************************************************************************
 */
void ble_max_push_frame_time_set(run_mode_t run_mode, uint16_t max_push_hus);

/**
 ****************************************************************************************
 * @brief  Set Minimum System Sleep Time.
 * @param[in] run_mode :  run mode.
 * @param[in] min_sleep_us :  Minimum Time Allowed For Sleep(unit: us).
 ****************************************************************************************
 */
void sys_min_sleep_threshold_set(run_mode_t run_mode, uint32_t min_sleep_us);

/**
 ****************************************************************************************
 * @brief  Platform low power clock init function.
 * @param[in] sys_clock:  System clock.
 * @param[in] clock     :  External RTC setting.
 * @param[in] accuracy  :  Low speed clock accuracy.
 * @param[in] xo_offset :  Clock calibration parameter.
 ****************************************************************************************
 */
void platform_clock_init(mcu_clock_type_t sys_clock, sdk_clock_type_t clock, uint16_t accuracy, uint16_t xo_offset);

/**
 ****************************************************************************************
 * @brief  Platform low power clock init function.
 * @param[in] sys_clock:  System clock.
 * @param[in] clock     :  Internal RNG/RNG2 setting.
 * @param[in] accuracy  :  Low speed clock accuracy.
 * @param[in] xo_offset :  Clock calibration parameter.
 ****************************************************************************************
 */
void platform_clock_init_rng(mcu_clock_type_t sys_clock, sdk_clock_type_t clock, uint16_t accuracy, uint16_t xo_offset);

/**
 ****************************************************************************************
 * @brief  Set RTC crystal oscillator stabilization time.
 * @param[in] wait : Delay time after RTC crystal oscillator starts.(Unit ms)
 ****************************************************************************************
 */
void platform_set_rtc_crystal_delay(uint16_t wait);

/**
 ****************************************************************************************
 * @brief  Start RNG2 OSC calibration.
 ****************************************************************************************
 */
void platform_rng2_calibration_start(void);

/**
 ****************************************************************************************
 * @brief  Stop RNG2 OSC calibration.
 * @param[in] wait : True will wait calibration register is cleared
 ****************************************************************************************
 */
void platform_rng2_calibration_stop(bool wait);

/**
 ****************************************************************************************
 * @brief  Check whether RNG2 OSC calibration is on going.
 *
 * @retval true     calibration is on going
 * @retval false    calibration is not started or done
 ****************************************************************************************
 */
bool platform_rng2_calibration_is_busy(void);

/**
 ****************************************************************************************
 * @brief  Get RNG2 OSC calibration result.
 * @param[in] wait_result : true will wait calibration done to get RNG2 OSC frequency
 * @param[in] allow_cached : true will using previous RNG2 OSC calibration frequency
 * @return RNG2 OSC frequency in HZ
 ****************************************************************************************
 */
uint32_t platform_rng2_calibration_get(bool wait_result, bool allow_cached);

/**
 ****************************************************************************************
 * @brief  Platform init function.
 ****************************************************************************************
 */
void platform_init(void);

/**
 ****************************************************************************************
 * @brief  the first warm boot stage.
 ****************************************************************************************
 */
void warm_boot_first(void);

 /**
 ****************************************************************************************
 * @brief  the second warm boot stage.
 ****************************************************************************************
 */
void warm_boot_second(void);

/**
 ****************************************************************************************
 * @brief  PMU init function.
 * @param[in] clock_type :  clock type to be configured.
 ****************************************************************************************
 */
void system_pmu_init(mcu_clock_type_t clock_type);

/**
 ****************************************************************************************
 * @brief  PMU deinit function.
 ****************************************************************************************
 */
void system_pmu_deinit(void);

/**
 ****************************************************************************************
 * @brief  Warm boot process.
 ****************************************************************************************
 */
void warm_boot(void);

/**
 ****************************************************************************************
 * @brief  Set delay time between flash wakeup and read chip id in warm boot.
 *         Please referrent the time of Flash Deep Power- down to Stand-by mode.
 * @param[in] delay_us: uinit :us
 ****************************************************************************************
 */
void warm_boot_set_exflash_readid_delay(uint32_t delay_us);

/**
 ****************************************************************************************
 * @brief  Get delay time between flash wakeup and read chip id in warm boot.
 * @return Delay time (unit :us)
 ****************************************************************************************
 */
uint32_t warm_boot_get_exflash_readid_delay(void);

/**
 ****************************************************************************************
 * @brief  PMU calibration handler.
 * @param[in] p_arg : no args.
 ****************************************************************************************
 */
void pmu_calibration_handler(void* p_arg);

/**
 ****************************************************************************************
 * @brief  LFRC32K calibration.
 ****************************************************************************************
 */
void lfrc32k_calibration(void);

/**
 ****************************************************************************************
 * @brief  write flash QE
 * @return HAL status ::hal_status_t
 ****************************************************************************************
 */
hal_status_t platform_flash_enable_quad(void);

/**
 ****************************************************************************************
 * @brief  Power Management warm boot.
 ****************************************************************************************
 */
void pwr_mgmt_warm_boot(void);

/** @} */

#endif

/** @} */
/** @} */

