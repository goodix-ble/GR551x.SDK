/**
 *******************************************************************************
 *
 * @file gr55xx_sys.h
 *
 * @brief GR55XX System API
 *
 *******************************************************************************
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
 @addtogroup SYSTEM
 @{
 */

/**
 * @addtogroup SYS System SDK
 * @{
 * @brief Definitions and prototypes for the system SDK interface.
*/



#ifndef __GR55XX_SYS_H__
#define __GR55XX_SYS_H__

#include "gr55xx_sys_cfg.h"
#include "gr55xx_nvds.h"
#include "gr55xx_dfu.h"
#include "gr55xx_pwr.h"
#include "gr55xx_fpb.h"
#include "ble.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>

/** @addtogroup GR55XX_SYS_DEFINES Defines
 * @{
 */
#define SYS_INVALID_TIMER_ID              0xFF                  /**< Invalid system Timer ID. */
#define SYS_BD_ADDR_LEN                   BLE_GAP_ADDR_LEN      /**< Length of Bluetoth Device Address. */
#define SYS_CHIP_UID_LEN                  0x10                  /**< Length of Bluetoth Chip UID. */
#define SYS_SET_BD_ADDR(BD_ADDR_ARRAY)    nvds_put(0xC001, SYS_BD_ADDR_LEN, BD_ADDR_ARRAY)  /**< NVDS put BD address. */
#define SYS_ROM_VERSION_ADDR              0x45000               /**< The rom version address. */
/** @} */

/**
 * @defgroup GR55XX_SYS_TYPEDEF Typedefs
 * @{
 */
/**@brief The function pointers to register event callback. */
typedef void (*callback_t)(int);

/** @brief Timer callback type. */
typedef void (*timer_callback_t)(uint8_t timer_id);

/**@brief Printf callback type. */
typedef int (*vprintf_callback_t) (const char *fmt, va_list argp);

/**@brief Low power clock update function type. */
typedef void (*void_func_t)(void);

/**@brief Low power clock update function type with resturn. */
typedef int (*int_func_t)(void);

/**@brief Function type for saving user context before deep sleep. */
typedef void (*sys_context_func_t)(void);

/**@brief Error assert callback type. */
typedef void (*assert_err_cb_t)(const char *expr, const char *file, int line);

/**@brief Parameter assert callback type. */
typedef void (*assert_param_cb_t)(int param0, int param1, const char *file, int line);

/**@brief Warning assert callback type. */
typedef void (*assert_warn_cb_t)(int param0, int param1, const char *file, int line);
/** @} */

/** @addtogroup GR55XX_SYS_ENUMERATIONS Enumerations
 * @{
 */
/**@brief Definition of Device SRAM Size Enumerations. */
typedef enum
{
    SYS_DEV_SRAM_64K           = 0x02,    /**< Supported 64K SRAM.                   */
    SYS_DEV_SRAM_128K          = 0x01,    /**< Supported 128K SRAM.                  */
    SYS_DEV_SRAM_256K          = 0x00,    /**< Supported 256K SRAM.                  */
} sram_size_t;

/**@brief package type. */
typedef enum
{
    PACKAGE_NONE            = 0,    /**< Package unused. */
    PACKAGE_GR5515RGBD      = 1,    /**< BGA68 package. */
    PACKAGE_GR5515GGBD      = 2,    /**< BGA55 package. */
    PACKAGE_GR5515IGND      = 3,    /**< QFN56 + 1024KB flash package. */
    PACKAGE_GR5515I0ND      = 4,    /**< QFN56 + no flash package, support external high voltage flash only*/
    PACKAGE_GR5513BEND      = 5,    /**< QFN40 + 128KB RAM + 512KB flash packet. */
    PACKAGE_GR5515BEND      = 6,    /**< QFN40 + 256KB RAM + 512KB flash packet. */
    PACKAGE_GR5513BENDU     = 7,    /**< QFN40 + 128KB RAM + 512KB flash packet @1.7V ~ 3.6V. */
    PACKAGE_GR5515I0NDA      = 8,    /**< QFN56 + no flash package, support external high/low voltage flash  */
    PACKAGE_GR5515IENDU      = 9,    /**< QFN56 + 512KB flash package */
} package_type_t;
/** @} */

/** @addtogroup GR55XX_SYS_STRUCTURES Structures
 * @{
 */
/**@brief SDK version definition. */
typedef struct
{
    uint8_t  major;                         /**< Major version. */
    uint8_t  minor;                         /**< Minor version. */
    uint16_t build;                         /**< Build number. */
    uint32_t commit_id;                     /**< commit ID. */
}sdk_version_t;

/**@brief Assert callbacks.*/
typedef struct
{
    assert_err_cb_t   assert_err_cb;    /**< Assert error type callback. */
    assert_param_cb_t assert_param_cb;  /**< Assert parameter error type callback. */
    assert_warn_cb_t  assert_warn_cb;   /**< Assert warning type callback. */
}sys_assert_cb_t;

/**@brief Link RX information definition. */
typedef struct
{
    uint32_t rx_total_cnt;     /**< Counts of RX times. */
    uint32_t rx_sync_err_cnt;  /**< Counts of RX sync error times. */
    uint32_t rx_crc_err_cnt;   /**< Counts of RX crc error times. */
    uint32_t rx_other_err_cnt; /**< Counts of RX other error times. */
    uint32_t rx_sn_err_cnt;    /**< Counts of sn CRC error times. */
    uint32_t rx_mic_err_cnt;   /**< Counts of mic CRC error times. */
    uint32_t rx_normal_cnt;    /**< Counts of RX normal times. */
} link_rx_info_t;

/**@brief RF trim parameter information definition. */
typedef struct
{
    int8_t  rssi_cali;    /**< RSSI calibration. */
    int8_t  tx_power;     /**< TX power. */
} rf_trim_info_t;

/**@brief ADC trim parameter information definition. */
typedef struct
{
    uint16_t adc_temp;                /**< ADC TEMP. */
    uint16_t slope_int_0p8;           /**< Internal reference 0.8v. */
    uint16_t offset_int_0p8;          /**< Internal reference 0.8v. */ 
    uint16_t slope_int_1p2;           /**< Internal reference 1.2v. */ 
    uint16_t offset_int_1p2;          /**< Internal reference 1.2v. */ 
    uint16_t slope_int_1p6;           /**< Internal reference 1.6v. */ 
    uint16_t offset_int_1p6;          /**< Internal reference 1.6v. */ 
    uint16_t slope_int_2p0;           /**< Internal reference 2.0v. */ 
    uint16_t offset_int_2p0;          /**< Internal reference 2.0v. */ 
    uint16_t slope_ext_1p0;           /**< External reference 1.0v. */ 
    uint16_t offset_ext_1p0;          /**< External reference 1.0v. */ 
} adc_trim_info_t;

/**@brief PMU trim parameter information definition. */
typedef struct
{
    uint8_t  io_ldo_bypass;    /**< IO LDO bypass */
    uint8_t  io_ldo_vout;      /**< IO LDO Vout. */
    uint8_t  dig_ldo_64m;      /**< DIG LDO 64m. */
    uint8_t  dig_ldo_16m;      /**< DIG LDO 16m */
    uint8_t  dcdc_vout;        /**< DCDC Vout */
} pmu_trim_info_t;

/** @} */

/** @addtogroup GR55XX_SYS_FUNCTIONS Functions
 *  @{
 */
/**
 *****************************************************************************************
 * @brief Output debug logs.
 *
 * @param[in] format: Pointer to the log information.
 *****************************************************************************************
 */
void sys_app_printf(const char *format, ...);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] us:  Microsecond.
 *****************************************************************************************
 */
void sys_delay_us(uint32_t us);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] ms:  Millisecond.
 *****************************************************************************************
 */
void sys_delay_ms(uint32_t ms);

/**
 *****************************************************************************************
 * @brief Memory allocation.
 *
 * @param[in] size:  Requested memory size.
 *
 * @return Valid memory location if successful, else null.
 *****************************************************************************************
 */
void *sys_malloc(uint32_t size);

/**
 *****************************************************************************************
 * @brief Free allocated memory.
 *
 * @param[in] p_mem: Pointer to memory block.
 *****************************************************************************************
 */
void sys_free(void *p_mem);

/**
 *****************************************************************************************
 * @brief Register signal handler.
 *
 * @note This function is mainly used to register the upper-layer APP callback functions to the protocol layer,
 *       which will be invoked when there are event responses in the protocol layer.
 *
 * @param[in] isr_handler: callback function which to be registered.
 *****************************************************************************************
 */
void sys_signal_handler_register(callback_t isr_handler);

/**
 *****************************************************************************************
 * @brief Get SDK version.
 *
 * @note This function is mainly used to get the version of SDK.
 *
 * @param[out] p_version: The pointer to struct of @ref sdk_version_t.
 *****************************************************************************************
 */
void sys_sdk_verison_get(sdk_version_t *p_version);

/**
 *****************************************************************************************
 * @brief Save system context.
 *
 * @note This function is used to save system context before the system goes to deep sleep.
 *       Boot codes will be used to restore system context in the wakeup procedure.
 *****************************************************************************************
 */
void sys_context_save(void);

/**
 *****************************************************************************************
 * @brief Load system context.
 *
 * @note This function is used to load system context after the system goes to deep sleep.
 *****************************************************************************************
 */
void restore_sys_context(void);

/**
 *****************************************************************************************
 * @brief Save system registers.
 *
 * @note This function is used to save system register before the system goes to deep sleep.
 *
 * @param[in] p_address: The pointer to register address.
 * @param[in] value: The register value to be saved, it will be restored when system wakes up.
 *****************************************************************************************
 */
void sys_regs_save(volatile uint32_t *p_address, uint32_t value);

/**
 *****************************************************************************************
 * @brief Generate checksum info for system context.
 *
 * @note This function is used to generate checksum for system context, it will be called
 *       before deep sleep in power management module.
 *****************************************************************************************
 */
void sys_context_checksum_gen(void);

/**
 *****************************************************************************************
 * @brief Encrypt and decrypt data using Present.
 *
 * @note  This function is only used to encrypt and decrypt data that needs to be stored in Flash.
 *
 * @param[in]  addr:   Operation address (Flash address minus Flash start address).
 * @param[in]  input:  Data before encryption and decryption.
 * @param[in]  size:   Data size.
 * @param[out] output: Data after encryption and decryption.
 *****************************************************************************************
 */
void sys_security_data_use_present(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output);

/**
 *****************************************************************************************
 * @brief Check the chip's security level.
 *
 * @return 0:  Security is not supported.
 *         1:  Security is supported.
 *****************************************************************************************
 */
uint32_t sys_security_enable_status_check(void);

/**
 *****************************************************************************************
 * @brief Get the RF trim information.
 *
 * @param[out] p_rf_trim: The pointer to struct of @ref rf_trim_info_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_rf_trim_get(rf_trim_info_t *p_rf_trim);

/**
 *****************************************************************************************
 * @brief Get the ADC trim information.
 *
 * @param[out] p_adc_trim: The pointer to struct of @ref adc_trim_info_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_adc_trim_get(adc_trim_info_t *p_adc_trim);

/**
 *****************************************************************************************
 * @brief Get the PMU trim information.
 *
 * @param[out] p_pmu_trim: The pointer to struct of @ref pmu_trim_info_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_pmu_trim_get(pmu_trim_info_t *p_pmu_trim);

/**
 *****************************************************************************************
 * @brief Get the crystal trim information.
 *
 * @param[out] p_crystal_trim: offset information for crystal.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_crystal_trim_get(uint16_t *p_crystal_trim);

/**
 *****************************************************************************************
 * @brief app boot project turn on the encrypt clock.
 *
 *****************************************************************************************
 */
void app_boot_turn_on_encrypt_clock(void);

/**
 *****************************************************************************************
 * @brief app boot project set  the security clock.
 *
 *****************************************************************************************
 */
void app_boot_security_clock_set(void);

/**
 *****************************************************************************************
 * @brief jump to app firmware.	69
 *
 * @param[in] p_boot_info: Firmware system firmware information	71
 *****************************************************************************************
 */
void sys_firmware_jump(dfu_boot_info_t *p_boot_info);

/**
 *****************************************************************************************
 * @brief Get the trim checksum.
 *
 * @param[out] p_trim_sum: The pointer to the buffer for trim checksum.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_trim_sum_get(uint16_t *p_trim_sum);

/**
 *****************************************************************************************
 * @brief Get the device address information.
 *
 * @param[out] p_device_addr: Bluetooth address by default.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_addr_get(uint8_t *p_device_addr);

/**
 *****************************************************************************************
 * @brief Get the device UID information.
 *
 * @param[out] p_device_uid: Device chip UID.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_uid_get(uint8_t *p_device_uid);

/**
 *****************************************************************************************
 * @brief Get the LP gain offset 2M information.
 *
 * @param[out] p_offset: the offset of LP gain.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_lp_gain_offset_2m_get(uint8_t *p_offset);

/**
 *****************************************************************************************
 * @brief Get the RAM size information.
 *
 * @param[out] p_sram_size: The pointer to enumeration of @ref sram_size_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_sram_get(sram_size_t *p_sram_size);

/**
 *****************************************************************************************
 * @brief Get the chip's package type.
 *
 * @param[out] p_package_type: The pointer to enumeration of @ref package_type_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_package_get(package_type_t *p_package_type);

/**
 *****************************************************************************************
 * @brief Get the chip's IO LDO voltage.
 *
 * This function is an API interface for special users.
 *
 * @param[out] io_ldo: The IO LDO voltage.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_get_efuse_io_ldo(uint16_t *io_ldo);

/**
 *****************************************************************************************
 * @brief Set low power CLK frequency.
 *
 * @param[in] user_lpclk: CLK frequency.
 *****************************************************************************************
 */
void sys_lpclk_set(uint32_t user_lpclk);

/**
 ****************************************************************************************
 * @brief Convert a duration in us into a duration in lp cycles.
 *
 * The function converts a duration in us into a duration in lp cycles, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * @param[in] us:    Duration in us.
 *
 * @return Duration in lpcycles.
 ****************************************************************************************
 */
uint32_t sys_us_2_lpcycles(uint32_t us);

/**
 ****************************************************************************************
 * @brief Convert a duration in lp cycles into a duration in half us.
 *
 * The function converts a duration in lp cycles into a duration in half us, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 * @param[in]     lpcycles:    Duration in lp cycles.
 * @param[in,out] error_corr:  Insert and retrieve error created by truncating the LP Cycle Time to a half us (in half us).
 *
 * @return Duration in half us
 ****************************************************************************************
 */
uint32_t sys_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr);

/**
 *****************************************************************************************
 * @brief Reverse the policy for static address created by chip uuid .
 * @note  After sdk_v1.6.10, this policy has been updated.
 *
 *****************************************************************************************
 */
void sys_ble_static_addr_policy_reverse(void);

/**
 *****************************************************************************************
 * @brief Set BLE Sleep HeartBeat Period.
 * @note  The BLE Sleep HeartBeat Period is used to Wakeup BLE Periodically when BLE is IDLE.
 *
 * @param[in] period_hus: The wake up duration of BLE when BLE is IDEL.
 *            Range 0x00000000-0xFFFFFFFF (in unit of us).
 *                               
 * @retval ::SDK_SUCCESS Operation is Success.
 *****************************************************************************************
 */
uint16_t sys_ble_heartbeat_period_set(uint32_t period_hus);


/**
 *****************************************************************************************
 * @brief Get BLE Sleep HeartBeat Period.
 * @note  The BLE Sleep HeartBeat Period is used to Wakeup BLE Periodically when BLE is IDLE.
 *
 * @param[in] p_period_hus: Pointer to the wake up duration.
 *            Range 0x00000000-0xFFFFFFFF (in unit of us).
 *                               
 * @retval ::SDK_SUCCESS Operation is Success.
 *****************************************************************************************
 */
uint16_t sys_ble_heartbeat_period_get(uint32_t* p_period_hus);

/**
 ****************************************************************************************
 * @brief Set system maximum usage ratio of message heap.
 *
 * The function will used to set message ratio of message heap.
 * Valid ratio range is 50 - 100 percent in full message size.
 *
 * @param[in]     usage_ratio:  Usage ratio of message heap size.
 *
 ****************************************************************************************
 */
void sys_max_msg_usage_ratio_set(uint8_t usage_ratio);

/**
 ****************************************************************************************
 * @brief Set system lld layer maximum usage ratio of message heap.
 *
 * The function will used to set message ratio of message heap.
 * Valid ratio range is 50 - 100 percent in full message size.
 *
 * @param[in]     usage_ratio:  Usage ratio of message heap size.
 *
 ****************************************************************************************
 */
void sys_lld_max_msg_usage_ratio_set(uint8_t usage_ratio);

/**
 ****************************************************************************************
 * @brief Get system message heap usage ratio.
 *
 * The function will used to get message ratio of message heap.
 * This ratio is heap used percent in full message size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_msg_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get system environment heap usage ratio.
 *
 * The function will used to get environment ratio of environment heap.
 * This ratio is heap used percent in full environment size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_env_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get system attriute database heap usage ratio.
 *
 * The function will used to get attriute database ratio of attriute database heap.
 * This ratio is heap used percent in full attriute database size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_attdb_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get system non retention heap usage ratio.
 *
 * The function will used to get non retention ratio of non retention heap.
 * This ratio is heap used percent in full non retention size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_nonret_usage_ratio_get(void);

/**
 ****************************************************************************************
 * @brief Get link quality info
 *
 * @param[in]      conn_idx:  Connect index.
 * @param[in,out]  rx_info:   RX detailed information.
 *
 * @return Current connect index link quality.
 ****************************************************************************************
 */
uint8_t sys_link_quality_get(uint8_t conn_idx, link_rx_info_t* rx_info);

/**
 ****************************************************************************************
 * @brief Clear link quality info.
 *
 * @param[in] conn_idx:  Connect index.
 ****************************************************************************************
 */
void sys_link_quality_clear(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief Register low power clock update function.
 *
 * @param[in]  func_update_lpclk: function pointer to update_lpclk.
 ****************************************************************************************
 */
void sys_lpclk_update_func_register(void_func_t func_update_lpclk);

/**
 ****************************************************************************************
 * @brief Register low power clock update function with int return.
 *
 * @param[in]  func_update_lpclk: function pointer to update_lpclk.
 ****************************************************************************************
 */
void sys_lpclk_update_func_with_return_register(int_func_t func_update_lpclk);

/**
 ****************************************************************************************
 * @brief Get low power CLK frequency.
 *
 * This function is used to get the low power clock frequency.
 *
 * @return Low power CLK frequency.
 ****************************************************************************************
 */
uint32_t sys_lpclk_get(void);

/**
 ****************************************************************************************
 * @brief Get low power CLK period.
 *
 * This function is used to get the low power CLK period.
 *
 * @return Low power CLK period.
 ****************************************************************************************
 */
uint32_t sys_lpper_get(void);

/**
 *****************************************************************************************
 * @brief Register assert callbacks.
 *
 * @param[in] p_assert_cb: Pointer to assert callbacks.
 *****************************************************************************************
 */
void sys_assert_cb_register(sys_assert_cb_t *p_assert_cb);

/**
 ****************************************************************************************
 * @brief Get status of ke_event list
 * @return  true: ke_event not busy, false : ke_event busy.
 ****************************************************************************************
 */
bool sys_ke_sleep_check(void);

/**
 ****************************************************************************************
 * @brief Enable swd function
 ****************************************************************************************
 */
void sys_swd_enable(void);

/**
 ****************************************************************************************
 * @brief Diable swd function
 ****************************************************************************************
 */
void sys_swd_disable(void);

/**
 ****************************************************************************************
 * @brief  RTC calibration function.
 ****************************************************************************************
 */
void rtc_calibration(void);

/**
 ****************************************************************************************
 * @brief  RNG calibration function.
 * @note The function will call between platform_init_push and platform_init_pop.
 ****************************************************************************************
 */
void rng_calibration(void);

/**
 ****************************************************************************************
 * @brief  Reverse byte order (32 bit).  For example, 0x12345678 becomes 0x78563412.
 * @return Byte Reversed value
 ****************************************************************************************
 */
uint32_t sys_reverse_word(uint32_t value);

/**
 ****************************************************************************************
 * @brief   Reverse byte order (16 bit). For example, 0x1234 becomes 0x3412.
 * @return  Byte Reversed value
 ****************************************************************************************
 */
uint16_t sys_reverse_hword(uint16_t value);

/** @} */
#endif

/** @} */
/** @} */
