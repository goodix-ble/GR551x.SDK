/**
 ****************************************************************************************
 *
 * @file    gr_mpu.h
 * @author  BLE Driver Team
 * @brief   Header file of CORTEX HAL module.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2024 GOODIX
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

/** @defgroup HAL_CORTEX CORTEX
  * @brief CORTEX HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GR_MPU_H_
#define GR_MPU_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup HAL_CORTEX_STRUCTURES Structures
  * @{
  */

#if (__MPU_PRESENT == 1U)

/** @defgroup CORTEX_MPU_Region_Configuration MPU Region Configuration
  * @{
  */

/**
  * @brief  MPU Region initialization structure
  */
typedef struct _mpu_region_init_t
{
    uint8_t                enable;                /**< Specifies the status of the region.
                                                       This parameter can be a value of @ref CORTEX_MPU_Region_Enable                 */

    uint8_t                number;                /**< Specifies the number of the region to protect.
                                                       This parameter can be a value of @ref CORTEX_MPU_Region_Number                 */

    uint32_t               base_address;          /**< Specifies the base address of the region to protect.                           */

    uint8_t                size;                  /**< Specifies the size of the region to protect.
                                                       This parameter can be a value of @ref CORTEX_MPU_Region_Size                   */

    uint8_t                subregion_disable;     /**< Specifies the number of the subregion protection to disable.
                                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF    */

    uint8_t                type_tex_field;        /**< Specifies the TEX field level.
                                                       This parameter can be a value of @ref CORTEX_MPU_TEX_Levels                    */

    uint8_t                access_permission;     /**< Specifies the region access permission type.
                                                       This parameter can be a value of @ref CORTEX_MPU_Region_Permission_Attributes  */

    uint8_t                disable_exec;          /**< Specifies the instruction access status.
                                                       This parameter can be a value of @ref CORTEX_MPU_Instruction_Access            */

    uint8_t                is_shareable;          /**< Specifies the shareability status of the protected region.
                                                       This parameter can be a value of @ref CORTEX_MPU_Access_Shareable              */

    uint8_t                is_cacheable;          /**< Specifies the cacheable status of the region protected.
                                                       This parameter can be a value of @ref CORTEX_MPU_Access_Cacheable              */

    uint8_t                is_bufferable;         /**< Specifies the bufferable status of the protected region.
                                                       This parameter can be a value of @ref CORTEX_MPU_Access_Bufferable             */

} mpu_region_init_t;

/** @} */

#endif /* __MPU_PRESENT */

/** @} */


/**
  * @defgroup  HAL_CORTEX_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup CORTEX_Exported_Constants CORTEX Exported Constants
  * @{
  */


/** @} */

#if (__MPU_PRESENT == 1U)
/** @defgroup CORTEX_MPU_HFNMI_PRIVDEF_Control MPU HFNMI and PRIVILEGED Access control
  * @{
  */
#define  MPU_HFNMI_PRIVDEF_NONE      (0x00000000U)  /**< HFNMIENA disable, PRIVDEFENA disable */
#define  MPU_HARDFAULT_NMI           (0x00000002U)  /**< HFNMIENA enable, PRIVDEFENA disable  */
#define  MPU_PRIVILEGED_DEFAULT      (0x00000004U)  /**< HFNMIENA disable, PRIVDEFENA enable */
#define  MPU_HFNMI_PRIVDEF           (0x00000006U)  /**< HFNMIENA enable, PRIVDEFENA enable  */
/** @} */

/** @defgroup CORTEX_MPU_Region_Enable CORTEX MPU Region Enable
  * @{
  */
#define  MPU_REGION_ENABLE     ((uint8_t)0x01U) /**< MPU Region Enable  */
#define  MPU_REGION_DISABLE    ((uint8_t)0x00U) /**< MPU Region Disable */
/** @} */

/** @defgroup CORTEX_MPU_Instruction_Access CORTEX MPU Instruction Access
  * @{
  */
#define  MPU_INSTRUCTION_ACCESS_ENABLE      ((uint8_t)0x00U)  /**< MPU Instruction Access Enable  */
#define  MPU_INSTRUCTION_ACCESS_DISABLE     ((uint8_t)0x01U)  /**< MPU Instruction Access Disable */
/** @} */

/** @defgroup CORTEX_MPU_Access_Shareable CORTEX MPU Instruction Access Shareable
  * @{
  */
#define  MPU_ACCESS_SHAREABLE        ((uint8_t)0x01U)  /**< MPU Instruction Access Shareable     */
#define  MPU_ACCESS_NOT_SHAREABLE    ((uint8_t)0x00U)  /**< MPU Instruction Access Not Shareable */
/** @} */

/** @defgroup CORTEX_MPU_Access_Cacheable CORTEX MPU Instruction Access Cacheable
  * @{
  */
#define  MPU_ACCESS_CACHEABLE         ((uint8_t)0x01U)  /**< MPU Instruction Access Cacheable    */
#define  MPU_ACCESS_NOT_CACHEABLE     ((uint8_t)0x00U)  /**< MPU Instruction Access Not Cacheable */
/** @} */

/** @defgroup CORTEX_MPU_Access_Bufferable CORTEX MPU Instruction Access Bufferable
  * @{
  */
#define  MPU_ACCESS_BUFFERABLE         ((uint8_t)0x01U)  /**< MPU Instruction Access Bufferable     */
#define  MPU_ACCESS_NOT_BUFFERABLE     ((uint8_t)0x00U)  /**< MPU Instruction Access Not Bufferable */
/** @} */

/** @defgroup CORTEX_MPU_TEX_Levels MPU TEX Levels
  * @{
  */
#define  MPU_TEX_LEVEL0    ((uint8_t)0x00U)   /**< MPU TEX Level 0 */
#define  MPU_TEX_LEVEL1    ((uint8_t)0x01U)   /**< MPU TEX Level 1 */
#define  MPU_TEX_LEVEL2    ((uint8_t)0x02U)   /**< MPU TEX Level 2 */
/** @} */

/** @defgroup CORTEX_MPU_Region_Size CORTEX MPU Region Size
  * @{
  */
#define   MPU_REGION_SIZE_32B      ((uint8_t)0x04U)   /**< MPU Region Size 32B    */
#define   MPU_REGION_SIZE_64B      ((uint8_t)0x05U)   /**< MPU Region Size 64B    */
#define   MPU_REGION_SIZE_128B     ((uint8_t)0x06U)   /**< MPU Region Size 128B   */
#define   MPU_REGION_SIZE_256B     ((uint8_t)0x07U)   /**< MPU Region Size 256B   */
#define   MPU_REGION_SIZE_512B     ((uint8_t)0x08U)   /**< MPU Region Size 512B   */
#define   MPU_REGION_SIZE_1KB      ((uint8_t)0x09U)   /**< MPU Region Size 1KB    */
#define   MPU_REGION_SIZE_2KB      ((uint8_t)0x0AU)   /**< MPU Region Size 2KB    */
#define   MPU_REGION_SIZE_4KB      ((uint8_t)0x0BU)   /**< MPU Region Size 4KB    */
#define   MPU_REGION_SIZE_8KB      ((uint8_t)0x0CU)   /**< MPU Region Size 8KB    */
#define   MPU_REGION_SIZE_16KB     ((uint8_t)0x0DU)   /**< MPU Region Size 16KB   */
#define   MPU_REGION_SIZE_32KB     ((uint8_t)0x0EU)   /**< MPU Region Size 32KB   */
#define   MPU_REGION_SIZE_64KB     ((uint8_t)0x0FU)   /**< MPU Region Size 64KB   */
#define   MPU_REGION_SIZE_128KB    ((uint8_t)0x10U)   /**< MPU Region Size 128KB  */
#define   MPU_REGION_SIZE_256KB    ((uint8_t)0x11U)   /**< MPU Region Size 256KB  */
#define   MPU_REGION_SIZE_512KB    ((uint8_t)0x12U)   /**< MPU Region Size 512KB  */
#define   MPU_REGION_SIZE_1MB      ((uint8_t)0x13U)   /**< MPU Region Size 1MB    */
#define   MPU_REGION_SIZE_2MB      ((uint8_t)0x14U)   /**< MPU Region Size 2MB    */
#define   MPU_REGION_SIZE_4MB      ((uint8_t)0x15U)   /**< MPU Region Size 4MB    */
#define   MPU_REGION_SIZE_8MB      ((uint8_t)0x16U)   /**< MPU Region Size 8MB    */
#define   MPU_REGION_SIZE_16MB     ((uint8_t)0x17U)   /**< MPU Region Size 16MB   */
#define   MPU_REGION_SIZE_32MB     ((uint8_t)0x18U)   /**< MPU Region Size 32MB   */
#define   MPU_REGION_SIZE_64MB     ((uint8_t)0x19U)   /**< MPU Region Size 64MB   */
#define   MPU_REGION_SIZE_128MB    ((uint8_t)0x1AU)   /**< MPU Region Size 128MB  */
#define   MPU_REGION_SIZE_256MB    ((uint8_t)0x1BU)   /**< MPU Region Size 256MB  */
#define   MPU_REGION_SIZE_512MB    ((uint8_t)0x1CU)   /**< MPU Region Size 512MB  */
#define   MPU_REGION_SIZE_1GB      ((uint8_t)0x1DU)   /**< MPU Region Size 1GB    */
#define   MPU_REGION_SIZE_2GB      ((uint8_t)0x1EU)   /**< MPU Region Size 2GB    */
#define   MPU_REGION_SIZE_4GB      ((uint8_t)0x1FU)   /**< MPU Region Size 4GB    */
/** @} */

/** @defgroup CORTEX_MPU_Region_Permission_Attributes CORTEX MPU Region Permission Attributes
  * @{
  */
#define  MPU_REGION_NO_ACCESS      ((uint8_t)0x00U)   /**< All accesses generate a permission fault */
#define  MPU_REGION_PRIV_RW        ((uint8_t)0x01U)   /**< Access from privileged software only */
#define  MPU_REGION_PRIV_RW_URO    ((uint8_t)0x02U)   /**< Writes by unprivileged software generate a permission fault */
#define  MPU_REGION_FULL_ACCESS    ((uint8_t)0x03U)   /**< Full access */
#define  MPU_REGION_PRIV_RO        ((uint8_t)0x05U)   /**< Reads by privileged software only */
#define  MPU_REGION_PRIV_RO_URO    ((uint8_t)0x06U)   /**< Read only, by privileged or unprivileged software */
/** @} */

/** @defgroup CORTEX_MPU_Region_Number CORTEX MPU Region Number
  * @{
  */
#define  MPU_REGION_NUMBER0    ((uint8_t)0x00U)   /**< MPU Region Number 0 */
#define  MPU_REGION_NUMBER1    ((uint8_t)0x01U)   /**< MPU Region Number 1 */
#define  MPU_REGION_NUMBER2    ((uint8_t)0x02U)   /**< MPU Region Number 2 */
#define  MPU_REGION_NUMBER3    ((uint8_t)0x03U)   /**< MPU Region Number 3 */
#define  MPU_REGION_NUMBER4    ((uint8_t)0x04U)   /**< MPU Region Number 4 */
#define  MPU_REGION_NUMBER5    ((uint8_t)0x05U)   /**< MPU Region Number 5 */
#define  MPU_REGION_NUMBER6    ((uint8_t)0x06U)   /**< MPU Region Number 6 */
#define  MPU_REGION_NUMBER7    ((uint8_t)0x07U)   /**< MPU Region Number 7 */
/** @} */
#endif /* __MPU_PRESENT */

/** @} */

/* Exported Macros -----------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
/** @defgroup CORTEX_Private_Macros CORTEX Private Macros
  * @{
  */


/**
  * @brief Check if SYSTICK clock source is valid.
  * @param __SOURCE__  SYSTICK clock source.
  * @retval SET (__SOURCE__ is valid) or RESET (__SOURCE__ is invalid)
  */
#define IS_SYSTICK_CLK_SOURCE(__SOURCE__)   (((__SOURCE__) == SYSTICK_CLKSOURCE_HCLK) || \
                                             ((__SOURCE__) == SYSTICK_CLKSOURCE_REFCLK))

#if (__MPU_PRESENT == 1U)

/**
  * @brief Check if MPU enable state is valid.
  * @param __STATE__  Enable state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is not invalid)
  */
#define IS_MPU_REGION_ENABLE(__STATE__) (((__STATE__) == MPU_REGION_ENABLE) || \
                                         ((__STATE__) == MPU_REGION_DISABLE))

/**
  * @brief Check if MPU instruction access state is valid.
  * @param __STATE__ MPU instruction access state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is not invalid)
  */
#define IS_MPU_INSTRUCTION_ACCESS(__STATE__) (((__STATE__) == MPU_INSTRUCTION_ACCESS_ENABLE) || \
                                              ((__STATE__) == MPU_INSTRUCTION_ACCESS_DISABLE))

/**
  * @brief Check if MPU access shareable state is valid.
  * @param __STATE__ MPU access shareable state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is not invalid)
  */
#define IS_MPU_ACCESS_SHAREABLE(__STATE__)   (((__STATE__) == MPU_ACCESS_SHAREABLE) || \
                                              ((__STATE__) == MPU_ACCESS_NOT_SHAREABLE))

/**
  * @brief Check if MPU access cacheable state is valid.
  * @param __STATE__ MPU access cacheable state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is not invalid)
  */
#define IS_MPU_ACCESS_CACHEABLE(__STATE__)   (((__STATE__) == MPU_ACCESS_CACHEABLE) || \
                                              ((__STATE__) == MPU_ACCESS_NOT_CACHEABLE))

/**
  * @brief Check if MPU access bufferable state is valid.
  * @param __STATE__ MPU access bufferable state.
  * @retval SET (__STATE__ is valid) or RESET (__STATE__ is not invalid)
  */
#define IS_MPU_ACCESS_BUFFERABLE(__STATE__)   (((__STATE__) == MPU_ACCESS_BUFFERABLE) || \
                                              ((__STATE__) == MPU_ACCESS_NOT_BUFFERABLE))

/**
  * @brief Check if MPU Tex level is valid.
  * @param __TYPE__  MPU Tex level.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_MPU_TEX_LEVEL(__TYPE__) (((__TYPE__) == MPU_TEX_LEVEL0)  || \
                                    ((__TYPE__) == MPU_TEX_LEVEL1)  || \
                                    ((__TYPE__) == MPU_TEX_LEVEL2))

/**
  * @brief Check if MPU region permission attribute type is valid.
  * @param __TYPE__  MPU region permission attribute type.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_MPU_REGION_PERMISSION_ATTRIBUTE(__TYPE__)  (((__TYPE__) == MPU_REGION_NO_ACCESS)   || \
                                                       ((__TYPE__) == MPU_REGION_PRIV_RW)     || \
                                                       ((__TYPE__) == MPU_REGION_PRIV_RW_URO) || \
                                                       ((__TYPE__) == MPU_REGION_FULL_ACCESS) || \
                                                       ((__TYPE__) == MPU_REGION_PRIV_RO)     || \
                                                       ((__TYPE__) == MPU_REGION_PRIV_RO_URO))

/**
  * @brief Check if MPU region number is valid.
  * @param __NUMBER__  MPU region number.
  * @retval SET (__NUMBER__ is valid) or RESET (__NUMBER__ is invalid)
  */
#define IS_MPU_REGION_NUMBER(__NUMBER__)    (((__NUMBER__) == MPU_REGION_NUMBER0) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER1) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER2) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER3) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER4) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER5) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER6) || \
                                             ((__NUMBER__) == MPU_REGION_NUMBER7))

/**
  * @brief Check if MPU region size is valid.
  * @param __SIZE__  MPU region size.
  * @retval SET (__SIZE__ is valid) or RESET (__SIZE__ is invalid)
  */
#define IS_MPU_REGION_SIZE(__SIZE__)    (((__SIZE__) == MPU_REGION_SIZE_32B)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_64B)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_128B)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_256B)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_512B)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_1KB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_2KB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_4KB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_8KB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_16KB)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_32KB)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_64KB)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_128KB) || \
                                         ((__SIZE__) == MPU_REGION_SIZE_256KB) || \
                                         ((__SIZE__) == MPU_REGION_SIZE_512KB) || \
                                         ((__SIZE__) == MPU_REGION_SIZE_1MB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_2MB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_4MB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_8MB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_16MB)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_32MB)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_64MB)  || \
                                         ((__SIZE__) == MPU_REGION_SIZE_128MB) || \
                                         ((__SIZE__) == MPU_REGION_SIZE_256MB) || \
                                         ((__SIZE__) == MPU_REGION_SIZE_512MB) || \
                                         ((__SIZE__) == MPU_REGION_SIZE_1GB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_2GB)   || \
                                         ((__SIZE__) == MPU_REGION_SIZE_4GB))


/**
  * @brief Check if MPU sub region is valid.
  * @param __SUBREGION__  MPU sub region.
  * @retval SET (__SUBREGION__ is valid) or RESET (__SUBREGION__ is invalid)
  */
#define IS_MPU_SUB_REGION_DISABLE(__SUBREGION__)  ((__SUBREGION__) < (uint16_t)0x00FFU)
#endif /* __MPU_PRESENT */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_CORTEX_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup CORTEX_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]
      This section provides the CORTEX HAL driver functions allowing to configure Interrupts
      Systick functionalities

@endverbatim
  * @{
  */



/**
 ****************************************************************************************
 * @brief  Initialize the System Timer and its interrupt, and start the System Tick Timer.
 *         Counter is in free running mode to generate periodic interrupts.
 *
 * @param[in]  ticks_number: Specifies the number of ticks between two interrupts.
 *
 * @retval status
 *             - 0  Function succeeded.
 *             - 1  Function failed.
 ****************************************************************************************
 */
uint32_t hal_systick_config(uint32_t ticks);

/** @} */

/** @addtogroup CORTEX_Exported_Functions_Group2 Peripheral Control functions
 *  @brief   Cortex control functions.
 *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
    [..]
      This subsection provides a set of functions allowing to control the CORTEX
      (NVIC, SYSTICK, MPU) functionalities.


@endverbatim
 * @{
 */

/* Private functions ---------------------------------------------------------*/
/** @defgroup CORTEX_Private_Functions CORTEX Private Functions
  * @brief    CORTEX private  functions
  * @{
  */

#if (__MPU_PRESENT == 1U)

/**
 ****************************************************************************************
 * @brief  Disables the MPU and clears the HFNMIENA bit (ARM recommendation)
 ****************************************************************************************
 */
void hal_mpu_disable(void);

/**
 ****************************************************************************************
 * @brief  Enable the MPU
 *
 * @param[in]  mpu_control: Specifies the control mode of the MPU during hard fault,
 *         NMI, FAULTMASK and privileged access to the default memory.
 *         This parameter can be one of the following values:
 *            @arg @ref MPU_HFNMI_PRIVDEF_NONE
 *            @arg @ref MPU_HARDFAULT_NMI
 *            @arg @ref MPU_PRIVILEGED_DEFAULT
 *            @arg @ref MPU_HFNMI_PRIVDEF
 ****************************************************************************************
 */
void hal_mpu_enable(uint32_t mpu_control);

/**
 ****************************************************************************************
 * @brief  Initialize and configures the Region and the memory to be protected.
 *
 * @param[in]  p_mpu_init: Pointer to a mpu_region_init_t structure that contains
 *                     the initialization and configuration information.
 ****************************************************************************************
 */
void hal_mpu_config_region(mpu_region_init_t *p_mpu_init);

/**
 ****************************************************************************************
 * @brief  Suspend some registers related to MPU configuration before sleep.
 ****************************************************************************************
 */
void hal_mpu_suspend_reg(void);

/**
 ****************************************************************************************
 * @brief  Restore some registers related to MPU configuration after sleep.
 *         This function must be used in conjunction with the hal_mpu_suspend_reg().
 ****************************************************************************************
 */
void hal_mpu_resume_reg(void);

#ifdef HAL_PM_ENABLE
/**
 ****************************************************************************************
 * @brief  Suspend MPU.
 * @retval ::HAL_PM_SLEEP: Allow sleep.
 ****************************************************************************************
 */
hal_pm_status_t hal_pm_mpu_suspend(void);

/**
 ****************************************************************************************
 * @brief  Resume MPU.
 *         This function must be used in conjunction with the hal_pm_mpu_suspend().
 * @retval ::None
 ****************************************************************************************
 */
void hal_pm_mpu_resume(void);
#endif /* HAL_PM_ENABLE */

#endif /* __MPU_PRESENT */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ___HAL_CORTEX_H__ */

/** @} */

/** @} */

/** @} */
