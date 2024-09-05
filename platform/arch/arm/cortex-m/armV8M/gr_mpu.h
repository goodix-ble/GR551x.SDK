/**
 ****************************************************************************************
 *
 * @file    gr_mpu.h
 * @author  BLE Driver Team
 * @brief   Header file of CORTEX HAL module.
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

/** @defgroup HAL_CORTEX CORTEX
  * @brief CORTEX HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR_MPU_H__
#define __GR_MPU_H__

#ifdef __cplusplus
extern "C" {
#endif


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

    uint32_t               limit_address;         /**< Specifies the limit address of the region to protect.                          */

    uint8_t                attributes_index;      /**< Specifies the memory attributes index.
                                                       This parameter can be a value of @ref CORTEX_MPU_Attributes_Number             */

    uint8_t                access_permission;     /**< Specifies the region access permission type.
                                                       This parameter can be a value of @ref CORTEX_MPU_Region_Permission_Attributes  */

    uint8_t                disable_exec;          /**< Specifies the instruction access status.
                                                       This parameter can be a value of @ref CORTEX_MPU_Instruction_Access            */

    uint8_t                is_shareable;          /**< Specifies the shareability status of the protected region.
                                                       This parameter can be a value of @ref CORTEX_MPU_Access_Shareable              */

} mpu_region_init_t;

/** @} */

/** @defgroup CORTEX_MPU_Attributes_Configuration MPU Memory Attributes Configuration
  * @{
  */

/**
  * @brief  MPU Memory Attributes initialization structure
  */
typedef struct _mpu_attributes_init_t
{

    uint8_t                number;            /**< Specifies the number of the memory attributes to configure.
                                                       This parameter can be a value of @ref CORTEX_MPU_Attributes_Number             */

    uint8_t                attributes;        /**< Specifies the memory attributes value.
                                                       This parameter can be a value of @ref CORTEX_MPU_Attributes                    */

} mpu_attributes_init_t;

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
#define  MPU_ACCESS_INNER_SHAREABLE        ((uint8_t)0x03U)  /**< MPU Instruction Access Inner Shareable */
#define  MPU_ACCESS_OUTER_SHAREABLE        ((uint8_t)0x01U)  /**< MPU Instruction Access Outer Shareable */
#define  MPU_ACCESS_NOT_SHAREABLE          ((uint8_t)0x00U)  /**< MPU Instruction Access Not Shareable   */
/** @} */

/** @defgroup CORTEX_MPU_Region_Permission_Attributes CORTEX MPU Region Permission Attributes
  * @{
  */
#define  MPU_REGION_PRIV_RW        ((uint8_t)0x00U)   /**< MPU region Read/write by privileged code only */
#define  MPU_REGION_ALL_RW         ((uint8_t)0x01U)   /**< MPU region Read/write by any privilege level  */
#define  MPU_REGION_PRIV_RO        ((uint8_t)0x02U)   /**< MPU region Read-only by privileged code only  */
#define  MPU_REGION_ALL_RO         ((uint8_t)0x03U)   /**< MPU region Read-only by any privilege level   */
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

/** @defgroup CORTEX_MPU_Attributes_Number CORTEX MPU Memory Attributes Number
  * @{
  */
#define  MPU_ATTRIBUTES_NUMBER0    ((uint8_t)0x00U)   /**< MPU Attribute Number 0 */
#define  MPU_ATTRIBUTES_NUMBER1    ((uint8_t)0x01U)   /**< MPU Attribute Number 1 */
#define  MPU_ATTRIBUTES_NUMBER2    ((uint8_t)0x02U)   /**< MPU Attribute Number 2 */
#define  MPU_ATTRIBUTES_NUMBER3    ((uint8_t)0x03U)   /**< MPU Attribute Number 3 */
#define  MPU_ATTRIBUTES_NUMBER4    ((uint8_t)0x04U)   /**< MPU Attribute Number 4 */
#define  MPU_ATTRIBUTES_NUMBER5    ((uint8_t)0x05U)   /**< MPU Attribute Number 5 */
#define  MPU_ATTRIBUTES_NUMBER6    ((uint8_t)0x06U)   /**< MPU Attribute Number 6 */
#define  MPU_ATTRIBUTES_NUMBER7    ((uint8_t)0x07U)   /**< MPU Attribute Number 7 */
/** @} */

/** @defgroup CORTEX_MPU_Attributes CORTEX MPU Attributes
  * @{
  */
#define  MPU_DEVICE_NGNRNE          ((uint8_t)0x00U)   /**< Device, noGather, noReorder, noEarly acknowledge. */
#define  MPU_DEVICE_NGNRE           ((uint8_t)0x04U)   /**< Device, noGather, noReorder, Early acknowledge.   */
#define  MPU_DEVICE_NGRE            ((uint8_t)0x08U)   /**< Device, noGather, Reorder, Early acknowledge.     */
#define  MPU_DEVICE_GRE             ((uint8_t)0x0CU)   /**< Device, Gather, Reorder, Early acknowledge.       */

#define  MPU_WRITE_THROUGH          ((uint8_t)0x00U)   /**< Normal memory, write-through. */
#define  MPU_NOT_CACHEABLE          ((uint8_t)0x04U)   /**< Normal memory, non-cacheable. */
#define  MPU_WRITE_BACK             ((uint8_t)0x04U)   /**< Normal memory, write-back.    */

#define  MPU_TRANSIENT              ((uint8_t)0x00U)   /**< Normal memory, transient.     */
#define  MPU_NON_TRANSIENT          ((uint8_t)0x08U)   /**< Normal memory, non-transient. */

#define  MPU_NO_ALLOCATE            ((uint8_t)0x00U)   /**< Normal memory, no allocate.         */
#define  MPU_W_ALLOCATE             ((uint8_t)0x01U)   /**< Normal memory, write allocate.      */
#define  MPU_R_ALLOCATE             ((uint8_t)0x02U)   /**< Normal memory, read allocate.       */
#define  MPU_RW_ALLOCATE            ((uint8_t)0x03U)   /**< Normal memory, read/write allocate. */
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
#define IS_MPU_ACCESS_SHAREABLE(__STATE__)   (((__STATE__) == MPU_ACCESS_INNER_SHAREABLE) || \
                                              ((__STATE__) == MPU_ACCESS_OUTER_SHAREABLE) || \
                                              ((__STATE__) == MPU_ACCESS_NOT_SHAREABLE))

/**
  * @brief Check if MPU region permission attribute type is valid.
  * @param __TYPE__  MPU region permission attribute type.
  * @retval SET (__TYPE__ is valid) or RESET (__TYPE__ is invalid)
  */
#define IS_MPU_REGION_PERMISSION_ATTRIBUTE(__TYPE__)  (((__TYPE__) == MPU_REGION_PRIV_RW) || \
                                                       ((__TYPE__) == MPU_REGION_ALL_RW)  || \
                                                       ((__TYPE__) == MPU_REGION_PRIV_RO) || \
                                                       ((__TYPE__) == MPU_REGION_ALL_RO))

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
  * @brief Check if MPU memory attributes number is valid.
  * @param __NUMBER__  MPU memory attributes number.
  * @retval SET (__NUMBER__ is valid) or RESET (__NUMBER__ is invalid)
  */
#define IS_MPU_ATTRIBUTES_NUMBER(NUMBER)  (((NUMBER) == MPU_ATTRIBUTES_NUMBER0) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER1) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER2) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER3) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER4) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER5) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER6) || \
                                           ((NUMBER) == MPU_ATTRIBUTES_NUMBER7))

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
uint32_t hal_systick_config(uint32_t ticks_number);

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
 * @param[in]  p_mpu_region_init: Pointer to a mpu_region_init_t structure that contains
 *                                the initialization and configuration information.
 ****************************************************************************************
 */
void hal_mpu_config_region(mpu_region_init_t *p_mpu_region_init);

/**
 ****************************************************************************************
 * @brief  Initialize and configure the memory attributes.
 *
 * @param[in]  p_mpu_attributes_init: Pointer to a mpu_attributes_init_t structure that
 *                            contains the initialization and configuration information.
 ****************************************************************************************
 */
void hal_mpu_config_memory_attributes(mpu_attributes_init_t *p_mpu_attributes_init);

#endif /* __MPU_PRESENT */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR5xx_HAL_CORTEX_H__ */

/** @} */

/** @} */

/** @} */
