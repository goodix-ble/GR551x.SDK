/**
  ******************************************************************************
  * @file    gr_mpu.c
  * @author  BLE Driver Team
  * @brief   CORTEX HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the CORTEX:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *
  *  @verbatim
  ==============================================================================
                        ##### How to use MPU #####
  ==============================================================================
    A. Configure MPU region, include start address, end address, access permission etc.
       Up to 8 MPU regions can be configured.
       The size of an MPU region can be any size in the granularity of 32 bytes.
       The starting address of an MPU region can now also be in any address which is a multiple of 32 bytes.
       Does not allow MPU regions to be overlapped.
    B. Enable MPU.

Example usage:
#include "gr_mpu.h"
void mpu_setup(void)
{
    // This function sets up MPU to provide memory protection including null pointer
    // protection and proper memory attributes for different memory regions.
    // The memory map is defined as follows:
    // - 0x00000000 - 0x000000DF: Unmapped (null pointer protection)
    // - 0x000000E0 - 0x1FFFFFFF: ROM/Flash/Code region (read-only)
    // - 0x20000000 - 0x20027FFF: RAM region (read-write)
    // - 0x40000000 - 0x400FFFFF: Peripheral region (read-write, no execute)
    // - 0xE0000000 - 0xE00FFFFF: ARM private peripheral region (read-write, no execute)
    // - Other regions: Unmapped (access triggers MemManage fault)

    // Memory attribute indices for different memory types
    #define MPU_UNALIGN_ACCESS_ATTR_IDX   0    // Index for normal memory (non-cacheable)
    #define MPU_ALIGN_ACCESS_ATTR_IDX     1    // Index for device memory (strict ordering)

    // Memory region definitions
    #define NULL_POINTER_PROTECTION_SIZE  (0xE0UL)          // 224Byte
    #define CHIP_RAM_BASE                 (0x20000000UL)    // RAM base address
    #define CHIP_RAM_MAX_SIZE             (160*1024UL)      // 160KB
    #define PERIPHERAL_BASE               (0x40000000UL)    // Peripheral base address
    #define PERIPHERAL_SIZE               (1024*1024UL)     // 1MB
    #define ARM_REGION_BASE               (0xE0000000UL)    // ARM private peripheral base
    #define ARM_REGION_SIZE               (1024*1024UL)     // 1MB

    // Configure memory attributes for different memory types
    // Attribute 0: Normal memory, non-cacheable (for ROM/Flash/RAM without cache)
    ARM_MPU_SetMemAttr(MPU_UNALIGN_ACCESS_ATTR_IDX,
                       ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));

    // Attribute 1: Device memory, non-gathering, non-reordering, no early write acknowledgment
    ARM_MPU_SetMemAttr(MPU_ALIGN_ACCESS_ATTR_IDX,
                       ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE_nGnRnE, ARM_MPU_ATTR_DEVICE_nGnRnE));

    // Initialize MPU region configuration structure
    mpu_region_init_t mpu_region;
    memset((void *)&mpu_region, 0, sizeof(mpu_region));
    mpu_region.enable = MPU_REGION_ENABLE;

    // Region 7: ROM/Flash/Code region [0xE0, CHIP_RAM_BASE-1] - Read-only for code protection
    // Note: Address 0x00-0xDF remains unmapped for null pointer protection
    mpu_region.attributes_index = MPU_UNALIGN_ACCESS_ATTR_IDX;
    mpu_region.number = MPU_REGION_NUMBER7;
    mpu_region.base_address = NULL_POINTER_PROTECTION_SIZE;
    mpu_region.limit_address = CHIP_RAM_BASE - 1;
    mpu_region.access_permission = MPU_REGION_PRIV_RO;  // Privileged read-only
    hal_mpu_config_region(&mpu_region);

    // Region 6: SRAM region - Read-write for data storage
    mpu_region.number = MPU_REGION_NUMBER6;
    mpu_region.base_address = CHIP_RAM_BASE;
    mpu_region.limit_address = CHIP_RAM_BASE + (CHIP_RAM_MAX_SIZE - 1);
    mpu_region.access_permission = MPU_REGION_ALL_RW;   // All privilege levels read-write
    hal_mpu_config_region(&mpu_region);

    // Region 5: General peripheral region - Read-write for peripheral access
    mpu_region.attributes_index = MPU_ALIGN_ACCESS_ATTR_IDX;  // Use device memory attributes
    mpu_region.disable_exec = MPU_INSTRUCTION_ACCESS_DISABLE; // no execute
    mpu_region.number = MPU_REGION_NUMBER5;
    mpu_region.base_address = PERIPHERAL_BASE;
    mpu_region.limit_address = PERIPHERAL_BASE + (PERIPHERAL_SIZE - 1);
    mpu_region.access_permission = MPU_REGION_ALL_RW;
    hal_mpu_config_region(&mpu_region);

    // Region 4: ARM private peripheral region (NVIC, SysTick, etc.) - Read-write
    mpu_region.number = MPU_REGION_NUMBER4;
    mpu_region.base_address = ARM_REGION_BASE;
    mpu_region.limit_address = ARM_REGION_BASE + (ARM_REGION_SIZE - 1);
    mpu_region.access_permission = MPU_REGION_ALL_RW;
    hal_mpu_config_region(&mpu_region);

    // Enable MPU with the following configuration:
    // 1. MPU disabled during HardFault and NMI handlers (prevents recursive faults)
    // 2. No privileged default background region (unmapped addresses trigger MemManage faults)
    // 3. Provides null pointer protection by keeping address 0x00-0xDF unmapped
    // 4. Only explicitly configured MPU regions are accessible
    hal_mpu_enable(MPU_HFNMI_PRIVDEF_NONE);
}
  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/

#include "gr5x.h"
#include "gr_common.h"
#include "gr_mpu.h"
#include "custom_config.h"
#if (CFG_APP_DRIVER_SUPPORT == 1)
#include "app_pwr_mgmt.h"
#endif

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_MPU_V8

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/** @defgroup CORTEX_Exported_Functions CORTEX Exported Functions
  * @{
  */


/** @defgroup CORTEX_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/*
 * Note that the following operation only turns on timer, but does not turn on interrupt,
 * because RTOS compatible running environment
*/
__WEAK uint32_t hal_systick_config(uint32_t ticks)
{
    if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
    {
       return (1UL);                                                  /* Reload value impossible */
    }

    SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
    NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
    return (0UL);
}
/** @} */

/** @defgroup CORTEX_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */

#if (__MPU_PRESENT == 1U)

#define MPU_REGION_NUM   8U

/**
  * @brief MPU retention region register definition
  */
typedef struct
{
    uint32_t RBAR;     /*!< Region Base Address Register value */
    uint32_t RLAR;     /*!< Region Limit Address Register value */
} mpu_region_reg_t;

/**
  * @brief MPU retention register definition
  */
typedef struct
{
    uint32_t ctrl;                         /* MPU Control Register value */
    mpu_region_reg_t reg[MPU_REGION_NUM];  /* MPU Region Register value */
    uint32_t mair0;                        /* MPU Memory Attribute Indirection Register 0 */
    uint32_t mair1;                        /* MPU Memory Attribute Indirection Register 1 */
} mpu_retention_t;

#if CFG_SLEEP_SUPPORT
static mpu_retention_t s_mpu_retention;

#if (CFG_APP_DRIVER_SUPPORT == 1)
static bool mpu_is_allow_sleep(void)
{
    hal_mpu_suspend_reg();
    return true;
}

static const app_sleep_callbacks_t s_mpu_sleep_cb =
{
    .app_prepare_for_sleep = mpu_is_allow_sleep,
    .app_wake_up_ind       = hal_mpu_resume_reg,
};
#endif

__WEAK void hal_mpu_suspend_reg(void)
{
    s_mpu_retention.ctrl = MPU->CTRL;
    s_mpu_retention.mair0 = MPU->MAIR0;
    s_mpu_retention.mair1 = MPU->MAIR1;
    for (uint32_t i = 0; i < MPU_REGION_NUM; i++)
    {
        MPU->RNR = i;
        s_mpu_retention.reg[i].RBAR = MPU->RBAR;
        s_mpu_retention.reg[i].RLAR = MPU->RLAR;
    }
}

__WEAK void hal_mpu_resume_reg(void)
{
    for (uint32_t i = 0; i < MPU_REGION_NUM; i++)
    {
        MPU->RNR = i;
        MPU->RBAR = s_mpu_retention.reg[i].RBAR;
        MPU->RLAR = s_mpu_retention.reg[i].RLAR;
    }
    MPU->MAIR0 = s_mpu_retention.mair0;
    MPU->MAIR1 = s_mpu_retention.mair1;
    MPU->CTRL  = s_mpu_retention.ctrl;
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_mpu_suspend(void)
{
    hal_mpu_suspend_reg();
    return HAL_PM_SLEEP;
}

__WEAK void hal_pm_mpu_resume(void)
{
    hal_mpu_resume_reg();
}
#endif /* HAL_PM_ENABLE */
#endif // CFG_SLEEP_SUPPORT

__WEAK void hal_mpu_enable(uint32_t mpu_control)
{
    __DMB(); /* Data Memory Barrier operation to force any outstanding writes to memory before enabling the MPU */

    /* Enable the MPU */
    MPU->CTRL   = mpu_control | MPU_CTRL_ENABLE_Msk;

    __DSB(); /* Ensure that the subsequent instruction is executed only after the write to memory */
    __ISB(); /* Flush and refill pipeline with updated MPU configuration settings */

    /* MPU support sleep */
#if CFG_SLEEP_SUPPORT
#if (CFG_APP_DRIVER_SUPPORT == 1)
    pwr_register_sleep_cb(&s_mpu_sleep_cb, WAKEUP_PRIORITY_LOW, MPU_PWR_ID);
#else
    /* Call hal_pm_mpu_resume/suspend in hal_pm_resume/suspend_user */
#endif
#endif
}

__WEAK void hal_mpu_disable(void)
{
    __DMB(); /* Force any outstanding transfers to complete before disabling MPU */

    /* Disable the MPU */
    MPU->CTRL  &= ~MPU_CTRL_ENABLE_Msk;

    __DSB(); /* Ensure that the subsequent instruction is executed only after the write to memory */
    __ISB(); /* Flush and refill pipeline with updated MPU configuration settings */

#if CFG_SLEEP_SUPPORT
#if (CFG_APP_DRIVER_SUPPORT == 1)
    pwr_unregister_sleep_cb(MPU_PWR_ID);
#endif
#endif
}

__WEAK void hal_mpu_config_region(mpu_region_init_t *p_mpu_region_init)
{
    /* Check the parameters */
    gr_assert_param(IS_MPU_REGION_NUMBER(p_mpu_region_init->number));
    gr_assert_param(IS_MPU_REGION_ENABLE(p_mpu_region_init->enable));

    /* Follow ARM recommendation with Data Memory Barrier prior to MPU configuration */
    __DMB();

    /* Set the Region number */
    MPU->RNR = p_mpu_region_init->number;

    if (p_mpu_region_init->enable != MPU_REGION_DISABLE)
    {
        /* Check the parameters */
        gr_assert_param(IS_MPU_INSTRUCTION_ACCESS(p_mpu_region_init->disable_exec));
        gr_assert_param(IS_MPU_REGION_PERMISSION_ATTRIBUTE(p_mpu_region_init->access_permission));
        gr_assert_param(IS_MPU_ACCESS_SHAREABLE(p_mpu_region_init->is_shareable));

        MPU->RBAR = (((uint32_t)p_mpu_region_init->base_address               & 0xFFFFFFE0UL)  |
                     ((uint32_t)p_mpu_region_init->is_shareable           << MPU_RBAR_SH_Pos)  |
                     ((uint32_t)p_mpu_region_init->access_permission      << MPU_RBAR_AP_Pos)  |
                     ((uint32_t)p_mpu_region_init->disable_exec           << MPU_RBAR_XN_Pos));

        MPU->RLAR = (((uint32_t)p_mpu_region_init->limit_address                    & 0xFFFFFFE0UL) |
                     ((uint32_t)p_mpu_region_init->attributes_index       << MPU_RLAR_AttrIndx_Pos) |
                     ((uint32_t)p_mpu_region_init->enable                 << MPU_RLAR_EN_Pos));
    }
    else
    {
        MPU->RLAR = 0U;
        MPU->RBAR = 0U;
    }
}

__WEAK void hal_mpu_config_memory_attributes(mpu_attributes_init_t *p_mpu_attributes_init)
{
    __IO uint32_t *p_mair;
    uint32_t      attr_values;
    uint32_t      attr_number;

    /* Check the parameters */
    gr_assert_param(IS_MPU_ATTRIBUTES_NUMBER(p_mpu_attributes_init->number));
    /* No need to check Attributes value as all 0x0..0xFF possible */

    /* Follow ARM recommendation with Data Memory Barrier prior to MPU configuration */
    __DMB();

    if (p_mpu_attributes_init->number < MPU_ATTRIBUTES_NUMBER4)
    {
        /* Program MPU_MAIR0 */
        p_mair = &(MPU->MAIR0);
        attr_number = p_mpu_attributes_init->number;
    }
    else
    {
        /* Program MPU_MAIR1 */
        p_mair = &(MPU->MAIR1);
        attr_number = (uint32_t)p_mpu_attributes_init->number - 4U;
    }

    attr_values = *(p_mair);
    attr_values &=  ~(0xFFUL << (attr_number * 8U));
    *(p_mair) = attr_values | ((uint32_t)p_mpu_attributes_init->attributes << (attr_number * 8U));
}

#endif /* __MPU_PRESENT */

/** @} */

/** @} */

#endif /* HAL_MPU_V8 */
/** @} */

/** @} */

