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
    A. Configure MPU region, include start address, size, access permission etc.
       Up to 8 MPU regions can be configured.
       The region size is 32 bytes to 4 gigabytes, refer CORTEX_MPU_Region_Size in gr_mpu.h.
       The region starting address must be a multiple of the region size.
       If regions overlap, the configuration with the larger number in the overlapping area takes effect.
    B. Enable MPU.

Example usage:
#include "gr_mpu.h"
void mpu_setup(void)
{
    mpu_region_init_t mpu_region;
    memset((void *)&mpu_region, 0 ,sizeof(mpu_region));
    mpu_region.enable = MPU_REGION_ENABLE;

    // Set 0(NULL) no access. Access to NULL pointers is prohibited
    mpu_region.number = MPU_REGION_NUMBER7;
    mpu_region.base_address = 0U;
    mpu_region.size = MPU_REGION_SIZE_32B; // 32B is the smallest MPU size
    mpu_region.access_permission = MPU_REGION_NO_ACCESS;
    hal_mpu_config_region(&mpu_region);

    // Set MSP minimum address(32 bytes) no access(check stack overflow)
    // If no reading or writing at the minimum address, the stack overflow will not be detected.
    mpu_region.number = MPU_REGION_NUMBER6;
#if defined ( __CC_ARM )
    extern uint32_t Image$$ARM_LIB_STACK$$ZI$$Base;
    uint32_t msp_min_addr = (uint32_t )&Image$$ARM_LIB_STACK$$ZI$$Base;
#elif defined ( __ICCARM__ )
    uint32_t msp_min_addr = (uint32_t )&__section_begin("CSTACK");
#elif defined ( __GNUC__ )
    extern char* __StackLimit;
    uint32_t msp_min_addr = (uint32_t)&__StackLimit;
#else
   #error "Not Support Other Compiler"
#endif
    mpu_region.base_address = ((msp_min_addr + 31) & ~31); // round up to 32
    mpu_region.size = MPU_REGION_SIZE_32B;
    mpu_region.access_permission = MPU_REGION_NO_ACCESS;
    hal_mpu_config_region(&mpu_region);

    // Set address [0, 0x20000000] to read-only. Code region Writing is prohibited
    mpu_region.number = MPU_REGION_NUMBER5;
    mpu_region.base_address = 0;
    mpu_region.size = MPU_REGION_SIZE_512MB;
    mpu_region.access_permission = MPU_REGION_PRIV_RO_URO;
    hal_mpu_config_region(&mpu_region);

    hal_mpu_enable(MPU_PRIVILEGED_DEFAULT);
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

#ifdef HAL_MPU_V7

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/** @defgroup CORTEX_Exported_Functions CORTEX Exported Functions
  * @{
  */

/*
 * Note that the following operation only turns on timer, but does not turn on interrupt,
 * because RTOS compatible running environment
*/
//lint -e904
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
    uint32_t RBAR;     /*!< MPU Region Base Address Register value */
    uint32_t RASR;     /*!< MPU Region Attribute and Size Register value */
} mpu_region_reg_t;

/**
  * @brief MPU retention register definition
  */
typedef struct
{
    uint32_t ctrl;                         /* MPU Control Register value */
    mpu_region_reg_t reg[MPU_REGION_NUM];  /* MPU Region Register value */
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
    for (uint32_t i = 0; i < MPU_REGION_NUM; i++)
    {
        MPU->RNR = i;
        s_mpu_retention.reg[i].RBAR = MPU->RBAR & MPU_RBAR_ADDR_Msk;
        s_mpu_retention.reg[i].RASR = MPU->RASR;
    }
}

__WEAK void hal_mpu_resume_reg(void)
{
    for (uint32_t i = 0; i < MPU_REGION_NUM; i++)
    {
        MPU->RNR = i;
        MPU->RBAR = s_mpu_retention.reg[i].RBAR;
        MPU->RASR = s_mpu_retention.reg[i].RASR;
    }
    MPU->CTRL = s_mpu_retention.ctrl;
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

__WEAK void hal_mpu_disable(void)
{
    __DMB();
    /* Disable the MPU */
    MPU->CTRL = 0U;
    __DSB();
    __ISB();

#if CFG_SLEEP_SUPPORT
#if (CFG_APP_DRIVER_SUPPORT == 1)
    pwr_unregister_sleep_cb(MPU_PWR_ID);
#endif
#endif
}

__WEAK void hal_mpu_enable(uint32_t mpu_control)
{
    __DMB();
    /* Enable the MPU */
    MPU->CTRL = mpu_control | MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();

    /* MPU support sleep */
#if CFG_SLEEP_SUPPORT
#if (CFG_APP_DRIVER_SUPPORT == 1)
    pwr_register_sleep_cb(&s_mpu_sleep_cb, WAKEUP_PRIORITY_LOW, MPU_PWR_ID);
#else
    /* Call hal_pm_mpu_resume/suspend in hal_pm_resume/suspend_user */
#endif
#endif
}

__WEAK void hal_mpu_config_region(mpu_region_init_t *p_mpu_init)
{
    /* Check the parameters */
    gr_assert_param(IS_MPU_REGION_NUMBER(p_mpu_init->number));
    gr_assert_param(IS_MPU_REGION_ENABLE(p_mpu_init->enable));

    /* Set the Region number */
    MPU->RNR = p_mpu_init->number;

    if ((uint8_t)RESET != (p_mpu_init->enable))
    {
        /* Check the parameters */
        gr_assert_param(IS_MPU_INSTRUCTION_ACCESS(p_mpu_init->disable_exec));
        gr_assert_param(IS_MPU_REGION_PERMISSION_ATTRIBUTE(p_mpu_init->access_permission));
        gr_assert_param(IS_MPU_TEX_LEVEL(p_mpu_init->type_tex_field));
        gr_assert_param(IS_MPU_ACCESS_SHAREABLE(p_mpu_init->is_shareable));
        gr_assert_param(IS_MPU_ACCESS_CACHEABLE(p_mpu_init->is_cacheable));
        gr_assert_param(IS_MPU_ACCESS_BUFFERABLE(p_mpu_init->is_bufferable));
        gr_assert_param(IS_MPU_SUB_REGION_DISABLE(p_mpu_init->subregion_disable));
        gr_assert_param(IS_MPU_REGION_SIZE(p_mpu_init->size));

        MPU->RBAR = p_mpu_init->base_address;
        MPU->RASR = ((uint32_t)p_mpu_init->disable_exec             << MPU_RASR_XN_Pos)   |
                    ((uint32_t)p_mpu_init->access_permission        << MPU_RASR_AP_Pos)   |
                    ((uint32_t)p_mpu_init->type_tex_field           << MPU_RASR_TEX_Pos)  |
                    ((uint32_t)p_mpu_init->is_shareable             << MPU_RASR_S_Pos)    |
                    ((uint32_t)p_mpu_init->is_cacheable             << MPU_RASR_C_Pos)    |
                    ((uint32_t)p_mpu_init->is_bufferable            << MPU_RASR_B_Pos)    |
                    ((uint32_t)p_mpu_init->subregion_disable        << MPU_RASR_SRD_Pos)  |
                    ((uint32_t)p_mpu_init->size                     << MPU_RASR_SIZE_Pos) |
                    ((uint32_t)p_mpu_init->enable                   << MPU_RASR_ENABLE_Pos);
    }
    else
    {
        MPU->RBAR = 0x00U;
        MPU->RASR = 0x00U;
    }
}
#endif /* __MPU_PRESENT */

/** @} */

/** @} */

#endif /* HAL_MPU_V7 */
/** @} */

/** @} */

