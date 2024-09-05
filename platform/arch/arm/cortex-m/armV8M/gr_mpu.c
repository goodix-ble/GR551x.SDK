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
                        ##### How to use this driver #####
  ==============================================================================

    [..]
    *** How to configure Interrupts using CORTEX HAL driver ***
    ===========================================================
    [..]
    This section provides functions allowing to configure the NVIC interrupts (IRQ).
    The Cortex-M4 exceptions are managed by CMSIS functions.

    (#) Configure the NVIC Priority Grouping using hal_nvic_set_priority_grouping() function

     (#)  Configure the priority of the selected IRQ Channels using hal_nvic_set_priority()

     (#)  Enable the selected IRQ Channels using hal_nvic_enable_irq()


     -@- When the NVIC_PRIORITYGROUP_0 is selected, IRQ pre-emption is no more possible.
         The pending IRQ priority will be managed only by the sub priority.

     -@- IRQ priority order (sorted by highest to lowest priority):
        (+@) Lowest pre-emption priority
        (+@) Lowest sub priority
        (+@) Lowest hardware priority (IRQ number)

    [..]
    *** How to configure Systick using CORTEX HAL driver ***
    ========================================================
    [..]
    Setup SysTick Timer for time base

   (+) The hal_systick_config() function calls the systick_config() function which
       is a CMSIS function that:
        (++) Configures the SysTick Reload register with value passed as function parameter.
        (++) Configures the SysTick IRQ priority to the lowest value (0x0FU).
        (++) Resets the SysTick Counter register.
        (++) Configures the SysTick Counter clock source to be Core Clock Source (HCLK).
        (++) Enables the SysTick Interrupt.
        (++) Starts the SysTick Counter.

   (+) You can change the SysTick Clock source to be HCLK_Div8 by calling the macro
       __HAL_CORTEX_SYSTICKCLK_CONFIG(SYSTICK_CLKSOURCE_REFCLK) just after the
       hal_systick_config() function call. The __HAL_CORTEX_SYSTICKCLK_CONFIG() macro is defined
       inside the gr5xx_hal_cortex.h file.

   (+) You can change the SysTick IRQ priority by calling the
       hal_nvic_set_priority(SysTick_IRQn,...) function just after the hal_systick_config() function
       call. The hal_nvic_set_priority() call the nvic_set_priority() function which is a CMSIS function.

   (+) To adjust the SysTick time base, use the following formula:

       Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
       (++) Reload Value is the parameter to be passed for hal_systick_config() function
       (++) Reload Value should not exceed 0xFFFFFF

  @endverbatim
  */

/*
  Additional Tables: CORTEX_NVIC_Priority_Table
     The table below gives the allowed values of the pre-emption priority and subpriority according
     to the Priority Grouping configuration performed by hal_nvic_set_priority_grouping() function
       ==========================================================================================================================
         NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  |       Description
       ==========================================================================================================================
        NVIC_PRIORITYGROUP_0  |                0                  |            0U-255           | 0 bits for pre-emption priority
                              |                                   |                             | 8 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_1  |                0U-1               |            0U-127           | 1 bits for pre-emption priority
                              |                                   |                             | 7 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_2  |                0U-3               |            0U-63            | 2 bits for pre-emption priority
                              |                                   |                             | 6 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_3  |                0U-7               |            0U-31            | 3 bits for pre-emption priority
                              |                                   |                             | 5 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_4  |                0U-15              |            0U-15            | 4 bits for pre-emption priority
                              |                                   |                             | 4 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_5  |                0U-31              |            0U-7             | 5 bits for pre-emption priority
                              |                                   |                             | 3 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_6  |                0U-63              |            0U-3             | 6 bits for pre-emption priority
                              |                                   |                             | 2 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_7  |                0U-127             |            0U-1             | 7 bits for pre-emption priority
                              |                                   |                             | 1 bits for subpriority
       ==========================================================================================================================

*/

/* Includes ------------------------------------------------------------------*/

#include "gr5x.h"
#include "gr_common.h"
#include "gr_mpu.h"

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

//__WEAK void hal_nvic_set_priority_grouping(uint32_t priority_group)
//{
//    /* Check the parameters */
//    gr_assert_param(IS_NVIC_PRIORITY_GROUP(priority_group));

//    /* Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value */
//    NVIC_SetPriorityGrouping(priority_group);
//}

//__WEAK void hal_nvic_set_priority(IRQn_Type IRQn, uint32_t preempt_priority, uint32_t sub_priority)
//{
//    uint32_t prioritygroup = 0x00U;

//    /* Check the parameters */
//    gr_assert_param(IS_NVIC_SUB_PRIORITY(sub_priority));
//    gr_assert_param(IS_NVIC_PREEMPTION_PRIORITY(preempt_priority));

//    prioritygroup = NVIC_GetPriorityGrouping();

//    NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, preempt_priority, sub_priority));
//}

//__WEAK void hal_nvic_enable_irq(IRQn_Type IRQn)
//{
//    /* Check the parameters */
//    gr_assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

//    /* Enable interrupt */
//    NVIC_EnableIRQ(IRQn);
//}

//__WEAK void hal_nvic_disable_irq(IRQn_Type IRQn)
//{
//    /* Check the parameters */
//    gr_assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

//    /* Disable interrupt */
//    NVIC_DisableIRQ(IRQn);
//}

//__WEAK void hal_nvic_system_reset(void)
//{
//    __set_PRIMASK(1); //Disable Global Interrupt.

//    //Power on memory
//    WRITE_REG(AON_MEM->MEM_PWR_WKUP0,   0x002AAAAAU);
//    WRITE_REG(AON_MEM->MEM_PWR_WKUP1, 0x000002AAU);
//    /* Write 1 to apply the memory settings in MEM_PWR_WKUP manually. */
//    WRITE_REG(AON_MEM->MEM_PWR_APPLY, AON_MEM_MEM_PWR_APPLY_APPLY);
//    /* during the bit being 1, writing mem_pwr_apply would not take any effect. */
//    while(READ_BITS(AON_MEM->MEM_PWR_APPLY, AON_MEM_MEM_PWR_APPLY_BUSY) == AON_MEM_MEM_PWR_APPLY_BUSY);

//    /* Disable isp check during cold start. */
//    //*(uint32_t *)(GR54XX_ALIAS_ADDRESS + 0x7FF0) = 0x676f6f64;

//    /* Clear the flag of cold boot. */
//    WRITE_REG(AON_CTRL->SW_REG0, 0x00U);

//    /* System Reset */
//    NVIC_SystemReset();
//}

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

__WEAK void hal_mpu_enable(uint32_t mpu_control)
{
    __DMB(); /* Data Memory Barrier operation to force any outstanding writes to memory before enabling the MPU */

    /* Enable the MPU */
    MPU->CTRL   = mpu_control | MPU_CTRL_ENABLE_Msk;

    /* Enable fault exceptions */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    __DSB(); /* Ensure that the subsequent instruction is executed only after the write to memory */
    __ISB(); /* Flush and refill pipeline with updated MPU configuration settings */
}

__WEAK void hal_mpu_disable(void)
{
    __DMB(); /* Force any outstanding transfers to complete before disabling MPU */

    /* Disable fault exceptions */
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;

    /* Disable the MPU */
    MPU->CTRL  &= ~MPU_CTRL_ENABLE_Msk;

    __DSB(); /* Ensure that the subsequent instruction is executed only after the write to memory */
    __ISB(); /* Flush and refill pipeline with updated MPU configuration settings */
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

//__WEAK uint32_t hal_nvic_get_priority_grouping(void)
//{
//    /* Get the PRIGROUP[10:8] field value */
//    return NVIC_GetPriorityGrouping();
//}

//__WEAK void hal_nvic_get_priority(IRQn_Type IRQn, uint32_t priority_group, uint32_t *p_preempt_priority, uint32_t *p_sub_priority)
//{
//    /* Check the parameters */
//    gr_assert_param(IS_NVIC_PRIORITY_GROUP(priority_group));
//    /* Get priority for Cortex-M system or device specific interrupts */
//    NVIC_DecodePriority(NVIC_GetPriority(IRQn), priority_group, p_preempt_priority, p_sub_priority);
//}

//__WEAK void hal_nvic_set_pending_irq(IRQn_Type IRQn)
//{
//    /* Set interrupt pending */
//    NVIC_SetPendingIRQ(IRQn);
//}

//__WEAK uint32_t hal_nvic_get_pending_irq(IRQn_Type IRQn)
//{
//    /* Return 1 if pending else 0U */
//    return NVIC_GetPendingIRQ(IRQn);
//}

//__WEAK void hal_nvic_clear_pending_irq(IRQn_Type IRQn)
//{
//    /* Clear pending interrupt */
//    NVIC_ClearPendingIRQ(IRQn);
//}

//__WEAK uint32_t hal_nvic_get_active(IRQn_Type IRQn)
//{
//    /* Return 1 if active else 0U */
//    return NVIC_GetActive(IRQn);
//}


/** @} */

/** @} */

#endif /* HAL_CORTEX_MODULE_ENABLED */
/** @} */

/** @} */

