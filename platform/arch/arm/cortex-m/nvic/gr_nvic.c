
#include "gr_common.h"
#include "gr_nvic.h"

#ifdef HAL_NVIC_RESET_VERSION_LEGACY
#include "ll_aon_wdt.h"
#else
#include "ll_misc.h"
#endif

__WEAK void hal_nvic_set_priority_grouping(uint32_t priority_group)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_PRIORITY_GROUP(priority_group));

    /* Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value */
    NVIC_SetPriorityGrouping(priority_group);
}

__WEAK void hal_nvic_set_priority(IRQn_Type IRQn, uint32_t preempt_priority, uint32_t sub_priority)
{
    uint32_t prioritygroup = 0x00U;

    /* Check the parameters */
    gr_assert_param(IS_NVIC_SUB_PRIORITY(sub_priority));
    gr_assert_param(IS_NVIC_PREEMPTION_PRIORITY(preempt_priority));

    prioritygroup = NVIC_GetPriorityGrouping();

    NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, preempt_priority, sub_priority));
}

__WEAK void hal_nvic_enable_irq(IRQn_Type IRQn)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

    /* Enable interrupt */
    NVIC_EnableIRQ(IRQn);
}

__WEAK void hal_nvic_disable_irq(IRQn_Type IRQn)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

    /* Disable interrupt */
    NVIC_DisableIRQ(IRQn);
}

void hal_nvic_system_reset(void)
{
    // design for new platform .
#ifndef HAL_NVIC_RESET_VERSION_LEGACY
    __set_PRIMASK(1); //Disable Global Interrupt.
    ll_misc_global_soft_reset();
    while(1)
    {
        ;
    }
#else
    __set_PRIMASK(1); //Disable Global Interrupt.

    //Power on memory
    WRITE_REG(AON_MEM->MEM_PWR_WKUP0,   0x002AAAAAU);
    WRITE_REG(AON_MEM->MEM_PWR_WKUP1, 0x000002AAU);
    /* Write 1 to apply the memory settings in MEM_PWR_WKUP manually. */
    WRITE_REG(AON_MEM->MEM_PWR_APPLY, AON_MEM_MEM_PWR_APPLY_APPLY);
    /* during the bit being 1, writing mem_pwr_apply would not take any effect. */
    while(READ_BITS(AON_MEM->MEM_PWR_APPLY, AON_MEM_MEM_PWR_APPLY_BUSY) == AON_MEM_MEM_PWR_APPLY_BUSY)
    {
    }

    /* Disable isp check during cold start. */
    *(uint32_t *)(GR5405_ALIAS_ADDRESS + 0x7FF0) = 0x676f6f64;

    /* Clear the flag of cold boot. */
    WRITE_REG(AON_CTL->SOFTWARE_REG0, 0x00U);

    /* Reset */
    ll_aon_wdt_unlock();

    ll_aon_wdt_disable();
    while(ll_aon_wdt_is_busy() == (uint32_t)0x1)
    {
    }

    ll_aon_wdt_set_reload_counter(5);
    ll_aon_wdt_reload_counter();
    while(ll_aon_wdt_is_busy() == (uint32_t)0x1)
    {
    }

    ll_aon_wdt_clear_flag_alarm();

    ll_aon_wdt_enable();
    while(ll_aon_wdt_is_busy() == (uint32_t)0x1)
    {
    }

    ll_aon_wdt_lock();

    // Wait reset.
    for (;;)
    {
    }
#endif
}


__WEAK uint32_t hal_nvic_get_priority_grouping(void)
{
    /* Get the PRIGROUP[10:8] field value */
    return NVIC_GetPriorityGrouping();
}

__WEAK void hal_nvic_get_priority(IRQn_Type IRQn, uint32_t priority_group, uint32_t *p_preempt_priority, uint32_t *p_sub_priority)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_PRIORITY_GROUP(priority_group));
    /* Get priority for Cortex-M system or device specific interrupts */
    NVIC_DecodePriority(NVIC_GetPriority(IRQn), priority_group, p_preempt_priority, p_sub_priority);
}

__WEAK void hal_nvic_set_pending_irq(IRQn_Type IRQn)
{
    /* Set interrupt pending */
    NVIC_SetPendingIRQ(IRQn);
}

__WEAK uint32_t hal_nvic_get_pending_irq(IRQn_Type IRQn)
{
    /* Return 1 if pending else 0U */
    return NVIC_GetPendingIRQ(IRQn);
}

__WEAK void hal_nvic_clear_pending_irq(IRQn_Type IRQn)
{
    /* Clear pending interrupt */
    NVIC_ClearPendingIRQ(IRQn);
}

__WEAK uint32_t hal_nvic_get_active(IRQn_Type IRQn)
{
    /* Return 1 if active else 0U */
    return NVIC_GetActive(IRQn);
}
