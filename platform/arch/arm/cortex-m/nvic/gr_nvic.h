
#ifndef GR_NVIC_H
#define GR_NVIC_H

#include "gr5x.h"

/** @defgroup CORTEX_Preemption_Priority_Group CORTEX Preemption Priority Group
  * @{
  */
#define NVIC_PRIORITYGROUP_0         (0x00000007U) /**< 0 bit  for pre-emption priority,
                                                                 8 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         (0x00000006U) /**< 1 bit  for pre-emption priority,
                                                                 7 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         (0x00000005U) /**< 2 bits for pre-emption priority,
                                                                 6 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         (0x00000004U) /**< 3 bits for pre-emption priority,
                                                                 5 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         (0x00000003U) /**< 4 bits for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_5         (0x00000002U) /**< 5 bits for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_6         (0x00000001U) /**< 6 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_7         (0x00000000U) /**< 7 bits for pre-emption priority,
                                                                 1 bit  for subpriority */


/**
  * @brief Check if NVIC priority group is valid.
  * @param __GROUP__  NVIC priority group.
  * @retval SET (__GROUP__ is valid) or RESET (__GROUP__ is invalid)
  */
#define IS_NVIC_PRIORITY_GROUP(__GROUP__) (((__GROUP__) == NVIC_PRIORITYGROUP_0) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_1) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_2) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_3) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_4) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_5) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_6) || \
                                           ((__GROUP__) == NVIC_PRIORITYGROUP_7))

/**
  * @brief Check if NVIC priority group is valid.
  * @param __PRIORITY__  NVIC priority group.
  * @retval SET (__PRIORITY__ is valid) or RESET (__PRIORITY__ is invalid)
  */
#define IS_NVIC_PREEMPTION_PRIORITY(__PRIORITY__)   ((__PRIORITY__) < 0x80U)

/**
  * @brief Check if NVIC sub priority is valid.
  * @param __PRIORITY__  NVIC sub priority.
  * @retval SET (__PRIORITY__ is valid) or RESET (__PRIORITY__ is invalid)
  */
#define IS_NVIC_SUB_PRIORITY(__PRIORITY__)          ((__PRIORITY__) <= 0xFFU)

/**
  * @brief Check if NVIC deivce IRQ is valid.
  * @param __IRQ__  NVIC device IRQ.
  * @retval SET (__IRQ__ is valid) or RESET (__IRQ__ is invalid)
  */
#define IS_NVIC_DEVICE_IRQ(__IRQ__)                 ((__IRQ__) >= 0x00)



/**
 ****************************************************************************************
 * @brief  Set the priority grouping field (pre-emption priority and subpriority)
 *         using the required unlock sequence.
 *
 * @note   When the NVIC_PriorityGroup_0 is selected, IRQ pre-emption is no more possible.
 *         The pending IRQ priority will be managed only by the subpriority.
 *
 * @param[in]  priority_group: The priority grouping bits length.
 *         This parameter can be one of the following values:
 *         @arg @ref NVIC_PRIORITYGROUP_0 0 bit  for pre-emption priority,
 *                                        8 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_1 1 bit  for pre-emption priority,
 *                                        7 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_2 2 bits for pre-emption priority,
 *                                        6 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_3 3 bits for pre-emption priority,
 *                                        5 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_4 4 bits for pre-emption priority,
 *                                        4 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_5 5 bits for pre-emption priority,
 *                                        3 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_6 6 bits for pre-emption priority,
 *                                        2 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_7 7 bits for pre-emption priority,
 *                                        1 bit  for subpriority
 ****************************************************************************************
 */
void hal_nvic_set_priority_grouping(uint32_t priority_group);

/**
 ****************************************************************************************
 * @brief  Set the priority of an interrupt.
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 * @param[in]  preempt_priority: The pre-emption priority for the IRQn channel.
 *         This parameter can be a value between 0 and 127 as described in the table CORTEX_NVIC_Priority_Table.
 *         A lower priority value indicates a higher priority
 * @param[in]  sub_priority: The subpriority level for the IRQ channel.
 *         This parameter can be a value between 0 and 255 as described in the table CORTEX_NVIC_Priority_Table.
 *         A lower priority value indicates a higher priority.
 ****************************************************************************************
 */
void hal_nvic_set_priority(IRQn_Type IRQn, uint32_t preempt_priority, uint32_t sub_priority);

/**
 ****************************************************************************************
 * @brief  Enable a device specific interrupt in the NVIC interrupt controller.
 *
 * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
 *         function should be called before.
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 ****************************************************************************************
 */
void hal_nvic_enable_irq(IRQn_Type IRQn);

/**
 ****************************************************************************************
 * @brief  Disable a device specific interrupt in the NVIC interrupt controller.
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 ****************************************************************************************
 */
void hal_nvic_disable_irq(IRQn_Type IRQn);

/**
 ****************************************************************************************
 * @brief  Initiate a system reset request to reset the MCU.
 ****************************************************************************************
 */
void hal_nvic_system_reset(void);



/**
 ****************************************************************************************
 * @brief  Get the priority grouping field from the NVIC Interrupt Controller.
 *
 * @return Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field)
 ****************************************************************************************
 */
uint32_t hal_nvic_get_priority_grouping(void);

/**
 ****************************************************************************************
 * @brief  Get the priority of an interrupt.
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration.
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 * @param[in]   priority_group: The priority grouping bits length.
 *         This parameter can be one of the following values:
 *         @arg @ref NVIC_PRIORITYGROUP_0 0 bit  for pre-emption priority,
 *                                        8 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_1 1 bit  for pre-emption priority,
 *                                        7 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_2 2 bits for pre-emption priority,
 *                                        6 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_3 3 bits for pre-emption priority,
 *                                        5 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_4 4 bits for pre-emption priority,
 *                                        4 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_5 5 bits for pre-emption priority,
 *                                        3 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_6 6 bits for pre-emption priority,
 *                                        2 bits for subpriority
 *         @arg @ref NVIC_PRIORITYGROUP_7 7 bits for pre-emption priority,
 *                                        1 bit  for subpriority
 * @param[in]  p_preempt_priority: Pointer on the Preemptive priority value (starting from 0).
 * @param[in]  p_sub_priority: Pointer on the Subpriority value (starting from 0).
 ****************************************************************************************
 */
void hal_nvic_get_priority(IRQn_Type IRQn, uint32_t priority_group, uint32_t *p_preempt_priority, uint32_t *p_sub_priority);

/**
 ****************************************************************************************
 * @brief  Set Pending bit of an external interrupt.
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 ****************************************************************************************
 */
void hal_nvic_set_pending_irq(IRQn_Type IRQn);

/**
 ****************************************************************************************
 * @brief  Get Pending Interrupt (reads the pending register in the NVIC
 *         and returns the pending bit for the specified interrupt).
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 *
 * @return status
 *             - 0  Interrupt status is not pending.
 *             - 1  Interrupt status is pending.
 ****************************************************************************************
 */
uint32_t hal_nvic_get_pending_irq(IRQn_Type IRQn);

/**
 ****************************************************************************************
 * @brief  Clear the pending bit of an external interrupt.
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 ****************************************************************************************
 */
void hal_nvic_clear_pending_irq(IRQn_Type IRQn);

/**
 ****************************************************************************************
 * @brief  Get active interrupt (reads the active register in NVIC and returns the active bit).
 *
 * @param[in]  IRQn: External interrupt number.
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete GR5xx Devices IRQ Channels list, please refer to the appropriate CMSIS device file (gr533xxx.h))
 *
 * @return status
 *             - 0  Interrupt status is not pending.
 *             - 1  Interrupt status is pending.
 ****************************************************************************************
 */
uint32_t hal_nvic_get_active(IRQn_Type IRQn);

#endif
