#ifndef __GR_COMMON_H
#define __GR_COMMON_H

#include "cmsis_compiler.h"
#include "gr5x.h"

//lint -e621  [required] Identifier clash, e.g. "ll_xx_status" and "ll_xx_enabled"
//lint -e9031 [required] Composite expression assigned to a wider essential type
//lint -e9032 [required] Composite expression with smaller essential type than other operand
//lint -e9053 [required] the shift value is at least the precision of the essential type of the left hand side
//lint -e715  [advisory] Symbol not referenced
//lint -e750  [advisory][Info] local macro not referenced
//lint -e765  [advisory][Info] XXX could be made static
//lint -e801  [advisory][Info] Use of goto is deprecated
//lint -e818  [advisory] Pointer parameter could be declared as pointing to const
//lint -e835  [advisory][Info] Use of goto is deprecated
//lint -e904  [advisory] Return statement before end of function
//lint -e954  [advisory] Pointer variable could be declared as pointing to const
//lint -e970  [advisory] Use of modifier or type '_Bool' outside of a typedef
//lint -e9003 [advisory] could define variable at block scope
//lint -e9011 [advisory]  more than one 'break' terminates loop
//lint -e9016 [advisory] pointer arithmetic other than array indexing used
//lint -e9026 [advisory] Function-like macro defined
//lint -e9030 [advisory] Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially Boolean'
//lint -e9045 [advisory] non-hidden definition of type 'xxx'
//lint -e9049 [advisory] increment/decrement operation combined with other operation with side-effects
//lint -e9050 [advisory] dependence placed on C/C++ operator precedence; operators '*' and '+'
//lint -e9058 [advisory] tag unused
//lint -e9078 [advisory] conversion between a pointer and integer type
//lint -e9079 [advisory] Conversion from pointer to void to pointer to other type
//lint -e701  [Info] Info 701: Shift left of signed quantity (int)
//lint -e716  [Info] while(1)
//lint -e717  [Info] do ... while(0);
//lint -e725  [Info] Expected positive indentation
//lint -e754  [Info] local struct member not referenced
//lint -e766  [Info] Header file not used in module
//lint -e788  [Info] enum constant not used within defaulted switch
//lint -e826  [Info] Suspicious pointer-to-pointer conversion (area too small)
//lint -e830  [Info] Location/Reference cited in prior message
//lint -e834  [Info] Operator is confusing.  Use parentheses.
//lint -e835  [Info] A zero has been given as right argument to operator '<<'
//lint -e838  [Info] Previously assigned value has not been used
//lint -e845  [Info] The right argument to operator '|' is certain to be 0

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The gr_assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
#define gr_assert_param(expr) ((expr) ? (void)0U : assert_failed((char *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
__STATIC_INLINE void assert_failed(char *file, uint32_t line)
{

}
#else
#define gr_assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */



/** @brief Disable interrupts globally in the system(apart from the NMI).
 *  This macro must be used in conjunction with the @ref GLOBAL_EXCEPTION_ENABLE macro
 *  since this last one will close the brace that the current macro opens.  This means
 *  that both macros must be located at the same scope level.
 */
#ifndef GLOBAL_EXCEPTION_DISABLE
#define GLOBAL_EXCEPTION_DISABLE()                         \
do {                                                       \
    uint32_t __l_irq_rest = __get_PRIMASK();               \
    __set_PRIMASK(1)
#endif

/** @brief Restore interrupts from the previous global disable(apart from the NMI).
 *  @sa GLOBAL_EXCEPTION_ENABLE
 */
#ifndef GLOBAL_EXCEPTION_ENABLE
//lint -e9036 while(0U) is right
#define GLOBAL_EXCEPTION_ENABLE()                          \
    if(__l_irq_rest == (uint32_t)0)                        \
    {                                                      \
        __set_PRIMASK(0);                                  \
    }                                                      \
    else                                                   \
    {                                                      \
        __set_PRIMASK(1);                                  \
    }                                                      \
} while(0)
#endif

/** @brief Disable interrupts globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE macro.
 */
#ifndef GLOBAL_INT_DISABLE
#define GLOBAL_INT_DISABLE()                               \
do {                                                       \
    extern uint32_t global_int_disable(void);              \
    uint32_t __res_mask = global_int_disable()
#endif

/** @brief Restore global interrupt.
 *  @sa GLOBAL_INT_RESTORE
 */
#ifndef GLOBAL_INT_RESTORE
#define GLOBAL_INT_RESTORE()                               \
    extern void global_int_enable(uint32_t mask);          \
    global_int_enable(__res_mask);                         \
} while(0)
#endif

/** @brief Disable external interrupts with a priority lower than IRQn_Type in the system.
 * This macro must be used in conjunction with the @ref LOCAL_INT_RESTORE macro
 * since this last one will close the brace that the current macro opens. This
 * means that both macros must be located at the same scope level.
 */
#define LOCAL_INT_DISABLE(IRQn_Type)                         \
do {                                                         \
    uint32_t __l_irq_rest = __get_BASEPRI();                 \
    __set_BASEPRI(NVIC_GetPriority(IRQn_Type) +              \
                 (1 << (NVIC_GetPriorityGrouping() + 1)));   \

/** @brief Restore external interrupts(apart from the BLE) from the previous disable.
 *  @sa EXP_BLE_INT_RESTORE
 */
#define LOCAL_INT_RESTORE()                                  \
    __set_BASEPRI(__l_irq_rest);                             \
} while(0)


/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
    HAL_OK       = 0x00U,    /**< Operation is OK. */
    HAL_ERROR    = 0x01U,    /**< Parameter error or operation is not supported. */
    HAL_BUSY     = 0x02U,    /**< Driver is busy. */
    HAL_TIMEOUT  = 0x03      /**< Timeout occurred. */
} hal_status_t;


#endif
