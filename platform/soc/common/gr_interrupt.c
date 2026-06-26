/**
 *****************************************************************************************
 *
 * @file   gr_interrupt.c
 *
 * @brief  Interrupt Service Routines.
 *
 *****************************************************************************************

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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "custom_config.h"
#include "grx_sys.h"

/******************************************************************************/
/*           Cortex-M4F Processor Interruption and Exception Handlers         */
/*           Add here the Interrupt Handler for the BLE peripheral(s)         */
/******************************************************************************/
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SecureFault_Handler(void);

#ifndef SYS_FAULT_TRACE_ENABLE
#define SYS_FAULT_TRACE_ENABLE    0
#endif

/**
 ****************************************************************************************
 * @brief  hardfault Interrupt Handler
 * @retval void
 ****************************************************************************************
 */
#if SYS_FAULT_TRACE_ENABLE
__WEAK void app_log_flush(void)
{
}

SECTION_RAM_CODE __WEAK void cortex_backtrace_fault_handler(uint32_t fault_handler_lr, uint32_t fault_handler_sp)
{
}

static void hardfault_trace_impl(uint32_t sp)
{
    uint32_t stacked_r0 = ((uint32_t *)sp)[0];
    uint32_t stacked_r1 = ((uint32_t *)sp)[1];
    uint32_t stacked_r2 = ((uint32_t *)sp)[2];
    uint32_t stacked_r3 = ((uint32_t *)sp)[3];

    uint32_t stacked_r12 = ((uint32_t *)sp)[4];
    uint32_t stacked_lr = ((uint32_t *)sp)[5];
    uint32_t stacked_pc = ((uint32_t *)sp)[6];
    uint32_t stacked_psr = ((uint32_t *)sp)[7];

    printf("HARDFAULT CALLSTACK INFO:\r\n");
    printf("================================\r\n");
    printf("R0 = %x\r\n", stacked_r0);
    printf("R1 = %x\r\n", stacked_r1);
    printf("R2 = %x\r\n", stacked_r2);
    printf("R3 = %x\r\n", stacked_r3);
    printf("R12 = %x\r\n", stacked_r12);
    printf("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
    printf("PC [R15] = %x  program counter\r\n", stacked_pc);
    printf("PSR = %x\r\n", stacked_psr);
    printf("BFAR = %x\r\n", (*((volatile uint32_t *)(0xE000ED38U))));
    printf("CFSR = %x\r\n", (*((volatile uint32_t *)(0xE000ED28U))));
    printf("HFSR = %x\r\n", (*((volatile uint32_t *)(0xE000ED2CU))));
    printf("DFSR = %x\r\n", (*((volatile uint32_t *)(0xE000ED30U))));
    printf("AFSR = %x\r\n", (*((volatile uint32_t *)(0xE000ED3CU))));
    printf("SCB_SHCSR = %x\r\n", SCB->SHCSR);
    printf("================================\r\n");
    app_log_flush();

    for (;;)
    {
    }
}

#if defined ( __CC_ARM )
SECTION_RAM_CODE __WEAK void hardfault_trace_handler(uint32_t sp)
{
    hardfault_trace_impl(sp);
}

SECTION_RAM_CODE __WEAK __asm void HardFault_Handler (void)
{
#if (ENABLE_BACKTRACE_FEA == 0)//use fault trace module
    PRESERVE8
    IMPORT  hardfault_trace_handler
    TST LR,#4
    ITE EQ
    MRSEQ R0,MSP
    MRSNE R0,PSP
    BL  hardfault_trace_handler
#elif (ENABLE_BACKTRACE_FEA == 1)//use cortex_backtrace module
    PRESERVE8
    IMPORT  cortex_backtrace_fault_handler
    MOV     r0, lr
    MOV     r1, sp
    BL      cortex_backtrace_fault_handler
#endif

Fault_Loop
    BL      Fault_Loop
    ALIGN
}
#elif(defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6100100))
__WEAK void __attribute__((used)) hardfault_trace_handler(uint32_t sp)
{
    hardfault_trace_impl(sp);
}

SECTION_RAM_CODE __WEAK void HardFault_Handler (void)
{
#if (ENABLE_BACKTRACE_FEA == 0)//use fault trace module
    __asm volatile(
        "TST     LR,#4\n\t"
        "ITE     EQ\n\t"
        "MRSEQ   R0,MSP\n\t"
        "MRSNE   R0,PSP\n\t"
        "BL      hardfault_trace_handler\n\t"
    );
#elif (ENABLE_BACKTRACE_FEA == 1)//use cortex_backtrace module
    __asm volatile(
        "MOV     r0, lr\n\t"
        "MOV     r1, sp\n\t"
        "BL      cortex_backtrace_fault_handler\n\t"
    );
#endif
    for (;;)
    {
    }
}
#elif defined ( __GNUC__ )
__WEAK void __attribute__((used)) hardfault_trace_handler(uint32_t sp)
{
   hardfault_trace_impl(sp);
}

SECTION_RAM_CODE __WEAK void HardFault_Handler (void)
{
#if (ENABLE_BACKTRACE_FEA == 0)//use fault trace module
    __asm("TST     LR,#4\n");
    __asm("ITE     EQ\n");
    __asm("MRSEQ   R0,MSP\n");
    __asm("MRSNE   R0,PSP\n");
    __asm("BL      hardfault_trace_handler\n");
#elif (ENABLE_BACKTRACE_FEA == 1)//use cortex_backtrace module
    __asm("MOV     r0, lr");
    __asm("MOV     r1, sp");
    __asm("BL      cortex_backtrace_fault_handler");
#endif

    for (;;)
    {
    }
}

#elif defined ( __ICCARM__ )
__root __WEAK void hardfault_trace_handler(uint32_t sp)
{
    hardfault_trace_impl(sp);
}

SECTION_RAM_CODE void __attribute__((naked))HardFault_Handler (void)
{
    extern void hardfault_trace_handler(uint32_t sp);
    extern void cortex_backtrace_fault_handler(uint32_t fault_handler_lr, uint32_t fault_handler_sp);

#if (ENABLE_BACKTRACE_FEA == 0)//use fault trace module
    asm volatile ("TST     LR,#4 \n\t"
                  "ITE     EQ \n\t"
                  "MRSEQ   R0,MSP \n\t"
                  "MRSNE   R0,PSP \n\t"
                  "LDR     R1, =hardfault_trace_handler \n\t"
                  "BLX     R1");
#elif (ENABLE_BACKTRACE_FEA == 1)//use cortex_backtrace module
    asm volatile ("MOV     r0, lr \n\t"
                  "MOV     r1, sp \n\t"
                  "LDR     R2, =cortex_backtrace_fault_handler \n\t"
                  "BLX     R2");
#endif
    for (;;)
    {
    }
}

#else

__WEAK void HardFault_Handler (void)
{
    for (;;)
    {
    }
}

#endif

#else /*SYS_FAULT_TRACE_ENABLE*/
SECTION_RAM_CODE __WEAK void HardFault_Handler (void)
{
    for (;;)
    {
    }
}

#endif  /*SYS_FAULT_TRACE_ENABLE*/

/**
 ****************************************************************************************
 * @brief  MemManage fault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void MemManage_Handler(void)
{
    for (;;)
    {
    }
}

/**
 ****************************************************************************************
 * @brief  Bus Fault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void BusFault_Handler(void)
{
    for (;;)
    {
    }
}

/**
 ****************************************************************************************
 * @brief  UsageFault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void UsageFault_Handler(void)
{
    for (;;)
    {
    }
}

/**
 ****************************************************************************************
 * @brief  SecureFault Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void SecureFault_Handler(void)
{
    for (;;)
    {
    }
}

/**
 ****************************************************************************************
 * @brief  SVC Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
#if defined (SOC_SGP)
__WEAK void SVC_Handler(void)
{
    for (;;)
    {
    }
}
#else
#if defined ( __CC_ARM )
SECTION_RAM_CODE __WEAK __ASM void SVC_Handler(void)
{
    PRESERVE8
    IMPORT gr5xx_svc_process
    LDR    R0, =gr5xx_svc_process
    BX     R0
    ALIGN
}
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6100100)
extern void gr5xx_svc_process(void);
void (*volatile gr5xx_svc_process_handle)(void) = gr5xx_svc_process;
SECTION_RAM_CODE __WEAK __attribute__((naked,used)) void SVC_Handler(void)
{
    __asm("LDR    R0, =gr5xx_svc_process\n"
          "BX     R0");
}
#elif !defined ( __CC_ARM ) && defined ( __GNUC__ )
SECTION_RAM_CODE __WEAK void __attribute__((naked))SVC_Handler(void)
{
    extern void gr5xx_svc_process(void);
    __asm("LDR    R0, =gr5xx_svc_process\n"
          "BX     R0");
}
#elif defined (__ICCARM__)
SECTION_RAM_CODE __WEAK void __attribute__((naked))SVC_Handler (void)
{
    extern void gr5xx_svc_process(void);
    asm volatile ("LDR    R0, =gr5xx_svc_process \n\t"
                  "BX     R0");
}
#endif
#endif /* SOC_SGP */
/**
 ****************************************************************************************
 * @brief  DebugMon Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void DebugMon_Handler(void)
{
    for (;;)
    {
    }
}

/**
 ****************************************************************************************
 * @brief  PendSV Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void PendSV_Handler(void)
{
    for (;;)
    {
    }
}

/**
 ****************************************************************************************
 * @brief  SysTick Interrupt Handler
 * @retval  void
 ****************************************************************************************
 */
__WEAK void SysTick_Handler(void)
{
    for (;;)
    {
    }
}

__WEAK void NMI_Handler(void)
{
    for (;;)
    {
    }
}
