#ifndef GR_SOC_H
#define GR_SOC_H

#include "grx_sys.h"

extern void Reset_Handler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void vector_table_init(void);
extern void soc_init(void);
extern void warm_boot_process(void);
extern void platform_init(void);
#if (CONFIG_ZEPHYR_OS)
__STATIC_INLINE void soc_register_nvic(IRQn_Type indx, uint32_t func){};
#else
extern void soc_register_nvic(IRQn_Type indx, uint32_t func);
#endif
extern uint32_t get_wakeup_flag(void);

extern uint8_t nvds_put_patch(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);
extern uint8_t nvds_put_rom(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);
extern void dfu_cmd_handler_replace_for_encrypt(void);

#if defined (__CC_ARM)
extern void tiny_rw_section_init(void);
#endif

typedef void (*FuncVector_t)(void);
typedef void (*FUNC_t)(void);

#define SYS_RESET_REASON_NONE          (0U)
#define SYS_RESET_REASON_AONWDT        (1U << 2U)
/**
  * @brief  Get chip reset reason.
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref SYS_RESET_REASON_NONE
  *         @arg @ref SYS_RESET_REASON_AONWDT
  */
uint8_t sys_device_reset_reason(void);


#endif
