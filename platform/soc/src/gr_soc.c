#include "gr_soc.h"
#ifndef DRIVER_TEST
#include "gr_includes.h"
#endif
#include "hal_flash.h"
#include "platform_sdk.h"

#include "grx_hal.h"
#include "hal_flash.h"
#include "pmu_calibration.h"
#include "platform_sdk.h"
#include "patch.h"
#include "patch_tab.h"

// NOTE: SVC #0 is reserved for freertos, DO NOT USE IT!
#define SVC_TABLE_NUM_MAX           4

#define FLASH_CS                    (LL_GPIO_PIN_2)      /* XQSPI flash CS        */
#define FLASH_CLK                   (LL_GPIO_PIN_4)      /* XQSPI flash CLK       */
#define FLASH_IO_0                  (LL_GPIO_PIN_7)      /* XQSPI flash IO0       */
#define FLASH_IO_1                  (LL_GPIO_PIN_6)      /* XQSPI flash IO1       */
#define FLASH_IO_2                  (LL_GPIO_PIN_5)      /* XQSPI flash IO2 (WP)  */
#define FLASH_IO_3                  (LL_GPIO_PIN_3)      /* XQSPI flash IO3 (HOLD)*/
#define HAL_EXFLASH_IO_PULL_SET(_PIN_, _PULL_)  ll_gpio_set_pin_pull(GPIO1, _PIN_, _PULL_)

#define REG_PL_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)
#define REG_PL_RD(addr)             (*(volatile uint32_t *)(addr))

#define SDK_VER_MAJOR                   2
#define SDK_VER_MINOR                   0
#define SDK_VER_BUILD                   2
#define COMMIT_ID                       0x502d20d5

static const sdk_version_t sdk_version = {SDK_VER_MAJOR,
                                          SDK_VER_MINOR,
                                          SDK_VER_BUILD,
                                          COMMIT_ID,};//sdk version

void sys_sdk_verison_get(sdk_version_t *p_version)
{
    memcpy(p_version, &sdk_version, sizeof(sdk_version_t));
}

volatile uint32_t g_app_msp_addr;   /* record app msp address */
static uint32_t SVC_TABLE_USER_SPACE[SVC_TABLE_NUM_MAX] __attribute__((section("SVC_TABLE")));
uint8_t g_device_reset_reason = SYS_RESET_REASON_NONE;

#if (CFG_LCP_SUPPORT && (CHIP_TYPE <= 5))
extern uint16_t gdx_lcp_buf_init(uint32_t buf_addr);
static uint8_t lcp_buf[280] __attribute__((section (".ARM.__at_0x00820000"), zero_init));
#endif

#if defined ( __CC_ARM )

SECTION_RAM_CODE __asm void SVC_Handler(void)
{
    PRESERVE8
    IMPORT   SVC_handler_proc

    TST      LR,#4                   ; Called from Handler Mode?
    MRSNE    R12,PSP                 ; Yes, use PSP
    MOVEQ    R12,SP                  ; No, use MSP
    PUSH     {R0-R3,LR}
    MOV      R0, R12
    BL       SVC_handler_proc
    MOV      R12, R0
    POP      {R0-R3,LR}
    CMP      R12,#0                  //make sure current point isn't null
    BLXNE    R12
    BX       LR                      ; RETI
SVC_Dead
    B        SVC_Dead                ; None Existing SVC
    ALIGN
}

#elif defined ( __GNUC__ )

SECTION_RAM_CODE void __attribute__((naked))SVC_Handler(void)
{
    __asm("TST R14,$4\n");
    __asm("IT NE\n");
    __asm("MRSNE   R12,PSP\n");
    __asm("IT EQ\n");
    __asm("MOVEQ   R12,SP\n");
    __asm("PUSH    {R0-R3,LR}\n");
    __asm("MOV  R0, R12\n");
    __asm("BL  SVC_handler_proc\n");
    __asm("MOV  R12, R0\n");
    __asm("POP {R0-R3,LR}\n");
    __asm("CMP R12,$0\n");
    __asm("IT NE\n");
    __asm("BLXNE     R12\n");
    __asm("BX      LR\n");
}

#elif defined (__ICCARM__)

extern uint32_t *SVC_Table;
extern uint32_t get_patch_rep_addr(uint32_t ori_func);
SECTION_RAM_CODE uint32_t SVC_handler_proc(uint32_t *svc_args)
{
    uint16_t svc_cmd;
    uint32_t svc_func_addr;
    uint32_t func_addr=0;
    svc_func_addr =svc_args[6];
    svc_cmd = *((uint16_t*)svc_func_addr-1);
    if((svc_cmd<=0xDFFF)&&(svc_cmd>=0xDF00))
    {
        func_addr =(uint32_t)SVC_Table[svc_cmd&(0xFF)];
        return func_addr ;
    }
    else
    {
        func_addr=get_patch_rep_addr(svc_func_addr);
        svc_args[6]=func_addr;
        return 0;
    }
}

SECTION_RAM_CODE void __attribute__((naked))SVC_Handler (void)
{
    asm volatile (
                  "TST R14,#4\n\t"
                  "IT NE\n\t"
                  "MRSNE   R12,PSP\n\t"
                  "IT EQ\n"
                  "MOVEQ   R12,SP \n\t"
                  "PUSH    {R0-R3,LR} \n\t"
                  "MOV  R0, R12 \n\t"
                  "BL  SVC_handler_proc \n\t"
                  "MOV  R12, R0 \n\t"
                  "POP {R0-R3,LR} \n\t"
                  "CMP R12,#0\n\t"
                  "IT NE\n\t"
                  "BLXNE R12\n\t"
                  "BX      LR\n\t");
}

#endif

__ALIGNED(0x100) FuncVector_t FuncVector_table[MAX_NUMS_IRQn + NVIC_USER_IRQ_OFFSET] = {
    0,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void soc_register_nvic(IRQn_Type indx, uint32_t func)
{
    FuncVector_table[indx + 16] = (FuncVector_t)func;
}

#if (PLAT_SUPPORT_SLEEP == 1)

__WEAK void nvds_init_error_handler(uint8_t err_code)
{
    /* nvds_deinit will erase the flash area and old data will be lost */
#ifdef NVDS_START_ADDR
    uint32_t start_addr  = NVDS_START_ADDR;
#else
    uint32_t start_addr  = 0U;
#endif
    nvds_deinit(start_addr, NVDS_NUM_SECTOR);
    nvds_init(start_addr, NVDS_NUM_SECTOR);
}

static void nvds_setup(void)
{

#ifdef NVDS_START_ADDR
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    uint8_t err_code = nvds_init(0, NVDS_NUM_SECTOR);
#endif

    switch(err_code)
    {
        case NVDS_SUCCESS:
            break;
        default:
            /* Nvds initialization other errors.
             * For more information, please see NVDS_INIT_ERR_CODE. */
            nvds_init_error_handler(err_code);
            break;
    }
}
#endif


void ble_sdk_env_init(void)
{
#if (PLAT_SUPPORT_BLE == 1)
#if (CFG_MAX_CONNECTIONS || CFG_MAX_SCAN || CFG_MAX_ADVS)
    #if (CFG_MAX_CONNECTIONS < 3) && (CFG_MAX_ADVS < 2) && (CFG_MESH_SUPPORT < 1)
    register_rwip_reset(rwip_reset_patch);
    register_rwip_init(rwip_init_patch);
    #endif

    // register the msg handler for patch
    uint16_t msg_cnt = sizeof(msg_tab) / sizeof(msg_tab_item_t);
    reg_msg_patch_tab(msg_tab, msg_cnt);

    // register the hci cmd handler for patch
    uint16_t hci_cmd_cnt = sizeof(hci_cmd_tab) / sizeof(hci_cmd_tab_item_t);
    reg_hci_cmd_patch_tab(hci_cmd_tab, hci_cmd_cnt);

    // register the gapm hci evt handler for patch
    uint16_t hci_evt_cnt = sizeof(gapm_hci_evt_tab) / sizeof(gapm_hci_evt_tab_item_t);
    reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab, hci_evt_cnt);

    #if CFG_MAX_CONNECTIONS
    ble_con_env_init();
    ble_adv_param_init(CFG_MAX_CONNECTIONS);
    #endif

    #if CFG_MAX_SCAN
    ble_scan_env_init();
    #endif

    #if CFG_MAX_ADVS
    ble_adv_env_init();
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    ble_sup_mul_link_with_same_dev();
    #endif

    #if CFG_BT_BREDR
    ble_enable_bt_bredr();
    #endif
#endif
#endif
}

static void exflash_io_pull_config(void)
{
    /* XQSPI IO configuration needs to match Flash.
       The default configuration can match most Flash */
    HAL_EXFLASH_IO_PULL_SET(FLASH_CS,   LL_GPIO_PULL_UP);
    HAL_EXFLASH_IO_PULL_SET(FLASH_CLK,  LL_GPIO_PULL_NO);
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_0, LL_GPIO_PULL_UP); /* MOSI */
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_1, LL_GPIO_PULL_UP); /* MISO */
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_2, LL_GPIO_PULL_UP); /* WP   */
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_3, LL_GPIO_PULL_UP); /* HOLD */
}

hal_status_t hal_exflash_read(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size)
{
#if (ENCRYPT_ENABLE || (CHIP_TYPE == 1) || (EXT_EXFLASH_ENABLE == 1))
    return hal_exflash_read_patch(p_exflash, addr, p_data, size);
#else
    return hal_exflash_read_rom(p_exflash, addr, p_data, size);
#endif
}

uint8_t sys_device_reset_reason(void)
{
    return g_device_reset_reason;
}

void platform_init(void)
{
    /* if BLE not fully power off, reset and power off it manually */
    if((AON->PWR_RET01 & AON_PWR_REG01_PWR_EN_PD_COMM_TIMER) || \
       (AON->PWR_RET01 & AON_PWR_REG01_PWR_EN_PD_COMM_CORE))
    {
        ll_pwr_enable_comm_core_reset();
        ll_pwr_enable_comm_timer_reset();
        ll_pwr_disable_comm_core_power();
        ll_pwr_disable_comm_timer_power();
        /* TODO: Reserve System Cold Fully Reset Method. */
        // hal_nvic_system_reset();
    }

    /* record AON_WDT reset reason */
    if(ll_aon_wdt_is_active_flag_reboot())
    {
        g_device_reset_reason |= SYS_RESET_REASON_AONWDT;
    }
    /* Clear All Wakeup Event When Cold Boot */
    ll_pwr_clear_wakeup_event(LL_PWR_WKUP_EVENT_ALL);
    for(uint8_t i = 0; i < MAX_NUMS_IRQn; i++)
    {
        NVIC_ClearPendingIRQ((IRQn_Type)(i));
    }

    #ifdef EXFLASH_WAKEUP_DELAY
    warm_boot_set_exflash_readid_delay(EXFLASH_WAKEUP_DELAY * 5);
    run_mode_t run_mode = (run_mode_t)(SYSTEM_CLOCK);
    uint16_t osc_time = ble_wakeup_osc_time_get(run_mode) + (EXFLASH_WAKEUP_DELAY * 5);
    ble_wakeup_osc_time_set(run_mode, osc_time);
    #endif

    /* enable protection. */
    platform_init_push();
    
#if (PLAT_SUPPORT_SLEEP == 1)
    /* set sram power state. */
    mem_pwr_mgmt_init();

    /* nvds module init process. */
    nvds_setup();

    /* To choose the System clock source and set the accuracy of OSC. */
    #if CFG_LPCLK_INTERNAL_EN
    platform_clock_init_rng((mcu_clock_type_t)SYSTEM_CLOCK, RNG_OSC_CLK2, 500, 0);
    #else
    #if CFG_CRYSTAL_DELAY
    platform_set_rtc_crystal_delay(CFG_CRYSTAL_DELAY);
    #endif
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RTC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
    #endif
#endif

    /* Register the SVC Table. */
    svc_table_register(SVC_TABLE_USER_SPACE);
    
#if (PLAT_SUPPORT_SLEEP == 1)
#if ENCRYPT_ENABLE
    fpb_register_patch_init_func(fpb_encrypt_mode_patch_enable);
#else
    fpb_register_patch_init_func(fpb_patch_enable);
#endif

    /* platform init process. */
    platform_sdk_init();

#if ENCRYPT_ENABLE
    dfu_cmd_handler_replace_for_encrypt();
#endif

    #ifndef DRIVER_TEST
    /* Enable auto pmu calibration function period =3s on default. */
    system_pmu_calibration_init(30000);
    #endif

    system_pmu_deinit();
#endif
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);
#if (PLAT_SUPPORT_SLEEP == 1)
    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);

    // recover the default setting by temperature, should be called in the end
    pmu_calibration_handler(NULL);

    /* RTC calibration function */
    #if !CFG_LPCLK_INTERNAL_EN
    /* Delayed for a while, because the GM of RTC has changed */
    #if CFG_CRYSTAL_DELAY
    delay_ms(CFG_CRYSTAL_DELAY);
    #endif
    rtc_calibration();
    #endif

    /* rng calibration */
    rng_calibration();
#endif

    #if (CFG_LCP_SUPPORT && (CHIP_TYPE <= 5))
    gdx_lcp_buf_init((uint32_t)lcp_buf);
    #endif

    exflash_io_pull_config();

    /* disable protection. */
    platform_init_pop();

    return;
}

void vector_table_init(void)
{
    __DMB(); // Data Memory Barrier
    FuncVector_table[0] = *(FuncVector_t *)(SCB->VTOR);
    SCB->VTOR = (uint32_t)FuncVector_table; // Set VTOR to the new vector table location
    __DSB(); // Data Synchronization Barrier to ensure all
}

void warm_boot_process(void)
{
    vector_table_init();
    pwr_mgmt_warm_boot();
}

#define SOFTWARE_REG_WAKEUP_FLAG_POS   (8)
uint32_t get_wakeup_flag(void)
{
    return (AON->SOFTWARE_2 & (1 << SOFTWARE_REG_WAKEUP_FLAG_POS));
}

void soc_init(void)
{
#if defined (__CC_ARM)
    tiny_rw_section_init();
#endif

    if (!hal_flash_init())
    {
        /* Flash fault, cannot startup.
         * TODO: Output log via UART or Dump an error code to flash. */
        while(1);
    }

    platform_flash_enable_quad();
    platform_flash_protection(FLASH_PROTECT_PRIORITY);

    platform_init();
    hal_init();

    /* record app msp */
    g_app_msp_addr = REG_PL_RD(APP_CODE_RUN_ADDR);

}

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
static const uint32_t systemClock[CLK_TYPE_NUM] = {
                                        CLK_64M, /*CPLL_S64M_CLK*/
                                        CLK_48M, /*CPLL_F48M_CLK*/
                                        CLK_16M, /*XO_S16M_CLK*/
                                        CLK_24M, /*CPLL_T24M_CLK*/
                                        CLK_16M, /*CPLL_S16M_CLK*/
                                        CLK_32M, /*CPLL_T32M_CLK*/
                                        };

// xqspi clock table by sys_clk_type
const uint32_t mcu_clk_2_qspi_clk[CLK_TYPE_NUM] = {
                                        [CPLL_S64M_CLK] = QSPI_64M_CLK,
                                        [CPLL_F48M_CLK] = QSPI_48M_CLK,
                                        [CPLL_T32M_CLK] = QSPI_32M_CLK,
                                        [CPLL_T24M_CLK] = QSPI_24M_CLK,
                                        [CPLL_S16M_CLK] = QSPI_16M_CLK,
                                        [XO_S16M_CLK] = QSPI_16M_CLK,
                                        };

uint32_t SystemCoreClock = CLK_64M;  /* System Core Clock Frequency as 64Mhz     */

//lint -e{2,10,48,63}
//The previous line of comment is to inhibit PC-Lint errors for next code block.
void SystemCoreSetClock(mcu_clock_type_t clock_type)
{
    if (clock_type >= CLK_TYPE_NUM)
        return;        // input parameter is out of range

    if ((AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL) != clock_type)
    {
        uint32_t temp = AON->PWR_RET01 & (~(AON_PWR_REG01_SYS_CLK_SEL | AON_PWR_REG01_XF_SCK_CLK_SEL));
        //When a 16M or 64M clock is switched to another clock, it needs to be switched to 32M first.
        AON->PWR_RET01 = (temp | (CPLL_T32M_CLK << AON_PWR_REG01_SYS_CLK_SEL_Pos) | (QSPI_32M_CLK << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos));

        __asm ("nop;nop;nop;nop;");
        temp = AON->PWR_RET01 & (~(AON_PWR_REG01_SYS_CLK_SEL | AON_PWR_REG01_XF_SCK_CLK_SEL));
        AON->PWR_RET01 = (temp | (clock_type << AON_PWR_REG01_SYS_CLK_SEL_Pos) | (mcu_clk_2_qspi_clk[clock_type] << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos));
    }

    SystemCoreClock = systemClock[clock_type];

    //update sleep parameters by system clock.
    pwr_mgmt_update_wkup_param();

    return;
}

void SystemCoreGetClock(mcu_clock_type_t *clock_type)
{
    *clock_type = (mcu_clock_type_t)(AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL);
}

void SystemCoreUpdateClock(void)
{
    SystemCoreClock  = systemClock[AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL];
}

void set_msp()
{
    #ifndef DRIVER_TEST
    #ifdef APP_CODE_RUN_ADDR
    __DMB();
     __set_MSP(REG_PL_RD(APP_CODE_RUN_ADDR));
    __DSB();
    #endif
    #endif
}

