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
#include "app_pwr_mgmt.h"

#define FLASH_CS                    (LL_GPIO_PIN_2)      /* XQSPI flash CS        */
#define FLASH_CLK                   (LL_GPIO_PIN_4)      /* XQSPI flash CLK       */
#define FLASH_IO_0                  (LL_GPIO_PIN_7)      /* XQSPI flash IO0       */
#define FLASH_IO_1                  (LL_GPIO_PIN_6)      /* XQSPI flash IO1       */
#define FLASH_IO_2                  (LL_GPIO_PIN_5)      /* XQSPI flash IO2 (WP)  */
#define FLASH_IO_3                  (LL_GPIO_PIN_3)      /* XQSPI flash IO3 (HOLD)*/
#define HAL_EXFLASH_IO_PULL_SET(_PIN_, _PULL_)  ll_gpio_set_pin_pull(GPIO1, _PIN_, _PULL_)

#define REG_PL_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)
#define REG_PL_RD(addr)             (*(volatile uint32_t *)(addr))

#define SOFTWARE_REG1_ULTRA_DEEP_SLEEP_MAGIC 0xF175

volatile uint32_t g_app_msp_addr;   /* record app msp address */
uint8_t g_device_reset_reason = SYS_RESET_REASON_NONE;

#if (CFG_LCP_SUPPORT && (CHIP_TYPE <= 5))
extern uint16_t gdx_lcp_buf_init(uint32_t buf_addr);
static uint8_t lcp_buf[280] __attribute__((section (".ARM.__at_0x00820000"), zero_init));
#endif

#define SDK_VER_MAJOR                   2
#define SDK_VER_MINOR                   1
#define SDK_VER_BUILD                   0
#define COMMIT_ID                       0x02ee0ce7

static const sdk_version_t sdk_version = {SDK_VER_MAJOR,
                                          SDK_VER_MINOR,
                                          SDK_VER_BUILD,
                                          COMMIT_ID,};//sdk version

void sys_sdk_verison_get(sdk_version_t *p_version)
{
    memcpy(p_version, &sdk_version, sizeof(sdk_version_t));
}


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

static fun_t svc_user_func = NULL;

void svc_func_register(uint8_t svc_num, uint32_t user_func)
{
    svc_user_func = (fun_t)user_func;
}

void svc_user_handler(uint8_t svc_num)
{
    if (svc_user_func)
        svc_user_func();
}

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

    /* set sram power state. */
    mem_pwr_mgmt_mode_set(MEM_POWER_AUTO_MODE);

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

    /* platform init process. */
    platform_sdk_init();

#if ENCRYPT_ENABLE
    dfu_cmd_handler_replace_for_encrypt();
#endif

    system_pmu_deinit();
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);
    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);

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

    #if (CFG_LCP_SUPPORT && (CHIP_TYPE <= 5))
    gdx_lcp_buf_init((uint32_t)lcp_buf);
    #endif

    exflash_io_pull_config();

    #ifndef DRIVER_TEST
    /* Enable auto pmu calibration which period is 30s on default. */
    system_pmu_calibration_init(30000);
    #endif

    /* recover the default setting by temperature, should be called in the end */
    pmu_calibration_handler(NULL);
    /* Init peripheral sleep management */
    app_pwr_mgmt_init();

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

static void patch_init(void)
{
    gr5xx_fpb_init(FPB_MODE_PATCH_AND_DEBUG);

    #if ENCRYPT_ENABLE
    extern void encrypt_mode_patch_enable(void);
    encrypt_mode_patch_enable();
    #endif
}

/**
 ****************************************************************************************
 * @brief  Check whether the system wakes up from ultra deep sleep. If it wakes up from ultra
 *         deep sleep, reset the entire system. If not, do nothing.
 * @retval: void
 ****************************************************************************************
 */
static void ultra_deep_sleep_wakeup_handle(void)
{
    if (SOFTWARE_REG1_ULTRA_DEEP_SLEEP_MAGIC == (AON->SOFTWARE_1 & 0xFFFF))
    {
        // Reset the whole system
        hal_nvic_system_reset();
        while (true)
            ;
    }
}

void soc_init(void)
{
    ultra_deep_sleep_wakeup_handle();
    patch_init();

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

