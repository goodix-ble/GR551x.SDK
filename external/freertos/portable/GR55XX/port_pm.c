/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"

/*
 * STRUCTURE DEFINITIONS
 *****************************************************************************************
 */
typedef struct
{
    uint32_t hs;
    uint32_t hus;
} ble_time_t;

/*
 * GLOBAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */
extern void ultra_wfi(void);
extern uint32_t get_remain_sleep_dur(void);
extern ble_time_t rwip_time_get(void);
extern uint16_t prevent_sleep_get(void);
extern bool ble_is_ready(void);
extern void warm_boot_second(void);

/*
 * OPTIONS FOR POWER OPTIMIZATION
 *****************************************************************************************
 */
// you can define WFI_16MHZ_ENABLE to save power if no system-clock sensive device is used.
// For example, no uart/spi/dma, etc
#define WFI_16MHZ_ENABLE             (0)

/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */
#define SYSTICK_LOAD_VALUE           ((configCPU_CLOCK_HZ / configTICK_RATE_HZ))
#define TICK_PERIOD_US               (1000000 / configTICK_RATE_HZ)
#define TICK_MS_IN_HUS               (TICK_PERIOD_US << 1)
#define HALF_SLOT_SIZE               (625)
#define RWIP_MAX_CLOCK_TIME          (0xFFFFFFF)
#define CLK_SUB(clock_a, clock_b)    ((uint32_t)(((clock_a) - (clock_b)) & RWIP_MAX_CLOCK_TIME))
#define RW_BB_FRAME_ONGOING          (0x0200)
#define WFI_16MHZ_CFG                ((CPLL_S16M_CLK << AON_PWR_REG01_SYS_CLK_SEL_Pos) | \
                                      (QSPI_16M_CLK << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos))
#define WFI_32MHZ_CFG                ((CPLL_T32M_CLK << AON_PWR_REG01_SYS_CLK_SEL_Pos) | \
                                      (QSPI_32M_CLK << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos))
#define U32(a)                       ((uint32_t)(a))
#define CO_MIN_U32(a,b)              (U32(a) < U32(b) ? U32(a) : U32(b))
#define MAX_BLE_SLEEP_TICKS          (100000) // limit for sys_lpcycles_2_hus overflow
#define TICK_MISS_COUNT_VAL          (2600)
#define LP_CLOCK_PERIOD_HUS          (60)
#define TICK_ERR_CALI_STEP           (LP_CLOCK_PERIOD_HUS)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint32_t s_tick_stop_lp_cnt = 0;
static uint32_t s_tick_reload_lp_cnt = 0;
static uint32_t s_lp_cnt_2_hus_err = 0;
static ble_time_t s_tick_stop_time = {0, 0};
static ble_time_t s_tick_reload_time = {0, 0};
static uint32_t s_rtos_idle_ticks = 0;
static int s_tick_err_us = 0;
static int s_ble_compensate_flag = 0;
static ble_time_t s_last_tick_time = {0, 0};
static uint32_t s_last_tick_cnt = 0;

/*
 * Global VARIABLE DEFINITIONS
 *****************************************************************************************
 */
int g_compensate_ticks = 0;
uint32_t g_miss_count_hus = TICK_MISS_COUNT_VAL;

/**
  * @brief Function Implementations
  */
TINY_RAM_SECTION uint32_t vPortLocker(void)
{
    uint32_t ret_pri = __get_PRIMASK();
    __set_PRIMASK(1);
    return ret_pri;
}

TINY_RAM_SECTION void vPortUnLocker(uint32_t set_pri)
{
    __set_PRIMASK(set_pri);
}

TINY_RAM_SECTION void ble_compensate_calculate(uint32_t xEstimateCnt)
{
    static ble_time_t s_cur_tick_time = {0, 0};
    static uint32_t s_cur_tick_cnt = 0;
    static uint32_t s_ble_compensate_cnt = 0;
    if(!ble_is_ready())
    {
        return;
    }
    s_cur_tick_time = rwip_time_get();
    s_cur_tick_cnt = xEstimateCnt;
    if ((0 == s_ble_compensate_cnt++) && (0 == g_compensate_ticks))
    {
        /* For ble time and systick pairs initialize */
        s_tick_err_us = 0;
        s_last_tick_time = s_cur_tick_time;
        s_last_tick_cnt = s_cur_tick_cnt;
        return;
    }
    int tick_diff_cnt = (s_cur_tick_cnt - s_last_tick_cnt);
    int ble_time_diff_hus = HALF_SLOT_SIZE * CLK_SUB(s_cur_tick_time.hs, s_last_tick_time.hs);
    int ble_time_diff_us = (ble_time_diff_hus + s_cur_tick_time.hus - s_last_tick_time.hus) >> 1;
    int systick_diff_us = tick_diff_cnt * TICK_PERIOD_US;
    if( ((int)ble_time_diff_us) > ((int)systick_diff_us) )
    {
        /* Systick is slower than real time */
        s_tick_err_us += (ble_time_diff_us - systick_diff_us);
    }
    else
    {
        /* Systick is faster than real time */
        s_tick_err_us -= (systick_diff_us - ble_time_diff_us);
    }
    s_ble_compensate_flag = 1;
}

TINY_RAM_SECTION void ble_compensate_execute(uint32_t* step_ticks)
{
    while( ((int)s_tick_err_us) > ((int)TICK_PERIOD_US) )
    {
        if( ((int)((*step_ticks) + 1)) <= ((int)s_rtos_idle_ticks) )
        {
            *step_ticks = *step_ticks + 1;
            s_tick_err_us = s_tick_err_us - TICK_PERIOD_US;
            g_compensate_ticks++;
            g_miss_count_hus += TICK_ERR_CALI_STEP;
        }
        else
        {
            //if error happend, reset s_tick_err_us
            s_tick_err_us = 0;
            break;
        }
    }
    while( ((int)s_tick_err_us) < ((int)(0 - TICK_PERIOD_US)) )
    {
        if( ((int)((*step_ticks) - 1)) > ((int)1) )
        {
            *step_ticks = *step_ticks - 1;
            s_tick_err_us = s_tick_err_us + TICK_PERIOD_US;
            g_compensate_ticks--;
            g_miss_count_hus = (g_miss_count_hus > TICK_ERR_CALI_STEP) ? \
            (g_miss_count_hus - TICK_ERR_CALI_STEP) : (g_miss_count_hus);
        }
        else
        {
            //if error happend, reset s_tick_err_us
            s_tick_err_us = 0;
            break;
        }
    }
}

TINY_RAM_SECTION void SysTickReload(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->LOAD = (uint32_t)(SYSTICK_LOAD_VALUE - 1UL); /* set reload register */
    SysTick->VAL = 0UL;                                   /* Load the SysTick Counter Value */
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);
}

TINY_RAM_SECTION void pwr_mgmt_tick_compensate(uint32_t step_ticks)
{
    if( 0 == step_ticks )
    {
        return;
    }
    ble_compensate_calculate(xTaskGetTickCount() + step_ticks);
    ble_compensate_execute(&step_ticks);
    step_ticks = CO_MIN_U32(s_rtos_idle_ticks, step_ticks);
    vTaskStepTick(step_ticks);
    //after ble compensate the tick, record the systick and ble pair
    if(s_ble_compensate_flag == 1)
    {
        s_ble_compensate_flag = 0;
        s_last_tick_time = rwip_time_get();
        s_last_tick_cnt = xTaskGetTickCount();
    }
}

TINY_RAM_SECTION void SysFreqSet(uint32_t clock_cfg)
{
    uint32_t temp = AON->PWR_RET01 & (~(AON_PWR_REG01_SYS_CLK_SEL | AON_PWR_REG01_XF_SCK_CLK_SEL));

    /* When switch 16M or 64M clock to another clock, clock needs to be switched to 32M first. */
    AON->PWR_RET01 = (temp | WFI_32MHZ_CFG);
    __asm ("nop;nop;nop;nop;");

    AON->PWR_RET01 = (temp | clock_cfg);
}

TINY_RAM_SECTION uint32_t SysFreqGet(void)
{
    return ((AON->PWR_RET01 & (AON_PWR_REG01_SYS_CLK_SEL | AON_PWR_REG01_XF_SCK_CLK_SEL)));
}
bool is_ble_allow_tickless(void);
TINY_RAM_SECTION void pwr_mgmt_ultra_wfi(void)
{
    uint32_t pwr_locker = vPortLocker();

    if((is_ble_allow_tickless()) && (prevent_sleep_get() & RW_BB_FRAME_ONGOING))
    {
        /* Stop systick and record tick stop time */
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        s_tick_stop_time = rwip_time_get();

        #if WFI_16MHZ_ENABLE
        /* Backup and lower the system frequency */
        uint32_t cur_clock_cfg = SysFreqGet();
        SysFreqSet(WFI_16MHZ_CFG);
        #endif

        ultra_wfi();

        #if WFI_16MHZ_ENABLE
        /* Restore the system frequency */
        SysFreqSet(cur_clock_cfg);
        #endif

        /* Restart SysTick and record tick reload time. */
        SysTickReload();
        s_tick_reload_time = rwip_time_get();

        /* Calculate step ticks with ble time. */
        uint32_t time_diff_hus = HALF_SLOT_SIZE * CLK_SUB(s_tick_reload_time.hs, s_tick_stop_time.hs);
        time_diff_hus = time_diff_hus + s_tick_reload_time.hus - s_tick_stop_time.hus;
        uint32_t step_ticks = time_diff_hus / TICK_MS_IN_HUS;

        /* Compensate Systick Increment. */
        pwr_mgmt_tick_compensate(step_ticks);
    }
    else
    {
        ultra_wfi();
    }

    vPortUnLocker(pwr_locker);
}

TINY_RAM_SECTION void pwr_mgmt_sleep_dur_limit(int rtos_idle_ticks)
{
    uint32_t expect_idle_ticks = CO_MIN_U32(rtos_idle_ticks, MAX_BLE_SLEEP_TICKS);
    /* The rtos_idle_ticks >= 5ms and rtos_idle_ticks <= 100second is ensured in this context. */
    uint32_t rtos_idle_hus = (expect_idle_ticks - 1) * TICK_MS_IN_HUS;
    if( get_remain_sleep_dur() > rtos_idle_hus)
    {
        pwr_mgmt_ble_wakeup();
    }
    sys_ble_heartbeat_period_set(rtos_idle_hus);
}

TINY_RAM_SECTION void pwr_mgmt_enter_sleep_with_cond(int rtos_idle_ticks)
{
    /* Limit maximum sleep duration. */
    pwr_mgmt_sleep_dur_limit(rtos_idle_ticks);

    /* To disbale global IRQ. */
    uint32_t pwr_locker = vPortLocker();

    /* If a context switch is pending or a task is waiting for the scheduler
     * to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Check Whether Device is busy. */
    if (DEVICE_BUSY == pwr_mgmt_dev_suspend())
    {
        pwr_mgmt_ultra_wfi();
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Before the lock, if the sleep mode is changed in a interrupt, take it effect immediately. */
    if (PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get())
    {
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Check BLE Sleep Status. Notice: ensure there is no other checks before baseband state check. */
    pwr_mgmt_mode_t ble_state = pwr_mgmt_baseband_state_get();
    if (PMR_MGMT_SLEEP_MODE != ble_state)
    {
        if (PMR_MGMT_IDLE_MODE == ble_state)
        {
            pwr_mgmt_ultra_wfi();
        }
        vPortUnLocker(pwr_locker);
        return;
    }

    /* Record Systick Stop Time. */
    s_tick_stop_lp_cnt = ll_pwr_get_comm_sleep_duration();

    /* Save the context of RTOS. */
    pwr_mgmt_save_context();

    /* Judge the current startup mode mark. */
    if (pwr_mgmt_get_wakeup_flag() == COLD_BOOT)
    {
        /* Shutdown all system power and wait some event to wake up. */
        if (PMR_MGMT_IDLE_MODE == pwr_mgmt_shutdown())
        {
            pwr_mgmt_ultra_wfi();
            vPortUnLocker(pwr_locker);
            return;
        }
        vPortUnLocker(pwr_locker);
    }
    else // Wakeup from deep sleep state
    {
        /* Clear wakeup mark, prepare next enter-sleep action. */
        pwr_mgmt_set_wakeup_flag(COLD_BOOT);

        /* To disbale global IRQ. */
        uint32_t _local_lock = vPortLocker();

        /* Reload SysTick and Record Tick Reload Time. */
        SysTickReload();
        s_tick_reload_lp_cnt = ll_pwr_get_comm_sleep_duration();

        /* Calculate Step Ticks With Low Power Counter. */
        uint32_t sleep_lp_cnt = s_tick_reload_lp_cnt - s_tick_stop_lp_cnt;
        uint32_t ticks_hus = sys_lpcycles_2_hus(sleep_lp_cnt, &s_lp_cnt_2_hus_err) + g_miss_count_hus;
        uint32_t step_ticks = ticks_hus / TICK_MS_IN_HUS;

        /* Compensate Systick Increment. */
        pwr_mgmt_tick_compensate(step_ticks);

        /* To enable global IRQ. */
        vPortUnLocker(_local_lock);

        warm_boot_second();
    }
}

TINY_RAM_SECTION void vPortEnterDeepSleep(TickType_t xExpectedIdleTime)
{
    s_rtos_idle_ticks = xExpectedIdleTime;

    if (xExpectedIdleTime < 5)
    {
        ultra_wfi();
        return;
    }

    if (PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get())
    {
        ultra_wfi();
        return;
    }

    pwr_mgmt_enter_sleep_with_cond(xExpectedIdleTime);
}
