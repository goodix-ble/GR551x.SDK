#include "user_app.h"
#include "user_periph_setup.h"
#include "grx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "custom_config.h"
#include "app_log.h"
#include "grx_hal.h"
#include "user_config.h"

#if (USE_FREERTOS == 0)
#if GR5526_SK_LOAD

#define PRINT_CHECK_TIME  0
#define TIMER_DIV_CONTANT 0.00000025
static timer_handle_t        g_TIM1Handle             = {0};
static dual_timer_handle_t   g_DUALTIM0Handle         = {0};
static uint32_t              g_total_ble_sdk_irq_time = 0;
volatile uint8_t             g_int_nested             = 0;

extern void BLE_SDK_Handler_func(void);
extern void BLE_IRQHandler_func(void);

#pragma arm section code = "RAM_CODE"

uint32_t dual_timer_read_value(void)
{
    return ll_dual_timer_get_counter(g_DUALTIM0Handle.p_instance);
}

void stamp_check_timer(void)
{
    g_DUALTIM0Handle.p_instance = DUAL_TIMER0;
    g_DUALTIM0Handle.init.prescaler = DUAL_TIMER_PRESCALER_DIV16;
    g_DUALTIM0Handle.init.counter_mode = DUAL_TIMER_COUNTERMODE_LOOP;
    g_DUALTIM0Handle.init.auto_reload = (unsigned int)-1;
    hal_dual_timer_base_init(&g_DUALTIM0Handle);
    hal_dual_timer_base_start_it(&g_DUALTIM0Handle);
}


#ifndef GR551xx_C4
#pragma arm section code = "RAM_CODE"

void $Sub$$BLE_IRQHandler(void)
{

    uint32_t before = 0, after = 0;
    before = dual_timer_read_value();
    BLE_IRQHandler_func();
    after = dual_timer_read_value();

    if (0 == g_int_nested)
    {
        g_total_ble_sdk_irq_time += (before-after);
    }
}
#pragma arm section
#endif


void hal_timer_period_elapsed_callback(timer_handle_t *htim)
{
    __disable_irq();
#if PRINT_CHECK_TIME
    static uint32_t last_stamp = 0;
    uint32_t curr_stamp = dual_timer_read_value();
    double total_ran_time = (double)( last_stamp - curr_stamp ) * TIMER_DIV_CONTANT;
    last_stamp = curr_stamp;
#else
    double total_ran_time = 1.0;
#endif
    double ble_cpu_load = g_total_ble_sdk_irq_time * TIMER_DIV_CONTANT;
    g_total_ble_sdk_irq_time = 0;
    __enable_irq();
    printf("BLE-CPU:%fs, CHECK PERIOD:%fs\r\n", ble_cpu_load, total_ran_time);
}

void TIMER1_IRQHandler(void)
{
    hal_timer_irq_handler(&g_TIM1Handle);
}

void period_check_timer(uint32_t unit_time)
{
    timer_handle_t* handler = &g_TIM1Handle;
    handler->p_instance = TIMER1;
    handler->init.auto_reload = (64000000/unit_time) - 1;
    hal_timer_base_stop_it(handler);
    hal_timer_base_deinit(handler);
    hal_timer_base_init(handler);
    hal_timer_base_start_it(handler);
    NVIC_ClearPendingIRQ(TIMER1_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn);
}


void user_test_cpu_load(void)
{
    period_check_timer(1);
    stamp_check_timer();
}

#pragma arm section
#endif
#endif
