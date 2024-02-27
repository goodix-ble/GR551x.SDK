/**
 *****************************************************************************************
 *
 * @file pmu_calibration.c
 *
 * @brief auto calibration function Implementation.
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
#include "pmu_calibration.h"
#include "platform_sdk.h"
#include "gr55xx_sys.h"
#include "app_timer.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t s_pmu_calibration_timer_id = {0};

extern uint32_t g_debug_temperature;
extern uint32_t g_debug_lpclk;
static uint32_t s_pre_temperature = 0;

#if CFG_LPCLK_INTERNAL_EN
#define LFRC32K_FAST_INTERVAL_ALLOW_PPM           (100)
#define LFRC32K_MIDDLE_INTERVAL_ALLOW_PPM         (300)
#define LFRC32K_SLOW_INTERVAL_ALLOW_PPM           (300)
#define LFRC32K_CHECK_STATUS_CNT                  (3)
#define LFRC32K_ALLOW_PPM_WITHIN_THE_RANGE_CNT    (20)
#define LFRC32K_ALLOW_PPM_WITHIN_MID_THE_RANGE_CNT (LFRC32K_ALLOW_PPM_WITHIN_THE_RANGE_CNT*100)
#define PMU_SMALL_INTERVAL_MS       (10*1000)
static uint32_t pmu_interval_init = 30 * 1000;
static uint32_t pmu_interval_prev = 0;
uint32_t lfrc32k_fast_check_interval = 500;
uint32_t lfrc32k_middle_check_interval = 500*10;
const uint32_t lfrc32k_check_interval_table[LFRC32K_CHECK_STATUS_CNT]={500,500*10,30*1000}; //500ms,    5s,         30s
const uint32_t lfrc32k_check_cnt[LFRC32K_CHECK_STATUS_CNT]={20,20*100,0xFFFFFFFF};          //20 times, 2000 times, forever
const uint16_t lfrc32k_allow_ppm[LFRC32K_CHECK_STATUS_CNT]={100,300,300};                   // 100ppm , 300ppm,   , 300ppm

uint32_t pmu_interval_get(uint32_t is_init)
{
    uint32_t interval = 0;

    if (g_debug_temperature > 44)
    {
        interval = PMU_SMALL_INTERVAL_MS;
    }
    else if (g_debug_temperature >= 40 && g_debug_temperature <= 44 && is_init)
    {
        interval = PMU_SMALL_INTERVAL_MS;
    }
    else if (g_debug_temperature < 40)
    {
        interval = pmu_interval_init;
    }
    
    return interval;
}


uint32_t lfrc32k_interval_get(uint32_t use_interval,uint16_t ppm,uint32_t original_interval)
{
    static uint32_t fast_check_index = 0;
    uint8_t current_check_status=0xFF;
    fast_check_index++;
    uint32_t new_use_interval = original_interval;

    for(uint8_t i=0;i<LFRC32K_CHECK_STATUS_CNT;i++)
    {
        if(use_interval == lfrc32k_check_interval_table[i])
        {
            current_check_status =i;
            break;
        }
    }
    if(current_check_status == 0 || current_check_status == 1)
    {
        if(ppm > lfrc32k_allow_ppm[current_check_status])
        {
            new_use_interval = lfrc32k_fast_check_interval;
            fast_check_index = 0;
        }
        else
        {
            if(fast_check_index < lfrc32k_check_cnt[current_check_status])
            {
                new_use_interval = lfrc32k_check_interval_table[current_check_status];
            }
        }
    }
    else
    {
        if(ppm > lfrc32k_allow_ppm[2])
        {
            new_use_interval = lfrc32k_check_interval_table[1];
            fast_check_index = 0;
        }
    }
    return new_use_interval;
}

uint32_t get_lfrc32k_calibration_interval(uint32_t original_interval)
{
    static uint32_t prev_debug_lpclk=0;
    static uint32_t use_interval=0;
    if(prev_debug_lpclk == 0)
    {
        prev_debug_lpclk = g_debug_lpclk;
        use_interval = lfrc32k_fast_check_interval;
        return use_interval;
    }
    else
    {
        uint16_t ppm=0;
        if(prev_debug_lpclk >= g_debug_lpclk)
        {
            ppm = ((prev_debug_lpclk - g_debug_lpclk)*100*10000)/prev_debug_lpclk;
        }
        else
        {
            ppm = ((g_debug_lpclk - prev_debug_lpclk)*100*10000)/g_debug_lpclk;
        }

        prev_debug_lpclk = g_debug_lpclk;
        use_interval = lfrc32k_interval_get(use_interval,ppm,original_interval);
        return use_interval;
    }
}

void pmu_timer_handler(void* p_arg)
{
    static uint32_t pmu_calbration_mod=1;
    static uint32_t pmu_calbration_index=0;
    //is timer interval is 500ms,make sure 30s interval to do pmu calibration
    if(pmu_calbration_index%pmu_calbration_mod == 0)
    {
        pmu_calibration_handler(p_arg);
        if (s_pre_temperature != g_debug_temperature)
        {
            s_pre_temperature = g_debug_temperature;
            rng_calibration();
        }
    }
    pmu_calbration_index++;
    lfrc32k_calibration();

    uint32_t interval_plan;
    uint32_t interval_new;
    uint32_t interval_diff;

    interval_plan = pmu_interval_get(0);

    if (interval_plan == 0)
    {
        interval_new = get_lfrc32k_calibration_interval(interval_plan);
        return;
    }
    interval_new = get_lfrc32k_calibration_interval(interval_plan);
    pmu_calbration_mod = interval_plan/ interval_new;

    interval_diff = interval_new > pmu_interval_prev ?
                              interval_new - pmu_interval_prev: pmu_interval_prev - interval_new;

    if (interval_diff > 2000)
    {
        app_timer_delete(&s_pmu_calibration_timer_id);
        app_timer_create(&s_pmu_calibration_timer_id, ATIMER_REPEAT, pmu_timer_handler);
        app_timer_start(s_pmu_calibration_timer_id, interval_new, NULL);
    }
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void pmu_calibration(void* p_arg)
{
    pmu_calibration_handler(p_arg);
    if (s_pre_temperature != g_debug_temperature)
    {
        s_pre_temperature = g_debug_temperature;
        rng_calibration();
    }
}

void system_pmu_calibration_init(uint32_t interval)
{
    if (interval)
    {
        uint32_t interval_new;
#if CFG_LPCLK_INTERNAL_EN
        pmu_interval_init = interval;
        interval_new = pmu_interval_get(1);
#else
        interval_new = interval;
#endif

        app_timer_delete(&s_pmu_calibration_timer_id);
        app_timer_create(&s_pmu_calibration_timer_id, ATIMER_REPEAT, 
#if CFG_LPCLK_INTERNAL_EN
                         pmu_timer_handler
#else
                         pmu_calibration
#endif  //CFG_LPCLK_INTERNAL_EN
                         );
        app_timer_start(s_pmu_calibration_timer_id, interval_new, NULL);

#if CFG_LPCLK_INTERNAL_EN
        pmu_interval_prev = interval_new;
#endif
    }
    return;
}


void system_pmu_calibration_stop(void)
{
    app_timer_delete(&s_pmu_calibration_timer_id);
    return;
}
