/**
 *****************************************************************************************
 *
 * @file gui_animation_config.c
 *
 * @brief Users should implement the timer-related interface themselves
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

#include "gui_config.h"
#include "gui_animation.h"

#if ANIMATION_EN==1

#include "grx_sys.h"
#define ANIMATION_TIMER_INTERVAL 150
                                                                                  
#ifdef ENV_USE_RTOS
#include "FreeRTOS.h"
#include "timers.h"
static TimerHandle_t timer_handle = NULL;

#else
#include "app_timer.h"
static p_app_timer_id_t animation_timer_id;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gui_animation_timerout_handler(void *p_arg)
{
    gui_animation_timer_task();
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void gui_animation_timer_start(void)
{
    #ifndef ENV_USE_FREERTOS
    if (NULL != animation_timer_id)
    {
        gui_animation_overlap();

    }
    app_timer_create(animation_timer_id, ATIMER_REPEAT, gui_animation_timerout_handler);
    app_timer_start(*animation_timer_id, ANIMATION_TIMER_INTERVAL, NULL);
    #else
    if (NULL != timer_handle)
    {
        gui_animation_overlap();

    }
    timer_handle = xTimerCreate(NULL, (ANIMATION_TIMER_INTERVAL), pdTRUE, NULL, gui_animation_timerout_handler); 
    xTimerStart(timer_handle, 0);
    #endif
}

void gui_animation_timer_stop(void)
{
    #ifndef ENV_USE_FREERTOS
    app_timer_delete(animation_timer_id);
    #else
    xTimerDelete(timer_handle, 0);
    timer_handle = NULL;
    #endif
}

#endif

