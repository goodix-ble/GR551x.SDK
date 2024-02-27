/**
 *****************************************************************************************
 *
 * @file watcher.c
 *
 * @brief watcher function Implementation.
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
#include <stdio.h>
#include "timer.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint32_t timer0_count;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void timer0_callback(void)
{
    timer0_count++;
}

void vCfgforTimer()
{
    timer_config_t timer_config = TIMER_DEFAULT_CONFIG;
    timer_config.period         = (64000000U) / (20000) - 1;
    timer_config.callback       = timer0_callback;
    timer_init(TIMER_DEVICE_ID_1, &timer_config);
}

typedef struct ARRAY
{
    char mark;
    unsigned int _times_sample_;

} ARRAY_t;

ARRAY_t my_arrary[] =
{
    {'a', 0},
    {'b', 0},
    {'I', 0},
};

void create_arrary()
{

}

unsigned int global_times = 0;
void input_data_to_watcher(unsigned int times)
{
    global_times += times;
}

char RunTimeInfo[400];

void Goodix_Watcher_Task()
{
    float tmp  = 0.0f, tmp2, tmp3;
    while (1)
    {
        vTaskDelay(100);
#if CFG_WATCHER
        __disable_irq();
        tmp = my_arrary[0]._times_sample_ / (float)global_times;

        tmp2 = my_arrary[1]._times_sample_ / (float)global_times;

        tmp3 = my_arrary[2]._times_sample_ / (float)global_times;
        printf("%f %f %f\r\n", tmp, tmp2, tmp3);
        global_times = 0;
        my_arrary[0]._times_sample_ = my_arrary[1]._times_sample_ = my_arrary[2]._times_sample_ = 0;
        __enable_irq();
#endif
        memset(RunTimeInfo, 0, 400);
        vTaskGetRunTimeStats(RunTimeInfo);
        printf("%s\r\n", RunTimeInfo);
    }
}


