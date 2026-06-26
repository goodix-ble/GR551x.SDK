/**
 ****************************************************************************************
 *
 * @file gr551x_temp_api.c
 *
 * @brief GR551x temperature module.
 *
 ****************************************************************************************
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
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "gr551x_temp_api.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static adc_handle_t gr551x_temp_handle = {0};
static double adc_temp = 0;
static double adc_slope = 0;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_gr551x_temp_init(void)
{
    adc_trim_info_t adc_trim = {0};

    gr551x_temp_handle.init.channel_n  = ADC_INPUT_SRC_TMP;
    gr551x_temp_handle.init.channel_p  = ADC_INPUT_SRC_TMP;
    gr551x_temp_handle.init.input_mode = ADC_INPUT_SINGLE;
    gr551x_temp_handle.init.ref_source = ADC_REF_SRC_BUF_INT;
    gr551x_temp_handle.init.ref_value  = ADC_REF_VALUE_0P8;
    gr551x_temp_handle.init.clock      = ADC_CLK_1P6M;
    hal_adc_init(&gr551x_temp_handle);

    if(SDK_SUCCESS == sys_adc_trim_get(&adc_trim))
    {
        adc_temp = (double)adc_trim.adc_temp;
        adc_slope = (-1) * (double)adc_trim.slope_int_0p8;
    }
    else
    {
        adc_temp = 4979;
        adc_slope = -4932;
    }
    return;
}

double hal_gr551x_temp_read(void)
{
    uint16_t conver_buff[16] = {0};
    uint16_t average = 0;

    /* Got the average of Temp */
    hal_adc_poll_for_conversion(&gr551x_temp_handle, conver_buff, 16);
    for(uint8_t i = 0; i < 8; i++)
    {
        average += conver_buff[8 + i];
    }
    average = average >> 3;
    return (((double)average - adc_temp) / adc_slope) / (-0.00175) + 25.0;
}
