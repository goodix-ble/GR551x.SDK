/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include <string.h>
#include "app_log.h"
#include "grx_hal.h"
#include "app_bod.h"
#include "board_SK.h"
#include "grx_sys.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
app_bod_params_t s_params;
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_bod_evt_handler(app_bod_evt_t * p_evt)
{
    if (p_evt->type == APP_BOD_EVT_TRIGGERED)
    {
        printf("BOD TRIGGERED EVENT\r\n");
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    if (p_evt->type == APP_BOD_EVT_REMOVED)
    {
        printf("BOD REMOVED EVENT\r\n");
    }
#endif
}

void app_bod(void)
{
    s_params.init.bod_en = APP_BOD_ENABLE;
    s_params.init.bod2_en = APP_BOD_EVENT_ENABLE;
    s_params.init.bod2_lvl = APP_BOD_EVENT_LEVEL_3;
    s_params.init.bod_static_en = APP_BOD_STATIC_ENABLE;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    s_params.init.bod2_auto_power_bypass_en = APP_BOD_EVENT_AUTO_POWER_BYPASS_ENABLE;
#endif
    app_bod_init(&s_params, app_bod_evt_handler);
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 BOD APP example.                   *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will show BOD TRIGGERED and REMOVED    *\r\n");
    printf("* interrupt. The voltage of VBAT needs to be lower   *\r\n");
    printf("* or higher than the voltage of the LEVEL_10 gea     *\r\n");
    printf("*                                                    *\r\n");
    printf("******************************************************\r\n");

    app_bod();

    while(1);
}
