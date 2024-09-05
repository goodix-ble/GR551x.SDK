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
#include "app_io.h"
#include "app_comp.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
app_comp_params_t params = {
    .pin_cfg = {
        .input = {
            .type = APP_IO_TYPE_MSIO,
            .mux  = APP_COMP_INPUT_PIN_MUX,
            .pin  = APP_COMP_INPUT_PIN,
        },
        .vref = {
            .type = APP_IO_TYPE_MSIO,
            .mux  = APP_COMP_VREF_PIN_MUX,
            .pin  = APP_COMP_VREF_PIN,
        },
    },
    .init = {
        .input_source = APP_COMP_INPUT_SRC,
        .ref_source   = COMP_REF_SRC_VREF,
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        .ref_value    = 30,
#else
        .ref_value    = 120,
#endif
    },
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
void app_comp_event_handler(app_comp_evt_t *p_evt)
{
    if (*p_evt == APP_COMP_EVT_DONE)
    {
        printf("Comp is triggered.\r\n");
    }
}
#endif
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
void app_comp_event_handler(app_comp_evt_t *p_evt)
{
    if (*p_evt == APP_COMP_EVT_RISING)
    {
        printf("Comp is rising triggered.\r\n");
    }
    else if(*p_evt == APP_COMP_EVT_FALLING)
    {
        printf("Comp is falling triggered.\r\n");
    }
}
#endif

void comp_interrupt(void)
{
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    params.init.edge = COMP_WAKEUP_EDGE_BOTH;
#endif
    app_comp_init(&params, app_comp_event_handler);
    app_comp_start();
}

int main(void)
{
    board_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   COMP example.                     *\r\n");
    printf("*                                                     *\r\n");
    printf("*       COMP_INPUT   <-----     MSIOAN                *\r\n");
    printf("*       COMP_VREF    <-----     VREF                  *\r\n");
    printf("*                                                     *\r\n");
    printf("* Please connect signal to MSIOAN.                    *\r\n");
    printf("* If the Input single is higher than the Reference,   *\r\n");
    printf("* the comparator interrupt will be triggered.         *\r\n");
    printf("* GR5526 Reference = 7.5mv * ref_value.               *\r\n");
    printf("* GR533x Reference = 50.82nA * (166K * ref_value).    *\r\n");
    printf("* GR551x Reference = 30mv * ref_value.                *\r\n");
    printf("*******************************************************\r\n");
    printf("\r\n");

    delay_ms(1000);

    comp_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
