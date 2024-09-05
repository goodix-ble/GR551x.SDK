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
#include "app_adc.h"
#include "app_adc_dma.h"
#include "app_io.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
// Note: Note: TEST_CONV_LENGTH must be aligned on a four-byte boundary.
#define TEST_CONV_LENGTH      (128UL)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint16_t covn_done = 0;
uint16_t conversion[TEST_CONV_LENGTH];
double   voltage[TEST_CONV_LENGTH];
app_adc_params_t adc_params = {
    .pin_cfg = {
        .channel_p = {
            .type = APP_IO_TYPE_MSIO,
            .mux  = APP_ADC_P_INPUT_PIN_MUX,
            .pin  = APP_ADC_P_INPUT_PIN,
        },
        .channel_n = {
            .type = APP_IO_TYPE_MSIO,
            .mux  = APP_ADC_N_INPUT_PIN_MUX,
            .pin  = APP_ADC_N_INPUT_PIN,
        },
    },
    .dma_cfg = {
        .dma_instance = DMA0,
        .dma_channel  = DMA_Channel0,
    },
    .init = {
        .channel_p  = APP_ADC_P_INPUT_SRC,
        .channel_n  = APP_ADC_N_INPUT_SRC,
        .input_mode = ADC_INPUT_SINGLE,
        .ref_source = ADC_REF_SRC_BUF_INT,
        .ref_value  = ADC_REF_VALUE_1P6,
        .clock      = ADC_CLK_1M,
    },
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_adc_evt_handler(app_adc_evt_t * p_evt)
{
    if (p_evt->type == APP_ADC_EVT_CONV_CPLT)
    {
        covn_done = 1;
    }
}

void adc_single(void)
{
    uint16_t ret = APP_DRV_SUCCESS;

    /* Please initialize DMA in the following order. */
    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_adc_init(&adc_params, app_adc_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nADC initial failed! Please check the input parameters.\r\n");
        return;
    }
    ret = app_adc_dma_init(&adc_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nADC initial dma failed! Please check the input parameters.\r\n");
        return;
    }

    memset(conversion, 0, sizeof(conversion));

    printf("Start single sampling.\r\n");
    covn_done = 0;
    app_adc_dma_conversion_async(conversion, TEST_CONV_LENGTH);
    while(covn_done == 0);

    app_adc_voltage_intern(conversion, voltage, TEST_CONV_LENGTH);
    printf("Conversion value:\r\n");
    for(uint32_t i = 0; i < TEST_CONV_LENGTH; i++)
    {
        printf("%0.3fV  ", voltage[i]);
        app_log_flush();
    }
    printf("\r\n");

    /* Please deinitialize DMA in the following order. */
    app_adc_dma_deinit();
    app_adc_deinit();
}

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
void temperater_measure(void)
{
    double temp;
    uint16_t ret = APP_DRV_SUCCESS;

    adc_params.init.channel_n = ADC_INPUT_SRC_TMP;
    adc_params.init.ref_value = ADC_REF_VALUE_0P8;
    /* Please initialize DMA in the following order. */
    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_adc_init(&adc_params, app_adc_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nADC initial failed! Please check the input parameters.\r\n");
        return;
    }
    ret = app_adc_dma_init(&adc_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nADC initial dma failed! Please check the input parameters.\r\n");
        return;
    }

    memset(conversion, 0, sizeof(conversion));
    printf("\r\nStart temperature sampling.\r\n");
    covn_done = 0;
    app_adc_dma_conversion_async(conversion, TEST_CONV_LENGTH);
    while(covn_done == 0);

    uint32_t aver = 0;
    for(uint32_t i = 0; i<TEST_CONV_LENGTH; i++)
    {
        aver = aver + conversion[i];
    }
    aver = aver/TEST_CONV_LENGTH;
    printf("Average Temperature cocdes = %d\r\n", aver);

    app_adc_temperature_conv((uint16_t *)&aver, &temp, 1);
    printf("Temperature value= %0.1f C\r\n", temp);
    app_log_flush();

    /* Please deinitialize DMA in the following order. */
    app_adc_dma_deinit();
    app_adc_deinit();
}

void vbattery_measure(void)
{
    double vbat;
    uint16_t ret = APP_DRV_SUCCESS;

    adc_params.init.channel_n = ADC_INPUT_SRC_BAT;
    adc_params.init.ref_value = ADC_REF_VALUE_0P8;
    /* Please initialize DMA in the following order. */
    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_adc_init(&adc_params, app_adc_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nADC initial failed! Please check the input parameters.\r\n");
        return;
    }
    ret = app_adc_dma_init(&adc_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nADC initial dma failed! Please check the input parameters.\r\n");
        return;
    }

    memset(conversion, 0, sizeof(conversion));
    printf("\r\nStart Vbattery sampling.\r\n");
    covn_done = 0;
    app_adc_dma_conversion_async(conversion, TEST_CONV_LENGTH);
    while(covn_done == 0);

    uint32_t aver = 0;
    for(uint32_t i = 0; i<TEST_CONV_LENGTH; i++)
    {
        aver = aver + conversion[i];
    }
    aver = aver/(TEST_CONV_LENGTH);
    printf("Average Vbattery cocdes = %d\r\n", aver);

    app_adc_vbat_conv((uint16_t *)&aver, &vbat, 1);
    printf("Vbattery= %0.3f V\r\n", vbat);
    app_log_flush();

    /* Please deinitialize DMA in the following order. */
    app_adc_dma_deinit();
    app_adc_deinit();
}
#endif

int main(void)
{
    board_init();

    printf("\r\n");
    printf("*********************************************************\r\n");
    printf("*                 ADC APP example.                      *\r\n");
    printf("*                                                       *\r\n");
    printf("*                                                       *\r\n");
    printf("* Please connect signal to ADC_INPUT_P and ADC_INPUT_N. *\r\n");
    printf("* This sample will show the ADC sample signal from      *\r\n");
    printf("* INPUT_P & INPUT_N.                                    *\r\n");
    printf("*********************************************************\r\n");

    adc_single();

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
    temperater_measure();
    vbattery_measure();
#endif

    printf("\r\nThis example demo end.\r\n");

    while(1);
}
