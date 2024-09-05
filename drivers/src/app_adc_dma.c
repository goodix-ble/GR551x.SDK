/**
  ****************************************************************************************
  * @file    app_adc_dma.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
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
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <string.h>

#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "grx_sys.h"
#include "app_adc.h"
#include "app_adc_dma.h"

#ifdef HAL_ADC_MODULE_ENABLED

#ifdef APP_DRIVER_WAKEUP_CALL_FUN
extern void adc_wake_up(void);
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
extern const uint32_t s_io_to_input_src[ADC_INPUT_SRC_REF + 1];
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern adc_env_t *p_adc_env;

static uint16_t adc_config_dma(app_adc_params_t *p_params)
{
    app_dma_params_t dma_params = { 0 };

    dma_params.p_instance                = p_params->dma_cfg.dma_instance;
    if (dma_params.p_instance != DMA0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    dma_params.channel_number            = p_params->dma_cfg.dma_channel;
    dma_params.init.src_request          = DMA0_REQUEST_SNSADC;
    dma_params.init.direction            = DMA_PERIPH_TO_MEMORY;
    dma_params.init.src_increment        = DMA_SRC_NO_CHANGE;
    dma_params.init.dst_increment        = DMA_DST_INCREMENT;
    dma_params.init.src_data_alignment   = DMA_SDATAALIGN_WORD;
    dma_params.init.dst_data_alignment   = DMA_DDATAALIGN_WORD;
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    dma_params.init.mode                 = DMA_NORMAL;
#endif
    dma_params.init.priority             = DMA_PRIORITY_LOW;

    p_adc_env->dma_id = app_dma_init(&dma_params, NULL);
    if (p_adc_env->dma_id < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    p_adc_env->handle.p_dma = app_dma_get_handle(p_adc_env->dma_id);
    p_adc_env->handle.p_dma->p_parent = (void*)&p_adc_env->handle;

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_adc_dma_init(app_adc_params_t *p_params)
{
    app_drv_err_t app_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if ((p_adc_env == NULL) || (p_adc_env->adc_state == APP_ADC_INVALID))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    if (p_adc_env->adc_dma_state != APP_ADC_DMA_INVALID)
    {
        return APP_DRV_ERR_INVALID_INIT;
    }

    GLOBAL_EXCEPTION_DISABLE();
    app_err_code = adc_config_dma(p_params);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        goto __exit;
    }
    p_adc_env->adc_dma_state = APP_ADC_DMA_ACTIVITY;
__exit:
    GLOBAL_EXCEPTION_ENABLE();

    return app_err_code;
}

uint16_t app_adc_dma_deinit(void)
{
    if ((p_adc_env == NULL) || (p_adc_env->adc_dma_state != APP_ADC_DMA_ACTIVITY))
    {
        return APP_DRV_ERR_NOT_INIT;
    }

    app_dma_deinit(p_adc_env->dma_id);

    p_adc_env->adc_dma_state = APP_ADC_DMA_INVALID;

    return APP_DRV_SUCCESS;
}

#endif

