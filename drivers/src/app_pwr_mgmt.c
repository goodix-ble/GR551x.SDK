/**
  ****************************************************************************************
  * @file    app_pwr_mgmt.c
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
#include "app_pwr_mgmt.h"
#include "grx_hal.h"
#include "grx_sys.h"

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
struct pwr_env_t
{
    app_sleep_callbacks_t *pwr_sleep_cb[APP_SLEEP_CB_MAX];
    wakeup_priority_t wakeup_priority[APP_SLEEP_CB_MAX];
    bool is_pwr_callback_reg;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
struct pwr_env_t s_pwr_env;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
pwr_mgmt_dev_state_t pwr_enter_sleep_check_new(void)
{
    uint32_t i;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    if (g_devices_state == 0x0)
#else
    if((g_devices_state == 0x0) && (g_extra_devices_state == 0x0))
#endif
    {
        if (g_devices_renew > 0)
        {
            for (i = 0x0; i < MAX_PERIPH_DEVICE_NUM; i++)
            {
                if (devices_suspend_cb[i] == NULL)
                    continue;

                if (devices_handle[i] == NULL)
                    continue;

                if (g_devices_renew & (1 << i))
                {
                    devices_suspend_cb[i](devices_handle[i]);
                }
            }

            g_devices_renew = ALL_DEVICES_BACKUP;
        }

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
        if (g_extra_devices_renew > 0)
        {
            for (i = 0x0; i < MAX_EXTRA_DEVICE_NUM; i++)
            {
                if (extra_devices_suspend_cb[i] == NULL)
                    continue;

                if (g_extra_devices_renew & (1 << i))
                {
                    extra_devices_suspend_cb[i](extra_devices_handle[i]);
                }
            }

            g_extra_devices_renew = ALL_DEVICES_BACKUP;
        }
#endif

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
        if (g_devices_state == 0x0)
#else
        if((g_devices_state == 0x0) && (g_extra_devices_state == 0x0))
#endif
        {
            #ifdef HAL_RECOVER_WHEN_USING
            g_devices_sleep = ALL_DEVICES_SLEEP;
            #endif
        }
        else
        {
            return DEVICE_BUSY;
        }

        return DEVICE_IDLE;
    }

    return DEVICE_BUSY;
}

SECTION_RAM_CODE void pwr_wake_up_ind_new(void)
{
    uint32_t i;

#ifndef HAL_RECOVER_WHEN_USING
    for (i = 0x0; i < MAX_PERIPH_DEVICE_NUM; i++)
    {
        if (devices_resume_cb[i] == NULL)
            continue;

        if (devices_handle[i] == NULL)
            continue;

        devices_resume_cb[i](devices_handle[i]);
    }
#endif

#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    for (i = 0x0; i < MAX_EXTRA_DEVICE_NUM; i++)
    {
        if (extra_devices_resume_cb[i] == NULL)
            continue;

        extra_devices_resume_cb[i](extra_devices_handle[i]);
    }
#endif
}
#endif

pwr_id_t pwr_register_sleep_cb(const app_sleep_callbacks_t *p_cb, wakeup_priority_t wakeup_priority)
{
    pwr_id_t id = -1;
    uint8_t  i  = 0;

    GLOBAL_EXCEPTION_DISABLE();

    if (!s_pwr_env.is_pwr_callback_reg)
    {
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        pwr_mgmt_dev_init(pwr_wake_up_ind_new);
        pwr_mgmt_set_callback(pwr_enter_sleep_check_new, NULL);
#else
        pwr_mgmt_dev_init(pwr_wake_up_ind);
        pwr_mgmt_set_callback(pwr_enter_sleep_check, NULL);
#endif

        s_pwr_env.is_pwr_callback_reg = true;
    }

    if (p_cb == NULL || wakeup_priority > WAPEUP_PRIORITY_HIGH || wakeup_priority < WAPEUP_PRIORITY_LOW)
    {
        goto exit;
    }

    while ((i < APP_SLEEP_CB_MAX) && (s_pwr_env.pwr_sleep_cb[i] != NULL))
    {
        i++;
    }
    if (i < APP_SLEEP_CB_MAX)
    {
        s_pwr_env.pwr_sleep_cb[i] = (app_sleep_callbacks_t *)p_cb;
        s_pwr_env.wakeup_priority[i] = wakeup_priority;
        id = i;
    }

exit:
    GLOBAL_EXCEPTION_ENABLE();

    return id;
}

void pwr_unregister_sleep_cb(pwr_id_t id)
{
    if((id >= 0) && (id < APP_SLEEP_CB_MAX))// Is id valid?
    {
        s_pwr_env.pwr_sleep_cb[id] = NULL;
    }
}

SECTION_RAM_CODE void pwr_wake_up_ind(void)
{
    uint8_t i;
    app_sleep_callbacks_t *p_cb;
    wakeup_priority_t priority;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    uint8_t hal_init_mark = 0;
#endif

    for (priority = WAPEUP_PRIORITY_HIGH; priority != 0; priority--)
    {
        for (i = 0; i < APP_SLEEP_CB_MAX; i++)
        {
            p_cb = s_pwr_env.pwr_sleep_cb[i];
            if ((p_cb != NULL) && (p_cb ->app_wake_up_ind != NULL) && (priority == s_pwr_env.wakeup_priority[i]))
            {
                p_cb ->app_wake_up_ind();
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
                if(hal_init_mark == 0)
                {
                    hal_init_mark = 1;
                    hal_init();
                }
#endif
            }
        }
    }
}

pwr_mgmt_dev_state_t pwr_enter_sleep_check(void)
{
    int16_t i;
    pwr_mgmt_dev_state_t allow_entering_sleep = DEVICE_IDLE;
    app_sleep_callbacks_t *p_cb;

    // 1. Inquiry Adapters
    for (i = APP_SLEEP_CB_MAX - 1; i >= 0; i--)
    {
        p_cb = s_pwr_env.pwr_sleep_cb[i];
        if (( p_cb != NULL) && (p_cb->app_prepare_for_sleep != NULL) )
        {
            if (!p_cb->app_prepare_for_sleep())
            {
                allow_entering_sleep = DEVICE_BUSY;
                break;
            }
        }
    }

    // 2. If an Adapter rejected sleep, resume any Adapters that have already accepted it.
    if ( allow_entering_sleep == DEVICE_BUSY )
    {
        i++;
        while (i < APP_SLEEP_CB_MAX)
        {
            p_cb = s_pwr_env.pwr_sleep_cb[i];
            if ( (p_cb != NULL) && (p_cb->app_sleep_canceled != NULL) )
            {
                p_cb->app_sleep_canceled();
            }
            i++;
        }
    }

    return allow_entering_sleep;
}

