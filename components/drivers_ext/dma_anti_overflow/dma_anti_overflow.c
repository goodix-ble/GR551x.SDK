/**
  ****************************************************************************************
  * @file    dma_anti_overflow.c
  * @author  BLE Driver Team
  * @brief   General-purpose DMA anti-overflow module.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2025 GOODIX
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
#include "dma_anti_overflow.h"
#include "app_assert.h"

#define ENTER_CRITICAL()        global_exception_disable()
#define EXIT_CRITICAL(set_pri)  global_exception_enable(set_pri)

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void dma_rx_harvest(dma_rx_anti_overflow_t *p_ctx)
{
    uint8_t evt;
    uint32_t rx_size;
    uint8_t *p_rx_buf;
    uint32_t done_epoch;
    uint32_t irq;
    uint16_t dma_ret = 0;
    uint16_t hw_ret = 0;

    while (1)
    {
        irq = ENTER_CRITICAL(); // close irq 1
        evt = p_ctx->rx_evt_pending;
        done_epoch = p_ctx->dma_done_epoch;
        p_ctx->rx_evt_pending = 0;
        EXIT_CRITICAL(irq);     // open irq 1

        if (!evt)
        {
            break;
        }

        irq = ENTER_CRITICAL(); // close irq 2
        if ((evt & DMA_RX_EVT_DMA_DONE) || ((p_ctx->rx_state_busy_flag == *(p_ctx->p_rx_state)) && (ENABLE == *(p_ctx->p_dma_rx_mode))))
        {
            // Nothing to do.
        }
        else
        {
            EXIT_CRITICAL(irq); // open irq 2
            continue;
        }

        /* Suspend the DMA channel to disable the Peripheral DMA RX request */
        ll_dma_suspend_channel(p_ctx->p_dma_handle->p_instance, p_ctx->dma_channel);
        while (!ll_dma_is_empty_fifo(p_ctx->p_dma_handle->p_instance, p_ctx->dma_channel));

        if ((evt & DMA_RX_EVT_DMA_DONE) && (done_epoch == p_ctx->dma_epoch))
        {
            rx_size = p_ctx->buf_len;
        }
        else
        {
            uint32_t blk = ll_dma_get_block_size(p_ctx->p_dma_handle->p_instance, p_ctx->dma_channel);
            if (blk == 0)
            {
                ll_dma_resume_channel(p_ctx->p_dma_handle->p_instance,
                                      p_ctx->dma_channel);
                EXIT_CRITICAL(irq); // open irq 2
                continue;
            }
            rx_size = blk * p_ctx->dma_data_alignment;
        }

        dma_ret = hal_dma_abort(p_ctx->p_dma_handle);
        APP_ASSERT_CHECK(0 == dma_ret);

        *(p_ctx->p_dma_rx_mode) = DISABLE;
        *(p_ctx->p_rx_state) = p_ctx->rx_state_idle_flag;

        p_ctx->p_active_rx_buf = (p_ctx->p_active_rx_buf == p_ctx->p_rx_buf_a) ? p_ctx->p_rx_buf_b : p_ctx->p_rx_buf_a;

        p_ctx->dma_epoch++;
        hw_ret = p_ctx->start_rx_dma(p_ctx->p_active_rx_buf, p_ctx->buf_len);
        APP_ASSERT_CHECK(0 == hw_ret);

        EXIT_CRITICAL(irq); // open irq 2

        p_rx_buf = (p_ctx->p_active_rx_buf == p_ctx->p_rx_buf_a) ? p_ctx->p_rx_buf_b : p_ctx->p_rx_buf_a;

        p_ctx->rx_callback(p_rx_buf, rx_size);
    }

    irq = ENTER_CRITICAL();  // close irq 3
    p_ctx->rx_harvesting = 0;
    // If the DMA terminates unexpectedly, restart the DMA.
    if ((p_ctx->rx_state_idle_flag == *(p_ctx->p_rx_state)) && (DISABLE == *(p_ctx->p_dma_rx_mode)))
    {
        uint16_t dma_ret = 0;
        uint16_t hw_ret = 0;

        if (ll_dma_is_enabled_channel(p_ctx->p_dma_handle->p_instance, p_ctx->p_dma_handle->channel))
        {
            dma_ret = hal_dma_abort(p_ctx->p_dma_handle);
            APP_ASSERT_CHECK(0 == dma_ret);
        }

        hw_ret = p_ctx->start_rx_dma(p_ctx->p_active_rx_buf, p_ctx->buf_len);
        APP_ASSERT_CHECK(0 == hw_ret);
    }
    EXIT_CRITICAL(irq);      // open irq 3
}

static void dma_rx_timeout_handler(void *p_arg)
{
    dma_rx_anti_overflow_t *p_ctx = (dma_rx_anti_overflow_t *)p_arg;
    dma_rx_schedule(p_ctx, DMA_RX_EVT_TIMER);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void dma_rx_start(dma_rx_anti_overflow_t *p_ctx)
{
    p_ctx->rx_harvesting = 0;
    p_ctx->rx_evt_pending = 0;
    p_ctx->dma_epoch = 0;
    p_ctx->dma_done_epoch = 0;

    p_ctx->start_rx_dma(p_ctx->p_active_rx_buf, p_ctx->buf_len);
    app_timer_create(&p_ctx->rx_timer_id, ATIMER_REPEAT, dma_rx_timeout_handler);
    app_timer_start(p_ctx->rx_timer_id, p_ctx->timer_period_ms, p_ctx);
}

void dma_rx_stop(dma_rx_anti_overflow_t *p_ctx)
{
    uint16_t dma_ret = 0;

    GLOBAL_EXCEPTION_DISABLE();

    dma_ret = hal_dma_abort(p_ctx->p_dma_handle);
    *(p_ctx->p_dma_rx_mode) = DISABLE;
    *(p_ctx->p_rx_state) = p_ctx->rx_state_idle_flag;
    app_timer_stop(p_ctx->rx_timer_id);
    app_timer_delete(&p_ctx->rx_timer_id);

    GLOBAL_EXCEPTION_ENABLE();

    APP_ASSERT_CHECK(0 == dma_ret);
}

void dma_rx_schedule(dma_rx_anti_overflow_t *p_ctx, uint8_t evt)
{
    uint32_t irq = ENTER_CRITICAL();

    p_ctx->rx_evt_pending |= evt;

    if (p_ctx->rx_harvesting)
    {
        EXIT_CRITICAL(irq);
        return;
    }

    p_ctx->rx_harvesting = 1;
    EXIT_CRITICAL(irq);

    dma_rx_harvest(p_ctx);
}
