/**
 ****************************************************************************************
 *
 * @file dma_anti_overflow.h
 *
 * @brief General-purpose DMA anti-overflow module.
 *
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
 *****************************************************************************************
 */

#ifndef _DMA_ANTI_OVERFLOW_H_
#define _DMA_ANTI_OVERFLOW_H_

#include "hal_dma.h"
#include "app_timer.h"

#define DMA_RX_EVT_TIMER     (1 << 0)
#define DMA_RX_EVT_DMA_DONE  (1 << 1)

typedef uint16_t (*dma_start_rx_func)(uint8_t *p_data, uint32_t size);
typedef void (*dma_rx_callback_func)(uint8_t *p_data, uint32_t size);

typedef struct
{
    volatile uint8_t rx_harvesting;
    volatile uint8_t rx_evt_pending;
    uint32_t dma_epoch;
    uint32_t dma_done_epoch;

    dma_handle_t *p_dma_handle;
    uint32_t dma_channel;

    dma_start_rx_func start_rx_dma;
    dma_rx_callback_func rx_callback;

    uint8_t *p_rx_buf_a;
    uint8_t *p_rx_buf_b;
    uint8_t *p_active_rx_buf;
    uint32_t buf_len;
    uint8_t dma_data_alignment;  // Unit:byte

    uint8_t rx_state_busy_flag;
    uint8_t rx_state_idle_flag;
    uint8_t *p_rx_state;
    uint8_t *p_dma_rx_mode;

    app_timer_id_t rx_timer_id;
    uint32_t timer_period_ms;
} dma_rx_anti_overflow_t;

#ifdef __cplusplus
extern "C" {
#endif

void dma_rx_start(dma_rx_anti_overflow_t *p_ctx);
void dma_rx_stop(dma_rx_anti_overflow_t *p_ctx);
void dma_rx_schedule(dma_rx_anti_overflow_t *p_ctx, uint8_t evt);

#ifdef __cplusplus
}
#endif

#endif /* _DMA_ANTI_OVERFLOW_H_ */
