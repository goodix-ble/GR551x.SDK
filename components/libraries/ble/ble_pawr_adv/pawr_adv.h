/**
 *****************************************************************************************
 *
 * @file pawr_adv.h
 *
 * @brief PAwR Advertising APIs
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

#ifndef _PAwR_ADV_H__
#define _PAwR_ADV_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_timer.h"
#include "grx_sys.h"
#include "utility.h"
#include <string.h>
#include "gr_includes.h"
#include "app_log.h"

#define PAWR_MAX_SENT_BUFF_NUM               10
#define PAWR_MAX_SENT_DATA_LEN               250
#define PAWR_SENT_DATA_LEN_OFFSET            0
#define PAWR_SENT_DATA_TYPE_OFFSET           1

struct ble_per_adv_param_t
{
    uint16_t interval_min;
    uint16_t interval_max;
    uint8_t num_subevents;
    uint16_t subevent_interval;
    uint8_t response_slot_delay;
    uint8_t response_slot_spacing;
    uint8_t num_response_slots;
};

typedef uint16_t (*pawr_subevt_data_get_func_t)(uint8_t subevet_idx, uint8_t **buffer_idx);
typedef void (*pawr_rsp_data_report_func_t)(uint8_t subevet_idx, uint8_t rsp_slot,const uint8_t *buffer_idx, uint16_t buff_len);

typedef struct
{
    uint8_t grp_id;
    uint8_t sent_num;
    uint16_t data_len;
    uint8_t data_arr[PAWR_MAX_SENT_DATA_LEN];
}pawr_sent_data_arr_t;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief PAwR advertising parameter setting.
 *
 * @param[in]  adv_idx: the idx of pawr adv
 * @param[in]  per_adv_params_p: the pointer of pawr adv parameters
 *
 *****************************************************************************************
 */
uint16_t pawr_adv_param_fast_set(uint8_t adv_idx, struct ble_per_adv_param_t *per_adv_params_p);

/**
 *****************************************************************************************
 * @brief PawR advertising callback register.
 *
 * @param[in]  ptr: Pointer to subevt data get function
 * @param[in]  report_cb: Pointer to response slot data
 *
 * @return Result of init operation.
 *****************************************************************************************
 */
void pawr_adv_env_init(pawr_subevt_data_get_func_t ptr, pawr_rsp_data_report_func_t report_cb);

/**
 *****************************************************************************************
 * @brief Capture pawr events on BLE.
 *
 * @param[in] p_evt:   pointer on BLE event.
 *****************************************************************************************
 */
void ble_pawr_adv_evt_on_ble_capture(const ble_evt_t *p_evt);

#endif /* _PAwR_ADV_H__ */
