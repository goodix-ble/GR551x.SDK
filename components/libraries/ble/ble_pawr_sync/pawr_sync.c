/**
 *****************************************************************************************
 *
 * @file pawr_sync.c
 *
 * @brief The implementation of PAwR sync functions.
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
#include "pawr_sync.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_log.h"
#include <string.h>
/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t pawr_sync_handle = 0xFF;
static uint8_t test_rsp_data[5] = {0x04, 0xFF, 0xaa, 0xbb, 0xcc};
static uint8_t pawr_rsp_slot = 0;
static uint8_t *sync_subevt_list = NULL;
static uint8_t sync_subevt_numb = 0;

static uint8_t def_sync_subevt_list[] = {0};
static uint8_t def_sync_subevt_numb = 1;

static ble_gap_per_sync_trans_param_t pawr_sync_past_param =
{
    .mode = BLE_GAP_SYNC_REP_EN,
    .skip = 0,
    .sync_to = 6000,/*sync timerout N * 10 ms*/
    .cte_type = BLE_GAP_NO_SYNC_IGNORE,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void ble_gap_adv_report_subevent_data_handler(const ble_evt_t *p_evt)
{
    uint16_t err_code = 0;

    if (p_evt->evt.gapm_evt.params.adv_report.adv_type == BLE_GAP_REPORT_TYPE_SUBEVT_DATA)
    {
        test_rsp_data[2] =  p_evt->evt.gapm_evt.params.adv_report.per_adv_evt_cnt;
        test_rsp_data[3] =  p_evt->evt.gapm_evt.params.adv_report.subevt_idx;
        test_rsp_data[4] =  pawr_rsp_slot;

        ble_gap_per_adv_rsp_data_t per_adv_rsp_data;
        per_adv_rsp_data.req_evt = p_evt->evt.gapm_evt.params.adv_report.per_adv_evt_cnt + 1;
        per_adv_rsp_data.req_subevt = p_evt->evt.gapm_evt.params.adv_report.subevt_idx;
        per_adv_rsp_data.rsp_subevt = p_evt->evt.gapm_evt.params.adv_report.subevt_idx;

        per_adv_rsp_data.rsp_slot = pawr_rsp_slot;
        per_adv_rsp_data.rsp_data_len = sizeof(test_rsp_data)/sizeof(uint8_t);
        per_adv_rsp_data.rsp_data = test_rsp_data;
        err_code = ble_gap_per_adv_rsp_data_set(p_evt->evt.gapm_evt.params.adv_report.per_sync_idx, &per_adv_rsp_data);

        APP_LOG_INFO("per_adv_evt_cnt = %d, subevt_idx = %d, response slot = %d err_code %d",
            p_evt->evt.gapm_evt.params.adv_report.per_adv_evt_cnt,
            p_evt->evt.gapm_evt.params.adv_report.subevt_idx,
            per_adv_rsp_data.rsp_slot,
            err_code);
        APP_LOG_RAW_INFO("data is : ");
        for(uint8_t idx = 0; idx < p_evt->evt.gapm_evt.params.adv_report.length; idx++)
        {
            APP_LOG_RAW_INFO("%02x", p_evt->evt.gapm_evt.params.adv_report.data[idx]);
        }
        APP_LOG_RAW_INFO("\r\n");
    }
}

static void ble_gap_pawr_sync_establish_evt_handler(const ble_evt_t *p_evt)
{
    APP_LOG_INFO("pawr sync estb: inst_idx = %d, status = 0x%x\n", p_evt->evt.gapm_evt.index, p_evt->evt_status);

    APP_LOG_INFO("num_subevt = %d, subevt_interval = %d, rsp_slot_delay = %d, rsp_slot_spacing = %d\n",
        p_evt->evt.gapm_evt.params.sync_established.num_subevt, p_evt->evt.gapm_evt.params.sync_established.subevt_interval,
        p_evt->evt.gapm_evt.params.sync_established.rsp_slot_delay, p_evt->evt.gapm_evt.params.sync_established.rsp_slot_spacing);

    pawr_sync_handle = p_evt->evt.gapm_evt.index;

    //select subevent to recv
    ble_gap_per_sync_subevt_t sync_subevt;
    sync_subevt.inc_tx_pwr_flag = true;
    sync_subevt.num_subevt = sync_subevt_numb;
    sync_subevt.subevt = sync_subevt_list;
    ble_gap_per_sync_subevt_set(pawr_sync_handle, &sync_subevt);
}

static void ble_gap_pawr_sync_lost_evt_handler(const ble_evt_t *p_evt)
{
    pawr_sync_handle = 0xFF;

    APP_LOG_INFO("ble_gap_sync_lost_evt_handler: status = 0x%x, restart advertising to wait for connecting\n", p_evt->evt_status);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void ble_gap_pawr_subevent_sync_param_set(ble_gap_per_sync_trans_param_t *sync_param, uint8_t *evt_list, uint8_t evt_num, uint8_t rsp_slot)
{
    if ((evt_num == 0) || (evt_list == NULL))
    {
        sync_subevt_numb = 1;
        sync_subevt_list = def_sync_subevt_list;
    }
    else
    {
        sync_subevt_numb = evt_num;
        sync_subevt_list = evt_list;
    }
    pawr_rsp_slot = rsp_slot;

    if (NULL != sync_param)
    {
        memcpy(&pawr_sync_past_param, sync_param, sizeof(ble_gap_per_sync_trans_param_t));
    }
}

void ble_pawr_sync_evt_on_ble_capture(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GAPM_EVT_ADV_REPORT:
            ble_gap_adv_report_subevent_data_handler(p_evt);
            break;

        case BLE_GAPC_EVT_CONNECTED:
        {
            /*start to sync with ap, and then wait sync success event report*/
            uint16_t status = ble_gap_per_sync_trans_param_set(p_evt->evt.gapc_evt.index, 0, &pawr_sync_past_param);
            APP_ERROR_CHECK(status);
            break;
        }

        case BLE_GAPM_EVT_SYNC_ESTABLISH:
            ble_gap_pawr_sync_establish_evt_handler(p_evt);
            break;

        case BLE_GAPM_EVT_SYNC_LOST:
            ble_gap_pawr_sync_lost_evt_handler(p_evt);
            break;

        default:
            break;
    }
}
