/**
 *******************************************************************************
 *
 * @file pawr_adv.c
 *
 * @brief The implementation of PAwR Advertising functions.
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
#include "pawr_adv.h"
#include "app_error.h"
#include "ring_buffer.h"
/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Sen data to peer.
 *
 * @param[in] index: Index of packet send.
 *****************************************************************************************
 */
static uint8_t g_num_rsp_slots = 0;

pawr_subevt_data_get_func_t pawr_subevt_data_get_func_cb = NULL;
pawr_rsp_data_report_func_t pawr_rsp_data_report_func_cb = NULL;

static void ble_gap_per_adv_subevent_data_req_handler(const ble_evt_t *p_evt)
{
    uint8_t subevt_start = p_evt->evt.gapm_evt.params.subevt_data_req.subevent_start;
    uint8_t subevt_data_cnt = p_evt->evt.gapm_evt.params.subevt_data_req.subevent_data_count;
    uint8_t loop_cnt = subevt_data_cnt / 15;

    if ((subevt_data_cnt % 15) > 0)
    {
        loop_cnt += 1;
    }

    ble_gap_per_adv_subevt_data_t subevt_data;

    memset(&subevt_data, 0, sizeof(ble_gap_per_adv_subevt_data_t));

    for (uint8_t i = 0; i < loop_cnt; i++)
    {
        uint8_t temp_subevt_data_cnt = 15;

        if ((i == 0) && (subevt_data_cnt <= 15))
        {
            temp_subevt_data_cnt = subevt_data_cnt;
        }
        else if (i == (loop_cnt - 1))
        {
            if ((subevt_data_cnt % 15) > 0)
            {
                temp_subevt_data_cnt = subevt_data_cnt % 15;
            }
        }

        for (uint8_t j = 0; j < temp_subevt_data_cnt; j++)
        {
            subevt_data.subevt_info[j].subevt = i * 15 + subevt_start + j;
            subevt_data.subevt_info[j].rsp_slot_start = 0;
            subevt_data.subevt_info[j].rsp_slot_cnt = g_num_rsp_slots;

            if (pawr_subevt_data_get_func_cb != NULL)
            {
                subevt_data.subevt_info[j].subevt_data_len = pawr_subevt_data_get_func_cb(subevt_data.subevt_info[j].subevt, &subevt_data.subevt_info[j].subevt_data);
            }
            else
            {
                subevt_data.subevt_info[j].subevt_data_len = 0;
            }
        }
        subevt_data.num_subevt = temp_subevt_data_cnt;

        ble_gap_per_adv_subevt_data_set(p_evt->evt.gapm_evt.index, &subevt_data);
    }
}

static void ble_gap_per_adv_rsp_report_handler(const ble_evt_t *p_evt)
{
    if ((pawr_rsp_data_report_func_cb != NULL)
        &&(p_evt->evt.gapm_evt.params.per_adv_rsp_report.len > 0))
    {
        pawr_rsp_data_report_func_cb(p_evt->evt.gapm_evt.params.per_adv_rsp_report.subevent,
                                        p_evt->evt.gapm_evt.params.per_adv_rsp_report.rsp_slot,
                                        p_evt->evt.gapm_evt.params.per_adv_rsp_report.data,
                                        p_evt->evt.gapm_evt.params.per_adv_rsp_report.len);
    }
}

static void ble_gap_set_per_adv_subevt_data_cmp_evt_handler(const ble_evt_t *p_evt)
{
    return ;
}
/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void pawr_adv_env_init(pawr_subevt_data_get_func_t ptr, pawr_rsp_data_report_func_t report_cb)
{
    pawr_subevt_data_get_func_cb = ptr;
    pawr_rsp_data_report_func_cb = report_cb;
}

void pawr_adv_param_fast_set(uint8_t adv_idx, struct ble_per_adv_param_t *per_adv_params_p)
{
    ble_gap_ext_adv_param_t gap_adv_param;
    uint8_t status;
    uint16_t prim_adv_min = per_adv_params_p->interval_min*2;
    uint16_t prim_adv_max = per_adv_params_p->interval_max*2;

    //if the adv interval more than 4s, the sync info maybe can not indicate the presence of a periodic advertising train.
    // So we adjust this interval
    if (per_adv_params_p->interval_min > (0x320 * 4))
    {
        prim_adv_min = 0x320 *2 *4 ;
        prim_adv_max = 0x320 *2 *4 ;
    }

    gap_adv_param.type = BLE_GAP_ADV_TYPE_PAWR;
    gap_adv_param.disc_mode = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;
    gap_adv_param.prop = 0;
    gap_adv_param.max_tx_pwr = 0;
    gap_adv_param.filter_pol = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    memset((uint8_t *)gap_adv_param.peer_addr.gap_addr.addr, 0, sizeof(ble_gap_bdaddr_t));
    gap_adv_param.peer_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    gap_adv_param.prim_cfg.adv_intv_min = prim_adv_min;
    gap_adv_param.prim_cfg.adv_intv_max = prim_adv_max;
    gap_adv_param.prim_cfg.chnl_map = BLE_GAP_ADV_CHANNEL_37_38_39;
    gap_adv_param.prim_cfg.phy = BLE_GAP_PHY_1MBPS_VALUE;
    gap_adv_param.prim_cfg.phy_op = BLE_GAP_PHY_NO_PREF_OR_REQ;
    gap_adv_param.second_cfg.max_skip = 0;
    gap_adv_param.second_cfg.phy = BLE_GAP_PHY_1MBPS_VALUE;
    gap_adv_param.second_cfg.phy_op = BLE_GAP_PHY_NO_PREF_OR_REQ;
    gap_adv_param.second_cfg.adv_sid= 0;
    gap_adv_param.period_cfg.adv_intv_min = per_adv_params_p->interval_min;
    gap_adv_param.period_cfg.adv_intv_max = per_adv_params_p->interval_max;

    gap_adv_param.period_cfg.pawr_cfg.num_subevts = per_adv_params_p->num_subevents;
    gap_adv_param.period_cfg.pawr_cfg.sub_intv = per_adv_params_p->subevent_interval;
    gap_adv_param.period_cfg.pawr_cfg.rsp_slot_delay = per_adv_params_p->response_slot_delay;
    gap_adv_param.period_cfg.pawr_cfg.rsp_slot_spacing = per_adv_params_p->response_slot_spacing;
    gap_adv_param.period_cfg.pawr_cfg.num_rsp_slots = per_adv_params_p->num_response_slots;
    g_num_rsp_slots = per_adv_params_p->num_response_slots;

    status = ble_gap_ext_adv_param_set(adv_idx, BLE_GAP_OWN_ADDR_STATIC, &gap_adv_param);
    APP_ERROR_CHECK(status);
}

void ble_pawr_adv_evt_on_ble_capture(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GAPM_EVT_PER_ADV_SUBEVENT_DATA_REQ:
            ble_gap_per_adv_subevent_data_req_handler(p_evt);
            break;

        case BLE_GAPM_EVT_PER_ADV_RSP_REPORT:
            ble_gap_per_adv_rsp_report_handler(p_evt);
            break;

        case BLE_GAPM_EVT_SET_PER_ADV_SUBEVT_DATA_CMP:
            ble_gap_set_per_adv_subevt_data_cmp_evt_handler(p_evt);
            break;

        default:
            break;
    }
}

