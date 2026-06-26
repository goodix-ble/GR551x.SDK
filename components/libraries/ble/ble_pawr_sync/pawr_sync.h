/**
 *****************************************************************************************
 *
 *
 * @file pawr sync.h
 *
 * @brief Header file - PAwR sync APIs
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

#ifndef _PAWR_SYNC_H_
#define _PAWR_SYNC_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr_includes.h"
#include "utility.h"
/*
 * ENUMERATIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief pawr sync parameter set.
 *
 * @param[in] sync_param:   pawr adv sync paramter.
 * @param[in] evt_list:     sync subevt idx list.
 * @param[in] evt_num:     sync subevt idx list number.
 * @param[in] rsp_slot:    sync subevt response slot.
 *****************************************************************************************
 */
void ble_gap_pawr_subevent_sync_param_set(ble_gap_per_sync_trans_param_t *sync_param, uint8_t *evt_list, uint8_t evt_num, uint8_t rsp_slot);

/**
 *****************************************************************************************
 * @brief Capture pawr sync events on BLE.
 *
 * @param[in] p_evt:   pointer on BLE event.
 *****************************************************************************************
 */
 void ble_pawr_sync_evt_on_ble_capture(const ble_evt_t *p_evt);

#endif

