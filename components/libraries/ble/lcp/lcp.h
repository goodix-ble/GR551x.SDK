/*****************************************************************************
######Copyright (c) Goodix 2022

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
******************************************************************************/
#ifndef __LCP_H__
#define __LCP_H__

#include "grx_sys.h"
#include "lcp_queue.h"
#define CRC_INIT          0x555555

#define CHANNEL_IDX       34

#define CURRENT_DEVICE_MODE_MASTER   1
#define CURRENT_DEVICE_MODE_SLAVE    2

#define CURRENT_MASTER_STATE_NORMAL    0x01
#define CURRENT_MASTER_STATE_ADV     0x02
#define CURRENT_MASTER_STATE_SYNC_IO    0x04
#define CURRENT_MASTER_STATE_TIME_SYNC    0x08
#define CURRENT_MASTER_STATE_CHANNEL_MAP_UPDATE    0x10
#define CURRENT_MASTER_STATE_HOP    0x20


#define SECTION_RAM_CODE __attribute__((section("RAM_CODE")))
struct lcp_sn_nesn_t
{
    uint8_t m_sn;
    uint8_t m_sn_prev;
    uint8_t m_nesn;
    uint8_t m_nesn_prev;
    
    uint8_t s_sn;
    uint8_t s_sn_prev;
    uint8_t s_nesn;
    uint8_t s_nesn_prev;
};

typedef struct
{
    /// Integer part of the time (in half-slot)
    uint32_t hs;
    /// Fractional part of the time (in half-us) (range: 0-624)
    uint16_t hus;
} stack_time_t;
typedef struct
{
    /// Integer part of the time (in half-ms)
    uint32_t hms;
    /// Fractional part of the time (in us)
    uint16_t us;
} system_timestamp_t;

typedef struct _pair_info
{
    uint32_t magic_words;
    uint32_t pair_flag[2];
    uint32_t pair_aa[2];
    uint32_t defalut_aa[2];
    uint32_t check_sum;
} pair_info_t;

typedef void (*timesync_cbk_t)(uint32_t timesync_offset);
typedef uint32_t (*timesync_offset_get_cbk_t)(void);
typedef void (*diag_debug_func_t)(void);
typedef void (*diag_tx_en_func_t)(void);
typedef void (*rx_handler_func_t)(uint32_t aa,uint8_t *data_buffer,uint16_t data_len);
typedef void (*dis_connect_handler_func_t)(uint32_t aa);
typedef void (*connect_handler_func_t)(uint8_t device_id,uint32_t aa);
typedef void (*adv_report_handler_func_t)(void);

extern void *memcpy_ram (void *dest, const void *src, size_t len);

uint32_t lcp_get_system_time(void);
system_timestamp_t* lcp_get_timestamp(uint8_t device_id);
bool lcp_is_current_device_online(uint8_t device_id);
void lcp_system_time_run(void);
void lcp_timesync_cbk_register(timesync_cbk_t timesync_cb,timesync_offset_get_cbk_t timesync_offset_get_cb);
void lcp_cbk_register(rx_handler_func_t rx_handler_cb,connect_handler_func_t  connect_handler_cb,dis_connect_handler_func_t  dis_connect_handler_cb);
void lcp_adv_report_cbk_register(adv_report_handler_func_t adv_report_handler_cb);
void lcp_debug_sync_cbk_register(diag_debug_func_t sync_set_cb,diag_debug_func_t sync_reset_cb);
void lcp_debug_tx_cbk_register(diag_debug_func_t tx_set_cb,diag_debug_func_t tx_reset_cb);
void lcp_debug_rx_cbk_register(diag_debug_func_t rx_set_cb,diag_debug_func_t rx_reset_cb);
void lcp_ext_pa_cbk_register(diag_tx_en_func_t tx_en_set_cb,diag_tx_en_func_t tx_en_reset_cb);
void lcp_debug_adv_cbk_register(diag_debug_func_t adv_set_cb,diag_debug_func_t adv_reset_cb);
void lcp_debug_hopping_cbk_register(diag_debug_func_t hopping_set_cb,diag_debug_func_t hopping_reset_cb);
void lcp_debug_hop_change_cbk_register(diag_debug_func_t hop_change_set_cb,diag_debug_func_t hop_change_reset_cb);
void lcp_debug_1ms_cbk_register(diag_debug_func_t ms_set_cb,diag_debug_func_t ms_reset_cb);
void lcp_debug_timesync_cbk_register(diag_debug_func_t timesync_set_cb,diag_debug_func_t timesync_reset_cb);
uint16_t lcp_init(uint8_t device_mode,uint8_t tx_pwr,uint32_t freq);
uint16_t lcp_hopping_init(uint32_t *hopping_map,uint8_t hopping_map_cnt,uint8_t standby_hopping_map_cnt);
void lcp_deinit(void);
uint16_t lcp_send_data(uint32_t aa,uint8_t* data,uint16_t data_len,uint8_t max_retry_time);
void lcp_run(void);
int8_t lcp_get_pkg_rssi_value(uint8_t device_id);
void lcp_debug_info_printf(void);
uint8_t lcp_get_current_device_mode(void);
stack_time_t lcp_get_stack_time(void);
pair_info_t * lcp_get_pair_info(void);
void lcp_pair_info_init(uint32_t aa0,uint32_t aa1);
bool pair_info_check_sum_check(uint8_t device_type,pair_info_t *p_pair_info);
void lcp_set_sync_request(void);
void lcp_set_pair_info(uint8_t device_id,uint32_t pair_flag,uint32_t aa);
uint32_t lcp_get_pair_flag(uint8_t device_id);
uint32_t lcp_get_default_aa(pair_info_t pair_info,uint8_t device_id);
bool pair_info_cal_check_sum(uint8_t device_type,pair_info_t *p_pair_info);
void lcp_aa_reset(void);
void lcp_is_need_hop_2_comm_channel(void);
stack_time_t lcp_get_sync_time(void);















#endif //__LCP_H__
