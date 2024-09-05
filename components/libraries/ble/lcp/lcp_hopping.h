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
#ifndef __LCP_HOPPING_H__
#define __LCP_HOPPING_H__

#include "grx_sys.h"
#define LCP_USE_FREQ_MHZ                       (2)
#define TOTAL_CHANNEL_CNT         (40)
#define LCP_START_FREQ            (2402)
#define LCP_END_FREQ              (LCP_START_FREQ + (TOTAL_CHANNEL_CNT-1)*LCP_USE_FREQ_MHZ)
#define MAX_CHANNEL_TABLE_SIZE    (TOTAL_CHANNEL_CNT)
#define CHANNEL_RECV_RATE_TIMEOUT_TIME         (10)

#define LCP_CURRENT_CONTROL_NONE_STATE         (0)
#define LCP_CURRENT_CONTROL_HOP_UPDATE_STATE   (1)

#define LCP_CI                                 (21)
#define LCP_CHANNEL_CNT_USED_TO_HOP            (9)

struct lcp_channel_recv_rate_t
{
    uint16_t send_cnt;
    uint16_t recv_cnt;
    uint32_t recv_rate;
    int16_t  ch_level;
    //int16_t  total_level;
};

typedef struct
{
	uint32_t channel_map[LCP_CHANNEL_CNT_USED_TO_HOP];
	uint8_t channel_used_flag[LCP_CHANNEL_CNT_USED_TO_HOP];
} lcp_hop_channel_map_t;

void lcp_hopping_disable(void);
uint16_t lcp_hopping_check(uint32_t system_time);
uint16_t lcp_hopping_map_update(uint32_t *new_hopping_map,uint8_t hopping_map_cnt);
uint16_t lcp_hopping_map_init(uint32_t *hopping_map,uint8_t hopping_map_cnt,uint8_t standby_hopping_map_cnt);
bool lcp_hopping_map_bit_2_hop_map(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size);
bool lcp_hopping_map_bit_2_standby_hop_map(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size);
void lcp_hoptable_replace(uint32_t device_id);
bool lcp_hopping_map_bit_2_new_hop_map(uint32_t *hop_map_bit,uint8_t hop_map_bit_size);
bool lcp_hop_map_2_hopping_map_bit(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size);
bool lcp_standby_hop_map_2_hopping_map_bit(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size);
bool lcp_used_hop_map_2_hopping_map_bit(uint8_t device_id,uint16_t *hop_map_bit);
bool lcp_hopping_map_bit_2_used_hop_map(uint8_t device_id,uint16_t hop_map_bit,uint8_t *used_ch_cnt);
void lcp_get_channel_cnt(uint8_t device_id,uint8_t *ch_index);
uint8_t lcp_get_current_hop_index(uint8_t device_id);
uint8_t lcp_get_current_standby_hop_index(uint8_t device_id);

void lcp_set_current_standby_hop_index(uint8_t device_id,uint8_t hop_map_index);
void lcp_set_current_hop_index(uint8_t device_id,uint8_t hop_map_index);
uint16_t slave_hoping_map_update(uint32_t *hopping_map_bit,uint8_t hopping_map_size,uint32_t update_time);
void lcp_hopping_2_comm_channel(uint8_t device_id);
void lcp_hopping_2_adv_channel(void);
void lcp_channel_scan(void);
void lcp_all_channel_scan(uint32_t init_freq,uint32_t *used_freq_table,uint8_t freq_table_size);
uint32_t *lcp_get_noise_scan_table(uint8_t* table_size);
uint32_t lcp_get_prev_recv_channel(uint8_t device_id);
void lcp_set_channel_recv_rate(uint8_t device_id,uint32_t freq,uint16_t send_cnt,uint16_t recv_cnt);
void lcp_set_prev_channel_recv_rate(uint8_t device_id,uint32_t freq,uint16_t send_cnt,uint16_t recv_cnt);
void lcp_hopping_set_default_adv_channel(uint32_t adv_channel);
void lcp_hopping_2_comm_channel_by_index(uint8_t device_id,uint8_t current_index);
bool lcp_get_scan_freq_table_flush_flag(void);
void lcp_channel_level_reset(uint8_t device_id);
bool lcp_get_update_cmd(uint8_t device_id,uint8_t *used_ch_cnt,uint8_t *update_index );
bool lcp_get_update_cmd_clear(uint8_t device_id);
bool lcp_set_update_cmd(uint8_t device_id,uint8_t *used_ch_cnt,uint8_t update_index );

#endif //__LCP_HOPPING_H__

