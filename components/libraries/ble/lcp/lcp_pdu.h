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
#ifndef __LCP_PDU_H__
#define __LCP_PDU_H__

#include "grx_sys.h"

#define LCP_HOP_ENABLE_REQ         (0x01)
#define LCP_HOP_ENABLE_RESP        (0x02)

#define LCP_HOP_MAP_UPDATE_REQ     (0x03)
#define LCP_HOP_MAP_UPDATE_RESP    (0x04)

#define LCP_TIME_SYNC              (0x05)

#define LCP_ADV                    (0x06)
#define LCP_CONNECT_IND            (0x07)

#define LCP_DATA                   (0x08)

#define LCP_DATA_EMPTY             (0x09)

#define LCP_MAP_SIZE                (2) //2 uint32_t  

#define LCP_HOP_UPDATE_PAYLOAD_LEN  (12+1)

#define LCP_HOP_UPDATE_TIME_MAX     0x1F

struct lcp_sync_packet
{
    uint32_t send_end_offset;
    uint32_t system_time;
};
struct lcp_adv_packet
{
    uint32_t send_end_offset;
    
    uint32_t map[LCP_MAP_SIZE];
	uint32_t system_time;
	uint16_t used_map_table;
	uint8_t  use_ch;
	uint8_t  use_ch_and_current_index;
};
struct lcp_hopmap_update_packet
{
    uint32_t map[LCP_MAP_SIZE];
    uint32_t update_time;
};
uint8_t *lcp_get_sync_payload(uint16_t *payload_len);
uint8_t *lcp_get_adv_payload(uint16_t *payload_len);
uint16_t lcp_data_head_pack(uint8_t lcp_data_type,uint8_t lcp_data_sn,uint8_t lcp_data_nesn,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_data_head_unpack(uint8_t *lcp_data_type,uint8_t *lcp_data_sn,uint8_t *lcp_data_nesn,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_data_sync_pack(uint32_t send_end_offset,uint32_t system_time,uint8_t lcp_data_sn,uint8_t lcp_data_nesn,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_data_sync_unpack(uint32_t *send_end_offset,uint32_t *system_time,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_adv_pack(uint32_t send_end_offset,uint32_t system_time,uint32_t *map_table,uint8_t map_table_size,uint16_t used_map_table,uint8_t ch0,uint8_t ch1,uint8_t ch2,uint8_t current_map_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_adv_unpack(uint32_t *send_end_offset,uint32_t *system_time,uint32_t *map_table,uint8_t map_table_size,uint16_t *used_map_table,uint8_t *ch0,uint8_t *ch1,uint8_t *ch2,uint8_t *current_map_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_hopmap_update_pack(uint32_t update_time,uint32_t *map_table,uint8_t map_table_size,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_hopmap_update_unpack(uint32_t *update_time,uint32_t *map_table,uint8_t map_table_size,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_data_update_cmd_pack(uint8_t ch0,uint8_t ch1,uint8_t ch2,uint8_t update_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);
uint16_t lcp_data_update_cmd_unpack(uint8_t *ch0,uint8_t *ch1,uint8_t *ch2,uint8_t *update_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len);

#endif //__LCP_PDU_H__
