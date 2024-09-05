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
#include "lcp.h"
#include "lcp_pdu.h"
//#include "flash_scatter_config.h"
//#include "app_log.h"
//#include "app_error.h"
#include "ble_error.h"

#define UPDATA_CMD_LEN      (2)

uint8_t sync_payload[9];
uint8_t adv_payload[29];

uint8_t *lcp_get_sync_payload(uint16_t *payload_len)
{
    if(payload_len != NULL)
    {
        *payload_len = sizeof(sync_payload);
        return sync_payload;
    }
    return NULL;
}
uint8_t *lcp_get_adv_payload(uint16_t *payload_len)
{
    if(payload_len != NULL)
    {
        *payload_len = sizeof(adv_payload);
        return adv_payload;
    }
    return NULL;
}
uint16_t lcp_data_head_pack(uint8_t lcp_data_type,uint8_t lcp_data_sn,uint8_t lcp_data_nesn,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    if(lcp_data_payload != NULL && lcp_data_payload_len >=1)
    {
        lcp_data_payload[0] = 0;
        lcp_data_payload[0] = (lcp_data_type & 0x0F) | ((lcp_data_sn<<4)&0x10) | ((lcp_data_nesn<<5)&0x20) ;
        //printf("\r\n%d %d %d %d \r\n",lcp_data_type,lcp_data_sn,lcp_data_nesn,lcp_data_payload[0]);
        return lcp_data_payload_len;
    }
    return 0;
}
uint16_t lcp_data_head_unpack(uint8_t *lcp_data_type,uint8_t *lcp_data_sn,uint8_t *lcp_data_nesn,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    if((lcp_data_payload != NULL) && (lcp_data_type!=NULL) && (lcp_data_sn!=NULL) && (lcp_data_nesn!=NULL) && lcp_data_payload_len >=1)
    {
        *lcp_data_type = (lcp_data_payload[0] & 0x0F);
        *lcp_data_sn =   (lcp_data_payload[0] & 0x10)>>4;
        *lcp_data_nesn = (lcp_data_payload[0] & 0x20)>>5;
        return lcp_data_payload_len;
    }
    return 0;
}
uint16_t lcp_data_update_cmd_pack(uint8_t ch0,uint8_t ch1,uint8_t ch2,uint8_t update_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    if(lcp_data_payload != NULL && lcp_data_payload_len >=UPDATA_CMD_LEN)
    {
//        lcp_data_payload[0] = channel_map & 0x00FF;//(update_index <<5) &0xE0;
//		lcp_data_payload[1] = (channel_map>>8) & 0x1 ;
//		lcp_data_payload[1] = lcp_data_payload[1] | ((update_index & 0x0F)<<4);
		lcp_data_payload[0] = ch0 & 0x0F;
		lcp_data_payload[0] = lcp_data_payload[0] |( (ch1 & 0x0F)<<4);
        lcp_data_payload[1] =  ch2 & 0x0F;
		lcp_data_payload[1] = lcp_data_payload[1]|((update_index&0x0F)<<4);
        //lcp_data_payload[0] = lcp_data_payload[0] | (update_time &LCP_HOP_UPDATE_TIME_MAX) ;
        //printf("%x %x \r\n",lcp_data_payload[0],lcp_data_payload[1]);
        return lcp_data_payload_len;
    }
    return 0;
}
uint16_t lcp_data_update_cmd_unpack(uint8_t *ch0,uint8_t *ch1,uint8_t *ch2,uint8_t *update_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    if((lcp_data_payload != NULL) && (update_index!=NULL)  && lcp_data_payload_len >=UPDATA_CMD_LEN)
    {
//        *channel_map = lcp_data_payload[0];//(lcp_data_payload[0] & 0xE0) >> 5;
//		*channel_map = *channel_map | ((lcp_data_payload[1] & 0x01)<<8);
//		*update_index = (lcp_data_payload[1] & 0xF0) >> 4;
		
		*update_index = (lcp_data_payload[1]& 0xF0)>>4;
		*ch2 = (lcp_data_payload[1] & 0x0F);
		*ch1 = (lcp_data_payload[0] & 0xF0)>>4;
		*ch0 = (lcp_data_payload[0] & 0x0F);
        //*update_time =   (lcp_data_payload[0] & LCP_HOP_UPDATE_TIME_MAX);
        return lcp_data_payload_len;
    }
    return 0;
}

uint16_t lcp_data_sync_pack(uint32_t send_end_offset,uint32_t system_time,uint8_t lcp_data_sn,uint8_t lcp_data_nesn,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    struct lcp_sync_packet *lcp_sync_pkg=NULL;
    if(lcp_data_payload != NULL && lcp_data_payload_len >= (sizeof(struct lcp_sync_packet)+1))
    {
        lcp_data_head_pack(LCP_TIME_SYNC,lcp_data_sn,lcp_data_nesn,lcp_data_payload,lcp_data_payload_len);
        lcp_sync_pkg =(struct lcp_sync_packet *)(lcp_data_payload+1);
        lcp_sync_pkg->send_end_offset = send_end_offset;
        lcp_sync_pkg->system_time     = system_time;
        return (sizeof(struct lcp_sync_packet)+1);
    }
    return 0;
}
uint16_t lcp_data_sync_unpack(uint32_t *send_end_offset,uint32_t *system_time,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    if((lcp_data_payload != NULL) && (lcp_data_payload_len >= (sizeof(system_time)+sizeof(send_end_offset))) && (send_end_offset!=NULL) && (system_time != NULL))
    {
        struct lcp_sync_packet *lcp_sync_pkg = (struct lcp_sync_packet *)(lcp_data_payload);
        *send_end_offset = lcp_sync_pkg->send_end_offset;
        *system_time     = lcp_sync_pkg->system_time;
        return (sizeof(struct lcp_sync_packet)+1);
    }
    return 0;
}
uint16_t lcp_adv_pack(uint32_t send_end_offset,uint32_t system_time,uint32_t *map_table,uint8_t map_table_size,uint16_t used_map_table,uint8_t ch0,uint8_t ch1,uint8_t ch2,uint8_t current_map_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    struct lcp_adv_packet *lcp_adv_pkg=NULL;
    if((lcp_data_payload != NULL) && ((map_table!=NULL && map_table_size==LCP_MAP_SIZE)))
    {
        lcp_data_head_pack(LCP_ADV,0,0,lcp_data_payload,lcp_data_payload_len);
        lcp_adv_pkg =(struct lcp_adv_packet *)(lcp_data_payload+1);
        lcp_adv_pkg->send_end_offset = send_end_offset;
        lcp_adv_pkg->system_time     = system_time;
        lcp_adv_pkg->map[0] = map_table[0];
        lcp_adv_pkg->map[1] = map_table[1];
		lcp_adv_pkg->used_map_table= used_map_table;
		lcp_adv_pkg->use_ch = ch0 & 0x0F;
		lcp_adv_pkg->use_ch = lcp_adv_pkg->use_ch |( (ch1 & 0x0F)<<4);
        lcp_adv_pkg->use_ch_and_current_index     =  ch2 & 0x0F;
		lcp_adv_pkg->use_ch_and_current_index = lcp_adv_pkg->use_ch_and_current_index |((current_map_index&0x0F)<<4);

        //lcp_data_payload[17] = current_map_index;
        return (sizeof(struct lcp_adv_packet)+1);
    }
    return 0;
}
uint16_t lcp_adv_unpack(uint32_t *send_end_offset,uint32_t *system_time,uint32_t *map_table,uint8_t map_table_size,uint16_t *used_map_table,uint8_t *ch0,uint8_t *ch1,uint8_t *ch2,uint8_t *current_map_index,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    struct lcp_adv_packet *lcp_adv_pkg=NULL;
    if((lcp_data_payload != NULL)  &&(map_table!=NULL && map_table_size==LCP_MAP_SIZE)
        && (send_end_offset!=NULL) &&(system_time != NULL)&&(current_map_index !=NULL))
    {
        lcp_adv_pkg =(struct lcp_adv_packet *)(lcp_data_payload);
        *send_end_offset =  lcp_adv_pkg->send_end_offset ;
        *system_time     =  lcp_adv_pkg->system_time     ;
        map_table[0]     =  lcp_adv_pkg->map[0] ;
        map_table[1]     =  lcp_adv_pkg->map[1] ;
        *current_map_index = (lcp_adv_pkg->use_ch_and_current_index & 0xF0)>>4;
		*ch2 = (lcp_adv_pkg->use_ch_and_current_index & 0x0F);
		*ch1 = (lcp_adv_pkg->use_ch & 0xF0)>>4;
		*ch0 = (lcp_adv_pkg->use_ch & 0x0F);
		*used_map_table = lcp_adv_pkg->used_map_table;
        return (sizeof(struct lcp_adv_packet)+1);
    }
    return 0;
}
uint16_t lcp_hopmap_update_pack(uint32_t update_time,uint32_t *map_table,uint8_t map_table_size,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    struct lcp_hopmap_update_packet *lcp_hopmap_update_pkg=NULL;
    if((lcp_data_payload != NULL) && (lcp_data_payload_len >= (sizeof(struct lcp_hopmap_update_packet)+1)) &&(map_table!=NULL && map_table_size==LCP_MAP_SIZE))
    {
        lcp_data_head_pack(LCP_HOP_MAP_UPDATE_REQ,0,0,lcp_data_payload,lcp_data_payload_len);
        lcp_hopmap_update_pkg =(struct lcp_hopmap_update_packet *)(lcp_data_payload+1);

        lcp_hopmap_update_pkg->update_time     = update_time;
        lcp_hopmap_update_pkg->map[0] = map_table[0];
        lcp_hopmap_update_pkg->map[1] = map_table[1];

        return (sizeof(struct lcp_hopmap_update_packet)+1);
    }
    return 0;
}
uint16_t lcp_hopmap_update_unpack(uint32_t *update_time,uint32_t *map_table,uint8_t map_table_size,uint8_t *lcp_data_payload,uint16_t lcp_data_payload_len)
{
    struct lcp_hopmap_update_packet *lcp_hopmap_update_pkg=NULL;
    if((lcp_data_payload != NULL) && (lcp_data_payload_len >= (sizeof(struct lcp_hopmap_update_packet))) &&(map_table!=NULL && map_table_size==LCP_MAP_SIZE)
        && (update_time!=NULL))
    {
        lcp_hopmap_update_pkg =(struct lcp_hopmap_update_packet *)(lcp_data_payload);
        *update_time =  lcp_hopmap_update_pkg->update_time ;
        map_table[0]     =  lcp_hopmap_update_pkg->map[0] ;
        map_table[1]     =  lcp_hopmap_update_pkg->map[1] ;

        return (sizeof(struct lcp_hopmap_update_packet)+1);
    }
    return 0;
}



