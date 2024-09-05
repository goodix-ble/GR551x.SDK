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
#ifndef __LCP_QUEUE_H__
#define __LCP_QUEUE_H__
#include "grx_sys.h"

#define MAX_QUEUE_SIZE   2
#define MAX_PKG_SIZE     140
#define MAX_DEVICE_CNT   2
    
struct lcp_pkg_queue
{
    uint8_t  head;
    uint8_t  tail;
    uint16_t buffer_len[MAX_QUEUE_SIZE];
    uint8_t  buffer[MAX_QUEUE_SIZE][MAX_PKG_SIZE];
    // buffer_get_time: current buffer get times count,if buffer_get_time greater than buffer_max_get_time ,
    // the buffer will be force popped
    uint8_t  buffer_get_time[MAX_QUEUE_SIZE];
    // buffer_max_get_time: the max get time count of a buffer,if buffer_get_time greater than buffer_max_get_time ,
    // the buffer will be force popped
    uint8_t  buffer_max_get_time[MAX_QUEUE_SIZE];
    // packet_system_time:the system time of the buffer pack
    uint32_t packet_system_time[MAX_QUEUE_SIZE];
};

struct lcp_control_t
{
    uint8_t  buffer[MAX_DEVICE_CNT][MAX_PKG_SIZE];
    uint8_t  buffer_ack_flag[MAX_DEVICE_CNT];
    uint8_t  buffer_new_flag[MAX_DEVICE_CNT];
    uint16_t buffer_len[MAX_DEVICE_CNT];
    uint16_t buffer_get_time[MAX_DEVICE_CNT];
};

void lcp_queue_init(struct lcp_pkg_queue *queue) ;
bool lcp_queue_push(struct lcp_pkg_queue *queue,uint8_t *buffer,uint16_t buffer_len,uint8_t max_get_time,uint32_t packet_system_time);
bool lcp_queue_pop(struct lcp_pkg_queue *queue) ;
uint8_t lcp_queue_get_buffer(struct lcp_pkg_queue *queue,uint8_t **buffer);
bool lcp_is_queue_full(struct lcp_pkg_queue *queue);
bool lcp_is_queue_empty(struct lcp_pkg_queue *queue);

bool lcp_control_cmd_set(struct lcp_control_t *control_cmd,uint8_t device_id,uint8_t *buffer,uint16_t buffer_len);
bool lcp_control_cmd_clear(struct lcp_control_t *control_cmd,uint8_t device_id);
bool lcp_control_cmd_is_new(struct lcp_control_t *control_cmd,uint8_t device_id);
uint8_t lcp_control_cmd_get_buffer(struct lcp_control_t *control_cmd,uint8_t device_id,uint8_t **buffer);
bool lcp_control_cmd_is_ack(struct lcp_control_t *control_cmd,uint8_t device_id);
uint8_t lcp_control_cmd_get_get_buffer_time(struct lcp_control_t *control_cmd,uint8_t device_id);
#endif //__LCP_QUEUE_H__
