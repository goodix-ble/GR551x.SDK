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
#include "lcp_queue.h"
#include "lcp.h"
//#include "flash_scatter_config.h"
//#include "app_log.h"
//#include "app_error.h"
#include "ble_error.h"
void lcp_queue_init(struct lcp_pkg_queue *queue) 
{
    if(queue != NULL)
    {
        queue->head = 0;
        queue->tail = 0;
    }
}

bool lcp_queue_push(struct lcp_pkg_queue *queue,uint8_t *buffer,uint16_t buffer_len,uint8_t max_get_time,uint32_t packet_system_time) 
{
    if((queue != NULL) && (buffer!=NULL) && (buffer_len >0 && buffer_len < MAX_PKG_SIZE))
    {
        if(!lcp_is_queue_full(queue))
        {
            memcpy_ram(&(queue->buffer[queue->tail][3]),buffer,buffer_len);
            queue->buffer_len[queue->tail] = buffer_len+3;
            //queue->buffer_get_time[queue->tail] = 0;
            //queue->buffer_max_get_time[queue->tail] = max_get_time;
            //queue->packet_system_time[queue->tail] = packet_system_time ;
            queue->tail = (queue->tail +1) % MAX_QUEUE_SIZE;
            return true;
        }
        return false;
    }
    return false;
}
bool lcp_queue_pop(struct lcp_pkg_queue *queue)
{
    if(queue != NULL)
    {
        if(!lcp_is_queue_empty(queue))
        {
            //if(queue->buffer_get_time[queue->head] != 0)
            {
                queue->buffer_get_time[queue->head] = 0;
                queue->head = (queue->head +1) % MAX_QUEUE_SIZE;
                return true;
            }
            //else
            {
              //  return false;
            }
            
        }
        return false;
    }
    return false;
}
#if 0
uint8_t lcp_queue_get_buffer(struct lcp_pkg_queue *queue,uint8_t **buffer,uint8_t pop_flag)
{
    if((queue != NULL) && (buffer != NULL))
    {
        if(!lcp_is_queue_empty(queue))
        {
            // SN NESN is right and current buffer has already been got for one time,pop this buffer
            if(pop_flag == 1 && queue->buffer_get_time[queue->head] >0)
            {
                lcp_queue_pop(queue);
            }
                // check is there any buffer has delay more than 4ms and pop delay buffer
                while((lcp_get_system_time()- queue->packet_system_time[queue->head]>4 ) && !lcp_is_queue_empty(queue))
                {
                    lcp_queue_pop(queue);
                }
                if(!lcp_is_queue_empty(queue))
                {
                    // check current buffer's get time is greater than max get time ,if so ,pop this buffer
                    if(queue->buffer_get_time[queue->head] >= queue->buffer_max_get_time[queue->head])
                    {
                       if(lcp_queue_pop(queue))
                       {
                           if(!lcp_is_queue_empty(queue))
                           {
                               queue->buffer_get_time[queue->head]++;
                               *buffer = queue->buffer[queue->head];
                           }
                           else
                           {
                               return 0;
                           }
                       }
                       else
                       {
                            return 0;
                       }
                    }
                    else
                    {
                        queue->buffer_get_time[queue->head]++;
                        *buffer = queue->buffer[queue->head];
                    }
                    return queue->buffer_len[queue->head] ;
                }
                else
                {
                    return 0;
                }
            

        }
        return 0;
    }
    return 0;
}
#else
uint8_t lcp_queue_get_buffer(struct lcp_pkg_queue *queue,uint8_t **buffer)
{
    if((queue != NULL) && (buffer != NULL))
    {
        if(!lcp_is_queue_empty(queue))
        {
            queue->buffer_get_time[queue->head]++;
            *buffer = queue->buffer[queue->head];
            return queue->buffer_len[queue->head] ;
        }
        return 0;
    }
    return 0;
}

#endif
bool lcp_is_queue_full(struct lcp_pkg_queue *queue)
{
   if(queue != NULL)
   {
      if((queue->tail+1)%MAX_QUEUE_SIZE == queue->head)
      {
           return true;
      }
      return false;
   }
   return false;
}
bool lcp_is_queue_empty(struct lcp_pkg_queue *queue)
{
   if(queue != NULL)
   {
      if(queue->tail == queue->head)
      {
           return true;
      }
      return false;
   }
   return false;
}
bool lcp_control_cmd_set(struct lcp_control_t *control_cmd,uint8_t device_id,uint8_t *buffer,uint16_t buffer_len)
{
    if((buffer != NULL) &&(buffer_len!=0 && buffer_len <=MAX_PKG_SIZE) && (device_id < MAX_DEVICE_CNT) && control_cmd!=NULL)
    {
        control_cmd->buffer_new_flag[device_id] = 1;
        memcpy_ram(control_cmd->buffer[device_id],buffer,buffer_len);
        control_cmd->buffer_ack_flag[device_id] = 0;
        control_cmd->buffer_len[device_id]= buffer_len;
        return true;
    }
    return false;
}
bool lcp_control_cmd_clear(struct lcp_control_t *control_cmd,uint8_t device_id)
{
    if( (device_id < MAX_DEVICE_CNT) && control_cmd!=NULL)
    {
        control_cmd->buffer_new_flag[device_id] = 0;
        control_cmd->buffer_ack_flag[device_id] = 0;
        control_cmd->buffer_get_time[device_id] = 0;
        return true;
    }
    return false;
}
bool lcp_control_cmd_is_new(struct lcp_control_t *control_cmd,uint8_t device_id)
{
    if( (device_id < MAX_DEVICE_CNT) && control_cmd!=NULL)
    {
        if(control_cmd->buffer_new_flag[device_id])
        {
            return true;
        }
        return false;
    }
    return false;
}
uint8_t lcp_control_cmd_get_buffer(struct lcp_control_t *control_cmd,uint8_t device_id,uint8_t **buffer)
{
    
    if((control_cmd != NULL) && (buffer != NULL))
    {
        if(lcp_control_cmd_is_new(control_cmd,device_id))
        {
            *buffer=control_cmd->buffer[device_id];
            control_cmd->buffer_get_time[device_id]++;
            return control_cmd->buffer_len[device_id];
        }
        return 0;
    }
    return 0;
}
uint8_t lcp_control_cmd_get_get_buffer_time(struct lcp_control_t *control_cmd,uint8_t device_id)
{
    if((control_cmd != NULL))
    {
        if(lcp_control_cmd_is_new(control_cmd,device_id))
        {

            return control_cmd->buffer_get_time[device_id];

        }
        return 0;
    }
    return 0;
}
bool lcp_control_cmd_is_ack(struct lcp_control_t *control_cmd,uint8_t device_id)
{
    if((control_cmd != NULL))
    {
        if(lcp_control_cmd_is_new(control_cmd,device_id))
        {
            if(control_cmd->buffer_ack_flag[device_id] == 0)
            {
                return false;
            }
        }
        return true;
    }
    return true;
}


