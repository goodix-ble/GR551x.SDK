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
#include "lcp_hopping.h"
#include "lcp_pdu.h"
#include "ble_lcp.h"
#include "lcp_queue.h"

#define TIME_SYNC_TIMES_MS  50 //  100 /2 ms
#define CONNECT_TIME_OUT_MS 40 //50 ms   50=100/2
#define ACCESS_ADDRESS    0x6730363F
#define ACCESS_ADDRESS_2  0x8E89BED6//0x341bfd79
#define MASTER_AGTER_HOP_DELAY_US  45  //ms
#define PAYLOAD_LEN (2+4)
#define PAIR_MAGIC_WORDS   0x12345678

const uint8_t whiten_channel_idx[40]=
{
    22,23,24,25,26,27,28,29,30,31,
    32,34,35,36,37,38,39,40,41,42,
    43,44,45,46,47,48,49,50,51,52,
    53,54,55,56,57,58,59,21,33,60,
};
stack_time_t lcp_sync_time;
uint8_t device_id = 0;
uint8_t lcp_slave_timeslot = 0;
uint8_t lcp_payload[PAYLOAD_LEN]={0x12,0x22};
struct lcp_pkg_queue lcp_send_pkg_queue[2];
bool lcp_init_flag=false;
uint32_t lcp_system_time=0;
uint8_t  lcp_current_device_mode = 0;
uint32_t lcp_aa_table[2]={0,0};
pair_info_t lcp_pair_info;

uint8_t  lcp_current_on_line_flag[2]={0,0};
uint8_t  lcp_report_connect_ind_flag=0;
uint8_t  lcp_first_connect_flag[2]={0,0};   
uint32_t lcp_sync_times[2]={0,0};
uint32_t lcp_connect_timeout[2]={0,0};
uint8_t  lcp_recv_success_flag[2]={0,0};
uint32_t lcp_send_cal_cnt[2]={0,0};
uint32_t lcp_recv_cal_cnt[2]={0,0};
uint8_t lcp_default_pair_flag[2]={0,0};
struct lcp_sn_nesn_t lcp_sn_nesn[2];
uint8_t lcp_current_sync_state = 0;
int8_t lcp_pkg_rssi_values[2]={0,0};
uint32_t lcp_send_sync_end_offset[2] = {23810,23810};
uint32_t lcp_send_adv_end_offset[2] = {19900,19900};
bool lcp_prev_adv_connect_flag=false;
uint32_t lcp_new_aa=0;
bool lcp_is_default_paired_flag=false;
uint32_t lcp_slave_offline_time=0;
system_timestamp_t system_timestamp[2];
static uint8_t lcp_disconnect_flag[2]={0,0};

timesync_cbk_t timesync_cbk= NULL;
timesync_offset_get_cbk_t timesync_offset_get_cbk=NULL;
diag_debug_func_t diag_sync_io_set_func=NULL;
diag_debug_func_t diag_sync_io_reset_func=NULL;
diag_debug_func_t diag_tx_io_set_func=NULL;
diag_debug_func_t diag_tx_io_reset_func=NULL;
diag_debug_func_t diag_rx_io_set_func=NULL;
diag_debug_func_t diag_rx_io_reset_func=NULL;
diag_debug_func_t diag_1ms_io_set_func=NULL;
diag_debug_func_t diag_1ms_io_reset_func=NULL;
diag_debug_func_t diag_timesync_io_set_func=NULL;
diag_debug_func_t diag_timesync_io_reset_func=NULL;
diag_debug_func_t adv_io_set_func=NULL;
diag_debug_func_t adv_io_reset_func=NULL;
rx_handler_func_t rx_handler_func=NULL;
dis_connect_handler_func_t  dis_connect_handler_func=NULL;
connect_handler_func_t  connect_handler_func=NULL;
adv_report_handler_func_t adv_report_handler_func=NULL;
uint8_t turn_rx_flag =0;

extern stack_time_t lld_lcp_get_sync_aa_time(void);
extern void lld_aa_gen(uint8_t *acc_addr, uint8_t act_id);
extern uint32_t lld_lcp_get_whiten_seed_by_channel(uint8_t  ch_idx);
#define LCP_PRINTF(...) //printf( __VA_ARGS__)

#define SWDIAG_SYNC_IO_SET()  if(diag_sync_io_set_func) diag_sync_io_set_func()
#define SWDIAG_SYNC_IO_RESET()  if(diag_sync_io_reset_func) diag_sync_io_reset_func()
    
#define SWDIAG_TX_IO_SET()  if(diag_tx_io_set_func) diag_tx_io_set_func()
#define SWDIAG_TX_IO_RESET()  if(diag_tx_io_reset_func) diag_tx_io_reset_func()

#define SWDIAG_RX_IO_SET()  if(diag_rx_io_set_func) diag_rx_io_set_func()
#define SWDIAG_RX_IO_RESET()  if(diag_rx_io_reset_func) diag_rx_io_reset_func()
    
#define SWDIAG_1MS_IO_SET()  if(diag_1ms_io_set_func) diag_1ms_io_set_func()
#define SWDIAG_1MS_IO_RESET()  if(diag_1ms_io_reset_func) diag_1ms_io_reset_func()
    
#define SWDIAG_TIMESYNC_IO_SET()  if(diag_timesync_io_set_func) diag_timesync_io_set_func()
#define SWDIAG_TIMESYNC_IO_RESET()  if(diag_timesync_io_reset_func) diag_timesync_io_reset_func()

#define SWDIAG_ADV_IO_SET()  if(adv_io_set_func) adv_io_set_func()
#define SWDIAG_ADV_IO_RESET()  if(adv_io_reset_func) adv_io_reset_func()

#define RX_HANDLER_FUNC(aa)         if((rx_handler_func!=NULL) && (length > 2)) rx_handler_func(aa,p_payload+2,length-2)

#define SLVAE_RX_HANDLER_FUNC(aa)         if((rx_handler_func!=NULL) && (length > 0)) rx_handler_func(aa,p_payload,length)

#define DISCONNECT_HANDLER(aa)  if(dis_connect_handler_func) dis_connect_handler_func(aa)
#define CONNECT_HANDLER(device_id,aa)  if(connect_handler_func) connect_handler_func(device_id,aa)
#define ADV_REPORT_HANDLER(aa)  if(adv_report_handler_func) adv_report_handler_func(aa)


#define TIMER_SYSCNT_TO_US(X)     ((SystemCoreClock / 1000000) *(X) - 1)
#define TIMER_US_TO_SYSCNT(X)     (((X + 1))/(SystemCoreClock/1000000))


extern stack_time_t rwip_time_get(void) ;


void lcp_system_time_run(void)
{
    lcp_system_time++;
}

uint32_t lcp_get_system_time(void)
{
    return lcp_system_time;
}

system_timestamp_t* lcp_get_timestamp(uint8_t device_id)
{
    return &system_timestamp[device_id];
}

void lcp_disconnect(uint32_t aa)
{
    if(lcp_current_device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
        if(lcp_current_on_line_flag[0] == 1)
        {
            lcp_disconnect_flag[0] = 1;
        }
    }
    else
    {
        if(lcp_pair_info.pair_aa[0] == aa)
        {
            if(lcp_current_on_line_flag[0] == 1)
            {
                lcp_disconnect_flag[0] = 1;
            }
        }
        else if(lcp_pair_info.pair_aa[1] == aa)
        {
            if(lcp_current_on_line_flag[1] == 1)
            {
                lcp_disconnect_flag[1] = 1;
            }
        }
    }
}

void lcp_cbk_register(rx_handler_func_t rx_handler_cb,connect_handler_func_t  connect_handler_cb,dis_connect_handler_func_t  dis_connect_handler_cb)
{
    rx_handler_func                 = rx_handler_cb;
    connect_handler_func            = connect_handler_cb;
    dis_connect_handler_func        = dis_connect_handler_cb;
}

void lcp_adv_report_cbk_register(adv_report_handler_func_t adv_report_handler_cb)
{
    adv_report_handler_func                 = adv_report_handler_cb;
}

void lcp_debug_adv_cbk_register(diag_debug_func_t adv_set_cb,diag_debug_func_t adv_reset_cb)
{
    adv_io_set_func              = adv_set_cb;
    adv_io_reset_func            = adv_reset_cb;
}

void lcp_debug_timesync_cbk_register(diag_debug_func_t timesync_set_cb,diag_debug_func_t timesync_reset_cb)
{
    diag_timesync_io_set_func              = timesync_set_cb;
    diag_timesync_io_reset_func            = timesync_reset_cb;
}

void lcp_debug_1ms_cbk_register(diag_debug_func_t ms_set_cb,diag_debug_func_t ms_reset_cb)
{
    diag_1ms_io_set_func              = ms_set_cb;
    diag_1ms_io_reset_func            = ms_reset_cb;
}

void lcp_debug_sync_cbk_register(diag_debug_func_t sync_set_cb,diag_debug_func_t sync_reset_cb)
{
    diag_sync_io_set_func              = sync_set_cb;
    diag_sync_io_reset_func            = sync_reset_cb;
}

void lcp_debug_tx_cbk_register(diag_debug_func_t tx_set_cb,diag_debug_func_t tx_reset_cb)
{
    diag_tx_io_set_func              = tx_set_cb;
    diag_tx_io_reset_func            = tx_reset_cb;
}

void lcp_debug_rx_cbk_register(diag_debug_func_t rx_set_cb,diag_debug_func_t rx_reset_cb)
{
    diag_rx_io_set_func              = rx_set_cb;
    diag_rx_io_reset_func            = rx_reset_cb;
}

void lcp_timesync_cbk_register(timesync_cbk_t timesync_cb,timesync_offset_get_cbk_t timesync_offset_get_cb)
{
    timesync_cbk              = timesync_cb;
    timesync_offset_get_cbk   = timesync_offset_get_cb;
}

int8_t lcp_get_pkg_rssi_value(uint8_t device_id)
{
    if(lcp_current_on_line_flag[device_id]==1)
    {
        return lcp_pkg_rssi_values[device_id];
    }
    else
    {
        return 0;
    }
}

bool lcp_check_connect_time(uint32_t system_time,uint8_t rx_flag)
{
    uint8_t current_device_id = system_time%2;
    if(lcp_current_device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
         current_device_id = 0;
    }
    if(rx_flag == 1)
    {
         lcp_connect_timeout[current_device_id] = 0;
        return false;
    }
    else
    {
        if(lcp_current_device_mode == CURRENT_DEVICE_MODE_MASTER)
        {
            lcp_connect_timeout[current_device_id] =lcp_connect_timeout[current_device_id] +2;
        }
        else
        {
            lcp_connect_timeout[current_device_id] ++;
        }
        if(lcp_connect_timeout[current_device_id] >= CONNECT_TIME_OUT_MS)
        {
            lcp_connect_timeout[current_device_id] = 0;
            return true;
        }
    }
    return false;
}

bool lcp_is_current_device_online(uint8_t device_id)
{
    if(lcp_current_on_line_flag[device_id] == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

uint16_t lcp_send_data(uint32_t aa,uint8_t* data,uint16_t data_len,uint8_t max_retry_time)
{
    if(data ==NULL || data_len ==0)
    {
        return 0;
    }
    if(lcp_current_device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
        if(lcp_current_on_line_flag[0]!=1)
        {
            return 0;
        }
        if(lcp_queue_push(&lcp_send_pkg_queue[0],data,data_len,max_retry_time,lcp_system_time))
        {
            return data_len;
        }
    }
    else
    {
        if(aa == lcp_pair_info.pair_aa[0])
        {
            if(lcp_current_on_line_flag[0]!=1)
            {
                return 0;
            }
            if(lcp_queue_push(&lcp_send_pkg_queue[0],data,data_len,max_retry_time,lcp_system_time))
            {
                return data_len;
            }
        }
        else if(aa == lcp_pair_info.pair_aa[1])
        {
            if(lcp_current_on_line_flag[1]!=1)
            {
                return 0;
            }
            if(lcp_queue_push(&lcp_send_pkg_queue[1],data,data_len,max_retry_time,lcp_system_time))
            {
                return data_len;
            }
        }
    }
    return 0;
}

stack_time_t lcp_get_stack_time(void)
{
    stack_time_t stack_res ;
    stack_time_t res;
    res = rwip_time_get();
    stack_res.hs = res.hs;
    stack_res.hus = res.hus;
    return stack_res;
}

stack_time_t lcp_get_sync_time(void)
{
    return lcp_sync_time;
}

void lcp_aa_reset(void)
{
    gdx_lcp_rx_stop();
    gdx_lcp_access_address_set(lcp_get_default_aa(lcp_pair_info,0));
    gdx_lcp_rx_start();
}
void lcp_is_need_hop_2_comm_channel(void)
{
    if(lcp_current_on_line_flag[(lcp_system_time-1)%2] == 0)
    {
        if((lcp_sync_times[(lcp_system_time-1)%2] %(TIME_SYNC_TIMES_MS) == 0) )
        {
            lcp_hopping_2_comm_channel_by_index(0,lcp_get_current_hop_index(0));
        }
    }
}
void lcp_run(void)
{
    uint16_t length = 0;
    uint8_t *send_payload=NULL;

    lcp_system_time++;
    SWDIAG_1MS_IO_SET();

    device_id = lcp_system_time%2;
    SWDIAG_1MS_IO_RESET();
    if(lcp_init_flag == false)
        return;

    if(lcp_current_device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
        if(lcp_current_on_line_flag[0] == 1 || lcp_disconnect_flag[0] == 1)
        {
            if(lcp_check_connect_time(lcp_system_time,0)==true || lcp_disconnect_flag[0] == 1)
            {
                lcp_current_on_line_flag[0] = 0;
                if(lcp_current_sync_state != 0)
                {
                    lcp_current_sync_state--;
                }
                SWDIAG_ADV_IO_SET();
                gdx_lcp_rx_stop();
                lcp_hopping_2_adv_channel();
                gdx_lcp_rx_start();
                SWDIAG_ADV_IO_RESET();
                lcp_channel_level_reset(0);
                if(lcp_report_connect_ind_flag == 0)
                {
                    DISCONNECT_HANDLER(lcp_pair_info.pair_aa[0]);
                    lcp_disconnect_flag[0]=0;
                    lcp_slave_offline_time = lcp_system_time;
                }
                else
                {
                    if(lcp_is_default_paired_flag == false)
                    {
                        lcp_set_pair_info(0,0,0);
                        lcp_aa_reset();
                    }
                    else
                    {
                        lcp_is_default_paired_flag = false;
                    }
                }
            }
        }
        else
        {
            if(lcp_system_time - lcp_slave_offline_time > 100)
            {
                lcp_aa_reset();
                lcp_slave_offline_time = lcp_system_time;
            }
        }

        if (device_id != lcp_slave_timeslot && lcp_current_on_line_flag[0] == 1)
        {
            //SWDIAG_ADV_IO_SET();
            gdx_lcp_rx_stop();
            lcp_hopping_check(0);
            gdx_lcp_access_address_set(lcp_pair_info.pair_aa[0]);
            gdx_lcp_rx_start();
            //SWDIAG_ADV_IO_RESET();
            if(lcp_send_cal_cnt[0] == 0)
            {
                if(lcp_first_connect_flag[0] ==1)
                    lcp_first_connect_flag[0] = 0;
            }

            lcp_recv_cal_cnt[0]=0;
            lcp_send_cal_cnt[0]=0;
        }
    }
    else
    {
        // master need to stop rx first
        SWDIAG_TX_IO_SET();
        gdx_lcp_rx_stop();
        gdx_lcp_access_address_set(lcp_pair_info.pair_aa[device_id]);

        //check 2 slave status then check hop or not

        lcp_hopping_check(device_id);

        lcp_sync_times[device_id]++;

        // if current time is sync io or hopping,do not send time sync packet
        if((lcp_sync_times[device_id] %(TIME_SYNC_TIMES_MS) == 0) )//(lcp_current_state &(CURRENT_MASTER_STATE_HOP | CURRENT_MASTER_STATE_SYNC_IO))==0
        {
            if(lcp_current_on_line_flag[device_id] == 0)
            {
                uint32_t hop_map[LCP_MAP_SIZE];
                uint8_t used_ch_cnt[3]={0,0,0};
                uint16_t used_hop_map;
                SWDIAG_ADV_IO_SET();
                if(lcp_prev_adv_connect_flag == false)
                {
                    lcp_hopping_2_adv_channel();
                    send_payload=lcp_get_adv_payload(&length);
                    lcp_hop_map_2_hopping_map_bit(device_id,hop_map,LCP_MAP_SIZE);
                    lcp_used_hop_map_2_hopping_map_bit(device_id,&used_hop_map);
                    lcp_get_channel_cnt(device_id,used_ch_cnt);
                    lcp_sn_nesn[device_id].m_sn=0;
                    lcp_sn_nesn[device_id].m_nesn=0;
                    lcp_sn_nesn[device_id].s_sn_prev=1;
                    lcp_sn_nesn[device_id].m_nesn_prev=0;
                    if(timesync_offset_get_cbk!=NULL)
                    {
                        lcp_send_adv_end_offset[device_id] =  timesync_offset_get_cbk();
                    }

                    lcp_send_adv_end_offset[device_id] = lcp_send_adv_end_offset[device_id]-TIMER_SYSCNT_TO_US(409);
                    lcp_adv_pack(lcp_send_adv_end_offset[device_id],lcp_system_time,hop_map,LCP_MAP_SIZE,used_hop_map,used_ch_cnt[0],used_ch_cnt[1],used_ch_cnt[2],lcp_get_current_hop_index(device_id),send_payload,length);

                    gdx_lcp_data_tx(*(send_payload), length-1, send_payload+1);
                    #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
                    gdx_lcp_rx_start();
                    #else
                    turn_rx_flag = 1;
                    #endif
                }
                SWDIAG_ADV_IO_RESET();
                lcp_send_cal_cnt[device_id] =1;
                lcp_recv_cal_cnt[device_id] =0;
                lcp_prev_adv_connect_flag = false;

            }
            else
            {
                uint8_t retry_flag =0;
                send_payload=lcp_get_sync_payload(&length);
                if((lcp_sn_nesn[device_id].s_sn_prev == lcp_sn_nesn[device_id].s_sn) && (lcp_sn_nesn[device_id].s_nesn_prev == lcp_sn_nesn[device_id].s_nesn))
                {
                    retry_flag = 1;
                }
                else
                {
                    //get new buffer
                    lcp_sn_nesn[device_id].m_sn = (lcp_sn_nesn[device_id].m_sn+1)%2;
                    lcp_sn_nesn[device_id].m_nesn = (lcp_sn_nesn[device_id].m_nesn+1)%2;
                }
                if(retry_flag == 0)
                {
                    system_timestamp[device_id].hms = lcp_get_system_time();
                    if(timesync_offset_get_cbk != NULL)
                    {
                        system_timestamp[device_id].us = TIMER_US_TO_SYSCNT(timesync_offset_get_cbk());
                    }
                }

                lcp_sn_nesn[device_id].s_sn_prev = lcp_sn_nesn[device_id].s_sn;
                lcp_sn_nesn[device_id].s_nesn_prev = lcp_sn_nesn[device_id].s_nesn;
                if(timesync_offset_get_cbk!=NULL)
                {
                    lcp_send_sync_end_offset[device_id] =  timesync_offset_get_cbk();
                }
                lcp_send_sync_end_offset[device_id] = lcp_send_sync_end_offset[device_id]-TIMER_SYSCNT_TO_US(409);
                lcp_data_sync_pack(lcp_send_sync_end_offset[device_id],lcp_system_time,lcp_sn_nesn[device_id].m_sn,lcp_sn_nesn[device_id].m_nesn,send_payload,length);
                SWDIAG_TIMESYNC_IO_SET();
                gdx_lcp_data_tx(*(send_payload), length-1, send_payload+1);
                #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
                gdx_lcp_rx_start();
                #else
                turn_rx_flag = 1;
                #endif
                SWDIAG_TIMESYNC_IO_RESET();

            }

            lcp_send_cal_cnt[device_id]++;
            lcp_recv_success_flag[device_id] = 0;
        }
        else
        {
            uint8_t retry_flag =0;
            if(lcp_current_on_line_flag[device_id] == 1)
            {
                //get Payload 
                if((lcp_sn_nesn[device_id].s_sn_prev == lcp_sn_nesn[device_id].s_sn) && (lcp_sn_nesn[device_id].s_nesn_prev == lcp_sn_nesn[device_id].s_nesn))
                {
                   //retry
                    retry_flag = 1;
                }
                else
                {
                   //get new buffer
                    lcp_sn_nesn[device_id].m_sn = (lcp_sn_nesn[device_id].m_sn+1)%2;
                    lcp_sn_nesn[device_id].m_nesn = (lcp_sn_nesn[device_id].m_nesn+1)%2;
                }
                lcp_sn_nesn[device_id].s_sn_prev = lcp_sn_nesn[device_id].s_sn;
                lcp_sn_nesn[device_id].s_nesn_prev = lcp_sn_nesn[device_id].s_nesn;

                //if current time is sync io or hopping, just send null packet

                //get payload
                uint8_t *data_payload=NULL;

                length = lcp_queue_get_buffer(&lcp_send_pkg_queue[device_id],&data_payload);

                uint8_t update_index =0;
                uint8_t used_ch_cnt[3]={0,0,0};
                lcp_get_update_cmd(device_id,used_ch_cnt,&update_index);
                if(retry_flag == 0)
                {
                    system_timestamp[device_id].hms = lcp_get_system_time();
                    if(timesync_offset_get_cbk != NULL)
                    {
                        system_timestamp[device_id].us = TIMER_US_TO_SYSCNT(timesync_offset_get_cbk());
                    }
                }
                if(length == 0)//NO data to send   send null packet
                {
                    length=2;
                    lcp_data_head_pack(LCP_DATA_EMPTY,lcp_sn_nesn[device_id].m_sn,lcp_sn_nesn[device_id].m_nesn,lcp_payload,length);
                    lcp_data_update_cmd_pack(used_ch_cnt[0],used_ch_cnt[1],used_ch_cnt[2],update_index,lcp_payload+1,2);
                    gdx_lcp_data_tx(*(lcp_payload), length-1, lcp_payload+1);
                    #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
                    gdx_lcp_rx_start();
                    #else
                    turn_rx_flag = 1;
                    #endif
                }
                else
                {
                    lcp_data_head_pack(LCP_DATA,lcp_sn_nesn[device_id].m_sn,lcp_sn_nesn[device_id].m_nesn,data_payload,length);
                    lcp_data_update_cmd_pack(used_ch_cnt[0],used_ch_cnt[1],used_ch_cnt[2],update_index,data_payload+1,2);
                    gdx_lcp_data_tx(*(data_payload), length-1, data_payload+1);
                    #if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
                    gdx_lcp_rx_start();
                    #else
                    turn_rx_flag = 1;
                    #endif
                }
                lcp_queue_pop(&lcp_send_pkg_queue[device_id]);

                if(lcp_check_connect_time(lcp_system_time,lcp_recv_success_flag[device_id])==true || lcp_disconnect_flag[device_id] == 1)
                {
                    lcp_current_on_line_flag[device_id] = 0;
                    lcp_channel_level_reset(device_id);

                    DISCONNECT_HANDLER(lcp_pair_info.pair_aa[device_id]);
                    lcp_disconnect_flag[device_id] =0;
                    if(lcp_first_connect_flag[device_id] == 1)
                    {
                        if(lcp_default_pair_flag[device_id] == 0)
                        {
                            lcp_set_pair_info(device_id,0,0);
                        }
                    }
                }
                lcp_recv_success_flag[device_id] = 0;

                lcp_set_channel_recv_rate(device_id,lcp_get_prev_recv_channel(device_id),lcp_send_cal_cnt[device_id],lcp_recv_cal_cnt[device_id]);

                lcp_send_cal_cnt[device_id]=0;
                lcp_recv_cal_cnt[device_id]=0;
                lcp_send_cal_cnt[device_id]++;
            }
        }
        SWDIAG_TX_IO_RESET();
    }
}


uint32_t test_500us_cnt=0;
uint16_t lcp_data_rx_handler_callback(uint8_t header, uint8_t length, uint8_t *p_payload)
{
    uint16_t status = 0;
    uint8_t length_ack;
    uint8_t head_type;
    uint8_t *data_payload=NULL;
    SWDIAG_RX_IO_SET();
    SWDIAG_RX_IO_RESET();
    if(lcp_current_device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
        //delay_us(50);
        lcp_data_head_unpack(&head_type,&(lcp_sn_nesn[0].m_sn),&(lcp_sn_nesn[0].m_nesn),&header,length+1);
        switch(head_type)
        {
            case LCP_ADV:
            {
                uint32_t send_end_offset=0;
                uint8_t current_map_index;
                uint32_t hop_map[LCP_MAP_SIZE];
                uint8_t used_ch_cnt[3]={0,0,0};
                uint16_t used_hop_map;
                lcp_sync_time = lld_lcp_get_sync_aa_time();

                lcp_adv_unpack(&send_end_offset,&lcp_system_time,hop_map,LCP_MAP_SIZE,&used_hop_map,&used_ch_cnt[0],&used_ch_cnt[1],&used_ch_cnt[2],&current_map_index,p_payload,length);
                lcp_slave_timeslot = lcp_system_time%2;
                if(lcp_current_sync_state == 0)
                {
                    if(timesync_cbk != NULL)
                    {
                        timesync_cbk(send_end_offset);
                    }
                }
                if(lcp_current_sync_state > 0)
                {
                    gdx_lcp_rx_stop();

                    lcp_sn_nesn[0].s_sn=0;
                    lcp_sn_nesn[0].s_nesn=1;
                    lcp_sn_nesn[0].s_sn_prev=1;
                    lcp_sn_nesn[0].s_nesn_prev=0;
                    lcp_sn_nesn[0].m_sn_prev = lcp_sn_nesn[0].m_sn ;
                    lcp_sn_nesn[0].m_nesn_prev = lcp_sn_nesn[0].m_nesn;
                    SWDIAG_TX_IO_SET();
                    if(lcp_get_pair_flag(0)==0)
                    {
                        lcp_data_head_pack(LCP_CONNECT_IND,lcp_sn_nesn[0].s_sn,lcp_sn_nesn[0].s_nesn,lcp_payload,1);
                        memcpy_ram(lcp_payload+2,(uint8_t*)&lcp_new_aa,4);
                        gdx_lcp_data_tx(*lcp_payload, 1+4, lcp_payload+1);
                        lcp_set_pair_info(0,1,lcp_new_aa);
                        //printf("old new aa 0x%x\r\n",lcp_new_aa);
                    }
                    else
                    {
                        uint32_t new_aa;
                        new_aa = lcp_get_default_aa(lcp_pair_info,0);
                        lcp_data_head_pack(LCP_CONNECT_IND,lcp_sn_nesn[0].s_sn,lcp_sn_nesn[0].s_nesn,lcp_payload,1);
                        memcpy_ram(lcp_payload+2,(uint8_t*)&new_aa,4);
                        gdx_lcp_data_tx(*lcp_payload, 1+4, lcp_payload+1);
                        lcp_set_pair_info(0,1,new_aa);
                         //printf("new aa 0x%x\r\n",new_aa);
                    }


                    SWDIAG_TX_IO_RESET();
                    lcp_first_connect_flag[0] = 1;

                    lcp_hopping_map_bit_2_hop_map(0,hop_map,LCP_MAP_SIZE);
                    lcp_hopping_map_bit_2_used_hop_map(0,used_hop_map,used_ch_cnt);
                    lcp_set_current_hop_index(0,current_map_index) ;

                    //lcp_hopping_2_comm_channel_by_index(0,current_map_index);

                    //gdx_lcp_access_address_set(lcp_pair_info.pair_aa[0]);

                    if(lcp_current_on_line_flag[0]==0)
                    {
                        lcp_current_on_line_flag[0]=1;
                        lcp_report_connect_ind_flag = 1;
                    }
                }
                else
                {
                    //gdx_lcp_rx_stop();
                    //gdx_lcp_rx_start();
                    if(lcp_pair_info.pair_flag[0]==0)
                    {
                        ADV_REPORT_HANDLER();
                    }
                    else if(lcp_pair_info.pair_flag[0]==1)
                    {
                        lcp_set_sync_request();
                        lcp_is_default_paired_flag = true;
                    }
                }
                //SWDIAG_RX_IO_RESET();
            }
            break;
            case LCP_TIME_SYNC:
            {
                uint32_t send_end_offset=0;
                lcp_current_sync_state = 0;
                 //uint8_t retry_flag=0;
                lcp_sync_time = lld_lcp_get_sync_aa_time();
                gdx_lcp_rx_stop();
                SWDIAG_TIMESYNC_IO_SET();
                lcp_data_sync_unpack(&send_end_offset,&lcp_system_time,p_payload,length);

                SWDIAG_TIMESYNC_IO_RESET();
                if(lcp_current_on_line_flag[0]==1)
                {
                    if((lcp_sn_nesn[0].m_sn_prev == lcp_sn_nesn[0].m_sn) && (lcp_sn_nesn[0].m_nesn_prev == lcp_sn_nesn[0].m_nesn))
                    {
                       //retry
                       LCP_PRINTF("retry %d [%d,%d]\r\n",lcp_payload[1],lcp_sn_nesn[0].m_sn,lcp_sn_nesn[0].m_nesn);
                       length_ack = lcp_queue_get_buffer(&lcp_send_pkg_queue[0],&data_payload);
                    }
                    else
                    {
                        //get new buffer
                        lcp_sn_nesn[0].s_sn = (lcp_sn_nesn[0].s_sn+1)%2;
                        lcp_sn_nesn[0].s_nesn = (lcp_sn_nesn[0].s_nesn+1)%2;
                        length_ack = lcp_queue_get_buffer(&lcp_send_pkg_queue[0],&data_payload);
                    }
                    lcp_sn_nesn[0].m_sn_prev = lcp_sn_nesn[0].m_sn;
                    lcp_sn_nesn[0].m_nesn_prev = lcp_sn_nesn[0].m_nesn;
                    // get payload
                    SWDIAG_TX_IO_SET();
                    if(length_ack !=0)
                    {
                        lcp_data_head_pack(LCP_DATA,lcp_sn_nesn[0].s_sn,lcp_sn_nesn[0].s_nesn,data_payload,length_ack);
                        gdx_lcp_data_tx(*(data_payload), length_ack-3, data_payload+3);
                    }
                    else
                    {
                        length_ack=sizeof(lcp_payload);
                        lcp_data_head_pack(LCP_DATA_EMPTY,lcp_sn_nesn[0].s_sn,lcp_sn_nesn[0].s_nesn,lcp_payload,length_ack);
                        gdx_lcp_data_tx(*(lcp_payload), length_ack-1, lcp_payload+1);
                    }
                    if(timesync_cbk != NULL)
                    {
                        timesync_cbk(send_end_offset);
                    }
                    SWDIAG_TX_IO_RESET();
                    lcp_send_cal_cnt[0]++;
                    lcp_queue_pop(&lcp_send_pkg_queue[0]);

                    //SWDIAG_RX_IO_RESET();
                    lcp_check_connect_time(lcp_system_time,1);
                }
                else
                {
                    gdx_lcp_rx_start();
                    //SWDIAG_RX_IO_RESET();
                }
                LCP_PRINTF("sync_recv %d %d\r\n",send_end_offset,lcp_system_time);
            }
            break;
            case LCP_DATA:
            case LCP_DATA_EMPTY:
            {
                gdx_lcp_rx_stop();
                if(lcp_current_on_line_flag[0] == 1)
                {
                    if((lcp_sn_nesn[0].m_sn_prev == lcp_sn_nesn[0].m_sn) && (lcp_sn_nesn[0].m_nesn_prev == lcp_sn_nesn[0].m_nesn))
                    {
                        //retry
                        LCP_PRINTF("retry %d [%d,%d]\r\n",lcp_payload[1],lcp_sn_nesn[0].m_sn,lcp_sn_nesn[0].m_nesn);
                        length_ack = lcp_queue_get_buffer(&lcp_send_pkg_queue[0],&data_payload);
                    }
                    else
                    {
                        //get new buffer
                        lcp_sn_nesn[0].s_sn = (lcp_sn_nesn[0].s_sn+1)%2;
                        lcp_sn_nesn[0].s_nesn = (lcp_sn_nesn[0].s_nesn+1)%2;
                        length_ack = lcp_queue_get_buffer(&lcp_send_pkg_queue[0],&data_payload);
                    }
                    lcp_sn_nesn[0].m_sn_prev = lcp_sn_nesn[0].m_sn;
                    lcp_sn_nesn[0].m_nesn_prev = lcp_sn_nesn[0].m_nesn;
                    SWDIAG_TX_IO_SET();
                    if(length_ack !=0)
                    {
                        lcp_data_head_pack(LCP_DATA,lcp_sn_nesn[0].s_sn,lcp_sn_nesn[0].s_nesn,data_payload,length_ack);
                        gdx_lcp_data_tx(*(data_payload), length_ack-3, data_payload+3);
                    }
                    else
                    {
                        length_ack=sizeof(lcp_payload);
                        lcp_data_head_pack(LCP_DATA_EMPTY,lcp_sn_nesn[0].s_sn,lcp_sn_nesn[0].s_nesn,lcp_payload,length_ack);
                        gdx_lcp_data_tx(*(lcp_payload), length_ack-1, lcp_payload+1);
                    }
                    if(lcp_report_connect_ind_flag == 1)
                    {
                        CONNECT_HANDLER(0,lcp_pair_info.pair_aa[0]);
                        lcp_report_connect_ind_flag = 0;
                    }
                    SWDIAG_TX_IO_RESET();
                    lcp_send_cal_cnt[0]++;
                    uint8_t update_index =0;
                    uint8_t used_ch_cnt[3]  ={0,0,0};
                    lcp_data_update_cmd_unpack(&used_ch_cnt[0],&used_ch_cnt[1],&used_ch_cnt[2],&update_index,p_payload,2);
                    lcp_set_update_cmd(0,used_ch_cnt,update_index);
                    lcp_queue_pop(&lcp_send_pkg_queue[0]);
                    //SWDIAG_RX_IO_RESET();
                    lcp_check_connect_time(lcp_system_time,1);
                    RX_HANDLER_FUNC(lcp_pair_info.pair_aa[0]);
                    LCP_PRINTF("normal recv \r\n");
                }
                else
                {
                    gdx_lcp_rx_start();
                    //SWDIAG_RX_IO_RESET();
                }
            }
            break;
            default:
            {
                gdx_lcp_rx_stop();
                gdx_lcp_rx_start();
            }
            break;
        }
        lcp_recv_cal_cnt[0]++;
    }
    else
    {
        uint8_t temp_device_id = device_id;
        lcp_data_head_unpack(&head_type,&(lcp_sn_nesn[temp_device_id].s_sn),&(lcp_sn_nesn[temp_device_id].s_nesn),&header,length+1);
        //pop payload
        lcp_recv_success_flag[temp_device_id] = 1;
        lcp_recv_cal_cnt[temp_device_id]++;

        if(lcp_current_on_line_flag[temp_device_id] == 0)
        {
            if(head_type ==LCP_CONNECT_IND )
            {
                uint32_t new_aa=0;
                lcp_prev_adv_connect_flag = true;
                lcp_current_on_line_flag[temp_device_id]=1;
                lcp_first_connect_flag[temp_device_id] = 1;
                memcpy_ram((uint8_t*)&new_aa,p_payload+1,4);
                lcp_set_pair_info(temp_device_id,1,new_aa);
                SWDIAG_ADV_IO_SET();
                SWDIAG_ADV_IO_RESET();
                CONNECT_HANDLER(temp_device_id,lcp_pair_info.pair_aa[temp_device_id]);
            }
        }
        else
        {
            if(head_type == LCP_DATA)
            {
                SLVAE_RX_HANDLER_FUNC(lcp_pair_info.pair_aa[temp_device_id]);
                lcp_default_pair_flag[temp_device_id] = 1;
            }
            lcp_first_connect_flag[temp_device_id] =0;
        }
        //SWDIAG_RX_IO_RESET();
        LCP_PRINTF("recv %d,%d\r\n",lcp_sn_nesn[temp_device_id].s_sn,lcp_sn_nesn[temp_device_id].s_nesn);
    }
    return status;
}

uint8_t lcp_get_current_device_mode(void)
{
    return lcp_current_device_mode;
}

void lcp_set_sync_request(void)
{
    lcp_current_sync_state = 2;	
}

pair_info_t * lcp_get_pair_info(void)
{
    return &lcp_pair_info;
}

bool pair_info_cal_check_sum(uint8_t device_type,pair_info_t *p_pair_info)
{
    if(p_pair_info != NULL)
    {
        uint32_t check_sum=0;
        check_sum = p_pair_info->pair_flag[0] + p_pair_info->pair_flag[1]
                + p_pair_info->pair_aa[0] + p_pair_info->pair_aa[1]
                + p_pair_info->defalut_aa[0] + p_pair_info->defalut_aa[1]
                + p_pair_info->magic_words;
        if(p_pair_info->check_sum == check_sum)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool pair_info_check_sum_check(uint8_t device_type,pair_info_t *p_pair_info)
{
    if(p_pair_info != NULL)
    {
        uint32_t check_sum=0;
        if(device_type == CURRENT_DEVICE_MODE_MASTER)
        {
            if(p_pair_info->pair_aa[0] == p_pair_info->pair_aa[1])
            {
                return false;
            }
            if(p_pair_info->pair_flag[0]==0 || p_pair_info->pair_flag[1]==0)
            {
                return false;
            }
        }
        else
        {
            if(p_pair_info->pair_flag[0]==0)
            {
                return false;
            }
        }
        check_sum = p_pair_info->pair_flag[0] + p_pair_info->pair_flag[1]
                + p_pair_info->pair_aa[0] + p_pair_info->pair_aa[1]
                + p_pair_info->defalut_aa[0] + p_pair_info->defalut_aa[1]
                + p_pair_info->magic_words;
        if(p_pair_info->check_sum == check_sum)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

void lcp_pair_info_init(uint32_t aa0,uint32_t aa1)
{
    if(lcp_pair_info.magic_words != PAIR_MAGIC_WORDS)
    {
        lcp_pair_info.pair_flag[0] = 0;
        lcp_pair_info.pair_aa[0] = aa0;
        lcp_pair_info.defalut_aa[0] = aa0;
        lcp_pair_info.pair_flag[1] = 0;
        lcp_pair_info.pair_aa[1] = aa1;
        lcp_pair_info.defalut_aa[1] = aa1;
        lcp_pair_info.magic_words = PAIR_MAGIC_WORDS;
    }
    if(lcp_pair_info.pair_flag[0] == 0)
    {
        lcp_pair_info.pair_flag[0] = 0;
        lcp_pair_info.pair_aa[0] = aa0;
        lcp_pair_info.defalut_aa[0] = aa0;
    }
    if(lcp_pair_info.pair_flag[1] == 0)
    {
        lcp_pair_info.pair_flag[1] = 0;
        lcp_pair_info.pair_aa[1] = aa1;
        lcp_pair_info.defalut_aa[1] = aa1;
    }

    lcp_pair_info.check_sum = lcp_pair_info.pair_flag[0] + lcp_pair_info.pair_flag[1]
                            + lcp_pair_info.pair_aa[0] + lcp_pair_info.pair_aa[1]
                            + lcp_pair_info.defalut_aa[0] + lcp_pair_info.defalut_aa[1]
                            + lcp_pair_info.magic_words;
}

uint32_t lcp_get_default_aa(pair_info_t pair_info,uint8_t device_id)
{
    if(pair_info.pair_flag[device_id] == 1)
    {
        return pair_info.pair_aa[device_id];
    }
    else
    {
        return pair_info.defalut_aa[device_id];
    }
}

uint32_t lcp_get_pair_flag(uint8_t device_id)
{
    return lcp_pair_info.pair_flag[device_id];
}

void lcp_set_pair_info(uint8_t device_id,uint32_t pair_flag,uint32_t aa)
{
    if(pair_flag == 0)
    {
        lcp_pair_info.pair_aa[device_id] = lcp_pair_info.defalut_aa[device_id]; 
        lcp_pair_info.pair_flag[device_id] = 0;
        //printf("set aa =0x%x\r\n",lcp_pair_info.pair_aa[device_id]);
    }
    else
    {
        lcp_pair_info.pair_aa[device_id] = aa; 
        lcp_pair_info.pair_flag[device_id] = 1;
        //printf("set aa =0x%x\r\n",lcp_pair_info.pair_aa[device_id]);
    }
    lcp_pair_info.check_sum = lcp_pair_info.pair_flag[0] + lcp_pair_info.pair_flag[1]
                            + lcp_pair_info.pair_aa[0] + lcp_pair_info.pair_aa[1]
                            + lcp_pair_info.defalut_aa[0] + lcp_pair_info.defalut_aa[1]
                            + lcp_pair_info.magic_words;
}

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)

void gdx_lcp_data_tx_handler_callback()
{
    if(turn_rx_flag == 1)
    {
        gdx_lcp_rx_start();
        turn_rx_flag = 0;
    }
}
void gdx_lcp_data_rx_done_callback(uint8_t type)
{
    //printf("rx done %d \r\n",type);
}
uint16_t lcp_init(uint8_t device_mode,uint8_t tx_pwr,uint32_t freq)
{
    sdk_err_t error_code;
    gdx_lcp_config_t lcp_config;
    lcp_current_device_mode =  device_mode;
    uint32_t map_table[MAX_CHANNEL_TABLE_SIZE] ;
    memset(lcp_sn_nesn,0,sizeof(lcp_sn_nesn));
    lcp_queue_init(&lcp_send_pkg_queue[0]);
    lcp_queue_init(&lcp_send_pkg_queue[1]);
    if(device_mode == CURRENT_DEVICE_MODE_MASTER)
    {
        lcp_config.trx_mode = LCP_TRX_MODE_SW_TX;
        printf("aa[0] =0x%x pair flag=%d\r\n",lcp_get_default_aa(lcp_pair_info,0),lcp_pair_info.pair_flag[0]);
        printf("aa[1] =0x%x pair flag=%d\r\n",lcp_get_default_aa(lcp_pair_info,1),lcp_pair_info.pair_flag[1]);
        lcp_default_pair_flag[0] = lcp_pair_info.pair_flag[0];
        lcp_default_pair_flag[1] = lcp_pair_info.pair_flag[1];
    }
    else
    {
        lcp_config.trx_mode = LCP_TRX_MODE_SW_RX;
        lld_aa_gen((uint8_t*)&lcp_new_aa,0);
        printf("aa =0x%x pair flag=%d\r\n",lcp_get_default_aa(lcp_pair_info,0),lcp_pair_info.pair_flag[0]);
        lcp_default_pair_flag[0] = lcp_pair_info.pair_flag[0];
    }
    lcp_hopping_set_default_adv_channel(freq);
    printf("freq =%d\r\n",freq);
    lcp_config.txpwr_dbm = tx_pwr;
    lcp_config.freq_mhz = freq;
    lcp_config.access_address = lcp_get_default_aa(lcp_pair_info,0);
    lcp_config.crc_init = CRC_INIT;
    lcp_config.rx_handler_cb = lcp_data_rx_handler_callback;
    lcp_config.tx_done_cb = gdx_lcp_data_tx_handler_callback;
    lcp_config.rx_done_cb = gdx_lcp_data_rx_done_callback;
    lcp_config.rate = LCP_RATE_2MBPS;
    lcp_config.rx_window_size_us = 0;
    lcp_config.trx_timer_period_us = 0;
    lcp_config.trx_timer_trigger_trx_time_us = 0;
    lcp_config.b_disable_rx_oneshot_mode = false;
    lcp_config.whiten_en = false;

    error_code = gdx_lcp_init(&lcp_config);
    gdx_lcp_whitening_seed_set(lld_lcp_get_whiten_seed_by_channel(whiten_channel_idx[CHANNEL_IDX] ));
    if(device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
        //gdx_lcp_rx_start();
        lcp_channel_level_reset( 0);
        lcp_current_on_line_flag[0]=0;
        lcp_slave_offline_time=0;
    }
    else
    {
        lcp_all_channel_scan(freq,map_table,3+6);
        lcp_hopping_init(map_table,3,6);
        lcp_current_on_line_flag[0]=0;
        lcp_current_on_line_flag[1]=0;
    }
    lcp_init_flag = true;
    return error_code;
}
#else
uint16_t lcp_init(uint8_t device_mode,uint8_t tx_pwr,uint32_t freq)
{
    sdk_err_t error_code;
    gdx_lcp_config_t lcp_config;
    lcp_current_device_mode =  device_mode;
    uint32_t map_table[MAX_CHANNEL_TABLE_SIZE] ;
    memset(lcp_sn_nesn,0,sizeof(lcp_sn_nesn));
    lcp_queue_init(&lcp_send_pkg_queue[0]);
    lcp_queue_init(&lcp_send_pkg_queue[1]);
    if(device_mode == CURRENT_DEVICE_MODE_MASTER)
    {
        lcp_config.mode = LCP_TX;
        printf("aa[0] =0x%x pair flag=%d\r\n",lcp_get_default_aa(lcp_pair_info,0),lcp_pair_info.pair_flag[0]);
        printf("aa[1] =0x%x pair flag=%d\r\n",lcp_get_default_aa(lcp_pair_info,1),lcp_pair_info.pair_flag[1]);
        lcp_default_pair_flag[0] = lcp_pair_info.pair_flag[0];
        lcp_default_pair_flag[1] = lcp_pair_info.pair_flag[1];
    }
    else
    {
        lcp_config.mode = LCP_RX;
        lld_aa_gen((uint8_t*)&lcp_new_aa,0);
        printf("aa =0x%x pair flag=%d\r\n",lcp_get_default_aa(lcp_pair_info,0),lcp_pair_info.pair_flag[0]);
        lcp_default_pair_flag[0] = lcp_pair_info.pair_flag[0];
    }
    lcp_hopping_set_default_adv_channel(freq);
    printf("freq =%d\r\n",freq);
    lcp_config.txpwr_dbm = tx_pwr;
    lcp_config.freq = freq;
    lcp_config.ch_idx = CHANNEL_IDX;
    lcp_config.access_address = lcp_get_default_aa(lcp_pair_info,0);
    lcp_config.crc_init = CRC_INIT;
    lcp_config.rx_handler_cb = &lcp_data_rx_handler_callback;
    gdx_lcp_rate_set(1);
    error_code = gdx_lcp_init(&lcp_config);

    if(device_mode == CURRENT_DEVICE_MODE_SLAVE)
    {
        gdx_lcp_rx_start();
        lcp_channel_level_reset( 0);
        lcp_current_on_line_flag[0]=0;
        lcp_slave_offline_time=0;
    }
    else
    {
        lcp_all_channel_scan(freq,map_table,3+6);
        lcp_hopping_init(map_table,3,6);
        lcp_current_on_line_flag[0]=0;
        lcp_current_on_line_flag[1]=0;
    }
    lcp_init_flag = true;
    return error_code;
}
#endif

void lcp_deinit(void)
{
    gdx_lcp_deinit();
    lcp_init_flag = false;
}

uint16_t lcp_hopping_init(uint32_t *hopping_map,uint8_t hopping_map_cnt,uint8_t standby_hopping_map_cnt)
{
     return lcp_hopping_map_init(hopping_map,hopping_map_cnt,standby_hopping_map_cnt);
}

