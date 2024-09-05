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
#include "lcp_queue.h"
#include "ble_lcp.h"
//#include "custom_config.h"
#include "lcp_hopping.h"
//#include "flash_scatter_config.h"
//#include "app_log.h"
//#include "app_error.h"
#include "ble_error.h"
#define READ_TIME 100
#define CHOSE_CHANNEL_MOD 3
//uint32_t temp_channel[MAX_CHANNEL_TABLE_SIZE];
uint32_t lcp_hop_map[2][MAX_CHANNEL_TABLE_SIZE];
uint32_t lcp_hop_map_level[2][MAX_CHANNEL_TABLE_SIZE];
//uint8_t  lcp_hop_update_time[2][MAX_CHANNEL_TABLE_SIZE];
//uint8_t  lcp_hop_update_flag[2][MAX_CHANNEL_TABLE_SIZE];
uint16_t lcp_used_hop_map_bit[2]={0,0};
uint32_t lcp_standby_hop_map[2][MAX_CHANNEL_TABLE_SIZE];
uint32_t lcp_standby_hop_map_level[2][MAX_CHANNEL_TABLE_SIZE];
struct lcp_channel_recv_rate_t lcp_channel_recv_rate[2][TOTAL_CHANNEL_CNT];

uint8_t  lcp_hop_ch_cnt[2]={0,0};
uint8_t  lcp_standby_hop_ch_cnt[2]={0,0};
uint8_t  lcp_current_hop_index[2]={0,0};
uint8_t  lcp_current_standby_hop_index[2]={0,0};
lcp_hop_channel_map_t lcp_hopping_channel_map[2];
//uint8_t  lcp_hop_update_flag=0;
uint32_t lcp_recv_ch[2]={0,0};
uint32_t lcp_prev_recv_ch[2]={0,0};
uint32_t lcp_default_adv_channel=2402;
//bool lcp_set_hop_flag[2]={false,false};

//bool     lcp_scan_freq_table_flush_flag=false;
diag_debug_func_t diag_hopping_io_set_func=NULL;
diag_debug_func_t diag_hopping_io_reset_func=NULL;
diag_debug_func_t diag_hop_change_io_set_func=NULL;
diag_debug_func_t diag_hop_change_io_reset_func=NULL;
extern diag_debug_func_t diag_1ms_io_set_func;
extern diag_debug_func_t diag_1ms_io_reset_func;
#define SWDIAG_HOPPING_IO_SET()  if(diag_hopping_io_set_func) diag_hopping_io_set_func()
#define SWDIAG_HOPPING_IO_RESET()  if(diag_hopping_io_reset_func) diag_hopping_io_reset_func()
#define SWDIAG_HOPCHANGE_IO_SET()  if(diag_hop_change_io_set_func) diag_hop_change_io_set_func()
#define SWDIAG_HOPCHANGE_IO_RESET()  if(diag_hop_change_io_reset_func) diag_hop_change_io_reset_func()

#define SWDIAG_1MS_IO_SET()  if(diag_1ms_io_set_func) diag_1ms_io_set_func()
#define SWDIAG_1MS_IO_RESET()  if(diag_1ms_io_reset_func) diag_1ms_io_reset_func()

void sort(uint32_t *freq,uint8_t size);
void sort_level(uint32_t *freq,uint32_t *level,uint8_t size);
void lcp_debug_hopping_cbk_register(diag_debug_func_t hopping_set_cb,diag_debug_func_t hopping_reset_cb)
{
    diag_hopping_io_set_func              = hopping_set_cb;
    diag_hopping_io_reset_func            = hopping_reset_cb;
}
void lcp_debug_hop_change_cbk_register(diag_debug_func_t hop_change_set_cb,diag_debug_func_t hop_change_reset_cb)
{
    diag_hop_change_io_set_func              = hop_change_set_cb;
    diag_hop_change_io_reset_func            = hop_change_reset_cb;
}
uint16_t test_channel[1024];
uint16_t test_index = 0;

void test_channel_record(uint16_t channel)
{
    if((channel<2402 || channel > 2500) && channel!=0   )
    {
        while(1);
    }
    test_channel[test_index]  =  channel;
    test_index++;
    if(test_index == 1024)
        test_index = 0;
}
 void lcp_get_channel_cnt(uint8_t device_id,uint8_t *ch_index)
{
	uint8_t i =0,j=0;
	uint8_t used_index=0;
	for(j=0;j<lcp_hop_ch_cnt[device_id];j++)
	{
		for(i=0;i<LCP_CHANNEL_CNT_USED_TO_HOP;i++)
		{
			if(lcp_hopping_channel_map[device_id].channel_map[i] == lcp_hop_map[device_id][j])
			{
				ch_index[used_index] = i;
				used_index++;
				break;
			}
		}
	}
}


 uint8_t lcp_get_current_hop_index(uint8_t device_id)
{
    return  lcp_current_hop_index[device_id];
}
 uint8_t lcp_get_current_standby_hop_index(uint8_t device_id)
{
    return  lcp_current_standby_hop_index[device_id];
}

 void lcp_set_current_hop_index(uint8_t device_id,uint8_t hop_map_index)
{
    lcp_current_hop_index[device_id] = hop_map_index;
}
 void lcp_set_current_standby_hop_index(uint8_t device_id,uint8_t hop_map_index)
{
    lcp_current_standby_hop_index[device_id] = hop_map_index;
}

 bool lcp_hopping_map_bit_2_hop_map(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size)
{
    uint8_t i ,j,used_channel_index;
    used_channel_index = 0;
    if(hop_map_bit_size == LCP_MAP_SIZE && hop_map_bit!=NULL)
    {
        for(j=0;j<LCP_MAP_SIZE;j++)
        {
            for(i=0;i<32;i++)
            {
                if(((*(hop_map_bit+j)>>i)&0x01) == 0x01)
                {
                    //lcp_hop_map[device_id][used_channel_index]= LCP_START_FREQ+ (i + j*32)*LCP_USE_FREQ_MHZ;
					lcp_hopping_channel_map[device_id].channel_map[used_channel_index]= LCP_START_FREQ+ (i + j*32)*LCP_USE_FREQ_MHZ;
                    used_channel_index++;
                    if(used_channel_index >=MAX_CHANNEL_TABLE_SIZE)
                    {
                        //lcp_hop_ch_cnt[device_id] = used_channel_index;
                        return true;
                    }
                }
            }
        }
        //lcp_hop_ch_cnt[device_id] = used_channel_index;
        return true;
    }
    return false;
}

uint16_t test_map_table=0;
 bool lcp_hopping_map_bit_2_used_hop_map(uint8_t device_id,uint16_t hop_map_bit,uint8_t *used_ch_cnt)
{
    uint8_t i ,j,used_channel_index;
    used_channel_index = 0;
	test_map_table = hop_map_bit;
    if(hop_map_bit != 0 && used_ch_cnt !=NULL)
    {
        for(j=0;j<LCP_CHANNEL_CNT_USED_TO_HOP;j++)
        {
			if(((hop_map_bit>>j)&0x01) == 0x01)
			{
				lcp_hopping_channel_map[device_id].channel_used_flag[j]=1;
				lcp_used_hop_map_bit[device_id]=lcp_used_hop_map_bit[device_id]|(0x1<<j);
				//lcp_hop_map[device_id][used_channel_index] = lcp_hopping_channel_map[device_id].channel_map[j];
				used_channel_index++;
			}
        }
		for(i=0;i<lcp_hop_ch_cnt[device_id];i++)
		{
			lcp_hop_map[device_id][i] = lcp_hopping_channel_map[device_id].channel_map[used_ch_cnt[i]];
		}
		lcp_hop_ch_cnt[device_id] = used_channel_index;
        return true;
    }
    return false;
}

//uint16_t temp_test[4][255];
//uint8_t  temp_test_index = 0;

 void lcp_set_channel_recv_rate(uint8_t device_id,uint32_t freq,uint16_t send_cnt,uint16_t recv_cnt)
{
    #if 1
    static uint16_t recv_rate_cal_cnt[2]={0,0};
    
    if(freq >= LCP_START_FREQ && freq<=LCP_END_FREQ)
    {
        bool all_0_flag=false;
        //bool sent_update_time_flag=true;
        uint8_t i;
        uint8_t channel =     (freq-LCP_START_FREQ)/LCP_USE_FREQ_MHZ;
        recv_rate_cal_cnt[device_id]++;
        //test_channel_record(freq);

        if(send_cnt == 0 && recv_cnt==0)
        {
            all_0_flag = true;
        }
        //lcp_channel_recv_rate[device_id][channel].recv_rate = (recv_cnt*100)/send_cnt;
        //if(lcp_channel_recv_rate[device_id][channel].recv_rate !=100)
        if((all_0_flag == true)||(send_cnt!=recv_cnt))
        {
            //SWDIAG_1MS_IO_SET();
            
            //lcp_channel_recv_rate[device_id][channel].ch_level=lcp_channel_recv_rate[device_id][channel].ch_level - 30;
            lcp_channel_recv_rate[device_id][channel].ch_level--;
            //lcp_channel_recv_rate[device_id][channel].total_level=lcp_channel_recv_rate[device_id][channel].total_level - 30;
            //SWDIAG_1MS_IO_RESET();
            lcp_channel_recv_rate[device_id][channel].send_cnt++;
            //lcp_channel_recv_rate[device_id][channel].recv_cnt++;

        }
        else
        {
            //lcp_channel_recv_rate[device_id][channel].total_level++;
           // if(lcp_channel_recv_rate[device_id][channel].total_level >=0)
           //     lcp_channel_recv_rate[device_id][channel].total_level = 0;
            lcp_channel_recv_rate[device_id][channel].ch_level = 0;
            lcp_channel_recv_rate[device_id][channel].send_cnt++;
            lcp_channel_recv_rate[device_id][channel].recv_cnt++;
        }
#if 0
        if(lcp_just_hop_flag[device_id] ==1)
        {
            for(i=0;i<MAX_CHANNEL_TABLE_SIZE;i++)
            {
                lcp_channel_recv_rate[device_id][i].ch_level=0;
            }
        }
        #endif
        if(lcp_channel_recv_rate[device_id][channel].ch_level <= -6 )
        {
            //replace channel
			SWDIAG_HOPCHANGE_IO_SET();
			//lcp_channel_recv_rate[device_id][channel].ch_level = 10;
            for(i=0;i<lcp_hop_ch_cnt[device_id];i++)
            {
                if(lcp_hop_map[device_id][i] == freq)
                {
					uint16_t new_map=0;
					SWDIAG_HOPCHANGE_IO_RESET();
					
					# if 0
					uint8_t current_standby_hop_index=lcp_get_current_standby_hop_index(device_id);
					
					current_standby_hop_index=(current_standby_hop_index+1)%lcp_standby_hop_ch_cnt[device_id];
					lcp_hop_map[device_id][i] = lcp_standby_hop_map[device_id][current_standby_hop_index];
					
					lcp_standby_hop_map[device_id][current_standby_hop_index] = freq;
					
					lcp_set_current_standby_hop_index(device_id,current_standby_hop_index);
					#else
					sort_level(lcp_standby_hop_map[device_id],lcp_standby_hop_map_level[device_id],lcp_standby_hop_ch_cnt[device_id]);
					lcp_hop_map[device_id][i] = lcp_standby_hop_map[device_id][0];
					lcp_standby_hop_map[device_id][0] = freq;
					
					uint32_t temp_level;
					lcp_hop_map_level[device_id][i] = lcp_hop_map_level[device_id][i]  - 10;
					temp_level = lcp_hop_map_level[device_id][i]; 
					lcp_hop_map_level[device_id][i] = lcp_standby_hop_map_level[device_id][0]; 
					lcp_standby_hop_map_level[device_id][0] = temp_level;
					#endif
					for(uint8_t j =0;j<LCP_CHANNEL_CNT_USED_TO_HOP ;j++)
					{
						lcp_hopping_channel_map[device_id].channel_used_flag[j] = 0;
					}
					for(uint8_t idx=0;idx<lcp_hop_ch_cnt[device_id];idx++)
					{
						for(uint8_t j =0;j<LCP_CHANNEL_CNT_USED_TO_HOP ;j++)
						{
							if(lcp_hop_map[device_id][idx] == lcp_hopping_channel_map[device_id].channel_map[j])
							{
								lcp_hopping_channel_map[device_id].channel_used_flag[j] = 1;
								break;
							}
						}
					}
					SWDIAG_HOPCHANGE_IO_SET();
					lcp_used_hop_map_2_hopping_map_bit(device_id,&new_map);
					//lcp_hoptable_replace(device_id);
					recv_rate_cal_cnt[device_id] = 0;
                    break;
                }
            }
			for(i=0;i<MAX_CHANNEL_TABLE_SIZE;i++)
			{
				lcp_channel_recv_rate[device_id][i].ch_level = 0;
                                    lcp_channel_recv_rate[device_id][i].send_cnt = 0;
                    lcp_channel_recv_rate[device_id][i].recv_cnt = 0;
                    lcp_channel_recv_rate[device_id][i].recv_rate = 0;
			}
			SWDIAG_HOPCHANGE_IO_RESET();
        }
        #if 1
        else 
        {
            uint16_t bad_channel_idx=0xFFFF;
            if(recv_rate_cal_cnt[device_id] >= 1000)
            {
                recv_rate_cal_cnt[device_id] = 0;
                for(i=0;i<MAX_CHANNEL_TABLE_SIZE;i++)
                {
                    if(lcp_channel_recv_rate[device_id][i].send_cnt != 0)
                    {
                        lcp_channel_recv_rate[device_id][i].recv_rate = (lcp_channel_recv_rate[device_id][i].recv_cnt*1000)/lcp_channel_recv_rate[device_id][i].send_cnt;
                        if(lcp_channel_recv_rate[device_id][i].recv_rate <940)
                        {
                        //SWDIAG_1MS_IO_SET();
                            bad_channel_idx = i;
                            break;
                        }
                    }
                }
                if(bad_channel_idx !=0xFFFF)
                {
                    freq =LCP_START_FREQ+ (bad_channel_idx*LCP_USE_FREQ_MHZ);
                    for(i=0;i<lcp_hop_ch_cnt[device_id];i++)
                                {
                                    if(lcp_hop_map[device_id][i] == freq)
                                    {
                                        uint16_t new_map=0;
                                        //SWDIAG_HOPCHANGE_IO_SET();
										SWDIAG_1MS_IO_SET();
										#if 0
										uint8_t current_standby_hop_index=lcp_get_current_standby_hop_index(device_id);
                                        
                                        current_standby_hop_index=(current_standby_hop_index+1)%lcp_standby_hop_ch_cnt[device_id];
                                        lcp_hop_map[device_id][i] = lcp_standby_hop_map[device_id][current_standby_hop_index];
                                        
                                        lcp_standby_hop_map[device_id][current_standby_hop_index] = freq;
                                        
                                        lcp_set_current_standby_hop_index(device_id,current_standby_hop_index);
										#else
                                        sort_level(lcp_standby_hop_map[device_id],lcp_standby_hop_map_level[device_id],lcp_standby_hop_ch_cnt[device_id]);
                                        
                                        lcp_hop_map[device_id][i] = lcp_standby_hop_map[device_id][0];
                                        
                                        lcp_standby_hop_map[device_id][0] = freq;
                                        
                                        
										uint32_t temp_level;
										lcp_hop_map_level[device_id][i] = lcp_hop_map_level[device_id][i]  - 1;
										temp_level = lcp_hop_map_level[device_id][i]; 
										lcp_hop_map_level[device_id][i] = lcp_standby_hop_map_level[device_id][0]; 
										lcp_standby_hop_map_level[device_id][0] = temp_level;
										#endif
                                        for(uint8_t j =0;j<LCP_CHANNEL_CNT_USED_TO_HOP ;j++)
                                        {
                                            lcp_hopping_channel_map[device_id].channel_used_flag[j] = 0;
                                        }
                                        for(uint8_t idx=0;idx<lcp_hop_ch_cnt[device_id];idx++)
                                        {
                                            for(uint8_t j =0;j<LCP_CHANNEL_CNT_USED_TO_HOP ;j++)
                                            {
                                                if(lcp_hop_map[device_id][idx] == lcp_hopping_channel_map[device_id].channel_map[j])
                                                {
                                                    lcp_hopping_channel_map[device_id].channel_used_flag[j] = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        //SWDIAG_HOPCHANGE_IO_RESET();
										SWDIAG_1MS_IO_RESET();
                                        lcp_used_hop_map_2_hopping_map_bit(device_id,&new_map);
                                        for(i=0;i<MAX_CHANNEL_TABLE_SIZE;i++)
                                        {
                                            lcp_channel_recv_rate[device_id][i].ch_level = 0;
                                        }
                                        //lcp_hoptable_replace(device_id);
                                        break;
                                    }
                                }


                }
                for(i=0;i<MAX_CHANNEL_TABLE_SIZE;i++)
                {
                    //lcp_channel_recv_rate[device_id][i].ch_level = 0;
                    lcp_channel_recv_rate[device_id][i].send_cnt = 0;
                    lcp_channel_recv_rate[device_id][i].recv_cnt = 0;
                    lcp_channel_recv_rate[device_id][i].recv_rate = 0;
                }

            }
        }
        #endif
    }
    #endif
}
 bool lcp_get_update_cmd(uint8_t device_id,uint8_t *used_ch_cnt,uint8_t *update_index )
{
//    uint8_t i;
    if(update_index!=NULL && used_ch_cnt!=NULL)
    {
		lcp_get_channel_cnt(device_id,used_ch_cnt);
		//*channel_map = lcp_used_hop_map_bit[device_id];
		*update_index = lcp_get_current_hop_index(device_id);
//		temp_test[0][temp_test_index] = used_ch_cnt[0];
//		temp_test[1][temp_test_index] = used_ch_cnt[1];
//		temp_test[2][temp_test_index] = used_ch_cnt[2];
//		temp_test[3][temp_test_index] = lcp_get_system_time();
//		temp_test_index++;
		return true;
    }
    return false;
}



 bool lcp_set_update_cmd(uint8_t device_id,uint8_t *used_ch_cnt,uint8_t update_index )
{
    if(used_ch_cnt !=NULL )
    {
//		temp_test[0][temp_test_index]=used_map_bit;
//		temp_test[1][temp_test_index]=lcp_used_hop_map_bit[device_id];
//		temp_test_index++;
		lcp_set_current_hop_index(device_id,update_index);
		lcp_hop_map[device_id][0]=lcp_hopping_channel_map[device_id].channel_map[used_ch_cnt[0]];
		lcp_hop_map[device_id][1]=lcp_hopping_channel_map[device_id].channel_map[used_ch_cnt[1]];
		lcp_hop_map[device_id][2]=lcp_hopping_channel_map[device_id].channel_map[used_ch_cnt[2]];
//		temp_test[0][temp_test_index] = used_ch_cnt[0];
//		temp_test[1][temp_test_index] = used_ch_cnt[1];
//		temp_test[2][temp_test_index] = used_ch_cnt[2];
//		temp_test[3][temp_test_index] = lcp_get_system_time();
//		temp_test_index++;
//		if(lcp_used_hop_map_bit[device_id] != used_map_bit)
//		{
//			SWDIAG_HOPCHANGE_IO_SET();
//			uint8_t used_index=0;
//			lcp_set_current_hop_index(device_id,update_index);
//			lcp_used_hop_map_bit[device_id] = used_map_bit;
//			for(uint8_t i = 0;i<LCP_CHANNEL_CNT_USED_TO_HOP;i++)
//			{
//				if(((used_map_bit>>i)&0x01) == 0x01)
//				{
//					lcp_hopping_channel_map[device_id].channel_used_flag[i] = 1;
//					lcp_hop_map[device_id][used_index] = lcp_hopping_channel_map[device_id].channel_map[i];
//					used_index++;
//				}
//				else
//				{
//					lcp_hopping_channel_map[device_id].channel_used_flag[i] = 0;
//				}
//			}
//			//sort(&lcp_hop_map[device_id][0],lcp_hop_ch_cnt[device_id]);
//			SWDIAG_HOPCHANGE_IO_RESET();
//		}
		
    }
    else
    {
		printf("0");
        #if 0
        if(set_hop_flag == 1)
        {
            uint16_t i=0;
            for(i=0;i<1024;i++)
            {
                printf("%d %d %d\r\n",testindex,testhop[i][0],testhop[i][1]);
            }
        }
        #endif
    }
    return true ;

}


void lcp_channel_level_reset(uint8_t device_id)
{
    memset(&lcp_channel_recv_rate[device_id][0],0,sizeof(struct lcp_channel_recv_rate_t)*MAX_CHANNEL_TABLE_SIZE);
    //memset(&lcp_hop_update_time[device_id][0],0,MAX_CHANNEL_TABLE_SIZE);
    //memset(&lcp_hop_update_flag[device_id][0],0,MAX_CHANNEL_TABLE_SIZE);
}
 bool lcp_hop_map_2_hopping_map_bit(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size)
{
    uint8_t i ,used_channel_index;
    used_channel_index = 0;
    if(hop_map_bit_size == LCP_MAP_SIZE && hop_map_bit!=NULL)
    {
        hop_map_bit[0]=0;
        hop_map_bit[1]=0;
        for(i=0;i<LCP_CHANNEL_CNT_USED_TO_HOP;i++)
        {
            used_channel_index =  (lcp_hopping_channel_map[device_id].channel_map[i]- LCP_START_FREQ)/LCP_USE_FREQ_MHZ;
            if(used_channel_index < 32)
            {
                hop_map_bit[0] = hop_map_bit[0] | (0x01 << used_channel_index) ;
            }
            else
            {
                hop_map_bit[1] = hop_map_bit[1] | (0x01 << (used_channel_index -32)) ;
            }
        }
        return true;
    }
    return false;
}
 bool lcp_used_hop_map_2_hopping_map_bit(uint8_t device_id,uint16_t *hop_map_bit)
{
    uint8_t i;
    //used_channel_index = 0;
    if(hop_map_bit!=NULL)
    {
		*hop_map_bit = 0;
        for(i=0;i<LCP_CHANNEL_CNT_USED_TO_HOP;i++)
        {
            if(lcp_hopping_channel_map[device_id].channel_used_flag[i] == 1)
			{
			    *hop_map_bit = *hop_map_bit | (1<<i);
			}
        }
		lcp_used_hop_map_bit[device_id] = *hop_map_bit ;
        return true;
    }
    return false;
}
//SECTION_RAM_CODE bool lcp_standby_hop_map_2_hopping_map_bit(uint8_t device_id,uint32_t *hop_map_bit,uint8_t hop_map_bit_size)
//{
//    uint8_t i ,used_channel_index;
//    used_channel_index = 0;
//    if(hop_map_bit_size == LCP_MAP_SIZE && hop_map_bit!=NULL)
//    {
//        hop_map_bit[0]=0;
//        hop_map_bit[1]=0;
//        for(i=0;i<lcp_standby_hop_ch_cnt[device_id];i++)
//        {
//            used_channel_index =  (lcp_standby_hop_map[device_id][i] - LCP_START_FREQ)/LCP_USE_FREQ_MHZ;
//            if(used_channel_index < 32)
//            {
//                hop_map_bit[0] = hop_map_bit[0] | (0x01 << used_channel_index) ;
//            }
//            else
//            {
//                hop_map_bit[1] = hop_map_bit[1] | (0x01 << (used_channel_index -32)) ;
//            }
//        }
//        return true;
//    }
//    return false;
//}
 uint32_t lcp_get_prev_recv_channel(uint8_t device_id)
{
    return lcp_prev_recv_ch[device_id];
}
 void sort_ex(uint32_t *freq,int16_t *rssi,uint8_t size)
{
    int16_t tmp;
    uint32_t tmp_freq;
    for (uint8_t i = 0; i < size - 1; ++i)
    {
        for (uint8_t j = 0; j < size - 1 - i; ++j)
        {
            if (rssi[j] > rssi[j + 1])
            {
                tmp = rssi[j];
                tmp_freq= freq[j];
                rssi[j] = rssi[j + 1];
                freq[j] = freq[j + 1];
                rssi[j + 1] = tmp;
                freq[j + 1] = tmp_freq;
            }
        }
    }
}
 void sort_level(uint32_t *freq,uint32_t *level,uint8_t size)
{
    uint32_t tmp;
    uint32_t tmp_freq;
    for (uint8_t i = 0; i < size - 1; ++i)
    {
        for (uint8_t j = 0; j < size - 1 - i; ++j)
        {
            if (level[j] < level[j + 1])
            {
                tmp = level[j];
                tmp_freq= freq[j];
                level[j] = level[j + 1];
                freq[j] = freq[j + 1];
                level[j + 1] = tmp;
                freq[j + 1] = tmp_freq;
            }
        }
    }
}

 void sort(uint32_t *freq,uint8_t size)
{
    uint32_t tmp_freq;
    for (uint8_t i = 0; i < size - 1; ++i)
    {
        for (uint8_t j = 0; j < size - 1 - i; ++j)
        {
            if (freq[j] > freq[j + 1])
            {
                tmp_freq= freq[j];
                freq[j] = freq[j + 1];
                freq[j + 1] = tmp_freq;
            }
        }
    }
}

void sort_freq_and_flag(uint32_t *freq,uint8_t *flag,uint8_t size)
{
    uint32_t tmp_freq;
    uint8_t temp_flag=0;
    for (uint8_t i = 0; i < size - 1; ++i)
    {
        for (uint8_t j = 0; j < size - 1 - i; ++j)
        {
            if (freq[j] > freq[j + 1])
            {
                tmp_freq= freq[j];
                freq[j] = freq[j + 1];
                freq[j + 1] = tmp_freq;
                temp_flag= flag[j];
                flag[j] = flag[j + 1];
                flag[j + 1] = temp_flag;
            }
        }
    }
}

 uint16_t lcp_hopping_check(uint32_t device_id)
{
    //uint32_t temp_freq;
    uint8_t update_flag=0;
    uint8_t current_hop_index = lcp_get_current_hop_index(device_id);
    uint8_t channel_index=((current_hop_index)%lcp_hop_ch_cnt[device_id]);
    //uint8_t current_standby_hop_index=lcp_get_current_standby_hop_index(device_id);
    SWDIAG_HOPPING_IO_SET();
    lcp_prev_recv_ch[device_id]= lcp_recv_ch[device_id];


//    if((lcp_set_hop_flag[device_id] == true)&&(lcp_hop_update_time[device_id][channel_index] == (lcp_get_system_time()&LCP_HOP_UPDATE_TIME_MAX)) 
//        && lcp_hop_update_flag[device_id][channel_index]!=0)
//    {
//            SWDIAG_HOPCHANGE_IO_SET();
//            
//            lcp_hop_update_time[device_id][channel_index]=0;
//            lcp_hop_update_flag[device_id][channel_index]=0;
//            temp_freq = lcp_hop_map[device_id][channel_index];
//            lcp_hop_map[device_id][channel_index] = lcp_standby_hop_map[device_id][current_standby_hop_index];
//            lcp_standby_hop_map[device_id][current_standby_hop_index] = temp_freq;
//            lcp_set_current_standby_hop_index(device_id,(current_standby_hop_index+1)%lcp_standby_hop_ch_cnt[device_id]);
//            SWDIAG_HOPCHANGE_IO_RESET();
//            lcp_set_hop_flag[device_id] = false;
//            update_flag = 1;

//    }
    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    gdx_lcp_channel_set(lcp_hop_map[device_id][channel_index]);  //
    #else
    gdx_lcp_channel_set(lcp_hop_map[device_id][channel_index],CHANNEL_IDX);  //
    #endif
    lcp_recv_ch[device_id] = lcp_hop_map[device_id][channel_index];
    lcp_set_current_hop_index(device_id,(current_hop_index +1)&0x0F);
    SWDIAG_HOPPING_IO_RESET();
    return update_flag;
}
void lcp_hoptable_replace(uint32_t device_id)
{
    //sort_freq_ex(lcp_hop_map[device_id],lcp_hop_update_flag[device_id],lcp_hop_ch_cnt[device_id]);
    sort(lcp_hop_map[device_id],lcp_hop_ch_cnt[device_id]);
    sort(lcp_standby_hop_map[device_id],lcp_standby_hop_ch_cnt[device_id]);
}


uint16_t lcp_hopping_map_init(uint32_t *hopping_map,uint8_t hopping_map_cnt,uint8_t standby_hopping_map_cnt)
{
    uint8_t i=0;
    if(hopping_map==NULL || hopping_map_cnt > MAX_CHANNEL_TABLE_SIZE)
    {
        return SDK_ERR_INVALID_PARAM;
    }
    sort(hopping_map,hopping_map_cnt);
    printf("\r\nselect use freq:");
    for(i=0;i<hopping_map_cnt;i++)
    {
        lcp_hop_map[0][i] = hopping_map[i];
        lcp_hop_map[1][i] = hopping_map[i];
		lcp_hopping_channel_map[0].channel_map[i]=hopping_map[i];
		lcp_hopping_channel_map[1].channel_map[i]=hopping_map[i];
		lcp_hopping_channel_map[0].channel_used_flag[i]=1;
		lcp_hopping_channel_map[1].channel_used_flag[i]=1;
        printf("%d ",hopping_map[i]);
    }
    sort(&hopping_map[3],standby_hopping_map_cnt);
	
	memset(lcp_hop_map_level,0xFFFFFFFF,sizeof(lcp_hop_map_level));
    printf("\r\nselect standby freq:");
    for(i=hopping_map_cnt;i< hopping_map_cnt+standby_hopping_map_cnt;i++)
    {
        lcp_standby_hop_map[0][i - hopping_map_cnt]=hopping_map[i];
        lcp_standby_hop_map[1][i - hopping_map_cnt]=hopping_map[i];
		lcp_hopping_channel_map[0].channel_map[i]=hopping_map[i];
		lcp_hopping_channel_map[1].channel_map[i]=hopping_map[i];
		lcp_hopping_channel_map[0].channel_used_flag[i]=0;
		lcp_hopping_channel_map[1].channel_used_flag[i]=0;
        printf("%d ",hopping_map[i]);
    }
	memset(lcp_standby_hop_map_level,0xFFFFFFFF,sizeof(lcp_standby_hop_map_level));
    printf("\r\n");
	sort_freq_and_flag(&lcp_hopping_channel_map[0].channel_map[0],&lcp_hopping_channel_map[0].channel_used_flag[0],LCP_CHANNEL_CNT_USED_TO_HOP);
	sort_freq_and_flag(&lcp_hopping_channel_map[1].channel_map[0],&lcp_hopping_channel_map[1].channel_used_flag[0],LCP_CHANNEL_CNT_USED_TO_HOP);
	for(i=0;i< 9;i++)
	{
		printf("channel =%d,flag=%d\r\n",lcp_hopping_channel_map[0].channel_map[i],lcp_hopping_channel_map[0].channel_used_flag[i]);
	}
    lcp_recv_ch[0] = lcp_hop_map[0][0];
    lcp_recv_ch[1] = lcp_hop_map[1][0];
    lcp_hop_ch_cnt[0] = hopping_map_cnt;
    lcp_hop_ch_cnt[1] = hopping_map_cnt;
    lcp_standby_hop_ch_cnt[0] = standby_hopping_map_cnt;
    lcp_standby_hop_ch_cnt[1] = standby_hopping_map_cnt;
    memset(lcp_channel_recv_rate,0,sizeof(lcp_channel_recv_rate));

    return  SDK_SUCCESS;
}
 void lcp_hopping_2_comm_channel(uint8_t device_id)
{
    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    gdx_lcp_channel_set(lcp_recv_ch[device_id]);  //
    #else
    gdx_lcp_channel_set(lcp_recv_ch[device_id],CHANNEL_IDX);  //
    #endif
}
 void lcp_hopping_2_comm_channel_by_index(uint8_t device_id,uint8_t current_index)
{
    //printf("%d\r\n",lcp_hop_map[((current_index-1)%lcp_hop_ch_cnt)]);
    if(current_index == 0)
        current_index = 1+0x0F;
    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    gdx_lcp_channel_set(lcp_hop_map[device_id][((current_index-1)%lcp_hop_ch_cnt[device_id])]);
    #else
    gdx_lcp_channel_set(lcp_hop_map[device_id][((current_index-1)%lcp_hop_ch_cnt[device_id])],CHANNEL_IDX);
    #endif
}

 void lcp_hopping_2_adv_channel(void)
{
    #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
    gdx_lcp_channel_set(lcp_default_adv_channel);
    #else
    gdx_lcp_channel_set(lcp_default_adv_channel,CHANNEL_IDX);
    #endif
}
void lcp_hopping_set_default_adv_channel(uint32_t adv_channel)
{
    lcp_default_adv_channel = adv_channel;
    printf("lcp_default_adv_channel=%d\r\n",lcp_default_adv_channel);
}


void lcp_debug_info_printf(void)
{
#if 0
    printf("\r\ndevice0:\r\n");
    for(uint8_t i=0;i<40;i++)
    {
        printf("ch%d send %d recv %d\r\n",i,lcp_channel_recv_rate[0][i].send_cnt,lcp_channel_recv_rate[0][i].recv_cnt);
        lcp_channel_recv_rate[0][i].send_cnt = 0;
        lcp_channel_recv_rate[0][i].recv_cnt = 0;
    }
    printf("\r\ndevice1:\r\n");
    for(uint8_t i=0;i<40;i++)
    {
        printf("ch%d send %d recv %d\r\n",i,lcp_channel_recv_rate[1][i].send_cnt,lcp_channel_recv_rate[1][i].recv_cnt);
        lcp_channel_recv_rate[1][i].send_cnt = 0;
        lcp_channel_recv_rate[1][i].recv_cnt = 0;

    }
#endif

}
void lcp_all_channel_scan(uint32_t init_freq,uint32_t *used_freq_table,uint8_t freq_table_size)    //uint32_t *used_freq_table,uint8_t freq_table_size
{
	uint8_t channel_index = 0;
    uint8_t current_scan_channel=0;
    uint16_t i =0;


    int16_t rssis[TOTAL_CHANNEL_CNT];

    uint32_t freqs_chose[TOTAL_CHANNEL_CNT];
    uint32_t freqs[TOTAL_CHANNEL_CNT];
    int32_t rssi_sum = 0;
    int16_t avg_rssi=0;
    uint8_t current_scan_channel_1=0;

    for(current_scan_channel_1=1;current_scan_channel_1<=TOTAL_CHANNEL_CNT;current_scan_channel_1++)
    {
        current_scan_channel = current_scan_channel_1 -1;
        gdx_lcp_rx_stop();
        avg_rssi = -120;
        //startfreq = 2402+index*2;
//        if((LCP_START_FREQ+(current_scan_channel*LCP_USE_FREQ_MHZ) >= 2432)&&
//			(LCP_START_FREQ+(current_scan_channel*LCP_USE_FREQ_MHZ)) <= 2452)
//		{
//			continue;
//		}
		
		freqs[channel_index] = LCP_START_FREQ+(current_scan_channel*LCP_USE_FREQ_MHZ);
        
        #if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X)
        gdx_lcp_channel_set(freqs[channel_index]);
        #else
        gdx_lcp_channel_set(freqs[channel_index],CHANNEL_IDX);
        #endif
        gdx_lcp_rx_start();
        delay_us(80);
        rssi_sum = 0;
        for(i=0;i<1000;i++)
        {
            delay_us(10);
            //avg_rssi=avg_rssi+gdx_lcp_rssi_get();
            int16_t temp_rssi = gdx_lcp_rssi_get();
            rssi_sum += (int32_t)temp_rssi;
            //if(avg_rssi < temp_rssi)
            //    avg_rssi = temp_rssi;
        }
        avg_rssi = (int16_t)(rssi_sum / i);
        rssis[channel_index]= avg_rssi;
		
		//if(rssis[channel_index])
			printf("index = %d ,ch=%d rssi=%d\r\n",channel_index,freqs[channel_index],rssis[channel_index]);
		
		channel_index ++ ;
		
	}
	

	
	
	//rssi list big to little
	printf("sort list high to low \r\n");
	
//	for(uint8_t channel_id =0;channel_id< channel_index;channel_id++)
//	{
//		for(uint8_t comp_channel_id=channel_id;comp_channel_id < channel_index; comp_channel_id++)
//		{	
//			if(rssis[channel_id]> rssis[comp_channel_id])
//			{
//				rssi_max_tmp = rssis[channel_id];
//				rssis[channel_id] = rssis[comp_channel_id];
//				rssis[comp_channel_id] = rssi_max_tmp;
//				
//				freq_tmp = freqs[channel_id];
//				freqs[channel_id] = freqs[comp_channel_id];
//				freqs[comp_channel_id] = freq_tmp;				
//			}
//		}		
//		printf("index = %d ,ch=%d rssi=%d \r\n",channel_id,freqs[channel_id],rssis[channel_id]);
//	}
	
	//printf("index = %d ,ch=%d rssi=%d\r\n",channel_index,freqs[channel_index],rssis[channel_index]);
	
	
	//pick up
	printf("pick up channel and rssi \r\n");
	
	//memcpy((void *)rssi_chose,(void  *)rssis,18);
	//memcpy((void *)freqs_chose,(void *)freqs,18);
	
//	for(uint8_t chose_id = 0;chose_id < 9; chose_id++)
//	{
//		
//		freqs_chose[chose_id] = freqs[chose_id];
//		rssi_chose[chose_id] = rssis[chose_id];
//		
//		printf("index = %d,ch=%d rssi=%d\r\n",chose_id,freqs_chose[chose_id],rssi_chose[chose_id]);
//	}
#if 0
    for(uint8_t chose_id=0;chose_id < channel_index;chose_id++)
    {
        if(chose_id %CHOSE_CHANNEL_MOD ==0)
        {
            rssi_max_tmp = 0;
        }
        if(rssi_max_tmp > rssis[chose_id])
        {
            freqs_chose[chose_id /CHOSE_CHANNEL_MOD] =  freqs[chose_id];
            rssi_max_tmp = rssis[chose_id];
        }
    }
	printf("pick up complete\r\n");
#else
	sort_ex(freqs,rssis,channel_index);
	
#if 0	
	for(i=0;i<9;i++)
	{
		freqs_chose[i] =  freqs[i];
	}
#else
uint8_t choose_index=1;
uint8_t j = 1;
freqs_chose[0] = freqs[0];
uint8_t not_used_flag=0;
printf("sort \r\n");
	for(i=0;i<channel_index;i++)
	{
		printf("channel=%d rssi =%d\r\n",freqs[i],rssis[i]);
	}
	//for(i=1;i<channel_index;i++)
	{
		for( ;j<channel_index;j++)
		{
			{
				for(uint8_t x=0;x<choose_index;x++)
				{
					if(freqs_chose[x] - freqs[j] == 2 || freqs[j] - freqs_chose[x]   == 2)
					{
						not_used_flag = 1;
						break;
					}
				}
				if(not_used_flag == 0)
				{
					freqs_chose[choose_index] = freqs[j];
					choose_index ++;
					if(choose_index == 9)
						break;
				}
				else
					not_used_flag = 0;
			}
		}
	}
	printf("after choose\r\n");
		for(i=0;i<9;i++)
	{
		printf("channel=%d \r\n",freqs_chose[i]);
	}
#endif
	
#endif
	
	
    //sort_ex(freqs,rssis,TOTAL_CHANNEL_CNT) ;
//    for(i=0;i<TOTAL_CHANNEL_CNT;i++)
//    printf("ch=%d rssi=%d\r\n",freqs[i],rssis[i]);

    //sort_ex(freqs_chose,rssi_chose,TOTAL_CHANNEL_CNT/CHOSE_CHANNEL_MOD);
    
	
	
	if(used_freq_table != NULL && freq_table_size <=TOTAL_CHANNEL_CNT)
    {
        for(i=0;i<freq_table_size;i++)
        {
//            if(init_freq == freqs_chose[i])
//            {
//                init_freq_in_flag =true;
//            }
            used_freq_table[i] = freqs_chose[i];
        }
//        if(init_freq_in_flag == false)
//        {
//            used_freq_table[i-1]= init_freq;
//        }
        //sort(used_freq_table,freq_table_size);
    }
}


