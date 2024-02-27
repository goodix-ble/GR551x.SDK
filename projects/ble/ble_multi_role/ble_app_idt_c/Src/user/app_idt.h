/**
 *****************************************************************************************
 *
 * @file app_idts.h
 *
 * @brief Header file - User Function
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
#ifndef ___APP_IDTS_H__
#define ___APP_IDTS_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * STRUCTURES
 *****************************************************************************************
 */
typedef struct
{
    uint8_t  cmd_id;
    uint8_t  channel_idx;
    uint16_t delay_time;
    uint32_t access_addr;

}idt_info_t;

typedef struct
{
    void (*transmit_cfm_cb)(void);
    void (*receive_cb)(uint8_t *p_data, uint8_t length);

}idt_cb_func_t;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Function for init IDT module.
 *****************************************************************************************
 */
void app_idt_init(idt_info_t *p_idt_info, idt_cb_func_t *p_idt_callbacks);

/**
 *****************************************************************************************
 *@brief Function for sending data by IDT.
 *****************************************************************************************
 */
void idt_data_send(uint16_t length, uint8_t *p_data);

/**
 *****************************************************************************************
 *@brief Function for checking the status of IDT module.
 *****************************************************************************************
 */
bool idt_status_check(void);

#endif

