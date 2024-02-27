/**
 *******************************************************************************
 *
 * @file ams_protocol.h
 *
 * @brief AMS Protocol API.
 *
 *******************************************************************************
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
#ifndef _AMS_PROTOCOL_H_
#define _AMS_PROTOCOL_H_

#include "ams_c.h"
#include "grx_sys.h"

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Print new command list.
 *
 * @param[in] p_cmd_list: Point to the structure containing new commands.
 *
 *****************************************************************************************
 */
void print_new_cmd_list(const ams_c_cmd_list_t *p_cmd_list);


/**
 *****************************************************************************************
 * @brief Print updated attribution information.
 *
 * @param[in] p_attr_info: Point to the structure containing the updated attribute information.
 *
 *****************************************************************************************
 */
void print_update_attr_info(ams_c_attr_info_t *p_attr_info);


/**
 *****************************************************************************************
 * @brief Print the data of an attribute completely.
 *
 * @param[in] p_attr_info: Point to a structure containing the complete attribute information.
 *
 *****************************************************************************************
 */
void print_cplt_attr_data(ams_c_cplt_attr_data_t *p_cplt_attr_data);

#endif  // _AMS_PROTOCOL_H_

