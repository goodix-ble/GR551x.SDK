/**
 *****************************************************************************************
 * 
 *
 * @file throughput.h
 *
 * @brief Header file - Throughput Function
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

#ifndef _THROUGHPUT_H_
#define _THROUGHPUT_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "ths.h"
#include <stdbool.h>

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief The type of paramter which is used to adjust throughput rate. */
typedef enum
{
    THRPT_PARAM_CI,
    THRPT_PARAM_PDU,
    THRPT_PARAM_PHY
} thrpt_param_type_t;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief The event handler for Throughput Service.
 *
 * @param[in] p_evt: Pointer to the event of Throughput Service.
 *****************************************************************************************
 */
void thrpt_event_process(ths_evt_t *p_evt);

/**
 *****************************************************************************************
 * @brief Perform average throughput calculated.
 *****************************************************************************************
 */
void thrpt_counter_handler(void *p_arg);

/**
 *****************************************************************************************
 * @brief Check whether the paramter update is started.
 *
 * @param[in] param_type: The parameter type to be checked.
 *
 * @return True if the update is started, otherwise false.
 *****************************************************************************************
 */
bool thrpt_is_update_started(thrpt_param_type_t param_type);

/**
 *****************************************************************************************
 * @brief Clean the status of parameter update.
 *
 * @param[in] param_type: The parameter type to be cleaned.
 *****************************************************************************************
 */
void thrpt_update_stat_clean(thrpt_param_type_t param_type);

/**
 *****************************************************************************************
 * @brief Update current mtu.
 *
 * @param[in] new_mtu: The value of new mtu size.
 *****************************************************************************************
 */
void thrpt_update_mtu(uint16_t new_mtu);

#endif

