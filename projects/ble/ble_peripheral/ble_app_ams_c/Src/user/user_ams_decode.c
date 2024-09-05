/**
 *******************************************************************************
 *
 * @file ams_protocol.c
 *
 * @brief AMS protocal implementation.
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


#include "user_ams_decode.h"
#include "app_log.h"
#include "utility.h"
#include <stdio.h>

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief String literals for the iOS remote command id types. Used then printing to UART. */
static char const * cmd_name[] =
{
    "Play",
    "Pause",
    "Toggle play pause",
    "Next track",
    "Previous track",
    "Volume up",
    "Volume down",
    "Advanced repead mode",
    "Advanced shuffle mode",
    "Skip forward",
    "Skip backward",
    "Like track",
    "Dislike track",
    "Book mark track"
};

/**@brief String literals for the iOS entity types. Used then printing to UART. */
static char const * ett_name[] =
{
    "Player",
    "Queue",
    "Track"
};

/**@brief String literals for the iOS player attribute types. Used when printing to UART. */
static char const * player_attr_name[] =
{
    "Name",
    "Playback info",
    "Volume"
};
/**@brief String literals for the iOS queue attribute types. Used when printing to UART. */
static char const * queue_attr_name[] =
{
    "Index",
    "Count",
    "Shuffle mode",
    "Repead mode"
};
/**@brief String literals for the iOS track attribute types. Used when printing to UART. */
static char const * track_attr_name[] =
{
    "Artist/Lyric",
    "Album",
    "Title",
    "Duration"
};
/**@brief String literals for the iOS all entity attribute types. Used when printing to UART. */
static char const ** attr_name[] =
{
    player_attr_name,
    queue_attr_name,
    track_attr_name
};

/*
*GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void print_new_cmd_list(const ams_c_cmd_list_t *p_cmd_list)
{
    for (uint16_t i = 0; i < p_cmd_list->length; i++)
    {
        APP_LOG_RAW_INFO("Available CMD: %s\r\n", cmd_name[p_cmd_list->p_cmd[i]]);
    }
    UNUSED_VARIABLE(cmd_name[0]);
}

void print_update_attr_info(ams_c_attr_info_t *p_attr_info)
{
    UNUSED_VARIABLE(ett_name[0]);
    UNUSED_VARIABLE(attr_name[0]);
    APP_LOG_RAW_INFO("%s-%s: ", ett_name[p_attr_info->ett_id],
                     attr_name[p_attr_info->ett_id][p_attr_info->attr_id]);
    for (uint16_t i = 0; i < p_attr_info->length; i++)
    {
        APP_LOG_RAW_INFO("%c", p_attr_info->p_data[i]);
    }
    if (p_attr_info->flag & AMS_C_TRUNCATED_FLAG)
    {
        APP_LOG_RAW_INFO("(truncated)");
    }
    APP_LOG_RAW_INFO("\r\n");
}

void print_cplt_attr_data(ams_c_cplt_attr_data_t *p_cplt_attr_data)
{
    for (uint16_t i = 0; i < p_cplt_attr_data->length; i++)
    {
        APP_LOG_RAW_INFO("%c", p_cplt_attr_data->p_data[i]);
    }
    APP_LOG_RAW_INFO("\r\n");
}
