/**
 *****************************************************************************************
 *
 * @file user_keyboard.c
 *
 * @brief  User keyboard Implementation.
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
#ifndef _USER_MOUSE_H_
#define _USER_MOUSE_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdbool.h>
#include <stdint.h>
#include "grx_sys.h"

/*
 * Typedefs
 *****************************************************************************************
 */
/**@brief Mouse input data define. */
typedef struct
{
    bool left_button_press;
    bool middle_button_press;
    bool right_button_press;
    int8_t x_delta;
    int8_t y_delta;
    int8_t wheel_delta;
} mouse_data_t;

/**@brief Mouse media data define. */
typedef struct
{
    uint8_t play_pause:1;
    uint8_t al_control:1;
    uint8_t next_track:1;
    uint8_t previous_track:1;
    uint8_t volume_down:1;
    uint8_t volume_up:1;
    uint8_t ac_foward:1;
    uint8_t ac_back:1;
} media_data_t;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Mouse used service init.
 *****************************************************************************************
 */
void user_mouse_service_init(void);

/**
 *****************************************************************************************
 * @brief Send mouse data to peer device.
 *
 * @param[in] conn_idx: Connected index.
 * @param[in] p_data: Pointer of mouse input data.
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t user_mouse_data_send(uint8_t conn_idx, mouse_data_t *p_data);

/**
 *****************************************************************************************
 * @brief Send mouse media data to peer device.
 *
 * @param[in] conn_idx: Connected index.
 * @param[in] p_data: Pointer of media data.
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t user_mouse_media_send(uint8_t conn_idx, media_data_t *p_data);

/**
 *****************************************************************************************
 * @brief Clear flags of boot mode and notification.
 *
 *****************************************************************************************
 */
void user_mouse_clear_flags(void);

#endif

