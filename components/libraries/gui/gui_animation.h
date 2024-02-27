/**
 *****************************************************************************************
 *
 * @file gui_animation.h
 *
 * @brief Animation API.
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

#ifndef _ANIMATION_H__
#define _ANIMATION_H__
#include "gui_config.h"

#if ANIMATION_EN==1

typedef void(*animation_stop_handle_t)(void);/**< animation stop call back dfine. */

/**@brief Animation types. */
typedef enum {
  MOVE_LEFT = 1,
  MOVE_RIGHT,
  MOVE_UP,
  MOVE_DOWN,
}animation_type_t;

/**
 *****************************************************************************************
 * @brief Animation init.
 *
 * @param[in] display_ram:  Pointer of display memory
 * @param[in] display_buffer: Pointer of display buffer.
 *****************************************************************************************
 */
void gui_animation_init(T_COLOR (*display_ram)[GUI_DISPLAY_X_MAX], T_COLOR (*display_buffer)[GUI_DISPLAY_X_MAX]);

/**
 *****************************************************************************************
 * @brief Animation timer task.
 * @note The function should be called in animation timer handler
 *****************************************************************************************
 */
void gui_animation_timer_task(void);

/**
 *****************************************************************************************
 * @brief Start a animation.
 *
 * @param[in] type:  Animation type 
 * @param[in] stop_call: Animation stop callback.
 *****************************************************************************************
 */
void gui_start_animation(animation_type_t type, animation_stop_handle_t stop_call);

/**
 *****************************************************************************************
 * @brief Process Animation overlap.
 * @note The function should be called in animation timer handler
 *****************************************************************************************
 */
void gui_animation_overlap(void);

#endif
#endif
