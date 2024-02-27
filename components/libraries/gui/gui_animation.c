/**
 *****************************************************************************************
 *
 * @file gui_animation.c
 *
 * @brief Animation Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gui_animation.h"
#include  <string.h>
#include <stdbool.h>

#if ANIMATION_EN==1

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static uint8_t all_move_frame = GUI_X_MOVE_FRAME;
static animation_stop_handle_t now_animation_stop_call;
static animation_type_t now_animation_type = MOVE_LEFT;
static uint8_t now_animation_frame = 0;
static bool start_animation_flag = false;

static T_COLOR (*gui_display_ram)[GUI_DISPLAY_X_MAX];
static T_COLOR (*gui_display_buffer)[GUI_DISPLAY_X_MAX];

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
 /**
 *****************************************************************************************
 * @brief Move up animation
 *
 * @param[in] frame  move frame.
 *****************************************************************************************
 */
static void gui_animation_move_up(uint8_t frame)
{
    uint8_t y;
    for(y=0; y<(GUI_Y_MOVE_FRAME-1); y++)
    {
        memcpy(&gui_display_ram[y*GUI_Y_MOVE_PIXEL], &gui_display_ram[(y+1)*GUI_Y_MOVE_PIXEL], GUI_DISPLAY_X_MAX*2*GUI_Y_MOVE_PIXEL);
    }
    memcpy(&gui_display_ram[GUI_DISPLAY_Y_MAX-1-GUI_Y_MOVE_PIXEL], &gui_display_buffer[frame*GUI_Y_MOVE_PIXEL], GUI_DISPLAY_X_MAX*2*GUI_Y_MOVE_PIXEL);
}

 /**
 *****************************************************************************************
 * @brief Move down animation
 *
 * @param[in] frame  move frame.
 *****************************************************************************************
 */
static void gui_animation_move_down(uint8_t frame)
{
    uint8_t y;
    for(y=GUI_Y_MOVE_FRAME-1; y>0; y--)
    {
        memcpy(&gui_display_ram[y*GUI_Y_MOVE_PIXEL], &gui_display_ram[(y-1)*GUI_Y_MOVE_PIXEL], GUI_DISPLAY_X_MAX*2*GUI_Y_MOVE_PIXEL);
    }
    memcpy(&gui_display_ram[0], &gui_display_buffer[(GUI_Y_MOVE_FRAME-1-frame)*GUI_Y_MOVE_PIXEL], GUI_DISPLAY_X_MAX*2*GUI_Y_MOVE_PIXEL);
}

 /**
 *****************************************************************************************
 * @brief Move left animation
 *
 * @param[in] frame  move frame.
 *****************************************************************************************
 */
static void gui_animation_move_left(uint8_t frame)
{
  uint16_t x,y;
  
  for(y=0; y<GUI_DISPLAY_Y_MAX; y++)
  {
      for(x=0; x<(GUI_X_MOVE_FRAME-1); x++)
      {
          memcpy(&gui_display_ram[y][x*GUI_X_MOVE_PIXEL], &gui_display_ram[y][(x+1)*GUI_X_MOVE_PIXEL], GUI_X_MOVE_PIXEL*2);
      }
  }
  
  for(y=0; y<GUI_DISPLAY_Y_MAX; y++)
  {
     memcpy(&gui_display_ram[y][x*GUI_X_MOVE_PIXEL], &gui_display_buffer[y][frame*GUI_X_MOVE_PIXEL], GUI_X_MOVE_PIXEL*2);
  }
}

 /**
 *****************************************************************************************
 * @brief Move right animation
 *
 * @param[in] frame  move frame.
 *****************************************************************************************
 */
static void gui_animation_move_right(uint8_t frame)
{
    uint16_t x,y;

    for(y=0; y<GUI_DISPLAY_Y_MAX; y++)
    {
        for(x=GUI_X_MOVE_FRAME-1; x>0; x--)
        {
            memcpy(&gui_display_ram[y][x*GUI_X_MOVE_PIXEL], &gui_display_ram[y][(x-1)*GUI_X_MOVE_PIXEL], GUI_X_MOVE_PIXEL*2);
        }
     }

    for(y=0; y<GUI_DISPLAY_Y_MAX; y++)
    {
      memcpy(gui_display_ram[y], &gui_display_buffer[y][(GUI_X_MOVE_FRAME-1-frame)*GUI_X_MOVE_PIXEL], GUI_X_MOVE_PIXEL*2);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void gui_animation_init(T_COLOR (*display_ram)[GUI_DISPLAY_X_MAX], T_COLOR (*display_buffer)[GUI_DISPLAY_X_MAX])
{
    gui_display_ram = display_ram;
    gui_display_buffer = display_buffer;
}

void gui_start_animation(animation_type_t type, animation_stop_handle_t stop_call)
{
   now_animation_type = type;
   now_animation_stop_call = stop_call;
   now_animation_frame = 0;
   start_animation_flag = true;
   all_move_frame = GUI_X_MOVE_FRAME;
   gui_animation_timer_start();
}

void gui_animation_timer_task(void)
{
  if(start_animation_flag)
  {
      switch(now_animation_type)
      {
          case MOVE_LEFT:
              gui_animation_move_left(now_animation_frame);
              break;
          case MOVE_RIGHT:
               gui_animation_move_right(now_animation_frame);
              break;
          case MOVE_UP:
              gui_animation_move_up(now_animation_frame);
              break;
          case MOVE_DOWN:
              gui_animation_move_down(now_animation_frame);
              break;
          default:break;
      }

    gui_refresh();
    now_animation_frame++;

    if(now_animation_frame == all_move_frame)
    {
      gui_animation_timer_stop();
      start_animation_flag = false;
      if(now_animation_stop_call != NULL)
      {
          now_animation_stop_call();
      }
    }
  }
}

void gui_animation_overlap(void)
{
    gui_animation_timer_stop();
    if(now_animation_stop_call != NULL)
    {
        now_animation_stop_call();
    }
}
#endif
