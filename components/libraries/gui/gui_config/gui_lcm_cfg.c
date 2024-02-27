/**
 *****************************************************************************************
 *
 * @file gui_lcm_config.c
 *
 * @brief Users should implement the timer-related interface themselves
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

#include "gui_config.h"
#include "lcm.h"
#include "gui_animation.h"


extern lcm_def_t st7735_lcm_opt;
const lcm_def_t *p_lcm_opt = &st7735_lcm_opt;
/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
 
void gui_init(void)
{
    p_lcm_opt->lcm_init();
    gui_animation_init(p_lcm_opt->lcm_freambuffer_draw, p_lcm_opt->lcm_freambuffer_map);
}

void gui_fill(T_COLOR dat)
{
    p_lcm_opt->lcm_fill_color(dat);
}

void gui_point(uint16_t x, uint16_t y, T_COLOR color)
{
    if (x >= p_lcm_opt->lcm_var.lcm_width)
      return;
    if (y >= p_lcm_opt->lcm_var.lcm_height)
      return;
    p_lcm_opt->lcm_draw_point(x, y, color);
}

T_COLOR gui_read_point(uint16_t x, uint16_t y)
{
    if (x >= p_lcm_opt->lcm_var.lcm_width)
      return 0x0;
    if (y >= p_lcm_opt->lcm_var.lcm_height)
      return 0x0;
    return (T_COLOR)p_lcm_opt->lcm_read_point(x, y);
}

void lcm_set_memory(lcm_memory_type_t type)
{
     //p_lcm_opt->lcm_set_buffer(type); 
}


void gui_refresh(void)
{
     p_lcm_opt->lcm_refresh();
}

void gui_render(uint8_t *p_map)
{
     p_lcm_opt->lcm_render(p_map);
}


void lcm_load_16x32(uint8_t x,uint8_t y_page, const uint8_t(*code16_32)[16],uint8_t index)
{
     //fill it
}
