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
#include "st7735.h"
#include "gui_animation.h"

static bool gui_refresh_flag = false;
/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
 
void gui_init(void)
{
    lcd_init();
    gui_animation_init(g_lcd_gram, g_lcd_buffer);
}

void gui_fill_mem(T_COLOR color)
{
    lcd_fill_mem(color);
}

void gui_rectangle_fill_mem(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color)
{
    lcd_rectangle_fill_mem(x0, y0, x1, y1, color);
}

void gui_point(uint16_t x, uint16_t y, T_COLOR color)
{
   lcd_draw_point(x, y, color);
}

T_COLOR gui_read_point(uint16_t x, uint16_t y)
{
    return lcd_read_point(x, y);
}

void gui_refresh(void)
{
    gui_refresh_flag = true;
}

void gui_rectangle_refresh(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    lcd_rectangle_refresh(x0, y0, x1, y1);
}

void gui_set_refresh_mem(bool gram_set)
{
    lcd_set_memory(gram_set);
}

void gui_refresh_schedule(void)
{
    if(gui_refresh_flag)
    {
        lcd_refresh();
        gui_refresh_flag = false;
    }
}

