/**
 *****************************************************************************************
 *
 * @file gui_font_other.c
 *
 * @brief Function of show other modulus font
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
#include "gui_font_other.h"

#if FONT_OTHER_EN==1

#include "gui_color.h"
#include "gui_basic.h"


/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
uint8_t gui_put_char16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t index)
{
    uint8_t i,j;
    
    if( x>(GUI_DISPLAY_X_MAX-16) ) return(0);
    if( y>(GUI_DISPLAY_Y_MAX-32) ) return(0);
    for(i=0; i<16; i++)
    {
        for(j=0; j<2; j++)
        {
            gui_point_color(x+j*8, y+i, code16_32[(index*4)+(j*2)][i], 8);
            gui_point_color(x+j*8, y+16+i, code16_32[(index*4)+(j*2+1)][i], 8);
        }
    }
    return (1);
}

void gui_put_string16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t num)
{
    uint16_t i;
    
    for(i=0; i<num; i++)
    {
        if(gui_put_char16_32(x, y, code16_32, i) == 0)
        {
            break;
        }
        x += 16;
    }
}

uint8_t gui_put_char32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t index)
{
    uint8_t i,j;
    
    if( x>(GUI_DISPLAY_X_MAX-32) ) return(0);
    if( y>(GUI_DISPLAY_Y_MAX-64) ) return(0);
    for(i=0; i<16; i++)
    {
        for(j=0; j<4; j++)
        {
            gui_point_color(x+j*8, y+i, code32_64[(index*16)+(j*4)][i], 8);
            gui_point_color(x+j*8, y+16+i, code32_64[(index*16)+(j*4+1)][i], 8);
            gui_point_color(x+j*8, y+32+i, code32_64[(index*16)+(j*4+2)][i], 8);
            gui_point_color(x+j*8, y+48+i, code32_64[(index*16)+(j*4+3)][i], 8);  
        }
    }
    return (1);
}

void gui_put_string32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t num)
{
    uint16_t i;
    
    for(i=0; i<num; i++)
    {
        if(gui_put_char32_64(x, y, code32_64, i) == 0)
        {
            break;
        }
        x += 32;
    }
}

void gui_put_code_other(uint16_t x, uint16_t y, const uint8_t *code_other, uint16_t x_size, uint16_t y_size)
{
    uint8_t i,j;
    uint8_t x_count = x_size / 8;
    uint8_t x_remain = x_size % 8;
    if(x_remain != 0)
    {
        x_count += 1;
    }
    for(i=0; i<x_count; i++)
    {
        for(j=0; j<y_size; j++)
        {
            gui_point_color(x+i*8, y+j, code_other[(i*y_size)+j], 8);
        }
    }
}

#endif

