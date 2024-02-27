/**
 ****************************************************************************************
 *
 * @file gui_color.c
 *
 * @brief Function of set color
 *
 ****************************************************************************************
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
#include  "gui_color.h"
#include  "gui_config.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t const  DCB2HEX_TAB[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
static T_COLOR  s_disp_color;/**< Display color. */
static T_COLOR  s_back_color;/**< Back color. */


uint8_t gui_dcb_to_hex(uint8_t dcb)
{
    return DCB2HEX_TAB[dcb];
}

void gui_set_color(T_COLOR disp_color, T_COLOR back_color)
{  
    s_disp_color = disp_color;
    s_back_color = back_color; 
}

T_COLOR gui_get_back_color(void)
{  
    return s_back_color;
}

T_COLOR gui_get_disp_color(void)
{  
    return s_disp_color;
}


void  gui_exchange_color(void)
{  
    T_COLOR  bakc;
    bakc = s_disp_color;
    s_disp_color = s_back_color;
    s_back_color = bakc;
}


void gui_point_color(uint8_t x, uint8_t y, uint8_t font_dat, uint8_t num)
{
    uint8_t j;
    uint8_t x_coord = x;
    T_COLOR   bakc;
    for(j=0; j<num; j++)
    {  
        if( (font_dat&DCB2HEX_TAB[j])==0 )
            bakc = s_back_color;
        else  
            bakc = s_disp_color;
        gui_point(x_coord, y, bakc);       
        x_coord++;
    }
}
