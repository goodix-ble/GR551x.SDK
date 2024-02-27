/**
 ****************************************************************************************
 *
 * @file gui_convert_color.c
 *
 * @brief Function of convert color
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
#include  "gui_convert_color.h"
#include  "gui_config.h"

#if CONVERT_COLOR_EN==1

uint16_t  gui_color_to_index_565(uint32_t color_rgb) 
{  uint8_t  r, g, b;

   b = ( color_rgb>>(0+3) ) & 0x1f;        
   g = ( color_rgb>>(8+2) ) & 0x3f;        
   r = ( color_rgb>>(16+3)) & 0x1f;        
   
   return( (r<<11) + (g<<5) + (b<<0) );        
}

uint32_t  gui_index_to_color_565(uint16_t index) 
{  uint32_t  r, g, b;

   b = (index>>0)  & 0x1f;
   g = (index>>5)  & 0x3f;
   r = (index>>11) & 0x1f;
   r = r * 255 / 31;
   g = g * 255 / 63;
   b = b * 255 / 31;
   
   return( (r<<16) + (g<<8) + (b<<0) );
}

uint16_t  gui_color_to_index_555(uint32_t color_rgb) 
{  uint8_t  r, g, b;

   b = ( color_rgb>>(0+3) ) & 0x1f;
   g = ( color_rgb>>(8+3) ) & 0x1f;
   r = ( color_rgb>>(16+3)) & 0x1f;
  
   return( (r<<10) + (g<<5) + (b<<0) );
}

uint32_t  gui_index_to_color_555(uint16_t index) 
{  uint32_t  r, g, b;

   b = (index>>0)  & 0x1f;
   g = (index>>5)  & 0x1f;
   r = (index>>10) & 0x1f;
   r = r * 255 / 31;
   g = g * 255 / 31;
   b = b * 255 / 31;
   
   return( (r<<16) + (g<<8) + (b<<0) );
}

uint16_t  gui_color_to_index_444(uint32_t color_rgb) 
{  uint8_t r,g,b;

   b = ( color_rgb>>(0+4) ) & 0x0f;
   g = ( color_rgb>>(8+4) ) & 0x0f;
   r = ( color_rgb>>(16+4)) & 0x0f;
   
   return ( (r<<8) + (g << 4) + (b<<0) );
}

uint32_t  gui_index_to_color_444(uint16_t index) 
{  uint8_t  r,g,b;

   b = (index >> 0) & 0x0f;
   g = (index >> 4) & 0x0f;
   r = (index >> 8) & 0x0f;
  
   r = r * 17;
   g = g * 17;
   b = b * 17;
   
   return ( (r<<16) + (g<<8) + (b<<0) );
}

uint8_t  gui_color_to_index_332(uint32_t color_rgb) 
{  uint32_t  r, g, b;

   b = (color_rgb>>0)  & 0xff;
   g = (color_rgb>>8)  & 0xff;
   r = (color_rgb>>16) & 0xff;
   r = (r * 7 + 127) / 255;
   g = (g * 7 + 127) / 255;
   b = (b + 42) / 85;
   
   return( (r<<5) + (g << 2) + (b<<0) );
}

uint32_t  gui_index_to_color_332(uint8_t index)
{  uint32_t  r, g, b;

   r = (index >> 5) * 255 / 7;
   g = ((index >> 3) & 7) * 255 / 7;
   b = (index & 3) * 85;
   
   return( (r<<16) + (g << 8) + (b<<0) );
}

uint8_t  gui_color_to_index_222(uint32_t color_rgb)
{  uint32_t  r, g, b;

   b = (((color_rgb>>0) &255)+0x2a)/0x55;
   g = (((color_rgb>>8) &255)+0x2a)/0x55;
   r = (((color_rgb>>16)&255)+0x2a)/0x55;
   
   return( (r<<4) + (g<<2) + (b<<0) );
}

uint32_t  gui_index_to_color_222(uint8_t index) 
{  uint8_t  r, g, b;

   b = ((index>>0)&3) * 0x55;
   g = ((index>>2)&3) * 0x55;
   r = ((index>>4)&3) * 0x55;
   
   return( (r<<16) + (g<<8) + (b<<0) );
}

uint8_t  gui_color_to_index_111(uint32_t color_rgb) 
{  uint8_t  r, g, b;

   b = (color_rgb>>(0+7))  &1;
   g = (color_rgb>>(8+7))  &1;
   r = (color_rgb>>(16+7)) &1;
   
   return( (r<<2) + (g<<1) + (b<<0) );
}

uint32_t  gui_index_to_color_111(uint8_t index) 
{  
   uint8_t  r, g, b;
   b = ((index>>0)&1) * 0xff;
   g = ((index>>1)&1) * 0xff;
   r = ((index>>2)&1) * 0xff;
   
   return( (r<<16) + (g<<8) + (b<<0) );
}
#endif
