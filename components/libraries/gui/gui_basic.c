/**
 *****************************************************************************************
 *
 * @file gui_basic.c
 *
 * @brief Gui base function Implementation.
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


#include "gui_basic.h"
#include <math.h>


uint32_t gui_pow(uint32_t m,uint8_t n)
{
    uint32_t result = 1;
    while (n--)result *= m;
    return result;
}


void  gui_line_hor(uint16_t x0, uint8_t y0, uint16_t x1, T_COLOR color)
{
    uint8_t  temp;
    if(x0>x1)               
    {
        temp = x1;
        x1 = x0;
        x0 = temp;
    }
    do
    {
        gui_point(x0, y0, color);   
        x0++;
    }
    while(x1>=x0);
}


void  gui_line_ver(uint16_t x0, uint8_t y0, uint8_t y1, T_COLOR color)
{
    uint8_t  temp;
    if(y0>y1)      
    {
        temp = y1;
        y1 = y0;
        y0 = temp;
    }
    do
    {
        gui_point(x0, y0, color);   
        y0++;
    }
    while(y1>=y0);
}

void  gui_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color)
{  gui_line_hor(x0, y0, x1, color);
   gui_line_hor(x0, y1, x1, color);
   gui_line_ver(x0, y0, y1, color);
   gui_line_ver(x1, y0, y1, color);
}


void  gui_rectangle_fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color)
{  
    uint16_t  i;
    if(x0>x1)                         
    {  i = x0;
      x0 = x1;
      x1 = i;
    }
    if(y0>y1)                        
    {  i = y0;
      y0 = y1;
      y1 = i;
    }

    if(y0==y1) 
    {  gui_line_hor(x0, y0, x1, color);
      return;
    }
    if(x0==x1) 
    {  gui_line_ver(x0, y0, y1, color);
      return;
    }

    while(y0<=y1)                        
    {  gui_line_hor(x0, y0, x1, color);    
      y0++;                            
    }
}


void  gui_square(uint16_t x0, uint16_t y0, uint16_t with, T_COLOR color)
{   if(with==0) return;
    if( (x0+with) > GUI_DISPLAY_X_MAX ) return;
    if( (y0+with) > GUI_DISPLAY_Y_MAX ) return;
    gui_rectangle(x0, y0, x0+with, y0+with, color);
}


void  gui_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color)
{  int16_t   dx;                        
   int16_t   dy;                      
   int8_t    dx_sym;                    
   int8_t    dy_sym;                    
   int16_t   dx_x2;                    
   int16_t   dy_x2;                    
   int16_t   di;                        
   
   
   dx = x1-x0;                        
   dy = y1-y0;
   
   
   if(dx>0)                            
   {  dx_sym = 1;                    
   }
   else
   {  if(dx<0)
      {  dx_sym = -1;                
      }
      else
      {  
         gui_line_ver(x0, y0, y1, color);
           return;
      }
   }
   
   if(dy>0)                            
   {  dy_sym = 1;                    
   }
   else
   {  if(dy<0)
      {  dy_sym = -1;                
      }
      else
      {  
         gui_line_hor(x0, y0, x1, color);
           return;
      }
   }
    
  
   dx = dx_sym * dx;
   dy = dy_sym * dy;
 
  
   dx_x2 = dx*2;
   dy_x2 = dy*2;
   
   /* Bresenham draw line */
   if(dx>=dy)                        
   {  di = dy_x2 - dx;
      while(x0!=x1)
      {  gui_point(x0, y0, color);
         x0 += dx_sym;
         if(di<0)
         {  di += dy_x2;            
         }
         else
         {  di += dy_x2 - dx_x2;
            y0 += dy_sym;
         }
      }
      gui_point(x0, y0, color);        
   }
   else                                
   {  di = dx_x2 - dy;
      while(y0!=y1)
      {  gui_point(x0, y0, color);
         y0 += dy_sym;
         if(di<0)
         {  di += dx_x2;
         }
         else
         {  di += dx_x2 - dy_x2;
            x0 += dx_sym;
         }
      }
      gui_point(x0, y0, color);        
   } 
  
}


void  gui_line_width(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, T_COLOR color)
{  int16_t   dx;                        
   int16_t   dy;                      
   int8_t    dx_sym;                    
   int8_t    dy_sym;                    
   int16_t   dx_x2;                    
   int16_t   dy_x2;                    
   int16_t   di;                        
   
   int16_t   wx, wy;                    
   int16_t   draw_a, draw_b;
   
   
   if(width==0) return;
   if(width>50) width = 50;
   
   dx = x1-x0;                        
   dy = y1-y0;
   
   wx = width/2;
   wy = width-wx-1;
   
  
   if(dx>0)                            
   {  dx_sym = 1;                    
   }
   else
   {  if(dx<0)
      {  dx_sym = -1;                
      }
      else
      {  
         wx = x0-wx;
         if(wx<0) wx = 0;
         wy = x0+wy;
         
         while(1)
         {  x0 = wx;
            gui_line_ver(x0, y0, y1, color);
            if(wx>=wy) break;
            wx++;
         }
         return;
      }
   }
   
   if(dy>0)                            
   {  dy_sym = 1;                    
   }
   else
   {  if(dy<0)
      {  dy_sym = -1;                
      }
      else
      { 
         wx = y0-wx;
         if(wx<0) wx = 0;
         wy = y0+wy;
         
         while(1)
         {  
            y0 = wx;
            gui_line_hor(x0, y0, x1, color);
            if(wx>=wy) break;
            wx++;
         }
         return;
      }
   }
    
  
   dx = dx_sym * dx;
   dy = dy_sym * dy;
 
   
   dx_x2 = dx*2;
   dy_x2 = dy*2;
   

   if(dx>=dy)                        
   {  di = dy_x2 - dx;
      while(x0!=x1)
      { 
         draw_a = y0-wx;
         if(draw_a<0) draw_a = 0;
         draw_b = y0+wy;
         gui_line_ver(x0, draw_a, draw_b, color);
         
         x0 += dx_sym;                
         if(di<0)
         {  di += dy_x2;            
         }
         else
         {  di += dy_x2 - dx_x2;
            y0 += dy_sym;
         }
      }
      draw_a = y0-wx;
      if(draw_a<0) draw_a = 0;
      draw_b = y0+wy;
      gui_line_ver(x0, draw_a, draw_b, color);
   }
   else                                
   {  di = dx_x2 - dy;
      while(y0!=y1)
      {  
         draw_a = x0-wx;
         if(draw_a<0) draw_a = 0;
         draw_b = x0+wy;
         gui_line_hor(draw_a, y0, draw_b, color);
         
         y0 += dy_sym;
         if(di<0)
         {  di += dx_x2;
         }
         else
         {  di += dx_x2 - dy_x2;
            x0 += dx_sym;
         }
      }
      draw_a = x0-wx;
      if(draw_a<0) draw_a = 0;
      draw_b = x0+wy;
      gui_line_hor(draw_a, y0, draw_b, color);
   } 
}



void  gui_line_s(uint16_t const *points, uint8_t no, T_COLOR color)
{  uint16_t  x0, y0;
   uint16_t  x1, y1;
   uint8_t  i;

  
   if(0==no) return;
   if(1==no)                    
   {  x0 = *points++;
      y0 = *points;
      gui_point(x0, y0, color);
   }
   
   
   x0 = *points++;                    
   y0 = *points++;
   for(i=1; i<no; i++)
   {  x1 = *points++;                
      y1 = *points++;
      gui_line(x0, y0, x1, y1, color);
      x0 = x1;                        
      y0 = y1;
   }
}

void  gui_circle(uint16_t x0, uint16_t y0, uint16_t r, T_COLOR color)
{  int16_t  draw_x0, draw_y0;            
   int16_t  draw_x1, draw_y1;    
   int16_t  draw_x2, draw_y2;    
   int16_t  draw_x3, draw_y3;    
   int16_t  draw_x4, draw_y4;    
   int16_t  draw_x5, draw_y5;    
   int16_t  draw_x6, draw_y6;    
   int16_t  draw_x7, draw_y7;    
   int16_t  xx, yy;                    
 
   int16_t  di;                        
   
   
   if(0==r) return;
   
   
   draw_x0 = draw_x1 = x0;
   draw_y0 = draw_y1 = y0 + r;
   if(draw_y0<GUI_DISPLAY_Y_MAX) gui_point(draw_x0, draw_y0, color);    
    
   draw_x2 = draw_x3 = x0;
   draw_y2 = draw_y3 = y0 - r;
   if(draw_y2>=0) gui_point(draw_x2, draw_y2, color);            
   
    
   draw_x4 = draw_x6 = x0 + r;
   draw_y4 = draw_y6 = y0;
   if(draw_x4<GUI_DISPLAY_X_MAX) gui_point(draw_x4, draw_y4, color);    
   
   draw_x5 = draw_x7 = x0 - r;
   draw_y5 = draw_y7 = y0;
   if(draw_x5>=0) gui_point(draw_x5, draw_y5, color);             
   if(1==r) return;                    
   
   
  
   di = 3 - 2*r;                
   
   xx = 0;
   yy = r;    
   while(xx<yy)
   {  if(di<0)
      {  di += 4*xx + 6;          
      }
      else
      {  di += 4*(xx - yy) + 10;
      
         yy--;      
         draw_y0--;
         draw_y1--;
         draw_y2++;
         draw_y3++;
         draw_x4--;
         draw_x5++;
         draw_x6--;
         draw_x7++;         
      }
      
      xx++;   
      draw_x0++;
      draw_x1--;
      draw_x2++;
      draw_x3--;
      draw_y4++;
      draw_y5++;
      draw_y6--;
      draw_y7--;
        
    
     
      if( (draw_x0<=GUI_DISPLAY_X_MAX)&&(draw_y0>=0) )    
      {  gui_point(draw_x0, draw_y0, color);
      }        
      if( (draw_x1>=0)&&(draw_y1>=0) )    
      {  gui_point(draw_x1, draw_y1, color);
      }
      if( (draw_x2<=GUI_DISPLAY_X_MAX)&&(draw_y2<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x2, draw_y2, color);   
      }
      if( (draw_x3>=0)&&(draw_y3<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x3, draw_y3, color);
      }
      if( (draw_x4<=GUI_DISPLAY_X_MAX)&&(draw_y4>=0) )    
      {  gui_point(draw_x4, draw_y4, color);
      }
      if( (draw_x5>=0)&&(draw_y5>=0) )    
      {  gui_point(draw_x5, draw_y5, color);
      }
      if( (draw_x6<=GUI_DISPLAY_X_MAX)&&(draw_y6<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x6, draw_y6, color);
      }
      if( (draw_x7>=0)&&(draw_y7<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x7, draw_y7, color);
      }
   }
}


void  gui_circle_fill(uint16_t x0, uint16_t y0, uint16_t r, T_COLOR color)
{  int16_t  draw_x0, draw_y0;            
   int16_t  draw_x1, draw_y1;    
   int16_t  draw_x2, draw_y2;    
   int16_t  draw_x3, draw_y3;    
   int16_t  draw_x4, draw_y4;    
   int16_t  draw_x5, draw_y5;    
   int16_t  draw_x6, draw_y6;    
   int16_t  draw_x7, draw_y7;    
   int16_t  fill_x0, fill_y0;            
   int16_t  fill_x1;
   int16_t  xx, yy;                    
 
   int16_t  di;                        
   
 
   if(0==r) return;
   
   draw_x0 = draw_x1 = x0;
   draw_y0 = draw_y1 = y0 + r;
   if(draw_y0<GUI_DISPLAY_Y_MAX)
   {  gui_point(draw_x0, draw_y0, color);    
   }
        
   draw_x2 = draw_x3 = x0;
   draw_y2 = draw_y3 = y0 - r;
   if(draw_y2>=0)
   {  gui_point(draw_x2, draw_y2, color);    
   }
      
   draw_x4 = draw_x6 = x0 + r;
   draw_y4 = draw_y6 = y0;
   if(draw_x4<GUI_DISPLAY_X_MAX) 
   {  gui_point(draw_x4, draw_y4, color);    
      fill_x1 = draw_x4;
   }
   else
   {  fill_x1 = GUI_DISPLAY_X_MAX;
   }
   fill_y0 = y0;                            
   fill_x0 = x0 - r;                    
   if(fill_x0<0) fill_x0 = 0;
   gui_line_hor(fill_x0, fill_y0, fill_x1, color);
   
   draw_x5 = draw_x7 = x0 - r;
   draw_y5 = draw_y7 = y0;
   if(draw_x5>=0) 
   {  gui_point(draw_x5, draw_y5, color);    
   }
   if(1==r) return;
   
   
   
   di = 3 - 2*r;                            
   
   xx = 0;
   yy = r;
   while(xx<yy)
   {  if(di<0)
      {  di += 4*xx + 6;
      }
      else
      {  di += 4*(xx - yy) + 10;
      
         yy--;      
         draw_y0--;
         draw_y1--;
         draw_y2++;
         draw_y3++;
         draw_x4--;
         draw_x5++;
         draw_x6--;
         draw_x7++;         
      }
      
      xx++;   
      draw_x0++;
      draw_x1--;
      draw_x2++;
      draw_x3--;
      draw_y4++;
      draw_y5++;
      draw_y6--;
      draw_y7--;
        
    
      
      if( (draw_x0<=GUI_DISPLAY_X_MAX)&&(draw_y0>=0) )    
      {  gui_point(draw_x0, draw_y0, color);
      }        
      if( (draw_x1>=0)&&(draw_y1>=0) )    
      {  gui_point(draw_x1, draw_y1, color);
      }
      
      
      if(draw_x1>=0)
      { 
         fill_x0 = draw_x1;
        
         fill_y0 = draw_y1;
         if(fill_y0>GUI_DISPLAY_Y_MAX) fill_y0 = GUI_DISPLAY_Y_MAX;
         if(fill_y0<0) fill_y0 = 0; 
                                         
         fill_x1 = x0*2 - draw_x1;                
         if(fill_x1>GUI_DISPLAY_X_MAX) fill_x1 = GUI_DISPLAY_X_MAX;
         gui_line_hor(fill_x0, fill_y0, fill_x1, color);
      }
      
      
      if( (draw_x2<=GUI_DISPLAY_X_MAX)&&(draw_y2<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x2, draw_y2, color);   
      }
              
      if( (draw_x3>=0)&&(draw_y3<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x3, draw_y3, color);
      }
      
      
      if(draw_x3>=0)
      {  
         fill_x0 = draw_x3;
         
         fill_y0 = draw_y3;
         if(fill_y0>GUI_DISPLAY_Y_MAX) fill_y0 = GUI_DISPLAY_Y_MAX;
         if(fill_y0<0) fill_y0 = 0;
                                            
         fill_x1 = x0*2 - draw_x3;                
         if(fill_x1>GUI_DISPLAY_X_MAX) fill_x1 = GUI_DISPLAY_X_MAX;
         gui_line_hor(fill_x0, fill_y0, fill_x1, color);
      }
      
            
      if( (draw_x4<=GUI_DISPLAY_X_MAX)&&(draw_y4>=0) )    
      {  gui_point(draw_x4, draw_y4, color);
      }
      if( (draw_x5>=0)&&(draw_y5>=0) )    
      {  gui_point(draw_x5, draw_y5, color);
      }
      
     
      if(draw_x5>=0)
      { 
         fill_x0 = draw_x5;
         
         fill_y0 = draw_y5;
         if(fill_y0>GUI_DISPLAY_Y_MAX) fill_y0 = GUI_DISPLAY_Y_MAX;
         if(fill_y0<0) fill_y0 = 0;
                                            
         fill_x1 = x0*2 - draw_x5;                
         if(fill_x1>GUI_DISPLAY_X_MAX) fill_x1 = GUI_DISPLAY_X_MAX;
         gui_line_hor(fill_x0, fill_y0, fill_x1, color);
      }
      
      
      if( (draw_x6<=GUI_DISPLAY_X_MAX)&&(draw_y6<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x6, draw_y6, color);
      }
      
      if( (draw_x7>=0)&&(draw_y7<=GUI_DISPLAY_Y_MAX) )    
      {  gui_point(draw_x7, draw_y7, color);
      }
      
     
      if(draw_x7>=0)
      { 
         fill_x0 = draw_x7;
         
         fill_y0 = draw_y7;
         if(fill_y0>GUI_DISPLAY_Y_MAX) fill_y0 = GUI_DISPLAY_Y_MAX;
         if(fill_y0<0) fill_y0 = 0;
                                             
         fill_x1 = x0*2 - draw_x7;                
         if(fill_x1>GUI_DISPLAY_X_MAX) fill_x1 = GUI_DISPLAY_X_MAX;
         gui_line_hor(fill_x0, fill_y0, fill_x1, color);
      }
      
   }
}



void  gui_ellipse(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, T_COLOR color)
{  int16_t  draw_x0, draw_y0;            
   int16_t  draw_x1, draw_y1;
   int16_t  draw_x2, draw_y2;
   int16_t  draw_x3, draw_y3;
   int16_t  xx, yy;                    
    
   int16_t  center_x, center_y;        
   int16_t  radius_x, radius_y;        
   int16_t  radius_xx, radius_yy;        
   int16_t  radius_xx2, radius_yy2;    
   int16_t  di;                        
    
  
   if( (x0==x1) || (y0==y1) ) return;
       
   
   center_x = (x0 + x1) >> 1;            
   center_y = (y0 + y1) >> 1;
   
   
   if(x0 > x1)
   {  radius_x = (x0 - x1) >> 1;
   }
   else
   {  radius_x = (x1 - x0) >> 1;
   }
   if(y0 > y1)
   {  radius_y = (y0 - y1) >> 1;
   }
   else
   {  radius_y = (y1 - y0) >> 1;
   }
        
   
   radius_xx = radius_x * radius_x;
   radius_yy = radius_y * radius_y;
    
   
   radius_xx2 = radius_xx<<1;
   radius_yy2 = radius_yy<<1;
    
   
   xx = 0;
   yy = radius_y;
  
   di = radius_yy2 + radius_xx - radius_xx2*radius_y ;    
    
  
   draw_x0 = draw_x1 = draw_x2 = draw_x3 = center_x;
   draw_y0 = draw_y1 = center_y + radius_y;
   draw_y2 = draw_y3 = center_y - radius_y;
  
     
   gui_point(draw_x0, draw_y0, color);                    
   gui_point(draw_x2, draw_y2, color);
    
   while( (radius_yy*xx) < (radius_xx*yy) ) 
   {  if(di<0)
      {  di+= radius_yy2*(2*xx+3);
      }
      else
      {  di += radius_yy2*(2*xx+3) + 4*radius_xx - 4*radius_xx*yy;
           
         yy--;
         draw_y0--;
         draw_y1--;
         draw_y2++;
         draw_y3++;                 
      }
      
      xx ++;                        
             
      draw_x0++;
      draw_x1--;
      draw_x2++;
      draw_x3--;
        
      gui_point(draw_x0, draw_y0, color);
      gui_point(draw_x1, draw_y1, color);
      gui_point(draw_x2, draw_y2, color);
      gui_point(draw_x3, draw_y3, color);
   }
  
   di = radius_xx2*(yy-1)*(yy-1) + radius_yy2*xx*xx + radius_yy + radius_yy2*xx - radius_xx2*radius_yy;
   while(yy>=0) 
   {  if(di<0)
      {  di+= radius_xx2*3 + 4*radius_yy*xx + 4*radius_yy - 2*radius_xx2*yy;
           
         xx ++;                                
         draw_x0++;
         draw_x1--;
         draw_x2++;
         draw_x3--;  
      }
      else
      {  di += radius_xx2*3 - 2*radius_xx2*yy;                                    
      }
      
      yy--;
       draw_y0--;
      draw_y1--;
      draw_y2++;
      draw_y3++;    
        
      gui_point(draw_x0, draw_y0, color);
      gui_point(draw_x1, draw_y1, color);
      gui_point(draw_x2, draw_y2, color);
      gui_point(draw_x3, draw_y3, color);
   }     
}


void  gui_ellipse_fill(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, T_COLOR color)
{  int16_t  draw_x0, draw_y0;            
   int16_t  draw_x1, draw_y1;
   int16_t  draw_x2, draw_y2;
   int16_t  draw_x3, draw_y3;
   int16_t  xx, yy;                    
    
   int16_t  center_x, center_y;        
   int16_t  radius_x, radius_y;        
   int16_t  radius_xx, radius_yy;        
   int16_t  radius_xx2, radius_yy2;
   int16_t  di;                        
    
   
   if( (x0==x1) || (y0==y1) ) return;
   
   
   center_x = (x0 + x1) >> 1;            
   center_y = (y0 + y1) >> 1;
   
   
   if(x0 > x1)
   {  radius_x = (x0 - x1) >> 1;
   }
   else
   {  radius_x = (x1 - x0) >> 1;
   }
   if(y0 > y1)
   {  radius_y = (y0 - y1) >> 1;
   }
   else
   {  radius_y = (y1 - y0) >> 1;
   }
        
   
   radius_xx = radius_x * radius_x;
   radius_yy = radius_y * radius_y;
    
   
   radius_xx2 = radius_xx<<1;
   radius_yy2 = radius_yy<<1;
   
   
   xx = 0;
   yy = radius_y;
  
   di = radius_yy2 + radius_xx - radius_xx2*radius_y ;    
    
   
   draw_x0 = draw_x1 = draw_x2 = draw_x3 = center_x;
   draw_y0 = draw_y1 = center_y + radius_y;
   draw_y2 = draw_y3 = center_y - radius_y;
  
     
   gui_point(draw_x0, draw_y0, color);                    
   gui_point(draw_x2, draw_y2, color);
    
   while( (radius_yy*xx) < (radius_xx*yy) ) 
   {  if(di<0)
      {  di+= radius_yy2*(2*xx+3);
      }
      else
      {  di += radius_yy2*(2*xx+3) + 4*radius_xx - 4*radius_xx*yy;
           
         yy--;
         draw_y0--;
         draw_y1--;
         draw_y2++;
         draw_y3++;                 
      }
      
      xx ++;                        
             
      draw_x0++;
      draw_x1--;
      draw_x2++;
      draw_x3--;
        
      gui_point(draw_x0, draw_y0, color);
      gui_point(draw_x1, draw_y1, color);
      gui_point(draw_x2, draw_y2, color);
      gui_point(draw_x3, draw_y3, color);
      
     
      if(di>=0)
      {  gui_line_hor(draw_x0, draw_y0, draw_x1, color);
         gui_line_hor(draw_x2, draw_y2, draw_x3, color);
      }
   }
  
   di = radius_xx2*(yy-1)*(yy-1) + radius_yy2*xx*xx + radius_yy + radius_yy2*xx - radius_xx2*radius_yy;
   while(yy>=0) 
   {  if(di<0)
      {  di+= radius_xx2*3 + 4*radius_yy*xx + 4*radius_yy - 2*radius_xx2*yy;
           
         xx ++;                                
         draw_x0++;
         draw_x1--;
         draw_x2++;
         draw_x3--;  
      }
      else
      {  di += radius_xx2*3 - 2*radius_xx2*yy;                                    
      }
      
      yy--;
       draw_y0--;
      draw_y1--;
      draw_y2++;
      draw_y3++;    
        
      gui_point(draw_x0, draw_y0, color);
      gui_point(draw_x1, draw_y1, color);
      gui_point(draw_x2, draw_y2, color);
      gui_point(draw_x3, draw_y3, color);
      
      
      gui_line_hor(draw_x0, draw_y0, draw_x1, color);
      gui_line_hor(draw_x2, draw_y2, draw_x3, color); 
   }     
}


void  gui_arc4(uint16_t x, uint16_t y, uint16_t r, uint8_t angle, T_COLOR color)
{  int16_t  draw_x, draw_y;

   int16_t  op_x, op_y;
   int16_t  op_2rr;
   
   if(r==0) return;
   
   op_2rr = 2*r*r;                                        
   
   switch(angle)
   {  case  1:
            draw_x = x+r;
            draw_y = y;
            
            op_x = r;
            op_y = 0;
 
            while(1)
            {  gui_point(draw_x, draw_y, color);        
                 
              
               op_y++;
               draw_y++;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )     
               {  op_x--;
                  draw_x--;
               }
               if(op_y>=op_x) break;
            }
            while(1)
            {  gui_point(draw_x, draw_y, color);        
                 
               op_x--;
               draw_x--;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_y +1)<=0 ) 
               {  op_y++;
                  draw_y++;
               }
               if(op_x<=0)
               {  gui_point(draw_x, draw_y, color);        
                  break;
               }
            }
   
            break;      
                   
      case  2:
            draw_x = x-r;
            draw_y = y;
            
            op_x = r;
            op_y = 0;
 
            while(1)
            {  gui_point(draw_x, draw_y, color);        
                 
               op_y++;
               draw_y++;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )     
               {  op_x--;
                  draw_x++;
               }
               if(op_y>=op_x) break;
            }
            while(1)
            {  gui_point(draw_x, draw_y, color);        
                 
               op_x--;
               draw_x++;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_y +1)<=0 )     
               {  op_y++;
                  draw_y++;
               }
               if(op_x<=0)
               {  gui_point(draw_x, draw_y, color);        
                  break;
               }
            }
  
            break;
            
      case  3:
            draw_x = x-r;
            draw_y = y;
            
            op_x = r;
            op_y = 0;
 
            while(1)
            {  gui_point(draw_x, draw_y, color);        
                 
               op_y++;
               draw_y--;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )     
               {  op_x--;
                  draw_x++;
               }
               if(op_y>=op_x) break;
            }
            while(1)
            {  gui_point(draw_x, draw_y, color);        
                 
               op_x--;
               draw_x++;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_y +1)<=0 )     
               {  op_y++;
                  draw_y--;
               }
               if(op_x<=0)
               {  gui_point(draw_x, draw_y, color);    
                  break;
               }
            }
      
            break;
            
      case  4:
            draw_x = x+r;
            draw_y = y;
            
            op_x = r;
            op_y = 0;
 
            while(1)
            {  gui_point(draw_x, draw_y, color);
                 
               op_y++;
               draw_y--;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )
               {  op_x--;
                  draw_x--;
               }
               if(op_y>=op_x) break;
            }
            while(1)
            {  gui_point(draw_x, draw_y, color);
                 
               op_x--;
               draw_x--;
               if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_y +1)<=0 )
               {  op_y++;
                  draw_y--;
               }
               if(op_x<=0)
               {  gui_point(draw_x, draw_y, color);
                  break;
               }
            }
            break;
            
      default:
            break;
      
   }

}

void  gui_arc(uint16_t x, uint16_t y, uint16_t r, uint16_t stangle, uint16_t endangle, T_COLOR color)
{  int16_t  draw_x, draw_y;                    
   int16_t  op_x, op_y;                    
   int16_t  op_2rr;    
   
   int16_t  pno_angle;
   uint8_t  draw_on;
   
   if(r==0) return;
   if(stangle==endangle) return;
   if( (stangle>=360) || (endangle>=360) ) return;

   op_2rr = 2*r*r;
   pno_angle = 0;
    
   op_x = r;
   op_y = 0;
   while(1)
   {  pno_angle++;
      op_y++;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )
      {  op_x--;
      }
      if(op_y>=op_x) break;
   }
   
   draw_on = 0;

   if(endangle>stangle) draw_on = 1;
   stangle = (360-stangle)*pno_angle/45;
   endangle = (360-endangle)*pno_angle/45;
   if(stangle==0) stangle=1;
   if(endangle==0) endangle=1;
   
   pno_angle = 0;
   
   draw_x = x+r;
   draw_y = y;         
   op_x = r;
   op_y = 0;
   while(1)
   { 
      op_y++;
      draw_y--;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )
      {  op_x--;
         draw_x--;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      if(op_y>=op_x)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
   
   while(1)
   { 
      op_x--;
      draw_x--;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_y +1)<=0 )
      {  op_y++;
         draw_y--;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      
      if(op_x<=0)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
    
    
   draw_y = y-r;
   draw_x = x;         
   op_y = r;
   op_x = 0;
   while(1)
   { 
      op_x++;
      draw_x--;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_y +1)>0 )
      {  op_y--;
         draw_y++;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      
      if(op_x>=op_y)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
   
   while(1)
   {  
      op_y--;
      draw_y++;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_x +1)<=0 )
      {  op_x++;
         draw_x--;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      if(op_y<=0)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
   
   draw_x = x-r;
   draw_y = y;         
   op_x = r;
   op_y = 0;
   while(1)
   { 
      op_y++;
      draw_y++;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_x +1)>0 )
      {  op_x--;
         draw_x++;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      if(op_y>=op_x)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
   
   while(1)
   { 
      op_x--;
      draw_x++;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_y +1)<=0 )
      {  op_y++;
         draw_y++;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      
      if(op_x<=0)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
  
   draw_y = y+r;
   draw_x = x;         
   op_y = r;
   op_x = 0;
   while(1)
   {
      op_x++;
      draw_x++;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr - 2*op_y +1)>0 )
      {  op_y--;
         draw_y--;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      
      if(op_x>=op_y)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
   
   while(1)
   { 
      op_y--;
      draw_y--;
      if( (2*op_x*op_x + 2*op_y*op_y - op_2rr + 2*op_x +1)<=0 )
      {  op_x++;
         draw_x++;
      }
      if(draw_on==1) gui_point(draw_x, draw_y, color);
      pno_angle++;
      if( (pno_angle==stangle)||(pno_angle==endangle) )
      {  draw_on = 1-draw_on;
         if(draw_on==1) gui_point(draw_x, draw_y, color);
      } 
      if(op_y<=0)
      {  if(draw_on==1) gui_point(draw_x, draw_y, color);
         break;
      }
   }
   
}






















