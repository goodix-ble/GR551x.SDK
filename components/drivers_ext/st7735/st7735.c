/**
 *****************************************************************************************
 *
 * @file st7533.c
 *
 * @brief LCD controller driver of st7735 IC.
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
#include "st7735.h"
#include "st7735_config.h"
#include <string.h>
#include <stdbool.h>

 /*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint16_t g_lcd_gram[LCD_YMAX][LCD_XMAX];
uint16_t g_lcd_buffer[LCD_YMAX][LCD_XMAX];

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint16_t (*s_lcd_memory)[LCD_XMAX] = g_lcd_gram;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void lcd_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    st7735_write_cmd(0x2a);
    st7735_write_data(0x00);
    st7735_write_data(x0+0x02);
    st7735_write_data(0x00);
    st7735_write_data(x1+0x02);
    
    st7735_write_cmd(0x2b);
    st7735_write_data(0x00);
    st7735_write_data(y0+0x03);
    st7735_write_data(0x00);
    st7735_write_data(y1+0x03);
    
    st7735_write_cmd(0x2C);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void lcd_set_memory(bool gram_set)
{
    if(gram_set)
    {
        s_lcd_memory = g_lcd_gram;
    }
    else
    {
        s_lcd_memory = g_lcd_buffer;
    }
}

void lcd_draw_point(uint8_t x, uint8_t y, uint16_t color)
{
    if(x > (LCD_XMAX-1) || y > (LCD_YMAX-1))
       return; 
    s_lcd_memory[y][x] = color;
}


void lcd_fill_mem(uint16_t color)   
{
    uint8_t x;
    uint8_t y;
    for(y=0; y<LCD_YMAX; y++)
    {
        for(x=0; x<LCD_XMAX; x++)
        {
            s_lcd_memory[y][x] = color;
        }
    }
}

void lcd_rectangle_fill_mem(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color) 
{
    uint8_t x;
    uint8_t y;
    for(y=y0; y<y1; y++)
    {
        for(x=x0; x<x1; x++)
        {
            s_lcd_memory[y][x] = color;
        }
    }
}    

void lcd_refresh(void)
{
    lcd_set_memory(true);
    lcd_set_window(0, 0, LCD_XMAX-1, LCD_YMAX-1); 
    st7735_write_buffer((uint8_t*)&s_lcd_memory[0][0], LCD_XMAX*LCD_YMAX*2);
}

void lcd_rectangle_refresh(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    lcd_set_memory(true);
    lcd_set_window(x0, y0, x1, y1); 
    st7735_write_buffer((uint8_t*)&s_lcd_memory[y0][x0], (x1-x0)*(y1-y0)*2);
}


uint16_t lcd_read_point(uint8_t x, uint8_t y)
{
    return s_lcd_memory[y][x];
}

void lcd_init(void)   
{   
    st7735_init();
    //--------------------------------End ST7735S Reset Sequence --------------------------------------// 
    st7735_write_cmd(0x11); 
    st7735_delay(120);    
    //------------------------------------ST7735S Frame Rate-----------------------------------------// 
    st7735_write_cmd(0xB1); 
    st7735_write_data(0x05); 
    st7735_write_data(0x3C); 
    st7735_write_data(0x3C); 
    st7735_write_cmd(0xB2); 
    st7735_write_data(0x05); 
    st7735_write_data(0x3C); 
    st7735_write_data(0x3C); 
    st7735_write_cmd(0xB3); 
    st7735_write_data(0x05); 
    st7735_write_data(0x3C); 
    st7735_write_data(0x3C); 
    st7735_write_data(0x05); 
    st7735_write_data(0x3C); 
    st7735_write_data(0x3C); 
    //------------------------------------End ST7735S Frame Rate---------------------------------// 
    st7735_write_cmd(0xB4);
    st7735_write_data(0x03); 
    //------------------------------------ST7735S Power Sequence---------------------------------// 
    st7735_write_cmd(0xC0); 
    st7735_write_data(0x28); 
    st7735_write_data(0x08); 
    st7735_write_data(0x04); 
    st7735_write_cmd(0xC1); 
    st7735_write_data(0XC0); 
    st7735_write_cmd(0xC2); 
    st7735_write_data(0x0D); 
    st7735_write_data(0x00); 
    st7735_write_cmd(0xC3); 
    st7735_write_data(0x8D); 
    st7735_write_data(0x2A); 
    st7735_write_cmd(0xC4); 
    st7735_write_data(0x8D); 
    st7735_write_data(0xEE); 
    //---------------------------------End ST7735S Power Sequence-------------------------------------// 
    st7735_write_cmd(0xC5); 
    st7735_write_data(0x1A); 
    st7735_write_cmd(0x36); 
    st7735_write_data(0xC8); 
    //------------------------------------ST7735S Gamma Sequence---------------------------------// 
    st7735_write_cmd(0xE0);
    st7735_write_data(0x04);
    st7735_write_data(0x22);
    st7735_write_data(0x07);
    st7735_write_data(0x0A);
    st7735_write_data(0x2E);
    st7735_write_data(0x30);
    st7735_write_data(0x25);
    st7735_write_data(0x2A);
    st7735_write_data(0x28);
    st7735_write_data(0x26);
    st7735_write_data(0x2E);
    st7735_write_data(0x3A);
    st7735_write_data(0x00);
    st7735_write_data(0x01);
    st7735_write_data(0x03); 
    st7735_write_data(0x13); 
    st7735_write_cmd(0xE1); 
    st7735_write_data(0x04); 
    st7735_write_data(0x16); 
    st7735_write_data(0x06); 
    st7735_write_data(0x0D); 
    st7735_write_data(0x2D); 
    st7735_write_data(0x26); 
    st7735_write_data(0x23); 
    st7735_write_data(0x27); 
    st7735_write_data(0x27); 
    st7735_write_data(0x25); 
    st7735_write_data(0x2D); 
    st7735_write_data(0x3B); 
    st7735_write_data(0x00); 
    st7735_write_data(0x01); 
    st7735_write_data(0x04); 
    st7735_write_data(0x13); 
    //------------------------------------End ST7735S Gamma Sequence-----------------------------// 
    st7735_write_cmd(0x3A); 
    st7735_write_data(0x05); 
    st7735_write_cmd(0x29);
}

