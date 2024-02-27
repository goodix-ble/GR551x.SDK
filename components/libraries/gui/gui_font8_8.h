/**
 *****************************************************************************************
 *
 * @file gui_font8_8.h
 *
 * @brief Show 8*8 ascii API
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



#ifndef  FONT8_8_H
#define  FONT8_8_H

#include "gui_config.h"

#if FONT8x8_EN==1

/**
 *****************************************************************************************
 * @brief Display one ascii.
 * @note  Display value between 0x20 to 0x7f, others show ' '.
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] ch: Display ascii.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */
uint8_t  gui_put_char8_8(uint16_t x, uint16_t y, uint8_t ch);

/**
 *****************************************************************************************
 * @brief Display one ascii string.
 * @note  No line feed.
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] p_str: Display ascii string.
 *****************************************************************************************
 */
void  gui_put_string8_8(uint16_t x, uint16_t y, char *p_str);

/**
 *****************************************************************************************
 * @brief Display num.
 *
 * @param[in] x:    X coordinate.
 * @param[in] y:    Y coordinate.
 * @param[in] num:  Display num.
 * @param[in] len:  Display len.
 * @param[in] mode: 0x01--the 0 in front of the value will be displayed
 *                  0x00--the 0 in front of the value will not be displayed
 *****************************************************************************************
 */
void gui_put_num8_8(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode);

#endif

#endif
