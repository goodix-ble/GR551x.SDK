/**
 *****************************************************************************************
 *
 * @file gui_color.h
 *
 * @brief Set gui color API
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

#ifndef  GUI_COLOR_H
#define  GUI_COLOR_H

#include "gui_config.h"

/**
 *****************************************************************************************
 * @brief 
 * @note  Hexadecimal bit conversion.
 *
 * @param[in] dcb:  DCB data.
 *
 * @return Hex data
 *****************************************************************************************
 */
uint8_t gui_dcb_to_hex(uint8_t dcb);

/**
 *****************************************************************************************
 * @brief Set display and background color.
 *
 * @param[in] disp_color:  Display color.
 * @param[in] back_color:  Background color.
 *****************************************************************************************
 */
void gui_set_color(T_COLOR disp_color, T_COLOR back_color);

/**
 *****************************************************************************************
 * @brief Get background color
 *
 * @return Background color
 *****************************************************************************************
 */
T_COLOR  gui_get_back_color(void);

/**
 *****************************************************************************************
 * @brief Get display color
 *
 * @return Display color
 *****************************************************************************************
 */
T_COLOR  gui_get_disp_color(void);

/**
 *****************************************************************************************
 * @brief Swap foreground and background colors for reverse display
 *****************************************************************************************
 */
void  gui_exchange_color(void);

/**
 *****************************************************************************************
 * @brief Based on the current color settings,draw point
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] font_dat: one byte point.
 * @param[in] num: point count.
 *****************************************************************************************
 */
void gui_point_color(uint8_t x, uint8_t y, uint8_t font_dat, uint8_t num);

#endif
