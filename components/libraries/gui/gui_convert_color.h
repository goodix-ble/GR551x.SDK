/**
 *****************************************************************************************
 *
 * @file gui_convert_color.h
 *
 * @brief Convert color API
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


#ifndef  GUI_CONVERT_COLOR_H
#define  GUI_CONVERT_COLOR_H

#include "gui_config.h"

#if CONVERT_COLOR_EN==1

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 16-bit index value.
 * @note  The convert value applies to a 64K color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 16-bit index value(64K color, d15-d11 for R value, d10-d5 for G value, d4-d0 for B value)
 *****************************************************************************************
 */
uint16_t  gui_color_to_index_565(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 16-bit index value to a RGB value.
 * @note  The convert value applies to a 64K color LCD screen.
 *
 * @param[in] index:  16 bit index value.
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_565(uint16_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 15-bit index value.
 * @note  The convert value applies to a 32K color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 15-bit index value.
 *****************************************************************************************
 */
uint16_t  gui_color_to_index_555(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 15-bit index value to a RGB value.
 * @note  The convert value applies to a 32K color LCD screen.
 *
 * @param[in] index:  15 bit index value.
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_555(uint16_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 12-bit index value.
 * @note  The convert value applies to a 4096 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 12-bit index value.(RRRRGGGGBBBB)
 *****************************************************************************************
 */
uint16_t  gui_color_to_index_444(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 12-bit index value to a RGB value.
 * @note  The convert value applies to a 4096 color LCD screen.
 *
 * @param[in] index:  12 bit index value.(RRRRGGGGBBBB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_444(uint16_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 8-bit index value.
 * @note  The convert value applies to a 256 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 8-bit index value.(RRRGGGBB)
 *****************************************************************************************
 */
uint8_t   gui_color_to_index_332(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 8-bit index value to a RGB value.
 * @note  The convert value applies to a 256 color LCD screen.
 *
 * @param[in] index:  8 bit index value.(RRRGGGBB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_332(uint8_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 6-bit index value.
 * @note  The convert value applies to a 256 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 6-bit index value.(RRGGBB)
 *****************************************************************************************
 */
uint8_t   gui_color_to_index_222(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 6-bit index value to a RGB value.
 * @note  The convert value applies to a 64K color LCD screen.
 *
 * @param[in] index:  6 bit index value.(RRGGBB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_222(uint8_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 3-bit index value.
 * @note  The convert value applies to a 64 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 3-bit index value.(RGB)
 *****************************************************************************************
 */
uint8_t   gui_color_to_index_111(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 3-bit index value to a RGB value.
 * @note  The convert value applies to a 64 color LCD screen.
 *
 * @param[in] index:  3 bit index value.(RGB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_111(uint8_t index);
#endif

#endif



