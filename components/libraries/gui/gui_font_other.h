/**
 *****************************************************************************************
 *
 * @file gui_font_other.h
 *
 * @brief Show other modulus font API
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

#ifndef _GUI_FONT_OTHER_H_
#define _GUI_FONT_OTHER_H_

#include "gui_config.h"

#if FONT_OTHER_EN==1
/**
 *****************************************************************************************
 * @brief Display one 16x32 Modulus font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:         X coordinate.
 * @param[in] y:         Y coordinate.
 * @param[in] code16_32: Array of font.
 * @param[in] index:     Index of array of font.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */
uint8_t gui_put_char16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t index);

/**
 *****************************************************************************************
 * @brief Display an Array font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] code16_32: Array of font.
 * @param[in] num:     Show the total number.
 *****************************************************************************************
 */
void gui_put_string16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t num);

/**
 *****************************************************************************************
 * @brief Display one 32x64 Modulus font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:         X coordinate.
 * @param[in] y:         Y coordinate.
 * @param[in] code32_64: Array of font.
 * @param[in] index:     Index of array of font.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */
uint8_t gui_put_char32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t index);

/**
 *****************************************************************************************
 * @brief Display an Array font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] code32_64: Array of font.
 * @param[in] num:     Show the total number.
 *****************************************************************************************
 */
void gui_put_string32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t num);

/**
 *****************************************************************************************
 * @brief Display an Array code.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] code_other: Array of picture code or custom font code.
 * @param[in] x_size:     picture x pixel
 * @param[in] y_size:     picture y pixel
 *****************************************************************************************
 */
void gui_put_code_other(uint16_t x, uint16_t y, const uint8_t*code_other, uint16_t x_size, uint16_t y_size);
#endif
#endif

