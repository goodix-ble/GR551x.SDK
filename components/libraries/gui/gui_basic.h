/**
 *****************************************************************************************
 *
 * @file gui_basic.h
 *
 * @brief Gui basic function API
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


#ifndef  GUI_BASIC_H
#define  GUI_BASIC_H

#include "gui_config.h"


uint32_t gui_pow(uint32_t m,uint8_t n);

/**
 *****************************************************************************************
 * @brief Draw a horizontal line.
 *
 * @param[in] x0:  X coordinate at the beginning of the horizontal line.
 * @param[in] y0:  Y coordinate at the beginning of the horizontal line.
 * @param[in] x1:  X coordinate at the end of the horizontal line.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_line_hor(uint16_t x0, uint8_t y0, uint16_t x1, T_COLOR color);   

/**
 *****************************************************************************************
 * @brief Draw a vertical line.
 *
 * @param[in] x0:  X coordinate at the beginning of the vertical line.
 * @param[in] y0:  Y coordinate at the beginning of the vertical line.
 * @param[in] x1:  X coordinate at the end of the vertical line.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_line_ver(uint16_t x0, uint8_t y0, uint8_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a rectangle.
 *
 * @param[in] x0:  The x coordinate of the upper-left corner of the rectangle.
 * @param[in] y0:  The y coordinate of the upper-left corner of the rectangle.
 * @param[in] x1:  The x coordinate of the bottom right corner of the rectangle.
 * @param[in] y1:  The y coordinate of the bottom right corner of the rectangle.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_rectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a filled rectangle.
 *
 * @param[in] x0:  The x coordinate of the upper-left corner of the rectangle.
 * @param[in] y0:  The y coordinate of the upper-left corner of the rectangle.
 * @param[in] x1:  The x coordinate of the bottom right corner of the rectangle.
 * @param[in] y1:  The y coordinate of the bottom right corner of the rectangle.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_rectangle_fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a square.
 *
 * @param[in] x0:     The x coordinate of the upper-left corner of the rectangle.
 * @param[in] y0:     The y coordinate of the upper-left corner of the rectangle.
 * @param[in] with:   With of square.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_square(uint16_t x0, uint16_t y0, uint16_t with, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a line between any two points.
 *
 * @param[in] x0:  The x coordinate of the starting of the line.
 * @param[in] y0:  The y coordinate of the starting of the line.
 * @param[in] x1:  The x coordinate of the end of the line.
 * @param[in] y1:  The y coordinate of the end of the line.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a line between any two points and can set width.
 *
 * @param[in] x0:  The x coordinate of the starting of the line.
 * @param[in] y0:  The y coordinate of the starting of the line.
 * @param[in] x1:  The x coordinate of the end of the line.
 * @param[in] y1:  The y coordinate of the end of the line.
 * @param[in] width:   Line width.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_line_width(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a line of points.
 *
 * @param[in] points:  line points.
 * @param[in] no:  count of points.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_line_s(uint16_t const *points, uint8_t no, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a circle.
 *
 * @param[in] x0:  The x coordinate of the center of the circle.
 * @param[in] y0:  The y coordinate of the center of the circle.
 * @param[in] r:   The radius of the circle
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_circle(uint16_t x0, uint16_t y0, uint16_t r, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a fill circle.
 *
 * @param[in] x0:  The x coordinate of the center of the circle.
 * @param[in] y0:  The y coordinate of the center of the circle.
 * @param[in] r:   The radius of the circle
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_circle_fill(uint16_t x0, uint16_t y0, uint16_t r, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a ellipse.
 *
 * @param[in] x0:  The x coordinate of the left of the ellipse.
 * @param[in] x1:  The x coordinate on the right of the ellipse.
 * @param[in] y0:  The y coordinate on the up of the ellipse.
 * @param[in] y1:  The y coordinate on the down of the ellipse.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_ellipse(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a fill ellipse.
 *
 * @param[in] x0:  The x coordinate of the left of the ellipse.
 * @param[in] x1:  The x coordinate on the right of the ellipse.
 * @param[in] y0:  The y coordinate on the up of the ellipse.
 * @param[in] y1:  The y coordinate on the down of the ellipse.
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_ellipse_fill(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a arc.
 * @note  The angle must be 0,90,-90,-180,180,-270,270
 *
 * @param[in] x:  The x coordinate of the center of the arc.
 * @param[in] y:  The y coordinate of the center of the arc.
 * @param[in] r:  The radius of the arc
 * @param[in] angle: The angle of the arc
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_arc4(uint16_t x, uint16_t y, uint16_t r, uint8_t angle, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a any angle arc.
 *
 * @param[in] x:  The x coordinate of the center of the arc.
 * @param[in] y:  The y coordinate of the center of the arc.
 * @param[in] r:  The radius of the arc
 * @param[in] stangle: Start angle of the arc
 * @param[in] endangle: End angle of the arc
 * @param[in] color:  Display color.
 *****************************************************************************************
 */
void  gui_arc(uint16_t x, uint16_t y, uint16_t r, uint16_t stangle, uint16_t endangle, T_COLOR color);

#endif
