/**
 *****************************************************************************************
 *
 * @file gui_config.c
 *
 * @brief Gui config
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

#ifndef  GUI_LCM_DRV_H
#define  GUI_LCM_DRV_H  
#include <stdint.h>
#include <stdbool.h>

/**@brief GUI module enable define. */
#define  FONT8x8_EN          0
#define  FONT5x7_EN          1
#define  FONT_OTHER_EN       1
#define  FONT_GB2312_EN      0
#define  CONVERT_COLOR_EN    0
#define  ANIMATION_EN        1

/**@brief DISPLAY param config. */
#define  T_COLOR             uint16_t  
#define  GUI_DISPLAY_X_MAX   128    
#define  GUI_DISPLAY_Y_MAX   128     

/**** display driver config ****/

/**
 *****************************************************************************************
 * @brief gui display init.
 *****************************************************************************************
 */
void  gui_init(void);

/**
 *****************************************************************************************
 * @brief Fill Data to gui display memory.
 *
 * @param[in] color:  Fill color.
 *****************************************************************************************
 */
void  gui_fill_mem(T_COLOR color);

/**
 *****************************************************************************************
 * @brief Fill Data to gui display rectangle memory.
 *
 * @param[in] x0: X0 coordinate.
 * @param[in] y0: Y0 coordinate.
 * @param[in] x1: X1 coordinate.
 * @param[in] y1: Y1 coordinate.
 * @param[in] color:  Fill color.
 *****************************************************************************************
 */
void gui_rectangle_fill_mem(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Draw a point to display memory.
 *
 * @param[in] x: X coordinate.
 * @param[in] y: Y coordinate.
 * @param[in] color: The color of the point.
 *****************************************************************************************
 */
void  gui_point(uint16_t x, uint16_t y, T_COLOR color);

/**
 *****************************************************************************************
 * @brief Read a point from display memory.
 *
 * @param[in] x: X coordinate.
 * @param[in] y: Y coordinate.
 *
 * @return The color of the read point.(Return 0xff:error)
 *****************************************************************************************
 */
T_COLOR gui_read_point(uint16_t x, uint16_t y);

/**
 *****************************************************************************************
 * @brief Set refresh flag to true.
 *****************************************************************************************
 */
void  gui_refresh(void);

/**
 *****************************************************************************************
 * @brief Check if the refresh flag is true, if flag is true, refresh the gram memory data to display.
 * 
 * @note  This function should be called in main while or in timer handler
 *****************************************************************************************
 */
void gui_refresh_schedule(void);

/**
 *****************************************************************************************
 * @brief Refresh the rectangle gram memory data to display.
 * 
 * @param[in] x0: X0 coordinate.
 * @param[in] y0: Y0 coordinate.
 * @param[in] x1: X1 coordinate.
 * @param[in] y1: Y1 coordinate.
 *****************************************************************************************
 */
void gui_rectangle_refresh(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 *****************************************************************************************
 * @brief If refresh gram memory to the display area
 *
 * @param[in] gram_enable: true -- gram memory  false -- buffer memory
 *****************************************************************************************
 */
void gui_set_refresh_mem(bool gram_enable);
/*------------------------*/

/**** animation config ****/
#if ANIMATION_EN==1

/**@brief Animation param config. */
#define GUI_X_MOVE_PIXEL   32
#define GUI_Y_MOVE_PIXEL   32
#define GUI_X_MOVE_FRAME   (GUI_DISPLAY_X_MAX / GUI_X_MOVE_PIXEL)
#define GUI_Y_MOVE_FRAME   (GUI_DISPLAY_Y_MAX / GUI_Y_MOVE_PIXEL)

/**
 *****************************************************************************************
 * @brief start animation timer
 *****************************************************************************************
 */
void gui_animation_timer_start(void);

/**
 *****************************************************************************************
 * @brief stop animation timer
 *****************************************************************************************
 */
void gui_animation_timer_stop(void);

#endif
/*------------------------*/

/**** Font GB2312 config ****/
#if FONT_GB2312_EN==1

/**
 *****************************************************************************************
 * @brief Reads gb2312 font data based on offset
 *
 * @param[in]  offset: font offset
 * @param[out] read_buf: Read datas
 *****************************************************************************************
 */
void gui_read_gb2312(uint32_t offset, uint8_t* read_buf);

/**
 *****************************************************************************************
 * @brief Reads ascii font data based on offset
 *
 * @param[in]  offset: font offset
 * @param[out] read_buf: Read datas
 *****************************************************************************************
 */
void gui_read_ascii(uint32_t offset, uint8_t* read_buf);
#endif

#endif
