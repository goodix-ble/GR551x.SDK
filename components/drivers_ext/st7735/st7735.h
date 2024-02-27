#ifndef __ST7735X_H__
#define __ST7735X_H__


#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup ST7735_MAROC Defines
 * @{
 */
#define  LCD_XMAX        128           /**< LCD Max Pixel of X coordinate. */
#define  LCD_YMAX        128           /**< LCD Max Pixel of Y coordinate. */

/**
 * @defgroup ST7735_EXTRERN Extern_Variable 
 * @{
 */
extern uint16_t g_lcd_gram[LCD_YMAX][LCD_XMAX];   /**< Gram memory define. */
extern uint16_t g_lcd_buffer[LCD_YMAX][LCD_XMAX]; /**< Buffer memory define. */
/** @} */

/**
 * @defgroup UC1701_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize lcd.
 *****************************************************************************************
 */
void lcd_init(void);

/**
 *****************************************************************************************
 * @brief Resume lcd.
 *****************************************************************************************
 */
void lcd_resume(void);

/**
 *****************************************************************************************
 * @brief Set the memory type of lcd.
 *
 * @param[in] type:  Memory type.
 *****************************************************************************************
 */
void lcd_set_memory(bool gram_set);

/**
 *****************************************************************************************
 * @brief Fill Data to Lcm memory.
 *
 * @param[in] data:  Fill data.
 *****************************************************************************************
 */
void lcd_fill_mem(uint16_t color);

/**
 *****************************************************************************************
 * @brief Refresh the memory data to lcd.
 *****************************************************************************************
 */
void lcd_refresh(void);

/**
 *****************************************************************************************
 * @brief Draw a point to lcd memory.
 *
 * @param[in] x: X coordinate.
 * @param[in] y: Y coordinate.
 * @param[in] color: The color of the memory.
 *****************************************************************************************
 */
void lcd_draw_point(uint8_t x, uint8_t y, uint16_t color);

/**
 *****************************************************************************************
 * @brief Read a point from lcd memory.
 *
 * @param[in] x: X coordinate.
 * @param[in] y: Y coordinate.
 *
 * @return The color of the read point.(Return 0xff:error)
 *****************************************************************************************
 */
uint16_t lcd_read_point(uint8_t x, uint8_t y);

/**
 *****************************************************************************************
 * @brief Fill Data to Lcm rectangle memory.
 *
 * @param[in] x0: X0 coordinate.
 * @param[in] y0: Y0 coordinate.
 * @param[in] x1: X1 coordinate.
 * @param[in] y1: Y1 coordinate.
 * @param[in] color: The color of the memory.
 *****************************************************************************************
 */
void lcd_rectangle_fill_mem(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);

/**
 *****************************************************************************************
 * @brief Refresh the rectangle memory data to lcd.
 *
 * @param[in] x0: X0 coordinate.
 * @param[in] y0: Y0 coordinate.
 * @param[in] x1: X1 coordinate.
 * @param[in] y1: Y1 coordinate.
 *****************************************************************************************
 */
void lcd_rectangle_refresh(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);


#endif


