#ifndef __ST7735X_CONFIG_H__
#define __ST7735X_CONFIG_H__
#include "board_SK.h"
#include "app_io.h"

#define CS_HIGH    app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_SPIM_CS0_PIN, APP_IO_PIN_SET)        /**< set cs pin to high. */
#define CS_LOW     app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_SPIM_CS0_PIN, APP_IO_PIN_RESET)      /**< set cs pin to low. */
#define SEND_CMD   app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_CMD_AND_DATA_PIN, APP_IO_PIN_RESET)  /**< set cmd pin to high. */
#define SEND_DATA  app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_CMD_AND_DATA_PIN, APP_IO_PIN_SET)    /**< set cmd pin to low. */

#ifdef DISPLAY_DRIVER_TYPE_SW_IO

#define SCK_HIGH   app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_SPIM_CLK_PIN, APP_IO_PIN_SET)        /**< set sck pin to high. */
#define SCK_LOW    app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_SPIM_CLK_PIN, APP_IO_PIN_RESET)      /**< set sck pin to low. */
#define SDA_HIGH   app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_SPIM_MOSI_PIN, APP_IO_PIN_SET)       /**< set sda pin to high. */
#define SDA_LOW    app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_SPIM_MOSI_PIN, APP_IO_PIN_RESET)     /**< set sda pin to low. */

#endif

/**
 * @defgroup st7735_CONFIG_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief st7735 init(config gpio spi).
 *****************************************************************************************
 */
void st7735_init(void);

/**
 *****************************************************************************************
 * @brief Write cmd to st7735.
 *
 * @param[in] cmd:  Cmd to write.
 *****************************************************************************************
 */
void st7735_write_cmd(uint8_t cmd);

/**
 *****************************************************************************************
 * @brief Write one data to st7735.
 *
 * @param[in] data:  Data to write.
 *****************************************************************************************
 */
void st7735_write_data(uint8_t data);

/**
 *****************************************************************************************
 * @brief Write data buffer to st7735.
 *
 * @param[in] p_data: The pointer of the data.
 * @param[in] length: The length of write data.
 *****************************************************************************************
 */
void st7735_write_buffer(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief st7735 delay function. 
 *
 * @param[in] time:  Delay time.
 *****************************************************************************************
 */
void st7735_delay(uint16_t time);



#endif

