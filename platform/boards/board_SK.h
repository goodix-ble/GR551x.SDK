/**
 ****************************************************************************************
 *
 * @file board_SK.h
 *
 * @brief Board Start Kit Macro.
 *
 ****************************************************************************************
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
#ifndef __BOARD_SK_H__
#define __BOARD_SK_H__

#include "app_key.h"
#include "app_uart.h"

/*******HCI UART IO CONFIG***********************/
#define APP_HCI_UART_ID                 APP_UART_ID_0
#define APP_HCI_UART_FLOW_ON            0
#define APP_HCI_UART_BAUDRATE           115200
#define APP_HCI_UART_TRN_PORT           APP_IO_TYPE_NORMAL
#define APP_HCI_UART_FLOW_PORT          APP_IO_TYPE_NORMAL
#define APP_HCI_UART_TX_PIN             APP_IO_PIN_10
#define APP_HCI_UART_RX_PIN             APP_IO_PIN_11
#define APP_HCI_UART_CTS_PIN            APP_IO_PIN_2
#define APP_HCI_UART_RTS_PIN            APP_IO_PIN_5
#define APP_HCI_UART_TX_PINMUX          APP_IO_MUX_2
#define APP_HCI_UART_RX_PINMUX          APP_IO_MUX_2
#define APP_HCI_UART_CTS_PINMUX         APP_IO_MUX_0
#define APP_HCI_UART_RTS_PINMUX         APP_IO_MUX_0

/*******UART DRIVER IO CONFIG*******************/
#define APP_UART_ID                     APP_UART_ID_0
#define APP_UART_BAUDRATE               115200
#define APP_UART_TX_IO_TYPE             APP_IO_TYPE_NORMAL
#define APP_UART_RX_IO_TYPE             APP_IO_TYPE_NORMAL
#define APP_UART_TX_PIN                 APP_IO_PIN_10
#define APP_UART_RX_PIN                 APP_IO_PIN_11
#define APP_UART_TX_PINMUX              APP_IO_MUX_2
#define APP_UART_RX_PINMUX              APP_IO_MUX_2
#define APP_UART_TX_PULL                APP_IO_PULLUP
#define APP_UART_RX_PULL                APP_IO_PULLUP

#define APP_UART1_ID                    APP_UART_ID_1
#define APP_UART1_BAUDRATE              115200
#define APP_UART1_TX_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_UART1_RX_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_UART1_TX_PIN                APP_IO_PIN_30
#define APP_UART1_RX_PIN                APP_IO_PIN_26
#define APP_UART1_TX_PINMUX             APP_IO_MUX_1
#define APP_UART1_RX_PINMUX             APP_IO_MUX_1

/*******KEY DRIVER IO CONFIG********************/
#define APP_KEY_OK_IO_TYPE              APP_IO_TYPE_AON
#define APP_KEY_UP_IO_TYPE              APP_IO_TYPE_NORMAL
#define APP_KEY_DOWN_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_KEY_LEFT_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_KEY_RIGHT_IO_TYPE           APP_IO_TYPE_NORMAL
#define APP_KEY_OK_PIN                  APP_IO_PIN_1
#define APP_KEY_UP_PIN                  APP_IO_PIN_12
#define APP_KEY_DOWN_PIN                APP_IO_PIN_13
#define APP_KEY_LEFT_PIN                APP_IO_PIN_14
#define APP_KEY_RIGHT_PIN               APP_IO_PIN_15
#define APP_KEY_OK_MUX                  APP_IO_MUX_7
#define APP_KEY_UP_MUX                  APP_IO_MUX_7
#define APP_KEY_DOWN_MUX                APP_IO_MUX_7
#define APP_KEY_LEFT_MUX                APP_IO_MUX_7
#define APP_KEY_RIGHT_MUX               APP_IO_MUX_7

/*******KEY TRIGGER & PULL MODE CONFIG*******************/
#define APP_KEY_TRIGGER_MODE            APP_IO_MODE_IT_FALLING
#define APP_KEY_PULL_MODE               APP_IO_PULLUP

/*******LED IO CONFIG FOR SK*********************/
#define APP_LED_NUM_0_IO                APP_IO_PIN_4
#define APP_LED_NUM_1_IO                MSIO_PIN_4

/*******ADC IO CONFIG***************************/
#define APP_ADC_P_INPUT_PIN             MSIO_PIN_0
#define APP_ADC_P_INPUT_PIN_MUX         APP_IO_MUX_7
#define APP_ADC_N_INPUT_PIN             MSIO_PIN_1
#define APP_ADC_N_INPUT_PIN_MUX         APP_IO_MUX_7

/*******COMP IO CONFIG***************************/
#define APP_COMP_INPUT_PIN              APP_IO_PIN_0
#define APP_COMP_INPUT_PIN_MUX          APP_IO_MUX_7
#define APP_COMP_VREF_PIN               APP_IO_PIN_1
#define APP_COMP_VREF_PIN_MUX           APP_IO_MUX_7

/*******I2C IO CONFIG***************************/
#define APP_I2C_MASTER_ID               APP_I2C_ID_1
#define APP_I2C_MASTER_SCL_PIN          APP_IO_PIN_30
#define APP_I2C_MASTER_SDA_PIN          APP_IO_PIN_26
#define APP_I2C_MASTER_SCL_IO_TYPE      APP_IO_TYPE_NORMAL
#define APP_I2C_MASTER_SDA_IO_TYPE      APP_IO_TYPE_NORMAL
#define APP_I2C_MASTER_SCL_PINMUX       APP_IO_MUX_0
#define APP_I2C_MASTER_SDA_PINMUX       APP_IO_MUX_0

#define APP_I2C_SLAVE_ID                APP_I2C_ID_1
#define APP_I2C_SLAVE_SCL_PIN           APP_IO_PIN_30
#define APP_I2C_SLAVE_SDA_PIN           APP_IO_PIN_26
#define APP_I2C_SLAVE_SCL_IO_TYPE       APP_IO_TYPE_NORMAL
#define APP_I2C_SLAVE_SDA_IO_TYPE       APP_IO_TYPE_NORMAL
#define APP_I2C_SLAVE_SCL_PINMUX        APP_IO_MUX_0
#define APP_I2C_SLAVE_SDA_PINMUX        APP_IO_MUX_0

/*******SPI IO CONFIG***************************/
#define APP_SPIM_CS_PIN                 APP_IO_PIN_17
#define APP_SPIM_CLK_PIN                APP_IO_PIN_24
#define APP_SPIM_MOSI_PIN               APP_IO_PIN_25
#define APP_SPIM_MISO_PIN               APP_IO_PIN_16
#define APP_SPIM_CS_IO_TYPE             APP_IO_TYPE_NORMAL
#define APP_SPIM_CLK_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_SPIM_MOSI_IO_TYPE           APP_IO_TYPE_NORMAL
#define APP_SPIM_MISO_IO_TYPE           APP_IO_TYPE_NORMAL
#define APP_SPIM_CS_PINMUX              APP_IO_MUX_0
#define APP_SPIM_CLK_PINMUX             APP_IO_MUX_0
#define APP_SPIM_MOSI_PINMUX            APP_IO_MUX_0
#define APP_SPIM_MISO_PINMUX            APP_IO_MUX_0

/*******QSPI IO CONFIG***************************/
#define APP_QSPI1_CS_PIN                 APP_IO_PIN_15
#define APP_QSPI1_CLK_PIN                APP_IO_PIN_9
#define APP_QSPI1_IO0_PIN                APP_IO_PIN_8
#define APP_QSPI1_IO1_PIN                APP_IO_PIN_14
#define APP_QSPI1_IO2_PIN                APP_IO_PIN_13
#define APP_QSPI1_IO3_PIN                APP_IO_PIN_12
#define APP_QSPI1_CS_IO_TYPE             APP_IO_TYPE_NORMAL
#define APP_QSPI1_CLK_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_QSPI1_IO0_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_QSPI1_IO1_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_QSPI1_IO2_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_QSPI1_IO3_IO_TYPE            APP_IO_TYPE_NORMAL
#define APP_QSPI1_GPIO_MUX               APP_IO_MUX_2

#define APP_FLASH_GPIO_MUX               APP_QSPI1_GPIO_MUX
#define APP_FLASH_CS_IO_TYPE             APP_QSPI1_CS_IO_TYPE
#define APP_FLASH_CLK_IO_TYPE            APP_QSPI1_CLK_IO_TYPE
#define APP_FLASH_IO0_IO_TYPE            APP_QSPI1_IO0_IO_TYPE
#define APP_FLASH_IO1_IO_TYPE            APP_QSPI1_IO1_IO_TYPE
#define APP_FLASH_IO2_IO_TYPE            APP_QSPI1_IO2_IO_TYPE
#define APP_FLASH_IO3_IO_TYPE            APP_QSPI1_IO3_IO_TYPE
#define APP_FLASH_CS_PIN                 APP_QSPI1_CS_PIN
#define APP_FLASH_CLK_PIN                APP_QSPI1_CLK_PIN
#define APP_FLASH_IO0_PIN                APP_QSPI1_IO0_PIN
#define APP_FLASH_IO1_PIN                APP_QSPI1_IO1_PIN
#define APP_FLASH_IO2_PIN                APP_QSPI1_IO2_PIN
#define APP_FLASH_IO3_PIN                APP_QSPI1_IO3_PIN
#define APP_FLASH_QSPI_ID                APP_QSPI_ID_1

/*******I2S IO CONFIG***************************/
#define APP_I2S_MASTER_WS_PIN           APP_IO_PIN_24
#define APP_I2S_MASTER_SDO_PIN          APP_IO_PIN_25
#define APP_I2S_MASTER_SDI_PIN          APP_IO_PIN_16
#define APP_I2S_MASTER_SCLK_PIN         APP_IO_PIN_17
#define APP_I2S_MASTER_WS_IO_TYPE       APP_IO_TYPE_NORMAL
#define APP_I2S_MASTER_SDO_IO_TYPE      APP_IO_TYPE_NORMAL
#define APP_I2S_MASTER_SDI_IO_TYPE      APP_IO_TYPE_NORMAL
#define APP_I2S_MASTER_SCLK_IO_TYPE     APP_IO_TYPE_NORMAL
#define APP_I2S_MASTER_WS_PINMUX        APP_IO_MUX_3
#define APP_I2S_MASTER_SDO_PINMUX       APP_IO_MUX_3
#define APP_I2S_MASTER_SDI_PINMUX       APP_IO_MUX_3
#define APP_I2S_MASTER_SCLK_PINMUX      APP_IO_MUX_3

#define APP_I2S_SLAVE_WS_PIN            APP_IO_PIN_2
#define APP_I2S_SLAVE_SDO_PIN           APP_IO_PIN_3
#define APP_I2S_SLAVE_SDI_PIN           APP_IO_PIN_4
#define APP_I2S_SLAVE_SCLK_PIN          APP_IO_PIN_5
#define APP_I2S_SLAVE_WS_IO_TYPE        APP_IO_TYPE_AON
#define APP_I2S_SLAVE_SDO_IO_TYPE       APP_IO_TYPE_AON
#define APP_I2S_SLAVE_SDI_IO_TYPE       APP_IO_TYPE_AON
#define APP_I2S_SLAVE_SCLK_IO_TYPE      APP_IO_TYPE_AON
#define APP_I2S_SLAVE_WS_PINMUX         APP_IO_MUX_3
#define APP_I2S_SLAVE_SDO_PINMUX        APP_IO_MUX_3
#define APP_I2S_SLAVE_SDI_PINMUX        APP_IO_MUX_3
#define APP_I2S_SLAVE_SCLK_PINMUX       APP_IO_MUX_3

/*******UC1701 DRIVER IO CONFIG*****************/
#define DISPLAY_DRIVER_TYPE_HW_SPI
#define DISPLAY_SPIM_CS0_PIN            APP_IO_PIN_3
#define DISPLAY_CMD_AND_DATA_PIN        APP_IO_PIN_5
#define DISPLAY_SPIM_CLK_PIN            APP_IO_PIN_7
#define DISPLAY_SPIM_MOSI_PIN           APP_IO_PIN_6
#define DISPLAY_BACK_LIGHT_PIN          APP_IO_PIN_2
#define DISPLAY_SPIM_GPIO_TYPE          APP_IO_TYPE_NORMAL

/*******PWM IO CONFIG***************************/
#define APP_PWM0_MODULE                 APP_PWM_ID_0
#define APP_PWM0_GPIO_MUX               APP_IO_MUX_5
#define APP_PWM0_CHANNEL_A              APP_IO_PIN_2
#define APP_PWM0_CHANNEL_B              APP_IO_PIN_3
#define APP_PWM0_CHANNEL_C              APP_IO_PIN_4
#define APP_PWM0_GPIO_TYPE              APP_IO_TYPE_NORMAL

#define APP_PWM1_MODULE                 APP_PWM_ID_1
#define APP_PWM1_GPIO_MUX               APP_IO_MUX_0
#define APP_PWM1_CHANNEL_A              APP_IO_PIN_3
#define APP_PWM1_CHANNEL_B              APP_IO_PIN_4
#define APP_PWM1_GPIO_TYPE              APP_IO_TYPE_MSIO



/**
 * @defgroup BSP_MAROC Defines
 * @{
 */
#if APP_DRIVER_USE_ENABLE
    #define BSP_KEY_UP_ID      0x00      /**< ID for UP KEY. */
    #define BSP_KEY_DOWN_ID    0x01      /**< ID for DOWN KEY. */
    #define BSP_KEY_LEFT_ID    0x02      /**< ID for LEFT KEY. */
    #define BSP_KEY_RIGHT_ID   0x03      /**< ID for RIGHT KEY. */
    #define BSP_KEY_OK_ID      0x04      /**< ID for OK KEY. */

    #define UART_TX_BUFF_SIZE  0x400    /**< Size of app uart tx buffer. */
#endif
/** @} */

/**
 * @defgroup BSP_ENUM Enumerations
 * @{
 */
typedef enum
{
    BSP_LED_NUM_0,
    BSP_LED_NUM_1,
} bsp_led_num_t;
/** @} */

/**
 * @defgroup BSP_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize boards key.
 *****************************************************************************************
 */
void bsp_key_init(void);

/**
 *****************************************************************************************
 * @brief App key event handler
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type);

/**
 *****************************************************************************************
 * @brief Initialize app uart.
 *****************************************************************************************
 */
void bsp_uart_init(void);

/**
 *****************************************************************************************
 * @brief Uart data send.
 *****************************************************************************************
 */
void bsp_uart_send(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Uart data flush.
 *****************************************************************************************
 */
void bsp_uart_flush(void);

/**
 *****************************************************************************************
 * @brief App uart event handler.
 *****************************************************************************************
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt);

/**
 *****************************************************************************************
 * @brief Initialize boards led.
 *****************************************************************************************
 */
void bsp_led_init(void);

/**
 *****************************************************************************************
 * @brief Open boards led.
 *
 * @param[in] led_num: Number of led needed open.
 *****************************************************************************************
 */
void bsp_led_open(bsp_led_num_t led_num);

/**
 *****************************************************************************************
 * @brief Close boards led.
 *
 * @param[in] led_num: Number of led needed close.
 *****************************************************************************************
 */
void bsp_led_close(bsp_led_num_t led_num);

/**
 *****************************************************************************************
 * @brief BSP log init.
 *****************************************************************************************
 */
void bsp_log_init(void);

/**
 *****************************************************************************************
 * @brief Board init.
 *****************************************************************************************
 */
void board_init(void);

 
#endif  /* __BOARD_SK_H__ */

