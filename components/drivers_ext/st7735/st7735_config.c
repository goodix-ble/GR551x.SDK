/**
 ****************************************************************************************
 *
 * @file uc1701_config.c
 *
 * @brief uc1701 config Implementation.
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


/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "st7735_config.h"
#include "grx_hal.h"
#include "app_spi_dma.h"
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#ifdef DISPLAY_DRIVER_TYPE_HW_SPI
#include "app_spi.h"
static volatile uint8_t  master_tx_done = 0;
static void app_spi_callback(app_spi_evt_t *p_evt);
static app_spi_params_t spi_params;
#endif
app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void st7735_init(void)
{
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = DISPLAY_BACK_LIGHT_PIN | DISPLAY_CMD_AND_DATA_PIN;
    io_init.mux  = APP_IO_MUX_7;
    app_io_init(APP_IO_TYPE_GPIOA, &io_init);
    app_io_write_pin(DISPLAY_SPIM_GPIO_TYPE, DISPLAY_BACK_LIGHT_PIN, APP_IO_PIN_SET);
    
#ifdef DISPLAY_DRIVER_TYPE_SW_IO
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = DISPLAY_SPIM_CS0_PIN | DISPLAY_SPIM_CLK_PIN | DISPLAY_SPIM_MOSI_PIN;
    io_init.mux  = APP_IO_MUX_7;
    app_io_init(DISPLAY_SPIM_GPIO_TYPE, &io_init);
#else
    spi_params.id = APP_SPI_ID_MASTER;

    spi_params.pin_cfg.cs.type   = DISPLAY_SPIM_GPIO_TYPE;
    spi_params.pin_cfg.cs.pin    = DISPLAY_SPIM_CS0_PIN;
    spi_params.pin_cfg.cs.mux    = APP_IO_MUX_7;
    spi_params.pin_cfg.cs.mode   = APP_IO_MODE_MUX;
    spi_params.pin_cfg.cs.pull   = APP_IO_PULLUP;
    spi_params.pin_cfg.cs.enable = APP_SPI_PIN_ENABLE;

    spi_params.pin_cfg.clk.type   = DISPLAY_SPIM_GPIO_TYPE;
    spi_params.pin_cfg.clk.pin    = DISPLAY_SPIM_CLK_PIN;
    spi_params.pin_cfg.clk.mux    = APP_IO_MUX_4;
    spi_params.pin_cfg.clk.mode   = APP_IO_MODE_MUX;
    spi_params.pin_cfg.clk.pull   = APP_IO_PULLUP;
    spi_params.pin_cfg.clk.enable = APP_SPI_PIN_ENABLE;

    spi_params.pin_cfg.mosi.type   = DISPLAY_SPIM_GPIO_TYPE;
    spi_params.pin_cfg.mosi.pin    = DISPLAY_SPIM_MOSI_PIN;
    spi_params.pin_cfg.mosi.mux    = APP_IO_MUX_4;
    spi_params.pin_cfg.mosi.mode   = APP_IO_MODE_MUX;
    spi_params.pin_cfg.mosi.pull   = APP_IO_PULLUP;
    spi_params.pin_cfg.mosi.enable = APP_SPI_PIN_ENABLE;

    spi_params.pin_cfg.miso.enable = APP_SPI_PIN_DISABLE;

    spi_params.dma_cfg.rx_dma_instance = DMA0;
    spi_params.dma_cfg.tx_dma_instance = DMA0;
    spi_params.dma_cfg.rx_dma_channel  = DMA_Channel1;
    spi_params.dma_cfg.tx_dma_channel  = DMA_Channel0;

    spi_params.init.data_size          = SPI_DATASIZE_8BIT;
    spi_params.init.clock_polarity     = SPI_POLARITY_LOW;
    spi_params.init.clock_phase        = SPI_PHASE_1EDGE;
    spi_params.init.baudrate_prescaler = SystemCoreClock / 4000000;
    spi_params.init.ti_mode            = SPI_TIMODE_DISABLE;
    spi_params.init.slave_select       = SPI_SLAVE_SELECT_0;

    spi_params.is_soft_cs = 1u;

    app_spi_init(&spi_params, app_spi_callback);
    app_spi_dma_init(&spi_params);
#endif
}

#ifdef DISPLAY_DRIVER_TYPE_SW_IO
/*--------------------------------DISPLAY_DRIVER_TYPE_SW_IO------------------------------------*/
void st7735_write_cmd(uint8_t cmd)
{
    uint8_t i;

    SEND_CMD;
    CS_LOW;
    for(i=0;i<8;i++)
    {
        if (cmd &0x80)
           SDA_HIGH;
        else
           SDA_LOW;

        SCK_LOW;
        SCK_HIGH;
        cmd <<= 1;
    }
    CS_HIGH;
}

void st7735_write_data(uint8_t data)
{
    uint8_t i;

    SEND_DATA;
    CS_LOW;
    for(i=0;i<8;i++)
    {
    if(data&0x80)
        SDA_HIGH;
    else
        SDA_LOW;
    SCK_LOW;
    SCK_HIGH;
    data <<= 1;
    }
    CS_HIGH;
}

void st7735_write_buffer(uint8_t *p_data, uint16_t length)
{
    uint16_t i= 0;
    SEND_DATA;
    CS_LOW;
    for (i=0; i<length; i ++)
    {
        st7735_write_data(p_data[i]);
    }
    CS_HIGH;
}

#else
/*--------------------------------DISPLAY_DRIVER_TYPE_HW_SPI------------------------------------*/

static void app_spi_callback(app_spi_evt_t *p_evt)
{
    if (p_evt->type == APP_SPI_EVT_TX_CPLT)
    {
        master_tx_done = 1;
    }
}

void st7735_write_cmd(uint8_t cmd)
{
    SEND_CMD;
    master_tx_done = 0;
    app_spi_transmit_async(APP_SPI_ID_MASTER, &cmd, 1);
    while(master_tx_done == 0);
}

void st7735_write_data(uint8_t data)
{
    SEND_DATA;
    master_tx_done = 0;
    app_spi_transmit_async(APP_SPI_ID_MASTER, &data, 1);
    while(master_tx_done == 0);
}

void st7735_write_buffer(uint8_t *p_data, uint16_t length)
{
    uint16_t last_size;
    uint16_t count_size;
    uint16_t i;
    SEND_DATA;
    if(length <= 4095)
    {
        master_tx_done = 0;
        app_spi_transmit_async(APP_SPI_ID_MASTER, p_data, length);
        while(master_tx_done == 0);
    }
    else
    {
        last_size = length % 4095;
        count_size = length / 4095;
        for(i = 0; i < count_size; i++)
        {
            master_tx_done = 0;
            app_spi_transmit_async(APP_SPI_ID_MASTER, p_data + i * 4095, 4095);
            while(master_tx_done == 0);
        }
        if(last_size)
        {
            master_tx_done = 0;
            app_spi_transmit_async(APP_SPI_ID_MASTER, p_data + i * 4095, last_size);
            while(master_tx_done == 0);
        }
    }
}

#endif

void st7735_delay(uint16_t time)
{
    delay_ms(time);
}

