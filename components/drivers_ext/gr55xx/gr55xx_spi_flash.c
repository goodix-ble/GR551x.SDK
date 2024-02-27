/**
  ****************************************************************************************
  * @file    gr55xx_spi_flash.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
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
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <string.h>
#include "grx_hal.h"
#include "gr55xx_spi_flash.h"


#define SSI_HIGH_FREQ_CLOCK_PRESCALER   4u

#define SSI_LOW_FREQ_CLOCK_PRESCALER    8u

/*
 * SPI Master DEFINES
 *****************************************************************************************
 */
#define SPI_CLOCK_PRESCALER             8u              /* The SPI CLOCK Freq = Peripheral CLK/SPI_CLOCK_PRESCALER */
#define SPI_SOFT_CS_MODE_ENABLE         1u              /* suggest to enable SOFT CS MODE */
#define SPI_WAIT_TIMEOUT_MS             1500u           /* default time(ms) for wait operation */

#if SPI_CLOCK_PRESCALER == 2u
    #define SPI_RX_SAMPLE_DELAY         1u
#else
    #define SPI_RX_SAMPLE_DELAY         0u
#endif

/* master spi parameters */
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X) //pin configuration should be specified in the input of spi_flash_init
#define MASTER_SPI_PIN_CONFIG           {{APP_IO_TYPE_GPIOB, APP_IO_MUX_3, APP_IO_PIN_9, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                         {APP_IO_TYPE_GPIOB, APP_IO_MUX_3, APP_IO_PIN_6, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                         {APP_IO_TYPE_GPIOB, APP_IO_MUX_3, APP_IO_PIN_7, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                         {APP_IO_TYPE_GPIOB, APP_IO_MUX_3, APP_IO_PIN_8, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE}}
#else
#define MASTER_SPI_PIN_CONFIG           {{APP_IO_TYPE_GPIOA, APP_IO_MUX_1, APP_IO_PIN_15, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                         {APP_IO_TYPE_GPIOA, APP_IO_MUX_1, APP_IO_PIN_12, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                         {APP_IO_TYPE_GPIOA, APP_IO_MUX_1, APP_IO_PIN_13, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                         {APP_IO_TYPE_GPIOA, APP_IO_MUX_1, APP_IO_PIN_14, APP_IO_MODE_MUX, APP_IO_PULLUP, APP_SPI_PIN_ENABLE}}
#endif
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
#define MASTER_SPI_MODE_CONFIG           {DMA0, DMA0, DMA_Channel0, DMA_Channel1, SPI_WAIT_TIMEOUT_MS, 0}
#define MASTER_SPI_CONFIG                {SPI_DATASIZE_8BIT,   SPI_POLARITY_LOW,   SPI_PHASE_1EDGE, \
                                          SPI_CLOCK_PRESCALER, SPI_TIMODE_DISABLE, SPI_SLAVE_SELECT_0, SPI_RX_SAMPLE_DELAY}
#else
#define MASTER_SPI_MODE_CONFIG           {DMA, DMA, DMA_Channel0, DMA_Channel1}
#define MASTER_SPI_CONFIG                {SPI_DATASIZE_8BIT,   SPI_POLARITY_LOW,   SPI_PHASE_1EDGE, \
                                          SPI_CLOCK_PRESCALER, SPI_TIMODE_DISABLE, SPI_SLAVE_SELECT_0}
#endif

#define MASTER_SPI_PARAM_CONFIG          {APP_SPI_ID_MASTER, MASTER_SPI_PIN_CONFIG, MASTER_SPI_MODE_CONFIG, MASTER_SPI_CONFIG, SPI_SOFT_CS_MODE_ENABLE}

/*
 * QSPI DEFINES
 *****************************************************************************************
 */
/*****************************************
 * CHANGE FOLLOWING SETTINGS By YOUR CASE !
 *****************************************/

#define QSPI_CLOCK_PRESCALER             8u                     /* The QSPI CLOCK Freq = Peripheral CLK/QSPI_CLOCK_PRESCALER */
#define QSPI_WAIT_TIMEOUT_MS             1500u                  /* default time(ms) for     wait operation */
#define QSPI_ID                          APP_QSPI_ID_0
#define QSPI_TIMING_MODE                 QSPI_CLOCK_MODE_0

/*****************************************
 * CHANGE FOLLOWING SETTINGS CAREFULLY !
 *****************************************/
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
#if QSPI_ID == APP_QSPI_ID_0
    #define QSPI_USED_DMA                DMA0
    #define QSPI_USED_DMA_CH             DMA_Channel0
#elif QSPI_ID == APP_QSPI_ID_1
    #define QSPI_USED_DMA                DMA0
    #define QSPI_USED_DMA_CH             DMA_Channel1
#else
    #define QSPI_USED_DMA                DMA1
    #define QSPI_USED_DMA_CH             DMA_Channel0
#endif
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    #define QSPI_USED_DMA                DMA
    #define QSPI_USED_DMA_CH             DMA_Channel7
#endif


#if QSPI_CLOCK_PRESCALER == 2u
    #define QSPI_RX_SAMPLE_DELAY         1u
#else
    #define QSPI_RX_SAMPLE_DELAY         0u
#endif
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static volatile qspi_control_t g_qspi_ctl;
static flash_init_t            g_flash_init;
static app_spi_params_t  spim_params = MASTER_SPI_PARAM_CONFIG;
static app_qspi_params_t qspi_params;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void spi_app_spim_callback(app_spi_evt_t *p_evt)
{
    if (p_evt->type == APP_SPI_EVT_TX_CPLT)
    {
        g_qspi_ctl.spi_tmt_done = 1;
    }
    if (p_evt->type == APP_SPI_EVT_RX_CPLT)
    {
        g_qspi_ctl.spi_rcv_done = 1;
    }
    if (p_evt->type == APP_SPI_EVT_TX_RX_CPLT)
    {
        g_qspi_ctl.spi_tx_rx_done = 1;
    }
    if (p_evt->type == APP_SPI_EVT_ERROR)
    {
        g_qspi_ctl.spi_tmt_done = 1;
        g_qspi_ctl.spi_rcv_done = 1;
        g_qspi_ctl.spi_tx_rx_done = 1;
        printf("spi error event!!");
    }
}

static void app_qspi_callback(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
        g_qspi_ctl.qspi_tmt_done = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
        g_qspi_ctl.qspi_rcv_done = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_ERROR)
    {
        g_qspi_ctl.qspi_tmt_done = 1;
        g_qspi_ctl.qspi_rcv_done = 1;
        printf("QSPI error event!!");
    }
}

static void spi_flash_write_enable(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_WREN};

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        g_qspi_ctl.spi_tmt_done = 0;
        app_spi_dma_transmit_async(g_qspi_ctl.spi_id, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.spi_tmt_done == 0);
    }
    else
    {
        qspi_command_t command = {
            .instruction                = SPI_FLASH_CMD_WREN,
            .address                    = 0,
            .instruction_size           = QSPI_INSTSIZE_08_BITS,
            .address_size               = QSPI_ADDRSIZE_00_BITS,
            .data_size                  = QSPI_DATASIZE_08_BITS,
            .dummy_cycles               = 0,
            .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
            .data_mode                  = QSPI_DATA_MODE_SPI,
            .length                     = 0,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .clock_stretch_en           = 1,
#endif
        };

        g_qspi_ctl.qspi_tmt_done = 0;
        app_qspi_dma_command_async(g_qspi_ctl.qspi_id, &command);
        while(g_qspi_ctl.qspi_tmt_done == 0);
    }

    return;
}

static uint8_t spi_flash_read_status(void)
{
    uint8_t status = 0;

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        uint8_t cmd = SPI_FLASH_CMD_RDSR;

        g_qspi_ctl.spi_tx_rx_done = 0;
        app_spi_dma_read_eeprom_async(g_qspi_ctl.spi_id, &cmd, &status, 1,  1);
        while(g_qspi_ctl.spi_tx_rx_done == 0);
    }
    else
    {
        qspi_command_t command = {
            .instruction      = SPI_FLASH_CMD_RDSR,
            .address          = 0,
            .instruction_size = QSPI_INSTSIZE_08_BITS,
            .address_size     = QSPI_ADDRSIZE_00_BITS,
            .data_size        = QSPI_DATASIZE_08_BITS,
            .dummy_cycles     = 0,
            .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
            .data_mode        = QSPI_DATA_MODE_SPI,
            .length           = 1,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .clock_stretch_en = 1,
#endif
        };
        g_qspi_ctl.qspi_rcv_done = 0;
        app_qspi_dma_command_receive_async(g_qspi_ctl.qspi_id, &command, &status);
        while(g_qspi_ctl.qspi_rcv_done == 0);
    }

    return status;
}

static uint32_t spi_flash_device_size(void)
{
    uint32_t flash_size = 0;

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        uint8_t data[5] = {0};
        uint8_t control_frame[5] = {SPI_FLASH_CMD_SFUD, 0, 0, 0x34, DUMMY_BYTE};

        g_qspi_ctl.spi_tx_rx_done = 0;
        app_spi_dma_read_eeprom_async(g_qspi_ctl.spi_id, control_frame, data, 5,  5);
        while(g_qspi_ctl.spi_tx_rx_done == 0);

        if (data[0] != 0 && data[3] < 0xFF)
        {
            flash_size = ((data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0) + 1) / 8;
        }
    }
    else
    {
        uint8_t data[4] = {0};
        qspi_command_t command = {
            .instruction      = SPI_FLASH_CMD_SFUD,   // SPI_FLASH_CMD_SFUD  //SPI_FLASH_CMD_RDSR
            .address          = 0x000034,
            .instruction_size = QSPI_INSTSIZE_08_BITS,
            .address_size     = QSPI_ADDRSIZE_24_BITS,
            .data_size        = QSPI_DATASIZE_08_BITS,
            .dummy_cycles     = 8,
            .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
            .data_mode        = QSPI_DATA_MODE_SPI,
            .length           = sizeof(data),
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .clock_stretch_en = 1,
#endif
        };

        g_qspi_ctl.qspi_rcv_done = 0;
        app_qspi_dma_command_receive_async(g_qspi_ctl.qspi_id, &command, &data[0]);
        while(g_qspi_ctl.qspi_rcv_done == 0);

        if (data[0] != 0 && data[3] < 0xFF)
        {
            flash_size = ((data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0) + 1) / 8;
        }
    }

    return flash_size;
}


static uint32_t spim_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    g_qspi_ctl.spi_tmt_done = 0;
    app_spim_dma_transmit_with_ia(g_qspi_ctl.spi_id, SPI_FLASH_CMD_PP, address, buffer, nbytes);
    while(g_qspi_ctl.spi_tmt_done == 0);

    return nbytes;
}

static uint32_t qspi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t ret;

    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_PP,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    g_qspi_ctl.qspi_tmt_done = 0;
    ret = app_qspi_dma_command_transmit_async(g_qspi_ctl.qspi_id, &command, buffer);
    if(ret == APP_DRV_SUCCESS)
    {
        while(g_qspi_ctl.qspi_tmt_done == 0);
        return nbytes;
    }
    else
    {
        return 0;
    }
}

static uint32_t qspi_flash_dual_write(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t ret;

    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DPP,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    g_qspi_ctl.qspi_tmt_done = 0;
    ret = app_qspi_dma_command_transmit_async(g_qspi_ctl.qspi_id, &command, buffer);
    if(ret == APP_DRV_SUCCESS)
    {
        while(g_qspi_ctl.qspi_tmt_done == 0);
        return nbytes;
    }
    else
    {
        return 0;
    }
}

static uint32_t spim_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint8_t control_frame[4] = {0};

    control_frame[0] = SPI_FLASH_CMD_READ;
    control_frame[1] = (address >> 16) & 0xFF;
    control_frame[2] = (address >> 8) & 0xFF;
    control_frame[3] = address & 0xFF;

    g_qspi_ctl.spi_tx_rx_done = 0;
    app_spi_dma_read_eeprom_async(g_qspi_ctl.spi_id, control_frame, buffer, 4,  nbytes);
    while(g_qspi_ctl.spi_tx_rx_done == 0);

    return nbytes;
}

static uint32_t qspi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t ret ;
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_READ,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    g_qspi_ctl.qspi_rcv_done = 0;
    ret = app_qspi_dma_command_receive_async(g_qspi_ctl.qspi_id, &command, &buffer[0]);
    if(ret == APP_DRV_SUCCESS)
    {
        while(g_qspi_ctl.qspi_rcv_done == 0);
        return nbytes;
    }
    else
    {
        return 0;
    }
}

static uint32_t qspi_flash_dual_read(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t ret ;
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DREAD,
        .address          = address,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .dummy_cycles     = 8,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .clock_stretch_en = 1,
#endif
    };

    g_qspi_ctl.qspi_rcv_done = 0;
    ret = app_qspi_dma_command_receive_async(g_qspi_ctl.qspi_id, &command, &buffer[0]);
    if(ret == APP_DRV_SUCCESS)
    {
        while(g_qspi_ctl.qspi_rcv_done == 0);
        return nbytes;
    }
    else
    {
        return 0;
    }
}

bool spim_flash_sector_erase(uint32_t address)
{
    uint8_t control_frame[4] = {0};
    uint32_t ret ;

    control_frame[0] = SPI_FLASH_CMD_SE;
    control_frame[1] = (address >> 16) & 0xFF;
    control_frame[2] = (address >> 8) & 0xFF;
    control_frame[3] = address & 0xFF;

    g_qspi_ctl.spi_tmt_done = 0;
    ret = app_spi_dma_transmit_async(g_qspi_ctl.spi_id, control_frame, sizeof(control_frame));
    while(g_qspi_ctl.spi_tmt_done == 0);

    return (ret == APP_DRV_SUCCESS) ? true : false;
}

bool qspi_flash_sector_erase(uint32_t address)
{
    uint32_t ret;
    uint8_t control_frame[4] = {0};

    control_frame[0] = SPI_FLASH_CMD_SE;
    control_frame[1] = (address >> 16) & 0xFF;
    control_frame[2] = (address >> 8) & 0xFF;
    control_frame[3] = address & 0xFF;

    g_qspi_ctl.qspi_tmt_done = 0;
    ret = app_qspi_dma_transmit_async_ex(g_qspi_ctl.qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
    if(ret == APP_DRV_SUCCESS)
    {
        while(g_qspi_ctl.qspi_tmt_done == 0);
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void spi_flash_init(flash_init_t *p_flash_init)
{
    uint16_t ret;

    memcpy(&g_flash_init, p_flash_init, sizeof(flash_init_t));

    if (FLASH_SPIM_ID == p_flash_init->spi_type)
    {
       if(p_flash_init->is_high_freq) {
            spim_params.init.baudrate_prescaler = SSI_HIGH_FREQ_CLOCK_PRESCALER;
        } else {
            spim_params.init.baudrate_prescaler = SSI_LOW_FREQ_CLOCK_PRESCALER;
        }

        spim_params.pin_cfg.cs.type     = p_flash_init->flash_io.spi_cs.gpio;
        spim_params.pin_cfg.cs.pin      = p_flash_init->flash_io.spi_cs.pin;
        spim_params.pin_cfg.cs.mux      = p_flash_init->flash_io.spi_cs.mux;

        spim_params.pin_cfg.clk.type    = p_flash_init->flash_io.spi_clk.gpio;
        spim_params.pin_cfg.clk.pin     = p_flash_init->flash_io.spi_clk.pin;
        spim_params.pin_cfg.clk.mux     = p_flash_init->flash_io.spi_clk.mux;

        spim_params.pin_cfg.mosi.type   = p_flash_init->flash_io.spi_io0.qspi_io0.gpio;
        spim_params.pin_cfg.mosi.pin    = p_flash_init->flash_io.spi_io0.qspi_io0.pin;
        spim_params.pin_cfg.mosi.mux    = p_flash_init->flash_io.spi_io0.qspi_io0.mux;

        spim_params.pin_cfg.miso.type   = p_flash_init->flash_io.spi_io1.qspi_io1.gpio;
        spim_params.pin_cfg.miso.pin    = p_flash_init->flash_io.spi_io1.qspi_io1.pin;
        spim_params.pin_cfg.miso.mux    = p_flash_init->flash_io.spi_io1.qspi_io1.mux;

        g_qspi_ctl.spi_id = APP_SPI_ID_MASTER;
        spim_params.id = APP_SPI_ID_MASTER;

        app_spi_deinit(g_qspi_ctl.spi_id);
        ret = app_spi_init(&spim_params, spi_app_spim_callback);
        if (ret != 0)
        {
            printf("SPI master initial failed! Please check the input paraments.\r\n");
            return;
        }
        ret = app_spi_dma_init(&spim_params);
        if (ret != 0)
        {
            printf("SPI master dma initial failed! Please check the input paraments.\r\n");
            return;
        }
    }
    else if ((FLASH_QSPI_ID0 == p_flash_init->spi_type)
             ||(FLASH_QSPI_ID1 == p_flash_init->spi_type)
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
             || (FLASH_QSPI_ID2 == p_flash_init->spi_type)
#endif
    )
    {
        if(FLASH_QSPI_ID0 == p_flash_init->spi_type){
            g_qspi_ctl.qspi_id = APP_QSPI_ID_0;
        } else if(FLASH_QSPI_ID1 == p_flash_init->spi_type){
            g_qspi_ctl.qspi_id = APP_QSPI_ID_1;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
        } else {
            g_qspi_ctl.qspi_id = APP_QSPI_ID_2;
#endif
        }

        if(p_flash_init->is_high_freq) {
            qspi_params.init.clock_prescaler = SSI_HIGH_FREQ_CLOCK_PRESCALER;
        } else {
            qspi_params.init.clock_prescaler = SSI_LOW_FREQ_CLOCK_PRESCALER;
        }

        qspi_params.init.clock_mode      = QSPI_TIMING_MODE;
        if(qspi_params.init.clock_prescaler == 2)
        {
            qspi_params.init.rx_sample_delay = 1;
        }
        else
        {
            qspi_params.init.rx_sample_delay = 0;
        }

        qspi_params.id                     = g_qspi_ctl.qspi_id;
        qspi_params.pin_cfg.cs.type        = p_flash_init->flash_io.spi_cs.gpio;
        qspi_params.pin_cfg.cs.pin         = p_flash_init->flash_io.spi_cs.pin;
        qspi_params.pin_cfg.cs.mux         = p_flash_init->flash_io.spi_cs.mux;
        qspi_params.pin_cfg.cs.mode        = APP_IO_MODE_MUX;
        qspi_params.pin_cfg.cs.pull        = APP_IO_PULLUP;
        qspi_params.pin_cfg.cs.enable      = APP_QSPI_PIN_ENABLE;
        qspi_params.pin_cfg.clk.type       = p_flash_init->flash_io.spi_clk.gpio;
        qspi_params.pin_cfg.clk.pin        = p_flash_init->flash_io.spi_clk.pin;
        qspi_params.pin_cfg.clk.mux        = p_flash_init->flash_io.spi_clk.mux;
        qspi_params.pin_cfg.clk.mode       = APP_IO_MODE_MUX;
        qspi_params.pin_cfg.clk.pull       = APP_IO_PULLUP;
        qspi_params.pin_cfg.clk.enable     = APP_QSPI_PIN_ENABLE;
        qspi_params.pin_cfg.io_0.type      = p_flash_init->flash_io.spi_io0.qspi_io0.gpio;
        qspi_params.pin_cfg.io_0.pin       = p_flash_init->flash_io.spi_io0.qspi_io0.pin;
        qspi_params.pin_cfg.io_0.mux       = p_flash_init->flash_io.spi_io0.qspi_io0.mux;
        qspi_params.pin_cfg.io_0.mode      = APP_IO_MODE_MUX;
        qspi_params.pin_cfg.io_0.pull      = APP_IO_PULLUP;
        qspi_params.pin_cfg.io_0.enable    = APP_QSPI_PIN_ENABLE;
        qspi_params.pin_cfg.io_1.type      = p_flash_init->flash_io.spi_io1.qspi_io1.gpio;
        qspi_params.pin_cfg.io_1.pin       = p_flash_init->flash_io.spi_io1.qspi_io1.pin;
        qspi_params.pin_cfg.io_1.mux       = p_flash_init->flash_io.spi_io1.qspi_io1.mux;
        qspi_params.pin_cfg.io_1.mode      = APP_IO_MODE_MUX;
        qspi_params.pin_cfg.io_1.pull      = APP_IO_PULLUP;
        qspi_params.pin_cfg.io_1.enable    = APP_QSPI_PIN_ENABLE;
        qspi_params.pin_cfg.io_2.type      = p_flash_init->flash_io.qspi_io2.gpio;
        qspi_params.pin_cfg.io_2.pin       = p_flash_init->flash_io.qspi_io2.pin;
        qspi_params.pin_cfg.io_2.mux       = p_flash_init->flash_io.qspi_io2.mux;
        qspi_params.pin_cfg.io_2.mode      = APP_IO_MODE_MUX;
        qspi_params.pin_cfg.io_2.pull      = APP_IO_PULLUP;
        qspi_params.pin_cfg.io_2.enable    = APP_QSPI_PIN_ENABLE;
        qspi_params.pin_cfg.io_3.type      = p_flash_init->flash_io.qspi_io3.gpio;
        qspi_params.pin_cfg.io_3.pin       = p_flash_init->flash_io.qspi_io3.pin;
        qspi_params.pin_cfg.io_3.mux       = p_flash_init->flash_io.qspi_io3.mux;
        qspi_params.pin_cfg.io_3.mode      = APP_IO_MODE_MUX;
        qspi_params.pin_cfg.io_3.pull      = APP_IO_PULLUP;
        qspi_params.pin_cfg.io_3.enable    = APP_QSPI_PIN_ENABLE;

        qspi_params.dma_cfg.dma_instance = QSPI_USED_DMA;
        qspi_params.dma_cfg.dma_channel = QSPI_USED_DMA_CH;

        app_qspi_dma_deinit(g_qspi_ctl.qspi_id);
        app_qspi_deinit(g_qspi_ctl.qspi_id);
        ret = app_qspi_init(&qspi_params, app_qspi_callback);
        if (ret != 0)
        {
            printf("QSPI initial failed! Please check the input paraments.");
            return ;
        }
        ret = app_qspi_dma_init(&qspi_params);
        if (ret != 0)
        {
            printf("QSPI initial dma failed! Please check the input paraments.");
            return ;
        }

        //set qspi hold/wp pin to high
        app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
        io_init.mode = APP_IO_MODE_OUTPUT;
        io_init.pull = APP_IO_PULLUP;
        io_init.pin  = qspi_params.pin_cfg.io_2.pin;
        io_init.mux  = APP_IO_MUX;
        app_io_init(qspi_params.pin_cfg.io_2.type, &io_init);

        io_init.mode = APP_IO_MODE_OUTPUT;
        io_init.pull = APP_IO_PULLUP;
        io_init.pin  = qspi_params.pin_cfg.io_3.pin;
        io_init.mux  = APP_IO_MUX;
        app_io_init(qspi_params.pin_cfg.io_3.type , &io_init);

        app_io_write_pin(qspi_params.pin_cfg.io_2.type, qspi_params.pin_cfg.io_2.pin, APP_IO_PIN_SET);
        app_io_write_pin(qspi_params.pin_cfg.io_3.type, qspi_params.pin_cfg.io_3.pin, APP_IO_PIN_SET);
    }

    return;
}

uint32_t spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t page_ofs, write_size, write_cont = nbytes;
    hal_status_t status      = HAL_OK;

    while (write_cont)
    {
        page_ofs = address & 0xFF;
        write_size = EXFLASH_SIZE_PAGE_BYTES - page_ofs;

        if (write_cont < write_size)
        {
            write_size = write_cont;
            write_cont = 0;
        }
        else
        {
            write_cont -= write_size;
        }

        spi_flash_write_enable();

        if (FLASH_SPIM_ID == g_flash_init.spi_type)
        {
            spim_flash_write(address, buffer, write_size);
        }
        else
        {
            if(g_flash_init.is_dual_line) {
                qspi_flash_dual_write(address, buffer, write_size);
            } else {
                qspi_flash_write(address, buffer, write_size);
            }
        }

        while(spi_flash_read_status() & 0x1);

        address += write_size;
        buffer += write_size;
    }

    return ((status == HAL_OK) ? nbytes : 0);
}

uint32_t spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t count = 0;

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        count = spim_flash_read(address, buffer, nbytes);
    }
    else
    {
        if(g_flash_init.is_dual_line) {
            count = qspi_flash_dual_read(address, buffer, nbytes);
        } else {
            count = qspi_flash_read(address, buffer, nbytes);
        }
    }

    return count;
}

bool spi_flash_sector_erase(uint32_t address, uint32_t size)
{
    bool status = true;

    uint32_t erase_addr = address;
    uint32_t sector_ofs, erase_size, erase_cont = size;

    while (erase_cont)
    {
        sector_ofs = erase_addr & 0xFFF;
        erase_size = EXFLASH_SIZE_SECTOR_BYTES - sector_ofs;

        if (erase_cont < erase_size)
        {
            erase_size = erase_cont;
            erase_cont = 0;
        }
        else
        {
            erase_cont -= erase_size;
        }

        spi_flash_write_enable();

        if (FLASH_SPIM_ID == g_flash_init.spi_type)
        {
            status = spim_flash_sector_erase(erase_addr);
        }
        else
        {
            status = qspi_flash_sector_erase(erase_addr);
        }

        while(spi_flash_read_status() & 0x1);

        erase_addr += erase_size;
    }

    return status;
}

bool spi_flash_chip_erase(void)
{
    uint32_t ret;
    uint8_t control_frame[1] = {SPI_FLASH_CMD_CE};

    spi_flash_write_enable();

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        g_qspi_ctl.spi_tmt_done = 0;
        ret = app_spi_dma_transmit_async(g_qspi_ctl.spi_id, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.spi_tmt_done == 0);
    }
    else
    {
        g_qspi_ctl.qspi_tmt_done = 0;
        ret = app_qspi_dma_transmit_async_ex(g_qspi_ctl.qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.qspi_tmt_done == 0);
    }
    while(spi_flash_read_status() & 0x1);

    return ((ret == APP_DRV_SUCCESS) ? true : false);
}

void spi_flash_chip_reset(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RSTEN};

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        g_qspi_ctl.spi_tmt_done = 0;
        app_spi_dma_transmit_async(g_qspi_ctl.spi_id, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.spi_tmt_done == 0);
    }
    else
    {
        g_qspi_ctl.qspi_tmt_done = 0;
        app_qspi_dma_transmit_async_ex(g_qspi_ctl.qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.qspi_tmt_done == 0);
    }

    control_frame[0] = SPI_FLASH_CMD_RST;
    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        g_qspi_ctl.spi_tmt_done = 0;
        app_spi_dma_transmit_async(g_qspi_ctl.spi_id, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.spi_tmt_done == 0);
    }
    else
    {
        g_qspi_ctl.qspi_tmt_done = 0;
        app_qspi_dma_transmit_async_ex(g_qspi_ctl.qspi_id, QSPI_DATA_MODE_SPI, QSPI_DATASIZE_08_BITS, control_frame, sizeof(control_frame));
        while(g_qspi_ctl.qspi_tmt_done == 0);
    }

    return;
}

uint32_t spi_flash_device_id(void)
{
    uint8_t data[3] = {0};

    if (FLASH_SPIM_ID == g_flash_init.spi_type)
    {
        uint8_t control_frame[1] = {SPI_FLASH_CMD_RDID};

        g_qspi_ctl.spi_tx_rx_done = 0;
        app_spi_dma_read_eeprom_async(g_qspi_ctl.spi_id, control_frame, data, 1, 3);
        while(g_qspi_ctl.spi_tx_rx_done == 0);
    }
    else
    {
        qspi_command_t command = {
            .instruction      = SPI_FLASH_CMD_RDID,
            .address          = 0,
            .instruction_size = QSPI_INSTSIZE_08_BITS,
            .address_size     = QSPI_ADDRSIZE_00_BITS,
            .data_size        = QSPI_DATASIZE_08_BITS,
            .dummy_cycles     = 0,
            .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
            .data_mode        = QSPI_DATA_MODE_SPI,
            .length           = 3,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
            .clock_stretch_en = 1,
#endif
            };
        g_qspi_ctl.qspi_rcv_done = 0;
        app_qspi_dma_command_receive_async(g_qspi_ctl.qspi_id, &command, data);
        while(g_qspi_ctl.qspi_rcv_done == 0);
    }

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

void spi_flash_device_info(uint32_t *id, uint32_t *size)
{
    if (NULL == id || NULL == size)
    {
        return;
    }

    *id   = spi_flash_device_id();
    *size = spi_flash_device_size();

    return;
}
