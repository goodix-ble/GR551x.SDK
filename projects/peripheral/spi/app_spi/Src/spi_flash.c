#include <stdlib.h>
#include <string.h>
#include "spi_flash.h"
#include "app_pwr_mgmt.h"
#include "board_SK.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define SPI_CLOCK_PRESCALER             16u             /* The SPI CLOCK Freq = Peripheral CLK/SPI_CLOCK_PRESCALER */
#define SPI_SOFT_CS_MODE_ENABLE         1u              /* suggest to enable SOFT CS MODE */
#define SPI_SOFT_CS_MODE_DISABLE        0u              /* suggest to enable SOFT CS MODE */
#define SPI_WAIT_TIMEOUT_MS             1500u           /* default time(ms) for wait operation */

#if SPI_CLOCK_PRESCALER == 2u
    #define RX_SAMPLE_DELAY             1u
#else
    #define RX_SAMPLE_DELAY             0u
#endif

/* master spi parameters */
app_spi_params_t spi_params = {
    .id = APP_SPI_ID_MASTER,
    .pin_cfg = {
       .cs = {
           .type   = APP_SPIM_CS_IO_TYPE,
           .mux    = APP_SPIM_CS_PINMUX,
           .pin    = APP_SPIM_CS_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .clk  = {
           .type   = APP_SPIM_CLK_IO_TYPE,
           .mux    = APP_SPIM_CLK_PINMUX,
           .pin    = APP_SPIM_CLK_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .mosi = {
           .type   = APP_SPIM_MOSI_IO_TYPE,
           .mux    = APP_SPIM_MOSI_PINMUX,
           .pin    = APP_SPIM_MOSI_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
       .miso = {
           .type   = APP_SPIM_MISO_IO_TYPE,
           .mux    = APP_SPIM_MISO_PINMUX,
           .pin    = APP_SPIM_MISO_PIN,
           .mode   = APP_IO_MODE_MUX,
           .pull   = APP_IO_PULLUP,
           .enable = APP_SPI_PIN_ENABLE,
       },
    },
    .dma_cfg = {
        .tx_dma_instance = DMA0,
        .rx_dma_instance = DMA0,
        .tx_dma_channel  = DMA_Channel0,
        .rx_dma_channel  = DMA_Channel1,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .wait_timeout_ms = SPI_WAIT_TIMEOUT_MS,
        .extend = 0,
#endif
    },
    .init = {
        .data_size      = SPI_DATASIZE_8BIT,
        .clock_polarity = SPI_POLARITY_LOW,
        .clock_phase    = SPI_PHASE_1EDGE,
        .baudrate_prescaler = SPI_CLOCK_PRESCALER,
        .ti_mode        = SPI_TIMODE_DISABLE,
        .slave_select   = SPI_SLAVE_SELECT_0,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        .rx_sample_delay = RX_SAMPLE_DELAY,
#endif
    },
 
    .is_soft_cs = SPI_SOFT_CS_MODE_ENABLE,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;
volatile uint8_t g_master_trdone = 0;
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void app_spi_master_callback(app_spi_evt_t *p_evt)
{
    if (p_evt->type == APP_SPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_RX_CPLT)
    {
        g_master_rdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_TX_RX_CPLT)
    {
        g_master_trdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_ERROR)
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

static void spi_flash_write_enable(void)
{
    uint8_t cmd = SPI_FLASH_CMD_WREN;

    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
}

static uint8_t spi_flash_read_status(void)
{
    uint8_t cmd = SPI_FLASH_CMD_RDSR;
    uint8_t state_reg = 0;

    app_spi_read_eeprom_sync(APP_SPI_ID_MASTER, &cmd, &state_reg, 1,  1, 1000);

    return state_reg;
}

static void spi_flash_wait_busy(void)
{
    while((spi_flash_read_status() & 0x01) == 0x01);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint32_t spi_flash_read_id(void)
{
    uint8_t data[3] = {0};
    uint8_t cmd = SPI_FLASH_CMD_RDID;

    app_spi_read_eeprom_sync(APP_SPI_ID_MASTER, &cmd, &data[0], 1,  3, 1000);

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

void spi_flash_sector_erase(uint32_t dst)
{
    uint8_t ctl_frame[4];
    ctl_frame[0] = SPI_FLASH_CMD_SE;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    spi_flash_write_enable();
    spi_flash_wait_busy();

    app_spi_transmit_sync(APP_SPI_ID_MASTER, &ctl_frame[0], 4, 1000);

    spi_flash_wait_busy();
}

void spi_flash_chip_erase(void)
{
    uint8_t cmd = SPI_FLASH_CMD_CE;
    spi_flash_write_enable();
    spi_flash_wait_busy();

    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    spi_flash_wait_busy();
}

void spi_flash_page_program(uint32_t dst, uint8_t *data, uint32_t nbytes)
{
    spi_flash_write_enable();
    spi_flash_wait_busy();
    g_master_tdone = 0;
    app_spim_dma_transmit_with_ia(APP_SPI_ID_MASTER, SPI_FLASH_CMD_PP, dst, data, nbytes);
    while(g_master_tdone == 0);
    spi_flash_wait_busy();
}

void spi_flash_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
    g_master_trdone = 0;
    app_spim_dma_receive_with_ia(APP_SPI_ID_MASTER, SPI_FLASH_CMD_READ, dst, 0, buffer, nbytes);
    while(g_master_trdone == 0);
}

void spi_flash_fast_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
    g_master_trdone = 0;
    app_spim_dma_receive_with_ia(APP_SPI_ID_MASTER, SPI_FLASH_CMD_FREAD, dst, 1, buffer, nbytes);
    while(g_master_trdone == 0);
}

void spi_flash_reset(void)
{
    uint8_t cmd = SPI_FLASH_CMD_RSTEN;
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);

    cmd = SPI_FLASH_CMD_RST;
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
}

void spi_flash_power_down(void)
{
    uint8_t cmd = SPI_FLASH_CMD_DP;

    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
}

void spi_flash_wakeup(void)
{
    uint8_t cmd = SPI_FLASH_CMD_RDP;

    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
}

void spi_flash_init(uint32_t clock_prescaler)
{
    app_drv_err_t ret = APP_DRV_SUCCESS;

    spi_params.init.baudrate_prescaler = clock_prescaler;
    /* Please initialize DMA in the following order. */
    /* Note: Initialization is not allowed during the transmission process. */
    ret = app_spi_init(&spi_params, app_spi_master_callback);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("SPI master initial failed! Please check the input paraments.\r\n");
    }

    ret = app_spi_dma_init(&spi_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("SPI master dma initial failed! Please check the input paraments.\r\n");
    }
    
    /* Reset flash */
    spi_flash_reset();

    /* Wakeup from deep power down */
    spi_flash_wakeup();
}

void spi_flash_deinit(void)
{
    /* Please deinitialize DMA in the following order. */
    app_spi_dma_deinit(spi_params.id);
    app_spi_deinit(spi_params.id);
}
