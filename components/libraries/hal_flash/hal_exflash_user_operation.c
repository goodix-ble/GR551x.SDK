#include "hal_exflash_user_operation.h"

#define FLASH_MANU_ID_INVALID0    0x00
#define FLASH_MANU_ID_INVALID1    0xFF
#define FLASH_QE_BIT_POS          9

extern exflash_handle_t g_exflash_handle;
extern hal_status_t hal_xqspi_command_receive_patch(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry);
extern hal_status_t exflash_enable_write(exflash_handle_t *p_exflash);
extern hal_status_t exflash_wait_busy(exflash_handle_t *p_exflash, uint32_t retry);
extern hal_status_t exflash_wakeup(exflash_handle_t *p_exflash);
extern hal_status_t exflash_deepsleep(exflash_handle_t *p_exflash);
extern hal_status_t exflash_check_id(exflash_handle_t *p_exflash);
extern hal_status_t hal_xqspi_init_ext(xqspi_handle_t *p_xqspi);

/* NOTE : Here is a function demo if user want to realize XIP Flash Quad mode enable.
          This is olny used when the default function is not work in GR5515I0ND.
 */
#define ENABLE_USER_QUAD_FUNC  0
#if ENABLE_USER_QUAD_FUNC == 1
SECTION_RAM_CODE static hal_status_t exflash_read_status_register(exflash_handle_t *p_exflash, uint16_t *p_status_register)
{
    hal_status_t status = HAL_OK;
    xqspi_command_t command;
    uint8_t         tmp = 0;

    command.inst           = 0x05;
    command.addr           = 0;
    command.inst_size      = XQSPI_INSTSIZE_08_BITS;
    command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles   = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode      = XQSPI_DATA_MODE_SPI;
    command.length         = 1;

    *p_status_register = 0x00;
    status = hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
    *p_status_register |= tmp;

    command.inst = 0x35;
    tmp = 0;
    hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
    *p_status_register |= (tmp << 8);

    return status;
}

SECTION_RAM_CODE static hal_status_t exflash_write_status_register(exflash_handle_t *p_exflash, uint16_t data)
{
    hal_status_t status = HAL_OK;

    uint8_t w_sr_cmd[3];
    w_sr_cmd[0] = 0x01; //Write Status Register
    w_sr_cmd[1] = data & 0xFF;
    w_sr_cmd[2] = (data >> 8) & 0xFF;

    do {
        status = exflash_enable_write(p_exflash);
        if (HAL_OK != status)
            break;

        status = hal_xqspi_transmit(p_exflash->p_xqspi, w_sr_cmd, sizeof(w_sr_cmd), 1000);
        if (HAL_OK != status)
            break;

        status = exflash_wait_busy(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);
    } while(0);

    return status;
}

SECTION_RAM_CODE hal_status_t enable_quad(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;
    uint16_t status_reg_value = 0;

    do {
        exflash_read_status_register(p_exflash, &status_reg_value);
        if (((status_reg_value & (1<<FLASH_QE_BIT_POS)) >> FLASH_QE_BIT_POS) == ENABLE)
            break;

        status_reg_value |= (uint16_t)(1<< FLASH_QE_BIT_POS);

        status = exflash_write_status_register(p_exflash, status_reg_value);
        if (HAL_OK != status)
            break;

        status_reg_value = 0;
        exflash_read_status_register(p_exflash, &status_reg_value);
        if (((status_reg_value & (1<<FLASH_QE_BIT_POS)) >> FLASH_QE_BIT_POS) == ENABLE)
            break;
    } while(1);
    return status;
}

SECTION_RAM_CODE hal_status_t platform_exflash_enable_quad(exflash_handle_t *p_exflash)
{
    hal_status_t    status = HAL_OK;

    if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
    {
        /* Disable global interrupt, aviod call function in flash */
        GLOBAL_EXCEPTION_DISABLE();

        /* Configure XQSPI initial paraments */
        p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
        hal_xqspi_init_ext(p_exflash->p_xqspi);
        /* Update EXFALSH state */
        p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        GLOBAL_EXCEPTION_ENABLE();
    }
    else
    {
        /* EXFLASH in QSPI mode */
        exflash_wakeup(p_exflash);
        status = (hal_status_t)exflash_check_id(p_exflash);
        if (HAL_OK != status)
        {
            p_exflash->error_code = HAL_EXFLASH_ERROR_ID;
        } else {
            /* Update EXFALSH state */
            p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        }
    }
    status = enable_quad(p_exflash);
    if (XQSPI_WORK_MODE_XIP == p_exflash->fw_mode)
    {
        /* Disable global interrupt, aviod call function in flash */
        GLOBAL_EXCEPTION_DISABLE();
        p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
        hal_xqspi_init_ext(p_exflash->p_xqspi);
        p_exflash->state = HAL_EXFLASH_STATE_READY;
        GLOBAL_EXCEPTION_ENABLE();
    }
    else
    {
        exflash_deepsleep(p_exflash);
        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }

    return status;
}
#endif /* ENABLE_USER_QUAD_FUNC */

/* NOTE : Here is a function demo if user want to realize XIP Flash operations
          Please refer hal_flash_read_identification_id()
 */
static uint32_t s_identification_id;
static SECTION_RAM_CODE hal_status_t exflash_read_identification_id(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;
    xqspi_command_t command;
    uint8_t id[3];

    command.inst = SPI_FLASH_CMD_RDID;
    command.addr = 0;
    command.inst_size = XQSPI_INSTSIZE_08_BITS;
    command.addr_size = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode = XQSPI_DATA_MODE_SPI;
    command.length = 3;

    status = hal_xqspi_command_receive_patch(p_exflash->p_xqspi, &command, id, 1000);
    if (HAL_OK != status)
    {
        return status;
    }

    if ((FLASH_MANU_ID_INVALID0 != id[0]) && (FLASH_MANU_ID_INVALID1 != id[0]))
    {
        s_identification_id = id[2] + (id[1] << 8) + (id[0] << 16);
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

uint32_t hal_flash_read_identification_id(void)
{
    hal_exflash_operation(&g_exflash_handle, exflash_read_identification_id);
    return s_identification_id;
}
