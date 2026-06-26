/**
 *****************************************************************************************
 *
 * @file dfu_master.c
 *
 * @brief  DFU master Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2024 GOODIX
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
 ****************************************************************************************
 */
#include "dfu_master.h"
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */
#define CMD_FRAME_HEADER_L           0x44U    /**< CMD header low byte. */
#define CMD_FRAME_HEADER_H           0x47U    /**< CMD header high byte. */
#define GET_INFO                     0x01U    /**< Get info cmd. */
#define PROGRAM_START                0x23U    /**< Program start cmd. */
#define PROGRAM_FLASH                0x24U    /**< Program flash cmd. */
#define PROGRAM_END                  0x25U    /**< Program end cmd. */
#define SYSTEM_INFO                  0x27U    /**< System information cmd. */
#define DFU_MODE_SET                 0x41U    /**< Dfu mode set cmd. */
#define DFU_FW_INFO_GET              0x42U    /**< Dfu fw info get cmd. */

#define ACK_SUCCESS                  0x01U    /**< CMD ack success. */
#define ACK_ERROR                    0x02U    /**< CMD ack error. */

#define DFU_ERASE_REGION_NOT_ALIGNED 0x00U    /**< FW erase region not aligned. */
#define DFU_ERASE_START_SUCCESS      0x01U    /**< FW erase start success event. */
#define DFU_ERASE_SUCCESS            0x02U    /**< FW erase flash success event. */
#define DFU_ERASE_END_SUCCESS        0x03U    /**< FW erase end success event. */
#define DFU_ERASE_REGIONS_OVERLAP    0x04U    /**< FW erase regions overlap. */
#define DFU_ERASE_FAIL               0x05U    /**< FW erase flash fail event. */
#define DFU_ERASE_REGIONS_NOT_EXIST  0x06U    /**< FW erase regions not exist. */
#define FAST_DFU_FLASH_SUCCESS       0xFFU    /**< FW write flash success. */

#define FLASH_OP_SECTOR_SIZE         0x1000U  /**< Flash sector size. */
#define PATTERN_VALUE                0x4744U  /**< Pattern value. */

#define FW_SIGN_FLAG_OFFSET          72U      /**< Firmware sign flag offset. */
#define SIGN_FW_TYPE                 0x10U    /**< Sign Firmware Type. */
#define NORMAL_FW_TYPE               0x00U    /**< Normal Firmware Type. */

#define DFU_SIGN_LEN                         856U   /**< Firmware signature length. */
#define DFU_IMAGE_INFO_LEN                   48U    /**< Image information length. */

#define DFU_CMD_GET_SYSTEM_INFO_DATA_LEN     7U     /**< Get system information command data length. */
#define DFU_CMD_GET_SYSTEM_INFO_LEN_L_POS    5U     /**< Get system information command data length low byte position. */
#define DFU_CMD_GET_SYSTEM_INFO_LEN_H_POS    6U     /**< Get system information command length high byte position. */
#define DFU_CMD_MODE_SET_DATA_LEN            1U     /**< DFU mode set command data length. */
#define DFU_CMD_PRO_START_DATA_LEN           41U    /**< Program start command data length. */
#define DFU_CMD_PRO_END_DATA_LEN             5U     /**< Program end command data length. */
#define DFU_CMD_PRO_FLASH_HEAD_LEN           7U     /**< Program flash command header length. */
#define DFU_CMD_PRO_FLASH_LEN_L_POS          5U     /**< Program flash command data length low byte position. */
#define DFU_CMD_PRO_FLASH_LEN_H_POS          6U     /**< Program flash command data length high byte position. */
#define DFU_RSP_DFU_VERSION_POS              17U    /**< Get info command response DFU version position. */
#define DFU_RSP_SYS_INFO_OP_POS              1U     /**< Get system info command response operation position. */
#define DFU_RSP_SYS_INFO_DATA_POS            8U     /**< Get system info command response data position. */
#define DFU_RSP_RUN_POSITION_POS             5U     /**< Get firmware info command response firmware run position position. */
#define DFU_RSP_IMG_INFO_POS                 6U     /**< Get firmware info command response image info position. */
#define DFU_RSP_ERASE_POS                    6U     /**< Program start command response erased sector position in fast mode . */

#define DFU_FRAME_HRD_L_POS                  0U     /**< DFU frame header low byte position. */
#define DFU_FRAME_HRD_H_POS                  1U     /**< DFU frame header high byte position. */
#define DFU_FRAME_TYPE_L_POS                 2U     /**< DFU frame type low byte position. */
#define DFU_FRAME_TYPE_H_POS                 3U     /**< DFU frame type high byte position. */
#define DFU_FRAME_LEN_L_POS                  4U     /**< DFU frame length low byte position. */
#define DFU_FRAME_LEN_H_POS                  5U     /**< DFU frame length high byte position. */
#define DFU_FRAME_DATA_POS                   6U     /**< DFU frame data position. */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief DFU master submachine state. */
typedef enum
{
    CHECK_FRAME_L_STATE = 0x00,
    CHECK_FRAME_H_STATE,
    RECEIVE_CMD_TYPE_L_STATE,
    RECEIVE_CMD_TYPE_H_STATE,
    RECEIVE_LEN_L_STATE,
    RECEIVE_LEN_H_STATE,
    RECEIVE_DATA_STATE,
    RECEIVE_CHECK_SUM_L_STATE,
    RECEIVE_CHECK_SUM_H_STATE,
} cmd_parse_state_t;

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief DFU master receive frame structure define. */
typedef struct
{
    uint16_t cmd_type;
    uint16_t data_len;
    uint8_t  data[DFU_RX_FRAME_MAX - DFU_FRAME_DATA_POS];
    uint16_t check_sum;
} receive_frame_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static uint8_t           s_dfu_tx_buf[DFU_TX_FRAME_MAX];
static receive_frame_t   s_receive_frame;
static bool              s_cmd_receive_flag;
static uint16_t          s_receive_data_count;
static uint16_t          s_receive_check_sum;
static boot_info_t       s_boot_info;    // Slave bootloader information
static dfu_img_info_t    s_now_img_info; // FW image information about the firmware to be upgraded
#if DFU_BANK_MODE == DFU_MODE_COPY_UPGRADE
dfu_img_info_t           s_app_info;     // Image information in slave APP Info area
#endif
static uint32_t          s_img_data_addr;
static uint32_t          s_all_check_sum;
static uint32_t          s_file_size;
static uint32_t          s_programed_size;
static bool              s_run_fw_flag;
static dfu_m_func_cfg_t *s_p_func_cfg;

#if DFU_ASYNC_TX_ENABLE
static uint16_t          s_once_size;
static uint16_t          s_sent_len;
static uint16_t          s_all_send_len;
#endif
static cmd_parse_state_t s_parse_state = CHECK_FRAME_L_STATE;

static bool              s_sec_flag = false;
static bool              s_new_version_flag = false;
#if DFU_BLE_ENABLE
static uint16_t          s_erase_sectors;
static volatile uint8_t  s_ble_fast_send_cplt_flag;
static uint8_t           s_fast_dfu_mode = FAST_DFU_MODE_DISABLE;
#endif
static uint32_t          s_dfu_save_addr = 0;  // New FW save address in slave
static uint32_t          s_dfu_timeout_start_time = 0;
static bool              s_dfu_timeout_started = false;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Function for transforming an array into a 32 digit unsigned number according to Little-Endian.
 *
 * @param[in] p_buf: Pointer to an array need to be transformed.
 *
 * @retval ::The result of transforming
 *****************************************************************************************
 */
static uint32_t dfu_le32toh(const void *buf)
{
    const uint8_t *u8ptr;
    uint32_t x;
    u8ptr = buf;
    x = u8ptr[0];
    x |= (uint32_t) u8ptr[1] << 8;
    x |= (uint32_t) u8ptr[2] << 16;
    x |= (uint32_t) u8ptr[3] << 24;
    return x;
}

/**
 *****************************************************************************************
 * @brief Function for transforming a 32 digit number into an array according to Little-Endian.
 *
 * @param[out] p_buf: Pointer to an array.
 * @param[in]  x:     The 32 digit number need to be transformed.
 *****************************************************************************************
 */
static void dfu_htole32(void *buf, uint32_t x)
{
    uint8_t *u8ptr;
    u8ptr = buf;
    u8ptr[0] = (uint8_t) x;
    u8ptr[1] = (uint8_t)(x >> 8);
    u8ptr[2] = (uint8_t)(x >> 16);
    u8ptr[3] = (uint8_t)(x >> 24);
}

/**
 *****************************************************************************************
 * @brief Function for getting updated firmware information.
 *
 * @param[in]  img_info: Pointer of firmware information
 *****************************************************************************************
 */
static void dfu_m_get_img_info(dfu_img_info_t *img_info)
{
    if (s_p_func_cfg->dfu_m_get_img_info != NULL)
    {
        s_p_func_cfg->dfu_m_get_img_info(img_info);
    }
}

/**
 *****************************************************************************************
 * @brief Get the firmware data to be upgraded.
 *
 * @param[in]  addr: Get data address.
 * @param[in]  data: Pointer of get data.
 * @param[in]  len: Get data length
 *****************************************************************************************
 */
static void dfu_m_get_img_data(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (s_p_func_cfg->dfu_m_get_img_data != NULL)
    {
        s_p_func_cfg->dfu_m_get_img_data(addr, data, len);
    }
}

/**
 *****************************************************************************************
 * @brief Send data to slave.
 *
 * @param[in]  data: Pointer to data.
 * @param[in]  len: Length of data.
 *****************************************************************************************
 */
static void dfu_m_send_data(uint8_t *data, uint16_t len)
{
    if (s_p_func_cfg->dfu_m_send_data != NULL)
    {
        s_p_func_cfg->dfu_m_send_data(data, len);
    }
}

/**
 *****************************************************************************************
 * @brief DFU master event handler.
 *
 * @param[in]  event: DFU event.
 * @param[in]  progress: Firmware upgrade progress.
 *****************************************************************************************
 */
static void dfu_m_event_handler(dfu_m_event_t event, uint8_t progress)
{
    if (s_p_func_cfg->dfu_m_event_handler != NULL)
    {
        s_p_func_cfg->dfu_m_event_handler(event, progress);
    }
}

/**
 *****************************************************************************************
 * @brief Get the time in milliseconds for timeout.
 *
 * @retval The time in milliseconds.
 *****************************************************************************************
 */
static uint32_t dfu_m_get_time(void)
{
    if (s_p_func_cfg->dfu_m_get_time != NULL)
    {
        return s_p_func_cfg->dfu_m_get_time();
    }
    else
    {
        return 0;
    }
}

/**
 *****************************************************************************************
 * @brief Check command validity.
 *
 *****************************************************************************************
 */
static void dfu_m_cmd_check(void)
{
    for (uint16_t i = 0; i < s_receive_frame.data_len; i++)
    {
        s_receive_check_sum += s_receive_frame.data[i];
    }

    if (s_receive_check_sum == s_receive_frame.check_sum)
    {
        s_cmd_receive_flag = true;
    }
    else
    {
        s_cmd_receive_flag = false;
        dfu_m_event_handler(FRAME_CHECK_ERROR, 0);
    }
}

/**
 *****************************************************************************************
 * @brief DFU master sends data to slave.
 *
 * @param[in]  len: Length of data.
 *****************************************************************************************
 */
static void dfu_m_send(uint16_t len)
{
#if DFU_ASYNC_TX_ENABLE
    s_all_send_len = len;
    if (len >= s_once_size)
    {
        s_sent_len = s_once_size;
    }
    else
    {
        s_sent_len = len;
    }
    dfu_m_send_data(s_dfu_tx_buf, s_sent_len);
#else
    dfu_m_send_data(s_dfu_tx_buf, len);
#endif
}

/**
 *****************************************************************************************
 * @brief Make frame and send to slave.
 *
 * @param[in]  data: Pointer to send data.
 * @param[in]  len: Length of data.
 * @param[in]  cmd_type: Commander type.
 *****************************************************************************************
 */
static void dfu_m_send_frame(const uint8_t *p_data, uint16_t len, uint16_t cmd_type)
{
    uint16_t i;
    uint16_t check_sum = 0;
    s_dfu_tx_buf[DFU_FRAME_HRD_L_POS]  = CMD_FRAME_HEADER_L;
    s_dfu_tx_buf[DFU_FRAME_HRD_H_POS]  = CMD_FRAME_HEADER_H;
    s_dfu_tx_buf[DFU_FRAME_TYPE_L_POS] = (uint8_t)cmd_type;
    s_dfu_tx_buf[DFU_FRAME_TYPE_H_POS] = (uint8_t)(cmd_type >> 8);
    s_dfu_tx_buf[DFU_FRAME_LEN_L_POS]  = (uint8_t)len;
    s_dfu_tx_buf[DFU_FRAME_LEN_H_POS]  = (uint8_t)(len >> 8);

    for (i = DFU_FRAME_TYPE_L_POS; i < DFU_FRAME_DATA_POS; i++)
    {
        check_sum += s_dfu_tx_buf[i];
    }

    memcpy(&s_dfu_tx_buf[DFU_FRAME_DATA_POS], (void *)p_data, len);
    for (i = 0; i < len; i++)
    {
        check_sum += s_dfu_tx_buf[DFU_FRAME_DATA_POS + i];
    }
    s_dfu_tx_buf[len + DFU_FRAME_DATA_POS] = (uint8_t)check_sum;
    s_dfu_tx_buf[len + DFU_FRAME_DATA_POS + 1U] = (uint8_t)(check_sum >> 8);
    dfu_m_send(len + DFU_FRAME_DATA_POS + 2U);
}

/**
 *****************************************************************************************
 * @brief Program slave flash.
 *
 * @param[in]  len: Length of data written to flash.
 *****************************************************************************************
 */
static void dfu_m_program_flash(uint16_t len)
{
    uint8_t data[DFU_ONCE_PROGRAM_LEN + DFU_CMD_PRO_FLASH_HEAD_LEN];
    s_programed_size += len;

    dfu_m_get_img_data(s_img_data_addr, &data[DFU_CMD_PRO_FLASH_HEAD_LEN], len);
    for (uint32_t i = 0; i < len; i++)
    {
        s_all_check_sum += data[i + DFU_CMD_PRO_FLASH_HEAD_LEN];
    }
    data[0] = 0x01U; // Write flash base on image Info

    dfu_htole32(&data[1], s_dfu_save_addr);

    data[DFU_CMD_PRO_FLASH_LEN_L_POS] = (uint8_t)len;
    data[DFU_CMD_PRO_FLASH_LEN_H_POS] = (uint8_t)(len >> 8);

    dfu_m_send_frame(data, len + DFU_CMD_PRO_FLASH_HEAD_LEN, PROGRAM_FLASH);
    s_dfu_save_addr += len;
    s_img_data_addr += len;
}

/**
 *****************************************************************************************
 * @brief Program slave flash in fast mode.
 * Note: Fast mode can be used in BLE mode.
 *****************************************************************************************
 */
#if DFU_BLE_ENABLE
static void dfu_m_fast_program_flash(void)
{
    uint16_t remain;
    uint16_t i = 0U;
    uint8_t progress = 0U;
    while (s_programed_size != s_file_size)
    {
        if ((s_ble_fast_send_cplt_flag != 0U) || (s_programed_size == 0U))
        {
            s_ble_fast_send_cplt_flag = 0U;

            dfu_m_get_img_data(s_img_data_addr, &s_dfu_tx_buf[0], s_once_size);

            if ((s_programed_size + s_once_size) > s_file_size)
            {
                remain = (uint16_t)(s_file_size - s_programed_size);
                dfu_m_send(remain);
                for (i = 0U; i < remain; i++)
                {
                    s_all_check_sum += s_dfu_tx_buf[i];
                }
                s_programed_size += remain;
            }
            else
            {
                s_programed_size += s_once_size;
                dfu_m_send(s_once_size);
                for (i = 0U; i < s_once_size; i++)
                {
                    s_all_check_sum += s_dfu_tx_buf[i];
                }
            }

            progress = (uint8_t)((s_programed_size * 100U) / s_file_size);
            dfu_m_event_handler(FAST_DFU_PRO_FLASH_SUCCESS, progress);
            s_img_data_addr += s_once_size;
        }
    }
}
#endif

/**
 *****************************************************************************************
 * @brief Function for start update firmware.
 *
 * @param[in]  security: Upgrade firmware is encrypted?.
 * @param[in]  run_fw: Whether to run the firmware immediately after the upgrade.
 *****************************************************************************************
 */
static void dfu_m_program_start(bool security, bool run_fw)
{
    DFU_MASTER_LOG(__FUNCTION__);
    s_run_fw_flag = run_fw;
    s_img_data_addr = DFU_FW_SAVE_MASTER_ADDR;
    s_all_check_sum = 0U;
    s_programed_size = 0U;

    uint8_t data[DFU_CMD_PRO_START_DATA_LEN];
    data[0] = 0;
    dfu_m_get_img_info(&s_now_img_info);
    if ((s_now_img_info.pattern != PATTERN_VALUE) || \
       ((s_now_img_info.boot_info.load_addr % FLASH_OP_SECTOR_SIZE) != 0U))
    {
        dfu_m_event_handler(IMG_INFO_CHECK_FAIL, 0);
        return;
    }

#if DFU_GET_FW_SAVE_ADDR_METHOD == GET_SAVE_ADDR_BY_NEW_FW
    s_dfu_save_addr = s_now_img_info.boot_info.load_addr;
#elif DFU_GET_FW_SAVE_ADDR_METHOD == GET_SAVE_ADDR_BY_USER
    s_now_img_info.boot_info.load_addr = DFU_FW_SAVE_ADDR;
    s_dfu_save_addr = DFU_FW_SAVE_ADDR;
#else
    s_now_img_info.boot_info.load_addr = s_dfu_save_addr;
#endif

    uint32_t bin_size = s_now_img_info.boot_info.bin_size + DFU_IMAGE_INFO_LEN;
    uint32_t tail_size = DFU_IMAGE_INFO_LEN;
    if (security)//security mode
    {
        bin_size += DFU_SIGN_LEN;
        tail_size += DFU_SIGN_LEN;
    }
    else
    {
        uint32_t fw_sign_flag0 = 0;
        uint32_t fw_sign_flag1 = 0;
        uint32_t flag_addr = s_img_data_addr + bin_size;
        dfu_m_get_img_data(flag_addr, (uint8_t*)&fw_sign_flag0, (uint16_t)sizeof(fw_sign_flag0));
        flag_addr = s_img_data_addr + bin_size + FW_SIGN_FLAG_OFFSET;
        dfu_m_get_img_data(flag_addr, (uint8_t*)&fw_sign_flag1, (uint16_t)sizeof(fw_sign_flag1));
        if (((fw_sign_flag0 == DFU_FW_ENC_OR_SIGN_PATTERN)) && (fw_sign_flag1 == DFU_FW_SIGN_PATTERN))
        {
            bin_size += DFU_SIGN_LEN;
            tail_size += DFU_SIGN_LEN;
            data[0] = SIGN_FW_TYPE;
        }
        else
        {
            data[0] = NORMAL_FW_TYPE;
        }
    }

    // the new FW cannot overlap with app bootloder
    uint32_t bootloader_end = s_boot_info.load_addr + s_boot_info.bin_size + tail_size;
    if (s_dfu_save_addr <= (bootloader_end))
    {
        dfu_m_event_handler(DFU_FW_SAVE_ADDR_CONFLICT, 0);
        return;
    }

#if DFU_BANK_MODE == DFU_MODE_COPY_UPGRADE
    // the new FW cannot overlap with bank0 FW
    uint32_t bank0_fw_end = s_app_info.boot_info.load_addr + s_app_info.boot_info.bin_size + tail_size;
    if (s_dfu_save_addr <= bank0_fw_end)
    {
        dfu_m_event_handler(DFU_FW_SAVE_ADDR_CONFLICT, 0);
        return;
    }
#endif
    s_file_size = bin_size;
#if DFU_BLE_ENABLE
    data[0] |= s_fast_dfu_mode;
#endif
    memcpy(&data[1], &s_now_img_info, sizeof(s_now_img_info));
    dfu_m_send_frame(data, DFU_CMD_PRO_START_DATA_LEN, PROGRAM_START);
}

static void dfu_m_get_info(void)
{
    DFU_MASTER_LOG(__FUNCTION__);
    dfu_m_send_frame(NULL, 0, GET_INFO);
}

static void dfu_m_dfu_mode_set(uint8_t dfu_mode)
{
    DFU_MASTER_LOG(__FUNCTION__);
    dfu_m_send_frame(&dfu_mode, DFU_CMD_MODE_SET_DATA_LEN, DFU_MODE_SET);
}

static void dfu_m_dfu_fw_info_get(void)
{
    DFU_MASTER_LOG(__FUNCTION__);
    dfu_m_send_frame(NULL, 0, DFU_FW_INFO_GET);
}

static void dfu_m_system_info_get(void)
{
    DFU_MASTER_LOG(__FUNCTION__);
    uint8_t data[DFU_CMD_GET_SYSTEM_INFO_DATA_LEN];
    // read
    data[0] = 0x00;
    // address
    uint32_t addr = DFU_SLAVE_FLASH_START_ADDR;
    dfu_htole32(&data[1], addr);
    //length
    data[DFU_CMD_GET_SYSTEM_INFO_LEN_L_POS] = DFU_IMAGE_INFO_LEN;
    data[DFU_CMD_GET_SYSTEM_INFO_LEN_H_POS] = 0U;

    dfu_m_send_frame(data, DFU_CMD_GET_SYSTEM_INFO_DATA_LEN, SYSTEM_INFO);
}

static void dfu_m_program_end(void)
{
    DFU_MASTER_LOG(__FUNCTION__);
    uint8_t data[DFU_CMD_PRO_END_DATA_LEN];
    data[0] = (uint8_t)s_run_fw_flag;
    dfu_htole32(&data[1], s_all_check_sum);
    dfu_m_send_frame(data, DFU_CMD_PRO_END_DATA_LEN, PROGRAM_END);
}

/**
 *****************************************************************************************
 * @brief Function for getting security mode.
 * @return Result of security mode.
 *****************************************************************************************
 */
static bool dfu_m_get_sec_flag(void)
{
    return s_sec_flag;
}

/**
 *****************************************************************************************
 * @brief DFU timeout schedule. If no response is received for a long time, a timeout will occur.
 *****************************************************************************************
 */
static void dfu_timeout_schedule(void)
{
    if (!s_dfu_timeout_started)
    {
        return;
    }

    if ((dfu_m_get_time() - s_dfu_timeout_start_time) > DFU_ACK_WAIT_TIMEOUT)
    {
        s_dfu_timeout_started = false;
        dfu_m_event_handler(DFU_ACK_TIMEOUT, 0);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint32_t dfu_m_get_program_size(void)
{
    return s_programed_size;
}

#if DFU_ASYNC_TX_ENABLE
void dfu_m_send_data_cmpl_process(void)
{
    uint16_t remain = s_all_send_len - s_sent_len;

    if (remain >= s_once_size)
    {
        dfu_m_send_data(&s_dfu_tx_buf[s_sent_len], s_once_size);
        s_sent_len += s_once_size;
    }
    else if (remain > 0U)
    {
        dfu_m_send_data(&s_dfu_tx_buf[s_sent_len], remain);
        s_sent_len += remain;
    }
    else
    {
        // Nothing to do
    }
}

#if DFU_BLE_ENABLE
void dfu_m_fast_send_data_cmpl_process(void)
{
    s_ble_fast_send_cplt_flag = 1;
}
#endif /* DFU_BLE_ENABLE */

#endif /* DFU_ASYNC_TX_ENABLE */

void dfu_m_cmd_parse(const uint8_t* data, uint16_t len)
{
    uint16_t i = 0;

    if (!s_cmd_receive_flag)
    {
        for (i = 0; i < len; i++)
        {
            switch (s_parse_state)
            {
                case CHECK_FRAME_L_STATE:
                    s_receive_check_sum = 0;
                    if (data[i] == CMD_FRAME_HEADER_L)
                    {
                        s_parse_state = CHECK_FRAME_H_STATE;
                    }
                    break;

                case CHECK_FRAME_H_STATE:
                    if (data[i] == CMD_FRAME_HEADER_H)
                    {
                        s_parse_state = RECEIVE_CMD_TYPE_L_STATE;
                    }
                    else if (data[i] == CMD_FRAME_HEADER_L)
                    {
                        s_parse_state = CHECK_FRAME_H_STATE;
                    }
                    else
                    {
                        s_parse_state = CHECK_FRAME_L_STATE;
                    }
                    break;

                case RECEIVE_CMD_TYPE_L_STATE:
                    s_receive_frame.cmd_type = data[i];
                    s_receive_check_sum += data[i];
                    s_parse_state = RECEIVE_CMD_TYPE_H_STATE;
                    break;

                case RECEIVE_CMD_TYPE_H_STATE:
                    s_receive_frame.cmd_type |= ((uint16_t)data[i] << 8);
                    s_receive_check_sum += data[i];
                    s_parse_state = RECEIVE_LEN_L_STATE;
                    break;

                case RECEIVE_LEN_L_STATE:
                    s_receive_frame.data_len = data[i];
                    s_receive_check_sum += data[i];
                    s_parse_state = RECEIVE_LEN_H_STATE;
                    break;

                case RECEIVE_LEN_H_STATE:
                    s_receive_frame.data_len |= ((uint16_t)data[i] << 8);
                    s_receive_check_sum += data[i];
                    if (s_receive_frame.data_len == 0U)
                    {
                        s_parse_state = RECEIVE_CHECK_SUM_L_STATE;
                    }
                    else if (s_receive_frame.data_len >= DFU_RX_FRAME_MAX)
                    {
                        s_parse_state = CHECK_FRAME_L_STATE;
                    }
                    else
                    {
                        s_receive_data_count = 0;
                        s_parse_state = RECEIVE_DATA_STATE;
                    }
                    break;

                case RECEIVE_DATA_STATE:
                    s_receive_frame.data[s_receive_data_count] = data[i];
                    if (++s_receive_data_count == s_receive_frame.data_len)
                    {
                        s_parse_state = RECEIVE_CHECK_SUM_L_STATE;
                    }
                    break;

                case RECEIVE_CHECK_SUM_L_STATE:
                    s_receive_frame.check_sum = data[i];
                    s_parse_state = RECEIVE_CHECK_SUM_H_STATE;
                    break;

                case RECEIVE_CHECK_SUM_H_STATE:
                    s_receive_frame.check_sum |= ((uint16_t)data[i] << 8);
                    s_parse_state = CHECK_FRAME_L_STATE;
                    dfu_m_cmd_check();
                    break;

                default:
                    s_parse_state = CHECK_FRAME_L_STATE;
                    break;
            }
        }
    }
}

void dfu_m_init(dfu_m_func_cfg_t *dfu_m_func_cfg, uint16_t once_send_size)
{
#if DFU_ASYNC_TX_ENABLE
    if (once_send_size != 0U)
    {
        s_once_size = once_send_size;
    }
#endif
    if (dfu_m_func_cfg != NULL)
    {
        s_p_func_cfg = dfu_m_func_cfg;
    }
}

void dfu_m_start(void)
{
    dfu_m_get_info();
    s_dfu_timeout_started = true;
    s_dfu_timeout_start_time = dfu_m_get_time();
}

void dfu_m_parse_state_reset(void)
{
    s_parse_state = CHECK_FRAME_L_STATE;
    s_cmd_receive_flag   = false;
    s_receive_data_count = 0;
    s_receive_check_sum  = 0;
    s_dfu_timeout_started = false;
}

#if DFU_BLE_ENABLE
void dfu_m_fast_dfu_mode_set(uint8_t setting)
{
    s_fast_dfu_mode = setting;
}

uint8_t dfu_m_fast_dfu_mode_get(void)
{
    return s_fast_dfu_mode;
}
#endif

void dfu_m_schedule(dfu_m_rev_cmd_cb_t rev_cmd_cb)
{
    uint8_t progress = 0;
#if DFU_BLE_ENABLE
    uint16_t erased_sectors = 0;
#endif
    if (s_cmd_receive_flag)
    {
        if (NULL != rev_cmd_cb)
        {
            rev_cmd_cb();
        }

        if (s_dfu_timeout_started)
        {
            s_dfu_timeout_start_time = dfu_m_get_time();
        }

        switch (s_receive_frame.cmd_type)
        {
            case GET_INFO:
                s_dfu_save_addr = 0;
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    if (s_receive_frame.data[DFU_RSP_DFU_VERSION_POS] >= DFU_VERSION)
                    {
                        s_new_version_flag = true; // new dfu version
                    }
                    else
                    {
                        s_new_version_flag = false; // old dfu version
                    }
                    dfu_m_system_info_get();
                }
                else
                {
                    dfu_m_event_handler(GET_INFO_FAIL, 0);
                }
                break;

            case SYSTEM_INFO:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    // security mode
                    if (0U != (s_receive_frame.data[DFU_RSP_SYS_INFO_OP_POS] & 0xF0U))
                    {
                        s_sec_flag = true;
                    }
                    else
                    {
                        s_sec_flag = false;
                    }
                    memcpy(&s_boot_info, &s_receive_frame.data[DFU_RSP_SYS_INFO_DATA_POS], sizeof(boot_info_t));
                    if (s_new_version_flag)
                    {
                        dfu_m_dfu_fw_info_get();
                    }
                }
                break;

            case DFU_FW_INFO_GET:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                #if DFU_GET_FW_SAVE_ADDR_METHOD == GET_SAVE_ADDR_BY_SLAVE
                    s_dfu_save_addr = dfu_le32toh(&s_receive_frame.data[1]);
                #endif

                #if DFU_BANK_MODE == DFU_MODE_COPY_UPGRADE
                    memcpy(&s_app_info, &s_receive_frame.data[DFU_RSP_IMG_INFO_POS], sizeof(dfu_img_info_t));
                #endif

                    dfu_m_dfu_mode_set(DFU_BANK_MODE);

                    // The command DFU_MODE_SET has no response. Wait at least 100ms before sending the next command.
                    uint32_t wait_time = 100U;
                    if (s_receive_frame.data[DFU_RSP_RUN_POSITION_POS])
                    {
                    // In DFU_MODE_NON_COPY_UPGRADE mode, if slave run in APP, slave need reset and run app bootloder
                    #if DFU_BANK_MODE == DFU_MODE_NON_COPY_UPGRADE
                        wait_time = DFU_SLAVE_RESET_TIME;
                    #endif
                    }
                    uint32_t start_time = dfu_m_get_time();
                    while ((dfu_m_get_time()- start_time) < wait_time)
                    {
                        // Wait for slave to be ready
                    }
                    dfu_m_program_start(dfu_m_get_sec_flag(), true); // true: run new fw after DFU
                }
                else
                {
                    // error
                    DFU_MASTER_LOG("DFU_FW_INFO_GET ERROR");
                }
                break;

            case PROGRAM_START:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                #if !DFU_BLE_ENABLE
                    dfu_m_program_flash(DFU_ONCE_PROGRAM_LEN);
                    dfu_m_event_handler(PRO_START_SUCCESS, 0);
                #else
                    if (FAST_DFU_MODE_DISABLE == s_fast_dfu_mode)
                    {
                        dfu_m_program_flash(DFU_ONCE_PROGRAM_LEN);
                        dfu_m_event_handler(PRO_START_SUCCESS, 0);
                    }
                    else if (FAST_DFU_MODE_ENABLE == s_fast_dfu_mode)
                    {
                        switch (s_receive_frame.data[1])
                        {
                            case DFU_ERASE_START_SUCCESS:
                                s_erase_sectors  = (uint16_t)(s_receive_frame.data[DFU_RSP_ERASE_POS]);
                                s_erase_sectors |= (((uint16_t)s_receive_frame.data[DFU_RSP_ERASE_POS + 1] << 8) & 0xff00U);
                                dfu_m_event_handler(ERASE_START_SUCCESS, 0);
                                break;

                            case DFU_ERASE_SUCCESS:
                                erased_sectors = (uint16_t)(s_receive_frame.data[DFU_RSP_ERASE_POS]);
                                erased_sectors |= (((uint16_t)s_receive_frame.data[DFU_RSP_ERASE_POS + 1] << 8) & 0xff00U);
                                progress = (uint8_t)((erased_sectors * 100U) / s_erase_sectors);
                                dfu_m_event_handler(ERASE_SUCCESS, progress);
                                break;

                            case DFU_ERASE_END_SUCCESS:
                                dfu_m_event_handler(ERASE_END_SUCCESS, 0);
                                dfu_m_fast_program_flash();
                                break;

                            case DFU_ERASE_REGION_NOT_ALIGNED:
                                dfu_m_event_handler(ERASE_REGION_NOT_ALIGNED, 0);
                                break;

                            case DFU_ERASE_REGIONS_OVERLAP:
                                dfu_m_event_handler(ERASE_REGION_OVERLAP, 0);
                                break;

                            case DFU_ERASE_FAIL:
                                dfu_m_event_handler(ERASE_FLASH_FAIL, 0);
                                break;

                            case DFU_ERASE_REGIONS_NOT_EXIST:
                                dfu_m_event_handler(ERASE_REGION_NOT_EXIST, 0);
                                break;

                            default:
                                // Nothing to do
                                break;
                        }
                    }
                    else
                    {
                        // Nothing to do
                    }
                #endif /* DFU_BLE_ENABLE */
                }
                else
                {
                    dfu_m_event_handler(PRO_START_ERROR, 0);
                }
                break;

            case PROGRAM_FLASH:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    progress = (uint8_t)((s_programed_size * 100U) / s_file_size);
                    dfu_m_event_handler(PRO_FLASH_SUCCESS, progress);
                    if (s_programed_size == s_file_size)
                    {
                        dfu_m_program_end();
                    }
                    else if ((s_programed_size + DFU_ONCE_PROGRAM_LEN) > s_file_size)
                    {
                        dfu_m_program_flash((uint16_t)(s_file_size - s_programed_size));
                    }
                    else
                    {
                        dfu_m_program_flash(DFU_ONCE_PROGRAM_LEN);
                    }
                }
                else
                {
                    dfu_m_event_handler(PRO_FLASH_FAIL, progress);
                }
                break;

            case PROGRAM_END:
                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                #if DFU_BLE_ENABLE
                    if (s_fast_dfu_mode == FAST_DFU_MODE_ENABLE)
                    {
                        uint32_t check_sum = dfu_le32toh(&s_receive_frame.data[1]);
                        if (check_sum == s_all_check_sum)
                        {
                            s_dfu_timeout_started = false;
                            dfu_m_event_handler(PRO_END_SUCCESS, 0);
                        }
                        else
                        {
                            s_dfu_timeout_started = false;
                            dfu_m_event_handler(PRO_END_FAIL, 0);
                        }
                    }
                    else if (s_fast_dfu_mode == FAST_DFU_MODE_DISABLE)
                    {
                        s_dfu_timeout_started = false;
                        dfu_m_event_handler(PRO_END_SUCCESS, 0);
                    }
                    else
                    {
                        // Nothing to do
                    }
                #else
                    s_dfu_timeout_started = false;
                    dfu_m_event_handler(PRO_END_SUCCESS, 0);
                #endif /* DFU_BLE_ENABLE */
                }
                else
                {
                    dfu_m_event_handler(PRO_END_FAIL, 0);
                }
                break;
            #if DFU_BLE_ENABLE
            case FAST_DFU_FLASH_SUCCESS:

                if (s_receive_frame.data[0] == ACK_SUCCESS)
                {
                    s_receive_frame.data[0] = (uint8_t)s_run_fw_flag;
                    dfu_htole32(&s_receive_frame.data[1], s_all_check_sum);
                    dfu_m_send_frame(s_receive_frame.data, DFU_CMD_PRO_END_DATA_LEN, PROGRAM_END);
                }
                else
                {
                    dfu_m_event_handler(FAST_DFU_FLASH_FAIL, 0);
                }
                break;
           #endif

            default:
                // Nothing to do
                break;
        }

        s_cmd_receive_flag = false;
    }

    dfu_timeout_schedule();
}
