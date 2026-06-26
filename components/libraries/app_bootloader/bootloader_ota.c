/**
 *****************************************************************************************
 *
 * @file bootloader_ota.c
 *
 * @brief Bootloader OTA Function Implementation.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "bootloader_config.h"
#include "user_periph_setup.h"
#if BOOTLOADER_DFU_BLE_ENABLE
#include "gr_includes.h"

#include "app_log.h"
#include "otas.h"
#include "dfu_port.h"

#include "bootloader_boot.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DEVICE_NAME                     "Bootloader_OTA"
#define APP_ADV_MIN_INTERVAL            32
#define APP_ADV_MAX_INTERVAL            32
#define NVDS_TAG_BD_ADDRESS_BACKUP      0xC0F1

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static ble_gap_bdaddr_t         s_bd_addr;
static bool                     s_bd_addr_changed_flag = false;

static ble_gap_adv_param_t      s_gap_adv_param;                     /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;                /**< Advertising time parameter. */


static const uint8_t s_adv_data_set[] =                          /**< Advertising data. */
{
    // Service UUIDs
    0x11,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    BLE_UUID_OTA_SERVICE,

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix Company ID:04F7
    0xF7,
    0x04,
    0x02,
    0x03,
};

#if OTAS_VERSION == 0x03
static uint8_t s_adv_rsp_data_set[] =                       /**< Scan responce data. */
{
    // length of this data
    0x16,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'B', 'o', 'o', 't', 'l', 'o', 'a', 'd', 'e', 'r','_', 'O', 'T', 'A','_', '0', '0', '0', '0', '0', '0'
};
#else
static const uint8_t s_adv_rsp_data_set[] =                       /**< Scan responce data. */
{
    // length of this data
    0x0f,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'B', 'o', 'o', 't', 'l', 'o', 'a', 'd', 'e', 'r','_', 'O', 'T', 'A'
};
#endif
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void bootloader_gap_params_init(void)
{
    ble_gap_pair_enable(false);
    ble_sec_params_set(NULL);

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;


    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
#if OTAS_VERSION == 0x03
    if (bootloader_dfu_info_get()->dfu_mode_pattern == DFU_NON_COPY_UPGRADE_MODE_PATTERN )
    {
        memcpy(s_adv_rsp_data_set + sizeof(s_adv_rsp_data_set) - DFU_ADV_NAME_RANDOM_DATA_SIZE, bootloader_dfu_info_get()->dfu_adv_name_random_data, DFU_ADV_NAME_RANDOM_DATA_SIZE);
    }
#endif
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));

    s_gap_adv_time_param.duration = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    ble_gap_data_length_set(251, 2120);
#if !defined(SOC_SGP)
    ble_gatt_mtu_set(247);
#else
    //Default MTU size is 247
    //ble_gap_l2cap_params_set(247, 247, 5);
#endif
    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}

static void bootloader_ota_init(void)
{
    dfu_service_init(NULL);
}

static void bootloader_address_backup_error_handler(void)
{
    nvds_put(NVDS_TAG_BD_ADDRESS_BACKUP, SYS_BD_ADDR_LEN, s_bd_addr.gap_addr.addr);
    s_bd_addr.gap_addr.addr[0] += 1;
    ble_gap_addr_set(&s_bd_addr);
    s_bd_addr_changed_flag = true;
}

static void bootloader_address_change(void)
{
    ble_gap_addr_get(&s_bd_addr);

    if (s_bd_addr.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)
    {
        uint8_t temp_bd_addr[SYS_BD_ADDR_LEN];
        uint16_t addr_len = sizeof(temp_bd_addr);
        if (NVDS_SUCCESS != nvds_get(NVDS_TAG_BD_ADDRESS_BACKUP, &addr_len, temp_bd_addr)
           || (0 != memcmp(&temp_bd_addr[1], &s_bd_addr.gap_addr.addr[1], SYS_BD_ADDR_LEN - 1)))
        {
            bootloader_address_backup_error_handler();
        }
        else
        {
            if ((temp_bd_addr[0] + 1) == s_bd_addr.gap_addr.addr[0])
            {
                // Have right backup
                s_bd_addr_changed_flag = true;
            }
            else if (temp_bd_addr[0] == s_bd_addr.gap_addr.addr[0])
            {
                // Chip reset before nvds_del(NVDS_TAG_BD_ADDRESS_BACKUP)
                s_bd_addr.gap_addr.addr[0] += 1;
                ble_gap_addr_set(&s_bd_addr);
                s_bd_addr_changed_flag = true;
            }
            else
            {
                bootloader_address_backup_error_handler();
            }
        }
    }
    else
    {
        s_bd_addr.gap_addr.addr[0] += 1;
        ble_gap_addr_set(&s_bd_addr);
    }
}

static void bootloader_address_restore(void)
{
    if (s_bd_addr_changed_flag)
    {
        s_bd_addr.gap_addr.addr[0] -= 1;
        SYS_SET_BD_ADDR(s_bd_addr.gap_addr.addr);
        nvds_del(NVDS_TAG_BD_ADDRESS_BACKUP);
        s_bd_addr_changed_flag = false;
    }
}

static void ble_evt_handler(const ble_evt_t *p_evt)
{
    bootloader_timeout_refresh();
    switch (p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
        {
            sdk_version_t     version;

            sys_sdk_verison_get(&version);
            APP_LOG_INFO(">>> Goodix BLE SDK V%d.%d.%d (commit %x)",
                        version.major, version.minor, version.build, version.commit_id);
            APP_LOG_DEBUG(">>> Enter bootloader OTA");
            bootloader_ota_init();
            bootloader_gap_params_init();
            bootloader_address_change();
            ble_gap_adv_start(0, &s_gap_adv_time_param);
            break;
        }
        case BLE_GAPC_EVT_CONNECTED:
            bootloader_address_restore();
            dfu_mode_update(DFU_NON_COPY_UPGRADE_MODE_PATTERN);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            APP_LOG_DEBUG(">>> Disconnected and restart adv.");
            ble_gap_adv_start(0, &s_gap_adv_time_param);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                dfu_ble_set_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
            }
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void bootloader_ota_task(void)
{
    ble_stack_init(ble_evt_handler, &heaps_table);
}
#endif

