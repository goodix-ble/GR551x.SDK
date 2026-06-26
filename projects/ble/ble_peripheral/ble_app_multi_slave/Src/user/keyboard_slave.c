/**
 *****************************************************************************************
 *
 * @file keyboard_slave.c
 *
 * @brief.
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
#include "keyboard_slave.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

#include "hids.h"

#include "utility.h"


/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_ADV_INTERVAL                    32
#define BASE_USB_HID_SPEC_VERSION           0x0101

#define INPUT_REPORT_COUNT                  1
#define KEYS_REPORT_KEYS_INDEX              0

#define KEYS_REP_REF_ID                     1
#define OUTPUT_REP_REF_ID                   1
#define FEATURE_REP_REF_ID                  1

#define OUTPUT_REPORT_MAX_LEN               1
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK    0x02
#define FEATURE_REPORT_MAX_LEN              2
#define FEATURE_REPORT_INDEX                0
#define INPUT_REPORT_KEYS_MAX_LEN           8

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_hid_slave_adv_param;
static ble_gap_adv_time_param_t s_hid_slave_adv_time_param;
static uint8_t                  s_hid_slave_addr[6] = {0x22, 0x22, 0x22, 0x22, 0x22, 0xc0};
static uint8_t                  s_hid_slave_adv_idx = 0x01;
static bool                     s_hid_slave_adv_cplt;
static uint8_t                  s_hid_slave_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
static uint8_t                  s_hid_slave_adv_data[] = {
    // Complete Name
    0x0a,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'H', 'I', 'D', '_', 'S', 'L', 'A', 'V', 'E',

    // Device appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),

    // Device Services uuid
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
};

static const uint8_t s_hid_slave_rep_map_data[] =
{
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       // Report Id 1
    0x05, 0x07,       // Usage Page (Key Codes)
    0x19, 0xe0,       // Usage Minimum (224)
    0x29, 0xe7,       // Usage Maximum (231)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x08,       // Report Count (8)
    0x81, 0x02,       // Input (Data, Variable, Absolute)

    0x95, 0x01,       // Report Count (1)
    0x75, 0x08,       // Report Size (8)
    0x81, 0x01,       // Input (Constant) reserved byte(1)

    0x95, 0x05,       // Report Count (5)
    0x75, 0x01,       // Report Size (1)
    0x05, 0x08,       // Usage Page (Page# for LEDs)
    0x19, 0x01,       // Usage Minimum (1)
    0x29, 0x05,       // Usage Maximum (5)
    0x91, 0x02,       // Output (Data, Variable, Absolute), Led report
    0x95, 0x01,       // Report Count (1)
    0x75, 0x03,       // Report Size (3)
    0x91, 0x01,       // Output (Data, Variable, Absolute), Led report padding

    0x95, 0x06,       // Report Count (6)
    0x75, 0x08,       // Report Size (8)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x65,       // Logical Maximum (101)
    0x05, 0x07,       // Usage Page (Key codes)
    0x19, 0x00,       // Usage Minimum (0)
    0x29, 0x65,       // Usage Maximum (101)
    0x81, 0x00,       // Input (Data, Array) Key array(6 bytes)

    0x09, 0x05,       // Usage (Vendor Defined)
    0x15, 0x00,       // Logical Minimum (0)
    0x26, 0xFF, 0x00, // Logical Maximum (255)
    0x75, 0x08,       // Report Size (8 bit)
    0x95, 0x02,       // Report Count (2)
    0xB1, 0x02,       // Feature (Data, Variable, Absolute)
    0xC0,             // End Collection (Application)
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void hid_service_event_handler(hids_evt_t *p_evt) {

}

static void adv_param_init(void) {
    sdk_err_t ret;

    s_hid_slave_adv_param.adv_intv_max = APP_ADV_INTERVAL;
    s_hid_slave_adv_param.adv_intv_min = APP_ADV_INTERVAL;
    s_hid_slave_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_hid_slave_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_hid_slave_adv_param.disc_mode    = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;
    s_hid_slave_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_hid_slave_adv_time_param.max_adv_evt = 0;
    s_hid_slave_adv_time_param.duration    = 0;

    ret = ble_gap_adv_data_set(s_hid_slave_adv_idx, BLE_GAP_ADV_DATA_TYPE_DATA, s_hid_slave_adv_data, sizeof(s_hid_slave_adv_data));
    APP_ERROR_CHECK(ret);

    ret = ble_gap_adv_param_set(s_hid_slave_adv_idx, BLE_GAP_OWN_ADDR_STATIC, &s_hid_slave_adv_param);
    APP_ERROR_CHECK(ret);
}

static void services_init(void) {
    sdk_err_t  ret;
    hids_init_t hids_init;
    uint8_t    hid_info_flags =  HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;
    hids_init.evt_handler = hid_service_event_handler;
    hids_init.is_kb = true;
    hids_init.is_mouse = false;

    hids_init.hid_info.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init.hid_info.b_country_code = 0;
    hids_init.hid_info.flags          = hid_info_flags;

    hids_init.report_map.p_map = (uint8_t*)&s_hid_slave_rep_map_data;
    hids_init.report_map.len   = sizeof(s_hid_slave_rep_map_data);

    hids_init.input_report_count = INPUT_REPORT_COUNT;
    hids_init.input_report_array[KEYS_REPORT_KEYS_INDEX].value_len       = INPUT_REPORT_KEYS_MAX_LEN;
    hids_init.input_report_array[KEYS_REPORT_KEYS_INDEX].ref.report_id   = KEYS_REP_REF_ID;
    hids_init.input_report_array[KEYS_REPORT_KEYS_INDEX].ref.report_type = HIDS_REP_TYPE_INPUT;

    hids_init.out_report_sup = true;
    hids_init.output_report.value_len       = OUTPUT_REPORT_MAX_LEN;
    hids_init.output_report.ref.report_id   = OUTPUT_REP_REF_ID;
    hids_init.output_report.ref.report_type = HIDS_REP_TYPE_OUTPUT;

    hids_init.feature_report_sup = true;
    hids_init.feature_report.value_len       = FEATURE_REPORT_MAX_LEN;
    hids_init.feature_report.ref.report_id   = FEATURE_REP_REF_ID;
    hids_init.feature_report.ref.report_type = HIDS_REP_TYPE_FEATURE;

    ret = hids_service_init(&hids_init);
    APP_ERROR_CHECK(ret);
}


static void adv_start(void) {
    sdk_err_t           ret;
    ble_gap_bdaddr_t    adv_addr;

    adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    memcpy(adv_addr.gap_addr.addr, s_hid_slave_addr, 6);

    ret = ble_gap_addr_set(&adv_addr);
    APP_ERROR_CHECK(ret);

    s_hid_slave_adv_cplt = false;
    ret = ble_gap_adv_start(s_hid_slave_adv_idx, &s_hid_slave_adv_time_param);
    APP_ERROR_CHECK(ret);

    if (!ret) {
        while(!s_hid_slave_adv_cplt) {
            extern void BLE_SDK_Handler_func(void);
            BLE_SDK_Handler_func();
            s_hid_slave_adv_cplt = true;
        }
    }
}

static void keyboard_slave_sec_gatt_param_change(uint8_t conn_idx) {
    sdk_err_t ret;

    ble_sec_param_t sec_param = {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ALL,
        .rkey_dist = BLE_SEC_KDIST_ALL,
    };

    ret = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(ret);

    ble_gatts_service_hide_all(conn_idx);
    ble_gatts_service_discoverable_set(conn_idx, hids_service_start_handle_get(), true);
}

void keyboard_slave_init(void) {
    adv_param_init();
    services_init();

    adv_start();
}

void keyboard_slave_ble_evt_handler(const ble_evt_t *p_evt) {
    switch(p_evt->evt_id) {
        case BLE_COMMON_EVT_STACK_INIT:
            keyboard_slave_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            if (p_evt->evt.gapm_evt.index == s_hid_slave_adv_idx) {
                APP_LOG_DEBUG("Keyboard Slave Adverting Start, status: %02x", p_evt->evt_status);
                s_hid_slave_adv_cplt = true;
            }
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.index == s_hid_slave_adv_idx) {
                APP_LOG_DEBUG("Keyboard Slave Adverting Stop, status: %02x, reason: %02x", p_evt->evt.gapm_evt.index, p_evt->evt_status, p_evt->evt.gapm_evt.params.adv_stop.reason);
            }
            break;

        case BLE_GAPC_EVT_CONNECTED: {
            extern void ble_gap_get_local_addr_by_conidx(uint8_t conidx,uint8_t *p_addr);
            uint8_t local_con_addr[6];

            ble_gap_get_local_addr_by_conidx(p_evt->evt.gapc_evt.index, local_con_addr);

            if(memcmp(s_hid_slave_addr, local_con_addr, 6) == 0) {
                s_hid_slave_conn_idx = p_evt->evt.gapc_evt.index;
                keyboard_slave_sec_gatt_param_change(s_hid_slave_conn_idx);
                APP_LOG_DEBUG("Keyboard Slave Connected, index: %02x", s_hid_slave_conn_idx);
            }
        }
        break;

        case BLE_GAPC_EVT_DISCONNECTED:
            if (s_hid_slave_conn_idx == p_evt->evt.gapc_evt.index) {
                s_hid_slave_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
                APP_LOG_DEBUG("Keyboard Slave Disconnected, index: %02x, reason: %02x", p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
                adv_start();
            }
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            if (s_hid_slave_conn_idx == p_evt->evt.sec_evt.index) {
                ble_sec_cfm_enc_t cfm_enc;

                memset((uint8_t *)&cfm_enc, 0, sizeof(cfm_enc));

                if (BLE_SEC_PAIR_REQ == p_evt->evt.sec_evt.params.enc_req.req_type) {
                        cfm_enc.req_type = BLE_SEC_PAIR_REQ;
                        cfm_enc.accept   = true;
                        ble_sec_enc_cfm(p_evt->evt.sec_evt.index, &cfm_enc);
                }
            }
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (s_hid_slave_conn_idx == p_evt->evt.sec_evt.index) {
                APP_LOG_DEBUG("Keyboard Slave Link has been encrypted, status: %02x", p_evt->evt_status);
            }
            break;

        case BLE_SEC_EVT_KEY_MISSING:
            if (s_hid_slave_conn_idx == p_evt->evt.sec_evt.index) {
                APP_LOG_DEBUG("Keyboard Slave Link Key Missing");
            }
            break;
    }
}

sdk_err_t keyboard_slave_keys_send_data(keyboard_keys_data_t *p_data)
{
    return hids_input_rep_send(s_hid_slave_conn_idx, KEYS_REPORT_KEYS_INDEX, (uint8_t*)p_data, INPUT_REPORT_KEYS_MAX_LEN);
}
