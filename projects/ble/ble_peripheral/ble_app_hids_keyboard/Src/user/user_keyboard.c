/**
 *****************************************************************************************
 *
 * @file user_keyboard.c
 *
 * @brief The implementation of BLE keyboard functions.
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
#include "user_keyboard.h"
#include "bas.h"
#include "dis.h"
#include "hids.h"
#include "app_error.h"
#include "utility.h"

/*
 * DEFINES
 *******************************************************************************
 */
#define INPUT_REPORT_COUNT                  2            /**< Number of input reports in this application. */

#define KEYS_REPORT_KEYS_INDEX              0            /**< Index of key Report. */
#define MEDIA_REPORT_INDEX                  1            /**< Index of media report. */

#define KEYS_REP_REF_ID                     1            /**< Id of reference to keys Input Report. */
#define OUTPUT_REP_REF_ID                   1            /**< Id of reference to Keyboard Output Report. */
#define FEATURE_REP_REF_ID                  1            /**< ID of reference to Keyboard Feature Report. */
#define MEDIA_REP_REF_ID                    2            /**< Id of reference to media control Report. */

#define OUTPUT_REPORT_MAX_LEN               1            /**< Maximum length of Output Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK    0x02         /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */
#define FEATURE_REPORT_MAX_LEN              2            /**< Maximum length of Feature Report. */
#define FEATURE_REPORT_INDEX                0            /**< Index of Feature Report. */
#define INPUT_REPORT_KEYS_MAX_LEN           8            /**< Maximum length of the Keys Input Report . */
#define INPUT_REPORT_MEDIA_MAX_LEN          1            /**< Maximum length of the Media Input Report . */
#define BASE_USB_HID_SPEC_VERSION           0x0101        /**< Version number of base USB HID Specification implemented by this application. */


/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */
static bool              m_in_boot_mode = false;                                    /**< Current protocol mode. */

static const uint8_t rep_map_data[] =
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

    // Report ID 2: Advanced buttons
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       // Report Id (2)
    0x15, 0x00,       // Logical minimum (0)
    0x25, 0x01,       // Logical maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x01,       // Report Count (1)

    0x09, 0xEA,       // Usage (Volume Down)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xE9,       // Usage (Volume Up)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xE1,       // Usage (Volume Mute)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xCD,       // Usage (Play/Pause)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB5,       // Usage (Scan Next Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB6,       // Usage (Scan Previous Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x25, 0x02, // Usage (AC Forward)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x24, 0x02, // Usage (AC Back)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0xC0              // End Collection
};


static dis_pnp_id_t s_devinfo_pnp_id =
{
    .vendor_id_source = 1,      // Vendor ID source (1=Bluetooth SIG)
    .vendor_id        = 0x04F7, // Vendor ID
    .product_id       = 0x1234, // Product ID (vendor-specific)
    .product_version  = 0x0110  // Product version (JJ.M.N)
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 */

/**
 *****************************************************************************************
 * @brief Process HID Service events.
 *
 * @param[in] p_evt: Pointer of HID Service event.
 *****************************************************************************************
 */
static void hid_service_event_process(hids_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief HID Service init.
 *****************************************************************************************
 */
static void hids_init(void)
{
    hids_init_t hids_init;
    uint8_t    hid_info_flags =  HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;
    hids_init.evt_handler = hid_service_event_process;
    hids_init.is_kb = true;
    hids_init.is_mouse = false;

    hids_init.hid_info.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init.hid_info.b_country_code = 0;
    hids_init.hid_info.flags          = hid_info_flags;

    hids_init.report_map.p_map = (uint8_t*)&rep_map_data;
    hids_init.report_map.len   = sizeof(rep_map_data);

    hids_init.input_report_count = INPUT_REPORT_COUNT;
    hids_init.input_report_array[KEYS_REPORT_KEYS_INDEX].value_len       = INPUT_REPORT_KEYS_MAX_LEN;
    hids_init.input_report_array[KEYS_REPORT_KEYS_INDEX].ref.report_id   = KEYS_REP_REF_ID;
    hids_init.input_report_array[KEYS_REPORT_KEYS_INDEX].ref.report_type = HIDS_REP_TYPE_INPUT;

    hids_init.input_report_array[MEDIA_REPORT_INDEX].value_len       = INPUT_REPORT_MEDIA_MAX_LEN;
    hids_init.input_report_array[MEDIA_REPORT_INDEX].ref.report_id   = MEDIA_REP_REF_ID;
    hids_init.input_report_array[MEDIA_REPORT_INDEX].ref.report_type = HIDS_REP_TYPE_INPUT;

    hids_init.out_report_sup = true;
    hids_init.output_report.value_len       = OUTPUT_REPORT_MAX_LEN;
    hids_init.output_report.ref.report_id   = OUTPUT_REP_REF_ID;
    hids_init.output_report.ref.report_type = HIDS_REP_TYPE_OUTPUT;

    hids_init.feature_report_sup = true;
    hids_init.feature_report.value_len       = FEATURE_REPORT_MAX_LEN;
    hids_init.feature_report.ref.report_id   = FEATURE_REP_REF_ID;
    hids_init.feature_report.ref.report_type = HIDS_REP_TYPE_FEATURE;

    hids_service_init(&hids_init);
}

/**
 *****************************************************************************************
 *@brief Battery Service init.
 *****************************************************************************************
 */
static void bas_init(void)
{
    bas_init_t bas_env_init[1];
    sdk_err_t  error_code;

    bas_env_init[0].char_mask   = BAS_CHAR_MANDATORY | BAS_CHAR_LVL_NTF_SUP;
    bas_env_init[0].batt_lvl    = 100;
    bas_env_init[0].evt_handler = NULL;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Device Information Service init.
 *****************************************************************************************
 */
static void dis_init(void)
{
    dis_init_t dis_env_init;
    sdk_err_t  error_code;

    dis_env_init.char_mask                   = DIS_CHAR_PNP_ID_SUP;
    dis_env_init.p_pnp_id                    = &s_devinfo_pnp_id;
    error_code = dis_service_init(&dis_env_init);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void user_keyboard_service_init(void)
{
    dis_init();
    bas_init();
    hids_init();
}


sdk_err_t user_keyboard_keys_send_data(uint8_t conn_idx, keyboard_keys_data_t *p_data)
{
    sdk_err_t err_code;
    if (m_in_boot_mode)
    {
        err_code = hids_boot_kb_in_rep_send(conn_idx, (uint8_t*)p_data, INPUT_REPORT_KEYS_MAX_LEN);
    }
    else
    {
        err_code = hids_input_rep_send(conn_idx, KEYS_REPORT_KEYS_INDEX, (uint8_t*)p_data, INPUT_REPORT_KEYS_MAX_LEN);
    }
    return err_code;
}

sdk_err_t user_keyboard_media_send_data(uint8_t conn_idx, keyboard_media_data_t *p_data)
{
    return hids_input_rep_send(conn_idx, MEDIA_REPORT_INDEX, (uint8_t*)p_data, INPUT_REPORT_MEDIA_MAX_LEN);
}
