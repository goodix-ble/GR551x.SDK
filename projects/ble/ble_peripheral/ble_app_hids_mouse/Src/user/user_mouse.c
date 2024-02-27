/**
 *****************************************************************************************
 *
 * @file user_mouse.c
 *
 * @brief The implementation of BLE Mouse functions.
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
#include "user_mouse.h"
#include "bas.h"
#include "dis.h"
#include "hids.h"
#include "app_log.h"
#include "app_error.h"
#include "utility.h"

/*
 * DEFINES
 *******************************************************************************
 */
#ifndef PTS_AUTO_TEST
#define INPUT_REPORT_COUNT              2           /**< Number of input reports in this application. */
#else
#define INPUT_REPORT_COUNT              1
#endif

#define INPUT_REP_MOUSE_LEN             4           /**< Length of Mouse Input Report data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1           /**< Length of Mouse Input Report containing media player data. */

#define INPUT_REP_MOUSE_INDEX           0           /**< Index of Mouse Input Report data. */
#define INPUT_REP_MPLAYER_INDEX         1           /**< Index of Mouse Input Report containing media player data. */

#define INPUT_REP_REF_MOUSE_ID          1           /**< Id of reference to Mouse Input Report data. */
#define INPUT_REP_REF_MPLAYER_ID        2           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101      /**< Version number of base USB HID Specification implemented by this application. */


/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */
static bool s_in_boot_mode = false;                                    /**< Current protocol mode. */
static bool s_in_notify_enabled = false;

static const uint8_t rep_map_data[] =
{
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)

    0xA1, 0x01, // Collection (Application)

    // Report ID 1:   Mouse button + motion
    0x85, 0x01,       // Report Id 1
    0x09, 0x01,       // Usage (Pointer)
    0xA1, 0x00,       // Collection (Physical)
    0x05, 0x09,       // Usage Page (Buttons)
    0x19, 0x01,       // Usage Minimum (01) - Button 1
    0x29, 0x03,       // Usage Maximum (03) - Button 3
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x02,       // Input (Data, Variable, Absolute) - Button states
    0x75, 0x05,       // Report Size (5)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x01,       // Input (Constant) - Padding or Reserved bits
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x30,       // Usage (X)
    0x09, 0x31,       // Usage (Y)
    0x09, 0x38,       // Usage (Wheel)
    0x15, 0x81,       // Logical Minimum (-127)
    0x25, 0x7F,       // Logical Maximum (127)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x06,       // Input (Data, Variable, Relative) - X & Y coordinate
    0xC0,             // End Collection (Physical)
    0xC0,             // End Collection (Application)
#ifndef PTS_AUTO_TEST
    // Report ID 2: Advanced buttons
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       // Report Id (2)
    0x15, 0x00,       // Logical minimum (0)
    0x25, 0x01,       // Logical maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x01,       // Report Count (1)

    0x09, 0xCD,       // Usage (Play/Pause)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB5,       // Usage (Scan Next Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB6,       // Usage (Scan Previous Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

    0x09, 0xEA,       // Usage (Volume Down)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xE9,       // Usage (Volume Up)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x25, 0x02, // Usage (AC Forward)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x24, 0x02, // Usage (AC Back)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0xC0              // End Collection
#endif
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
            s_in_boot_mode = true;
            break;

        case HIDS_EVT_REPORT_MODE_ENTERED:
            s_in_boot_mode = false;
            break;

        case HIDS_EVT_IN_REP_NOTIFY_ENABLED:
            s_in_notify_enabled = true;
            break;

        case HIDS_EVT_IN_REP_NOTIFY_DISABLED:
            s_in_notify_enabled = false;
            break;

#ifdef PTS_AUTO_TEST
        case HIDS_EVT_HOST_SUSP:
            APP_LOG_DEBUG("HID Control Point CMD: Suspend");
            break;

        case HIDS_EVT_HOST_EXIT_SUSP:
            APP_LOG_DEBUG("HID Control Point CMD: Exit Suspend");
            break;
#endif

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
    uint8_t     hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK |
                                 HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    hids_init.evt_handler  = hid_service_event_process;
    hids_init.is_kb        = false;
    hids_init.is_mouse     = true;

    hids_init.hid_info.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init.hid_info.b_country_code = 0;
    hids_init.hid_info.flags          = hid_info_flags;

    hids_init.report_map.p_map = (uint8_t*)&rep_map_data;
    hids_init.report_map.len   = sizeof(rep_map_data);

    hids_init.input_report_count                                          = INPUT_REPORT_COUNT;
    hids_init.input_report_array[INPUT_REP_MOUSE_INDEX].value_len         = INPUT_REP_MOUSE_LEN;
    hids_init.input_report_array[INPUT_REP_MOUSE_INDEX].ref.report_id     = INPUT_REP_REF_MOUSE_ID;
    hids_init.input_report_array[INPUT_REP_MOUSE_INDEX].ref.report_type   = HIDS_REP_TYPE_INPUT;

#ifndef PTS_AUTO_TEST
    hids_init.input_report_array[INPUT_REP_MPLAYER_INDEX].value_len       = INPUT_REP_MEDIA_PLAYER_LEN;
    hids_init.input_report_array[INPUT_REP_MPLAYER_INDEX].ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    hids_init.input_report_array[INPUT_REP_MPLAYER_INDEX].ref.report_type = HIDS_REP_TYPE_INPUT;
#endif

    hids_init.out_report_sup = false;
    hids_init.feature_report_sup = false;

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
void user_mouse_service_init(void)
{
    dis_init();
    bas_init();
    hids_init();
}


sdk_err_t user_mouse_data_send(uint8_t conn_idx, mouse_data_t *p_data)
{
    sdk_err_t err_code = SDK_SUCCESS;

    if (s_in_notify_enabled)
    {
        uint8_t buffer[INPUT_REP_MOUSE_LEN];

        if (p_data == NULL)
        {
            return SDK_ERR_POINTER_NULL;
        }

        memset(buffer, 0, INPUT_REP_MOUSE_LEN);
        if (p_data->left_button_press)
        {
            buffer[0] |= 0x01;
        }
        if (p_data->right_button_press)
        {
            buffer[0] |= 0x02;
        }
        if (p_data->middle_button_press)
        {
            buffer[0] |= 0x04;
        }

        buffer[1] = p_data->x_delta;
        buffer[2] = p_data->y_delta;
        if (s_in_boot_mode)
        {
            err_code = hids_boot_mouse_in_rep_send(conn_idx, buffer, 3);
        }
        else
        {
            buffer[3] = p_data->wheel_delta;
            err_code = hids_input_rep_send(conn_idx, INPUT_REP_MOUSE_INDEX,
                                           buffer, INPUT_REP_MOUSE_LEN);
        }
    }

    return err_code;
}


sdk_err_t user_mouse_media_send(uint8_t conn_idx, media_data_t *p_data)
{
    sdk_err_t err_code;
    if (p_data == NULL)
    {
        return SDK_ERR_POINTER_NULL;
    }
    err_code = hids_input_rep_send(conn_idx, INPUT_REP_MPLAYER_INDEX,
                                   (uint8_t*)p_data, INPUT_REP_MEDIA_PLAYER_LEN);
    return err_code;
}


void user_mouse_clear_flags(void)
{
    s_in_boot_mode      = false;
}

