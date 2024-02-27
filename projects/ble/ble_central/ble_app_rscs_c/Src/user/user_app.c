/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "user_app.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL               160      /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                 80       /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION               1000     /**< Duration of the scanning(in units of 10 ms). */

#define APP_CONN_INTERVAL_MIN           6        /**< Minimum connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX           24       /**< Maximum connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY          0        /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT            400      /**< Connection supervisory timeout(in unit of 10 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_bdaddr_t         s_target_bdaddr;     /**< Target device address. */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t            error_code;
    ble_gap_scan_param_t gap_scan_param;

    gap_scan_param.scan_type     = BLE_GAP_SCAN_ACTIVE;
    gap_scan_param.scan_mode     = BLE_GAP_SCAN_OBSERVER_MODE;
    gap_scan_param.scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    gap_scan_param.use_whitelist = false;
    gap_scan_param.interval      = APP_SCAN_INTERVAL;
    gap_scan_param.window        = APP_SCAN_WINDOW;
    gap_scan_param.timeout       = APP_SCAN_DURATION;

    error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &gap_scan_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Find target device by RSCS UUID.
 *****************************************************************************************
 */
static bool rscs_srvc_uuid_find(const uint8_t *p_data, const uint16_t length)
{
    uint16_t current_pos = 0;

    if (NULL == p_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t filed_type      = 0;
        uint8_t data_length     = 0;
        uint8_t fragment_length = p_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        filed_type  = p_data[current_pos++];

        if (BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID == filed_type || BLE_GAP_AD_TYPE_MORE_16_BIT_UUID == filed_type)
        {
            uint8_t  counter_16_bit_uuid =  data_length / 2;
            uint16_t parsed_16_bit_uuid  = 0;

            for (uint8_t i = 0; i < counter_16_bit_uuid; i++)
            {
                parsed_16_bit_uuid = BUILD_U16(p_data[current_pos + (2 * i)], p_data[current_pos + (2 * i) + 1]);

                if (BLE_ATT_SVC_RUNNING_SPEED_CADENCE == parsed_16_bit_uuid)
                {
                    return true;
                }
            }

            return false;
        }

        current_pos += data_length;
    }

    return false;
}

/**
 *****************************************************************************************
 * @brief Print RSCS Client feature read response.
 *****************************************************************************************
 */
static void rscs_c_feat_read_rsp_print(uint16_t rsc_feature)
{
    if (rsc_feature & RSCS_C_FEAT_INSTANT_STRIDE_LEN_BIT)
    {
        APP_LOG_INFO("Instantaneous Stride Length Measurement Supported");
    }
    else
    {
        APP_LOG_INFO("Instantaneous Stride Length Measurement  NOT Supported");
    }

    if (rsc_feature & RSCS_C_FEAT_TOTAL_DISTANCE_BIT)
    {
        APP_LOG_INFO("Total Distance Measurement Supported");
    }
    else
    {
        APP_LOG_INFO("Total Distance Measurement NOT Supported");
    }

    if (rsc_feature & RSCS_C_FEAT_RUNNING_OR_WALKING_STATUS_BIT)
    {
        APP_LOG_INFO("Running or Walking Status Supported");
    }
    else
    {
        APP_LOG_INFO("Running or Walking Status NOT Supported");
    }

    if (rsc_feature & RSCS_C_FEAT_CALIBRATION_PROCEDURE_BIT)
    {
        APP_LOG_INFO("Calibration Procedure Supported");
    }
    else
    {
        APP_LOG_INFO("Calibration Procedure NOT Supported");
    }

    if (rsc_feature & RSCS_C_FEAT_MULTIPLE_SENSORS_BIT)
    {
        APP_LOG_INFO("Multiple Sensor Locations Supported");
    }
    else
    {
        APP_LOG_INFO("Multiple Sensor Locations NOT Supported");
    }
}

/**
 *****************************************************************************************
 * @brief Print RSCS Client sensor location read response.
 *****************************************************************************************
 */
static void rscs_c_sensor_loc_read_rsp_print(rscs_c_sensor_loc_t sensor_loc)
{
    switch (sensor_loc)
    {
        case RSCS_C_SENSOR_LOC_OTHER:
            APP_LOG_INFO("Sensor location: Other");
            break;

        case RSCS_C_SENSOR_LOC_SHOE_TOP:
            APP_LOG_INFO("Sensor location: Top of shoe");
            break;

        case RSCS_C_SENSOR_LOC_SHOE_IN:
            APP_LOG_INFO("Sensor location: Inside of shoe");
            break;

        case RSCS_C_SENSOR_LOC_HIP:
            APP_LOG_INFO("Sensor location: Hip");
            break;

        case RSCS_C_SENSOR_LOC_FRONT_WHEEL:
            APP_LOG_INFO("Sensor location: Front wheel");
            break;

        case RSCS_C_SENSOR_LOC_LEFT_PEDAL:
            APP_LOG_INFO("Sensor location: Left pedal");
            break;

        case RSCS_C_SENSOR_LOC_RIGHT_PEDAL:
            APP_LOG_INFO("Sensor location: Right pedal");
            break;

        case RSCS_C_SENSOR_LOC_FRONT_HUB:
            APP_LOG_INFO("Sensor location: Front hub");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Print RSCS Client rsc measurement value.
 *****************************************************************************************
 */
static void rscs_c_rsc_meas_value_print(rscs_c_meas_val_t *p_rsc_meas_buff)
{
    if (p_rsc_meas_buff->is_run_or_walk)
    {
        APP_LOG_INFO("Ruuning");
    }
    else
    {
        APP_LOG_INFO("Walking");
    }

    APP_LOG_INFO("Speed:%0.2fm/s", (double)p_rsc_meas_buff->inst_speed / 256);

    APP_LOG_INFO("Cadence:%dRPM", p_rsc_meas_buff->inst_cadence);

    if (p_rsc_meas_buff->inst_stride_length_present)
    {
        APP_LOG_INFO("Instantaneous Stride Length:%0.2fm", (double)p_rsc_meas_buff->inst_stride_length / 100);
    }

    if (p_rsc_meas_buff->total_distance_present)
    {
        APP_LOG_INFO("Total Distance: %0.2fm\r\n", (double)p_rsc_meas_buff->total_distance / 10);
    }
}

/**
 *****************************************************************************************
 * @brief Process RSCS Client event.
 *****************************************************************************************
 */
static void rscs_c_evt_process(rscs_c_evt_t *p_evt)
{
    sdk_err_t error_code;

    switch (p_evt->evt_type)
    {
        case RSCS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Running Speed and Cadence Service discovery completely.");
            error_code = bas_c_disc_srvc_start(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_C_EVT_RSC_MEAS_NTF_SET_SUCCESS:
            APP_LOG_INFO("Running Speed and Cadence Measurment Notification had been set.");
            break;

        case RSCS_C_EVT_RSC_FEATURE_RECEIVE:
            rscs_c_feat_read_rsp_print(p_evt->value.rsc_feature);
            break;

        case RSCS_C_EVT_SENSOR_LOC_RECEIVE:
            rscs_c_sensor_loc_read_rsp_print(p_evt->value.rsc_sensor_loc);
            break;

        case RSCS_C_EVT_RSC_MEAS_VAL_RECEIVE:
            rscs_c_rsc_meas_value_print(&p_evt->value.rsc_meas_buff);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process BAS Client event.
 *****************************************************************************************
 */
static void bas_c_evt_process(bas_c_evt_t *p_evt)
{
    sdk_err_t error_code;

    switch (p_evt->evt_type)
    {
        case BAS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Battery Service discovery completely.");
            error_code = dis_c_disc_srvc_start(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case BAS_C_EVT_BAT_LEVEL_NTF_SET_SUCCESS:
            APP_LOG_INFO("Battery Level Notification had been set.");
            break;

        case BAS_C_EVT_BAT_LEVE_RECEIVE:
            APP_LOG_INFO("Battery Level: %d%%", p_evt->bat_level);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Print system id response.
 *****************************************************************************************
 */
static void system_id_print(dis_c_sys_id_t *p_sys_id)
{
    APP_LOG_INFO("System ID");
    APP_LOG_INFO("Manufacturer Identifier: 0X%02x%02x%02x%02x%02x",
                 p_sys_id->manufacturer_id[4],
                 p_sys_id->manufacturer_id[3],
                 p_sys_id->manufacturer_id[2],
                 p_sys_id->manufacturer_id[1],
                 p_sys_id->manufacturer_id[0]);

    APP_LOG_INFO("Organizationally Unique Identifier:0X%02x%02x%02x",
                 p_sys_id->org_unique_id[2],
                 p_sys_id->org_unique_id[1],
                 p_sys_id->org_unique_id[0]);
}

/**
 *****************************************************************************************
 * @brief Print certification data list response.
 *****************************************************************************************
 */
static void cert_data_print(uint8_t *p_data, uint16_t length)
{
    uint8_t str[DIS_C_STRING_LEN_MAX] = {0};

    APP_LOG_INFO("IEEE 11073-20601 CERT");

    switch (p_data[0])
    {
        case DIS_C_11073_BODY_EMPTY:
            APP_LOG_INFO("EMPT BODY");
            break;

        case DIS_C_11073_BODY_IEEE:
            APP_LOG_INFO("IEEE BODY");
            break;
        case DIS_C_11073_BODY_CONTINUA:
            APP_LOG_INFO("CONTINUA BODY");
            break;

        case DIS_C_11073_BODY_EXP:
            APP_LOG_INFO("EXP BODY");
            break;

        default:
            break;
    }
    memcpy(str, &p_data[2], length - 2);
    APP_LOG_INFO("Authoritative Body Structure:0X%02x", p_data[1]);
    APP_LOG_INFO("Authoritative Body Data:%s", str);
}

/**
 *****************************************************************************************
 * @brief Print pnp id response.
 *****************************************************************************************
 */
static void pnp_id_print(dis_c_pnp_id_t *p_pnp_id)
{
    APP_LOG_INFO("PnP ID");
    APP_LOG_INFO("Vendor ID Source:0X%02x", p_pnp_id->vendor_id_source);
    APP_LOG_INFO("Vendor ID:       0X%04x", p_pnp_id->vendor_id);
    APP_LOG_INFO("Product ID:      0X%04x", p_pnp_id->product_id);
    APP_LOG_INFO("Product Version: 0X%04x", p_pnp_id->product_version);
}

/**
 *****************************************************************************************
 * @brief Print DIS Client read response.
 *****************************************************************************************
 */
static void dis_c_read_rsp_print(ble_dis_c_read_rsp_t *p_char_read_rsp)
{
    uint8_t str[DIS_C_STRING_LEN_MAX] = {0};

    switch (p_char_read_rsp->char_type)
    {
        case DIS_C_SYS_ID:
            system_id_print(&p_char_read_rsp->encode_rst.sys_id);
            break;

        case DIS_C_MODEL_NUM:
            memcpy(str, p_char_read_rsp->encode_rst.string_data.p_data, p_char_read_rsp->encode_rst.string_data.length);
            APP_LOG_INFO("Model Number:      %s", str);
            break;

        case DIS_C_SERIAL_NUM:
            memcpy(str, p_char_read_rsp->encode_rst.string_data.p_data, p_char_read_rsp->encode_rst.string_data.length);
            APP_LOG_INFO("Serial Number:     %s", str);
            break;

        case DIS_C_HW_REV:
            memcpy(str, p_char_read_rsp->encode_rst.string_data.p_data, p_char_read_rsp->encode_rst.string_data.length);
            APP_LOG_INFO("Hardware Revision: %s", str);
            break;

        case DIS_C_FW_REV:
            memcpy(str, p_char_read_rsp->encode_rst.string_data.p_data, p_char_read_rsp->encode_rst.string_data.length);
            APP_LOG_INFO("Firmware Revision: %s", str);
            break;

        case DIS_C_SW_REV:
            memcpy(str, p_char_read_rsp->encode_rst.string_data.p_data, p_char_read_rsp->encode_rst.string_data.length);
            APP_LOG_INFO("Software Revision: %s", str);
            break;

        case DIS_C_MANUF_NAME:
            memcpy(str, p_char_read_rsp->encode_rst.string_data.p_data, p_char_read_rsp->encode_rst.string_data.length);
            APP_LOG_INFO("Manufacturer Name: %s", str);
            break;

        case DIS_C_CERT_LIST:
            cert_data_print(p_char_read_rsp->encode_rst.cert_list.p_list,
                            p_char_read_rsp->encode_rst.cert_list.list_length);
            break;

        case DIS_C_PNP_ID:
            pnp_id_print(&p_char_read_rsp->encode_rst.pnp_id);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process DIS Client event.
 *****************************************************************************************
 */
static void dis_c_evt_process(dis_c_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case DIS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_DEBUG("Device Information Service discovery completely.");
            break;

        case DIS_C_EVT_DEV_INFORMATION_READ_RSP:
            dis_c_read_rsp_print(&p_evt->read_rsp);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Deal receive advertising report task.
 *
 * @param[in] p_data:   Pointer to advertising report data.
 * @param[in] length:   Length of advertising report data.
 * @param[in] p_bdaddr: Pointer of broadcast address with broadcast type.
 *****************************************************************************************
 */
static void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t error_code;

    if (rscs_srvc_uuid_find(p_data, length))
    {
        memcpy(&s_target_bdaddr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
        error_code = ble_gap_scan_stop();
        APP_ERROR_CHECK(error_code);
    }
}

/**
 *****************************************************************************************
 * @brief Deal device stop scan task.
 *****************************************************************************************
 */
static void app_scan_stop_handler(void)
{
    sdk_err_t               error_code;
    ble_gap_init_param_t    gap_connect_param;

    gap_connect_param.type                = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    gap_connect_param.interval_min        = APP_CONN_INTERVAL_MIN;
    gap_connect_param.interval_max        = APP_CONN_INTERVAL_MAX;
    gap_connect_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    gap_connect_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    gap_connect_param.peer_addr.gap_addr  = s_target_bdaddr.gap_addr;
    gap_connect_param.peer_addr.addr_type = s_target_bdaddr.addr_type;
    gap_connect_param.conn_timeout        = 0;

    error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal device connect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] p_param:  Pointer of connection complete event data.
 *****************************************************************************************
 */
static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t    error_code;

    error_code = rscs_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal device disconnect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] disconnect_reason: disconnect reason.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    sdk_err_t    error_code;

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_SCAN_START:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                APP_LOG_DEBUG("Scan started failed(0X%02X).", p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            if (BLE_GAP_STOPPED_REASON_TIMEOUT == p_evt->evt.gapm_evt.params.scan_stop.reason)
            {
                APP_LOG_DEBUG("Scan Timeout.");
            }
            else
            {
                app_scan_stop_handler();
            }
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            app_adv_report_handler(p_evt->evt.gapm_evt.params.adv_report.data, p_evt->evt.gapm_evt.params.adv_report.length, &p_evt->evt.gapm_evt.params.adv_report.broadcaster_addr);
            break;

        case BLE_GAPC_EVT_CONNECTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Connected.");
                app_connected_handler(p_evt->evt.gapc_evt.index, &p_evt->evt.gapc_evt.params.connected);
            }
            break; 

        case BLE_GAPC_EVT_DISCONNECTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Disconnected (0x%02X).", p_evt->evt.gapc_evt.params.disconnected.reason);
                app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            }
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;
    }
}

void ble_app_init(void)
{
    sdk_err_t         error_code;
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
    APP_LOG_INFO("Running Speed and Cadence Service Client example started.");

    error_code = bas_client_init(bas_c_evt_process);
    APP_ERROR_CHECK(error_code);

    error_code = dis_client_init(dis_c_evt_process);
    APP_ERROR_CHECK(error_code);

    error_code = rscs_client_init(rscs_c_evt_process);
    APP_ERROR_CHECK(error_code);


    gap_params_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}
