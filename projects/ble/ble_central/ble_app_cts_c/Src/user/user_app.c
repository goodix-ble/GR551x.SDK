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
#include "ble_scanner.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   160             /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     80              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   0               /**< Duration of the scanning(in units of 10 ms). */

#define APP_CONN_INTERVAL_MIN               6               /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               24              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

char *str_day_week[8] = {"Unknown_day",
                         "Monday",
                         "Tuesday",
                         "Wednesday",
                         "Thursday",
                         "Friday",
                         "Saturday",
                         "Sunday"
                         };
char *str_time_src[7] = {"Unknown",
                         "Network Time Protocol",
                         "GPS",
                         "Radio Time Signal",
                         "Manual",
                         "Atomic Clock",
                         "Cellular Network"
                        };

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const char       s_target_name[] = "Goodix_CTS";
static const uint8_t    s_target_addr[SYS_BD_ADDR_LEN] = {0x05, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void ble_scanner_evt_handler(ble_scanner_evt_t *p_evt)
{
    sdk_err_t    error_code;

    switch (p_evt->evt_type)
    {
        case BLE_SCANNER_EVT_CONNECTED:
            APP_LOG_INFO("Discover the service.");
            error_code = cts_c_disc_srvc_start(p_evt->param.conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case BLE_SCANNER_EVT_FILTER_MATCH:
            APP_LOG_INFO("Found target device.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
static void gap_scan_init(void)
{
    sdk_err_t                 error_code;
    ble_scanner_init_t        scan_init;
    ble_scanner_filter_data_t filter_data;

    memset(&scan_init, 0x00, sizeof(ble_scanner_init_t));
    memset(&filter_data, 0x00, sizeof(ble_scanner_filter_data_t));
    scan_init.scan_param.scan_type     = BLE_GAP_SCAN_ACTIVE;
    scan_init.scan_param.scan_mode     = BLE_GAP_SCAN_OBSERVER_MODE;
    scan_init.scan_param.scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    scan_init.scan_param.use_whitelist = false;
    scan_init.scan_param.interval      = APP_SCAN_INTERVAL;
    scan_init.scan_param.window        = APP_SCAN_WINDOW;
    scan_init.scan_param.timeout       = APP_SCAN_DURATION;

    scan_init.conn_param.type                = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    scan_init.conn_param.interval_min        = APP_CONN_INTERVAL_MIN;
    scan_init.conn_param.interval_max        = APP_CONN_INTERVAL_MAX;
    scan_init.conn_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    scan_init.conn_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;

    scan_init.connect_auto = true;

    scan_init.err_handler  = NULL;
    scan_init.evt_handler  = ble_scanner_evt_handler;

    filter_data.dev_name.length         = strlen(s_target_name);
    filter_data.dev_name.p_data         = (uint8_t *)s_target_name;
    filter_data.target_addr.addr_type   = BLE_GAP_ADDR_TYPE_PUBLIC;
    memcpy(filter_data.target_addr.gap_addr.addr, s_target_addr, SYS_BD_ADDR_LEN);

    ble_scanner_filter_set(BLE_SCANNER_ADDR_FILTER | BLE_SCANNER_NAME_FILTER, &filter_data);

    error_code = ble_scanner_init(&scan_init);
    APP_ERROR_CHECK(error_code);

    ble_scanner_filter_enable(BLE_SCANNER_FILTER_ALL_MATCH);

    error_code = ble_scanner_start();
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Process Current Time Service Client event.
 *****************************************************************************************
 */
static void cts_c_evt_process(cts_c_evt_t *p_evt)
{
    sdk_err_t error_code;
    switch (p_evt->evt_type)
    {
        case CTS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Current Time Service discovery completely.");
            error_code = dis_c_disc_srvc_start(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            cts_c_cur_time_notify_set(p_evt->conn_idx, true);
            break;

        case CTS_C_EVT_CUR_TIME_NTF_SET_SUCCESS:
            break;

        case CTS_C_EVT_VALID_CUR_TIME_REC:
            APP_LOG_INFO("Receive Local Current Time from peer. \r\n");
            APP_LOG_INFO("%d/%d/%d %s %d:%d:%d", p_evt->value.cur_time.day_date_time.date_time.year,
                                                 p_evt->value.cur_time.day_date_time.date_time.month,
                                                 p_evt->value.cur_time.day_date_time.date_time.day,
                                                 str_day_week[p_evt->value.cur_time.day_date_time.day_of_week],
                                                 p_evt->value.cur_time.day_date_time.date_time.hour,
                                                 p_evt->value.cur_time.day_date_time.date_time.min,
                                                 p_evt->value.cur_time.day_date_time.date_time.sec);
            APP_LOG_INFO("Fractions_256:%d Adjust_reason:%d \r\n", p_evt->value.cur_time.day_date_time.fractions_256,
                                                              p_evt->value.cur_time.adjust_reason);
            break;

        case CTS_C_EVT_VALID_LOC_TIME_INFO_REC:
            APP_LOG_INFO("Receive Local Time Information from peer. \r\n");
            APP_LOG_INFO("Time Zone:%d, DST offset:%d \r\n", p_evt->value.loc_time_info.time_zone,
                                                        p_evt->value.loc_time_info.dst_offset);
            break;

        case CTS_C_EVT_VALID_REF_TIME_INFO_REC:
            APP_LOG_INFO("Receive Reference Time Information from peer. \r\n");
            APP_LOG_INFO("Time Source: %s, Accuracy: %d, Days Since Update: %d, Hours Since Update: %d. \r\n",
                           str_time_src[p_evt->value.ref_time_info.source],
                           p_evt->value.ref_time_info.accuracy,
                           p_evt->value.ref_time_info.days_since_update,
                           p_evt->value.ref_time_info.hours_since_update);
            break;

        default:
            break;
    }
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
            cert_data_print(p_char_read_rsp->encode_rst.cert_list.p_list, p_char_read_rsp->encode_rst.cert_list.list_length);
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
            APP_LOG_INFO("Device Information Service discovery completely.");
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
 * @brief Deal device disconnect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] disconnect_reason: disconnect reason.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    sdk_err_t    error_code;

    error_code = ble_scanner_start();
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    if (p_evt->evt_status)
    {
        return;
    }
    ble_scanner_evt_on_ble_capture(p_evt);

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
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            break;

        case BLE_GAPC_EVT_CONNECTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Connected.");
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
    sdk_err_t     error_code;
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t version;

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
    APP_LOG_INFO("Current Time Service Client example started.");

    error_code = dis_client_init(dis_c_evt_process);
    APP_ERROR_CHECK(error_code);

    error_code = cts_client_init(cts_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_scan_init();

}

