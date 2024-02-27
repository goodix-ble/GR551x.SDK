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
#include "hrs.h"
#include "hrs_c.h"
#include "rscs.h"
#include "rscs_c.h"
#include "hrrcps.h"
#include "gr_includes.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "ble_srv_disc_utils.h"


/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                    "Goodix_HRS_RSCS_RELAY"      /**< Name of device which will be included in the advertising data. */
#define APP_ADV_MIN_INTERVAL           32                           /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL           48                           /**< The advertising max interval (in units of 0.625 ms). */

#define APP_SCAN_INTERVAL              160                          /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                80                           /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_TIMEOUT               1000                         /**< Duration of the scanning(in units of 10 ms). */

#define APP_CONN_INTERVAL_MIN          100                          /**< Minimum connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX          100                          /**< Maximum connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY         0                            /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT           400                          /**< Connection supervisory timeout(in unit of 10 ms). */

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
enum
{
    USER_WR_NO_OPERATION,
    USER_WR_HRS_NTF_EN,
    USER_WR_HRS_NTF_DIS,
    USER_WR_RSCS_NTF_EN,
    USER_WR_RSCS_NTF_DIS,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_hrs_active_state   = NO_ACTIVE_STATE;           /**< HRS active state. */
uint8_t g_rscs_active_state  = NO_ACTIVE_STATE;           /**< RSCS active state. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                                    /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;                               /**< Advertising time parameter. */
static ble_gap_scan_param_t     s_scan_param;                                       /**< Scan parameter. */
static ble_gap_ext_init_param_t s_gap_connect_param;                                /**< Connect parameter. */
static ble_gap_bdaddr_t         s_hrs_target_addr;                                  /**< HRS target device address. */
static ble_gap_bdaddr_t         s_rscs_target_addr;                                 /**< RSCS target device address. */
static uint8_t                  s_conn_idx_collector   = BLE_GAP_INVALID_CONN_INDEX;    /**< Connection index for the Collector central application */
static uint8_t                  s_conn_idx_hrs_c       = BLE_GAP_INVALID_CONN_INDEX;    /**< Connection index for the HRS central application */
static uint8_t                  s_conn_idx_rscs_c      = BLE_GAP_INVALID_CONN_INDEX;    /**< Connection index for the RSC central application */
static uint8_t                  s_user_write_id        = USER_WR_NO_OPERATION;

static const uint8_t s_adv_data_set[] =                                         /**< Advertising data. */
{
    0x05,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE),
    HI_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE),
    LO_U16(BLE_ATT_SVC_HEART_RATE),
    HI_U16(BLE_ATT_SVC_HEART_RATE),

    0x11,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    HRRCPS_SERVICE_UUID,
};

static const uint8_t s_adv_rsp_data_set[] =                      /**< Scan responce data. */
{
    0x16,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'H', 'R', 'S', '_', 'R', 'S', 'C', 'S', '_', 'R', 'E', 'L', 'A', 'Y'
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(false);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Initialize advertising parameters.
 *****************************************************************************************
 */
static void adv_params_init(void)
{
    sdk_err_t   error_code;

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 * @brief Initialize scan parameters.
 *****************************************************************************************
 */
static void scan_params_init(void)
{
    sdk_err_t   error_code;

    s_scan_param.scan_type     = BLE_GAP_SCAN_ACTIVE;
    s_scan_param.scan_mode     = BLE_GAP_SCAN_OBSERVER_MODE;
    s_scan_param.scan_dup_filt = BLE_GAP_SCAN_FILT_DUPLIC_EN;
    s_scan_param.use_whitelist = false;
    s_scan_param.interval      = APP_SCAN_INTERVAL;
    s_scan_param.window        = APP_SCAN_WINDOW;
    s_scan_param.timeout       = APP_SCAN_TIMEOUT;

    error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &s_scan_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Initialize connect parameters.
 *****************************************************************************************
 */
static void conn_params_init(void)
{
    s_gap_connect_param.type          = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    s_gap_connect_param.prop          = BLE_GAP_INIT_PROP_1M_BIT;
    s_gap_connect_param.conn_to       = 0;
    s_gap_connect_param.scan_param_1m.scan_intv      = 15;
    s_gap_connect_param.scan_param_1m.scan_wd        = 15;
    s_gap_connect_param.conn_param_1m.conn_intv_min  = APP_CONN_INTERVAL_MIN;
    s_gap_connect_param.conn_param_1m.conn_intv_max  = APP_CONN_INTERVAL_MAX;
    s_gap_connect_param.conn_param_1m.conn_latency   = APP_CONN_SLAVE_LATENCY;
    s_gap_connect_param.conn_param_1m.supervision_to = APP_CONN_SUP_TIMEOUT;
    s_gap_connect_param.conn_param_1m.ce_len         = 10;
}

/**
 *****************************************************************************************
 * @brief Find target device by UUID.
 *****************************************************************************************
 */
static bool target_srvc_uuid_find(const uint8_t *p_adv_data, const uint16_t length, uint16_t target_uuid)
{
    uint16_t current_pos = 0;

    if (NULL == p_adv_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t filed_type      = 0;
        uint8_t data_length     = 0;
        uint8_t fragment_length = p_adv_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        filed_type  = p_adv_data[current_pos++];

        if (BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID == filed_type || BLE_GAP_AD_TYPE_MORE_16_BIT_UUID == filed_type)
        {
            uint8_t  counter_16_bit_uuid =  data_length / 2;
            uint16_t parsed_16_bit_uuid  = 0;

            for (uint8_t i = 0; i < counter_16_bit_uuid; i++)
            {
                parsed_16_bit_uuid = p_adv_data[current_pos + 2 * i] | p_adv_data[current_pos + 2 * i + 1] << 8;

                if (target_uuid == parsed_16_bit_uuid)
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
 * @brief Process Running Speed and Cadence Service event.
 *****************************************************************************************
 */
static void rsc_service_process_event(rscs_evt_t *p_rscs_evt)
{
    if (p_rscs_evt->conn_idx != s_conn_idx_collector)
    {
        return;
    }

    switch (p_rscs_evt->evt_type)
    {
        case RSCS_EVT_RSC_MEAS_NOTIFICATION_ENABLE:
            APP_LOG_DEBUG("RSC Measurement Notification is enabled.");
            break;

        case RSCS_EVT_RSC_MEAS_NOTIFICATION_DISABLE:
            APP_LOG_DEBUG("RSC Measurement Notification is enabled.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Process heart rate service event
 *****************************************************************************************
 */
static void hrs_service_process_event(hrs_evt_t *p_evt)
{
    if (p_evt->conn_idx != s_conn_idx_collector)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case HRS_EVT_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Heart Rate Measurement Notification is enabled.");
            break;

        case HRS_EVT_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Heart Rate Measurement Notification is disabled.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process HRS Client event.
 *****************************************************************************************
 */
static void hrs_c_evt_process(hrs_c_evt_t *p_evt)
{
    uint8_t           rr_intervals_idx = 0;
    hrrcps_rsp_val_t  rsp_val;
    sdk_err_t         error_code;

    switch (p_evt->evt_type)
    {
        case HRS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_DEBUG("Heart Rate Service discovery completely.");
            rsp_val.cmd_id = HRRCPS_CTRL_PT_SCAN_HRS;
            rsp_val.rsp_id = HRRCPS_RSP_ID_OK;
            rsp_val.is_inc_prama = false;
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);

            break;

        case HRS_C_EVT_DISCOVERY_FAIL:
            APP_LOG_DEBUG("Heart Rate Service discovery failed.");
            g_hrs_active_state = NO_ACTIVE_STATE;
            rsp_val.cmd_id = HRRCPS_CTRL_PT_SCAN_HRS;
            rsp_val.rsp_id = HRRCPS_RSP_ID_ERROR;
            rsp_val.is_inc_prama = false;
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            break;

        case HRS_C_EVT_HR_MEAS_NTF_SET_SUCCESS:
            if (USER_WR_HRS_NTF_EN == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_HRS_NTF_ENABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_OK;
                rsp_val.is_inc_prama = false;
                error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
                APP_ERROR_CHECK(error_code);
                s_user_write_id = USER_WR_NO_OPERATION;
            }
            else if (USER_WR_HRS_NTF_DIS == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_HRS_NTF_DISABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_OK;
                rsp_val.is_inc_prama = false;
                error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
                APP_ERROR_CHECK(error_code);
                s_user_write_id = USER_WR_NO_OPERATION;
            }
            break;

        case HRS_C_EVT_WRITE_OP_ERR:
            if (USER_WR_HRS_NTF_EN == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_HRS_NTF_ENABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_ERROR;
                rsp_val.is_inc_prama = false;
            }
            else if (USER_WR_HRS_NTF_DIS == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_HRS_NTF_DISABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_ERROR;
                rsp_val.is_inc_prama = false;
            }
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            s_user_write_id = USER_WR_NO_OPERATION;
            break;

        case HRS_C_EVT_HR_MEAS_VAL_RECEIVE:
            for (rr_intervals_idx = 0; rr_intervals_idx < p_evt->value.hr_meas_buff.rr_intervals_num; rr_intervals_idx++)
            {
                hrs_rr_interval_add(p_evt->value.hr_meas_buff.rr_intervals[rr_intervals_idx]);
            }
            hrs_sensor_contact_detected_update(p_evt->value.hr_meas_buff.is_sensor_contact_detected);

            hrs_heart_rate_measurement_send(s_conn_idx_collector,
                                            p_evt->value.hr_meas_buff.hr_value,
                                            p_evt->value.hr_meas_buff.energy_expended);
            break;

        case HRS_C_EVT_SENSOR_LOC_READ_RSP:
            APP_LOG_DEBUG("HRS sensor location is got.");
            hrs_sensor_location_set((hrs_sensor_loc_t)p_evt->value.sensor_loc);
            rsp_val.cmd_id       = HRRCPS_CTRL_PT_HRS_SEN_LOC_READ;
            rsp_val.rsp_id       = HRRCPS_RSP_ID_OK;
            rsp_val.is_inc_prama = true;
            rsp_val.rsp_param    = p_evt->value.sensor_loc;
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process RSCS Client event.
 *****************************************************************************************
 */
static void rscs_c_evt_process(rscs_c_evt_t *p_evt)
{
    hrrcps_rsp_val_t  rsp_val;
    sdk_err_t         error_code;

    switch (p_evt->evt_type)
    {
        case RSCS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_DEBUG("Running Speed and Cadence Service discovery completely.");
            rsp_val.cmd_id = HRRCPS_CTRL_PT_SCAN_RSCS;
            rsp_val.rsp_id = HRRCPS_RSP_ID_OK;
            rsp_val.is_inc_prama = false;
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_C_EVT_DISCOVERY_FAIL:
            APP_LOG_DEBUG("Running Speed and Cadence Service discovery failed.");
            g_rscs_active_state = NO_ACTIVE_STATE;
            rsp_val.cmd_id = HRRCPS_CTRL_PT_SCAN_RSCS;
            rsp_val.rsp_id = HRRCPS_RSP_ID_ERROR;
            rsp_val.is_inc_prama = false;
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_C_EVT_RSC_MEAS_NTF_SET_SUCCESS:
            if (USER_WR_RSCS_NTF_EN == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_RSCS_NTF_ENABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_OK;
                rsp_val.is_inc_prama = false;
                error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
                APP_ERROR_CHECK(error_code);
                s_user_write_id = USER_WR_NO_OPERATION;
            }
            else if (USER_WR_HRS_NTF_DIS == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_RSCS_NTF_DISABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_OK;
                rsp_val.is_inc_prama = false;
                error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
                APP_ERROR_CHECK(error_code);
                s_user_write_id = USER_WR_NO_OPERATION;
            }
            break;

        case RSCS_C_EVT_WRITE_OP_ERR:
            if (USER_WR_RSCS_NTF_EN == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_RSCS_NTF_ENABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_ERROR;
                rsp_val.is_inc_prama = false;
            }
            else if (USER_WR_HRS_NTF_DIS == s_user_write_id)
            {
                rsp_val.cmd_id = HRRCPS_CTRL_PT_RSCS_NTF_DISABLE;
                rsp_val.rsp_id = HRRCPS_RSP_ID_ERROR;
                rsp_val.is_inc_prama = false;
            }
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            s_user_write_id = USER_WR_NO_OPERATION;
            break;

        case RSCS_C_EVT_SENSOR_LOC_RECEIVE:
            APP_LOG_DEBUG("RSCS sensor location is got.");
            rscs_sensor_loc_update((rscs_sensor_loc_t)p_evt->value.rsc_sensor_loc);
            rsp_val.cmd_id       = HRRCPS_CTRL_PT_RSCS_SEN_LOC_READ;
            rsp_val.rsp_id       = HRRCPS_RSP_ID_OK;
            rsp_val.is_inc_prama = true;
            rsp_val.rsp_param    = p_evt->value.rsc_sensor_loc;
            error_code = hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_C_EVT_RSC_MEAS_VAL_RECEIVE:
            rscs_measurement_send(s_conn_idx_collector, (rscs_meas_val_t *)&p_evt->value.rsc_meas_buff);
            break;

        default:
            break;
    }
}


/**
 *****************************************************************************************
 * @brief Initialize service client that will be used by the application.
 *****************************************************************************************
 */
static void services_client_init(void)
{
    sdk_err_t   error_code;

    error_code = hrs_client_init(hrs_c_evt_process);
    APP_ERROR_CHECK(error_code);

    error_code = rscs_client_init(rscs_c_evt_process);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Deal HRRCPS operation error task
 *****************************************************************************************
 */
static void hrrcps_op_error_handler(hrrcps_ctrl_pt_id_t cmd_id)
{
    hrrcps_rsp_val_t  rsp_val;

    rsp_val.cmd_id       = cmd_id;
    rsp_val.rsp_id       = HRRCPS_RSP_ID_ERROR;
    rsp_val.is_inc_prama = false;

    hrrcps_ctrl_pt_rsp_send(s_conn_idx_collector, &rsp_val);
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

    if (SCAN_DEV_STATE == g_hrs_active_state)
    {
        if (target_srvc_uuid_find(p_data, length, BLE_ATT_SVC_HEART_RATE))
        {
            memcpy(&s_hrs_target_addr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
            APP_LOG_DEBUG("Stop scanning for Connecting HRS procedure.");
            g_hrs_active_state = CONN_UNDERWAY_STATE;
            return;
        }
    }

    if (SCAN_DEV_STATE == g_rscs_active_state)
    {
        if (target_srvc_uuid_find(p_data, length, BLE_ATT_SVC_RUNNING_SPEED_CADENCE))
        {
            memcpy(&s_rscs_target_addr, p_bdaddr, sizeof(ble_gap_bdaddr_t));
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
            APP_LOG_DEBUG("Stop scanning for Connecting RSCS procedure.");
            g_rscs_active_state = CONN_UNDERWAY_STATE;
            return;
        }
    }
}

/**
 *****************************************************************************************
 * @brief Deal device stop scan task.
 *****************************************************************************************
 */
static void app_scan_stop_handler(void)
{
    if (CONN_UNDERWAY_STATE == g_hrs_active_state)
    {
        s_gap_connect_param.peer_addr.addr_type = s_hrs_target_addr.addr_type;
        s_gap_connect_param.peer_addr.gap_addr  = s_hrs_target_addr.gap_addr;
        ble_gap_ext_connect(BLE_GAP_OWN_ADDR_STATIC, &s_gap_connect_param);
    }

    if (CONN_UNDERWAY_STATE == g_rscs_active_state)
    {
        s_gap_connect_param.peer_addr.addr_type = s_rscs_target_addr.addr_type;
        s_gap_connect_param.peer_addr.gap_addr  = s_rscs_target_addr.gap_addr;
        ble_gap_ext_connect(BLE_GAP_OWN_ADDR_STATIC, &s_gap_connect_param);
    }
}

/**
 *****************************************************************************************
 * @brief Deal device connect task.
 *
 * @param[in] conn_idx: index of connection.
 * @param[in] p_param:  Pointer of connection complete event data.
 *****************************************************************************************
 */
static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_conn_param)
{
    if (BLE_GAP_LL_ROLE_MASTER == p_conn_param->ll_role)
    {
        if (CONN_UNDERWAY_STATE == g_hrs_active_state)
        {
            APP_LOG_DEBUG("Connected to HRS, IDX:%d.", conn_idx);

            s_conn_idx_hrs_c   = conn_idx;
            g_hrs_active_state = CONNECTED_STATE;

            hrs_c_disc_srvc_start(s_conn_idx_hrs_c);
            ble_srv_disc_proc_state_set(HRS_DISC_PROC_ID, BLE_SRV_DISC_UNDERWAY);

            APP_LOG_DEBUG("Start discovery HRS service.");
        }

        if (CONN_UNDERWAY_STATE == g_rscs_active_state)
        {
            APP_LOG_DEBUG("Connected to RSCS, IDX:%d.", conn_idx);

            s_conn_idx_rscs_c   = conn_idx;
            g_rscs_active_state = CONNECTED_STATE;

            rscs_c_disc_srvc_start(s_conn_idx_rscs_c);;
            ble_srv_disc_proc_state_set(RSCS_DISC_PROC_ID, BLE_SRV_DISC_UNDERWAY);

            APP_LOG_DEBUG("Start discovery RSCS service.");
        }
    }

    if (BLE_GAP_LL_ROLE_SLAVE == p_conn_param->ll_role)
    {
        APP_LOG_DEBUG("Connected to Collector, IDX:%d.", conn_idx);
        s_conn_idx_collector = conn_idx;
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
    if (conn_idx == s_conn_idx_hrs_c)
    {
        APP_LOG_DEBUG("Disconnected from HRS(0x%02X)", disconnect_reason);

        s_conn_idx_hrs_c   = BLE_GAP_INVALID_CONN_INDEX;
        g_hrs_active_state = NO_ACTIVE_STATE;

        ble_srv_disc_proc_state_set(HRS_DISC_PROC_ID, BLE_SRV_DISC_NO_IMPLEMENT);
        hrrcps_op_error_handler(HRRCPS_CTRL_PT_HRS_CONN_STA_REPORT);
    }
    else if (conn_idx == s_conn_idx_rscs_c)
    {
        APP_LOG_DEBUG("Disconnected from RSCS(0x%02X)", disconnect_reason);

        s_conn_idx_rscs_c   = BLE_GAP_INVALID_CONN_INDEX;
        g_rscs_active_state = NO_ACTIVE_STATE;

        ble_srv_disc_proc_state_set(RSCS_DISC_PROC_ID, BLE_SRV_DISC_NO_IMPLEMENT);
        hrrcps_op_error_handler(HRRCPS_CTRL_PT_RSCS_CONN_STA_REPORT);
    }
    else if (conn_idx == s_conn_idx_collector)
    {
        APP_LOG_DEBUG("Disconnected from Collector(0x%02X)", disconnect_reason);

        s_conn_idx_collector = BLE_GAP_INVALID_CONN_INDEX;

        if (BLE_GAP_INVALID_CONN_INDEX != s_conn_idx_rscs_c)
        {
            ble_gap_disconnect(s_conn_idx_rscs_c);
        }

        if (BLE_GAP_INVALID_CONN_INDEX != s_conn_idx_hrs_c)
        {
            ble_gap_disconnect(s_conn_idx_hrs_c);
        }

        ble_gap_adv_start(0, &s_gap_adv_time_param);

        APP_LOG_DEBUG("Start advertising.");
    }
}

/**
 *****************************************************************************************
 * @brief Process HRRCPS event.
 *****************************************************************************************
 */
static void hrrcps_evt_process(hrrcps_evt_t *p_evt)
{
    sdk_err_t   error_code;

    if (p_evt->conn_idx == s_conn_idx_collector)
    {
        switch (p_evt->evt_type)
        {
            case HRRCPS_EVT_CTRL_PT_IND_ENABLE:
                APP_LOG_DEBUG("HRR Control Point Indication is enabled.");
                break;

            case HRRCPS_EVT_CTRL_PT_IND_DISABLE:
                APP_LOG_DEBUG("HRR Control Point Indication is disabled.");
                break;

            case HRRCPS_EVT_SCAN_HRS:
                if (NO_ACTIVE_STATE != g_hrs_active_state)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_HRS);
                }

                pwr_mgmt_ble_wakeup();
                error_code = ble_gap_scan_start();

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_HRS);
                }

                g_hrs_active_state = SCAN_DEV_STATE;

                APP_LOG_DEBUG("Start scanning, target device: HRS.");
                break;

            case HRRCPS_EVT_SCAN_RSCS:
                if (NO_ACTIVE_STATE != g_rscs_active_state)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_RSCS);
                }

                pwr_mgmt_ble_wakeup();
                error_code = ble_gap_scan_start();

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_RSCS);
                }

                g_rscs_active_state = SCAN_DEV_STATE;

                APP_LOG_DEBUG("Start scanning, target device: RSCS.");
                break;

            case HRRCPS_EVT_ENABLE_HRS_NTF:
                error_code = hrs_c_heart_rate_meas_notify_set(s_conn_idx_hrs_c, true);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_HRS_NTF_ENABLE);
                }

                s_user_write_id = USER_WR_HRS_NTF_EN;

                APP_LOG_DEBUG("Enable HRS notification.");
                break;

            case HRRCPS_EVT_DISABLE_HRS_NTF:
                error_code = hrs_c_heart_rate_meas_notify_set(s_conn_idx_hrs_c, false);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_HRS_NTF_DISABLE);
                }

                s_user_write_id = USER_WR_HRS_NTF_DIS;

                APP_LOG_DEBUG("Disable HRS notification.");
                break;

            case HRRCPS_EVT_ENABLE_RSCS_NTF:
                error_code = rscs_c_rsc_meas_notify_set(s_conn_idx_rscs_c, true);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_RSCS_NTF_ENABLE);
                }

                s_user_write_id = USER_WR_RSCS_NTF_EN;

                APP_LOG_DEBUG("Enable RSCS notification.");
                break;

            case HRRCPS_EVT_DISABLE_RSCS_NTF:
                error_code = rscs_c_rsc_meas_notify_set(s_conn_idx_rscs_c, false);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_RSCS_NTF_DISABLE);
                }

                s_user_write_id = USER_WR_RSCS_NTF_DIS;

                APP_LOG_DEBUG("Disable RSCS notification.");
                break;

            case HRRCPS_EVT_HRS_SENSOR_LOC_READ:
                error_code = hrs_c_sensor_loc_read(s_conn_idx_hrs_c);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_HRS_SEN_LOC_READ);
                }

                APP_LOG_DEBUG("Read HRS sensor location.");
                break;

            case HRRCPS_EVT_RSCS_SENSOR_LOC_READ:
                error_code = rscs_c_sensor_loc_read(s_conn_idx_rscs_c);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_RSCS_SEN_LOC_READ);
                }

                APP_LOG_DEBUG("Read RSCS sensor location.");
                break;

            case HRRCPS_EVT_DISCONN_HRS_LINK:
                error_code = ble_gap_disconnect(s_conn_idx_hrs_c);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_HRS_DISCONN);
                }
                APP_LOG_DEBUG("Disconnect HRS Link.");

                break;

            case HRRCPS_EVT_DISCONN_RSCS_LINK:
                error_code = ble_gap_disconnect(s_conn_idx_rscs_c);

                if (error_code != SDK_SUCCESS)
                {
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_RSCS_DISCONN);
                }

                APP_LOG_DEBUG("Disconnect RSCS Link.");
                break;

            default:
                break;
        }
    }
}

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *
 * @details Initialize the Running Speed and Cadence and Heart Rate services.
 *****************************************************************************************
 */
static void services_init(void)
{
    rscs_init_t rscs_env_init;
    hrs_init_t  hrs_env_init;
    sdk_err_t   error_code;

    /*------------------------------------------------------------------*/
    hrs_env_init.sensor_loc                  = (hrs_sensor_loc_t)0xff;
    hrs_env_init.char_mask                   = HRS_CHAR_MANDATORY | HRS_CHAR_BODY_SENSOR_LOC_SUP;
    hrs_env_init.evt_handler                 = hrs_service_process_event;
    hrs_env_init.is_sensor_contact_supported = true;
    error_code = hrs_service_init(&hrs_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    rscs_env_init.char_mask                  = HRS_CHAR_MANDATORY | RSCS_CHAR_SENSOR_LOC_SUP;
    rscs_env_init.feature                    = RSCS_FEAR_FULL_BIT;
    rscs_env_init.sensor_location            = (rscs_sensor_loc_t)0xff;
    rscs_env_init.evt_handler                = rsc_service_process_event;
    error_code = rscs_service_init(&rscs_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    error_code = hrrcps_service_init(hrrcps_evt_process);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            if (p_evt->evt_status)
            {
                APP_LOG_DEBUG("Adverting started failed(0x%02X).", p_evt->evt_status);
            }
            break;
            
        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.params.adv_stop.reason == BLE_GAP_STOPPED_REASON_TIMEOUT)
            {
                APP_LOG_DEBUG("Advertising timeout.");
            }
            break;
            
        case BLE_GAPM_EVT_SCAN_START:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                if (SCAN_DEV_STATE == g_hrs_active_state)
                {
                    g_hrs_active_state = NO_ACTIVE_STATE;
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_HRS);
                }
                else if (SCAN_DEV_STATE == g_rscs_active_state)
                {
                    g_rscs_active_state = NO_ACTIVE_STATE;
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_RSCS);
                }

                APP_LOG_DEBUG("Scan started failed(0x%02X).", p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                if (CONN_UNDERWAY_STATE == g_hrs_active_state)
                {
                    g_hrs_active_state = NO_ACTIVE_STATE;
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_HRS);
                }
                else if (CONN_UNDERWAY_STATE == g_rscs_active_state)
                {
                    g_rscs_active_state = NO_ACTIVE_STATE;
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_RSCS);
                }
            }
            else
            {
                if (BLE_GAP_STOPPED_REASON_TIMEOUT == p_evt->evt.gapm_evt.params.scan_stop.reason)
                {
                    APP_LOG_DEBUG("Stop scanning timeout: Can not scan any target device.");
                    if (SCAN_DEV_STATE == g_hrs_active_state)
                    {
                        g_hrs_active_state = NO_ACTIVE_STATE;
                        hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_HRS);
                    }
                    else if (SCAN_DEV_STATE == g_rscs_active_state)
                    {
                        g_rscs_active_state = NO_ACTIVE_STATE;
                        hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_RSCS);
                    }
                }
                else
                {
                    app_scan_stop_handler();
                }
            }
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            app_adv_report_handler(p_evt->evt.gapm_evt.params.adv_report.data, 
                                   p_evt->evt.gapm_evt.params.adv_report.length, 
                                   &p_evt->evt.gapm_evt.params.adv_report.broadcaster_addr);
            break;

        case BLE_GAPC_EVT_CONNECTED:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                if (CONN_UNDERWAY_STATE == g_hrs_active_state)
                {
                    g_hrs_active_state = NO_ACTIVE_STATE;
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_HRS);
                }
                else if (CONN_UNDERWAY_STATE == g_rscs_active_state)
                {
                    g_rscs_active_state = NO_ACTIVE_STATE;
                    hrrcps_op_error_handler(HRRCPS_CTRL_PT_SCAN_RSCS);
                }
            }
            else
            {
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
    sdk_err_t        error_code;
    ble_gap_bdaddr_t bd_addr;
    sdk_version_t    version;

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
    APP_LOG_INFO("HRS RSCS Relay example started.");

    services_init();
    services_client_init();
    gap_params_init();
    adv_params_init();
    scan_params_init();
    conn_params_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}
