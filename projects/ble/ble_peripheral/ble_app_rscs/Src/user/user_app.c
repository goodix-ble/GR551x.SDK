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
#include "dis.h"
#include "bas.h"
#include "rscs.h"
#include "sensorsim.h"
#include "grx_sys.h"
#include "utility.h"
#include "board_SK.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_log_dump_port.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                         "Goodix_RSCS"           /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL           32                      /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_FAST_MAX_INTERVAL           48                      /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL           160                     /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL           160                     /**< The slow advertising max interval (in units of 0.625 ms). */
#define FAST_ADV_DURATION                   3000                    /**< Advertising duration (in unit of 10 ms).*/
#define APP_ADV_TIMEOUT_IN_SECONDS          0                       /**< The advertising timeout(in units of 1 s). */

/**@brief Battery sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL         2000                    /**< Battery level measurement interval (in uint of 1 ms). */
#define MIN_BATTERY_LEVEL                   81                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                     /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                       /**< Increment between each simulated battery level measurement. */

/**@brief Running Speed and Cadence sensorsim data. */
#define RCS_MEAS_INTERVAL                   1000                    /**< Running Speed and Cadence measurement interval (in uint of 1 ms). */

#define MIN_SPEED_MPS_LEVEL                 128                     /**< Minimum speed in meters per second for use in the simulated measurement function. */
#define MAX_SPEED_MPS_LEVEL                 1664                    /**< Maximum speed in meters per second for use in the simulated measurement function. */
#define SPEED_LEVEL_INCREMENT               384                     /**< Increment between each simulated speed level measurement. */

#define MIN_CADENCE_RPM_LEVAL               40                      /**< Minimum cadence in revolutions per minute for use in the simulated measurement function. */
#define MAX_CADENCE_RPM_LEVAL               160                     /**< Maximum cadence in revolutions per minute for use in the simulated measurement function. */
#define CADENCE_LEVEL_INCREMENT             20                      /**< Increment between each simulated cadence level measurement. */

#define MIN_STRIDE_LEN_LEVEL                20                      /**< Minimum stride length in decimeter for use in the simulated measurement function. */
#define MAX_STRIDE_LEN_LEVEL                125                     /**< Maximum stride length in decimeter for use in the simulated measurement function. */
#define STRIDE_LEN_LEVAL_INCREMENT          5                       /**< Increment between each simulated stride length level measurement. */

#define MIN_SPEED_MPS_BY_RUNNNING           768                     /**< When speed reach this value, should lable walking to running. */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool g_sensor_calibration_done_flag = false;                        /**< Flag for sensor calibration has been done. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                    /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;               /**< Advertising time parameter. */

static app_timer_id_t       s_battery_level_timer_id;               /**< Battery measurement timer id. */
static app_timer_id_t       s_rsc_meas_timer_id;                    /**< Running Speed and Cadence measurement timer id. */
static app_timer_id_t       s_sensor_calibration_execute_timer_id;  /**< Sensor calibration timer id.*/

static sensorsim_cfg_t      s_battery_sim_cfg;                      /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t    s_battery_sim_state;                    /**< Battery Level sensor simulator state. */

static sensorsim_cfg_t      s_speed_mps_sim_cfg;                    /**< Speed simulator configuration. */
static sensorsim_state_t    s_speed_mps_sim_state;                  /**< Speed simulator state. */
static sensorsim_cfg_t      s_cadence_rpm_sim_cfg;                  /**< Cadence simulator configuration. */
static sensorsim_state_t    s_cadence_rpm_sim_state;                /**< Cadence simulator state. */
static sensorsim_cfg_t      s_stride_len_sim_cfg;                   /**< stride length simulator configuration. */
static sensorsim_state_t    s_stride_len_sim_state;                 /**< stride length simulator state. */

static uint32_t             s_total_distance_value = 0;             /**< Total distance. */

static const uint8_t s_adv_data_set[] =                             /**< Advertising data. */
{
    // Device Appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR),
    HI_U16(BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR),

    // Device Service UUID
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE),
    HI_U16(BLE_ATT_SVC_RUNNING_SPEED_CADENCE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier:0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02, 0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                      /**< Scan responce data. */
{
    0x0c,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'R', 'S', 'C', 'S',
};

static dis_sys_id_t s_devinfo_system_id =               /**< Device system id. */
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},  /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}               /**< DUMMY Organisation Unique ID (OUI),
                                                             You shall use the OUI of your company. */
};

static char s_devinfo_model_number[]  = "rsc_sensor_01";       /**< Device model number string. */
static char s_devinfo_serial_number[] = "0001";                /**< Device serial number string. */
static char s_devinfo_firmware_rev[]  = "1.0";                 /**< Device firmware revision string. */
static char s_devinfo_hardware_rev[]  = "1.0";                 /**< Device hardware revision string. */
static char s_devinfo_software_rev[]  = "0.80";                /**< Device software revision string. */
static char s_devinfo_mfr_name[]      = "Goodix";              /**< Device manufacture name string. */

static char s_devinfo_cert[] =                               /**< Device regulatory certification data. */
{
    DIS_11073_BODY_EXP,                                          /**< authoritative body type. */
    0x00,                                                        /**< authoritative body structure type. */
    'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'   /**< authoritative body data. */
};

static dis_pnp_id_t s_devinfo_pnp_id =                           /**< Device PnP id. */
{
    .vendor_id_source = 1,                                       /**< Vendor ID source (1=Bluetooth SIG). */
    .vendor_id        = 0x04F7,                                  /**< Vendor ID. */
    .product_id       = 0x1234,                                  /**< Product ID (vendor-specific). */
    .product_version  = 0x0110                                   /**< Product version (JJ.M.N). */
};

static uint8_t s_battery_level;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(false);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_FAST_MIN_INTERVAL;
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
 * @brief Initialize the sensor simulators.
 *****************************************************************************************
 */
static void sensor_simulator_init(void)
{
    s_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    s_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    s_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    s_battery_sim_cfg.start_at_max = true;
    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);

    s_speed_mps_sim_cfg.min          = MIN_SPEED_MPS_LEVEL;
    s_speed_mps_sim_cfg.max          = MAX_SPEED_MPS_LEVEL;
    s_speed_mps_sim_cfg.incr         = SPEED_LEVEL_INCREMENT;
    s_speed_mps_sim_cfg.start_at_max = false;
    sensorsim_init(&s_speed_mps_sim_state, &s_speed_mps_sim_cfg);

    s_cadence_rpm_sim_cfg.min          = MIN_CADENCE_RPM_LEVAL;
    s_cadence_rpm_sim_cfg.max          = MAX_CADENCE_RPM_LEVAL;
    s_cadence_rpm_sim_cfg.incr         = CADENCE_LEVEL_INCREMENT;
    s_cadence_rpm_sim_cfg.start_at_max = false;
    sensorsim_init(&s_cadence_rpm_sim_state, &s_cadence_rpm_sim_cfg);

    s_stride_len_sim_cfg.min          = MIN_STRIDE_LEN_LEVEL;
    s_stride_len_sim_cfg.max          = MAX_STRIDE_LEN_LEVEL;
    s_stride_len_sim_cfg.incr         = STRIDE_LEN_LEVAL_INCREMENT;
    s_stride_len_sim_cfg.start_at_max = false;
    sensorsim_init(&s_stride_len_sim_state, &s_stride_len_sim_cfg);
}

static void battery_level_send(void *p_evt_data, uint16_t evt_data_size)
{
    sdk_err_t error_code;

    error_code = bas_batt_lvl_update(0, 0, *(uint8_t *)p_evt_data);
    if (SDK_ERR_NTF_DISABLED != error_code)
    {
        APP_ERROR_CHECK(error_code);
    }
}

static void battery_level_update(void *p_arg)
{
    s_battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state, &s_battery_sim_cfg);

    app_scheduler_evt_put(&s_battery_level, sizeof(uint8_t), battery_level_send);
}

/**
 *****************************************************************************************
 * @brief Process battery service event.
 *****************************************************************************************
 */
static void battery_service_process_event(bas_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BAS_EVT_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Battery Level Notification Enabled.");
            break;

        case BAS_EVT_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Battery Level Notification Disabled.");
            break;

        default:
            break;
    }
}

static void rsc_meas_data_send(void *p_evt_data, uint16_t evt_data_size)
{
    sdk_err_t error_code;

    error_code = rscs_measurement_send(0, (rscs_meas_val_t *)p_evt_data);
    APP_ERROR_CHECK(error_code);
}

static void rsc_meas_update(void *p_arg)
{
    rscs_meas_val_t rsc_measurement;
    uint16_t        inst_speed_value;
    uint8_t         inst_cadence_value;
    uint16_t        inst_stride_len_value;

    inst_speed_value        = sensorsim_measure(&s_speed_mps_sim_state, &s_speed_mps_sim_cfg);
    inst_cadence_value      = sensorsim_measure(&s_cadence_rpm_sim_state, &s_cadence_rpm_sim_cfg);
    inst_stride_len_value   = sensorsim_measure(&s_stride_len_sim_state, &s_stride_len_sim_cfg);

    rsc_measurement.inst_speed         = inst_speed_value;
    rsc_measurement.inst_cadence       = inst_cadence_value;
    rsc_measurement.inst_stride_length = inst_stride_len_value;
    rsc_measurement.total_distance     = s_total_distance_value;
    rsc_measurement.inst_stride_length_present = true;
    rsc_measurement.total_distance_present     = true;

    if (MIN_SPEED_MPS_BY_RUNNNING < inst_speed_value)
    {
        rsc_measurement.is_run_or_walk = true;
    }
    else
    {
        rsc_measurement.is_run_or_walk = false;
    }

    app_scheduler_evt_put(&rsc_measurement, sizeof(rsc_measurement), rsc_meas_data_send);

    s_total_distance_value += inst_stride_len_value;
}

static void sensor_calibration_timeout_handler(void *p_arg)
{
    uint8_t rsp[RSCS_CTRL_PT_RSP_LEN_MIN];
    sdk_err_t   error_code;

    rsp[0] = RSCS_CTRL_PT_OP_RSP_CODE;
    rsp[1] = RSCS_CTRL_PT_OP_START_CALIB;
    if (g_sensor_calibration_done_flag)
    {
        rsp[2] = RSCS_CTRL_PT_RSP_SUCCESS;
        g_sensor_calibration_done_flag = false;
        APP_LOG_DEBUG("Sensor Calibration Successfully.");
    }
    else
    {
        rsp[2] = RSCS_CTRL_PT_RSP_FAILED;
        APP_LOG_DEBUG("Sensor Calibration Failed.");
    }

    error_code = rscs_ctrl_pt_rsp_send(0, rsp, RSCS_CTRL_PT_RSP_LEN_MIN);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Process Running Speed and Cadence Service event.
 *
 * @param[in] event: Running Speed and Cadence Service event types.
 *****************************************************************************************
 */
static void rsc_service_process_event(rscs_evt_t *p_rscs_evt)
{
    uint8_t rsp[RSCS_CTRL_PT_RSP_LEN_MIN];
    sdk_err_t   error_code;

    switch (p_rscs_evt->evt_type)
    {
        case RSCS_EVT_RSC_MEAS_NOTIFICATION_ENABLE:
            error_code = app_timer_start(s_rsc_meas_timer_id, RCS_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(error_code);
            APP_LOG_DEBUG("Running Speed and Cadence Timer Start");
            break;

        case RSCS_EVT_RSC_MEAS_NOTIFICATION_DISABLE:
            app_timer_stop(s_rsc_meas_timer_id);
            APP_LOG_DEBUG("Running Speed and Cadence Timer Stop");
            break;

        case RSCS_EVT_CUMUL_VAL_SET:
            s_total_distance_value = BUILD_U32(p_rscs_evt->p_data[0],
                                               p_rscs_evt->p_data[1],
                                               p_rscs_evt->p_data[2],
                                               p_rscs_evt->p_data[3]);
            APP_LOG_DEBUG("Total Distance Set to %u", s_total_distance_value);
            rsp[0] = RSCS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = RSCS_CTRL_PT_OP_SET_CUMUL_VAL;
            rsp[2] = RSCS_CTRL_PT_RSP_SUCCESS;
            error_code = rscs_ctrl_pt_rsp_send(p_rscs_evt->conn_idx, rsp, RSCS_CTRL_PT_RSP_LEN_MIN);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_EVT_SEBSOR_CALIBRATION:
            APP_LOG_DEBUG("Please Start Sensor Calibration in 5s");
            error_code = app_timer_start(s_sensor_calibration_execute_timer_id, 5000, NULL);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_EVT_SEBSOR_LOC_UPD:
            error_code = rscs_sensor_loc_update((rscs_sensor_loc_t)(p_rscs_evt->p_data[0]));
            APP_ERROR_CHECK(error_code);
            APP_LOG_DEBUG("Update Sensor Location:0X%02x", p_rscs_evt->p_data[0]);
            rsp[0] = RSCS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = RSCS_CTRL_PT_OP_UPD_LOC;
            rsp[2] = RSCS_CTRL_PT_RSP_SUCCESS;
            error_code = rscs_ctrl_pt_rsp_send(p_rscs_evt->conn_idx, rsp, RSCS_CTRL_PT_RSP_LEN_MIN);
            APP_ERROR_CHECK(error_code);
            break;

        case RSCS_EVT_SUP_SEBSOR_LOC_REQ:
            APP_LOG_DEBUG("Request Support Sensor Location List");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *
 * @details Initialize the Running Speed and Cadence, Battery and Device Information services.
 *****************************************************************************************
 */
static void services_init(void)
{
    dis_init_t  dis_env_init;
    bas_init_t  bas_env_init[1];
    rscs_init_t rscs_env_init;
    sdk_err_t   error_code;

    /*------------------------------------------------------------------*/
    dis_env_init.char_mask                   = DIS_CHAR_FULL;
    dis_env_init.manufact_name_str.p_str     = s_devinfo_mfr_name;
    dis_env_init.manufact_name_str.length    = strlen(s_devinfo_mfr_name);
    dis_env_init.model_num_str.p_str         = s_devinfo_model_number;
    dis_env_init.model_num_str.length        = strlen(s_devinfo_model_number);
    dis_env_init.serial_num_str.p_str        = s_devinfo_serial_number;
    dis_env_init.serial_num_str.length       = strlen(s_devinfo_serial_number);
    dis_env_init.hw_rev_str.p_str            = s_devinfo_hardware_rev;
    dis_env_init.hw_rev_str.length           = strlen(s_devinfo_hardware_rev);
    dis_env_init.fw_rev_str.p_str            =  s_devinfo_firmware_rev;
    dis_env_init.fw_rev_str.length           = strlen(s_devinfo_firmware_rev);
    dis_env_init.sw_rev_str.p_str            = s_devinfo_software_rev;
    dis_env_init.sw_rev_str.length           = strlen(s_devinfo_software_rev);
    dis_env_init.p_sys_id                    = &s_devinfo_system_id;
    dis_env_init.reg_cert_data_list.p_list   = s_devinfo_cert;
    dis_env_init.reg_cert_data_list.list_len = strlen(s_devinfo_cert);
    dis_env_init.p_pnp_id                    = &s_devinfo_pnp_id;
    error_code = dis_service_init(&dis_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    bas_env_init[0].char_mask   = BAS_CHAR_FULL;
    bas_env_init[0].batt_lvl    = 87;
    bas_env_init[0].evt_handler = battery_service_process_event;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    rscs_env_init.char_mask                  = RSCS_CHAR_FULL;
    rscs_env_init.feature                    = RSCS_FEAR_FULL_BIT;
    rscs_env_init.sensor_location            = RSCS_SENSOR_LOC_SHOE_TOP;
    rscs_env_init.evt_handler                = rsc_service_process_event;
    error_code = rscs_service_init(&rscs_env_init);
    APP_ERROR_CHECK(error_code);
    
#if APP_LOG_STORE_ENABLE
    app_log_dump_service_init();
#endif
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t   error_code;

    error_code = app_timer_create(&s_rsc_meas_timer_id, ATIMER_REPEAT, rsc_meas_update);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_battery_level_timer_id, ATIMER_REPEAT, battery_level_update);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_sensor_calibration_execute_timer_id, ATIMER_ONE_SHOT, sensor_calibration_timeout_handler);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void app_connected_handler(void *p_evt_data, uint16_t evt_data_size)
{
    sdk_err_t               error_code;
    ble_gap_evt_connected_t *p_conn_param = (ble_gap_evt_connected_t *)p_evt_data;
    UNUSED_VARIABLE(p_conn_param->peer_addr.addr[0]);
    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
             p_conn_param->peer_addr.addr[5],
             p_conn_param->peer_addr.addr[4],
             p_conn_param->peer_addr.addr[3],
             p_conn_param->peer_addr.addr[2],
             p_conn_param->peer_addr.addr[1],
             p_conn_param->peer_addr.addr[0]);

    error_code = app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

void app_disconnected_handler(void *p_evt_data, uint16_t evt_data_size)
{
    sdk_err_t error_code;

    app_timer_stop(s_rsc_meas_timer_id);
    app_timer_stop(s_battery_level_timer_id);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

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
                APP_LOG_DEBUG("Adverting started failed(0X%02X).", p_evt->evt_status);
            }
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_scheduler_evt_put(&p_evt->evt.gapc_evt.params.connected, sizeof(ble_gap_evt_connected_t), app_connected_handler);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            APP_LOG_INFO("Disconnected (0x%02X).", p_evt->evt.gapc_evt.params.disconnected.reason);
            app_scheduler_evt_put(&p_evt->evt.gapc_evt.params.disconnected.reason, sizeof(uint8_t), app_disconnected_handler);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                #if APP_LOG_STORE_ENABLE
                lms_update_mtu_size(p_evt->evt.gatt_common_evt.params.mtu_exchange.mtu);
                #endif
            }
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
    APP_LOG_INFO("Running Speed and Cadence example started.");

    sensor_simulator_init();
    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

