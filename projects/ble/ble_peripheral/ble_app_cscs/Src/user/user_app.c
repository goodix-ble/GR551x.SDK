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
#include "cscs.h"
#include "sensorsim.h"
#include "grx_sys.h"
#include "utility.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "board_SK.h"
/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                         "Goodix_CSCS"       /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                160                 /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                160                 /**< The advertising max interval (in units of 0.625 ms). */

/**@brief Battery sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL         2000                /**< Battery level measurement interval (in uint of 1 ms). */
#define BATTERY_LEVEL_MIN                   81                  /**< Minimum simulated battery level. */
#define BATTERY_LEVEL_MAX                   100                 /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                   /**< Increment between each simulated battery level measurement. */

/**@brief Cycling Speed and Cadence sensorsim data. */
#define CSC_MEAS_INTERVAL                   1000                /**< CSC measurement interval (in uint of 1 ms). */
#define SPEED_KPH_MIN                       10                  /**< Minimum simulated speed in kilometers per hour. */
#define SPEED_KPH_MAX                       40                  /**< Maximum simulated speed in kilometers per hour. */
#define SPEED_KPH_INCREMENT                 1                   /**< Increment between each simulated speed measurement. */
#define CRANK_RPM_MIN                       20                  /**< Minimum simulated cadence in RPM. */
#define CRANK_RPM_MAX                       110                 /**< Maximum simulated cadence in RPM. */
#define CRANK_RPM_INCREMENT                 3                   /**< Increment between each simulated cadence measurement. */

#define WHEEL_CIRCUMFERENCE_MM              2100                /**< Simulated wheel circumference in millimeters. */
#define KPH_TO_MM_PER_SEC                   278                 /**< Constant to convert kilometers per hour into millimeters per second. */
#define DEGREES_PER_REVOLUTION              360                 /**< Constant used in simulation for calculating crank speed. */
#define RPM_TO_DEGREES_PER_SEC              6                   /**< Constant to convert revolutions per minute into degrees per second. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;           /**< Advertising time parameter. */

static app_timer_id_t       s_battery_level_timer_id;           /**< Battery timer id. */
static app_timer_id_t       s_csc_meas_timer_id;                /**< CSC measurement timer id. */
static sensorsim_cfg_t      s_battery_sim_cfg;                  /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t    s_battery_sim_state;                /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t      s_speed_kph_sim_cfg;                /**< Speed in KPH sensor simulator configuration. */
static sensorsim_state_t    s_speed_kph_sim_state;              /**< Speed in KPH sensor simulator state. */
static sensorsim_cfg_t      s_cadence_rpm_sim_cfg;              /**< Cadence in RPM sensor simulator configuration. */
static sensorsim_state_t    s_cadence_rpm_sim_state;            /**< Cadence in RPM sensor simulator state. */

static uint32_t             s_cumulative_wheel_revs;
static bool                 s_reverse_rotate_flag;

static const uint8_t s_adv_data_set[] =                         /**< Advertising data. */
{
    // Device Appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR),
    HI_U16(BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR),

    // Device Service UUID
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_CYCLING_SPEED_CADENCE),
    HI_U16(BLE_ATT_SVC_CYCLING_SPEED_CADENCE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix Company ID:04F7
    0xF7,
    0x04,
    0x02,
    0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                    /**< Scan responce data. */
{
    0x0c,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'C', 'S', 'C', 'S',
};

static dis_sys_id_t s_devinfo_system_id =                      /**< Device system id. */
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},         /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}                      /**< DUMMY Organisation Unique ID (OUI),
                                                                    You shall use the OUI of your company. */
};

static char s_devinfo_model_number[]  = "csc_sensor_01";       /**< Device model number string. */
static char s_devinfo_serial_number[] = "0001";                /**< Device serial number string. */
static char s_devinfo_firmware_rev[]  = "1.0";                 /**< Device firmware revision string. */
static char s_devinfo_hardware_rev[]  = "1.0";                 /**< Device hardware revision string. */
static char s_devinfo_software_rev[]  = "0.80";                /**< Device software revision string. */
static char s_devinfo_mfr_name[]      = "Goodix";              /**< Device manufacture name string. */

static char s_devinfo_cert[] =                                 /**< Device regulatory certification data. */
{
    DIS_11073_BODY_EXP,                                        /**< authoritative body type. */
    0x00,                                                      /**< authoritative body structure type. */
    'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l' /**< authoritative body data. */
};

static dis_pnp_id_t s_devinfo_pnp_id =                         /**< Device PnP id. */
{
    .vendor_id_source = 1,                                     /**< Vendor ID source (1=Bluetooth SIG). */
    .vendor_id        = 0x04F7,                                /**< Vendor ID. */
    .product_id       = 0x0000,                                /**< Product ID (vendor-specific). */
    .product_version  = 0x0110                                 /**< Product version (JJ.M.N). */
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
    sdk_err_t error_code;

    ble_gap_pair_enable(false);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL_MAX;
    s_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL_MIN;
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
    s_battery_sim_cfg.min          = BATTERY_LEVEL_MIN;
    s_battery_sim_cfg.max          = BATTERY_LEVEL_MAX;
    s_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    s_battery_sim_cfg.start_at_max = true;
    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);

    s_speed_kph_sim_cfg.min          = SPEED_KPH_MIN;
    s_speed_kph_sim_cfg.max          = SPEED_KPH_MAX;
    s_speed_kph_sim_cfg.incr         = SPEED_KPH_INCREMENT;
    s_speed_kph_sim_cfg.start_at_max = false;
    sensorsim_init(&s_speed_kph_sim_state, &s_speed_kph_sim_cfg);

    s_cadence_rpm_sim_cfg.min          = CRANK_RPM_MIN;
    s_cadence_rpm_sim_cfg.max          = CRANK_RPM_MAX;
    s_cadence_rpm_sim_cfg.incr         = CRANK_RPM_INCREMENT;
    s_cadence_rpm_sim_cfg.start_at_max = true;
    sensorsim_init(&s_cadence_rpm_sim_state, &s_cadence_rpm_sim_cfg);
}

/**
 *****************************************************************************************
 * @brief Perform battery measurement and updating the battery level in Battery Service.
 *****************************************************************************************
 */
static void battery_level_update(void *p_arg)
{
    sdk_err_t   error_code;

    s_battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state, &s_battery_sim_cfg);
    error_code = bas_batt_lvl_update(0, 0, s_battery_level);
    if (SDK_ERR_NTF_DISABLED != error_code)
    {
        APP_ERROR_CHECK(error_code);
    }
}

/**
 *****************************************************************************************
 * @brief Process battery service event.
 *****************************************************************************************
 */
static void battery_service_event_process(bas_evt_t *p_evt)
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

/**
 *****************************************************************************************
 * @brief Perform CSC measurement and updating CSC value.
 *****************************************************************************************
 */
static void csc_meas_timeout_handler(void *p_arg)
{
    static uint16_t  event_time            = 0;
    static uint16_t  cumulative_crank_revs = 0;

    cscs_meas_val_t cscs_meas_val;
    uint16_t        speed_kph;
    uint16_t        crank_rpm;
    uint16_t        speed_mm_per_sec;
    uint16_t        degrees_per_sec;
    uint16_t        event_time_inc;
    sdk_err_t       error_code;

    event_time_inc  = 1024 * CSC_MEAS_INTERVAL / 100;
    event_time     += event_time_inc;

    speed_kph                 = sensorsim_measure(&s_speed_kph_sim_state, &s_speed_kph_sim_cfg);
    speed_mm_per_sec          = speed_kph * KPH_TO_MM_PER_SEC * CSC_MEAS_INTERVAL / 100;

    if(!s_reverse_rotate_flag)
    {
        s_cumulative_wheel_revs  += speed_mm_per_sec / WHEEL_CIRCUMFERENCE_MM;
    }
    else
    {
        s_reverse_rotate_flag = false;
    }

    cscs_meas_val.wheel_rev_data_present = true;
    cscs_meas_val.cumulative_wheel_revs  = s_cumulative_wheel_revs;
    cscs_meas_val.last_wheel_event_time  = event_time;

    crank_rpm              = sensorsim_measure(&s_cadence_rpm_sim_state, &s_cadence_rpm_sim_cfg);
    degrees_per_sec        = crank_rpm * RPM_TO_DEGREES_PER_SEC * CSC_MEAS_INTERVAL / 100;
    cumulative_crank_revs += degrees_per_sec / DEGREES_PER_REVOLUTION;

    cscs_meas_val.crank_rev_data_present = true;
    cscs_meas_val.cumulative_crank_revs  = cumulative_crank_revs;
    cscs_meas_val.last_crank_event_time  = event_time;

    error_code = cscs_measurement_send(0, &cscs_meas_val);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Process Cycling Speed and Cadence service event
 *****************************************************************************************
 */
static void csc_service_event_process(cscs_evt_t *p_cscs_evt)
{
    uint8_t   rsp[CSCS_CTRL_PT_RSP_LEN_MIN];
    uint32_t  cumulative_wheel_revs_set;
    sdk_err_t error_code;

    switch (p_cscs_evt->evt_type)
    {
        case CSCS_EVT_CSC_MEAS_NOTIFICATION_ENABLE:
            error_code = app_timer_start(s_csc_meas_timer_id, CSC_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(error_code);
            APP_LOG_DEBUG("Cycling Speed and Cadence Timer Start.");
            break;

        case CSCS_EVT_CSC_MEAS_NOTIFICATION_DISABLE:
            app_timer_stop(s_csc_meas_timer_id);
            APP_LOG_DEBUG("Cycling Speed and Cadence Timer Stop");
            break;

        case CSCS_EVT_CUMUL_VAL_SET:
            cumulative_wheel_revs_set = BUILD_U32(p_cscs_evt->p_data[0], p_cscs_evt->p_data[1], p_cscs_evt->p_data[2], p_cscs_evt->p_data[3]);
            if (s_cumulative_wheel_revs <= cumulative_wheel_revs_set)
            {
                s_reverse_rotate_flag = true;
            }
            s_cumulative_wheel_revs = cumulative_wheel_revs_set;
            APP_LOG_DEBUG("Cumulative Wheel Revolution Set to %u", s_cumulative_wheel_revs);
            rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = CSCS_CTRL_PT_OP_SET_CUMUL_VAL;
            rsp[2] = CSCS_CTRL_PT_RSP_SUCCESS;
            error_code = cscs_ctrl_pt_rsp_send(p_cscs_evt->conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
            APP_ERROR_CHECK(error_code);
            break;

        case CSCS_EVT_SEBSOR_LOC_UPD:
            cscs_sensor_loc_update((cscs_sensor_loc_t)(p_cscs_evt->p_data[0]));
            APP_LOG_DEBUG("Update Sensor Location:0X%02x", p_cscs_evt->p_data[0]);
            rsp[0] = CSCS_CTRL_PT_OP_RSP_CODE;
            rsp[1] = CSCS_CTRL_PT_OP_UPD_LOC;
            rsp[2] = CSCS_CTRL_PT_RSP_SUCCESS;
            error_code = cscs_ctrl_pt_rsp_send(p_cscs_evt->conn_idx, rsp, CSCS_CTRL_PT_RSP_LEN_MIN);
            APP_ERROR_CHECK(error_code);
            break;

        case CSCS_EVT_SUP_SEBSOR_LOC_REQ:
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
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t   error_code;
    dis_init_t  dis_env_init;
    bas_init_t  bas_env_init[1];
    cscs_init_t cscs_env_init;

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
    dis_env_init.fw_rev_str.p_str            = s_devinfo_firmware_rev;
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
    bas_env_init[0].evt_handler = battery_service_event_process;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    cscs_env_init.char_mask       = CSCS_CHAR_FULL;
    cscs_env_init.feature         = CSCS_FEAR_FULL_BIT;
    cscs_env_init.sensor_location = CSCS_SENSOR_LOC_SHOE_TOP;
    cscs_env_init.evt_handler     = csc_service_event_process;
    error_code = cscs_service_init(&cscs_env_init);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t error_code;

    error_code = app_timer_create(&s_csc_meas_timer_id, ATIMER_REPEAT, csc_meas_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_battery_level_timer_id, ATIMER_REPEAT, battery_level_update);
    APP_ERROR_CHECK(error_code);
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;
    
    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                  p_param->peer_addr.addr[5],
                  p_param->peer_addr.addr[4],
                  p_param->peer_addr.addr[3],
                  p_param->peer_addr.addr[2],
                  p_param->peer_addr.addr[1],
                  p_param->peer_addr.addr[0]);

    error_code = app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    app_timer_stop(s_csc_meas_timer_id);
    app_timer_stop(s_battery_level_timer_id);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
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
                APP_LOG_DEBUG("Adverting started failed(0X%02X).", p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            if (BLE_GAP_STOPPED_REASON_TIMEOUT == p_evt->evt.gapm_evt.params.adv_stop.reason && BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Advertising timeout.");
            }
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
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
    APP_LOG_INFO("Cycling Speed and Cadence example started.");

    sensor_simulator_init();
    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

