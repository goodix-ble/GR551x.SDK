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

#define APP_LOG_TAG "user_app.c"

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_app.h"
#include "dis.h"
#include "hts.h"
#include "bas.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "sensorsim.h"
#include "utility.h"
#include "grx_sys.h"
#include "board_SK.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                         "Goodix_HTS"      /**< Device Name which will be set in GAP. */
#define APP_ADV_MIN_INTERVAL                32                /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL                160               /**< The slow advertising max interval (in units of 0.625 ms). */
#define FAST_ADV_DURATION                   3000              /**< Advertising duration (in unit of 10 ms).*/
#define APP_ADV_TIMEOUT_IN_SECONDS          0                 /**< The advertising timeout(in unit of 1 s). */

/**@brief Battery sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL         2000              /**< Battery level measurement interval (in unit of 1 ms). */
#define MIN_BATTERY_LEVEL                   81                /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100               /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                 /**< Increment between each simulated battery level measurement. */

/**@brief Temperature sensorsim data. */
#define TEMPERATURE_MEAS_INTERVAL           1                 /**< Temperature measurement interval (in unit of 1 s). */
#define MIN_CELCIUS_LEVEL                   3688              /**< Minimum simulated temperature level (in uints of 0.001 celcius). */
#define MAX_CELCIUS_LEVEL                   3972              /**< Maximum simulated temperature level (in uints of 0.001 celcius). */
#define CELCIUS_LEVEL_INCREMENT             36                /**< Increment between each simulated temperature level measurement. */
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                     /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;                /**< Advertising time parameter. */
static ble_sec_param_t          s_sec_param;                         /**< Security parameter. */
static app_timer_id_t           s_battery_level_timer_id;            /**< Battery timer id. */
static app_timer_id_t           s_temperature_level_timer_id;        /**< Temperature measurement timer id. */
static sensorsim_cfg_t          s_battery_sim_cfg;                   /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t        s_battery_sim_state;                 /**< Battery Level sensor simulator state. */
static uint8_t                  s_battery_level;
static sensorsim_cfg_t          s_temperature_sim_cfg;               /**< Temperature Level sensor simulator configuration. */
static sensorsim_state_t        s_temperature_sim_state;             /**< Temperature Level sensor simulator state. */
static uint32_t                 s_meas_interval;                     /**< Temperature measurement interval. */
static bool                     s_hts_temp_meas_ind_flag;            /**< Temperature measurement indication flag. */
static bool                     s_hts_intm_temp_ntf_flag;            /**< Intermediate temperature notificaiton flag. */
static bool                     s_hts_meas_intv_ind_flag;
static bool                     s_hts_indicate_interval_update;      /**< Indicate interval update flag. */
static bool                     s_is_hts_timer_set;
static uint8_t                  s_conn_idx;
static uint16_t                 s_pairing_result;

static hts_date_time_t      s_time_stamp     = {2018, 8, 20, 11, 02, 0};

static const uint8_t s_adv_data_set[] =                          /**< Advertising data. */
{
    // Device Appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_THERMOMETER),
    HI_U16(BLE_APPEARANCE_GENERIC_THERMOMETER),

    // Device Service UUID
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HEALTH_THERMOM),
    HI_U16(BLE_ATT_SVC_HEALTH_THERMOM),
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
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'H', 'T', 'S',
};

static dis_sys_id_t s_devinfo_system_id =               /**< Device system id. */
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},  /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}               /**< DUMMY Organisation Unique ID (OUI),
                                                             You shall use the OUI of your company. */
};

static char s_devinfo_model_number[]  = "thmeter-01";   /**< Device model number string. */
static char s_devinfo_serial_number[] = "0001";         /**< Device serial number string. */
static char s_devinfo_firmware_rev[]  = "1.0";          /**< Device firmware revision string. */
static char s_devinfo_hardware_rev[]  = "1.0";          /**< Device hardware revision string. */
static char s_devinfo_software_rev[]  = "0.80";         /**< Device software revision string. */
static char s_devinfo_mfr_name[]      = "Goodix";       /**< Device manufacture name string. */

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

    ble_gap_pair_enable(true);
#ifndef PTS_AUTO_TEST
    error_code = ble_gap_privacy_params_set(900, true);
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE,
                                         (uint8_t *)DEVICE_NAME,
                                         strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_sec_param.level     = BLE_SEC_MODE1_LEVEL2;
    s_sec_param.io_cap    = BLE_SEC_IO_DISPLAY_ONLY;
    s_sec_param.oob       = false;
    s_sec_param.auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON;
    s_sec_param.key_size  = 16;
    s_sec_param.ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY;
    s_sec_param.rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY;

    error_code = ble_sec_params_set(&s_sec_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set,
                                      sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set,
                                      sizeof(s_adv_rsp_data_set));
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

    s_temperature_sim_cfg.min          = MIN_CELCIUS_LEVEL;
    s_temperature_sim_cfg.max          = MAX_CELCIUS_LEVEL;
    s_temperature_sim_cfg.incr         = CELCIUS_LEVEL_INCREMENT;
    s_temperature_sim_cfg.start_at_max = false;
    sensorsim_init(&s_temperature_sim_state, &s_temperature_sim_cfg);
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
    error_code = bas_batt_lvl_update(s_conn_idx, 0, s_battery_level);
    if (SDK_ERR_NTF_DISABLED != error_code)
    {
        APP_ERROR_CHECK(error_code);
    }

}

/**
 *****************************************************************************************
 * @brief Process battery service event.
 *
 *****************************************************************************************
 */
static void battery_service_process_event(bas_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BAS_EVT_NOTIFICATION_ENABLED:
            APP_LOG_INFO("Battery Level Notification Enabled.");
            break;

        case BAS_EVT_NOTIFICATION_DISABLED:
            APP_LOG_INFO("Battery Level Notification Disabled.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Perform temperature measurement and updating temperature value in Battery Service.
 *****************************************************************************************
 */
static void hts_meas_timeout_handler(void *p_arg)
{
    hts_meas_val_t simulated_meas;
    uint16_t       temperature_value;
    sdk_err_t      error_code;
    static uint8_t s_temp_meas_count = 0;

    if (BLE_GAP_INVALID_CONN_INDEX == s_conn_idx)
    {
        return;
    }

    if (s_hts_meas_intv_ind_flag)
    {
#ifdef PTS_AUTO_TEST
    /* Send the indication of Mesaurement Interval to test HTS/SEN/CI/BV-02-C
     * automatically.
     * The indication shall be sent mannually in real product when the user
     * change the value in local.
     */
        hts_measurement_interval_send(s_conn_idx);
        s_hts_meas_intv_ind_flag = false;
#endif
    }

    if (s_hts_indicate_interval_update)
    {
#ifndef PTS_AUTO_TEST
        /* Don't indicate the interval to pass HTS/SEN/SP/BV-01-C.
         * PTS may has a bug that treat the invertal indication as measurement
         * indication.
         */
        if (BLE_SUCCESS == hts_measurement_interval_send(s_conn_idx))
        {
            s_hts_indicate_interval_update = false;
        }
#else
        s_hts_indicate_interval_update = false;
#endif
    }

    temperature_value = (uint16_t)sensorsim_measure(&s_temperature_sim_state,
                                                    &s_temperature_sim_cfg);
    simulated_meas.temp_original_value = temperature_value;
    simulated_meas.time_stamp          = s_time_stamp;

    s_time_stamp.sec++;
    s_temp_meas_count++;

    if (s_time_stamp.sec > 59)
    {
        s_time_stamp.sec = 0;
        s_time_stamp.min++;
        if (s_time_stamp.min > 59)
        {
            s_time_stamp.min = 0;
        }
    }

    if (s_hts_intm_temp_ntf_flag && s_hts_temp_meas_ind_flag)
    {
        if (s_temp_meas_count % s_meas_interval)
        {
            simulated_meas.temp_meas_type = HTS_TEMPERATURE_INTERMEDIATE;
            error_code = hts_measurement_send(s_conn_idx, &simulated_meas);
            APP_ERROR_CHECK(error_code);
        }
        else
        {
            s_temp_meas_count = 0;
            simulated_meas.temp_meas_type = HTS_TEMPERATURE_STABLE;
            error_code = hts_measurement_send(s_conn_idx, &simulated_meas);
            APP_ERROR_CHECK(error_code);
        }
    }

    if (s_hts_intm_temp_ntf_flag && !s_hts_temp_meas_ind_flag)
    {
        simulated_meas.temp_meas_type = HTS_TEMPERATURE_INTERMEDIATE;
        error_code = hts_measurement_send(s_conn_idx, &simulated_meas);
        APP_ERROR_CHECK(error_code);
    }

    if (s_hts_temp_meas_ind_flag && !s_hts_intm_temp_ntf_flag &&
        !(s_temp_meas_count % s_meas_interval))
    {
        simulated_meas.temp_meas_type = HTS_TEMPERATURE_STABLE;
        error_code = hts_measurement_send(s_conn_idx, &simulated_meas);
        APP_ERROR_CHECK(error_code);
    }
}

/**
 *****************************************************************************************
 * @brief Process health thermometer service event
 *****************************************************************************************
 */
static void hts_process_event(hts_evt_t *p_evt)
{
    sdk_err_t error_code;

    switch (p_evt->evt_type)
    {
        case HTS_EVT_TEM_MEAS_INDICATION_ENABLE:
            if (!s_is_hts_timer_set)
            {
                error_code = app_timer_start(s_temperature_level_timer_id,
                                             TEMPERATURE_MEAS_INTERVAL * 1000, NULL);
                APP_ERROR_CHECK(error_code);
                s_is_hts_timer_set = true;
            }
            s_hts_temp_meas_ind_flag = true;
            APP_LOG_INFO("Temperature Measurement Indication Enabled.");
            break;

        case HTS_EVT_TEM_MEAS_INDICATION_DISABLE:
            s_hts_temp_meas_ind_flag = false;
            if (s_is_hts_timer_set && !s_hts_intm_temp_ntf_flag)
            {
                app_timer_stop(s_temperature_level_timer_id);
                s_is_hts_timer_set = false;
            }
            APP_LOG_INFO("Temperature Measurement Indication Disabled.");
            break;

        case HTS_EVT_INTM_TEM_NOTIFICATION_ENABLE:
            if (!s_is_hts_timer_set)
            {
                error_code = app_timer_start(s_temperature_level_timer_id,
                                             TEMPERATURE_MEAS_INTERVAL * 1000, NULL);
                APP_ERROR_CHECK(error_code);
                s_is_hts_timer_set = true;
            }
            s_hts_intm_temp_ntf_flag = true;
            APP_LOG_INFO("Intermediate Temperature Notification Enabled.");
            break;

        case HTS_EVT_INTM_TEM_NOTIFICATION_DISABLE:
            s_hts_intm_temp_ntf_flag = false;
            if (s_is_hts_timer_set && !s_hts_temp_meas_ind_flag)
            {
                app_timer_stop(s_temperature_level_timer_id);
                s_is_hts_timer_set = false;
            }
            APP_LOG_INFO("Intermediate Temperature Notification Disabled.");
            break;

        case HTS_EVT_MEAS_INTERVAL_UPDATE:
            s_meas_interval = p_evt->p_data[0];
            s_hts_indicate_interval_update = true;
            if (0 != s_meas_interval && !s_is_hts_timer_set)
            {
                error_code = app_timer_start(s_temperature_level_timer_id,
                                             TEMPERATURE_MEAS_INTERVAL * 1000, NULL);
                APP_ERROR_CHECK(error_code);
                s_is_hts_timer_set = true;
            }
            if (0 == s_meas_interval && s_is_hts_timer_set)
            {
                app_timer_stop(s_temperature_level_timer_id);
                s_is_hts_timer_set = false;
            }
            APP_LOG_INFO("Temperature Measurement Interval update to %ds.", s_meas_interval);
            break;

        case HTS_EVT_MEAS_INTREVAL_INDICATION_ENABLE:
            s_hts_meas_intv_ind_flag = true;
            if (!s_is_hts_timer_set)
            {
                error_code = app_timer_start(s_temperature_level_timer_id,
                                             TEMPERATURE_MEAS_INTERVAL * 1000, NULL);
                APP_ERROR_CHECK(error_code);
                s_is_hts_timer_set = true;
            }
            APP_LOG_INFO("Measurement Interval Indication Enabled.");
            break;

        case HTS_EVT_MEAS_INTERVAL_INDICATION_DISABLE:
            s_hts_meas_intv_ind_flag = false;
            if (s_is_hts_timer_set &&
                !s_hts_temp_meas_ind_flag && !s_hts_intm_temp_ntf_flag)
            {
                app_timer_stop(s_temperature_level_timer_id);
                s_is_hts_timer_set = false;
            }
            APP_LOG_INFO("Measurement Interval Indication Disabled.");
            break;

        case HTS_EVT_READ_CHARACTERISTIC:
            if (HTS_READ_CHAR_TEMP_TYPE == *p_evt->p_data)
            {
                APP_LOG_INFO("Temperature Type: 0x%02X", HTS_TEMP_TYPE_BODY);
            }
            else if (HTS_READ_CHAR_MEAS_INTL == *p_evt->p_data)
            {
                APP_LOG_INFO("Measurement Interval: %ds", s_meas_interval);
            }
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
    sdk_err_t  error_code;
    dis_init_t dis_env_init;
    bas_init_t bas_env_init[1];
    hts_init_t hts_env_init;

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
    hts_env_init.char_mask             = HTS_CHAR_FULL;
    hts_env_init.evt_handler           = hts_process_event;
    hts_env_init.temperature_units     = HTS_TEMPERATURE_CELCIUS;
    hts_env_init.time_stamp_present    = true;
    hts_env_init.temp_type             = HTS_TEMP_TYPE_BODY;
    hts_env_init.meas_interval         = HTS_MEAS_INTV_DFLT_MIN;
    hts_env_init.min_meas_interval_sup = HTS_MEAS_INTV_DFLT_MIN;
    hts_env_init.max_meas_interval_sup = HTS_MEAS_INTV_DFLT_MAX;
    error_code = hts_service_init(&hts_env_init);
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

    error_code = app_timer_create(&s_temperature_level_timer_id, ATIMER_REPEAT,
                                  hts_meas_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_battery_level_timer_id, ATIMER_REPEAT,
                                  battery_level_update);
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

    s_conn_idx = conn_idx;

    error_code = app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

static void app_disconnected_handler(uint8_t reason)
{
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    sdk_err_t error_code;

    app_timer_stop(s_battery_level_timer_id);

    s_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    s_hts_temp_meas_ind_flag = false;
    s_hts_intm_temp_ntf_flag = false;

    app_timer_stop(s_temperature_level_timer_id);
    s_is_hts_timer_set = false;

    if (SDK_SUCCESS == s_pairing_result)
    {
        error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
        APP_ERROR_CHECK(error_code);
        APP_LOG_INFO("Starting the advertising.");
    }
    else
    {
        /* If disconnect for pairing failure, don't restart advertising. */
        APP_LOG_DEBUG("No advertising restarts for pairing failure");
    }
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t tk;

    if (NULL == p_enc_req)
    {
        return;
    }
    memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));

    switch (p_enc_req->req_type)
    {
        // user need to decide whether to accept the pair request
        case BLE_SEC_PAIR_REQ:
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept = true;
            break;

        // user need to input the password
        case BLE_SEC_TK_REQ:
            APP_LOG_INFO("Please Input pin code: 123456.");
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept = true;
            tk = 123456;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
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

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.params.disconnected.reason);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;
        
        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.params.adv_stop.reason == BLE_GAP_STOPPED_REASON_TIMEOUT)
            {
                APP_LOG_DEBUG("Advertising timeout.");
            }
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            s_pairing_result = p_evt->evt_status;
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
    APP_LOG_INFO("Health Thermometer example started.");

    /* PTS auto test checkes the value of measurement interval in HTS/SEN/CR/BV-03-C.
     * If change the initial value of s_meas_interval, must update PTS auto test
     * config file for HTS.
     */
    s_meas_interval  = HTS_MEAS_INTV_DFLT_MIN;
    s_conn_idx       = BLE_GAP_INVALID_CONN_INDEX;
    s_pairing_result = SDK_SUCCESS;

    sensor_simulator_init();
    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}
