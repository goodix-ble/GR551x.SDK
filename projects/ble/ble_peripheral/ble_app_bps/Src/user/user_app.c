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
#include "sensorsim.h"
#include "dis.h"
#include "bas.h"
#include "bps.h"
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
/**@brief GAP configuration parameters. */
#define DEVICE_NAME             "Goodix_BLP"    /**< Device Name which will be set in GAP. */

#define ADV_INTERVAL            320             /**< The advertising min interval (in units of 0.625 ms). */
#define ADV_TIMEOUT             0               /**< The advertising timeout in units of seconds. */
#define ADV_MAX_EVT             0

#define MAX_TX_POWER            0               /**< 0 dBm */
#define PRIVACY_RENEW_DURATION  900             /**< 900 seconds */

/**@brief Battery level and Blood pressure simulation parameters. */
#define BAT_LVL_MEAS_INTVL      2000            /**< Battery level measurement interval (in unit of 1 ms). */
#define MIN_BAT_LVL             81              /**< Minimum simulated battery level. */
#define MAX_BAT_LVL             100             /**< Maximum simulated 7battery level. */
#define BAT_LVL_INC             1               /**< Increment between each simulated battery level measurement. */
#define BLPS_MEAS_INTVL         1000            /**< Blood Pressure measurement interval (in unit of 1 ms). */
#define NUM_SIM_MEAS_VALUES     4               /**< Number of simulated measurements to cycle through. */
#define SIM_MEAS_1_SYSTOLIC     117             /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_1_DIASTOLIC    76              /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_1_MEAN_AP      103             /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_1_PULSE_RATE   60              /**< Simulated measurement value for pulse rate. */
#define SIM_MEAS_2_SYSTOLIC     121             /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_2_DIASTOLIC    81              /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_2_MEAN_AP      106             /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_2_PULSE_RATE   72              /**< Simulated measurement value for pulse rate. */
#define SIM_MEAS_3_SYSTOLIC     138             /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_3_DIASTOLIC    88              /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_3_MEAN_AP      120             /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_3_PULSE_RATE   105             /**< Simulated measurement value for pulse rate. */
#define SIM_MEAS_4_SYSTOLIC     145             /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_4_DIASTOLIC    100             /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_4_MEAN_AP      131             /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_4_PULSE_RATE   125             /**< Simulated measurement value for pulse rate. */

#ifdef MULTI_CONN
#define MAX_CONN_NUM            BPS_CONNECTION_MAX
#define APP_MAX_ADV_NUM         CFG_MAX_ADVS
#else
#define MAX_CONN_NUM            1
#define APP_MAX_ADV_NUM         1
#endif

/**@brief Structure for a simulated blood pressure measurment. An instance of
 *        this struct is filled out before sending a notification to the peer
          with ble_bps_measurement_send. */
typedef struct
{
    bps_ieee_float16_t systolic;
    bps_ieee_float16_t diastolic;
    bps_ieee_float16_t mean_arterial_pr;
    bps_ieee_float16_t pulse_rate;
} bls_meas_sim_value_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#ifdef MULTI_CONN
static uint8_t              s_adv_counter;                    /**< The counter of existing advertisings. */
static bool                 s_valid_adv_idx[APP_MAX_ADV_NUM];
#endif
static uint8_t              s_conn_counter;                   /**< The counter of established connections. */
static bool                 s_bl_inds[MAX_CONN_NUM];          /**< The array for BL indication flag of each connection. */
static uint8_t              s_bl_ind_counter;                 /**< The counter of the connnection with BL indication. */
static bool                 s_bat_ntfs[MAX_CONN_NUM];         /**< The array for BAT notification flag of each connection. */
static uint8_t              s_bat_ntf_counter;                /**< The counter of the connnection with BAT notification. */

static app_timer_id_t       s_bat_timer_id;                   /**< Battery timer id. */
static bool                 s_bat_timer_set;
static sensorsim_cfg_t      s_bat_sim_cfg;                    /** Battery Level sensor simulator configuration. */
static sensorsim_state_t    s_bat_sim_state;                  /** Battery Level sensor simulator state. */
static uint8_t              s_bat_lvl;

static app_timer_id_t       s_bl_meas_timer_id;               /**< Blood pressure measurement timer id. */
static bool                 s_bl_meas_timer_set;
static bls_meas_sim_value_t s_bl_meas_sim_val[NUM_SIM_MEAS_VALUES];

static ble_gap_adv_time_param_t s_adv_time_param;             /**< Advertising time parameter. */
static const uint8_t s_adv_data_set[] =                       /**< Advertising data. */
{
    // Don't put the name in advertising response data.
    // Extended connectable advertising does not support response data.
    0x0B,
    BLE_GAP_AD_TYPE_SHORTENED_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'B', 'L', 'P',

    // Device Appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE),
    HI_U16(BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE),

    // Device Service UUID
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_BLOOD_PRESSURE),
    HI_U16(BLE_ATT_SVC_BLOOD_PRESSURE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
};

/* Define the three DIS characteristics as 'Section 3.2 Increamental Device
 * Information Service Requirements' in <Blood Pressure Profile v1.0.1>*/
static char      s_devinfo_mfr_name[]     = "Goodix";         /**< Device manufacture name string. */
static char      s_devinfo_model_number[] = "blp-sensor-01";  /**< Device model number string. */
static dis_sys_id_t s_devinfo_system_id   =
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},        /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}                     /**< DUMMY Organisation Unique ID (OUI),
                                                                   You shall use the OUI of your company. */
};

/*
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */
#ifdef MULTI_CONN
static void start_extended_adv(uint8_t adv_idx);
#else
static void start_legacy_adv(void);
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Function for initializing the sensor simulators.
 *****************************************************************************************
 */
static void sensor_simulator_init(void)
{
    s_bat_sim_cfg.min          = MIN_BAT_LVL;
    s_bat_sim_cfg.max          = MAX_BAT_LVL;
    s_bat_sim_cfg.incr         = BAT_LVL_INC;
    s_bat_sim_cfg.start_at_max = true;

    sensorsim_init(&s_bat_sim_state, &s_bat_sim_cfg);

    // Simulated measurement #1.
    s_bl_meas_sim_val[0].systolic.mantissa         = SIM_MEAS_1_SYSTOLIC;
    s_bl_meas_sim_val[0].systolic.exponent         = 0;
    s_bl_meas_sim_val[0].diastolic.mantissa        = SIM_MEAS_1_DIASTOLIC;
    s_bl_meas_sim_val[0].diastolic.exponent        = 0;
    s_bl_meas_sim_val[0].mean_arterial_pr.mantissa = SIM_MEAS_1_MEAN_AP;
    s_bl_meas_sim_val[0].mean_arterial_pr.exponent = 0;
    s_bl_meas_sim_val[0].pulse_rate.mantissa       = SIM_MEAS_1_PULSE_RATE;
    s_bl_meas_sim_val[0].pulse_rate.exponent       = 0;

    // Simulated measurement #2.
    s_bl_meas_sim_val[1].systolic.mantissa         = SIM_MEAS_2_SYSTOLIC;
    s_bl_meas_sim_val[1].systolic.exponent         = 0;
    s_bl_meas_sim_val[1].diastolic.mantissa        = SIM_MEAS_2_DIASTOLIC;
    s_bl_meas_sim_val[1].diastolic.exponent        = 0;
    s_bl_meas_sim_val[1].mean_arterial_pr.mantissa = SIM_MEAS_2_MEAN_AP;
    s_bl_meas_sim_val[1].mean_arterial_pr.exponent = 0;
    s_bl_meas_sim_val[1].pulse_rate.mantissa       = SIM_MEAS_2_PULSE_RATE;
    s_bl_meas_sim_val[1].pulse_rate.exponent       = 0;

    // Simulated measurement #3.
    s_bl_meas_sim_val[2].systolic.mantissa         = SIM_MEAS_3_SYSTOLIC;
    s_bl_meas_sim_val[2].systolic.exponent         = 0;
    s_bl_meas_sim_val[2].diastolic.mantissa        = SIM_MEAS_3_DIASTOLIC;
    s_bl_meas_sim_val[2].diastolic.exponent        = 0;
    s_bl_meas_sim_val[2].mean_arterial_pr.mantissa = SIM_MEAS_3_MEAN_AP;
    s_bl_meas_sim_val[2].mean_arterial_pr.exponent = 0;
    s_bl_meas_sim_val[2].pulse_rate.mantissa       = SIM_MEAS_3_PULSE_RATE;
    s_bl_meas_sim_val[2].pulse_rate.exponent       = 0;

    // Simulated measurement #4.
    s_bl_meas_sim_val[3].systolic.mantissa         = SIM_MEAS_4_SYSTOLIC;
    s_bl_meas_sim_val[3].systolic.exponent         = 0;
    s_bl_meas_sim_val[3].diastolic.mantissa        = SIM_MEAS_4_DIASTOLIC;
    s_bl_meas_sim_val[3].diastolic.exponent        = 0;
    s_bl_meas_sim_val[3].mean_arterial_pr.mantissa = SIM_MEAS_4_MEAN_AP;
    s_bl_meas_sim_val[3].mean_arterial_pr.exponent = 0;
    s_bl_meas_sim_val[3].pulse_rate.mantissa       = SIM_MEAS_4_PULSE_RATE;
    s_bl_meas_sim_val[3].pulse_rate.exponent       = 0;
}

/**
 *****************************************************************************************
 * @brief Perform battery measurement and updating the battery level in Battery Service.
 *****************************************************************************************
 */
static void battery_level_update(void *p_arg)
{
    s_bat_lvl = (uint8_t)sensorsim_measure(&s_bat_sim_state, &s_bat_sim_cfg);

    for (uint8_t i = 0; i < MAX_CONN_NUM; i++)
    {
        if (s_bat_ntfs[i])
        {
            sdk_err_t error_code;
            error_code = bas_batt_lvl_update(i, 0, s_bat_lvl);

            if (SDK_ERR_NTF_DISABLED != error_code)
            {
                APP_ERROR_CHECK(error_code);
            }
        }
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
            APP_LOG_DEBUG("Battery Level Notification Enabled");
            s_bat_ntf_counter++;
            s_bat_ntfs[p_evt->conn_idx] = true;
            break;

        case BAS_EVT_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Battery Level Notification Disabled");
            s_bat_ntf_counter--;
            s_bat_ntfs[p_evt->conn_idx] = false;
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Populating simulated blood pressure measurements.
 *****************************************************************************************
 */
static void bps_sim_measurement(void *p_arg)
{
    bps_meas_t             sim_meas;
    static uint8_t         s_ndx = 0;
    static prf_date_time_t s_time_stamp = {2018, 06, 15, 11, 05, 03};

    sim_meas.bl_unit_in_kpa      = (s_ndx == 0) || (s_ndx == 2);
    sim_meas.time_stamp_present  = true;
    sim_meas.pulse_rate_present  = true;
    sim_meas.user_id_present     = true;
    sim_meas.meas_status_present = true;

    sim_meas.systolic.mantissa  = s_bl_meas_sim_val[s_ndx].systolic.mantissa;
    sim_meas.systolic.exponent  = s_bl_meas_sim_val[s_ndx].systolic.exponent;
    sim_meas.diastolic.mantissa = s_bl_meas_sim_val[s_ndx].diastolic.mantissa;
    sim_meas.diastolic.exponent = s_bl_meas_sim_val[s_ndx].diastolic.exponent;
    sim_meas.mean_arterial_pr.mantissa = s_bl_meas_sim_val[s_ndx].mean_arterial_pr.mantissa;
    sim_meas.mean_arterial_pr.exponent = s_bl_meas_sim_val[s_ndx].mean_arterial_pr.exponent;

    sim_meas.time_stamp = s_time_stamp;

    sim_meas.pulse_rate.mantissa = s_bl_meas_sim_val[s_ndx].pulse_rate.mantissa;
    sim_meas.pulse_rate.exponent = s_bl_meas_sim_val[s_ndx].pulse_rate.exponent;

    sim_meas.meas_status = BP_FEATURE_BODY_MOVEMENT_BIT | BP_FEATURE_MEASUREMENT_POSITION_BIT;

    // Update index to simulated measurements.
    s_ndx++;
    if (NUM_SIM_MEAS_VALUES == s_ndx)
    {
        s_ndx = 0;
    }

    // Update simulated time stamp.
    s_time_stamp.sec += 1;
    if (s_time_stamp.sec > 59)
    {
        s_time_stamp.sec = 0;
        s_time_stamp.min++;
        if (s_time_stamp.min > 59)
        {
            s_time_stamp.min = 0;
        }
    }

    for (uint8_t i = 0; i < MAX_CONN_NUM; i++)
    {
        if (s_bl_inds[i])
        {
            sdk_err_t error_code;

            error_code = bps_measurement_send(i, &sim_meas);
            APP_ERROR_CHECK(error_code);
        }
    }
}

/**
 *****************************************************************************************
 **@brief Process blood service event
 *****************************************************************************************
 */
static void blp_service_process_event(uint8_t conn_idx, bps_evt_type_t event)
{
    switch (event)
    {
        case BPS_EVT_BP_MEAS_INDICATION_ENABLED:
            s_bl_ind_counter++;
            s_bl_inds[conn_idx] = true;
            if (!s_bl_meas_timer_set)
            {
                sdk_err_t error_code = app_timer_start(s_bl_meas_timer_id,
                                                       BLPS_MEAS_INTVL, NULL);
                APP_ERROR_CHECK(error_code);
                s_bl_meas_timer_set = true;
                APP_LOG_DEBUG("Blood Pressure Timer Start");
            }
            break;

        case BPS_EVT_BP_MEAS_INDICATION_DISABLED:
            s_bl_ind_counter--;
            s_bl_inds[conn_idx] = false;
            if (0 == s_bl_ind_counter)
            {
                app_timer_stop(s_bl_meas_timer_id);
                s_bl_meas_timer_set = false;
                APP_LOG_DEBUG("Blood Pressure Timer Stop");
            }
            break;

        case BPS_EVT_READ_BL_PRESSURE_FEATURE:
            // The value 0x0011 is set in services_init().
            APP_LOG_DEBUG("Blood Pressure Feature: 0x0011");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 *          parameters of the device including the device name, appearance, and
 *          the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t error_code;

    ble_gap_pair_enable(true);
#ifndef PTS_AUTO_TEST
    error_code = ble_gap_privacy_params_set(PRIVACY_RENEW_DURATION, true);
    APP_ERROR_CHECK(error_code);
#endif

    // Set the default security parameters.
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL1,
        .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY | BLE_SEC_KDIST_SIGNKEY,
    };
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE,
                                         (uint8_t const *)DEVICE_NAME,
                                         strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

#ifndef PTS_AUTO_TEST
    ble_gap_conn_param_t conn_params;

    conn_params.interval_max  = CONN_INTERVAL;
    conn_params.interval_min  = CONN_INTERVAL;
    conn_params.slave_latency = SLAVE_LATENCY;
    conn_params.sup_timeout   = SUP_TIMEOUT;
    error_code = ble_gap_ppcp_set(&conn_params);
    APP_ERROR_CHECK(error_code);
#endif

    s_adv_time_param.duration    = ADV_TIMEOUT;
    s_adv_time_param.max_adv_evt = ADV_MAX_EVT;

    s_conn_counter = 0;
#ifdef MULTI_CONN
    s_adv_counter  = 0;
    start_extended_adv(0);
#else
    start_legacy_adv();
#endif
}

#ifdef MULTI_CONN
static void start_extended_adv(uint8_t adv_idx)
{
    sdk_err_t               error_code;
    ble_gap_ext_adv_param_t ext_adv_param;

    ext_adv_param.type                    = BLE_GAP_ADV_TYPE_EXTENDED;
    ext_adv_param.disc_mode               = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    ext_adv_param.prop                    = BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    ext_adv_param.max_tx_pwr              = MAX_TX_POWER;
    ext_adv_param.filter_pol              = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    ext_adv_param.prim_cfg.adv_intv_min   = ADV_INTERVAL;
    ext_adv_param.prim_cfg.adv_intv_max   = ADV_INTERVAL;
    ext_adv_param.prim_cfg.chnl_map       = BLE_GAP_ADV_CHANNEL_37_38_39;
    ext_adv_param.prim_cfg.phy            = BLE_GAP_PHY_1MBPS_VALUE;
    ext_adv_param.second_cfg.max_skip     = 0;
    ext_adv_param.second_cfg.phy          = BLE_GAP_PHY_1MBPS_VALUE;
    ext_adv_param.second_cfg.adv_sid      = 0x00;
    ext_adv_param.period_cfg.adv_intv_min = 0;
    ext_adv_param.period_cfg.adv_intv_max = 0;
    memset(&ext_adv_param.peer_addr, 0, sizeof(ble_gap_bdaddr_t));

    error_code = ble_gap_ext_adv_param_set(adv_idx,
                                           BLE_GAP_OWN_ADDR_GEN_RSLV,
                                           &ext_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(adv_idx,
                                      BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set,
                                      sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(adv_idx, &s_adv_time_param);
    APP_ERROR_CHECK(error_code);
    APP_LOG_DEBUG("Starting extended advertising");
}

#else
static void start_legacy_adv(void)
{
    sdk_err_t       error_code;
    ble_gap_adv_param_t lgc_adv_param;

    lgc_adv_param.adv_mode        = BLE_GAP_ADV_TYPE_ADV_IND;
    lgc_adv_param.disc_mode       = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    lgc_adv_param.filter_pol      = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    lgc_adv_param.adv_intv_max    = ADV_INTERVAL;
    lgc_adv_param.adv_intv_min    = ADV_INTERVAL;
    lgc_adv_param.chnl_map        = BLE_GAP_ADV_CHANNEL_37_38_39;
    lgc_adv_param.scan_req_ind_en = false;
    lgc_adv_param.max_tx_pwr      = MAX_TX_POWER;
    memset(&lgc_adv_param.peer_addr, 0, sizeof(ble_gap_bdaddr_t));

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &lgc_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set,
                                      sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(0, &s_adv_time_param);
    APP_ERROR_CHECK(error_code);
    APP_LOG_DEBUG("Starting legacy advertising");
}
#endif

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t  error_code;
    dis_init_t dis_env_init;
    bas_init_t bas_env_init[1];
    bps_init_t bps_env_init;

    /*------------------------------------------------------------------*/
    dis_env_init.char_mask                   = DIS_CHAR_MANUFACTURER_NAME_SUP |
                                               DIS_CHAR_MODEL_NUMBER_SUP |
                                               DIS_CHAR_SYSTEM_ID_SUP;
    dis_env_init.manufact_name_str.p_str     = s_devinfo_mfr_name;
    dis_env_init.manufact_name_str.length    = strlen(s_devinfo_mfr_name);
    dis_env_init.model_num_str.p_str         = s_devinfo_model_number;
    dis_env_init.model_num_str.length        = strlen(s_devinfo_model_number);
    dis_env_init.p_sys_id                    = &s_devinfo_system_id;
    dis_env_init.serial_num_str.p_str        = NULL;
    dis_env_init.serial_num_str.length       = 0;
    dis_env_init.hw_rev_str.p_str            = NULL;
    dis_env_init.hw_rev_str.length           = 0;
    dis_env_init.fw_rev_str.p_str            = NULL;
    dis_env_init.fw_rev_str.length           = 0;
    dis_env_init.sw_rev_str.p_str            = NULL;
    dis_env_init.sw_rev_str.length           = 0;
    dis_env_init.reg_cert_data_list.p_list   = NULL;
    dis_env_init.reg_cert_data_list.list_len = 0;
    dis_env_init.p_pnp_id                    = NULL;
    error_code = dis_service_init(&dis_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    bas_env_init[0].char_mask   = BAS_CHAR_MANDATORY | BAS_CHAR_LVL_NTF_SUP;
    bas_env_init[0].batt_lvl    = 0;
    bas_env_init[0].evt_handler = battery_service_process_event;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    bps_env_init.bp_feature  = BP_FEATURE_BODY_MOVEMENT_BIT |
                               BP_FEATURE_MEASUREMENT_POSITION_BIT;
    bps_env_init.char_mask   = BPS_CHAR_MANDATORY;
    bps_env_init.evt_handler = blp_service_process_event;
    error_code = bps_service_init(&bps_env_init);
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

    error_code = app_timer_create(&s_bl_meas_timer_id, ATIMER_REPEAT,
                                  bps_sim_measurement);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_bat_timer_id, ATIMER_REPEAT,
                                  battery_level_update);
    APP_ERROR_CHECK(error_code);
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;

    if (!s_bat_timer_set)
    {
        error_code = app_timer_start(s_bat_timer_id,
                                     BAT_LVL_MEAS_INTVL, NULL);
        APP_ERROR_CHECK(error_code);
        s_bat_timer_set = true;
    }

    s_conn_counter++;
    APP_LOG_INFO("Connected with No.%d peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 s_conn_counter - 1,
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);

#ifndef PTS_AUTO_TEST
    ble_gap_conn_update_param_t conn_param;

    conn_param.interval_min  = CONN_INTERVAL;
    conn_param.interval_max  = CONN_INTERVAL;
    conn_param.slave_latency = SLAVE_LATENCY;
    conn_param.sup_timeout   = SUP_TIMEOUT;
    conn_param.ce_len        = CONN_EVT_LEN;
    error_code = ble_gap_conn_param_update(conn_idx, &conn_param);
    APP_ERROR_CHECK(error_code);
#endif
}

static void start_next_adv(uint8_t adv_idx, start_adv_reason_t reason)
{
#ifdef MULTI_CONN
    switch (reason)
    {
        case ST_RSN_ADV_STARTED:
            s_adv_counter++;
            s_valid_adv_idx[adv_idx] = false;
            if (s_adv_counter < APP_MAX_ADV_NUM)
            {
                for (uint8_t i = 0; i < APP_MAX_ADV_NUM; i++)
                {
                    if (s_valid_adv_idx[i])
                    {
                        start_extended_adv(i);
                        break;
                    }
                }
            }
            break;

        case ST_RSN_ADV_STOPPED:
            /* Advertising is stopped for connection establishment. */
            s_adv_counter--;
            s_valid_adv_idx[adv_idx] = true;
            /* s_conn_counter has been increased in app_connected_handler(). */
            if ((s_conn_counter + s_adv_counter) < MAX_CONN_NUM)
            {
                start_extended_adv(adv_idx);
            }
            break;

        case ST_RSN_CONN_DISC:
            for (uint8_t i = 0; i < APP_MAX_ADV_NUM; i++)
            {
                if (s_valid_adv_idx[i])
                {
                    start_extended_adv(i);
                }
            }
            break;
        default:
            break;
    }
#else
    if (reason == ST_RSN_CONN_DISC)
    {
        start_legacy_adv();
    }
#endif
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    s_conn_counter--;
    s_bl_inds[conn_idx] = false;
    s_bat_ntfs[conn_idx] = false;

    if (0 == s_conn_counter)
    {
        APP_LOG_DEBUG("Blood Pressure Timer Stop");
        app_timer_stop(s_bat_timer_id);
        s_bat_timer_set = false;
        app_timer_stop(s_bl_meas_timer_id);
        s_bl_meas_timer_set = false;
    }

    start_next_adv(SDK_ERR_INVALID_ADV_IDX, ST_RSN_CONN_DISC);
}

static void app_gap_conn_update_req_handler(uint8_t conn_idx, const ble_gap_evt_conn_param_update_req_t *p_conn_param_update_req)
{
    bool accept;

    APP_LOG_INFO("peer_%d requests connection interval [%d, %d]", conn_idx, 
                  p_conn_param_update_req->interval_min, p_conn_param_update_req->interval_max);
    
    if (p_conn_param_update_req->interval_min >= CONN_INTERVAL)
    {
        accept = true;
        APP_LOG_INFO("Accept the peer's connection parameters");
    }
    else
    {
        accept = false;
        APP_LOG_INFO("Reject the peer's connection parameters");
    }
    ble_gap_conn_param_update_reply(conn_idx, accept);
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;

    if (NULL == p_enc_req)
    {
        return;
    }

    memset((uint8_t *)&cfm_enc, 0, sizeof(cfm_enc));

    switch (p_enc_req->req_type)
    {
        // User needs to decide whether to accept the pair request.
        case BLE_SEC_PAIR_REQ:
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept   = true;
            break;

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
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

        case BLE_GAPM_EVT_ADV_START:
            if (BLE_SUCCESS != p_evt->evt_status)
            {
                APP_LOG_DEBUG("Advertising is NOT started (0x%02X).", p_evt->evt_status);
            }
            else
            {
                APP_LOG_DEBUG("Advertising is started");
                start_next_adv(p_evt->evt.gapm_evt.index, ST_RSN_ADV_STARTED);
            }
            break;
            
        case BLE_GAPM_EVT_ADV_STOP:
            APP_LOG_DEBUG("Advertising stopped (0x%02X)", p_evt->evt.gapm_evt.params.adv_stop.reason);
            if (p_evt->evt.gapm_evt.params.adv_stop.reason == BLE_GAP_STOPPED_REASON_CONN_EST)
            {
                start_next_adv(p_evt->evt.gapm_evt.index, ST_RSN_ADV_STOPPED);
            }
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
            
        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;
        
        case BLE_SEC_EVT_LINK_ENCRYPTED:
            APP_LOG_DEBUG("Link has been successfully encrypted.");
            break;
        
        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            app_gap_conn_update_req_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.conn_param_update_req));
            break;
        
        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("Update conn_param of conn_%d successful.", p_evt->evt.gapc_evt.index);
            }
            else
            {
                APP_LOG_INFO("Update conn_param of conn_%d failed.", p_evt->evt.gapc_evt.index);
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
    APP_LOG_INFO("Blood Pressure example started.");

#ifdef MULTI_CONN
    for (uint8_t i = 0; i < APP_MAX_ADV_NUM; i++)
    {
        s_valid_adv_idx[i] = true;
    }
#endif

    sensor_simulator_init();
    services_init();
    gap_params_init();
    app_timer_init();
}

