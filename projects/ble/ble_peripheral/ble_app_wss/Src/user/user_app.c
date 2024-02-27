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
#include "cts.h"
#include "bcs.h"
#include "wss.h"
#include "uds.h"
#include "wss_db.h"
#include "sensorsim.h"
#include "grx_sys.h"
#include "utility.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"


/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                          "Goodix_WSS" /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                 48           /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                 160          /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT                      0            /**< Advertising timeout (in units of 10ms). */
#define CONN_INTERVAL                        80           /**< Connection interval. */
#define SLAVE_LATENCY                        0            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     500          /**< Connection support timeout. */

/**@brief Weight Scale Database sensorsim data. */
#define DATABASE_UPDATE_INTERVAL             2000         /**< WSS Database Value Update interval (in units of 1 ms). */

/**@brief Body Composition sensorsim data. */
#define FAT_PERC_MIN                         110          /**< Minimum Body Fat Percentage value. Unit is 1/10 of a percent.*/
#define FAT_PERC_MAX                         340          /**< Maximum Body Fat Percentage value. */
#define FAT_PERC_INCREMENT                   1            /**< Increment between each Body Fat Percentage value measurement. */
#define BM_KCAL_MIN                          1000         /**< Minimum Basal Metabolism value. Unit is  */
#define BM_KCAL_MAX                          2000         /**< Maximum Basal Metabolism value. */
#define BM_KCAL_INCREMENT                    20           /**< Increment between each Basal Metabolism value measurement. */
#define MUSCLE_PERC_MIN                      460          /**< Minimum Muscle Percentage value. Unit is 1/10 of a percent.*/
#define MUSCLE_PERC_MAX                      600          /**< Maximum Muscle Percentage value. */
#define MUSCLE_PERC_INCREMENT                1            /**< Increment between each Muscle Percentage value measurement. */
#define WATER_PERC_MIN                       45           /**< Minimum Body Water Percentage value. Unit is a percent. */
#define WATER_PERC_MAX                       60           /**< Maximum Body Water Percentage value. */
#define WATER_PERC_INCREMENT                 1            /**< Increment between each Body Water Percentage value measurement. */
#define IMPEDANCE_OHM_MIN                    10000        /**< Minimum Impedance value. Unit is 1/10 of an Ohm*/
#define IMPEDANCE_OHM_MAX                    20000        /**< Minimum Impedance value. */
#define IMPEDANCE_OHM_INCREMENT              20           /**< Increment between each Impedance value measurement. */

/**@brief Weight Scale sensorsim data. */
#define WEIGHT_5G_MIN                        8000         /**< Minimum Weight value. */
#define WEIGHT_5G_MAX                        10500        /**< Maximum Weight value. */
#define WEIGHT_5G_INCREMENT                  20           /**< Increment between each weight value measurement. */

/**@brief Current Time sensorsim data. */
#define CURRENT_TIME_UPDATE_INTERVAL         1000         /**< Current Time Update interval (in uint of 1 ms). */
#define DSTTUC_CONVERT_TO_HOURS              15           /**< DST and UTC are converted to hours */
#define DAY_CONTAINS_MINUTES                 1440         /**< A day contains minutes */

/**@brief Battery sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL          4000         /**< Battery level measurement interval (in uint of 1 ms). */
#define BATTERY_LEVEL_MIN                    81           /**< Minimum simulated battery level. */
#define BATTERY_LEVEL_MAX                    100          /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1            /**< Increment between each simulated battery level measurement. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t         s_gap_adv_param;            /**< Advertising parameters for legcay advertising. */
static ble_gap_adv_time_param_t    s_gap_adv_time_param;       /**< Advertising time parameter. */
static ble_gap_conn_update_param_t s_gap_conn_param;           /**< Connection parameter. */

static app_timer_id_t              s_db_timer_id;              /**< Database timer id. */
static app_timer_id_t              s_cts_timer_id;             /**< Current Time timer id. */
static app_timer_id_t              s_battery_level_timer_id;   /**< Battery timer id. */

static sensorsim_cfg_t             s_fat_perc_sim_cfg;         /**< Body Fat Percentage sensor simulator configuration. */
static sensorsim_state_t           s_fat_perc_sim_state;       /**< Body Fat Percentage sensor simulator state. */
static sensorsim_cfg_t             s_water_perc_sim_cfg;       /**< Water Percentage sensor simulator configuration. */
static sensorsim_state_t           s_water_perc_sim_state;     /**< Water Percentage sensor simulator state. */
static sensorsim_cfg_t             s_muscle_perc_sim_cfg;      /**< Muscle Percentage sensor simulator configuration. */
static sensorsim_state_t           s_muscle_perc_sim_state;    /**< Muscle Percentage sensor simulator state. */
static sensorsim_cfg_t             s_bm_kcal_sim_cfg;          /**< Basal Metabolism in kcal sensor simulator configuration. */
static sensorsim_state_t           s_bm_kcal_sim_state;        /**< Basal Metabolism in kcal sensor simulator state. */
static sensorsim_cfg_t             s_impedance_ohm_sim_cfg;    /**< Body Impedance in ohm sensor simulator configuration. */
static sensorsim_state_t           s_impedance_ohm_sim_state;  /**< Body Impedance in ohm sensor simulator state. */
static sensorsim_cfg_t             s_weight_5g_sim_cfg;        /**< Weight in 5G sensor simulator configuration. */
static sensorsim_state_t           s_weight_5g_sim_state;      /**< Weight in 5G sensor simulator state. */
static sensorsim_cfg_t             s_battery_sim_cfg;          /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t           s_battery_sim_state;        /**< Battery Level sensor simulator state. */

static uint8_t                     s_cur_user_id;              /**< Current user index. */ 
static uint16_t                    s_wss_weight;               /**< Current user's weight value. */ 
static uint16_t                    s_uds_height;               /**< Current user's height value. */ 
static cts_init_t                  s_current_exact_time;       /**< Current Exact Time value. */

static uint8_t                     s_bc_meas_start;
static uint8_t                     s_ws_meas_start;
static meas_val_t                  s_meas_val[25];
static bcs_meas_val_t              s_bcs_meas_val[25];
static wss_meas_val_t              s_wss_meas_val[25];

uint8_t                            db_wait_update;

static const uint8_t s_adv_data_set[] =                        /**< Advertising data. */
{
    // Device Apperance
    0X03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_WEIGHT_SCALE),
    HI_U16(BLE_APPEARANCE_GENERIC_WEIGHT_SCALE),

    // Device Service UUID
    0x0D,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_WEIGHT_SCALE),
    HI_U16(BLE_ATT_SVC_WEIGHT_SCALE),
    LO_U16(BLE_ATT_SVC_BODY_COMPOSITION),
    HI_U16(BLE_ATT_SVC_BODY_COMPOSITION),
    LO_U16(BLE_ATT_SVC_USER_DATA),
    HI_U16(BLE_ATT_SVC_USER_DATA),
    LO_U16(BLE_ATT_SVC_CURRENT_TIME),
    HI_U16(BLE_ATT_SVC_CURRENT_TIME),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02, 
    0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                      /**< Scan responce data. */
{
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'W', 'S', 'S',
};

static dis_sys_id_t s_devinfo_system_id =                        /**< Device system id. */
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},           /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}                        /**< DUMMY Organisation Unique ID (OUI),
                                                                      You shall use the OUI of your company. */
};

static char s_devinfo_model_number[]  = "ws_sensor_01";          /**< Device model number string. */
static char s_devinfo_serial_number[] = "0001";                  /**< Device serial number string. */
static char s_devinfo_firmware_rev[]  = "1.0";                   /**< Device firmware revision string. */
static char s_devinfo_hardware_rev[]  = "1.0";                   /**< Device hardware revision string. */
static char s_devinfo_software_rev[]  = "0.80";                  /**< Device software revision string. */
static char s_devinfo_mfr_name[]      = "Goodix";                /**< Device manufacture name string. */

static char s_devinfo_cert[] =                                   /**< Device regulatory certification data. */
{
    DIS_11073_BODY_EXP,                                          /**< authoritative body type. */
    0x00,                                                        /**< authoritative body structure type. */
    'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'   /**< authoritative body data. */
};

static dis_pnp_id_t s_devinfo_pnp_id =                           /**< Device PnP id. */
{
    .vendor_id_source = 1,                                       /**< Vendor ID source (1=Bluetooth SIG). */
    .vendor_id        = 0x04F7,                                  /**< Vendor ID. */
    .product_id       = 0x0000,                                  /**< Product ID (vendor-specific). */
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
    sdk_err_t error_code;

    ble_gap_pair_enable(true);
    error_code = ble_gap_privacy_params_set(900, true);

    // Set the default security parameters.
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
    };
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max  = APP_ADV_INTERVAL_MAX;
    s_gap_adv_param.adv_intv_min  = APP_ADV_INTERVAL_MIN;
    s_gap_adv_param.adv_mode      = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map      = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode     = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol    = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    uint8_t dev_name[32];
    uint16_t dev_name_len = 32;

    error_code = ble_gap_device_name_get(dev_name, &dev_name_len);
    APP_ERROR_CHECK(error_code);

    if (!strcmp((const char *)dev_name, BLE_GAP_DEVNAME_DEFAULT))
    {
        // Set the default Device Name.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_UNAUTH, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        // Set the Device Name is writable from the peer.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_AUTH, NULL, 0);
        APP_ERROR_CHECK(error_code);
    }

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration     = APP_ADV_TIMEOUT;
    s_gap_adv_time_param.max_adv_evt  = 0;

    s_gap_conn_param.slave_latency = SLAVE_LATENCY;
    s_gap_conn_param.interval_min  = CONN_INTERVAL;
    s_gap_conn_param.interval_max  = CONN_INTERVAL;
    s_gap_conn_param.sup_timeout   = CONN_SUP_TIMEOUT;
}

/**
 *****************************************************************************************
 * @brief Initialize the sensor simulators.
 *****************************************************************************************
 */
static void sensor_simulator_init(void)
{
    s_battery_sim_cfg.min             = BATTERY_LEVEL_MIN;
    s_battery_sim_cfg.max             = BATTERY_LEVEL_MAX;
    s_battery_sim_cfg.incr            = BATTERY_LEVEL_INCREMENT;
    s_battery_sim_cfg.start_at_max    = true;
    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);

    s_weight_5g_sim_cfg.min           = WEIGHT_5G_MIN;
    s_weight_5g_sim_cfg.max           = WEIGHT_5G_MAX;
    s_weight_5g_sim_cfg.incr          = WEIGHT_5G_INCREMENT;
    s_weight_5g_sim_cfg.start_at_max  = true;
    sensorsim_init(&s_weight_5g_sim_state, &s_weight_5g_sim_cfg);

    s_fat_perc_sim_cfg.min            = FAT_PERC_MIN;
    s_fat_perc_sim_cfg.max            = FAT_PERC_MAX;
    s_fat_perc_sim_cfg.incr           = FAT_PERC_INCREMENT;
    s_fat_perc_sim_cfg.start_at_max   = true;
    sensorsim_init(&s_fat_perc_sim_state, &s_fat_perc_sim_cfg);

    s_water_perc_sim_cfg.min          = WATER_PERC_MIN;
    s_water_perc_sim_cfg.max          = WATER_PERC_MAX;
    s_water_perc_sim_cfg.incr         = WATER_PERC_INCREMENT;
    s_water_perc_sim_cfg.start_at_max = true;
    sensorsim_init(&s_water_perc_sim_state, &s_water_perc_sim_cfg);

    s_muscle_perc_sim_cfg.min            = MUSCLE_PERC_MIN;
    s_muscle_perc_sim_cfg.max            = MUSCLE_PERC_MAX;
    s_muscle_perc_sim_cfg.incr           = MUSCLE_PERC_INCREMENT;
    s_muscle_perc_sim_cfg.start_at_max   = true;

    s_bm_kcal_sim_cfg.min                = BM_KCAL_MIN;
    s_bm_kcal_sim_cfg.max                = BM_KCAL_MAX;     
    s_bm_kcal_sim_cfg.incr               = BM_KCAL_INCREMENT; 
    s_bm_kcal_sim_cfg.start_at_max       = true; 

    s_impedance_ohm_sim_cfg.min          = IMPEDANCE_OHM_MIN;
    s_impedance_ohm_sim_cfg.max          = IMPEDANCE_OHM_MAX;
    s_impedance_ohm_sim_cfg.incr         = IMPEDANCE_OHM_INCREMENT;
    s_impedance_ohm_sim_cfg.start_at_max = true;
}

/**
 *****************************************************************************************
 * @brief Perform battery measurement and updating the battery level in Battery Service.
 *****************************************************************************************
 */
static void battery_level_update(void *p_arg)
{
    sdk_err_t error_code;

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
 * @brief Perform Current Time update.
 *****************************************************************************************
 */
static void current_time_update(void *p_arg)
{
    s_current_exact_time.cur_time.day_date_time.date_time.sec++;
    if (60 == s_current_exact_time.cur_time.day_date_time.date_time.sec)
    {
        s_current_exact_time.cur_time.day_date_time.date_time.sec = 0;
        s_current_exact_time.cur_time.day_date_time.date_time.min++;
    }

    if (60 == s_current_exact_time.cur_time.day_date_time.date_time.min)
    {
        s_current_exact_time.cur_time.day_date_time.date_time.min = 0;
        s_current_exact_time.cur_time.day_date_time.date_time.hour++;
    }

    if (24 == s_current_exact_time.cur_time.day_date_time.date_time.hour)
    {
        s_current_exact_time.cur_time.day_date_time.date_time.hour = 0;
        s_current_exact_time.cur_time.day_date_time.date_time.day++;
        if (0 != s_current_exact_time.cur_time.day_date_time.day_of_week)
        {
            s_current_exact_time.cur_time.day_date_time.day_of_week++;
        }

        if (8 == s_current_exact_time.cur_time.day_date_time.day_of_week)
        {
            s_current_exact_time.cur_time.day_date_time.day_of_week = 1;
        }
    }

    if (32== s_current_exact_time.cur_time.day_date_time.date_time.day)
    {
        s_current_exact_time.cur_time.day_date_time.date_time.day = 1;
        s_current_exact_time.cur_time.day_date_time.date_time.month++;
    }

    if (13== s_current_exact_time.cur_time.day_date_time.date_time.month)
    {
        s_current_exact_time.cur_time.day_date_time.date_time.month = 1;
        s_current_exact_time.cur_time.day_date_time.date_time.year ++;
    }

    cts_exact_time_update(&s_current_exact_time);
}

/**
 *****************************************************************************************
 * @brief Perform Current Time update.
 *****************************************************************************************
 */
static void current_time_service_event_process(cts_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case CTS_EVT_CUR_TIME_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Current Time Notification is enabled.");
            break;

        case CTS_EVT_CUR_TIME_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Current Time Notification is disabled");
            break;

        case CTS_EVT_CUR_TIME_SET_BY_PEER:
            if(p_evt->length)
            {
              APP_LOG_DEBUG("Fractions_256:%d Adjust_reason:%d", p_evt->cur_time.day_date_time.fractions_256,
                                                                 p_evt->cur_time.adjust_reason);
              memcpy(&s_current_exact_time.cur_time,&p_evt->cur_time, sizeof(cts_cur_time_t));
            }
            break;

        case CTS_EVT_LOC_TIME_INFO_SET_BY_PEER:
            if(p_evt->length)
            {
              memcpy(&s_current_exact_time.loc_time_info,&p_evt->loc_time_info, sizeof(cts_loc_time_info_t));
              APP_LOG_DEBUG("Peer has set Local Time Information.");
              APP_LOG_DEBUG("Time Zone:%d, DST offset:%d", s_current_exact_time.loc_time_info.time_zone, s_current_exact_time.loc_time_info.dst_offset);
              time_adjust_dstutc();
            }
            break;

        default:
            break;
    }
}

void time_adjust_dstutc(void)
{
    int16_t du_time_min=0;
    int16_t now_time_min=0;
    int16_t total_time_min=0;
    // adjust time(min)
    if( s_current_exact_time.loc_time_info.time_zone == -128 )
    {
      du_time_min = DSTTUC_CONVERT_TO_HOURS*(0 + s_current_exact_time.loc_time_info.dst_offset);
    }
    else if( s_current_exact_time.loc_time_info.dst_offset == 0xff)
    {
      du_time_min = DSTTUC_CONVERT_TO_HOURS*(s_current_exact_time.loc_time_info.time_zone + 0 );
    }
    else
    {
      du_time_min = DSTTUC_CONVERT_TO_HOURS*(s_current_exact_time.loc_time_info.time_zone + s_current_exact_time.loc_time_info.dst_offset);
    }

    if( s_current_exact_time.loc_time_info.dst_offset == 0xff && s_current_exact_time.loc_time_info.time_zone == -128 )
    {
      du_time_min = DSTTUC_CONVERT_TO_HOURS*(0 + 0 );
    }

    // now time(min)
    now_time_min = s_current_exact_time.cur_time.day_date_time.date_time.min + s_current_exact_time.cur_time.day_date_time.date_time.hour*60  ;
    //total time(min)
    total_time_min=du_time_min+now_time_min;

    if( total_time_min >=0 )
    {
      s_current_exact_time.cur_time.day_date_time.date_time.hour = total_time_min/60;
      s_current_exact_time.cur_time.day_date_time.date_time.min  = total_time_min%60;

      if (60 < s_current_exact_time.cur_time.day_date_time.date_time.min)
      {
        s_current_exact_time.cur_time.day_date_time.date_time.min -= 60;
        s_current_exact_time.cur_time.day_date_time.date_time.hour++;
      }

      if (24 < s_current_exact_time.cur_time.day_date_time.date_time.hour)
      {
        s_current_exact_time.cur_time.day_date_time.date_time.hour -= 24;
        s_current_exact_time.cur_time.day_date_time.date_time.day++;

        if (32== s_current_exact_time.cur_time.day_date_time.date_time.day)
        {
          s_current_exact_time.cur_time.day_date_time.date_time.day = 1;
          s_current_exact_time.cur_time.day_date_time.date_time.month++;
        }

        if (13== s_current_exact_time.cur_time.day_date_time.date_time.month)
        {
          s_current_exact_time.cur_time.day_date_time.date_time.month = 1;
          s_current_exact_time.cur_time.day_date_time.date_time.year ++;
        }

        if (0 != s_current_exact_time.cur_time.day_date_time.day_of_week)
        {
          s_current_exact_time.cur_time.day_date_time.day_of_week++;
        }

        if (8 == s_current_exact_time.cur_time.day_date_time.day_of_week)
        {
          s_current_exact_time.cur_time.day_date_time.day_of_week = 1;
        }
      }
    }

    else
    {
        total_time_min += DAY_CONTAINS_MINUTES ;

        s_current_exact_time.cur_time.day_date_time.date_time.hour = total_time_min/60;
        s_current_exact_time.cur_time.day_date_time.date_time.min  = total_time_min%60;

        if ( s_current_exact_time.cur_time.day_date_time.date_time.day == 1)
        {
          s_current_exact_time.cur_time.day_date_time.date_time.day = 31;

          if ( s_current_exact_time.cur_time.day_date_time.date_time.month == 1)
          {
            s_current_exact_time.cur_time.day_date_time.date_time.month = 12;
            s_current_exact_time.cur_time.day_date_time.date_time.year --;
          }

          else
          {
           s_current_exact_time.cur_time.day_date_time.date_time.month --;
          }
        }

        else
        {
          s_current_exact_time.cur_time.day_date_time.date_time.day --;
        }
    }

    APP_LOG_DEBUG("adjust time_min=%dmin", du_time_min );
}

/**
 *****************************************************************************************
 * @brief Process Body Composition service event.
 *
 * @param[in] p_bcs_evt: Pointer to Body Composition service event.
 *****************************************************************************************
 */
static void bc_service_event_process(bcs_evt_t *p_bcs_evt)
{
    switch (p_bcs_evt->evt_type)
    {
        case BCS_EVT_MEAS_INDICATION_ENABLE:
            s_bc_meas_start = true;
            APP_LOG_DEBUG("Body Composition Measurement Indication is enabled.");
            break;

        case BCS_EVT_MEAS_INDICATION_DISABLE:
            s_bc_meas_start = false;
            APP_LOG_DEBUG("Body Composition Measurement Indication is disabled.");
            break;

        case BCS_EVT_MEAS_INDICATION_CPLT:
            break;

        case BCS_EVT_MEAS_READ_CHARACTERISTIC:
            break;
        
        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process Weight Scale service event.
 *
 * @param[in] p_wss_evt: Pointer to Weight Scale service event.
 *****************************************************************************************
 */
static void ws_service_event_process(wss_evt_t *p_wss_evt)
{   
    switch (p_wss_evt->evt_type)
    {
        case WSS_EVT_MEAS_INDICATION_ENABLE:
            s_ws_meas_start = true;
            APP_LOG_DEBUG("Weight Scale Measurement Indication is enabled.");
            break;

        case WSS_EVT_MEAS_INDICATION_DISABLE:
            s_ws_meas_start = false;
            APP_LOG_DEBUG("Weight Scale Measurement Indication is disabled.");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process User Data service event.
 *
 * @param[in] p_uds_evt: Pointer to User Data service event.
 *****************************************************************************************
 */
static void ud_service_event_process(uds_evt_t *p_uds_evt)
{
    switch (p_uds_evt->evt_type)
    {
        case UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_ENABLE:
            APP_LOG_DEBUG("Database Change Increment Notification is enabled.");
            break;

        case UDS_EVT_DB_CHANGE_INCR_NOTIFICATION_DISABLE:
            APP_LOG_DEBUG("Database Change Increment Notification is disabled.");
            break;

        case UDS_EVT_CTRL_POINT_INDICATION_ENABLE:
            APP_LOG_DEBUG("User Control Point Indication is enabled.");
            break;

        case UDS_EVT_CTRL_POINT_INDICATION_DISABLE:
            APP_LOG_DEBUG("User Control Point Indication is disabled.");
            break;

        case UDS_EVT_REGIST_USER_INDICATION_ENABLE:                  
            APP_LOG_DEBUG("Registered User Indication is enabled.");
            break;

        case UDS_EVT_REGIST_USER_INDICATION_DISABLE:
            APP_LOG_DEBUG("Registered User Indication is disabled.");
            break;

        case UDS_EVT_AGE_SET_BY_PEER:
            APP_LOG_DEBUG("Peer has set the Age Characteristic value.");
            break;

        case UDS_EVT_DATE_OF_BIRTH_SET_BY_PEER:
            APP_LOG_DEBUG("Peer has set the Date of Birth Characteristic value.");
            break;

        case UDS_EVT_FIRST_NAME_SET_BY_PEER:
            APP_LOG_DEBUG("Peer has set the First Name Characteristic value.");
            break;

        case UDS_EVT_HEIGHT_SET_BY_PEER:
            s_uds_height = BUILD_U16(p_uds_evt->p_data[0], p_uds_evt->p_data[1]);
            APP_LOG_DEBUG("Peer has set the Height Characteristic value.");
            break;

        case UDS_EVT_GENDER_SET_BY_PEER:
            APP_LOG_DEBUG("Peer has set the Gender Characteristic value.");
            break;

        case UDS_EVT_DB_CHANGE_INCR_SET_BY_PEER:
            APP_LOG_DEBUG("Peer has set the Database Change Increment value.");
            break;

        case UDS_EVT_CTRL_POINT_SET_BY_PEER:
            APP_LOG_DEBUG("Peer has set the User Control Point value.");
            break;

        case UDS_EVT_USER_GRANT_ACCESS:
            APP_LOG_DEBUG("User is waiting to be granted access.");
            s_cur_user_id = p_uds_evt->p_data[0];
            s_uds_height  = p_uds_evt->uds_chars_val.height;       
            break;

        case UDS_EVT_REGIST_NEW_USER:
            APP_LOG_DEBUG("Client is registering a new user.");
            s_cur_user_id = p_uds_evt->p_data[0];
            break;

        case UDS_EVT_DEL_USER_DATA:
            APP_LOG_DEBUG("Client is deleteing the user data if the current user.");
            s_cur_user_id = UDS_UNKNOWN_USER;
            break;

        case UDS_EVT_DEL_USERS:
            APP_LOG_DEBUG("Client is deleteing the indicated user or all users.");
            if (s_cur_user_id == p_uds_evt->p_data[0] || UDS_UNKNOWN_USER == p_uds_evt->p_data[0])
            {
                s_cur_user_id = UDS_UNKNOWN_USER;
            }
            break;
                
        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Perform Database Value update.
 *****************************************************************************************
 */
static void db_flag_update_handler(void *p_arg)
{
    db_wait_update = true;
}

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
    cts_init_t cts_env_init;
    wss_init_t wss_env_init;
    bcs_init_t bcs_env_init;
    uds_init_t uds_env_init;

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
    cts_env_init.char_mask                              = CTS_CHAR_FULL;
    cts_env_init.cur_time.day_date_time.date_time.year  = 2019;
    cts_env_init.cur_time.day_date_time.date_time.month = 2;
    cts_env_init.cur_time.day_date_time.date_time.day   = 26;
    cts_env_init.cur_time.day_date_time.date_time.hour  = 11;
    cts_env_init.cur_time.day_date_time.date_time.min   = 20;
    cts_env_init.cur_time.day_date_time.date_time.sec   = 0;
    cts_env_init.cur_time.day_date_time.day_of_week     = CTS_WEEK_TUSEDAY;
    cts_env_init.cur_time.day_date_time.fractions_256   = 0;
    cts_env_init.cur_time.adjust_reason                 = CTS_AR_NO_CHANGE;
    cts_env_init.loc_time_info.time_zone                = 0;
    cts_env_init.loc_time_info.dst_offset               = CTS_DST_OFFSET_STANDAR_TIME;
    cts_env_init.ref_time_info.source                   = CTS_REF_TIME_SRC_MANUAL;
    cts_env_init.ref_time_info.accuracy                 = CTS_TIME_ACCURACT_UNKNOWN;
    cts_env_init.ref_time_info.days_since_update        = 0;
    cts_env_init.ref_time_info.hours_since_update       = 0;
    cts_env_init.evt_handler                            = current_time_service_event_process;
    error_code = cts_service_init(&cts_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    uint16_t bcs_meas_flag = BCS_MEAS_FLAG_DEFAULT;
    memcpy(&(bcs_env_init.bcs_meas_flags), &bcs_meas_flag, sizeof(bcs_meas_flag_t));
    bcs_env_init.char_mask                                = BCS_CHAR_FEAT_MANDATORY;
    bcs_env_init.feature                                  = BCS_FEAT_FULL_BIT;
    bcs_env_init.bcs_unit                                 = BCS_UNIT_SI;
    bcs_env_init.bcs_mass_res                             = BCS_MASS_RES_5G;
    bcs_env_init.bcs_height_res                           = BCS_HEIGHT_RES_1MM;
    bcs_env_init.evt_handler                              = bc_service_event_process;   
    error_code = bcs_service_init(&bcs_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    uint8_t uds_meas_flag = UDS_MEAS_FLAG_DEFAULT;
    memcpy(&(uds_env_init.uds_chars_flag), &uds_meas_flag, sizeof(uds_chars_flag_t));
    uds_env_init.char_mask                                      = UDS_CHAR_FULL;
    uds_env_init.user_index                                     = UDS_UNKNOWN_USER;
    uds_env_init.db_change_incr_val                             = UDS_DB_CHANGE_INCR_DEFAULT_VAL;
    uds_env_init.uds_regi_user_data_flag.regi_user_name_present = 0x01;
    uds_env_init.uds_regi_user_data_flag.user_name_truncated    = 0x00;
    uds_env_init.evt_handler = ud_service_event_process;   
    error_code = uds_service_init(&uds_env_init);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    wss_env_init.char_mask          = WSS_CHAR_FEAT_MANDATORY;
    wss_env_init.feature            = WSS_FEAT_FULL_BIT;
    wss_env_init.multi_user_present = true;
    wss_env_init.time_stamp_present = true;
    wss_env_init.bmi_present        = true;
    wss_env_init.wss_unit           = WSS_UNIT_SI;
    wss_env_init.wss_mass_res       = WSS_MASS_RES_5G;
    wss_env_init.wss_height_res     = WSS_HEIGHT_RES_1MM;
    wss_env_init.evt_handler        = ws_service_event_process;
    error_code = wss_service_init(&wss_env_init, bcs_start_handle_get());
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

    error_code = app_timer_create(&s_db_timer_id, ATIMER_REPEAT, db_flag_update_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_cts_timer_id, ATIMER_REPEAT, current_time_update);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_battery_level_timer_id, ATIMER_REPEAT, battery_level_update);
    APP_ERROR_CHECK(error_code);
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t          tk;

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

        // User needs to input the password.
        case BLE_SEC_TK_REQ:
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept   = true;

            tk = 123456;    // 0x0001E240

            memset(cfm_enc.data.tk.key, 0, sizeof(cfm_enc.data.tk.key));
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;

        default:
            APP_LOG_DEBUG("Unsupported pairing method.");
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
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

    s_cur_user_id = UDS_UNKNOWN_USER;
    uds_set_cur_user_index(conn_idx, s_cur_user_id);
    
    cts_exact_time_get(&s_current_exact_time);
    
    error_code = app_timer_start(s_cts_timer_id, CURRENT_TIME_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
    
    error_code = app_timer_start(s_db_timer_id, DATABASE_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);

    ble_sec_enc_start(conn_idx);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    app_timer_stop(s_db_timer_id);    
    app_timer_stop(s_cts_timer_id);
    app_timer_stop(s_battery_level_timer_id);

    error_code = ble_gap_adv_start(conn_idx, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void db_value_update_schedule(void)
{
    if (UDS_UNKNOWN_USER != s_cur_user_id)
    {
        /* Update the measurement data. */
        meas_val_t     meas_val[2];
        double         body_water_percentage;

        s_wss_weight                                 = sensorsim_measure(&s_weight_5g_sim_state, &s_weight_5g_sim_cfg);
        meas_val[0].wss_meas_val.weight              = s_wss_weight;
        meas_val[0].wss_meas_val.height              = s_uds_height;
        meas_val[0].wss_meas_val.user_id             = s_cur_user_id;
        meas_val[0].wss_meas_val.time_stamp          = s_current_exact_time.cur_time.day_date_time.date_time;

        body_water_percentage                        = sensorsim_measure(&s_water_perc_sim_state, &s_water_perc_sim_cfg);
        meas_val[1].bcs_meas_val.body_fat_percentage = sensorsim_measure(&s_fat_perc_sim_state, &s_fat_perc_sim_cfg);
        meas_val[1].bcs_meas_val.basal_metabolism    = sensorsim_measure(&s_bm_kcal_sim_state, &s_bm_kcal_sim_cfg);
        meas_val[1].bcs_meas_val.muscle_percentage   = sensorsim_measure(&s_muscle_perc_sim_state, &s_muscle_perc_sim_cfg);
        meas_val[1].bcs_meas_val.muscle_mass         = s_wss_weight * meas_val[1].bcs_meas_val.muscle_percentage / 1000;
        meas_val[1].bcs_meas_val.fat_free_mass       = s_wss_weight - s_wss_weight * meas_val[1].bcs_meas_val.body_fat_percentage / 1000;
        meas_val[1].bcs_meas_val.body_water_mass     = s_wss_weight * body_water_percentage / 100;
        meas_val[1].bcs_meas_val.soft_lean_mass      = meas_val[1].bcs_meas_val.body_water_mass + 500;
        meas_val[1].bcs_meas_val.impedance           = sensorsim_measure(&s_impedance_ohm_sim_state, &s_impedance_ohm_sim_cfg);  
        meas_val[1].bcs_meas_val.user_id             = s_cur_user_id;
        meas_val[1].bcs_meas_val.time_stamp          = s_current_exact_time.cur_time.day_date_time.date_time;

        if (wss_db_record_meas_fifo_set(s_cur_user_id, &meas_val[0], WSS_DB_WSS_MEAS_TYPE))
        {
            bool status = wss_db_record_meas_fifo_set(s_cur_user_id, &meas_val[1], WSS_DB_BCS_MEAS_TYPE);
        }

        /* Get and encode the body composition measurement data, then send them. */
        if (s_bc_meas_start)
        {
            uint8_t cache_num;
            cache_num = wss_db_record_meas_fifo_get(s_cur_user_id, &s_meas_val[0], WSS_DB_BCS_MEAS_TYPE);
            for (uint8_t i = 0; i < cache_num; i++)
            {
                s_bcs_meas_val[i] = s_meas_val[i].bcs_meas_val;
            }

            bcs_measurement_send(0, s_bcs_meas_val, cache_num);
        }

        /* Get and encode the weight scale measurement data, then send them. */
        if (s_ws_meas_start)
        {
            uint8_t cache_num;
            cache_num = wss_db_record_meas_fifo_get(s_cur_user_id, &s_meas_val[0], WSS_DB_WSS_MEAS_TYPE);
            for (uint8_t i = 0; i < cache_num; i++)
            {
                s_wss_meas_val[i] = s_meas_val[i].wss_meas_val;
            }
            
            wss_measurement_send(0, s_wss_meas_val, cache_num);
        }
    }
    db_wait_update = false;
}

void ble_evt_handler(const ble_evt_t *p_evt)
{
    sdk_err_t error_code;

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

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;
            
        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                error_code = ble_gap_conn_param_update(0, &s_gap_conn_param);
                APP_ERROR_CHECK(error_code);
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
    APP_LOG_INFO("Weight Scale example started.");

    sensor_simulator_init();
    services_init();
    gap_params_init();
    app_timer_init();
    wss_db_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    db_wait_update      = false;
    s_bc_meas_start     = false;
    s_ws_meas_start     = false;
    s_uds_height        = 165;
    s_wss_weight        = 50;
}

