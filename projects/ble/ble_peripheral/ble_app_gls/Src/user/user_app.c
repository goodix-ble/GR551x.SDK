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
#include "gls.h"
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
/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_GLS"     /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                32              /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                48              /**< The advertising max interval (in units of 0.625 ms). */

/**@brief Battery sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL         2000            /**< Battery level measurement interval (in unit of 1 ms). */
#define BATTERY_LEVEL_MIN                   81              /**< Minimum simulated battery level. */
#define BATTERY_LEVEL_MAX                   100             /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1               /**< Increment between each simulated battery level measurement. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;                     /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;                /**< Advertising time parameter. */
static ble_sec_param_t          s_sec_param;                         /**< Security parameter. */

static app_timer_id_t           s_battery_level_timer_id;            /**< Battery measurement timer id. */
static sensorsim_cfg_t          s_battery_sim_cfg;                   /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t        s_battery_sim_state;                 /**< Battery Level sensor simulator state. */

static const uint8_t            s_adv_data_set[] =                   /**< Advertising data. */
{
    // service UUIDs
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_GLUCOSE_METER),
    HI_U16(BLE_APPEARANCE_GENERIC_GLUCOSE_METER),

    // Device Service UUID
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_GLUCOSE),
    HI_U16(BLE_ATT_SVC_GLUCOSE),
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
    0x02,
    0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                      /**< Scan responce data. */
{
    0x0B,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'G', 'L', 'S',
};

static dis_sys_id_t s_devinfo_system_id =                        /**< Device system id. */
{
    .manufacturer_id = {0x12, 0x34, 0x56, 0x78, 0x9A},           /**< The manufacturer-defined identifier. */
    .org_unique_id   = {0xBC, 0xDE, 0xF0}                        /**< DUMMY Organisation Unique ID (OUI),
                                                                      You shall use the OUI of your company. */
};

static char s_devinfo_model_number[]  = "Glucose-01";            /**< Device model number string. */
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
    .product_id       = 0x1234,                                  /**< Product ID (vendor-specific). */
    .product_version  = 0x0110                                   /**< Product version (JJ.M.N). */
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
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
}

/**
 *****************************************************************************************
 * @brief Perform battery measurement and updating the battery level in Battery Service.
 *****************************************************************************************
 */
static void battery_level_update(void *p_arg)
{
    uint8_t     battery_level;
    sdk_err_t   error_code;

    battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state, &s_battery_sim_cfg);
    error_code = bas_batt_lvl_update(0, 0, battery_level);
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
 *@brief Process glucose service event
 *****************************************************************************************
 */
static void glucose_service_event_process(gls_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case GLS_EVT_MEAS_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Glucose Measurement Notify Enabled.");
            break;

        case GLS_EVT_MEAS_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Glucose Measurement Notify Disabled.");
            break;

        case GLS_EVT_CTX_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Glucose Measurement Context Notify Enabled.");
            break;

        case GLS_EVT_CTX_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Glucose Measurement Context Notify Disabled.");
            break;

        case GLS_EVT_CTRL_INDICATION_ENABLED:
            APP_LOG_DEBUG("Record Access Control Point Indicate Enabled.");
            break;

        case GLS_EVT_CTRL_INDICATION_DISABLED:
            APP_LOG_DEBUG("Record Access Control Point Indicate Disabled.");
            break;

        default:
            break;
    }
}

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

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(900, true);
    APP_ERROR_CHECK(error_code);

#if defined(PTS_AUTO_TEST)
    s_sec_param.level     = BLE_SEC_MODE1_LEVEL2;
    s_sec_param.io_cap    = BLE_SEC_IO_DISPLAY_ONLY;
    s_sec_param.oob       = false;
    s_sec_param.auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_SEC_CON;
    s_sec_param.key_size  = 16;
    s_sec_param.ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY;
    s_sec_param.rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY;
#else
    s_sec_param.level     = BLE_SEC_MODE1_LEVEL3;
    s_sec_param.io_cap    = BLE_SEC_IO_DISPLAY_ONLY;
    s_sec_param.oob       = false;
    s_sec_param.auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON;
    s_sec_param.key_size  = 16;
    s_sec_param.ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY;
    s_sec_param.rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY;
#endif

    error_code = ble_sec_params_set(&s_sec_param);
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

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, 
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 *@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 *****************************************************************************************
 */
static void services_init(void)
{
    dis_init_t dis_env_init;
    bas_init_t bas_env_init[1];
    gls_init_t gls_env_init;
    sdk_err_t  error_code;

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
    bas_env_init[0].char_mask   = BAS_CHAR_MANDATORY | BAS_CHAR_LVL_NTF_SUP;
    bas_env_init[0].batt_lvl    = 0;
    bas_env_init[0].evt_handler = battery_service_event_process;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    gls_env_init.char_mask   = GLS_CHAR_MANDATORY;
    gls_env_init.feature     = GLS_FEAT_FULL;
    gls_env_init.evt_handler = glucose_service_event_process;
    error_code = gls_service_init(&gls_env_init);
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
    
    error_code = app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    app_timer_stop(s_battery_level_timer_id);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void glucose_measurement_excute(void)
{
    static bool    s_units_mol_l = true;
    static int16_t s_mantissa    = 550;
    static int16_t s_exponent    = -5;
    static uint8_t s_secs        = 0;
    gls_rec_t      gls_rec_meas;

    if (s_units_mol_l)
    {
        gls_rec_meas.meas_val.flags = GLS_MEAS_FLAG_TIME_OFFSET |
                                      GLS_MEAS_FLAG_CONC_TYPE_LOC |
                                      GLS_MEAS_FLAG_UNITS_MOL_L |
                                      GLS_MEAS_FLAG_SENSOR_STATUS;
    }
    else
    {
        gls_rec_meas.meas_val.flags = GLS_MEAS_FLAG_TIME_OFFSET |
                                      GLS_MEAS_FLAG_CONC_TYPE_LOC |
                                      GLS_MEAS_FLAG_UNITS_KG_L |
                                      GLS_MEAS_FLAG_SENSOR_STATUS;
    }

    s_units_mol_l = !s_units_mol_l;

    gls_rec_meas.meas_val.base_time.year                 = 2022;
    gls_rec_meas.meas_val.base_time.month                = 2;
    gls_rec_meas.meas_val.base_time.day                  = 19;
    gls_rec_meas.meas_val.base_time.hour                 = 11;
    gls_rec_meas.meas_val.base_time.min                  = 4;
    gls_rec_meas.meas_val.base_time.sec                  = s_secs;
    gls_rec_meas.meas_val.glucose_concentration.exponent = s_exponent;
    gls_rec_meas.meas_val.glucose_concentration.mantissa = s_mantissa;
    gls_rec_meas.meas_val.time_offset                    = 0;
    gls_rec_meas.meas_val.type                           = GLS_MEAS_TYPE_CAP_BLOOD;
    gls_rec_meas.meas_val.sample_location                = GLS_MEAS_LOC_FINGER;
    gls_rec_meas.meas_val.sensor_status_annunciation     = GLS_MEAS_STATUS_BATT_LOW;

    // Simulated next measurement.
    s_mantissa += 23;
    if (s_mantissa > 939)
    {
        s_mantissa -= 434;
    }

    s_secs += 3;
    if (s_secs > 59)
    {
        s_secs = 0;
        gls_rec_meas.meas_val.base_time.min++;
        if (gls_rec_meas.meas_val.base_time.min > 59)
        {
            gls_rec_meas.meas_val.base_time.min = 0;
        }
    }

    if (gls_new_meas_record(& gls_rec_meas))
    {
        APP_LOG_DEBUG("Glucose value measure and store successfully.");
    }
    else
    {
        APP_LOG_DEBUG("Glucose value store failed.");
    }
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

        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.params.adv_stop.reason == BLE_GAP_STOPPED_REASON_TIMEOUT)
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
    APP_LOG_INFO("Glucose Sensor example started.");

    sensor_simulator_init();
    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    // Load 3 glucose measure records.
    glucose_measurement_excute();
    glucose_measurement_excute();
    glucose_measurement_excute();
}
