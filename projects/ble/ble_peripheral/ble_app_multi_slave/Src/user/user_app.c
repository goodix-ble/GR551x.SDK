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
#include "grx_sys.h"
#include "sensorsim.h"
#include "dis.h"
#include "hrs.h"
#include "bas.h"
#include "hids.h"
#include "utility.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "board_SK.h"
#include "user_keyboard.h"
#include "app_uart.h"
#include "user_periph_setup.h"
#include "gr55xx_nvds.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "GR_Multi_Slave"       /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL          32                   /**< The fast advertising min interval (in units of 0.625 ms. This value corresponds to 160 ms). */
#define APP_ADV_FAST_MAX_INTERVAL          48                   /**< The fast advertising max interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL          160                  /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL          160                  /**< The slow advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0                    /**< The advertising timeout in units of seconds. */
#define MIN_CONN_INTERVAL                  400                  /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  650                  /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   4000                 /**< Connection supervisory timeout (4 seconds). */
#define ADV_FAST_DURATION                  18000                /**< The advertising timeout in units of 10ms. */
#define ADV_LOW_LATENCY_DURATION           128                  /**< The advertising low latency duration. */
#define ADV_HIGHER_LATENCY_MIN_INTERVAL    32                   /**< The advertising min interval (in units of 0.625 ms). */
#define ADV_HIGHER_LATENCY_MAX_INTERVAL    48                   /**< The advertising max interval (in units of 0.625 ms). */
#define ADV_HIGHER_LATENCY_DURATION        3000                 /**< The advertising timeout in units of 10ms. */
#define ADV_LOW_LATENCY_DURATION           128                  /**< The advertising low latency duration. */
#if defined (SWIFT_PAIR_SUPPORTED)
#define ADV_PERMANERT_MIN_INTERVAL          160                 /**< The advertising min interval 1s to 2.5s (in units of 0.625 ms). */
#define ADV_PERMANERT_MAX_INTERVAL          244                 /**< The advertising max interval 1s to 2.5s (in units of 0.625 ms). */
#else
#define ADV_PERMANERT_MIN_INTERVAL          1600                /**< The advertising min interval 1s to 2.5s (in units of 0.625 ms). */
#define ADV_PERMANERT_MAX_INTERVAL          4000                /**< The advertising max interval 1s to 2.5s (in units of 0.625 ms). */
#endif

/**@brief sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL        2000                 /**< Battery level measurement interval (in unit of 1 ms). */
#define MIN_BATTERY_LEVEL                  81                   /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                  100                  /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT            1                    /**< Increment between each simulated battery level measurement. */
#define HEART_RATE_MEAS_INTERVAL           1000                 /**< Heart rate measurement interval (in unit of 1 ms). */
#define MIN_HEART_RATE                     55                   /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                     300                  /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT               10                   /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */
#define RR_INTERVAL_INTERVAL               300                  /**< RR interval interval (in unit of 1 ms). */
#define MIN_RR_INTERVAL                    100                  /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                    500                  /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT              1                    /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */
#define MIN_HEART_RATE_ENGRY               0                    /**< Min heart rate engry. */
#define MAX_HEART_RATE_ENGRY               65535                /**< Max heart reat engty. */
#define HEART_RATE_ENGRY_INCREMENT         100                  /**< Heart rate engry increment. */
#define FAST_ADV_DURATION                  3000                 /**< Fast advertising duration. */

#define CONN_LINK_MAX                      CFG_MAX_CONNECTIONS  /**< Maximum number of connect links which are allowed. */

/**< macros for simulating hardware. */
#define BATTERY_LEVEL_MIN                   81
#define BATTERY_LEVEL_MAX                   100
#define BATTERY_LEVEL_INCREAMENT            1
#define HW_SIM_UPDATE_INTERVAL              2000

/**< macros for nvds. */
#define PEER_BD_ADDR_LEN                                           BLE_GAP_ADDR_LEN                                             /**< Length of Bluetoth Device Address. */
#define BONDING_PEER_BD_ADDR_PUT(ADV_IDX,BD_ADDR_ARRAY)            nvds_put(NV_TAG_APP(ADV_IDX), PEER_BD_ADDR_LEN, BD_ADDR_ARRAY)  /**< NVDS put BD address. */
#define BONDING_PEER_BD_ADDR_GET(ADV_IDX,GET_LEN,BD_ADDR_ARRAY)    nvds_get(NV_TAG_APP(ADV_IDX), GET_LEN, BD_ADDR_ARRAY)  /**< NVDS put BD address. */

enum
{
    APP_ADV_TYPE_FAST,
    APP_ADV_TYPE_LOW_LATENCY,
    APP_ADV_TYPE_HIGHER_LATENCY,
    APP_ADV_TYPE_PERMANENT,
};

/*
 * EXTERNAL SYMBOLS DEFINITIONS
 *****************************************************************************************
 */
extern void ble_gap_conn_local_addr_get(uint8_t conidx,uint8_t *p_addr);                  /**< Get the location mac address of the current connection. */
extern uint16_t ble_gatts_service_hide_set(uint8_t conn_idx, uint16_t handle);                 /**< Set whether the corresponding service is discovered according to the service handle. */
extern uint16_t ble_gatts_service_hide_clear(uint8_t conn_idx);                                /**< Clean the service hided setting. */

static void fast_adv_start(void);
static void low_latency_adv_start(ble_gap_bdaddr_t *p_peer_bdaddr);
static void higher_latency_adv_start(void);

/**
 *****************************************************************************************
 * @brief Dev management for multi slave.
 *****************************************************************************************
 */
typedef struct dev_mgr
{
    uint8_t                  conn_idx;                        /**< connection index. */
    uint8_t                  adv_idx;                         /**< Advertising index. */
    uint8_t                  adv_status;                      /**< Advertising status--0: adv stop  1:adv started. */
    ble_gap_adv_param_t      adv_params;                      /**< Advertising parameters for legay advertising. */
    ble_gap_adv_time_param_t adv_time_params;                 /**< Advertising time parameter. */
    ble_gap_bdaddr_t         adv_addr;                        /**< Advertising device address. */
    ble_gap_bdaddr_t         peer_bond_addr;                  /**< The bonded peer dev addr. */
    uint32_t                 pair_start_flag : 1;             /**< Pair started--0:waiting pair  1: pair start. */
    uint32_t                 pair_status_flag : 1;            /**< Pair status--0:unpair stutas  1: pair status. */
    uint32_t                 unused : 30;                     /**< bits unused. */
}multi_s_dev_management_t;

/**
 *****************************************************************************************
 * @brief Ble pair,evt,status and so on management.
 *****************************************************************************************
 */
typedef struct ble_perfomrance_mgr
{
    uint8_t                  current_handle_conn_idx;         /**< Current handle connection index. */
}ble_perfomrance_management_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t                 s_slave_hids_addr[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0xc0};
static const uint8_t                 s_slave_hrs_addr[6] = {0x22, 0x22, 0x22, 0x22, 0x22, 0xc0};

static multi_s_dev_management_t      s_multi_s_mgr[TOP_ADV_INDEX];
static ble_perfomrance_management_t  s_ble_performance_mgr;
static bool                          s_hrs_timer_set_flag  = false;
static bool                          s_bas_timer_set_flag  = false;

static uint8_t                       s_bas_cccd_set_nb = 0;
static uint8_t                       s_hrs_cccd_set_nb = 0;
static bool                          s_hrs_cccd_set[CFG_MAX_CONNECTIONS];
static bool                          s_bas_cccd_set[CFG_MAX_CONNECTIONS];

static const uint8_t s_adv_hrs_data_set[] =                         /**< Advertising data. */
{
    // Complete Name
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'H', 'R', 'S',

    // Device appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),
    HI_U16(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),

    // Device Services uuid
    0x05,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HEART_RATE),
    HI_U16(BLE_ATT_SVC_HEART_RATE),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
};

static const uint8_t s_adv_hrs_rsp_data_set[] =
{
    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02, 0x03,
};

const uint8_t invalid_bd_addr[BLE_GAP_ADDR_LEN] = {0};
const uint8_t invalid_nvds_data[BLE_GAP_ADDR_LEN] = {0xff};

static app_timer_id_t    s_battery_level_timer_id;
static app_timer_id_t    s_heart_rate_meas_timer_id;
static app_timer_id_t    s_rr_interval_meas_timer_id;
static sensorsim_cfg_t   s_battery_sim_cfg;             /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t s_battery_sim_state;           /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   s_heart_rate_sim_cfg;          /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t s_heart_rate_sim_state;        /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   s_rr_interval_sim_cfg;         /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t s_rr_interval_sim_state;       /**< RR Interval sensor simulator state. */

static uint16_t s_energy_expended;
static uint8_t  s_energy_cnt;
static uint8_t  s_battery_level;

/*
 * Hids LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

#if defined(SWIFT_PAIR_SUPPORTED)
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0A,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'K', 'B',

    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),

    0x06,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    LO_U16(MICROSOFT_VENDOR_ID),
    HI_U16(MICROSOFT_VENDOR_ID),
    MICROSOFT_BEACON_ID,
    MICROSOFT_BEACON_SUB_SCENARIO,
    RESERVED_RSSI_BYTE,
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};

static const uint8_t s_new_adv_data_set[] =
{
    0x0A,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'K', 'B',

    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),

    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};
#else
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0A,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'K', 'B',

    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),

    0x05,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};
#endif

//~!@#$%^&*()_+{}|:"<>?
//`-=[]\;',./
static const uint8_t symbol_ascii_tab[] =
{
    0x7E, 0x21, 0x40, 0x23, 0x24, 0x25, 0x5E, 0x26, 0x2A, 0x28, 0x29, 0x5F, 0x2B, 0x7B, 0x7D, 0x7C, 0x3A,
    0x22, 0x3C, 0x3E, 0x3F, 0x60, 0x2D, 0x3D, 0x5B, 0x5D, 0x5C, 0x3B, 0x27, 0x2C, 0x2E, 0x2F, 0x20, 0x7F,
};

static const uint8_t symbol_hid_tab[] =
{
    HID_KEYBOARD_GRV_ACCENT,HID_KEYBOARD_1,HID_KEYBOARD_2,HID_KEYBOARD_3,HID_KEYBOARD_4,HID_KEYBOARD_5,\
    HID_KEYBOARD_6,HID_KEYBOARD_7,HID_KEYBOARD_8,HID_KEYBOARD_9,HID_KEYBOARD_0,HID_KEYBOARD_MINUS,\
    HID_KEYBOARD_EQUAL,HID_KEYBOARD_LEFT_BRKT,HID_KEYBOARD_RIGHT_BRKT,HID_KEYBOARD_BACK_SLASH,HID_KEYBOARD_SEMI_COLON,\
    HID_KEYBOARD_SGL_QUOTE,HID_KEYBOARD_COMMA,HID_KEYBOARD_DOT, HID_KEYBOARD_FWD_SLASH,\

    HID_KEYBOARD_GRV_ACCENT,HID_KEYBOARD_MINUS,HID_KEYBOARD_EQUAL,HID_KEYBOARD_LEFT_BRKT,HID_KEYBOARD_RIGHT_BRKT,\
    HID_KEYBOARD_BACK_SLASH,HID_KEYBOARD_SEMI_COLON,HID_KEYBOARD_SGL_QUOTE,HID_KEYBOARD_COMMA,\
    HID_KEYBOARD_DOT,HID_KEYBOARD_FWD_SLASH,HID_KEYBOARD_SPACEBAR,HID_KEYBOARD_DELETE
};

static uint8_t          s_app_adv_type;
static ble_gap_bdaddr_t s_bonded_bdaddr;

/*
 * LOCAL FUNCTION DECLARATIONS
 *******************************************************************************
 */
static void low_latency_adv_start(ble_gap_bdaddr_t *p_peer_bdaddr);
static void higher_latency_adv_start(void);
static void fast_adv_start(void);
multi_s_dev_management_t *current_dev_entity_get(uint8_t conn_idx);
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
#define DEV_ENTITY(conn_idx) current_dev_entity_get(conn_idx)
/**
 *****************************************************************************************
 *@brief Connection info get
 *
 * @param[in] conn_idx: nex connection index.
 *
 * @return Link index of the corresponding device.
 *****************************************************************************************
 */
multi_s_dev_management_t *current_dev_entity_get(uint8_t conn_idx)
{
    if(conn_idx == s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx)
    {
        return &s_multi_s_mgr[HIDS_ADV_INDEX];
    }

    return &s_multi_s_mgr[HRS_ADV_INDEX];
}


/**
 *****************************************************************************************
 *@brief Function for initializing the sensor simulators.
 *****************************************************************************************
 */
static void sensor_simulator_init(void)
{
    s_battery_sim_cfg.min             = MIN_BATTERY_LEVEL;
    s_battery_sim_cfg.max             = MAX_BATTERY_LEVEL;
    s_battery_sim_cfg.incr            = BATTERY_LEVEL_INCREMENT;
    s_battery_sim_cfg.start_at_max    = true;

    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);

    s_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    s_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    s_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    s_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&s_heart_rate_sim_state, &s_heart_rate_sim_cfg);

    s_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    s_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    s_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    s_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);

    s_energy_expended = 0;
    s_energy_cnt = 0;
}

/**
 *****************************************************************************************
 *@brief Performe battery measurement and updating the Battery Level characteristic in Battery Service.
 *****************************************************************************************
 */
static void battery_level_update(void *p_arg)
{
    sdk_err_t   error_code;

    s_battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state, &s_battery_sim_cfg);

    for (uint8_t i = 0; i < CONN_LINK_MAX; i++)
    {
        if (s_bas_cccd_set[i])
        {
            error_code = bas_batt_lvl_update(i, 0, s_battery_level);
            if (SDK_ERR_NTF_DISABLED != error_code)
            {
                APP_ERROR_CHECK(error_code);
            }
        }
    }
}

/**
 *****************************************************************************************
 *@brief Update energy expended once every 10 heart rate measurements.
 *
 * @return the result of updating energy expended
 *****************************************************************************************
 */
static bool updated_energy_expended(void)
{
    bool update_energy;

    // If revicved reset cmd, send energy_expened=0 before add HEART_RATE_ENGRY_INCREMENT
    if (10 == s_energy_cnt)
    {
        hrs_energy_update(s_energy_expended);
        s_energy_cnt = 0;
        update_energy = true;
    }
    else
    {
        s_energy_cnt += 1;
        update_energy = false;
    }

    if (MAX_HEART_RATE_ENGRY - s_energy_expended > HEART_RATE_ENGRY_INCREMENT)
    {
        s_energy_expended += HEART_RATE_ENGRY_INCREMENT;
    }
    else
    {
        s_energy_expended = MAX_HEART_RATE_ENGRY;
    }

    return update_energy;
}

/**
 *****************************************************************************************
 *@brief Handle the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *****************************************************************************************
 */
static void heart_rate_meas_timeout_handler(void *p_arg)
{
    uint16_t       heart_rate;
    static uint8_t contact_chg_flag = 0xFF;
    bool           update_energy;
    sdk_err_t      error_code;

    heart_rate    = (uint16_t)sensorsim_measure(&s_heart_rate_sim_state, &s_heart_rate_sim_cfg);
    update_energy = updated_energy_expended();


    if (s_hrs_cccd_set[0])
    {
        error_code = hrs_heart_rate_measurement_send(s_multi_s_mgr[HRS_ADV_INDEX].conn_idx, heart_rate, update_energy);
        APP_ERROR_CHECK(error_code);
    }

    hrs_sensor_contact_detected_update(contact_chg_flag);
    contact_chg_flag = ~contact_chg_flag;
}

/**
 *****************************************************************************************
 *@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *****************************************************************************************
 */
static void rr_interval_timeout_handler(void *p_arg)
{
    uint16_t rr_interval;

    rr_interval = (uint16_t)sensorsim_measure(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);
    hrs_rr_interval_add(rr_interval);
    rr_interval = (uint16_t)sensorsim_measure(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);
    hrs_rr_interval_add(rr_interval);
    rr_interval = (uint16_t)sensorsim_measure(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);
    hrs_rr_interval_add(rr_interval);
}

/**
 *****************************************************************************************
 *@brief Process battery service event
 *****************************************************************************************
 */
static void battery_service_process_event(bas_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BAS_EVT_NOTIFICATION_ENABLED:
            s_bas_cccd_set[p_evt->conn_idx] = true;
            s_bas_cccd_set_nb++;
            break;

        case BAS_EVT_NOTIFICATION_DISABLED:
            s_bas_cccd_set[p_evt->conn_idx] = false;
            s_bas_cccd_set_nb--;
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Process heart service event
 *****************************************************************************************
 */
static void heartrate_service_process_event(hrs_evt_t *p_hrs_evt)
{
    sdk_err_t   error_code;

    switch (p_hrs_evt->evt_type)
    {
        case HRS_EVT_NOTIFICATION_ENABLED:
            s_hrs_cccd_set[p_hrs_evt->conn_idx] = true;
            s_hrs_cccd_set_nb++;
            if (!s_hrs_timer_set_flag)
            {
                error_code = app_timer_start(s_heart_rate_meas_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
                APP_ERROR_CHECK(error_code);

                error_code = app_timer_start(s_rr_interval_meas_timer_id, RR_INTERVAL_INTERVAL, NULL);
                APP_ERROR_CHECK(error_code);

                APP_LOG_DEBUG("Heart Rate timer start.");
                s_hrs_timer_set_flag = true;
            }
            break;

        case HRS_EVT_NOTIFICATION_DISABLED:
            s_hrs_cccd_set[p_hrs_evt->conn_idx] = false;
            s_hrs_cccd_set_nb--;
            if (0 == s_hrs_cccd_set_nb)
            {
                app_timer_stop(s_heart_rate_meas_timer_id);
                app_timer_stop(s_rr_interval_meas_timer_id);
                APP_LOG_DEBUG("Heart Rate Timer Stop");
                s_hrs_timer_set_flag = false;
            }
            break;

        case HRS_EVT_RESET_ENERGY_EXPENDED:
            s_energy_expended = 0;
            s_energy_cnt = 10;    // trigger sending m_energy_expended=0
            hrs_energy_update(0);
            APP_LOG_DEBUG("Heart energy expended reset");
            break;

        case HRS_EVT_READ_BODY_SEN_LOCATION:
            // Output log for PTS Automation.
            // The log must be same with the HRS/SEN/CR/BV-01-C's condition defined in hrs_config.xml.
            APP_LOG_DEBUG("Body Sensor Location: 0x%02x", HRS_SENS_LOC_FINGER);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 *          parameters of the device including the device name, appearance, and
 *          the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t        error_code;
    ble_gap_conn_param_t gap_conn_param;

    ble_gap_pair_enable(true);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE,
                                         (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    gap_conn_param.interval_min  = MIN_CONN_INTERVAL;
    gap_conn_param.interval_max  = MAX_CONN_INTERVAL;
    gap_conn_param.slave_latency = SLAVE_LATENCY;
    gap_conn_param.sup_timeout   = CONN_SUP_TIMEOUT;
    error_code = ble_gap_ppcp_set(&gap_conn_param);
    APP_ERROR_CHECK(error_code);

    //set the default security parameters.
#if defined(SWIFT_PAIR_SUPPORTED)
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
    };
#else
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL3,
        .io_cap    = BLE_SEC_IO_KEYBOARD_ONLY,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
        .rkey_dist = BLE_SEC_KDIST_ENCKEY | BLE_SEC_KDIST_IDKEY,
    };
#endif
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(150, true);
    APP_ERROR_CHECK(error_code);

    memcpy(s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr, s_slave_hids_addr,sizeof(s_slave_hids_addr));
}

/**
 *****************************************************************************************
 *@brief Initialize the hrs advertising parameters.
 *****************************************************************************************
 */
static void hrs_adv_params_init(void)
{
    sdk_err_t error_code;

    s_multi_s_mgr[HRS_ADV_INDEX].adv_params.adv_intv_max = APP_ADV_SLOW_MAX_INTERVAL;
    s_multi_s_mgr[HRS_ADV_INDEX].adv_params.adv_intv_min = APP_ADV_FAST_MIN_INTERVAL;
    s_multi_s_mgr[HRS_ADV_INDEX].adv_params.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_multi_s_mgr[HRS_ADV_INDEX].adv_params.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_multi_s_mgr[HRS_ADV_INDEX].adv_params.disc_mode    = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_multi_s_mgr[HRS_ADV_INDEX].adv_params.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_multi_s_mgr[HRS_ADV_INDEX].adv_time_params.max_adv_evt = 0;
    s_multi_s_mgr[HRS_ADV_INDEX].adv_time_params.duration    = 0;

    error_code = ble_gap_adv_data_set(HRS_ADV_INDEX, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_hrs_data_set, sizeof(s_adv_hrs_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(HRS_ADV_INDEX, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_hrs_rsp_data_set, sizeof(s_adv_hrs_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_param_set(HRS_ADV_INDEX, BLE_GAP_OWN_ADDR_STATIC, &s_multi_s_mgr[HRS_ADV_INDEX].adv_params);
    APP_ERROR_CHECK(error_code);
    
    s_multi_s_mgr[HRS_ADV_INDEX].conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    memcpy(s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr, s_slave_hrs_addr,sizeof(s_slave_hrs_addr));
}
/**
 *****************************************************************************************
 * @brief Reset the device bond info.
 *
 * @param[in] erase_bond: 1 erase bond info,0 do nothing.
 *****************************************************************************************
 */
static void bond_info_reset(bool erase_bond)
{
    sdk_err_t            error_code;

    if (erase_bond)
    {
        error_code = ble_gap_bond_devs_clear();
        APP_ERROR_CHECK(error_code);

        error_code = ble_gap_whitelist_clear();
        APP_ERROR_CHECK(error_code);
        
        s_multi_s_mgr[HIDS_ADV_INDEX].pair_start_flag = WAITING_PAIR;
        s_multi_s_mgr[HIDS_ADV_INDEX].pair_status_flag = UNPAIR_STATUS;
        memset(s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr, 0, BLE_GAP_ADDR_LEN);
        BONDING_PEER_BD_ADDR_PUT(HIDS_ADV_INDEX,invalid_bd_addr);

        s_multi_s_mgr[HRS_ADV_INDEX].pair_start_flag = WAITING_PAIR;
        s_multi_s_mgr[HRS_ADV_INDEX].pair_status_flag = UNPAIR_STATUS;
        memset(s_multi_s_mgr[HRS_ADV_INDEX].peer_bond_addr.gap_addr.addr, 0, BLE_GAP_ADDR_LEN);
        BONDING_PEER_BD_ADDR_PUT(HRS_ADV_INDEX,invalid_bd_addr);

        APP_LOG_DEBUG("Bonding and Whitelist are cleared");
    }
}
/**
 *****************************************************************************************
 *@brief Initialize the hids advertising parameters.
 *****************************************************************************************
 */
static void hids_adv_params_init(void)
{
    sdk_err_t            error_code;
    ble_gap_white_list_t whitelist;
    uint16_t             len_get = BLE_GAP_ADDR_LEN;
    uint8_t              peer_addr_get[BLE_GAP_ADDR_LEN];

    s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params.max_adv_evt = 0;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.chnl_map         = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.max_tx_pwr       = 0;

    error_code = ble_gap_whitelist_get(&whitelist);
    APP_ERROR_CHECK(error_code);

    /* Initiate connection procedure for a bonded device */
    error_code = BONDING_PEER_BD_ADDR_GET(HIDS_ADV_INDEX,&len_get,peer_addr_get);
    APP_LOG_DEBUG("BONDING_PEER_BD_ADDR_GET %d",error_code);
    if(error_code == BLE_SUCCESS)
    {
         uint32_t i = 0;
         for(i=0; i<whitelist.num; i++)
         {
             if(memcmp(whitelist.items[i].gap_addr.addr,peer_addr_get,BLE_GAP_ADDR_LEN) == 0)
             {
                 s_bonded_bdaddr = whitelist.items[i];
                 s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr = whitelist.items[i];
                 APP_LOG_DEBUG("white liste dev-%02X:%02X:%02X:%02X:%02X:%02X",
                               s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr[5],
                               s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr[4],
                               s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr[3],
                               s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr[2],
                               s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr[1],
                               s_multi_s_mgr[HIDS_ADV_INDEX].peer_bond_addr.gap_addr.addr[0]);
                 low_latency_adv_start(&s_bonded_bdaddr);
                 return;
             }
         }
    }

    /* Initiate connection procedure for Non-bonded devices */
    s_bonded_bdaddr.addr_type = 0xFF;   /* Invalid address type. */
    fast_adv_start();
}
/**
 *****************************************************************************************
 *@brief Start the hids fast advertising.
 *****************************************************************************************
 */
static void fast_adv_start(void)
{
    sdk_err_t   error_code;

    memset(&s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.peer_addr, 0, sizeof(ble_gap_bdaddr_t));
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.disc_mode  = BLE_GAP_DISC_MODE_LIM_DISCOVERABLE;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_mode   = BLE_GAP_ADV_TYPE_ADV_IND;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.filter_pol = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_intv_max = APP_ADV_FAST_MAX_INTERVAL;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_intv_min = APP_ADV_FAST_MIN_INTERVAL;

    error_code = ble_gap_adv_data_set(HIDS_ADV_INDEX, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

#if defined(SWIFT_PAIR_SUPPORTED)
    error_code = ble_gap_adv_data_set(HIDS_ADV_INDEX, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_adv_param_set(HIDS_ADV_INDEX, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_multi_s_mgr[HIDS_ADV_INDEX].adv_params);
    APP_ERROR_CHECK(error_code);

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params.duration = ADV_FAST_DURATION;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    error_code = ble_gap_addr_set(&s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(HIDS_ADV_INDEX, &s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_FAST;
    APP_LOG_DEBUG("Starting fast advertising");
}
/**
 *****************************************************************************************
 *@brief Start the hids low latency advertising.
 *****************************************************************************************
 */
static void low_latency_adv_start(ble_gap_bdaddr_t *p_peer_bdaddr)
{
    sdk_err_t   error_code;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.disc_mode  = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_mode   = BLE_GAP_ADV_TYPE_ADV_HIGH_DIRECT_IND;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.filter_pol = BLE_GAP_ADV_ALLOW_SCAN_WLST_CON_WLST;

    memcpy(&s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.peer_addr, p_peer_bdaddr, sizeof(ble_gap_bdaddr_t));

    error_code = ble_gap_adv_param_set(HIDS_ADV_INDEX, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_multi_s_mgr[HIDS_ADV_INDEX].adv_params);
    APP_ERROR_CHECK(error_code);

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params.duration = ADV_LOW_LATENCY_DURATION;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    error_code = ble_gap_addr_set(&s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(HIDS_ADV_INDEX, &s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_LOW_LATENCY;
    APP_LOG_DEBUG("Starting high direct advertising to %02X:%02X:%02X:%02X:%02X:%02X",
                  p_peer_bdaddr->gap_addr.addr[5],
                  p_peer_bdaddr->gap_addr.addr[4],
                  p_peer_bdaddr->gap_addr.addr[3],
                  p_peer_bdaddr->gap_addr.addr[2],
                  p_peer_bdaddr->gap_addr.addr[1],
                  p_peer_bdaddr->gap_addr.addr[0]);
}
/**
 *****************************************************************************************
 *@brief Start the hids high latency advertising.
 *****************************************************************************************
 */
static void higher_latency_adv_start(void)
{
    sdk_err_t error_code;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.disc_mode  = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_mode   = BLE_GAP_ADV_TYPE_ADV_IND;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.filter_pol = BLE_GAP_ADV_ALLOW_SCAN_WLST_CON_WLST;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_intv_min = ADV_HIGHER_LATENCY_MIN_INTERVAL;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_intv_max = ADV_HIGHER_LATENCY_MAX_INTERVAL;

    error_code = ble_gap_adv_data_set(HIDS_ADV_INDEX, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

#if defined(SWIFT_PAIR_SUPPORTED)
    error_code = ble_gap_adv_data_set(HIDS_ADV_INDEX, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_adv_param_set(HIDS_ADV_INDEX, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_multi_s_mgr[HIDS_ADV_INDEX].adv_params);
    APP_ERROR_CHECK(error_code);

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    error_code = ble_gap_addr_set(&s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr);
    APP_ERROR_CHECK(error_code);

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params.duration = ADV_HIGHER_LATENCY_DURATION;

    error_code = ble_gap_adv_start(HIDS_ADV_INDEX, &s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_HIGHER_LATENCY;
    APP_LOG_DEBUG("Starting higher latency advertising");
}
/**
 *****************************************************************************************
 *@brief Start the hids permanent advertising.
 *****************************************************************************************
 */
static void hids_permanent_adv_start(void)
{
    sdk_err_t   error_code;

    memset(&s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.peer_addr, 0, sizeof(ble_gap_bdaddr_t));
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.disc_mode  = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_mode   = BLE_GAP_ADV_TYPE_ADV_IND;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.filter_pol = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_intv_min = ADV_PERMANERT_MIN_INTERVAL;
    s_multi_s_mgr[HIDS_ADV_INDEX].adv_params.adv_intv_max = ADV_PERMANERT_MAX_INTERVAL;

#if defined(SWIFT_PAIR_SUPPORTED)
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_new_adv_data_set, sizeof(s_new_adv_data_set));
    APP_ERROR_CHECK(error_code);
#else
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_multi_s_mgr[HIDS_ADV_INDEX].adv_params);
    APP_ERROR_CHECK(error_code);

    s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params.duration = 0;

    error_code = ble_gap_adv_start(0, &s_multi_s_mgr[HIDS_ADV_INDEX].adv_time_params);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_PERMANENT;
    APP_LOG_DEBUG("Starting permanent advertising");
}
/**
 *****************************************************************************************
 *@brief Start the hrs advertising.
 *****************************************************************************************
 */
static void hrs_advertising_start(void)
{
    sdk_err_t error_code;

    s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    error_code = ble_gap_addr_set(&s_multi_s_mgr[HRS_ADV_INDEX].adv_addr);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(HRS_ADV_INDEX, &s_multi_s_mgr[HRS_ADV_INDEX].adv_time_params);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t  error_code;
    bas_init_t bas_env_init[1];
    hrs_init_t hrs_init;

    /*------------------------------------------------------------------*/
    bas_env_init[0].char_mask   = BAS_CHAR_MANDATORY | BAS_CHAR_LVL_NTF_SUP;
    bas_env_init[0].batt_lvl    = 0;
    bas_env_init[0].evt_handler = battery_service_process_event;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);

    /*------------------------------------------------------------------*/
    hrs_init.sensor_loc                      = HRS_SENS_LOC_FINGER;
    hrs_init.char_mask                       = HRS_CHAR_MANDATORY |
                                               HRS_CHAR_BODY_SENSOR_LOC_SUP |
                                               HRS_CHAR_ENGY_EXP_SUP;
    hrs_init.evt_handler                     = heartrate_service_process_event;
    hrs_init.is_sensor_contact_supported     = true;
    error_code = hrs_service_init(&hrs_init);
    APP_ERROR_CHECK(error_code);

    user_keyboard_service_init();
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
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
            ble_sec_enc_cfm(conn_idx, &cfm_enc);
            break;

#if !defined(SWIFT_PAIR_SUPPORTED)
        // User needs to input the password.
        case BLE_SEC_TK_REQ:
            DEV_ENTITY(conn_idx)->pair_start_flag = START_PAIR;
            APP_LOG_INFO("Please input password");
            break;
#endif

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief App timer init
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t error_code;

    error_code = app_timer_create(&s_heart_rate_meas_timer_id, ATIMER_REPEAT, heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_rr_interval_meas_timer_id, ATIMER_REPEAT, rr_interval_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_battery_level_timer_id, ATIMER_REPEAT, battery_level_update);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Connected handler callback
 *
 * @param[in] conn_idx: new connection index.
 * @param[in] p_param : connection params.
 *****************************************************************************************
 */
static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;
    uint8_t local_con_addr[6];
    ble_gap_conn_local_addr_get(conn_idx,local_con_addr);

    if(memcmp(s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr,local_con_addr,6) == 0)
    {
        s_multi_s_mgr[HRS_ADV_INDEX].conn_idx = conn_idx;
        ble_gatts_service_hide_clear(conn_idx);
        ble_gatts_service_hide_set(conn_idx, dis_service_start_handle_get());
        ble_gatts_service_hide_set(conn_idx, hids_service_start_handle_get());
        APP_LOG_INFO("dev hrs connected %d\r\n",conn_idx);
    }

    if(memcmp(s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr,local_con_addr,6) == 0)
    {
        s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx = conn_idx;
        ble_gatts_service_hide_clear(conn_idx);
        ble_gatts_service_hide_set(conn_idx, bas_service_start_handle_get());
        ble_gatts_service_hide_set(conn_idx, hrs_service_start_handle_get());
        APP_LOG_INFO("dev hids connected %d\r\n",conn_idx);
    }

    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                     p_param->peer_addr.addr[5],
                     p_param->peer_addr.addr[4],
                     p_param->peer_addr.addr[3],
                     p_param->peer_addr.addr[2],
                     p_param->peer_addr.addr[1],
                     p_param->peer_addr.addr[0]);

    if (!s_bas_timer_set_flag)
    {
        error_code = app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
        APP_ERROR_CHECK(error_code);
        s_bas_timer_set_flag = true;
    }
}

/**
 *****************************************************************************************
 * @brief Disconnected handler callback
 *
 * @param[in] conn_idx: new connection index.
 * @param[in] reason  : the device disconnected reason.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    APP_LOG_INFO("Disconnected (0x%02X), Link IDX:%d.", reason, conn_idx);

    s_hrs_cccd_set[conn_idx] = false;
    s_bas_cccd_set[conn_idx] = false;

    if (conn_idx == s_multi_s_mgr[HRS_ADV_INDEX].conn_idx)
    {
        app_timer_stop(s_heart_rate_meas_timer_id);
        app_timer_stop(s_rr_interval_meas_timer_id);
        app_timer_stop(s_battery_level_timer_id);
        DEV_ENTITY(conn_idx)->pair_status_flag = UNPAIR_STATUS;
        DEV_ENTITY(conn_idx)->conn_idx = BLE_GAP_INVALID_CONN_INDEX;
        s_hrs_timer_set_flag = false;
        s_bas_timer_set_flag = false;

        hrs_advertising_start();
    }

    if (conn_idx == s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx)
    {
        //todo
        APP_LOG_INFO("disconnect pair status %d.", DEV_ENTITY(conn_idx)->pair_status_flag);

        if (DEV_ENTITY(conn_idx)->pair_status_flag == PAIRED_STATUS)
        {
            low_latency_adv_start(&DEV_ENTITY(conn_idx)->peer_bond_addr);
        }
        else
        {
            fast_adv_start();
        }

        DEV_ENTITY(conn_idx)->pair_status_flag = UNPAIR_STATUS;
        DEV_ENTITY(conn_idx)->conn_idx = BLE_GAP_INVALID_CONN_INDEX;
    }
}

/**
 *****************************************************************************************
 * @brief Pair success handler callback
 *****************************************************************************************
 */
static void app_paring_succeed_handler(void)
{
    ble_gap_privacy_mode_set(s_bonded_bdaddr, BLE_GAP_PRIVACY_MODE_DEVICE);
}

/**
 *****************************************************************************************
 * @brief Pair fail handler callback
 *****************************************************************************************
 */
static void app_paring_failed_handler(uint8_t enc_ind)
{
    /* How to handle the pairing errors is highly application dependent.
     * It usually should not be restarted for security reasons. But it is
     * possible for changing some security parameters or the peer not support.
     */
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    static bool adv_power_on = true;
    if(p_evt == NULL)
    {
        return;
    }

    s_ble_performance_mgr.current_handle_conn_idx = p_evt->evt.gapm_evt.index;

    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            APP_LOG_DEBUG("Adverting started index %d",p_evt->evt.gapm_evt.index);
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Adverting success");
                if((adv_power_on) && (p_evt->evt.gapm_evt.index == HIDS_ADV_INDEX))
                {
                    adv_power_on = 0;
                    hrs_advertising_start();
                }
            }
            else
            {
                APP_LOG_DEBUG("Adverting failed(0x%02x).", p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            APP_LOG_DEBUG("Advertising stop status %d,reason %d.",p_evt->evt.gapm_evt.index,p_evt->evt.gapm_evt.params.adv_stop.reason);
            if (p_evt->evt.gapm_evt.params.adv_stop.reason == BLE_GAP_STOPPED_REASON_TIMEOUT)
            {
                switch (s_app_adv_type)
                {
                    case APP_ADV_TYPE_LOW_LATENCY:
                        higher_latency_adv_start();
                        break;

                    case APP_ADV_TYPE_HIGHER_LATENCY:
                        hids_permanent_adv_start();
                        break;

                    default:
                        APP_LOG_DEBUG("Advertising timeout %d.",p_evt->evt.gapm_evt.index);
                        break;
                }
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

        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("Connection update completed, intvl %dx1.25ms, ltcy %d, to %dms",
                             p_evt->evt.gapc_evt.params.conn_param_updated.conn_interval,
                             p_evt->evt.gapc_evt.params.conn_param_updated.slave_latency,
                             p_evt->evt.gapc_evt.params.conn_param_updated.sup_timeout * 10);
            }
            break;

        case BLE_GAPM_EVT_SCAN_REQUEST:
            APP_LOG_DEBUG("Received the scan request from the peer %02X:%02X:%02X:%02X:%02X:%02X",
                           p_evt->evt.gapm_evt.params.scan_req.peer_addr.gap_addr.addr[5],
                           p_evt->evt.gapm_evt.params.scan_req.peer_addr.gap_addr.addr[4],
                           p_evt->evt.gapm_evt.params.scan_req.peer_addr.gap_addr.addr[3],
                           p_evt->evt.gapm_evt.params.scan_req.peer_addr.gap_addr.addr[2],
                           p_evt->evt.gapm_evt.params.scan_req.peer_addr.gap_addr.addr[1],
                           p_evt->evt.gapm_evt.params.scan_req.peer_addr.gap_addr.addr[0]);
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_DEBUG("Link has been successfully encrypted.");
                DEV_ENTITY(p_evt->evt.sec_evt.index)->pair_status_flag = PAIRED_STATUS;
                app_paring_succeed_handler();
            }
            else
            {
                APP_LOG_DEBUG("Pairing failed for error 0x%x.", p_evt->evt_status);
                app_paring_failed_handler(p_evt->evt_status);
            }
            break;
    }
}

void pair_input_handler(app_uart_evt_t *p_rec_line)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t          pair_code = 0x00;

    if (DEV_ENTITY(s_ble_performance_mgr.current_handle_conn_idx)->pair_start_flag == START_PAIR)// start pairing
    {
        pair_code += (uart_rec()[0] - 0x30) * 100000;
        pair_code += (uart_rec()[1] - 0x30) * 10000;
        pair_code += (uart_rec()[2] - 0x30) * 1000;
        pair_code += (uart_rec()[3] - 0x30) * 100;
        pair_code += (uart_rec()[4] - 0x30) * 10;
        pair_code += (uart_rec()[5] - 0x30);
        APP_LOG_INFO("Received Pass code: %d", pair_code);

        memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));
        cfm_enc.req_type = BLE_SEC_TK_REQ;
        cfm_enc.accept   = true;
        memset(cfm_enc.data.tk.key, 0, 16);
        cfm_enc.data.tk.key[0] = (uint8_t)((pair_code & 0x000000FF) >> 0);
        cfm_enc.data.tk.key[1] = (uint8_t)((pair_code & 0x0000FF00) >> 8);
        cfm_enc.data.tk.key[2] = (uint8_t)((pair_code & 0x00FF0000) >> 16);
        cfm_enc.data.tk.key[3] = (uint8_t)((pair_code & 0xFF000000) >> 24);
        ble_sec_enc_cfm(s_ble_performance_mgr.current_handle_conn_idx, &cfm_enc);

        DEV_ENTITY(s_ble_performance_mgr.current_handle_conn_idx)->pair_start_flag = WAITING_PAIR;
    }

    if (DEV_ENTITY(s_ble_performance_mgr.current_handle_conn_idx)->pair_status_flag == PAIRED_STATUS && 
        s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx != BLE_GAP_INVALID_CONN_INDEX)//paired successful
    {
        keyboard_keys_data_t data;
        memset(&data, HID_KEYBOARD_RESERVED, sizeof(keyboard_keys_data_t));

        if (uart_rec()[0] >='0' && uart_rec()[0]<= '9')// 0-9
        {
            if (uart_rec()[0] =='0')
            {
                data.key_code[0] = HID_KEYBOARD_0;
            }
            else
            {
                data.key_code[0] = (uart_rec()[0]-'1') + HID_KEYBOARD_1;
            }
        }
        else if (uart_rec()[0] >='A' && uart_rec()[0]<= 'Z')// A-Z
        {
            data.key_code[0] = (uart_rec()[0]-0x41) + HID_KEYBOARD_A;
            data.left_shift = 0x01;
        }
        else if (uart_rec()[0] >='a' && uart_rec()[0]<= 'z')// a-z
        {
            data.key_code[0] = (uart_rec()[0]-0x61) + HID_KEYBOARD_A;
        }
        else if (uart_rec()[0] == '\r')//enter key
        {
            data.key_code[0] = HID_KEYBOARD_RETURN;
        }
        else
        {
            for (uint8_t i=0; i < sizeof(symbol_ascii_tab); i++)
            {
                if (uart_rec()[0] == symbol_ascii_tab[i])
                {
                    data.key_code[0] = symbol_hid_tab[i];
                    if (i < 21)
                    {
                        data.left_shift = 0x01;
                    }
                    break;
                }
            }
        }
        user_keyboard_keys_send_data(s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx, &data);
        memset(&data, HID_KEYBOARD_RESERVED, sizeof(keyboard_keys_data_t));
        user_keyboard_keys_send_data(s_multi_s_mgr[HIDS_ADV_INDEX].conn_idx, &data);
    }
}

void ble_app_init(void)
{
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    sensor_simulator_init();
    services_init();
    gap_params_init();
    hrs_adv_params_init();
    bond_info_reset(ble_bond_state_get());
    hids_adv_params_init();
    
    APP_LOG_INFO("Local hids Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr[5],
                 s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr[4],
                 s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr[3],
                 s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr[2],
                 s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr[1],
                 s_multi_s_mgr[HIDS_ADV_INDEX].adv_addr.gap_addr.addr[0]);

    APP_LOG_INFO("Local hrs Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr[5],
                 s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr[4],
                 s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr[3],
                 s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr[2],
                 s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr[1],
                 s_multi_s_mgr[HRS_ADV_INDEX].adv_addr.gap_addr.addr[0]);

    app_timer_init();
}

