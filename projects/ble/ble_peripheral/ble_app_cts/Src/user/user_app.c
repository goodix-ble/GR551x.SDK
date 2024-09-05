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
#include "cts.h"
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
#define DEVICE_NAME                         "Goodix_CTS"      /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                48                /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                160               /**< The advertising max interval (in units of 0.625 ms). */

#define CURRENT_TIME_UPDATE_INTERVAL        1000              /**< Current Time Update interval (in uint of 1 ms). */

#define DSTTUC_CONVERT_TO_HOURS             15                /**< DST and UTC are converted to hours */
#define DAY_CONTAINS_MINUTES                1440              /**< A day contains minutes */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t        s_gap_adv_param;              /**< Advertising parameters for legay advertising. */
static ble_gap_adv_time_param_t   s_gap_adv_time_param;         /**< Advertising time parameter. */
static app_timer_id_t             s_cts_timer_id;               /**< Current time timer id. */
static cts_init_t                 s_current_exact_time;         /**< Current Exact Time value. */
static char *str[8] = { "Unknown_day", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

static const uint8_t s_adv_data_set[] =                       /**< Advertising data. */
{
    // Device Service UUID
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_CURRENT_TIME),
    HI_U16(BLE_ATT_SVC_CURRENT_TIME),
    LO_U16(BLE_ATT_SVC_REF_TIME_UPDATE),
    HI_U16(BLE_ATT_SVC_REF_TIME_UPDATE),
    LO_U16(BLE_ATT_SVC_NEXT_DST_CHANGE),
    HI_U16(BLE_ATT_SVC_NEXT_DST_CHANGE),

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix Company ID:04F7
    0xF7,
    0x04,
    0x02,
    0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                  /**< Scan responce data. */
{
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'C', 'T', 'S',
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

    if (s_current_exact_time.cur_time.day_date_time.date_time.year <= 9999)
    {
        cts_exact_time_update(&s_current_exact_time);
    }
}

/**
 *****************************************************************************************
 * @brief Process Current Time service event.
 *
 * @param[in] p_evt: Pointer to Current Time service event.
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
              APP_LOG_DEBUG("%d/%d/%d %s %d:%d:%d",  p_evt->cur_time.day_date_time.date_time.year,
                                                     p_evt->cur_time.day_date_time.date_time.month,
                                                     p_evt->cur_time.day_date_time.date_time.day,
                                                     str[p_evt->cur_time.day_date_time.day_of_week],
                                                     p_evt->cur_time.day_date_time.date_time.hour,
                                                     p_evt->cur_time.day_date_time.date_time.min,
                                                     p_evt->cur_time.day_date_time.date_time.sec);
              UNUSED_VARIABLE(str[0]);
              APP_LOG_DEBUG("Fractions_256:%d Adjust_reason:%d \r\n", p_evt->cur_time.day_date_time.fractions_256,
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
 * @brief Initialize services that will be used by the application.
 *
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t   error_code;
    cts_init_t  cts_init;

    /*------------------------------------------------------------------*/
    cts_init.char_mask                              = CTS_CHAR_FULL;
    cts_init.cur_time.day_date_time.date_time.year  = 2019;
    cts_init.cur_time.day_date_time.date_time.month = 2;
    cts_init.cur_time.day_date_time.date_time.day   = 26;
    cts_init.cur_time.day_date_time.date_time.hour  = 11;
    cts_init.cur_time.day_date_time.date_time.min   = 20;
    cts_init.cur_time.day_date_time.date_time.sec   = 0;
    cts_init.cur_time.day_date_time.day_of_week     = CTS_WEEK_TUSEDAY;
    cts_init.cur_time.day_date_time.fractions_256   = 0;
    cts_init.cur_time.adjust_reason                 = CTS_AR_NO_CHANGE;
    cts_init.loc_time_info.time_zone                = 0;
    cts_init.loc_time_info.dst_offset               = CTS_DST_OFFSET_STANDAR_TIME;
    cts_init.ref_time_info.source                   = CTS_REF_TIME_SRC_GPS;
    cts_init.ref_time_info.accuracy                 = CTS_TIME_ACCURACT_UNKNOWN;
    cts_init.ref_time_info.days_since_update        = 0;
    cts_init.ref_time_info.hours_since_update       = 0;
    cts_init.evt_handler                            = current_time_service_event_process;
    error_code = cts_service_init(&cts_init);
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

    error_code = app_timer_create(&s_cts_timer_id, ATIMER_REPEAT, current_time_update);
    APP_ERROR_CHECK(error_code);
}

static void app_connected_handler(const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);

    cts_exact_time_get(&s_current_exact_time);
    error_code = app_timer_start(s_cts_timer_id, CURRENT_TIME_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
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
            app_connected_handler(&(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            APP_LOG_INFO("Disconnected (0x%02X).", p_evt->evt.gapc_evt.params.disconnected.reason);
            app_timer_stop(s_cts_timer_id);
            error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
            APP_ERROR_CHECK(error_code);
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
    APP_LOG_INFO("Current Time example started.");

    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}
