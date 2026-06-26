/**
 *****************************************************************************************
 *
 * @file heartrate_slave.c
 *
 * @brief.
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
#include "heartrate_slave.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

#include "hrs.h"

#include "utility.h"


/*
 * DEFINES
 *****************************************************************************************
 */


#define HEART_RATE_MEAS_INTERVAL           1000
#define APP_ADV_INTERVAL                   32

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t           s_heart_rate_meas_timer_id;
static ble_gap_adv_param_t      s_hrs_slave_adv_param;
static ble_gap_adv_time_param_t s_hrs_slave_adv_time_param;
static uint8_t                  s_hrs_slave_addr[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0xc0};
static uint8_t                  s_hrs_slave_adv_idx = 0x00;
static bool                     s_hrs_slave_adv_cplt;
static uint8_t                  s_hrs_slave_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
static uint8_t                  s_hrs_slave_adv_data[] = {
    // Complete Name
    0x0a,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'H', 'R', 'M', '_', 'S', 'L', 'A', 'V', 'E',

    // Device appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),
    HI_U16(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),

    // Device Services uuid
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HEART_RATE),
    HI_U16(BLE_ATT_SVC_HEART_RATE),
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void heartrate_meas_timeout_handler(void *p_arg) {
    static uint16_t heart_rate = 80;

    if (BLE_GAP_INVALID_CONN_INDEX == s_hrs_slave_conn_idx) {
        return;
    }

    if (++heart_rate > 120) {
        heart_rate = 80;
    }

    hrs_heart_rate_measurement_send(s_hrs_slave_conn_idx, heart_rate, false);
}

static void heartrate_service_event_handler(hrs_evt_t *p_evt) {
    sdk_err_t   ret;

    switch (p_evt->evt_type) {
        case HRS_EVT_NOTIFICATION_ENABLED:
            ret = app_timer_start(s_heart_rate_meas_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(ret);
            break;

        case HRS_EVT_NOTIFICATION_DISABLED:
            app_timer_stop(s_heart_rate_meas_timer_id);
            break;

        default:
            break;
    }
}

static void adv_param_init(void) {
    sdk_err_t ret;

    s_hrs_slave_adv_param.adv_intv_max = APP_ADV_INTERVAL;
    s_hrs_slave_adv_param.adv_intv_min = APP_ADV_INTERVAL;
    s_hrs_slave_adv_param.adv_mode     = BLE_GAP_ADV_TYPE_ADV_IND;
    s_hrs_slave_adv_param.chnl_map     = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_hrs_slave_adv_param.disc_mode    = BLE_GAP_DISC_MODE_NON_DISCOVERABLE;
    s_hrs_slave_adv_param.filter_pol   = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_hrs_slave_adv_time_param.max_adv_evt = 0;
    s_hrs_slave_adv_time_param.duration    = 0;

    ret = ble_gap_adv_data_set(s_hrs_slave_adv_idx, BLE_GAP_ADV_DATA_TYPE_DATA, s_hrs_slave_adv_data, sizeof(s_hrs_slave_adv_data));
    APP_ERROR_CHECK(ret);

    ret = ble_gap_adv_param_set(s_hrs_slave_adv_idx, BLE_GAP_OWN_ADDR_STATIC, &s_hrs_slave_adv_param);
    APP_ERROR_CHECK(ret);
}

static void services_init(void) {
    sdk_err_t  ret;
    hrs_init_t hrs_init;

    hrs_init.sensor_loc                      = HRS_SENS_LOC_FINGER;
    hrs_init.char_mask                       = HRS_CHAR_MANDATORY;
    hrs_init.evt_handler                     = heartrate_service_event_handler;
    hrs_init.is_sensor_contact_supported     = true;
    ret = hrs_service_init(&hrs_init);
    APP_ERROR_CHECK(ret);
}

static void app_timer_init(void) {
    sdk_err_t ret;

    ret = app_timer_create(&s_heart_rate_meas_timer_id, ATIMER_REPEAT, heartrate_meas_timeout_handler);
    APP_ERROR_CHECK(ret);
}

static void adv_start(void) {
    sdk_err_t           ret;
    ble_gap_bdaddr_t    adv_addr;

    adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    memcpy(adv_addr.gap_addr.addr, s_hrs_slave_addr, 6);

    ret = ble_gap_addr_set(&adv_addr);
    APP_ERROR_CHECK(ret);

    s_hrs_slave_adv_cplt = false;
    ret = ble_gap_adv_start(s_hrs_slave_adv_idx, &s_hrs_slave_adv_time_param);
    APP_ERROR_CHECK(ret);

    if (!ret) {
        while(!s_hrs_slave_adv_cplt) {
            extern void BLE_SDK_Handler_func(void);
            BLE_SDK_Handler_func();
            s_hrs_slave_adv_cplt = true;
        }
    }
}


static void heartrate_slave_sec_gatt_param_change(uint8_t conn_idx) {
    sdk_err_t ret;

    ble_sec_param_t sec_param = {
        .level     = BLE_SEC_MODE1_LEVEL2,
        .io_cap    = BLE_SEC_IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ALL,
        .rkey_dist = BLE_SEC_KDIST_ALL,
    };

    ret = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(ret);

    ble_gatts_service_hide_all(conn_idx);
    ble_gatts_service_discoverable_set(conn_idx, hrs_service_start_handle_get(), true);
}


void heartrate_slave_init(void) {
    adv_param_init();
    services_init();
    app_timer_init();

    adv_start();
}

void heartrate_slave_ble_evt_handler(const ble_evt_t *p_evt) {
    switch(p_evt->evt_id) {
        case BLE_GAPM_EVT_ADV_START:
            if (p_evt->evt.gapm_evt.index == s_hrs_slave_adv_idx) {
                APP_LOG_DEBUG("Heart Rate Slave Adverting Start, status: %02x", p_evt->evt_status);
                s_hrs_slave_adv_cplt = true;
            }
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.index == s_hrs_slave_adv_idx) {
                APP_LOG_DEBUG("Heart Rate Slave Adverting Stop, status: %02x, reason: %02x", p_evt->evt.gapm_evt.index, p_evt->evt_status, p_evt->evt.gapm_evt.params.adv_stop.reason);
            }
            break;

        case BLE_GAPC_EVT_CONNECTED: {
            extern void ble_gap_get_local_addr_by_conidx(uint8_t conidx,uint8_t *p_addr);
            uint8_t local_con_addr[6];

            ble_gap_get_local_addr_by_conidx(p_evt->evt.gapc_evt.index, local_con_addr);

            if(memcmp(s_hrs_slave_addr, local_con_addr, 6) == 0) {
                s_hrs_slave_conn_idx = p_evt->evt.gapc_evt.index;
                heartrate_slave_sec_gatt_param_change(s_hrs_slave_conn_idx);
                APP_LOG_DEBUG("Heart Rate Slave Connected, index: %02x", s_hrs_slave_conn_idx);
            }
        }
        break;

        case BLE_GAPC_EVT_DISCONNECTED:
            if (s_hrs_slave_conn_idx == p_evt->evt.gapc_evt.index) {
                s_hrs_slave_conn_idx = BLE_GAP_INVALID_CONN_INDEX;
                APP_LOG_DEBUG("Heart Rate Slave Disconnected, index: %02x, reason: %02x", p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
                app_timer_stop(s_heart_rate_meas_timer_id);
                adv_start();
            }
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            if (s_hrs_slave_conn_idx == p_evt->evt.sec_evt.index) {
                ble_sec_cfm_enc_t cfm_enc;

                memset((uint8_t *)&cfm_enc, 0, sizeof(cfm_enc));

                if (BLE_SEC_PAIR_REQ == p_evt->evt.sec_evt.params.enc_req.req_type) {
                        cfm_enc.req_type = BLE_SEC_PAIR_REQ;
                        cfm_enc.accept   = true;
                        ble_sec_enc_cfm(p_evt->evt.sec_evt.index, &cfm_enc);
                }
            }
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (s_hrs_slave_conn_idx == p_evt->evt.sec_evt.index) {
                APP_LOG_DEBUG("Heart Rate Slave Link has been encrypted, status: %02x", p_evt->evt_status);
            }
            break;

        case BLE_SEC_EVT_KEY_MISSING:
            if (s_hrs_slave_conn_idx == p_evt->evt.sec_evt.index) {
                APP_LOG_DEBUG("Keyboard Slave Link Key Missing");
            }
            break;
    }
}

