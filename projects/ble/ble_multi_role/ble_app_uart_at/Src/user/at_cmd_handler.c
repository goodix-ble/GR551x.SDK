/**
 *****************************************************************************************
 *
 * @file codeless_handler.c
 *
 * @brief Codeless Handler Implementation.
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
#include "at_cmd_handler.h"
#include "at_cmd_utils.h"
#include "user_periph_setup.h"
#include "transport_scheduler.h"
#include "ring_buffer.h"
#include "gus_c.h"
#include "app_timer.h"
#include "grx_hal.h"
#include <stdio.h>
#include <stdarg.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define ADV_DATA_DEFAULT_LEN                0x1C       /**< Default advertising data length. */
#define GAP_ADV_INTERVAL_MIN                32         /**< Minimum valid advertising interval. */
#define GAP_ADV_INTERVAL_MAX                16384      /**< Maximum valid advertising interval. */
#define GAP_ADV_TIMEOUT_MAX                 18000      /**< Maximum valid advertising timeout. */
#define GAP_SCAN_INTERVAL_MIN               4          /**< Minimum valid scan interval. */
#define GAP_SCAN_INTERVAL_MAX               16384      /**< Maximum valid scan interval. */
#define GAP_SCAN_TIMEOUT_MAX                65535      /**< Maximum valid scan timeout. */
#define GAP_CONN_INTERVAL_MIN               6          /**< Minimum valid connect interval. */
#define GAP_CONN_INTERVAL_MAX               3200       /**< Maximum valid connect interval. */
#define GAP_CONN_SUP_TIMEOUT_MAX            3200       /**< Maximum valid connect timeout. */
#define APP_AT_CMD_TIMING                   5000       /**< Timing for AT CMD execute timeout (in uint of 1ms). */
#define RING_BUFFER_SIZE                    5120       /**< Size of ring buffer. */
#define UART_ONCE_SEND_SIZE                 244        /**< Size of uart once send. */

#define UART_BAUD_RATE_MAX                  2000000    /**< Configure the maximum baud rate of the serial port. */
#define UART_AT_VERSION                     "1.0.0"    /**< Uart AT version number. */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern ble_gap_adv_param_t      g_gap_adv_param;
extern ble_gap_adv_time_param_t g_gap_adv_time_param;
extern ble_gap_scan_param_t     g_gap_scan_param;
extern ble_gap_init_param_t     g_gap_connect_param;
extern uint8_t                  g_adv_data_set[28];
extern uint8_t                  g_adv_rsp_data_set[28];

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static at_cmd_rsp_dest_t           s_curr_rsp_dest;
static app_timer_id_t              s_at_cmd_timing_id;
static ble_gap_bdaddr_t            s_target_bdaddr;               /**< Target board address. */
static ble_gap_role_t              s_curr_gap_role;
static dev_state_t                 s_curr_dev_state;
static bool                        s_is_target_found;

static at_cmd_attr_t s_at_cmd_attr_table[] =
{
    {AT_CMD_INVALID,         "",            0,  NULL},
    {AT_CMD_TEST,            "TEST",        4,  uart_at_test},
    {AT_CMD_VERSION_GET,     "VERSION?",    8,  uart_at_version_get},
    {AT_CMD_RESET,           "RESET",       5,  uart_at_app_reset},
    {AT_CMD_BAUD_SET,        "BAUD=",       5,  uart_at_baud_set},
    {AT_CMD_ADDR_GET,        "ADDR?",       5,  uart_at_bd_addr_get},
    {AT_CMD_GAP_ROLE_GET,    "GAP_ROLE?",   9,  uart_at_gap_role_get},
    {AT_CMD_GAP_ROLE_SET,    "GAP_ROLE=",   9,  uart_at_gap_role_set},
    {AT_CMD_GAP_NAME_GET,    "GAP_NAME?",   9,  uart_at_gap_name_get},
    {AT_CMD_GAP_NAME_SET,    "GAP_NAME=",   9,  uart_at_gap_name_set},
    {AT_CMD_ADV_PARAM_SET,   "ADV_PARAM=",  10, uart_at_adv_param_set},
    {AT_CMD_ADV_START,       "ADV_START",   9,  uart_at_adv_start},
    {AT_CMD_ADV_STOP,        "ADV_STOP",    8,  uart_at_adv_stop},
    {AT_CMD_SCAN_PARAM_SET,  "SCAN_PARAM=", 11, uart_at_scan_param_set},
    {AT_CMD_SCAN_START,      "SCAN_START",  10, uart_at_scan_start},
    {AT_CMD_SCAN_STOP,       "SCAN_STOP",   9,  uart_at_scan_stop},
    {AT_CMD_CONN_PARAM_SET,  "CONN_PARAM=", 11, uart_at_conn_param_set},
    {AT_CMD_CONN_INIT,       "CONN_INIT=",  10, uart_at_conn_init},
    {AT_CMD_CONN_CANCEL,     "CONN_CANCEL", 11, uart_at_conn_cancle},
    {AT_CMD_DISCONN,         "DISCONN",     7,  uart_at_disconnect},
    {AT_CMD_MTU_EXCHANGE,    "MTU_EXC",     7,  uart_at_mtu_exchange},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */


static void user_at_cmd_timeout_check(void *p_arg)
{
    at_cmd_execute_timing_process();
}

static void user_at_cmd_timing_start(void)
{
    app_timer_create(&s_at_cmd_timing_id, ATIMER_ONE_SHOT, user_at_cmd_timeout_check);
    app_timer_start(s_at_cmd_timing_id, APP_AT_CMD_TIMING, NULL);
}

static void user_at_cmd_callback(at_cmd_rsp_dest_t rsp_dest, const uint8_t *p_data, uint8_t length)
{
    s_curr_rsp_dest = rsp_dest;

    if (AT_CMD_RSP_DEST_UART == s_curr_rsp_dest)
    {
        ble_to_uart_buff_data_push(p_data, length);
    }
    else if (AT_CMD_RSP_DEST_BLE == s_curr_rsp_dest)
    {
        uart_to_ble_buff_data_push(p_data, length);
    }

    app_timer_delete(&s_at_cmd_timing_id);
}

static bool uart_at_gus_uuid_find(const uint8_t *p_data, const uint16_t length)
{
    uint16_t current_pos = 0;

    if (NULL == p_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t filed_type  = 0;
        uint8_t data_length = 0;
        uint8_t fragment_length = p_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        filed_type  = p_data[current_pos++];

        if ((BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID == filed_type) || \
                (BLE_GAP_AD_TYPE_MORE_128_BIT_UUID == filed_type))
        {
            uint8_t parase_uuid[16] = {0};
            uint8_t target_uuid[16] = GUS_SVC_UUID;
            uint8_t counter_128_bit_uuid =  data_length / 16;

            for (uint8_t i = 0; i < counter_128_bit_uuid; i++)
            {
                memcpy(parase_uuid, &p_data[current_pos + (16 * i)], 16);

                if (0 == memcmp(target_uuid, parase_uuid, 16))
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

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void uart_at_adv_report_task(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr)
{
    if (s_is_target_found)
    {
        return;
    }

    if (uart_at_gus_uuid_find(p_data, length))
    {
        memcpy(&s_target_bdaddr, p_bdaddr, sizeof(ble_gap_bdaddr_t));

        uart_at_printf("Target Device Found\r\n");
        s_is_target_found = true;
    }
}

void uart_at_conn_task(ble_gap_ll_role_type_t ll_role)
{
    if (BLE_GAP_LL_ROLE_MASTER == ll_role)
    {
        s_curr_gap_role = BLE_GAP_ROLE_CENTRAL;
    }
    else if (BLE_GAP_LL_ROLE_SLAVE == ll_role)
    {
        s_curr_gap_role = BLE_GAP_ROLE_PERIPHERAL;
    }
}

void uart_at_printf(const char *format, ...)
{
    char    str_temp[AT_CMD_BUFFER_SIZE_MAX];
    va_list ap;

    va_start(ap, format);
    vsprintf(str_temp, format, ap);
    va_end(ap);

    if (AT_CMD_RSP_DEST_UART == s_curr_rsp_dest)
    {
        ble_to_uart_buff_data_push((uint8_t *)str_temp, strlen(str_temp));
    }
    else if (AT_CMD_RSP_DEST_BLE == s_curr_rsp_dest)
    {
        uart_to_ble_buff_data_push((uint8_t *)str_temp, strlen(str_temp));
    }
}

void uart_at_dev_state_set(dev_state_t dev_state)
{
    s_curr_dev_state = dev_state;
}

ble_gap_role_t uart_at_curr_gap_role_get(void)
{
    return s_curr_gap_role;
}

void uart_at_init(ble_gap_role_t gap_role)
{
    at_cmd_init_t  cmd_init;

    cmd_init.p_cmd_attr         = s_at_cmd_attr_table;
    cmd_init.cmd_num            = sizeof(s_at_cmd_attr_table) / sizeof(at_cmd_attr_t);
    cmd_init.cmd_time_cb        = user_at_cmd_timing_start;
    cmd_init.cmd_cplt_cb        = user_at_cmd_callback;
    at_cmd_init(&cmd_init);

    s_curr_gap_role  = gap_role;
    s_curr_dev_state = STANDBY;
    s_curr_rsp_dest  = AT_CMD_RSP_DEST_UART;
}

void uart_at_app_reset(at_cmd_parse_t *p_cmd_param)
{
    hal_nvic_system_reset();
}

void uart_at_test(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);

    cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_version_get(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);

    cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, UART_AT_VERSION);
    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_baud_set(at_cmd_parse_t *p_cmd_param)
{
    uint32_t     baud_set;
    AT_CMD_RSP_DEF(cmd_rsp);

    if (at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[0]],
                                 p_cmd_param->arg_length[0],
                                 &baud_set))
    {
        if(0<baud_set && baud_set<=UART_BAUD_RATE_MAX)
        {
          uart_init(baud_set);
          cmd_rsp.error_code = at_cmd_hal_err_convert(HAL_OK);
        }
        else
        {
          cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        }
    }
    else
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
    }

    if (AT_CMD_ERR_NO_ERROR == cmd_rsp.error_code)
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
    }

    at_cmd_execute_cplt(&cmd_rsp);

}

void uart_at_bd_addr_get(at_cmd_parse_t *p_cmd_param)
{
    ble_gap_bdaddr_t bd_addr;
    AT_CMD_RSP_DEF(cmd_rsp);

    ble_gap_addr_get(&bd_addr);

    cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "%d-%02X:%02X:%02X:%02X:%02X:%02X",
                                        bd_addr.addr_type,
                                        bd_addr.gap_addr.addr[5],
                                        bd_addr.gap_addr.addr[4],
                                        bd_addr.gap_addr.addr[3],
                                        bd_addr.gap_addr.addr[2],
                                        bd_addr.gap_addr.addr[1],
                                        bd_addr.gap_addr.addr[0]);
    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_gap_role_get(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);

    switch (s_curr_gap_role)
    {
        case BLE_GAP_ROLE_NONE:
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "NONE");
            break;

        case BLE_GAP_ROLE_OBSERVER:
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OBSERVER");
            break;

        case BLE_GAP_ROLE_BROADCASTER:
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "BROADCASTER");
            break;

        case BLE_GAP_ROLE_CENTRAL:
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "CENTRAL");
            break;


        case BLE_GAP_ROLE_PERIPHERAL:
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "PERIPHERAL");
            break;

        case BLE_GAP_ROLE_ALL:
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "ALL");
            break;

        default:
            break;
    }

    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_gap_role_set(at_cmd_parse_t *p_cmd_param)
{
    ble_gap_role_t  gap_role_set;

    AT_CMD_RSP_DEF(cmd_rsp);

    switch (p_cmd_param->p_buff[p_cmd_param->arg_idx[0]])
    {
        case 'N':
        case 'n':
            gap_role_set = BLE_GAP_ROLE_NONE;
            break;

        case 'O':
        case 'o':
            gap_role_set = BLE_GAP_ROLE_OBSERVER;
            break;

        case 'B':
        case 'b':
            gap_role_set = BLE_GAP_ROLE_BROADCASTER;
            break;

        case 'C':
        case 'c':
            gap_role_set = BLE_GAP_ROLE_CENTRAL;
            break;

        case 'P':
        case 'p':
            gap_role_set = BLE_GAP_ROLE_PERIPHERAL;
            break;

        case 'A':
        case 'a':
            gap_role_set = BLE_GAP_ROLE_ALL;
            break;

        default:
            cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
            at_cmd_execute_cplt(&cmd_rsp);
            return;
    }

    if (STANDBY == s_curr_dev_state)
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        s_curr_gap_role = gap_role_set;
    }
    else
    {
        cmd_rsp.error_code = AT_CMD_ERR_CMD_REQ_ALLOWED;
    }

    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_gap_name_get(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    cmd_rsp.length = AT_CMD_BUFFER_SIZE_MAX;

    error_code = ble_gap_device_name_get(cmd_rsp.data, &cmd_rsp.length);
    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    at_cmd_execute_cplt(&cmd_rsp);

}

void uart_at_gap_name_set(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;
    uint32_t    index;

    if (2 != p_cmd_param->arg_count)
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
    }
    else
    {
        if (at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[0]],
                                     p_cmd_param->arg_length[0],
                                     &index))
        {
            error_code = ble_gap_device_name_set((ble_gap_dev_name_write_perm_t)index,
                                                 &p_cmd_param->p_buff[p_cmd_param->arg_idx[1]],
                                                 p_cmd_param->arg_length[1]);

            cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);
        }
        else
        {
            cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        }
    }

    if (AT_CMD_ERR_NO_ERROR == cmd_rsp.error_code)
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
    }

    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_adv_param_set(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;
    uint16_t    adv_interval;
    uint16_t    adv_duration;

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[0]],
                                  p_cmd_param->arg_length[0],
                                  (uint32_t *)&adv_interval))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[1]],
                                  p_cmd_param->arg_length[1],
                                  (uint32_t *)&adv_duration))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if ((GAP_ADV_INTERVAL_MIN > adv_interval) || \
        (GAP_ADV_INTERVAL_MAX < adv_interval) || \
        (GAP_ADV_TIMEOUT_MAX < adv_duration))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }
    else
    {
        g_gap_adv_param.adv_intv_min  = adv_interval;
        g_gap_adv_param.adv_intv_max  = adv_interval;
        g_gap_adv_time_param.duration = adv_duration;

        error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &g_gap_adv_param);

        cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

        if (AT_CMD_ERR_NO_ERROR == cmd_rsp.error_code)
        {
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        }

        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_adv_start(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &g_gap_adv_param);

    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, g_adv_data_set, ADV_DATA_DEFAULT_LEN);

    error_code = ble_gap_adv_start(0, &g_gap_adv_time_param);

    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
    else
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_adv_stop(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    error_code = ble_gap_adv_stop(0);

    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
    else
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_scan_param_set(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;
    uint16_t    scan_interval;
    uint32_t    scan_duration;

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[0]],
                                  p_cmd_param->arg_length[0],
                                  (uint32_t *)&scan_interval))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[1]],
                                  p_cmd_param->arg_length[1],
                                  (uint32_t *)&scan_duration))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if ((GAP_SCAN_INTERVAL_MIN > scan_interval) || \
        (GAP_SCAN_INTERVAL_MAX < scan_interval) || \
        (GAP_SCAN_TIMEOUT_MAX < scan_duration))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }
    else
    {
        g_gap_scan_param.interval = scan_interval;
        g_gap_scan_param.timeout  = scan_duration;

        error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &g_gap_scan_param);

        cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

        if (AT_CMD_ERR_NO_ERROR == cmd_rsp.error_code)
        {
            cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        }

        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_scan_start(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &g_gap_scan_param);

    pwr_mgmt_ble_wakeup();
    error_code = ble_gap_scan_start();

    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
    else
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_scan_stop(at_cmd_parse_t *p_cmd_param)
{

    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    pwr_mgmt_ble_wakeup();
    error_code = ble_gap_scan_stop();

    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
    else
    {
        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
        at_cmd_execute_cplt(&cmd_rsp);
    }

    s_is_target_found = false;
}

void uart_at_conn_param_set(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    uint16_t    conn_interval;
    uint16_t    conn_latency;
    uint16_t    conn_sup_timeout;

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[0]],
                                  p_cmd_param->arg_length[0],
                                  (uint32_t *)&conn_interval))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[1]],
                                  p_cmd_param->arg_length[1],
                                  (uint32_t *)&conn_latency))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if (!at_cmd_decimal_num_check(&p_cmd_param->p_buff[p_cmd_param->arg_idx[2]],
                                  p_cmd_param->arg_length[2],
                                  (uint32_t *)&conn_sup_timeout))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
        at_cmd_execute_cplt(&cmd_rsp);
        return;
    }

    if ((GAP_CONN_INTERVAL_MIN > conn_interval) || \
        (GAP_CONN_INTERVAL_MAX < conn_interval) || \
        (GAP_CONN_SUP_TIMEOUT_MAX < conn_sup_timeout))
    {
        cmd_rsp.error_code = AT_CMD_ERR_INVALID_PARAM;
    }
    else
    {
        g_gap_connect_param.interval_min  = conn_interval;
        g_gap_connect_param.interval_max  = conn_interval;
        g_gap_connect_param.slave_latency = conn_latency;
        g_gap_connect_param.sup_timeout   = conn_sup_timeout;

        cmd_rsp.length = at_cmd_printf_bush(cmd_rsp.data, "OK");
    }

    at_cmd_execute_cplt(&cmd_rsp);
}

void uart_at_conn_init(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    memcpy(&g_gap_connect_param.peer_addr, &s_target_bdaddr, sizeof(ble_gap_bdaddr_t));
    error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &g_gap_connect_param);

    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_conn_cancle(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    error_code = ble_gap_connect_cancel();
    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_disconnect(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    error_code = ble_gap_disconnect(0);
    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
}

void uart_at_mtu_exchange(at_cmd_parse_t *p_cmd_param)
{
    AT_CMD_RSP_DEF(cmd_rsp);
    sdk_err_t   error_code;

    error_code = ble_gattc_mtu_exchange(0);
    cmd_rsp.error_code = at_cmd_ble_err_convert(error_code);

    if (AT_CMD_ERR_NO_ERROR != cmd_rsp.error_code)
    {
        at_cmd_execute_cplt(&cmd_rsp);
    }
}


