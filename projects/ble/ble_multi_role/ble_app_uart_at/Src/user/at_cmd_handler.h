/**
 *****************************************************************************************
 *
 * @file at_cmd_handler.h
 *
 * @brief AT Command handler API
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
#ifndef __AT_CMD_HANDLER_H__
#define __AT_CMD_HANDLER_H__

#include "at_cmd_utils.h"
#include "user_periph_setup.h"
#include "grx_hal.h"
#include "gr_includes.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Device working state. */
typedef enum
{
    STANDBY     = 0x01,
    ADVERTISING,
    SCANNING,
    INITIATING,
    CONNECTED,
} dev_state_t;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Process advertising report receive task .
 *****************************************************************************************
 */
void uart_at_adv_report_task(const uint8_t *p_data, uint16_t length, const ble_gap_bdaddr_t *p_bdaddr);

/**
 *****************************************************************************************
 * @brief Process connection completed task .
 *****************************************************************************************
 */
void uart_at_conn_task(ble_gap_ll_role_type_t ll_role);

/**
 *****************************************************************************************
 * @brief Printf and push to buffer.
 *****************************************************************************************
 */
void uart_at_printf(const char *format, ...);

/**
 *****************************************************************************************
 * @brief Set current device working state.
 *****************************************************************************************
 */
void uart_at_dev_state_set(dev_state_t dev_state);

/**
 *****************************************************************************************
 * @brief Get current GAP role.
 *****************************************************************************************
 */
ble_gap_role_t uart_at_curr_gap_role_get(void);

/**
 *****************************************************************************************
 * @brief Initialize uart at module.
 *****************************************************************************************
 */
void uart_at_init(ble_gap_role_t gap_role);

/**
 *****************************************************************************************
 * @brief Process TEST CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_test(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process VERSION GET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_version_get(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Uart at app reset.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_app_reset(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process BAUD SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_baud_set(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process BOARD ADDRESS GET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_bd_addr_get(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process GAP ROLE GET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_gap_role_get(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process GAP ROLE SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_gap_role_set(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process GAP NAME GET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_gap_name_get(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process GAP NAME SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_gap_name_set(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process ADVERTISING PARAMETER SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_adv_param_set(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process ADVERTISING START CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_adv_start(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process ADVERTISING STOP CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_adv_stop(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process SCAN PARAMETERS SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_scan_param_set(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process SCAN START CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_scan_start(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process SCAN STOP CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_scan_stop(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process CONNECT PARAMETERS SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_conn_param_set(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process CONNECT INITIATE SET CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_conn_init(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process CONNECT CANCEL CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_conn_cancle(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process DISCONNECT CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_disconnect(at_cmd_parse_t *p_cmd_param);

/**
 *****************************************************************************************
 * @brief Process MTU EXCHANGE CMD.
 *
 * @param[in] p_cmd_param: Pointer to CMD parameters.
 *****************************************************************************************
 */
void uart_at_mtu_exchange(at_cmd_parse_t *p_cmd_param);


/**
 *****************************************************************************************
 * @brief Process connection task.
 *
 * @param[in] conn_idx: Index of connection.
 * @param[in] ll_role:  Device role of LL layer type
 *****************************************************************************************
 */
void codeless_conn_task(uint8_t conn_idx, ble_gap_ll_role_type_t ll_role);

/**
 *****************************************************************************************
 * @brief Process disconnection task.
 *
 * @param[in] conn_idx:          The connection index.
 * @param[in] disconnect_reason: The reason of disconnect. See @ref ble_hci_error_t.
 *****************************************************************************************
 */
void codeless_disconn_task(uint8_t  conn_idx,  const uint8_t disconnect_reason);

/**
 *****************************************************************************************
 * @brief Process mtu has been exchanged task.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] mtu:      The value of exchanged mtu.
 *****************************************************************************************
 */
void codeless_mtu_exc_task(uint8_t conn_idx, uint16_t mtu);

#endif


