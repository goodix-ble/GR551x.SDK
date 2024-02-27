/**
 *****************************************************************************************
 *
 * @file idts_c.h
 *
 * @brief Header file - Goodix UART Service Client
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_IDTS_C Goodix UART Service Client (IDTS_C)
 * @{
 * @brief Goodix UART Service Client module.
 *
 * @details The Goodix Uart Service Client contains the APIs and types, which can be used by the 
 *          application to perform scanning, connection and discover Goodix Uart Service at 
 *          peer and interact with it.
 *
 *          The application must provide an event handler, then call \ref idts_client_init(). After the
 *          module can send and receive BLE data, application can call \ref idts_c_tx_data_send() to 
 *          send data to peer, and receive data from peer \ref IDTS_C_EVT_PEER_DATA_RECEIVE,
 *          meanwhile update its received BLE data state \ref idts_c_rx_flow_ctrl_set() to peer.
 */

#ifndef __IDTS_C_H__
#define __IDTS_C_H__

#include "ble_prf_types.h"
#include "grx_sys.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup IDTS_C_MACRO Defines
 * @{
 */
#define IDTS_C_CONNECTION_MAX                (10 < CFG_MAX_CONNECTIONS ?\
                                             10 : CFG_MAX_CONNECTIONS)      /**< Maximum number of IDTS Client connections. */

/**
 * @defgroup IDTS_UUID Service and Characteristics UUID
 * @{
 */
#define IDTS_SVC_UUID       {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x01, 0xA0, 0xED, 0xA6}       /**< UUID of IDTS Service. */
#define IDTS_TX_CHAR_UUID   {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x02, 0xA0, 0xED, 0xA6}       /**< UUID of IDTS Tx characterisitc. */
#define IDTS_RX_CHAR_UUID   {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                            0x0A, 0x46, 0x44, 0xD3, 0x03, 0xA0, 0xED, 0xA6}       /**< UUID of IDTS Rx characterisitc. */
/** @} */
/** @} */

/**
 * @defgroup IDTS_C_ENUM Enumerations
 * @{
 */
/**@brief Goodix UART Service Client event type. */
typedef enum
{
    IDTS_C_EVT_INVALID,                      /**< Invalid IDTS Client event. */
    IDTS_C_EVT_DISCOVERY_COMPLETE,           /**< IDTS Client has found service and its characteristics at peer. */
    IDTS_C_EVT_DISCOVERY_FAIL,               /**< IDTS Client found THS service failed because of invalid operation or no found at peer. */
    IDTS_C_EVT_TX_IND_SET_SUCCESS,           /**< IDTS Client has set peer Tx notify. */
    IDTS_C_EVT_PEER_DATA_RECEIVE,            /**< IDTS Client has received something from peer. */
    IDTS_C_EVT_TX_CPLT,                      /**< IDTS Client has sent something to peer successfully. */
    IDTS_C_EVT_WRITE_OP_ERR,                 /**< Error occured when IDTS Client wrote to peer. */
} idts_c_evt_type_t;

/**@brief Flow control state for IDTS Client service. */
enum  idts_c_flow_ctrl_state
{
  IDTS_C_FLOW_CTRL_STATE_OFF = 0,      /**< Indicate that IDTS Client can not receive data from peer. */
  IDTS_C_FLOW_CTRL_STATE_ON            /**< Indicate that IDTS Client can receive data from peer. */
};
/**@brief Underlying type used for the IDTS Client flow control state. */
typedef uint8_t idts_c_flow_ctrl_state_t;
/** @} */

/**
 * @defgroup IDTS_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t idts_srvc_start_handle;     /**< IDTS Service start handle. */
    uint16_t idts_srvc_end_handle;       /**< IDTS Service end handle. */
    uint16_t idts_tx_handle;             /**< Handle of IDTS Tx characteristic as provided by a discovery. */
    uint16_t idts_tx_cccd_handle;        /**< Handle of CCCD of IDTS Tx characteristic as provided by a discovery. */
    uint16_t idts_rx_handle;             /**< Handle of IDTS Rx characteristic as provided by a discovery. */
} idts_c_handles_t;

/**@brief Goodix UART Service Client event. */
typedef struct
{
    uint8_t           conn_idx;           /**< Connection index. */
    idts_c_evt_type_t  evt_type;           /**< IDTS Client event type. */
    uint16_t          length;             /**< Length of event data. */
    uint8_t          *p_data;             /**< Pointer to event data. */
} idts_c_evt_t;
/** @} */

/**
 * @defgroup IDTS_C_TYPEDEF Typedefs
 * @{
 */
/**@brief Goodix UART Service Client event handler type. */
typedef void (* idts_c_evt_handler_t)(idts_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup IDTS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register IDTS Client event handler.
 *
 * @param[in] evt_handler: Goodix UART Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t idts_client_init(idts_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery IDTS on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t idts_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer IDTS Tx characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable ths Tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t idts_c_tx_indicate_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Send data to the server.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data need sent.
 * @param[in] length:   Length of data need sent.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t idts_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/** @} */
#endif
/** @} */
/** @} */

