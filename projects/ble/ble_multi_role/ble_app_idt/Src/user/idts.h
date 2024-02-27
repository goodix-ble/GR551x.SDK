/**
 *****************************************************************************************
 *
 * @file idts.h
 *
 * @brief Goodix UART Service API
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
 * @defgroup BLE_SDK_IDTS Goodix UART Service (IDTS)
 * @{
 * @brief Definitions and prototypes for the IDTS interface.
 *
 * @details The Goodix UART Service is a customized GATT-based service with Tx, Rx and Flow Control
 *          characteristics. The application uses the service to send and receive data to and
 *          from the peer. The application data is sent to the peer as Handle Value Notification,
 *          and the data received from the peer is transmitted with GATT Write Command.
 *
 *          After \ref idts_init_t variable is initialized , the application must call \ref idts_service_init()
 *          to add the Goodix Uart Service and Rx, Tx, Flow Control characteristics to the BLE Stack
 *          database. The application can send the data to the peer with \ref idts_tx_data_send() after 
 *          \ref IDTS_EVT_TX_PORT_OPENED received. The application should copy the received data to its own buffer
 *          when handling \ref IDTS_EVT_RX_DATA_RECEIVED.
 */

#ifndef __IDTS_H__
#define __IDTS_H__

#include "grx_sys.h"
#include "custom_config.h"

/**
 * @defgroup IDTS_MACRO Defines
 * @{
 */
#define IDTS_CONNECTION_MAX      (10 < CFG_MAX_CONNECTIONS ?\
                                10 : CFG_MAX_CONNECTIONS)                          /**< Maximum number of Goodix UART Service connections. */
#define IDTS_MAX_DATA_LEN        247                                                /**< Maximum length of application data packet which is transmitted via IDTS. */
#define IDTS_FLOW_CTRL_LEN       1                                                  /**< Maximum length of ble flow control data packet which is transmitted via IDTS. */
#define IDTS_SERVICE_UUID        0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                0x0A, 0x46, 0x44, 0xD3, 0x01, 0xA0, 0xED, 0xA6     /**< The UUID of Goodix UART Service for setting advertising data. */
/** @} */

/**
 * @defgroup IDTS_ENUM Enumerations
 * @{
 */
/**@brief Goodix UART Service event types. */
typedef enum
{
    IDTS_EVT_INVALID,                /**< Invalid IDTS event. */
    IDTS_EVT_RX_DATA_RECEIVED,       /**< The data from the peer has been received. */
    IDTS_EVT_TX_DATA_SENT,           /**< The data from the application has been sent, and the service is ready to accept new data from the application. */
    IDTS_EVT_TX_PORT_OPENED,         /**< Tx port has been opened. */
    IDTS_EVT_TX_PORT_CLOSED,         /**< Tx port has been closed. */
} idts_evt_type_t;

/**@brief Flow control state for IDTS service. */
enum idts_flow_ctrl_state
{
  IDTS_FLOW_CTRL_STATE_OFF = 0,      /**< Indicate that IDTS can not receive data from peer. */
  IDTS_FLOW_CTRL_STATE_ON            /**< Indicate that IDTS can receive data from peer. */
};
/**@brief Underlying type used for the IDTS flow control state. */
typedef uint8_t idts_flow_ctrl_state_t;
/** @} */

/**
 * @defgroup IDTS_STRUCT Structures
 * @{
 */
/**@brief Goodix UART Service event. */
typedef struct
{
    idts_evt_type_t  evt_type;   /**< The IDTS event. */
    uint8_t         conn_idx;   /**< The index of the connection for the data transmission. */
    uint8_t        *p_data;     /**< Pointer to the buffer within received data. */
    uint16_t        length;     /**< Length of received data. */
} idts_evt_t;
/** @} */

/**
 * @defgroup IDTS_TYPEDEF Typedefs
 * @{
 */
/**@brief Goodix UART Service event handler type. */
typedef void (*idts_evt_handler_t)(idts_evt_t *p_evt);
/** @} */

/**
 * @addtogroup IDTS_STRUCT Structures
 * @{
 */
/**@brief Goodix UART Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    idts_evt_handler_t evt_handler;                     /**< Goodix UART Service event handler which must be provided by the application to send and receive the data. */
} idts_init_t;
/** @} */

/**
 * @defgroup IDTS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Goodix UART Service instance and add in the database.
 *
 * @param[in] p_idts_init: Pointer to Goodix UART Service initialization variables.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t idts_service_init(idts_init_t *p_idts_init);

/**
 *****************************************************************************************
 * @brief Send data to peer device.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_data:   Pointer to sent data.
 * @param[in] length:   Length of sent data.
 *
 * @return Result of sending data.
 *****************************************************************************************
 */
sdk_err_t idts_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/** @} */

#endif
/** @} */
/** @} */
