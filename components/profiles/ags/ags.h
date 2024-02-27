/**
 *****************************************************************************************
 *
 * @file ags.h
 *
 * @brief Alexa Gadgets Service API.
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
 * @defgroup BLE_SDK_AGS Alexa Gadget Service (AGS)
 * @{
 * @brief Definitions and prototypes for the AGS interface.
 *
 * @details The Alexa Gadget (AG) Service is defined by Amazon. It can connect a Alexa gadget with an Echo 
 *          device. This module implements the Alexa Gadget Service with TX and RX characteristics.
 *
 *          After \ref ags_init_t variable is initialized, the application must call \ref ags_service_init()
 *          to add the Alexa Gadget Service and TX, RX characteristics to the BLE Stack database according to
 *          \ref ags_init_t.char_mask.
 */

#ifndef __AGS_H__
#define __AGS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup AGS_MACRO Defines
 * @{
 */
#define AGS_SERVICE_UUID            0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,\
                                    0x00, 0x10, 0x00, 0x00, 0x03, 0xFE, 0x00, 0x00     /**< The UUID of Alexa Gadget Service for setting advertising data. */

#define AGS_HEADER_FIRST_RES        0x00                                               /**< Reserved value of header in the first packet. */

#define AGS_HEADER_ACK_RES_1        0x00                                               /**< The first reserved value of header in the ACK packet. */
#define AGS_HEADER_ACK_PAYLOAD_LEN  0x02                                               /**< Length of the ACK packet, in bytes. */
#define AGS_HEADER_ACK_RES_2        0x01                                               /**< The second reserved value of header in the ACK packet. */

#define AGS_TX_VAL_LEN_MAX          244                                                /**< Maximum length of TX Characteristic value. */
#define AGS_RX_VAL_LEN_MAX          244                                                /**< Maximum length of RX Characteristic value. */

/**
 * @defgroup AGS_ADV_TYPE Advertising type
 * @{
 * @brief The advertisiment type.
 *        Bit 0 : Pairing or reconnection mode, 0 - reconnection mode, 1 - pairing mode;
 *        Bit 1 : Classic Bluetooth discoverability;
 *        Bit 2~7 : Reserved for future use, set to 0x00.
 */
#define AGS_PAIR_ADV_FLAG           0x01    /**< The Pairing advertisiment flag. */
#define AGS_RECONNECT_ADV_FLAG      0x00    /**< The reconnecting advertisiment flag. */
/** @} */

#define AGS_CONNECTION_MAX          10      /**< Maximum number of AGS connections. */

/**
 * @defgroup AGS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref ags_init_t.char_mask.
 */
#define AGS_CHAR_MANDATORY               0x003f     /**< Bit mask for mandatory characteristic in AGS. */
#define AGS_CHAR_FULL                    0x003f     /**< Bit mask of the full characteristic. */
/** @} */
/** @} */

/**
 * @defgroup AGS_ENUM Enumerations
 * @{
 */
/**@brief Alexa Gadget Service Stream ID. 
 *        At any time, there is only one control stream, one Alexa stream, and one OTA stream.
 *        Each packet belongs to one of these streams, which is specified in the stream ID field of the packet's header.
 *        See Gadgets documentation for details:
 *        https://developer.amazon.com/en-US/docs/alexa/alexa-gadgets-toolkit/packet-ble.html#streams
 */
typedef enum
{
    AGS_CONTROL_STREAM_ID,             /**< Control stream ID, used to communicate. */
    AGS_ALEXA_STREAM_ID   = 0x06,      /**< Alexa stream ID, used to send directives and events. */
    AGS_OTA_STREAM_ID     = 0x02,      /**< OTA stream ID, used to update the gadget's firmware. */
} ags_header_stream_id_t;

/**@brief Alexa Gadget Service transaction type. 
 *        It indicate where the packet is within the transaction. The transaction type and
 *        protocol of a single transaction are defined by the first packet of the transaction.
 */
typedef enum
{
    AGS_TRANSACTION_TYPE_FIRST,      /**< First packet of a transaction, or the only packet in a single-packet transaction. */
    AGS_TRANSACTION_TYPE_CONT,       /**< Continuation packet of a transaction. */
    AGS_TRANSACTION_TYPE_LAST,       /**< Last packet of a transaction. */
    AGS_TRANSACTION_TYPE_CTRL,       /**< Control packet. Currently, the only type of control packet is the ACK packet. */
} ags_header_trxn_type_t;

/**@brief Alexa Gadget Service ACK Flag.
 */
typedef enum
{
    AGS_ACK_NACK,    /**< The gadget doesn't need to send an ACK packet in response to the last packet of the 
                          transaction unless another packet in the transaction has this bit set to 1. */
    AGS_ACK_ACK,     /**< The gadget must send an ACK packet in response to the last packet of the transaction. */
} ags_header_ack_flag_t;

/**@brief Alexa Gadget Service Length extender.
 */
typedef enum
{
    AGS_LEN_EXT_NO_EXT,   /**< The packet's payload is 8 bits. */
    AGS_LEN_EXT_EXT,      /**< The packet's payload is 16 bits. */

} ags_header_length_ext_t;

/**@brief Alexa Gadget Service result code.
 */
typedef enum
{
    AGS_RES_CODE_SUCCESS,             /**< The message was valid and acted on (if applicable) successfully. */
    AGS_RES_CODE_UNKNOWN,             /**< The message was valid but rusulted in a failure or error. */
    AGS_RES_CODE_UNSUPPORTED = 0x03,  /**< The message was invalid because it contained a command or other field that
                                           was not supported by the receiver. */    
} ags_header_result_code_t;

/**@brief Alexa Gadget Service event type.*/
typedef enum
{
    AGS_EVT_INVALID,                       /**< Indicate that invalid event. */
    AGS_EVT_ECHO_RX_DATA_SENT,             /**< Indicate that the gadget has been sent the data to an Echo device. */
    AGS_EVT_ECHO_TX_DATA_RECEIVED,         /**< Indicate that the gadget has been received the data from an Echo device. */
    AGS_EVT_ECHO_RX_NOTI_ENABLE,           /**< Indicate that the Rx notification has been enabled. */
    AGS_EVT_ECHO_RX_NOTI_DISABLE,          /**< Indicate that the Rx notification has been disabled. */
} ags_evt_type_t;
/** @} */

/**
 * @defgroup AGS_STRUCT Structures
 * @{
 */
/**@brief Alexa Gadget Service base part of data packet header. */
typedef struct
{
    uint8_t  trxn_id    :4;     /**< The transaction ID. */
    uint8_t  stream_id  :4;     /**< The stream ID. @ref ags_header_stream_id_t */
    uint8_t  length_ext :1;     /**< Length extender. It indicates the length of the payload field. @ref ags_header_length_ext_t */
    uint8_t  ack_flag   :1;     /**< Acknowledgement(ACK) flag. @ref ags_header_ack_flag_t */
    uint8_t  trxn_type  :2;     /**< Transaction type. @ref ags_header_trxn_type_t */
    uint8_t  sequ_num   :4;     /**< Sequence number. Since this is four bits long, it supports 
                                     inplace sequencing of up to 16 packet at a given point in time. 
                                     Sequence numbers can rool over.*/
} ags_header_base_t;

/**@brief Alexa Gadget Service header in first packet. */
typedef struct
{
    ags_header_base_t header_base;          /**< Header base. */
    uint8_t           reserved;             /**< Reserved. This is only present in the first packet of the transaction. */
    uint8_t           total_trxn_length[2]; /**< Total transaction length. This is only present in the first packet of the transaction. */
    uint8_t           payload_length;       /**< Payload Length. */
} ags_header_first_t;

/**@brief Alexa Gadget Service extended version of header in first packet. */
typedef struct
{
    ags_header_base_t header_base;          /**< Header base. */
    uint8_t           reserved;             /**< Reserved. This is only present in the first packet of the transaction. */
    uint8_t           total_trxn_length[2]; /**< Total transaction length. This is only present in the first packet of the transaction. */
    uint8_t           payload_length[2];    /**< Payload Length. */
} ags_header_first_ext_t;

/**@brief Alexa Gadget Service header in subsequent packets. */
typedef struct
{
    ags_header_base_t header_base;    /**< Header base. */
    uint8_t           payload_length; /**< Payload Length. */
} ags_header_subs_t;

/**@brief Alexa Gadget Service extended version of header in subsequent packets. */
typedef struct
{
    ags_header_base_t header_base;       /**< Header base. */
    uint8_t           payload_length[2]; /**< Payload Length. */
} ags_header_subs_ext_t;

/**@brief Alexa Gadget Service ACK packet. */
typedef struct
{
    ags_header_base_t header_base;       /**< Header base. */
    uint8_t           reserved_1;        /**< Reserved. */
    uint8_t           payload_length;    /**< Length of the ACK packet. */
    uint8_t           reserved_2;        /**< Reserved. */
    uint8_t           result_code;       /**< A result code, which depends on whether this is an ACK or a NACK. @ref ags_header_ack_flag_t */
} ags_ack_packet_t;

/**@brief Alexa Gadget Service gadget stream enviorenment variable. */
typedef struct
{
    ags_header_stream_id_t stream_id;       /**< The stream ID. */
    uint8_t                is_active;       /**< Whether the stream is active. */
    uint8_t                ack_flag;        /**< Whether need to return ACK packet. */
    uint8_t                trxn_id;         /**< The transaction ID of the packet that requested the ACK packet. */
    uint16_t               total_length;    /**< Total stream length. */
    uint16_t               received_length; /**< Received stream length. */
} ags_stream_env_t;

/**@brief Alexa Gadget Service event. */
typedef struct
{
    ags_evt_type_t  evt_type;     /**< The AGS event type. */
    uint8_t         conn_idx;     /**< The index of the connection. */
    const uint8_t   *p_data;      /**< Pointer to event data. */
    uint16_t        length;       /**< Length of event data. */
} ags_evt_t;
/** @} */

/**
 * @defgroup AGS_TYPEDEF Typedefs
 * @{
 */
/**@brief Alexa Gadget Service event handler type.*/
typedef void (*ags_evt_handler_t)(ags_evt_t *p_evt);

/**@brief Alexa Gadget Service stream callback.*/
typedef bool (*ags_stream_cb_t)(uint8_t conn_idx, const uint8_t *const p_data, uint16_t length, uint8_t still_receiving);
/** @} */

/**
 * @defgroup AGS_STRUCT Structures
 * @{
 */
/**@brief Alexa Gadget Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    ags_evt_handler_t evt_handler;           /**< Alexa Gadget Service event handler. */
    ags_stream_cb_t   ags_control_stream_cb; /**< Callback for incoming control streams. */
    ags_stream_cb_t   ags_alexa_stream_cb;   /**< Callback for incoming Alexa streams. */  
    uint16_t          char_mask;             /**< Initial mask of supported characteristics, and configured with \ref AGS_CHAR_MASK. */
} ags_init_t;
/** @} */

/**
 * @defgroup AGS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Alexa Gadget Service instance and add in the DB.
 *
 * @param[in] p_ags_init: Pointer to ags_init_t Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t ags_service_init(ags_init_t *p_ags_init);

/**
 *****************************************************************************************
 * @brief Send data to the Echo device in stream format.
 *
 * @param[in] conn_idx:  Connnection index.
 * @param[in] stream_id: Stream ID, @ref ags_header_stream_id_t.
 * @param[in] p_data:    Pointer to the data to be sent.
 * @param[in] length:    Length of data.
 *
 * @return Result of sending data.
 *****************************************************************************************
 */
sdk_err_t ags_stream_send(uint8_t conn_idx, uint8_t stream_id, void *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send data to the Echo device in non-stream format.
 *
 * @param[in] conn_idx:  Connnection index.
 * @param[in] p_data:    Pointer to the data to be sent.
 * @param[in] length:    Length of data.
 *
 * @return Result of sending data.
 *****************************************************************************************
 */
sdk_err_t ags_non_stream_send(uint8_t conn_idx, void *p_data, uint16_t length);
/** @} */

#endif
/** @} */
/** @} */

