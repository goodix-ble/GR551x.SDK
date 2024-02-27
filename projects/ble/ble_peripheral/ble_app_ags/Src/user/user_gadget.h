/**
 *****************************************************************************************
 *
 * @file user_gadget.h
 *
 * @brief user gadget.
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
 
#ifndef _USER_GADGET_H_
#define _USER_GADGET_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_error.h"
#include "user_config.h"
#include "accessories.pb.h"
#include "directiveParser.pb.h"

#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
#include "alertsSetAlertDirective.pb.h"
#include "alertsSetAlertDirectivePayload.pb.h"
#include "alertsDeleteAlertDirective.pb.h"
#include "alertsDeleteAlertDirectivePayload.pb.h"
#endif
#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
#include "notificationsSetIndicatorDirective.pb.h"
#include "notificationsSetIndicatorDirectivePayload.pb.h"
#include "notificationsClearIndicatorDirective.pb.h"
#endif
#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
#include "alexaGadgetStateListenerStateUpdateDirective.pb.h"
#include "alexaGadgetStateListenerStateUpdateDirectivePayload.pb.h"
#endif
#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
#include "alexaGadgetMusicDataTempoDirective.pb.h"
#include "alexaGadgetMusicDataTempoDirectivePayload.pb.h"
#endif
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
#include "alexaGadgetSpeechDataSpeechmarksDirective.pb.h"
#include "alexaGadgetSpeechDataSpeechmarksDirectivePayload.pb.h"
#endif
#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
#include "custom_event.pb.h"
#endif

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @addtogroup BLE_SDK_AGS Alexa Gadget Service (AGS)
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

/**
 * @addtogroup AGS_MACRO Defines
 * @{
 */
/**@brief A gadget identifier.
 *        This identifier must meet the requirements in Gadget ID Requirements for Alexa Gadgets.
 */
#define USER_GADGET_DSN                 "DEV20210528143015"
#define USER_GADGET_TOKEN_LEN           32                   /**< The length of Device token. */
#define USER_GADGET_PV_LEN              20                   /**< The length of Protocol Version. */
#define USER_GADGET_PV_IDENTIFIER       0xFE03               /**< Protocol identifier. */
#define USER_GADGET_PV_MAJOR_VERSION    0x03                 /**< The major version of Protocol Version. */
#define USER_GADGET_PV_MINOR_VERSION    0X00                 /**< The minor version of Protocol Version. */
/** @} */

/**
 * @addtogroup AGS_UNION Unions
 * @{
 */
/**@brief Buffer used to store the decoded C struectur. */
typedef union
{
    ControlEnvelope control_envelope;
    directive_DirectiveParserProto directive_parser;
#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
    alerts_SetAlertDirectiveProto    set_alert_drt;
    alerts_DeleteAlertDirectiveProto delete_alert_drt;
#endif
#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
    notifications_SetIndicatorDirectiveProto   set_indicator_drt;
    notifications_ClearIndicatorDirectiveProto clear_indicator_drt;
#endif
#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
    alexaGadgetStateListener_StateUpdateDirectiveProto state_update_drt;
#endif
#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
    alexaGadgetMusicData_TempoDirectiveProto tempo_drt;
#endif
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
    alexaGadgetSpeechData_SpeechmarksDirectiveProto speechmarks_drt;
#endif
#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
    custom_event_proto custom_event_drt;
#endif
} protobuf_decoded_t;
/** @} */

/**
 * @addtogroup AGS_ENUM Enumerations
 * @{
 */
/**@brief Alexa Gadget Service user gadget event type.*/
typedef enum
{
    USER_GADGET_EVT_HANDSHAKE_DONE,  /**< Handshake completion event. */
    USER_GADGET_EVT_SET_ALERT,       /**< Set Alert event. */
    USER_GADGET_EVT_DELETE_ALERT,    /**< Delete Alert event. */
    USER_GADGET_EVT_TEMPO,           /**< Tempo event. */
    USER_GADGET_EVT_SPEECHMARKS,     /**< Speechmarks event. */
    USER_GADGET_EVT_STATE_UPDATE,    /**< StateUpdate event. */
    USER_GADGET_EVT_SET_INDICATOR,   /**< SetIndicator event. */
    USER_GADGET_EVT_CLEAR_INDICATOR, /**< ClearIndicator event. */
    USER_GADGET_EVT_CUSTOM,          /**< Custom event. */
} user_gadget_evt_type_t;

/**@brief User of protocol buffer. */
typedef enum
{
    PROTOBUF_USER_NONE,            /**< No one use the protocol buffer. */
    PROTOBUF_USER_CONTROL_STREAM,  /**< Control Stream is using the protocol buffer. */
    PROTOBUF_USER_ALEXA_STREAM,    /**< Alexa Stream is using the protocol buffer. */
    PROTOBUF_USER_CUSTOM_EVENT,    /**< Custom event is using the protocol buffer. */
} user_gadget_protobuf_user_t;
/** @} */

/**
 * @addtogroup AGS_STRUCT Structures
 * @{
 */
/**@brief Alexa Gadget Service event. */
typedef struct
{
    user_gadget_evt_type_t  user_gadget_evt_type; /**< The user gadget event type. */
    uint8_t                 conn_idx;             /**< The index of the connection. */
    union
    {
#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
		alerts_SetAlertDirectivePayloadProto                      *p_set_alert_drt_payload;     /**< Pointer to the payload of SetAlert directive. */
		alerts_DeleteAlertDirectivePayloadProto                   *p_delete_alert_drt_payload;  /**< Pointer to the payload of DeleteAlert directive. */
#endif
#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
		notifications_SetIndicatorDirectivePayloadProto           *p_set_indicator_drt_payload; /**< Pointer to the payload of SetIndicator directive. */
#endif
#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
		alexaGadgetStateListener_StateUpdateDirectivePayloadProto *p_state_update_drt_payload;  /**< Pointer to the payload of StateUpdate directive. */
#endif
#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
		alexaGadgetMusicData_TempoDirectivePayloadProto           *p_tempo_drt_payload;         /**< Pointer to the payload of Tempo directive. */
#endif
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
		alexaGadgetSpeechData_SpeechmarksDirectivePayloadProto    *p_speechmarks_drt_payload;   /**< Pointer to the payload of Speechmarks directive. */
#endif
        struct
        {
            const char *name;    /**< Pointer to the name of Custom directive. */
            const void *payload; /**< Pointer to the payload of Custom directive. */
            uint16_t   size;     /**< The size of Custom directive. */
        } CustomDirectivePayloadProto;
    } data;  /**< The payload of each directive. */
} user_gadget_evt_t;

/**@brief Alexa Gadget Service directive handler. */
typedef struct
{
    const char *p_directive_namespace; /**< Pointer to the namespace of each directive. */
    /**@brief The handler of each directive. */
    uint16_t  (*directive_handler)(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
} directive_handler_t;
/** @} */

/**
 * @addtogroup AGS_TYPEDEF Typedefs
 * @{
 */
/**@brief Alexa Gadget Service event handler type.*/
typedef void (*user_gadget_evt_handler_t)(user_gadget_evt_t *p_evt);
/** @} */

/**
 * @addtogroup AGS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize the user gadget event handler.
 *
 * @param[in] user_gadget_evt_handler: The user gadget event handler.
 *
 * @return The result of the initialization.
 *****************************************************************************************
 */
sdk_err_t user_gadget_evt_handler_init(user_gadget_evt_handler_t user_gadget_evt_handler);

/**
 *****************************************************************************************
 * @brief Release the ownership of protobuf.
 *
 * @param[in] conn_idx: The connection index.
 *
 * @return the old user.
 *****************************************************************************************
 */
uint16_t user_gadget_protobuf_owner_release(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief The callback for incoming control streams.
 *
 * @param[in] conn_idx:        The connection index.
 * @param[in] p_data:          Pointer to the control stream payload.
 * @param[in] len:             The payload length.
 * @param[in] still_receiving: Is it still receiving.
 *
 * @return The ack flag of response ACK packet.
 *****************************************************************************************
 */
bool user_gadget_control_stream_cb(uint8_t conn_idx, const uint8_t *p_data, uint16_t len, uint8_t still_receiving);

/**
 *****************************************************************************************
 * @brief The callback for incoming alexa streams.
 *
 * @param[in] conn_idx:        The connection index.
 * @param[in] p_data:          Pointer to the alexa stream payload.
 * @param[in] len:             The payload length.
 * @param[in] still_receiving: Is it still receiving.
 *
 * @return The ack flag of response ACK packet.
 *****************************************************************************************
 */
bool user_gadget_alexa_stream_cb(uint8_t conn_idx, const uint8_t *p_data, uint16_t len, uint8_t still_receiving);

/**
 *****************************************************************************************
 * @brief Send the protocol version.
 *
 * @param[in] conn_idx: The connection index.
 *****************************************************************************************
 */
void user_gadget_protocol_version_send(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Send the custom event data to Alexa cloud in JSON-format.
 *
 * @param[in] conn_idx:    The connection index.
 * @param[in] p_name:      Pointer to the custom event name.
 * @param[in] name_len:    The length of name.
 * @param[in] p_payload:   Pointer to the custom event payload, it should be in JSON-format.
 * @param[in] payload_len: The payload length.
 *
 * @return The result of the operation.
 *****************************************************************************************
 */
uint16_t user_gadget_custom_event_send(uint8_t conn_idx, uint8_t *p_name, uint8_t *p_payload);
/** @} */

#endif
/** @} */
/** @} */
