/**
 *****************************************************************************************
 *
 * @file user_gadget.c
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_app.h"
#include "user_gadget.h"
#include "ags.h"
#include "grx_hal.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"
#include "sha256.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "accessories.pb.h"
#include "directiveParser.pb.h"
#include "alexaDiscoveryDiscoverResponseEvent.pb.h"
#include "alexaDiscoveryDiscoverResponseEventPayload.pb.h"

#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
#include "custom_event.pb.h"
#endif

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The number of supported Alexa Gadget capabilities. */
#define USER_GADGET_CAP_COUNT                      \
    (USER_GADGET_CAPABILITY_STATELISTENER_ENABLE + \
     USER_GADGET_CAPABILITY_ALERTS_ENABLE +        \
     USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE + \
     USER_GADGET_CAPABILITY_MUSICDATA_ENABLE +     \
     USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE +    \
     USER_GADGET_CAPABILITY_CUSTOM_ENABLE)

/**@brief The placeholder of the device token
 *        The actual value is a UTF-8-encoded string that contains the SHA256 of the following: 
 *        the endpointId concatenated with the Alexa Gadget Secret.
 */
#define USER_GADGET_DEV_TOKEN_PLACEHOLDER "SIXTY-FOUR CHARACTER PLACEHOLDER VALUE FOR DEVICE TOKEN---------"

/**@brief The number of supported Alexa Gadget capabilities.
 * Bit 0: Set to 1 if the gadget supports the Alexa Gadgets Toolkit featrue set.
 * Bit 1: Set to 1 if the gadget supports OTA updates.
 * Bit 2: Set to 0.
 * Bit 3: Set to 0.
 * Bit 4: Set to 1.
 * Bit 5-63: Set to 0.*/
//#define USER_GADGET_DEV_FEATURES 1 | (USER_GADGET_OTA_ENABLE << 1) 
#define USER_GADGET_DEV_FEATURES 0x0000000000000009

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t                    s_protobuf[USER_GADGET_TRANSACTION_BUF_SIZE];
static uint8_t                    s_protobuf_user;
static uint16_t                   s_protobuf_offset;
static protobuf_decoded_t         s_protobuf_decoded;
static user_gadget_evt_handler_t  s_user_gadget_evt_handler;

/**@brief The response of the Discover directive.
 *        When the Echo device sends the gadget a Discover directive, the gadget responds with a 
 *        Discover.Response event that contains all the details of the gadget.
 *        The device token should be updated when the gadget generates the device token.
 */
static alexaDiscovery_DiscoverResponseEventProto discover_response_event =
{
    .has_event                     = true,
    .event.has_header              = true,
    .event.header.namespace        = "Alexa.Discovery",
    .event.header.name             = "Discover.Response",
//    .event.messageId               = NULL, // An ID that uniquely defines an instance of this directive. This string can be empty.
    .event.has_payload             = true,
    .event.payload.endpoints_count = 1,
    .event.payload.endpoints       =
    {
      {
          .endpointId         = USER_GADGET_DSN,
          .friendlyName       = DEVICE_NAME,
          .description        = USER_GADGET_DEVICE_DESCRIPTION,
          .manufacturerName   = USER_GADGET_MANU_NAME,
          .capabilities_count = USER_GADGET_CAP_COUNT,
          .capabilities       =
          {
#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
              {
                  .type                               = "AlexaInterface",
                  .interface                          = "Alerts",
                  .version                            = "1.1",
                  .has_configuration                  = true,
                  .configuration.supportedTypes_count = 0,
              },
#endif

#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
              {
                  .type                               = "AlexaInterface",
                  .interface                          = "Notifications",
                  .version                            = "1.0",
                  .has_configuration                  = true,
                  .configuration.supportedTypes_count = 0,
              },
#endif

#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
              {
                  .type                               = "AlexaInterface",
                  .interface                          = "Alexa.Gadget.StateListener",
                  .version                            = "1.0",
                  .has_configuration                  = true,
                  .configuration.supportedTypes_count = 5,
                  .configuration.supportedTypes =
                  {
                      {.name = "timeinfo"},
                      {.name = "timers"},
                      {.name = "alarms"},
                      {.name = "reminders"},
                      {.name = "wakeword"},
                  }
              },
#endif

#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
              {
                  .type                                 = "AlexaInterface",
                  .interface                            = "Alexa.Gadget.MusicData",
                  .version                              = "1.0",
                  .has_configuration                    = true,
                  .configuration.supportedTypes_count   = 1,
                  .configuration.supportedTypes[0].name = "tempo",
              },
#endif

#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
              {
                  .type                                 = "AlexaInterface",
                  .interface                            = "Alexa.Gadget.SpeechData",
                  .version                              = "1.0",
                  .has_configuration                    = true,
                  .configuration.supportedTypes_count   = 1,
                  .configuration.supportedTypes[0].name = "viseme",
              },
#endif

#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
              {
                  .type                               = "AlexaInterface",
                  .interface                          = USER_GADGET_CUSTOM_NAMESPACE,
                  .version                            = "1.0",
                  .has_configuration                  = true,
                  .configuration.supportedTypes_count = 0,
              },
#endif
          },
          .has_additionalIdentification  = true,
          .additionalIdentification      =
          {
              .firmwareVersion           = USER_GADGET_FW_VERSION,
              .deviceToken               = USER_GADGET_DEV_TOKEN_PLACEHOLDER,
              .deviceTokenEncryptionType = USER_GADGET_DEV_TOKEN_ENC_TYPE,
              .amazonDeviceType          = USER_GADGET_AMAZON_ID,
              .modelName                 = USER_GADGET_MODEL_NAME,
              .radioAddress              = USER_GADGET_RADIO_ADDR,
          }
      }  
    }
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint16_t discovery_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);

#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
static uint16_t alerts_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
#endif

#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
static uint16_t notifications_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
#endif

#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
static uint16_t statelistener_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
#endif
#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
static uint16_t musicdata_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
#endif
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
static uint16_t speechdata_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
#endif

#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
static uint16_t custom_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len);
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Handle corresponding to each directive. */
static directive_handler_t directive_handlers[] =
{
    {.p_directive_namespace = "Alexa.Discovery", .directive_handler = discovery_directive_handler},

#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
    {.p_directive_namespace = "Alerts", .directive_handler = alerts_directive_handler},
#endif
#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
    {.p_directive_namespace = "Notifications", .directive_handler = notifications_directive_handler},
#endif 
#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
    {.p_directive_namespace = "Alexa.Gadget.StateListener", .directive_handler = statelistener_directive_handler},
#endif   
#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
    {.p_directive_namespace = "Alexa.Gadget.MusicData", .directive_handler = musicdata_directive_handler},
#endif
#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
    {.p_directive_namespace = "Alexa.Gadget.SpeechData", .directive_handler = speechdata_directive_handler},
#endif   
#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
    {.p_directive_namespace = USER_GADGET_CUSTOM_NAMESPACE, .directive_handler = custom_directive_handler},
#endif
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Caculate the sha256 hash value of source data.
 *
 * @param[in] p_source: Pointer to the source data.
 * @param[in] len:      The length of source data.
 * @param[in] p_dest:   Pointer to the hash value.
 *
 * @return The result of caculating.
 *****************************************************************************************
 */
static uint16_t sha256(uint8_t *p_source, uint8_t len, uint8_t *p_dest)
{
    mbedtls_sha256_context ctx;
    
    mbedtls_sha256_init(&ctx);

    if (mbedtls_sha256_starts_ret(&ctx, 0) != 0)
    {
        APP_LOG_DEBUG("Error initializing sha-256 context\n");
        return 1;
    }

    if (mbedtls_sha256_update_ret(&ctx, (uint8_t *) p_source, len) != 0)
    {
        APP_LOG_DEBUG("Error updating sha-256 context\n");
        return 1;
    }

    if (mbedtls_sha256_finish_ret(&ctx, (uint8_t *) p_dest) != 0) {
        APP_LOG_DEBUG("Error finalizing and calculating sha-256 context\n");
        return 1;
    }
    return 0;
}

/**
 *****************************************************************************************
 * @brief Convert the hexadecimal number to string.
 *
 * @param[in] p_dst:  Pointer to the string.
 * @param[in] p_src:  Pointer to the hexadecimal number.
 * @param[in] length: The length of hexadecimal number.
 *****************************************************************************************
 */
static void convert_to_utf8(char *p_dst, uint8_t *p_src, uint16_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        //Upper nibble
        uint8_t value = (p_src[i] >> 4) & 0x0f;
        if (value > 9)
        {
            *p_dst++ = 'a' + value - 10;
        }
        else
        {
            *p_dst++ = '0' + value;
        }

        //Lower nibble
        value = p_src[i] & 0x0f;

        if (value > 9)
        {
            *p_dst++ = 'a' + value - 10;
        }
        else
        {
            *p_dst++ = '0' + value;
        }
    }
    *p_dst++ = 0; //null terminate
    APP_LOG_DEBUG("\n");
}

/**
 *****************************************************************************************
 * @brief Caculate the value of device token according to the DSN and Device Secret.
 *
 * @param[in] p_dev_token: Pointer to the device token.
 *****************************************************************************************
 */
static void user_gadget_dev_toke_generate(char *p_dev_token)
{   
    // Device Token Recevied from the dev-portal at gadget registration
    char *p_device_secret = USER_GADGET_DEVICE_SECRET;

    // Gadget ID is the endpointId (device serial number) that the gadget sends the Echo device in the Alexa.Discovery.Discover.Response event
    char *p_gadget_id = USER_GADGET_DSN;

    // The concatenated result of gadgetID and the deviceType
    char secret[200] = {0};
    strcat(secret, p_gadget_id);
    strcat(secret, p_device_secret);

    // Destination buffer for device Token
    uint8_t token[32];

    // Destination buffer for utf-8 representation of the 32 byte sha (32*2 + 1 byte for escape)
    char dst[65];

    uint16_t sha_result = sha256((uint8_t*)secret, strlen(secret), token);
    if (sha_result)
    {
        APP_LOG_DEBUG("Error calculating sha for device Token\n");
    }

    convert_to_utf8(dst, token, 32);
    memcpy(p_dev_token, dst, sizeof(dst));
}

/**
 *****************************************************************************************
 * @brief Discover directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t discovery_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t error_code;

    if (strcmp(p_name, "Discover") != 0)
    {
        return SDK_ERR_INVALID_PARAM;
    }
    // update the device token placeholders with actual values.
    char dev_token[USER_GADGET_TOKEN_LEN*2+1];

    user_gadget_dev_toke_generate(&dev_token[0]);
    memcpy(discover_response_event.event.payload.endpoints[0].additionalIdentification.deviceToken, dev_token, sizeof(dev_token));

    pb_ostream_t output_stream = pb_ostream_from_buffer(s_protobuf, sizeof(s_protobuf));
    bool status = pb_encode(&output_stream, &alexaDiscovery_DiscoverResponseEventProto_msg, &discover_response_event);
    if (!status)
    {
        return SDK_ERR_APP_ERROR;
    }

    APP_LOG_DEBUG("Send the discover response event.");
    error_code = ags_stream_send(conn_idx, AGS_ALEXA_STREAM_ID, s_protobuf, output_stream.bytes_written);
    if (!error_code)
    {
        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type = USER_GADGET_EVT_HANDSHAKE_DONE;
        user_gadget_evt.conn_idx             = conn_idx;
        s_user_gadget_evt_handler(&user_gadget_evt);
    }
    return error_code;
}

#if USER_GADGET_CAPABILITY_ALERTS_ENABLE
/**
 *****************************************************************************************
 * @brief Alerts directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t alerts_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t    error_code;
    pb_istream_t input_stream;
    bool         status;
    
    input_stream = pb_istream_from_buffer(p_data, len);
    if (strcmp(p_name, "SetAlert") == 0)
    {
        status = pb_decode(&input_stream, &alerts_SetAlertDirectiveProto_msg, &s_protobuf_decoded.set_alert_drt);
        if (!status)
        {
            APP_LOG_DEBUG("Decode Set Alert directive failed.");
            return SDK_ERR_APP_ERROR;
        }

        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type         = USER_GADGET_EVT_SET_ALERT;
        user_gadget_evt.conn_idx                     = conn_idx;
        user_gadget_evt.data.p_set_alert_drt_payload = NULL;
        if (s_protobuf_decoded.set_alert_drt.has_directive && s_protobuf_decoded.set_alert_drt.directive.has_payload)
        {
            user_gadget_evt.data.p_set_alert_drt_payload = &s_protobuf_decoded.set_alert_drt.directive.payload;
        }

        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code = SDK_SUCCESS;
    }
    else if (strcmp(p_name, "DeleteAlert") == 0)
    {
        status = pb_decode(&input_stream, &alerts_DeleteAlertDirectiveProto_msg, &s_protobuf_decoded.delete_alert_drt);
        if (!status)
        {
            APP_LOG_DEBUG("Decode Delete Alert directive failed.");
            return SDK_ERR_APP_ERROR;
        }

        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type         = USER_GADGET_EVT_DELETE_ALERT;
        user_gadget_evt.conn_idx                     = conn_idx;
        user_gadget_evt.data.p_set_alert_drt_payload = NULL;
        if (s_protobuf_decoded.delete_alert_drt.has_directive && s_protobuf_decoded.delete_alert_drt.directive.has_payload)
        {
            user_gadget_evt.data.p_delete_alert_drt_payload = &s_protobuf_decoded.delete_alert_drt.directive.payload;
        }
        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code = SDK_SUCCESS;
    }
    else
    {
        error_code = SDK_ERR_INVALID_PARAM;
    }
    return error_code;
}
#endif

#if USER_GADGET_CAPABILITY_NOTIFICATIONS_ENABLE
/**
 *****************************************************************************************
 * @brief Notification directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t notifications_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t    error_code;
    pb_istream_t input_stream;
    bool         status;
    
    input_stream = pb_istream_from_buffer(p_data, len);
    if (strcmp(p_name, "SetIndicator") == 0)
    {
        status = pb_decode(&input_stream, &notifications_SetIndicatorDirectiveProto_msg, &s_protobuf_decoded.set_indicator_drt);
        if (!status)
        {
            APP_LOG_DEBUG("Decode Set Indicator directive failed.");
            return SDK_ERR_APP_ERROR;
        }

        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type         = USER_GADGET_EVT_SET_INDICATOR;
        user_gadget_evt.conn_idx                     = conn_idx;
        user_gadget_evt.data.p_set_alert_drt_payload = NULL;
        if (s_protobuf_decoded.set_indicator_drt.has_directive && s_protobuf_decoded.set_indicator_drt.directive.has_payload)
        {
            user_gadget_evt.data.p_set_indicator_drt_payload = &s_protobuf_decoded.set_indicator_drt.directive.payload;
        }

        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code = SDK_SUCCESS;
    }
    else if (strcmp(p_name, "ClearIndicator") == 0)
    {
        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type         = USER_GADGET_EVT_CLEAR_INDICATOR;
        user_gadget_evt.conn_idx                     = conn_idx;
        user_gadget_evt.data.p_set_alert_drt_payload = NULL;

        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code = SDK_SUCCESS;
    }
    else
    {
        error_code = SDK_ERR_INVALID_PARAM;
    }
    return error_code;
}
#endif

#if USER_GADGET_CAPABILITY_STATELISTENER_ENABLE
/**
 *****************************************************************************************
 * @brief Statelistener directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t statelistener_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t    error_code;
    pb_istream_t input_stream;
    bool         status;

    input_stream = pb_istream_from_buffer(p_data, len);
    if (strcmp(p_name, "StateUpdate") == 0)
    {
        status = pb_decode(&input_stream, &alexaGadgetStateListener_StateUpdateDirectiveProto_msg, &s_protobuf_decoded.state_update_drt);
        if (!status)
        {
            APP_LOG_DEBUG("Decode State Update directive failed.");
            return SDK_ERR_APP_ERROR;
        }

        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type            = USER_GADGET_EVT_STATE_UPDATE;
        user_gadget_evt.conn_idx                        = conn_idx;
        user_gadget_evt.data.p_state_update_drt_payload = NULL;
        if (s_protobuf_decoded.state_update_drt.has_directive && s_protobuf_decoded.state_update_drt.directive.has_payload)
        {
            user_gadget_evt.data.p_state_update_drt_payload = &s_protobuf_decoded.state_update_drt.directive.payload;
        }
        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code = SDK_SUCCESS;
    }
    else
    {
        error_code = SDK_ERR_INVALID_PARAM;
    }
    return error_code;
}
#endif

#if USER_GADGET_CAPABILITY_MUSICDATA_ENABLE
/**
 *****************************************************************************************
 * @brief Musicdata directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t musicdata_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t    error_code;
    pb_istream_t input_stream;
    bool         status;

    input_stream = pb_istream_from_buffer(p_data, len);
    if (strcmp(p_name, "Tempo") == 0)
    {
        status = pb_decode(&input_stream, &alexaGadgetMusicData_TempoDirectiveProto_msg, &s_protobuf_decoded.tempo_drt);
        if (!status)
        {
            APP_LOG_DEBUG("Decode Tempo directive failed.");
            return SDK_ERR_APP_ERROR;
        }

        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type     = USER_GADGET_EVT_TEMPO;
        user_gadget_evt.conn_idx                 = conn_idx;
        user_gadget_evt.data.p_tempo_drt_payload = NULL;
        if (s_protobuf_decoded.tempo_drt.has_directive && s_protobuf_decoded.tempo_drt.directive.has_payload)
        {
            user_gadget_evt.data.p_tempo_drt_payload = &s_protobuf_decoded.tempo_drt.directive.payload;
        }
        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code = SDK_SUCCESS;
    }
    else
    {
        error_code = SDK_ERR_INVALID_PARAM;
    }
    return error_code;
}
#endif

#if USER_GADGET_CAPABILITY_SPEECHDATA_ENABLE
/**
 *****************************************************************************************
 * @brief Speechdata directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t speechdata_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t    error_code;
    pb_istream_t input_stream;
    bool         status;

    input_stream = pb_istream_from_buffer(p_data, len);
    if (strcmp(p_name, "Speechmarks") == 0)
    {
        status = pb_decode(&input_stream, &alexaGadgetSpeechData_SpeechmarksDirectiveProto_msg, &s_protobuf_decoded.speechmarks_drt);
        if (!status)
        {
            APP_LOG_DEBUG("Decode Speechmarks directive failed.");
            return SDK_ERR_APP_ERROR;
        }

        user_gadget_evt_t user_gadget_evt;
        user_gadget_evt.user_gadget_evt_type            = USER_GADGET_EVT_SPEECHMARKS;
        user_gadget_evt.conn_idx                        = conn_idx;
        user_gadget_evt.data.p_state_update_drt_payload = NULL;
        if (s_protobuf_decoded.speechmarks_drt.has_directive && s_protobuf_decoded.speechmarks_drt.directive.has_payload)
        {
            user_gadget_evt.data.p_speechmarks_drt_payload = &s_protobuf_decoded.speechmarks_drt.directive.payload;
        }
        s_user_gadget_evt_handler(&user_gadget_evt);
        error_code=  SDK_SUCCESS;
    }
    else
    {
        error_code = SDK_ERR_INVALID_PARAM;
    }
    return error_code;
}
#endif

#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
/**
 *****************************************************************************************
 * @brief Custom directive handler.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] p_name:   Pointer to the directive name.
 * @param[in] p_data:   Pointer to the data.
 * @param[in] len:      Data length.
 *
 * @return the resule of directive handler.
 *****************************************************************************************
 */
static uint16_t custom_directive_handler(uint8_t conn_idx, const char *p_name, const uint8_t *p_data, uint16_t len)
{
    sdk_err_t    error_code;

    if (!s_protobuf_decoded.directive_parser.has_directive)
    {
        APP_LOG_DEBUG("Custom directive is empty.");
        return SDK_ERR_INVALID_PARAM;
    }

    user_gadget_evt_t user_gadget_evt;
    user_gadget_evt.user_gadget_evt_type            = USER_GADGET_EVT_CUSTOM;
    user_gadget_evt.conn_idx                        = conn_idx;
    user_gadget_evt.data.p_state_update_drt_payload = NULL;

    s_user_gadget_evt_handler(&user_gadget_evt);
    error_code = SDK_SUCCESS;

    return error_code;
}
#endif

/**
 *****************************************************************************************
 * @brief Obtain the ownership of protobuf.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] user:     The new user.
 *
 * @return the result of obtaing the ownship operation.
 *****************************************************************************************
 */
static uint16_t user_gadget_protobuf_owner_obtain(uint8_t conn_idx, uint8_t user)
{
    if (PROTOBUF_USER_NONE == s_protobuf_user)
    {
        s_protobuf_user = user;
        return SDK_SUCCESS;
    }
    else
    {
        APP_LOG_DEBUG("The protobuf is in use.");
        return SDK_ERR_DISALLOWED;
    }
}


/**
 *****************************************************************************************
 * @brief Fill the protobuf.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] user:     The user who want to use the protobuf.
 * @param[in] p_data:   The pointer to the data.
 * @param[in] len:      The data length.
 *
 * @return the resule of write operation.
 *****************************************************************************************
 */
static uint16_t user_gadget_protobuf_fill(uint8_t conn_idx, uint8_t user, const uint8_t *p_data, uint16_t len)
{
    if (user != s_protobuf_user)
    {
        sdk_err_t error_code;
        error_code = user_gadget_protobuf_owner_obtain(conn_idx, user);
        if (error_code)
        {
            return error_code;
        }
    }

    if ((s_protobuf_offset+len) > sizeof(s_protobuf))
    {
        APP_LOG_DEBUG("Buffer is full.");
        return SDK_ERR_INVALID_OFFSET;
    }

    memcpy(&s_protobuf[s_protobuf_offset], p_data, len);
    s_protobuf_offset += len;
    
    return SDK_SUCCESS;
}

/**
 *****************************************************************************************
 * @brief Reset the protobuf.
 *
 * @param[in] conn_idx: The connection index.
 *
 * @return the resule of reset operation.
 *****************************************************************************************
 */
static void user_gadget_protobuf_reset(uint8_t conn_idx)
{
    s_protobuf_offset = 0;
    memset(s_protobuf, 0, sizeof(s_protobuf));

    user_gadget_protobuf_owner_release(conn_idx);
}

/**
 *****************************************************************************************
 * @brief Respond to the command from echo device.
 *
 * @param[in] conn_idx: The connection index.
 *
 * @return the resule of this operation.
 *****************************************************************************************
 */
static sdk_err_t user_gadget_command_respond(uint8_t conn_idx)
{
    sdk_err_t         error_code;

    pb_istream_t      input_stream = pb_istream_from_buffer(s_protobuf, s_protobuf_offset);
    bool              status = pb_decode(&input_stream, &ControlEnvelope_msg, &s_protobuf_decoded.control_envelope);
    if (!status)
    {
        error_code = 1;
        APP_LOG_DEBUG("Decode command failed. status: %x.", status);
        return error_code;
    }

    ControlEnvelope   cmd_response;
    Response          *p_response;
    DeviceInformation *p_dev_info;
    DeviceFeatures    *p_dev_feat;
    
    memset(&cmd_response, 0, sizeof(ControlEnvelope));
    cmd_response.command = Command_NONE;
    cmd_response.which_payload = ControlEnvelope_response_tag;

    p_response = &cmd_response.payload.response;
    p_response->error_code = (ErrorCode)AGS_RES_CODE_SUCCESS;
    
    switch (s_protobuf_decoded.control_envelope.command)
    {
        case Command_NONE:
            break;

        case Command_GET_DEVICE_INFORMATION:
            p_dev_info = &p_response->payload.device_information;
            
            strncpy(p_dev_info->name, DEVICE_NAME, sizeof(p_dev_info->name));
            strncpy(p_dev_info->device_type, USER_GADGET_AMAZON_ID, sizeof(p_dev_info->device_type));
            p_dev_info->supported_transports_count = 1;
            p_dev_info->supported_transports[0]    = Transport_BLUETOOTH_LOW_ENERGY;
            p_response->which_payload              = Response_device_information_tag;
            APP_LOG_DEBUG("Send the device information.");
            break;

        case Command_GET_DEVICE_FEATURES:
            p_dev_feat = &p_response->payload.device_features;

            p_dev_feat->features          = USER_GADGET_DEV_FEATURES;
            p_dev_feat->device_attributes = 0; 
            p_response->which_payload     = Response_device_features_tag;
            APP_LOG_DEBUG("Send the device feature.");
            break;

#if USER_GADGET_OTA_ENABLE
        case Command_UPDATE_COMPONENT_SEGMENT:
            cmd_response.which_payload = ControlEnvelope_update_component_segment_tag;
            memset(&cmd_response.payload.response, 0, sizeof(Response));
            break;

        case Command_APPLY_FIRMWARE:
            cmd_response.which_payload = ControlEnvelope_apply_firmware_tag;
            memset(&cmd_response.payload.response, 0, sizeof(Response));
            break;
#endif
        default:
            APP_LOG_DEBUG("Unknown command:%02x.", s_protobuf_decoded.control_envelope.command);
            p_response->error_code    = (ErrorCode)AGS_RES_CODE_UNSUPPORTED;
            p_response->which_payload = Response_error_code_tag;
            break;
    }

    pb_ostream_t out_stream = pb_ostream_from_buffer(s_protobuf, sizeof(s_protobuf));
    status = pb_encode(&out_stream, &ControlEnvelope_msg, &cmd_response);
    if (!status)
    {
        APP_LOG_DEBUG("Encode command response failed.");
        return SDK_ERR_APP_ERROR;
    }

    return ags_stream_send(conn_idx, AGS_CONTROL_STREAM_ID, s_protobuf, out_stream.bytes_written);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint16_t user_gadget_protobuf_owner_release(uint8_t conn_idx)
{
    uint16_t user = s_protobuf_user;

    s_protobuf_user = PROTOBUF_USER_NONE;
    return user;
}

bool user_gadget_control_stream_cb(uint8_t conn_idx, const uint8_t *p_data, uint16_t len, uint8_t still_receiving)
{
    sdk_err_t error_code;

    error_code = user_gadget_protobuf_fill(conn_idx, PROTOBUF_USER_CONTROL_STREAM, p_data, len);
    if (error_code)
    {
        if (SDK_ERR_INVALID_OFFSET == error_code)
        {
            user_gadget_protobuf_reset(conn_idx);
        }
        return false;
    }

    if (still_receiving)
    {
        return false;
    }
    else
    {
        error_code = user_gadget_command_respond(conn_idx);
        if (error_code)
        {
            APP_LOG_DEBUG("Respond to command failed.");
            user_gadget_protobuf_reset(conn_idx);
            return false;
        }
        user_gadget_protobuf_reset(conn_idx);
    }
    return true;
}

bool user_gadget_alexa_stream_cb(uint8_t conn_idx, const uint8_t *p_data, uint16_t len, uint8_t still_receiving)
{
    bool      ret;
    sdk_err_t error_code;

    error_code = user_gadget_protobuf_fill(conn_idx, PROTOBUF_USER_ALEXA_STREAM, p_data, len);
    if (error_code)
    {
        if (SDK_ERR_INVALID_OFFSET == error_code)
        {
            user_gadget_protobuf_reset(conn_idx);
        }
        return false;
    }

    if (still_receiving)
    {
        return false;
    }
    else
    {
        pb_istream_t input_stream;
        bool         status;

        input_stream = pb_istream_from_buffer(s_protobuf, s_protobuf_offset);
        status = pb_decode(&input_stream, &directive_DirectiveParserProto_msg, &s_protobuf_decoded.directive_parser);
        if (!status)
        {
            APP_LOG_DEBUG("Decode alexa stream failed.");
            user_gadget_protobuf_reset(conn_idx);
            return false;
        }

        const char *p_name      = s_protobuf_decoded.directive_parser.directive.header.name;
        const char *p_namespace = s_protobuf_decoded.directive_parser.directive.header.namespace;

        for (uint8_t i=0; i<ARRAY_SIZE(directive_handlers);i++)
        {
            if (strcmp(p_namespace, directive_handlers[i].p_directive_namespace) == 0)
            {
                error_code = directive_handlers[i].directive_handler(conn_idx, p_name, s_protobuf, s_protobuf_offset);
                if (error_code)
                {
                    APP_LOG_DEBUG("Unsupported name.");
                    ret = false;
                }
                else
                {
                    ret = true;
                }
                break;
            }
        }
        user_gadget_protobuf_reset(conn_idx);
    }
    return ret;
}

void user_gadget_protocol_version_send(uint8_t conn_idx)
{
    APP_LOG_DEBUG("Send the protocol version.");
    sdk_err_t error_code;
    uint8_t protocol_version[USER_GADGET_PV_LEN];    
    uint16_t mtu_size;

    ble_gatt_mtu_get(conn_idx, &mtu_size);

    memset(protocol_version, 0, sizeof(protocol_version));
    htobe16(&protocol_version, USER_GADGET_PV_IDENTIFIER);
    protocol_version[2] = USER_GADGET_PV_MAJOR_VERSION;
    protocol_version[3] = USER_GADGET_PV_MINOR_VERSION;
    htobe16(&protocol_version[4], mtu_size);
    htobe16(&protocol_version[6], USER_GADGET_TRANSACTION_BUF_SIZE);

    error_code = ags_non_stream_send(conn_idx, protocol_version, sizeof(protocol_version));
    APP_ERROR_CHECK(error_code);
}

uint16_t user_gadget_custom_event_send(uint8_t conn_idx, uint8_t *p_name, uint8_t *p_payload)
{
#if USER_GADGET_CAPABILITY_CUSTOM_ENABLE
    sdk_err_t error_code;
    
    error_code = user_gadget_protobuf_owner_obtain(conn_idx, PROTOBUF_USER_CUSTOM_EVENT);
    if (error_code)
    {
        return error_code;
    }

    pb_ostream_t output_stream;
    bool         status;

    s_protobuf_decoded.custom_event_drt.has_event        = true;
    s_protobuf_decoded.custom_event_drt.event.has_header = true;
    strcpy(s_protobuf_decoded.custom_event_drt.event.header.namespace, USER_GADGET_CUSTOM_NAMESPACE);
    strcpy(s_protobuf_decoded.custom_event_drt.event.header.name, (char *)p_name);
    strcpy(s_protobuf_decoded.custom_event_drt.event.payload, (char *)p_payload);
    
    output_stream = pb_ostream_from_buffer(s_protobuf, sizeof(s_protobuf));
    status        = pb_encode(&output_stream, &custom_event_proto_msg, &s_protobuf_decoded.custom_event_drt);
    if (!status)
    {
        APP_LOG_DEBUG("Encode custom event data failed.");
        return SDK_ERR_APP_ERROR;
    }

    APP_LOG_DEBUG("Send the custom event.");
    error_code = ags_stream_send(conn_idx, AGS_ALEXA_STREAM_ID, s_protobuf, output_stream.bytes_written);

    return error_code;
#else
    return SDK_ERR_REQ_NOT_SUPPORTED;
#endif
}

sdk_err_t user_gadget_evt_handler_init(user_gadget_evt_handler_t user_gadget_evt_handler)
{
    sdk_err_t error_code = SDK_SUCCESS;

    if (NULL == user_gadget_evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    s_user_gadget_evt_handler = user_gadget_evt_handler;

    return error_code;
}

